#!/usr/bin/env python3
"""
Oxidized Swarm - Multi-drone control stress test.

Demonstrates the performance of the Rust-based Crazyflie library by controlling
multiple drones through a single Crazyradio.

Architecture:
- Supports synchronized operations (all drones complete before proceeding)
- Supports async batches (multiple operations in parallel, then sync)
- Easy to extend with additional mission steps
"""

import argparse
import asyncio
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Any

from cflib import Crazyflie, LinkContext, FileTocCache, NoTocCache


# Supervisor state bit positions
# Bit 0 = Can be armed - the system can be armed and will accept an arming command
# Bit 1 = Is armed - the system is armed
# Bit 2 = Is auto armed - the system is configured to automatically arm
# Bit 3 = Can fly - the Crazyflie is ready to fly
# Bit 4 = Is flying - the Crazyflie is flying
# Bit 5 = Is tumbled - the Crazyflie is upside down
# Bit 6 = Is locked - the Crazyflie is in the locked state and must be restarted
# Bit 7 = Is crashed - the Crazyflie has crashed
# Bit 8 = High level control is actively flying the drone
# Bit 9 = High level trajectory has finished
# Bit 10 = High level control is disabled and not producing setpoints
BIT_CAN_BE_ARMED = 0
BIT_IS_ARMED = 1
BIT_IS_AUTO_ARMED = 2
BIT_CAN_FLY = 3
BIT_IS_FLYING = 4
BIT_IS_TUMBLED = 5
BIT_IS_LOCKED = 6
BIT_IS_CRASHED = 7
BIT_HL_CONTROL_ACTIVE = 8
BIT_HL_TRAJ_FINISHED = 9
BIT_HL_CONTROL_DISABLED = 10

# Log block interval in ms
SUPERVISOR_LOG_INTERVAL = 100


@dataclass
class DroneConfig:
    """Configuration for a single drone."""
    id: int
    uri: str
    platform: str


@dataclass
class DroneState:
    """Runtime state for a connected drone."""
    cf: Crazyflie
    config: DroneConfig
    supervisor_stream: Any = None
    supervisor_state: int = 0

    def _bit(self, position: int) -> bool:
        return bool((self.supervisor_state >> position) & 1)

    def can_be_armed(self) -> bool:
        return self._bit(BIT_CAN_BE_ARMED)

    def is_armed(self) -> bool:
        return self._bit(BIT_IS_ARMED)

    def is_auto_armed(self) -> bool:
        return self._bit(BIT_IS_AUTO_ARMED)

    def can_fly(self) -> bool:
        return self._bit(BIT_CAN_FLY)

    def is_flying(self) -> bool:
        return self._bit(BIT_IS_FLYING)

    def is_tumbled(self) -> bool:
        return self._bit(BIT_IS_TUMBLED)

    def is_locked(self) -> bool:
        return self._bit(BIT_IS_LOCKED)

    def is_crashed(self) -> bool:
        return self._bit(BIT_IS_CRASHED)

    def hl_control_active(self) -> bool:
        return self._bit(BIT_HL_CONTROL_ACTIVE)

    def hl_traj_finished(self) -> bool:
        return self._bit(BIT_HL_TRAJ_FINISHED)

    def hl_control_disabled(self) -> bool:
        return self._bit(BIT_HL_CONTROL_DISABLED)


class SwarmController:
    """Manages a swarm of Crazyflies with synchronized operations."""

    def __init__(self, drones: List[DroneConfig], cache):
        self.drone_configs = drones
        self.cache = cache
        self.context = LinkContext()
        self.drones: List[DroneState] = []

    async def connect_all(self) -> None:
        """Connect to all drones concurrently."""
        print(f"Connecting to {len(self.drone_configs)} drones...")
        start = time.perf_counter()

        async def connect_one(config: DroneConfig) -> tuple[DroneState, float]:
            cf_start = time.perf_counter()
            cf = await Crazyflie.connect_from_uri(self.context, config.uri, self.cache)
            elapsed = time.perf_counter() - cf_start
            return DroneState(cf=cf, config=config), elapsed

        results = await asyncio.gather(*[connect_one(d) for d in self.drone_configs])
        self.drones = [state for state, _ in results]
        connect_times = [t for _, t in results]
        total_time = time.perf_counter() - start

        print(f"✓ All connected in {total_time:.3f}s")
        for drone, t in zip(self.drones, connect_times):
            print(f"  ID {drone.config.id:02d}: {t:.3f}s")
        print()

    async def setup_logging(self) -> None:
        """Set up log blocks for all drones (call once after connect)."""
        print("Setting up supervisor log blocks...")
        start = time.perf_counter()

        async def setup_one(drone: DroneState):
            log = drone.cf.log()
            block = await log.create_block()
            await block.add_variable("supervisor.info")
            drone.supervisor_stream = await block.start(SUPERVISOR_LOG_INTERVAL)

        await asyncio.gather(*[setup_one(d) for d in self.drones])

        elapsed = time.perf_counter() - start
        print(f"✓ Log blocks created in {elapsed:.3f}s\n")

    async def stop_logging(self) -> None:
        """Stop all log blocks."""
        async def stop_one(drone: DroneState):
            if drone.supervisor_stream:
                await drone.supervisor_stream.stop()

        await asyncio.gather(*[stop_one(d) for d in self.drones])

    async def update_supervisor_state(self) -> None:
        """Fetch latest supervisor state from all drones."""
        async def update_one(drone: DroneState):
            if drone.supervisor_stream:
                data = await drone.supervisor_stream.next()
                drone.supervisor_state = data["data"]["supervisor.info"]

        await asyncio.gather(*[update_one(d) for d in self.drones])

    async def disconnect_all(self) -> None:
        """Disconnect from all drones concurrently."""
        print("Disconnecting from all drones...")
        await self.stop_logging()
        await asyncio.gather(*[d.cf.disconnect() for d in self.drones])
        print("✓ All disconnected\n")

    async def sync_step(self, step_name: str, operation) -> None:
        """Execute an operation on all drones and wait for all to complete.

        Args:
            step_name: Description of the operation for logging
            operation: Async function that takes DroneState and performs the operation
        """
        print(f"{step_name}...")
        start = time.perf_counter()

        await asyncio.gather(*[operation(drone) for drone in self.drones])

        elapsed = time.perf_counter() - start
        print(f"✓ {step_name} completed in {elapsed:.3f}s\n")

    async def arm_all(self) -> None:
        """Arm all drones (synchronized operation)."""
        # First, update supervisor state and check all brushless drones can be armed
        await self.update_supervisor_state()

        not_ready = []
        for drone in self.drones:
            if drone.config.platform != "cf21bl":
                continue
            if drone.is_locked():
                not_ready.append(f"ID {drone.config.id:02d}: LOCKED (requires reboot)")
            elif drone.is_tumbled():
                not_ready.append(f"ID {drone.config.id:02d}: TUMBLED")
            elif not drone.can_be_armed():
                not_ready.append(f"ID {drone.config.id:02d}: Cannot be armed")

        if not_ready:
            print("ERROR: Some drones are not ready to arm:")
            for msg in not_ready:
                print(f"  {msg}")
            raise RuntimeError("Not all drones can be armed")

        async def arm_one(drone: DroneState):
            if drone.config.platform != "cf21bl":
                print(f"  ID {drone.config.id:02d}: Skipped (not brushless)")
                return
            platform = drone.cf.platform()
            await platform.send_arming_request(do_arm=True)
            print(f"  ID {drone.config.id:02d}: Armed")

        await self.sync_step("Arming all drones", arm_one)

    async def disarm_all(self) -> None:
        """Disarm all drones (synchronized operation)."""
        async def disarm_one(drone: DroneState):
            if drone.config.platform != "cf21bl":
                print(f"  ID {drone.config.id:02d}: Skipped (not brushless)")
                return
            platform = drone.cf.platform()
            await platform.send_arming_request(do_arm=False)
            print(f"  ID {drone.config.id:02d}: Disarmed")

        await self.sync_step("Disarming all drones", disarm_one)

    async def blink_all(self, times: int = 3, on_time: float = 0.5, off_time: float = 0.5) -> None:
        """Blink LEDs on all drones (synchronized operation)."""
        async def blink_one(drone: DroneState):
            param = drone.cf.param()
            for i in range(times):
                await param.set("led.bitmask", 212)
                await asyncio.sleep(on_time)
                await param.set("led.bitmask", 0)
                if i < times - 1:
                    await asyncio.sleep(off_time)
            print(f"  ID {drone.config.id:02d}: Blinked {times}x")

        await self.sync_step(f"Blinking LEDs ({times}x)", blink_one)

    async def run_mission(self) -> None:
        """Execute the main mission sequence."""
        print("="*60)
        print("Starting mission")
        print("="*60)
        print()

        # Blink to show we're connected
        await self.blink_all(times=3)

        # Mission steps
        await self.arm_all()

        print("Waiting 3 seconds...")
        await asyncio.sleep(3.0)
        print()

        await self.disarm_all()

        # Blink to show mission complete
        await self.blink_all(times=2)

        print("="*60)
        print("Mission complete")
        print("="*60)


def load_swarm_config(config_file: str) -> List[DroneConfig]:
    """Load drone configurations from swarm.json."""
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)

        drones = []
        for drone in config.get('drones', []):
            drones.append(DroneConfig(
                id=drone['id'],
                uri=drone['uri'],
                platform=drone['platform']
            ))
        return sorted(drones, key=lambda d: d.id)

    except FileNotFoundError:
        print(f"Error: Configuration file not found: {config_file}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON config: {e}")
        sys.exit(1)


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Control Crazyflie swarm using Rust-based cflib"
    )
    parser.add_argument(
        "--config",
        default="swarm.json",
        help="Path to swarm configuration JSON file (default: swarm.json)"
    )
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help="Disable TOC file caching (caching is enabled by default)"
    )
    parser.add_argument(
        "--ids",
        nargs="+",
        type=int,
        metavar="ID",
        help="Only use specific drone IDs (e.g., --ids 1 2 3)"
    )
    args = parser.parse_args()

    # Load configuration
    all_drones = load_swarm_config(args.config)

    # Filter drones if --ids specified
    if args.ids:
        drones = [d for d in all_drones if d.id in args.ids]
        if not drones:
            print(f"Error: No drones found with IDs: {args.ids}")
            sys.exit(1)
    else:
        drones = all_drones

    print(f"Selected {len(drones)} drone(s):")
    for drone in drones:
        print(f"  ID {drone.id:02d}: {drone.platform} - {drone.uri}")
    print()

    # Set up TOC cache
    if args.no_cache:
        cache = NoTocCache()
    else:
        cache_dir = str(Path.cwd() / "cache")
        cache = FileTocCache(cache_dir)
        print(f"Using TOC cache: {cache.get_cache_dir()}\n")

    # Create swarm controller and run mission
    swarm = SwarmController(drones, cache)

    try:
        await swarm.connect_all()
        await swarm.setup_logging()
        await swarm.run_mission()
    finally:
        await swarm.disconnect_all()


if __name__ == "__main__":
    asyncio.run(main())
