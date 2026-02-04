#!/usr/bin/env python3
"""
Oxidized Swarm - Multi-drone control with subswarm missions.

Demonstrates the Rust-based Crazyflie library controlling multiple drones
across separate radios, each subswarm running a different mission concurrently:
  - trajectory: upload and fly a figure-8 polynomial trajectory
  - hover: takeoff, hold position, land
  - blink: LED patterns only (no flight)
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
from cflib.trajectory import Poly, Poly4D


# Supervisor state bit positions
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

# Hover mission duration in seconds
HOVER_DURATION = 15.0

# Figure-8 trajectory coefficients (Poly4D segments)
# Duration,x^0..x^7,y^0..y^7,z^0..z^7,yaw^0..yaw^7
# See https://github.com/whoenig/uav_trajectories
FIGURE8_DATA = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493,
     -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049,
     -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926,
     -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166,
     0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627,
     0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130,
     0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253,
     -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463,
     -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294,
     0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567,
     0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
]


def build_figure8_trajectory() -> tuple[list[Poly4D], float]:
    """Convert FIGURE8_DATA into Poly4D segments, return (segments, total_duration)."""
    trajectory: list[Poly4D] = []
    total_duration = 0.0
    for row in FIGURE8_DATA:
        duration = row[0]
        x = Poly(row[1:9])
        y = Poly(row[9:17])
        z = Poly(row[17:25])
        yaw = Poly(row[25:33])
        trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration
    return trajectory, total_duration


@dataclass
class DroneConfig:
    """Configuration for a single drone."""
    id: int
    uri: str
    platform: str


@dataclass
class SubswarmConfig:
    """Configuration for a subswarm group."""
    drones: List[DroneConfig]
    config_fragments: List[str]


@dataclass
class SwarmConfig:
    """Top-level swarm configuration with three subswarm groups."""
    trajectory: SubswarmConfig
    hover: SubswarmConfig
    blink: SubswarmConfig

    def all_drones(self) -> List[DroneConfig]:
        return self.trajectory.drones + self.hover.drones + self.blink.drones


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
        self._log_tasks: List[asyncio.Task] = []

    def get_drones_by_ids(self, ids: set[int]) -> List[DroneState]:
        """Return DroneState objects matching the given drone IDs."""
        return [d for d in self.drones if d.config.id in ids]

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

        print(f"All connected in {total_time:.3f}s")
        for drone, t in zip(self.drones, connect_times):
            print(f"  ID {drone.config.id:02d}: {t:.3f}s")
        print()

    async def setup_logging(self) -> None:
        """Set up log blocks for all drones and start background drain tasks."""
        print("Setting up supervisor log blocks...")
        start = time.perf_counter()

        async def setup_one(drone: DroneState):
            log = drone.cf.log()
            block = await log.create_block()
            await block.add_variable("supervisor.info")
            drone.supervisor_stream = await block.start(SUPERVISOR_LOG_INTERVAL)

        await asyncio.gather(*[setup_one(d) for d in self.drones])

        for drone in self.drones:
            task = asyncio.create_task(self._drain_supervisor_log(drone))
            self._log_tasks.append(task)

        elapsed = time.perf_counter() - start
        print(f"Log blocks created in {elapsed:.3f}s\n")

    async def _drain_supervisor_log(self, drone: DroneState) -> None:
        """Continuously read from the log stream, keeping supervisor_state fresh."""
        try:
            while drone.supervisor_stream:
                data = await drone.supervisor_stream.next()
                drone.supervisor_state = data["data"]["supervisor.info"]
        except Exception:
            pass

    async def stop_logging(self) -> None:
        """Cancel background drain tasks and stop all log blocks."""
        for task in self._log_tasks:
            task.cancel()
        await asyncio.gather(*self._log_tasks, return_exceptions=True)
        self._log_tasks.clear()

        async def stop_one(drone: DroneState):
            if drone.supervisor_stream:
                await drone.supervisor_stream.stop()

        await asyncio.gather(*[stop_one(d) for d in self.drones])

    async def disconnect_all(self) -> None:
        """Disconnect from all drones concurrently."""
        print("Disconnecting from all drones...")
        await self.stop_logging()
        await asyncio.gather(*[d.cf.disconnect() for d in self.drones])
        print("All disconnected\n")

    async def sync_step(self, step_name: str, operation,
                        drones: List[DroneState] | None = None) -> None:
        """Execute an operation on drones and wait for all to complete."""
        target = drones if drones is not None else self.drones
        if not target:
            return
        print(f"{step_name}...")
        start = time.perf_counter()
        await asyncio.gather(*[operation(drone) for drone in target])
        elapsed = time.perf_counter() - start
        print(f"{step_name} completed in {elapsed:.3f}s\n")

    async def arm_all(self, drones: List[DroneState] | None = None,
                      label: str = "") -> None:
        """Arm drones (synchronized operation)."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""

        not_ready = [d for d in target
                     if d.config.platform == "cf21bl" and not d.can_be_armed()]
        if not_ready:
            ids = ", ".join(f"{d.config.id:02d}" for d in not_ready)
            raise RuntimeError(f"{prefix}Cannot arm drones: {ids}")

        async def arm_one(drone: DroneState):
            if drone.config.platform != "cf21bl":
                print(f"  {prefix}ID {drone.config.id:02d}: Skipped (not brushless)")
                return
            platform = drone.cf.platform()
            await platform.send_arming_request(do_arm=True)
            print(f"  {prefix}ID {drone.config.id:02d}: Armed")

        await self.sync_step(f"{prefix}Arming drones", arm_one, target)

    async def disarm_all(self, drones: List[DroneState] | None = None,
                         label: str = "") -> None:
        """Disarm drones (synchronized operation)."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""

        async def disarm_one(drone: DroneState):
            if drone.config.platform != "cf21bl":
                print(f"  {prefix}ID {drone.config.id:02d}: Skipped (not brushless)")
                return
            platform = drone.cf.platform()
            await platform.send_arming_request(do_arm=False)
            print(f"  {prefix}ID {drone.config.id:02d}: Disarmed")

        await self.sync_step(f"{prefix}Disarming drones", disarm_one, target)

    async def blink_all(self, times: int = 3, on_time: float = 0.5,
                        off_time: float = 0.5,
                        drones: List[DroneState] | None = None,
                        label: str = "") -> None:
        """Blink LEDs on drones (synchronized operation)."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""

        async def blink_one(drone: DroneState):
            param = drone.cf.param()
            for i in range(times):
                await param.set("led.bitmask", 212)
                await asyncio.sleep(on_time)
                await param.set("led.bitmask", 0)
                if i < times - 1:
                    await asyncio.sleep(off_time)
            print(f"  {prefix}ID {drone.config.id:02d}: Blinked {times}x")

        await self.sync_step(f"{prefix}Blinking LEDs ({times}x)", blink_one, target)

    async def preflight_check(self, drones: List[DroneState] | None = None,
                              label: str = "") -> None:
        """Check that drones report can_fly."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""
        not_ready = [d for d in target if not d.can_fly()]
        if not_ready:
            ids = ", ".join(f"{d.config.id:02d}" for d in not_ready)
            raise RuntimeError(f"{prefix}Preflight check failed, drones not ready: {ids}")
        print(f"{prefix}Preflight check passed\n")

    async def takeoff_all(self, height: float = 0.5, duration: float = 2.0,
                          drones: List[DroneState] | None = None,
                          label: str = "") -> None:
        """Take off drones (synchronized operation)."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""

        async def takeoff_one(drone: DroneState):
            hlc = drone.cf.high_level_commander()
            await hlc.take_off(height, None, duration, None)
            await asyncio.sleep(duration)
            print(f"  {prefix}ID {drone.config.id:02d}: Airborne at {height}m")

        await self.sync_step(f"{prefix}Taking off", takeoff_one, target)

    async def land_all(self, height: float = 0.0, duration: float = 2.0,
                       drones: List[DroneState] | None = None,
                       label: str = "") -> None:
        """Land drones (synchronized operation)."""
        target = drones if drones is not None else self.drones
        prefix = f"[{label}] " if label else ""

        async def land_one(drone: DroneState):
            hlc = drone.cf.high_level_commander()
            await hlc.land(height, None, duration, None)
            await asyncio.sleep(duration)
            await hlc.stop(None)
            print(f"  {prefix}ID {drone.config.id:02d}: Landed")

        await self.sync_step(f"{prefix}Landing", land_one, target)


# ---------------------------------------------------------------------------
# Subswarm missions
# ---------------------------------------------------------------------------

async def run_trajectory_mission(swarm: SwarmController,
                                 drones: List[DroneState]) -> None:
    """Trajectory subswarm: upload and fly a figure-8 trajectory."""
    if not drones:
        return

    label = "TRAJECTORY"
    print(f"[{label}] Starting mission with {len(drones)} drone(s)")

    # Build trajectory segments
    trajectory, total_duration = build_figure8_trajectory()
    print(f"[{label}] Trajectory: {len(trajectory)} segments, {total_duration:.1f}s")

    # Upload trajectory to each drone and define it
    async def upload_trajectory(drone: DroneState):
        mem = drone.cf.memory()
        bytes_written = await mem.write_trajectory(trajectory)
        hlc = drone.cf.high_level_commander()
        await hlc.define_trajectory(1, 0, len(trajectory), 0)
        print(f"  [{label}] ID {drone.config.id:02d}: Uploaded {bytes_written} bytes")

    await swarm.sync_step(f"[{label}] Uploading trajectories", upload_trajectory, drones)

    # Blink to confirm ready
    await swarm.blink_all(times=3, drones=drones, label=label)

    # Arm
    await swarm.arm_all(drones=drones, label=label)

    try:
        await asyncio.sleep(1.0)
        await swarm.preflight_check(drones=drones, label=label)

        # Takeoff
        await swarm.takeoff_all(height=1.0, duration=2.0, drones=drones, label=label)

        # Start trajectory on all drones
        print(f"[{label}] Starting figure-8 trajectory...")
        await asyncio.gather(*[
            drone.cf.high_level_commander().start_trajectory(
                1, 1.0, True, False, False, None
            )
            for drone in drones
        ])
        await asyncio.sleep(total_duration)
        print(f"[{label}] Trajectory complete")

        # Land
        await swarm.land_all(drones=drones, label=label)
    finally:
        await swarm.disarm_all(drones=drones, label=label)

    # Blink to signal completion
    await swarm.blink_all(times=2, drones=drones, label=label)
    print(f"[{label}] Mission complete\n")


async def run_hover_mission(swarm: SwarmController,
                            drones: List[DroneState]) -> None:
    """Hover subswarm: takeoff, hold position, land."""
    if not drones:
        return

    label = "HOVER"
    print(f"[{label}] Starting mission with {len(drones)} drone(s)")

    # Blink to confirm ready
    await swarm.blink_all(times=3, drones=drones, label=label)

    # Arm
    await swarm.arm_all(drones=drones, label=label)

    try:
        await asyncio.sleep(1.0)
        await swarm.preflight_check(drones=drones, label=label)

        # Takeoff
        await swarm.takeoff_all(height=0.5, duration=2.0, drones=drones, label=label)

        # Hover
        print(f"[{label}] Hovering for {HOVER_DURATION:.1f}s...")
        await asyncio.sleep(HOVER_DURATION)

        # Land
        await swarm.land_all(drones=drones, label=label)
    finally:
        await swarm.disarm_all(drones=drones, label=label)

    # Blink to signal completion
    await swarm.blink_all(times=2, drones=drones, label=label)
    print(f"[{label}] Mission complete\n")


async def run_blink_mission(swarm: SwarmController,
                            drones: List[DroneState]) -> None:
    """Blink subswarm: LED patterns only, no flight."""
    if not drones:
        return

    label = "BLINK"
    print(f"[{label}] Starting mission with {len(drones)} drone(s)")

    # Pattern 1: Rapid triple blink
    await swarm.blink_all(times=3, on_time=0.3, off_time=0.3,
                          drones=drones, label=label)
    await asyncio.sleep(1.0)

    # Pattern 2: Slow pulse
    await swarm.blink_all(times=5, on_time=1.0, off_time=0.5,
                          drones=drones, label=label)
    await asyncio.sleep(1.0)

    # Pattern 3: Fast strobe
    await swarm.blink_all(times=8, on_time=0.15, off_time=0.15,
                          drones=drones, label=label)
    await asyncio.sleep(1.0)

    # Pattern 4: Final confirmation
    await swarm.blink_all(times=2, on_time=0.5, off_time=0.5,
                          drones=drones, label=label)

    print(f"[{label}] Mission complete\n")


# ---------------------------------------------------------------------------
# Configuration loading
# ---------------------------------------------------------------------------

def load_swarm_config(config_file: str) -> SwarmConfig:
    """Load swarm configuration from JSON file."""
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)

        def parse_group(key: str) -> SubswarmConfig:
            group = config.get(key, {"config_fragments": [], "drones": []})
            drones = sorted(
                [DroneConfig(id=d['id'], uri=d['uri'], platform=d['platform'])
                 for d in group.get('drones', [])],
                key=lambda d: d.id,
            )
            return SubswarmConfig(
                drones=drones,
                config_fragments=group.get('config_fragments', []),
            )

        return SwarmConfig(
            trajectory=parse_group('trajectory_drones'),
            hover=parse_group('hover_drones'),
            blink=parse_group('blink_drones'),
        )

    except FileNotFoundError:
        print(f"Error: Configuration file not found: {config_file}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON config: {e}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Control Crazyflie subswarms using Rust-based cflib"
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
    parser.add_argument(
        "--subswarm",
        nargs="+",
        choices=["trajectory", "hover", "blink"],
        metavar="NAME",
        help="Only run specific subswarms (e.g., --subswarm blink hover)"
    )
    args = parser.parse_args()

    # Load configuration
    swarm_config = load_swarm_config(args.config)

    # Filter by --subswarm if specified
    if args.subswarm:
        active = set(args.subswarm)
        if "trajectory" not in active:
            swarm_config.trajectory.drones = []
        if "hover" not in active:
            swarm_config.hover.drones = []
        if "blink" not in active:
            swarm_config.blink.drones = []

    # Filter by --ids if specified
    if args.ids:
        id_set = set(args.ids)
        swarm_config.trajectory.drones = [
            d for d in swarm_config.trajectory.drones if d.id in id_set
        ]
        swarm_config.hover.drones = [
            d for d in swarm_config.hover.drones if d.id in id_set
        ]
        swarm_config.blink.drones = [
            d for d in swarm_config.blink.drones if d.id in id_set
        ]

    all_drones = swarm_config.all_drones()
    if not all_drones:
        print("Error: No drones selected")
        sys.exit(1)

    # Print subswarm summary
    print("Subswarm configuration:")
    for name, group in [("Trajectory", swarm_config.trajectory),
                        ("Hover", swarm_config.hover),
                        ("Blink", swarm_config.blink)]:
        print(f"  {name}: {len(group.drones)} drone(s)")
        for d in group.drones:
            print(f"    ID {d.id:02d}: {d.platform} - {d.uri}")
    print()

    # Set up TOC cache
    if args.no_cache:
        cache = NoTocCache()
    else:
        cache_dir = str(Path.cwd() / "cache")
        cache = FileTocCache(cache_dir)
        print(f"Using TOC cache: {cache.get_cache_dir()}\n")

    # Create controller with ALL drones
    swarm = SwarmController(all_drones, cache)

    try:
        await swarm.connect_all()
        await swarm.setup_logging()

        # Partition connected DroneState objects into subswarm groups
        traj_ids = {d.id for d in swarm_config.trajectory.drones}
        hover_ids = {d.id for d in swarm_config.hover.drones}
        blink_ids = {d.id for d in swarm_config.blink.drones}

        traj_drones = swarm.get_drones_by_ids(traj_ids)
        hover_drones = swarm.get_drones_by_ids(hover_ids)
        blink_drones = swarm.get_drones_by_ids(blink_ids)

        # Run all subswarm missions concurrently
        print("=" * 60)
        print("Starting all subswarm missions")
        print("=" * 60)
        print()

        results = await asyncio.gather(
            run_trajectory_mission(swarm, traj_drones),
            run_hover_mission(swarm, hover_drones),
            run_blink_mission(swarm, blink_drones),
            return_exceptions=True,
        )

        # Report any mission failures
        mission_names = ["trajectory", "hover", "blink"]
        for name, result in zip(mission_names, results):
            if isinstance(result, Exception):
                print(f"WARNING: {name} mission failed: {result}")

        print("=" * 60)
        print("All missions complete")
        print("=" * 60)
    finally:
        await swarm.disconnect_all()


if __name__ == "__main__":
    asyncio.run(main())
