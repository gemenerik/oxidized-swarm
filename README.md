# oxidized-swarm

Performance stress test for the Rust-based [cflib](https://github.com/bitcraze/crazyflie-lib-python) rewrite. Pushes the Python bindings to their limits by running synchronized multi-drone operations across multiple Crazyradios, testing the power and reliability of the Rust library under real flight conditions.

## Repository contents

- `main.py` — Swarm controller that runs three concurrent subswarm missions (trajectory, hover, blink)
- `swarm.json` — Drone definitions grouped by subswarm, with per-group firmware config fragments
- `crazyflie-firmware/` — Bitcraze firmware submodule, fixed at release **2025.12.1**
- `flasher/` — Firmware build & flash utility (see [flasher/README.md](flasher/README.md))

## Setup

Requires Python 3.13+. [uv](https://github.com/astral-sh/uv) is recommended.

```bash
uv sync
```

## Configuration

Drones are defined in `swarm.json`, organized into three subswarm groups. Each group has its own `config_fragments` (used by the flasher for firmware builds) and a list of drones:

```json
{
  "trajectory_drones": {
    "config_fragments": ["lighthouse_8bs.conf"],
    "drones": [
      { "id": 10, "uri": "radio://0/80/2M/ABAD1DEA10", "platform": "cf21bl" }
    ]
  },
  "hover_drones": {
    "config_fragments": ["lighthouse_8bs.conf"],
    "drones": [
      { "id": 1, "uri": "radio://0/90/2M/ABAD1DEA01", "platform": "cf21bl" }
    ]
  },
  "blink_drones": {
    "config_fragments": [],
    "drones": [
      { "id": 20, "uri": "radio://2/70/2M/ABAD1DEA20", "platform": "cf2" }
    ]
  }
}
```

Platform build configs (`defconfig`, `make_flags`) live under the top-level `platforms` key. Any subswarm group can contain a mix of platforms.

## Subswarm missions

All drones connect first, then the three missions launch concurrently:

**Trajectory** — Uploads a figure-8 polynomial trajectory (Poly4D) to drone memory, then flies it using the high-level commander. Sequence: blink 3x → arm → preflight → takeoff to 1.0 m → fly trajectory → land → disarm → blink 2x.

**Hover** — Takes off, holds position for a fixed duration, and lands. Sequence: blink 3x → arm → preflight → takeoff to 0.5 m → hover 15 s → land → disarm → blink 2x.

**Blink** — LED-only patterns for drones without motors. Runs several blink patterns with varying timing (rapid, slow pulse, fast strobe, confirmation).

Drones are always disarmed on exit, even if a mission fails. If one subswarm mission fails, the others continue independently.

## Usage

### Run the swarm controller

```bash
uv run main.py
```

### Options

```
--config PATH    Path to swarm config file (default: swarm.json)
--no-cache       Disable TOC caching (caching is enabled by default)
--ids ID [ID …]  Only control specific drone IDs (e.g. --ids 1 2 3)
```

### Examples

Run with a subset of drones (filters across all subswarm groups):

```bash
uv run main.py --ids 1 2 3
```

Disable TOC caching (useful for debugging connection issues):

```bash
uv run main.py --no-cache
```

## Flashing firmware

The `crazyflie-firmware/` submodule is checked out at release **2025.12.1**. We overwrite `configs/lighthouse_8bs.conf` during the build because our lab section uses base station IDs 5, 6, 7, and 8 instead of the defaults.

See [flasher/README.md](flasher/README.md) for instructions on building and flashing firmware to the drones. The flasher uses the stable PyPI version of cflib since bootloader support is not yet available in the Rust variant.
