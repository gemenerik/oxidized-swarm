# oxidized-swarm

Performance stress test for the Rust-based [cflib](https://github.com/bitcraze/crazyflie-lib-python) rewrite. Pushes the Python bindings to their limits by running synchronized multi-drone operations over a single Crazyradio link, testing the power and reliability of the Rust library under real flight conditions.

## Repository contents

- `main.py` — Swarm controller that connects, arms, flies, and lands all drones concurrently
- `swarm.json` — Drone definitions (IDs, radio URIs, platform types)
- `crazyflie-firmware/` — Bitcraze firmware submodule, fixed at release **2025.12.1**
- `flasher/` — Firmware build & flash utility (see [flasher/README.md](flasher/README.md))

## Setup

Requires Python 3.13+. [uv](https://github.com/astral-sh/uv) is recommended.

```bash
uv sync
```

## Configuration

Drones are defined in `swarm.json`. Each entry specifies an ID, radio URI, and platform type (`cf2`, `cf21bl`, etc.):

```json
{
  "id": 1,
  "uri": "radio://0/90/2M/ABAD1DEA01",
  "platform": "cf21bl"
}
```

## Usage

### Run the swarm controller

```bash
uv run main.py
```

This connects to all drones defined in `swarm.json`, then runs the following mission sequence:

1. Blink LEDs 3 times to confirm connection
2. Arm all brushless drones
3. Run a preflight check (waits up to 5 s for all drones to report ready)
4. Take off to 0.5 m
5. Land
6. Disarm all drones
7. Blink LEDs 2 times to confirm completion

Drones are always disarmed on exit, even if the mission fails.

### Options

```
--config PATH    Path to swarm config file (default: swarm.json)
--no-cache       Disable TOC caching (caching is enabled by default)
--ids ID [ID …]  Only control specific drone IDs (e.g. --ids 1 2 3)
```

### Examples

Run with a subset of drones:

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
