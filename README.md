# oxidized-swarm

Minimal demo that flies two Crazyflie drones through a figure-8 trajectory on a single Crazyradio, using the Rust-based [cflib](https://github.com/bitcraze/crazyflie-lib-python) Python bindings.

## What it does

`main.py` is a single flat script that walks through a complete flight sequence:

1. Connect to both drones
2. Read battery voltage (`pm.vbat`)
3. Upload a polynomial trajectory to each drone's memory
6. Arm, take off, fly the figure-8, land, disarm
7. Disconnect

All steps run concurrently on both drones using `asyncio.gather`.

## Setup

Requires Python 3.10+ and a positioning system (Lighthouse, LPS, flow deck, etc.).

**With [uv](https://github.com/astral-sh/uv) (recommended):**

```bash
uv sync
```

**With pip:**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install .
```

Then edit the `URIS` list at the top of `main.py` to match your drones:

```python
URIS = [
    "radio://0/80/2M/E7E7E7E7E7",
    "radio://0/80/2M/E7E7E7E702",
]
```

## Usage

```bash
uv run main.py        # with uv
python main.py        # with pip/venv
```

## Concurrent vs sequential operations

The library is fully async. This demo runs every step concurrently across all drones with `asyncio.gather`:

```python
# Concurrent - all drones execute at the same time
await asyncio.gather(*[
    cf.high_level_commander().take_off(1.0, None, 2.0, None)
    for cf in cfs
])
```

If you prefer sequential execution, just use a regular loop:

```python
# Sequential - one drone at a time
for cf in cfs:
    await cf.high_level_commander().take_off(1.0, None, 2.0, None)
```

Both styles work. The concurrent approach is faster since the Rust library multiplexes all drones over a single radio link.
