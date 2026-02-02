# Flasher Utility

This directory contains the firmware flashing utility for the Crazyflie swarm.

## Why a separate directory?

The flashing utility uses the **stable cflib** from PyPI because bootloader functionality is not yet implemented in the Rust-based cflib variant. The main project uses the Rust-based cflib for performance testing.

## Setup

Install dependencies in this directory:

```bash
cd flasher
uv sync
```

## Usage

Flash all drones:
```bash
uv run flash_all.py --all
```

Flash specific drones by ID:
```bash
uv run flash_all.py --ids 1 2 3
```

Flash a range of drones:
```bash
uv run flash_all.py --range 1-6
```

Flash only cf21bl platform drones:
```bash
uv run flash_all.py --platform cf21bl
```

## Configuration

The script reads drone configurations from `../swarm.json` in the repository root.
