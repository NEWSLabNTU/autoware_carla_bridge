# Scripts Directory

Utility scripts for the autoware_carla_bridge project.

## Available Scripts

### `setup_carla.py`
**Configure CARLA map and synchronous mode**

Python script to connect to CARLA and configure the simulation environment.

**Usage:**
```bash
# Load a specific map with synchronous mode
./scripts/setup_carla.py --map Town10HD --sync

# Load map on custom port
./scripts/setup_carla.py --port 2000 --map Town01 --sync

# Switch to asynchronous mode
./scripts/setup_carla.py --async

# Set custom fixed delta for synchronous mode
./scripts/setup_carla.py --sync --fixed-delta 0.1
```

**Options:**
- `--port`, `-p`: CARLA server port (default: 2000)
- `--host`: CARLA server host (default: localhost)
- `--map`, `-m`: Map to load (e.g., Town01, Town10HD)
- `--timeout`, `-t`: Connection timeout in seconds (default: 10.0)
- `--sync`: Enable synchronous mode
- `--async`: Enable asynchronous mode
- `--fixed-delta`: Fixed delta seconds for synchronous mode (default: 0.05)

**Examples:**
```bash
# Quick setup for testing
./scripts/setup_carla.py --map Town01 --sync

# Production setup with specific timing
./scripts/setup_carla.py --map Town10HD --sync --fixed-delta 0.05

# Connect to remote CARLA server
./scripts/setup_carla.py --host 192.168.1.100 --port 2000 --map Town03
```

---

### `download_carla_maps_for_autoware.sh`
**Download pre-converted CARLA maps for Autoware**

Downloads and extracts TUMFTM's pre-converted Lanelet2 maps from sync&share.

**Usage:**
```bash
./scripts/download_carla_maps_for_autoware.sh
```

**Features:**
- Maps saved to `data/carla-autoware-bridge/`
- Idempotent (safe to run multiple times)
- Automatic resume if download interrupted
- SHA256 checksum verification

**Details:**
- **Source**: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/
- **Size**: ~240 MB compressed
- **Includes**: Town10HD and other CARLA town maps
- **Format**: Lanelet2 (.osm) vector maps

---

### `install_deps.sh`
**Install build dependencies**

Installs required colcon plugins for building the ROS 2 workspace.

**Usage:**
```bash
./scripts/install_deps.sh
```

**Installs:**
- `python3-pip`
- `python3-colcon-common-extensions`
- `python3-colcon-cargo` (via pip)
- `python3-colcon-ros-cargo` (via pip)

---

## See Also

- [`../docs/carla-map-acquisition-guide.md`](../docs/carla-map-acquisition-guide.md) - Complete guide for CARLA map generation
- [`../README.md`](../README.md) - Main project documentation
- [`../CLAUDE.md`](../CLAUDE.md) - Development session history

---

**Last Updated**: 2025-11-18
