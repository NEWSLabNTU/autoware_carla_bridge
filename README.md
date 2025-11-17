# Autoware CARLA Bridge

Native ROS 2 bridge between CARLA and Autoware. Written in Rust.

## Features

- Auto-detects Autoware and reads sensor config from URDF
- Spawns vehicles in CARLA with matching sensors
- Bridges sensor data (LiDAR, cameras, GNSS, IMU) to ROS 2
- Supports CARLA 0.9.14, 0.9.15, 0.9.16

## Quick Start

```bash
# One command to run everything
env DISPLAY=:1 just demo start

# Check status
just demo status

# Stop
just demo stop
```

## Setup

### Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Built Autoware 2025.02 workspace
- CARLA 0.9.16 (download from [carla.org](https://carla.org))
- Rust + Just (`cargo install just`)

### Install

```bash
# 1. Clone and initialize
git clone <repo-url>
cd autoware_carla_bridge
git submodule update --init --recursive

# 2. Link your Autoware workspace
mkdir -p third_party/autoware
ln -s /path/to/autoware/workspace third_party/autoware/autoware_repo

# 3. Configure CARLA symlinks (for just commands)
cd third_party/carla
ln -s /path/to/CARLA_0.9.16 carla-0.9.16
ln -s /path/to/CARLA_0.9.15 carla-0.9.15  # optional
cd ../..

# 4. Install deps and build
just install-deps
just build
```

**Run scripts**: CARLA run scripts (`run-0.9.*.sh`) are provided in `third_party/carla/`. Autoware run script (`run-planning-simulation.sh`) is provided in `third_party/autoware/`.

## Running

### Demo Mode

```bash
env DISPLAY=:1 just demo start    # Start all
just demo logs                     # View logs
just demo stop                     # Stop all
```

### Manual

```bash
just carla start 0.9.16 2000      # Start CARLA
just autoware start                # Start Autoware
just bridge start                  # Start bridge
```

### Direct

```bash
source install/setup.bash
ros2 run autoware_carla_bridge autoware_carla_bridge --carla-port 2000
```

## Configuration

**CARLA symlinks:** Configured in `third_party/carla/carla-0.9.*/` (see Install step 3)

**Autoware maps:** Uses `~/autoware_map/sample-map-planning` in demo mode

## Verify

```bash
ros2 topic list
ros2 topic echo /clock
just bridge logs
```

## Troubleshooting

| Issue               | Solution                                              |
|---------------------|-------------------------------------------------------|
| Can't find Autoware | Check `ls -la third_party/autoware/autoware_repo`     |
| CARLA run script not found | Configure symlinks: `ls -la third_party/carla/` |
| CARLA timeout       | Verify CARLA running: `just carla status 0.9.16 2000` |
| Build fails         | Run `just clean && just build`                        |

## Docs

- [Sensor Configuration](docs/sensor-configuration-strategy.md)
- [Map Integration](docs/carla-autoware-map-integration.md)
- [Development Roadmap](docs/roadmap.md)

## License

TBD
