# Autoware CARLA Bridge

Native ROS 2 bridge between CARLA and Autoware. Written in Rust.

## Features

- Auto-detects Autoware and reads sensor config from URDF
- Spawns vehicles in CARLA with matching sensors
- Bridges sensor data (LiDAR, cameras, GNSS, IMU) to ROS 2
- Supports CARLA 0.9.16

## Quick Start

Ensure `DISPLAY` is set to an available display before starting CARLA or Autoware.

```bash
# 1. Start CARLA (runs as background service)
just carla-start

# 2. Start demo (Autoware + bridge + scenario)
just run-demo

# Check status
just carla-status

# Stop
just carla-stop
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

# 3. Configure CARLA path
# Edit CARLA_DIR in third_party/carla/run.sh to point to your installation
vi third_party/carla/run.sh

# 4. Install deps and build
just setup
just build
```

## Running

### Demo Mode

```bash
just carla-start             # Start CARLA service
just run-demo                # Start Autoware + bridge + scenario
just carla-stop              # Stop CARLA service
```

### Manual

```bash
just carla-start      # Start CARLA
just run-autoware     # Start Autoware (foreground)
just run-bridge       # Start bridge (foreground)
```

### Direct

```bash
source install/setup.bash
ros2 launch autoware_carla_bridge autoware_carla_bridge.launch.xml carla_port:=2000
```

## Configuration

**CARLA path:** Edit `CARLA_DIR` in `third_party/carla/run.sh` (see Install step 3)

**Autoware maps:** Pre-converted maps in `data/carla-autoware-bridge/` (Town01-10)

## Verify

```bash
ros2 topic list
ros2 topic echo /clock
just carla-status
```

## Troubleshooting

| Issue               | Solution                                              |
|---------------------|-------------------------------------------------------|
| Can't find Autoware | Check `ls -la third_party/autoware/autoware_repo`     |
| CARLA run script not found | Edit `CARLA_DIR` in `third_party/carla/run.sh`  |
| CARLA timeout       | Verify CARLA running: `just carla-status`             |
| Build fails         | Run `just clean && just build`                        |

## Docs

- [Sensor Configuration](docs/design/sensor-configuration-strategy.md)
- [Map Generation Guide](docs/guides/automated-map-generation.md)
- [Development Roadmap](docs/roadmap/)

## License

TBD
