# Autoware Integration Scripts

This directory contains scripts and tools for launching and managing Autoware with the CARLA bridge.

## Prerequisites

- Autoware 2025.02 workspace built at `autoware_repo/` (symlink)
- Sample map at `$HOME/autoware_map/sample-map-planning/`
- `play-launch` tool installed (see below)

## Quick Start

### 1. Install Dependencies

```bash
make install-deps
```

This installs the `play-launch` tool, which is used to launch Autoware with proper environment setup.

### 2. Launch Autoware

```bash
make launch
```

This launches the Autoware planning simulator with:
- **Map**: `sample-map-planning`
- **Vehicle Model**: `sample_vehicle`
- **Sensor Model**: `sample_sensor_kit`

The simulator runs in the background and logs are saved to `play_log/` directory.

## Available Targets

Run `make help` to see all available targets:

```
  help                 Display available targets
  install-deps         Install play-launch tool for launching Autoware
  launch               Launch Autoware planning simulator with sample vehicle and sensor kit
```

## Configuration

### Changing Vehicle or Sensor Models

Edit the `launch` target in `Makefile` to use different models:

```makefile
launch:
	cd autoware_repo && \
	. install/setup.sh && \
	play_launch launch \
	  autoware_launch planning_simulator.launch.xml \
	  map_path:=$$HOME/autoware_map/sample-map-planning \
	  vehicle_model:=my_custom_vehicle \
	  sensor_model:=carla_sensor_kit
```

Available sensor models (in Autoware repo):
- `sample_sensor_kit` - Default full sensor suite
- `awsim_sensor_kit` - AWSIM-compatible sensors
- `carla_sensor_kit` - Custom kit for CARLA (if created)

### Changing Map

Set `map_path` to a different map directory:

```bash
# Manually launch with different map
cd autoware_repo && \
. install/setup.sh && \
play_launch launch \
  autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/my-custom-map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

## Integration with CARLA Bridge

Once Autoware is running, the CARLA bridge can detect and connect to it:

1. **Launch Autoware** (this script)
2. **Launch CARLA Simulator** (`scripts/simulators/install.sh carla`)
3. **Launch CARLA Bridge** (with Autoware auto-detection)

The bridge will:
- Detect the running Autoware instance via `/robot_description` topic
- Parse sensor configuration from URDF
- Spawn a vehicle in CARLA with matching sensors
- Publish sensor data to ROS topics
- Subscribe to control commands from Autoware

See `docs/autoware-integration-design.md` for detailed architecture.

## Troubleshooting

### play-launch Not Found

If `make launch` fails with "play_launch: command not found":

```bash
make install-deps
```

### RMW Implementation Error

If you see "RMW implementation not installed" error, ensure `rmw_cyclonedds_cpp` is installed:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

The Makefile automatically sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

### Map Not Found

Download the sample map:

```bash
mkdir -p ~/autoware_map
cd ~/autoware_map
git clone https://github.com/autowarefoundation/sample-map-planning.git
```

### Autoware Won't Stop

If `make stop` doesn't work, manually kill processes:

```bash
pkill -9 -f "planning_simulator"
pkill -9 -f "play_launch"
```

## Logs

Logs are stored in `play_log/` directory:
- `play_log/launcher.log` - Launch script output
- `play_log/ros2_launch_*.log` - ROS 2 launch logs
- `play_log/nodes/` - Individual node logs

To view logs in real-time:

```bash
tail -f play_log/launcher.log
```

## References

- **play-launch**: https://github.com/NEWSLabNTU/play-launch
- **Autoware Documentation**: https://autowarefoundation.github.io/autoware-documentation/
- **Planning Simulator Guide**: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/
