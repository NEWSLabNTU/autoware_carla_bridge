# Autoware Autonomous Driving Scripts

## Overview

Python scripts for autonomous driving with modern Autoware (2024/2025).

## Scripts

### `drive_in_autoware.py`

Main autonomous driving script that:
1. Loads initial and goal poses from `poses.json`
2. Initializes localization using `/api/localization/initialize` service
3. Clears existing route and sets new route to goal
4. Engages autonomous mode via `/api/operation_mode/change_operation_mode`
5. Monitors progress until goal is reached

**Requirements:**
- Autoware planning simulator running (`just autoware start`)
- Valid `poses.json` with connected poses in the lanelet2 map

**Usage:**
```bash
./scripts/drive_in_autoware.py
```

**Modern Autoware API:**
- `/api/localization/initialize` - Initialize localization with pose
- `/api/routing/set_route_points` - Set route to goal
- `/api/routing/clear_route` - Clear existing route
- `/api/operation_mode/change_to_autonomous` - Change to autonomous mode

**Timing Requirements:**
- 15s wait after localization initialization (vehicle spawn + diagnostics)
- 10s wait after route is set (planning/control pipeline activation)
- These waits are critical for autonomous mode engagement in automated workflows

### `read_poses.py`

Captures initial pose and goal pose from Autoware/RViz and saves to `poses.json`.

**Usage:**
```bash
# In RViz:
# 1. Click "2D Pose Estimate" and set initial pose
# 2. Click "2D Goal Pose" and set goal pose

# Run script to capture:
./scripts/read_poses.py

# Poses saved to: scripts/poses.json
```

**Topics subscribed:**
- `/initialpose` - Initial pose from RViz
- `/planning/mission_planning/goal` - Goal pose from Autoware

### `get_carla_spawn_points.py`

Gets valid spawn points from CARLA and suggests pose pairs for autonomous driving.

**Requirements:**
- CARLA simulator running (`just carla start 0.9.16 2000`)

**Usage:**
```bash
# Get spawn points and suggest valid pairs
./scripts/get_carla_spawn_points.py --town Town01

# With custom distance range
./scripts/get_carla_spawn_points.py --town Town01 --min-distance 50 --max-distance 100
```

**Output:**
- Lists all spawn points with coordinates
- Suggests pairs of poses that are 50-150m apart in the same direction

## Troubleshooting

### Route Planning Fails

**Error:** `The planned route is empty`

**Cause:** Initial and goal poses are not connected in the lanelet2 map.

**Solutions:**
1. Use RViz to select poses manually (guaranteed to be on valid lanes)
2. Use `get_carla_spawn_points.py` to find valid CARLA spawn points
3. Check Autoware logs: `just autoware logs -n 50`
4. Verify map is loaded: `ros2 topic echo /api/autoware/get/map/info/hash --once`

### Localization Initialization Fails

**Error:** `Localization initialize service not available`

**Solutions:**
- Check if Autoware is running: `just autoware status`
- Verify map is loaded: `ros2 topic echo /api/autoware/get/map/info/hash --once`
- Wait for Autoware to fully start (may take 10-15 seconds after launch)
- Check if the service exists: `ros2 service list | grep localization`

## Files

- `poses.json` - Current initial and goal poses (auto-generated)
- `poses_town01_example.json` - Example poses for Town01 (may need adjustment)

## References

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
- [Planning Simulation Tutorial](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)
