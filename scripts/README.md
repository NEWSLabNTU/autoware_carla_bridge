# Scripts

## CARLA Service

- **`carla_start.sh`** - Start CARLA simulator as a systemd user service
- **`carla_stop.sh`** - Stop CARLA simulator service

## Demo

- **`demo_scenario.py`** - Load CARLA map, configure weather/traffic, monitor actors. Waits for CARLA connection with retry.

## Setup

- **`install_deps.sh`** - Install system dependencies
- **`install_autoware_debian.sh`** - Install Autoware 1.5.0 Debian packages
- **`download_carla_maps_for_autoware.sh`** - Download pre-converted CARLA maps for Autoware

## Map Tools

- **`compare_lanelet2.py`** - Compare two Lanelet2 maps (structure, attributes, traffic lights)

## Debug

- **`debug_carla_actors.py`** - List all actors in a running CARLA server
- **`debug_carla_vehicles.py`** - List vehicle blueprints and their attributes
- **`check_spawn_points.py`** - Show valid spawn points for current CARLA map
- **`inspect_carla_map.py`** - Inspect OpenDRIVE map data from CARLA
- **`find_map_offset.py`** - Find coordinate offset between CARLA and Autoware maps
