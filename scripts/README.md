# Scripts

## CARLA Service

- **`carla_start.sh`** - Start CARLA simulator as a systemd user service
  (`carla_start.sh [PORT] [map:=] [quality:=epic|low] [render:=] [extra_args:=]`)
- **`carla_args.bash`** - The `name:=value` CARLA launch arguments `carla_start.sh` and
  `third_party/carla/run.sh` share, and the refusal of the env variables they replaced
- **`carla_stop.sh`** - Stop CARLA simulator service
- **`carla_load_town.py`** - Load a town unless it is already loaded (`--port`, `--town`).
  `just sim` runs it before its launch: a load is a new CARLA episode, which ends a running
  acb_bridge and its stack

## Demo

- **`demo_scenario.py`** - Load CARLA map, spawn the hero, tick CARLA, destroy the hero's sensors and then the hero on exit. Waits for CARLA connection with retry. (`just sim` and `just scenario` run `acb_scenario`'s copy, `src/acb_scenario/acb_scenario/demo_scenario.py`.)

## Vehicle check

- **`vehicle_check_host.py`** - The scene `just vehicle-check` drives: a hero on a straight lane, CARLA ticked synchronously in real time (`--port`, `--blueprint`)

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
