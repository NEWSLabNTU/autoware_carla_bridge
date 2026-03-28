#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
project_root=$( cd "$script_dir/../.." && pwd )

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export DISPLAY=:1

# Source only the overlay workspace - it will automatically source underlays via prefix chain
source "$project_root/install/setup.bash"

# Use carla_autoware_launch with CARLA-optimized NDT parameters
# NOTE: use_sim_time:=true is critical for CARLA simulation - all nodes must use sim time
play_launch launch --web-addr 0.0.0.0:8080 \
    -c "$project_root/config/play_launch.yaml" \
    carla_autoware_launch carla_simulator.launch.xml \
    map_path:="$project_root/data/carla-autoware-bridge/Town01" \
    vehicle_model:=carla_vehicle \
    sensor_model:=carla_sensor_kit \
    use_sim_time:=true
