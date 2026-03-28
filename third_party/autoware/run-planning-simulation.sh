#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
project_root=$( cd "$script_dir/../.." && pwd )

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export DISPLAY=:1

source "$script_dir/setup.bash"
source "$project_root/install/setup.bash"

play_launch launch --web-addr 0.0.0.0:8080 \
    -c "$project_root/config/play_launch.yaml" \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$project_root/data/carla-autoware-bridge/Town01" \
    vehicle_model:=carla_vehicle \
    sensor_model:=carla_sensor_kit
