#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
project_root=$( cd "$script_dir/../.." && pwd )
cd "$script_dir/autoware_repo"

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export DISPLAY=:1

source install/setup.sh
ros2 launch \
     autoware_launch planning_simulator.launch.xml \
     map_path:="$project_root/data/carla-autoware-bridge/Town01" \
     vehicle_model:=sample_vehicle \
     sensor_model:=sample_sensor_kit
