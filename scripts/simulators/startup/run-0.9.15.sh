#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir/../carla-0.9.15"

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export DISPLAY=:1

./CarlaUE4.sh -quality-level=Low -carla-rpc-port=3000
