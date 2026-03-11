#!/usr/bin/env bash
# Run CARLA simulator.
# Edit CARLA_DIR below to point to your CARLA installation.
# Requires a display (DISPLAY must be set before running).
set -e

CARLA_DIR="$HOME/Downloads/CARLA_0.9.16"

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

PORT="${CARLA_PORT:-2000}"
exec "$CARLA_DIR/CarlaUE4.sh" -quality-level=Low -carla-rpc-port="$PORT"
