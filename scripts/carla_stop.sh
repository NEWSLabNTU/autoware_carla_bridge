#!/usr/bin/env bash
# Stop CARLA simulator
set -e

PORT="${1:-2000}"
UNIT_NAME="carla-run-$PORT"

if systemctl --user is-active --quiet "$UNIT_NAME"; then
    echo "Stopping CARLA on port $PORT..."
    systemctl --user stop "$UNIT_NAME"
    echo "CARLA stopped"
else
    echo "CARLA is not running on port $PORT"
fi
