#!/usr/bin/env bash
# Stop Autoware simulator
set -e

UNIT_NAME="autoware-simulator"

if systemctl --user is-active --quiet "$UNIT_NAME"; then
    echo "Stopping Autoware simulator..."
    systemctl --user stop "$UNIT_NAME"
    echo "Autoware stopped"
else
    echo "Autoware is not running"
fi
