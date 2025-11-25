#!/usr/bin/env bash
# Stop CARLA simulator
set -e

if [ $# -lt 2 ]; then
    echo "Usage: $0 <version> <port>"
    exit 1
fi

VERSION="$1"
PORT="$2"
UNIT_NAME="carla-run-$VERSION-$PORT"

if systemctl --user is-active --quiet "$UNIT_NAME"; then
    echo "Stopping CARLA $VERSION on port $PORT..."
    systemctl --user stop "$UNIT_NAME"
    echo "CARLA stopped"
else
    echo "CARLA $VERSION is not running on port $PORT"
fi
