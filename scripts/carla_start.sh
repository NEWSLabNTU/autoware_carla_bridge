#!/usr/bin/env bash
# Start CARLA simulator via systemd
set -e

if [ $# -lt 2 ]; then
    echo "Usage: $0 <version> <port>"
    echo "Example: $0 0.9.16 2000"
    exit 1
fi

VERSION="$1"
PORT="$2"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY environment variable is not set"
    echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
    exit 1
fi

# Check if run script exists
RUN_SCRIPT="$PROJECT_ROOT/third_party/carla/run-$VERSION.sh"
if [ ! -f "$RUN_SCRIPT" ]; then
    echo "Error: CARLA run script not found: $RUN_SCRIPT"
    echo "Available versions: 0.9.14, 0.9.15, 0.9.16"
    echo "Please configure symlinks in third_party/carla/"
    exit 1
fi

# Use a unique transient unit name to avoid conflicts with template units
UNIT_NAME="carla-run-$VERSION-$PORT"

# Stop any existing unit (running or not) and reset failed state
echo "Ensuring no existing $UNIT_NAME unit..."
systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
sleep 0.5

echo "Starting CARLA $VERSION on port $PORT with DISPLAY=$DISPLAY..."

# Start CARLA using systemd-run with the run script
systemd-run --user \
    --unit="$UNIT_NAME" \
    --setenv=DISPLAY="$DISPLAY" \
    --setenv=CARLA_PORT="$PORT" \
    bash "$RUN_SCRIPT"

echo "CARLA $VERSION started on port $PORT"
echo "Use 'just carla status $VERSION $PORT' to check status"
echo "Use 'just carla logs $VERSION $PORT' to view logs"
echo "Use 'just carla stop $VERSION $PORT' to stop"
