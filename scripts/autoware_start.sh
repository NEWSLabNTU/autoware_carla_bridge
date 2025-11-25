#!/usr/bin/env bash
# Start Autoware simulator via systemd
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UNIT_NAME="autoware-simulator"
RUN_SCRIPT="$PROJECT_ROOT/third_party/autoware/run-e2e-simulator.sh"

if [ ! -f "$RUN_SCRIPT" ]; then
    echo "Error: Autoware run script not found: $RUN_SCRIPT"
    echo "Please configure third_party/autoware/run-e2e-simulator.sh"
    exit 1
fi

# Stop any existing unit and reset failed state
echo "Ensuring no existing $UNIT_NAME unit..."
systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
sleep 0.5

echo "Starting Autoware simulator..."

# Build setenv arguments for ROS environment variables
SETENV_ARGS=()
if [ -n "$RMW_IMPLEMENTATION" ]; then
    SETENV_ARGS+=(--setenv=RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
fi
if [ -n "$ROS_DOMAIN_ID" ]; then
    SETENV_ARGS+=(--setenv=ROS_DOMAIN_ID="$ROS_DOMAIN_ID")
fi
if [ -n "$ROS_LOCALHOST_ONLY" ]; then
    SETENV_ARGS+=(--setenv=ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY")
fi

# Start Autoware using systemd-run with the run script
systemd-run --user \
    --unit="$UNIT_NAME" \
    "${SETENV_ARGS[@]}" \
    bash "$RUN_SCRIPT"

echo "Autoware simulator started"
echo "Use 'just autoware status' to check status"
echo "Use 'just autoware logs' to view logs"
echo "Use 'just autoware stop' to stop"
