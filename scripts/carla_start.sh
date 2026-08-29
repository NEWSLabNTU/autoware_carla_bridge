#!/usr/bin/env bash
# Start CARLA simulator via systemd
set -e

PORT="${1:-2000}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Launch knobs, forwarded to the unit so `just carla-start` and a bare run.sh agree.
# See third_party/carla/run.sh for what each one does.
CARLA_MAP="${CARLA_MAP-Town01}"
CARLA_QUALITY="${CARLA_QUALITY:-Low}"
CARLA_RENDER="${CARLA_RENDER:-offscreen}"

if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY environment variable is not set"
    echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
    exit 1
fi

RUN_SCRIPT="$PROJECT_ROOT/third_party/carla/run.sh"
if [ ! -f "$RUN_SCRIPT" ]; then
    echo "Error: CARLA run script not found: $RUN_SCRIPT"
    exit 1
fi

UNIT_NAME="carla-run-$PORT"

# Stop any existing unit and reset failed state
echo "Ensuring no existing $UNIT_NAME unit..."
systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
sleep 0.5

echo "Starting CARLA on port $PORT (map=${CARLA_MAP:-package default}, quality=$CARLA_QUALITY, render=$CARLA_RENDER, DISPLAY=${DISPLAY:-none})..."

systemd-run --user \
    --unit="$UNIT_NAME" \
    --setenv=DISPLAY="$DISPLAY" \
    --setenv=CARLA_PORT="$PORT" \
    --setenv=CARLA_MAP="$CARLA_MAP" \
    --setenv=CARLA_QUALITY="$CARLA_QUALITY" \
    --setenv=CARLA_RENDER="$CARLA_RENDER" \
    ${CARLA_DIR:+--setenv=CARLA_DIR="$CARLA_DIR"} \
    -p Restart=on-failure \
    -p RestartSec=10 \
    -p StartLimitBurst=5 \
    -p StartLimitIntervalSec=300 \
    -p TimeoutStartSec=300 \
    -p ExecStartPost="/usr/bin/env bash $SCRIPT_DIR/carla_wait_ready.sh" \
    bash "$RUN_SCRIPT"

echo "CARLA started on port $PORT"
echo "Use 'just carla-status' to check status"
echo "Use 'just carla-logs' to view logs"
echo "Use 'just carla-stop' to stop"
