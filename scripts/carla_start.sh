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

# LimitCORE=infinity below is what makes a segfault leave evidence. The server has crashed
# repeatedly under long uptime (docs/issues/017) and no core was ever kept: the unit inherited a
# soft core limit of 0, so "(core dumped)" in the journal wrote nothing. The hard limit was
# already infinity; only the soft one was in the way.
#
# core_pattern on this host pipes to apport, which is enabled and demonstrably writing reports
# for other binaries into /var/crash. So with the soft limit raised the next CARLA segfault
# lands there as a .crash carrying the core and a stack, without needing root -- changing
# core_pattern would.
#
# The cost is disk: the server runs at about 2.4 GB resident, so expect a report of that order.
# apport keeps one report per executable until it is cleared, so this does not grow per crash.
echo "Starting CARLA on port $PORT (map=${CARLA_MAP:-package default}, quality=$CARLA_QUALITY, render=$CARLA_RENDER, DISPLAY=${DISPLAY:-none})..."

systemd-run --user \
    --unit="$UNIT_NAME" \
    --setenv=DISPLAY="$DISPLAY" \
    --setenv=CARLA_PORT="$PORT" \
    --setenv=CARLA_MAP="$CARLA_MAP" \
    --setenv=CARLA_QUALITY="$CARLA_QUALITY" \
    --setenv=CARLA_RENDER="$CARLA_RENDER" \
    ${CARLA_DIR:+--setenv=CARLA_DIR="$CARLA_DIR"} \
    ${CARLA_EXTRA_ARGS:+--setenv=CARLA_EXTRA_ARGS="$CARLA_EXTRA_ARGS"} \
    -p Restart=on-failure \
    -p RestartSec=10 \
    -p StartLimitBurst=5 \
    -p StartLimitIntervalSec=300 \
    -p TimeoutStartSec=300 \
    -p LimitCORE=infinity \
    -p ExecStartPost="/usr/bin/env bash $SCRIPT_DIR/carla_wait_ready.sh" \
    bash "$RUN_SCRIPT"

echo "CARLA started on port $PORT"
echo "Use 'just carla-status' to check status"
echo "Use 'just carla-logs' to view logs"
echo "Use 'just carla-stop' to stop"
