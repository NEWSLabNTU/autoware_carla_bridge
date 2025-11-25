#!/usr/bin/env bash
# Start demo environment (CARLA + Autoware + Bridge)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Default configuration
CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
CARLA_PORT="${CARLA_PORT:-2000}"
BRIDGE_PORT="${BRIDGE_PORT:-2000}"

echo "=== Starting Demo Environment ==="
echo "CARLA Version: $CARLA_VERSION, Port: $CARLA_PORT"
echo ""

# Check if DISPLAY is set for CARLA
if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY environment variable is not set"
    echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
    exit 1
fi

# Check if GNU Parallel is available
if ! command -v parallel &> /dev/null; then
    echo "Error: GNU Parallel is not installed"
    echo "Please install it: sudo apt install parallel"
    exit 1
fi

echo "=== Phase 1: Starting CARLA and Autoware in Parallel ==="
echo ""

# Define function for starting CARLA
start_carla() {
    local CARLA_VERSION=$1
    local CARLA_PORT=$2

    echo "[CARLA] Starting CARLA $CARLA_VERSION on port $CARLA_PORT..."
    "$SCRIPT_DIR/carla_start.sh" "$CARLA_VERSION" "$CARLA_PORT" > /tmp/demo-carla-start.log 2>&1

    # Wait for CARLA to be ready
    echo "[CARLA] Waiting for CARLA to be ready..."
    for i in {1..60}; do
        if timeout 3 python3 -c "import carla; client = carla.Client('127.0.0.1', $CARLA_PORT); client.set_timeout(2.0); client.get_world()" 2>/dev/null; then
            echo "[CARLA] ✓ CARLA is ready (${i}s elapsed)"
            break
        fi
        sleep 1
    done

    # Configure CARLA
    echo "[CARLA] Configuring CARLA (Town01, synchronous mode)..."
    python3 "$SCRIPT_DIR/setup_carla.py" --port "$CARLA_PORT" --map Town01 --sync --timeout 60
    echo "[CARLA] ✓ CARLA configured successfully"
}
export -f start_carla

# Define function for starting Autoware
start_autoware() {
    echo "[Autoware] Starting Autoware simulator..."
    "$SCRIPT_DIR/autoware_start.sh" > /tmp/demo-autoware-start.log 2>&1
    echo "[Autoware] Autoware started"

    # Wait for Autoware to initialize
    echo "[Autoware] Waiting for Autoware to initialize..."
    sleep 15

    # Check if Autoware services are available
    echo "[Autoware] Checking Autoware services..."
    for i in {1..10}; do
        if timeout 3 ros2 service list 2>/dev/null | grep -q "/api/localization/initialize"; then
            echo "[Autoware] ✓ Autoware services are ready"
            break
        fi
        sleep 1
    done
}
export -f start_autoware

# Export SCRIPT_DIR for use in parallel functions
export SCRIPT_DIR

# Run both tasks in parallel using GNU Parallel
echo "Starting CARLA and Autoware in parallel..."
parallel --line-buffer --halt now,fail=1 ::: \
    "start_carla $CARLA_VERSION $CARLA_PORT" \
    "start_autoware"

if [ $? -ne 0 ]; then
    echo "✗ Failed to start services. Check logs:"
    echo "  - CARLA:    /tmp/demo-carla-start.log"
    echo "  - Autoware: /tmp/demo-autoware-start.log"
    exit 1
fi

echo "✓ Both CARLA and Autoware are ready"
echo ""

# Start Bridge
echo "=== Phase 2: Starting Bridge ==="
cd "$PROJECT_ROOT"
just bridge start "$BRIDGE_PORT"

# Wait for bridge to be ready to receive initial pose
echo "Waiting for bridge to initialize..."
sleep 5

echo "✓ Bridge started and ready"
echo ""

echo "=== Demo Environment Started Successfully ==="
echo ""
echo "All services are now running:"
echo "  - CARLA:    just carla status $CARLA_VERSION $CARLA_PORT"
echo "  - Autoware: just autoware status"
echo "  - Bridge:   just bridge status"
echo ""
echo "Next steps:"
echo "  1. Run: just drive           # Run autonomous driving"
echo "  2. Watch logs: just bridge logs -f"
echo ""
echo "Management commands:"
echo "  - just demo status   # Check all services"
echo "  - just demo logs     # View all logs"
echo "  - just demo stop     # Stop all services"
