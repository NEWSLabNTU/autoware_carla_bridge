#!/usr/bin/env bash
# Stop demo environment (Bridge + Autoware + CARLA)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Default configuration
CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
CARLA_PORT="${CARLA_PORT:-2000}"

echo "=== Stopping Demo Environment ==="
echo ""

# Stop in reverse order
echo "Step 1/3: Stopping bridge..."
cd "$PROJECT_ROOT"
just bridge stop
echo ""

echo "Step 2/3: Stopping Autoware..."
"$SCRIPT_DIR/autoware_stop.sh"
echo ""

echo "Step 3/3: Stopping CARLA..."
"$SCRIPT_DIR/carla_stop.sh" "$CARLA_VERSION" "$CARLA_PORT"
echo ""

echo "=== Demo Environment Stopped ==="
