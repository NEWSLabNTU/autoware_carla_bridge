#!/usr/bin/env bash
#
# Test Environment Setup Script for autoware_carla_bridge
#
# This script automates the test environment setup:
# 1. Installs and starts CARLA simulator service
# 2. Waits for CARLA to be ready
# 3. Spawns test vehicles
# 4. Runs the bridge
# 5. Cleans up on exit
#

set -e

# Get the project root directory (parent of scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default values
CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
CARLA_PORT="${CARLA_PORT:-3000}"
SERVICE_NAME="carla-${CARLA_VERSION}@${CARLA_PORT}"

# Flags
SKIP_SERVICE_INSTALL=false
SKIP_AGENT_SPAWN=false

usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Automates test environment setup for autoware_carla_bridge:
  1. Installs and starts CARLA simulator service
  2. Waits for CARLA to be ready
  3. Spawns test vehicles
  4. Runs the bridge
  5. Cleans up on exit (Ctrl+C)

Options:
  -v, --version VERSION    CARLA version to use (default: 0.9.16)
                          Supported: 0.9.14, 0.9.15, 0.9.16
  -p, --port PORT         CARLA RPC port (default: 3000)
  -s, --skip-install      Skip CARLA service installation (assume already installed)
  -n, --no-agents         Skip agent spawning (run bridge without vehicles)
  -h, --help              Show this help message

Environment Variables:
  CARLA_VERSION           CARLA version (overridden by -v)
  CARLA_PORT             CARLA port (overridden by -p)

Examples:
  # Default: CARLA 0.9.16 on port 3000 with agent spawning
  $0

  # Use CARLA 0.9.15 on port 2000
  $0 -v 0.9.15 -p 2000

  # Skip service installation (already running)
  $0 -s

  # Run without spawning vehicles
  $0 -n

EOF
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--version)
            CARLA_VERSION="$2"
            shift 2
            ;;
        -p|--port)
            CARLA_PORT="$2"
            shift 2
            ;;
        -s|--skip-install)
            SKIP_SERVICE_INSTALL=true
            shift
            ;;
        -n|--no-agents)
            SKIP_AGENT_SPAWN=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo -e "${RED}Error: Unknown option '$1'${NC}"
            usage
            ;;
    esac
done

# Update service name with parsed values
SERVICE_NAME="carla-${CARLA_VERSION}@${CARLA_PORT}"

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}Cleaning up...${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    # Kill bridge if running
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo -e "${YELLOW}Stopping bridge (PID: $BRIDGE_PID)...${NC}"
        kill -TERM "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Bridge stopped${NC}"
    fi

    # Note: We don't stop CARLA service as user might want to keep it running
    echo ""
    echo -e "${BLUE}Note: CARLA service is still running.${NC}"
    echo -e "${BLUE}To stop: systemctl --user stop ${SERVICE_NAME}${NC}"
    echo -e "${BLUE}To check status: systemctl --user status ${SERVICE_NAME}${NC}"
    echo ""
    echo -e "${GREEN}Cleanup complete!${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

print_header() {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_step() {
    echo -e "${BLUE}▶ $1${NC}"
}

wait_for_carla() {
    local port=$1
    local max_attempts=30
    local attempt=0

    print_step "Waiting for CARLA to be ready on port ${port}..."

    while [ $attempt -lt $max_attempts ]; do
        if nc -z localhost "$port" 2>/dev/null; then
            echo -e "${GREEN}✓ CARLA is ready on port ${port}${NC}"
            # Give it a bit more time to fully initialize
            sleep 2
            return 0
        fi
        attempt=$((attempt + 1))
        echo -n "."
        sleep 1
    done

    echo ""
    echo -e "${RED}✗ Timeout waiting for CARLA to start${NC}"
    echo -e "${YELLOW}Check logs: journalctl --user -u ${SERVICE_NAME} -n 50${NC}"
    return 1
}

# ============================================================================
# Main Script
# ============================================================================

print_header "autoware_carla_bridge Test Environment Setup"

echo -e "${CYAN}Configuration:${NC}"
echo -e "  CARLA Version: ${GREEN}${CARLA_VERSION}${NC}"
echo -e "  CARLA Port:    ${GREEN}${CARLA_PORT}${NC}"
echo -e "  Service Name:  ${GREEN}${SERVICE_NAME}${NC}"
echo -e "  Project Root:  ${GREEN}${PROJECT_ROOT}${NC}"
echo ""

# ============================================================================
# Step 1: Install and start CARLA service
# ============================================================================

if [ "$SKIP_SERVICE_INSTALL" = false ]; then
    print_header "Step 1: Installing CARLA Service"

    print_step "Running: scripts/simulators/install.sh install ${CARLA_VERSION}"
    cd "$PROJECT_ROOT"
    "$SCRIPT_DIR/simulators/install.sh" install "$CARLA_VERSION"

    echo ""
    print_step "Starting CARLA service: ${SERVICE_NAME}"

    # Stop service if already running
    if systemctl --user is-active "$SERVICE_NAME" >/dev/null 2>&1; then
        echo -e "${YELLOW}Service already running, restarting...${NC}"
        systemctl --user restart "$SERVICE_NAME"
    else
        systemctl --user start "$SERVICE_NAME"
    fi

    echo -e "${GREEN}✓ CARLA service started${NC}"

    # Wait for CARLA to be ready
    if ! wait_for_carla "$CARLA_PORT"; then
        echo -e "${RED}Failed to start CARLA. Check service logs.${NC}"
        exit 1
    fi
else
    print_header "Step 1: Skipping CARLA Service Installation"
    echo -e "${YELLOW}Assuming CARLA is already running on port ${CARLA_PORT}${NC}"

    # Still check if CARLA is accessible
    if ! wait_for_carla "$CARLA_PORT"; then
        echo -e "${RED}CARLA is not accessible on port ${CARLA_PORT}${NC}"
        echo -e "${YELLOW}Start CARLA manually or run without -s flag${NC}"
        exit 1
    fi
fi

# ============================================================================
# Step 2: Spawn test vehicles
# ============================================================================

if [ "$SKIP_AGENT_SPAWN" = false ]; then
    print_header "Step 2: Spawning Test Vehicles"

    cd "$PROJECT_ROOT"
    print_step "Running: make agent-spawn"

    # Set CARLA_PORT environment variable for agent spawning
    export CARLA_HOST=localhost
    export CARLA_PORT="$CARLA_PORT"

    if make agent-spawn; then
        echo -e "${GREEN}✓ Test vehicles spawned successfully${NC}"
    else
        echo -e "${RED}✗ Failed to spawn test vehicles${NC}"
        echo -e "${YELLOW}Continuing anyway... You can spawn vehicles manually later.${NC}"
    fi
else
    print_header "Step 2: Skipping Agent Spawning"
    echo -e "${YELLOW}Run 'make agent-spawn' manually if you need test vehicles${NC}"
fi

# ============================================================================
# Step 3: Run the bridge
# ============================================================================

print_header "Step 3: Running autoware_carla_bridge"

cd "$PROJECT_ROOT"
print_step "Running: make run"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}Bridge Starting...${NC}"
echo -e "${CYAN}Press Ctrl+C to stop and cleanup${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Run make run in background so we can track PID
make run &
BRIDGE_PID=$!

# Wait for bridge process
wait "$BRIDGE_PID" 2>/dev/null || true

# If we get here without interrupt, cleanup
echo ""
echo -e "${YELLOW}Bridge exited${NC}"
cleanup
