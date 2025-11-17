# Autoware-CARLA Bridge Build System
# Use `just --list` to see all available recipes

# CARLA Version Selection
# Supported versions: 0.9.14, 0.9.15, 0.9.16
# The carla-rust build system uses this to select appropriate bindings
carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')

# Default recipe (runs when you type `just`)
default:
    @just --list

# Install colcon plugins and dependencies
install-deps:
    ./scripts/install_deps.sh

# Build autoware_carla_bridge package (colcon-cargo-ros2 handles everything)
build:
    #!/usr/bin/env bash
    set -e
    source src/external/autoware/install/setup.bash
    export CARLA_VERSION={{carla_version}}
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release

# Launch the bridge with ros2 run
run port:
    #!/usr/bin/env bash
    source install/setup.bash
    ros2 run autoware_carla_bridge autoware_carla_bridge --carla-port {{port}}

# Clean all build artifacts
clean:
    rm -rf build install log

# Format code with rustfmt
format:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cd src/autoware_carla_bridge
    cargo +nightly fmt

# Run format check and clippy
lint:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cd src/autoware_carla_bridge
    cargo +nightly fmt --check
    cargo clippy -- -D warnings

# Run tests
test:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    cd src/autoware_carla_bridge
    cargo nextest run --no-tests pass --no-fail-fast

# Setup carla_agent environment
agent-setup:
    cd carla_agent && uv sync

# Spawn test vehicles (requires CARLA running)
agent-spawn:
    cd carla_agent && uv run python simple_spawn.py

# Run complete test environment (CARLA + agents + bridge)
test-env:
    ./scripts/run_test_env.sh

# Start CARLA simulator with systemd-run
start-carla version port:
    #!/usr/bin/env bash
    set -e

    # Check if DISPLAY is set
    if [ -z "$DISPLAY" ]; then
        echo "Error: DISPLAY environment variable is not set"
        echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
        exit 1
    fi

    # Determine CARLA directory based on version
    CARLA_DIR="$HOME/repos/autoware_carla_bridge/scripts/simulators/startup/carla-{{version}}"

    if [ ! -d "$CARLA_DIR" ]; then
        echo "Error: CARLA directory not found: $CARLA_DIR"
        echo "Available versions: 0.9.14, 0.9.15, 0.9.16"
        exit 1
    fi

    # Use a unique transient unit name to avoid conflicts with template units
    UNIT_NAME="carla-run-{{version}}-{{port}}"

    # Stop any existing unit (running or not) and reset failed state
    echo "Ensuring no existing $UNIT_NAME unit..."
    systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
    systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
    # Wait a moment for cleanup
    sleep 0.5

    echo "Starting CARLA {{version}} on port {{port}} with DISPLAY=$DISPLAY..."

    # Start CARLA using systemd-run
    systemd-run --user \
        --unit="$UNIT_NAME" \
        --working-directory="$CARLA_DIR" \
        --setenv=VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
        --setenv=DISPLAY="$DISPLAY" \
        --setenv=CARLA_PORT={{port}} \
        bash -c './CarlaUE4.sh -quality-level=Low -carla-rpc-port={{port}}'

    echo "CARLA {{version}} started on port {{port}}"
    echo "Use 'just status-carla {{version}} {{port}}' to check status"
    echo "Use 'just logs-carla {{version}} {{port}}' to view logs"
    echo "Use 'just stop-carla {{version}} {{port}}' to stop"

# Stop CARLA simulator
stop-carla version port:
    #!/usr/bin/env bash
    UNIT_NAME="carla-run-{{version}}-{{port}}"

    if systemctl --user is-active --quiet "$UNIT_NAME"; then
        echo "Stopping CARLA {{version}} on port {{port}}..."
        systemctl --user stop "$UNIT_NAME"
        echo "CARLA stopped"
    else
        echo "CARLA {{version}} is not running on port {{port}}"
    fi

# Show CARLA logs
logs-carla version port follow="":
    #!/usr/bin/env bash
    UNIT_NAME="carla-run-{{version}}-{{port}}"

    if [ "{{follow}}" = "follow" ] || [ "{{follow}}" = "-f" ]; then
        journalctl --user -u "$UNIT_NAME" -f
    else
        journalctl --user -u "$UNIT_NAME" --no-pager
    fi

# Check CARLA status
status-carla version port:
    #!/usr/bin/env bash
    UNIT_NAME="carla-run-{{version}}-{{port}}"

    echo "=== CARLA {{version}} Status (port {{port}}) ==="
    systemctl --user status "$UNIT_NAME" --no-pager || true
    echo ""
    echo "=== Recent logs ==="
    journalctl --user -u "$UNIT_NAME" -n 20 --no-pager

# Start Autoware planning simulator with systemd-run
start-autoware:
    #!/usr/bin/env bash
    set -e

    AUTOWARE_DIR="$HOME/repos/autoware/2025.02-ws"
    UNIT_NAME="autoware-planning-simulator"

    if [ ! -d "$AUTOWARE_DIR" ]; then
        echo "Error: Autoware directory not found: $AUTOWARE_DIR"
        exit 1
    fi

    # Stop any existing unit and reset failed state
    echo "Ensuring no existing $UNIT_NAME unit..."
    systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
    systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
    sleep 0.5

    echo "Starting Autoware planning simulator..."

    # Start Autoware using systemd-run
    systemd-run --user \
        --unit="$UNIT_NAME" \
        --working-directory="$AUTOWARE_DIR" \
        bash -c '\
            source install/setup.sh && \
            play_launch launch \
                autoware_launch planning_simulator.launch.xml \
                map_path:=$HOME/autoware_map/sample-map-planning \
                vehicle_model:=sample_vehicle \
                sensor_model:=sample_sensor_kit'

    echo "Autoware planning simulator started"
    echo "Use 'just status-autoware' to check status"
    echo "Use 'just logs-autoware' to view logs"
    echo "Use 'just stop-autoware' to stop"

# Stop Autoware simulator
stop-autoware:
    #!/usr/bin/env bash
    UNIT_NAME="autoware-planning-simulator"

    if systemctl --user is-active --quiet "$UNIT_NAME"; then
        echo "Stopping Autoware planning simulator..."
        systemctl --user stop "$UNIT_NAME"
        echo "Autoware stopped"
    else
        echo "Autoware is not running"
    fi

# Show Autoware logs
logs-autoware follow="":
    #!/usr/bin/env bash
    UNIT_NAME="autoware-planning-simulator"

    if [ "{{follow}}" = "follow" ] || [ "{{follow}}" = "-f" ]; then
        journalctl --user -u "$UNIT_NAME" -f
    else
        journalctl --user -u "$UNIT_NAME" --no-pager
    fi

# Check Autoware status
status-autoware:
    #!/usr/bin/env bash
    UNIT_NAME="autoware-planning-simulator"

    echo "=== Autoware Planning Simulator Status ==="
    systemctl --user status "$UNIT_NAME" --no-pager || true
    echo ""
    echo "=== Recent logs ==="
    journalctl --user -u "$UNIT_NAME" -n 20 --no-pager
