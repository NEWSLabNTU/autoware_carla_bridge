# Autoware-CARLA Bridge Build System
# Use `just --list` to see all available recipes

# CARLA Version Selection
# Supported versions: 0.9.14, 0.9.15, 0.9.16
# The carla-rust build system uses this to select appropriate bindings
carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')

# Default recipe - list all available recipes
default:
    @just --list

# Install colcon plugins and dependencies
install-deps:
    ./scripts/install_deps.sh

# Build autoware_carla_bridge package (colcon-cargo-ros2 handles everything)
build:
    #!/usr/bin/env bash
    set -e
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
    source install/setup.bash
    cargo +nightly fmt

# Run format check and clippy
lint:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    if [ ! -f build/ros2_cargo_config.toml ]; then
        echo "Error: build/ros2_cargo_config.toml not found"
        echo "Run 'just build' first to generate ROS 2 cargo configuration"
        exit 1
    fi

    cargo +nightly fmt --check
    cargo clippy --config build/ros2_cargo_config.toml -- -D warnings

# Run tests
test:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    if [ ! -f build/ros2_cargo_config.toml ]; then
        echo "Error: build/ros2_cargo_config.toml not found"
        echo "Run 'just build' first to generate ROS 2 cargo configuration"
        exit 1
    fi

    # Setup cleanup trap to remove .cargo/config.toml on exit
    cleanup() {
        rm -f .cargo/config.toml
    }
    trap cleanup EXIT INT TERM

    # Create .cargo directory and symlink
    mkdir -p .cargo
    ln -sf ../build/ros2_cargo_config.toml .cargo/config.toml

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

# CARLA simulator management: just carla {start|stop|logs|status} [ARGS...]
carla command *ARGS:
    #!/usr/bin/env bash
    set -e

    case "{{command}}" in
        start)
            if [ $# -lt 2 ]; then
                echo "Usage: just carla start <version> <port>"
                echo "Example: just carla start 0.9.16 2000"
                exit 1
            fi
            VERSION="$1"
            PORT="$2"

            # Check if DISPLAY is set
            if [ -z "$DISPLAY" ]; then
                echo "Error: DISPLAY environment variable is not set"
                echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
                exit 1
            fi

            # Determine CARLA directory based on version
            CARLA_DIR="$HOME/repos/autoware_carla_bridge/scripts/simulators/startup/carla-$VERSION"

            if [ ! -d "$CARLA_DIR" ]; then
                echo "Error: CARLA directory not found: $CARLA_DIR"
                echo "Available versions: 0.9.14, 0.9.15, 0.9.16"
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

            # Start CARLA using systemd-run
            systemd-run --user \
                --unit="$UNIT_NAME" \
                --working-directory="$CARLA_DIR" \
                --setenv=VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
                --setenv=DISPLAY="$DISPLAY" \
                --setenv=CARLA_PORT="$PORT" \
                bash -c "./CarlaUE4.sh -quality-level=Low -carla-rpc-port=$PORT"

            echo "CARLA $VERSION started on port $PORT"
            echo "Use 'just carla status $VERSION $PORT' to check status"
            echo "Use 'just carla logs $VERSION $PORT' to view logs"
            echo "Use 'just carla stop $VERSION $PORT' to stop"
            ;;

        stop)
            if [ $# -lt 2 ]; then
                echo "Usage: just carla stop <version> <port>"
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
            ;;

        logs)
            if [ $# -lt 2 ]; then
                echo "Usage: just carla logs <version> <port> [journalctl args...]"
                exit 1
            fi
            VERSION="$1"
            PORT="$2"
            shift 2
            UNIT_NAME="carla-run-$VERSION-$PORT"
            journalctl --user -u "$UNIT_NAME" "$@"
            ;;

        status)
            if [ $# -lt 2 ]; then
                echo "Usage: just carla status <version> <port>"
                exit 1
            fi
            VERSION="$1"
            PORT="$2"
            UNIT_NAME="carla-run-$VERSION-$PORT"

            echo "=== CARLA $VERSION Status (port $PORT) ==="
            systemctl --user status "$UNIT_NAME" --no-pager || true
            echo ""
            echo "=== Recent logs ==="
            journalctl --user -u "$UNIT_NAME" -n 20 --no-pager
            ;;

        *)
            echo "Usage: just carla {start|stop|logs|status} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start <version> <port>           Start CARLA simulator"
            echo "  stop <version> <port>            Stop CARLA simulator"
            echo "  logs <version> <port> [args...]  View CARLA logs"
            echo "  status <version> <port>          Check CARLA status"
            exit 1
            ;;
    esac

# Autoware planning simulator management: just autoware {start|stop|logs|status} [ARGS...]
autoware command *ARGS:
    #!/usr/bin/env bash
    set -e

    UNIT_NAME="autoware-planning-simulator"

    case "{{command}}" in
        start)
            AUTOWARE_DIR="$(pwd)/third_party/autoware"

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
            echo "Use 'just autoware status' to check status"
            echo "Use 'just autoware logs' to view logs"
            echo "Use 'just autoware stop' to stop"
            ;;

        stop)
            if systemctl --user is-active --quiet "$UNIT_NAME"; then
                echo "Stopping Autoware planning simulator..."
                systemctl --user stop "$UNIT_NAME"
                echo "Autoware stopped"
            else
                echo "Autoware is not running"
            fi
            ;;

        logs)
            journalctl --user -u "$UNIT_NAME" {{ARGS}}
            ;;

        status)
            echo "=== Autoware Planning Simulator Status ==="
            systemctl --user status "$UNIT_NAME" --no-pager || true
            echo ""
            echo "=== Recent logs ==="
            journalctl --user -u "$UNIT_NAME" -n 20 --no-pager
            ;;

        *)
            echo "Usage: just autoware {start|stop|logs|status} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start              Start Autoware planning simulator"
            echo "  stop               Stop Autoware planning simulator"
            echo "  logs [args...]     View Autoware logs"
            echo "  status             Check Autoware status"
            exit 1
            ;;
    esac
