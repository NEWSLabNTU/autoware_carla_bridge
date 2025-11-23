# Autoware-CARLA Bridge Build System
# Use `just --list` to see all available recipes

# CARLA Version Selection
# Supported versions: 0.9.14, 0.9.15, 0.9.16
# The carla-rust build system uses this to select appropriate bindings
carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')

# Show help message
help:
    @echo "Autoware-CARLA Bridge - Available Commands"
    @echo ""
    @echo "Build & Development:"
    @echo "  just build              Build all packages"
    @echo "  just clean              Remove build artifacts"
    @echo "  just format             Format code with rustfmt"
    @echo "  just lint               Run format check and clippy"
    @echo "  just test               Run tests with nextest"
    @echo ""
    @echo "Bridge Control:"
    @echo "  just bridge start [port]    Start bridge (default port: 2000)"
    @echo "  just bridge stop            Stop bridge"
    @echo "  just bridge logs [args...]  View bridge logs"
    @echo "  just bridge status          Check bridge status"
    @echo ""
    @echo "CARLA Simulator:"
    @echo "  just carla start <version> <port>  Start CARLA (e.g., 0.9.16 2000)"
    @echo "  just carla stop <version> <port>   Stop CARLA"
    @echo "  just carla logs <version> <port>   View CARLA logs"
    @echo "  just carla status <version> <port> Check CARLA status"
    @echo ""
    @echo "Autoware Simulator:"
    @echo "  just autoware start         Start Autoware planning simulator"
    @echo "  just autoware restart       Restart Autoware"
    @echo "  just autoware stop          Stop Autoware"
    @echo "  just autoware logs [args...]  View Autoware logs"
    @echo "  just autoware status        Check Autoware status"
    @echo ""
    @echo "Demo Environment (All-in-One):"
    @echo "  just demo start             Start CARLA + Autoware + Bridge"
    @echo "  just demo restart           Restart all services"
    @echo "  just demo stop              Stop all services"
    @echo "  just demo status            Show status of all services"
    @echo "  just demo logs [args...]    View logs from all services"
    @echo ""
    @echo "Other:"
    @echo "  just install-deps           Install dependencies and download maps"
    @echo "  just help                   Show this help message"

# Default recipe
default: help

# Install colcon plugins and dependencies
install-deps:
    ./scripts/install_deps.sh
    ./scripts/download_carla_maps_for_autoware.sh

# Build autoware_carla_bridge package (colcon-cargo-ros2 handles everything)
build:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    # Source Autoware environment to make interface packages discoverable
    source third_party/autoware/autoware_repo/install/setup.bash
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release

# Autoware-CARLA bridge management: just bridge {start|stop|logs|status} [ARGS...]
bridge command *ARGS:
    #!/usr/bin/env bash
    set -e
    set -- {{ARGS}}

    UNIT_NAME="autoware-carla-bridge"

    case "{{command}}" in
        start)
            PORT="${1:-2000}"
            BRIDGE_DIR="$(pwd)"

            if [ ! -f "$BRIDGE_DIR/install/setup.bash" ]; then
                echo "Error: Bridge not built. Run 'just build' first."
                exit 1
            fi

            # Stop any existing unit and reset failed state
            echo "Ensuring no existing $UNIT_NAME unit..."
            systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
            sleep 0.5

            echo "Starting Autoware-CARLA bridge on port $PORT..."

            # Start bridge using systemd-run
            systemd-run --user \
                --unit="$UNIT_NAME" \
                --working-directory="$BRIDGE_DIR" \
                bash -c "\
                    source install/setup.bash && \
                    ros2 run autoware_carla_bridge autoware_carla_bridge --carla-port $PORT"

            echo "Autoware-CARLA bridge started on port $PORT"
            echo "Use 'just bridge status' to check status"
            echo "Use 'just bridge logs' to view logs"
            echo "Use 'just bridge stop' to stop"
            ;;

        stop)
            if systemctl --user is-active --quiet "$UNIT_NAME"; then
                echo "Stopping Autoware-CARLA bridge..."
                systemctl --user stop "$UNIT_NAME"
                echo "Bridge stopped"
            else
                echo "Bridge is not running"
            fi
            ;;

        logs)
            journalctl --user -u "$UNIT_NAME" {{ARGS}}
            ;;

        status)
            echo "=== Autoware-CARLA Bridge Status ==="
            systemctl --user status "$UNIT_NAME" --no-pager || true
            echo ""
            echo "=== Recent logs ==="
            journalctl --user -u "$UNIT_NAME" -n 20 --no-pager
            ;;

        *)
            echo "Usage: just bridge {start|stop|logs|status} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start [port]       Start bridge (default port: 2000)"
            echo "  stop               Stop bridge"
            echo "  logs [args...]     View bridge logs"
            echo "  status             Check bridge status"
            exit 1
            ;;
    esac

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
    cargo clippy --config build/ros2_cargo_config.toml --all-targets -- -D warnings

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

# CARLA simulator management: just carla {start|stop|logs|status} [ARGS...]
carla command *ARGS:
    #!/usr/bin/env bash
    set -e
    set -- {{ARGS}}

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

            # Check if run script exists
            RUN_SCRIPT="$(pwd)/third_party/carla/run-$VERSION.sh"
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
    set -- {{ARGS}}

    UNIT_NAME="autoware-planning-simulator"

    case "{{command}}" in
        start)
            RUN_SCRIPT="$(pwd)/third_party/autoware/run-planning-simulation.sh"

            if [ ! -f "$RUN_SCRIPT" ]; then
                echo "Error: Autoware run script not found: $RUN_SCRIPT"
                echo "Please configure third_party/autoware/run-planning-simulation.sh"
                exit 1
            fi

            # Stop any existing unit and reset failed state
            echo "Ensuring no existing $UNIT_NAME unit..."
            systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
            sleep 0.5

            echo "Starting Autoware planning simulator..."

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

            echo "Autoware planning simulator started"
            echo "Use 'just autoware status' to check status"
            echo "Use 'just autoware logs' to view logs"
            echo "Use 'just autoware stop' to stop"
            ;;

        restart)
            RUN_SCRIPT="$(pwd)/third_party/autoware/run-planning-simulation.sh"

            if [ ! -f "$RUN_SCRIPT" ]; then
                echo "Error: Autoware run script not found: $RUN_SCRIPT"
                echo "Please configure third_party/autoware/run-planning-simulation.sh"
                exit 1
            fi

            echo "=== Restarting Autoware Planning Simulator ==="

            # Stop and cleanup
            echo "Stopping Autoware (if running)..."
            systemctl --user stop "$UNIT_NAME" 2>/dev/null || true
            systemctl --user reset-failed "$UNIT_NAME" 2>/dev/null || true
            sleep 2

            # Start
            echo "Starting Autoware planning simulator..."

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

            echo "Autoware planning simulator restarted"
            echo "Use 'just autoware status' to check status"
            echo "Use 'just autoware logs' to view logs"
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
            echo "Usage: just autoware {start|restart|stop|logs|status} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start              Start Autoware planning simulator"
            echo "  restart            Restart Autoware planning simulator"
            echo "  stop               Stop Autoware planning simulator"
            echo "  logs [args...]     View Autoware logs"
            echo "  status             Check Autoware status"
            exit 1
            ;;
    esac

# Demo environment management: just demo {start|restart|stop|status|logs} [ARGS...]
demo command *ARGS:
    #!/usr/bin/env bash
    set -e
    set -- {{ARGS}}

    # Default configuration
    CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
    CARLA_PORT="${CARLA_PORT:-2000}"
    BRIDGE_PORT="${BRIDGE_PORT:-2000}"

    # Unit names
    CARLA_UNIT="carla-run-$CARLA_VERSION-$CARLA_PORT"
    AUTOWARE_UNIT="autoware-planning-simulator"
    BRIDGE_UNIT="autoware-carla-bridge"

    case "{{command}}" in
        start)
            echo "=== Starting Demo Environment ==="
            echo "CARLA Version: $CARLA_VERSION, Port: $CARLA_PORT"
            echo ""

            # Check if DISPLAY is set for CARLA
            if [ -z "$DISPLAY" ]; then
                echo "Error: DISPLAY environment variable is not set"
                echo "Please set DISPLAY (e.g., export DISPLAY=:1)"
                exit 1
            fi

            # Check if poses.json exists for autonomous driving
            POSES_FILE="$(pwd)/scripts/poses.json"
            if [ ! -f "$POSES_FILE" ]; then
                echo "Warning: poses.json not found at $POSES_FILE"
                echo "Please run './scripts/read_poses.py' after starting Autoware to capture poses"
                echo ""
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
                just carla start "$CARLA_VERSION" "$CARLA_PORT" > /tmp/demo-carla-start.log 2>&1

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
                python3 "$(pwd)/scripts/setup_carla.py" --port "$CARLA_PORT" --map Town01 --sync --timeout 60
                echo "[CARLA] ✓ CARLA configured successfully"
            }
            export -f start_carla

            # Define function for starting Autoware
            start_autoware() {
                echo "[Autoware] Starting Autoware planning simulator..."
                just autoware start > /tmp/demo-autoware-start.log 2>&1
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
            just bridge start "$BRIDGE_PORT"

            # Wait for bridge to be ready to receive initial pose
            # Note: Bridge won't publish vehicle status until initial pose is set
            echo "Waiting for bridge to initialize..."
            sleep 5

            echo "✓ Bridge started and ready"
            echo ""

            # Run autonomous driving if poses exist
            if [ -f "$POSES_FILE" ]; then
                echo "=== Phase 3: Running Autonomous Driving ==="
                echo ""

                # Note: drive_in_autoware.py will set initial pose, which spawns the vehicle
                # After vehicle spawn, diagnostics need time to stabilize
                # The script has built-in waits, so no additional delay needed here

                echo "Starting autonomous driving sequence..."
                echo "(Initial pose will spawn vehicle in CARLA)"

                echo ""
                "$(pwd)/scripts/drive_in_autoware.py"
                echo ""
                echo "✓ Autonomous driving completed"
            else
                echo "=== Phase 3: Skipped (No poses.json) ==="
                echo "To run autonomous driving:"
                echo "  1. In RViz, set initial pose and goal pose"
                echo "  2. Run: ./scripts/read_poses.py"
                echo "  3. Run: ./scripts/drive_in_autoware.py"
            fi
            echo ""

            echo "=== Demo Environment Started Successfully ==="
            echo ""
            echo "All services are now running:"
            echo "  - CARLA:    just carla status $CARLA_VERSION $CARLA_PORT"
            echo "  - Autoware: just autoware status"
            echo "  - Bridge:   just bridge status"
            echo ""
            echo "Use 'just demo status' to check all services"
            echo "Use 'just demo logs' to view all logs"
            echo "Use 'just demo stop' to stop all services"
            ;;

        restart)
            echo "=== Restarting Demo Environment ==="
            just demo stop
            sleep 2
            just demo start
            ;;

        stop)
            echo "=== Stopping Demo Environment ==="
            echo ""

            # Stop in reverse order using just commands
            echo "Step 1/3: Stopping bridge..."
            just bridge stop
            echo ""

            echo "Step 2/3: Stopping Autoware..."
            just autoware stop
            echo ""

            echo "Step 3/3: Stopping CARLA..."
            just carla stop "$CARLA_VERSION" "$CARLA_PORT"
            echo ""

            echo "=== Demo Environment Stopped ==="
            ;;

        status)
            echo "=== Demo Environment Status ==="
            echo ""
            echo "--- CARLA Simulator ---"
            systemctl --user status "$CARLA_UNIT" --no-pager || echo "CARLA is not running"
            echo ""
            echo "--- Autoware Planning Simulator ---"
            systemctl --user status "$AUTOWARE_UNIT" --no-pager || echo "Autoware is not running"
            echo ""
            echo "--- Autoware-CARLA Bridge ---"
            systemctl --user status "$BRIDGE_UNIT" --no-pager || echo "Bridge is not running"
            ;;

        logs)
            echo "=== Demo Environment Logs (all services) ==="
            echo "Services: CARLA, Autoware, Bridge"
            echo "Press Ctrl-C to exit (if following logs with -f)"
            echo ""
            journalctl --user \
                -u "$CARLA_UNIT" \
                -u "$AUTOWARE_UNIT" \
                -u "$BRIDGE_UNIT" \
                {{ARGS}}
            ;;

        *)
            echo "Usage: just demo {start|restart|stop|status|logs} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start              Start CARLA + Autoware + Bridge"
            echo "  restart            Restart all services"
            echo "  stop               Stop all services"
            echo "  status             Show status of all services"
            echo "  logs [args...]     View logs from all services (interleaved)"
            echo ""
            echo "Examples:"
            echo "  just demo logs -n 50        Show last 50 lines"
            echo "  just demo logs -f           Follow logs in real-time"
            echo "  just demo logs --since=1h   Show logs from last hour"
            echo ""
            echo "Environment variables:"
            echo "  CARLA_VERSION      CARLA version (default: 0.9.16)"
            echo "  CARLA_PORT         CARLA port (default: 2000)"
            echo "  BRIDGE_PORT        Bridge port (default: 2000)"
            echo "  DISPLAY            X11 display for CARLA (required)"
            echo ""
            echo "Requirements:"
            echo "  - GNU Parallel: sudo apt install parallel"
            exit 1
            ;;
    esac
