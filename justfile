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
    @echo "  just autoware start         Start Autoware simulator"
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
            ./scripts/carla_start.sh "$1" "$2"
            ;;

        stop)
            if [ $# -lt 2 ]; then
                echo "Usage: just carla stop <version> <port>"
                exit 1
            fi
            ./scripts/carla_stop.sh "$1" "$2"
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

# Autoware simulator management: just autoware {start|stop|logs|status} [ARGS...]
autoware command *ARGS:
    #!/usr/bin/env bash
    set -e
    set -- {{ARGS}}

    UNIT_NAME="autoware-simulator"

    case "{{command}}" in
        start)
            ./scripts/autoware_start.sh
            ;;

        restart)
            echo "=== Restarting Autoware Simulator ==="
            just autoware stop
            sleep 2
            just autoware start
            ;;

        stop)
            ./scripts/autoware_stop.sh
            ;;

        logs)
            journalctl --user -u "$UNIT_NAME" {{ARGS}}
            ;;

        status)
            echo "=== Autoware Simulator Status ==="
            systemctl --user status "$UNIT_NAME" --no-pager || true
            echo ""
            echo "=== Recent logs ==="
            journalctl --user -u "$UNIT_NAME" -n 20 --no-pager
            ;;

        *)
            echo "Usage: just autoware {start|restart|stop|logs|status} [ARGS...]"
            echo ""
            echo "Commands:"
            echo "  start              Start Autoware simulator"
            echo "  restart            Restart Autoware simulator"
            echo "  stop               Stop Autoware simulator"
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

    # Unit names
    CARLA_UNIT="carla-run-$CARLA_VERSION-$CARLA_PORT"
    AUTOWARE_UNIT="autoware-simulator"
    BRIDGE_UNIT="autoware-carla-bridge"

    case "{{command}}" in
        start)
            ./scripts/demo_start.sh
            ;;

        restart)
            echo "=== Restarting Demo Environment ==="
            just demo stop
            sleep 2
            just demo start
            ;;

        stop)
            ./scripts/demo_stop.sh
            ;;

        status)
            echo "=== Demo Environment Status ==="
            echo ""
            echo "--- CARLA Simulator ---"
            systemctl --user status "$CARLA_UNIT" --no-pager || echo "CARLA is not running"
            echo ""
            echo "--- Autoware Simulator ---"
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

# Run autonomous driving
drive:
    #!/usr/bin/env bash
    set -euo pipefail

    POSES_FILE="$(pwd)/scripts/poses.json"

    # Check if poses.json exists
    if [ ! -f "$POSES_FILE" ]; then
        echo "Error: poses.json not found at $POSES_FILE"
        echo ""
        echo "Please capture poses first:"
        echo "  1. In RViz, click '2D Pose Estimate' and set initial pose"
        echo "  2. In RViz, click '2D Goal Pose' and set goal pose"
        echo "  3. Run: ./scripts/read_poses.py"
        echo ""
        exit 1
    fi

    # Run autonomous driving
    echo "=== Running Autonomous Driving ==="
    echo ""
    "$(pwd)/scripts/drive_in_autoware.py"
