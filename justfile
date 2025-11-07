# Autoware-CARLA Bridge Build System
# Use `just --list` to see all available recipes

# CARLA Version Selection
# Supported versions: 0.9.14, 0.9.15, 0.9.16
# The carla-rust build system uses this to select appropriate bindings
carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')

# Common colcon build flags
colcon_build_flags := "--symlink-install --cargo-args --release"

# Default recipe (runs when you type `just`)
default:
    @just --list

# Install colcon plugins and dependencies
install-deps:
    ./scripts/install_deps.sh

# Build ros2_rust packages (Rust generator, runtime, and rclrs)
build-ros2-rust:
    cd src/ros2_rust && \
    colcon build {{colcon_build_flags}}

# Build message packages (generates Rust crates)
build-interface:
    #!/usr/bin/env bash
    source src/ros2_rust/install/setup.bash
    cd src/interface
    colcon build {{colcon_build_flags}} \
        --packages-select \
        builtin_interfaces \
        std_msgs \
        geometry_msgs \
        sensor_msgs \
        tf2_msgs \
        autoware_vehicle_msgs \
        tier4_vehicle_msgs \
        tier4_control_msgs

# Build autoware_carla_bridge package
build-bridge:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source src/interface/install/setup.bash
    cd src/autoware_carla_bridge
    CARLA_VERSION={{carla_version}} colcon build {{colcon_build_flags}}

# Build all stages (complete build)
build: build-ros2-rust build-interface build-bridge

# Launch the bridge with ros2 run
run port="2000":
    #!/usr/bin/env bash
    source src/autoware_carla_bridge/install/setup.bash
    ros2 run autoware_carla_bridge autoware_carla_bridge --carla-port {{port}}

# Clean ros2_rust build artifacts
clean-ros2-rust:
    cd src/ros2_rust && rm -rf build install log

# Clean interface build artifacts
clean-interface:
    cd src/interface && rm -rf build install log

# Clean bridge build artifacts
clean-bridge:
    cd src/autoware_carla_bridge && rm -rf build install log

# Clean all build artifacts
clean: clean-ros2-rust clean-interface clean-bridge

# Format code with rustfmt
format:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source src/interface/install/setup.bash
    cd src/autoware_carla_bridge
    cargo +nightly fmt

# Run format check and clippy
lint:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source src/interface/install/setup.bash
    cd src/autoware_carla_bridge
    cargo +nightly fmt --check
    cargo clippy

# Run tests
test:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source src/interface/install/setup.bash
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
