# Autoware-CARLA Bridge

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
map_name := env_var_or_default('MAP_NAME', 'Town01')
project := justfile_directory()

# List available recipes
default:
    @just --list

# Install deps, Autoware 1.5.0 Debian, and CARLA maps
setup:
    ./scripts/install_deps.sh
    ./scripts/install_autoware_debian.sh
    ./scripts/download_carla_maps_for_autoware.sh

# Build all packages
build:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    source /opt/autoware/1.5.0/setup.bash
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release

# Remove build artifacts
clean:
    rm -rf build install log .cargo/config.toml target

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
    if [ ! -f .cargo/config.toml ]; then
        echo "Error: .cargo/config.toml not found. Run 'just build' first."
        exit 1
    fi
    cargo +nightly fmt --check
    cargo clippy --all-targets -- -D warnings

# Run tests with nextest
test:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    if [ ! -f .cargo/config.toml ]; then
        echo "Error: .cargo/config.toml not found. Run 'just build' first."
        exit 1
    fi
    cargo nextest run --no-tests pass --no-fail-fast

# Run CARLA simulator (foreground)
run-carla:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    cd "{{project}}/third_party/carla/carla-{{carla_version}}"
    exec ./CarlaUE4.sh -quality-level=Low -carla-rpc-port={{carla_port}}

# Run Autoware planning simulator (foreground)
run-autoware:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    source "{{project}}/install/setup.bash"
    exec play_launch launch \
        carla_autoware_launch carla_simulator.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/{{map_name}}" \
        vehicle_model:=carla_vehicle \
        sensor_model:=carla_sensor_kit \
        use_sim_time:=true

# Run CARLA-Autoware bridge (foreground)
run-bridge:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch \
        autoware_carla_bridge autoware_carla_bridge.launch.xml \
        carla_port:={{carla_port}}

# Run demo scenario - loads map and monitors CARLA actors (foreground)
run-scenario:
    #!/usr/bin/env bash
    set -e
    exec "{{project}}/scripts/demo_scenario.py" --port {{carla_port}}

# Run vehicle monitor GUI (foreground)
run-monitor:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    source "{{project}}/install/setup.bash"
    exec play_launch run manual_control manual_control

# Run full demo - CARLA + Autoware + bridge + scenario + pilot + monitor (foreground)
run-demo:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    source "{{project}}/install/setup.bash"
    exec play_launch launch \
        carla_demo_launch demo.launch.xml \
        project_dir:="{{project}}" \
        carla_version:={{carla_version}} \
        carla_port:={{carla_port}} \
        map_name:={{map_name}}

# Run autonomous driving demo
run-pilot:
    #!/usr/bin/env bash
    set -euo pipefail
    POSES_FILE="{{project}}/scripts/poses.json"
    if [ ! -f "$POSES_FILE" ]; then
        echo "Error: poses.json not found at $POSES_FILE"
        echo ""
        echo "Please capture poses first:"
        echo "  1. In RViz, click '2D Pose Estimate' and set initial pose"
        echo "  2. In RViz, click '2D Goal Pose' and set goal pose"
        echo "  3. Run: just run-read-poses"
        exit 1
    fi
    source "{{project}}/install/setup.bash"
    exec play_launch run carla_pilot drive --ros-args -p poses_file:="$POSES_FILE"

# Capture poses from RViz interactively
run-read-poses:
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    exec play_launch run carla_pilot read_poses --ros-args -p output_file:="{{project}}/scripts/poses.json"
