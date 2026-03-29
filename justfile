# Autoware-CARLA Bridge

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
map_name := env_var_or_default('MAP_NAME', 'Town01')
data_path := env_var_or_default('AUTOWARE_DATA_PATH', justfile_directory() + '/data')
project := justfile_directory()

# List available recipes
default:
    @just --list

# Install deps, Autoware 1.5.0 Debian, CARLA maps, and tools
setup:
    ./scripts/install_deps.sh
    ./scripts/install_autoware_debian.sh
    ./scripts/download_carla_maps_for_autoware.sh
    pip install play-launch

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
check:
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

# Run CI checks: build, check (format + clippy), and tests
ci: build check test

# Run CARLA simulator (foreground)
run-carla:
    #!/usr/bin/env bash
    set -e
    export CARLA_PORT={{carla_port}}
    exec "{{project}}/third_party/carla/run.sh"

# Start CARLA simulator as a background service
carla-start:
    ./scripts/carla_start.sh {{carla_port}}

# Stop CARLA simulator service
carla-stop:
    ./scripts/carla_stop.sh {{carla_port}}

# Check CARLA service status
carla-status:
    systemctl --user status "carla-run-{{carla_port}}" || true

# View CARLA service logs
carla-logs *args:
    journalctl --user -u "carla-run-{{carla_port}}" {{args}}

# Run Autoware planning simulator (foreground)
run-autoware:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_launch carla_simulator.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/{{map_name}}" \
        vehicle_model:=acb_vehicle \
        sensor_model:=acb_sensor_kit \
        use_sim_time:=true \
        data_path:="{{data_path}}"

# Run CARLA-Autoware bridge (foreground)
run-bridge:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_bridge autoware_carla_bridge.launch.xml \
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
    source "{{project}}/install/setup.bash"
    exec play_launch run carla_manual_control carla_manual_control

# Run full demo - Autoware + bridge + scenario + pilot + monitor (CARLA must be running)
# Pre-build TensorRT engines for lidar_centerpoint (first-time only, takes 2-5 min)
build-engines:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    source /opt/autoware/1.5.0/setup.bash
    MODEL_PATH="{{project}}/data/lidar_centerpoint"
    if [ -f "$MODEL_PATH/pts_voxel_encoder_centerpoint_tiny.engine" ] && \
       [ -f "$MODEL_PATH/pts_backbone_neck_head_centerpoint_tiny.engine" ]; then
        echo "TensorRT engines already exist, skipping build"
        ls -lh "$MODEL_PATH"/*.engine
        exit 0
    fi
    echo "Building TensorRT engines (this takes 2-5 minutes on first run)..."
    ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml \
        build_only:=true \
        data_path:="{{project}}/data" \
        model_name:=centerpoint_tiny
    echo "TensorRT engines built:"
    ls -lh "$MODEL_PATH"/*.engine

run-demo:
    #!/usr/bin/env bash
    set -e
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        -c "{{project}}/config/play_launch.yaml" \
        acb_demo_launch demo.launch.xml \
        project_dir:="{{project}}" \
        carla_port:={{carla_port}} \
        map_name:={{map_name}} \
        data_path:="{{data_path}}"

# Run autonomous driving demo (optional: just run-pilot /path/to/poses.yaml)
run-pilot poses_file="":
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    POSES_FILE="{{poses_file}}"
    if [ -z "$POSES_FILE" ]; then
        POSES_FILE="$(ros2 pkg prefix acb_pilot)/share/acb_pilot/config/example_poses.yaml"
    fi
    exec ros2 run acb_pilot auto_drive --ros-args -p poses_file:="$POSES_FILE"

# Capture poses from RViz interactively
run-capture-poses output_file:
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    exec ros2 run acb_pilot capture_poses --ros-args -p output_file:="{{output_file}}"

# Generate Lanelet2 map from running CARLA server
generate-lanelet2 map_dir:
    #!/usr/bin/env bash
    set -eo pipefail
    source "{{project}}/install/setup.bash"
    ros2 run carla_map_gen carla_map_gen -- --port {{carla_port}} --output-dir "{{map_dir}}"

# Compare generated Lanelet2 map against reference
compare-lanelet2 generated reference:
    python3 "{{project}}/scripts/compare_lanelet2.py" "{{generated}}" "{{reference}}"

# Generate point cloud map from running CARLA server
generate-pcd map_dir:
    #!/usr/bin/env bash
    set -eo pipefail
    source "{{project}}/install/setup.bash"
    ros2 run carla_pcd_gen carla_pcd_gen -- --port {{carla_port}} --map-dir "{{map_dir}}"

# Generate both Lanelet2 + PCD maps
generate-map map_dir:
    #!/usr/bin/env bash
    set -eo pipefail
    source "{{project}}/install/setup.bash"
    ros2 run carla_map_gen carla_map_gen -- --port {{carla_port}} --output-dir "{{map_dir}}"
    ros2 run carla_pcd_gen carla_pcd_gen -- --port {{carla_port}} --map-dir "{{map_dir}}"
