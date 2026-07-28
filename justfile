# Autoware-CARLA Bridge
set dotenv-load

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
    @just _warn-if-setuptools-shadowed

# Warn (do not fail) if the pip installs above pulled a setuptools that shadows the
# system one. `just build` turns this into a hard error.
_warn-if-setuptools-shadowed:
    #!/usr/bin/env bash
    path=$(python3 -c 'import setuptools; print(setuptools.__file__)')
    case "$path" in
        /usr/lib/python3/dist-packages/*) ;;
        *)
            echo ""
            echo "WARNING: setuptools now resolves to $path"
            echo "  A pip-installed setuptools shadows the system one and makes every"
            echo "  Python package fail with 'option --editable not recognized'."
            echo "  Run: pip uninstall -y setuptools"
            ;;
    esac

# Fail fast if a pip setuptools shadows the system one.
#
# colcon's --symlink-install runs `setup.py develop --editable`, which setuptools removed
# in v80. When a pip-installed setuptools in ~/.local takes precedence over the apt one,
# acb_scenario, acb_pilot, carla_map_gen and carla_manual_control all die with "option
# --editable not recognized" -- and colcon then aborts acb_bridge before it compiles, so a
# Rust change looks broken when it never ran. Check up front instead.
_check-setuptools:
    #!/usr/bin/env bash
    set -e
    path=$(python3 -c 'import setuptools; print(setuptools.__file__)')
    case "$path" in
        /usr/lib/python3/dist-packages/*) ;;
        *)
            echo "ERROR: setuptools resolves to $path" >&2
            echo "  Expected the system package (/usr/lib/python3/dist-packages/setuptools)." >&2
            echo "  colcon --symlink-install will fail with 'option --editable not recognized'." >&2
            echo "  Fix: pip uninstall -y setuptools" >&2
            exit 1
            ;;
    esac

# Build all packages
build: _check-setuptools
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
    rm -rf build install log .cargo/config.toml target staging *.deb

# Build release tarball (position-independent, source local_setup.bash)
package:
    #!/usr/bin/env bash
    set -e

    VERSION=$(grep -oP '(?<=<version>)[^<]+' src/acb_bridge/package.xml)
    PKG_NAME="autoware-carla-bridge"
    TARBALL="${PKG_NAME}-${VERSION}-$(uname -m).tar.gz"

    echo "Building ${PKG_NAME} ${VERSION}..."

    # Clean staging directory and stale build state
    rm -rf pkg_install .cargo/config.toml build/.colcon/bindgen.lock

    # Build all packages with merge-install
    export CARLA_VERSION={{carla_version}}
    source /opt/autoware/1.5.0/setup.bash
    colcon build \
        --base-paths src \
        --merge-install \
        --install-base pkg_install \
        --cargo-args --release

    # Remove build-time artifacts not needed at runtime
    rm -f pkg_install/.colcon_install_layout
    rm -f pkg_install/COLCON_IGNORE

    # Replace colcon-generated setup.bash (hardcodes build-time paths)
    # with our position-independent version
    rm -f pkg_install/setup.bash pkg_install/setup.sh pkg_install/setup.ps1
    rm -f pkg_install/setup.zsh
    cp debian/setup.bash pkg_install/setup.bash

    # Create empty local_setup.bash for ament_cmake packages that reference it
    # in package.dsv but don't generate the file under --merge-install
    for dsv in pkg_install/share/*/package.dsv; do
        pkg_dir=$(dirname "$dsv")
        for ext in bash sh zsh ps1; do
            ref="$pkg_dir/local_setup.$ext"
            [ -f "$ref" ] || touch "$ref"
        done
    done

    # Remove .pyc caches (contain build-time source paths, regenerated on use)
    find pkg_install -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true

    # Create tarball with directory prefix
    PREFIX="${PKG_NAME}-${VERSION}"
    mv pkg_install "${PREFIX}"
    tar -czf "${TARBALL}" "${PREFIX}"
    rm -rf "${PREFIX}"
    echo "Created ${TARBALL}"

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

# Run Autoware + bridge + scenario + monitor (CARLA must be running)
# Use auto_drive=true to also run the autonomous driving pilot
sim auto_drive="false":
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    if [ "{{auto_drive}}" = "true" ]; then
        POSES_FILE="$(ros2 pkg prefix acb_pilot)/share/acb_pilot/config/poses/{{map_name}}.yaml"
        ros2 run acb_pilot auto_drive --ros-args -p poses_file:="$POSES_FILE" &
    fi
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        -c "{{project}}/config/play_launch.yaml" \
        acb_demo_launch single_vehicle_autoware.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/{{map_name}}" \
        carla_port:={{carla_port}} \
        map_name:={{map_name}} \
        data_path:="{{data_path}}"

# Run Autoware + bridge + monitor (CARLA must be running, scenario started separately)
autoware:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_launch carla_simulator.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/{{map_name}}" \
        data_path:="{{data_path}}"

# Run CARLA-Autoware bridge only
bridge:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_bridge acb_bridge.launch.xml \
        carla_port:={{carla_port}}

# Run scenario script only (loads map, spawns hero vehicle, ticks CARLA)
scenario:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_scenario single_vehicle_scenario.launch.xml \
        carla_port:={{carla_port}} \
        map_name:={{map_name}}

# Run vehicle monitor GUI
monitor:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch run carla_manual_control carla_manual_control

# Run autonomous driving pilot (optional: just pilot /path/to/poses.yaml)
pilot poses_file="":
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    POSES_FILE="{{poses_file}}"
    if [ -z "$POSES_FILE" ]; then
        POSES_FILE="$(ros2 pkg prefix acb_pilot)/share/acb_pilot/config/example_poses.yaml"
    fi
    exec ros2 run acb_pilot auto_drive --ros-args -p poses_file:="$POSES_FILE"

# Capture poses from RViz interactively
capture-poses output_file:
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    exec ros2 run acb_pilot capture_poses --ros-args -p output_file:="{{output_file}}"

# Pre-build TensorRT engines for lidar_centerpoint (first-time only, takes 2-5 min)
build-engines:
    #!/usr/bin/env bash
    set -e
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
