# Autoware-CARLA Bridge
set dotenv-load

# Host settings come from the environment (or .env): ports, install and data paths.
# Project settings -- which town, CARLA's quality and render mode -- are recipe arguments,
# name:=value as for a launch file (`just carla-start quality:=low`, `just autoware
# map:=Town02`); the old MAP_NAME / CARLA_MAP / CARLA_QUALITY / CARLA_RENDER /
# CARLA_EXTRA_ARGS variables are refused.
carla_port := env_var_or_default('CARLA_PORT', '2000')
# Autoware's writable model directory (populate once: ros2 run autoware_sim_profile setup_autoware_data).
# A host path: where this machine keeps the models, the same for every run.
data_path := env_var_or_default('AUTOWARE_DATA_PATH', env_var('HOME') + '/autoware_data')
project := justfile_directory()
# Where Autoware's own setup.bash lives. The NEWSLabNTU 1.5.0 Debian installs under
# /opt/autoware/1.5.0; other hosts install the same packages straight into the ROS
# prefix (/opt/ros/humble). Override with AUTOWARE_SETUP when yours is elsewhere.
autoware_setup := env_var_or_default('AUTOWARE_SETUP', '/opt/autoware/1.5.0/setup.bash')

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
# acb_scenario, carla_map_gen and carla_manual_control all die with "option
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
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    source "{{autoware_setup}}"
    # deps/autoware_sim_vehicle (a submodule): the Autoware profile acb_launch includes
    colcon build \
        --base-paths src deps/autoware_sim_vehicle deps/sim_vehicle_spec \
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

    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    # Build all packages with merge-install
    source "{{autoware_setup}}"
    colcon build \
        --base-paths src deps/autoware_sim_vehicle deps/sim_vehicle_spec \
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
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
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
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    cargo nextest run --no-tests pass --no-fail-fast

# Run CI checks: build, check (format + clippy), and tests
ci: build check test

# quality:=low saves memory but cannot change towns (load_world to another town segfaults;
# scripts/carla_args.bash).
# Run CARLA in the foreground: [map:=Town01] [quality:=epic|low] [render:=offscreen|window] [extra_args:=]
[positional-arguments]
run-carla *args:
    #!/usr/bin/env bash
    set -e
    export CARLA_PORT={{carla_port}}
    exec "{{project}}/third_party/carla/run.sh" "$@"

# Start CARLA simulator as a background service; arguments as for run-carla
[positional-arguments]
carla-start *args:
    ./scripts/carla_start.sh {{carla_port}} "$@"

# map:=<town> from a recipe's arguments (default Town01); refuses MAP_NAME and anything else.
[positional-arguments]
_map *args:
    #!/usr/bin/env bash
    if [ -n "${MAP_NAME+x}" ]; then
        echo "[just] MAP_NAME is no longer read; pass map:=<town> instead (and unset it)" >&2
        exit 2
    fi
    map=Town01
    for a in "$@"; do
        case "$a" in
            map:=?*) map="${a#*:=}" ;;
            *) echo "[just] unknown argument '$a' (map:=<town>)" >&2; exit 2 ;;
        esac
    done
    echo "$map"

# Stop CARLA simulator service
carla-stop:
    ./scripts/carla_stop.sh {{carla_port}}

# Check CARLA service status
carla-status:
    systemctl --user status "carla-run-{{carla_port}}" || true

# View CARLA service logs
carla-logs *args:
    journalctl --user -u "carla-run-{{carla_port}}" {{args}}

# CARLA must be running.
# Run Autoware + bridge + demo scenario (loads the town first); auto_drive:=true adds the pilot; map:=<town>
[positional-arguments]
sim *args:
    #!/usr/bin/env bash
    set -e
    auto_drive=false; rest=()
    for a in "$@"; do
        case "$a" in
            auto_drive:=*) auto_drive="${a#*:=}" ;;
            *) rest+=("$a") ;;
        esac
    done
    map_name=$(just _map "${rest[@]}")
    # The town first: acb_bridge ends the stack on an episode change, so the demo scenario
    # must find its town loaded rather than load it under a running bridge.
    "{{project}}/scripts/carla_load_town.py" --port {{carla_port}} --town "$map_name"
    source "{{project}}/install/setup.bash"
    if [ "$auto_drive" = "true" ]; then
        POSES_FILE="$(ros2 pkg prefix autoware_sim_pilot)/share/autoware_sim_pilot/config/poses/$map_name.yaml"
        ros2 run autoware_sim_pilot auto_drive --ros-args -p poses_file:="$POSES_FILE" &
    fi
    # Each composable a process of its own (play_launch's isolated mode), reloaded alone when
    # it crashes, as swb's `just ego-av` runs Autoware: in one process per container, a
    # composable whose constructor blocks holds up the rest of its container, and the stack
    # never finished loading its perception (2026-10-24: 9 pointcloud_container composables
    # pending for good, a map-less obstacle stopping the car at the first corner).
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        --container-mode isolated --composable-respawn on-crash \
        -c "{{project}}/config/play_launch.yaml" \
        acb_demo_launch single_vehicle_autoware.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/$map_name" \
        carla_port:={{carla_port}} \
        map_name:="$map_name" \
        data_path:="{{data_path}}"

# CARLA must be running; the scenario is started separately.
# Run Autoware + bridge (`just monitor` for the GUI); map:=<town> (default Town01)
[positional-arguments]
autoware *args:
    #!/usr/bin/env bash
    set -e
    map_name=$(just _map "$@")
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        --container-mode isolated --composable-respawn on-crash \
        acb_launch carla_simulator.launch.xml \
        map_path:="{{project}}/data/carla-autoware-bridge/$map_name" \
        data_path:="{{data_path}}"

# vehicle_check (autoware_sim_vehicle's autoware_sim_check) on CARLA, without SSv2 or Autoware:
# a hero on a straight of the current town (scripts/vehicle_check_host.py ticks CARLA in real
# time), acb_bridge applying the spec's physics, then the contract (topics, types, rates,
# frames, stamps, /robot_description) and the dynamics run (acceleration and brake steps
# against 0.35 m/s^2, a steering sweep against the bicycle model). CARLA must be running and
# not ticked by anyone else. Writes <out>/report.json; exits 0 only if every check passed.
#
# Usage: just vehicle-check [vehicle_model:=tesla_model3] [out:=<dir>]
[positional-arguments]
vehicle-check *args:
    #!/usr/bin/env bash
    set -e
    model=tesla_model3; out="{{project}}/play_log/vehicle_check/$(date +%F_%H-%M-%S)"
    for a in "$@"; do
        case "$a" in
            vehicle_model:=?*) model="${a#*:=}" ;;
            out:=?*) out="${a#*:=}" ;;
            *) echo "[just] unknown argument '$a' (vehicle_model:=, out:=)" >&2; exit 2 ;;
        esac
    done
    mkdir -p "$out"
    source "{{project}}/install/setup.bash"
    physics="$(ros2 pkg prefix sim_vehicle_spec)/share/sim_vehicle_spec/generated/$model/carla_physics.yaml"
    blueprint=$(sed -n 's/^blueprint: *//p' "$physics" | tr -d "'\"")
    groups=()
    stop() { for g in "${groups[@]}"; do kill -TERM -"$g" 2>/dev/null || true; done
             sleep 8; for g in "${groups[@]}"; do kill -KILL -"$g" 2>/dev/null || true; done; }
    trap stop EXIT
    setsid python3 "{{project}}/scripts/vehicle_check_host.py" --port {{carla_port}} \
        --blueprint "$blueprint" > "$out/host.log" 2>&1 < /dev/null &
    groups+=($!)
    sleep 8
    setsid play_launch launch --disable-web-ui --log-dir "$out/acb_log" \
        acb_bridge acb_bridge.launch.xml carla_port:={{carla_port}} \
        carla_physics:="$physics" carla_physics_mode:=apply \
        autoware_spec_hash_file:="$(ros2 pkg prefix ${model}_description)/share/${model}_description/config/spec_hash.yaml" \
        seed_localization_on_attach:=false > "$out/acb.log" 2>&1 < /dev/null &
    groups+=($!)
    play_launch launch --disable-web-ui --log-dir "$out/check_log" \
        autoware_sim_check vehicle_check.launch.xml simulator:=carla vehicle_model:="$model" \
        out:="$out/report.json" > "$out/check.log" 2>&1 < /dev/null || true
    [ -f "$out/report.json" ] || { echo "[vehicle-check] no report; see $out/check.log" >&2; exit 1; }
    python3 - "$out/report.json" <<'PY'
    import json, sys
    r = json.load(open(sys.argv[1]))
    bad = [t for t, p in r["contract"].items() if p] + [s["step"] for s in r["dynamics"] if not s["pass"]]
    for s in r["dynamics"]:
        print(f'  {s["step"]:12} {"pass" if s["pass"] else "FAIL"}  ' + ", ".join(f"{k} {v}" for k, v in s.items() if k not in ("step", "pass")))
    print(f'[vehicle-check] {r["vehicle_model"]} on {r["simulator"]}: ' + ("PASS" if not bad else "FAIL: " + ", ".join(bad)))
    sys.exit(1 if bad else 0)
    PY

# Run CARLA-Autoware bridge only
bridge:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_bridge acb_bridge.launch.xml \
        carla_port:={{carla_port}}

# Run scenario script only (loads map, spawns hero vehicle, ticks CARLA); map:=<town>
[positional-arguments]
scenario *args:
    #!/usr/bin/env bash
    set -e
    map_name=$(just _map "$@")
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        acb_scenario single_vehicle_scenario.launch.xml \
        carla_port:={{carla_port}} \
        map_name:="$map_name"

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
        POSES_FILE="$(ros2 pkg prefix autoware_sim_pilot)/share/autoware_sim_pilot/config/example_poses.yaml"
    fi
    exec ros2 run autoware_sim_pilot auto_drive --ros-args -p poses_file:="$POSES_FILE"

# Capture poses from RViz interactively
capture-poses output_file:
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{project}}/install/setup.bash"
    exec ros2 run autoware_sim_pilot capture_poses --ros-args -p output_file:="{{output_file}}"

# Pre-build TensorRT engines for lidar_centerpoint (first-time only, takes 2-5 min)
build-engines:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    MODEL_PATH="{{data_path}}/lidar_centerpoint"
    if [ -f "$MODEL_PATH/pts_voxel_encoder_centerpoint_tiny.engine" ] && \
       [ -f "$MODEL_PATH/pts_backbone_neck_head_centerpoint_tiny.engine" ]; then
        echo "TensorRT engines already exist, skipping build"
        ls -lh "$MODEL_PATH"/*.engine
        exit 0
    fi
    echo "Building TensorRT engines (this takes 2-5 minutes on first run)..."
    ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml \
        build_only:=true \
        data_path:="{{data_path}}" \
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
