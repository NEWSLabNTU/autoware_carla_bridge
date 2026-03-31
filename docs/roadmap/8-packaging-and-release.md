# Packaging and Release

Release tarball, map data, CI/CD, and documentation.

**Status**: IN PROGRESS — tarball packaging complete, demo launch self-contained

**Prerequisites**:
- Phase 6 complete (package renaming to `acb_*`)
- Phase 7 at least informally verified (testing)

---

## 8.1 Release Tarball

**Status**: DONE

**Objective**: Distribute all ROS packages as a position-independent tarball. Users extract anywhere and source `setup.bash` as a ROS 2 workspace overlay.

### User Workflow

```bash
# 1. Source Autoware (prerequisite)
source ~/autoware/install/setup.bash

# 2. Extract and source the bridge
tar -xzf autoware-carla-bridge-0.12.0-x86_64.tar.gz
source autoware-carla-bridge-0.12.0/setup.bash

# 3. Start CARLA (separate terminal)
./CarlaUE4.sh -carla-rpc-port=2000

# 4. Launch demo
ros2 launch acb_demo_launch demo.launch.xml \
    map_path:=/path/to/maps/Town01
```

No build tools, no root access, no cloning required. The tarball is self-contained — all executables, launch files, configs, and Python packages are included.

### Included Packages

All 12 ROS packages (3 Rust, 2 Python, 7 CMake):

| Package                      | Type         | Contents                                 |
|------------------------------|--------------|------------------------------------------|
| `acb_bridge`                 | ament_cargo  | Core bridge binary + vehicle config      |
| `acb_launch`                 | ament_cmake  | Autoware launch, NDT/MRM configs         |
| `acb_demo_launch`            | ament_cmake  | Demo orchestration launch file           |
| `acb_pilot`                  | ament_python | auto_drive, capture_poses, demo_scenario |
| `acb_individual_params`      | ament_cmake  | Sensor calibration params                |
| `acb_sensor_kit_launch`      | ament_cmake  | Sensor kit launch files                  |
| `acb_sensor_kit_description` | ament_cmake  | Sensor kit URDF/xacro                    |
| `acb_vehicle_launch`         | ament_cmake  | Vehicle launch files                     |
| `acb_vehicle_description`    | ament_cmake  | Vehicle URDF/xacro                       |
| `carla_pcd_gen`              | ament_cargo  | PCD map generator CLI                    |
| `carla_manual_control`       | ament_cargo  | Manual vehicle controller GUI            |
| `carla_map_gen`              | ament_python | Lanelet2 map generator                   |

### Building the Tarball

```bash
just package
# => autoware-carla-bridge-0.12.0-x86_64.tar.gz (~11MB)
```

The recipe:
1. Runs `colcon build --merge-install --install-base pkg_install --cargo-args --release`
2. Replaces colcon's `setup.bash` with a position-independent version (`debian/setup.bash`) that self-locates via `BASH_SOURCE`
3. Creates `local_setup.bash` stubs for ament_cmake packages (colcon `--merge-install` omits them but `package.dsv` references them)
4. Strips `.pyc` caches (contain build-time paths)
5. Packs into `autoware-carla-bridge-VERSION-ARCH.tar.gz` with directory prefix

### Position Independence

Colcon's generated `setup.bash` hardcodes the build-time workspace chain (e.g., `/opt/ros/humble`, `/opt/autoware/1.5.0`, `/home/user/ws/install`). Our replacement (`debian/setup.bash`) resolves its own directory at source time:

```bash
_ACB_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
COLCON_CURRENT_PREFIX="$_ACB_DIR" source "$_ACB_DIR/local_setup.bash"
```

Users source their own Autoware environment first, then source our `setup.bash` — standard ROS 2 overlay pattern.

### Self-Contained Demo Launch

`demo.launch.xml` takes `map_path` directly (not a `project_dir`). All components — Autoware, bridge, scenario ticker, monitor, pilot — launch as ROS nodes from installed packages. No references to source tree paths.

### Design Notes

- **Tarball over `.deb`**: Autoware is typically built from source, so `ros-humble-autoware-*` apt packages don't exist. A `.deb` with those dependencies fails to install.
- **`--merge-install`** for flat layout (`lib/`, `share/`) matching standard ROS workspace structure.
- **Build to `pkg_install/`** then rename: `colcon-cargo-ros2` derives workspace root from install path; deeply nested `--install-base` breaks `.cargo/config.toml` generation.
- **CARLA client library** is statically linked by carla-rust. No shared library vendoring.

### Files

- `justfile` — `package` recipe
- `debian/setup.bash` — position-independent setup script

---

## 8.2 Map Data

**Status**: PENDING

**Objective**: Document how users download pre-converted TUM map data.

Maps are Autoware-compatible (lanelet2 + PCD) for CARLA towns, originally from [TUMFTM/carla-autoware-bridge](https://github.com/TUMFTM/carla-autoware-bridge).

### Per-town contents

```
Town01/
  lanelet2_map.osm
  pointcloud_map.pcd
  map_config.yaml
  map_projector_info.yaml
```

### Available towns

Town01, Town02, Town03, Town05, Town10HD

### Tasks

- [ ] Document TUM map download procedure in README.md
- [ ] Add a download script or `just` recipe (`just download-maps`)

---

## 8.3 CI/CD Pipeline

**Status**: IN PROGRESS — release workflow done, CI workflow pending

**Objective**: Automate build, lint, test, and release via GitHub Actions.

### Release workflow (on tag push `v*`)

Implemented in `.github/workflows/release.yml`. Triggered by pushing a `v*` tag:

```bash
git tag v0.12.0
git push origin v0.12.0
```

Steps: checkout → install Rust + colcon-cargo-ros2 + clang → install Autoware 1.5.0 deb → `just package` → upload tarball to GitHub Release with auto-generated release notes.

### CI workflow (on push / PR)

Not yet implemented.

```
Build (colcon build on ubuntu-22.04 + ROS Humble)
Lint (cargo clippy + cargo fmt --check)
Test (cargo nextest run)
```

### Tasks

- [x] Create `.github/workflows/release.yml`
- [ ] Create `.github/workflows/ci.yml`

---

## 8.4 README Documentation

**Status**: PENDING

**Objective**: Complete user-facing documentation in README.md.

Should cover:
1. **Prerequisites** — Ubuntu 22.04, ROS 2 Humble, Autoware, CARLA 0.9.16
2. **Binary install** — download tarball, extract, source, run
3. **Map data** — download TUM maps
4. **Source build** — clone, build, run (for developers)

### Tasks

- [ ] Rewrite README.md
- [ ] Verify instructions on a clean system

---

## 8.5 Versioning

All packages share a single version. Currently `0.12.0` (from `acb_bridge/package.xml`).

---

## Summary

| Deliverable                                 | Status  |
|---------------------------------------------|---------|
| Release tarball (`just package`)            | DONE    |
| Self-contained demo launch (no project_dir) | DONE    |
| Position-independent install                | DONE    |
| Map data download documented                | PENDING |
| Release workflow (tag → GitHub Release)      | DONE    |
| CI workflow (push/PR → build + lint + test) | PENDING |
| README.md rewrite                           | PENDING |
| Tagged release with changelog               | PENDING |
