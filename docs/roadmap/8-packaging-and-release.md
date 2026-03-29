# Packaging and Release

Debian packaging, CI/CD, binary release distribution, and documentation for the project.

**Status**: ⏳ **PENDING**

**Prerequisites**:
- Phase 6 complete (package renaming to `acb_*`)
- Phase 7 at least informally verified (testing)

---

## 8.1 Debian Packages

**Objective**: Produce `ros-humble-*` Debian packages for apt-based installation alongside Autoware 1.5.0.

### Package Layout

| Debian Package          | Source Packages                                                                                                                        | Description                                    |
|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|
| `ros-humble-acb-bridge` | `acb_bridge`                                                                                                                           | Core bridge binary, launch file, vehicle config |
| `ros-humble-acb-config` | `acb_launch`, `acb_sensor_kit_launch`, `acb_sensor_kit_description`, `acb_vehicle_launch`, `acb_vehicle_description`, `acb_individual_params` | All Autoware integration launch/config/URDF    |
| `ros-humble-acb-demo`   | `acb_demo_launch`, `acb_pilot`                                                                                                        | Demo orchestration + autonomous pilot          |

### Dependencies

**`ros-humble-acb-bridge`**:
- `ros-humble-rclrs`
- `ros-humble-sensor-msgs`, `ros-humble-geometry-msgs`, `ros-humble-nav-msgs`, `ros-humble-tf2-msgs`, `ros-humble-std-msgs`
- `ros-humble-autoware-vehicle-msgs`, `ros-humble-autoware-control-msgs`, `ros-humble-autoware-adapi-v1-msgs`
- `ros-humble-tier4-vehicle-msgs`, `ros-humble-tier4-control-msgs`
- Vendored `libcarla_client.so` (RPATH `$ORIGIN/../lib/`)

**`ros-humble-acb-config`**:
- `ros-humble-autoware-launch` (Autoware 1.5.0 Debian)
- `ros-humble-xacro`

**`ros-humble-acb-demo`**:
- `ros-humble-acb-bridge`
- `ros-humble-acb-config`
- `ros-humble-rclpy`
- `python3-yaml`

### CARLA Client Library Strategy

CARLA does not provide official Debian packages. Vendor `libcarla_client.so` inside `ros-humble-acb-bridge` with RPATH. Simplest for users; version tracked in package metadata.

### Tasks

- [ ] Create `debian/` scaffolding for each Debian package (or bloom release repo)
- [ ] Handle `ament_cargo` -> Debian build (thin CMakeLists.txt wrapper or raw `dpkg-deb`)
- [ ] Vendor `libcarla_client.so` with RPATH patching in build step
- [ ] Bundle 6 config packages into single `ros-humble-acb-config` .deb
- [ ] Test install on clean Ubuntu 22.04 + ROS Humble + Autoware 1.5.0
- [ ] Set up apt repository (GitHub Packages, packagecloud, or self-hosted aptly)

---

## 8.2 Standalone Tool Releases

**Objective**: Distribute standalone CARLA tools as prebuilt binaries on GitHub Releases.

### Artifacts

| Tool                   | Language | Distribution               | Notes                                  |
|------------------------|----------|----------------------------|----------------------------------------|
| `carla_pcd_gen`        | Rust     | GitHub Release binary      | Standalone CLI, needs CARLA connection |
| `carla_map_gen`        | Python   | GitHub Release (or `pipx`) | Python CLI, no ROS runtime needed      |
| `carla_manual_control` | Rust     | GitHub Release binary      | GUI app (macroquad), no ROS dependency |

### Tasks

- [ ] Set up `cargo build --release` for `carla_pcd_gen` and `carla_manual_control`
- [ ] Vendor or statically link CARLA client lib for standalone Rust binaries
- [ ] Package `carla_map_gen` as pip-installable (`pyproject.toml` or `setup.py`)
- [ ] Create GitHub Release workflow to upload binaries on tag push

---

## 8.3 Map Data Distribution

**Objective**: Distribute pre-converted map data (PCD + lanelet2) separately from code packages.

Maps are large, change independently from code, and not all users need all towns. Distribute as tarballs on GitHub Releases, downloaded on demand.

**Per-town tarball contents**:
```
Town01.tar.gz
├── lanelet2_map.osm
├── pointcloud_map.pcd
├── map_config.yaml
└── map_projector_info.yaml
```

### Tasks

- [ ] Create map tarball build script
- [ ] Upload map tarballs to GitHub Releases (or separate data repo)
- [ ] Update `just setup` / `scripts/download_carla_maps_for_autoware.sh` to fetch from release URL
- [ ] Document map download and installation in README

---

## 8.4 CI/CD Pipeline

**Objective**: Automate build, test, packaging, and release via GitHub Actions.

### Pipeline Overview

```
GitHub Actions (on tag push vX.Y.Z)
|
+-- Build & Test
|   +-- colcon build (ubuntu-22.04 + ROS Humble)
|   +-- cargo clippy + fmt --check
|   +-- cargo nextest run
|
+-- Package Debian
|   +-- ros-humble-acb-bridge    (Rust binary + vendored libcarla)
|   +-- ros-humble-acb-config    (launch/config/URDF bundle)
|   +-- ros-humble-acb-demo      (demo launch + Python pilot)
|
+-- Build Standalone Binaries
|   +-- carla_pcd_gen       (x86_64)
|   +-- carla_manual_control (x86_64)
|   +-- carla_map_gen        (Python wheel)
|
+-- Publish
|   +-- .deb -> apt repository
|   +-- binaries -> GitHub Releases
|
+-- (Optional) Integration Test
    +-- Start CARLA in container/VM
    +-- Install Debian packages
    +-- Run smoke test (bridge connects, sensors publish)
```

### Tasks

- [ ] Create `.github/workflows/ci.yml` (build + check + test on every push)
- [ ] Create `.github/workflows/release.yml` (package + publish on tag)
- [ ] Set up CARLA Docker image or VM for integration tests
- [ ] Configure apt repository credentials in GitHub Secrets
- [ ] Test full pipeline end-to-end

---

## 8.5 Documentation and Release Preparation

**Objective**: Complete user-facing documentation and publish the first packaged release.

### Tasks

- [ ] Update README.md with install instructions (apt + standalone tools)
- [ ] Create integration guide (`docs/guides/autoware-integration.md`)
- [ ] Update CHANGELOG.md with all changes
- [ ] Version bump in Cargo.toml and package.xml files
- [ ] Create GitHub release notes
- [ ] Tag release: `git tag v1.0.0`
- [ ] Build and test release artifacts on clean system
- [ ] Publish release on GitHub

---

## 8.6 Release Versioning

All `acb_*` packages share a single version number, bumped together. Standalone `carla_*` tools are versioned independently.

| Component              | Version                                                      | Notes               |
|------------------------|--------------------------------------------------------------|----------------------|
| `acb_*` packages       | Semantic versioning (e.g., 1.0.0)                            | All bumped together  |
| `carla_pcd_gen`        | Independent semver                                           | Standalone tool      |
| `carla_map_gen`        | Independent semver                                           | Standalone tool      |
| `carla_manual_control` | Independent semver                                           | Standalone tool      |
| Map data tarballs      | Tagged by CARLA version + date (e.g., `maps-0.9.16-20260329`) | Independent from code |

---

## User Install Experience

```bash
# Add apt repository (one-time)
curl -s https://example.com/acb.gpg | sudo tee /usr/share/keyrings/acb.gpg
echo "deb [signed-by=/usr/share/keyrings/acb.gpg] https://example.com/apt stable main" \
  | sudo tee /etc/apt/sources.list.d/acb.list
sudo apt update

# Install core bridge + Autoware integration configs
sudo apt install ros-humble-acb-bridge ros-humble-acb-config

# Optional: demo workflow
sudo apt install ros-humble-acb-demo

# Download map data
acb-download-maps Town01 Town03

# Standalone tools (from GitHub Releases)
# carla_pcd_gen, carla_manual_control -- download binary
# carla_map_gen -- pip install
```

---

## Summary

**Deliverables**:
- [ ] Three Debian packages built and published
- [ ] Three standalone tools available as GitHub Release artifacts
- [ ] Map data tarballs on GitHub Releases
- [ ] CI/CD pipeline for automated build, test, and release
- [ ] User-facing install documentation
- [ ] Tagged release with changelog

**Success Criteria**:
- `sudo apt install ros-humble-acb-bridge ros-humble-acb-config` works on clean Ubuntu 22.04 + ROS Humble + Autoware 1.5.0
- `just run-demo` equivalent works with installed packages (no source build)
- Standalone tools run without ROS environment
- CI produces all artifacts on tag push
