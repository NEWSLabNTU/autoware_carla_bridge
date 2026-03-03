# Infrastructure

This document covers the core infrastructure work for the autoware_carla_bridge project, including environment setup, core migration from Zenoh to rclrs, and carla-rust integration.

**Status**: ✅ **COMPLETE** - All infrastructure phases complete

---

## Preparation

**Objective**: Set up the development environment and understand the current codebase.

**Status**: ✅ **COMPLETE** (2025-10-27)

**Duration**: 3-5 days

### Tasks

- [x] Study Zenoh API usage in current codebase
- [x] Study rclrs API and examples
- [x] Document API differences and migration strategy
- [x] Set up Autoware environment
  - Symlink created: `third_party/autoware` → `/home/aeon/repos/autoware/2025.02-ws`
  - Autoware workspace configured and accessible
- [x] Review and understand all bridge types:
  - [x] `sensor_bridge.rs` (Camera, LiDAR, IMU, GNSS)
  - [x] `vehicle_bridge.rs` (Control, status, velocity)
  - [x] `trafficlight_bridge.rs`
  - [x] `trafficsign_bridge.rs`
  - [x] `other_bridge.rs`
- [x] Set up colcon workspace and three-stage build system
- [x] Create package.xml and launch file for ROS 2 integration

**Deliverables**:
- [x] `docs/zenoh-to-rclrs-api-comparison.md`
- [x] `docs/roadmap.md` (now split into multiple files)
- [x] `docs/message-type-migration.md`
- [x] `docs/carla-rust-integration.md`
- [x] Autoware environment configured (symlink created)
- [x] Colcon workspace structure created
- [x] Three-stage build system implemented

**Success Criteria**:
- [x] All team members understand migration approach
- [x] Build system configured and working
- [x] Autoware workspace accessible

---

## Core Infrastructure

**Objective**: Replace Zenoh session with rclrs context and node, update dependencies.

**Status**: ✅ **COMPLETE** (2025-10-22)

**Duration**: 1 week

### 1.1 Understand ROS Message Type Provision

**Reference**: See `docs/message-type-migration.md` for comprehensive guide on migrating from zenoh-ros-type to rclrs interface packages.

- [x] Study how rclrs provides ROS message types:
  - rclrs generates `.cargo/config.toml` to link ROS interface packages
  - Message types (sensor_msgs, std_msgs, etc.) are sourced from colcon workspace
  - Three-stage build system implemented with rosidl_generator_rs
  - direnv configured for automatic environment sourcing

- [x] Verify ROS message availability:
  ```bash
  # Source Autoware environment
  source external/autoware/install/setup.bash

  # Check available message packages
  ros2 interface package sensor_msgs
  ros2 interface package autoware_vehicle_msgs
  ros2 interface package tier4_vehicle_msgs

  # Check specific message definitions
  ros2 interface show sensor_msgs/msg/Image
  ros2 interface show autoware_vehicle_msgs/msg/VelocityReport
  ```
  Message availability verified through colcon build system

- [x] Test rclrs message type access:
  - Created `examples/test_message_types.rs` for verification
  - Three-stage build generates all required message crates

- [x] Examine generated `.cargo/config.toml`:
  - Generated at project root with ROS message patches
  - Contains paths to all required message crates (50+ packages)
  - Verified patches include: std_msgs, sensor_msgs, geometry_msgs, autoware_vehicle_msgs, tier4_vehicle_msgs

### 1.2 Update Dependencies

- [x] Update `Cargo.toml`:
  - Removed all zenoh dependencies (zenoh, zenoh-ros-type)
  - Added rclrs with wildcard version
  - Added ROS 2 message package dependencies (wildcards resolved via .cargo/config.toml)
  - Updated carla to 0.12.0 with local path dependency: `{ version = "0.12.0", path = "../../../carla-rust/carla" }`
  - Kept all utility crates (arc-swap, atomic_float, clap, etc.)

- [x] Remove Zenoh-specific crates from dependencies
  - All zenoh and zenoh-ros-type references removed
  - Only repository URL contains "zenoh" (harmless)

- [x] Build with colcon build system:
  - Three-stage build implemented in Makefile
  - direnv configured for automatic environment
  - Successfully built on 2025-10-29

- [x] Verify `.cargo/config.toml` is generated with ROS message paths
  - Generated with 50+ message package patches
  - All required message types accessible

### 1.3 Update Main Entry Point

File: `src/main.rs`

- [x] Remove Zenoh imports:
  - All `zenoh::` imports removed
  - Replaced with `use rclrs::CreateBasicExecutor;`

- [x] Add rclrs imports:
  - `use rclrs::CreateBasicExecutor;`
  - Standard Arc and threading imports retained

- [x] Remove `Mode` enum - no longer needed
  - Mode enum completely removed
  - No mode-specific logic remains

- [x] Remove Zenoh config and listen endpoints from CLI arguments
  - Removed: `zenoh_listen`, `mode`, `zenoh_config`
  - Kept: `carla_address`, `carla_port`, `tick`, `slowdown`

- [x] Update `Opts` struct:
  - Simplified to CARLA-only parameters
  - Clean CLI interface with no Zenoh references

- [x] Replace Zenoh session initialization with rclrs:
  ```rust
  // Implemented:
  let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
  let executor = ctx.create_basic_executor();
  let node = executor.create_node("autoware_carla_bridge")?;
  ```

- [x] Update function signatures to pass `rclrs::Node` (not Arc - Node is Arc internally)

- [x] Add ROS 2 executor:
  - Basic executor created with `create_basic_executor()`
  - Node created from executor

### 1.4 Update Bridge Infrastructure

Files: `src/bridge/actor_bridge.rs`, `src/bridge/mod.rs`

- [x] Update `create_bridge()` signature to accept `rclrs::Node`
  - Changed from `Arc<Session>` to `rclrs::Node`
  - Node is `Arc<NodeState>` internally, cheap to clone

- [x] Update all bridge constructors to accept `rclrs::Node`:
  - SensorBridge::new()
  - VehicleBridge::new()
  - TrafficLightBridge::new()
  - TrafficSignBridge::new()
  - OtherActorBridge::new()

- [x] Remove `mode` parameter from all bridge-related functions
  - No mode parameter anywhere in codebase
  - Simplified bridge creation logic

- [x] Update `ActorBridge` trait:
  - Trait interfaces updated for rclrs
  - No breaking changes to external API

### 1.5 Remove Mode-Specific Logic

Files: `src/autoware.rs`, `src/utils.rs`

- [x] Delete `Mode` enum - completely removed
- [x] Remove `setup_topics()` function - deleted
- [x] Remove `declare_node_liveliness()` - deleted
- [x] Remove `declare_topic_liveliness()` - deleted
- [x] Remove `undeclare_all_liveliness()` - deleted
- [x] Remove `format_topic_key()` - replaced with simple string concatenation
- [x] Remove `generate_attachment()` function - deleted
- [x] Remove `put_with_attachment!` macro - deleted

**Code Reduction**:
- autoware.rs: 336 → 134 lines (-60% reduction)
- sensor_bridge.rs: 773 → 629 lines (-19% reduction)
- ~300 lines removed, ~500 lines modified
- All Zenoh complexity eliminated

**Deliverables**:
- [x] Understanding of rclrs ROS message type provision mechanism
- [x] Updated `Cargo.toml` with rclrs dependencies
- [x] Updated `main.rs` with rclrs initialization
- [x] Cleaned up mode-specific logic
- [x] Generated `.cargo/config.toml` with ROS message paths
- [x] Code compiles and passes lint checks

**Success Criteria**:
- [x] `make build` succeeds with direnv environment
- [x] No Zenoh dependencies remain (only in repo URL)
- [x] rclrs node created successfully
- [x] ROS message types accessible from colcon workspace
- [x] Binary built: 9.3 MB at `install/autoware_carla_bridge/lib/`

---

## carla-rust Integration and Enhancements

**Objective**: Integrate local carla-rust repository for enhanced CARLA API access.

**Status**: ✅ **COMPLETE** (2025-10-29 to 2025-11-04)

**Duration**: 2-3 days

### 1.6 Local Repository Integration - ✅ COMPLETE

- [x] Configure local path dependency in `Cargo.toml`:
  ```toml
  carla = { version = "0.12.0", path = "../../../carla-rust/carla" }
  ```
- [x] Verify build succeeds with local dependency
- [x] Test with carla-rust multi-version support (CARLA_VERSION env var)
- [x] Document integration in `docs/carla-rust-integration.md`

**Benefits**:
- Access to latest carla-rust APIs (not yet on crates.io)
- Multi-version CARLA support (0.9.14, 0.9.15, 0.9.16)
- Ability to contribute improvements back to carla-rust
- Local debugging and development

### 1.7 Build System Configuration - ✅ COMPLETE

- [x] Configure direnv for automatic environment setup
- [x] Update `.envrc` with CARLA_VERSION and paths
- [x] Simplify Makefile (remove manual sourcing)
- [x] Test build process:
  ```bash
  # Stage 1: Build ros2_rust (rosidl_generator_rs)
  make build-ros2-rust

  # Stage 2: Build interface packages (generates .cargo/config.toml)
  make build-interfaces

  # Stage 3: Build autoware_carla_bridge
  make build-bridge

  # Or all at once:
  make build
  ```

**Automation**:
- direnv automatically sources ROS 2 and Autoware environments
- CARLA_VERSION environment variable selects CARLA version at build time
- Three-stage build ensures correct dependency resolution

### 1.8 Actor Cleanup Implementation - ✅ COMPLETE

**Implementation**:
- `CarlaVehicle::cleanup()` uses `ActorBase::destroy()` for explicit cleanup
- Called on Autoware disconnection (`main.rs:478`) and graceful shutdown (`main.rs:536`)
- CARLA automatically destroys attached sensors when vehicle is destroyed
- Graceful error handling for already-destroyed actors

### 1.9 Documentation - ✅ COMPLETE

- [x] Create `docs/carla-rust-integration.md` (386 lines)
  - Integration setup instructions
  - Multi-version CARLA support guide
  - Environment configuration
  - Build troubleshooting
  - API usage examples

- [x] Update roadmap with Phase 7 tasks
- [x] Mark Phase 2 as runtime verified in roadmap

### 1.10 Advanced Features - ⚠️ DEFERRED TO FUTURE

The following enhancements are available but not yet implemented:

**Efficient World Loading**:
- `Client::load_world_if_different()` - Avoids unnecessary world reloads
- Checks current world name before loading
- Reduces testing overhead

**Debug Data Recording** (Phase 9):
- `Client::start_recorder()` / `stop_recorder()` - Record simulation
- `Client::show_recorder_file_info()` - Inspect recordings
- `Client::replay_file()` - Replay recorded sessions
- Useful for reproducible bug reports and integration testing

**Multi-Version Support**:
- Use `CARLA_VERSION` env var to target specific CARLA version
- Build-time conditional compilation for version-specific APIs
- Supports 0.9.14, 0.9.15, 0.9.16

---

## Build System Overview

### Build Process

The project uses `just build` (colcon with `--symlink-install`) to build all packages:

```bash
just build   # Build all packages
just clean   # Clean build artifacts
```

colcon-cargo-ros2 generates `.cargo/config.toml` with ROS message package patches. All Rust, Python, and CMake packages are built together.

### Environment Configuration

**direnv** automatically configures:
- ROS 2 Humble environment (`/opt/ros/humble/setup.bash`)
- Autoware environment (`third_party/autoware/install/setup.bash`)
- Local colcon workspace (`install/setup.bash`)
- CARLA_VERSION environment variable
- Additional build paths and flags

**Manual activation** (if direnv not installed):
```bash
source /opt/ros/humble/setup.bash
source third_party/autoware/autoware_repo/install/setup.bash
source install/setup.bash
```

### Key Files

- **package.xml** - ROS 2 package manifest (colcon metadata)
- **.cargo/config.toml** - Generated cargo patches for message packages (by colcon-cargo-ros2)
- **.envrc** - direnv configuration for automatic environment setup
- **justfile** - Build and service management commands
- **Cargo.toml** (workspace) - Workspace configuration
- **src/autoware_carla_bridge/Cargo.toml** - Bridge package dependencies
