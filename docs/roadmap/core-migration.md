# Core Migration (Phases 0-2)

This document covers the completed core migration phases from Zenoh to rclrs.

**Status**: ✅ **COMPLETE** - All phases runtime verified

---

## Phase 0: Preparation

**Objective**: Set up the development environment and understand the current codebase.

**Status**: ✅ **COMPLETE** (2025-10-27)

**Duration**: 3-5 days

### Tasks

- [x] ✅ Study Zenoh API usage in current codebase
- [x] ✅ Study rclrs API and examples
- [x] ✅ Document API differences and migration strategy
- [x] ✅ Set up Autoware environment
  - Symlink created: `src/external/autoware` → `/home/aeon/repos/autoware/2025.02-ws`
  - Autoware workspace configured and accessible
- [x] ✅ Review and understand all bridge types:
  - [x] ✅ `sensor_bridge.rs` (Camera, LiDAR, IMU, GNSS)
  - [x] ✅ `vehicle_bridge.rs` (Control, status, velocity)
  - [x] ✅ `trafficlight_bridge.rs`
  - [x] ✅ `trafficsign_bridge.rs`
  - [x] ✅ `other_bridge.rs`
- [x] ✅ Set up colcon workspace and three-stage build system
- [x] ✅ Create package.xml and launch file for ROS 2 integration

**Deliverables**:
- [x] ✅ `docs/zenoh-to-rclrs-api-comparison.md`
- [x] ✅ `docs/roadmap.md` (now split into multiple files)
- [x] ✅ `docs/message-type-migration.md`
- [x] ✅ `docs/carla-rust-integration.md`
- [x] ✅ Autoware environment configured (symlink created)
- [x] ✅ Colcon workspace structure created
- [x] ✅ Three-stage build system implemented

**Success Criteria**:
- ✅ All team members understand migration approach
- ✅ Build system configured and working
- ✅ Autoware workspace accessible

---

## Phase 1: Core Infrastructure

**Objective**: Replace Zenoh session with rclrs context and node, update dependencies.

**Status**: ✅ **COMPLETE** (2025-10-22)

**Duration**: 1 week

### 1.1 Understand ROS Message Type Provision

**Reference**: See `docs/message-type-migration.md` for comprehensive guide on migrating from zenoh-ros-type to rclrs interface packages.

- [x] ✅ Study how rclrs provides ROS message types:
  - ✅ rclrs generates `.cargo/config.toml` to link ROS interface packages
  - ✅ Message types (sensor_msgs, std_msgs, etc.) are sourced from colcon workspace
  - ✅ Three-stage build system implemented with rosidl_generator_rs
  - ✅ direnv configured for automatic environment sourcing

- [x] ✅ Verify ROS message availability:
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
  ✅ Message availability verified through colcon build system

- [x] ✅ Test rclrs message type access:
  - ✅ Created `examples/test_message_types.rs` for verification
  - ✅ Three-stage build generates all required message crates

- [x] ✅ Examine generated `.cargo/config.toml`:
  - ✅ Generated at project root with ROS message patches
  - ✅ Contains paths to all required message crates (50+ packages)
  - ✅ Verified patches include: std_msgs, sensor_msgs, geometry_msgs, autoware_vehicle_msgs, tier4_vehicle_msgs, tier4_control_msgs

### 1.2 Update Dependencies

- [x] ✅ Update `Cargo.toml`:
  - ✅ Removed all zenoh dependencies (zenoh, zenoh-ros-type)
  - ✅ Added rclrs with wildcard version
  - ✅ Added ROS 2 message package dependencies (wildcards resolved via .cargo/config.toml)
  - ✅ Updated carla to 0.12.0 with local path dependency: `{ version = "0.12.0", path = "../../../carla-rust/carla" }`
  - ✅ Kept all utility crates (arc-swap, atomic_float, clap, etc.)

- [x] ✅ Remove Zenoh-specific crates from dependencies
  - ✅ All zenoh and zenoh-ros-type references removed
  - ✅ Only repository URL contains "zenoh" (harmless)

- [x] ✅ Build with colcon build system:
  - ✅ Three-stage build implemented in Makefile
  - ✅ direnv configured for automatic environment
  - ✅ Successfully built on 2025-10-29

- [x] ✅ Verify `.cargo/config.toml` is generated with ROS message paths
  - ✅ Generated with 50+ message package patches
  - ✅ All required message types accessible

### 1.3 Update Main Entry Point

File: `src/main.rs`

- [x] ✅ Remove Zenoh imports:
  - ✅ All `zenoh::` imports removed
  - ✅ Replaced with `use rclrs::CreateBasicExecutor;`

- [x] ✅ Add rclrs imports:
  - ✅ `use rclrs::CreateBasicExecutor;`
  - ✅ Standard Arc and threading imports retained

- [x] ✅ Remove `Mode` enum - no longer needed
  - ✅ Mode enum completely removed
  - ✅ No mode-specific logic remains

- [x] ✅ Remove Zenoh config and listen endpoints from CLI arguments
  - ✅ Removed: `zenoh_listen`, `mode`, `zenoh_config`
  - ✅ Kept: `carla_address`, `carla_port`, `tick`, `slowdown`

- [x] ✅ Update `Opts` struct:
  - ✅ Simplified to CARLA-only parameters
  - ✅ Clean CLI interface with no Zenoh references

- [x] ✅ Replace Zenoh session initialization with rclrs:
  ```rust
  // Implemented:
  let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
  let executor = ctx.create_basic_executor();
  let node = executor.create_node("autoware_carla_bridge")?;
  ```

- [x] ✅ Update function signatures to pass `rclrs::Node` (not Arc - Node is Arc internally)

- [x] ✅ Add ROS 2 executor:
  - ✅ Basic executor created with `create_basic_executor()`
  - ✅ Node created from executor

### 1.4 Update Bridge Infrastructure

Files: `src/bridge/actor_bridge.rs`, `src/bridge/mod.rs`

- [x] ✅ Update `create_bridge()` signature to accept `rclrs::Node`
  - ✅ Changed from `Arc<Session>` to `rclrs::Node`
  - ✅ Node is `Arc<NodeState>` internally, cheap to clone

- [x] ✅ Update all bridge constructors to accept `rclrs::Node`:
  - ✅ SensorBridge::new()
  - ✅ VehicleBridge::new()
  - ✅ TrafficLightBridge::new()
  - ✅ TrafficSignBridge::new()
  - ✅ OtherActorBridge::new()

- [x] ✅ Remove `mode` parameter from all bridge-related functions
  - ✅ No mode parameter anywhere in codebase
  - ✅ Simplified bridge creation logic

- [x] ✅ Update `ActorBridge` trait:
  - ✅ Trait interfaces updated for rclrs
  - ✅ No breaking changes to external API

### 1.5 Remove Mode-Specific Logic

Files: `src/autoware.rs`, `src/utils.rs`

- [x] ✅ Delete `Mode` enum - completely removed
- [x] ✅ Remove `setup_topics()` function - deleted
- [x] ✅ Remove `declare_node_liveliness()` - deleted
- [x] ✅ Remove `declare_topic_liveliness()` - deleted
- [x] ✅ Remove `undeclare_all_liveliness()` - deleted
- [x] ✅ Remove `format_topic_key()` - replaced with simple string concatenation
- [x] ✅ Remove `generate_attachment()` function - deleted
- [x] ✅ Remove `put_with_attachment!` macro - deleted

**Code Reduction**:
- ✅ autoware.rs: 336 → 134 lines (-60% reduction)
- ✅ sensor_bridge.rs: 773 → 629 lines (-19% reduction)
- ✅ ~300 lines removed, ~500 lines modified
- ✅ All Zenoh complexity eliminated

**Deliverables**:
- [x] ✅ Understanding of rclrs ROS message type provision mechanism
- [x] ✅ Updated `Cargo.toml` with rclrs dependencies
- [x] ✅ Updated `main.rs` with rclrs initialization
- [x] ✅ Cleaned up mode-specific logic
- [x] ✅ Generated `.cargo/config.toml` with ROS message paths
- [x] ✅ Code compiles and passes lint checks

**Success Criteria**:
- ✅ `make build` succeeds with direnv environment
- ✅ No Zenoh dependencies remain (only in repo URL)
- ✅ rclrs node created successfully
- ✅ ROS message types accessible from colcon workspace
- ✅ Binary built: 9.3 MB at `install/autoware_carla_bridge/lib/`

---

## Phase 2: Clock and Simple Publishers

**Objective**: Migrate the simplest publisher (clock) as a proof of concept.

**Status**: ✅ **COMPLETE** - Code migration (2025-10-22), Runtime verified (2025-10-31)

**Duration**: N/A (integrated with Phase 1, testing completed separately)

### 2.1 Migrate Clock Publisher

File: `src/clock.rs`

- [x] ✅ Review current Zenoh implementation
- [x] ✅ Refactor to use rclrs:
  ```rust
  pub struct SimulatorClock {
      publisher_clock: Arc<rclrs::Publisher<builtin_interfaces::msg::Time>>,
  }
  ```
- [x] ✅ Update `new()` function:
  ```rust
  pub fn new(node: rclrs::Node) -> Result<SimulatorClock> {
      let publisher_clock = node.create_publisher("/clock")?;
      Ok(SimulatorClock {
          publisher_clock: Arc::new(publisher_clock),
      })
  }
  ```
- [x] ✅ Update `publish_clock()` function:
  ```rust
  pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
      let time = if let Some(sec) = timestamp {
          builtin_interfaces::msg::Time {
              sec: sec.floor() as i32,
              nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
          }
      } else {
          let now = SystemTime::now()
              .duration_since(UNIX_EPOCH)
              .expect("Unable to get current time");
          builtin_interfaces::msg::Time {
              sec: now.as_secs() as i32,
              nanosec: now.subsec_nanos(),
          }
      };
      self.publisher_clock.publish(time)?;
      Ok(())
  }
  ```
- [x] ✅ Remove CDR serialization
- [x] ✅ Remove attachment logic
- [x] ✅ Remove mode handling

### 2.2 Update Utility Functions

File: `src/utils.rs`

- [x] ✅ Keep `is_bigendian()` function
- [x] ✅ Update `create_ros_header()` to work with rclrs message types:
  ```rust
  pub fn create_ros_header(timestamp: Option<f64>) -> std_msgs::msg::Header {
      let time = if let Some(sec) = timestamp {
          builtin_interfaces::msg::Time {
              sec: sec.floor() as i32,
              nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
          }
      } else {
          let now = SystemTime::now()
              .duration_since(UNIX_EPOCH)
              .expect("Unable to get current time");
          builtin_interfaces::msg::Time {
              sec: now.as_secs() as i32,
              nanosec: now.subsec_nanos(),
          }
      };
      std_msgs::msg::Header {
          stamp: time,
          frame_id: "".to_string(),
      }
  }
  ```

### 2.3 Test Clock Publisher

- [x] ✅ Build the project: `make build`
- [x] ✅ Start CARLA simulator (port 3000)
- [x] ✅ Run the bridge: `make run` (connects to CARLA on port 3000)
- [x] ✅ In another terminal, verify clock topic:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic list | grep clock
  ros2 topic echo /clock
  ros2 topic hz /clock
  ```
- [x] ✅ Verify clock publishes successfully *(verified 2025-10-31)*

**Deliverables**:
- [x] ✅ Working clock publisher using rclrs
- [x] ✅ Updated utility functions
- [x] ✅ **Verification test results - PASSED** *(2025-10-31)*

**Success Criteria**:
- ✅ Clock publisher compiles and integrates with bridge
- ✅ No Zenoh dependencies or CDR serialization
- ✅ Clock topic appears in `ros2 topic list` - **VERIFIED**
- ✅ Clock messages publish successfully - **VERIFIED**
- ✅ Bridge connects to CARLA and runs - **VERIFIED**

---

**Document Version**: 1.0
**Last Updated**: 2025-11-02
**Related Documents**: [roadmap.md](../roadmap.md), [meta.md](meta.md)
