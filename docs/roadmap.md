# autoware_carla_bridge Migration Roadmap

This document outlines the phased approach for migrating `zenoh_carla_bridge` to `autoware_carla_bridge` using rclrs (ROS 2 Rust client library).

## Table of Contents

- [Project Overview](#project-overview)
- [Migration Goals](#migration-goals)
- [Prerequisites](#prerequisites)
- [Phase 0: Preparation](#phase-0-preparation)
- [Phase 1: Core Infrastructure](#phase-1-core-infrastructure)
- [Phase 2: Clock and Simple Publishers](#phase-2-clock-and-simple-publishers)
- [Phase 3: Sensor Bridge Migration](#phase-3-sensor-bridge-migration)
- [Phase 4: Vehicle Bridge Migration](#phase-4-vehicle-bridge-migration)
- [Phase 5: Testing and Optimization](#phase-5-testing-and-optimization)
- [Phase 6: Documentation and Release](#phase-6-documentation-and-release)
- [Phase 7: carla-rust Integration and Enhancements](#phase-7-carla-rust-integration-and-enhancements)
- [Success Criteria](#success-criteria)
- [Risk Management](#risk-management)

## Project Overview

**Current State**: `zenoh_carla_bridge` uses Zenoh to bridge CARLA simulator data to ROS 2 systems through multiple modes (DDS, ROS2, RmwZenoh).

**Target State**: `autoware_carla_bridge` will use rclrs to publish CARLA data directly as native ROS 2 topics, eliminating the need for intermediate bridges.

**Timeline**: Estimated 4-6 weeks for complete migration and testing.

## Migration Goals

- ✅ Replace Zenoh with native ROS 2 communication (rclrs)
- ✅ Maintain compatibility with Autoware 2025.22
- ✅ Simplify codebase by removing bridge-specific logic
- ✅ Improve type safety with compile-time message type checking
- ✅ Achieve comparable or better performance
- ✅ Provide comprehensive testing and documentation

## Prerequisites

Before starting the migration, ensure:

- [ ] CARLA 0.9.15 is installed and working
- [ ] ROS 2 Humble is installed
- [ ] Autoware 2025.22 is installed (for ROS message types)
- [ ] Rust toolchain is configured (rustc, cargo)
- [ ] LLVM/Clang 12 is installed for CARLA Rust bindings
- [ ] `carla_agent` vehicle spawning tool is working
- [ ] `external/autoware` symlink points to Autoware installation
- [ ] Familiarity with both Zenoh and rclrs APIs (see `docs/zenoh-to-rclrs-api-comparison.md`)

---

## Phase 0: Preparation

**Objective**: Set up the development environment and understand the current codebase.

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
- [x] ✅ `docs/roadmap.md` (this document)
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

**Status**: ✅ **COMPLETE** - Completed during Phase 1 (2025-10-22)

**Duration**: N/A (integrated with Phase 1)

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

- [ ] Build the project: `make build` *(already tested)*
- [ ] Start CARLA simulator *(pending runtime testing)*
- [ ] Spawn test vehicle with `make agent-spawn` *(pending runtime testing)*
- [ ] Run the bridge: `make launch` *(pending runtime testing)*
- [ ] In another terminal, verify clock topic:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic list | grep clock
  ros2 topic echo /clock
  ros2 topic hz /clock
  ```
- [ ] Verify clock publishes at expected rate *(pending runtime testing)*

**Deliverables**:
- [x] ✅ Working clock publisher using rclrs
- [x] ✅ Updated utility functions
- [ ] Verification test results *(pending CARLA runtime testing)*

**Success Criteria**:
- ✅ Clock publisher compiles and integrates with bridge
- ✅ No Zenoh dependencies or CDR serialization
- ⏳ Clock topic appears in `ros2 topic list` *(pending CARLA testing)*
- ⏳ Clock messages publish at correct rate *(pending CARLA testing)*
- ⏳ Timestamps are accurate *(pending CARLA testing)*

---

## Phase 3: Sensor Bridge Migration

**Objective**: Migrate all sensor publishers (camera, LiDAR, IMU, GNSS).

**Duration**: 1-2 weeks

### 3.1 Define Message Type Mappings

- [ ] Document message type mappings:
  | Sensor Type | ROS Message Type                | QoS Profile               |
  |-------------|---------------------------------|---------------------------|
  | Camera RGB  | `sensor_msgs::msg::Image`       | `QOS_PROFILE_SENSOR_DATA` |
  | Camera Info | `sensor_msgs::msg::CameraInfo`  | `QOS_PROFILE_SENSOR_DATA` |
  | LiDAR       | `sensor_msgs::msg::PointCloud2` | `QOS_PROFILE_SENSOR_DATA` |
  | IMU         | `sensor_msgs::msg::Imu`         | `QOS_PROFILE_DEFAULT`     |
  | GNSS        | `sensor_msgs::msg::NavSatFix`   | `QOS_PROFILE_DEFAULT`     |

### 3.2 Refactor Sensor Bridge Structure

File: `src/bridge/sensor_bridge.rs`

- [ ] Update `SensorBridge` struct:
  ```rust
  pub struct SensorBridge {
      _vehicle_name: String,
      sensor_type: SensorType,
      _actor: Sensor,
      sensor_name: String,
      // Remove: tx: Sender<(MessageType, Vec<u8>)>,
      // Publishers will be stored per sensor type
  }
  ```
- [ ] Remove channel-based threading (`tx`, `rx`, `MessageType` enum)
- [ ] Publishers will be stored in individual sensor structs or as trait objects

### 3.3 Migrate Camera RGB Sensor

- [ ] Update `register_camera_rgb()` signature:
  ```rust
  fn register_camera_rgb(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::Image>>>
  ```
- [ ] Create publisher:
  ```rust
  let image_publisher = node.create_publisher::<sensor_msgs::msg::Image>(
      &format!("{}/image_raw", topic_name),
      rclrs::QOS_PROFILE_SENSOR_DATA,
  )?;
  let info_publisher = node.create_publisher::<sensor_msgs::msg::CameraInfo>(
      &format!("{}/camera_info", topic_name),
      rclrs::QOS_PROFILE_SENSOR_DATA,
  )?;
  ```
- [ ] Update `camera_callback()` to publish directly:
  ```rust
  fn camera_callback(
      publisher: Arc<rclrs::Publisher<sensor_msgs::msg::Image>>,
      header: std_msgs::msg::Header,
      image: CarlaImage,
  ) -> Result<()> {
      let image_msg = sensor_msgs::msg::Image {
          header,
          height: image.height() as u32,
          width: image.width() as u32,
          encoding: "bgra8".to_string(),
          is_bigendian: utils::is_bigendian() as u8,
          step: (image.width() * 4) as u32,
          data: image.as_slice()
              .iter()
              .flat_map(|&Color { b, g, r, a }| [b, g, r, a])
              .collect(),
      };
      publisher.publish(&image_msg)?;
      Ok(())
  }
  ```
- [ ] Remove CDR serialization
- [ ] Remove channel send
- [ ] Update `camera_info_callback()` similarly
- [ ] Test camera topic:
  ```bash
  ros2 topic list | grep camera
  ros2 topic echo /autoware_v1/sensing/camera/traffic_light/image_raw --no-arr
  ros2 topic hz /autoware_v1/sensing/camera/traffic_light/image_raw
  ```

### 3.4 Migrate LiDAR Sensors

- [ ] Update `register_lidar_raycast()`:
  ```rust
  fn register_lidar_raycast(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::PointCloud2>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_SENSOR_DATA`
- [ ] Update `lidar_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Update `register_lidar_raycast_semantic()` similarly
- [ ] Update `sematic_lidar_callback()` similarly
- [ ] Test LiDAR topics:
  ```bash
  ros2 topic echo /autoware_v1/sensing/lidar/top/pointcloud --no-arr
  ros2 topic hz /autoware_v1/sensing/lidar/top/pointcloud
  ```

### 3.5 Migrate IMU Sensor

- [ ] Update `register_imu()`:
  ```rust
  fn register_imu(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::Imu>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_DEFAULT`
- [ ] Update `imu_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Test IMU topic:
  ```bash
  ros2 topic echo /autoware_v1/sensing/imu/tamagawa/imu_raw
  ros2 topic hz /autoware_v1/sensing/imu/tamagawa/imu_raw
  ```

### 3.6 Migrate GNSS Sensor

- [ ] Update `register_gnss()`:
  ```rust
  fn register_gnss(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::NavSatFix>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_DEFAULT`
- [ ] Update `gnss_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Test GNSS topic:
  ```bash
  ros2 topic echo /autoware_v1/sensing/gnss/ublox/nav_sat_fix
  ros2 topic hz /autoware_v1/sensing/gnss/ublox/nav_sat_fix
  ```

### 3.7 Update SensorBridge::new()

- [ ] Refactor to create appropriate publisher based on sensor type
- [ ] Store publishers (may need to use trait objects or enum variants)
- [ ] Remove attachment and mode parameters
- [ ] Update error handling

### 3.8 Clean up SensorBridge::Drop

- [ ] Remove thread stopping logic
- [ ] Publishers will be automatically cleaned up when dropped

**Deliverables**:
- [ ] All sensor publishers migrated to rclrs
- [ ] No CDR serialization code remains
- [ ] All sensor topics verified with ROS 2 tools

**Success Criteria**:
- All sensor types publish correctly
- Topics appear in `ros2 topic list`
- Message data is correct (verify with `ros2 topic echo`)
- Topic rates match expected values

---

## Phase 4: Vehicle Bridge Migration

**Objective**: Migrate vehicle control subscribers and status publishers.

**Duration**: 1-2 weeks

### 4.1 Verify Autoware Message Types

- [ ] Verify Autoware message types are accessible through rclrs:
  ```bash
  # Source Autoware environment
  source external/autoware/install/setup.bash

  # Check Autoware message packages
  ros2 interface package autoware_vehicle_msgs
  ros2 interface package tier4_vehicle_msgs
  ros2 interface package tier4_control_msgs

  # List specific message types
  ros2 interface show autoware_vehicle_msgs/msg/VelocityReport
  ros2 interface show tier4_vehicle_msgs/msg/ActuationCommandStamped
  ```

- [ ] Verify `.cargo/config.toml` includes Autoware message paths:
  ```bash
  # After running make build, check generated config
  cat .cargo/config.toml | grep autoware
  ```

- [ ] Test Autoware message imports in Rust:
  ```rust
  use autoware_vehicle_msgs::msg::VelocityReport;
  use tier4_vehicle_msgs::msg::ActuationCommandStamped;
  use tier4_control_msgs::msg::GateMode;
  ```

**Note**: If message types are not available through Autoware workspace:
  - Ensure Autoware 2025.22 is fully built with all message packages
  - Check that `external/autoware` symlink points to correct installation
  - Verify ROS 2 environment is properly sourced in Makefile

### 4.2 Migrate Vehicle Status Publishers

File: `src/bridge/vehicle_bridge.rs`

- [ ] Update `VehicleBridge` struct:
  ```rust
  pub struct VehicleBridge {
      vehicle_name: String,
      actor: Vehicle,
      // Subscribers
      _subscription_actuation_cmd: Arc<rclrs::Subscription<ActuationCommandStamped>>,
      _subscription_gear_cmd: Arc<rclrs::Subscription<GearCommand>>,
      _subscription_gate_mode: Arc<rclrs::Subscription<GateMode>>,
      // Publishers
      publisher_actuation: Arc<rclrs::Publisher<ActuationStatusStamped>>,
      publisher_velocity: Arc<rclrs::Publisher<VelocityReport>>,
      publisher_steer: Arc<rclrs::Publisher<SteeringReport>>,
      publisher_gear: Arc<rclrs::Publisher<GearReport>>,
      publisher_control: Arc<rclrs::Publisher<ControlModeReport>>,
      publisher_turnindicator: Arc<rclrs::Publisher<TurnIndicatorsReport>>,
      publisher_hazardlight: Arc<rclrs::Publisher<HazardLightsReport>>,
      // State
      velocity: Arc<AtomicF32>,
      current_actuation_cmd: Arc<ArcSwap<ActuationCommandStamped>>,
      current_gear: Arc<ArcSwap<u8>>,
      current_gate_mode: Arc<ArcSwap<GateMode>>,
      // Remove: attachment, mode
      // Control parameters
      tau: f32,
      prev_timestamp: Option<f64>,
      prev_steer_output: f32,
  }
  ```

- [ ] Update `VehicleBridge::new()` to create publishers:
  ```rust
  let publisher_actuation = node.create_publisher::<ActuationStatusStamped>(
      &format!("/{}/vehicle/status/actuation_status", vehicle_name),
      rclrs::QOS_PROFILE_DEFAULT,
  )?;
  // ... create other publishers
  ```

- [ ] Create subscribers:
  ```rust
  let cloned_cmd = current_actuation_cmd.clone();
  let subscription_actuation_cmd = node.create_subscription::<ActuationCommandStamped, _>(
      &format!("/{}/control/command/actuation_cmd", vehicle_name),
      rclrs::QOS_PROFILE_DEFAULT,
      move |msg: ActuationCommandStamped| {
          cloned_cmd.store(Arc::new(msg));
      }
  )?;
  // ... create other subscriptions
  ```

### 4.3 Update Status Publishing Functions

- [ ] Update `pub_current_actuation()`:
  ```rust
  fn pub_current_actuation(&mut self, timestamp: f64) -> Result<()> {
      let control = self.actor.control();
      let mut header = utils::create_ros_header(Some(timestamp));
      header.frame_id = String::from("base_link");
      let actuation_msg = ActuationStatusStamped {
          header,
          status: ActuationStatus {
              accel_status: control.throttle as f64,
              brake_status: control.brake as f64,
              steer_status: -control.steer as f64,
          },
      };
      self.publisher_actuation.publish(&actuation_msg)?;
      Ok(())
  }
  ```
- [ ] Remove CDR serialization from all publishing functions:
  - [ ] `pub_current_velocity()`
  - [ ] `pub_current_steer()`
  - [ ] `pub_current_gear()`
  - [ ] `pub_current_control()`
  - [ ] `pub_current_indicator()`
  - [ ] `pub_hazard_light()`
- [ ] Remove attachment publishing logic
- [ ] Keep `update_carla_control()` logic unchanged (it reads from subscriptions)

### 4.4 Test Vehicle Bridge

- [ ] Start CARLA simulator
- [ ] Spawn test vehicle
- [ ] Run autoware_carla_bridge
- [ ] Verify vehicle status topics:
  ```bash
  ros2 topic list | grep vehicle
  ros2 topic echo /autoware_v1/vehicle/status/velocity_status
  ros2 topic echo /autoware_v1/vehicle/status/steering_status
  ```
- [ ] Test vehicle control by publishing commands:
  ```bash
  # Terminal 1: Monitor velocity
  ros2 topic echo /autoware_v1/vehicle/status/velocity_status

  # Terminal 2: Send control command
  ros2 topic pub /autoware_v1/control/command/actuation_cmd \
    tier4_vehicle_msgs/msg/ActuationCommandStamped \
    "{actuation: {accel_cmd: 0.3, brake_cmd: 0.0, steer_cmd: 0.0}}"
  ```
- [ ] Verify vehicle moves in CARLA

**Deliverables**:
- [ ] Vehicle publishers migrated to rclrs
- [ ] Vehicle subscribers migrated to rclrs
- [ ] Control loop functional
- [ ] All vehicle topics verified

**Success Criteria**:
- Vehicle status topics publish correctly
- Vehicle responds to control commands
- No crashes or errors during operation

---

## Phase 5: Testing and Optimization

**Objective**: Comprehensive testing and performance optimization.

**Duration**: 1-2 weeks

### 5.1 Integration Testing

- [ ] Create test scenarios:
  - [ ] Single vehicle spawn and control
  - [ ] Multiple vehicles (v1, v2)
  - [ ] All sensor types enabled
  - [ ] Long-running stability test (30+ minutes)
  - [ ] Rapid vehicle spawn/despawn
- [ ] Document test procedures in `docs/testing.md`
- [ ] Create test scripts in `tests/` directory

### 5.2 ROS 2 Executor Integration

File: `src/main.rs`

- [ ] Add executor spin to main loop:
  ```rust
  let executor = rclrs::SingleThreadedExecutor::new();

  while running.load(Ordering::SeqCst) {
      // Bridge management logic
      {
          // Actor spawn/despawn
      }

      // Update bridges
      let sec = world.snapshot().timestamp().elapsed_seconds;
      bridge_list.values_mut().try_for_each(|bridge| bridge.step(sec))?;

      // Spin ROS callbacks
      executor.spin_once(Some(std::time::Duration::from_millis(10)))?;

      world.wait_for_tick();
  }
  ```
- [ ] Test that callbacks are processed
- [ ] Adjust spin timeout if needed
- [ ] Consider `MultiThreadedExecutor` if performance requires it

### 5.3 Performance Testing

- [ ] Measure topic publication rates:
  ```bash
  ros2 topic hz /autoware_v1/sensing/lidar/top/pointcloud
  ros2 topic hz /autoware_v1/sensing/camera/traffic_light/image_raw
  ros2 topic hz /autoware_v1/vehicle/status/velocity_status
  ```
- [ ] Compare with Zenoh version benchmarks
- [ ] Monitor CPU and memory usage:
  ```bash
  top -p $(pgrep autoware_carla_bridge)
  ```
- [ ] Profile with `flamegraph` or `perf`:
  ```bash
  # Build the project
  make build

  # Profile with flamegraph
  cargo flamegraph --profile release-with-debug --bin autoware_carla_bridge

  # Or use perf directly
  perf record -g ./target/release-with-debug/autoware_carla_bridge
  perf report
  ```
- [ ] Optimize hot paths if needed

### 5.4 Error Handling and Robustness

- [ ] Test error scenarios:
  - [ ] CARLA disconnection during operation
  - [ ] Invalid sensor data
  - [ ] Rapid topic subscription/unsubscription
  - [ ] ROS 2 node termination
- [ ] Improve error messages
- [ ] Add graceful degradation where possible
- [ ] Ensure proper cleanup on Ctrl-C

### 5.5 Code Quality

- [ ] Format and lint code:
  ```bash
  make format  # Format code
  make lint    # Check formatting and run clippy
  ```
- [ ] Add documentation comments to public APIs
- [ ] Remove dead code and unused imports
- [ ] Update code comments referencing Zenoh

### 5.6 Autoware Integration Test

- [ ] Launch Autoware 2025.22
- [ ] Run autoware_carla_bridge with spawned vehicles
- [ ] Verify Autoware receives all expected topics:
  ```bash
  ros2 node info /autoware_node
  ros2 topic list
  ```
- [ ] Test basic Autoware functionality:
  - [ ] Localization with GNSS
  - [ ] Perception with LiDAR/Camera
  - [ ] Planning with vehicle control
- [ ] Document any Autoware-specific configuration needed

**Deliverables**:
- [ ] Test suite and documentation
- [ ] Performance benchmarks
- [ ] Autoware integration validated
- [ ] Code quality improvements

**Success Criteria**:
- All tests pass consistently
- Performance meets or exceeds Zenoh version
- No memory leaks or resource issues
- Autoware integration works correctly

---

## Phase 6: Documentation and Release

**Objective**: Complete documentation and prepare for release.

**Duration**: 3-5 days

### 6.1 Update README.md

- [ ] Update project description to reflect rclrs usage
- [ ] Remove references to Zenoh bridges
- [ ] Update execution instructions:
  - [ ] Remove zenoh router step
  - [ ] Simplify to just CARLA + autoware_carla_bridge
- [ ] Update topic verification examples
- [ ] Update prerequisites (remove Zenoh installation)
- [ ] Update roadmap status

### 6.2 Write Migration Guide

File: `docs/migration-from-zenoh.md`

- [ ] Document breaking changes
- [ ] Provide topic name mapping (if changed)
- [ ] Explain QoS profile changes
- [ ] Document command-line argument changes
- [ ] Provide troubleshooting section

### 6.3 API Documentation

- [ ] Add rustdoc comments to all public items:
  ```rust
  /// A bridge for CARLA sensor data to ROS 2 topics.
  ///
  /// This bridge handles camera, LiDAR, IMU, and GNSS sensors,
  /// publishing their data to appropriate ROS 2 topics.
  pub struct SensorBridge {
      // ...
  }
  ```
- [ ] Generate documentation:
  ```bash
  cargo doc --no-deps --open
  ```
- [ ] Review generated docs for completeness

### 6.4 Examples and Tutorials

- [ ] Create example launch files (if using launch system)
- [ ] Write tutorial: "Getting Started with autoware_carla_bridge"
- [ ] Write tutorial: "Integrating with Autoware"
- [ ] Create example ROS 2 bag recording script

### 6.5 Final Testing

- [ ] Fresh installation test on clean Ubuntu 22.04
- [ ] Follow README instructions exactly
- [ ] Verify all examples work
- [ ] Test with different ROS 2 distributions (if supported)

### 6.6 Release Preparation

- [ ] Update `Cargo.toml` version to `0.13.0`
- [ ] Create CHANGELOG.md:
  ```markdown
  # Changelog

  ## [0.13.0] - 2025-XX-XX

  ### Changed
  - Migrated from Zenoh to native ROS 2 (rclrs)
  - Removed bridge mode selection (DDS, ROS2, RmwZenoh)
  - Simplified codebase and improved type safety

  ### Removed
  - Zenoh dependencies
  - Liveliness token management
  - Attachment metadata handling
  - Mode-specific logic

  ### Fixed
  - [List any bugs fixed during migration]
  ```
- [ ] Create git tag:
  ```bash
  git tag -a v0.13.0 -m "Release v0.13.0: rclrs migration"
  ```
- [ ] Merge feature branch to main:
  ```bash
  git checkout main
  git merge feature/rclrs-migration
  ```
- [ ] Push to repository:
  ```bash
  git push origin main --tags
  ```

**Deliverables**:
- [ ] Updated README.md
- [ ] Migration guide
- [ ] API documentation
- [ ] Examples and tutorials
- [ ] CHANGELOG.md
- [ ] Git release tag

**Success Criteria**:
- Documentation is clear and comprehensive
- New users can get started without external help
- All examples work correctly

---

## Success Criteria

The migration is considered successful when:

### Functional Requirements
- ✅ All sensor types publish to ROS 2 topics (Camera, LiDAR, IMU, GNSS)
- ✅ Vehicle control works bidirectionally (status out, commands in)
- ✅ Multiple vehicles can be bridged simultaneously
- ✅ Clock synchronization works correctly
- ✅ Integration with Autoware 2025.22 is functional

### Technical Requirements
- ✅ Zero Zenoh dependencies remain
- ✅ All ROS 2 topics use correct message types
- ✅ QoS profiles are appropriate for each topic type
- ✅ No CDR serialization code remains
- ✅ Code passes `make lint` with no warnings
- ✅ Code is formatted with `make format`

### Performance Requirements
- ✅ Topic publication rates match or exceed Zenoh version
- ✅ CPU usage is comparable or better
- ✅ Memory usage is stable (no leaks)
- ✅ Latency is acceptable for real-time control

### Documentation Requirements
- ✅ README.md accurately describes the project
- ✅ API documentation is complete
- ✅ Migration guide helps Zenoh users transition
- ✅ Examples and tutorials are working

### Testing Requirements
- ✅ All integration tests pass
- ✅ Autoware integration is verified
- ✅ Long-running stability test completes
- ✅ Error handling is robust

---

## Risk Management

### Identified Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Autoware message types unavailable in Rust | High | Medium | Generate from `.msg` files or use CDR temporarily |
| Performance regression vs. Zenoh | Medium | Low | Profile and optimize hot paths |
| rclrs API limitations | Medium | Low | Engage with ros2-rust community |
| Breaking changes during development | Medium | Medium | Use feature branch, frequent testing |
| CARLA API incompatibilities | Low | Low | Keep CARLA version pinned |

### Contingency Plans

- **If Autoware messages unavailable**:
  - Option 1: Generate Rust bindings using rosidl
  - Option 2: Define structs manually and use CDR
  - Option 3: Contribute message generation to ros2-rust project

- **If performance issues**:
  - Profile with flamegraph
  - Consider `MultiThreadedExecutor`
  - Optimize message copying
  - Use `Arc` more aggressively

- **If rclrs bugs found**:
  - Report to ros2-rust GitHub
  - Fork and patch if critical
  - Consider temporary workarounds

---

## Progress Tracking

Track progress by marking tasks complete in this document:

- `[ ]` = Not started
- `[~]` = In progress
- `[x]` = Complete
- `[!]` = Blocked

### Current Phase: Phase 7 (carla-rust Integration)

**Last Updated**: 2025-10-31

### Phase Completion Status

**✅ Phase 0: Preparation** - COMPLETE (2025-10-27)
- ✅ All documentation created (4 docs, 2,713 lines)
- ✅ Autoware environment configured
- ✅ Colcon workspace structure created
- ✅ Three-stage build system implemented
- ✅ All bridge types reviewed and understood

**✅ Phase 1: Core Infrastructure** - COMPLETE (2025-10-22)
- ✅ Zenoh → rclrs migration complete (12 files, ~800 lines changed)
- ✅ All Zenoh dependencies removed
- ✅ Mode enum and liveliness tokens removed
- ✅ Code compiles and passes lint checks (zero warnings)
- ✅ Binary built: 9.3 MB
- ✅ .cargo/config.toml generated with 50+ message packages

**✅ Phase 2: Clock and Simple Publishers** - COMPLETE (2025-10-22, integrated with Phase 1)
- ✅ Clock publisher migrated to rclrs (src/clock.rs)
- ✅ Utility functions updated (src/utils.rs)
- ✅ No CDR serialization or Zenoh dependencies
- ✅ Code compiles successfully
- ⏳ Runtime testing pending (requires CARLA)

**🔄 Phase 7: carla-rust Integration** - IN PROGRESS (Started 2025-10-29)
- ✅ Local carla-rust path dependency configured
- ✅ Build system verified
- ✅ Documentation created (`docs/carla-rust-integration.md`)
- ✅ Roadmap updated with enhancement tasks (Phase 2 marked complete)
- ⏳ Actor cleanup implementation (pending)
- ⏳ Multi-version CARLA testing (pending)

**⏳ Phase 3+: Sensor/Vehicle Bridge Testing and Further Development** - PENDING
- Awaiting CARLA simulator testing
- Sensor bridge runtime verification (Camera, LiDAR, IMU, GNSS)
- Vehicle bridge runtime verification (control subscribers, status publishers)
- Integration testing with Autoware
- Performance optimization

### Completed Items (All Phases)
- [x] ✅ Study Zenoh API usage
- [x] ✅ Study rclrs API
- [x] ✅ Create API comparison document (647 lines)
- [x] ✅ Create roadmap document (1,198 lines)
- [x] ✅ Create message type migration guide (482 lines)
- [x] ✅ Complete Phase 1: Core Infrastructure migration
- [x] ✅ Remove all Zenoh dependencies
- [x] ✅ Remove Mode enum and liveliness logic
- [x] ✅ Update all 5 bridge types to rclrs
- [x] ✅ Complete Phase 2: Clock and Simple Publishers migration
- [x] ✅ Migrate clock publisher to rclrs (src/clock.rs)
- [x] ✅ Update utility functions (src/utils.rs)
- [x] ✅ Remove CDR serialization from clock publisher
- [x] ✅ Integrate local carla-rust repository
- [x] ✅ Switch to path dependency for carla crate
- [x] ✅ Create carla-rust integration documentation (386 lines)
- [x] ✅ Configure direnv for automatic environment
- [x] ✅ Simplify Makefile (removed manual sourcing)
- [x] ✅ Fix Makefile typo (build-packages → build-bridge)

### Current Focus (Phase 7)
1. ⏳ Implement actor cleanup with `ActorBase::destroy()`
2. ⏳ Add efficient world loading with `Client::load_world_if_different()`
3. ⏳ Multi-version CARLA support testing (0.9.14, 0.9.15, 0.9.16)
4. ⏳ Documentation updates for new APIs

### Next Steps
1. **Immediate**: Begin Phase 3 runtime testing with CARLA simulator
   - Verify clock publisher (Phase 2) publishes to `/clock` topic
   - Test sensor bridges (Camera, LiDAR, IMU, GNSS) with spawned vehicles
   - Validate vehicle control and status publishing
2. Add Drop implementation with `actor.destroy()` to SensorBridge and VehicleBridge
3. Add world loading utility with version-conditional compilation
4. Test bridge with multiple CARLA versions (0.9.14, 0.9.15, 0.9.16)
5. Explore advanced carla-rust APIs (walker control, batch operations, debug visualization)

### Metrics
- **Documentation**: 4 guides, 2,713 total lines
- **Phases Complete**: 3 of 7 (Phase 0, 1, 2)
- **Code Changes**: 15 files modified, ~800 lines changed, ~300 lines removed
- **Build Time**: ~5.5 minutes (first build), ~3 minutes (incremental)
- **Binary Size**: 9.3 MB
- **Lint Warnings**: 0
- **Compilation Status**: ✅ Success
- **Runtime Testing**: ⏳ Pending CARLA availability

---

## Phase 7: carla-rust Integration and Enhancements

**Objective**: Leverage new APIs from local carla-rust repository to improve bridge functionality and robustness.

**Status**: 🔄 **IN PROGRESS** - Local carla-rust integrated (2025-10-29)

**Duration**: 1-2 weeks

### 7.1 Implement Proper Actor Cleanup

**Priority**: High

- [ ] Add `ActorBase::destroy()` to SensorBridge Drop implementation
  ```rust
  impl Drop for SensorBridge {
      fn drop(&mut self) {
          if self._actor.destroy() {
              log::info!("Destroyed sensor: {}", self._sensor_name);
          }
      }
  }
  ```
- [ ] Add `ActorBase::destroy()` to VehicleBridge Drop implementation
- [ ] Test actor cleanup with rapid spawn/despawn cycles
- [ ] Verify no resource leaks with CARLA monitor tools
- [ ] Document actor lifecycle management

**Benefits**:
- Explicit resource cleanup instead of relying on CARLA GC
- Faster actor removal from simulation
- Better memory management
- Clearer resource lifecycle

### 7.2 Implement Efficient World Loading

**Priority**: Medium

- [ ] Add world loading utility with `Client::load_world_if_different()`
  ```rust
  pub fn load_world_smart(client: &Client, map_name: &str) -> World {
      #[cfg(carla_0916)]
      {
          client.load_world_if_different(map_name)
      }
      #[cfg(not(carla_0916))]
      {
          client.load_world(map_name)
      }
  }
  ```
- [ ] Add CLI option for initial map selection
- [ ] Support map switching without bridge restart
- [ ] Test with different map combinations
- [ ] Benchmark loading time improvement vs. reload

**Benefits**:
- Avoids unnecessary map reloads (saves 5-10 seconds per switch)
- Smoother testing workflow
- Less CARLA server restarts needed

### 7.3 Add Debug Data Recording

**Priority**: Low

- [ ] Add `--debug-record` CLI flag
- [ ] Implement sensor data recording with `Sensor::save_to_disk()`
  ```rust
  if args.debug_record {
      sensor.save_to_disk(&format!("debug/{}_{}", vehicle_name, sensor_name));
  }
  ```
- [ ] Create debug output directory structure
- [ ] Add recording controls (start/stop via ROS service)
- [ ] Document debug recording usage

**Benefits**:
- Capture sensor data for offline debugging
- Reproduce issues without running full simulation
- Compare sensor outputs across CARLA versions

### 7.4 Multi-Version CARLA Support

**Priority**: Medium

- [ ] Add `CARLA_VERSION` variable to Makefile
  ```makefile
  CARLA_VERSION ?= 0.9.16
  ```
- [ ] Document version selection in README.md
- [ ] Create test scripts for each CARLA version
  - [ ] `scripts/test_carla_0914.sh`
  - [ ] `scripts/test_carla_0915.sh`
  - [ ] `scripts/test_carla_0916.sh`
- [ ] Test bridge with CARLA 0.9.14
- [ ] Test bridge with CARLA 0.9.15
- [ ] Test bridge with CARLA 0.9.16 (current)
- [ ] Document version-specific features and limitations
- [ ] Add CI/CD matrix testing for multiple versions

**Benefits**:
- Support users with different CARLA installations
- Test backward compatibility
- Identify version-specific issues early

### 7.5 Explore Advanced carla-rust APIs

**Priority**: Low (Research)

Investigate and document potential uses for:

- [ ] **Walker Control APIs**
  - Walker bone control for pedestrian simulation
  - Walker AI controller integration
- [ ] **Batch Operations**
  - Batch spawn/destroy for performance
  - Bulk command execution
- [ ] **Debug Visualization**
  - Add ROS markers for CARLA debug shapes
  - Visualize waypoints, bounding boxes in RViz
- [ ] **Recording/Playback**
  - CARLA native recording integration
  - Playback for deterministic testing
- [ ] **Traffic Light Extensions**
  - Traffic light group control
  - Traffic light state synchronization
- [ ] **Vehicle Telemetry**
  - Failure state monitoring
  - Advanced telemetry publishing

**Deliverables**:
- [ ] Research document: `docs/advanced-carla-apis.md`
- [ ] Proof-of-concept implementations in separate branch
- [ ] Recommendations for future integration

### 7.6 Update Documentation

- [ ] Update README.md with carla-rust integration notes
- [ ] Update CLAUDE.md with new API patterns
- [ ] Add troubleshooting section for multi-version builds
- [ ] Create examples for new features
- [ ] Update migration guide with carla-rust benefits

**Deliverables**:
- Updated documentation across all docs
- Example code demonstrating new APIs
- Version compatibility matrix

**Success Criteria**:
- ✅ Actor cleanup implemented and tested
- ✅ Smart world loading reduces unnecessary reloads
- ✅ Debug recording available for troubleshooting
- ✅ Bridge tested with at least 2 CARLA versions
- ✅ Documentation updated with new features
- ✅ No performance regression from new features

**Risks**:
- Version-specific APIs may require conditional compilation
- Debug recording may impact performance
- Multi-version testing increases CI/CD complexity

---

**Document Version**: 1.2
**Last Updated**: 2025-10-31
**Maintained By**: Autoware CARLA Bridge Team
