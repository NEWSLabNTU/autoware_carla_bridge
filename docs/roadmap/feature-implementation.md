# Feature Implementation (Phases 3-6)

This document covers pending feature implementation phases for sensor bridges, vehicle control, testing, and release preparation.

**Status**: ⏳ **PENDING** - Awaiting CARLA simulator testing

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

**Document Version**: 1.0
**Last Updated**: 2025-11-02
**Related Documents**: [roadmap.md](../roadmap.md), [core-migration.md](core-migration.md), [enhancements.md](enhancements.md)
