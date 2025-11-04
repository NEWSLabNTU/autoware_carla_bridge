# Data Bridge

This document covers the data bridge implementation for the autoware_carla_bridge project, including publishers, subscribers, sensor data, and vehicle control.

**Status**: Phase 2, 8 ✅ Complete | Phase 5 🔧 In Progress (80%) | Phase 6 ⏳ Pending

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

## Phase 8: Architecture Refactoring - 1-to-1 Design

**Objective**: Refactor bridge architecture to implement Autoware-centric 1-to-1 design where one bridge instance manages exactly one CARLA vehicle.

**Status**: ✅ **COMPLETE** (2025-11-04)

**Duration**: 1 week

### 8.1 Remove Simulation Control - ✅ COMPLETE

- [x] ✅ Removed tick thread spawn logic
- [x] ✅ Removed `tick` and `slowdown` CLI parameters
- [x] ✅ Removed synchronous mode configuration
- [x] ✅ Updated SimulatorClock to passive mode
- [x] ✅ Bridge now waits for external simulation control

**Benefits**:
- Cleaner separation of concerns (bridge = passive adapter)
- External scenario scripts control CARLA ticking
- No risk of tick conflicts

### 8.2 Add Vehicle Selection - ✅ COMPLETE

- [x] ✅ Added `--vehicle-name` CLI parameter for role_name selection
- [x] ✅ Added `--vehicle-id` CLI parameter for actor ID selection
- [x] ✅ Added `--vehicle-wait-timeout` parameter (default: 30s)
- [x] ✅ Implemented `find_target_vehicle()` with polling and timeout
- [x] ✅ Validated mutual exclusivity (name XOR id)
- [x] ✅ Added progress logging during vehicle search

**Benefits**:
- Explicit vehicle selection (no auto-discovery)
- Supports both role_name and actor ID
- Flexible timing (vehicle can spawn before/after bridge starts)

### 8.3 Simplify Actor Management - ✅ COMPLETE

- [x] ✅ Replaced HashMap-based multi-vehicle tracking with single ego vehicle
- [x] ✅ Removed actor discovery diff logic (added_ids, deleted_ids)
- [x] ✅ Removed "autoware_" prefix filtering
- [x] ✅ Implemented exact vehicle matching
- [x] ✅ Filter sensors by parent vehicle ID only
- [x] ✅ Removed BridgeError::Npc variant

**Benefits**:
- Simpler codebase (~300 lines removed)
- Clear single-vehicle focus
- No NPC filtering needed
- Easier to understand and maintain

### 8.4 Root Namespace Topics - ✅ COMPLETE

**Changes**:
- Topics now use standard Autoware names without vehicle prefixes:
  ```
  /vehicle/status/velocity_status
  /vehicle/status/actuation_status
  /sensing/camera/traffic_light/image_raw
  /sensing/lidar/top/pointcloud
  /sensing/imu/tamagawa/imu_raw
  /sensing/gnss/ublox/nav_sat_fix
  /control/command/actuation_cmd
  /clock
  ```

**Multi-Vehicle Support**:
- Use `ROS_DOMAIN_ID` environment variable for isolation
- Each bridge runs in separate domain
- Example:
  ```bash
  ROS_DOMAIN_ID=0 ros2 run autoware_carla_bridge autoware_carla_bridge --vehicle-name ego_1
  ROS_DOMAIN_ID=1 ros2 run autoware_carla_bridge autoware_carla_bridge --vehicle-name ego_2
  ```

### Core Changes Summary

**Files Modified**:
- `src/main.rs` - Vehicle selection, timeout logic, CLI parameters
- `src/bridge/vehicle_bridge.rs` - Actor cleanup, prefix removal
- `src/bridge/sensor_bridge.rs` - Actor cleanup, prefix removal
- `src/autoware.rs` - Root namespace topics
- `src/utils.rs` - Efficient world loading

**Code Statistics**:
- ~300 lines removed (simplification)
- 12 files modified
- Zero lint warnings
- All tests passing

**Verification**:
- [x] ✅ Code compiles successfully
- [x] ✅ Lint checks pass (zero warnings)
- [x] ✅ Format checks pass
- [x] ✅ Vehicle selection tested (--vehicle-name, --vehicle-id)
- [x] ✅ Timeout behavior verified
- [x] ✅ Actor cleanup verified (Drop implementations)

---

## Phase 5: Sensor Data Publishing

**Objective**: Implement sensor data publishing for Camera, LiDAR, IMU, and GNSS sensors.

**Status**: 🔧 **IN PROGRESS** - Publishing code complete (Phase 1), integration pending

**Duration**: 1-2 weeks remaining (core publishing already migrated in Phase 1)

**Prerequisites**:
- ✅ Phase 3 (Autoware Integration Foundation) - Complete
- 🔧 Phase 4 (Vehicle Lifecycle Management) - In Progress

**Note**: Sensor publishing code was migrated to rclrs in Phase 1. Remaining work is integration with VehicleLifecycle for sensor spawning.

### 5.1 Camera Sensors

**Objective**: Publish camera sensor data to ROS topics

**Status**: ✅ **COMPLETE** - Migrated in Phase 1, frame IDs need updating

**Topics**:
- `/sensing/camera/traffic_light/image_raw` (sensor_msgs/Image) ✅
- `/sensing/camera/traffic_light/camera_info` (sensor_msgs/CameraInfo) ✅

**Tasks**:
- [x] Create camera publisher in SensorBridge (`sensor_bridge.rs:165-233`)
- [x] Configure camera QoS (sensor_data_qos)
- [x] Implement CARLA callback → ROS Image conversion:
  ```rust
  fn on_camera_data(&self, image: carla::sensor::Image) {
      let mut ros_image = sensor_msgs::msg::Image::default();
      ros_image.header = create_ros_header(Some(image.timestamp));
      ros_image.header.frame_id = self.sensor_frame_id.clone();
      ros_image.height = image.height;
      ros_image.width = image.width;
      ros_image.encoding = "bgra8".to_string();
      ros_image.is_bigendian = 0;
      ros_image.step = image.width * 4;
      ros_image.data = image.data;

      self.image_publisher.publish(&ros_image)?;
  }
  ```
- [x] Publish CameraInfo with camera intrinsics
- [ ] Test with CARLA RGB camera (requires sensor spawning integration)
- [ ] Verify image data in RViz (requires sensor spawning integration)
- [ ] Update hardcoded frame_id ("camera4/camera_link") to use TF data

**Success Criteria**:
- [x] Camera images publish correctly (code complete)
- [ ] Correct frame_id (currently hardcoded)
- [x] Image encoding matches expectations (bgra8)
- [ ] Publish rate verification (pending integration testing)

### 5.2 LiDAR Sensors

**Objective**: Publish LiDAR point cloud data to ROS topics

**Status**: ✅ **COMPLETE** - Migrated in Phase 1, frame IDs need updating

**Topics**:
- `/sensing/lidar/top/pointcloud` (sensor_msgs/PointCloud2) ✅

**Tasks**:
- [x] Create LiDAR publisher in SensorBridge (`sensor_bridge.rs:235-302`)
- [x] Configure LiDAR QoS (sensor_data_qos)
- [x] Implement CARLA callback → ROS PointCloud2 conversion:
  ```rust
  fn on_lidar_data(&self, scan: carla::sensor::LidarMeasurement) {
      let mut cloud = sensor_msgs::msg::PointCloud2::default();
      cloud.header = create_ros_header(Some(scan.timestamp));
      cloud.header.frame_id = self.sensor_frame_id.clone();
      cloud.height = 1;
      cloud.width = scan.point_count as u32;
      cloud.is_bigendian = false;
      cloud.is_dense = true;
      cloud.point_step = 16; // x, y, z, intensity (4 bytes each)
      cloud.row_step = cloud.point_step * cloud.width;

      // Define fields
      cloud.fields = vec![
          create_point_field("x", 0, PointFieldType::Float32, 1),
          create_point_field("y", 4, PointFieldType::Float32, 1),
          create_point_field("z", 8, PointFieldType::Float32, 1),
          create_point_field("intensity", 12, PointFieldType::Float32, 1),
      ];

      // Convert CARLA points to ROS format
      cloud.data = convert_lidar_points(&scan.points);

      self.pointcloud_publisher.publish(&cloud)?;
  }
  ```
- [x] Handle coordinate system conversion (CARLA → ROS)
- [ ] Test with CARLA ray_cast LiDAR sensor (requires sensor spawning)
- [ ] Verify point cloud in RViz (requires sensor spawning)
- [ ] Update hardcoded frame_id ("velodyne_top_base_link") to use TF data

**Success Criteria**:
- [x] Point cloud publishing code complete
- [x] Correct coordinate transformation implemented
- [x] Intensity values included
- [ ] Publish rate verification (pending integration testing)

### 5.3 IMU Sensor

**Objective**: Publish IMU data to ROS topics

**Status**: ✅ **COMPLETE** - Migrated in Phase 1

**Topics**:
- `/sensing/imu/tamagawa/imu_raw` (sensor_msgs/Imu) ✅

**Tasks**:
- [x] Create IMU publisher in SensorBridge (`sensor_bridge.rs:356-424`)
- [x] Configure IMU QoS (sensor_data_qos)
- [x] Implement CARLA callback → ROS Imu conversion:
  ```rust
  fn on_imu_data(&self, imu: carla::sensor::ImuMeasurement) {
      let mut ros_imu = sensor_msgs::msg::Imu::default();
      ros_imu.header = create_ros_header(Some(imu.timestamp));
      ros_imu.header.frame_id = self.sensor_frame_id.clone();

      // Linear acceleration (m/s²)
      ros_imu.linear_acceleration.x = imu.accelerometer.x;
      ros_imu.linear_acceleration.y = -imu.accelerometer.y; // Y-axis flip
      ros_imu.linear_acceleration.z = imu.accelerometer.z;

      // Angular velocity (rad/s)
      ros_imu.angular_velocity.x = imu.gyroscope.x.to_radians();
      ros_imu.angular_velocity.y = -imu.gyroscope.y.to_radians(); // Y-axis flip
      ros_imu.angular_velocity.z = imu.gyroscope.z.to_radians();

      // Orientation (quaternion)
      ros_imu.orientation = euler_to_quaternion(
          imu.compass.to_radians(),
          -imu.compass.to_radians(), // Adjust for coordinate system
          0.0
      );

      self.imu_publisher.publish(&ros_imu)?;
  }
  ```
- [x] Handle coordinate system conversion (Y-axis flip, deg → rad)
- [ ] Test with CARLA IMU sensor (requires sensor spawning)
- [ ] Verify IMU data in RViz (requires sensor spawning)

**Success Criteria**:
- [x] IMU data publishing code complete
- [x] Coordinate transformations implemented
- [x] Orientation quaternion conversion included
- [ ] Publish rate verification (pending integration testing)

### 5.4 GNSS Sensor

**Objective**: Publish GNSS data to ROS topics

**Status**: ✅ **COMPLETE** - Migrated in Phase 1

**Topics**:
- `/sensing/gnss/ublox/nav_sat_fix` (sensor_msgs/NavSatFix) ✅

**Tasks**:
- [x] Create GNSS publisher in SensorBridge (`sensor_bridge.rs:426-463`)
- [x] Configure GNSS QoS (sensor_data_qos)
- [x] Implement CARLA callback → ROS NavSatFix conversion:
  ```rust
  fn on_gnss_data(&self, gnss: carla::sensor::GnssMeasurement) {
      let mut nav_sat_fix = sensor_msgs::msg::NavSatFix::default();
      nav_sat_fix.header = create_ros_header(Some(gnss.timestamp));
      nav_sat_fix.header.frame_id = "gnss_link";

      nav_sat_fix.latitude = gnss.latitude;
      nav_sat_fix.longitude = gnss.longitude;
      nav_sat_fix.altitude = gnss.altitude;

      nav_sat_fix.status.status = NavSatStatus::StatusFix;
      nav_sat_fix.status.service = NavSatStatus::ServiceGps;

      nav_sat_fix.position_covariance_type =
          NavSatFix::CovarianceTypeApproximated;

      self.gnss_publisher.publish(&nav_sat_fix)?;
  }
  ```
- [ ] Test with CARLA GNSS sensor (requires sensor spawning)
- [ ] Verify GNSS data in RViz (requires sensor spawning)

**Success Criteria**:
- [x] GNSS coordinates publishing code complete
- [x] Fix status and service type implemented
- [ ] Publish rate verification (pending integration testing)

### 5.5 Data Verification

**Objective**: Verify sensor data accuracy and consistency

**Status**: ⏳ **PENDING** - Requires full integration

**Tasks**:
- [ ] Create verification script in `scripts/verify_sensors.py`
- [ ] Compare CARLA sensor data with ROS topic data
- [ ] Check timestamp synchronization across sensors
- [x] Coordinate transformations implemented:
  - CARLA (left-handed) → ROS (right-handed)
  - Y-axis flip for IMU
  - Rotation conversions (deg → rad)
- [ ] Performance testing:
  - Measure publish latency
  - Check CPU usage
  - Monitor memory usage
- [ ] Integration test:
  - Start CARLA
  - Start bridge with spawned vehicle + sensors
  - Start Autoware
  - Set initial pose
  - Verify all sensor topics publish
  - Check sensor data in Autoware

**Success Criteria**:
- [x] All sensor types have publishing code
- [x] Coordinate conversion logic implemented
- [ ] Data verification against CARLA (pending integration)
- [ ] Timestamp synchronization testing (pending)
- [ ] Performance benchmarks (pending)
- [ ] Autoware integration validation (pending)

### Phase 5 Summary

**Status**: 🔧 **IN PROGRESS** - Core publishing complete (80%), integration work remaining (20%)

**Completed** (Phase 1 migration):
- [x] All sensor publishing code migrated to rclrs (~635 lines in sensor_bridge.rs)
- [x] Camera: Image + CameraInfo publishing
- [x] LiDAR: PointCloud2 with intensity
- [x] IMU: Linear acceleration, angular velocity, orientation
- [x] GNSS: NavSatFix with status
- [x] Proper QoS profiles (sensor_data_qos)
- [x] CARLA callback → ROS message conversions
- [x] Coordinate system transformations

**Remaining Work**:
- [ ] **Sensor Spawning Integration** (Critical - requires VehicleLifecycle)
  - Load vehicle_config.yaml sensor configuration
  - Parse URDF sensor data from Autoware
  - Use TF transforms for sensor positioning
  - Spawn sensors attached to vehicle in CARLA
  - Connect sensor bridges to spawned sensors
- [ ] **Dynamic Frame IDs** (Important - quality issue)
  - Replace hardcoded frame_ids with TF-based lookup
  - Ensure sensors use correct Autoware frame names
- [ ] **Testing & Verification** (Important - validation)
  - Data accuracy verification
  - Timestamp synchronization checks
  - Performance benchmarks
  - Full Autoware integration testing

**Dependencies**:
- **Phase 4** (Vehicle Lifecycle) must be integrated first
- Sensor spawning requires VehicleLifecycle.spawn_vehicle() extension
- Frame ID updates require TFBuffer integration

**Estimated Remaining Effort**: 1-2 weeks
- Sensor spawning logic: 3-5 days
- Frame ID updates: 1-2 days
- Testing & verification: 3-5 days

**Next Steps**:
1. Complete Phase 4 integration (VehicleLifecycle → main.rs)
2. Extend VehicleLifecycle.spawn_vehicle() to attach sensors
3. Load sensor config from vehicle_config.yaml
4. Update sensor bridge frame IDs to use TF data
5. Test with live CARLA + Autoware

---

## Phase 6: Vehicle Control Integration

**Objective**: Implement vehicle control command subscription and status publishing.

**Status**: ⏳ **PENDING**

**Duration**: 1-2 weeks

**Prerequisites**: Phase 5 (Sensor Data Publishing)

### 6.1 Control Command Subscription

**Objective**: Subscribe to Autoware control commands and apply to CARLA vehicle

**Topics (Subscribe)**:
- `/control/command/actuation_cmd` (tier4_vehicle_msgs/ActuationCommandStamped)

**Tasks**:
- [ ] Create control command subscriber in VehicleBridge
- [ ] Configure control QoS (reliable, keep_last(1))
- [ ] Implement ROS callback → CARLA control conversion:
  ```rust
  fn on_actuation_cmd(&mut self, cmd: tier4_vehicle_msgs::msg::ActuationCommandStamped) {
      let control = carla::rpc::VehicleControl {
          throttle: cmd.actuation.accel_cmd.max(0.0).min(1.0),
          brake: cmd.actuation.brake_cmd.max(0.0).min(1.0),
          steer: cmd.actuation.steer_cmd.max(-1.0).min(1.0),
          hand_brake: false,
          reverse: false,
          manual_gear_shift: false,
          gear: 0,
      };

      self.carla_vehicle.apply_control(&control)?;
  }
  ```
- [ ] Handle control value clamping (0.0-1.0 ranges)
- [ ] Add control logging
- [ ] Test with manual control commands
- [ ] Test with Autoware control output

**Success Criteria**:
- Bridge receives control commands from Autoware
- Control values correctly mapped to CARLA
- Vehicle responds to throttle, brake, and steering
- No control command latency issues

### 6.2 Vehicle Status Publishing

**Objective**: Publish CARLA vehicle status to Autoware

**Topics (Publish)**:
- `/vehicle/status/velocity_status` (autoware_vehicle_msgs/VelocityReport)
- `/vehicle/status/actuation_status` (tier4_vehicle_msgs/ActuationStatusStamped)
- `/vehicle/status/steering_status` (autoware_vehicle_msgs/SteeringReport)

**Tasks**:
- [ ] Create status publishers in VehicleBridge
- [ ] Configure status QoS (reliable, keep_last(10))
- [ ] Implement CARLA → ROS status conversion:
  ```rust
  fn publish_velocity_status(&self) -> Result<()> {
      let velocity = self.carla_vehicle.get_velocity()?;
      let speed = (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt();

      let mut velocity_report = autoware_vehicle_msgs::msg::VelocityReport::default();
      velocity_report.header = create_ros_header(None);
      velocity_report.header.frame_id = "base_link";
      velocity_report.longitudinal_velocity = speed; // m/s
      velocity_report.lateral_velocity = 0.0;
      velocity_report.heading_rate = self.carla_vehicle.get_angular_velocity()?.z.to_radians();

      self.velocity_publisher.publish(&velocity_report)?;
      Ok(())
  }

  fn publish_actuation_status(&self) -> Result<()> {
      let control = self.carla_vehicle.get_control()?;

      let mut actuation_status = tier4_vehicle_msgs::msg::ActuationStatusStamped::default();
      actuation_status.header = create_ros_header(None);
      actuation_status.status.accel_status = control.throttle;
      actuation_status.status.brake_status = control.brake;
      actuation_status.status.steer_status = control.steer;

      self.actuation_publisher.publish(&actuation_status)?;
      Ok(())
  }

  fn publish_steering_status(&self) -> Result<()> {
      let control = self.carla_vehicle.get_control()?;

      let mut steering_report = autoware_vehicle_msgs::msg::SteeringReport::default();
      steering_report.stamp = create_ros_header(None).stamp;
      steering_report.steering_tire_angle = control.steer * MAX_STEERING_ANGLE;

      self.steering_publisher.publish(&steering_report)?;
      Ok(())
  }
  ```
- [ ] Publish status at regular intervals (e.g., 50Hz)
- [ ] Add coordinate system conversions where needed
- [ ] Test status publishing with Autoware
- [ ] Verify Autoware receives and uses status data

**Success Criteria**:
- Status messages published at expected rate
- Autoware receives and displays vehicle status
- Velocity and steering values accurate
- Actuation status reflects current control

### 6.3 Control Verification

**Objective**: Verify bidirectional control integration

**Tasks**:
- [ ] Create control verification script in `scripts/verify_control.py`
- [ ] Test control loop:
  1. Send control command from Autoware
  2. Verify command received by bridge
  3. Verify CARLA vehicle responds
  4. Verify status update published
  5. Verify Autoware receives status
- [ ] Test edge cases:
  - Maximum throttle/brake/steer values
  - Rapid control changes
  - Control command timeout handling
- [ ] Performance testing:
  - Measure control loop latency
  - Check control responsiveness
  - Monitor message rate
- [ ] Integration test:
  - Start full Autoware stack
  - Set navigation goal
  - Verify vehicle follows planned path
  - Check control smoothness

**Success Criteria**:
- Control loop latency <50ms
- Smooth vehicle control (no jitter)
- Autoware successfully controls CARLA vehicle
- Emergency stop works correctly
- Status updates accurate and timely

---

**Document Version**: 1.1
**Last Updated**: 2025-11-05
**Status**: Phase 2, 8 Complete | Phase 5 In Progress (80%) | Phase 6 Pending
**Related Documents**:
- [roadmap.md](../roadmap.md) - Main roadmap index
- [infrastructure.md](infrastructure.md) - Infrastructure setup (Phases 0, 1, 7)
- [integration.md](integration.md) - Autoware integration (Phases 3-4)
- [testing-and-release.md](testing-and-release.md) - Testing and release (Phases 9-10)
- [../architecture.md](../architecture.md) - Architecture design and ADRs
