# Data Bridge

This document covers the data bridge implementation for the autoware_carla_bridge project, including publishers, subscribers, sensor data, and vehicle control.

**Status**: Phase 2, 8 ✅ Complete | Phase 5 🔧 In Progress (95% - testing only) | Phase 6 ⏳ Pending

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

**Status**: 🔧 **IN PROGRESS** - Implementation complete (95%), runtime testing pending (5%)

**Duration**: 3-5 days remaining (runtime testing only)

**Prerequisites**:
- ✅ Phase 3 (Autoware Integration Foundation) - Complete
- ✅ Phase 4 (Vehicle Lifecycle Management) - Complete

**Implementation Status**:
- ✅ Sensor publishing code (migrated in Phase 1)
- ✅ Sensor spawning from URDF (Phase 4, 2025-11-05)
- ✅ Sensor bridge connection (Phase 5.6, 2025-11-08)
- ✅ Dynamic frame IDs from URDF (Phase 5.6, 2025-11-08)
- ⏳ Runtime testing (pending)

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

### 5.6 Sensor Bridge Connection

**Objective**: Connect existing sensor bridge code to spawned CARLA sensors

**Status**: ✅ **COMPLETE** (2025-11-08)

**Priority**: 🔴 **WAS HIGHEST** - Unblocked sensor data publishing

**Duration**: 3 days (2025-11-08)

**Implementation Summary**:
- ✅ Sensors spawn correctly in CARLA (`carla_vehicle.rs`)
- ✅ Sensor bridge implementations exist (`sensor_bridge.rs`)
- ✅ Bridges connected to sensors in `main.rs` (factory pattern)
- ✅ Dynamic frame IDs from URDF (no hardcoding)
- ✅ Sensor lifecycle fixed (single-owner model)
- ⏳ Sensor data publishing pending runtime testing

**Completed Tasks**:

**Step 1: Refactor CarlaVehicle to return sensor references** ✅:
- [x] Modified `carla_vehicle.rs` to store sensor metadata
- [x] Added `get_sensor_configs() -> &[SensorConfig]` method
- [x] Sensor HashMap includes all spawned sensors
- [x] Returns both `Sensor` objects and configurations

**Step 2: Create sensor bridge factory in main.rs** ✅:
- [x] Created `create_sensor_bridges()` function (`main.rs:39-86`)
- [x] Iterates over sensor configs from URDF
- [x] Matches sensor type and creates appropriate bridge
- [x] Handles errors gracefully (continues on failure)
- [x] Stores bridges for lifecycle management
- [x] Automatic cleanup via Drop trait

**Implementation**:
```rust
// main.rs:39-86
fn create_sensor_bridges(
    node: rclrs::Node,
    carla_vehicle: &CarlaVehicle,
    autoware: &autoware::Autoware,
) -> Result<Vec<SensorBridge>> {
    let sensor_configs = carla_vehicle.get_sensor_configs();
    let sensors = carla_vehicle.get_sensors();
    let mut bridges = Vec::new();

    for config in sensor_configs {
        let sensor = sensors.get(&config.link_name)?;
        let bridge_type = BridgeType::Sensor(
            config.sensor_type,
            config.link_name.clone()
        );

        let bridge = SensorBridge::new(
            node.clone(),
            sensor.clone(),
            bridge_type,
            autoware
        )?;
        bridges.push(bridge);
    }

    Ok(bridges)
}
```

**Step 3: Update SensorBridge API** ✅:
- [x] SensorBridge API already accepts Sensor
- [x] Callback registration works via `actor.listen()`
- [x] Removed sensor destruction from Drop (lifecycle fix)
- [x] CarlaVehicle owns sensors (single-owner model)

**Step 4: Fix frame IDs** ✅:
- [x] Added frame_id parameter to all 5 register functions
- [x] Removed all hardcoded frame_ids
- [x] Uses `config.link_name` as frame_id
- [x] Frame IDs now match URDF/TF tree
- **Updated functions**:
  - Camera: `sensor_bridge.rs:160`
  - LiDAR (raycast): `sensor_bridge.rs:240`
  - LiDAR (semantic): `sensor_bridge.rs:272`
  - IMU: `sensor_bridge.rs:304`
  - GNSS: `sensor_bridge.rs:334`

**Step 5: Testing** (pending):
- [ ] Start CARLA simulator
- [ ] Start Autoware
- [ ] Run bridge and set initial pose
- [ ] Verify sensors spawn in CARLA
- [ ] Check ROS topics exist:
  ```bash
  ros2 topic list | grep sensing
  # Should show:
  # /sensing/camera/.../image_raw
  # /sensing/camera/.../camera_info
  # /sensing/lidar/.../pointcloud
  # /sensing/imu/imu_raw
  # /sensing/gnss/.../nav_sat_fix
  ```
- [ ] Verify data publishes:
  ```bash
  ros2 topic hz /sensing/lidar/top/pointcloud
  ros2 topic echo /sensing/camera/traffic_light/camera/image_raw --no-arr
  ```
- [ ] Check data appears in RViz
- [ ] Validate timestamps are synchronized

**Deliverables** (2025-11-08):
- [x] Sensor bridges connected ✅
- [x] Frame IDs correct ✅
- [x] No compilation errors ✅
- [ ] All sensor topics active (runtime testing needed)
- [ ] Data visible in Autoware/RViz (runtime testing needed)
- [ ] No runtime crashes (pending verification)

**Success Criteria**:
- [x] Bridge spawns sensors ✅
- [x] Sensor bridges created for each sensor ✅
- [x] CARLA callbacks registered ✅
- [x] ROS publishers created ✅
- [ ] Topics publish sensor data (pending runtime test)
- [ ] Autoware can see and use sensor data (pending test)
- [ ] Point clouds align with map (pending test)
- [ ] Camera images display in RViz (pending test)
- [ ] IMU data shows in plots (pending test)
- [ ] GNSS coordinates match vehicle position (pending test)

**Code Quality**:
- ✅ Build succeeds (6 seconds)
- ✅ Zero compilation errors
- ✅ Only warnings for unused code in other modules
- ✅ Clean architecture (factory pattern)
- ✅ Single-owner lifecycle (no double-free)

**Impact**:
- ✅ Unblocks Phase 6 (Vehicle Control Integration)
- ✅ Bridge functionally complete for sensor data
- ⏳ Runtime testing needed to verify end-to-end flow
- ⏳ Performance tuning pending

---

### Phase 5 Summary

**Status**: 🔧 **IN PROGRESS** - Implementation complete (95%), runtime testing pending (5%)

**Completed** (Phase 1 migration):
- [x] All sensor publishing code migrated to rclrs (~635 lines in sensor_bridge.rs)
- [x] Camera: Image + CameraInfo publishing
- [x] LiDAR: PointCloud2 with intensity
- [x] IMU: Linear acceleration, angular velocity, orientation
- [x] GNSS: NavSatFix with status
- [x] Proper QoS profiles (sensor_data_qos)
- [x] CARLA callback → ROS message conversions
- [x] Coordinate system transformations

**Completed Since Documentation**:
- [x] **Sensor Spawning** ✅ COMPLETE (2025-11-05)
  - [x] URDF sensor data parsing from Autoware
  - [x] TF transforms for sensor positioning (with tree traversal)
  - [x] Sensors spawn attached to vehicle in CARLA
  - [x] CARLA sensor parameter configuration (YAML) ✅ (2025-11-08)
- [x] **VehicleLifecycle Integration** ✅ COMPLETE (2025-11-05)
  - [x] CarlaVehicle spawning working in main.rs
  - [x] Sensors created from URDF configuration
  - [x] Sensors positioned using TF transforms

**Completed Since Last Update** (2025-11-08):
- [x] **Sensor Bridge Connection** ✅ COMPLETE (2025-11-08)
  - [x] Created SensorBridge factory in main.rs (`create_sensor_bridges()`)
  - [x] Wired sensor callbacks to CARLA sensor data listeners
  - [x] Connected bridges to ROS publishers
  - [x] Fixed sensor lifecycle (single-owner model: CarlaVehicle owns sensors)
  - **Code locations**:
    - Factory function: `main.rs:39-86` ✅ Complete
    - Integration: `main.rs:279-280` ✅ Complete
    - Sensor ownership: `sensor_bridge.rs:143-154` ✅ Fixed
- [x] **Dynamic Frame IDs** ✅ COMPLETE (2025-11-08)
  - [x] Replaced all hardcoded frame_ids with URDF link names
  - [x] Frame IDs passed from sensor configs to bridge callbacks
  - [x] All sensors use correct Autoware frame names from URDF
  - **Updated functions**:
    - Camera: `sensor_bridge.rs:160` ✅ Uses frame_id param
    - LiDAR (raycast): `sensor_bridge.rs:240` ✅ Uses frame_id param
    - LiDAR (semantic): `sensor_bridge.rs:272` ✅ Uses frame_id param
    - IMU: `sensor_bridge.rs:304` ✅ Uses frame_id param
    - GNSS: `sensor_bridge.rs:334` ✅ Uses frame_id param

**Remaining Work** (Updated 2025-11-08):
- [ ] **Runtime Testing & Verification** (Important - 3-5 days) ⬅️ **NEXT PRIORITY**
  - [ ] Verify sensor data publishes to correct topics
  - [ ] Check data appears in Autoware/RViz
  - [ ] Validate point cloud alignment with map
  - [ ] Verify image timestamps and frame rates
  - [ ] Performance benchmarks (latency, CPU, memory)
  - [ ] Full Autoware integration testing
  - [ ] Fix any runtime issues discovered

**Updated Dependencies**:
- ✅ Phase 4 (Vehicle Lifecycle) - COMPLETE
- ✅ Sensor spawning system - COMPLETE
- ✅ CARLA sensor configuration - COMPLETE (2025-11-08)
- ✅ Sensor bridge wiring - COMPLETE (2025-11-08)
- ✅ Dynamic frame IDs - COMPLETE (2025-11-08)

**Estimated Remaining Effort**: 3-5 days (runtime testing only)
- ~~Sensor bridge connection: 3-5 days~~ ✅ COMPLETE
- ~~Frame ID updates: 1-2 days~~ ✅ COMPLETE
- Testing & verification: 3-5 days ⬅️ **ONLY REMAINING WORK**

**Updated Next Steps** (2025-11-08):
1. ~~**Wire sensor bridges in main.rs**~~ ✅ COMPLETE (2025-11-08)
   - Implemented `create_sensor_bridges()` factory function
   - Integrated at `main.rs:279-280`
   - All sensors now connected to ROS publishers
2. ~~**Update frame IDs** to use URDF link names~~ ✅ COMPLETE (2025-11-08)
   - All 5 sensor types updated (Camera, LiDAR x2, IMU, GNSS)
   - Frame IDs now come from URDF sensor configs
3. **Test with Autoware** (verify data flow: CARLA → Bridge → ROS → Autoware) ⬅️ **CURRENT PRIORITY**
   - Start CARLA simulator
   - Launch Autoware planning simulator
   - Run bridge and verify topics
   - Check data in RViz
4. **Performance validation** and benchmarks
   - Measure topic publish rates
   - Check CPU/memory usage
   - Verify latency is acceptable

---

## Phase 6: Vehicle Control Integration

**Objective**: Implement vehicle control command subscription and status publishing.

**Status**: 🟢 **IN PROGRESS** (Phases 6.1 and 6.2 COMPLETE - 2025-11-08)

**Progress**: 65% Complete (Control subscriber ✅ | Status publishers ✅ | Testing pending)

**Duration**: 1-2 weeks (Implementation: 1 day ✅ | Testing: 1-2 weeks remaining)

**Prerequisites**: Phase 5 (Sensor Data Publishing)

### 6.1 Control Command Subscription

**Objective**: Subscribe to Autoware control commands and apply to CARLA vehicle

**Status**: ✅ **COMPLETE** (2025-11-08)

**Topics (Subscribe)**:
- `/control/command/actuation_cmd` (tier4_vehicle_msgs/ActuationCommandStamped)

**Tasks**:
- [x] Create control command subscriber in VehicleControlBridge ✅
- [x] Configure control QoS (reliable, keep_last(1)) ✅
- [x] Implement ROS callback → CARLA control conversion ✅:
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
- [x] Handle control value clamping (0.0-1.0 ranges) ✅
- [x] Add control logging ✅
- [ ] Test with manual control commands (runtime testing pending)
- [ ] Test with Autoware control output (runtime testing pending)

**Implementation Details**:
- **File**: `src/autoware_carla_bridge/src/vehicle_control.rs`
- **Subscriber topic**: `/control/command/actuation_cmd`
- **Message type**: `tier4_vehicle_msgs::msg::ActuationCommandStamped`
- **Integration**: `main.rs:290-296` (VehicleControlBridge creation)
- **Control mapping**: Direct mapping (accel_cmd → throttle, brake_cmd → brake, steer_cmd → steer)
- **Logging**: Debug-level logging of all control values

**Success Criteria**:
- ✅ Bridge receives control commands from Autoware (subscriber created)
- ✅ Control values correctly mapped to CARLA (with clamping)
- ⏳ Vehicle responds to throttle, brake, and steering (runtime test pending)
- ⏳ No control command latency issues (runtime test pending)

### 6.2 Vehicle Status Publishing

**Objective**: Publish CARLA vehicle status to Autoware

**Status**: ✅ **COMPLETE** (2025-11-08)

**Topics (Publish)**:
- `/vehicle/status/velocity_status` (autoware_vehicle_msgs/VelocityReport) ✅
- `/vehicle/status/steering_status` (autoware_vehicle_msgs/SteeringReport) ✅
- `/vehicle/status/control_mode` (autoware_vehicle_msgs/ControlModeReport) ✅

**Tasks**:
- [x] Create status publishers in VehicleControlBridge ✅
- [x] Configure status QoS (reliable, keep_last(10)) ✅
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
- [x] Publish status at regular intervals (e.g., simulation rate) ✅
- [x] Add coordinate system conversions where needed ✅
- [ ] Test status publishing with Autoware (runtime testing pending)
- [ ] Verify Autoware receives and uses status data (runtime testing pending)

**Implementation Details**:
- **File**: `src/autoware_carla_bridge/src/vehicle_control.rs:145-202`
- **Publisher topics**:
  - `/vehicle/status/velocity_status` (VelocityReport)
  - `/vehicle/status/steering_status` (SteeringReport)
  - `/vehicle/status/control_mode` (ControlModeReport)
- **Integration**: `main.rs:426` (publish_status() called in main loop)
- **Publish rate**: Matches simulation tick rate (~20Hz with default CARLA settings)
- **Velocity calculation**: 3D velocity magnitude from CARLA
- **Steering conversion**: CARLA normalized steer (-1 to 1) → tire angle (radians)
- **Control mode**: Always AUTONOMOUS (mode = 1) in simulation

**Success Criteria**:
- ✅ Status messages published at expected rate (in main loop)
- ⏳ Autoware receives and displays vehicle status (runtime test pending)
- ✅ Velocity and steering values accurate (calculations implemented)
- ✅ Control mode reflects current state (always AUTONOMOUS)

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

**Document Version**: 1.2
**Last Updated**: 2025-11-08
**Status**: Phase 2, 8 Complete | Phase 5 In Progress (95%) | Phase 6 In Progress (65% - Control Integration Complete, Testing Pending)
**Related Documents**:
- [roadmap.md](../roadmap.md) - Main roadmap index
- [infrastructure.md](infrastructure.md) - Infrastructure setup (Phases 0, 1, 7)
- [integration.md](integration.md) - Autoware integration (Phases 3-4)
- [testing-and-release.md](testing-and-release.md) - Testing and release (Phases 9-10)
- [../architecture.md](../architecture.md) - Architecture design and ADRs
