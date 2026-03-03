# Autoware Integration

This document covers the Autoware integration implementation for the autoware_carla_bridge project, including instance detection, sensor configuration, and vehicle lifecycle management.

**Status**: ✅ **COMPLETE** - All integration modules implemented and verified via end-to-end autonomous driving

**Design Reference**: See `docs/design/autoware-integration-design.md` for complete architecture and API details.

---

## Autoware Integration Foundation

**Objective**: Implement Autoware instance detection and sensor configuration extraction from Autoware topics.

**Status**: ✅ **COMPLETE** (2025-11-05)

**Duration**: 1 week (completed)

**Prerequisites**:
- Phase 8 complete (1-to-1 architecture)
- Autoware 2025.02 environment configured
- Understanding of `/robot_description` and `/tf_static` topics

### 3.1 Add Dependencies

**Objective**: Add required dependencies for URDF parsing and TF2 support.

**Status**: ✅ **COMPLETE**

**Tasks**:
- ✅ Add tf2_msgs package to `src/interface/` (built from geometry2 submodule)
- ✅ Added tf2_msgs = "*" dependency to Cargo.toml
- ✅ Added cargo patch to .cargo/config.toml
- ✅ Used roxmltree for lightweight XML parsing (no urdf-rs dependency)
- ✅ Update colcon build system (Stage 2) to include tf2_msgs
- ✅ Run build to generate tf2_msgs Rust bindings
- ✅ Verify tf2_msgs available in install directory

**Deliverables**:
- ✅ tf2_msgs package integrated in workspace
- ✅ XML parsing capability added (roxmltree)
- ✅ Build system generates tf2_msgs bindings successfully

**Testing**:
- ✅ Verify tf2_msgs types accessible in Rust
- ✅ Test URDF parsing with sample robot_description
- ✅ Confirm no build errors

---

### 3.2 Autoware Instance Detection

**Objective**: Detect Autoware instance via `/robot_description` topic and handle lifecycle.

**Status**: ✅ **COMPLETE**

**Tasks**:
- ✅ Create `src/autoware_carla_bridge/src/autoware_detection.rs` module
- ✅ Implement AutowareDetector struct
  ```rust
  pub struct AutowareDetector {
      robot_desc_sub: Arc<Subscription<std_msgs::msg::String>>,
      robot_desc_received: Arc<AtomicBool>,
      latest_urdf: Arc<Mutex<Option<String>>>,
  }
  ```
- ✅ Subscribe to `/robot_description` topic
  ```rust
  let subscription = node.create_subscription(
      "/robot_description",
      rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable(),
      move |msg: std_msgs::msg::String| {
          *latest_urdf.lock().unwrap() = Some(msg.data.clone());
          robot_desc_received.store(true, Ordering::SeqCst);
          log::info!("Autoware detected: robot_description received ({} bytes)", msg.data.len());
      },
  )?;
  ```
- ✅ Implement detection state machine (NotDetected → Detected → Lost)
- ✅ Detect `/robot_state_publisher` node presence
  ```rust
  let nodes = node.get_node_names()?;
  let has_rsp = nodes.iter().any(|name| name.contains("robot_state_publisher"));
  ```
- ✅ Check for `/tf_static` topic
  ```rust
  let topics = node.get_topic_names_and_types()?;
  let has_tf_static = topics.iter().any(|(name, _)| name == "/tf_static");
  ```
- ✅ Implement periodic health check (detect Autoware disappearance)
- ✅ Add timeout for initial detection (default: 60s)
- ✅ Add logging for state transitions

**Deliverables**:
- ✅ `autoware_detection.rs` module with AutowareDetector
- ✅ State machine for Autoware lifecycle
- ✅ Health check and timeout logic
- ✅ Unit tests for detection logic

**Testing**:
- ✅ Test detection when Autoware already running
- ✅ Test detection when Autoware starts after bridge
- ✅ Test disappearance detection (stop Autoware)
- ✅ Test timeout when Autoware not present
- ✅ Verify logs show clear state transitions

**Benefits**:
- Automatic Autoware detection (no manual configuration)
- Robust handling of Autoware lifecycle events
- Clear visibility into integration state

---

### 3.3 URDF Parsing

**Objective**: Parse `/robot_description` URDF to extract sensor definitions.

**Status**: ✅ **COMPLETE**

**Tasks**:
- ✅ Implement URDF parser using roxmltree (lightweight XML parser)
  ```rust
  // Implemented using roxmltree for lightweight XML parsing
  pub fn parse_urdf_sensors(urdf_xml: &str) -> Result<Vec<SensorConfig>> {
      // Parse XML and extract sensor links
  }
  ```
- ✅ Create SensorConfig struct (in src/urdf_parser.rs)
  ```rust
  pub struct SensorConfig {
      pub link_name: String,
      pub parent_frame: String,
      pub sensor_type: SensorType,
      pub position: nalgebra::Vector3<f64>,
      pub rotation: nalgebra::Vector3<f64>,
  }

  pub enum SensorType {
      Lidar,
      Camera,
      CameraOptical,
      Imu,
      Gnss,
  }
  ```
- ✅ Implement sensor classifier
  ```rust
  fn classify_sensor(link_name: &str) -> Option<SensorType> {
      if link_name.contains("velodyne") || link_name.contains("lidar") {
          Some(SensorType::Lidar)
      } else if link_name.contains("camera") {
          if link_name.contains("optical") {
              Some(SensorType::CameraOptical)
          } else {
              Some(SensorType::Camera)
          }
      } else if link_name.contains("imu") {
          Some(SensorType::Imu)
      } else if link_name.contains("gnss") {
          Some(SensorType::Gnss)
      } else {
          None
      }
  }
  ```
- ✅ Extract all sensor links from URDF
  ```rust
  // Implemented in parse_urdf_sensors() function
  // Parses XML, extracts joints, classifies sensors by name patterns
  ```
- ✅ Log detected sensors with types and frame IDs
- ✅ Handle malformed URDF gracefully (error messages)

**Deliverables**:
- ✅ URDF parsing functions in src/urdf_parser.rs module
- ✅ SensorConfig struct and SensorType enum
- ✅ Sensor extraction and classification logic
- ✅ Unit tests with sample URDF (9 tests passing)

**Testing**:
- ✅ Test with Autoware sample_sensor_kit URDF
- ✅ Test with custom sensor configurations
- ✅ Test error handling (malformed XML)
- ✅ Verify all sensor types detected correctly
- ✅ Test with empty/minimal URDF

**Benefits**:
- Automatic sensor discovery from Autoware
- No manual sensor configuration needed
- Works with any Autoware sensor kit

---

### 3.4 TF2 Transform Parsing

**Objective**: Subscribe to `/tf_static` and build local TF buffer for sensor transforms.

**Status**: ✅ **COMPLETE**

**Tasks**:
- ✅ Create `src/autoware_carla_bridge/src/tf_bridge.rs` module
- ✅ Subscribe to `/tf_static` topic with TRANSIENT_LOCAL QoS
  ```rust
  let tf_static_sub = node.create_subscription(
      "/tf_static",
      rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable(),
      move |msg: tf2_msgs::msg::TFMessage| {
          for transform in &msg.transforms {
              log::debug!(
                  "TF: {} → {} [{:.3}, {:.3}, {:.3}]",
                  transform.header.frame_id,
                  transform.child_frame_id,
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z
              );
              tf_buffer.insert(
                  transform.child_frame_id.clone(),
                  transform.clone()
              );
          }
      },
  )?;
  ```
- ✅ Implement local TF buffer (HashMap-based)
  ```rust
  pub struct TFBuffer {
      transforms: Arc<Mutex<HashMap<String, geometry_msgs::msg::TransformStamped>>>,
  }

  impl TFBuffer {
      pub fn lookup_transform(&self, target_frame: &str, source_frame: &str) -> Result<Transform> {
          // Implement transform lookup logic
          // Handle parent-child relationships
          // Compose transforms if needed
      }
  }
  ```
- ✅ Implement transform lookup for sensor frames
  - Direct lookup (child_frame → parent_frame)
  - Reverse lookup with inversion (parent_frame → child_frame)
  - Identity transforms for same-frame queries
- ✅ Handle missing transforms gracefully (return error)
- ✅ Add comprehensive logging for debugging
- ✅ Helper methods: get_all_frames(), len(), is_empty()

**Deliverables**:
- ✅ `tf_bridge.rs` module with TFBuffer (~210 lines)
- ✅ `/tf_static` subscription with proper QoS
- ✅ Transform lookup with direct/reverse/identity support
- ✅ Tested with live Autoware

**Testing**:
- ✅ Test with sample_sensor_kit TF tree (26 transforms)
- ✅ Test transform lookup between available frames
- ✅ Test missing frame handling (error returned)
- ✅ Verify transform values received from /tf_static
- ✅ Tested with live Autoware instance

**Benefits**:
- Accurate sensor positions from Autoware TF
- Supports complex sensor hierarchies
- No manual transform configuration

---

### 3.5 Coordinate System Conversion

**Objective**: Implement bidirectional ROS ↔ CARLA coordinate transformations.

**Status**: ✅ **COMPLETE**

**Tasks**:
- ✅ Create `src/autoware_carla_bridge/src/coordinate_conversion.rs` module (~450 lines)
- ✅ Document coordinate system differences
  ```rust
  /// ROS (Autoware): Right-handed, +X forward, +Y left, +Z up (meters, radians)
  /// CARLA: Left-handed, +X forward, +Y right, +Z up (centimeters, degrees)
  ///
  /// Conversion requires:
  /// - Y-axis inversion (left→right, right→left)
  /// - Rotation sign flips (pitch, yaw) due to handedness
  /// - Unit conversion (m→cm, rad→deg)
  ```
- ✅ Implement ROS → CARLA position and rotation converters
  ```rust
  pub fn ros_to_carla_transform(ros_tf: &geometry_msgs::msg::Transform) -> carla::geom::Transform {
      carla::geom::Transform {
          location: carla::geom::Location {
              x: ros_tf.translation.x * 100.0,   // m → cm
              y: -ros_tf.translation.y * 100.0,  // m → cm, Y-FLIP
              z: ros_tf.translation.z * 100.0,   // m → cm
          },
          rotation: carla::geom::Rotation {
              roll: quaternion_to_euler(&ros_tf.rotation).roll.to_degrees(),
              pitch: -quaternion_to_euler(&ros_tf.rotation).pitch.to_degrees(),  // SIGN FLIP
              yaw: -quaternion_to_euler(&ros_tf.rotation).yaw.to_degrees(),      // SIGN FLIP
          },
      }
  }
  ```
- ✅ Implement CARLA → ROS position and rotation converters
  ```rust
  pub fn carla_to_ros_transform(carla_tf: &carla::geom::Transform) -> geometry_msgs::msg::Transform {
      geometry_msgs::msg::Transform {
          translation: geometry_msgs::msg::Vector3 {
              x: carla_tf.location.x / 100.0,   // cm → m
              y: -carla_tf.location.y / 100.0,  // cm → m, Y-FLIP
              z: carla_tf.location.z / 100.0,   // cm → m
          },
          rotation: euler_to_quaternion(
              carla_tf.rotation.roll.to_radians(),
              -carla_tf.rotation.pitch.to_radians(),  // SIGN FLIP
              -carla_tf.rotation.yaw.to_radians(),    // SIGN FLIP
          ),
      }
  }
  ```
- ✅ Implement quaternion ↔ Euler angle conversions (ZYX convention)
- ✅ Add extensive unit tests for coordinate math (15 tests)
- ✅ Add documentation with code examples

**Deliverables**:
- ✅ `coordinate_conversion.rs` module (~450 lines)
- ✅ ROS → CARLA position/rotation functions
- ✅ CARLA → ROS position/rotation functions
- ✅ Quaternion/Euler conversion utilities
- ✅ Comprehensive unit tests (15 tests, all passing)
- ✅ Inline documentation with examples

**Testing**:
- ✅ Test identity transform (no change)
- ✅ Test 90° rotations (verify handedness flip)
- ✅ Test translations on each axis
- ✅ Test composed transformations
- ✅ Test round-trip (ROS → CARLA → ROS, within epsilon)
- ✅ Tested with example code in test_autoware_detection.rs

**Benefits**:
- Correct sensor placement in CARLA
- Accurate data transformations
- Well-tested coordinate math

---

### Summary

**Deliverables**:
- ✅ tf2_msgs integrated and building
- ✅ roxmltree for XML parsing (lightweight alternative to urdf-rs)
- ✅ `autoware_detection.rs` - Autoware detection and lifecycle (~350 lines)
- ✅ `urdf_parser.rs` - URDF sensor extraction (~280 lines)
- ✅ `tf_bridge.rs` - TF buffer and transform lookup (~210 lines)
- ✅ `coordinate_conversion.rs` - ROS ↔ CARLA transforms (~450 lines)
- ✅ Unit tests for all modules (54 tests total, all passing)
- ✅ `examples/test_autoware_detection.rs` - Integration test example
- ✅ Documentation updated

**Success Criteria**:
- ✅ Bridge detects running Autoware instance
- ✅ URDF parsed successfully, sensors extracted (26 sensors from sample_sensor_kit)
- ✅ TF transforms received from /tf_static
- ✅ Coordinate conversions tested and accurate (15 unit tests)
- ✅ Code compiles with zero warnings
- ✅ All unit tests pass (54/54)

**Risks** (Mitigated):
- ✅ TF transform composition complexity - Mitigated with simple direct/reverse lookup
- ✅ Coordinate conversion edge cases - Mitigated with 15 comprehensive unit tests
- ✅ URDF parsing robustness - Mitigated with roxmltree and graceful error handling

**Completed**: 2025-11-05

---

## Vehicle Lifecycle Management

**Objective**: Manage CARLA vehicle lifecycle tied to Autoware instance, including spawning, initial pose, and cleanup.

**Status**: ✅ **COMPLETE** - Vehicle spawning, sensors, cleanup all working

**Note**: The original design used `/initialpose` topic and a `vehicle_lifecycle.rs` module. The actual implementation evolved to use modern Autoware localization API (`/localization/initialization_state` + `/localization/kinematic_state`) with spawning logic in `main.rs` and vehicle management in `carla_vehicle.rs`.

### 3.6 Initial Pose / Localization Integration

**Objective**: Determine vehicle spawn pose and trigger spawning.

**Status**: ✅ **COMPLETE** - Uses modern Autoware localization API

**Actual Implementation** (differs from original design):
- Subscribes to `/localization/initialization_state` (monitors INITIALIZED state)
- Subscribes to `/localization/kinematic_state` (receives vehicle pose)
- Spawns vehicle when localization becomes INITIALIZED (state 3)
- Initial spawn pose configured in `config/bridge.yaml`
- No backward compatibility with legacy `/initialpose` topic (removed 2025-11-23)

**Implementation**: `autoware.rs` (localization state monitoring), `bridge_config.rs` (initial pose)

**Verified**: Vehicle spawns at correct location when Autoware localization initializes

---

### 3.7 Sensor Parameter Configuration

**Objective**: Load CARLA sensor parameters from configuration file.

**Status**: ✅ **COMPLETE** - Full config-driven sensor spawning implemented

**Implementation**: `sensor_config.rs` (VehicleConfig + SensorDefinition structs)

**Config file**: `src/autoware_carla_bridge/config/vehicle_config.yaml` (single source of truth)
- Vehicle blueprint selection
- Per-sensor CARLA blueprint and parameters
- Link name for TF transform lookup
- Loaded via `vehicle_config` ROS parameter

**Current sensors configured**:
- LiDAR: 128 channels, 200m range, 20Hz rotation
- Camera: 800x400, 74.5 FOV
- IMU: 10ms tick
- GNSS: 100ms tick

---

### 3.8 Vehicle Spawning Logic

**Objective**: Spawn CARLA vehicle with sensors when all prerequisites ready.

**Status**: ✅ **COMPLETE** - Vehicle and sensors spawn correctly

**Actual Implementation** (differs from original design):
- No `vehicle_lifecycle.rs` state machine - stateless spawning in `main.rs`
- `CarlaVehicle::new()` in `carla_vehicle.rs` handles both vehicle and sensor spawning
- Sequential workflow: CARLA → Autoware → TF → Config → Spawn
- Sensors attached with TF transforms from `/tf_static`
- Sensor parameters from `vehicle_config.yaml`

**Workflow** (`main.rs`):
1. Connect to CARLA (infinite retry)
2. Wait for Autoware detection (`/robot_description`)
3. Wait for TF transforms (minimum 4 frames)
4. Load VehicleConfig from YAML
5. `CarlaVehicle::new()` spawns vehicle + all sensors
6. Create sensor bridges for each spawned sensor
7. Create vehicle control bridge

**Verified**: Vehicle and 4 sensors (LiDAR, camera, IMU, GNSS) spawn correctly in CARLA

---

### 3.9 Vehicle Cleanup on Autoware Loss

**Objective**: Detect Autoware disappearance and clean up CARLA vehicle.

**Status**: ✅ **COMPLETE** (cleanup works, respawn is TODO)

**Implementation** (`main.rs:474-499`):
- Health check every tick: `autoware.health_check()` + `autoware.is_alive()`
- On Autoware loss: `carla_vehicle.lock().unwrap().cleanup()?`
- Bridge waits for Autoware to reconnect
- **Known limitation**: Vehicle respawn after reconnection not yet implemented (TODO at `main.rs:496`)
  - Bridge logs "please restart the bridge" after reconnection
  - Full restart required for respawn

---

### 3.10 Vehicle Teleportation

**Objective**: Update vehicle pose when receiving new pose messages.

**Status**: ⏳ **NOT IMPLEMENTED** - Original `/initialpose`-based design was replaced by modern API

**Note**: The bridge now uses modern Autoware localization API instead of `/initialpose`. Teleportation via RViz pose updates is not currently supported. The vehicle spawns once at the configured initial pose and doesn't move in response to new pose estimates.

---

### Summary

**Status**: ✅ **COMPLETE** - All integration modules implemented and verified

**Implemented modules**:
- ✅ `autoware_detection.rs` - Autoware detection via `/robot_description` (~340 lines)
- ✅ `urdf_parser.rs` - URDF sensor extraction (~172 lines, deprecated in favor of config)
- ✅ `tf_bridge.rs` - TF buffer and transform lookup (~307 lines)
- ✅ `coordinate_conversion.rs` - ROS ↔ CARLA transforms (~651 lines, 15 unit tests)
- ✅ `carla_vehicle.rs` - Vehicle and sensor spawning (~329 lines)
- ✅ `sensor_config.rs` - Config-driven sensor definitions (~637 lines)
- ✅ `autoware.rs` - Autoware coordination, localization, ground truth (~1423 lines)
- ✅ `vehicle_control.rs` - Bidirectional vehicle control (~215 lines)

**Verified via end-to-end autonomous driving**:
- ✅ Bridge detects Autoware, waits for TF, spawns vehicle with sensors
- ✅ Sensors publish to correct ROS topics
- ✅ Vehicle control works (Autoware controls CARLA vehicle)
- ✅ Localization auto-initialization works
- ✅ Cleanup on Autoware loss works (respawn TODO)

**Known limitations**:
- Vehicle teleportation via RViz not implemented (modern API doesn't use `/initialpose`)
- Vehicle respawn after Autoware reconnection requires bridge restart
