# Autoware Integration

This document covers the Autoware integration implementation for the autoware_carla_bridge project, including instance detection, sensor configuration, and vehicle lifecycle management.

**Status**: ⏳ **PENDING** - Design complete, ready for implementation

**Design Reference**: See `docs/autoware-integration-design.md` for complete architecture and API details.

---

## Phase 3: Autoware Integration Foundation

**Objective**: Implement Autoware instance detection and sensor configuration extraction from Autoware topics.

**Status**: ⏳ **PENDING**

**Duration**: 1-2 weeks

**Prerequisites**:
- Phase 8 complete (1-to-1 architecture)
- Autoware 2025.02 environment configured
- Understanding of `/robot_description` and `/tf_static` topics

### 3.1 Add Dependencies

**Objective**: Add required dependencies for URDF parsing and TF2 support.

**Tasks**:
- [ ] Add tf2_msgs package to `src/interface/`
  ```bash
  cd src/interface
  git submodule add https://github.com/ros2/common_interfaces.git -b humble tf2_msgs_source
  ln -s tf2_msgs_source/tf2_msgs tf2_msgs
  ```
- [ ] Add urdf-rs crate to `src/autoware_carla_bridge/Cargo.toml`
  ```toml
  urdf-rs = "0.7"
  ```
  **Alternative**: roxmltree for lightweight XML parsing
- [ ] Update colcon build system (Stage 2) to include tf2_msgs
- [ ] Run build to generate tf2_msgs Rust bindings
- [ ] Verify tf2_msgs available: `ls src/interface/install/tf2_msgs/share/tf2_msgs/rust/`

**Deliverables**:
- [ ] tf2_msgs package integrated in workspace
- [ ] urdf-rs dependency added and compiling
- [ ] Build system generates tf2_msgs bindings successfully

**Testing**:
- [ ] Verify tf2_msgs types accessible in Rust
- [ ] Test URDF parsing with sample robot_description
- [ ] Confirm no build errors

---

### 3.2 Autoware Instance Detection

**Objective**: Detect Autoware instance via `/robot_description` topic and handle lifecycle.

**Tasks**:
- [ ] Create `src/autoware_carla_bridge/src/autoware_detection.rs` module
- [ ] Implement AutowareDetector struct
  ```rust
  pub struct AutowareDetector {
      robot_desc_sub: Arc<Subscription<std_msgs::msg::String>>,
      robot_desc_received: Arc<AtomicBool>,
      latest_urdf: Arc<Mutex<Option<String>>>,
  }
  ```
- [ ] Subscribe to `/robot_description` topic
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
- [ ] Implement detection state machine (NotDetected → Detected → Lost)
- [ ] Detect `/robot_state_publisher` node presence
  ```rust
  let nodes = node.get_node_names()?;
  let has_rsp = nodes.iter().any(|name| name.contains("robot_state_publisher"));
  ```
- [ ] Check for `/tf_static` topic
  ```rust
  let topics = node.get_topic_names_and_types()?;
  let has_tf_static = topics.iter().any(|(name, _)| name == "/tf_static");
  ```
- [ ] Implement periodic health check (detect Autoware disappearance)
- [ ] Add timeout for initial detection (default: 60s)
- [ ] Add logging for state transitions

**Deliverables**:
- [ ] `autoware_detection.rs` module with AutowareDetector
- [ ] State machine for Autoware lifecycle
- [ ] Health check and timeout logic
- [ ] Unit tests for detection logic

**Testing**:
- [ ] Test detection when Autoware already running
- [ ] Test detection when Autoware starts after bridge
- [ ] Test disappearance detection (stop Autoware)
- [ ] Test timeout when Autoware not present
- [ ] Verify logs show clear state transitions

**Benefits**:
- Automatic Autoware detection (no manual configuration)
- Robust handling of Autoware lifecycle events
- Clear visibility into integration state

---

### 3.3 URDF Parsing

**Objective**: Parse `/robot_description` URDF to extract sensor definitions.

**Tasks**:
- [ ] Implement URDF parser using urdf-rs
  ```rust
  use urdf_rs::Robot;

  pub fn parse_robot_description(urdf_xml: &str) -> Result<Robot> {
      urdf_rs::read_from_string(urdf_xml)
          .map_err(|e| BridgeError::AutowareIssue(format!("URDF parse error: {}", e)))
  }
  ```
- [ ] Create SensorConfig struct
  ```rust
  pub struct SensorConfig {
      pub name: String,
      pub sensor_type: SensorType,
      pub frame_id: String,
      pub carla_blueprint_id: String,
      pub topic_name: String,
  }

  pub enum SensorType {
      Lidar,
      Camera,
      Imu,
      Gnss,
  }
  ```
- [ ] Implement sensor classifier
  ```rust
  fn classify_sensor(link_name: &str) -> Option<SensorType> {
      if link_name.contains("velodyne") || link_name.contains("lidar") {
          Some(SensorType::Lidar)
      } else if link_name.contains("camera") && !link_name.contains("optical") {
          Some(SensorType::Camera)
      } else if link_name.contains("imu") {
          Some(SensorType::Imu)
      } else if link_name.contains("gnss") {
          Some(SensorType::Gnss)
      } else {
          None
      }
  }
  ```
- [ ] Extract all sensor links from URDF
  ```rust
  pub fn extract_sensors(robot: &Robot) -> Vec<SensorConfig> {
      robot.links.iter()
          .filter_map(|link| {
              classify_sensor(&link.name).map(|sensor_type| {
                  SensorConfig {
                      name: link.name.clone(),
                      sensor_type,
                      frame_id: link.name.clone(),
                      carla_blueprint_id: infer_carla_blueprint(&sensor_type, &link.name),
                      topic_name: format_topic_name(&sensor_type, &link.name),
                  }
              })
          })
          .collect()
  }
  ```
- [ ] Log detected sensors with types and frame IDs
- [ ] Handle malformed URDF gracefully (error messages)

**Deliverables**:
- [ ] URDF parsing functions in autoware_detection.rs
- [ ] SensorConfig struct and SensorType enum
- [ ] Sensor extraction and classification logic
- [ ] Unit tests with sample URDF

**Testing**:
- [ ] Test with Autoware sample_sensor_kit URDF
- [ ] Test with custom sensor configurations
- [ ] Test error handling (malformed XML)
- [ ] Verify all sensor types detected correctly
- [ ] Test with empty/minimal URDF

**Benefits**:
- Automatic sensor discovery from Autoware
- No manual sensor configuration needed
- Works with any Autoware sensor kit

---

### 3.4 TF2 Transform Parsing

**Objective**: Subscribe to `/tf_static` and build local TF buffer for sensor transforms.

**Tasks**:
- [ ] Create `src/autoware_carla_bridge/src/tf_bridge.rs` module
- [ ] Subscribe to `/tf_static` topic
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
- [ ] Implement local TF buffer (HashMap-based)
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
- [ ] Implement transform lookup for sensor frames
  - Lookup from base_link to sensor frame
  - Handle multi-level hierarchy (base_link → sensor_kit_base_link → sensor)
  - Compose transforms along chain
- [ ] Handle missing transforms gracefully (log warning, skip sensor)
- [ ] Add frame hierarchy visualization in logs
- [ ] Implement timeout for TF data availability

**Deliverables**:
- [ ] `tf_bridge.rs` module with TFBuffer
- [ ] `/tf_static` subscription
- [ ] Transform lookup and composition logic
- [ ] Unit tests for transform math

**Testing**:
- [ ] Test with sample_sensor_kit TF tree
- [ ] Test multi-level transform composition
- [ ] Test missing frame handling
- [ ] Verify transform values match TF tree
- [ ] Test with `ros2 run tf2_ros tf2_echo base_link sensor_link`

**Benefits**:
- Accurate sensor positions from Autoware TF
- Supports complex sensor hierarchies
- No manual transform configuration

---

### 3.5 Coordinate System Conversion

**Objective**: Implement bidirectional ROS ↔ CARLA coordinate transformations.

**Tasks**:
- [ ] Create `src/autoware_carla_bridge/src/coordinate_conversion.rs` module
- [ ] Document coordinate system differences
  ```rust
  /// ROS (Autoware): Right-handed, +X forward, +Y left, +Z up (meters, radians)
  /// CARLA: Left-handed, +X forward, +Y right, +Z up (centimeters, degrees)
  ///
  /// Conversion requires:
  /// - Y-axis inversion (left→right, right→left)
  /// - Rotation sign flips (pitch, yaw) due to handedness
  /// - Unit conversion (m→cm, rad→deg)
  ```
- [ ] Implement ROS → CARLA transform converter
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
- [ ] Implement CARLA → ROS transform converter
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
- [ ] Implement quaternion ↔ Euler angle conversions
- [ ] Add extensive unit tests for coordinate math
- [ ] Add visual diagram in documentation

**Deliverables**:
- [ ] `coordinate_conversion.rs` module
- [ ] ROS → CARLA transform functions
- [ ] CARLA → ROS transform functions
- [ ] Quaternion/Euler conversion utilities
- [ ] Comprehensive unit tests (100+ test cases)
- [ ] Documentation with diagrams

**Testing**:
- [ ] Test identity transform (no change)
- [ ] Test 90° rotations (verify handedness flip)
- [ ] Test translations on each axis
- [ ] Test composed transformations
- [ ] Test round-trip (ROS → CARLA → ROS)
- [ ] Compare with known good transforms from sample_sensor_kit

**Benefits**:
- Correct sensor placement in CARLA
- Accurate data transformations
- Well-tested coordinate math

---

### Phase 3 Summary

**Deliverables**:
- ✅ tf2_msgs integrated and building
- ✅ urdf-rs dependency added
- ✅ `autoware_detection.rs` - Autoware detection and lifecycle
- ✅ `tf_bridge.rs` - TF buffer and transform lookup
- ✅ `coordinate_conversion.rs` - ROS ↔ CARLA transforms
- ✅ Unit tests for all modules
- ✅ Documentation updated

**Success Criteria**:
- [ ] Bridge detects running Autoware instance
- [ ] URDF parsed successfully, sensors extracted
- [ ] TF transforms queried from /tf_static
- [ ] Coordinate conversions tested and accurate
- [ ] Code compiles with zero warnings
- [ ] All unit tests pass

**Risks**:
- TF transform composition complexity
- Coordinate conversion edge cases
- URDF parsing robustness

**Mitigation**:
- Extensive testing with real Autoware data
- Compare with known good transforms
- Handle malformed input gracefully

---

## Phase 4: Vehicle Lifecycle Management

**Objective**: Manage CARLA vehicle lifecycle tied to Autoware instance, including spawning, initial pose, and cleanup.

**Status**: ⏳ **PENDING**

**Duration**: 1-2 weeks

**Prerequisites**:
- Phase 3 complete (Autoware detection, sensor config)
- Understanding of Autoware initial pose workflow

### 4.1 Initial Pose Subscription

**Objective**: Subscribe to `/initialpose` from RViz and use it to spawn/teleport vehicle.

**Tasks**:
- [ ] Subscribe to `/initialpose` topic
  ```rust
  let initialpose_sub = node.create_subscription(
      "/initialpose",
      rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable(),
      move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
          log::info!(
              "Initial pose received: ({:.2}, {:.2}, {:.2}) in frame '{}'",
              msg.pose.pose.position.x,
              msg.pose.pose.position.y,
              msg.pose.pose.position.z,
              msg.header.frame_id
          );
          let carla_transform = ros_pose_to_carla_transform(&msg.pose.pose);
          vehicle_spawner.set_initial_pose(carla_transform);
      },
  )?;
  ```
- [ ] Implement pose → CARLA transform conversion (with coordinate conversion)
- [ ] Handle map frame alignment
  - Default: assume map origin = CARLA (0,0,0)
  - Optional: Add ROS parameter for map_origin offset
- [ ] Add ROS parameter for map origin configuration
  ```yaml
  carla_bridge:
    ros__parameters:
      map_origin:
        x: 0.0
        y: 0.0
        z: 0.0
        yaw: 0.0
  ```
- [ ] Update initial pose on new messages (teleport vehicle if already spawned)

**Deliverables**:
- [ ] `/initialpose` subscription
- [ ] Pose conversion logic
- [ ] Map origin ROS parameter
- [ ] Teleportation support

**Testing**:
- [ ] Test with RViz 2D Pose Estimate tool
- [ ] Verify vehicle spawns at correct location in CARLA
- [ ] Test map origin offset parameter
- [ ] Test pose updates (teleportation)

---

### 4.2 Sensor Parameter Configuration

**Objective**: Load CARLA sensor parameters from configuration file.

**Tasks**:
- [ ] Create `config/carla_sensor_mappings.yaml` (template from design doc)
- [ ] Define sensor → CARLA blueprint mappings
  ```yaml
  sensors:
    velodyne_top:
      carla_blueprint: sensor.lidar.ray_cast
      parameters:
        channels: '128'
        range: '130.0'
        points_per_second: '1280000'
        rotation_frequency: '10.0'
    camera0:
      carla_blueprint: sensor.camera.rgb
      parameters:
        image_size_x: '800'
        image_size_y: '400'
        fov: '74.5'
        sensor_tick: '0.033'
  ```
- [ ] Add serde_yaml to parse config
  ```toml
  serde_yaml = "0.9"
  ```
- [ ] Implement config loader
  ```rust
  pub struct SensorMappings {
      pub sensors: HashMap<String, SensorMapping>,
  }

  pub struct SensorMapping {
      pub carla_blueprint: String,
      pub parameters: HashMap<String, String>,
      pub topic: String,
  }

  pub fn load_sensor_mappings(path: &str) -> Result<SensorMappings> {
      let file = std::fs::File::open(path)?;
      serde_yaml::from_reader(file)
          .map_err(|e| BridgeError::Configuration(e.to_string()))
  }
  ```
- [ ] Match Autoware sensors to CARLA blueprints
- [ ] Apply fallback defaults for unknown sensors
- [ ] Add CLI parameter for config file path (`--sensor-config`)

**Deliverables**:
- [ ] `config/carla_sensor_mappings.yaml` template
- [ ] Config loader implementation
- [ ] Sensor matching logic
- [ ] CLI parameter for custom config

**Testing**:
- [ ] Test with sample_sensor_kit config
- [ ] Test with custom sensor config
- [ ] Test fallback for missing sensors
- [ ] Verify all parameters applied correctly

---

### 4.3 Vehicle Spawning Logic

**Objective**: Spawn CARLA vehicle with sensors when all prerequisites ready.

**Tasks**:
- [ ] Create `src/autoware_carla_bridge/src/vehicle_lifecycle.rs` module
- [ ] Implement VehicleSpawner
  ```rust
  pub struct VehicleSpawner {
      autoware_detected: bool,
      initial_pose: Option<carla::geom::Transform>,
      sensor_configs: Vec<SensorConfig>,
  }

  impl VehicleSpawner {
      pub fn is_ready(&self) -> bool {
          self.autoware_detected && self.initial_pose.is_some() && !self.sensor_configs.is_empty()
      }
  }
  ```
- [ ] Wait for prerequisites:
  1. Autoware detected (`/robot_description` received)
  2. Initial pose received (`/initialpose` message)
  3. Sensor configs loaded (URDF + TF + config file)
- [ ] Select vehicle blueprint from config
- [ ] Spawn vehicle at initial pose
  ```rust
  let vehicle_bp = carla_world.blueprint_library()
      .find("vehicle.tesla.model3")?;  // From config
  let vehicle = carla_world.spawn_actor(&vehicle_bp, &initial_pose)?;
  log::info!("Spawned vehicle: ID={}", vehicle.id());
  ```
- [ ] Attach sensors with transforms from TF
  ```rust
  for sensor_config in sensor_configs {
      let tf = tf_buffer.lookup_transform("base_link", &sensor_config.frame_id)?;
      let carla_tf = ros_to_carla_transform(&tf);

      let sensor_bp = carla_world.blueprint_library().find(&sensor_config.carla_blueprint_id)?;
      apply_sensor_parameters(&sensor_bp, &sensor_config);

      let sensor = carla_world.spawn_actor_attached_to(&sensor_bp, &vehicle, &carla_tf)?;
      log::info!("Attached sensor: {} at ({:.2}, {:.2}, {:.2})",
          sensor_config.name, carla_tf.location.x, carla_tf.location.y, carla_tf.location.z);
  }
  ```
- [ ] Log spawned vehicle and all sensors

**Deliverables**:
- [ ] `vehicle_lifecycle.rs` module
- [ ] VehicleSpawner with prerequisite tracking
- [ ] Vehicle and sensor spawning logic
- [ ] Detailed logging

**Testing**:
- [ ] Test spawning with all prerequisites
- [ ] Test waiting for each prerequisite
- [ ] Verify vehicle appears at correct pose in CARLA
- [ ] Verify all sensors attached correctly
- [ ] Compare sensor positions with TF tree

---

### 4.4 Vehicle Cleanup on Autoware Loss

**Objective**: Detect Autoware disappearance and clean up CARLA vehicle.

**Tasks**:
- [ ] Implement Autoware health check (periodic `/robot_description` topic check)
- [ ] Detect topic disappearance (no messages for N seconds)
- [ ] Trigger cleanup on detection
  ```rust
  if !autoware_detector.is_alive() {
      log::warn!("Autoware disappeared - cleaning up vehicle");
      destroy_vehicle_and_sensors()?;
      reset_to_waiting_state();
  }
  ```
- [ ] Destroy all sensors using `ActorBase::destroy()` (from Phase 7.1)
- [ ] Destroy vehicle actor
- [ ] Reset bridge state to await new Autoware instance
- [ ] Log cleanup events clearly

**Deliverables**:
- [ ] Autoware health check logic
- [ ] Vehicle cleanup implementation
- [ ] State reset logic

**Testing**:
- [ ] Stop Autoware while bridge running
- [ ] Verify bridge detects loss
- [ ] Verify vehicle and sensors destroyed in CARLA
- [ ] Verify bridge returns to waiting state
- [ ] Restart Autoware, verify bridge detects and respawns

---

### 4.5 Vehicle Teleportation

**Objective**: Update vehicle pose when receiving new `/initialpose` messages.

**Tasks**:
- [ ] Check if vehicle already spawned when receiving `/initialpose`
- [ ] If spawned, use `vehicle.set_transform()` to teleport
  ```rust
  if let Some(ref vehicle) = spawned_vehicle {
      vehicle.set_transform(&new_carla_transform)?;
      log::info!("Teleported vehicle to new pose");
  } else {
      // Store pose for initial spawn
      initial_pose = Some(new_carla_transform);
  }
  ```
- [ ] Handle teleportation vs. initial spawn decision
- [ ] Log teleportation events

**Deliverables**:
- [ ] Teleportation logic
- [ ] Logging for pose updates

**Testing**:
- [ ] Spawn vehicle with initial pose
- [ ] Use RViz to set new pose
- [ ] Verify vehicle teleports in CARLA
- [ ] Test multiple teleportations

---

### Phase 4 Summary

**Deliverables**:
- ✅ `/initialpose` subscription and handling
- ✅ `config/carla_sensor_mappings.yaml` template
- ✅ `vehicle_lifecycle.rs` module
- ✅ Vehicle spawning with sensor attachment
- ✅ Vehicle cleanup on Autoware loss
- ✅ Vehicle teleportation support
- ✅ Map origin configuration (ROS parameter)

**Success Criteria**:
- [ ] Vehicle spawns when prerequisites met
- [ ] Sensors attached at correct TF positions
- [ ] Vehicle cleans up when Autoware stops
- [ ] Teleportation works from RViz
- [ ] State machine handles all lifecycle events

**Risks**:
- Timing issues (spawn before prerequisites)
- CARLA spawn failures
- Sensor attachment errors

**Mitigation**:
- Robust prerequisite checking
- Detailed error messages
- Graceful degradation (skip failed sensors)

---

**Document Version**: 1.0
**Last Updated**: 2025-11-04
**Related Documents**:
- [roadmap.md](../roadmap.md) - Main roadmap index
- [infrastructure.md](infrastructure.md) - Infrastructure setup (Phases 0, 1, 7)
- [bridge.md](bridge.md) - Data bridge implementation (Phases 2, 5, 6, 8)
- [testing-and-release.md](testing-and-release.md) - Testing and release (Phases 9-10)
- [../autoware-integration-design.md](../autoware-integration-design.md) - Detailed Autoware integration design
- [../architecture.md](../architecture.md) - Architecture design and ADRs
