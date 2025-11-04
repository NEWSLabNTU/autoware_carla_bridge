# Autoware Integration Design for CARLA Bridge

**Document Version**: 1.0
**Date**: 2025-11-04
**Status**: Design Proposal

## Executive Summary

This document outlines the recommended approach for integrating the CARLA bridge with Autoware in **1-to-1 mode** (one bridge instance per Autoware vehicle). The design leverages Autoware's native sensor description system (`/robot_description` topic and TF tree) to automatically configure CARLA vehicle sensors.

---

## 1. Architecture Overview

### 1.1 Current State (Verified Runtime Behavior)

**Autoware Publishes:**
- `/robot_description` (std_msgs/msg/String) - Complete URDF with sensor definitions
- `/tf_static` (tf2_msgs/msg/TFMessage) - Static transforms for all sensors
- `/robot_state_publisher` node - Maintains transform tree

**Detected Sensors in sample_sensor_kit:**
- **LiDARs**: 4 units (velodyne_top, velodyne_left, velodyne_right, velodyne_rear)
- **Cameras**: 8 units (camera0-5, traffic_light_left/right)
- **IMU**: 1 unit (tamagawa/imu_link)
- **GNSS**: 1 unit (gnss_link)

**Transform Hierarchy Confirmed:**
```
base_link (vehicle origin)
├── sensor_kit_base_link [x:0.9, y:0.0, z:2.0]
    ├── velodyne_top [x:0.001, y:0.0, z:0.066]
    ├── velodyne_left [x:-0.055, y:0.498, z:-0.31]
    ├── velodyne_right [x:-0.055, y:-0.498, z:-0.31]
    ├── camera0/camera_link [x:0.124, y:0.559, z:-0.278]
    ├── camera1/camera_link [x:0.115, y:0.072, z:-0.279]
    ├── camera2/camera_link [x:0.124, y:-0.562, z:-0.279]
    ├── camera3/camera_link [x:0.037, y:-0.562, z:-0.279]
    ├── camera4/camera_link [x:-0.109, y:-0.017, z:-0.279]
    ├── camera5/camera_link [x:0.037, y:0.556, z:-0.279]
    ├── traffic_light_left_camera/camera_link [x:0.9, y:0.3, z:0.4]
    ├── traffic_light_right_camera/camera_link [x:0.9, y:-0.3, z:0.4]
    ├── gnss_link [x:-0.265, y:0.0, z:0.25]
    └── tamagawa/imu_link [x:-0.28, y:0.0, z:0.0]

└── velodyne_rear_base_link [x:-1.0, y:0.0, z:1.5]
    └── velodyne_rear [x:0.0, y:0.0, z:0.0]
```

### 1.2 Proposed Integration Flow

```
┌─────────────────────┐
│   Autoware Stack    │
│  (Planning Sim)     │
│                     │
│  • robot_state_pub  │──┐
│  • /robot_desc      │  │
│  • /tf_static       │  │
└─────────────────────┘  │
                         │ Reads sensor config
                         ▼
┌─────────────────────────────────────┐
│    CARLA Bridge (1-to-1 mode)       │
│                                     │
│  1. Detect Autoware instance        │
│  2. Parse /robot_description        │
│  3. Query TF for sensor poses       │
│  4. Spawn vehicle in CARLA          │
│  5. Attach sensors with transforms  │
│  6. Publish sensor data to ROS      │
│  7. Subscribe to control commands   │
└─────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────┐
│   CARLA Simulator   │
│                     │
│  • Spawned vehicle  │
│  • Attached sensors │
│  • Physics sim      │
└─────────────────────┘
```

---

## 2. Detection Strategy

### 2.1 How Bridge Detects Autoware Instance

**Primary Detection Method:**

```rust
// 1. Check for /robot_description topic
let topics = node.get_topic_names_and_types()?;
let has_robot_desc = topics.iter().any(|(name, _)| name == "/robot_description");

// 2. Verify robot_state_publisher node exists
let nodes = node.get_node_names()?;
let has_rsp = nodes.iter().any(|name| name.contains("robot_state_publisher"));

// 3. Wait for initial /robot_description message
let robot_desc_sub = node.create_subscription(
    "/robot_description",
    |msg: std_msgs::msg::String| {
        log::info!("Detected Autoware instance - robot_description received");
        parse_and_spawn_vehicle(&msg.data);
    }
)?;
```

**Detection Criteria (all must be true):**
- ✅ `/robot_description` topic exists
- ✅ `/robot_state_publisher` node is running
- ✅ `/tf_static` topic has sensor transforms
- ✅ Standard Autoware topics present (`/control/command/control_cmd`, `/vehicle/status/*`)

**Startup Sequence:**

1. Bridge launches and waits for Autoware
2. Polls for `/robot_description` topic (timeout: 30s)
3. Once detected, subscribes and waits for first message
4. Parses URDF to extract sensor definitions
5. Queries TF tree for exact transforms
6. Spawns vehicle and sensors in CARLA
7. Begins normal operation (sensor publishing, control subscription)

---

## 3. Sensor Configuration Extraction

### 3.1 Parsing /robot_description

**URDF Parsing Approach:**

Use `urdf-rs` crate (or `roxmltree` for lightweight XML parsing):

```rust
use urdf_rs::Robot;

fn parse_robot_description(urdf_xml: &str) -> Result<Robot> {
    urdf_rs::read_from_string(urdf_xml)
}

fn extract_sensors(robot: &Robot) -> Vec<SensorConfig> {
    let mut sensors = Vec::new();

    // Find all links with sensor-related names
    for link in &robot.links {
        if let Some(sensor_info) = classify_sensor(&link.name) {
            sensors.push(SensorConfig {
                name: link.name.clone(),
                sensor_type: sensor_info.sensor_type,
                frame_id: link.name.clone(),
            });
        }
    }

    sensors
}

fn classify_sensor(link_name: &str) -> Option<SensorInfo> {
    if link_name.contains("velodyne") || link_name.contains("lidar") {
        Some(SensorInfo { sensor_type: SensorType::Lidar })
    } else if link_name.contains("camera") && !link_name.contains("optical") {
        Some(SensorInfo { sensor_type: SensorType::Camera })
    } else if link_name.contains("imu") {
        Some(SensorInfo { sensor_type: SensorType::Imu })
    } else if link_name.contains("gnss") {
        Some(SensorInfo { sensor_type: SensorType::Gnss })
    } else {
        None
    }
}
```

### 3.2 Querying TF for Exact Transforms

**TF Buffer Integration:**

```rust
use tf2_ros::Buffer;

fn get_sensor_transform(
    tf_buffer: &Buffer,
    sensor_frame: &str,
) -> Result<Transform> {
    // Query static transform from base_link to sensor
    let transform = tf_buffer.lookup_transform(
        "base_link",
        sensor_frame,
        Time::from_seconds(0.0), // Use latest available
    )?;

    Ok(Transform {
        x: transform.transform.translation.x,
        y: transform.transform.translation.y,
        z: transform.transform.translation.z,
        roll: transform.rotation.roll(),
        pitch: transform.rotation.pitch(),
        yaw: transform.rotation.yaw(),
    })
}
```

**Note**: rclrs may not have tf2_ros bindings yet. Alternative: Use `tf2_msgs/msg/TFMessage` subscription and build local transform buffer.

### 3.3 Sensor Parameter Extraction

**Challenge**: URDF doesn't embed all sensor parameters (resolution, FOV, etc.)

**Solution 1 - Convention-Based Defaults:**

```rust
fn get_sensor_params(sensor_type: SensorType, sensor_name: &str) -> SensorParams {
    match sensor_type {
        SensorType::Lidar => {
            if sensor_name.contains("VLS-128") || sensor_name.contains("top") {
                SensorParams::Lidar {
                    channels: 128,
                    range: 130.0,
                    rotation_frequency: 10.0,
                    points_per_second: 1280000,
                }
            } else if sensor_name.contains("VLP-16") {
                SensorParams::Lidar {
                    channels: 16,
                    range: 130.0,
                    rotation_frequency: 10.0,
                    points_per_second: 300000,
                }
            } else {
                SensorParams::default_lidar()
            }
        }
        SensorType::Camera => {
            // sample_sensor_kit uses 800x400, 30fps, 74.5° FOV
            SensorParams::Camera {
                width: 800,
                height: 400,
                fov: 74.5,
                fps: 30,
            }
        }
        SensorType::Imu => SensorParams::Imu { update_rate: 100.0 },
        SensorType::Gnss => SensorParams::Gnss {},
    }
}
```

**Solution 2 - Configuration File:**

Create `carla_sensor_mappings.yaml` in bridge config:

```yaml
# Maps Autoware sensor names to CARLA parameters
sensors:
  velodyne_top:
    carla_blueprint: sensor.lidar.ray_cast
    params:
      channels: 128
      range: 130.0
      rotation_frequency: 10.0
      points_per_second: 1280000

  velodyne_left:
    carla_blueprint: sensor.lidar.ray_cast
    params:
      channels: 16
      range: 130.0
      rotation_frequency: 10.0
      points_per_second: 300000

  camera0:
    carla_blueprint: sensor.camera.rgb
    params:
      image_size_x: 800
      image_size_y: 400
      fov: 74.5

  tamagawa/imu_link:
    carla_blueprint: sensor.other.imu
    params: {}

  gnss_link:
    carla_blueprint: sensor.other.gnss
    params: {}
```

**Recommendation**: Use Solution 2 (config file) for flexibility. Fall back to Solution 1 for unknown sensors.

---

## 4. Vehicle Spawning Workflow

### 4.1 Complete Spawn Sequence

```rust
async fn spawn_vehicle_from_autoware(
    robot_desc: &str,
    carla_world: &World,
) -> Result<Vehicle> {
    // 1. Parse URDF
    let robot = parse_robot_description(robot_desc)?;
    let sensors = extract_sensors(&robot);
    log::info!("Detected {} sensors in Autoware config", sensors.len());

    // 2. Get vehicle spawn point (from map or default)
    let spawn_point = carla_world.get_map().get_spawn_points()[0];

    // 3. Select vehicle blueprint (configurable)
    let vehicle_bp = carla_world
        .blueprint_library()
        .filter("vehicle.tesla.model3") // Or from config
        .unwrap();

    // 4. Spawn vehicle
    let vehicle = carla_world.spawn_actor(&vehicle_bp, &spawn_point)?;
    log::info!("Spawned vehicle: ID={}", vehicle.id());

    // 5. Attach sensors
    for sensor_config in sensors {
        let transform = get_sensor_transform_from_tf(&sensor_config.frame_id)?;
        let sensor_params = load_sensor_params(&sensor_config)?;

        attach_sensor(
            &vehicle,
            &sensor_config,
            &transform,
            &sensor_params,
            carla_world,
        )?;
    }

    Ok(vehicle)
}

fn attach_sensor(
    vehicle: &Vehicle,
    config: &SensorConfig,
    transform: &Transform,
    params: &SensorParams,
    world: &World,
) -> Result<Sensor> {
    // Get CARLA blueprint
    let sensor_bp = world
        .blueprint_library()
        .find(&config.carla_blueprint_id)?;

    // Apply parameters
    sensor_bp.set_attribute("channels", &params.channels.to_string());
    sensor_bp.set_attribute("range", &params.range.to_string());
    // ... other params

    // Convert transform (Autoware ROS convention → CARLA convention)
    let carla_transform = convert_ros_to_carla_transform(transform);

    // Spawn sensor attached to vehicle
    let sensor = world.spawn_actor_attached_to(
        &sensor_bp,
        vehicle,
        &carla_transform,
    )?;

    log::info!("Attached sensor: {} (type: {:?})", config.name, config.sensor_type);
    Ok(sensor)
}
```

### 4.2 Transform Coordinate Conversion

**Autoware (ROS) vs CARLA Conventions:**

| Axis | ROS (Autoware) | CARLA |
|------|----------------|-------|
| Forward | +X | +X |
| Left | +Y | +Y |
| Up | +Z | +Z |
| Rotation | Right-handed | Right-handed |

**Note**: Both use same coordinate system! But CARLA uses Unreal Engine units (cm), ROS uses meters.

```rust
fn convert_ros_to_carla_transform(ros_tf: &Transform) -> carla::Transform {
    carla::Transform {
        location: carla::Location {
            x: ros_tf.x * 100.0,  // m → cm
            y: ros_tf.y * 100.0,
            z: ros_tf.z * 100.0,
        },
        rotation: carla::Rotation {
            roll: ros_tf.roll.to_degrees(),    // rad → deg
            pitch: ros_tf.pitch.to_degrees(),
            yaw: ros_tf.yaw.to_degrees(),
        },
    }
}
```

---

## 5. Custom Sensor Kit Package Design

### 5.1 Why Custom Sensor Kit?

**Problem**: Autoware's default sensor kits may not perfectly match CARLA's sensor capabilities.

**Solution**: Create `carla_sensor_kit` package with CARLA-optimized sensor configurations.

### 5.2 Package Structure

```
src/interface/carla_sensor_kit_launch/
├── carla_sensor_kit_description/
│   ├── urdf/
│   │   ├── sensor_kit.xacro          # Main sensor definitions
│   │   └── sensors.xacro              # Top-level include
│   ├── config/
│   │   ├── sensor_kit_calibration.yaml
│   │   └── sensors_calibration.yaml
│   └── meshes/                        # Optional visual meshes
├── carla_sensor_kit_launch/
│   └── launch/
│       └── sensing.launch.xml         # Sensor driver launch (if needed)
├── config/
│   └── carla_sensor_mappings.yaml    # CARLA-specific params
└── package.xml
```

### 5.3 Example sensor_kit.xacro for CARLA

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="sensor_kit_macro" params="parent x y z roll pitch yaw">

    <!-- Sensor Kit Base -->
    <link name="sensor_kit_base_link"/>
    <joint name="sensor_kit_base_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="sensor_kit_base_link"/>
      <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
    </joint>

    <!-- LiDAR: VLS-128 (Top) -->
    <xacro:VLS-128
      parent="sensor_kit_base_link"
      name="velodyne_top"
      x="0.0" y="0.0" z="0.066"
      roll="0.0" pitch="0.0" yaw="1.575"/>

    <!-- LiDAR: VLP-16 (Left) -->
    <xacro:VLP-16
      parent="sensor_kit_base_link"
      name="velodyne_left"
      x="-0.055" y="0.498" z="-0.31"
      roll="0.0" pitch="0.0" yaw="1.57"/>

    <!-- LiDAR: VLP-16 (Right) -->
    <xacro:VLP-16
      parent="sensor_kit_base_link"
      name="velodyne_right"
      x="-0.055" y="-0.498" z="-0.31"
      roll="0.0" pitch="0.0" yaw="-1.57"/>

    <!-- Camera: Front Center (camera0) -->
    <xacro:monocular_camera_macro
      parent="sensor_kit_base_link"
      name="camera0"
      x="0.124" y="0.559" z="-0.278"
      roll="-0.025" pitch="0.315" yaw="1.035"
      fps="30"
      width="800"
      height="400"
      fov="1.3"/>

    <!-- IMU -->
    <xacro:imu_macro
      parent="sensor_kit_base_link"
      name="tamagawa/imu"
      x="-0.28" y="0.0" z="0.0"
      roll="0.0" pitch="0.0" yaw="0.0"/>

    <!-- GNSS -->
    <link name="gnss_link"/>
    <joint name="gnss_joint" type="fixed">
      <parent link="sensor_kit_base_link"/>
      <child link="gnss_link"/>
      <origin xyz="-0.265 0.0 0.25" rpy="0 0 0"/>
    </joint>

  </xacro:macro>
</robot>
```

### 5.4 CARLA Sensor Mappings Config

```yaml
# carla_sensor_mappings.yaml
# Maps sensor kit components to CARLA blueprints and parameters

vehicle:
  blueprint: vehicle.tesla.model3  # Default CARLA vehicle
  role_name: ego_vehicle

sensors:
  velodyne_top:
    carla_blueprint: sensor.lidar.ray_cast
    description: "VLS-128 128-beam LiDAR"
    parameters:
      channels: '128'
      range: '130.0'
      points_per_second: '1280000'
      rotation_frequency: '10.0'
      upper_fov: '10.0'
      lower_fov: '-10.0'
      horizontal_fov: '360.0'
      atmosphere_attenuation_rate: '0.004'
      dropoff_general_rate: '0.45'
      dropoff_intensity_limit: '0.8'
      dropoff_zero_intensity: '0.4'
    topic: /sensing/lidar/top/pointcloud

  velodyne_left:
    carla_blueprint: sensor.lidar.ray_cast
    description: "VLP-16 16-beam LiDAR (Left)"
    parameters:
      channels: '16'
      range: '130.0'
      points_per_second: '300000'
      rotation_frequency: '10.0'
      upper_fov: '15.0'
      lower_fov: '-15.0'
      horizontal_fov: '360.0'
    topic: /sensing/lidar/left/pointcloud

  velodyne_right:
    carla_blueprint: sensor.lidar.ray_cast
    description: "VLP-16 16-beam LiDAR (Right)"
    parameters:
      channels: '16'
      range: '130.0'
      points_per_second: '300000'
      rotation_frequency: '10.0'
      upper_fov: '15.0'
      lower_fov: '-15.0'
      horizontal_fov: '360.0'
    topic: /sensing/lidar/right/pointcloud

  camera0:
    carla_blueprint: sensor.camera.rgb
    description: "Front center RGB camera"
    parameters:
      image_size_x: '800'
      image_size_y: '400'
      fov: '74.5'
      sensor_tick: '0.033'  # 30 FPS
    topic: /sensing/camera/camera0/image_raw

  camera1:
    carla_blueprint: sensor.camera.rgb
    description: "Front-right RGB camera"
    parameters:
      image_size_x: '800'
      image_size_y: '400'
      fov: '74.5'
      sensor_tick: '0.033'
    topic: /sensing/camera/camera1/image_raw

  tamagawa/imu_link:
    carla_blueprint: sensor.other.imu
    description: "IMU sensor"
    parameters: {}
    topic: /sensing/imu/imu_data

  gnss_link:
    carla_blueprint: sensor.other.gnss
    description: "GNSS sensor"
    parameters: {}
    topic: /sensing/gnss/pose
```

### 5.5 Launch File for CARLA Sensor Kit

```xml
<!-- carla_sensor_kit.launch.xml -->
<launch>
  <arg name="vehicle_id" default="default"/>

  <!-- Include in Autoware launch with sensor_model:=carla_sensor_kit -->
  <let name="sensor_model" value="carla_sensor_kit"/>
  <let name="config_dir" value="$(find-pkg-share carla_sensor_kit_launch)/config"/>

  <!-- Note: Actual sensor drivers not needed - CARLA bridge handles data -->
  <!-- This package just provides URDF/calibration for robot_state_publisher -->
</launch>
```

---

## 6. Implementation Recommendations

### 6.1 Phase 1: Basic Detection and Parsing

**Deliverables:**
1. ✅ rclrs subscriber for `/robot_description`
2. ✅ URDF parser integration (urdf-rs or roxmltree)
3. ✅ Sensor classifier (link name → sensor type)
4. ✅ Autoware detection logic
5. ✅ Unit tests for URDF parsing

**Code Location:** `src/autoware_carla_bridge/src/autoware_detection.rs`

### 6.2 Phase 2: TF Integration

**Deliverables:**
1. ✅ `/tf_static` subscriber
2. ✅ Local TF buffer implementation
3. ✅ Transform lookup for sensor frames
4. ✅ Coordinate conversion (ROS ↔ CARLA)
5. ✅ Tests with known transforms

**Code Location:** `src/autoware_carla_bridge/src/tf_bridge.rs`

### 6.3 Phase 3: CARLA Spawning

**Deliverables:**
1. ✅ Vehicle spawner from Autoware config
2. ✅ Dynamic sensor attachment
3. ✅ Sensor parameter mapping
4. ✅ carla_sensor_mappings.yaml parser
5. ✅ Integration test with CARLA running

**Code Location:** `src/autoware_carla_bridge/src/carla_spawner.rs`

### 6.4 Phase 4: Custom Sensor Kit

**Deliverables:**
1. ✅ carla_sensor_kit_launch package
2. ✅ URDF/xacro files
3. ✅ Calibration YAML files
4. ✅ carla_sensor_mappings.yaml
5. ✅ Documentation and README
6. ✅ Test with Autoware planning_simulator

**Code Location:** `src/interface/carla_sensor_kit_launch/`

### 6.5 Phase 5: End-to-End Integration

**Deliverables:**
1. ✅ Launch file orchestration
2. ✅ Autoware + CARLA + Bridge workflow
3. ✅ Validation: sensor data flows correctly
4. ✅ Validation: control commands work
5. ✅ Documentation update

**Scripts Location:** `scripts/autoware/`

---

## 7. Alternative Approaches Considered

### 7.1 Manual Configuration (Rejected)

**Approach**: Require users to manually specify sensors in bridge config file.

**Pros:**
- Simple implementation
- Full control over sensor params

**Cons:**
- ❌ Not automatic - defeats 1-to-1 integration goal
- ❌ Error-prone (mismatches between Autoware and CARLA)
- ❌ Maintenance burden (two configs to keep in sync)

**Decision**: Rejected in favor of automatic detection.

### 7.2 Parameter Server Query (Considered)

**Approach**: Query ROS 2 parameter server for sensor parameters.

**Pros:**
- Could extract additional params not in URDF

**Cons:**
- ⚠️ Parameters may not be structured/standardized
- ⚠️ Requires Autoware parameter naming conventions
- ⚠️ Still needs fallback for missing params

**Decision**: Use as supplementary source, not primary. Config file + URDF parsing is more reliable.

### 7.3 CARLA-Specific ROS Parameters (Recommended for Phase 4)

**Approach**: Add ROS parameters to bridge for CARLA-specific overrides.

```yaml
carla_bridge:
  ros__parameters:
    sensor_overrides:
      camera0:
        fov: 90.0  # Override Autoware's 74.5° with wider FOV
        lens_distortion_k1: -0.1
      velodyne_top:
        noise_stddev: 0.01
```

**Pros:**
- ✅ Allows runtime tuning
- ✅ Doesn't require modifying Autoware configs
- ✅ Can add CARLA-specific parameters (noise, weather effects)

**Decision**: Implement in Phase 5 as optional enhancement.

---

## 8. Key Recommendations Summary

### 8.1 Immediate Actions

1. **✅ Start with Phase 1**: Implement /robot_description subscriber and URDF parsing
2. **✅ Create carla_sensor_mappings.yaml**: Define CARLA blueprint mappings for common sensors
3. **✅ Test with sample_sensor_kit**: Validate detection and parsing with Autoware's default kit
4. **✅ Document workflow**: Update README with Autoware integration instructions

### 8.2 Architecture Decisions

| Decision | Rationale |
|----------|-----------|
| Auto-detect via /robot_description | Enables true 1-to-1 integration without manual config |
| Use TF for transforms | Authoritative source, handles dynamic updates |
| Config file for CARLA params | Flexibility for sim-specific parameters |
| Custom sensor kit optional | Start with sample_sensor_kit, add carla_sensor_kit for optimization |
| rclrs native implementation | Stay consistent with existing bridge architecture |

### 8.3 Open Questions for User

1. **Vehicle Selection**: Should bridge auto-select CARLA vehicle blueprint, or make it configurable via ROS param?
   - **Recommendation**: Configurable via `carla_sensor_mappings.yaml` → `vehicle.blueprint`

2. **Spawn Location**: How to determine vehicle spawn point?
   - **Option A**: Use first spawn point from CARLA map
   - **Option B**: ROS parameter for spawn pose
   - **Option C**: Subscribe to `/initialpose` topic (RViz 2D Pose Estimate)
   - **Recommendation**: Option B with Option C as enhancement

3. **Multiple Autoware Instances**: How to handle discovery when multiple Autoware instances run?
   - **Approach**: Use ROS_DOMAIN_ID isolation + namespace matching
   - **Recommendation**: Document as Phase 6 feature, focus on single instance first

4. **Sensor Data Latency**: Should bridge match Autoware's sensor timing or use CARLA's tick rate?
   - **Recommendation**: Use CARLA's simulation tick for sensors, timestamps from CARLA simulation time

5. **Error Handling**: What if CARLA doesn't support a sensor defined in Autoware?
   - **Recommendation**: Log warning, skip sensor, continue with available sensors

---

## 9. Testing Strategy

### 9.1 Unit Tests
- URDF parsing with sample robot_description
- Sensor classification logic
- Transform coordinate conversions
- Config file parsing

### 9.2 Integration Tests
- Autoware detection when planning_simulator running
- Sensor spawning in CARLA
- Data flow: CARLA → Bridge → ROS topics
- Control flow: Autoware → Bridge → CARLA vehicle

### 9.3 End-to-End Validation
- Launch Autoware planning_simulator
- Launch CARLA
- Launch bridge (auto-detects and spawns)
- Verify in RViz: sensor data visualized correctly
- Send goal pose: vehicle navigates in CARLA

---

## 10. Documentation Requirements

### 10.1 User Documentation
- **Quick Start Guide**: Autoware + CARLA + Bridge setup
- **Sensor Kit Guide**: How to create custom kits
- **Troubleshooting**: Common issues and solutions

### 10.2 Developer Documentation
- **Architecture Diagram**: System components and data flow
- **API Reference**: Key functions and data structures
- **Extension Guide**: How to add new sensor types

### 10.3 Configuration Examples
- **Example 1**: Using sample_sensor_kit
- **Example 2**: Custom sensor configuration
- **Example 3**: Multi-vehicle with domain isolation

---

## Appendix A: Verified Runtime Data

**Test Environment:**
- Autoware Version: 2025.02
- Map: sample-map-planning
- Vehicle: sample_vehicle
- Sensor Kit: sample_sensor_kit

**Observed Topics (495 total):**
- `/robot_description`: ✅ Publishing URDF
- `/tf_static`: ✅ Contains 30+ sensor frames
- `/control/command/control_cmd`: ✅ Type: autoware_control_msgs/msg/Control
- `/vehicle/status/velocity_status`: ✅ Type: autoware_vehicle_msgs/msg/VelocityReport
- Sensor data topics: `/sensing/camera/camera{0-5,6,7}/...`, `/sensing/imu/imu_data`

**Transform Samples:**
```
base_link → sensor_kit_base_link:
  Translation: [0.900, 0.000, 2.000] m
  Rotation: [-0.001, 0.015, -0.036] rad

base_link → camera0/camera_link:
  Translation: [1.024, 0.559, 1.721] m
  Rotation: [-0.012, 0.323, 1.003] rad

base_link → velodyne_top:
  Translation: [0.901, 0.000, 2.066] m
  Rotation: [0.015, 0.001, 1.539] rad
```

---

**End of Document**
