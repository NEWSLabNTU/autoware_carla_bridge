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

| Axis     | ROS (Autoware)      | CARLA (Unreal Engine) |
|----------|---------------------|-----------------------|
| Forward  | +X                  | +X                    |
| Lateral  | +Y (Left)           | +Y (Right)            |
| Up       | +Z                  | +Z                    |
| Rotation | Right-handed        | Left-handed           |
| Units    | meters, radians     | centimeters, degrees  |

**⚠️ CRITICAL**: CARLA uses a **left-handed** coordinate system (inherited from Unreal Engine 4), while ROS/Autoware uses a **right-handed** coordinate system. This requires Y-axis inversion during conversion.

**Coordinate Handedness Difference:**
- **ROS (Right-handed)**: When looking down from +Z, positive rotation is counter-clockwise
- **CARLA (Left-handed)**: When looking down from +Z, positive rotation is clockwise

**Conversion Formula:**

```rust
fn convert_ros_to_carla_transform(ros_tf: &Transform) -> carla::Transform {
    carla::Transform {
        location: carla::Location {
            x: ros_tf.x * 100.0,   // m → cm
            y: -ros_tf.y * 100.0,  // m → cm, LEFT/RIGHT FLIP (right-handed → left-handed)
            z: ros_tf.z * 100.0,   // m → cm
        },
        rotation: carla::Rotation {
            roll: ros_tf.roll.to_degrees(),      // rad → deg
            pitch: -ros_tf.pitch.to_degrees(),   // rad → deg, SIGN FLIP
            yaw: -ros_tf.yaw.to_degrees(),       // rad → deg, SIGN FLIP
        },
    }
}

fn convert_carla_to_ros_transform(carla_tf: &carla::Transform) -> Transform {
    Transform {
        x: carla_tf.location.x / 100.0,   // cm → m
        y: -carla_tf.location.y / 100.0,  // cm → m, LEFT/RIGHT FLIP
        z: carla_tf.location.z / 100.0,   // cm → m
        roll: carla_tf.rotation.roll.to_radians(),     // deg → rad
        pitch: -carla_tf.rotation.pitch.to_radians(),  // deg → rad, SIGN FLIP
        yaw: -carla_tf.rotation.yaw.to_radians(),      // deg → rad, SIGN FLIP
    }
}
```

**Visual Example:**

```
ROS (Right-handed):          CARLA (Left-handed):
      Z↑                           Z↑
       |                            |
       |                            |
       o----→ X                     o----→ X
      /                            /
     / Y (Left)               Y (Right)
    ↙                           ↙
```

### 4.3 Initial Pose Workflow from RViz

**Background**: In Autoware's planning simulator, the initial vehicle pose is set interactively using RViz's "2D Pose Estimate" tool. The bridge should replicate this workflow for CARLA simulation.

**Initial Pose Topic:**

```
Topic: /initialpose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
QoS: Transient Local, Reliable
```

**Message Structure:**

```rust
geometry_msgs::msg::PoseWithCovarianceStamped {
    header: Header {
        stamp: Time,
        frame_id: "map",  // Coordinate frame (typically "map")
    },
    pose: PoseWithCovariance {
        pose: Pose {
            position: Point { x, y, z },      // Position in map frame
            orientation: Quaternion { x, y, z, w },  // Orientation
        },
        covariance: [f64; 36],  // Position and orientation uncertainty
    }
}
```

**RViz Interaction:**

- User clicks "2D Pose Estimate" button (or presses 'P' key) in RViz toolbar
- User clicks on map to set position and drags to set orientation
- RViz publishes pose to `/initialpose` topic

**Bridge Implementation:**

```rust
fn subscribe_to_initial_pose(node: &Arc<Node>, carla_world: Arc<World>) -> Result<()> {
    let subscription = node.create_subscription(
        "/initialpose",
        rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable(),
        move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
            log::info!(
                "Received initial pose: ({:.2}, {:.2}, {:.2})",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            );

            // Convert ROS pose to CARLA transform
            let carla_transform = convert_ros_pose_to_carla_transform(&msg.pose.pose);

            // Spawn or teleport vehicle to new pose
            match spawn_or_teleport_vehicle(&carla_world, &carla_transform) {
                Ok(_) => log::info!("Vehicle spawned/teleported to initial pose"),
                Err(e) => log::error!("Failed to set vehicle pose: {}", e),
            }
        },
    )?;

    Ok(())
}

fn convert_ros_pose_to_carla_transform(pose: &geometry_msgs::msg::Pose) -> carla::Transform {
    // Extract position
    let location = carla::Location {
        x: pose.position.x * 100.0,   // m → cm
        y: -pose.position.y * 100.0,  // m → cm, LEFT/RIGHT FLIP
        z: pose.position.z * 100.0,   // m → cm
    };

    // Convert quaternion to Euler angles (ROS convention)
    let (roll, pitch, yaw) = quaternion_to_euler(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    );

    // Convert to CARLA rotation with sign flips
    let rotation = carla::Rotation {
        roll: roll.to_degrees(),
        pitch: -pitch.to_degrees(),   // SIGN FLIP
        yaw: -yaw.to_degrees(),       // SIGN FLIP
    };

    carla::Transform { location, rotation }
}

fn spawn_or_teleport_vehicle(
    world: &World,
    transform: &carla::Transform,
) -> Result<Vehicle> {
    // Check if vehicle already exists
    if let Some(vehicle) = get_existing_ego_vehicle(world) {
        log::info!("Teleporting existing vehicle to new pose");
        vehicle.set_transform(transform)?;
        Ok(vehicle)
    } else {
        log::info!("Spawning new vehicle at initial pose");
        spawn_vehicle_from_autoware(world, transform)?
    }
}
```

**Startup Sequence Options:**

**Option A - Wait for Initial Pose:**
1. Bridge starts and detects Autoware
2. Waits for `/initialpose` message before spawning vehicle
3. User sets initial pose in RViz → vehicle spawns in CARLA

**Option B - Default Spawn with Update:**
1. Bridge spawns vehicle at CARLA's default spawn point
2. Subscribes to `/initialpose` for updates
3. When user sets pose in RViz → vehicle teleports in CARLA

**Recommendation**: Use **Option B** for better user experience. The bridge can spawn a vehicle immediately and update its pose when the user provides one. This matches Autoware's behavior where the vehicle model is visible in RViz before setting initial pose.

**Coordinate Frame Considerations:**

- `/initialpose` uses the `map` frame (Autoware's global coordinate system)
- CARLA uses its own world coordinate system
- For proper alignment, the bridge needs to know the map origin in CARLA coordinates
- **Simplest approach**: Assume map and CARLA world origins align (both at 0,0,0)
- **Advanced approach**: Add ROS parameter for map origin offset

```yaml
carla_bridge:
  ros__parameters:
    map_origin:
      x: 0.0  # CARLA X coordinate of map origin
      y: 0.0  # CARLA Y coordinate of map origin
      z: 0.0  # CARLA Z coordinate of map origin
      yaw: 0.0  # Rotation offset
```

---

## 5. CARLA Map Export to Autoware

**Challenge**: Autoware requires map data (road network topology and 3D point cloud) to operate. CARLA provides maps in OpenDRIVE format, which must be converted to Autoware-compatible formats.

### 5.1 Map Data Requirements

**Autoware Map Components:**

1. **Lanelet2 Map** (.osm file)
   - Road network topology (lanes, intersections, traffic rules)
   - Format: XML-based Lanelet2 format
   - Loaded by `map_loader` node
   - Published to: `/map/vector_map` and `/lanelet2_map`

2. **Point Cloud Map** (.pcd file)
   - 3D environmental structure for localization
   - Format: PCL Point Cloud Data format
   - Loaded by `map_loader` node
   - Published to: `/map/pointcloud_map`

**CARLA Map Format:**
- OpenDRIVE (.xodr) - Road network definition
- No native point cloud (must be generated from simulation)

### 5.2 Approach 1: Offline Export (File-Based)

**Overview**: Export CARLA map data to files that Autoware can load using standard `map_loader` package.

**Step 1 - Export OpenDRIVE from CARLA:**

```python
# Python script to export CARLA map
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
carla_map = world.get_map()

# Save OpenDRIVE XML
opendrive_xml = carla_map.to_opendrive()
with open('Town01.xodr', 'w') as f:
    f.write(opendrive_xml)
```

**Step 2 - Convert OpenDRIVE to Lanelet2:**

**Tool 1: opendrive2lanelet**
- GitHub: https://github.com/fzi-forschungszentrum-informatik/opendrive2lanelet
- Conversion: .xodr → .osm (Lanelet2)

```bash
# Install
pip install opendrive2lanelet

# Convert
opendrive2lanelet Town01.xodr -o Town01.osm
```

**Tool 2: AssuremappingTools (TIER IV)**
- GitHub: https://github.com/tier4/autoware_tools/tree/main/map/autoware_lanelet2_map_validator
- Better integration with Autoware

**Known Limitations:**
- ⚠️ OpenDRIVE to Lanelet2 conversion is not perfect
- ⚠️ Traffic light positions may be missing or incorrect
- ⚠️ Lane width/boundary interpretation differences
- ⚠️ Requires manual validation and corrections

**Step 3 - Generate Point Cloud Map:**

**Option A - Use CARLA Bridge to Record Point Cloud:**

```rust
// Spawn LiDAR sensor in CARLA
// Drive vehicle around map to collect points
// Save accumulated point cloud to .pcd file

fn record_map_pointcloud(world: &World, duration: Duration) -> Result<()> {
    let lidar_bp = world.blueprint_library()
        .find("sensor.lidar.ray_cast")?;

    // Configure high-density LiDAR for mapping
    lidar_bp.set_attribute("channels", "128");
    lidar_bp.set_attribute("range", "200.0");
    lidar_bp.set_attribute("points_per_second", "2560000");

    let mut accumulated_points = Vec::new();

    // Attach to vehicle and drive around
    let subscription = lidar.listen(move |point_cloud| {
        accumulated_points.extend(point_cloud.points());
    });

    // Wait for recording duration
    std::thread::sleep(duration);

    // Save to PCD file
    save_to_pcd("Town01.pcd", &accumulated_points)?;
    Ok(())
}
```

**Option B - Use Semantic Segmentation:**

```python
# Generate point cloud from semantic camera
# Place cameras at multiple viewpoints
# Combine depth and semantic info to create map
```

**Step 4 - Place Files in Autoware Map Directory:**

```bash
mkdir -p ~/autoware_map/carla_town01/
cp Town01.osm ~/autoware_map/carla_town01/lanelet2_map.osm
cp Town01.pcd ~/autoware_map/carla_town01/pointcloud_map.pcd
```

**Step 5 - Launch Autoware with CARLA Map:**

```bash
cd autoware_repo && \
. install/setup.sh && \
play_launch launch \
  autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/carla_town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit
```

**Pros:**
- ✅ Uses standard Autoware workflow
- ✅ No code changes to bridge
- ✅ Maps can be pre-generated and shared
- ✅ Works offline (no CARLA needed after export)

**Cons:**
- ❌ Manual conversion process
- ❌ Potential conversion errors require manual fixes
- ❌ Point cloud recording requires driving around map
- ❌ Static map (doesn't update if CARLA map changes)

### 5.3 Approach 2: Runtime Topic Publishing

**Overview**: Bridge directly publishes map data to Autoware's map loader topics, bypassing file-based loading.

**Map Loader Topics:**

```
Topic: /map/vector_map
Type: autoware_lanelet2_msgs/msg/MapBin
QoS: Transient Local, Reliable

Topic: /map/pointcloud_map
Type: sensor_msgs/msg/PointCloud2
QoS: Transient Local, Reliable
```

**Implementation Approach:**

```rust
fn publish_carla_map_to_autoware(
    node: &Arc<Node>,
    carla_world: &World,
) -> Result<()> {
    // Get CARLA map
    let carla_map = carla_world.get_map();
    let opendrive_xml = carla_map.to_opendrive();

    // Convert OpenDRIVE to Lanelet2 (in-memory)
    let lanelet2_map = convert_opendrive_to_lanelet2(&opendrive_xml)?;

    // Serialize to MapBin message
    let map_bin_msg = autoware_lanelet2_msgs::msg::MapBin {
        header: create_header("map"),
        version_map_format: "lanelet2_map".to_string(),
        format_version: "1.0".to_string(),
        data: lanelet2_map.serialize()?,
    };

    // Publish vector map (transient local = latching)
    let vector_map_pub = node.create_publisher(
        "/map/vector_map",
        rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable()
    )?;
    vector_map_pub.publish(&map_bin_msg)?;

    // Generate point cloud from CARLA
    let point_cloud = generate_pointcloud_from_carla(&carla_world)?;

    // Publish point cloud map
    let pcd_map_pub = node.create_publisher(
        "/map/pointcloud_map",
        rclrs::QOS_PROFILE_TRANSIENT_LOCAL.reliable()
    )?;
    pcd_map_pub.publish(&point_cloud)?;

    log::info!("Published CARLA map to Autoware");
    Ok(())
}

fn generate_pointcloud_from_carla(world: &World) -> Result<sensor_msgs::msg::PointCloud2> {
    // Approach 1: Use CARLA semantic LiDAR to scan environment
    // Approach 2: Raycasting from multiple viewpoints
    // Approach 3: Extract mesh geometry and convert to points

    // This is complex and may require significant development
    unimplemented!("Point cloud generation from CARLA mesh")
}
```

**Pros:**
- ✅ Fully automated (no manual steps)
- ✅ Dynamic map updates possible
- ✅ No file management
- ✅ Consistent map between CARLA and Autoware

**Cons:**
- ❌ Complex implementation (OpenDRIVE → Lanelet2 conversion in Rust)
- ❌ Point cloud generation non-trivial
- ❌ Requires CARLA running for map loading
- ❌ May have memory overhead for large maps

### 5.4 Recommendation

**Phase 1 (Immediate)**: Use **Approach 1 (Offline Export)** for initial integration
- Focus on getting basic workflow working
- Use manually converted maps
- Document conversion process
- Start with small maps (Town01, Town03)

**Phase 2 (Future Enhancement)**: Add **Approach 2 (Runtime Publishing)** as optional feature
- Implement OpenDRIVE to Lanelet2 converter in Rust
- Research point cloud generation methods
- Provide CLI flag: `--publish-map-topics`

**Example Workflow for Phase 1:**

```bash
# 1. Export CARLA map
python scripts/export_carla_map.py --town Town01

# 2. Convert to Lanelet2 (may require manual edits)
opendrive2lanelet Town01.xodr -o Town01.osm
# Edit Town01.osm if needed

# 3. Generate point cloud (drive in CARLA with recording)
just run -- --map-name Town01 --record-pointcloud --duration 300

# 4. Place in Autoware map directory
make install-map MAP=Town01

# 5. Launch Autoware
cd scripts/autoware && make launch MAP_PATH=$HOME/autoware_map/carla_town01

# 6. Launch bridge
just run -- --map-name Town01 --vehicle-name ego_vehicle
```

**Documentation TODO:**
- Create `docs/map-export-guide.md` with step-by-step instructions
- Add Python scripts to `scripts/map_export/`
- Document known conversion issues and workarounds
- Provide pre-converted maps for common CARLA towns

---

## 6. Custom Sensor Kit Package Design

### 6.1 Why Custom Sensor Kit?

**Problem**: Autoware's default sensor kits may not perfectly match CARLA's sensor capabilities.

**Solution**: Create `carla_sensor_kit` package with CARLA-optimized sensor configurations.

### 6.2 Package Structure

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

### 6.3 Example sensor_kit.xacro for CARLA

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

### 6.4 CARLA Sensor Mappings Config

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

### 6.5 Launch File for CARLA Sensor Kit

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

## 7. Implementation Recommendations

### 7.1 Phase 1: Basic Detection and Parsing

**Deliverables:**
1. ✅ rclrs subscriber for `/robot_description`
2. ✅ URDF parser integration (urdf-rs or roxmltree)
3. ✅ Sensor classifier (link name → sensor type)
4. ✅ Autoware detection logic
5. ✅ Unit tests for URDF parsing

**Code Location:** `src/autoware_carla_bridge/src/autoware_detection.rs`

### 7.2 Phase 2: TF Integration

**Deliverables:**
1. ✅ `/tf_static` subscriber
2. ✅ Local TF buffer implementation
3. ✅ Transform lookup for sensor frames
4. ✅ Coordinate conversion (ROS ↔ CARLA)
5. ✅ Tests with known transforms

**Code Location:** `src/autoware_carla_bridge/src/tf_bridge.rs`

### 7.3 Phase 3: CARLA Spawning

**Deliverables:**
1. ✅ Vehicle spawner from Autoware config
2. ✅ Dynamic sensor attachment
3. ✅ Sensor parameter mapping
4. ✅ carla_sensor_mappings.yaml parser
5. ✅ Integration test with CARLA running

**Code Location:** `src/autoware_carla_bridge/src/carla_spawner.rs`

### 7.4 Phase 4: Custom Sensor Kit

**Deliverables:**
1. ✅ carla_sensor_kit_launch package
2. ✅ URDF/xacro files
3. ✅ Calibration YAML files
4. ✅ carla_sensor_mappings.yaml
5. ✅ Documentation and README
6. ✅ Test with Autoware planning_simulator

**Code Location:** `src/interface/carla_sensor_kit_launch/`

### 7.5 Phase 5: End-to-End Integration

**Deliverables:**
1. ✅ Launch file orchestration
2. ✅ Autoware + CARLA + Bridge workflow
3. ✅ Validation: sensor data flows correctly
4. ✅ Validation: control commands work
5. ✅ Documentation update

**Scripts Location:** `scripts/autoware/`

---

## 8. Alternative Approaches Considered

### 8.1 Manual Configuration (Rejected)

**Approach**: Require users to manually specify sensors in bridge config file.

**Pros:**
- Simple implementation
- Full control over sensor params

**Cons:**
- ❌ Not automatic - defeats 1-to-1 integration goal
- ❌ Error-prone (mismatches between Autoware and CARLA)
- ❌ Maintenance burden (two configs to keep in sync)

**Decision**: Rejected in favor of automatic detection.

### 8.2 Parameter Server Query (Considered)

**Approach**: Query ROS 2 parameter server for sensor parameters.

**Pros:**
- Could extract additional params not in URDF

**Cons:**
- ⚠️ Parameters may not be structured/standardized
- ⚠️ Requires Autoware parameter naming conventions
- ⚠️ Still needs fallback for missing params

**Decision**: Use as supplementary source, not primary. Config file + URDF parsing is more reliable.

### 8.3 CARLA-Specific ROS Parameters (Recommended for Phase 4)

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

## 9. Key Recommendations Summary

### 9.1 Immediate Actions

1. **✅ Start with Phase 1**: Implement /robot_description subscriber and URDF parsing
2. **✅ Create carla_sensor_mappings.yaml**: Define CARLA blueprint mappings for common sensors
3. **✅ Test with sample_sensor_kit**: Validate detection and parsing with Autoware's default kit
4. **✅ Document workflow**: Update README with Autoware integration instructions

### 9.2 Architecture Decisions

| Decision | Rationale |
|----------|-----------|
| Auto-detect via /robot_description | Enables true 1-to-1 integration without manual config |
| Use TF for transforms | Authoritative source, handles dynamic updates |
| Config file for CARLA params | Flexibility for sim-specific parameters |
| Custom sensor kit optional | Start with sample_sensor_kit, add carla_sensor_kit for optimization |
| rclrs native implementation | Stay consistent with existing bridge architecture |

### 9.3 Open Questions for User

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

## 10. Testing Strategy

### 10.1 Unit Tests
- URDF parsing with sample robot_description
- Sensor classification logic
- Transform coordinate conversions
- Config file parsing

### 10.2 Integration Tests
- Autoware detection when planning_simulator running
- Sensor spawning in CARLA
- Data flow: CARLA → Bridge → ROS topics
- Control flow: Autoware → Bridge → CARLA vehicle

### 10.3 End-to-End Validation
- Launch Autoware planning_simulator
- Launch CARLA
- Launch bridge (auto-detects and spawns)
- Verify in RViz: sensor data visualized correctly
- Send goal pose: vehicle navigates in CARLA

---

## 11. Documentation Requirements

### 11.1 User Documentation
- **Quick Start Guide**: Autoware + CARLA + Bridge setup
- **Sensor Kit Guide**: How to create custom kits
- **Troubleshooting**: Common issues and solutions

### 11.2 Developer Documentation
- **Architecture Diagram**: System components and data flow
- **API Reference**: Key functions and data structures
- **Extension Guide**: How to add new sensor types

### 11.3 Configuration Examples
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
