# TUMFTM Carla-Autoware-Bridge Analysis

**Source**: https://github.com/TUMFTM/Carla-Autoware-Bridge
**Downloaded**: 2025-11-08
**Location**: `src/external/tumftm_carla_autoware_bridge/`

## Overview

The TUMFTM Carla-Autoware-Bridge is a Python-based bridge connecting CARLA simulator to Autoware Core/Universe. It's built on top of the existing CARLA-ROS-Bridge and acts as an adapter layer between CARLA topics and Autoware topics.

## Architecture

### Component Stack

```
┌─────────────────────────────────────────┐
│           Autoware                       │
└─────────────┬───────────────────────────┘
              │ Autoware topics
              │ (/vehicle/status/*, /control/command/*, etc.)
┌─────────────┴───────────────────────────┐
│  carla_autoware_bridge (Python)          │  ← TUMFTM's code (thin adapter)
│  - aw_bridge.py                          │
│  - Message converters                    │
└─────────────┬───────────────────────────┘
              │ CARLA topics
              │ (/carla/ego_vehicle/*)
┌─────────────┴───────────────────────────┐
│  carla_ros_bridge (Python)               │  ← External dependency
│  - CARLA Python API wrapper              │
│  - Sensor data publishing                │
│  - Vehicle control subscription          │
└─────────────┬───────────────────────────┘
              │ CARLA Python API
┌─────────────┴───────────────────────────┐
│       CARLA Simulator                    │
└─────────────────────────────────────────┘
```

### Our Architecture (autoware_carla_bridge)

```
┌─────────────────────────────────────────┐
│           Autoware                       │
└─────────────┬───────────────────────────┘
              │ Autoware topics
┌─────────────┴───────────────────────────┐
│  autoware_carla_bridge (Rust)            │  ← Our code (direct bridge)
│  - Sensor bridges                        │
│  - Vehicle bridge                        │
│  - TF bridge                             │
│  - Coordinate conversion                 │
└─────────────┬───────────────────────────┘
              │ carla-rust crate
┌─────────────┴───────────────────────────┐
│       CARLA Simulator                    │
└─────────────────────────────────────────┘
```

## Key Differences

| Aspect              | TUMFTM Approach                                        | Our Approach                   |
|---------------------|--------------------------------------------------------|--------------------------------|
| **Language**        | Python                                                 | Rust                           |
| **CARLA Interface** | Indirect (via carla_ros_bridge)                        | Direct (via carla-rust)        |
| **Architecture**    | Adapter/converter layer                                | Full bridge implementation     |
| **Dependencies**    | High (carla_ros_bridge, carla_ackermann_control, etc.) | Low (rclrs, carla-rust)        |
| **Message Flow**    | CARLA → Python bridge → Adapter → Autoware             | CARLA → Rust bridge → Autoware |
| **Sensor Config**   | JSON file (objects.json)                               | URDF parsing from Autoware     |
| **Performance**     | Multiple Python processes                              | Single Rust process            |
| **Type Safety**     | Runtime (Python)                                       | Compile-time (Rust)            |
| **Memory**          | Higher (Python overhead)                               | Lower (Rust efficiency)        |

## TUMFTM Components

### 1. aw_bridge.py

**Purpose**: Thin adapter converting between CARLA-ROS-Bridge topics and Autoware topics

**Message conversions**:

```python
# Input: CARLA-ROS-Bridge topics
/carla/ego_vehicle/vehicle_status    (CarlaEgoVehicleStatus)
/carla/ego_vehicle/odometry          (Odometry)
/carla/ego_vehicle/ackermann_cmd     (AckermannDrive) - OUTPUT

# Output: Autoware topics
/vehicle/status/steering_status      (SteeringReport)
/vehicle/status/velocity_status      (VelocityReport)
/vehicle/status/control_mode         (ControlModeReport)
/sensing/gnss/pose_with_covariance   (PoseWithCovarianceStamped)
/control/command/control_cmd         (AckermannControlCommand) - INPUT
```

**Key implementation details**:

1. **Velocity Status Conversion**:
   - Extracts velocity from odometry
   - Publishes to `/vehicle/status/velocity_status`

2. **Pose with Covariance**:
   - Converts odometry to pose_with_covariance
   - **Hardcoded covariance matrix**:
     ```python
     covariance = [
         0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0
     ]
     ```
   - Frame: `map`
   - Published to `/sensing/gnss/pose_with_covariance`

3. **Control Mode**:
   - **Always autonomous mode** (`mode = 1`)
   - No manual/auto switching

4. **Steering Conversion**:
   - Uses `SteeringStatusConverter` class
   - Publishes to `/vehicle/status/steering_status`

5. **Ackermann Control**:
   - Receives Autoware's `/control/command/control_cmd`
   - Converts to CARLA's `/carla/ego_vehicle/ackermann_cmd`
   - **Applies 1.2x multiplier to steering angles**:
     ```python
     carla_ackermann.steering_angle = aw_cmd.lateral.steering_tire_angle * 1.2
     carla_ackermann.steering_angle_velocity = aw_cmd.lateral.steering_tire_rotation_rate * 1.2
     ```
   - Forwards speed, acceleration, jerk directly

### 2. objects.json

**Purpose**: Sensor and vehicle configuration

**Structure**:
```json
{
  "objects": [
    {
      "type": "vehicle.volkswagen.t2_2021",
      "id": "ego_vehicle",
      "sensors": [
        {
          "type": "sensor.camera.rgb",
          "id": "sensing/camera/traffic_light",
          "frame_id": "camera_front_link",
          "spawn_point": {"x": 1.80, "y": 0.25, "z": 1.65, ...},
          "image_size_x": 800,
          "image_size_y": 600,
          "fov": 90.0
        },
        // ... more sensors
      ]
    }
  ]
}
```

**Sensor types defined**:
- `sensor.camera.rgb` - RGB camera
- `sensor.lidar.ray_cast` - LiDAR (front and rear)
- `sensor.other.gnss` - GNSS
- `sensor.other.imu` - IMU
- `sensor.other.collision` - Collision detector
- `sensor.other.lane_invasion` - Lane invasion detector
- `sensor.pseudo.tf` - TF publisher (from carla_ros_bridge)
- `sensor.pseudo.objects` - Object detector (from carla_ros_bridge)
- `sensor.pseudo.odom` - Odometry (from carla_ros_bridge)
- `actor.pseudo.control` - Control interface (from carla_ros_bridge)

**Comparison with our approach**:
- **TUMFTM**: Static JSON configuration, manually defined
- **Our bridge**: Dynamic URDF parsing from Autoware's `/robot_description`
- **Advantage of JSON**: Explicit sensor parameters (fov, points_per_second, etc.)
- **Advantage of URDF**: Automatically matches Autoware's sensor configuration

### 3. Launch Files

**carla_aw_bridge.launch.py** launches:

1. **carla_ros_bridge** (external)
   - Parameters: host, port, town, synchronous_mode, fixed_delta_seconds
   - Spawns sensors defined in objects.json
   - Publishes sensor data to `/carla/ego_vehicle/*` topics

2. **carla_autoware_bridge** (TUMFTM's code)
   - Runs aw_bridge.py
   - Converts topics for Autoware

3. **spawn_ego_vehicle** (external)
   - Spawns vehicle in CARLA

4. **carla_ackermann_control** (external)
   - Ackermann controller for vehicle

5. **carla_manual_control** (optional)
   - Manual control window

**Total process count**: 5+ Python processes

### 4. Traffic Generation

**generate_traffic.py** - Standard CARLA traffic spawner

**Usage**:
```bash
python3 generate_traffic.py -p 2000 -n 30 -w 10
```

**Features**:
- Spawns vehicles and walkers
- Configurable number and types
- Uses CARLA's Traffic Manager

**Integration potential**: We could adapt this to Rust for our bridge.

## What We Can Learn

### 1. Message Conversions

**Useful patterns from TUMFTM**:

1. **Pose with Covariance**:
   - They publish directly to `/sensing/gnss/pose_with_covariance`
   - We currently publish odometry, but Autoware might prefer pose_with_covariance
   - **Action**: Consider adding pose_with_covariance publisher

2. **Control Mode Report**:
   - Always set to autonomous mode
   - **Action**: We should add this publisher (currently missing)

3. **Steering Report**:
   - Convert vehicle status to steering report
   - **Action**: We need to implement this

4. **1.2x Steering Multiplier**:
   - They multiply steering angles by 1.2
   - Might be vehicle-specific calibration
   - **Action**: Investigate if we need similar calibration

### 2. Sensor Configuration

**JSON vs URDF**:

| Approach          | Pros                                                                                 | Cons                                                                               |
|-------------------|--------------------------------------------------------------------------------------|------------------------------------------------------------------------------------|
| **JSON** (TUMFTM) | - Explicit sensor parameters<br>- Easy to modify<br>- Supports CARLA-specific params | - Duplicate configuration<br>- Manual sync with Autoware<br>- Not vehicle-agnostic |
| **URDF** (Ours)   | - Single source of truth<br>- Auto-sync with Autoware<br>- Vehicle-agnostic          | - CARLA params not in URDF<br>- Less explicit                                      |

**Potential hybrid approach**:
- Parse URDF for sensor poses and types (our current approach)
- Optionally load CARLA-specific parameters from JSON/YAML
- Example: LiDAR `points_per_second`, camera `fov`, etc.

### 3. Autoware Integration

**Topics they publish** (that we might be missing):

1. `/vehicle/status/control_mode` - Control mode report ✗ **Missing in our bridge**
2. `/vehicle/status/steering_status` - Steering status ✗ **Missing in our bridge**
3. `/sensing/gnss/pose_with_covariance` - Pose with covariance ✗ **We publish odometry instead**

**Topics they subscribe**:

1. `/control/command/control_cmd` - Ackermann control ✗ **Missing in our bridge**

**Action items**:
- [ ] Add control_mode publisher
- [ ] Add steering_status publisher
- [ ] Add pose_with_covariance publisher (or check if odometry is sufficient)
- [ ] Add control_cmd subscriber and vehicle actuation

### 4. Performance Considerations

**TUMFTM limitations** (from their README):

1. **"We aim to enhance future efficiency by ensuring that the bridge is Python-free, utilizing native DDS connection"**
   - They acknowledge Python performance limitations
   - Our Rust approach already achieves this goal

2. **Multiple processes**:
   - carla_ros_bridge (main CARLA interface)
   - carla_autoware_bridge (adapter)
   - carla_ackermann_control (controller)
   - spawn_ego_vehicle (spawner)
   - Total: 4-5 Python processes
   - Our bridge: 1 Rust process

3. **Memory overhead**:
   - Python processes + CARLA Python API overhead
   - Our Rust approach has minimal overhead

## Integration Opportunities

### 1. Pre-Converted Maps

**TUMFTM provides**:
- Converted Lanelet2 maps for CARLA towns
- Hosted at: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/

**Action**: Download and test with our bridge

### 2. Traffic Generation

**Current**: TUMFTM's generate_traffic.py (Python)

**Potential**: Port to Rust
- Use carla-rust API to spawn traffic
- Integrate into our bridge as optional feature
- CLI parameter: `--traffic-vehicles 30 --traffic-walkers 10`

### 3. Sensor Configuration Format

**Potential**: Support JSON sensor config as alternative to URDF

**Benefits**:
- Explicit CARLA sensor parameters
- Easier testing without full Autoware stack
- Override URDF sensors for specific scenarios

**Implementation**:
```rust
pub enum SensorConfigSource {
    Urdf(String),           // From /robot_description
    Json(PathBuf),          // From objects.json
    Hybrid(String, PathBuf), // URDF poses + JSON params
}
```

### 4. Message Converters

**Reusable patterns**:
- Velocity report generation from odometry
- Steering status from vehicle state
- Control mode reporting
- Pose with covariance generation

**Action**: Implement these in our bridge's Autoware module

## Limitations of TUMFTM Approach

1. **Dependency on carla_ros_bridge**:
   - Requires maintaining fork of carla_ros_bridge
   - Limited by carla_ros_bridge update cycle
   - Python performance bottleneck

2. **No direct CARLA control**:
   - Can't optimize CARLA API usage
   - Relies on carla_ros_bridge's implementation
   - Limited customization

3. **Multiple process overhead**:
   - Inter-process communication latency
   - Higher memory footprint
   - Complex debugging

4. **Manual sensor configuration**:
   - Objects.json must be kept in sync manually
   - Duplicate of URDF information
   - Error-prone

5. **Limited vehicle actuation**:
   - Only Ackermann control via separate controller
   - No direct throttle/brake/steering control
   - Hardcoded 1.2x steering multiplier

## Advantages of Our Approach

1. **Direct CARLA integration**:
   - No intermediate Python bridge
   - Lower latency
   - Full control over CARLA API usage

2. **Single process**:
   - Rust's async/multithreading
   - Lower memory footprint
   - Simpler deployment

3. **URDF-based configuration**:
   - Single source of truth
   - Auto-sync with Autoware
   - Vehicle-agnostic

4. **Type safety**:
   - Compile-time checking
   - Fewer runtime errors
   - Better IDE support

5. **Performance**:
   - Rust's zero-cost abstractions
   - No GIL (Global Interpreter Lock)
   - Efficient memory usage

## Recommendations for Our Bridge

### High Priority

1. **Add missing Autoware topics**:
   - [ ] `/vehicle/status/control_mode` publisher
   - [ ] `/vehicle/status/steering_status` publisher
   - [ ] `/control/command/control_cmd` subscriber
   - [ ] Implement vehicle actuation (throttle/brake/steering)

2. **Verify localization topic**:
   - [ ] Check if Autoware prefers `/sensing/gnss/pose_with_covariance` over odometry
   - [ ] Add pose_with_covariance publisher if needed

3. **Test with TUMFTM maps**:
   - [ ] Download their converted Lanelet2 maps
   - [ ] Verify compatibility with our bridge

### Medium Priority

1. **Sensor parameter configuration**:
   - [ ] Design JSON/YAML format for CARLA sensor params
   - [ ] Implement loader for sensor params
   - [ ] Support hybrid URDF + JSON approach

2. **Traffic generation**:
   - [ ] Port generate_traffic.py to Rust
   - [ ] Integrate as optional feature
   - [ ] Add CLI parameters for traffic control

3. **Control calibration**:
   - [ ] Investigate steering angle multiplier
   - [ ] Add vehicle-specific calibration config
   - [ ] Test with different vehicle models

### Low Priority

1. **Covariance configuration**:
   - [ ] Make covariance values configurable
   - [ ] Add to config file
   - [ ] Default values from TUMFTM (0.1 diagonal)

2. **Manual control integration**:
   - [ ] Consider adding manual control mode
   - [ ] Switch between autonomous and manual
   - [ ] Publish control_mode accordingly

## Conclusion

The TUMFTM Carla-Autoware-Bridge provides valuable insights into Autoware integration patterns and message conversions. While their architecture relies on Python and the existing carla_ros_bridge, their work validates several design decisions:

1. **Message conversion patterns** - Useful reference for Autoware topic structure
2. **Sensor configuration** - JSON format can complement our URDF parsing
3. **Control interface** - Identifies missing publishers/subscribers in our bridge
4. **Map resources** - Pre-converted maps for testing

Our direct Rust approach offers superior performance and maintainability while achieving their stated goal of a "Python-free, native DDS connection" bridge.

**Next steps**:
1. Implement missing Autoware topics (control_mode, steering_status, control_cmd)
2. Add vehicle actuation
3. Test with TUMFTM's converted maps
4. Consider optional JSON sensor configuration

---

**Last Updated**: 2025-11-08
**Related Documents**:
- `carla-autoware-map-integration.md` - Map conversion guide
- `autoware-integration-design.md` - Our bridge architecture
- `roadmap.md` - Project development plan
