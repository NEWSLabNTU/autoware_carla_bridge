# CARLA Vehicle & Sensor Kit Launch Packages - Implementation Roadmap

**Goal**: Enable launching Autoware with CARLA using native Autoware commands:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=carla_vehicle \
  sensor_model:=carla_sensor_kit \
  map_path:=...
```

**Status**: Planning Phase
**Start Date**: 2025-11-23
**Target Completion**: TBD

---

## Package Overview

### carla_vehicle_launch
- **Purpose**: Provides vehicle description and interface for CARLA vehicles
- **Components**:
  - `carla_vehicle_description`: Vehicle URDF, dimensions, visualization
  - `carla_vehicle_launch`: Vehicle interface launch files

### carla_sensor_kit_launch
- **Purpose**: Provides sensor description and processing pipeline for CARLA sensors
- **Components**:
  - `carla_sensor_kit_description`: Sensor URDF, TF calibrations
  - `carla_sensor_kit_launch`: Sensor processing launch files

---

## Implementation Phases

## Phase 1: Vehicle Package - Description ✅ STARTED

**Goal**: Create carla_vehicle_description package with vehicle parameters

### Tasks

#### 1.1 Package Structure ✅
- [x] Copy from awsim_labs_vehicle_description
- [x] Update package.xml metadata
- [ ] Review and clean CMakeLists.txt

**Files**:
```
carla_vehicle_description/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── vehicle_info.param.yaml       # Vehicle dimensions
│   ├── simulator_model.param.yaml    # Simulator-specific (mostly empty)
│   └── mirror.param.yaml             # Mirror positions (optional)
├── urdf/
│   └── vehicle.xacro                 # Vehicle visualization
└── mesh/                             # Optional: vehicle 3D model
```

#### 1.2 Configure vehicle_info.param.yaml
- [ ] Document current default values (from awsim_labs)
- [ ] Add comments explaining each parameter
- [ ] Note: Will create extraction script later (Phase 6)

**Parameters to configure**:
```yaml
/**:
  ros__parameters:
    wheel_radius: 0.383      # meters - from CARLA vehicle physics
    wheel_width: 0.235       # meters
    wheel_base: 2.79         # meters - distance between axles
    wheel_tread: 1.64        # meters - distance between left/right wheels
    front_overhang: 1.0      # meters - from wheel center to front
    rear_overhang: 1.1       # meters - from wheel center to rear
    left_overhang: 0.128     # meters - from wheel center to left edge
    right_overhang: 0.128    # meters - from wheel center to right edge
    vehicle_height: 2.5      # meters - total height
    max_steer_angle: 0.70    # radians (~40 degrees)
```

#### 1.3 Simplify vehicle.xacro
- [ ] Review current URDF
- [ ] Simplify to basic box or keep awsim_labs mesh
- [ ] Document that this is for visualization only (CARLA handles physics)

**Decision needed**: Use simple box geometry or keep Lexus mesh?

#### 1.4 Review Other Config Files
- [ ] Check if simulator_model.param.yaml is needed (likely empty)
- [ ] Check if mirror.param.yaml is needed for CARLA
- [ ] Remove unnecessary files

**Estimated Time**: 2 hours

---

## Phase 2: Vehicle Package - Launch Files ✅ STARTED

**Goal**: Create minimal vehicle interface launch file

**Design Decision**: Bridge subscribes to `/control/command/control_cmd` directly instead of `/control/command/actuation_cmd`. This eliminates the need for `raw_vehicle_cmd_converter` since:
- CARLA has ideal actuators (no real hardware pedal mapping needed)
- Simple linear conversion from ControlCommand → CARLA VehicleControl
- Reduces pipeline complexity for simulation use case
- Bridge implements direct acceleration/steering mapping

### Tasks

#### 2.1 Create minimal vehicle_interface.launch.xml
- [x] Copied from awsim_labs_vehicle_launch
- [ ] Simplify to minimal/empty launch file (no raw_vehicle_cmd_converter needed)
- [ ] Add documentation comment explaining bridge handles control directly

**New file** (minimal for CARLA):
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!--
    CARLA Vehicle Interface

    For CARLA simulation, the bridge subscribes directly to /control/command/control_cmd
    and converts ControlCommand to CARLA VehicleControl internally.

    No raw_vehicle_cmd_converter needed since CARLA has ideal actuators.
    If actuation-level control is needed in the future, add raw_vehicle_cmd_converter here.
  -->
</launch>
```

#### 2.2 Clean up unnecessary files
- [ ] Remove or archive config/raw_vehicle_cmd_converter/ (not needed for direct control_cmd)
- [ ] Remove or archive data/*.csv files (accel/brake/steer maps not needed)
- [ ] Update package.xml if dependencies changed

**Optional**: Keep these files for future reference or alternative control modes, but mark as unused.

#### 2.3 Test vehicle package independently
- [ ] Build package: `colcon build --packages-select carla_vehicle_description carla_vehicle_launch`
- [ ] Verify URDF loads: `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix carla_vehicle_description)/share/carla_vehicle_description/urdf/vehicle.xacro)"`
- [ ] Check parameters load: `ros2 param list` after launching vehicle_info.launch.py

**Estimated Time**: 1 hour (reduced from 3 hours)

---

## Phase 3: Sensor Kit Package - Description ✅ STARTED

**Goal**: Create carla_sensor_kit_description with sensor definitions and TF calibrations

### Tasks

#### 3.1 Package Structure ✅
- [x] Copy from awsim_labs or sample_sensor_kit
- [x] Update package.xml metadata
- [ ] Review and clean CMakeLists.txt

**Files**:
```
carla_sensor_kit_description/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── sensor_kit_calibration.yaml   # TF transforms for sensors
│   └── sensors_calibration.yaml      # Sensor-specific calibrations
└── urdf/
    ├── sensor_kit.xacro              # Main sensor kit macro
    └── sensors.xacro                 # Individual sensor definitions
```

#### 3.2 Extract sensor definitions from bridge
- [ ] Review bridge's current URDF parsing in `src/autoware_carla_bridge/src/autoware.rs`
- [ ] Document which sensors are defined
- [ ] Create sensor_kit.xacro based on bridge's expectations

**Bridge expects these sensors** (from URDF):
```
- velodyne_top (LiDAR)
- velodyne_left (LiDAR)
- velodyne_right (LiDAR)
- velodyne_rear (LiDAR)
- traffic_light_right_camera
- traffic_light_left_camera
- camera0, camera1, camera2, camera3, camera4, camera5
- gnss_link (GNSS)
- tamagawa/imu_link (IMU)
```

#### 3.3 Create sensor_kit_calibration.yaml
- [ ] Extract TF transforms from bridge's current URDF parsing
- [ ] Document sensor positions relative to sensor_kit_base_link
- [ ] Match coordinate frames expected by bridge

**Example structure**:
```yaml
sensor_kit_base_link:
  velodyne_top_base_link:
    x: 0.9
    y: 0.0
    z: 2.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  # ... other sensors
```

#### 3.4 Create sensor_kit.xacro
- [ ] Define sensor_kit_macro with all sensors
- [ ] Include sensor models (velodyne, camera, IMU, GNSS xacros)
- [ ] Load calibrations from sensor_kit_calibration.yaml
- [ ] Ensure frame names match what bridge expects

**Key requirement**: Frame names must match bridge's sensor topic mapping!

**Estimated Time**: 4 hours

---

## Phase 4: Sensor Kit Package - Launch Files

**Goal**: Create sensing.launch.xml with sensor processing pipeline

### Tasks

#### 4.1 Create main sensing.launch.xml ✅ STARTED
- [x] Copy from awsim_labs_sensor_kit_launch
- [ ] Review and update includes
- [ ] Verify launch_driver flags are set to false (bridge is the driver)

**Structure**:
```xml
<launch>
  <!-- LiDAR processing -->
  <include file=".../lidar.launch.xml">
    <arg name="launch_driver" value="false"/>  <!-- Bridge publishes -->
  </include>

  <!-- IMU processing -->
  <include file=".../imu.launch.xml"/>

  <!-- GNSS processing -->
  <include file=".../gnss.launch.xml"/>

  <!-- Vehicle velocity converter (REQUIRED) -->
  <include file="$(find-pkg-share vehicle_velocity_converter)/launch/...">
    <arg name="input_vehicle_velocity_topic" value="/vehicle/status/velocity_status"/>
    <arg name="output_twist_with_covariance" value="/sensing/vehicle_velocity_converter/twist_with_covariance"/>
  </include>
</launch>
```

#### 4.2 Create lidar.launch.xml
- [ ] Review current file from awsim_labs
- [ ] Remove driver nodes (bridge publishes raw data)
- [ ] Keep pointcloud preprocessing if needed
- [ ] Configure pointcloud_preprocessor.launch.py

**Topics bridge publishes**:
```
/sensing/lidar/top/pointcloud
/sensing/lidar/left/pointcloud
/sensing/lidar/right/pointcloud
/sensing/lidar/rear/pointcloud
```

**Decision needed**:
- Do we need pointcloud concatenation?
- Do we need distortion correction?

#### 4.3 Create imu.launch.xml ✅ STARTED
- [x] Copy from awsim_labs
- [ ] Remove hardware driver (bridge publishes)
- [ ] Keep imu_corrector (REQUIRED for localization)
- [ ] Keep gyro_bias_estimator

**Processing pipeline**:
```
Bridge → /sensing/imu/tamagawa/imu_raw (raw IMU data)
  ↓
imu_corrector → /sensing/imu/imu_data (corrected IMU)
  ↓
Used by localization
```

#### 4.4 Create gnss.launch.xml
- [ ] Remove hardware driver (ublox_gps, septentrio)
- [ ] Add autoware_gnss_poser (REQUIRED)
- [ ] Configure topic remapping

**Processing pipeline**:
```
Bridge → /sensing/gnss/nav_sat_fix (sensor_msgs/NavSatFix)
  ↓
autoware_gnss_poser → /sensing/gnss/pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped)
  ↓
Used by localization
```

**New file to create**:
```xml
<launch>
  <group>
    <push-ros-namespace namespace="gnss"/>
    <include file="$(find-pkg-share autoware_gnss_poser)/launch/gnss_poser.launch.xml">
      <arg name="input_topic_fix" value="/sensing/gnss/nav_sat_fix"/>
      <arg name="output_topic_gnss_pose" value="pose"/>
      <arg name="output_topic_gnss_pose_cov" value="pose_with_covariance"/>
      <arg name="output_topic_gnss_fixed" value="fixed"/>
      <arg name="use_gnss_ins_orientation" value="false"/>
    </include>
  </group>
</launch>
```

#### 4.5 Configure diagnostic_aggregator (optional)
- [ ] Review diagnostic_aggregator config
- [ ] Update or remove if not needed for simulation

**Estimated Time**: 5 hours

---

## Phase 5: Integration & Testing

**Goal**: Integrate packages with Autoware and verify end-to-end functionality

### Tasks

#### 5.1 Build all packages
- [ ] Build: `colcon build --packages-select carla_vehicle_description carla_vehicle_launch carla_sensor_kit_description carla_sensor_kit_launch`
- [ ] Source: `source install/setup.bash`
- [ ] Verify packages are found: `ros2 pkg list | grep carla`

#### 5.2 Test URDF loading
- [ ] Launch robot_state_publisher with vehicle URDF
- [ ] Launch robot_state_publisher with sensor_kit URDF
- [ ] Check TF tree: `ros2 run tf2_tools view_frames`
- [ ] Verify all expected frames exist

**Commands**:
```bash
# Test vehicle URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix carla_vehicle_description)/share/carla_vehicle_description/urdf/vehicle.xacro)"

# Test sensor URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix carla_sensor_kit_description)/share/carla_sensor_kit_description/urdf/sensor_kit.xacro)"
```

#### 5.3 Test parameter loading
- [ ] Launch vehicle_info.launch.py manually
- [ ] Verify parameters are loaded: `ros2 param list`
- [ ] Check parameter values: `ros2 param get /vehicle_info wheel_base`

#### 5.4 Test with planning_simulator (without bridge)
- [ ] Launch Autoware planning_simulator with carla packages
- [ ] Verify no errors in launch
- [ ] Check all required nodes start
- [ ] Verify RViz displays vehicle correctly

**Command**:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=carla_vehicle \
  sensor_model:=carla_sensor_kit \
  map_path:=$HOME/autoware_map/sample-map-planning
```

#### 5.5 Test with CARLA + bridge
- [ ] Start CARLA: `just carla start 0.9.16 2000`
- [ ] Start bridge: `just bridge start 2000`
- [ ] Launch Autoware with carla packages
- [ ] Verify bridge spawns vehicle and sensors
- [ ] Check all sensor topics are published
- [ ] Verify vehicle status topics are published

#### 5.6 End-to-end autonomous driving test
- [ ] Initialize localization via bridge
- [ ] Run autonomous driving script: `just drive`
- [ ] Monitor vehicle behavior in CARLA
- [ ] Verify vehicle reaches goal
- [ ] Check for any errors or warnings

**Success criteria**:
- ✅ Vehicle spawns in CARLA
- ✅ All sensors publish data
- ✅ Localization works
- ✅ Planning generates valid paths
- ✅ Control commands reach CARLA
- ✅ Vehicle drives autonomously to goal

**Estimated Time**: 6 hours

---

## Phase 6: CARLA Parameter Extraction Script (Future)

**Goal**: Create script to automatically extract vehicle_info from CARLA blueprints

### Tasks

#### 6.1 Create extraction script
- [ ] Create `scripts/extract_vehicle_info_from_carla.py`
- [ ] Connect to CARLA server
- [ ] Query vehicle blueprint
- [ ] Extract physics parameters
- [ ] Calculate dimensions from bounding box
- [ ] Generate vehicle_info.param.yaml

**Script features**:
```python
#!/usr/bin/env python3
"""Extract vehicle_info.param.yaml from CARLA blueprint."""

Features:
- Connect to CARLA server
- List available vehicle blueprints
- Select blueprint (arg or interactive)
- Extract:
  - wheel_radius, wheel_width (from physics)
  - wheel_base (from wheel positions)
  - wheel_tread (from wheel positions)
  - overhangs (from bounding box - wheel positions)
  - vehicle_height (from bounding box)
  - max_steer_angle (from physics)
- Generate YAML file
- Option to save to specific location
```

#### 6.2 Support multiple vehicle types
- [ ] Create template for multiple vehicle configs
- [ ] Generate multiple YAML files for different CARLA vehicles
- [ ] Document how to select at launch time

**Usage**:
```bash
# Extract from default vehicle
./scripts/extract_vehicle_info_from_carla.py

# Extract from specific blueprint
./scripts/extract_vehicle_info_from_carla.py --blueprint vehicle.tesla.model3

# Save to specific location
./scripts/extract_vehicle_info_from_carla.py \
  --blueprint vehicle.lincoln.mkz_2017 \
  --output src/carla_vehicle_launch/carla_vehicle_description/config/vehicle_info_lincoln.param.yaml
```

**Estimated Time**: 4 hours

---

## Phase 7: Documentation & Cleanup

**Goal**: Document the packages and clean up code

### Tasks

#### 7.1 Create package READMEs
- [ ] Create carla_vehicle_launch/README.md
- [ ] Create carla_sensor_kit_launch/README.md
- [ ] Document package purpose
- [ ] Document how to use
- [ ] Document configuration options

#### 7.2 Update main README
- [ ] Add section about carla launch packages
- [ ] Update quick start guide
- [ ] Add examples of launching Autoware with CARLA

#### 7.3 Update CLAUDE.md
- [ ] Document carla launch packages
- [ ] Update "What Works" section
- [ ] Add usage examples
- [ ] Document integration points

#### 7.4 Code cleanup
- [ ] Remove unused files from copied packages
- [ ] Clean up comments
- [ ] Verify all package.xml dependencies are correct
- [ ] Run linters/formatters if applicable

**Estimated Time**: 3 hours

---

## Testing Checklist

### Unit Tests
- [ ] Vehicle URDF loads without errors
- [ ] Sensor URDF loads without errors
- [ ] vehicle_info parameters are valid (all positive, correct units)
- [ ] All sensor frames are defined
- [ ] TF tree is complete (no missing transforms)

### Integration Tests
- [ ] Autoware launches with carla packages (no bridge)
- [ ] Bridge spawns vehicle using carla URDF
- [ ] Bridge spawns all sensors from carla URDF
- [ ] All sensor topics publish data
- [ ] All vehicle status topics publish data
- [ ] Localization receives and processes data
- [ ] Planning generates paths
- [ ] Control sends commands to bridge

### End-to-End Tests
- [ ] Initialize localization
- [ ] Set route
- [ ] Engage autonomous mode
- [ ] Vehicle drives to goal
- [ ] No crashes or errors
- [ ] Performance is acceptable

---

## Known Issues & Decisions

### Decisions Made
1. **Static vehicle_info.param.yaml** - No dynamic parameter publishing from bridge
   - Rationale: Autoware nodes read parameters once at startup
   - Future: Create extraction script (Phase 6)

2. **Direct control_cmd subscription** - Bridge subscribes to `/control/command/control_cmd`
   - Rationale: CARLA has ideal actuators, no need for pedal mapping
   - Simple linear conversion: ControlCommand → CARLA VehicleControl
   - Eliminates raw_vehicle_cmd_converter node (reduces pipeline complexity)
   - Alternative: Could add raw_vehicle_cmd_converter later if actuation-level control needed

3. **Sensor descriptions in URDF** - Bridge reads from /robot_description
   - Rationale: Bridge doesn't create URDF, it reads it
   - sensor_kit_description provides the source of truth

### Open Questions
- [ ] Do we need pointcloud concatenation for multiple LiDARs?
- [ ] Do we need distortion correction for pointclouds?
- [ ] Should we use simple box or keep mesh for vehicle visualization?
- [ ] What default CARLA vehicle blueprint should we target?

### Future Enhancements
- [ ] Support multiple CARLA vehicle types (different YAML files)
- [ ] Auto-detect CARLA vehicle and load matching config
- [ ] Sensor configuration from CARLA blueprint
- [ ] Dynamic sensor spawning based on CARLA vehicle

---

## Dependencies

### ROS 2 Packages Required
- `autoware_vehicle_info_utils` - Vehicle parameter utilities
- `vehicle_velocity_converter` - Velocity to twist conversion
- `autoware_imu_corrector` - IMU bias correction
- `autoware_gnss_poser` - GNSS to pose conversion
- `robot_state_publisher` - Publish TF from URDF

**Note**: `autoware_raw_vehicle_cmd_converter` is NOT required since bridge subscribes to `/control/command/control_cmd` directly

### External Dependencies
- CARLA simulator (0.9.14, 0.9.15, or 0.9.16)
- Autoware.universe (2024/2025 version)
- carla-rust bindings (for bridge)

---

## Success Metrics

### Milestone 1: Packages Build ✅
- [ ] All 4 packages build without errors
- [ ] All packages can be found by ROS 2

### Milestone 2: Standalone Launch ✅
- [ ] Can launch Autoware planning_simulator with carla packages
- [ ] RViz displays vehicle correctly
- [ ] No launch errors

### Milestone 3: Bridge Integration ✅
- [ ] Bridge reads URDF from carla_sensor_kit_description
- [ ] Bridge spawns vehicle with correct dimensions
- [ ] Bridge spawns all sensors with correct positions

### Milestone 4: End-to-End ✅
- [ ] Full autonomous driving works with carla packages
- [ ] Performance is comparable to using planning_simulator
- [ ] No regression in bridge functionality

---

## Timeline Estimate

| Phase                      | Tasks                              | Estimated Time |
|----------------------------|------------------------------------|----------------|
| 1. Vehicle Description     | Package setup, params, URDF        | 2 hours        |
| 2. Vehicle Launch          | Minimal launch file (simplified)   | 1 hour         |
| 3. Sensor Description      | URDF, calibrations, TF             | 4 hours        |
| 4. Sensor Launch           | Processing pipeline, converters    | 5 hours        |
| 5. Integration & Testing   | Build, test, verify                | 6 hours        |
| 6. CARLA Extraction Script | Auto-generate params               | 4 hours        |
| 7. Documentation           | READMEs, cleanup                   | 3 hours        |
| **Total**                  |                                    | **25 hours**   |

**Realistic timeline**: 4-5 working days (accounting for debugging and iteration)

**Note**: Phase 2 reduced from 3 to 1 hour due to simplified control design (no raw_vehicle_cmd_converter needed)

---

## Next Steps

### Immediate Actions (Phase 1 & 2)
1. ✅ Review copied package structure
2. [ ] Clean up unnecessary files from awsim_labs
3. [ ] Update package.xml metadata
4. [ ] Document current vehicle_info.param.yaml values
5. [ ] Test vehicle package builds independently

### After Vehicle Package Works
6. [ ] Move to Phase 3 (Sensor Kit Description)
7. [ ] Extract sensor definitions from bridge
8. [ ] Create sensor_kit.xacro matching bridge expectations

### After Both Packages Work
9. [ ] Move to Phase 5 (Integration Testing)
10. [ ] Test with Autoware + CARLA + Bridge
11. [ ] Run end-to-end autonomous driving test

---

**Last Updated**: 2025-11-23
**Status**: Phase 1 & 2 - Package structure created, needs configuration
