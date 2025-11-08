# Sensor Configuration Strategy

Comprehensive analysis of sensor configuration sources, gaps, and strategy for automated CARLA sensor spawning from Autoware configuration.

**Last Updated**: 2025-11-08

---

## Overview

Our bridge follows a **fully automated** approach: spawn CARLA sensors based entirely on Autoware's configuration with minimal additional config files only for parameters that cannot be obtained from ROS.

**Design Principle**: URDF as primary source, TF2 for transforms, supplemental config file only for CARLA-specific parameters not available through ROS.

---

## What Autoware Provides

### 1. URDF (Robot Description)

**Source**: `/robot_description` topic
**Format**: XML (URDF)

**Information Available**:
- Sensor link names (e.g., `velodyne_top_base_link`, `camera0/camera_link`)
- Parent-child relationships (joints)
- **Initial transforms** (xyz, rpy) - **Not used** (we use TF2 instead for real-time accuracy)

**Example**:
```xml
<robot name="sample_sensor_kit">
  <link name="sensor_kit_base_link"/>
  <link name="velodyne_top_base_link"/>
  <joint name="velodyne_top_base_joint" type="fixed">
    <parent link="sensor_kit_base_link"/>
    <child link="velodyne_top_base_link"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 1.575"/>
  </joint>
</robot>
```

**What We Extract**:
- `link_name`: Name of sensor link
- `sensor_type`: Classified from link name (camera, lidar, imu, gnss)
- `parent_frame`: Parent link name for TF lookup

**Classification Rules** (from `urdf_parser.rs:classify_sensor_type()`):
```rust
// Camera: contains "camera" or "rgb"
"camera0/camera_link" → CameraRgb
"traffic_light_left_camera" → CameraRgb

// LiDAR: contains "lidar", "velodyne", "pointcloud", "laser"
"velodyne_top_base_link" → LidarRayCast
"lidar_front" → LidarRayCast

// IMU: contains "imu" or "inertial"
"tamagawa/imu_link" → Imu

// GNSS: contains "gnss" or "gps"
"gnss_link" → Gnss
"ublox_gps" → Gnss
```

### 2. TF2 (Transform Tree)

**Source**: `/tf_static` topic
**Format**: `tf2_msgs/TFMessage`

**Information Available**:
- **Real-time transforms** between all frames
- Position (x, y, z) in meters
- Orientation (quaternion w, x, y, z)
- Parent → child relationships

**Example TF transform**:
```
base_link → sensor_kit_base_link → velodyne_top_base_link
```

**What We Extract**:
- Full transform chain from `base_link` to sensor link
- Composed transform if multi-hop (e.g., `base_link → A → B → sensor`)
- Used for **actual CARLA spawn positions** (more accurate than URDF static values)

**Implementation**: `tf_bridge.rs:lookup_transform()` with tree walking

### 3. Camera Calibration Files

**Source**: YAML files (e.g., `traffic_light_camera.yaml`)
**Format**: ROS camera_info YAML

**Information Available**:
- `image_width`, `image_height` (resolution)
- `camera_matrix` (intrinsics: fx, fy, cx, cy)
- `distortion_coefficients` (k1, k2, p1, p2, k3)
- `rectification_matrix`
- `projection_matrix`

**Usage**:
- For **post-processing** (image rectification, calibration)
- **Not directly used for spawning** CARLA cameras
- CARLA uses FOV-based model, not intrinsic matrix

**Gap**: No direct FOV value in camera calibration files (we need to estimate or configure separately)

### 4. Sensor Calibration

**Source**: `sensor_kit_calibration.yaml`
**Format**: YAML with transforms

**Information Available**:
- Calibration offsets for each sensor
- Same information as TF2 but in static file

**Example**:
```yaml
sensor_kit_base_link:
  camera0/camera_link:
    x: 0.10731
    y: 0.56343
    z: -0.27697
    roll: -0.025
    pitch: 0.315
    yaw: 1.035
```

**Usage**: Redundant with TF2, **prefer TF2** for real-time accuracy

---

## What CARLA Requires

### 1. Camera Sensor (sensor.camera.rgb)

**Required Attributes**:
| Attribute      | Type  | Default | Source                     | Gap?                  |
|----------------|-------|---------|----------------------------|-----------------------|
| `image_size_x` | int   | 800     | ✅ Camera calibration YAML | No                    |
| `image_size_y` | int   | 600     | ✅ Camera calibration YAML | No                    |
| `fov`          | float | 90.0    | ❌ Not in Autoware config  | **Yes - Need config** |
| `sensor_tick`  | float | 0.0     | ❌ Not in Autoware config  | **Yes - Need config** |

**Optional Attributes** (quality/effects):
- `bloom_intensity`, `fstop`, `iso`, `gamma`, `lens_flare_intensity`, `shutter_speed`
- Lens distortion: `lens_k`, `lens_kcube`, `lens_circle_*`, etc.

**Recommendation**:
- Use defaults for optional attributes
- **FOV must be configured**: Add to supplemental config file
- `image_size` can be extracted from camera calibration YAML
- `sensor_tick` defaults to 0.0 (every frame) - acceptable

### 2. LiDAR Sensor (sensor.lidar.ray_cast)

**Required Attributes**:
| Attribute            | Type  | Default | Source                    | Gap?                  |
|----------------------|-------|---------|---------------------------|-----------------------|
| `channels`           | int   | 32      | ❌ Not in Autoware config | **Yes - Need config** |
| `range`              | float | 10.0    | ❌ Not in Autoware config | **Yes - Need config** |
| `points_per_second`  | int   | 56000   | ❌ Not in Autoware config | **Yes - Need config** |
| `rotation_frequency` | float | 10.0    | ❌ Not in Autoware config | **Yes - Need config** |
| `upper_fov`          | float | 10.0    | ❌ Not in Autoware config | **Yes - Need config** |
| `lower_fov`          | float | -30.0   | ❌ Not in Autoware config | **Yes - Need config** |
| `horizontal_fov`     | float | 360.0   | ✅ Default acceptable     | No (use default)      |
| `sensor_tick`        | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |

**Optional Attributes**:
- `atmosphere_attenuation_rate`, `dropoff_general_rate`, `dropoff_intensity_limit`, `dropoff_zero_intensity`, `noise_stddev`

**Recommendation**:
- **Most LiDAR params must be configured**
- Typical Velodyne VLP-32C: `channels=32`, `rotation_frequency=20`, `points_per_second=1.2M`
- Add to supplemental config file with sensor-specific values

### 3. IMU Sensor (sensor.other.imu)

**Required Attributes**:
| Attribute                  | Type  | Default | Source                    | Gap?                  |
|----------------------------|-------|---------|---------------------------|-----------------------|
| `noise_accel_stddev_x/y/z` | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_gyro_bias_x/y/z`    | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_gyro_stddev_x/y/z`  | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_seed`               | int   | 0       | ❌ Not in Autoware config | Optional (default OK) |
| `sensor_tick`              | float | 0.0     | ❌ Not in Autoware config | Optional (default OK) |

**Recommendation**:
- Defaults (0.0) = perfect sensor (no noise)
- **Acceptable for initial testing**
- For realistic simulation, add noise parameters to config
- Reference: Typical IMU specs (e.g., Xsens MTi-30)

### 4. GNSS Sensor (sensor.other.gnss)

**Required Attributes**:
| Attribute               | Type  | Default | Source                    | Gap?                  |
|-------------------------|-------|---------|---------------------------|-----------------------|
| `noise_alt_bias/stddev` | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_lat_bias/stddev` | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_lon_bias/stddev` | float | 0.0     | ❌ Not in Autoware config | **Yes - Need config** |
| `noise_seed`            | int   | 0       | ❌ Not in Autoware config | Optional (default OK) |
| `sensor_tick`           | float | 0.0     | ❌ Not in Autoware config | Optional (default OK) |

**Recommendation**:
- Defaults (0.0) = perfect GNSS (no noise)
- **Acceptable for initial testing**
- For realistic simulation, add noise parameters to config
- Reference: Typical GNSS accuracy (e.g., u-blox F9P: ~0.01m)

---

## Gap Summary

### ✅ Available from Autoware (via ROS)

| Information                | Source                  | Usage                    |
|----------------------------|-------------------------|--------------------------|
| Sensor existence and type  | URDF link names         | Automatic classification |
| Sensor positions           | TF2 /tf_static          | Spawn positions          |
| Sensor orientations        | TF2 /tf_static          | Spawn orientations       |
| Parent-child relationships | URDF joints + TF2       | Transform tree walking   |
| Camera resolution          | Camera calibration YAML | Image size_x/y           |

### ❌ Missing from Autoware (Need Config File)

| Parameter              | Sensor Types | Criticality | Recommendation            |
|------------------------|--------------|-------------|---------------------------|
| **FOV**                | Camera       | **High**    | Must configure            |
| **Channels**           | LiDAR        | **High**    | Must configure            |
| **Range**              | LiDAR        | **High**    | Must configure            |
| **Points per second**  | LiDAR        | **High**    | Must configure            |
| **Rotation frequency** | LiDAR        | **High**    | Must configure            |
| **Upper/Lower FOV**    | LiDAR        | **High**    | Must configure            |
| **Noise parameters**   | IMU, GNSS    | Medium      | Optional (defaults OK)    |
| **Sensor tick**        | All          | Low         | Optional (default 0.0 OK) |

---

## Proposed Configuration Strategy

### 1. Primary Source: URDF + TF2 (Automated)

**Always use for**:
- Sensor detection (which sensors exist)
- Sensor type classification (camera, lidar, imu, gnss)
- Sensor positions and orientations (via TF2 lookup)
- Parent-child relationships

**Implementation** (current):
```rust
// 1. Parse URDF to get sensor links
let sensors = parse_urdf_sensors(&urdf_xml)?;

// 2. For each sensor, lookup TF transform
let transform = tf_buffer.lookup_transform("base_link", &sensor.link_name)?;

// 3. Spawn sensor in CARLA
let carla_transform = Transform::from_na(&transform);
world.spawn_actor(&sensor_blueprint, &carla_transform)?;
```

### 2. Supplemental Source: CARLA Sensor Config (Manual)

**Purpose**: Provide CARLA-specific parameters not available via ROS

**Format**: YAML file
**Location**: `config/carla_sensors.yaml` (or similar)

**Proposed Structure**:
```yaml
# CARLA Sensor Configuration
# Supplements Autoware URDF with CARLA-specific parameters

sensors:
  # Camera sensors
  camera_rgb:  # Applies to all RGB cameras
    fov: 90.0
    sensor_tick: 0.0  # Every frame
    # Optional quality settings (defaults used if omitted)
    # bloom_intensity: 0.675
    # fstop: 1.4
    # iso: 100.0

  # LiDAR sensors
  lidar:  # Default for all LiDARs
    channels: 32
    range: 100.0
    points_per_second: 600000
    rotation_frequency: 20.0
    upper_fov: 15.0
    lower_fov: -25.0
    horizontal_fov: 360.0
    sensor_tick: 0.0

  # Sensor-specific overrides
  sensor_specific:
    velodyne_top_base_link:  # Specific sensor by URDF link name
      channels: 32
      range: 200.0
      points_per_second: 1200000
      rotation_frequency: 20.0
      upper_fov: 10.0
      lower_fov: -30.0

    velodyne_left_base_link:
      channels: 16
      range: 100.0
      points_per_second: 300000
      rotation_frequency: 10.0
      upper_fov: 15.0
      lower_fov: -15.0

    camera0/camera_link:
      fov: 110.0  # Wide-angle camera

  # IMU sensors (optional noise)
  imu:
    noise_accel_stddev_x: 0.0
    noise_accel_stddev_y: 0.0
    noise_accel_stddev_z: 0.0
    noise_gyro_bias_x: 0.0
    noise_gyro_bias_y: 0.0
    noise_gyro_bias_z: 0.0
    noise_gyro_stddev_x: 0.0
    noise_gyro_stddev_y: 0.0
    noise_gyro_stddev_z: 0.0

  # GNSS sensors (optional noise)
  gnss:
    noise_alt_bias: 0.0
    noise_alt_stddev: 0.0
    noise_lat_bias: 0.0
    noise_lat_stddev: 0.0
    noise_lon_bias: 0.0
    noise_lon_stddev: 0.0
```

**Usage Priority**:
1. Check `sensor_specific` for exact link name match → Use those values
2. Else check `lidar`/`camera_rgb`/`imu`/`gnss` for type defaults → Use those values
3. Else use CARLA blueprint defaults

**Implementation**:
```rust
pub struct CarlaSensorConfig {
    // Defaults for each sensor type
    camera_defaults: CameraConfig,
    lidar_defaults: LidarConfig,
    imu_defaults: ImuConfig,
    gnss_defaults: GnssConfig,

    // Sensor-specific overrides
    sensor_overrides: HashMap<String, SensorOverride>,
}

impl CarlaSensorConfig {
    pub fn get_lidar_params(&self, link_name: &str) -> LidarConfig {
        // 1. Check sensor_specific
        if let Some(override) = self.sensor_overrides.get(link_name) {
            return override.lidar_config.clone();
        }

        // 2. Use type defaults
        self.lidar_defaults.clone()
    }
}
```

### 3. Extraction from Camera Calibration (Optional Enhancement)

**For cameras**: Extract `image_width`, `image_height` from calibration YAML

**Challenge**: Need to map URDF link names to calibration file names

**Example**:
- URDF link: `traffic_light_left_camera/camera_link`
- Calibration file: `traffic_light_camera.yaml`

**Solution**: Naming convention or explicit mapping in config

**Priority**: **Low** (image size can be in CARLA config for simplicity)

---

## Implementation Plan

### Phase 1: Enhanced URDF + TF2 (Current - ✅ Done)

- [x] Parse URDF for sensor links
- [x] Classify sensor types from link names
- [x] Lookup TF transforms with tree walking
- [x] Spawn sensors in CARLA with TF-based positions

### Phase 2: CARLA Sensor Config File (Next)

- [ ] Define YAML schema for CARLA sensor parameters
- [ ] Implement config parser in Rust
- [ ] Add default values for each sensor type
- [ ] Support sensor-specific overrides
- [ ] Integrate with sensor spawning logic
- [ ] Document config file format

### Phase 3: Testing and Refinement

- [ ] Test with sample_sensor_kit
- [ ] Verify sensor parameters match expected output
- [ ] Tune default values for realistic simulation
- [ ] Add validation for config values
- [ ] Add error handling for missing parameters

### Phase 4: Optional Enhancements

- [ ] Extract camera resolution from calibration YAML
- [ ] Support camera FOV estimation from intrinsics
- [ ] Add noise parameter profiles (e.g., "realistic", "perfect")
- [ ] CLI flag to override config file path

---

## Comparison with TUMFTM Approach

### TUMFTM (Static JSON)

**Format**: `objects.json` with vehicle and sensors

**Example**:
```json
{
  "type": "sensor.lidar.ray_cast",
  "id": "sensor/lidar/front",
  "spawn_point": {"x": 1.5, "y": 0.0, "z": 2.1},
  "range": 50,
  "channels": 64,
  "points_per_second": 480000
}
```

**Characteristics**:
- ✅ All CARLA parameters explicit
- ✅ Easy to understand
- ❌ Duplicate configuration (must match URDF)
- ❌ Manual synchronization required
- ❌ Not vehicle-agnostic

### Our Approach (URDF + Config)

**Primary**: URDF + TF2 (automatic)
**Supplemental**: `carla_sensors.yaml` (CARLA-specific params only)

**Characteristics**:
- ✅ Single source of truth for positions (TF2)
- ✅ Automatically adapts to Autoware config changes
- ✅ Vehicle-agnostic (works with any URDF)
- ✅ Minimal config file (only CARLA-specific params)
- ⚠️ Requires understanding of both sources

**Advantage**: Fully automated for sensor detection and positioning, manual config only for simulation parameters that Autoware doesn't define.

---

## Map Creation Workflow

Based on TUMFTM and community resources:

### 1. Point Cloud Map (PCD)

**Source**: CARLA HDMaps or recorded from simulation

**Option A: Use Pre-existing Maps**
```bash
# CARLA installation includes PCD maps
CARLA/HDMaps/Town01.pcd
CARLA/HDMaps/Town02.pcd
...
CARLA/HDMaps/Town07.pcd
```

**Option B: Record from CARLA**
1. Spawn vehicle with LiDAR in CARLA
2. Drive around map (manual or autopilot)
3. Record point cloud data
4. Convert/merge to single PCD file

**Tools**:
- CARLA Python API for recording
- PCL (Point Cloud Library) for processing
- ROS bag recording if using bridge

### 2. Lanelet2 Vector Map (OSM)

**Source**: Convert from CARLA OpenDRIVE

**Input**:
```bash
CARLA/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr
```

**Conversion Tools**:

1. **CommonRoad Scenario Designer** (Recommended)
   - URL: https://commonroad.in.tum.de/tools/scenario-designer
   - Features: Bidirectional OpenDRIVE ↔ Lanelet2
   - Limitations: Requires manual editing

2. **assuremappingtools**
   - URL: https://github.com/hatem-darweesh/assuremappingtools
   - Features: OpenDRIVE to Lanelet2 conversion
   - Command: `./op_converter --input Town01.xodr --output Town01.osm --format lanelet2`

3. **TUMFTM FlexMap_Fusion** (Advanced)
   - URL: https://github.com/TUMFTM/FlexMap_Fusion
   - Features: Georeferencing and OSM fusion
   - Purpose: Fuse semantic info from OpenStreetMap

**Workflow**:
```bash
# 1. Export OpenDRIVE from CARLA
# (already available in CARLA installation)

# 2. Convert to Lanelet2
./assuremappingtools/op_converter \
    --input CARLA/Maps/OpenDrive/Town01.xodr \
    --output Town01.osm \
    --format lanelet2

# 3. Manual editing with Vector Map Builder
# - Add traffic lights
# - Fix lane connections
# - Verify coordinate alignment
```

### 3. Manual Editing

**Tool**: TIER IV Vector Map Builder
- URL: https://tools.tier4.jp/feature/vector_map_builder_ll2/
- Purpose: Edit Lanelet2 maps
- Required fixes:
  - Add traffic light information
  - Verify lane connections
  - Align with point cloud map
  - Add stop lines, regulatory elements

### 4. Pre-converted Maps (Quick Start)

**TUMFTM Maps**:
- URL: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/
- Includes: Lanelet2 (.osm) + Point Cloud (.pcd)
- Towns: Town01-Town07, Town10HD
- Limitation: Traffic lights not included

**Bitbucket autoware-contents**:
- URL: https://bitbucket.org/carla-simulator/autoware-contents
- Includes: Both map types
- Limitation: Traffic lights missing

---

## Recommendations

### Immediate (Phase 2)

1. **Implement CARLA sensor config file**
   - Schema: `config/carla_sensors.yaml`
   - Parser: `carla_sensor_config.rs`
   - Integrate with spawning logic
   - Document format and examples

2. **Define default sensor parameters**
   - Research typical sensor specs (Velodyne, Xsens, u-blox)
   - Create realistic default profiles
   - Test with sample_sensor_kit

### Short-term

3. **Test with TUMFTM pre-converted maps**
   - Download maps from their hosting
   - Verify compatibility with our bridge
   - Document any issues or required adjustments

4. **Document map creation workflow**
   - Step-by-step guide for converting CARLA towns
   - Include tool installation instructions
   - Provide troubleshooting tips

### Long-term (Optional)

5. **Camera calibration extraction**
   - Parse camera calibration YAML files
   - Extract image resolution
   - Optionally estimate FOV from intrinsics

6. **Noise parameter profiles**
   - Define sensor noise profiles (perfect, realistic, noisy)
   - CLI flag to select profile: `--sensor-noise realistic`
   - Document noise parameter sources and reasoning

---

## Related Documents

- `architecture-comparison.md` - Comparison with TUMFTM approach
- `tumftm-bridge-analysis.md` - Detailed TUMFTM analysis
- `carla-autoware-map-integration.md` - Comprehensive map conversion guide
- `autoware-integration-design.md` - Overall bridge architecture

---

**Last Updated**: 2025-11-08
**Status**: Phase 1 complete, Phase 2 design documented
