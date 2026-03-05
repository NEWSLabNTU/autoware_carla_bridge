# Automated Map Generation for CARLA-Autoware Integration

Comprehensive guide to automated and semi-automated workflows for generating Autoware-compatible maps from CARLA.

**Last Updated**: 2026-03-05

---

## Overview

This document focuses on **automated processes** for converting CARLA maps to Autoware format, minimizing manual editing requirements.

**Goal**: Generate both **Point Cloud (.pcd)** and **Lanelet2 (.osm)** maps for Autoware with maximum automation.

---

## Automated Conversion Tools

### 1. CommonRoad Scenario Designer (TUM) - RECOMMENDED

**Source**: https://github.com/CommonRoad/commonroad-scenario-designer (local: `src/external/commonroad-scenario-designer/`)
**PyPI**: `commonroad-scenario-designer` v0.8.5 (Sep 2025), v1.0+ in development
**Paper**: Althoff, Urban, Koschi (2018) - "Automatic Conversion of Road Networks from OpenDRIVE to Lanelets"
**Automation Level**: ⭐⭐⭐⭐ (High - mostly automated)

**Status**: De facto standard. Supersedes the old `opendrive2lanelet` package (now archived on GitLab). Used by TUMFTM for their Carla-Autoware-Bridge maps.

**Features**:
- Direct OpenDRIVE (.xodr) → Lanelet2 (.osm) conversion via CLI
- Also supports: CommonRoad XML, OSM, SUMO formats
- Python package with GUI and CLI
- Python 3.10-3.13 support
- Active development (bug fixes for OpenDRIVE traffic light assignment, Apr 2025)

**Installation**:
```bash
pip install commonroad-scenario-designer
```

**Usage**:
```bash
# Direct OpenDRIVE to Lanelet2 conversion (CLI)
crdesigner --input-file Town01.xodr --output-file Town01.osm odrlanelet2
```

**CRITICAL: Autoware mode** - Must enable `autoware=True` and `use_local_coordinates=True` for Autoware-compatible output. These are **disabled by default** and add: `mgrs_code`, `local_x`/`local_y`, `ele` tags on all nodes; `height` on traffic light ways; 2-node TL format.

```python
# Python API with Autoware flags (recommended)
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet
from crdesigner.common.config.lanelet2_config import Lanelet2Config
from crdesigner.common.config.opendrive_config import OpenDriveConfig

lanelet2_cfg = Lanelet2Config()
lanelet2_cfg.autoware = True
lanelet2_cfg.use_local_coordinates = True

opendrive_to_lanelet(
    input_file="Town01.xodr",
    output_file="Town01.osm",
    odr_config=OpenDriveConfig(),
    lanelet2_config=lanelet2_cfg,
)
```

**Post-processing required**: The output needs one fix before Autoware validation passes:
- Traffic light way `subtype` is empty (CommonRoad bug: `light.color` not populated from OpenDRIVE)
- Must set `subtype=red_yellow_green` (the Autoware/lanelet2 canonical value: `AttributeValueString::RedYellowGreen`)
- Speed limit regulatory elements are empty shells (36 elements with zero members) — remove them
- TUMFTM's `red_redYellow_green_yellow` subtype on TL ways is **incorrect** per Autoware spec

**Evaluation results** (2026-03-05, Town01):
| Metric | CR+Autoware | TUMFTM Reference |
|--------|-------------|------------------|
| Nodes | 43,154 | 43,082 |
| Lanelets | 300 | 300 (exact match) |
| TL regulatory elements | 36 | 36 |
| TL ways with height | 36 | 36 |
| local_x/local_y | Yes | Yes |
| TL subtype correct | No (empty→fixable) | No (wrong value) |

**Limitations**:
- TL way `subtype` empty (fixable with post-processing)
- Speed limit regulatory elements have no members (remove them)
- `lane_change=no` added to all ways (may need cleanup)
- 88 lanelets have no subtype (walkways — may need reclassification)
- May produce excessive points on straight roads (causes Autoware planning issues)

---

### 2. joel-mb/odr2lanelet2 - UNIQUE: Traffic Light Support

**Source**: https://github.com/joel-mb/odr2lanelet2 (local: `src/external/odr2lanelet2/`)
**Automation Level**: ⭐⭐⭐ (Medium - requires CARLA server for full features)

**Status**: Only tool that extracts traffic light regulatory elements from CARLA. Last updated Jul 2024 (added traffic lights, stop signs, crosswalks, lane changes).

**Features**:
- Direct OpenDRIVE → Lanelet2 (.osm) conversion
- Traffic light extraction (boxes, bulbs with colors, stop lines) - requires CARLA server
- Stop sign extraction with regulatory elements - requires CARLA server
- Crosswalk conversion
- Lane change and turn direction attributes
- Uses `carla.Map` Python API for lane geometry (works offline from .xodr)
- Small codebase (~880 lines converter + ~330 lines OpenDRIVE wrapper)

**Dependencies**: CARLA Python API (`import carla`)

**Usage**:
```bash
cd src/external/odr2lanelet2

# Lanes only (no CARLA server needed)
python odr2lanelet2.py -i Town01.xodr -o Town01.osm

# Lanes + traffic lights + stop signs (CARLA server required)
python odr2lanelet2.py -i Town01.xodr -o Town01.osm --carla
```

**Traffic light output format** (Autoware-compatible):
- `traffic_light` linestrings with `subtype: red_yellow_green` and `height`
- `light_bulbs` linestrings with individual bulb points (green, yellow, red colors)
- `stop_line` linestrings at landmark positions
- `regulatory_element` linking traffic lights to affected lanelets

**Limitations**:
- Small community (13 stars, 6 forks)
- No releases or packages
- Traffic light extraction quality not validated against Autoware
- Hardcoded speed limit of 30 for all lanelets
- CARLA server must be running with the target map loaded for traffic light extraction

---

### 3. usdot-fhwa-stol/opendrive2lanelet

**Source**: https://github.com/usdot-fhwa-stol/opendrive2lanelet
**Automation Level**: ⭐⭐⭐⭐ (High - US DOT maintained)

**Features**:
- Automatic conversion for US Department of Transportation
- Part of CARMA platform
- Well-maintained repository

**Usage**: Similar to CommonRoad opendrive2lanelet (likely based on it)

```bash
pip install opendrive2lanelet
opendrive2lanelet-convert input.xodr -o output.xml
```

**Limitations**:
- Focused on US road networks
- CARMA platform specific features
- May not handle all OpenDRIVE features

---

### 4. Lanelet2 Python API (Direct Creation)

**Source**: https://github.com/fzi-forschungszentrum-informatik/Lanelet2
**Automation Level**: ⭐⭐ (Low - requires programming)

**Features**:
- Create Lanelet2 maps programmatically
- Full control over map structure
- Python bindings for C++ library

**Installation**:
```bash
# Build from source (requires ROS environment)
git clone https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git
cd Lanelet2
# Follow build instructions in README
```

**Usage Example**:
```python
import lanelet2
from lanelet2.core import AttributeMap, getId, GPSPoint
from lanelet2.projection import UtmProjector

# Create projector
projector = UtmProjector(lanelet2.io.Origin(49.0, 8.4))

# Create map
map = lanelet2.core.LaneletMap()

# Add points, linestrings, lanelets manually
# ... (requires detailed knowledge of Lanelet2 structure)

# Write to file
lanelet2.io.write("output.osm", map, projector)
```

**Limitations**:
- Requires manual coding for each map
- Steep learning curve
- Not practical for CARLA map conversion
- Best for creating custom test maps

---

## Automated Point Cloud Generation

### Option 1: Use Pre-existing CARLA PCD Maps

**Source**: CARLA installation
**Automation Level**: ⭐⭐⭐⭐⭐ (Fully automated - already exists)

**Location**:
```bash
CARLA/HDMaps/Town01.pcd
CARLA/HDMaps/Town02.pcd
...
CARLA/HDMaps/Town07.pcd
```

**Advantages**:
- ✅ Already generated
- ✅ High quality
- ✅ Tested with CARLA

**Limitations**:
- ❌ Only Town01-07 available
- ❌ May not match custom CARLA maps
- ❌ Fixed density/quality

---

### Option 2: Automated Recording with CARLA Python API

**Automation Level**: ⭐⭐⭐⭐ (High - scriptable)

**Workflow**:
```python
import carla
import numpy as np
import open3d as o3d

# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()

# Spawn vehicle with LiDAR
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

# Configure LiDAR
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('points_per_second', '1000000')
lidar_bp.set_attribute('rotation_frequency', '20')
lidar_bp.set_attribute('range', '100')

# Spawn
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
lidar_transform = carla.Transform(carla.Location(z=2.5))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

# Collect point clouds
point_clouds = []

def lidar_callback(point_cloud):
    # Convert to numpy array
    data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Extract x, y, z (drop intensity)
    points = data[:, :3]
    point_clouds.append(points)

lidar.listen(lidar_callback)

# Enable autopilot to drive around
vehicle.set_autopilot(True)

# Record for N seconds
import time
duration = 300  # 5 minutes
time.sleep(duration)

# Stop recording
lidar.stop()
vehicle.destroy()
lidar.destroy()

# Merge point clouds
all_points = np.vstack(point_clouds)

# Save as PCD using Open3D
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(all_points)
o3d.io.write_point_cloud("Town_custom.pcd", pcd)

print(f"Saved {len(all_points)} points to Town_custom.pcd")
```

**Advantages**:
- ✅ Fully automated (no manual driving)
- ✅ Works with any CARLA map
- ✅ Configurable LiDAR parameters
- ✅ Autopilot coverage

**Limitations**:
- ⚠️ Autopilot may not cover all areas
- ⚠️ Requires Open3D or similar library
- ⚠️ May need multiple passes for full coverage

**Improvements**:
- Use multiple spawn points
- Manually define waypoint paths for complete coverage
- Filter ground points
- Downsample for performance

---

### Option 3: ROS-based Recording

**Automation Level**: ⭐⭐⭐ (Medium - requires ROS setup)

**Workflow**:
1. Run CARLA with our bridge
2. Record LiDAR topic to rosbag
3. Convert rosbag to PCD

```bash
# Record LiDAR data
ros2 bag record /sensing/lidar/top/pointcloud

# Convert to PCD (after recording)
rosrun pcl_ros bag_to_pcd input.bag /sensing/lidar/top/pointcloud output_dir/

# Merge PCDs
pcl_concatenate_points_pcd output_dir/*.pcd output.pcd
```

**Advantages**:
- ✅ Integrates with our bridge
- ✅ Standard ROS workflow
- ✅ Easy to filter/process with ROS tools

**Limitations**:
- ❌ Requires running bridge and CARLA
- ❌ More complex setup
- ❌ Larger storage (rosbag + PCD)

---

## End-to-End Automated Workflows

### Workflow 1: Pre-converted Maps (Fastest)

**Automation Level**: ⭐⭐⭐⭐⭐ (Fully automated)
**Time**: ~5 minutes
**Effort**: Minimal

**Steps**:
```bash
# 1. Download pre-converted maps from TUMFTM
wget https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/Town01.tar.gz
tar -xzf Town01.tar.gz

# 2. Extract PCD and OSM files
# Town01/
#   ├── lanelet2_map.osm
#   └── pointcloud_map.pcd

# 3. Copy to Autoware maps directory
mkdir -p ~/autoware_maps/Town01
cp Town01/* ~/autoware_maps/Town01/

# 4. Launch Autoware with map
ros2 launch autoware_launch ... map_path:=~/autoware_maps/Town01
```

**Advantages**:
- ✅ Zero conversion effort
- ✅ Tested and validated
- ✅ Includes Town01-07, Town10HD

**Limitations**:
- ❌ Limited to available towns
- ❌ Traffic lights missing
- ❌ Cannot customize

---

### Workflow 2: Semi-Automated (OpenDRIVE → Lanelet2)

**Automation Level**: ⭐⭐⭐⭐ (Mostly automated)
**Time**: ~30 minutes
**Effort**: Low

**Steps**:
```bash
# 1. Export OpenDRIVE from CARLA
# Already available in CARLA installation:
# CARLA/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr

# Or use Python API to export:
python3 << EOF
import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
xodr = world.get_map().to_opendrive()
with open('Town_custom.xodr', 'w') as f:
    f.write(xodr)
EOF

# 2. Convert OpenDRIVE to Lanelet2 using opendrive2lanelet
pip install opendrive2lanelet
opendrive2lanelet-convert Town01.xodr -o Town01.xml

# 3. Convert CommonRoad XML to Lanelet2 OSM
# (Using Lanelet2 library or CommonRoad Scenario Designer GUI)
# Or use odr2lanelet2:
git clone https://github.com/joel-mb/odr2lanelet2.git
cd odr2lanelet2
python odr2lanelet2.py ../Town01.xodr ../Town01.osm

# 4. Use CARLA HDMaps PCD or record new one
cp CARLA/HDMaps/Town01.pcd ~/autoware_maps/Town01/pointcloud_map.pcd
cp Town01.osm ~/autoware_maps/Town01/lanelet2_map.osm

# 5. (Optional) Manual editing with Vector Map Builder
# - Add traffic lights
# - Fix lane connections
# - Adjust coordinate alignment
```

**Advantages**:
- ✅ Works with any CARLA map
- ✅ Mostly automated
- ✅ Uses standard tools

**Limitations**:
- ⚠️ May require manual editing for traffic lights
- ⚠️ CommonRoad XML intermediate format
- ⚠️ Coordinate alignment may need adjustment

---

### Workflow 3: Fully Custom (Real-world OSM → CARLA → Autoware)

**Automation Level**: ⭐⭐⭐ (Medium)
**Time**: ~2 hours
**Effort**: Medium

**Steps**:
```bash
# 1. Export OSM from OpenStreetMap
# Go to openstreetmap.org
# Select area and export to .osm file

# 2. Convert OSM to OpenDRIVE using CARLA API
python3 << EOF
import carla
import io

# Read OSM file
with io.open('area.osm', mode='r', encoding='utf-8') as f:
    osm_data = f.read()

# Configure conversion settings
settings = carla.Osm2OdrSettings()
settings.set_osm_way_types([
    "motorway", "trunk", "primary",
    "secondary", "tertiary", "residential"
])
settings.generate_traffic_lights = True
settings.all_junctions_with_traffic_lights = True

# Convert to OpenDRIVE
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# Save OpenDRIVE
with open('area.xodr', 'w') as f:
    f.write(xodr_data)
EOF

# 3. Load map in CARLA
python3 << EOF
import carla

client = carla.Client('localhost', 2000)

# Read OpenDRIVE
with open('area.xodr', 'r') as f:
    xodr_xml = f.read()

# Generate world in CARLA
world = client.generate_opendrive_world(
    xodr_xml,
    carla.OpendriveGenerationParameters(
        vertex_distance=2.0,
        max_road_length=500.0,
        wall_height=0.0,
        additional_width=0.6,
        smooth_junctions=True
    )
)
print("CARLA map loaded!")
EOF

# 4. Record point cloud (autopilot method)
python3 scripts/record_pointcloud.py --duration 600 --output area.pcd

# 5. Convert OpenDRIVE to Lanelet2
opendrive2lanelet-convert area.xodr -o area.xml
# Then convert XML to OSM or use odr2lanelet2

# 6. Setup Autoware map
mkdir -p ~/autoware_maps/area_custom
cp area.pcd ~/autoware_maps/area_custom/pointcloud_map.pcd
cp area.osm ~/autoware_maps/area_custom/lanelet2_map.osm
```

**Advantages**:
- ✅ Create any real-world location
- ✅ CARLA can load directly
- ✅ Traffic lights included (if configured)

**Limitations**:
- ⚠️ OSM quality varies by location
- ⚠️ May need manual cleanup
- ⚠️ Large maps may have issues

---

## Recommended Automated Workflow

**For quick testing** → Use **Workflow 1** (pre-converted maps)

**For custom CARLA towns** → Use **Workflow 2** (semi-automated):
```bash
# Complete script
#!/bin/bash
TOWN=$1  # e.g., Town01

# 1. Get OpenDRIVE
XODR_FILE="CARLA/CarlaUE4/Content/Carla/Maps/OpenDrive/${TOWN}.xodr"

# 2. Convert to Lanelet2
pip install opendrive2lanelet
git clone https://github.com/joel-mb/odr2lanelet2.git

cd odr2lanelet2
python odr2lanelet2.py "${XODR_FILE}" "${TOWN}.osm"
cd ..

# 3. Copy PCD
cp "CARLA/HDMaps/${TOWN}.pcd" .

# 4. Setup Autoware map directory
MAP_DIR="$HOME/autoware_maps/${TOWN}"
mkdir -p "${MAP_DIR}"
mv "${TOWN}.osm" "${MAP_DIR}/lanelet2_map.osm"
mv "${TOWN}.pcd" "${MAP_DIR}/pointcloud_map.pcd"

echo "Map ready at: ${MAP_DIR}"
echo "Limitations: Traffic lights not included, may need manual editing"
```

---

## Traffic Light Integration

**Problem**: Automated converters typically **don't include traffic lights**

**Solutions**:

### Option 1: Manual Addition (Vector Map Builder)
- Use TIER IV Vector Map Builder web tool
- Manually place traffic lights at intersections
- Time: ~30 min per town

### Option 2: Extract from CARLA API
```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()

# Get all traffic lights
traffic_lights = world.get_actors().filter('traffic.traffic_light')

for tl in traffic_lights:
    location = tl.get_location()
    print(f"Traffic Light at: {location.x}, {location.y}, {location.z}")
    # Export coordinates for manual insertion
```

### Option 3: Automated with OSM Conversion (CARLA API)
```python
settings = carla.Osm2OdrSettings()
settings.generate_traffic_lights = True
settings.all_junctions_with_traffic_lights = True
```
Then convert OpenDRIVE to Lanelet2 (traffic lights should be preserved)

---

## Tool Comparison Matrix

Updated 2026-03:

| Tool | Automation | Input | Output | Traffic Lights | Last Active | Recommended |
|------|------------|-------|--------|----------------|-------------|-------------|
| **CommonRoad Scenario Designer** | High | .xodr | .osm (direct) | ⚠️ Partial (empty subtype) | Oct 2025 (v0.8.5) | ⭐⭐⭐⭐ |
| **odr2lanelet2** | Medium | .xodr | .osm (direct) | ✅ Yes (CARLA) | Jul 2024 | ⭐⭐⭐ |
| **usdot opendrive2lanelet** | High | .xodr | CommonRoad XML | ❌ No | Mar 2025 | ⭐⭐⭐ |
| **GDAL 3.10 XODR driver** | Medium | .xodr | Any GDAL format | ❌ No | Sep 2024+ | ⭐⭐ (new) |
| **Lanelet2 Python API** | Low | - | .osm | ✅ Yes (manual) | Active | ⭐⭐ |
| **TIER IV Vector Map Builder** | Manual | .pcd ref | .osm | ✅ Yes (manual) | Active (web) | ⭐⭐⭐⭐ |
| **autoware_lanelet2_map_validator** | High | .osm | Report | N/A | Jan 2026 (v1.6.0) | ⭐⭐⭐⭐⭐ |
| **TUMFTM Pre-converted** | **Full** | - | .osm + .pcd | ❌ No | Apr 2025 | ⭐⭐⭐⭐⭐ |
| **Bitbucket autoware-contents** | **Full** | - | .osm + .pcd | ❌ No | Oct 2025 | ⭐⭐⭐⭐ |

**Note**: The old standalone `opendrive2lanelet` PyPI package (TUM) is **archived** - use `commonroad-scenario-designer` instead.

---

## Future Automation Possibilities

### 1. CARLA Python API Extension
Create a CARLA plugin that:
- Automatically spawns vehicle with LiDAR
- Drives autopilot coverage pattern
- Records and merges point clouds
- Exports OpenDRIVE
- Converts to Lanelet2 in one command

```bash
# Hypothetical future tool
carla-to-autoware --town Town01 --output ~/autoware_maps/Town01
```

### 2. ROS 2 Service for Map Conversion
Integrate into our bridge:
```bash
# Service call to generate maps on-demand
ros2 service call /bridge/generate_map \
    autoware_carla_bridge/GenerateMap \
    "{town_name: 'Town01', output_path: '~/autoware_maps/Town01'}"
```

### 3. CI/CD Pipeline
Automated map generation in GitHub Actions:
- Trigger on new CARLA release
- Auto-convert all towns
- Upload to releases
- Update documentation

---

## Conclusion

**Current Best Practice** (2026-03):
1. **Testing**: Use TUMFTM pre-converted maps (in `data/carla-autoware-bridge/`)
2. **Custom towns**: CommonRoad Scenario Designer with `autoware=True` + post-processing script (fix TL subtypes, remove empty speed limits)
3. **Custom towns with traffic lights**: Add `odr2lanelet2 --carla` for TL extraction (requires CARLA server), or post-process CommonRoad TL output
4. **Point clouds**: Use CARLA HDMaps or autopilot recording script

**Automation Status**:
- Point Cloud: ✅ Fully automated
- Lanelet2 (lanes + basic TL): ⭐⭐⭐⭐ 90% automated (CommonRoad `autoware=True` + post-processing script)
- Lanelet2 (with full TL regulatory elements): ⭐⭐⭐ 60% automated (odr2lanelet2, needs CARLA server)
- Map Validation: ⭐⭐⭐⭐⭐ Fully automated (autoware_lanelet2_map_validator)

**Key finding** (2026-03-05): CommonRoad's `autoware=True` flag produces output structurally identical to TUMFTM reference maps. The remaining differences are minor post-processing fixes (TL subtype, empty speed limits) and TUMFTM's manual reclassification of walkway vs road lanelets.

**Ecosystem notes** (from Autoware ODD WG, 2025):
- No tool produces fully Autoware-compliant Lanelet2 directly
- Converted maps may have excessive points (reduces planning performance)
- Native OpenDRIVE support in Autoware was proposed but not adopted
- All major projects (TUMFTM, TIER IV, Autoware Foundation) use manual post-processing

**Next steps for this project**:
- Create wrapper script for end-to-end conversion pipeline (CommonRoad + post-processing)
- Test converted maps loading in Autoware
- Validate with autoware_lanelet2_map_validator (build from src/external/)

---

## Related Documents

- `sensor-configuration-strategy.md` - Sensor configuration gaps and strategy
- `carla-autoware-map-integration.md` - Complete map integration guide
- `tumftm-bridge-analysis.md` - TUMFTM implementation analysis

---

**Last Updated**: 2026-03-05
**Contributors**: Research and documentation based on community tools and workflows
