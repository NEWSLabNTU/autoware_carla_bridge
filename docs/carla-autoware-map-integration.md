# CARLA to Autoware Map Integration Guide

## Overview

This guide covers the process of integrating CARLA simulator maps with Autoware for autonomous driving simulation. CARLA and Autoware use different map formats, requiring conversion and manual editing for proper integration.

## Map Format Comparison

### CARLA Map Format

- **Format**: OpenDRIVE (.xodr)
- **Purpose**: Road network description for simulation
- **Characteristics**:
  - XML-based format
  - Describes road geometry, lanes, intersections, traffic rules
  - Native to CARLA simulator
  - Industry standard for driving simulators

### Autoware Map Requirements

Autoware requires **two separate map types**:

1. **Vector Map (Lanelet2)**
   - **Format**: .osm (OpenStreetMap XML)
   - **Purpose**: Semantic road network for path planning
   - **Contents**: Lanes, lane boundaries, traffic rules, regulatory elements
   - **Used by**: Mission planner, behavior planner, lane planner

2. **Point Cloud Map**
   - **Format**: .pcd (Point Cloud Data)
   - **Purpose**: 3D environment representation for localization
   - **Contents**: LiDAR scan of the environment
   - **Used by**: NDT scan matching for localization

## Pre-Converted Maps

The quickest way to start is using pre-converted maps:

### Official CARLA-Autoware Maps

**Repository**: https://bitbucket.org/carla-simulator/autoware-contents/src/master/

**Available maps**:
- Town01 through Town07
- Includes both Lanelet2 (.osm) and point cloud (.pcd) files

**Directory structure**:
```
CARLA/
├── HDMaps/
│   ├── Town01/
│   │   ├── Town01.pcd
│   │   └── Town01.osm
│   ├── Town02/
│   ...
```

**Known limitations**:
- Traffic light information is missing from Lanelet2 maps
- Some coordinate misalignment between vector and point cloud maps
- May require manual editing for production use

### TUMFTM Pre-Converted Maps

**Project**: https://github.com/TUMFTM/Carla-Autoware-Bridge

**Features**:
- Latest Autoware Core/Universe support
- CARLA 0.9.15 compatibility
- Published at IEEE IV 2024
- More recent and better maintained than official maps

## Map Conversion Workflow

### Method 1: Using assuremappingtools

**Tool**: [assuremappingtools](https://github.com/hatem-darweesh/assuremappingtools)

**Description**: Desktop application for viewing, editing, and converting road network maps

**Conversion steps**:

1. **Export OpenDRIVE from CARLA**:
   ```python
   # In CARLA Python API
   import carla
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   opendrive_map = world.get_map().to_opendrive()

   # Save to file
   with open('Town01.xodr', 'w') as f:
       f.write(opendrive_map)
   ```

2. **Convert using assuremappingtools**:
   ```bash
   # Install assuremappingtools
   git clone https://github.com/hatem-darweesh/assuremappingtools.git
   cd assuremappingtools
   # Follow build instructions in README

   # Convert OpenDRIVE to Lanelet2
   ./op_converter --input Town01.xodr --output Town01.osm --format lanelet2
   ```

3. **Generate point cloud map**:
   - Run CARLA with LiDAR sensor
   - Drive around the map to collect data
   - Use Autoware's `ndt_mapping` or similar tools to create .pcd file

4. **Manual editing** (required):
   - Use TIER IV Vector Map Builder
   - Fix coordinate alignment
   - Add traffic light information
   - Verify lane connections

### Method 2: Using CommonRoad Scenario Designer

**Tool**: [CommonRoad Scenario Designer](https://commonroad.in.tum.de/tools/scenario-designer)

**Description**: TUM toolbox for bidirectional format conversion (OpenDRIVE ↔ Lanelet2)

**Features**:
- GUI-based editing
- Bidirectional conversion
- Scenario validation

**Limitations**:
- Requires manual post-processing
- May not preserve all CARLA-specific features
- Traffic light information needs manual addition

**Usage**:
1. Install CommonRoad Scenario Designer
2. Import OpenDRIVE file
3. Edit and validate map
4. Export as Lanelet2 format
5. Manually add traffic lights and regulatory elements

### Method 3: Using carla-to-lanelet2 Script

**Project**: Part of various CARLA-Autoware bridge implementations

**Typical workflow**:
```bash
# Example from TUMFTM bridge
python3 scripts/convert_carla_map.py \
    --carla-host localhost \
    --carla-port 2000 \
    --map Town01 \
    --output Town01.osm
```

**Advantages**:
- Automated export from running CARLA
- Consistent coordinate system
- Minimal manual work

**Disadvantages**:
- Script-dependent quality
- May miss complex road features
- Traffic lights still need manual addition

## Point Cloud Map Generation

### Using CARLA LiDAR Sensor

1. **Spawn vehicle with LiDAR**:
   ```python
   # Attach LiDAR to vehicle
   lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
   lidar_bp.set_attribute('channels', '64')
   lidar_bp.set_attribute('points_per_second', '100000')
   lidar_bp.set_attribute('rotation_frequency', '10')
   lidar_bp.set_attribute('range', '100')

   lidar_sensor = world.spawn_actor(
       lidar_bp,
       carla.Transform(carla.Location(x=0.0, z=2.4)),
       attach_to=vehicle
   )
   ```

2. **Record point clouds**:
   ```python
   def lidar_callback(point_cloud):
       # Convert CARLA point cloud to ROS PointCloud2
       # Save to rosbag or directly to PCD
       pass

   lidar_sensor.listen(lidar_callback)
   ```

3. **Drive around the map**:
   - Manual control or autopilot
   - Cover all drivable areas
   - Collect multiple passes for better coverage

4. **Convert to PCD format**:
   ```bash
   # If using rosbag
   rosrun pcl_ros bag_to_pcd input.bag /lidar_topic output_dir

   # Merge all PCD files
   pcl_concatenate_points_pcd output_dir/*.pcd -o Town01.pcd
   ```

### Using Autoware's ndt_mapping

Alternative approach using Autoware's mapping tools:

1. Run CARLA with bridge
2. Launch Autoware mapping configuration
3. Drive vehicle around map
4. Save generated map

```bash
# Example launch command
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/maps/Town01 \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    use_sim_time:=true
```

## Coordinate System Alignment

### CARLA Coordinate System
- **X**: Forward (North)
- **Y**: Right (East)
- **Z**: Up
- **Units**: Meters
- **Origin**: Map-dependent

### Autoware Coordinate System
- **X**: East
- **Y**: North
- **Z**: Up
- **Units**: Meters
- **Origin**: Map-dependent (usually geodetic)

### Transformation Required

When converting between systems:

```python
# CARLA to Autoware coordinate transformation
def carla_to_autoware(carla_x, carla_y, carla_z):
    autoware_x = carla_y
    autoware_y = carla_x
    autoware_z = carla_z
    return (autoware_x, autoware_y, autoware_z)
```

**Note**: This transformation should be applied during map conversion, not at runtime in the bridge.

## Manual Editing Requirements

### Using TIER IV Vector Map Builder

**Tool**: https://tools.tier4.jp/feature/vector_map_builder_ll2/

**Required edits**:

1. **Traffic Light Addition**:
   - Identify traffic light positions from CARLA
   - Add regulatory elements to Lanelet2 map
   - Link traffic lights to relevant lanes

2. **Stop Line Verification**:
   - Check stop line positions
   - Adjust if misaligned with point cloud

3. **Lane Connection Fixes**:
   - Verify lane-to-lane connections
   - Fix any broken connections from conversion

4. **Speed Limit Assignment**:
   - Set appropriate speed limits for lanes
   - Match CARLA road attributes

5. **Right-of-Way Rules**:
   - Add yield/priority rules at intersections
   - Ensure compliance with traffic regulations

## Integration with autoware_carla_bridge

### Map File Setup

1. **Create map directory**:
   ```bash
   mkdir -p ~/autoware_maps/Town01
   cd ~/autoware_maps/Town01
   ```

2. **Place map files**:
   ```
   Town01/
   ├── lanelet2_map.osm          # Vector map
   └── pointcloud_map.pcd        # Point cloud map
   ```

3. **Update Autoware launch configuration**:
   ```yaml
   # Edit autoware.launch.xml or similar
   map_path: "/home/user/autoware_maps/Town01"
   ```

### Verification Steps

1. **Visualize in RViz**:
   - Check point cloud display
   - Verify Lanelet2 overlay alignment
   - Inspect traffic light positions

2. **Test localization**:
   - Set initial pose
   - Verify NDT scan matching converges
   - Check pose drift over time

3. **Test planning**:
   - Set navigation goal
   - Verify route planning works
   - Check lane following behavior

## Known Limitations and Workarounds

### Issue 1: Missing Traffic Lights

**Problem**: Auto-converted Lanelet2 maps don't include traffic lights

**Workaround**:
- Export traffic light positions from CARLA
- Manually add to Lanelet2 map using Vector Map Builder
- Alternatively, use CARLA's traffic light API directly in bridge

### Issue 2: Coordinate Misalignment

**Problem**: Vector map and point cloud don't align perfectly

**Workaround**:
- Use Vector Map Builder to manually adjust
- Re-generate point cloud with corrected poses
- Apply translation/rotation offset in Autoware launch

### Issue 3: Limited Reachable Positions

**Problem**: Map-based approach restricts valid spawn locations to roads

**Workaround**:
- Ensure CARLA spawn points are on-road
- Use CARLA's `get_spawn_points()` to find valid locations
- Add off-road areas to Lanelet2 map if needed

### Issue 4: Dynamic Object Representation

**Problem**: Static maps don't represent CARLA's dynamic actors

**Solution**:
- Use bridge to publish dynamic objects separately
- Autoware's perception module will track other vehicles
- Don't rely solely on map data for obstacle detection

## Alternative Approaches

### 1. Map-Free Approach

Instead of converting CARLA maps to Autoware format:
- Use only point cloud for localization
- Generate paths dynamically from CARLA waypoints
- Suitable for basic testing, not for full Autoware stack

### 2. Hybrid Approach

Combine CARLA and Autoware representations:
- Use CARLA map for simulation ground truth
- Generate minimal Lanelet2 map for Autoware planning
- Bridge translates between coordinate systems at runtime

### 3. Real-Time Map Generation

Generate Lanelet2 map on-the-fly from CARLA:
- Query CARLA waypoints and topology
- Convert to Lanelet2 format dynamically
- Eliminates pre-conversion step
- Higher computational cost

## Recommended Workflow

For most users, we recommend:

1. **Start with pre-converted maps** (TUMFTM or official)
2. **Test basic integration** before custom maps
3. **For custom maps**:
   - Use assuremappingtools for conversion
   - Generate point cloud with CARLA LiDAR
   - Manual editing with Vector Map Builder
4. **Verify thoroughly** before production use

## Tools and Resources

### Conversion Tools
- **assuremappingtools**: https://github.com/hatem-darweesh/assuremappingtools
- **CommonRoad**: https://commonroad.in.tum.de/tools/scenario-designer

### Map Repositories
- **Official CARLA-Autoware**: https://bitbucket.org/carla-simulator/autoware-contents
- **TUMFTM Bridge**: https://github.com/TUMFTM/Carla-Autoware-Bridge

### Editing Tools
- **Vector Map Builder**: https://tools.tier4.jp/feature/vector_map_builder_ll2/
- **JOSM** (for .osm editing): https://josm.openstreetmap.de/

### Documentation
- **Lanelet2 Format**: https://github.com/fzi-forschungszentrum-informatik/Lanelet2
- **OpenDRIVE Spec**: https://www.asam.net/standards/detail/opendrive/
- **Autoware Maps**: https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/

## Future Improvements

Potential enhancements for our bridge:

1. **Automated Map Conversion**:
   - Integrate conversion directly into bridge
   - Generate Lanelet2 from CARLA on startup
   - Export traffic light data automatically

2. **Dynamic Map Updates**:
   - Update Lanelet2 map when CARLA map changes
   - Handle map switching at runtime

3. **Improved Traffic Light Handling**:
   - Query CARLA traffic lights via API
   - Publish as ROS messages for Autoware
   - Eliminate need for manual traffic light addition

4. **Coordinate Frame Management**:
   - Automatic coordinate transformation
   - Unified frame handling in TF tree
   - Eliminate manual alignment step

## Conclusion

While CARLA to Autoware map integration requires manual work, the process is well-established. For initial testing, use pre-converted maps. For production use, invest time in proper conversion and editing to ensure accurate planning and localization.

---

**Last Updated**: 2025-11-07
**Related Documents**:
- `autoware-integration-design.md` - Overall bridge architecture
- `carla-rust-integration.md` - CARLA API usage
- `roadmap.md` - Project development plan
