# Map Integration Roadmap

Track progress for CARLA map integration with Autoware, including Lanelet2 conversion and point cloud map generation.

**Reference Documents**:
- `docs/automated-map-generation.md` - Complete map generation guide
- `docs/carla-autoware-map-integration.md` - Map integration design
- TUMFTM pre-converted maps: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/

**Current Status**: 🔴 **NOT STARTED** - Research complete, implementation pending

---

## Progress Overview

### ✅ Completed Research

- ✅ Identified automated conversion tools (opendrive2lanelet, odr2lanelet2)
- ✅ Documented point cloud generation methods (autopilot recording, CARLA HDMaps)
- ✅ Analyzed TUMFTM's pre-converted maps
- ✅ Created comprehensive automation workflows
- ✅ Documented traffic light integration challenges

### 🔴 Not Started

- Map conversion automation scripts
- Point cloud generation scripts
- Traffic light configuration
- Map validation tools
- Integration with bridge

---

## Phase 1: Quick Start with Pre-Converted Maps

**Objective**: Use TUMFTM's pre-converted maps to get running quickly

**Status**: 🔴 **NOT STARTED**

**Priority**: 🔴 **HIGH** - Fastest path to testing

**Duration**: 1-2 days

**Why This First**: Allows us to test bridge functionality without spending weeks on map conversion

### 1.1 Download TUMFTM Maps

**Tasks**:
- [ ] Download TUMFTM maps from https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/
- [ ] Extract to `maps/tumftm/`
- [ ] Document available maps and their characteristics

**Expected Maps**:
- Town01.osm / Town01.pcd
- Town02.osm / Town02.pcd
- Town03.osm / Town03.pcd
- Town04.osm / Town04.pcd
- Town05.osm / Town05.pcd
- Town06.osm / Town06.pcd
- Town07.osm / Town07.pcd
- Town10HD.osm / Town10HD.pcd

**Deliverables**:
- Downloaded map files in project
- README documenting each map's characteristics
- Quick reference for which map to use for testing

---

### 1.2 Configure Autoware for TUMFTM Maps

**Tasks**:
- [ ] Create Autoware map configuration for each TUMFTM map
- [ ] Set up map paths in Autoware launch files
- [ ] Verify map loads in RViz
- [ ] Test localization with point cloud map

**Autoware Map Structure**:
```
autoware_map/
├── lanelet2_map.osm      # Vector map (from TUMFTM)
└── pointcloud_map.pcd    # Point cloud (from TUMFTM)
```

**Deliverables**:
- Autoware can load TUMFTM maps
- Maps display correctly in RViz
- Localization works with point cloud

**Testing**:
- [ ] Load each map in Autoware
- [ ] Verify all lane lines visible
- [ ] Check point cloud alignment
- [ ] Test NDT localization

---

### 1.3 Validate Bridge with TUMFTM Maps

**Tasks**:
- [ ] Test bridge with each TUMFTM map
- [ ] Verify vehicle spawns correctly
- [ ] Check sensor alignment with map
- [ ] Validate localization accuracy

**Deliverables**:
- Bridge works with all TUMFTM maps
- Documentation of any map-specific issues
- Recommendations for best maps for testing

---

## Phase 2: Automated Point Cloud Generation

**Objective**: Automate point cloud map creation from CARLA

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟡 **MEDIUM** - Needed for custom maps

**Duration**: 1 week

**Why After Phase 1**: TUMFTM maps are good enough for initial testing

### 2.1 Autopilot Recording Script

**Objective**: Create script to record point clouds using CARLA autopilot

**Tasks**:
- [ ] Port Python script from `docs/automated-map-generation.md`
- [ ] Add CLI parameters (town, duration, output path)
- [ ] Implement autopilot-based recording
- [ ] Merge and downsample point clouds
- [ ] Save to PCD format

**Script Design**:
```bash
# Usage
./scripts/generate_pcd.py \
    --town Town10HD \
    --duration 300 \
    --output maps/custom/Town10HD_custom.pcd \
    --lidar-channels 64 \
    --points-per-second 1000000
```

**Implementation** (`scripts/generate_pcd.py`):
```python
#!/usr/bin/env python3
"""
Automated point cloud map generation for CARLA towns.

Uses CARLA autopilot to drive through the entire map while recording
LiDAR data, then merges into a single PCD file for Autoware localization.
"""

import carla
import numpy as np
import open3d as o3d
import argparse
from tqdm import tqdm

def generate_point_cloud(town, duration, output_path, **lidar_params):
    """Generate point cloud map for a CARLA town."""
    # Implementation from automated-map-generation.md
    # ... (full script documented in that file)
```

**Deliverables**:
- Working PCD generation script
- Can generate maps for any CARLA town
- Output compatible with Autoware NDT

**Testing**:
- [ ] Generate PCD for Town01 (simple)
- [ ] Generate PCD for Town10HD (complex)
- [ ] Compare with TUMFTM maps for quality
- [ ] Test localization with generated maps

---

### 2.2 Map Quality Validation

**Objective**: Ensure generated point clouds are suitable for NDT localization

**Tasks**:
- [ ] Implement density analysis
- [ ] Check for gaps/holes in coverage
- [ ] Validate ground plane detection
- [ ] Compare against reference maps

**Validation Metrics**:
- Point density (points/m²)
- Coverage percentage
- Ground plane flatness
- Localization convergence rate

**Tools**:
- CloudCompare for visual inspection
- Custom scripts for automated validation

**Deliverables**:
- Validation tool for PCD quality
- Report template for map characteristics
- Pass/fail criteria for maps

---

## Phase 3: Lanelet2 Map Conversion

**Objective**: Automate OpenDRIVE to Lanelet2 conversion

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟡 **MEDIUM** - Needed for custom maps

**Duration**: 1-2 weeks

**Challenge**: Traffic light integration requires manual editing

### 3.1 Install Conversion Tools

**Tasks**:
- [ ] Install opendrive2lanelet (CommonRoad/TUM)
- [ ] Install dependencies (Python packages)
- [ ] Test with CARLA Town01
- [ ] Verify output format

**Installation**:
```bash
pip install opendrive2lanelet lxml numpy scipy matplotlib commonroad-io
```

**Deliverables**:
- opendrive2lanelet installed and working
- Tested on sample CARLA map

---

### 3.2 Automated Conversion Script

**Objective**: Script to convert any CARLA town to Lanelet2

**Tasks**:
- [ ] Export OpenDRIVE from CARLA
- [ ] Run opendrive2lanelet conversion
- [ ] Post-process OSM file (fix common issues)
- [ ] Validate output with Autoware

**Script Design**:
```bash
# Usage
./scripts/convert_carla_map.py \
    --town Town10HD \
    --output maps/custom/Town10HD.osm \
    --fix-traffic-lights
```

**Implementation** (`scripts/convert_carla_map.py`):
```python
#!/usr/bin/env python3
"""
Automated CARLA to Lanelet2 map conversion.

Exports OpenDRIVE from CARLA, converts to Lanelet2 using opendrive2lanelet,
and applies post-processing fixes for Autoware compatibility.
"""

import carla
from opendrive2lanelet.io import opendrive_to_lanelet
import argparse

def convert_carla_town(town, output_path, **options):
    """Convert CARLA town to Lanelet2 format."""
    # 1. Connect to CARLA and get map
    # 2. Export OpenDRIVE
    # 3. Convert with opendrive2lanelet
    # 4. Post-process OSM
    # 5. Validate
```

**Deliverables**:
- Automated conversion script
- Can convert any CARLA town
- Basic validation included

**Testing**:
- [ ] Convert Town01 and compare with TUMFTM
- [ ] Convert Town10HD (complex map)
- [ ] Load in Autoware and verify lane detection
- [ ] Check routing functionality

---

### 3.3 Traffic Light Integration

**Objective**: Add traffic light regulatory elements to Lanelet2 maps

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟢 **LOW** - Autoware can run without traffic lights

**Challenge**: opendrive2lanelet doesn't preserve traffic lights well

**Tasks**:
- [ ] Research traffic light data in CARLA
- [ ] Find traffic light positions from CARLA API
- [ ] Add regulatory elements to OSM manually or via script
- [ ] Test with Autoware traffic light recognition

**CARLA Traffic Light API**:
```python
# Get all traffic lights in world
traffic_lights = world.get_actors().filter('traffic.traffic_light')

for tl in traffic_lights:
    location = tl.get_location()
    state = tl.get_state()  # Red, Yellow, Green
    # Map to Lanelet2 regulatory element
```

**Manual Editing** (if automation fails):
- Use TIER IV Vector Map Builder
- Web interface: https://tools.tier4.jp/vector_map_builder/
- Add traffic light regulatory elements manually

**Deliverables**:
- Traffic light positions extracted from CARLA
- Script or process to add to Lanelet2
- Working traffic light detection in Autoware

**Testing**:
- [ ] Verify Autoware detects traffic lights
- [ ] Test traffic light state changes
- [ ] Validate stopping at red lights

---

## Phase 4: Map Management & Tooling

**Objective**: Build infrastructure for managing and validating maps

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟢 **LOW** - Quality of life improvements

**Duration**: 1 week

### 4.1 Map Repository Structure

**Design**:
```
maps/
├── tumftm/          # Pre-converted TUMFTM maps
│   ├── Town01/
│   │   ├── lanelet2_map.osm
│   │   └── pointcloud_map.pcd
│   ├── Town10HD/
│   │   ├── lanelet2_map.osm
│   │   └── pointcloud_map.pcd
│   └── ...
├── custom/          # Our generated maps
│   ├── Town01_v2/
│   └── ...
└── README.md        # Map catalog
```

**Tasks**:
- [ ] Create directory structure
- [ ] Add .gitignore for large map files
- [ ] Document map characteristics
- [ ] Create map selection guide

---

### 4.2 Map Validation Suite

**Objective**: Automated validation of map quality

**Tasks**:
- [ ] Check OSM syntax
- [ ] Validate PCD format
- [ ] Check map alignment (OSM ↔ PCD)
- [ ] Verify required Lanelet2 elements

**Validation Script**:
```bash
./scripts/validate_map.py maps/custom/Town01/
# Output:
# ✓ OSM file valid
# ✓ PCD file valid
# ✓ Map alignment OK
# ✗ Missing traffic light regulatory elements
# Score: 75/100
```

**Deliverables**:
- Validation script
- Report generation
- Pass/fail criteria

---

### 4.3 Map Catalog & Documentation

**Tasks**:
- [ ] Document each available map
- [ ] List map characteristics (size, complexity, features)
- [ ] Provide usage recommendations
- [ ] Include screenshots

**Map Catalog** (`maps/README.md`):
```markdown
# Available Maps

## TUMFTM Pre-Converted

### Town01 (Simple Urban)
- **Size**: 400m x 400m
- **Lanes**: ~50 lane segments
- **Traffic Lights**: 12
- **Complexity**: Low
- **Use For**: Basic testing, simple scenarios

### Town10HD (Complex Highway)
- **Size**: 12.3km x 1.5km
- **Lanes**: ~500 lane segments
- **Traffic Lights**: 48
- **Complexity**: High
- **Use For**: Highway scenarios, performance testing
```

---

## Phase 5: Advanced Map Features

**Status**: ⏳ **FUTURE**

**Priority**: 🟢 **LOW** - Not critical

### 5.1 Dynamic Map Elements

- [ ] Movable objects (traffic cones, barriers)
- [ ] Construction zones
- [ ] Temporary lane closures

### 5.2 Multi-Floor Maps

- [ ] Parking garages
- [ ] Multi-level interchanges
- [ ] Bridge/tunnel handling

### 5.3 HD Map Enhancements

- [ ] Lane markings (solid, dashed, etc.)
- [ ] Road surface types
- [ ] Speed limit zones

---

## Tools & Dependencies

### Python Packages

```bash
# Map conversion
pip install opendrive2lanelet lxml numpy scipy matplotlib commonroad-io

# Point cloud processing
pip install open3d numpy

# CARLA Python API
pip install carla  # or use CARLA's PythonAPI
```

### External Tools

- **TIER IV Vector Map Builder**: https://tools.tier4.jp/vector_map_builder/
- **CloudCompare**: Point cloud visualization and editing
- **JOSM**: OpenStreetMap editor (for Lanelet2 editing)

---

## Automation Levels

Based on `docs/automated-map-generation.md`:

| Task | Automation | Tool | Effort |
|------|------------|------|--------|
| **Point Cloud Generation** | ⭐⭐⭐⭐⭐ Fully Automated | Python + CARLA autopilot | 1 day |
| **OpenDRIVE → Lanelet2** | ⭐⭐⭐⭐ 80% Automated | opendrive2lanelet | 2 days |
| **Traffic Lights** | ⭐⭐ 40% Automated | Manual editing in Vector Map Builder | 1 week |
| **Map Validation** | ⭐⭐⭐⭐ Automated | Custom scripts | 3 days |

---

## Progress Tracking

### Completion Status

| Phase | Status | Completion | Priority |
|-------|--------|------------|----------|
| Phase 1: TUMFTM Maps | 🔴 Not Started | 0% | 🔴 HIGH |
| Phase 2: PCD Generation | 🔴 Not Started | 0% | 🟡 MEDIUM |
| Phase 3: Lanelet2 Conversion | 🔴 Not Started | 0% | 🟡 MEDIUM |
| Phase 4: Map Management | 🔴 Not Started | 0% | 🟢 LOW |
| Phase 5: Advanced Features | ⏳ Future | 0% | 🟢 LOW |

**Overall Progress**: 0% complete

**Estimated Total Time**: 3-4 weeks (Phase 1-3 only)

---

## Recommended Workflow

**Quick Start** (1-2 days):
1. Download TUMFTM maps (Phase 1.1)
2. Test with Autoware (Phase 1.2)
3. Validate bridge works (Phase 1.3)
4. Start testing scenarios

**Custom Maps** (2-3 weeks):
1. Set up PCD generation (Phase 2.1)
2. Generate PCDs for needed towns (Phase 2.2)
3. Convert OpenDRIVE to Lanelet2 (Phase 3.1-3.2)
4. Manual traffic light editing if needed (Phase 3.3)
5. Validate maps (Phase 2.2 + 4.2)

**Long Term** (ongoing):
1. Build map catalog (Phase 4.3)
2. Improve automation (Phase 3.3, 5.x)
3. Contribute improvements back to tools

---

## Risks & Mitigation

### High Risk

1. **Traffic light integration is manual**
   - **Impact**: Time-consuming for each map
   - **Mitigation**: Start with maps without traffic light scenarios, use TUMFTM maps

2. **Map quality varies by tool**
   - **Impact**: Some maps may not work well with Autoware
   - **Mitigation**: Thorough validation, compare with TUMFTM references

### Medium Risk

1. **Large file sizes**
   - **Impact**: PCD files can be 1-2 GB each
   - **Mitigation**: Use Git LFS, provide download links

2. **Tool compatibility**
   - **Impact**: opendrive2lanelet may not work with all OpenDRIVE versions
   - **Mitigation**: Test with multiple CARLA versions, document compatibility

### Low Risk

1. **Map updates**
   - **Impact**: CARLA map changes require regeneration
   - **Mitigation**: Automation makes regeneration easy

---

## Success Criteria

### Phase 1 Success
- [ ] Can load and use TUMFTM maps in Autoware
- [ ] Bridge works with all TUMFTM maps
- [ ] Localization works reliably
- [ ] Can run basic scenarios

### Phase 2 Success
- [ ] Can generate PCD for any CARLA town
- [ ] Generated PCDs work with NDT localization
- [ ] Quality comparable to TUMFTM maps
- [ ] Process is fully automated

### Phase 3 Success
- [ ] Can convert any CARLA town to Lanelet2
- [ ] Autoware can load and use converted maps
- [ ] Lane-following works correctly
- [ ] Routing works between waypoints

### Overall Success
- [ ] Have working maps for common CARLA towns
- [ ] Can generate new maps in < 1 day
- [ ] Maps support all Autoware features
- [ ] Process is documented and repeatable

---

**Last Updated**: 2025-11-08
**Next Review**: After Phase 1 completion
**Owner**: Integration Team
