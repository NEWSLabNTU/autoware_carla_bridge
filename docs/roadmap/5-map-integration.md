# Map Integration Roadmap

Track progress for CARLA map integration with Autoware, including Lanelet2 conversion and point cloud map generation.

**Reference Documents**:
- `docs/guides/automated-map-generation.md` - Complete map generation guide
- `docs/archive/carla-autoware-map-integration.md` - Map integration design
- TUMFTM pre-converted maps: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/
- Bitbucket pre-converted maps: https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/

**Local Conversion Tools** (in `src/external/`):
- `commonroad-scenario-designer/` - De facto standard converter (TUM, v0.8.5, Oct 2025)
- `odr2lanelet2/` - Direct xodr→osm with traffic light support (Jul 2024)
- `autoware_lanelet2_map_validator/` - Validates maps for Autoware compliance (v1.6.0, Jan 2026)

**Current Status**: 🟡 **PARTIALLY COMPLETE** - Pre-converted maps working, map generation tools designed but not yet implemented

---

## Map Strategy

**The bridge does not publish map topics.** Autoware loads maps from static files on disk via its own `map_loader`. Our job is to provide those files.

**Map sources** (per CARLA town, pick the best available):
1. **TUMFTM pre-converted maps** — manually curated, known working, available for Town01/02/03/05/10
2. **Our `carla_map_gen` / `carla_pcd_gen` tools** — automated, works for any CARLA map

**Approach**: Implement our tools based on CommonRoad, then compare generated maps against TUMFTM references. For each town, use whichever produces better results. For towns without TUMFTM coverage, our tools are the only option.

**Map files** (stored in `data/carla-autoware-bridge/<map_name>/`):
- `lanelet2_map.osm` — lane topology, traffic lights, regulatory elements
- `pointcloud_map.pcd` — 3D point cloud for NDT localization
- `map_config.yaml` — map origin (lat/lon/ele)
- `map_projector_info.yaml` — projection type (local)

---

## Progress Overview

### ✅ Completed

- ✅ Pre-converted maps downloaded for Town01, Town02, Town03, Town05, Town10
- ✅ Maps stored in `data/carla-autoware-bridge/` with full Autoware structure
- ✅ Helper scripts: `scripts/download_carla_maps_for_autoware.sh`, `scripts/find_map_offset.py`, `scripts/inspect_carla_map.py`
- ✅ End-to-end autonomous driving verified with pre-converted maps
- ✅ NDT localization works with point cloud maps
- ✅ Routing and planning work with lanelet2 maps
- ✅ CommonRoad Scenario Designer evaluated (v0.8.5, `autoware=True` produces TUMFTM-equivalent output)
- ✅ Conversion tools cloned to `src/external/` (CommonRoad, odr2lanelet2, validator)
- ✅ Map generation tools designed (`carla_map_gen` Python + `carla_pcd_gen` Rust)

### 🟡 In Progress

- 🔴 Implement `carla_map_gen` tool (Lanelet2 + post-processing + projection files)
- 🔴 Compare `carla_map_gen` output vs TUMFTM maps — identify and minimize gaps
- 🔴 Implement `carla_pcd_gen` tool (LiDAR collection + voxel downsample + PCD writer)
- 🔴 Test generated maps loading in Autoware
- 🔴 Build and run autoware_lanelet2_map_validator

---

## Quick Start with Pre-Converted Maps

**Objective**: Use TUMFTM's pre-converted maps to get running quickly

**Status**: ✅ **COMPLETE**

### 5.1 Download TUMFTM Maps - ✅ COMPLETE

Maps downloaded and stored in `data/carla-autoware-bridge/`:
- ✅ Town01 (lanelet2_map.osm + pointcloud_map.pcd + map_config.yaml + map_projector_info.yaml)
- ✅ Town02
- ✅ Town03
- ✅ Town05
- ✅ Town10

Download script: `scripts/download_carla_maps_for_autoware.sh`

### 5.2 Configure Autoware for TUMFTM Maps - ✅ COMPLETE

- ✅ Maps load in Autoware via `map_path` launch argument
- ✅ Maps display correctly in RViz
- ✅ NDT localization converges with point cloud maps

### 5.3 Validate Bridge with TUMFTM Maps - ✅ COMPLETE

- ✅ End-to-end autonomous driving verified
- ✅ Vehicle spawns correctly
- ✅ Sensor data aligns with map
- ✅ Localization accurate enough for autonomous driving

---

## Map Generation Tools

**Objective**: Provide standalone tools that create Autoware-compatible maps from a running CARLA server. Implement, compare against TUMFTM references, and use the better map source per town.

**Status**: 🟡 **DESIGNED** — Tools designed, implementation pending

**Priority**: 🟡 **MEDIUM** — Needed for maps beyond TUMFTM coverage, and to validate our pipeline against known-good references

### Architecture

Two standalone CLI tools that connect to a running CARLA server and produce the 4-file Autoware map structure:

```
                     ┌──────────────────────────────┐
                     │   CARLA Server (running)      │
                     └──────────┬───────────────────┘
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                      │
          ▼                     │                      ▼
  carla_map_gen (Python)        │          carla_pcd_gen (Rust)
  ───────────────────           │          ────────────────────
  1. Get OpenDRIVE via          │          1. Spawn vehicle + LiDAR
     carla.Client               │          2. Autopilot through spawn
  2. Convert via CommonRoad     │             points (~2-5 min)
     (autoware=True)            │          3. Transform LiDAR to world
  3. Post-process (fix TL       │          4. Voxel downsample
     subtypes, remove empty     │
     speed limits)              │
          │                     │                      │
          ▼                     │                      ▼
  lanelet2_map.osm              │          pointcloud_map.pcd
  map_config.yaml               │          map_config.yaml
  map_projector_info.yaml       │          map_projector_info.yaml
          │                     │                      │
          └─────────────────────┼──────────────────────┘
                                ▼
              data/carla-autoware-bridge/<map_name>/
```

Both tools auto-detect the map name from CARLA and output to `data/carla-autoware-bridge/<map_name>/`.

### Workflow

Map generation is a **one-time preparation step** per CARLA map. Once generated, maps are reused across sessions.

```bash
# Step 1: Start CARLA with the desired map
just carla start

# Step 2: Generate Autoware maps (one-time per CARLA map)
just generate-map              # Both Lanelet2 + PCD
just generate-lanelet2         # Lanelet2 only (~5s)
just generate-pcd              # PCD only (~2-5 min)

# Step 3: Stop CARLA (map generation complete)
just carla stop

# Step 4: Use generated maps with demo (any number of times)
MAP_NAME=Town01 just demo start
```

---

### 5.4 Tool: `carla_map_gen` (Python)

**Location**: `src/carla_map_gen/`

**Structure**:
```
src/carla_map_gen/
├── package.xml              # ament_python
├── setup.py                 # entry point: carla_map_gen
├── setup.cfg
├── resource/carla_map_gen
└── carla_map_gen/
    ├── __init__.py
    ├── generate.py          # Main entry point + CLI
    └── postprocess.py       # Fix TL subtypes, remove empty speed limits
```

**`generate.py` logic**:
1. Connect to CARLA via `carla.Client(host, port)`
2. Get OpenDRIVE XML: `world.get_map().to_opendrive()`
3. Extract map name (e.g. `Town01` from `/Game/Carla/Maps/Town01/Town01`)
4. Write OpenDRIVE to temp file → CommonRoad `opendrive_to_lanelet()` with `autoware=True`, `use_local_coordinates=True`
5. Post-process: fix empty TL `subtype` → `red_yellow_green`, remove empty `speed_limit` elements
6. Write `map_config.yaml` (all-zeros origin) and `map_projector_info.yaml` (`projector_type: local`)

**CLI**: `carla_map_gen [--host HOST] [--port PORT] [--output-dir DIR] [--project-dir DIR]`

**Dependencies**: `carla` (Python API), `commonroad-scenario-designer`, `lxml`

**Tasks**:
- [ ] Create package structure (package.xml, setup.py, etc.)
- [ ] Implement `generate.py` with CARLA connection + CommonRoad conversion
- [ ] Implement `postprocess.py` (TL subtype fix, speed limit removal)
- [ ] Add justfile recipe `generate-lanelet2`
- [ ] Compare output vs TUMFTM for Town01/02/03/05/10 — identify remaining gaps
- [ ] Iterate on post-processing to minimize differences
- [ ] Decide per town: use tool output or TUMFTM (whichever is better)

---

### 5.5 Tool: `carla_pcd_gen` (Rust)

**Location**: `src/carla_pcd_gen/`

**Structure**:
```
src/carla_pcd_gen/
├── Cargo.toml               # carla, nalgebra, clap, tracing, color-eyre
├── package.xml               # ament_cargo
└── src/
    ├── main.rs               # CLI (clap), orchestration
    ├── collector.rs           # LiDAR collection + world-frame transform
    ├── pcd_writer.rs          # PCD v0.7 binary file writer
    └── voxel_grid.rs          # HashMap-based voxel downsampling
```

**No ROS dependency** — standalone CARLA client binary. Users run this separately before starting Autoware.

**Key APIs from carla-rust** (`~/repos/carla-rust/`):
- `Client::connect(host, port, worker_threads)` — connect to CARLA
- `world.map().name()` — get map name
- `world.map().recommended_spawn_points()` — spawn point coverage
- `world.spawn_actor(bp, transform)` — spawn vehicle + LiDAR
- `Vehicle::set_autopilot(true)` — enable autopilot
- `Sensor::listen(callback)` — receive LiDAR data on separate thread
- `SensorDataBase::sensor_transform()` — world-frame transform at capture time
- `LidarMeasurement::as_slice() → &[LidarDetection]` — zero-copy point access
- `LidarDetection { point: FfiLocation { x, y, z: f32 }, intensity: f32 }`
- `ActorBase::set_transform()` — teleport vehicle between spawn points
- `Sensor::stop()` — stop listening

**Coverage strategy**: Sequential multi-spawn-point autopilot
- Get all recommended spawn points
- For each: teleport → enable autopilot → collect LiDAR for N seconds → next
- Configurable: `--spawn-points N` (0 = all), `--seconds-per-spawn N` (default: 10)

**Coordinate transform**: LiDAR points are sensor-local, must be transformed to world frame:
```rust
lidar.listen(move |data| {
    let sensor_tf = data.sensor_transform();  // world-frame Transform
    if let Ok(measurement) = LidarMeasurement::try_from(data) {
        // world_point = rotation(sensor_tf.rotation) * local_point + sensor_tf.location
        // Insert directly into voxel grid (online, caps memory)
    }
});
```

**Memory optimization**: Insert points directly into voxel grid `HashMap<(i32,i32,i32), (sum_xyz, count)>` during collection. This caps memory at ~200MB (voxel grid) instead of accumulating ~16GB of raw points.

**Voxel downsampling**: HashMap keyed by `(floor(x/res), floor(y/res), floor(z/res))`, value is running average. Default resolution: 0.1m.

**PCD output format**: PCD v0.7 binary, `FIELDS x y z intensity`, `DATA binary`. Autoware's `map_loader` supports this natively.

**CLI**:
```
carla_pcd_gen [--host HOST] [--port PORT] [--output-dir DIR] [--project-dir DIR]
              [--spawn-points N] [--seconds-per-spawn N] [--voxel-size F]
              [--lidar-range F] [--lidar-channels N]
```

**Tasks**:
- [ ] Create package structure + add to `Cargo.toml` workspace members
- [ ] Implement `main.rs` with clap CLI + orchestration
- [ ] Implement `collector.rs` with LiDAR listener + world-frame transform
- [ ] Implement `voxel_grid.rs` with online HashMap insertion
- [ ] Implement `pcd_writer.rs` with PCD v0.7 binary format
- [ ] Add justfile recipes `generate-pcd`, `generate-map`
- [ ] Test: generate PCD for Town01, load in Autoware, verify NDT localization

---

### 5.5.1 Map Quality Validation

**Objective**: Compare tool-generated maps against TUMFTM and decide which to use per town

**Comparison criteria**:
1. **Structural match** — lanelet count, TL count, regulatory element count
2. **Functional test** — load in Autoware, verify NDT localization, routing, autonomous driving
3. **Validator pass** — run `autoware_lanelet2_map_validator`
4. **PCD quality** — point density, NDT convergence rate

**Decision per town**: If our tool output matches or exceeds TUMFTM quality, use it. Otherwise, keep TUMFTM. For towns without TUMFTM coverage, our tool is the only source.

**Tasks**:
- [ ] Compare generated vs TUMFTM for Town01 (structural + functional)
- [ ] Compare generated vs TUMFTM for remaining towns
- [ ] Compare PCD density and NDT convergence
- [ ] Build and run `autoware_lanelet2_map_validator` on both sources
- [ ] Document which source is used per town and why

---

## Ecosystem Research (2026-03)

No fully automated, Autoware-ready pipeline exists in the ecosystem. All projects (TUMFTM, TIER IV, Autoware Foundation ODD WG) use the same approach: **automated conversion + manual post-processing**.

**Autoware ODD Working Group findings** (2025):
- Converted Lanelet2 maps lose speed limits, lane change flags, LHT/RHT info
- Map bloat: 16,000 points on a 1km straight road (causes Autoware planning failures)
- Proposal for native OpenDRIVE support in Autoware discussed but **not adopted**

**Available conversion tools** (evaluated):

| Tool                                  | Output          | Traffic Lights          | Status                    | Notes                              |
|---------------------------------------|-----------------|-------------------------|---------------------------|------------------------------------|
| **CommonRoad Scenario Designer**      | .osm (direct)   | No                      | Active (v0.8.5, Oct 2025) | De facto standard, used by TUMFTM  |
| **joel-mb/odr2lanelet2**              | .osm (direct)   | Yes (with CARLA server) | Low activity (Jul 2024)   | Only tool with regulatory elements |
| **usdot-fhwa-stol/opendrive2lanelet** | CommonRoad XML  | No                      | Active (Mar 2025)         | CARMA platform, extra step needed  |
| **GDAL 3.10 XODR driver**             | Any GDAL format | No                      | New (Sep 2024)            | Unvalidated for Autoware           |
| **TIER IV Vector Map Builder**        | .osm (manual)   | Yes (manual)            | Active (web tool)         | Post-processing / manual editing   |

**Key insight**: `odr2lanelet2` (in `src/external/`) is the **only tool** that extracts traffic light regulatory elements from CARLA. It uses `carla.Map` Python API for lane conversion and requires a running CARLA server for traffic lights (accesses actor component transforms for bulb positions).

### CommonRoad Evaluation Results

**Status**: ✅ **EVALUATED** — CommonRoad with `autoware=True` produces TUMFTM-equivalent output. This is the basis for our `carla_map_gen` tool.

**Town01** (CommonRoad `autoware=True` vs TUMFTM reference):
- ✅ **300 lanelets** (exact match)
- ✅ **43,154 nodes** with `local_x`/`local_y`, `mgrs_code`, `ele` (matches TUMFTM 43,082)
- ✅ **36 TL ways** with `height=1.2` and 2-node format (matches TUMFTM)
- ✅ **36 TL regulatory elements** with `refers` + `ref_line` members
- ⚠️ TL way `subtype` empty (CommonRoad bug: `light.color` not populated from OpenDRIVE)
- ⚠️ 36 speed_limit regulatory elements have zero members (empty shells)
- ⚠️ 88 lanelets missing subtype (walkways)
- ℹ️ `lane_change=no` added to all ways (TUMFTM removes these)

**Town10HD**: ✅ **391 lanelets** (matches TUMFTM)

**Known gaps** (to be addressed by post-processing in `carla_map_gen`):
1. TL way `subtype` empty → fix to `red_yellow_green` (canonical Autoware value from `lanelet2_core::Attribute.h:329`)
2. 36 empty speed_limit regulatory elements → remove
3. 88 walkway lanelets missing subtype → (optional) reclassify

---

### 5.7 Traffic Light Integration

**Objective**: Get traffic light regulatory elements into Lanelet2 maps

**Status**: 🟡 **PARTIALLY EVALUATED** - CommonRoad produces TL structure but with empty subtypes; odr2lanelet2 not yet tested

**Priority**: 🟢 **LOW** - Autoware can run without traffic lights

**Approach**: Use `odr2lanelet2` with `--carla` flag. It extracts:
- Traffic light box geometry (bottom_left, bottom_right, height)
- Individual bulb positions (green, yellow, red) as `light_bulbs` linestrings
- Stop lines at landmark positions
- Regulatory elements linking traffic lights to affected lanelets

**Requirement**: Running CARLA server (traffic light actors must be spawned to read component transforms)

**Fallback**: TIER IV Vector Map Builder (web, manual, ~30 min per town)

**Tasks**:
- [ ] Test odr2lanelet2 traffic light extraction on Town01
- [ ] Validate output against Autoware Lanelet2 format extension spec
- [ ] Test with Autoware traffic light recognition module
- [ ] Document limitations and manual corrections needed

**Testing**:
- [ ] Verify Autoware detects traffic lights from converted map
- [ ] Test traffic light state changes via bridge
- [ ] Validate stopping behavior at red lights

---

## Map Management & Tooling

**Objective**: Build infrastructure for managing and validating maps

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟢 **LOW** - Quality of life improvements

**Duration**: 1 week

### 5.9 Map Repository Structure

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

### 5.10 Map Validation Suite

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

### 5.11 Map Catalog & Documentation

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

## Advanced Map Features

**Status**: ⏳ **FUTURE**

**Priority**: 🟢 **LOW** - Not critical

### 5.12 Dynamic Map Elements

- [ ] Movable objects (traffic cones, barriers)
- [ ] Construction zones
- [ ] Temporary lane closures

### 5.13 Multi-Floor Maps

- [ ] Parking garages
- [ ] Multi-level interchanges
- [ ] Bridge/tunnel handling

### 5.14 HD Map Enhancements

- [ ] Lane markings (solid, dashed, etc.)
- [ ] Road surface types
- [ ] Speed limit zones

---

## Tools & Dependencies

### Map Generation Tools (in `src/`)

- **`carla_map_gen/`** - Python CLI tool for Lanelet2 + projection file generation
  - Depends on: `carla` (Python API), `commonroad-scenario-designer`, `lxml`
  - Entry point: `carla_map_gen` CLI
- **`carla_pcd_gen/`** - Rust CLI tool for PCD point cloud generation
  - Depends on: `carla` (carla-rust), `nalgebra`, `clap`, `tracing`, `color-eyre`
  - Entry point: `carla_pcd_gen` CLI (standalone, no ROS dependency)

### Python Packages

```bash
pip install commonroad-scenario-designer  # Lanelet2 conversion
pip install carla                          # CARLA Python API
pip install lxml                           # XML post-processing
```

### Local Reference Tools (in `src/external/`)

- **commonroad-scenario-designer/** - Source for understanding conversion internals
- **odr2lanelet2/** - Converter with traffic light support (requires CARLA server)
- **autoware_lanelet2_map_validator/** - Validates Lanelet2 maps for Autoware (C++, needs build)

### External Tools

- **TIER IV Vector Map Builder**: https://tools.tier4.jp/feature/vector_map_builder_ll2 (web, free)
- **CloudCompare**: Point cloud visualization and editing
- **JOSM**: OpenStreetMap editor (for Lanelet2 editing)

### Related Projects

- **tier4/carla-autoware-native**: CARLA fork with native Autoware integration + Map Editor GUI (active, Mar 2026)
- **evshary/autoware_carla_launch**: Multi-vehicle CARLA-Autoware using Zenoh (active, Feb 2026)
- **autoware_carla_interface**: Official Autoware CARLA integration in autoware_universe (active, Feb 2026)

---

## Automation Levels

Updated based on ecosystem research (2026-03):

| Task                                           | Automation                 | Tool                             | Status     |
|------------------------------------------------|----------------------------|----------------------------------|------------|
| **Point Cloud Generation**                     | ⭐⭐⭐⭐⭐ Fully Automated | `carla_pcd_gen` tool (Rust)      | Designed   |
| **OpenDRIVE → Lanelet2 (lanes + basic TL)**    | ⭐⭐⭐⭐ 90% Automated     | `carla_map_gen` tool (Python)    | Designed   |
| **OpenDRIVE → Lanelet2 (full TL regulatory)**  | ⭐⭐⭐ 60% Automated       | odr2lanelet2 + CARLA server      | Evaluated  |
| **Traffic Lights (manual fallback)**           | ⭐⭐ Manual                | TIER IV Vector Map Builder       | Available  |
| **Map Validation**                             | ⭐⭐⭐⭐⭐ Fully Automated | autoware_lanelet2_map_validator  | Cloned     |

---

## Progress Tracking

### Completion Status

| Phase                                | Status         | Completion | Priority  |
|--------------------------------------|----------------|------------|-----------|
| Phase 1: TUMFTM Maps                 | ✅ Complete    | 100%       | Done      |
| Phase 2: Map Gen Tools               | 🟡 Designed    | 20%        | 🟡 MEDIUM |
| Phase 3: Comparison with TUMFTM      | 🔴 Not Started | 0%         | 🟡 MEDIUM |
| Phase 4: Map Management              | 🔴 Not Started | 0%         | 🟢 LOW    |
| Phase 5: Advanced Features           | ⏳ Future      | 0%         | 🟢 LOW    |

**Overall Progress**: 40% complete (pre-converted maps working, CommonRoad evaluated, tools designed)

**Estimated Total Time**: 3-4 weeks (Phase 1-3 only)

---

## Recommended Workflow

### Option A: Use pre-converted TUMFTM maps

For towns with TUMFTM coverage (Town01/02/03/05/10), these are known to work:

```bash
just setup              # Downloads TUMFTM pre-converted maps
MAP_NAME=Town01 just demo start
```

### Option B: Generate maps with our tools

For towns without TUMFTM coverage, or when our tool produces better results:

**Preparation** (one-time per CARLA map):
```bash
# 1. Start CARLA with the desired map
just carla start

# 2. Generate Autoware maps
just generate-map       # Lanelet2 (~5s) + PCD (~2-5 min)

# 3. Stop CARLA when done
just carla stop
```

**Usage** (as many times as needed):
```bash
MAP_NAME=Town01 just demo start
```

### Choosing the map source

For each town, the map in `data/carla-autoware-bridge/<map_name>/` is what Autoware loads. Either populate it from TUMFTM or generate with our tools — whichever works better for that town.

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

### Phase 2 Success (Map Generation Tools)
- [ ] `just build` builds both `carla_map_gen` and `carla_pcd_gen`
- [ ] `just generate-lanelet2` produces valid Lanelet2 for any running CARLA map
- [ ] `just generate-pcd` produces PCD with sufficient density for NDT
- [ ] `just generate-map` produces complete 4-file Autoware map structure
- [ ] Generated maps work end-to-end with Autoware (routing + autonomous driving)

### Phase 3 Success (Comparison with TUMFTM)
- [ ] Tool output compared against TUMFTM for all 5 towns
- [ ] Remaining gaps identified and documented
- [ ] Post-processing minimizes differences (TL subtypes, empty speed limits, etc.)
- [ ] Per-town decision made: tool output vs TUMFTM
- [ ] Generated PCD quality sufficient for NDT localization

### Overall Success
- [ ] Every supported CARLA town has a working Autoware map (from tool or TUMFTM)
- [ ] User can generate maps for any CARLA map with `just generate-map`
- [ ] Process is documented and repeatable
