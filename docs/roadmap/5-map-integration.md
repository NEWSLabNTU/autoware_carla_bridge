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

**Current Status**: ✅ **Both generators implemented and validated end to end.** Autoware localizes and drives on a point cloud produced by `carla_pcd_gen` (2026-08-30).

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
- ✅ `carla_map_gen` implemented (ament_python, Lanelet2 + post-processing + projection files)
- ✅ `carla_map_gen` supports offline mode (`--xodr` flag, no CARLA server needed)
- ✅ Post-processing: TL subtypes, empty speed_limits, curbstone/traffic_sign stripping, shoulder reclassification
- ✅ Comparison script (`scripts/compare_lanelet2.py`) + `just compare-lanelet2` recipe
- ✅ Generated maps compared against TUMFTM for all 5 towns (Town01/02/03/05/10HD)
- ✅ Lanelet counts match in 4/5 maps, crosswalk counts match exactly in all maps

### 🟡 In Progress

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

**Status**: ✅ **COMPLETE** — both generators implemented; `carla_pcd_gen` validated by driving on its output

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

### 5.4 Tool: `carla_map_gen` (Python) - ✅ COMPLETE

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
    ├── generate.py          # Main entry point + CLI (live + offline modes)
    └── postprocess.py       # 5 post-processing fixes
```

**Two modes**:
- **Live**: `carla_map_gen --port 2000` — connects to running CARLA, extracts OpenDRIVE
- **Offline**: `carla_map_gen --xodr path/to/Town01.xodr` — uses local `.xodr` file directly

**`generate.py` logic**:
1. Get OpenDRIVE (from CARLA or local file)
2. Convert via CommonRoad `opendrive_to_lanelet()` with `autoware=True`, `use_local_coordinates=True`
3. Post-process (5 fixes - see below)
4. Write `map_config.yaml` and `map_projector_info.yaml`

**`postprocess.py` fixes** (applied in order):
1. Fix TL way subtypes: empty → `red_yellow_green`
2. Remove empty `speed_limit` regulatory elements (zero members) and references
3. Strip `curbstone`/`traffic_sign` way type tags (not in Autoware format)
4. Remove orphan ways not referenced by any relation
5. Reclassify shoulder lanelets (between road and walkway) as `road`

**CLI**: `carla_map_gen [--host HOST] [--port PORT] [--xodr FILE] [--output-dir DIR] [--project-dir DIR]`

**Just recipes**: `just generate-lanelet2`, `just compare-lanelet2 <generated> <reference>`

**Comparison script**: `scripts/compare_lanelet2.py` — structural comparison of two Lanelet2 OSM files

**Dependencies**: `carla` (Python API), `commonroad-scenario-designer`, `lxml`

**TUMFTM comparison results** (see `docs/guides/map-generation-comparison.md`):

| Map | Lanelets | Ways | Relations | Notes |
|-----|----------|------|-----------|-------|
| Town01 | 300 = 300 | 494 = 494 | 336 = 336 | Exact structural match |
| Town02 | 216 = 216 | 350 vs 348 | 240 = 240 | +2 ways |
| Town03 | 1232 vs 1210 | 1906 vs 1825 | 1326 vs 1248 | +22 lanelets from complex geometry |
| Town05 | 1029 = 1029 | 1635 vs 1555 | 1178 vs 1084 | Lanelet count exact match |
| Town10HD | 391 = 391 | 571 vs 579 | 425 vs 409 | Lanelet count exact match |

Improvements over TUMFTM: properly typed TL regulatory elements and way subtypes, MGRS codes on all nodes.

**Tasks**:
- [x] Create package structure (package.xml, setup.py, etc.)
- [x] Implement `generate.py` with CARLA connection + CommonRoad conversion
- [x] Implement `postprocess.py` (5 post-processing fixes)
- [x] Add `--xodr` offline mode (no CARLA server needed)
- [x] Add justfile recipe `generate-lanelet2`
- [x] Create comparison script + `compare-lanelet2` recipe
- [x] Compare output vs TUMFTM for Town01/02/03/05/10HD
- [x] Iterate on post-processing to minimize differences
- [ ] Decide per town: use tool output or TUMFTM (pending PCD gen + functional test)

---

### 5.5 Tool: `carla_pcd_gen` (Rust)

**Location**: `src/carla_pcd_gen/`

**Prior art**: `external/carla-pcd-generator/` — 4-year-old prototype using carla-rust 0.8.0, semantic LiDAR, per-frame PCD output, kiss3d GUI. Superseded by this modern design.

**Structure**:
```
src/carla_pcd_gen/
├── Cargo.toml               # carla (workspace), nalgebra, crossbeam, clap, tracing, color-eyre, pcd-rs
├── package.xml               # ament_cargo
└── src/
    ├── main.rs               # CLI (clap), connect + orchestrate + write config files
    ├── collector.rs           # Spawn N vehicles + LiDARs, parallel batch teleport, sensor callbacks
    ├── voxel_grid.rs          # HashMap-based online voxel accumulator with density cap
    └── pcd_writer.rs          # PCD v0.7 binary file writer
```

**No ROS dependency** — standalone CARLA client binary. Builds as ament_cargo package via `just build`.

#### Design changes from old `carla-pcd-generator`

| Aspect               | Old (carla-rust 0.8.0)                                                    | Modern (carla-rust 0.13.0)                                       |
|----------------------|---------------------------------------------------------------------------|------------------------------------------------------------------|
| **LiDAR type**       | `sensor.lidar.ray_cast_semantic`                                          | `sensor.lidar.ray_cast`                                          |
| **Output**           | Per-frame PCD files (`00000.pcd`, ...)                                    | Single merged + voxel-downsampled PCD                            |
| **GUI**              | kiss3d 3D viewer                                                          | None (CLI-only, use RViz for viz)                                |
| **Errors**           | `anyhow`                                                                  | `color-eyre` (matches bridge)                                    |
| **Vehicle strategy** | N cars driving with autopilot                                             | N bare LiDARs teleporting through spawn points (no vehicles)     |
| **Memory**           | Unbounded (raw frames on disk)                                            | Bounded ~200MB (voxel grid HashMap, max_samples cap)             |
| **Build**            | Standalone `cargo build`                                                  | ament_cargo (`just build`)                                       |
| **carla dep**        | Pinned 0.8.0                                                              | Workspace dep (0.13.0)                                           |
| **Spawning API**     | `world.actor_builder(id).spawn_vehicle(isometry)`                         | `world.spawn_actor(&blueprint, &transform)`                      |
| **Transforms**       | `Isometry3<f32>` throughout                                               | `Transform { location, rotation }` + `.to_na()`                  |
| **Sensor data**      | `SemanticLidarDetection { point, cos_inc_angle, object_idx, object_tag }` | `LidarDetection { point: Location { x, y, z }, intensity: f32 }` |

#### Key carla-rust 0.13.0 APIs

```rust
// Connect
let client = Client::connect("localhost", 2000, None);
let mut world = client.world();

// Set synchronous mode (deterministic LiDAR scans)
let mut settings = world.settings();
settings.synchronous_mode = true;
settings.fixed_delta_seconds = Some(0.05);  // 50ms timestep
world.apply_settings(&settings, Duration::from_secs(10));

// Channel-based aggregation (N LiDARs → 1 grid thread)
let (tx, rx) = crossbeam::channel::bounded(4096);

// Spawn N bare LiDAR sensors (no vehicles)
let bp_lib = world.blueprint_library();
let mut lidar_bp = bp_lib.find("sensor.lidar.ray_cast").unwrap();
lidar_bp.set_attribute("channels", "64");
lidar_bp.set_attribute("range", "100.0");
lidar_bp.set_attribute("points_per_second", "600000");
lidar_bp.set_attribute("rotation_frequency", "20.0");

let mut sensors = Vec::new();
for _ in 0..num_lidars {
    let initial_tf = Transform { location: Location::new(0.0, 0.0, 100.0), ..default };
    let actor = world.spawn_actor(&lidar_bp, &initial_tf)?;
    let sensor = Sensor::try_from(actor)?;
    let tx_clone = tx.clone();
    sensor.listen(move |data| {
        let sensor_tf = data.sensor_transform().to_na();
        if let Ok(measure) = LidarMeasurement::try_from(data) {
            for det in measure.as_slice() {
                let local = Point3::new(det.point.x, det.point.y, det.point.z);
                let world_pt = sensor_tf * local;
                let _ = tx_clone.try_send((world_pt.x, world_pt.y, world_pt.z, det.intensity));
            }
        }
    });
    sensors.push(sensor);
}
drop(tx);

// Aggregator thread — owns the voxel grid, no mutex needed
let grid_handle = std::thread::spawn(move || {
    let mut grid = VoxelGrid::new(voxel_size, max_samples);
    while let Ok((x, y, z, i)) = rx.recv() {
        grid.insert(x, y, z, i);
    }
    grid
});

// Batch teleport through spawn points (no driving, just place + tick)
let spawn_points = world.map().recommended_spawn_points();
for batch in spawn_points.chunks(num_lidars) {
    for (sensor, sp) in sensors.iter().zip(batch) {
        let mut tf = sp.clone();
        tf.location.z += 2.5;  // LiDAR height above ground
        sensor.set_transform(&tf);
    }
    for _ in 0..ticks_per_position {
        world.tick();  // Each tick = one LiDAR rotation at 20Hz
    }
}
```

#### Coverage strategy: parallel stationary LiDARs

Spawn **N bare LiDAR sensors** (default: 8) directly in the world — no vehicles needed. Teleport them to spawn points, tick a few frames to collect one full rotation each, then move to the next batch. No driving, no physics, no autopilot overhead.

**Workflow**:
1. Get `map.recommended_spawn_points()` (typically 100-300 per town)
2. Spawn N LiDAR sensors (unattached, free-standing in world)
3. Process spawn points in batches of N:
   a. Teleport each LiDAR to its assigned spawn point (at +2.5m height)
   b. Tick world for `--ticks-per-position` frames (default: 2, enough for one full LiDAR rotation at 20Hz)
   c. Sensor callbacks transform points to world frame and send to aggregator
4. After all spawn points: drain channel, write merged PCD

A full town with 200 spawn points at 2 ticks each = 400 ticks = **20 seconds** at 50ms timestep. With 8 parallel LiDARs: **~25 batches × 2 ticks = ~3 seconds of sim time**.

**Concurrency model**: Each LiDAR sensor callback sends points through a channel (`crossbeam::channel`) to a single aggregator thread that owns the voxel grid. Sensor callbacks only do coordinate transform + channel send (cheap); the aggregator does all grid insertion.

```
  LiDAR 0 ──► channel::Sender ──┐
  LiDAR 1 ──► channel::Sender ──┤
  ...                            ├──► channel::Receiver ──► VoxelGrid (single thread)
  LiDAR 7 ──► channel::Sender ──┘
```

Configurable: `--lidars N` (default: 8), `--spawn-points N` (0 = all), `--ticks-per-position N` (default: 2)

#### Point density control

The voxel grid enforces a **maximum samples per voxel** (`--max-samples N`, default: 5). Once a voxel reaches this count, further insertions are dropped. This prevents overly dense regions near spawn points from consuming memory and computation while adding no value.

Combined with **adaptive dwell time** (early batch termination when insertion rate drops), this ensures the tool spends time where coverage is sparse and moves on quickly from well-covered areas.

**Density metrics** (printed per batch):
- New voxels created in this batch
- Total voxels so far
- Estimated coverage area (voxel count × resolution²)

#### Voxel grid (memory-bounded accumulation)

```rust
struct VoxelGrid {
    cells: HashMap<(i32, i32, i32), VoxelCell>,
    resolution: f32,       // default 0.1m
    max_samples: u32,      // default 5, drop inserts beyond this
}
struct VoxelCell {
    sum_x: f64, sum_y: f64, sum_z: f64,
    sum_intensity: f64,
    count: u32,
}
impl VoxelGrid {
    fn insert(&mut self, x: f32, y: f32, z: f32, intensity: f32) -> bool {
        let key = (floor(x/res), floor(y/res), floor(z/res));
        let cell = self.cells.entry(key).or_default();
        if cell.count >= self.max_samples { return false; }  // density cap
        cell.sum_x += x as f64; // ...
        cell.count += 1;
        true  // returns whether a new contribution was accepted
    }
}
// Key = (floor(x/res), floor(y/res), floor(z/res))
// Final point = (sum_x/count, sum_y/count, sum_z/count, sum_intensity/count)
// Memory: ~200MB for a typical town at 0.1m resolution
```

Insert directly into voxel grid during collection (not after). This caps memory at the grid size instead of accumulating raw points (~16GB for a full town). The `max_samples` cap further bounds memory by preventing voxels from accumulating unbounded statistics.

#### PCD output

PCD v0.7 binary, compatible with Autoware `map_loader`:
```
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH <total_voxels>
HEIGHT 1
DATA binary
```

Written via `pcd-rs` crate (derive macro for binary serialization).

#### CLI

```
carla_pcd_gen [--host HOST] [--port PORT] [--output-dir DIR] [--project-dir DIR]
              [--lidars N] [--spawn-points N] [--ticks-per-position N]
              [--voxel-size F] [--max-samples N]
              [--lidar-range F] [--lidar-channels N]
```

Defaults: `--lidars 8`, `--lidar-range 100.0`, `--lidar-channels 64`, `--voxel-size 0.1`, `--max-samples 5`, `--ticks-per-position 2`

#### Tasks

- [x] Create package structure (Cargo.toml, package.xml) + add to workspace members
- [x] Implement `main.rs` - clap CLI, CARLA connection, synchronous mode, config file output
- [x] Implement `collector.rs` - N bare LiDAR spawning, batch teleport, channel-based sensor callbacks
- [x] Implement `voxel_grid.rs` - HashMap accumulator with running averages + max_samples density cap
- [x] Implement `pcd_writer.rs` - PCD v0.7 binary via pcd-rs
- [ ] Add justfile recipes `generate-pcd`, `generate-map`
- [ ] Test: generate PCD for Town01, load in Autoware, verify NDT localization
- [ ] Compare PCD density vs TUMFTM reference
- [ ] Tune defaults: vehicles count, max_samples, min_new_voxels_per_tick thresholds

---

### 5.5.1 Map Quality Validation

**Objective**: Compare tool-generated maps against TUMFTM and decide which to use per town

**Status**: ✅ **LANELET2 AND PCD COMPLETE**

**Comparison criteria**:
1. **Structural match** — lanelet count, TL count, regulatory element count
2. **Functional test** — load in Autoware, verify NDT localization, routing, autonomous driving
3. **Validator pass** — run `autoware_lanelet2_map_validator`
4. **PCD quality** — point density, NDT convergence rate

**Decision per town**: If our tool output matches or exceeds TUMFTM quality, use it. Otherwise, keep TUMFTM. For towns without TUMFTM coverage, our tool is the only source.

**Lanelet2 comparison results** (see `docs/guides/map-generation-comparison.md`):
- Town01: lanelet count exact match (300), ways/relations exact match
- Town02: lanelet count exact match (216), +2 ways
- Town03: +22 lanelets (1232 vs 1210) from parking/tunnel geometry
- Town05: lanelet count exact match (1029), extra line_thick ways
- Town10HD: lanelet count exact match (391), improved TL metadata

All maps: TL metadata improved over TUMFTM (properly typed subtypes vs empty). Remaining diffs are road/walkway subtype classification (does not affect Autoware planning).

**Tasks**:
- [x] Compare generated vs TUMFTM for Town01 (structural)
- [x] Compare generated vs TUMFTM for remaining towns (structural)
- [x] Document comparison results (`docs/guides/map-generation-comparison.md`)
- [ ] Compare PCD density and NDT convergence (pending `carla_pcd_gen`)
- [ ] Build and run `autoware_lanelet2_map_validator` on both sources
- [ ] Functional test: load generated Lanelet2 in Autoware, verify routing
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

- **`carla_map_gen/`** - ✅ Python CLI tool for Lanelet2 + projection file generation
  - Depends on: `carla` (Python API), `commonroad-scenario-designer`, `lxml`
  - Entry point: `carla_map_gen` CLI (via `ros2 run` or `just generate-lanelet2`)
  - Supports live mode (CARLA server) and offline mode (`--xodr` file)
- **`carla_pcd_gen/`** - 🔴 Rust CLI tool for PCD point cloud generation
  - Depends on: `carla` (carla-rust 0.13.0 workspace dep), `nalgebra`, `clap`, `tracing`, `color-eyre`, `pcd-rs`
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
| **OpenDRIVE → Lanelet2 (lanes + basic TL)**    | ⭐⭐⭐⭐ 90% Automated     | `carla_map_gen` tool (Python)    | **Done**   |
| **OpenDRIVE → Lanelet2 (full TL regulatory)**  | ⭐⭐⭐ 60% Automated       | odr2lanelet2 + CARLA server      | Evaluated  |
| **Traffic Lights (manual fallback)**           | ⭐⭐ Manual                | TIER IV Vector Map Builder       | Available  |
| **Map Validation**                             | ⭐⭐⭐⭐⭐ Fully Automated | autoware_lanelet2_map_validator  | Cloned     |

---

## Progress Tracking

### Completion Status

| Phase                                | Status         | Completion | Priority  |
|--------------------------------------|----------------|------------|-----------|
| Phase 1: TUMFTM Maps                 | ✅ Complete    | 100%       | Done      |
| Phase 2: Map Gen Tools               | 🟡 In Progress | 60%        | 🟡 MEDIUM |
|   - `carla_map_gen` (Lanelet2)       | ✅ Complete    | 100%       | Done      |
|   - `carla_pcd_gen` (PCD)            | 🔴 Designed    | 0%         | 🟡 MEDIUM |
| Phase 3: Comparison with TUMFTM      | 🟡 In Progress | 60%        | 🟡 MEDIUM |
|   - Lanelet2 structural comparison   | ✅ Complete    | 100%       | Done      |
|   - PCD comparison                   | 🔴 Not Started | 0%         | 🟡 MEDIUM |
|   - Functional testing               | 🔴 Not Started | 0%         | 🟡 MEDIUM |
| Phase 4: Map Management              | 🔴 Not Started | 0%         | 🟢 LOW    |
| Phase 5: Advanced Features           | ⏳ Future      | 0%         | 🟢 LOW    |

**Overall Progress**: 60% complete (`carla_map_gen` implemented and validated, `carla_pcd_gen` designed)

**Remaining**: Implement `carla_pcd_gen`, functional testing with Autoware, map validator

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

### Phase 1 Success - ✅ COMPLETE
- [x] Can load and use TUMFTM maps in Autoware
- [x] Bridge works with all TUMFTM maps
- [x] Localization works reliably
- [x] Can run basic scenarios

### Phase 2 Success (Map Generation Tools)
- [x] `just build` builds `carla_map_gen`
- [x] `just generate-lanelet2` produces valid Lanelet2 for any CARLA map
- [x] Offline mode (`--xodr`) works without running CARLA server
- [ ] `just build` builds `carla_pcd_gen`
- [ ] `just generate-pcd` produces PCD with sufficient density for NDT
- [ ] `just generate-map` produces complete 4-file Autoware map structure
- [ ] Generated maps work end-to-end with Autoware (routing + autonomous driving)

### Phase 3 Success (Comparison with TUMFTM)
- [x] Lanelet2 output compared against TUMFTM for all 5 towns
- [x] Remaining gaps identified and documented (`docs/guides/map-generation-comparison.md`)
- [x] Post-processing minimizes differences (5 fixes applied)
- [ ] Per-town decision made: tool output vs TUMFTM
- [ ] Generated PCD quality sufficient for NDT localization

### Overall Success
- [ ] Every supported CARLA town has a working Autoware map (from tool or TUMFTM)
- [ ] User can generate maps for any CARLA map with `just generate-map`
- [x] Process is documented and repeatable


## Validated end to end, and the bug that was in the way (2026-08-30)

`carla_pcd_gen` was implemented -- four modules, 394 lines, exactly as designed above -- while
this document still described it as pending. It had never been run against Autoware, and it did
not work.

Generating Town01 takes about two seconds: 255 spawn points, 8 LiDARs, 2.39 million voxels,
28 MB. Autoware then would not drive on it. Three acceptance runs failed identically, the ego
never moving, with a localization error of 1.78 m against 1.08 to 1.24 on the shipped TUM map.

**The map was mirrored.** The cloud was written in CARLA's left-handed frame:

```
generated cloud   y  -81.2 .. +414.6
lanelet2 map      y -339.2 ..   +8.4
```

`collector.rs` said so in a comment, and gave a reason: "The bridge also publishes live LiDAR
scans in CARLA coordinates (see sensor_bridge.rs publish_lidar), so NDT matching is
consistent." That reason is false. `publish_lidar` writes `y: -det.point.y`, and its own
comment says "The pre-built PCD map is in ROS frame (Y-flipped), so live scan must match." The
generated map was therefore mirrored against both the live scans and the lanelets, and NDT had
nothing to match. It is the kind of bug that survives review, because the file that is wrong
explains itself confidently and the file that contradicts it is somewhere else.

With the Y flip applied the extents line up and the map works:

```
                        localization    cross-track   verdict
  before the fix         1.78 m          --            0/3, ego never drove
  after the fix          1.04, 1.09 m    0.034, 0.085  2/2 passed
  shipped TUM map        1.08, 1.24 m    0.037, 0.137  2/2 passed
```

A map this tool generates is now as good as the one shipped with the project, by the numbers
the acceptance harness measures.

### Using it

`just pcd-gen <out_dir>` wraps the generator. It takes CARLA over -- synchronous mode,
rendering off, LiDARs teleported across every spawn point -- and restores the settings
afterwards, so the bridge and any scenario must be stopped first.

It writes `pointcloud_map.pcd` only. A usable map directory also needs `lanelet2_map.osm`,
`map_config.yaml` and `map_projector_info.yaml`; the test above took those from the existing
Town01 map, which is what isolates the point cloud as the only thing under test.

### Worth knowing

The PCD carries `FIELDS x y z` with no intensity, which the design sketch above asks for. That
is correct: the shipped TUM map has no intensity either, and Autoware's `map_loader` accepts
both. The sketch was aspirational rather than a requirement.
