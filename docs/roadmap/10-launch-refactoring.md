# Launch File Refactoring

Restructure launch files so the bridge integrates into Autoware's standard launch system via the vehicle interface pattern.

**Status**: IN PROGRESS

---

## Motivation

Currently the bridge runs as a separate process alongside Autoware. The user must start Autoware, the bridge, and the scenario script independently. This refactoring makes the bridge start automatically as part of Autoware's vehicle interface, matching how other simulator integrations (AWSIM, autoware_carla_interface) work.

---

## Design

### Architecture

```
CARLA instance (1 per simulation)
  └─ scenario script (loads map, spawns vehicles, ticks simulation)

Per vehicle:
  └─ carla_simulator.launch.xml (Autoware + CARLA overrides)
       └─ autoware.launch.xml
            ├─ vehicle interface (acb_vehicle_launch)
            │    ├─ acb_bridge node        ← starts automatically
            │    └─ carla_manual_control   ← optional monitor
            └─ sensor kit (acb_sensor_kit_launch)
```

### Bridge in the Vehicle Interface

Autoware's launch chain:
```
autoware.launch.xml
  └─ tier4_vehicle_launch/vehicle.launch.xml
       └─ $(vehicle_model)_launch/vehicle_interface.launch.xml
```

When `vehicle_model:=acb_vehicle`, Autoware finds `acb_vehicle_launch` and includes `vehicle_interface.launch.xml`. This is where the bridge and optional monitor launch.

The bridge has **no map dependency** — it only handles sensors, vehicle control, and clock. Map loading is entirely owned by Autoware.

### `carla_simulator.launch.xml` as Integration Layer

Users always launch Autoware through `carla_simulator.launch.xml`, which wraps the user's `autoware.launch.xml` with CARLA-specific overrides that cannot be passed as simple arguments:

| Override                                               | Why                                                   |
|--------------------------------------------------------|-------------------------------------------------------|
| Separate localization launch with CARLA NDT/EKF params | `autoware.launch.xml` doesn't expose config path args |
| Separate system launch with relaxed timeouts           | MRM handler needs 30s timeout vs 0.5s default         |
| Traffic light monitoring disabled                      | Not available in CARLA                                |
| Planning preset (disabled modules)                     | CARLA maps lack required lanelet attributes           |
| `use_sim_time` global `set_parameter`                  | Fixes propagation bug in Autoware                     |
| `system_run_mode=logging_simulation`                   | Disables stop_check in pose_initializer               |

The user's `autoware.launch.xml` is accepted as an argument (default: upstream `autoware_launch`):

```xml
<arg name="autoware_launch"
     default="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml"/>
```

Vehicle model and sensor kit are forced to `acb_vehicle` and `acb_sensor_kit` — these are not user-configurable since the bridge, URDF, and sensing pipeline are a matched set.

### Launch File Layout

```
acb_bridge/launch/
  acb_bridge.launch.xml                  # bridge node only
                                         #   args: vehicle_name, carla_port

acb_vehicle_launch/acb_vehicle_launch/launch/
  vehicle_interface.launch.xml           # bridge + optional monitor
                                         #   args: vehicle_name, carla_port, monitor
                                         #   included by Autoware via vehicle_model

acb_scenario/launch/
  single_vehicle_scenario.launch.xml     # scenario script only
                                         #   args: map_name, spawn_index, carla_port

acb_launch/launch/
  carla_simulator.launch.xml             # Autoware + CARLA overrides + bridge
                                         #   args: autoware_launch, map_path,
                                         #          carla_port, map_name, ...

acb_demo_launch/launch/
  single_vehicle_autoware.launch.xml     # carla_simulator + scenario (all-in-one)
```

### Package Responsibilities

| Package                 | Role                                                              |
|-------------------------|-------------------------------------------------------------------|
| `acb_bridge`            | Bridge node launch (single node)                                  |
| `acb_vehicle_launch`    | Vehicle interface for Autoware (bridge + monitor)                 |
| `acb_sensor_kit_launch` | Sensor kit for Autoware (sensing pipeline)                        |
| `acb_launch`            | Autoware integration layer (CARLA-specific config overrides)      |
| `acb_scenario`          | Scenario scripts (CARLA map loading, vehicle spawning, tick loop) |
| `acb_demo_launch`       | All-in-one demo launch (Autoware + scenario)                      |

### Justfile Recipes

| Recipe               | What it runs                                               |
|----------------------|------------------------------------------------------------|
| `just sim`           | `single_vehicle_autoware.launch.xml` (everything)          |
| `just autoware`      | `carla_simulator.launch.xml` (Autoware + bridge + monitor) |
| `just scenario`      | `single_vehicle_scenario.launch.xml` (scenario only)       |
| `just bridge`        | `acb_bridge.launch.xml` (bridge only, for debugging)       |
| `just monitor`       | `carla_manual_control` (standalone)                        |
| `just pilot`         | Autonomous driving pilot                                   |
| `just capture-poses` | RViz pose capture                                          |

### User Workflows

**Recommended (all-in-one demo)**:
```bash
just carla-start
just sim
```

**Manual (separate components)**:
```bash
just carla-start
just autoware        # terminal 1: Autoware + bridge + monitor
just scenario        # terminal 2: scenario script
```

**Tarball user**:
```bash
source autoware-carla-bridge-0.12.0/setup.bash
# Terminal 1: Autoware + bridge + monitor
ros2 launch acb_launch carla_simulator.launch.xml \
    map_path:=/path/to/Town01
# Terminal 2: scenario
ros2 launch acb_scenario single_vehicle_scenario.launch.xml
```

**Custom Autoware fork**:
```bash
ros2 launch acb_launch carla_simulator.launch.xml \
    autoware_launch:=/path/to/my_autoware.launch.xml \
    map_path:=/path/to/Town01
```

---

## Work Items

### 10.1 Create `acb_scenario` package
- [x] Create ament_python package with `demo_scenario.py`
- [x] Move `demo_scenario.py` from `acb_pilot` to `acb_scenario`
- [x] Add `single_vehicle_scenario.launch.xml`
- [x] Entry point: `demo_scenario = acb_scenario.demo_scenario:main`

### 10.2 Update `acb_bridge.launch.xml`
- [x] Add `vehicle_name` arg (default: `hero`)
- [x] Pass `vehicle_name` to bridge node as parameter

### 10.3 Update `acb_vehicle_launch/vehicle_interface.launch.xml`
- [x] Include `acb_bridge.launch.xml`
- [x] Add `monitor` arg (default: `true`) to optionally launch `carla_manual_control`
- [x] Forward `vehicle_name` and `carla_port` args

### 10.4 Update `carla_simulator.launch.xml`
- [x] Add `autoware_launch` arg (path to user's autoware.launch.xml)
- [x] Set `launch_vehicle_interface=true` (bridge starts via vehicle interface)
- [x] Force `vehicle_model:=acb_vehicle`, `sensor_model:=acb_sensor_kit`

### 10.5 Update `acb_bridge` to support `vehicle_name` parameter
- [x] Accept `vehicle_name` ROS parameter (default: `hero`)
- [x] Use `vehicle_name` instead of hardcoded `"hero"` in `wait_for_hero_vehicle()`

### 10.6 Simplify `acb_demo_launch`
- [x] Keep only `single_vehicle_autoware.launch.xml`
- [x] Compose from `carla_simulator.launch.xml` + `single_vehicle_scenario.launch.xml`
- [x] Remove `single_vehicle.launch.xml` (redundant — bridge is in vehicle interface)

### 10.7 Update justfile recipes
- [x] Rename `run-*` recipes to short names (`sim`, `bridge`, `scenario`, etc.)
- [x] Update `sim` to use `single_vehicle_autoware.launch.xml`
- [x] Update `autoware` to use `carla_simulator.launch.xml` (now includes bridge)
- [x] Verify `just scenario` and `just bridge` work

### 10.8 Update tarball packaging
- [x] `just package` includes `acb_scenario` (13 packages)
- [x] All launch files present in tarball

### 10.9 Verify end-to-end
- [x] `just scenario` starts scenario and spawns vehicle
- [x] `just bridge` starts bridge and waits for Autoware
- [ ] `just sim` starts everything and vehicle drives autonomously
- [ ] `just autoware` + `just scenario` work in separate terminals
- [ ] Tarball user can run `ros2 launch acb_launch carla_simulator.launch.xml`

---

## Success Criteria

- [x] Bridge starts automatically when Autoware launches via vehicle_interface.launch.xml
- [x] `carla_simulator.launch.xml` accepts user's `autoware_launch` path
- [x] Scenario script is a standalone package (`acb_scenario`)
- [x] `vehicle_name` parameter enables future multi-vehicle support
- [x] Tarball is self-contained (no source tree references)
- [ ] Full `just sim` end-to-end test with autonomous driving
