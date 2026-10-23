# Autoware CARLA Bridge (acb)

## Project Overview

Native ROS 2 bridge between CARLA and Autoware, written in Rust using rclrs: CARLA's vehicle
interface for Autoware. `acb_bridge` attaches to the CARLA vehicle with `role_name="hero"`,
spawns and publishes its sensors, publishes the vehicle status and `/clock`, and applies
Autoware's control.

**Repository**: https://github.com/NEWSLabNTU/autoware_carla_bridge (branch `target-shape`
during roadmap 051)

**Its place**: one of the eight repositories of scenario-world-bridge's roadmap 051 ("the
target shape", `docs/roadmap/051-target-shape.md` in scenario-world-bridge). acb holds the
CARLA vehicle bridge, CARLA's stand-alone bring-up and demo (`demo_scenario.py`, the pilot
from `autoware_sim_pilot`) and the map tools. It depends on `sim_vehicle_spec` and
`autoware_sim_vehicle` (submodules under `deps/`), and on nothing of scenario-world-bridge
(swb): acb plus those two is a complete Autoware-on-CARLA system without SSv2. swb runs the
same `acb_bridge` as a scenario's ego, next to its CARLA world server `swb_carla_world`.

---

## Repository Structure

```
.
├── src/
│   ├── acb_bridge/                # The bridge (Rust, rclrs); launch/acb_bridge.launch.xml
│   ├── acb_launch/                # carla_simulator.launch.xml: the Autoware profile + acb_bridge
│   ├── acb_demo_launch/           # single_vehicle_autoware.launch.xml: the above + acb_scenario
│   ├── acb_scenario/              # demo_scenario (spawns the hero, ticks CARLA at 20 Hz)
│   ├── carla_manual_control/      # Vehicle monitor / manual control GUI (Rust)
│   ├── carla_map_gen/             # Lanelet2 map from a running CARLA (Python)
│   ├── carla_pcd_gen/             # Point cloud map from a running CARLA (Rust)
│   └── external/                  # COLCON_IGNORE only
├── deps/                          # Submodules (branch target-shape), COLCON_IGNOREd
│   ├── sim_vehicle_spec/          #   the car's spec; generated/<car>/carla_sensors.yaml,
│   │                              #   carla_physics.yaml; crates/carla_physics_spec (Rust reader)
│   └── autoware_sim_vehicle/      #   cars/<car>/ (Autoware packages), individual_params,
│                                  #   autoware_sim_profile, autoware_sim_pilot, autoware_sim_check
├── scripts/                       # CARLA service, demo, map, probe and debug scripts (README.md)
├── config/play_launch.yaml        # Composable-node load timeout for `just sim`
├── third_party/carla/run.sh       # CARLA in the foreground (`just run-carla`)
├── data/carla-autoware-bridge/    # Pre-converted TUM maps (`just setup`; git-ignored)
├── docs/                          # design/, guides/, issues/ (one defect per file), roadmap/
└── justfile
```

### The car (roadmap 051 K2, K3)

The car is the spec `tesla_model3`; everything selects it with `vehicle_model:=tesla_model3`.
acb has no vehicle packages of its own any more (`acb_vehicle*`, `acb_sensor_kit*`,
`individual_params` and `vehicle_config.yaml` are gone). From the spec:

- `sim_vehicle_spec/generated/<car>/carla_sensors.yaml` -- the sensors acb spawns
  (parameter `vehicle_config`). Positions come from TF (`/robot_description`), not the file.
  A `map_origin` key is refused by name: the origin is the map's
  (`<map dir>/carla/map_origin.yaml`, parameter `map_origin_path`).
- `sim_vehicle_spec/generated/<car>/carla_physics.yaml` -- CARLA's `VehiclePhysicsControl`
  for the car, plus `command_lags` (brake dead time and time constant, motor time constant),
  `spec_hash` and `physics_hash`. Read through `carla_physics_spec` (`vehicle_physics.rs`).
- `autoware_sim_vehicle/cars/<car>/` -- `<car>_description` (with `config/accel_map.csv`,
  `brake_map.csv`, `spec_hash.yaml`), `<car>_launch`, `<car>_sensor_kit_description`,
  `<car>_sensor_kit_launch`; `individual_params` beside them.

Never edit generated files: change the spec and regenerate (`just generate` in
sim_vehicle_spec, and autoware_sim_vehicle's generator).

### Launch layering

- `acb_bridge.launch.xml`: the node, every parameter documented inline.
  `on_exit="shutdown"`, never respawned (see "Time"). Its own `carla_physics` default is
  `none` (blueprint physics), so `just bridge` runs the blueprint's car.
- `carla_simulator.launch.xml`: `autoware_sim_profile/launch/autoware.launch.xml` plus
  `acb_bridge.launch.xml`, with `carla_physics` = the spec's file and
  `carla_physics_mode:=apply`. Any argument it does not declare reaches the profile
  (`max_velocity`, `lidar_detection_model`, `rviz`, `gpu`, `extra_launch`, `data_path`, the
  `sim_*` switches).
- The profile's switches are `sim_localization`, `sim_perception`, `sim_system`,
  `sim_control`, `sim_planning`, `sim_clock_guard` (formerly `carla_*`).
  `autoware_sim_profile`'s `check_launch_args` names the replacement for an old argument; swb's
  wrappers run it. acb's own `just autoware` and `just sim` accept only `map:=` (and
  `auto_drive:=`) and refuse anything else. A raw `ros2 launch`/`play_launch launch` with
  `carla_control:=false` is silently ignored by ROS launch.
- The profile's settings (MRM handler timeouts, component-state topics, MPC without steering
  offset learning, the tracker's UNKNOWN pruning, `max_velocity`, perception without object
  recognition) and their sync scripts live in `autoware_sim_vehicle` now; read its
  `autoware_sim_profile/launch/autoware.launch.xml` and `config/`, and acb's `docs/issues/`
  021-028 for why.

---

## Build & Run

```bash
git submodule update --init --recursive   # deps/ first: the build needs them
just setup    # one-time: system deps, Autoware 1.5.0 Debian, TUM maps, play_launch
just build    # colcon over src, deps/autoware_sim_vehicle, deps/sim_vehicle_spec;
              #   --symlink-install, cargo profile dev-release
just check    # rustfmt (nightly) + clippy -D warnings
just test     # cargo nextest
just clean
```

Recipes (`just --list`):

```bash
just carla-start [map:=] [quality:=epic|low] [render:=] [extra_args:=]  # systemd user unit carla-run-<port>
just carla-stop | carla-status | carla-logs
just run-carla ...         # CARLA in the foreground, same arguments
just sim [map:=Town01] [auto_drive:=true]   # load the town, then Autoware + bridge + demo scenario
just autoware [map:=]      # Autoware + bridge (does not load the town)
just scenario [map:=]      # demo scenario only
just bridge                # acb_bridge only, blueprint physics
just vehicle-check [vehicle_model:=tesla_model3] [out:=<dir>]
just pilot [poses_file]    # autoware_sim_pilot auto_drive
just capture-poses <file>  # autoware_sim_pilot capture_poses
just monitor               # carla_manual_control
just generate-lanelet2|generate-pcd|generate-map <dir>; compare-lanelet2 <gen> <ref>
just build-engines         # CenterPoint TensorRT engines
just package               # release tarball (src + deps, merge-install)
```

None of `sim`, `autoware`, `scenario` or `bridge` starts the vehicle monitor; run
`just monitor` for it.

Project settings are `name:=value` arguments; host settings (`CARLA_PORT`, `CARLA_DIR`,
`DISPLAY`, `AUTOWARE_DATA_PATH`, `AUTOWARE_SETUP`) come from the environment or `.env`
(just's `dotenv-load`). `MAP_NAME`, `CARLA_MAP`, `CARLA_QUALITY`, `CARLA_RENDER` and
`CARLA_EXTRA_ARGS` are refused.

`quality:=low` must never change towns: CARLA 0.9.16 at Low segfaults on `load_world` to
another town and on `reload_world()` (carla-simulator/carla#4940).

### vehicle-check

`just vehicle-check` checks the car on CARLA without Autoware or SSv2: CARLA running and
ticked by nobody else; `scripts/vehicle_check_host.py` spawns the hero on a straight and ticks
in real time; `acb_bridge` with `carla_physics_mode:=apply`; `autoware_sim_check`'s
`vehicle_check.launch.xml` checks the contract (topics, types, rates, frames, stamps,
`/robot_description`) and the dynamics (acceleration and brake steps against 0.35 m/s^2, a
steering sweep against the bicycle model). It prints PASS/FAIL per step and exits 0 only if
everything passed; logs and `report.json` in `play_log/vehicle_check/<time>/` (or `out:=`).

### Under scenario-world-bridge

The workspace builds acb twice: acb's own `just build` (`src/autoware_carla_bridge/install`)
and swb's (`--base-paths src`, which includes acb's packages). Which `acb_bridge` an ego
stack runs depends on how acb's workspace was built; swb's CLAUDE.md ("Use just build")
explains it. After an acb change, rebuild both from a clean environment, acb's first, and
check what the live process loaded (`/proc/<pid>/cmdline`) before trusting a measurement. A
stale `acb_bridge` measures plausibly and wrongly.

---

## Environment Setup

`.envrc` (direnv) sources `/opt/autoware/1.5.0/setup.bash` (ROS 2 Humble included; falls back
to `/opt/ros/humble`) and `install/setup.bash`. Without direnv:

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash  # after the first build
```

Build failures with "package not found" usually mean the environment is not sourced, or
`deps/` is not initialised (`tesla_model3_description`, `autoware_sim_profile`,
`carla_physics_spec` all come from there).

### CARLA version is a Cargo feature

`carla = { version = "0.15.1", features = ["carla-0916"] }` (workspace `Cargo.toml`, crates.io,
no local carla-rust). `CARLA_DIR` (CARLA's install) is read by carla-sys as a CARLA source
tree and breaks the build; the cargo recipes unset it, a bare `cargo` or `colcon build` in a
shell that exports it fails. A stray `CARLA_VERSION` overrides the feature. At run time
acb_bridge compares its CARLA version with the server's on connect and exits on a mismatch.

---

## Key Technical Decisions

### Architecture
- **Hero pattern**: acb attaches to an existing vehicle with `role_name="hero"`; it never
  spawns or destroys it. Whoever spawned it (demo_scenario, swb's world server, a hand-made
  scene) owns it. acb owns the sensors it spawned and destroys them when the hero goes.
- **Synchronous mode**: CARLA at 20 Hz (`fixed_delta_seconds=0.05`); the scene's owner is the
  sole ticker (demo_scenario, `vehicle_check_host.py`, or swb's world server). acb follows.
- **Physics, one writer per mode** (`vehicle_physics.rs`): `carla_physics_mode:=apply` (acb's
  launches) puts `carla_physics.yaml` on the hero at attach; `verify` (swb's ego launches;
  the world server applied it) reads it back. Both refuse a hero of another blueprint than
  the file's, and a read-back that differs from the file. CARLA applies physics at its next
  tick, so read back after a tick, never straight after the apply.
- **Pedal lags** (`pedal_lag.rs`): first-order lag on each pedal plus the brake's dead time,
  from `command_lags`, on simulation time. The filter snaps to its target within 1e-3:
  without that, f32 decay left a ~1e-6 brake for 5 s after release, and CARLA held the car
  with it.
- **Pedal maps**: measured on the spec's physics (`measured_on: spec` and the
  `physics_hash` in `<car>_description/config/pedal_maps.yaml`). Re-measure them when the
  physics changes; a single-constant conversion is wrong by 2x at full throttle (issue 019).
- **spec_hash** (`main.rs::check_spec_hash`): the sensor file's `spec_hash` against
  `autoware_spec_hash_file` (`<car>_description/config/spec_hash.yaml`). A warning, not a
  refusal: stand-alone acb has no third party to say which side is stale (under SSv2, swb's
  bridge refuses a mismatch). `none` skips the check.
- **GNSS auto-localization**: NavSatFix → gnss_poser → `autoware_automatic_pose_initializer_node`
  → NDT align → initialized; no manual pose estimate.

### Time (roadmap 051 K4)

- Every stamp and `/clock` value is `nanos(elapsed_seconds) + BASE_NS` (`clock.rs`;
  `BASE_NS` = 1000 s, rounding half away from zero). No epoch, no wall clock. It must stay
  bit-identical to swb_world_protocol's `sim_time` (swb's `scripts/test_sim_time.py` checks
  both against one table).
- acb attaches to the CARLA episode (world id) it first sees. On a frame or a hero of another
  episode -- a world load, a restarted CARLA -- it logs the change naming both ids, flushes its
  log writer and exits with status 3 (`EPISODE_CHANGED_EXIT`) before publishing anything of
  the new episode. Its launch node is `on_exit="shutdown"`, so the whole Autoware stack ends;
  whoever started the stack restarts it (swb: `swb-agent supervise`). The profile's
  `clock_guard` (`sim_clock_guard`) ends the stack if `/clock` ever goes back.
- So load the town **before** the stack starts. `just sim` runs `scripts/carla_load_town.py`
  first; `just autoware` + `just scenario` on a CARLA showing another town ends the stack at
  the scenario's load. An exit with status 3 is the end of an episode, not a crash.
- `/clock` comes from its own `on_tick` subscription on its own client, established when
  CARLA is reachable (not when a hero is found), so it runs between scenarios too.

### Coordinate Systems

CARLA uses a left-handed system (Y = right), ROS uses right-handed (Y = left). The conversion is a Y-axis flip on positions and roll/yaw sign flips on rotations (see `coordinate_conversion.rs`).

| Data | Coordinate System | Y-Flip? | Why |
|------|-------------------|---------|-----|
| LiDAR points (live scan) | ROS (right-handed) | Yes | Must match pre-built PCD map (confirmed ROS frame) |
| PCD map (pre-built TUMFTM) | ROS (right-handed) | Yes | Verified: Y range -379 to +39, points at (190.8,-130.1) |
| GNSS NavSatFix | Map frame (y=-CARLA_y) | Yes (lat negated) | CARLA maps CARLA_y→Northing without flip; negate lat to get y=-CARLA_y |
| Vehicle pose, GNSS pose_with_cov | ROS (right-handed) | Yes | Autoware planning expects ROS coords |
| Velocities | ROS (right-handed) | Yes | Autoware control expects ROS coords |
| Lanelet2 map | Local coordinates | No | Generated from OpenDRIVE, consistent with poses |

NDT scan matching is purely geometric: as long as the live scans and the PCD map share a
frame, NDT works regardless of handedness. Maps use `projector_type: TransverseMercator`
(matching CARLA's `+proj=tmerc +lat_0=0 +lon_0=0`). `base_link` is the rear-axle centre,
measured from CARLA's rear wheel positions.

### Dependencies
- **rclrs** 0.7: native ROS 2 Rust bindings
- **carla** 0.15.1 (`carla-0916`), **carla_physics_spec** (path dependency into `deps/sim_vehicle_spec`)
- **tracing** for logging (through a writer thread, `log_writer.rs`: a node logging from its
  real-time loop to a busy disk stalls), **color-eyre** for errors
- **nalgebra**: must match carla-rust's version

---

## Key Learnings

### rclrs API (Rust)
- Builder pattern for QoS: `"topic".sensor_data_qos()`, `"topic".reliable()`
- Node is `Arc<NodeState>` internally (cheap clone)
- Publishers need `Arc<Publisher>` for thread sharing
- Automatic serialization (no CDR)
- **Subscription callbacks**: Use `Arc<std::sync::Mutex<T>>` for shared state between callbacks
  - Clone the Arc before moving into closures
  - Subscriptions must be kept alive as struct fields (e.g., `_localization_init_state_sub`)
- A double parameter given an integer stops the node at startup
  (`OverrideValueTypeMismatch`): coerce in the launch file (`$(eval 'float(...)')`).

### rclpy API (Python scripts)
- Use `call_async()` for service calls, followed by `spin_until_future_complete()`
- Don't instantiate service classes directly (raises `NotImplementedError`)
- Use `ServiceClass.Request()` to create request objects
- QoS profiles: `TRANSIENT_LOCAL` for latched topics, `VOLATILE` for streaming data

### Autoware AD API
- `/api/localization/initialize`, `/api/routing/set_route_points`, `/api/routing/clear_route`,
  `/api/operation_mode/change_to_autonomous` (services used by `auto_drive`)
- Route planning requires poses on **connected lanes**; RViz "2D Pose Estimate" / "2D Goal
  Pose" give valid ones (`just capture-poses`)
- Service calls fail until Autoware is fully up; `auto_drive` retries, and engaging needs the
  planning/control pipeline active after the route is set

### CARLA Integration
- Sensor callbacks run in separate threads
- Executor must spin in wait loops for callbacks
- carla-rust uses nalgebra types: `.to_na()` / `Transform::from_na()`
- **Destroy a vehicle's sensors before the vehicle**, in every client: CARLA leaves them alive
  and an orphaned IMU segfaults the server (`AInertialMeasurementUnit::ComputeGyroscope`).
  In Rust `destroy_with_children()`; in Python sweep `sensor.*` whose `parent.id` is the
  vehicle (`scripts/demo_scenario.py::destroy_attached_sensors`).
- A LibCarla client must not outlive its server: its streaming side reconnects to the next
  server on the port and can hang a synchronous CARLA. acb drops and rebuilds such clients
  (and now exits anyway, the episode being gone).
- Pin Vulkan to the NVIDIA card when starting CARLA by hand
  (`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`; `third_party/carla/run.sh`
  sets it when unset).

### use_sim_time Parameter Propagation - CRITICAL

ROS 2 launch files don't propagate `use_sim_time` through nested includes; Autoware's
`global_params.launch.py` only affects nodes in its own context. Symptom: the Auto button
stays disabled, and logs show
`Lookup would require extrapolation into the past. Requested time 527.29 but earliest data is at time 1765365855.01`
(TF on wall clock, sensors on sim time → MessageFilter drops every pointcloud → no occupancy
grid → no trajectory → autonomous mode unavailable).

Fix: a global `<set_parameter name="use_sim_time" value="$(var use_sim_time)"/>` in the
top-level launch file, before any include. The profile (`autoware_sim_profile`'s
`autoware.launch.xml`) does this, and also sets `system_run_mode=logging_simulation`, which
turns off pose_initializer's stop check. Any new top-level launch must do the same.

```bash
ros2 param get /localization/pose_estimator/ndt_scan_matcher use_sim_time   # True
ros2 param get /localization/util/pose_initializer stop_check_enabled       # False
```

### URDF/Xacro XML Comments - CRITICAL

**NEVER use colons in URDF/Xacro XML comments.** ROS 2 launch parses the robot_description
parameter as YAML, and a colon, even inside an XML comment, becomes a YAML mapping:

```
mapping values are not allowed here
  in "<unicode string>", line XX, column YY:
```

Use `<!-- NOTE - ... -->`, not `<!-- NOTE: ... -->`. The impact is total:
robot_state_publisher crashes → no `/robot_description` → acb never detects Autoware → no
sensors → localization never initializes. The URDFs are generated in autoware_sim_vehicle
now; the rule applies to its templates and to any hand-written xacro.

### Diagnostics (known, harmless)

Seen during autonomous driving with `/autoware/modes/autonomous` OK: `/autoware/debug/tools`,
`duplicated_node_checker` (traffic light nodes with fusion-only lights),
`service_log_checker`, and `control_command_gate` (absent in Autoware 1.5.0) report ERROR;
`ndt_align_service_status` says "Waiting for pcd loader service" after the first alignment;
`/adapi/node/vehicle_door` is unknown (CARLA has no door API). AD API adapter nodes may time
out in LoadNode and still start. `pose_instability_detector` sometimes dies with "cannot
store a negative time point" at stack start; the stack runs on without it.

---

## Coding Practices

### Build Commands - CRITICAL

**Always use `just build` instead of direct `colcon build`.** The justfile passes
`--symlink-install` and the `deps/` base paths; a plain `colcon build` creates file copies
that conflict with the symlinks, and without `deps/` misses the car's packages. If it
happened: `just clean && just build`. For one broken package: `rm -rf build/<pkg>
install/<pkg> && just build`. Never fix `install/` by hand.

`just build` refuses to start when a pip-installed setuptools shadows the system one
(`option --editable not recognized`); fix with `pip uninstall -y setuptools`.

### Background Commands

Start long-running recipes (`just sim`, `just autoware`) with the Bash tool's
`run_in_background`, not with `&`, `sleep` and `echo` in the command. `just carla-start` and
`carla-stop` control a systemd user unit and return at once; do not background them.

### ROS 2 Topic Monitoring

Bound streaming commands with `timeout`: `timeout 5 ros2 topic hz /some/topic --window 3`,
not a background job and `kill`.

### Processes

- Never `pkill -f` a pattern your own shell's command line contains; match the executable
  (`pgrep -x`, but `comm` is truncated to 15 characters) or read `/proc/<pid>/cmdline`, or
  start the tree under `setsid` and signal the group.
- Kill CARLA by pid; if it ignores SIGTERM, SIGKILL it before starting another on the port.

### Error Handling

**Never silence `Result` types without justification**

```rust
// Preferred: Propagate errors
blueprint.set_attribute("fov", &value.to_string())?;

// Last resort: Document why ignoring is safe
// SAFETY: Best-effort operation, failure is non-critical
let _ = optional_operation();
```

### No shims

A renamed or removed package, argument, parameter or entry point is gone, not aliased;
where a user could still pass the old one, refuse it with a message naming the new one (as
`map_origin` in a sensor config, the `carla_*` profile arguments, the retired environment
variables).

### Unused Code

**Delete unused code unless documented.** Keep only resource holders (subscriptions kept
alive: `/// NOTE:`), planned work (`/// TODO(...)`), or a module-level
`#![allow(dead_code)]` with a reason. Delete experiments, old implementations and helpers
with no use.

---

## Debugging

- play_launch logs: `play_log/<time>/node/<node>/{out,err}` under the directory the recipe
  ran in (`just vehicle-check`: `<out>/acb_log`, `check_log`).
- acb_bridge exited 3: an episode change (its log names both episode ids); see "Time".
- acb_bridge refused the hero: wrong blueprint or physics read-back mismatch, both named in
  the error; check `carla_physics_mode` (`verify` with nobody applying) and the blueprint.
- "vehicle_config was generated from spec X, Autoware's vehicle package from Y": one install
  is stale; update `deps/` and `just build`.
- Probes for the control chain, steering and pedals are in `scripts/probe_*.py`; defects with
  their evidence are in `docs/issues/` (index in its README).

---

## Documentation

- `README.md` -- setup, the car, time and episodes, recipes
- `scripts/README.md` -- the scripts
- `docs/README.md` -- docs index (design, guides, issues, roadmap, archive)
- scenario-world-bridge `docs/roadmap/051-target-shape.md` -- the target shape and its progress (K1-K5)
- [carla-rust](https://github.com/jerry73204/carla-rust), [rclrs](https://github.com/ros2-rust/ros2_rust),
  [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
