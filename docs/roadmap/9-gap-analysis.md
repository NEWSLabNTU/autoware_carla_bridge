# Gap Analysis and Remaining Work

Functional gaps that affect simulation fidelity.

**Status**: Bridge is functionally complete -- vehicle drives to goal with zero MRM interrupts. Gaps below are refinements, not blockers.

---

## 1. Vehicle Model Calibration

**Gap**: Vehicle physics params are hardcoded for one model (Tesla Model 3). Switching the CARLA blueprint requires manually updating `vehicle_info.param.yaml`.

**What we have**:
- `vehicle_info.param.yaml` with wheelbase=2.79, max_steer_angle=0.70 rad
- `vehicle_config.yaml` with `blueprint: "vehicle.tesla.model3"`
- Steering conversion: `tire_angle / MAX_STEER_ANGLE` (linear, no per-vehicle multiplier)

**What to do**:
- [x] Add a script to extract vehicle params (wheelbase, dimensions, steer limits) from CARLA's `physics_control` API -- `scripts/extract_vehicle_params.py`
- [x] Populate `vehicle_info.param.yaml` from the selected blueprint (done for `vehicle.tesla.model3`; re-run the script when the blueprint changes)
- [x] Add optional steering multiplier (default 1.0) -- `steering_multiplier` ROS parameter, settable via `STEERING_MULTIPLIER` on `just ego-av`

**Automated 2026-08-27**: `just vehicle-params [blueprint] [--write]` measures a blueprint and
rewrites `vehicle_info.param.yaml` in place, so switching vehicles is one command rather than a
manual transcription.

```
$ just vehicle-params vehicle.audi.etron --write
# vehicle.audi.etron on CARLA 0.9.16
#   bounding box      4.856 x 2.033 x 1.649 m
#   wheel base        2.900 m
#   wheel tread       1.593 m
#   wheel radius      0.370 m
#   steering          70.0 deg per wheel, 58.8 deg effective (1.026 rad)
```

Three things the script now gets right that the first version did not:

- **It refuses to run while CARLA is synchronous.** Settling a freshly spawned car needs the
  simulation to advance, and the original did that by switching on synchronous mode and
  ticking -- on whatever server it found, then restoring *its* idea of the settings afterwards.
  Only `carla_scenario_bridge` may tick (invariant 1), so that would have corrupted any run in
  progress. In asynchronous mode the server free-runs and the car settles on its own, so the
  script never touches world settings at all.
- **It writes the file, and the comment that names the blueprint.** Rewriting the numbers while
  leaving "Measured from `vehicle.tesla.model3`" above them would be worse than not writing.
- **It leaves `max_steer_angle` alone by default.** The shipped 0.70 rad is a planning limit
  chosen inside the vehicle's physical capability (1.025 rad effective), not a bad measurement;
  writing the physical value is a behaviour change and needs `--physical-steer-angle`.

**Verified** across four body types -- sedan, SUV, fire truck and motorcycle:

| blueprint | wheel base | tread | radius | body (L x W x H) |
|---|---|---|---|---|
| `vehicle.tesla.model3` | 3.005 | 1.667 | 0.370 | 4.792 x 2.163 x 1.488 |
| `vehicle.lincoln.mkz_2020` | 2.860 | 1.593 | 0.355 | 4.892 x 1.837 x 1.490 |
| `vehicle.audi.etron` | 2.900 | 1.593 | 0.370 | 4.856 x 2.033 x 1.649 |
| `vehicle.carlamotors.firetruck` | 4.995 | 2.383 | 0.570 | 8.468 x 2.891 x 3.827 |
| `vehicle.harley-davidson.low_rider` | 1.620 | 0.400 | 0.350 | 2.350 x 0.766 x 1.649 |

The wheelbases match the real vehicles (an MKZ is 2.85 m, an e-tron 2.93 m), and writing the
e-tron's numbers and then the Tesla's again returns the file to a clean git diff -- so the
script reproduces the hand-written values exactly.

**Caveat for two-wheelers**: CARLA reports four wheels for a motorcycle or bicycle, the left
and right of each pair 40 cm apart, so the extracted `wheel_tread` is an internal placeholder
rather than a track width a real motorcycle has. Autoware's vehicle model assumes a car in any
case.

**Priority**: Closed. The Tesla Model 3 remains the vehicle this project runs; switching is now
a single command.

---

## 2. Control Module

### Current state

`vehicle_control.rs` subscribes to `autoware_control_msgs/Control` and converts physical
units to CARLA's normalized input:

```
steering_tire_angle (rad) -> steer, through CARLA's own Ackermann geometry (issue 006)
                             with an optional `steering_multiplier` trim
acceleration (m/s2)       -> throttle = accel / 3.0, or brake = -accel / 3.0
```

It publishes velocity, steering, control_mode, gear, turn indicators, hazard lights and
actuation status -- seven topics, at ~20 Hz.

### Resolved: `vehicle_bridge.rs` deleted (2026-08-27)

The alternative control module was dead: reachable only through `actor_bridge::create_bridge`,
which nothing called. It was kept on the argument that it was more sophisticated. That
argument no longer holds:

- **Topic parity.** Both published the same seven status topics.
- **Steering.** Its speed-dependent `steering_curve` lookup was the main draw. Measured, the
  curve's real effect is about 4% at operating speed and does not match the values CARLA
  declares (issue 016). `vehicle_control.rs` now inverts the actual Ackermann geometry
  exactly and carries a calibration trim, which is a better answer than interpolating a
  table that measurement contradicts.
- **Smoothing.** Its first-order filter (tau = 0.2 s) would add lag on top of CARLA's own
  steering slew, measured at 2.5 units/s. That is the wrong direction for a loop this issue
  spent a long time proving is sensitive to delay.
- **Interface.** Activating it meant switching Autoware's `raw_vehicle_cmd_converter` to emit
  `ActuationCommandStamped`, trading the physical-unit interface for a normalized one.

Deleted with it: `other_bridge.rs` and `trafficsign_bridge.rs`, unreachable by the same
trace and untouched since the March package rename, plus the `create_bridge` and
`get_bridge_type` dispatchers and the helpers left stranded. 646 lines. `ActorBridge` and
`BridgeType` stay -- `sensor_bridge` uses both.

Worth noting the cost of having kept it: dead code still attracts maintenance. `vehicle_bridge.rs`
received a bug fix in July 2026 for a connection-handling problem it could never encounter.

- [x] Evaluate whether to activate `vehicle_bridge.rs` or keep the simpler `vehicle_control.rs` -- deleted, reasoning above
- [x] Add configurable steering multiplier (useful for calibration without changing code) -- see gap 1

**Priority**: Closed.

---

## 3. Ground-Truth Object Detection

**Gap**: We use Autoware's real LiDAR perception pipeline (lidar_centerpoint), not CARLA ground-truth bounding boxes.

**Our approach is more realistic** -- it tests the full perception stack. Ground-truth objects are useful for:
- Bypassing perception when debugging planning/control
- Faster iteration (no GPU-heavy inference)

**What to do**:
- [x] Investigate if Autoware has a standard topic/interface for injecting ground-truth objects
- [x] If so, add an optional publisher that reads CARLA actor bounding boxes
- [x] Gate behind a config flag (default: off, use real perception)

**Done**: `src/acb_bridge/src/ground_truth_objects.rs`, behind the
`publish_ground_truth_objects` parameter (default `false`), forwarded through
`acb_bridge.launch.xml`, `ego_av.launch.xml` and the justfile's `GROUND_TRUTH_OBJECTS`
environment variable.

The topic is `/perception/object_recognition/objects`
(`autoware_perception_msgs/PredictedObjects`), which is what the perception stack itself
publishes and what `map_based_prediction` and planning consume. There is no separate
injection interface in Autoware 1.5.0, so the publisher takes the perception stack's place
rather than sitting alongside it. That makes it the caller's job to launch with
`perception:=false` -- `carla_simulator.launch.xml` gates the whole stack on that argument,
so with it off nothing else writes the topic. Running both would put two publishers on one
topic, which this repository has already paid for twice.

Each non-ego CARLA actor within `ground_truth_range_m` (default 100 m) becomes one
`PredictedObject`: the actor id little-endian in the UUID, the bounding box as `shape`,
`existence_probability` 1.0, and a classification of CAR or PEDESTRIAN by actor type. Poses
are converted to the ROS frame, and velocity is rotated into the object's body frame so the
longitudinal and lateral components mean what Autoware expects.

**Verified** against a live scenario: the publisher ran at 14.9 Hz and reported the
background AV at (230.00, -129.80) where CARLA's own transform put it, a position error of
0.000 m, with dimensions 4.79 x 2.16 x 1.49 m matching the actor's bounding box exactly. The
ego was correctly absent from the set.

**Caveat worth remembering**: ground truth is not free of lies. It reports every actor the
server knows about within range, including ones no sensor could see -- through buildings,
behind other vehicles. Planning tested against it is tested against an oracle, so results do
not transfer to a run using real perception.

**Priority**: Closed. Real perception remains the default and preferred approach.

---

## 4. Multi-Vehicle Support

**Gap**: One bridge instance per vehicle.

**Current design**: Use separate `ROS_DOMAIN_ID` for each bridge+Autoware pair.

**What to do**:
- [x] Document the multi-vehicle setup with domain IDs
- [x] Test with 2+ vehicles simultaneously

**Done**: verified on 2026-08-27 with two complete Autoware stacks driving two CARLA vehicles
at once -- the SSv2 scenario ego in ROS domain 1, and a background AV in domain 2 driven by
its own Autoware and pilot, outside SSv2's model.

```
bg_av_1    start (230.0,-129.8) -> end (130.5,-129.4)  travelled 99.5 m  peak 5.03 m/s
hero       start (190.8,-130.1) -> end (124.7,-129.7)  travelled 66.2 m  peak 3.99 m/s
```

Exactly one `/clock` publisher and one `acb_bridge` in each domain, SSv2's interpreter present
in domain 1 and absent from domain 2, and each vehicle's status published by its own bridge.
Two stacks plus CARLA used about 42 GB of RAM and 391 nodes between them.

The setup, the measured invariants and three operational constraints the run established --
per-stack `--log-dir`, restarting `just bg-av` between scenarios, and detaching the bridge with
`setsid --fork` -- are written up in the parent repository's
`docs/design/multi-instance-architecture.md`, whose status line had been stale at "not yet
implemented".

**Priority**: Closed. Single-vehicle remains the primary use case.

---

## Summary

| Gap | Impact | Status |
|-----|--------|--------|
| Vehicle model calibration script | Needed for multi-vehicle support | Closed (gap 1) |
| Activate advanced control module | Better steering at speed | Closed (gap 2) |
| Configurable steering multiplier | Per-vehicle tuning | Closed (gap 2) |
| Ground-truth object publisher | Debug/fast-iteration mode | Closed (gap 3) |
| Multi-vehicle documentation | Scalability | Closed (gap 4) |

**Remaining work**: none. Every gap in this analysis is closed. The parameters shipped are
correct for the Tesla Model 3 this project runs, and `just vehicle-params` regenerates them for
any other CARLA blueprint.
