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

**Priority**: Low. Current params work for Tesla Model 3. Only needed when supporting multiple vehicles.

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
- [ ] Document the multi-vehicle setup with domain IDs
- [ ] Test with 2+ vehicles simultaneously

**Priority**: Low. Single-vehicle is the primary use case.

---

## Summary

| Gap | Impact | Status |
|-----|--------|--------|
| Vehicle model calibration script | Needed for multi-vehicle support | Open, low priority |
| Activate advanced control module | Better steering at speed | Closed (gap 2) |
| Configurable steering multiplier | Per-vehicle tuning | Closed (gap 2) |
| Ground-truth object publisher | Debug/fast-iteration mode | Closed (gap 3) |
| Multi-vehicle documentation | Scalability | Open, low priority |

**Remaining work**: gap 1's calibration script and gap 4's multi-vehicle documentation and
two-vehicle test. Neither is blocking.
