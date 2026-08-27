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
- [ ] Investigate if Autoware has a standard topic/interface for injecting ground-truth objects
- [ ] If so, add an optional publisher that reads CARLA actor bounding boxes
- [ ] Gate behind a config flag (default: off, use real perception)

**Priority**: Low. Real perception is the default and preferred approach.

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

| Gap | Impact | Priority | Effort |
|-----|--------|----------|--------|
| Vehicle model calibration script | Needed for multi-vehicle support | Low | 1-2 days |
| Activate advanced control module | Better steering at speed | Low | 1 day |
| Configurable steering multiplier | Per-vehicle tuning | Low | 0.5 day |
| Ground-truth object publisher | Debug/fast-iteration mode | Low | 1-2 days |
| Multi-vehicle documentation | Scalability | Low | 0.5 day |

**Total estimated remaining work**: ~5 days of refinement, none blocking.
