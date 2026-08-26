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

Our `vehicle_control.rs` subscribes to `autoware_control_msgs/Control`:

```
steering_tire_angle (rad) → steer = -angle / MAX_STEER_ANGLE
acceleration (m/s²)       → throttle = accel / 3.0, or brake = -accel / 3.0
```

Simple, direct, works. Publishes velocity, steering, control_mode, gear at ~20 Hz.

### Dead code: `vehicle_bridge.rs`

We have a more sophisticated control module (`src/acb_bridge/src/bridge/vehicle_bridge.rs`, 466 lines, currently `#[allow(dead_code)]`) that:
- Subscribes to `ActuationCommandStamped` (normalized throttle/brake/steer)
- Reads CARLA's `physics_control.steering_curve` for speed-dependent steering
- Applies first-order filter (tau=0.2s) for steering smoothing
- Publishes 7 status topics (adds actuation_status, turn_indicators, hazard_lights)

### What to do

- [ ] Evaluate whether to activate `vehicle_bridge.rs` or keep the simpler `vehicle_control.rs`
- [ ] If activating: switch Autoware's raw_vehicle_cmd_converter to output `ActuationCommandStamped`
- [x] Add configurable steering multiplier (useful for calibration without changing code) -- see gap 1

**Priority**: Low. Current control is sufficient -- vehicle completes routes successfully.

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
