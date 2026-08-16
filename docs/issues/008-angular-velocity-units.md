# 008 — `Actor::angular_velocity()` is degrees per second, not radians

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `src/acb_bridge/src/autoware.rs`,
and carla-rust `carla/src/client/actor_base.rs`
**Status**: Confirmed by measurement, fixed in both repos

## What is wrong

carla-rust documents `ActorBase::angular_velocity()` as

```rust
/// Returns the actor's angular velocity in radians/s.
```

CARLA's own Python API documents `carla.Actor.get_angular_velocity()` as deg/s. The two
cannot both be right, and every consumer in this bridge believed the Rust doc:

- `VelocityReport.heading_rate` was `-angular_velocity.z`, a field ROS defines in rad/s.
- `/carla/ground_truth/odom`'s `twist.twist.angular` was the same value, Y-flipped.

## The measurement

`scripts/probe_carla_conventions.py` drives a Tesla Model 3 in a steady turn in
synchronous mode at `fixed_delta_seconds = 0.05` and compares three quantities over 39
samples of the steady phase:

```
differentiated yaw rate :  74.0024 deg/s = 1.2916 rad/s
get_angular_velocity().z:  74.2289
imu gyroscope.z         :   1.2856   (documented rad/s, and correct)

angular_velocity / differentiated-rad-per-s : 57.4711
angular_velocity / differentiated-deg-per-s :  1.0031
angular_velocity / gyro.z                   : 57.7388
```

`get_angular_velocity()` tracks the yaw rate in **degrees** per second to within 0.3 %.
The IMU gyroscope, on the same actor in the same turn, tracks it in radians. CARLA's
Python documentation is right and carla-rust's doc comment is wrong.

## Why nothing broke

`heading_rate` was 57.3x too large for the whole life of the bridge. Autoware's
localization did not notice because it does not read that field: `gyro_odometer` fuses
`VelocityReport.longitudinal_velocity` with the **IMU's** `angular_velocity.z`, which was
in the right units all along. `vehicle_velocity_converter`, which does read
`heading_rate`, feeds `/sensing/vehicle_velocity_converter/twist_with_covariance` — used
by the pose initializer's stop check, which this profile disables
(`system_run_mode=logging_simulation`).

So the wrong value went to a topic nothing in this configuration consumes. Turn either of
those two things back on and the localization goes with it.

## Fix

**acb**: convert at the boundary, once, where CARLA's value is read.
`heading_rate = -angular_velocity.z.to_radians()`, and the ground-truth twist likewise.

**carla-rust**: correct the doc comment on `angular_velocity()` and
`set_target_angular_velocity()` to say degrees per second, matching CARLA's own API
reference, and add the measurement above to the note so the next reader does not have to
repeat it. The units themselves are left alone — silently changing the return value of a
published API is worse than the wrong doc, and every existing caller has been calibrated
against the real behaviour.

## Also measured, for the record

The same probe pinned two other conventions this bridge depends on:

- `compass − yaw = 90.00°` at every heading tested (0, ±90, 180). This is what issue
  [001](001-imu-orientation-is-constant.md)'s `ros_yaw = π/2 − compass` rests on, and the
  probe confirms it reproduces `−carla_yaw` exactly.
- `physics_control` reports `max_steer_angle = 70.00°` for `vehicle.tesla.model3`, i.e.
  1.2217 rad — the value issue [006](006-hardcoded-max-steer-angle.md) used to hardcode
  as 1.22.
