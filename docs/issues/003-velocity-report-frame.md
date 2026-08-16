# 003 — VelocityReport is world-frame and unsigned

**Severity**: High
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `publish_status`
**Status**: Fixed

## What is wrong

`autoware_vehicle_msgs/VelocityReport` carries the vehicle's velocity **in `base_link`**
— the header even says so, and the bridge sets `frame_id: "base_link"`. It was filled
from CARLA's world-frame velocity instead:

```rust
let longitudinal_velocity = (vx² + vy² + vz²).sqrt();   // magnitude: never negative
let lateral_velocity = -velocity_vec.y;                 // world Y, not body Y
```

Two distinct defects:

1. **Longitudinal is a magnitude.** Reversing reports positive speed. Autoware's
   longitudinal controller and `gyro_odometer` both treat this as signed forward speed,
   so a reversing vehicle appears to be driving forward at the same rate.
2. **Lateral is world-frame.** `-vy` is the world Y component. It only coincides with
   body lateral velocity when the vehicle happens to face along world `+x`. Driving
   north at 10 m/s reported a "lateral" velocity of 10 m/s.

## Why the stack still worked

`vehicle_velocity_converter` and `gyro_odometer` consume `longitudinal_velocity` and
ignore `lateral_velocity`, and every scenario run so far has been forward-only, where
magnitude and signed speed agree. The bug is latent until something reverses — which
issue [004](004-gear-command-ignored.md) is about making possible.

## Fix

Project the world velocity onto the vehicle's own axes before publishing:

```rust
let body = transform.rotation.inverse_rotate_vector(&velocity);
longitudinal_velocity = body.x;     // signed: negative when reversing
lateral_velocity = -body.y;         // CARLA body y is right; ROS lateral is left
```

`Rotation::inverse_rotate_vector` is CARLA's own inverse rotation, so it is exact for
pitch and roll as well, not just yaw. Unit tests cover facing east, facing north, and
reversing.

## Note on `heading_rate`

`heading_rate` is `-ω_z` taken straight from `Actor::angular_velocity()`. The sign is
right; the units are the subject of issue [008](008-angular-velocity-units.md).
