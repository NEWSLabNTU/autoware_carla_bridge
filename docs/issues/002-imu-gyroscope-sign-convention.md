# 002 — IMU angular velocity has the wrong sign on x and y

**Severity**: Medium
**Component**: `src/acb_bridge/src/bridge/sensor_bridge.rs`, `publish_imu`
**Status**: Fixed

## What is wrong

The bridge published CARLA's gyroscope as `(x, -y, -z)`. The correct mapping is
`(-x, y, -z)`.

## Why

Converting CARLA's left-handed sensor frame (x forward, y right, z up) to ROS's
right-handed one (x forward, y left, z up) is the reflection `diag(1, -1, 1)`. Positions
and linear accelerations are ordinary vectors and transform as `(x, -y, z)` — which is
what `publish_imu` already does for the accelerometer, correctly.

Angular velocity is a pseudovector. Under a reflection `R` it transforms as
`det(R) · R ω`, and `det(diag(1,-1,1)) = -1`, giving:

```
ω_ros = −1 · (ω_x, −ω_y, ω_z) = (−ω_x, ω_y, −ω_z)
```

This matches `carla_ros_bridge`'s own IMU conversion, which negates x and z and keeps y.

## Why it did not break anything

Only the yaw rate (`z`) reaches Autoware's localization: `gyro_odometer` fuses
`VelocityReport.longitudinal_velocity` with `Imu.angular_velocity.z`. `z` was already
correct. The roll and pitch rates were sign-flipped, and nothing in this configuration
reads them.

## Fix

`publish_imu` now emits `(-gyro.x, gyro.y, -gyro.z)`, with a unit test asserting the
pseudovector rule so a future "tidy-up" cannot quietly restore the old signs.
