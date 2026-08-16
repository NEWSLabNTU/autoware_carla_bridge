# 001 — IMU orientation is a constant, not the heading

**Severity**: High
**Component**: `src/acb_bridge/src/bridge/sensor_bridge.rs`, `publish_imu`
**Status**: Fixed

## What is wrong

The IMU's orientation quaternion was derived from CARLA's compass reading like this:

```rust
let yaw = compass.atan2(-compass);
```

`compass` is a scalar — the sensor's heading in radians. `atan2(c, -c)` does not depend
on the magnitude of `c` at all: it is `3π/4` for every positive reading and `-π/4` for
every negative one. The published `sensor_msgs/Imu.orientation` was therefore one of two
fixed quaternions no matter which way the vehicle pointed.

## Why it matters

`sensor_msgs/Imu.orientation` is the field Autoware reads when
`use_gnss_ins_orientation` is true, and it is what any consumer of the raw IMU topic
will trust. The current CARLA profile sets `use_gnss_ins_orientation: false`, so
`gnss_poser` derives heading from GNSS motion instead and localization survives — which
is exactly why this went unnoticed. Anything that turns that parameter on, or that reads
`/sensing/imu/*/imu_raw` directly, gets a heading that never moves.

## The correct conversion

CARLA's compass is measured clockwise from north, in radians. The map frame this bridge
publishes into has `x` = CARLA `x` (east) and `y` = `-CARLA_y` (north), so ROS yaw is
measured counter-clockwise from east:

```
ros_yaw = π/2 − compass
```

Two checks against the frame conventions already documented in `CLAUDE.md`:

- Vehicle facing CARLA `+x` (east): CARLA yaw `0`, compass `π/2`, so `ros_yaw = 0`. ROS
  yaw `0` is `+x`, which is east. Agrees.
- Vehicle facing CARLA `-y` (north): CARLA yaw `-90°`, compass `0`, so `ros_yaw = π/2`.
  The bridge's pose conversion gives `ros_yaw = -carla_yaw = +90°`. Agrees.

Roll and pitch stay zero: CARLA's compass carries heading only.

## Fix

`publish_imu` computes `yaw = FRAC_PI_2 - compass` and builds the quaternion from it.
Unit tests in `sensor_bridge.rs` pin the two checks above plus north, so the constant can
never come back silently.

## Related

`orientation_covariance` is still all zeros. ROS convention is that element 0 set to `-1`
means "no orientation estimate"; all-zero means "covariance unknown", which is what this
is. Autoware's `imu_corrector` overrides both from its own parameters, so this is left
alone.
