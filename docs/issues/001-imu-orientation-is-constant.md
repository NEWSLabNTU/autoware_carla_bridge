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

## Measured live

`scripts/check_vehicle_interface.py` against a running `town01_ego_drive.xosc`, 2742
samples:

```
imu yaw vs base_link   mean |err| 3.1051 rad   worst 3.1092 rad
imu yaw spread         0.0811 rad
```

Two things to read out of that.

The **spread is non-zero**, which is the whole point: before the fix the orientation was
one of two constants and the spread would have been 0. It now tracks the vehicle's
heading change over the run.

The **offset is π, and that is correct.** `carla/imu_link` is mounted with `roll: π,
yaw: π` relative to `base_link` (`acb_sensor_kit_description/config/
sensor_kit_calibration.yaml`), so the sensor's own heading genuinely is
`vehicle_heading + π`. CARLA's compass reports the *sensor's* heading, and the message is
stamped `frame_id: carla/imu_link` — a consumer transforms it through TF. Comparing it
directly against `base_link` yaw is comparing two different frames.

The evidence that this is a mount offset rather than a sign error is that it stays
constant: across a 0.081 rad heading change the error moved by 0.004 rad. A sign error
would have made it vary by twice the heading change.

## Related

`orientation_covariance` is still all zeros. ROS convention is that element 0 set to `-1`
means "no orientation estimate"; all-zero means "covariance unknown", which is what this
is. Autoware's `imu_corrector` overrides both from its own parameters, so this is left
alone.
