# 009 — SteeringReport echoes the command instead of the measured angle

**Severity**: Low
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `publish_status`
**Status**: Fixed

## What is wrong

```rust
let control = vehicle.control()?;              // the last control APPLIED
steering_tire_angle: -control.steer * MAX_STEER_ANGLE
```

`Vehicle::control()` returns the `VehicleControl` last handed to the actor, not anything
the physics produced. `/vehicle/status/steering_status` therefore reported the commanded
steer with zero lag and zero error — a perfect actuator.

## Why it matters

Autoware's lateral controller (MPC) closes a loop on this topic. Feeding it back the
command it just issued removes the actuator dynamics from the loop entirely: the
controller is tuned against a plant that responds instantly, and any steering-lag
compensation it applies is compensating for nothing. It also makes the simulation useless
for measuring steering tracking error, which is one of the things a simulator is for.

CARLA exposes the real value: `Vehicle::get_wheel_steer_angle(wheel_location)` returns
the physical wheel angle in degrees, and carla-rust wraps it as `wheel_steer_angle`.

## Fix

Report the average of the two front wheels' measured angles, converted to radians and
sign-flipped for ROS. Falls back to the commanded value with a rate-limited warning if
the query fails, so a CARLA version without the API degrades rather than stops.

## Cost

Two extra RPCs per publish cycle at 20 Hz. Measured at well under a millisecond against a
local server; the reads are not batched with anything else, so this is worth re-checking
if the cycle ever gets tight.
