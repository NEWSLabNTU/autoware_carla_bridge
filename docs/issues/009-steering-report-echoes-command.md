# 009 — SteeringReport echoes the command instead of the measured angle

**Severity**: Low
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `publish_status`
**Status**: Reverted — the fix destabilised lateral control; available behind a flag

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


## Default reverted, but NOT for the reason first written here

`report_measured_steering` defaults to **false** (echo the command). The honest reading of
why is: *no evidence either way*, so the default is the historical behaviour until someone
runs a controlled test.

### The A/B that motivated this was confounded

It looked decisive. One parameter changed, everything else identical:

| | measured | commanded |
|---|---|---|
| lateral position (lane at −129.8) | −130.1 → −132.9 → −126.3, departs | −129.4 … −129.9 |
| steering command | ±0.33 rad | ±0.013 rad |
| result | wedged on kerb, 180 s timeout | reached the goal |

**It was one run per arm, and the arms differed in something else.** The measured arm ran
as the second-or-later scenario on an ego stack that had been up a while; the commanded arm
ran as the *first* scenario after an `ego-av` restart. Every run this session, of either
kind, follows that split — see issue
[016](016-the-ego-stack-degrades-after-its-first-run.md). Three consecutive runs on an aged
stack with **commanded** steering then failed 3/3, which is what the parameter was supposed
to prevent.

So the difference I measured is explained by stack freshness, and this parameter is
unproven in both directions.

### What is still true

The oscillation seen in the measured arm was real: a 6.6 m lateral swing in six seconds at
3.6 m/s is a lateral loop going unstable, not a car tracking a path. Whether measured
feedback caused it, or merely coincided with a degraded stack, is not established.

There is also a genuine scale mismatch waiting underneath, from
[006](006-hardcoded-max-steer-angle.md): the command maps a tire angle by the wheel
**limit** (70°) while the vehicle turns at the Ackermann **mean** (58.7° at full lock, per
`scripts/probe_carla_conventions.py`). Honest feedback therefore reads ~18% below the
command, which is a real loop-gain error regardless of what caused these particular
failures.

### How to settle it

Fix the command mapping first (006), then A/B this flag **with every run as run 1 on a
freshly restarted ego stack**, several runs per arm. Anything less is measuring stack age.
