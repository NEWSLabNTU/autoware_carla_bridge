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


## Reverted: it destabilises the lateral controller

Reporting the measured angle is the honest answer, and it is **off by default** anyway,
because measuring what it does to Autoware showed it breaks lane keeping here.

Same stack, same scenario, same build, one parameter changed:

| | measured (`report_measured_steering:=true`) | commanded (`false`) |
|---|---|---|
| lateral position | −130.1 → **−132.9** → **−126.3** | stays −129.4 … −129.9 |
| steering command | swings ±0.33 rad | ±0.013 rad |
| result | leaves the lane, wedges on the kerb, 180 s timeout | reaches the goal, **passes** |

The lane is at y = −129.8. In the measured arm the ego swings 6.6 m laterally in six
seconds at 3.6 m/s — a lateral loop oscillating with growing amplitude, not a car tracking
a path. It then sits with a sustained +0.5 m/s² command and zero motion, stuck against the
kerb it climbed.

### Why

The loop gain is wrong, and this change is what exposed it. The command maps a requested
tire angle onto CARLA's normalised steer by the wheel **limit** (70° for the Tesla), but
the vehicle turns at the Ackermann **mean** of its two front wheels — 58.7° at full lock,
per `scripts/probe_carla_conventions.py`:

```
steer cmd=0.25  FL= 15.02  FR= 17.50  mean= 16.26 deg
steer cmd=0.60  FL= 30.98  FR= 42.00  mean= 36.49 deg
steer cmd=1.00  FL= 47.43  FR= 70.00  mean= 58.71 deg
```

So honest feedback reports ~18% less angle than was asked for. MPC sees a persistent
shortfall, winds up against it, and with the actuator lag that the measured signal *also*
newly exposes, the loop goes unstable. Echoing the command hides both, which is why it was
stable — a perfect actuator is a lie, but a consistent one.

This is exactly the follow-up issue [006](006-hardcoded-max-steer-angle.md) left open:
*"a calibrated inverse would remove the steady-state error instead of asking the controller
to integrate it away."* Until that exists, honest feedback cannot be the default.

### What would make it safe

Map the command through the achieved-angle relationship rather than the wheel limit —
`cmd = desired_angle / radians(58.7)` for this vehicle, ideally derived at runtime rather
than hardcoded, since it is a property of each blueprint's Ackermann geometry. With
commanded and achieved agreeing, measured feedback becomes consistent and the flag can go
back to true. Re-run the A/B above to confirm before flipping it.

### Note on how this was found

It hid for a long time because it is *marginal*, not deterministic: earlier runs with
measured reporting passed. It surfaced as "the second run on a stack fails", and I spent
considerable effort attributing that to the orphaned-stream storm (issue 015) and to clock
rewinds, neither of which was the cause. A full-run trace of pose, velocity, steering and
CARLA ground truth is what settled it — `scripts/trace_run.py`.
