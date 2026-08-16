# 006 — Max steer angle hardcoded instead of read from CARLA

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`
**Status**: Fixed

## What is wrong

```rust
const MAX_STEER_ANGLE: f32 = 1.22;   // ~70 degrees
```

This constant is used in both directions: it scales Autoware's commanded tire angle into
CARLA's normalized `[-1, 1]` steer input, and it scales CARLA's steer value back into the
tire angle reported on `/vehicle/status/steering_status`.

1.22 rad is right for `vehicle.tesla.model3`, whose front wheels have a 70° limit. It is
wrong for every other blueprint — `vehicle.nissan.patrol` and the vans are lower, some
bikes higher — and `vehicle_config.yaml` explicitly offers alternative blueprints. A
mismatch scales every steering command by a constant factor: the vehicle under- or
over-steers by that ratio while both Autoware and the bridge believe the command was
delivered exactly.

## Why it did not show up

Every run so far used the Tesla, where the constant happens to be correct, and
`acb_vehicle_description/config/vehicle_info.param.yaml` declares
`max_steer_angle: 0.70` — a *planning* limit well inside CARLA's 1.22 physical limit, so
Autoware never commands an angle where the difference would be visible as saturation.

## Fix

Read the physical limit from the vehicle CARLA actually spawned:

```rust
let physics = vehicle.physics_control()?;
// Front wheels; CARLA reports max_steer_angle in degrees, per wheel.
let max_steer_rad = front_wheels.map(|w| w.max_steer_angle).max().to_radians();
```

Taken once when the control bridge is created, with a fall back to 1.22 rad and a warning
if the query fails, so a CARLA hiccup degrades to today's behaviour rather than to no
steering. Logged at startup so the value in use is visible in the run log.

## Measured, for the Tesla

`scripts/probe_carla_conventions.py` against CARLA 0.9.16:

```
physics max_steer_angle over wheels: 70.00 deg = 1.2217 rad

steer cmd=0.00  FL=  0.00  FR=  0.00  mean=  0.00 deg
steer cmd=0.25  FL= 15.02  FR= 17.50  mean= 16.26 deg (0.2838 rad)
steer cmd=0.60  FL= 30.98  FR= 42.00  mean= 36.49 deg (0.6369 rad)
steer cmd=1.00  FL= 47.43  FR= 70.00  mean= 58.71 deg (1.0247 rad)
```

So 1.22 rad was the right number for this blueprint, and the fix changes nothing for the
Tesla — it changes the answer for every other one.

## Not done

Two things this does not address:

- **A per-vehicle steering multiplier** (gap 1 in `docs/roadmap/9-gap-analysis.md`) is a
  separate calibration knob and still open. Reading the physical limit removes the part
  of that gap that is a correctness bug rather than a tuning preference.

- **The command mapping is linear in the wheel maximum, but the plant is not.** CARLA
  steers the wheels through an Ackermann geometry: at `steer = 1.0` the *inner* wheel
  reaches the 70 deg limit while the outer sits at 47 deg, so the bicycle-model angle the
  vehicle actually turns at is the 58.7 deg mean, not 70. `steer = angle / 1.2217`
  therefore under-delivers by ~13 % at the top of Autoware's 0.70 rad planning range. The
  loop closes over it now that issue [009](009-steering-report-echoes-command.md) reports
  the measured angle rather than the command, but a calibrated inverse (from
  `physics_control.steering_curve`, or a fitted table) would remove the steady-state
  error instead of asking the controller to integrate it away.
