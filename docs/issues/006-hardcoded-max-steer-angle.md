# 006 — Max steer angle hardcoded instead of read from CARLA

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`
**Status**: Fixed, including the Ackermann command mapping (2026-08-19)

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

## The command mapping, fixed (2026-08-19)

The remaining half of this issue -- that the mapping was linear in the wheel maximum while
the plant is not -- is now fixed, because it sits directly in the path of the lateral
instability in [016](016-the-ego-stack-degrades-after-its-first-run.md).

CARLA drives the **inner** wheel to `cmd * max_steer_angle` and places the outer wheel by
Ackermann geometry. Measured on a live server with `scripts/probe_steer_curve.py`, the inner
wheel is exactly `cmd * 70` deg and the outer follows
`cot(outer) = cot(inner) + track / wheelbase` to better than 0.05 deg over the whole range:

```
   cmd       FL       FR  mean_deg  mean_rad  rad/cmd
  0.10     6.56     7.00      6.78    0.1183   1.1831
  0.50    26.76    35.00     30.88    0.5390   1.0780
  1.00    47.43    70.00     58.71    1.0247   1.0247
```

`steer = angle / max_steer_angle` asks for the angle the *inner* wheel would reach, so it
under-delivers by 7-13% across Autoware's 0.70 rad planning range -- worse at larger angles,
which is where a lateral correction lives.

`vehicle_control.rs` now models that geometry and inverts it by bisection.
`track / wheelbase` comes from the wheel positions in `physics_control` (0.5548 for the
Tesla, against 0.5524 fitted from the measured angles), so it follows the blueprint like the
limit does. With no usable geometry it degrades to `track / wheelbase = 0`, which is exactly
the linear mapping this replaced. `/vehicle/status/steering_status` reports the same model
when it is echoing rather than measuring, so the report matches what was delivered.

Unit tests check the model against all eleven measured wheel angles, the round trip through
the inverse, and that the old mapping under-delivers.

### Measured but deliberately not modelled

`physics_control.steering_curve` declares a speed derating of
`[(0, 1.0), (20, 0.9), (60, 0.8), (120, 0.7)]` in km/h. It is real but smaller than declared:
holding `steer = 0.50`, the delivered wheel angle falls to 0.962 of its at-rest value by
7.3 km/h and 0.960 by 16.6 km/h, against the 0.917 the curve predicts at that speed.
Attempts to hold 40 and 60 km/h with `set_target_velocity` did not reach speed, so the top of
the curve is unmeasured.

A 4% speed-dependent term is worth having eventually -- gain that changes with speed is a
better instability candidate than a constant error -- but modelling it from two points, to a
curve the measurements already contradict, would add error rather than remove it. Left
unmodelled and recorded here.

## Not done

- **A per-vehicle steering multiplier** (gap 1 in `docs/roadmap/9-gap-analysis.md`) is a
  separate calibration knob and still open. Reading the physical geometry removes the part
  of that gap that is a correctness bug rather than a tuning preference.

- **The speed derating above.**
