# 019 — One constant converts every acceleration request into a pedal

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `apply_control_command`
**Status**: Fixed by measured pedal maps; verified over seven runs

## What was wrong

Autoware asks for an acceleration in m/s². CARLA takes a throttle and a brake, each 0 to 1.
The bridge converted between them by dividing by a single constant:

```rust
const MAX_ACCEL: f32 = 3.0;
control.throttle = (accel / MAX_ACCEL).clamp(0.0, 1.0);
control.brake    = (-accel / MAX_ACCEL).clamp(0.0, 1.0);   // the same divisor
```

That asserts three things, and measurement contradicts all three:

1. **The response is linear in the pedal.** It is not.
2. **It does not depend on speed.** It depends on speed enormously.
3. **The engine and the brakes have the same authority.** They do not -- brakes are far
   stronger, which is why the shared divisor is the worst part of this.

Measured on `vehicle.tesla.model3`, CARLA 0.9.16, by `scripts/probe_longitudinal.py`:

| | the constant assumes | measured at rest | measured at 24 m/s |
|---|---|---|---|
| throttle 1.0 | +3.00 | **+6.03** | **+6.00** |
| throttle 0.5 | +1.50 | +2.62 | +0.12 (at 14 m/s) |
| brake 1.0 | −3.00 | **−5.76** | **−11.19** |
| brake 0.0, coasting | 0 | −2.55 | −8.83 |

Half throttle produces 2.62 m/s² at rest and 0.12 m/s² at 14 m/s: the same pedal, a factor
of twenty apart. Full throttle delivers twice what the constant claims. Full brake delivers
between two and four times it, depending on speed.

The last row is the one most easily missed. CARLA's drag and engine braking are strong
enough that the car sheds 2.6 m/s² at rest and 8.8 m/s² at 24 m/s with **no pedal at all**,
so a request for gentle braking at speed needs no brake, and applying one overshoots.

## Fix

`longitudinal_map.rs` looks the pedal up in a measured table instead, in the shape Autoware's
own `raw_vehicle_cmd_converter` uses -- rows are pedal positions, columns are speeds, cells
are the acceleration that combination produces. `config/accel_map.csv` and
`config/brake_map.csv` ship alongside `vehicle_info.param.yaml` in `acb_vehicle_description`.

Which table a request goes to is decided by what the car does when left alone: above the
coasting curve it is a throttle request, below it a brake request. Requests beyond what the
vehicle can do at that speed saturate at the nearest pedal rather than failing.

Setting `accel_map_path` or `brake_map_path` to `none` restores the old constant, which is
how the two arms below were compared. A missing or malformed map logs an error and falls back
rather than refusing to drive.

## Verified

Seven scenario runs, `town01_ego_drive.xosc`, scored by `scripts/score_longitudinal.py`:
each command Autoware issued paired with the acceleration that followed it 0.3 s later,
derived from the bridge's own velocity report.

```
                 median |error|          range          mean bias    gain
  maps on  (n=4)     0.182 m/s^2     0.143 - 0.504       -0.25      1.64
  maps off (n=3)     0.592 m/s^2     0.520 - 0.613       -0.55      2.19
```

Every run with the maps beat every run without them; the ranges touch only at ON's worst
(0.504) against OFF's best (0.520). Median error falls by a factor of 3.3, and the systematic
bias -- the ego consistently decelerating harder than asked -- halves.

**What did not improve.** Worst-case error is larger with the maps (5.8 to 7.5 m/s² against
1.5 to 3.8), concentrated in transients at the standstill and gear boundaries where the
handbrake logic takes over from the pedal maps. Gain is closer to 1.0 but still well above
it, so delivered acceleration still exceeds what is asked on average; this issue improves
that relationship without closing it.

Both arms show one poor run out of three or four, with far more command samples and far fewer
usable pairs -- a run that went long. It appears in both arms, so it is run-to-run variation
rather than anything to do with the maps.

## The first map had fabricated cells (corrected 2026-08-28)

The map builder filled cells no ramp could reach by holding the last measured value. Its own
comment called that "the honest extrapolation". It was not, and the error ran the wrong way.

A low throttle cannot accelerate the car to high speed -- pedal 0.2 never gets past 3.6 m/s --
so every high-speed cell of every low-pedal row was invented. The table therefore claimed
throttle 0.2 produces **+0.010 m/s^2 at 24 m/s**, where the truth is **-6.53**: at that speed a
barely-open throttle is heavy deceleration, not gentle acceleration. Holding a value forward is
defensible along the pedal axis, where the response really does saturate. Along the speed axis
it inverts the sign of the physics.

The fix is to measure those cells rather than infer them. `probe_longitudinal.py` now runs
throttle ramps in both directions: up from rest, which is what a low pedal can do, and *down*
from the top of the range, which is the only way to learn what a low pedal does at speed. The
grid is now measured end to end, monotonic in both axes, and its coasting row (pedal 0.0)
agrees with the brake map's to within 0.1 m/s^2 -- two independent passes at the same physical
quantity.

### Verified against CARLA rather than against itself

The scoring used so far differentiates the bridge's velocity report, which is two inferences
deep. `scripts/validate_longitudinal.py` checks both against the server, as a passive observer
that never ticks:

```
1. reported speed vs CARLA speed        |difference| median 0.0000  p95 0.0004 m/s
2. differentiated report vs CARLA accel |error|      median 0.067   p95 0.276  m/s^2
```

The report is exact and the differentiation is sound, so the metric can be trusted. Against
CARLA's own acceleration, three runs each:

```
                        median |error|                gain
  fabricated cells       0.179 m/s^2                  2.18
  measured end to end    0.093 / 0.336 / 0.144        1.58 / 1.76 / 1.89
```

**Gain falls from 2.18 to about 1.76**, consistently across runs. The error medians overlap and
do not separate, so the honest claim is the gain, plus the fact that the corrected cells are
right where the old ones were wrong by 6.5 m/s^2.

### Still open

Gain remains near 1.8, so the ego still delivers close to twice the acceleration *variation*
Autoware asks for, and worst-case error is unchanged around 1.4 m/s^2 at p90. The pedal maps
are no longer the explanation -- they are measured, and the delivered acceleration follows them.
What remains points at the standstill and gear boundaries, where the handbrake logic overrides
the maps entirely, and at whatever lag sits between command and delivery. That wants its own
investigation.

## Measuring it again

`scripts/probe_longitudinal.py` holds one pedal constant and records (v, dv/dt) every tick
while the car ramps through the speed range. Its docstring records two earlier methods that
produced confident, wrong numbers and how each was caught -- worth reading before changing it,
because both looked plausible until their output was checked against an independent ramp.

The probe refuses to run while CARLA is synchronous, since settling a car needs the tick and
that belongs to `carla_scenario_bridge` during a scenario.

`scripts/build_longitudinal_maps.py` turns its samples into the two CSVs.
