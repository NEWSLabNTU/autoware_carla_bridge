# 009 — SteeringReport echoes the command instead of the measured angle

**Severity**: Low
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `publish_status`
**Status**: Available behind a flag, default off — and the default is now measured, not historical: enabling it degrades lateral tracking by 67x (2026-08-30)

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


## The A/B, run properly: a null (2026-08-20)

The parameter is now forwarded through `ego_av.launch.xml`, so the arms differ in exactly
one thing and no rebuild is needed between them.

**Outcome.** Not pass/fail, which barely discriminates here -- fresh stacks pass almost
always, and one traced FAIL had a peak cross-track of 0.24 m while a PASS had 3.44 m.
Instead, lateral tracking quality over the straight approach: the spread and peak of the
ego's cross-track error, plus its deviation from the lane centre, scored by
`scripts/score_tracking.py` from `trace_run.py` output.

**Design.** Arms alternated across freshly restarted stacks, each stack contributing a run 1
and a run 2, so stack age is balanced across arms rather than confounded with them. The arm
actually launched is confirmed per stack from play_launch's own
`play_log/ego/<dir>/node/acb_bridge/params_files/overrides.yaml`.

**Result**, cross-track standard deviation in metres, 13 usable trials:

```
  run1 measured   n=4 median 0.019  range 0.016-0.051
  run1 commanded  n=3 median 0.100  range 0.017-0.188
  run2 measured   n=4 median 1.538  range 0.158-2.051
  run2 commanded  n=2 median 0.983-1.424
```

**The arms do not separate.** The direction reverses between run indices -- measured looks
better on run 1, commanded on run 2 -- and the ranges overlap in both cells, with commanded's
best run 1 (0.017) matching measured's best (0.016). Nothing here supports an effect in
either direction.

**What does separate, by 59x**, is run order, pooled across arms:

```
  run 1 median 0.021 (n=7)      run 2 median 1.234 (n=6)
```

So the honest reading is that this parameter's effect, if any, is small enough to be
invisible against an effect 59 times larger. That specifically does **not** support this
issue's original claim that measured feedback destabilises lateral control, and equally does
not support the converse. The default stays `false` because it is the historical behaviour,
not because it is better.

At n = 4 and 3 stacks per arm a modest effect cannot be excluded. Anyone resuming should
hold run index fixed and raise n, and should expect to need many more runs than this to see
anything under the run-order effect. The lesson from
[016](016-the-ego-stack-degrades-after-its-first-run.md) applies here too: the run-order
effect is the thing worth explaining, and it is where the measured ~478 ms command-to-wheel
lag points.

### Trials lost, and why

Twelve of 25 trials produced no data. Two stacks never localized at all, and the run
stopped early when another user's job on this shared host grew to 83 GB of its 125 GB and
Autoware stacks stopped coming up. Those rows are recorded as `no-data` and excluded rather
than scored as zeros.


## Re-run on uncontaminated runs: still a null on pass rate (2026-08-21)

The A/B above is void: every one of its second runs shared the host with the previous run's
orphaned `openscenario_interpreter` ([016](016-the-ego-stack-degrades-after-its-first-run.md)),
so the arms were compared across runs that were not equivalent. Re-run with orphan cleanup
active, one arm per stack and four runs per stack, arms alternating so stack-to-stack
variation spreads over both, and the arm verified per stack from play_launch's own parameter
file:

```
measured    pass 3/8   median xt_sd 1.319   range 0.105-2.416
commanded   pass 3/8   median xt_sd 0.157   range 0.018-1.704
```

**Identical pass rates.** The tracking medians differ by a factor of eight in favour of
echoing the command, which is the direction this issue originally claimed, but the ranges
overlap heavily and the run-order effect that dominates 016 is far larger than the gap:
pooled across arms, run 1 scores 0.068 and run 2 scores 1.212, an order of magnitude apart.
With eight data runs per arm sitting inside that, the difference is not established.

So the default stays `false`, still for the historical reason rather than a measured one.
Settling this needs the 016 split fixed first -- otherwise most of the variance in any arm
comparison belongs to run order rather than to the parameter.

## Settled: measured feedback destabilises lateral control (2026-08-30)

Two earlier attempts at this comparison were void, and this issue's own text set the conditions for a
third: fix the run-order effect first, hold run index fixed, and raise n. All three are now
possible. `substeps: 2` closed issue 016, the bridge no longer strands itself between
scenarios, and `scripts/acceptance.py` reports a cross-track number per run, so the outcome is
measured rather than eyeballed.

**Design.** Four freshly restarted ego stacks, arms alternating across them
(measured, commanded, measured, commanded), three consecutive scenario runs each. Every run
index appears in both arms, so the run-order effect that voided the first attempt cannot be
confused with the parameter. The arm was read back from play_launch's own
`params_files/overrides.yaml` on every stack, never inferred from behaviour; all four matched
what was requested.

**Result**, median distance from the planned trajectory:

```
arm          n   passed   cross-track (m)                  longitudinal (m/s^2)
measured     6    0/6     median 4.037   range 2.187-13.277   median 0.370
commanded    6    5/6     median 0.060   range 0.036- 0.134   median 0.057
```

**The arms separate by 67x in the median and do not overlap at all** -- the best measured run,
2.187 m, is sixteen times worse than the worst commanded run, 0.134 m. And the separation holds
at every run index, which is the check the earlier attempts failed:

```
         measured   commanded
  run 1    3.522      0.038
  run 2    8.743      0.095
  run 3    3.608      0.077
```

Peak speed was 4.17 to 5.40 m/s in both arms, so this is not a stack that failed to drive. The
ego drives; it wanders. One measured run reached **13.3 m** from its trajectory.

The single commanded-arm failure was an unrelated `pose_instability_detector` crash, caught by
a different check; its cross-track was 0.090 m. On tracking quality the arms are 6/6 against
6/6.

### What this means for the issue

The original claim here -- that feeding back the measured wheel angle destabilises the lateral
loop -- **was right**, and was retracted too readily. The retraction was still correct at the
time: the evidence then was one run per arm with the arms confounded by stack freshness, which
is no evidence at all. Being right for bad reasons is not being right.

The default stays `false`, but no longer "because it is the historical behaviour". It is the
measured one.

The mechanism is most likely the scale mismatch this issue already described under "What is
still true": the command maps a tire angle through the wheel **limit** while the vehicle turns
at the Ackermann **mean**, so honest feedback reads about 18% below the command and the
controller integrates against a plant it mismeasures. Issue 006 fixed the command mapping;
whether that leaves a residual mismatch in the *report* has not been checked, and would be the
next thing to look at if anyone wants this parameter usable rather than merely off.

### Enabling it

`REPORT_MEASURED_STEERING=true just ego-av` still works and is still worth having for anyone
studying the actuator, but it should be understood as a diagnostic rather than a fidelity
improvement: with it on, this stack cannot hold a lane.
