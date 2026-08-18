# 016 — The ego stack only passes its first scenario after a restart

**Severity**: High
**Component**: not yet identified; Autoware-side state carried across runs is the lead
**Status**: Open

## The pattern, as far as it goes

The ego **stops mid-route and never resumes**, and the scenario hits its 180 s timeout.
That much is consistent. What predicts it is not.

Verdicts collected on 2026-08-18 across several builds:

| condition | passes |
|---|---|
| first scenario after an `ego-av` restart | 6 of 7 |
| second and later scenarios on the same stack | 2 of 12 |

Run order clearly matters, but it is **not deterministic in either direction**: one run 2
passed, and one first-run-after-restart failed. An earlier version of this issue stated the
split as a rule; it is a tendency.

Two distinct signatures have been traced, both ending in the same stall:

**With lateral departure** — the ego oscillates out of its lane and wedges on the kerb:

```
t=23  pose (190.8,-130.1)  vel 0.00
t=35  pose (165.3,-132.9)  vel 3.62   <- 3 m one way
t=41  pose (158.3,-126.3)  vel 0.02   <- 6.6 m back the other way, in 6 s
t=48+ pose (158.4,-126.3)  vel 0.00   <- +0.5 m/s2 commanded, no motion
```

**Without** — lane tracking is near perfect and it stops anyway, 14 m short of the goal:

```
t=25  190.8 -130.0  vel 0.00
t=50  154.4 -129.4  vel 4.81          <- within 0.4 m of the lane centre
t=58  133.7 -129.1  vel 0.00          <- stops
t=74  133.7 -129.1  vel 0.00          <- never resumes
```

The second one rules out lateral control as *the* cause: something stops the vehicle while
it is tracking the path correctly. Localization is not the cause either — in the traced
failures Autoware's pose agrees with CARLA ground truth throughout (`(157.8,-128.1)` against
truth `(159,-128)`).

## What it is not

- **Not the orphaned-stream storm** ([015](015-sensors-destroyed-while-still-listening.md)).
  That is fixed and verified at zero; the three consecutive failures above all recorded
  `storm_delta=0`.
- **Not CARLA.** The server stays up, the world is clean between runs, and a fresh CARLA
  is not required — restarting only the ego stack is enough to get a pass.
- **Not the sensor teardown.** The release protocol acknowledges and stops cleanly on every
  run.
- **Not `SteeringReport` reporting the measured angle** ([009](009-steering-report-echoes-command.md)).
  That was the first conclusion and it was wrong: the A/B behind it had one run per arm and
  the arms differed in stack freshness as well as in the parameter. Aged stacks fail with
  *either* setting.

## Symptoms worth chasing

From `scripts/trace_run.py` on a failing run — pose, velocity, steering and CARLA ground
truth once a second:

```
t=23  pose (190.8,-130.1)  vel  0.00   ← spawn, on the lane
t=29  pose (183.9,-129.6)  vel  3.41
t=35  pose (165.3,-132.9)  vel  3.62   ← 3 m one way
t=41  pose (158.3,-126.3)  vel  0.02   ← 6.6 m back the other way, in 6 s
t=48+ pose (158.4,-126.3)  vel  0.00   ← stopped, +0.5 m/s² commanded, never moves
```

Two things to explain:

1. **The ego oscillates out of its lane** and ends up wedged against the kerb, with a
   sustained positive acceleration command and no motion.
2. **Localization initialises from a stale pose.** On later runs the first
   `/localization/kinematic_state` values are nowhere near the spawn point: `(155.8,
   -116.3)`, `(158.25, …)` and `(118.54, -110.32)` were all observed where the scenario
   spawns the ego at `(190.8, -130.1)`. `(118, -110)` is close to where the *previous* run
   ended.

The obvious hypothesis, untested: Autoware's localization carries state across runs, so a
re-spawned ego initialises against the old estimate, NDT converges slowly or to the wrong
place, and planning and control then act on a pose that does not match the vehicle. That
would explain both symptoms without anything being wrong in the bridge.

## How to investigate

The vehicle stops while correctly localized and correctly on its path, so look at what
would command a stop:

- `/planning/scenario_planning/trajectory` — does it still reach the goal, or does it end
  at the stopping point? A truncated trajectory is the likeliest single explanation.
- `/perception/object_recognition/objects` — is something being perceived in the lane?
  `lidar_detection_model` is `clustering` here, a rule-based detector that can promote
  kerbs and walls to obstacles.
- `/planning/scenario_planning/status/stop_reason`, and the behaviour-velocity modules.
- The background AV `bg_av_1`, parked at (230, -130). It is behind the ego for this
  scenario, but confirm it is not being perceived.

**On method**, learned the hard way here: single-run comparisons cannot distinguish causes
in this system. Two conclusions were drawn and retracted this session — the orphaned-stream
storm ([015](015-sensors-destroyed-while-still-listening.md)) and the steering report
([009](009-steering-report-echoes-command.md)) — both from one run per arm, both explained
afterwards by run order or by nothing at all. Any future claim needs n >= 10 per condition
with run order held fixed.
