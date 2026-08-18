# 016 — The ego stack only passes its first scenario after a restart

**Severity**: High
**Component**: not yet identified; Autoware-side state carried across runs is the lead
**Status**: Open

## The pattern

Across every run measured on 2026-08-18, with several different builds:

| stack state | verdict |
|---|---|
| first scenario after `just ego-av` | **PASS** |
| second and later scenarios on the same stack | **FAIL**, 180 s timeout |

Samples: demo run PASS; four-run samples that went PASS/FAIL/FAIL/FAIL twice; a two-run
sample PASS/FAIL; a freshly restarted stack PASS; and three consecutive runs on an aged
stack **FAIL/FAIL/FAIL**. The one exception in the record is a PASS/PASS/FAIL, which puts
the boundary after run 2 rather than run 1 that time.

This is what phase 007's "second run on one stack" refers to, and it is still open.

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

- Trace a failing run with `scripts/trace_run.py` and compare `/localization/kinematic_state`
  against CARLA ground truth from the first frame, not just after convergence.
- Check whether the concealer re-initialises localization per run
  (`/api/localization/initialize`), or whether GNSS auto-init is left to reconcile a stale
  EKF.
- Try clearing localization between runs and see whether the pass rate follows.

Every measurement must treat "first run after restart" as a separate condition; mixing the
two is what produced the wrong conclusion in 009.
