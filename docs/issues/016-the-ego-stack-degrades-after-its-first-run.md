# 016 — The ego stack only passes its first scenario after a restart

**Severity**: High
**Component**: lateral control path (`vehicle_control.rs` steer scale, MPC feedback); run-order
dependence still unexplained
**Status**: Open

## The pattern, as far as it goes

The ego **stops mid-route and never resumes**, and the scenario hits its 180 s timeout.
That much is consistent. What predicts it is not.

The stall itself is now understood -- the ego departs its lane through a diverging steering
oscillation and wedges off-road, where the commanded acceleration cannot free it. See
"What the trajectory and perception instrumentation showed" below. What remains unexplained
is why this depends on how many scenarios the stack has already run.

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

## What the trajectory and perception instrumentation showed

`scripts/trace_run.py` now samples three more things once a second: the planned trajectory
(point count, distance to its end, arc length to its first zero-velocity point), the
predicted objects (count and distance to the nearest), and `/api/planning/velocity_factors`,
which is the planner stating its own reason for slowing.

Two runs on one freshly restarted stack, 2026-08-19. **The stop is not commanded by
anything.** At the stalled pose, held for 130 s:

```
 t   pose            vel   steer   cmd_a  tpts  tend  tstop  obj  onear  velocity_factors
 50  158.8 -116.3  -0.02   0.159   +0.43   162  32.8   34.1    0    nan                 -
```

The trajectory is 162 points reaching **32.8 m ahead**, and its first zero-velocity point is
at 34.3 m — its far end, not the vehicle. Control is commanding **+0.43 m/s²**. No velocity
factor is present. So the planner still intends to drive, the controller is still asking for
acceleration, and the vehicle does not move.

That exonerates the two suspects this section previously named. The trajectory is not
truncated and no perceived object is stopping the vehicle. In fact the *entire* velocity
factor record for a run is a single sample, `route-obstacle:APPROACHING@-5`, logged at the
instant of the lane departure and referring to a point 5 m **behind** the ego.

### The real failure is a diverging lateral oscillation

At one-second resolution the departure is unambiguous. The route is straight — spawn
`(190.8, -130.1)`, goal `(120.0, -129.8)` — so every metre of lateral movement below is
error:

```
 t   pose            vel   steer
 26  180.5 -128.8   4.11   0.037
 27  176.2 -128.4   4.32   0.267
 29  170.7 -130.5   1.90   0.017
 31  165.9 -132.2   3.55  -0.402
 32  161.9 -131.0   4.39  -0.578
 33  159.0 -126.7   5.01  -0.357
 35  159.0 -119.8   0.00
```

Steering amplitude grows 0.04 -> 0.27 -> -0.40 -> -0.58 rad while the lateral error grows
with it: -128.4, -132.2, -126.7, -119.8. That is a positive-feedback loop, not a vehicle
tracking a path. It ends 13.5 m off the lane at `(158.5, -116.3)`, and CARLA ground truth
agrees the vehicle is physically there.

**The stall is a consequence, not a cause.** Once off the road the ego is wedged, and
`+0.43 m/s²` becomes `0.43 / MAX_ACCEL = 0.14` throttle, which will not climb a kerb. The
vehicle then sits there commanding acceleration until the scenario times out.

### A sign error that is not one

Worth recording to save the next reader the false start: in these traces a *positive*
commanded tire angle moves the ego toward **-y**, which looks like an inverted steering
sign. It is not. The ego drives west (`h = 3.14159`, decreasing x), so its left hand points
toward -y. The conversion in `vehicle_control.rs` is correct.

### The second signature may not be a failure at all

The "stops 14 m short with near-perfect tracking" trace recorded above needs re-reading. In
the equivalent run here, `tend` shrinks monotonically — 29.9, 25.2, 21.6, 18.2, 14.4, 9.7,
6.5, 4.7 — which is exactly the distance to the goal at each step, so the trajectory was
never truncated. The ego reached `(124.7, -129.7)`, **4.7 m from the goal and inside the
scenario's 5.0 m `ReachPositionCondition` tolerance**, and the trace then froze: every
field, including CARLA ground truth and a non-zero 0.62 m/s velocity, identical for 25 s.
A simulation that stops ticking mid-motion is a scenario that has ended, not a vehicle that
has stopped. That run most likely passed.

This is not confirmed — the harness deletes `result.junit.xml` before each run and its
verdict echo was swallowed by pipe buffering, so run 1's verdict was lost. Fix the harness
to save each run's junit before drawing on this.

## Where this leaves the issue

The failure mode to explain is now specific: **the lateral controller goes unstable**, and
everything after that is consequence. Candidates, in order:

- The steer command scale ([006](006-hardcoded-max-steer-angle.md)). The command maps a tire
  angle through the wheel **limit** (70 deg) while the vehicle turns at the Ackermann
  **mean** (58.7 deg). That is an 18% loop-gain error. Low gain alone does not destabilise a
  loop, but it is a known-wrong constant sitting directly in the path.
- Actuator lag hidden from the controller ([009](009-steering-report-echoes-command.md)).
  With `report_measured_steering=false` the MPC is handed back its own command with zero lag
  while the CARLA wheel has real dynamics. A plant slower than the controller's model is a
  classic instability source, and this is exactly what 009 argued before its A/B was found
  to be confounded.
- Whatever makes it run-order dependent, which neither of the above explains on its own.

Note that stack freshness still predicts the outcome, and lateral instability does not
obviously depend on it. Fix 006 first, since it is wrong regardless, then A/B 009 properly.

**On method**, learned the hard way here: single-run comparisons cannot distinguish causes
in this system. Two conclusions were drawn and retracted this session — the orphaned-stream
storm ([015](015-sensors-destroyed-while-still-listening.md)) and the steering report
([009](009-steering-report-echoes-command.md)) — both from one run per arm, both explained
afterwards by run order or by nothing at all. Any future claim needs n >= 10 per condition
with run order held fixed.

## Mechanism at the stall, measured (2026-08-19)

Traced on a `town01_traffic_light.xosc` run (the y=-55.9 street, not y=-129.8) with
csb's `scripts/stall_probe.py`, which samples pose, trajectory, velocity factors and
perceived objects on one timeline. This is a single run, so it establishes **mechanism
only** -- no causal claim, per the method note above.

Three of this issue's leads are ruled out at the stall.

**The trajectory is not truncated.** It stays healthy and keeps commanding motion for as
long as the ego sits there:

```
t+26s ego(291.3,-54.4) 0.0 m/s traj[n=161 start=(295,-55) end=(261,-55) vmax=4.2 start_gap=4.1m]
t+57s ego(291.3,-54.2) 0.0 m/s traj[n=159 start=(295,-55) end=(261,-55) vmax=4.2 start_gap=3.9m]
                               factors[traffic-signal@186.2m/st1]  objs<30m=4
```

159 points reaching 30 m ahead at 4.2 m/s. **No velocity factor demands a stop** -- the
only one present is the traffic signal 186 m away. Four perceived objects within 30 m,
none of them stopping anything.

**Control is delivering, and the vehicle is wedged.** CARLA's own view of the actor:

```
hero carla=(291.5,54.1) yaw=-140.9
   speed=0.00  throttle=0.27  brake=0.00  steer=-0.02  hand_brake=False  gear=1
```

27% throttle, no brake, no hand brake, and no motion. So nothing is commanding a stop and
nothing is applying one; the vehicle is physically stuck.

**Localization is right, including heading.** At the same moment:

```
CARLA    pos=(291.4,-54.2)  yaw(ROS)=140.7
Autoware pos=(291.4,-54.1)  yaw=139.0
```

0.1 m and 1.7 deg of agreement. The ego therefore *knows* it is pointing 139 deg while its
lane runs 180 deg -- 41 deg out, nose into the kerb -- and lateral control is commanding
`steer=-0.02`, essentially straight. (At zero speed that may be a consequence rather than
a cause; most lateral controllers have no authority on a stopped vehicle.)

## What that leaves

The question is not what stops the ego. It is what turns it out of the lane, and the
approach shows a lateral oscillation that grows until the kerb ends it:

```
t+16s  -55.9    t+18s  -54.7    t+20s  -56.3    t+22s  -57.6
t+24s  -55.4    t+26s  -54.4    then stuck at -54.2
```

±1.5 m about a lane centre of -55.85, at 3-4.5 m/s, diverging over about 10 s. That is the
"lateral departure" signature of this issue, caught with pose, planning and control all
verified good at the same instant.

An untested hypothesis that fits it: steering command scaling. `CLAUDE.md` still lists
"vehicle calibration per CARLA model (steering multiplier, wheelbase)" as open, and an
over-large lateral gain produces exactly this -- growing oscillation, divergence, kerb.
Worth an n>=10 A/B against the steering multiplier with run order held fixed, which is the
only kind of comparison this issue has found trustworthy.
