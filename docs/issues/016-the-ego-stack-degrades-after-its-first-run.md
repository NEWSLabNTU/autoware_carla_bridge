# 016 — The ego stack only passes its first scenario after a restart

**Severity**: High
**Component**: lateral control path (`vehicle_control.rs` steer scale, MPC feedback); run-order
dependence still unexplained
**Status**: Open

## The pattern, as far as it goes

The ego **stops mid-route and never resumes**, and the scenario hits its 180 s timeout.
That much is consistent. What predicts it is not.

The stall itself is now understood, and confirmed by two independent traces on two different
scenarios: the ego departs its lane through a diverging steering oscillation and wedges
off-road, where the commanded throttle cannot free it. What remains unexplained is why this
depends on how many scenarios the stack has already run.

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

## The mechanism, measured twice independently (2026-08-19)

Two traces, taken separately and on different scenarios, agree on every point below. One is
`scripts/trace_run.py` on `town01_ego_drive.xosc` (the y=-129.8 street), which samples the
planned trajectory, the predicted objects and `/api/planning/velocity_factors` alongside
pose and control. The other is csb's `scripts/stall_probe.py` on
`town01_traffic_light.xosc` (the y=-55.9 street), which adds CARLA's own view of the applied
`VehicleControl`. Each is a single run, so together they establish **mechanism, not cause**.

### Nothing commands the stop

At the stalled pose, held for over two minutes:

```
 t   pose            vel   steer   cmd_a  tpts  tend  tstop  obj  onear  velocity_factors
 50  158.8 -116.3  -0.02   0.159   +0.43   162  32.8   34.1    0    nan                 -
```

```
t+26s ego(291.3,-54.4) 0.0 m/s traj[n=161 start=(295,-55) end=(261,-55) vmax=4.2 start_gap=4.1m]
t+57s ego(291.3,-54.2) 0.0 m/s traj[n=159 start=(295,-55) end=(261,-55) vmax=4.2 start_gap=3.9m]
                               factors[traffic-signal@186.2m/st1]  objs<30m=4
```

**The trajectory is not truncated.** 162 points reaching 32.8 m ahead in one trace, 159
points reaching 30 m at 4.2 m/s in the other, both still commanding motion for as long as
the ego sits there. The first zero-velocity point is at 34.3 m -- the trajectory's far end,
not the vehicle.

**No velocity factor demands a stop.** The second trace's only factor is a traffic signal
186 m away. The first run's *entire* velocity-factor record is a single sample,
`route-obstacle:APPROACHING@-5`, logged at the instant of the lane departure and referring
to a point 5 m **behind** the ego. Perceived objects are present and stopping nothing.

Both of the leads this issue previously named -- a truncated trajectory, and the rule-based
`clustering` detector promoting a kerb to an obstacle -- are therefore ruled out at the
stall.

### The vehicle is physically wedged

CARLA's own view of the actor settles it:

```
hero carla=(291.5,54.1) yaw=-140.9
   speed=0.00  throttle=0.27  brake=0.00  steer=-0.02  hand_brake=False  gear=1
```

27% throttle, no brake, no hand brake, no motion. Control is delivering and the vehicle
cannot move. The same holds in the other trace by arithmetic: `+0.43 m/s²` becomes
`0.43 / MAX_ACCEL = 0.14` throttle, which will not climb a kerb.

### Localization is right at the stall, including heading

```
CARLA    pos=(291.4,-54.2)  yaw(ROS)=140.7
Autoware pos=(291.4,-54.1)  yaw=139.0
```

0.1 m and 1.7 deg of agreement. The ego therefore *knows* it is pointing 139 deg while its
lane runs 180 deg -- 41 deg out, nose into the kerb -- and lateral control is commanding
`steer=-0.02`, essentially straight. At zero speed that is likely a consequence rather than
a cause; most lateral controllers have no authority on a stopped vehicle.

### The real failure is a diverging lateral oscillation

The stall is a consequence. What precedes it, in both traces, is a steering loop going
unstable. On the straight route -- spawn `(190.8, -130.1)`, goal `(120.0, -129.8)` -- every
metre of lateral movement below is error:

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
with it: -128.4, -132.2, -126.7, -119.8. It ends 13.5 m off the lane at `(158.5, -116.3)`,
and CARLA ground truth agrees the vehicle is physically there.

The other scenario shows the same shape about its own lane centre of -55.85:

```
t+16s  -55.9    t+18s  -54.7    t+20s  -56.3    t+22s  -57.6
t+24s  -55.4    t+26s  -54.4    then stuck at -54.2
```

±1.5 m, at 3-4.5 m/s, diverging over about 10 s. That is a positive-feedback loop, not a
vehicle tracking a path.

### A sign error that is not one

Worth recording to save the next reader the false start: in the first trace a *positive*
commanded tire angle moves the ego toward **-y**, which looks like an inverted steering
sign. It is not. The ego drives west (`h = 3.14159`, decreasing x), so its left hand points
toward -y. The conversion in `vehicle_control.rs` is correct.

### The second signature may not be a failure at all

The "stops 14 m short with near-perfect tracking" trace recorded earlier in this issue needs
re-reading. In the equivalent run here, `tend` shrinks monotonically -- 29.9, 25.2, 21.6,
18.2, 14.4, 9.7, 6.5, 4.7 -- which is exactly the distance to the goal at each step, so the
trajectory was never truncated. The ego reached `(124.7, -129.7)`, **4.7 m from the goal and
inside the scenario's 5.0 m `ReachPositionCondition` tolerance**, and the trace then froze:
every field, including CARLA ground truth and a non-zero 0.62 m/s velocity, identical for
25 s. A simulation that stops ticking mid-motion is a scenario that has ended, not a vehicle
that has stopped. That run most likely passed.

This is not confirmed -- the harness deletes `result.junit.xml` before each run and its
verdict echo was swallowed by pipe buffering, so that run's verdict was lost. Fix the
harness to save each run's junit before drawing on this.

## Ruled out: the steer command scale

[006](006-hardcoded-max-steer-angle.md)'s Ackermann mapping fix landed and changed nothing
here. Run 1 PASS, run 2 FAIL, with the failing run's oscillation the same shape and the same
amplitude as before it. In hindsight the prediction was available beforehand: the old
mapping under-delivered, which is low loop gain, and low gain is stabilising.

## Ruled out: the burst of perceived objects

A previous version of this issue offered the object burst as the best lead, on the strength
of a passing run never exceeding 3 objects while a failing one reached 40 as it left its
lane. Dumping the objects settles it, and the lead was wrong -- the causality runs the other
way.

`scripts/dump_objects.py` prints each object in the ego's own frame. During the burst, on a
third (failing) run:

```
t=40 ego(160.7,-120.3) yaw=82.0  tracked=13  clustering=54
    lon=  -1.0 lat=   0.0 d=  1.0 UNKNOWN p=1.00 exist=0.94 size=( 0.0, 0.0, 0.1)
    lon=   2.4 lat=  31.2 d= 31.3 UNKNOWN p=1.00 exist=0.60 size=( 0.0, 0.0, 1.0)
    lon=   6.7 lat= -17.4 d= 18.7 UNKNOWN p=1.00 exist=0.94 size=( 0.0, 0.0, 0.4)
    lon= -65.0 lat=  18.1 d= 67.5 UNKNOWN p=1.00 exist=0.98 size=( 0.0, 0.0, 1.5)
    lon= -58.3 lat= -60.2 d= 83.8 UNKNOWN p=1.00 exist=0.81 size=( 0.0, 0.0, 2.4)
```

Every object is either the ego's own body (`lon=-1.0, lat=0.0`, a 0.1 m high box one metre
behind it) or roadside clutter 20-80 m off to the sides and behind. All are `UNKNOWN` with
**zero length and zero width** -- degenerate clusters, not detections of anything. Nothing
in that set would make a planner swerve.

The order of events points the same way. At 1 Hz:

```
t=27 ego(188.7,-130.1) yaw= 180.0 clustering= 3    on the lane centre, heading west
t=28 ego(185.7,-130.0) yaw= 178.5 clustering= 3    yaw already moving, clusters unchanged
t=29 ego(182.4,-129.5) yaw= 168.0 clustering=28    10 deg/s yaw rate, then the burst
```

The yaw departs first and the clusters follow it. As the car rotates, the LiDAR sweeps the
kerb and buildings at a new angle and ground segmentation leaks, so cluster count is a
*measure* of the departure rather than its cause. The count then tracks the yaw error all
the way out: 3, 28, 19, 44, 34 ... 66 at 36 degrees off.

This is the third lead in this issue to be raised and then withdrawn, and the second where a
real correlation turned out to point backwards.

## What is left

The ego yaws 10 degrees in one second while sitting on the lane centre of a straight route,
correctly localized, with nothing ahead of it. The controller is issuing the large steering
that produces that. The open question is whether it is being *asked* to.

The discriminator is the planned path itself, not its endpoints: sample the first several
trajectory points in the ego frame and see whether the path is straight down the lane while
the controller steers off it (a control fault) or whether the path itself bends (a planning
fault). `tend` jumping from 31 to 44 m at the departure says the trajectory changed at that
moment; it does not say into what shape.

## Still unexplained

**Why it depends on run order.** Nothing above explains why a freshly restarted stack
usually passes and a used one usually does not. Lateral instability has no obvious reason to
care how many scenarios preceded it.

**A stale initial pose on later runs.** The first `/localization/kinematic_state` values on
later runs are nowhere near the spawn point: `(155.8, -116.3)`, `(158.25, ...)` and
`(118.54, -110.32)` were observed where the scenario spawns the ego at `(190.8, -130.1)`.
`(118, -110)` is close to where the *previous* run ended. Localization is correct by the
time of the stall, but a bad initial estimate would give the controller a large transient to
reject -- which is exactly the disturbance an unstable loop needs to diverge. That would
link the two halves of this issue, and it is untested.

## Where this leaves the issue

The question is no longer what stops the ego. It is what turns it out of the lane.
Candidates, in order:

- **The steer command scale** ([006](006-hardcoded-max-steer-angle.md)). The command maps a
  tire angle through the wheel **limit** (70 deg) while the vehicle turns at the Ackermann
  **mean** (58.7 deg). `CLAUDE.md` still lists "vehicle calibration per CARLA model
  (steering multiplier, wheelbase)" as open, and a mis-scaled lateral gain produces exactly
  this -- growing oscillation, divergence, kerb. Fix this first; it is wrong regardless.
- **Actuator lag hidden from the controller**
  ([009](009-steering-report-echoes-command.md)). With `report_measured_steering=false` the
  MPC is handed back its own command with zero lag while the CARLA wheel has real dynamics.
  A plant slower than the controller's model is a classic instability source, and this is
  what 009 argued before its A/B was found to be confounded.
- **Whatever makes it run-order dependent**, for which the stale initial pose above is the
  only lead.

**On method**, learned the hard way here: single-run comparisons cannot distinguish causes
in this system. Two conclusions were drawn and retracted this session — the orphaned-stream
storm ([015](015-sensors-destroyed-while-still-listening.md)) and the steering report
([009](009-steering-report-echoes-command.md)) — both from one run per arm, both explained
afterwards by run order or by nothing at all. Any future claim needs n >= 10 per condition
with run order held fixed.

## The steering A/B: no effect, and the failure did not reproduce (2026-08-19)

Autoware's `vehicle_info.param.yaml` declares `max_steer_angle: 0.70` rad while CARLA's
Model 3 reports 70 deg = **1.222 rad** on its steered wheels, so the controller plans and
tracks believing the car can steer 40 deg when it can actually do 70. That mismatch was
the last standing version of the "steering calibration" hypothesis, and it is now tested.

First, what is *not* wrong. acb converts a commanded tire angle to CARLA's [-1,1] by
dividing by the limit it reads from CARLA physics (`vehicle_control.rs`:
`control.steer = -steering_tire_angle / max_steer_angle`). That mapping is
self-consistent: ask for 20 deg and the wheels go to 20 deg. There is no gain error and
no "steering multiplier" to mis-set, so an over-steering explanation for the lateral
divergence has nothing to stand on.

The A/B varied the remaining knob, Autoware's assumed limit, 20 runs alternating arms,
**every run preceded by an `ego-av` restart** so that stack age -- the confound that
sank the earlier single-run comparisons -- is held constant at "first run after restart":

| arm | `max_steer_angle` | stack failures | reached the stop line | min x reached |
|---|---|---|---|---|
| A | 0.70 rad (current) | 1 of 10 | **8 of 9** | mean 105.03, sd 0.72 |
| B | 1.22 rad (CARLA's real limit) | 1 of 10 | **9 of 9** | mean 104.53, sd 0.71 |

The outcome is the minimum x the ego reaches, sampled from CARLA during the run, because
this scenario times out by design while the ego waits at a red -- a healthy run and a
wedged one both score `failures=1`, so the junit verdict cannot separate them.

**No detectable effect.** 8/9 against 9/9 is a coin flip at this n, and the reached runs
land in a 103.7-106.1 m band in both arms with identical spread. At n=10 per arm this
design would catch a large effect; it would miss a subtle one, so the honest reading is
"no effect large enough to matter here", not "no effect".

**The more useful result is the one the A/B was not looking for: the failure barely
occurred.** Across 18 valid runs there was **not a single lateral-departure stall** -- no
oscillation, no drift, no kerb wedge. 17 of 18 reached the stop line. The one that did not
(run 7) sat at x=320.0, which is the spawn point: the ego never moved at all, an engage
failure rather than the stall this issue is about.

That is direct support for this issue's own framing. With a fresh stack for every run the
pass rate is 17/18 (94%), consistent with the 6/7 (86%) recorded here for first-runs and
nothing like the 2/12 for later runs. Whatever degrades the stack, it is not the steering
model, and controlling stack age is enough to make the failure go away.

Both arms lost one run to the stack failing to come up at all, which is its own small
signal: roughly 1 in 10 `ego-av` starts did not reach "Startup complete" within 15
minutes.

### The one stall traced in detail

Measured before this A/B, on an aged stack, with pose, trajectory, velocity factors and
perceived objects sampled together (csb `scripts/stall_probe.py`). Recorded here because
the numbers are not elsewhere in this issue:

```
t+26s ego(291.3,-54.4) 0.0 m/s traj[n=161 start=(295,-55) end=(261,-55) vmax=4.2]
t+57s ego(291.3,-54.2) 0.0 m/s traj[n=159 start=(295,-55) end=(261,-55) vmax=4.2]
                               factors[traffic-signal@186.2m/st1]  objs<30m=4
CARLA: yaw=-140.9  throttle=0.27  brake=0.00  steer=-0.02  hand_brake=False  speed=0.00
Autoware pose (291.4,-54.1) yaw 139.0  vs  CARLA (291.4,-54.2) yaw(ROS) 140.7
```

The trajectory was never truncated and kept commanding 4.2 m/s; no velocity factor asked
for a stop; 27% throttle was applied with no brake and no hand brake; and localization
agreed with ground truth to 0.1 m and 1.7 deg, heading included. The ego therefore knew it
was 41 deg out of a lane running 180 deg and was simply wedged. This is consistent with
the lateral-departure signature, and it is the state the A/B above then failed to
reproduce even once in 18 runs.

Harness: `/tmp/steer_ab.sh` in this session; csb's `scripts/stall_probe.py` is the tool
for reading a stall once one occurs.

## The plant really is slower than the model (2026-08-19)

The feedback-lag hypothesis above says acb hands MPC back its own command with zero lag
while the CARLA wheel has real dynamics. That is only worth testing if the wheel is
actually slow, so it was measured: command (`/control/command/control_cmd`), report
(`/vehicle/status/steering_status`) and CARLA's own `get_wheel_steer_angle` sampled
together for 1675 samples over ~100 s of driving.

```
|reported - commanded|:     mean 0.034 rad, max 0.184 rad
best command->wheel shift:  8 samples = ~478 ms   (rms 0.0083 rad)
zero-shift rms:             0.043 rad
command range: -0.1721 .. +0.1537 rad
wheel   range: -0.1647 .. +0.1376 rad
```

**The wheel lags the command by roughly half a second.** Shifting the command forward by
~478 ms matches the measured wheel five times more tightly than comparing them at the same
instant (0.0083 vs 0.043 rad), and the wheel's range is slightly compressed against the
command's -- lag plus attenuation, which is what a first-order actuator does.

So the premise holds. With `report_measured_steering: false` the loop Autoware closes
contains no actuator at all, while the plant it is really driving takes ~0.5 s to arrive.
That is a textbook destabiliser, and unlike the steering-multiplier idea -- whose premise
was disproved, since acb's tire-angle mapping is self-consistent -- this one survives
first contact with a measurement.

Two limits on the number. The probe sampled at ~60 ms against a 100 ms simulation step, so
treat 478 ms as 0.4-0.6 s rather than a precise figure. And this establishes that the lag
exists, not that it causes the stall.

### Why the obvious A/B does not work, and what to run instead

An A/B of `report_measured_steering` true against false cannot use pass/fail as its
outcome. The steering A/B above established that the stall does not reproduce at all when
every run starts on a fresh stack -- 17 of 18 reached the stop line -- so both arms would
score ~100% and the result would be a null for want of failures, not for want of an
effect.

The outcome has to be continuous. **Lateral tracking quality** is the right one: the
standard deviation of the ego's lateral position over the straight approach, and its peak
excursion. It measures the oscillation this issue is actually about, needs no failures to
be informative, and will separate the arms at far smaller n than a pass count.
