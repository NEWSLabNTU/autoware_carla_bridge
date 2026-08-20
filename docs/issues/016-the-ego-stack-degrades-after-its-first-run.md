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

## The path shape in the ego frame, and why it does not settle the question

`scripts/trace_run.py` now walks the planned path outward from the ego and reports its
*shape* rather than its endpoints: `xtrack` is the lateral offset of the path point nearest
the ego, `lat20` its offset 20 m along, and `dyaw` its heading 5 m out relative to the ego.

A failing run, through the departure:

```
 t   ego_y   xtrack  lat20   dyaw   obj   cmd_a
28  -130.0   -0.18   +0.21   -2.4    2    +0.78
29  -129.6   -0.07   +2.53   +6.5    1    +0.80
30  -128.9   +0.61   +2.84   +7.2    2    +0.64
31  -129.1   +0.34   -3.03  -12.7    3    +0.11
32  -130.3   -0.83   -6.12  -20.5    1    +0.30
33  -131.0   -1.56   -4.61  -11.9    1    +1.00
34  -131.0   -1.58   +0.05   +5.8    3    +1.00
35  -129.7   -0.20   +9.33  +36.6   17    +0.72
36  -129.1   +0.26   +9.26  +34.3   17    -1.16
```

### What this does establish

**It is not obstacle avoidance.** Object count through the three path swings is 1, 2, 3, 1,
1, 3 -- the same as during healthy cruise (3, 3, 3, 2). The burst reaches 17 only at t=35,
after the path has already swung three times, and 30-43 later still. That is
frame-independent and confirms the object dump's conclusion at the moment that actually
matters.

### What it does NOT establish

An earlier version of this section read the swinging `lat20` as the planner handing the
controller a thrashing reference. **That inference does not hold**, and the measurement as
taken cannot support it.

`behavior_path_planner` anchors its path to the ego's current pose. So a *yawed ego* and a
*bent path* produce the same ego-frame signature: `xtrack` small near the ego by
construction, and a large lateral offset far ahead. With the ego's heading swinging tens of
degrees -- which the wheel-lag measurement above gives a mechanism for -- a perfectly good
lane-following path would look exactly like the table above.

The ego frame was the wrong frame to ask the question in.

### The discriminator, measured: the path is fine and the ego is yawing

On this straight route the lane centre is a constant map y of about -129.9, so the map-frame
position of the path 20 m ahead separates the two cases outright. Measured on a failing run:

```
 t   ego_y   xtrack  lat20   mapy20   egoyaw   dyaw
26  -130.1   -0.66   -0.61  -129.40   179.8    0.2
27  -130.0   -0.63   +0.04  -129.40   177.5    2.5
28  -129.2   +0.18   +4.01  -129.39   166.1   13.8
29  -128.1   +1.24   +5.03  -129.39   166.3   13.7
30  -127.9   +1.46   -0.40  -129.39  -173.1   -6.9
31  -129.1   +0.30   -6.77  -129.39  -153.8  -26.3
32  -130.2   -0.71   -8.45  -129.38  -149.9  -30.1
34  -132.7   -3.32   -5.05  -129.37  -173.8   -6.2
35  -130.6   -0.94   +9.57  -129.35   139.4   40.6
36  -126.0   +0.45  +16.62  -129.33    97.6   82.3
```

**`mapy20` moves 0.07 m across the entire departure** -- -129.40 to -129.33 -- while `lat20`
swings over 25 m and the ego's heading swings from 180 to 97 degrees. `dyaw` tracks the
ego's yaw error exactly (at t=28, `egoyaw` 166.1 is 13.9 degrees off and `dyaw` is +13.8).

The planner is emitting a path that sits on the lane centreline, continuously, throughout the
failure. Every bit of the ego-frame path swing was the ego's own heading.

The passing run's baseline agrees and shows what healthy looks like:

```
 t    ego_y   xtrack  lat20   mapy20   egoyaw
28   -130.1   -0.03   -0.59  -129.49   -180.0
41   -129.5   -0.00   +0.26  -129.38    178.5
47   -129.4   -0.01   -0.02  -129.32    179.8
53   -129.3   +0.00   +0.64  -129.79    179.4
```

Same path quality in both. `mapy20` is indistinguishable between the passing and failing run.
The entire difference is the ego's heading: within 2 degrees of 180 when it passes, swinging
80+ degrees when it fails.

**So planning is exonerated and this is the lateral control loop**, which is where the
measured ~478 ms command-to-wheel lag points. The two independent lines of evidence agree.

## What CARLA's steering actuator actually does (2026-08-20)

The ~478 ms command-to-wheel lag measured above is real as a correlation, but the number
alone does not say what produces it, and the candidates call for different fixes. Measured
directly against CARLA with no ROS in the path (`scripts/probe_steer_lag.py`,
`probe_steer_inputrate.py`, `probe_steer_tickrate.py`):

**It is a rate limit, not a first-order lag.** Step responses at four sizes:

```
 step   final   t63    t95   peak_rate
 0.10    6.78   50ms   50ms      0 deg/s
 0.20   13.18   50ms  100ms     95 deg/s
 0.40   25.16  100ms  200ms    157 deg/s
 0.80   47.56  200ms  350ms    157 deg/s
```

`t63` grows with step size and the peak rate saturates. A first-order lag would hold `t63`
constant regardless of step size.

**The limit is on the normalized steer input, at exactly 0.125 per tick at 20 Hz.**
Inverting the Ackermann map turns the slightly-decaying wheel increments into a dead
constant:

```
tick   wheel_deg   implied_cmd   d_cmd
   1        8.41        0.1250  0.1250
   4       30.88        0.5000  0.1250
   8       58.71        1.0000  0.1250
```

**It is per second, not per tick**, so the simulation step size is not a lever:

```
  dt(s)    Hz  ticks_to_full  ms_to_full  d_cmd/tick  d_cmd/s
  0.100    10              4         400      0.2500     2.50
  0.050    20              8         400      0.1250     2.50
  0.025    40             16         400      0.0625     2.50
```

So CARLA slews the steer input at a constant **2.5 units/s**: full lock in 400 ms, and any
commanded change in `|delta_cmd| / 2.5` seconds. Small corrections complete within a single
tick and are effectively instantaneous.

### This exonerates the actuator

2.5 units/s is **147-169 deg/s** of tire angle, depending where on the Ackermann curve it
sits. Autoware limits itself well below that:

```
mpc.param.yaml         steer_rate_lim_dps_list_by_curvature   40, 50, 60 deg/s
mpc.param.yaml         steer_rate_lim_dps_list_by_velocity    60, 50, 40 deg/s
vehicle_cmd_gate       steer_rate_lim_for_steer_cmd           1.0 rad/s = 57 deg/s
```

At 20 Hz, Autoware's own 60 deg/s ceiling is 3 deg per tick, or about 0.044 input units --
roughly a third of the 0.125 CARLA allows per tick. **CARLA's rate limit therefore never
binds on anything Autoware commands**, and the actuator tracks within one tick.

So the 478 ms cannot be actuator dynamics. Whatever produces that correlation shift lives in
the pipeline -- ROS transport, the bridge's own apply path, tick alignment between SSv2's
ticking and the control stream, or the measurement itself, which cross-correlated two
asynchronously sampled streams at ~60 ms against a 100 ms step. That is where to look next,
and it is a different search than "the plant is slow".

### It also explains the steering A/B null

[009](009-steering-report-echoes-command.md)'s A/B found no difference between reporting the
measured wheel angle and echoing the command. That is consistent with this: if the actuator
tracks within a tick, the measured angle and the command are nearly the same signal, so
which one is reported barely matters. The null was not a failure to detect an effect -- there
was very little effect to detect.

## The command path IS tick-synchronised, and the earlier finding was a probe artifact

An earlier version of this section reported that the command-to-wheel delay varied between
1 and 10 ticks across identical runs and concluded the control path was not synchronised
with the tick. **That was wrong, and the cause was the probe.**

`scripts/probe_bridge_latency.py` injects a square wave on `/control/command/control_cmd`,
drives the ticks itself, and records the wheel angle per tick. Its first version ticked in a
tight loop with no wall-clock pacing, so the simulation advanced far faster than real time
and acb_bridge's loop was starved of the tick period it gets in production. The varying
delay measured the starvation, not the bridge.

Production runs at `fixed_delta_seconds=0.1` with SSv2 ticking at real time -- RTF 0.998,
measured. Pacing the probe's ticks to wall clock to match:

```
                    paced (production, 10 Hz)   unpaced (sim outruns real time)
callback-apply      1, 1, 1 ticks               1, 3, 10 ticks
```

**One tick, deterministically, every run.** 100 ms, which is the floor: `apply_control`
takes effect at the next tick and cannot do better. There is no tick-synchronisation defect
and nothing here to fix.

### A fix was written, measured, and reverted

The obvious change -- have the subscription callback store the newest command and apply it
from the tick loop instead -- was implemented and tested. It is architecturally tidier, takes
an RPC out of the callback and halves the RPC count, since commands arrive at 20 Hz while
the simulation ticks at 10 Hz.

It also made things worse:

```
                    paced (production)          unpaced (sim outruns real time)
callback-apply      1, 1, 1 ticks               1, 3, 10 ticks
tick-loop apply     1, 1, 1 ticks               10, 10, 11 ticks
```

No difference at production pacing, and consistently pinned at the worst end when the
simulation outruns real time, because binding application to the bridge's own loop lets it
fall behind while the callback keeps up. A change with no measured benefit and a measured
regression is not worth carrying, so it was reverted. The patch is kept out of tree.

### The lesson, which cost the most here

A probe that drives the clock has to drive it at the rate the system really runs at.
Ticking as fast as the RPCs allow measures a regime the software never sees, and it does so
convincingly -- the numbers were stable, repeatable, and completely misleading. Any future
tick-driving probe in this repo should pace to wall clock and say so.

So the ~478 ms figure remains unexplained by anything measured so far, and the actuator,
the queue and the tick synchronisation are all now excluded as candidates.

## Re-measuring the 478 ms figure: it is about 200 ms (2026-08-20)

A synthetic square wave through the same path measures a deterministic one tick, so the
478 ms needed reproducing on a real Autoware-driven run before anything was built on it.
`scripts/probe_478.py` samples the command and the physical wheel angle at 50 Hz during a
scenario and applies three estimators to the same samples:

```
A. raw signals (the original method)   best 280 ms   56% better than zero shift
B. derivatives (high-passed)           best 200 ms   uniquely minimised
C. edge timing at sharp command steps  median 200 ms   p10 0, p90 380
```

**The lag is real, and it is about 200 ms -- two ticks at the production 10 Hz.**

**The original method reads high.** On identical samples, correlating the raw signals gives
280 ms while both estimators that are robust to smoothness give 200 ms. Autoware's steering
command is rate-limited and slowly varying, and cross-correlating two smooth signals is
ill-conditioned: many shifts fit nearly as well, so the argmin drifts. With coarser sampling
and a longer window, 478 ms is plausibly that same bias stretched further. Treat 478 as an
upper bound produced by the estimator rather than as a measurement.

### Where two ticks comes from

Autoware publishes control at 20 Hz. SSv2 ticks CARLA at 10 Hz. **Nothing synchronises the
two**, so a command waits 0-100 ms for the next tick and then takes effect on it. Part of
the remainder is observation: a separate client reading `get_wheel_steer_angle` sees the
last *completed* tick, which adds up to a tick of apparent delay that the vehicle does not
actually experience.

So the honest decomposition of ~200 ms is roughly one tick of real waiting plus up to one
tick of measurement, against a floor of one tick that cannot be removed at all.

Note this is **not** the tick-synchronisation defect withdrawn above. Applying on the tick
boundary would not shorten this: the wait exists because the producer and the tick are on
different clocks, not because the bridge applies at the wrong moment. Shortening it would
mean either ticking at the command rate, or having whoever owns the tick pull the newest
command as part of the tick.

### Caveats

The run was 31.6 s rather than the intended 100 s -- the ego despawned when the scenario
ended -- and the ego was driving nearly straight, so the command spanned only -0.055 to
+0.023 rad. Small, smooth signals are the hardest case for exactly the estimator problem
described above, which is why C, a model-free edge measurement, matters more than A here.

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

### The A/B of that switch: underpowered, direction matches

Twenty runs alternating `report_measured_steering`, fresh ego stack before each, outcome
the lateral standard deviation and peak excursion over the straight approach (x 300 down
to 140), measured about each run's own mean so a trim offset does not count as
instability.

| arm | | n | sd median | sd mean | peak median | peak max |
|---|---|---|---|---|---|---|
| A | `false`, echo the command | 8 | 0.0284 | 0.0712 | 0.077 | **1.045** |
| B | `true`, report the wheel | 8 | 0.0235 | 0.0277 | 0.064 | **0.183** |

**The medians are nearly identical**, so nominal tracking is the same either way. The
entire difference sits in the tail:

- Both large excursions were in the echo arm -- run 3 peaked 1.04 m off the line, run 5
  peaked 0.49 m. Arm B's worst run was 0.18 m.
- Run 3 also produced **the only stall in the whole set**, stopping at x=212 and never
  reaching the stop line.

That is the shape the hypothesis predicts: feeding MPC its own command does not degrade
ordinary tracking, it permits divergence. **It is also two runs.** At 2 of 8 against 0 of
8, Fisher's exact gives p ~ 0.47. This is not a result, it is a reason to run more.

Which is now cheap: csb exposes the switch as a launch argument and `just ego-av` forwards
`REPORT_MEASURED_STEERING` from the environment (csb f1adc16), so an arm costs a variable
assignment.

Two caveats on the data. Four of the twenty runs produced no measurement at all -- one
stack that never came up, three with no samples in the straight segment -- so the same
roughly-1-in-10 start flakiness noted above is still present and cost more here. And the
harness's own CSV extraction was buggy (its `n=` match also caught the `n` inside
`x_span=`), splitting every record across two lines; the numbers above come from
reparsing, but the raw file is malformed and the harness needs that fixed before reuse.

### Twenty more runs: the effect was noise

The tail asymmetry above did not survive replication. A second block of 20, identical
design, reversed it:

| | arm A (echo) | arm B (measured) |
|---|---|---|
| round 1 | peak max **1.045**, 2 excursions, 1 stall | peak max 0.183, 0 excursions |
| round 2 | peak max 0.099, 0 excursions | peak max **1.145**, 1 excursion, 1 stall |
| **pooled** | n=16, sd median 0.0257, **2** excursions | n=17, sd median 0.0264, **1** excursion |

Pooled medians are indistinguishable, 0.0257 against 0.0264, and the large excursions run
2 of 16 against 1 of 17 -- p ~ 0.6. **`report_measured_steering` has no detectable effect
on lateral tracking across 33 valid runs.**

The premise still holds: the wheel really does lag the command by ~0.5 s, and echoing it
really does leave no actuator in the loop MPC closes. The predicted consequence simply
does not appear at this scenario's speeds and steering amplitudes (commands stayed inside
+-0.17 rad). Feeding MPC the measured angle is arguably still the more honest thing to
report, but it is not a fix for this issue and should not be sold as one.

Two corrections to earlier claims in this document, both from the same 33 runs:

- **The stall does occur on fresh stacks**, twice here, at x=212 and x=254, once in each
  arm. The earlier "17 of 18 reached the stop line" reading was right about the rate and
  wrong to imply the failure is gated on stack age. It is a strong tendency, not a gate.
- **The start flakiness is measurable**: 3 of 40 runs produced no measurement at all --
  one stack that never reached "Startup complete", two with no samples in the straight
  segment. Roughly 1 in 13.

Which leaves stack age as the only thing yet shown to reproduce the failure, and nothing
identified inside it.

## Nothing external yaws the ego (2026-08-20)

The open question left by the stall trace was what rotates the ego 41 deg out of a lane it
is correctly localized in. Three candidates are worth eliminating before looking harder at
the controller: a second writer on the vehicle, a teleport (invariant 5 says only PhysX
moves the ego), or a kinematic gain error making the car turn more than its wheels ask.

Measured directly against a bicycle model, `yaw_rate = v * tan(delta) / L`, using CARLA's
own angular velocity and the *measured* front wheel angle, over 826 samples where the ego
was both moving and steering:

```
observed |yaw_rate|:      mean 0.0298  max 0.1193 rad/s
bicycle prediction:       mean 0.0323  max 0.1293 rad/s
|observed - predicted|:   mean 0.0027 rad/s
ratio observed/predicted: median 0.922   (L = 2.79 m, Autoware's wheel_base)
position jumps > 25 m/s:  0
```

**The rotation is fully accounted for by the front wheels.** The ratio sits within 8% of
unity and on the low side, which is what tyre slip does; there is no excess yaw to
attribute to anything else. No position jumps, so nothing is teleporting the ego, and no
sign of a second actor writing to the vehicle.

So the yaw is not externally applied: whatever the ego does, it does by steering. Combined
with the earlier finding that acb's tire-angle mapping is self-consistent, and with
planning already exonerated, the departure has to be the lateral control loop commanding
those angles -- genuine closed-loop behaviour rather than a bridge or physics artefact.

**Limit worth stating**: this measured a normal run, not a divergence. It rules out an
external torque during ordinary driving; it does not rule out kerb contact adding yaw once
a departure is already under way, which is a different claim and would need the same
measurement captured during a stall.

Two operational notes from getting this measurement. The first attempt produced 2766
samples and no usable ones because the ego never engaged and sat at its spawn point the
whole run -- the probe now waits for motion before sampling and reports moving and
steering counts separately, so a null is interpretable rather than silent. That is the
same never-engaged failure seen as `min_x=320` in the earlier A/B, and it cost a full run.

## Two stalls caught on tape, and they are the same stall (2026-08-20)

The yaw measurement above answered the healthy case and left one question open: does the
rotation stay wheel-explained while the ego is actually diverging? Catching a stall with
the trace attached needed a harness that keeps one aged stack and cycles the scenario on
it, rather than restarting between runs and pinning stack age at "fresh".

Two stalls were caught, one before and one after the CARLA sync-mode fix in csb. They are
near enough to identical that they are clearly one phenomenon:

```
              distance   final x   head_err   lane_off   max speed   yaw ratio
  catch 1      22.1 m     297.90    +35.8       0.35      2.67 m/s     0.927
  catch 2      22.1 m     297.93    +34.0       0.38      2.40 m/s     0.934
  healthy run     --      104.3        --         --         --        0.921
```

**The rotation is wheel-explained during the divergence too.** 0.927 and 0.934 against
0.921 on a run that drove the whole route: the same ratio, inside the failure and outside
it. No position jumps in either. Together with the earlier healthy-run measurement this
closes the external-torque question in both directions -- kerb contact, a second writer,
a teleport. Whatever the ego does, it does by steering.

**It is not the road.** Both runs stop within 3 cm of each other, which looks like geometry
until the map is checked: from x=320 to x=280 the ego is on road 10, lane 1, no junction,
lane width a constant 4.00 m, lane yaw a constant 180.0 deg. There is nothing there. The
repeatability comes from the start condition being identical, so the same transient plays
out the same way.

**The shape is a growing heading oscillation, not a drift.** Heading error swings roughly
0 -> +9 -> -17 -> +18 -> wedged in catch 1, and 0 -> +8 -> -17 -> +25 -> wedged in catch 2,
while the wheel swings between -0.33 and +0.56 rad. Lateral offset never exceeds 1.4 m in a
4 m lane: the ego does not leave its lane, it rotates inside it. The steering command peaks
at 0.53 of an available 1.0, so this is not command saturation.

**What the wedge looks like is worth a separate look.** For the last 120 s of both runs the
ego is pinned -- x drifts 0.00 m -- while throttle sits latched at exactly 0.067 with no
brake, and the heading still creeps about 5 deg. A control output that takes one or two
distinct values across two minutes is not a live controller responding to a stopped car;
MPC would wind up against a car that will not move. That points at the control command
having gone stale rather than at lateral control still fighting, which is a different
failure from the one this issue has been chasing and needs its own measurement: whether
/control/command/control_cmd is still publishing during the wedge, and whether hand_brake,
reverse or gear are set.

Note the earlier instrumented stall in this issue recorded planning commanding 4.2 m/s and
control applying 27% throttle, so control was live in that instance. Either these are two
distinct failures or the wedge is a late phase of one; the trace above cannot tell which.

### The harness that caught it

Kept for reuse, because the run conditions matter more than the probe:

* one aged stack, scenario cycled on it -- restarting between runs pins age at the
  condition where the stall is rarest
* a stall is stopped **out of lane**, not merely stopped: this scenario commands a red
  light, and a car waiting at a red is stopped and correct. Heading error against the
  nearest driving lane is what separates them
* an ego that spawns and never engages is a dud, not a data point; it restarts the stack
  rather than counting toward anything
* the trace is written per sample as it goes, so a killed probe still leaves the data

## The command stream is live while the ego sits pinned (2026-08-20)

The previous section flagged a control output that took one or two distinct values across
two minutes and suggested the command stream might have stopped, with acb holding the last
value. That is wrong, and the measurement that settles it is direct: record
`/control/command/control_cmd` on the same wall clock as the CARLA trace, since a stale
command and a live command for the same value are indistinguishable from the plant.

A third stall was caught with both probes attached. Over the whole 240 s run:

```
control_cmd messages: 2532 in 240s (10.6 Hz average)
final gear: DRIVE   control mode: AUTONOMOUS
gaps > 1 s: none
silent 10 s windows: 0/23
```

And in the wedged window specifically, with the ego pinned at x = 294.65 and never
exceeding 0.035 m/s:

```
t+140..240s, 1086 msgs (10.9 Hz)
  commanded velocity  mean +0.214  range +0.000..+0.288 m/s
  commanded accel     mean -0.053  range -2.011..+0.207 m/s2
  commanded steer     mean +0.0284 range +0.0142..+0.0439 rad, 140 distinct values
```

**Autoware is actively commanding throughout.** It asks the ego to creep forward at about
0.2 m/s, never commands reverse, never stops publishing, and emits 140 distinct steering
values in the window -- that is a controller running, not a latched output. The near
constant applied throttle seen in the first two catches is the controller's own output at
a standstill, not a frozen stream.

So the failure is not a lost command. It is that a commanded 0.2 m/s produces 0.035 m/s:
roughly 6.7% throttle, with the ego sitting at about 54 deg to its lane, does not move the
car. The planner has already backed its target down to a crawl by this point, so nothing
in the loop asks hard enough to break the car out of it.

Worth separating from the earlier instrumented stall in this issue, where planning
commanded 4.2 m/s and control applied 27% throttle. Those are different regimes, so either
there are two failures or the crawl is the late phase of one.

### Two corrections to earlier numbers here

**`max_head_err` of 180 deg is an artefact, not a spin.** Heading error is measured against
the nearest driving lane, and once the ego rotates past about 90 deg the nearest lane can
be the opposing one, so the error jumps to near 180 with the ego having turned only a
degree. In this trace it goes -31.5 to +149.7 while the yaw moves 148.5 to 149.7. The same
applies to the 130.5 and 131.6 figures reported for healthy runs, which are junction turns
matching an adjacent lane. The probe now folds the error onto the lane axis. Nothing else
in this issue rests on those numbers -- the stall detector uses the same value but at a
20 deg threshold, well below where the flip happens.

**The excursion does leave the lane, unlike the first two catches.** Lateral offset reaches
1.99 m in a 4.00 m lane, so the ego is at the lane edge, and y swings 54.5 to 57.8 and back
while the commanded steering swings between -0.70 and +0.48 rad. The first two catches
stayed within 1.4 m and rotated in place.

**Still open**: the hand brake is engaged in 4% of samples in this run, first at t+0 before
the ego is even engaged and last at t+156.7, so it appears intermittently inside the wedged
window. It is not a clean explanation for the pin and it is not ruled out either.

## The hand brake is not what pins the ego (2026-08-20)

Left open above: the hand brake appears inside the wedged window, so it might be what holds
the car. It is not, and the trace already caught the moment that shows it.

The hand brake fires in four short episodes, and in every one the speed is exactly 0.000
and the throttle exactly 0.000:

```
  t+  0.0..  1.7s    t+ 78.5.. 82.4s    t+145.0..147.2s    t+150.8..156.7s
```

The last one ends at t+156.7. The run continues to t+240, and across those remaining 83 s:

```
  hand_brake ever on: False
  x  294.65 -> 294.65   (drift 0.001 m)
  throttle mean 0.067   brake max 0.000   reverse: never
  speed max 0.035 m/s   head_err +53.2 -> +54.5
```

**The ego stays pinned for 83 s with the hand brake off and throttle applied.** Whatever
holds it is still holding it when the hand brake is not there, so the hand brake cannot be
the cause. It is a consequence of stopping, not a reason for it, which matches what the
code does: `vehicle_control.rs` engages it when the gear is PARK or when commanded velocity
and acceleration are both within 0.01 of zero -- a commanded standstill hold, added because
CARLA's automatic transmission idle-creeps at zero throttle.

That comment is also what makes the remaining behaviour strange, and it is the thread worth
pulling next. acb engages the hand brake precisely because a stopped CARLA vehicle *creeps
to about 1 m/s on its own at zero throttle and zero brake*. Here the vehicle has 6.7%
throttle, no brake, no hand brake, and moves 1 mm in 83 s. It is not that the command is too
weak to overcome standstill friction: by acb's own measurement this vehicle should be
creeping forward with no command at all. Something is physically holding it.

The likely candidate given the pose: the ego sits at 54 deg across a lane whose right
neighbour is Shoulder, so at that angle its nose is plausibly into the kerb. Testing that
means recording the collision sensor and the wheel contact state, not more control-side
measurement.
