# 016 — The ego stack only passes its first scenario after a restart

**Severity**: High
**Component**: lateral control path (`vehicle_control.rs` steer scale, MPC feedback); run-order
dependence still unexplained
**Status**: Open. Clearing orphaned interpreters helps but does not cure it -- the split still reproduces

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

## Sub-stepping cuts the command lag from ~200 ms to ~60 ms (2026-08-20)

The ~200 ms above is mostly the frame quantum: Autoware publishes control at 20 Hz on its
own clock, SSv2 asks for a 0.1 s frame, and nothing synchronises them, so a command waits up
to a full step before any tick can act on it.

csb can divide that without touching the scenario's timeline. `substeps` (csb
`bridge_config.yaml`) splits the SSv2 frame into N CARLA ticks at `step_time / N`, so a frame
still advances by `step_time` while control is picked up N times as often. Measured on a
live run at `substeps: 2`, against the same three estimators as before:

```
                                      substeps 1      substeps 2
A. raw signals (original method)         280 ms          60 ms
B. derivatives (high-passed)             200 ms          60 ms
C. edge timing at sharp steps       median 200 ms   median 20 ms (p90 100)
```

All three agree and the improvement is larger than the halving predicted, because both
components halve: the phase wait and the observation artifact, since a client reading wheel
angle now sees a 50 ms tick rather than a 100 ms one.

The scenario passed on that run. That is **one run**, and this issue's whole history says a
single passing run means very little -- it is reported as "did not break it", not as a fix.

### What it costs, and what is unverified

Every tick fires sensors configured to publish per frame rather than on a timer. The ego
LiDAR is one (`sensor_tick: 0.0`), so its point clouds double to 20 Hz. That suits the sensor
-- `rotation_frequency` is 20 Hz, so at one substep each scan sweeps two rotations and at two
it sweeps exactly one, which is what its own config comment asks for -- but it doubles the
point cloud rate into Autoware, and physics runs at the finer delta.

Rates measured during real drives, `substeps` 1 against 2:

```
/sensing/imu/imu_data                     10.71 -> 20.61 Hz    doubled
/localization/kinematic_state             10.25 ->  9.95 Hz    unchanged
/control/command/control_cmd               9.91 -> 10.53 Hz    unchanged
/perception/object_recognition/objects     5.39 ->  5.35 Hz    unchanged
```

The IMU confirms the mechanism directly: `sensor_tick: 0.01` is shorter than either tick
period, so it fires once per tick and doubles with the tick rate. The LiDAR is
`sensor_tick: 0.0`, so the same applies to it.

What matters more is what did not move. Autoware's own pipeline -- localization, control,
perception -- runs at the same rates either way, so sub-stepping doubles the raw sensor
input without destabilising anything downstream.

**The LiDAR rate, measured (2026-08-21).** It was left inferred above; the inference was
wrong in detail, and the truth is more interesting:

```
                                            substeps=1   substeps=2
/sensing/imu/imu_data                          10.00        20.61 Hz
/sensing/lidar/top/pointcloud_before_sync       4.99        13.71 Hz
/sensing/lidar/concatenated/pointcloud          5.38         9.92 Hz
```

Point clouds do not double to 20 Hz. At one substep the LiDAR runs at **half the frame
rate** -- 4.99 Hz against a 10 Hz frame -- because `rotation_frequency` is 20 Hz and each
frame therefore sweeps two rotations, some 262k points. That is the mismatch the sensor's
own config comment warns about: it asks for `rotation_frequency` to equal simulation FPS,
and at 10 Hz it does not.

At two substeps each scan is exactly one rotation, about 131k points, and the rate rises
2.75x to 13.7 Hz -- more than doubling, because the cheaper frames are ones the sensor can
actually deliver. So sub-stepping does not merely cost more here; it repairs a standing
misconfiguration and roughly doubles the point cloud rate Autoware sees, from 5.4 to 9.9 Hz
on the concatenated topic.

The IMU doubles exactly, 10.0 to 20.6 Hz, which is the plain per-tick behaviour and confirms
the mechanism.

Also note `/control/command/control_cmd` publishes at ~10 Hz, not the 20 Hz assumed earlier
in this issue: Autoware's control loop runs on simulation time, so it tracks the frame rate.
The command and the tick are therefore rate-matched and merely out of phase.

`substeps` therefore defaults to **1**, exactly the previous behaviour.

### It does not address the departure

Lower command latency is worth having on its own, but the ego leaves the drivable surface
before it is pinned, and the vegetation contact above is a consequence of that. Nothing here
explains the lateral behaviour that steers it off the road.

## The previous run's orphaned interpreter is what breaks the next one (2026-08-20)

Every harness that produced this issue's data killed the scenario's `play_launch` between
runs -- `pkill -f "csb_launch carla_scenario.launch.xml"` or equivalent -- which does **not**
reap its child `openscenario_interpreter_node`. The child survived as an orphan, holding the
`/simulation` namespace and its simulator connection, and the next run started alongside it.

So every run 2 in this issue's history ran with run 1's interpreter still alive. Run 1 never
had a predecessor. That is the split.

`just scenario` now clears them first (csb `9d2fa71`). Re-running the same experiment --
one fresh ego stack, two runs on it, nothing else changed:

```
pair  run  verdict  stale processes cleared
1     1    PASS     0
1     2    PASS     2
2     1    PASS     0
2     2    PASS     2
3     1    PASS     0
3     2    PASS     2
```

Plus one earlier pair on the same fix, also PASS/PASS. **Four run 2s, four passes**, against
2 of 13 historically and a split that reproduced 7 for 7 in dedicated pairs. Under the
historical failure rate that is a 0.05% coincidence. The `cleared` column is the tell: a
fresh stack has nothing to clear, and every second run had exactly two orphans waiting.

### Why this fits everything else in this issue

- **Perfect run-order dependence.** Run 1 has no predecessor; every later run does.
- **Only a stack restart cured it.** A restart kills the orphans as collateral, which is why
  restarting looked like the remedy and why "stack age" looked like the variable.
- **Nothing downstream was ever wrong.** CARLA, the actuator, localization, path shape, the
  validators and the steering report were each measured and each came back clean, because
  the contamination sits at the SSv2/ROS-graph level above all of them. Every section above
  that ruled something out was ruling out the wrong layer.
- **Two unstable failure signatures.** A second interpreter contending over entity state and
  the clock has no reason to fail the same way twice, which is why a commanded standstill and
  a drive off the road both appeared.

### What is not established

The *mechanism* by which a live orphan breaks the next run. Correlation between clearing it
and passing is strong, but nothing here shows what the orphan actually does -- whether it
answers SSv2 requests, republishes entity state, fights over `/clock`, or something else.
That is worth pinning down, because the fix currently works by removing the process rather
than by making concurrent interpreters safe.

Four pairs is also a modest n for a stochastic failure. It is enough to act on and not enough
to close the issue.

## One stack, eight consecutive runs, eight passes (2026-08-21)

If the orphaned interpreter was the whole story, a stack should now run indefinitely --
which is what the project actually needs, and a sharper test than pairs. One fresh ego
stack, eight scenario runs on it, nothing touched in between:

```
run  verdict  cleared   xt_sd   xt_max
1    PASS     0         0.017   0.050
2    PASS     2         0.130   0.370
3    PASS     2         0.030   0.090
4    PASS     2         0.016   0.040
5    PASS     2         0.109   0.430
6    PASS     2         0.121   0.500
7    PASS     2         0.138   0.560
8    PASS     2         0.017   0.040
```

**Eight for eight.** Under the old behaviour this stack fails at run 2 and stays failed; the
historical rate for run 2 and later was 2 passes in 13. The `cleared` column is the
mechanism in miniature -- nothing to clear on a fresh stack, exactly two orphans waiting
before every subsequent run.

Lateral tracking quality was recorded per run rather than pass/fail alone, because a stack
degrading gradually would show there long before a verdict flipped. It does not degrade:
median `xt_sd` 0.070, range 0.016 to 0.138, against 0.98 to 2.05 on the historical failing
runs. An order of magnitude clear of the failure band.

Two honest caveats. Runs 5 through 7 rise monotonically -- 0.109, 0.121, 0.138 -- and runs
1-4 median 0.024 against runs 5-8 median 0.115, so the later half is looser. Run 8 then
returns 0.017, the tightest value in the set, which is not what a degrading stack does.
Call it run-to-run noise with a wide spread rather than accumulation, and re-check if a
longer sequence ever shows the same halves.

### Where that leaves this issue

The cause is identified and fixed at the harness level, and the fix holds across eight
consecutive runs and four pairs. What remains open is the **mechanism**: nothing here shows
what a live orphan actually does to the next run -- answers SSv2 requests, republishes
entity state, contends over `/clock`. Worth pinning down, because the fix works by removing
the process rather than by making a second interpreter harmless, so anything launching SSv2
outside `just scenario` is still exposed.

### What this invalidates

Several conclusions in the sections above were measured on second runs, which are now known
to have been contaminated by a live orphan. They are not necessarily wrong, but they were
not measuring what they claimed to. The steering A/B in
[009](009-steering-report-echoes-command.md) is the clearest case: every one of its
second-run trials had an orphan present, so its null result should be re-established on
clean runs before it is relied on.

## A ROUTER-based session guard was tried and reverted (2026-08-21)

The orphan fix lives in `just scenario`, so anything else launching SSv2 is still exposed.
The obvious hardening is to make csb refuse a second client itself. csb's ZMQ socket is
`REP`, which hides which peer sent a request; `ROUTER` speaks the same wire protocol to a
REQ client but prepends an identity frame, so the bridge can tell the live scenario from a
previous run's orphan and reject the latter.

It was implemented, unit-tested and measured. **It regresses the simulation and was
reverted.**

```
                     runs   passes   xt_sd range
REP (unchanged)        8      8/8     0.016 - 0.138
ROUTER session guard   4      2/4     0.032 - 2.210
```

Same harness, same scenario, fresh stack each. The failures are not the guard firing -- it
rejected nothing in any run, and no malformed frame or send error was logged. The tell is
that even the *passing* ROUTER runs are degraded: run 4 passed with `xt_sd` 0.647, five
times worse than the worst of the eight REP runs. Lateral tracking gets worse across the
board, which is what a disrupted frame cadence looks like rather than a protocol fault.

A likely mechanism, unconfirmed: `ROUTER` does not enforce the strict request-reply lockstep
`REP` does, and it drops unroutable messages silently unless `ROUTER_MANDATORY` is set. A
reply that vanishes leaves SSv2 waiting and stalls the frame it was driving. Anyone
retrying this should set `ROUTER_MANDATORY`, check every `send_multipart` return, and
measure tracking quality rather than pass/fail -- pass/fail alone would have called run 4 a
success.

The rejected implementation is kept out of tree. The session-guard idea is not disproved;
this way of building it is.

### It also rescues the eight-run result

The failing runs in this experiment all carried the ROUTER change, so they are not evidence
against the orphaned-interpreter finding above. The eight consecutive passes were measured
on unmodified `REP` and stand.

## The split still reproduces with orphan cleanup active (2026-08-21)

The eight-run sequence above was read as the orphaned interpreter being the whole story.
**Sixteen further runs say it is not.** Four fresh stacks, four scenario runs each, orphan
cleanup confirmed running before every run (`cleared 2` in each log):

```
by run index, both steering arms pooled and balanced across stacks

run 1:  pass 4/4   median xt_sd 0.068
run 2:  pass 0/4   median xt_sd 1.212
run 3:  pass 1/4
run 4:  pass 1/4
```

Run 1 passes on every stack. Run 2 fails on every stack. That is the original split, intact,
with the fix in place -- and it happens on both steering arms, so it is not the parameter
under test either.

### What that changes

Clearing orphans is still a real improvement. Across everything measured since the fix the
aggregate is roughly 22 of 36 later runs passing, against 2 of 13 historically. But it is a
contributing cause, not the cause, and "cause identified and fixed" was wrong.

The eight consecutive passes remain unexplained as an outlier. The obvious differences were
checked and eliminated: same tick rate (`fixed_delta_seconds=0.1`, `substeps: 1`), same
config, same inter-run handling in the harness, and the only acb commit in between changed
comments alone. Something separated that sequence from these sixteen runs and it is not yet
known what.

### On how this was over-claimed

The eight-run result was written up while its own caveat -- "four pairs is a modest n for a
stochastic failure" -- was still on the page a few sections above. A clean streak is exactly
what a stochastic failure produces some of the time, and a run of eight is not strong
evidence when the base rate being tested against is 15%. The correct response to 8/8 was to
keep going, not to change the status line.

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

## What holds the ego is vegetation, and the pin is a consequence (2026-08-20)

The previous section reasoned that something must be physically holding the car, since acb
engages the hand brake at standstill precisely because a stopped CARLA vehicle idle-creeps
at zero throttle, and this one had 6.7% throttle and moved 1 mm in 83 s. The kerb was the
guess. A collision sensor and a bounding-box footprint check, run on every run of a hunt,
answer it directly.

Across eleven runs, collisions separate cleanly from outcomes: nine runs with 0 events,
one partial run with 1, and the stalling run with **1323 events, every one against
`static.vegetation`**. First contact at t+18.0 s, where the speed drops from 0.83 to 0.02
m/s within 0.4 s, and contact continues to t+229.4 -- 1312 of 2385 samples are touching it.

The footprint says the same thing from the other side. While pinned, all 896 samples read:

```
  front-left  corner: Sidewalk        rear-left  corner: Driving
  front-right corner: Sidewalk        rear-right corner: Driving
```

**The ego is diagonally off the road with its nose on the pavement, jammed against
vegetation.** Over the last 80 s it travels 0.10 m while pushing with a mean 0.033 throttle
and a mean acceleration magnitude of 0.029 m/s2.

So the guess was right in substance and wrong in detail: the obstruction is real, but it is
vegetation on the sidewalk rather than the kerb face. More importantly this settles what the
pin *is*. It is not a control failure at all -- it is the ordinary consequence of having
already left the road. The failure to explain remains the lateral behaviour that steers the
ego off the drivable surface in the first place, which happens well before contact.

### Correction: the stopping position is not fixed

An earlier section made much of two catches stopping within 3 cm of each other and argued
the repeatability came from an identical start replaying the same transient. With four
catches the stopping points are 297.90, 297.93, 294.6 and 289.2. The first two agreeing was
a coincidence of those two runs; there is no fixed stopping place, and the reasoning built
on it does not stand. What is consistent is the shape -- a growing heading oscillation that
ends with the car off the road -- not where it ends.

## The first run drives, the second one fails (2026-08-20)

The hunt above was not designed to test this issue's central claim, but it produced the
cleanest evidence for it so far, because the harness restarts the ego stack after every
run that fails to engage and so alternates fresh and aged stacks by itself.

```
  run  1  first on its stack   DROVE               run  2  second  NEVER_MOVED
  run  3  first                void, CARLA crashed run  4  first usable  DROVE
  run  5  second               NEVER_MOVED         run  6  first   DROVE
  run  7  second               NEVER_MOVED         run  8  first   DROVE
  run  9  second               NEVER_MOVED         run 10  first   drove 96 m
  run 11  second               STALL_OUT_OF_LANE
```

**Every first run on a stack drove; every second run failed. Five out of five each way.**
Run 3 is void -- CARLA segfaulted during it -- so run 4 is counted as the first usable run
on that stack.

This is a far better reproducer than the roughly 3-in-33 rate this issue has been working
with. It also means the two failure modes are the same phenomenon wearing different faces:
four second-runs never engaged at all and one drove off the road, and nothing distinguishes
them but which way the degraded stack fails that time.

Worth noting the never-engage failures are not a silent Autoware: the control probe reports
`STREAM_LIVE` at about 10 Hz through a run where the ego never leaves its spawn point.

### CARLA segfaults on long uptime

Separately, CARLA died with `Signal=11` and dumped core after 19 hours, and systemd restarted
it -- restart counter at 3. Not memory pressure. It cost one run of this hunt and it is worth
knowing before attributing any long-session flakiness to the bridge or to Autoware.

## Diffing a fresh stack against the same stack's second run (2026-08-20)

With the first-run/second-run split reproducing deterministically, the experiment is cheap:
start one ego stack, run the scenario twice without touching the stack in between, and
record the same signals on both. Two pairs were run. The split held both times, which puts
it at seven for seven counting the hunt.

Both pairs agree on what is *not* different. Operation mode reaches AUTONOMOUS, routing
reaches SET, and localization reaches INITIALIZED on the second run exactly as on the
first. Trajectories keep arriving at roughly the same rate. So the second run is not a
stack that failed to start, failed to accept a route, or stopped planning.

**Pair A** -- second run never moved:

```
                        run 1 (fresh)        run 2 (same stack)
  commanded velocity    max +4.457           max +0.000, mean +0.000
  trajectory points     min 142 max 172      min 11  max 170
  trajectory max speed  4.17 m/s             0.88 m/s
  ego speed max         5.31 m/s             0.00 m/s
  new errors            --                   planning_validator:
                                               trajectory_validation_distance_deviation x2042
                                               ..._longitudinal_distance_deviation x2042
                                             control_validator: ..._yaw_deviation x1980
```

**Pair B** -- second run drove 29 m and wedged:

```
                        run 1 (fresh)        run 2 (same stack)
  commanded velocity    max +4.280           max +4.170, mean +0.142
  trajectory max speed  4.17 m/s             4.17 m/s
  control_cmd rate      9.8 Hz               17.7 Hz
  new errors            --                   localization: ekf_localizer x548
                                             localization: pose_instability_detector x6
```

### A hypothesis this refutes

The obvious reading of pair A is that the stack keeps the previous run's route while SSv2
respawns the ego at the start line, so the trajectory sits 200 m away and the validator
rejects it for distance deviation. Pair B says no. Its trajectory spans (295.0, -55.5) to
(260.6, -55.5) with the ego at (290.6, -54.7), and the largest gap between the ego and the
nearest trajectory point across the whole run is 2.5 m. The trajectory tracks the ego. It
is not stale and it is not somewhere else.

### What actually separates the two runs

The failure mode is not stable between pairs -- once a commanded standstill, once ordinary
looking commands and a drive off the road -- so a single mechanism is not yet established.
Two things do stand out and neither appears on a first run:

* `control_validator: control_validation_max_distance_deviation` fires on both second runs
  in volume, 1980 and 2938 times, against 1187 and 1324 on the corresponding first runs.
* Pair B's second run is the first sighting of localization complaining directly:
  `ekf_localizer` 548 times and `pose_instability_detector` six times, both absent from
  every first run recorded. Localization sits upstream of both failure modes, which makes
  it the more interesting of the two.

The doubled control_cmd rate in pair B is worth a note and not a conclusion: a publisher
count taken on the idle stack afterwards reads 1, so it is not a second controller left
over from the previous run. It might be `vehicle_cmd_gate` behaving differently under an
MRM condition. n = 1.

### A standing anomaly, not a differentiator

`duplicated_node_checker` reports errors on every run measured, first and second alike --
867 to 930 times per run. Whatever it is finding, it is there before the stack degrades, so
it does not explain the split. It may still be worth fixing on its own.

## Localization is not the cause; it is more accurate on the failing run (2026-08-20)

The diff left localization as the lead, on the strength of `ekf_localizer` and
`pose_instability_detector` errors appearing on a second run and never on a first. An error
count says a node is unhappy, not that the estimate is wrong, so the test is to put
Autoware's estimate and CARLA's ground truth in one process and subtract them.

Measured across a pair on one stack, the second run being the one that failed:

```
                              run 1 (drove 216 m)     run 2 (never moved)
  position error vs CARLA     mean 0.511  max 3.363   mean 0.080  max 0.099 m
  yaw error vs CARLA          mean 0.55   max 5.25    mean 0.01   max 0.26 deg
  kinematic_state rate        9.9 Hz                  9.8 Hz
  localization diagnostics    all OK                  all OK but three single hits
```

**Localization on the failing run is an order of magnitude more accurate than on the run
that worked**: 8 cm against 51 cm, and 0.01 deg against 0.55 deg. The healthy run is the
one that drifts, reaching 3.4 m of position error while moving. So localization errors do
not predict the failure, and the estimate cannot be what stops the ego -- it knows where the
car is to within 8 cm while the car refuses to move.

The obvious caveat is that run 2's ego is stationary, which makes estimation easy, so this
is not a like-for-like comparison of estimator difficulty. It does not need to be. The claim
being tested was that the second run's localization is wrong; it is not wrong, it is right
to 8 cm. What remains untested is localization during the other failure mode, where the ego
does drive off the road.

That moves the question back upstream. On both never-move failures the commanded velocity
is exactly +0.000 m/s with a route SET, mode AUTONOMOUS, and trajectories arriving at the
normal rate -- and in the first pair the trajectory itself carried a maximum speed of
0.88 m/s against 4.17 on the healthy run. Something is planning a stop, rather than
something failing to follow a plan. The velocity factors published by the behaviour
velocity modules name which module inserted a stop, and that is the next thing to read.

### A note on the harness, not the bug

Two attempts at this measurement were lost to ego stacks that never reached
`Startup complete`, sitting at 86/89 composables with `LoadNode service call timed out after
120s`. That was not load and not the bridge: the installed play_launch was a hand-patched
hybrid, with three binaries replaced by hand in August and the original wheel copies left
beside them. Rebuilding the wheel from the checkout and reinstalling it fixed startup
immediately -- 3.5 minutes to `Startup complete` where the patched install had wedged for
15 minutes twice. Worth remembering before blaming a slow ONNX load for a stack that will
not come up.

## Correction: the first run is not immune (2026-08-20)

Two sections above report the first-run/second-run split as five out of five each way, and
then as seven for seven. A further pair broke it: on a freshly started stack, the **first**
run drove 120 m and then stalled off the road, and the second run stalled at 28 m. The
claim that a fresh stack always drives is wrong.

The corrected tally across every pair and hunt run recorded here:

```
  first run on a stack    8 drove, 1 failed   (8/9)
  second run on a stack   0 drove, 9 failed   (9/9)
```

The effect is still large and still worth building on -- a second run has failed every single
time it has been tried -- but it is a strong tendency on the first run, not immunity, and it
matches the roughly 3-in-33 rate this issue originally measured on fresh stacks. Any
experiment that treats a fresh run as a guaranteed control will occasionally be wrong, so
the control has to be measured rather than assumed.

## Nothing claims responsibility for the stop (2026-08-20)

With localization ruled out, the remaining reading was that something plans a stop: the
never-move runs command exactly +0.000 m/s, and one of them carried a trajectory whose
maximum speed was 0.88 m/s. Velocity factors are Autoware's own account of why it is
slowing down, naming the behaviour module responsible.

Across a pair, the only factor either run ever reports is:

```
  run 1   991/1000 msgs   traffic-signal/APPROACHING   distance 94.5..214.8 m
  run 2  1001/1001 msgs   traffic-signal/APPROACHING   distance 95.4..214.8 m
```

That is the red light this scenario commands at Init, sitting 95 to 215 m ahead. It is
expected, and it is not what stops a car that wedges 28 m from its spawn point. **No
behaviour module claims to be stopping the ego.** In this pair the trajectory head never
asks for zero either -- its minimum target velocity is 0.250 m/s on the failing run.

The caveat that matters: both runs in this pair failed by driving off the road, not by
refusing to move. So this measures the off-road variant and leaves the never-move variant
unexplained -- the one where commanded velocity really is +0.000 and the trajectory carried
0.88 m/s. The same probe needs to catch a never-move run before that variant can be
attributed, and a run that fails one way cannot be used to explain the other.
