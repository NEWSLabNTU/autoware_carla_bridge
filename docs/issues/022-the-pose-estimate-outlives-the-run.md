# 022 — The pose estimate outlives the vehicle it belongs to

**Severity**: High
**Component**: `src/acb_bridge/src/autoware.rs` (`request_localization_seed`,
`service_localization_seed`), `src/acb_bridge/src/main.rs`
**Status**: Fixed by seeding `/initialpose` on attach, on by default
(`seed_localization_on_attach`)

## What is wrong

Nothing re-seeds Autoware's pose estimator when a new ego is spawned into a stack that
has already run a scenario.

Autoware's automatic pose initializer fires on the `UNINITIALIZED -> INITIALIZED` edge of
`/api/localization/initialization_state`. On the first scenario of a session that edge
happens and the estimate is placed correctly. On the second and later scenarios the state
never leaves `INITIALIZED`, so the initializer never fires again, and the estimate simply
carries on from wherever the previous run left it — which is the previous run's goal,
about 95 m from where the new ego is standing.

`acb_bridge` had no part in this either way: it has no client for
`/api/localization/initialize` and never published `/initialpose`. The only component in
the workspace that re-initializes deliberately is `acb_pilot`, and the pilot does not run
on a managed stack, which is where scenarios are driven from.

## Measured

Recorded live with a probe on `/localization/kinematic_state` and
`/sensing/gnss/pose_with_covariance`, starting before the scenario so the transient is
visible rather than averaged away. Second scenario on one stack, before the fix:

```
t= 14.0s  waiting for the first EKF message
t= 16.0s  gap=  95.18 m   ekf=( 94.1, -131.2)   gnss=( 189.2, -129.3)
t= 18.1s  gap=  95.17 m   ekf=( 94.1, -131.2)   gnss=( 189.2, -129.3)
t= 20.1s  gap=  95.17 m   ekf=( 94.1, -131.2)   gnss=( 189.2, -129.3)
t= 22.0s  gap=   1.75 m   ekf=(190.8, -130.1)   gnss=( 189.2, -129.3)
```

The ego spawns at x = 190.8 and drives towards x = 94; `x = 94.1` is where the *previous*
run stopped. So for the first several seconds of the run the vehicle is localized at the
previous run's goal, and then the estimate jumps ~95 m to catch up.

That jump is the "the pose travelled 93 m while the car was stationary" signature recorded
against earlier failing runs, and the 80–95 m EKF-to-GNSS gaps seen there are this same
carry-over. A healthy stack sits at 1.2–1.3 m, which is the GNSS antenna offset rather
than error.

Two things this is **not**, both checked rather than assumed:

- **Not NDT failing to match.** Through a full healthy run, `transform_probability` held a
  median of 6.6 against a converged threshold of 3.0 and never once dropped below it, and
  `nearest_voxel_transformation_likelihood` held 3.10 against 2.3. The scan matcher is
  healthy; it is being asked to start from the wrong place.
- **Not orphaned sensors starving the server.** Parentless sensors left behind by a killed
  client were suspected of stealing simulation time from the live LiDAR. Measured directly,
  five orphaned 128-channel LiDARs cost **+0.0 ms per frame** (1.00x, 165 fps either way):
  CARLA does not ray-cast a sensor nothing is listening to, so they are free. The
  correlation that suggested them was three runs of noise.

## Why it matters

The planner acts on the estimate. For the first seconds of every run after the first, it
is planning for a vehicle 95 m from the real one, on a different piece of road. Usually
the estimate is dragged back and the run recovers, which is why this hid for so long — but
the recovery is incidental, nothing guarantees it, and when it does not happen the run
fails with the ego stationary and its pose somewhere else entirely.

## Fix

`acb_bridge` knows exactly when a new vehicle appears, because it attaches sensors to it,
and it knows the vehicle's true pose in the frame `/initialpose` wants, because `tick`
already converts it to publish ground truth. So on attach it asks for a re-seed, and
`tick` services the request: publish `/initialpose` at the vehicle's true pose, then watch
`/localization/kinematic_state` until it agrees.

The seed is deliberately narrow, and the narrowing is the important part:

- **It only acts on an estimate that already exists and is in the wrong place.** If no
  estimate has arrived at all, the stack is cold, Autoware's own initializer is doing this
  job, and seeding into it would restart the alignment it is running. That case — the
  first run of a session — is the case that never failed, and the seed stays out of it.
  Confirmed: on a cold run the seed sent **zero** messages and the run passed.
- **It stops on agreement**, at 5 m, chosen to sit well clear of the healthy 1.2–1.3 m and
  well below the 95 m fault.
- **It retries no faster than an alignment takes** (10 s), because a second seed sent into
  a running NDT alignment restarts it, and gives up after 60 s with a warning rather than
  publishing into a run forever.

Measured on the second scenario of a stack, with the fix:

```
Localization re-seed requested; publishing /initialpose until the estimate agrees ...
Sent /initialpose #1 at (190.80, -130.10); the estimate was 97.6 m away
Localization seeded: the estimate is 0.06 m from the vehicle after 1 /initialpose message(s)
```

One message, 7.1 s to converge, from 97.6 m to 0.06 m.

### What the fix does not do

It does not remove the window. The sequence is: ego spawns, `acb_bridge` attaches (~1 s),
the seed goes out, NDT aligns and the EKF accepts the jump (~7 s). Those seven seconds are
the pipeline's own latency, not slack in the fix, and the estimate is wrong for the whole
of it. What changes is that the correction is now *caused and confirmed* — the bridge logs
the distance it converged to — rather than left to a recovery that sometimes did not
happen.

Closing the window properly would mean seeding before the run is allowed to start, which
needs the scenario runner to wait on localization rather than on the clock.

### A live estimate and a readable one are not the same thing

The first version of the convergence check read the last `/localization/kinematic_state`
message with no notion of when it arrived. That is wrong in a way worth recording, because
it looked right: a subscription holds the last message it ever received, so on a stack
whose localization had not started publishing for *this* run yet, the check happily read
the *previous* run's final pose and reported a rock-steady 96.6 m gap for a full minute --
six `/initialpose` messages into a pipeline that was not listening. The estimate being
frozen to the centimetre across sixty seconds is what gave it away; a live EKF tracking a
car does not do that.

So `estimate_gap` now requires the last message to be under two seconds old, and a stale
one counts as *no* estimate rather than as a distant one. That change alone took the seed
from six messages per run to exactly one.

## Does it help?

Honestly: it fixes the error, but on this scenario it does not move the pass rate.

Arms alternating across freshly restarted ego stacks, three consecutive runs each, run
index appearing in both arms, and the arm read back per stack from play_launch's own
`params_files/overrides.yaml` rather than inferred:

```
seed on     8/9 passed
seed off    7/9 passed
```

At nine runs per arm that difference is nothing, and it should not be reported as an
improvement. What the seed demonstrably does is turn a 95 m error that was corrected by
luck into one corrected on purpose and confirmed in the log -- one `/initialpose`, 0.04 m.
The reason the pass rate barely moves is the same reason this hid for months: the
incidental recovery usually did happen. This fix is justified by the measured error and
its deterministic correction, not by a pass-rate delta it is far too small a sample to
show.

## Related

- `acb_pilot` had the same blind spot from the other end: it re-initialized on startup but
  then accepted any *fresh* EKF pose as proof of success. A stale estimate publishes at
  full rate forever, so it satisfied both `INITIALIZED` and "fresh" while being 95 m out.
  It now requires the pose to agree with GNSS (`localization_gap_limit_m`, default 5 m)
  before it will drive, and re-initializes when it does not.
