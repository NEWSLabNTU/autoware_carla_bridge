# 015 — Sensors are destroyed while still listening

**Severity**: High
**Component**: `src/acb_bridge/src/carla_vehicle.rs`, `cleanup`
**Status**: Fixed; crash-rate measurement in progress

## What is wrong

```rust
for (name, sensor) in self.sensors.drain() {
    tracing::info!("Destroying sensor '{}' (ID: {})", name, sensor.id());
    match sensor.destroy() { ... }
}
```

The sensor is destroyed while its stream is still open. `Sensor::stop()` is never called.

Every CARLA example does `sensor.stop()` before `sensor.destroy()`, and the reason is the
server side: destroying the actor tears down the stream session while the server may still
be delivering frames into it.

## What it actually does: an error storm, then a crash

The crash is the end of the story, not the whole of it. The server's log, captured across
one CARLA instance that served several scenario runs:

```
$ du -sh carla.log
184G    carla.log

$ head -c 3000 carla.log | tail
ERROR: Invalid session: no stream available with id 2      # camera
ERROR: Invalid session: no stream available with id 3      # gnss
ERROR: Invalid session: no stream available with id 4      # imu
ERROR: Invalid session: no stream available with id 5      # lidar
...
```

**184 GB of that one line**, four stream ids — exactly the four sensors acb attaches —
accumulated over about five hours, roughly **10 MB/s**, continuously, from the moment the
first scenario tore its sensors down. At ~48 bytes a line that is on the order of 200,000
errors a second; this is not one error per dropped frame, it is a hot loop.

What that does to a run, in order:

1. **The first scenario on a fresh server passes.** Nothing is orphaned yet.
2. Its teardown orphans four streams, and the storm starts.
3. **Every later run degrades.** The server spends its time on the error path and on I/O;
   the ego barely moves (1286 of 8786 samples above 0.5 m/s in one measured run) and the
   scenario hits its own 180 s timeout with the vehicle nowhere near the goal.
4. Eventually the server segfaults.

Measured directly, on a stack whose CARLA had already served one run — three consecutive
scenarios, all failing the same way, with the server still up:

```
run  verdict  ego_s  carla_alive_after
1    FAIL     -      yes
2    FAIL     190    yes
3    FAIL     190    yes
```

This is the "second run on one stack" problem that phase 007 is about, and it is why a
fresh CARLA has always looked like the fix.

It is also a disk hazard in its own right: left alone over a long session it will fill the
filesystem.

## The crash

Observed on 2026-08-17 after a scenario run, in the CARLA server's own log:

```
ERROR: Invalid session: no stream available with id 124      (x13)
Signal 11 caught.
CommonUnixCrashHandler: Signal=11
Engine crash handling finished; re-raising signal 11 for the default handler. Good bye.
Segmentation fault (core dumped)
```

The thirteen `Invalid session` lines are frames arriving for a stream whose subscriber has
gone. The segfault follows.

This was not free. It cost three scenario runs before it was understood:

- two runs failed on the scenario's own 180 s timeout, with the ego mostly stationary
  (1286 of 8786 samples moving) — the sensor streams had broken, so Autoware had no LiDAR
  to plan on;
- one run failed at `change_to_stop`, because by then the server was gone.

None of the three looked like a server problem from the ROS side. Restarting CARLA and
running again, with no code change, passed in 47 s.

## Relationship to the known teardown rule

`docs/CHECKPOINT.md` already carries a rule for the other half of this: *never destroy a
vehicle while its sensors are attached* — the `AInertialMeasurementUnit::ComputeGyroscope`
crash on an orphaned sensor, worked around by destroying children first.

This is the complementary case. That rule is about **who** is destroyed first; this is
about **what state** the sensor is in when it goes. A sensor can be correctly destroyed
before its parent and still take the server down if it is still streaming.

## Fix

`stop()` before `destroy()`, best-effort — a sensor that was never listening, or whose
actor is already gone, has nothing useful to report:

```rust
if let Err(e) = sensor.stop() {
    tracing::debug!("Sensor '{}' stop failed (already gone?): {e}", name);
}
```

## Measured: partly fixed, root cause still open

Three arms, each a fresh CARLA plus the same 6-run harness, recording the SSv2 verdict and
the server's `Invalid session` line count after each run.

| arm | run 1 | run 2 | run 3 | storm after run 1 |
|---|---|---|---|---|
| **A** no `stop()` anywhere | PASS | FAIL | FAIL | 184 GB over 5 h |
| **B** `stop()` in acb's `cleanup()` only | PASS | — | — | 114,456 |
| **C** + carla-rust `destroy_with_children` (`68aef48`) | PASS | PASS | FAIL | 29,465 |

C is a real improvement — the first failure moves from run 2 to run 3, and run 2 drives
clean in 40 s — but the storm still grows (29k → 565k → 1.1M) and still ends in failure.

**Arm B is the useful negative result.** acb's own `cleanup()` fix is unreachable in this
workflow: csb owns the ego's lifetime and destroys it, sensors and all, through
`destroy_with_children()` *before* acb notices the vehicle is gone. That is why the fix
had to move into carla-rust, where both bridges go through one implementation.

Note what `stop()` can and cannot do. `ServerSideSensor::Stop()` early-returns unless the
calling client's own `_is_listening` flag is set, and that flag is per client object. csb
never called `Listen()` on the ego's sensors — acb did — so csb stopping them is a no-op
for exactly the streams that matter. This is why C helps but does not cure.

### What it is not

A second hypothesis, that acb held its subscriptions open across the gap between scenario
runs and the server spun on them, was tested and **refuted**. With the ego despawned and
both clients alive and idle, the server emits nothing:

```
baseline storm rate: 0 B/s   (acb_bridge: 1, adapter: 1)
after acb_bridge died:   0 B/s
after adapter died:      0 B/s
```

The storm grows *during* runs, against stream ids belonging to the *previous* run's
sensors. Something re-touches those ids when the next run's sensors are created, and the
ids are reused. That is where the remaining cause is, and it is not yet found.

The idle actor-poll added to `main.rs` alongside this work is therefore a promptness fix
only — the bridge now notices a despawn within ~2 s instead of at the next run's first
frame — and is documented as such rather than as a fix for this issue.

## Measuring it

`stop()` before `destroy()` is correct regardless — it is what the API asks for. Whether it
*fixes the crash* is a separate claim, and CARLA 0.9.16 has more than one teardown race, so
it may only reduce the rate.

The measurement is a repeat harness (`scripts/`, run out of tree): N consecutive
`town01_ego_drive.xosc` runs on one stack, recording the SSv2 verdict, whether the server
survived, and how long the ego took, restarting CARLA when it dies so a crash costs one run
rather than the sample. Run before the fix, then after, same N.

Until those two numbers exist, this issue claims only that the code was wrong, not that the
crash is gone.
