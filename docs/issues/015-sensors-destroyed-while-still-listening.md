# 015 — A listening client's streams are destroyed underneath it

**Severity**: High
**Component**: `src/acb_bridge/src/main.rs`, `carla_vehicle.rs`; carla-rust
`destroy_with_children`
**Status**: Mitigated in acb; the ordering problem it works around is still open

## Symptom

The CARLA server logs, at ~48,000 lines a second:

```
ERROR: Invalid session: no stream available with id 2      # camera
ERROR: Invalid session: no stream available with id 3      # gnss
ERROR: Invalid session: no stream available with id 4      # imu
ERROR: Invalid session: no stream available with id 5      # lidar
```

Four ids, exactly the four sensors acb attaches. Left alone it reached **184 GB** of that
one line in five hours and the server took SIGSEGV. Long before the crash it starves the
simulation: the ego barely moves (1286 of 8786 samples above 0.5 m/s in one measured run)
and the scenario times out with the vehicle nowhere near its goal. This is the "second run
on one stack" failure phase 007 exists for, and it is why a fresh CARLA has always looked
like the cure.

## Cause

The message comes from `carla::streaming::detail::Dispatcher::RegisterSession` (confirmed
by symbol in the shipped binary). It fires when a **client opens a streaming session for a
stream id with no live entry in the dispatcher's map** — the map holds `weak_ptr`s, so a
destroyed sensor's entry expires. The client retries, forever.

So the storm is one client's reconnect loop against streams that no longer exist. It takes
**two** clients to create:

- **acb** attaches the sensors and calls `Listen()` on them. It owns the sessions.
- **csb** owns the ego's lifetime and despawns it — vehicle *and children* — so acb's
  sensors are destroyed while acb is still listening.

Once destroyed there is no way back: `Sensor::stop()` on a dead actor fails with
`close: Bad file descriptor`, so the listener cannot unsubscribe retroactively.

## Reproduced, minimally

`scripts/repro_two_clients.py` (`CARLA_LOG=/path/to/server.log`) — no ROS, no Autoware. Client A spawns four sensors on B's
vehicle and listens; B destroys them.

```
mode=nostop   (A listens, B destroys)
cycle 1
  A listening, 20 ticks             0.00 MB  errors=0
  B destroys children+vehicle       0.04 MB  errors=804     id87=197 id101=217 id102=198 id103=192
  60 ticks, nothing alive           0.15 MB  errors=2702    id87=683 id101=691 id102=672 id103=656
  5 s paused                       12.42 MB  errors=229556  id87=57425 id101=57404 id102=57302 id103=57425
cycle 2
  A listening, 20 ticks             0.04 MB  errors=829     id87=258 id101=255 id102=52  id103=264
  B destroys children+vehicle       0.03 MB  errors=612     id104=151 id118=151 id119=154 id120=156
```

Three things to read out of that:

1. **Stream ids are not reused.** Cycle 1 gets 87/101/102/103, cycle 2 gets
   104/118/119/120. Each teardown adds four more permanently-retrying sessions, which is
   why the storm grows run over run and never subsides.
2. **It storms hardest while paused.** Ticking is irrelevant; the retry loop is the
   client's own.
3. **In cycle 2, the errors are still about cycle 1's ids.** The old loops never stop.

The control settles the rule — A stopping its sensors *before* B destroys them:

```
mode=stop     every phase, both cycles:  errors=0
```

And the ownership, at process granularity (`scripts/repro_killproc.py`):

```
listener alive, sensors alive        0.00 MB/s        0 errors/s
sensors destroyed, listener alive    2.63 MB/s   48,313 errors/s
listener process KILLED              0.00 MB/s        0 errors/s
```

Killing the listening client stops it instantly. Nothing else does — dropping the Python
handles only halved it, because Python does not actually close the connection.

## What does not fix it

- **A single client destroying its own listening sensors.** `scripts/repro_streams.py`
  does exactly that, both with and without `stop()`, and produces **zero** errors. The bug
  needs the listener and the destroyer to be different clients. An earlier version of this
  issue claimed the single-client case was the cause; it was wrong.
- **`stop()` from the destroying client.** `ServerSideSensor::Stop()` early-returns unless
  the calling client's own `_is_listening` flag is set, and that flag is per client object.
  csb never called `Listen()`, so csb stopping them is a no-op for the streams that matter.
  carla-rust `68aef48` does this and is still worth having — it fixes the case where the
  destroyer *is* the listener — but it is not the cure here.
- **Noticing the despawn sooner.** The idle actor-poll in `main.rs` makes acb see the
  despawn in ~2 s instead of at the next run's first frame. Useful on its own, but the
  sessions are already unrecoverable by then.

## Mitigation, and what it is worth

`SessionExit::VehicleLost` now **reconnects** to CARLA instead of keeping the connection.
Dropping the connection is the only lever the listener owns once its actors are gone, and
it was already the accidental cure: `wait_for_vehicle` reconnects after
`WAIT_RECONNECT_INTERVAL` (60 s), so a long idle gap self-healed while back-to-back
scenario runs never got that far. Doing it at once bounds the damage to the detection
window.

The clock epoch is deliberately **not** reset on this reconnect. It talks to the same
server, whose uptime never paused; rebasing there would rewind `/clock` under a background
AV that owns it — the "Detected jump back in time" failure. `carla_may_have_restarted`
separates the two reasons for reconnecting. `Autoware::clear_vehicle` drops the
coordinator's `CarlaVehicle` first, since it outlives the connection loop and would
otherwise hold a `Vehicle` — and through it the episode and the old client.

Profiling a run shows the shape this produces. The storm is **not** continuous; it is one
burst per teardown, and nothing at all in between:

```
t=  5s  delta=10975 KB      <- previous run's ego destroyed under us
t= 10s  delta= 3864 KB
t= 15s ... t=195s  delta=0 KB
t=200s  delta= 6157 KB      <- this run's teardown
```

Measured per run, with a fresh CARLA each time:

| arm | run 1 | run 2 | run 3 | per-run storm |
|---|---|---|---|---|
| no `stop()` anywhere | PASS | FAIL | FAIL | unbounded, 184 GB over 5 h |
| + carla-rust `68aef48` | PASS | PASS | FAIL | ~540k lines |
| + `VehicleLost` reconnect | PASS | FAIL | FAIL | ~400k lines |
| + `clear_vehicle` | PASS | FAIL | — | ~410k lines |

The honest reading: the leak between runs is gone — measured directly at **0 KB/s** during
a gap, against 2.6 MB/s unmitigated — but each teardown still costs one burst whose size is
the detection latency times 2.6 MB/s. The run-level pass rate on four runs per arm is too
noisy to claim anything from; the storm rate is the measurement that means something here.

`IDLE_TICKS_BETWEEN_ACTOR_CHECKS` is therefore the whole remaining cost, and is set to
~0.5 s. At 2 s a burst was ~10 MB; this quarters it.

## Still open

The mitigation drops sessions after the fact and pays one burst per teardown. The actual
fix is ordering: **the listener must `stop()` before the owner destroys**, which needs the
two processes to agree. Options, none implemented:

- csb signals acb to release its sensors before despawning the ego, and waits for an ack.
- acb, not csb, destroys the sensors it attached — but then csb must not use
  `destroy_with_children` on the ego, which is what protects against the orphaned-sensor
  server crash documented in `docs/CHECKPOINT.md`. Those two rules are in direct tension
  and the resolution needs a protocol, not a patch.

Either would take the burst to zero, which is the only way this stops costing anything.
