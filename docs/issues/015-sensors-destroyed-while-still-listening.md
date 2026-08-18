# 015 — A listening client's streams are destroyed underneath it

**Severity**: High
**Component**: `src/acb_bridge/src/main.rs`, `carla_vehicle.rs`; carla-rust
`destroy_with_children`
**Status**: Fixed by the release protocol (acb `33d90c2`, csb `d4d6f68`)

## Symptom

The CARLA server logs, at ~48,000 lines a second:

```
ERROR: Invalid session: no stream available with id 2      # camera
ERROR: Invalid session: no stream available with id 3      # gnss
ERROR: Invalid session: no stream available with id 4      # imu
ERROR: Invalid session: no stream available with id 5      # lidar
```

Four ids, exactly the four sensors acb attaches. Left alone it reached **184 GB** of that
one line in five hours, and the server took SIGSEGV.

### What it does *not* explain

An earlier version of this issue claimed the storm also starved the simulation — that the
ego barely moved (1286 of 8786 samples above 0.5 m/s in one measured run) and the scenario
timed out with the vehicle nowhere near its goal, making this the "second run on one stack"
failure that phase 007 exists for.

**That was wrong, and the fix disproved it.** With the release protocol in place the storm
is gone completely — `carla.log` stayed at 56 bytes, the startup banner, across a four-run
sample — and runs 2, 3 and 4 still failed on the scenario's own 180 s timeout:

```
run  verdict  ego_s  log_MB  storm_lines
1    PASS     -      0       0
2    FAIL     189    0       0
3    FAIL     187    0       0
4    FAIL     187    0       0
```

The two symptoms co-occurred because both follow a teardown; neither causes the other. The
storm was real, is understood, and is fixed. The second-run failure is a **separate,
undiagnosed problem** and is not tracked by this issue.

The lead worth pulling on there — evidence, not conclusion — is that localization
initializes at a pose unrelated to the spawn point on later runs: `(158.25, ...)` and
`(118.54, -110.32)` were observed where the scenario spawns the ego at `(190.8, -130.1)`.
That points at Autoware-side state carried across runs rather than anything in CARLA.

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

## Fix: the release protocol

The bug needed two clients to happen, so it needed two clients to fix. Neither side can do
it alone — the listener cannot unsubscribe after the destroy, and the destroyer cannot stop
a stream it never subscribed to — so the two processes now agree on *when*.

csb announces a despawn while the sensors are still alive, acb stops them and acknowledges,
csb destroys. Text frames over ZMQ, keyed on actor id:

| direction | socket | frame |
|---|---|---|
| csb -> acb | PUB / SUB | `release <actor_id> <role_name>` |
| acb -> csb | PULL / PUSH | `released <actor_id>` |

ZMQ rather than ROS because acb instances live in different ROS domains (ego on 1,
background AVs on 2+) while csb is domain-less. Endpoints are configured in
`bridge_config.yaml` (`sensor_release`) and as acb ROS parameters
(`release_notify_endpoint`, `release_ack_endpoint`).

Best effort in both directions, deliberately: a missing or slow bridge costs csb a 300 ms
timeout and a burst of log, never a stalled scenario. acb **stops only** — csb still does
the destroying, so `destroy_with_children` keeps protecting against the orphaned-sensor
crash in `docs/CHECKPOINT.md`. That tension is exactly why this needed a protocol rather
than a patch.

Measured end to end:

```
08:12:38.340  acb: Release requested for 'ego' (actor 175): stopping our sensors
08:12:38.345  acb: Stopped 4 sensor stream(s) before the vehicle is despawned
08:12:38.347  csb: Despawned 'ego' (actor_id=175)
```

7 ms, and `carla.log` stayed at **56 bytes** — the startup banner — across a four-run
sample. Before the fix the same runs produced 129k-1.4M error lines.

## Defence in depth, kept

Three earlier changes remain. None of them cured this on its own, and the write-ups above
say so, but each is correct in its own right and each shortens the window if the protocol
is ever unavailable:

- **carla-rust `68aef48`** — `destroy_with_children` stops a child sensor before destroying
  it. Fixes the case where the destroyer *is* the listener.
- **`VehicleLost` reconnects** rather than keeping the connection, dropping any sessions
  that were orphaned anyway. The clock epoch is deliberately kept: same server, continuous
  uptime, and rebasing would rewind `/clock` under a background AV that owns it.
- **The idle actor-poll** notices a despawn in ~0.5 s instead of at the next run's first
  frame, which is a promptness fix worth having regardless.

## Still open

Nothing in this issue. The storm is fixed and verified at zero.

The separate second-run failure described under "What it does not explain" is untracked and
wants its own investigation.
