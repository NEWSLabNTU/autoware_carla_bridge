# 014 — A paused simulation is indistinguishable from a running one

**Severity**: High
**Component**: `src/acb_bridge/src/main.rs`, `carla_tick` and the main loop
**Status**: Fixed

## What is wrong

```rust
fn carla_tick(world: &World, timeout: Duration) -> Result<f64, CarlaError> {
    let _ = world.wait_for_tick_or_timeout(timeout)?;   // return value discarded
    let snapshot = world.snapshot()?;
    Ok(snapshot.timestamp().elapsed_seconds)
}
```

Two mistakes, and they compound:

1. **The wait's result was thrown away.** carla-rust's signature is
   `Result<Option<WorldSnapshot>>`, and its doc says so: *"Returns `Ok(None)` if the
   timeout expires (expected behavior)."* A timeout is not an `Err`. `let _ = ...?` kept
   the error path and discarded the `Option`, so an expired wait fell through as though a
   frame had arrived.

2. **The world's state was then read from `World::snapshot()`.** That answers from the
   client's *cached* episode state, not the server. Once frames stop arriving it returns
   the last frame it saw — forever, with every actor still in it.

Together: when the simulation pauses, the bridge cannot tell. It runs its loop at the
timeout rate, reads a frozen snapshot, and publishes the dead frame's values as current.

The irony is that the code knew about this hazard. `TickOutcome::Idle` exists, is
carefully documented ("A timeout is **not** evidence that CARLA is gone"), and has a unit
test — `a_tick_timeout_is_not_a_disconnect`. All of it is about classifying a timeout
`Err` that the API never produces, so the idle branch was unreachable in practice.

## What it does to a run

Observed directly, right after `town01_ego_drive.xosc` finished and SSv2 despawned the
ego. CARLA, from a fresh client:

```
sync True dt 0.1
snapshot frame 0 elapsed 0.0        # nobody is ticking
has_actor(178) False                # the ego is gone
vehicles 0
```

The bridge, in the same moment:

```
$ ros2 topic hz /vehicle/status/velocity_status
average rate: 16.386

$ ros2 topic echo --once /vehicle/status/velocity_status   # three times, 2 s apart
longitudinal_velocity: 1.7796714305877686
longitudinal_velocity: 1.7796714305877686
longitudinal_velocity: 1.7796714305877686
```

Autoware is being told, sixteen times a second, that its vehicle is rolling at 1.78 m/s.
There is no vehicle. The value is bit-identical every time because it is one dead frame
being republished.

The despawn detection added for issue 007 could never fire either: it asked
`world.snapshot().contains(vehicle_id)`, and that snapshot still contains the actor. So
the bridge never released its sensors, never went back to `wait_for_vehicle`, and never
picked up the next scenario's ego — the exact failure the despawn check was written to
prevent, defeated by the same cached-snapshot problem one layer down.

## Fix

Return the `Option` and use the snapshot the tick actually delivered:

```rust
fn carla_tick(world, timeout) -> Result<Option<WorldSnapshot>, CarlaError> {
    world.wait_for_tick_or_timeout(timeout)
}
```

- `Ok(None)` takes the idle path — which now runs.
- `Ok(Some(snapshot))` is a real frame; `elapsed_seconds`, and the despawn check, both
  read from it.
- `Err` still goes through `classify_tick_error`.

`World::snapshot()` is no longer called in the loop at all, which also removes one RPC per
cycle.

## Verified

With the fix, on an ego stack that has just finished a scenario and has no vehicle:

```
$ ros2 topic hz /vehicle/status/velocity_status
(nothing — the bridge is correctly silent)
```

and the branch that could never run before now does, in the bridge's own log:

```
No CARLA frame for 2s; the simulation is paused (waiting for the scenario to advance frames)
No CARLA frame for 30s; the simulation is paused (waiting for the scenario to advance frames)
No CARLA frame for 60s; the simulation is paused (waiting for the scenario to advance frames)
```

Note what it says: *paused*, not *vehicle gone*. That is the honest report. Between
scenarios nobody ticks CARLA, so no frame arrives and the bridge has no information about
the actor either way. The despawn is detected on the first frame of the next run, which is
the earliest moment the server actually says anything.

`town01_ego_drive.xosc` passes with the fix in place (SSv2 JUnit `failures="0"
errors="0"`), twice.

## Rule

**Never ask `World::snapshot()`, `World::GetActor` or `Actor::IsAlive` whether something
is true right now.** All three can answer from client-side state. The only account of the
current frame is the snapshot the tick handed you.
