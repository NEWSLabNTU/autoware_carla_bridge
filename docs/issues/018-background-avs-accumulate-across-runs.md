# 018 — Background AVs accumulate across bridge processes

**Severity**: Medium
**Component**: `carla_scenario_bridge`, `coordinator.rs` (Initialize, background AV spawning)
**Status**: Fixed by the orphan reaper; verified over three consecutive runs

## Symptom

Two CARLA vehicles answering to one `role_name`, parked on top of each other:

```
  actor 192 (bg_av_1) at (229.4, 128.7, 0.74)
  actor 180 (bg_av_1) at (230.0, 129.7, -0.03)
```

Found by the ground-truth object publisher, which reported two objects where the scenario
defines one. `bridge_config.yaml` validation already rejects a duplicate `role_name` at
parse time, because acb_bridge finds its vehicle by that name and cannot choose between two
-- so two live actors sharing one is a violation of the bridge's own contract, reached at
runtime instead of in the config.

## Cause

**Not** what the first version of this issue claimed. That version said the bridge re-spawns
its background AVs every `Initialize` and never despawns the previous run's. The log
disproves it: within one process the `SpawnLedger` teardown works, and says so on every
re-initialise.

```
Teardown: 2 spawned, 2 destroyed, 0 failed
Initialize: cleaned up 2 actor(s) from the previous run
```

The real gap is **across** processes. `destroy_all_spawned` iterates the ledger, and the
ledger only holds what *this* process created. A bridge that was killed, or that died with
the server, leaves its vehicles in the world with no record of them anywhere. The next
bridge starts with an empty ledger, sees nothing to clean up, and spawns its own `bg_av_1`
beside the old one.

This repository's own notes say a bridge outliving its session is routine -- CLAUDE.md
describes one found still serving scenarios 44 hours later -- so the cross-process case is
the normal case, not an edge one.

### The retry ladder hid it

`spawn_entity` retries a failed spawn at increasing height, for ground that sits higher than
the pose claims. A leftover vehicle on the spawn point produces the same failure, so the
ladder lifted the new vehicle over the old one and reported success:

```
Spawn of 'bg_av_1' at z=0.30 failed (attempt 1/5): Spawn returned null (likely collision ...)
Spawn of 'bg_av_1' at z=0.80 failed (attempt 2/5): ...
Spawn of 'bg_av_1' at z=1.30 failed (attempt 3/5): ...
Spawned 'bg_av_1' on attempt 4 at z=1.80 (commanded z=0.30)
```

Three collisions and a car dropped on a car, logged at INFO as a successful spawn. This is
why the problem survived unnoticed: the mechanism that should have reported "something is
already parked here" was busy working around it.

## Fix

**Reap orphans by role name at Initialize** (`reap_orphaned_vehicles`), after the map is
settled and before anything spawns. Any vehicle carrying the configured ego role name or a
configured background AV's role name, and absent from this process's ledger, is a previous
process's leftover and is destroyed. Their acb_bridge may still be listening to sensors on
them, so they go through the same release announcement as our own actors -- see issue
[015](015-sensors-destroyed-while-still-listening.md).

Only role names this bridge is configured to own are touched. A manually spawned vehicle, or
another tool's traffic, is left alone.

**And make the ladder honest.** When a spawn succeeds only after lifting, the bridge now
looks for a vehicle on the commanded point and names it:

```
Spawned 'X' on attempt 4 at z=1.80 (commanded z=0.30) because actor 180 ('bg_av_1') is
already on that point -- 'X' is now stacked above it
```

at WARN rather than INFO. Reaping should mean this never fires; if it does, the run knows
what it landed on.

## Verified

Against the two orphans above, left by bridges killed during earlier work:

```
WARN  Found 2 vehicle(s) from a previous bridge process still in the world; destroying them
WARN  Destroyed orphaned 'bg_av_1' (actor 192)
WARN  Destroyed orphaned 'bg_av_1' (actor 180)
INFO  Spawned background AV 'bg_av_1' (actor_id=198, ...)
```

The spawn then succeeded on the first attempt at the commanded z -- no collisions, no lift,
where the same configuration had needed four attempts before.

Then three consecutive scenarios on one bridge and a healthy server:

```
run 1                                    Spawned background AV 'bg_av_1' (actor_id=174)
run 2  Teardown: 1 spawned, 1 destroyed  Spawned background AV 'bg_av_1' (actor_id=180)
run 3  Teardown: 2 spawned, 2 destroyed  Spawned background AV 'bg_av_1' (actor_id=186)
```

No orphans reported on runs 2 and 3, which is the point: the ledger had already accounted for
those actors, so the reaper found nothing left to do. No failed spawn attempts and no lifts
anywhere in the three runs.

## Why it mattered

Beyond the leak, the duplicates sat exactly where the next vehicle spawns, so a stationary
unexpected obstacle appeared in the scenario without the scenario saying so -- which makes
run 2 a different experiment from run 1. That is the same shape as issue
[016](016-the-ego-stack-degrades-after-its-first-run.md), and anyone measuring run-order
effects on a long-lived bridge should rule this out first.
