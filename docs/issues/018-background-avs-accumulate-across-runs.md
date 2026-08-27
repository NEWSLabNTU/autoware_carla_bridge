# 018 — Background AVs accumulate across scenario runs

**Severity**: Medium
**Component**: `carla_scenario_bridge`, `coordinator.rs` (background AV spawning)
**Status**: Open

## Symptom

A long-lived `carla_scenario_bridge` spawns its configured background AVs on every scenario
`Initialize` and never despawns the previous run's. They pile up at the same spawn point, so
run 2 has two `bg_av_1`s, run 3 has three, and so on.

Caught by the new ground-truth object publisher, which reported two objects where the
scenario defines one:

```
  actor 192 (bg_av_1) pub (230.01,-129.79)  carla speed 0.01 m/s
  actor 180 (bg_av_1) pub (230.01,-129.80)  carla speed 0.00 m/s
```

Two actors, the same role name, the same spot, both stationary. The bridge's own log shows
the two spawns on one process with nothing in between:

```
11:09:08  Spawning 1 background AV(s)
11:09:08  Spawned background AV 'bg_av_1' (actor_id=186, ...) at CARLA(230.0, 129.8, 1.8)
11:10:25  Spawning 1 background AV(s)
11:10:25  Spawned background AV 'bg_av_1' (actor_id=192, ...) at CARLA(230.0, 129.8, 1.8)
```

## Why it matters

Beyond the leak, the duplicates sit exactly where the next one spawns, so they collide with
each other and with any vehicle routed through that point. A stationary unexpected obstacle
in the ego's lane changes what the scenario is testing, which makes run 2 a different
experiment from run 1 without saying so. That is the same shape as issue
[016](016-the-ego-stack-degrades-after-its-first-run.md), and anyone measuring run-order
effects on a long-lived bridge should rule this out first.

It also puts two actors on one SSv2 name. `entity_manager`'s name-to-actor mapping can only
hold one, so commands addressed to `bg_av_1` reach whichever won the map, while the others
stay parked and unaddressable.

## Cause, not yet confirmed

The despawn path exists for the ego, which is removed at scenario end via the release
protocol from issue [015](015-sensors-destroyed-while-still-listening.md). Background AVs
appear to have no equivalent teardown: they are spawned from `bridge_config.yaml` at
`Initialize`, not from the scenario's entity list, so nothing in the scenario's own lifecycle
removes them.

## What to check when fixing

- Despawn background AVs at scenario end, symmetrically with the ego, rather than at the next
  `Initialize` -- a bridge sitting idle between runs should not be holding vehicles.
- Their acb_bridge instances are listening to their sensors, so the release protocol applies
  to them too. Destroying them without the handshake is what issue 015 is about.
- If a run leaves them behind anyway (a killed bridge, a crashed server), the next
  `Initialize` should reconcile: an existing actor with a configured background AV's role
  name is the previous run's, not a second vehicle.

## Workaround

Restart `carla_scenario_bridge` between runs, or check for duplicates before trusting a run:

```bash
python3 -c "
import carla
w = carla.Client('localhost', 2000).get_world()
print([(a.id, a.attributes.get('role_name')) for a in w.get_actors().filter('vehicle.*')])"
```
