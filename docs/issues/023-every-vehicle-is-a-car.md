# 023 — Every vehicle is published as a car

**Severity**: Medium
**Component**: `src/acb_bridge/src/ground_truth_objects.rs` (`classify`, `vehicle_label`)
**Status**: Fixed (classified by CARLA's `base_type` attribute)

## What is wrong

The ground-truth object publisher (`publish_ground_truth_objects`, roadmap 9 gap 3)
labelled each actor by the prefix of its type id alone:

```rust
let label = if type_id.starts_with("vehicle.") {
    ObjectClassification::CAR
} else if type_id.starts_with("walker.") {
    ObjectClassification::PEDESTRIAN
} ...
```

CARLA files bicycles, motorbikes, trucks and buses under `vehicle.*` too, so a cyclist, a
Vespa, a fire truck and the Fusorosa bus all reached Autoware as `CAR`. Only `CAR`,
`PEDESTRIAN` and nothing else ever appeared on `/perception/object_recognition/objects`.

## Why it matters

Autoware branches on the label. `map_based_prediction` predicts vehicles along lanes and
pedestrians and bicycles with its own models; behavior modules such as `blind_spot` and
`intersection` filter their targets by class (blind_spot checks two-wheelers on the turn
side); `obstacle_cruise_planner` picks its target types from the label list. A cyclist
labelled `CAR` is handled as a slow car in the lane, which is not what a scenario built
around a cyclist is testing. Real perception does emit `BICYCLE`, `MOTORCYCLE`, `TRUCK`
and `BUS`, so ground-truth runs and LiDAR runs were not exercising the same planning
paths.

Medium rather than High: the publisher is off by default, and only runs that turn it on
(`GROUND_TRUTH_OBJECTS=true`) are affected.

## What CARLA 0.9.16 provides

Every `vehicle.*` blueprint carries a `base_type` string attribute (since 0.9.14), plus
`special_type` and the integer `number_of_wheels`. The attributes are copied into the
spawned actor's description, so `Actor::attributes()` reads them locally. Listed from the
live 0.9.16 server (all 41 vehicle blueprints, 2026-10-17):

| `base_type` | Blueprints |
|-------------|------------|
| `car` | 28 passenger cars incl. police, taxi, microlino |
| `truck` | `carlamotors.carlacola`, `carlamotors.european_hgv` (6 wheels), `carlamotors.firetruck`, `tesla.cybertruck` |
| `van` | `ford.ambulance`, `mercedes.sprinter`, `volkswagen.t2`, `volkswagen.t2_2021` |
| `Bus` (capital B) | `mitsubishi.fusorosa` |
| `motorcycle` | `harley-davidson.low_rider`, `kawasaki.ninja`, `vespa.zx125`, `yamaha.yzf` |
| `bicycle` | `bh.crossbike`, `diamondback.century`, `gazelle.omafiets` |
| *(empty)* | `bmw.grandtourer`, `mini.cooper_s` (both 4 wheels) |

Two things the table forces: the match must ignore case (`Bus`), and an empty value is
real, not hypothetical.

## Fix

`classify(type_id, base_type, number_of_wheels)` maps, case-insensitively:

| CARLA | Autoware `ObjectClassification` |
|-------|---------------------------------|
| `walker.*` | `PEDESTRIAN` |
| `base_type = car` | `CAR` |
| `base_type = van` | `CAR` — Autoware has no van class; CARLA's vans are 4.5–6.4 m passenger-sized bodies, and `map_based_prediction` treats CAR/TRUCK/BUS alike anyway |
| `base_type = truck` | `TRUCK` (the cybertruck too: CARLA calls it a truck) |
| `base_type = bus` | `BUS` |
| `base_type = motorcycle` | `MOTORCYCLE` |
| `base_type = bicycle` | `BICYCLE` |
| anything else, 2 wheels, id in the known-bicycle list | `BICYCLE` |
| anything else, 2 wheels | `MOTORCYCLE` |
| anything else | `CAR` |
| not `vehicle.*` / `walker.*` | not published |

`TRAILER` is never produced: 0.9.16 has no trailer blueprint (`european_hgv` is one
tractor body with `base_type = truck`).

**Fallback, when `base_type` is empty or unrecognised.** `number_of_wheels` separates
two-wheelers from everything else. Between bicycle and motorcycle no attribute decides
(`special_type` is empty on all seven two-wheelers), so the three bicycle ids CARLA
ships are listed by name and every other two-wheeler is a `MOTORCYCLE`. The default goes
that way because a motorcycle is the faster class: an unknown two-wheeler predicted as a
motorcycle is given room it may not need, one predicted as a bicycle can be given too
little. Today the fallback only fires for the two empty-`base_type` cars, which fall to
`CAR`; it is there for custom blueprints and future CARLA releases.

Walkers stay `PEDESTRIAN`. Only vehicles read attributes; the type-id prefix still
discards the hundred-odd `static.*` props first.

## Verification

- Table-driven unit test `classify_*` in `ground_truth_objects.rs`, covering every
  `base_type` value 0.9.16 emits, the `Bus` capitalisation, both fallbacks and the
  walker and non-traffic cases. Written first and seen failing against the old
  prefix rule.
- Live, 2026-10-17: ego stack with `GROUND_TRUTH_OBJECTS=true` and `perception:=false`,
  csb's `scenarios/gap/town01_entity_bodies.xosc` (one parked entity per subtype), a probe
  on `/perception/object_recognition/objects` in the ego domain:

  ```
  actor=792 label=CAR        size=(3.71,1.79,1.55) pos=(203.7,-129.5)
  actor=794 label=MOTORCYCLE size=(2.04,0.80,1.52) pos=(214.4,-129.5)
  actor=797 label=BICYCLE    size=(1.66,0.58,1.62) pos=(223.5,-129.5)
  actor=800 label=TRUCK      size=(7.94,2.89,3.46) pos=(245.5,-129.5)
  actor=803 label=BUS        size=(10.27,3.94,4.25) pos=(264.5,-129.5)
  ```

  The old prefix rule labels all five `CAR` (reproduced by the unit test, not re-run
  live). The scenario's barrel is a `static.*` prop and is
  not published, as before. The run itself ended in SSv2's `AutowareError` (Autoware
  stayed in `PLANNING`, never `WAITING_FOR_ENGAGE`) — unrelated to the labels; it was
  the perception component's other outputs going missing with it, fixed in issue 026.
