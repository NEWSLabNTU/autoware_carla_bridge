# 029: From Town01's spawn point 0 the ego is held at the first corner by a street lamp

**Severity**: Medium (the stand-alone demo's default route never arrives). **Status**: Open.
Found 2026-10-24 (scenario-world-bridge roadmap 051, the "Without SSv2" acceptance).

## What happens

`just sim map:=Town01 auto_drive:=true` spawns the hero at spawn point 0 (335.5, 273.7 in
CARLA) and routes it to `autoware_sim_pilot`'s `config/poses/Town01.yaml`. Every route
from there leaves along the south edge of the town. The ego engages, drives about 70 m and
stops for good at (380.7, 330.6), heading east, short of the south-east corner:

- `/api/planning/velocity_factors`: one factor, `behavior: route-obstacle`, `status: 2`,
  0.38 m ahead;
- `/perception/object_recognition/objects`: an UNKNOWN object at (385.1, -333.2) in the map
  frame, 5.8 m ahead and 2.6 m to the right of the ego;
- CARLA's environment objects: `ArtificialLight_CityLamp80` (a street lamp, class Poles) at
  (385.2, 333.3), 0.4 m from that object. Its bounding box reaches 1.9 m from the pole, the
  lamp's arm over the road.

The same happened with the lights cycling normally, with every perception composable loaded,
with the demo ticking in real time, and with another goal (`scenarios/ego_poses.yaml` in
scenario-world-bridge): the route from spawn point 0 has no other way out.

## What it is not

Not an acb_bridge fault: the lamp is in the lidar, the clustering pipeline reports it, and
the profile's obstacle stop holds the ego for it. Nothing in roadmap 051 changed that part of
the stack, and no scenario-world-bridge scenario starts at spawn point 0 or drives this
corner, which is why nothing caught it. The same stack from spawn point 74 (the start of
scenario-world-bridge's `town01_traffic_light.xosc`) drives to that scenario's goal.

## Closing it

Find why the lamp is an obstacle on this route: whether the TUM point cloud map lacks the
lamp (so the compare-map filter keeps its points), whether the Lanelet2 lane runs closer to
the kerb here than CARLA's lane, or whether the obstacle stop's lateral margin catches the
pole on the turn. Until then, `just sim spawn_index:=74` gives the stand-alone demo a start
whose route arrives.
