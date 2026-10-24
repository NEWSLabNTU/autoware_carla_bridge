# 029: From Town01's spawn point 0 the ego is held at the first corner by a street lamp

**Severity**: Medium (the stand-alone demo's default route never arrives). **Status**: Fixed
(autoware_sim_vehicle `25c4f29`: detection's compare-map filter loads the map statically).
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
the stack. No scenario-world-bridge scenario starts at spawn point 0, which is why nothing
caught it. The same stack from spawn point 74 (the start of scenario-world-bridge's
`town01_traffic_light.xosc`) drives to that scenario's goal.

One scenario does drive this corner: scenario-world-bridge's demo incident
`town01_i3_red_light` passes it after its light. On the default stack (clustering) the lamp
holds that ego too (seen 2026-10-24, roadmap 051 K5, after issue 030's repair). The suite
runs the incidents on the CenterPoint stack (`lidar_detection_model:=centerpoint`), which
does not report the lamp, and i3 passes there in 87 s. So the incident passes or not by
which detector the stack runs.

## Cause

Not the map, the lane or the obstacle stop's margin. The TUM point cloud map has the lamp:
its pole at (385.2, -333.3), sampled every 0.1-0.2 m from the ground to 7.8 m, and the arm.
What failed was detection's compare-map filter (`voxel_based_compare_map_filter`), which is
there to drop the points the map already has before clustering. Measured 2026-10-24 on
`town01_i3_red_light`, the ego held at (379.4, -330.6) by `route-obstacle`:

- 305 lamp points within 1.5 m of the pole in `/perception/obstacle_segmentation/pointcloud`,
  53 of them still in the filter's output, every one within 0.13 m of a map point;
- within 80 m of the ego the output held 7903 points, 7332 of them within 0.3 m of the map:
  the filter was removing almost nothing.

The filter ran with upstream's `use_dynamic_map_loading: true`. The CARLA towns' maps are one
unsplit `pointcloud_map.pcd`, which the map loader serves as a single cell (Town01: one
497 x 418 m cell); the dynamic loader in Autoware 1.5 fetched it (after about 80 s of
`Waiting for response...`) and still matched next to nothing. The grid index arithmetic in
`voxel_grid_map_loader.hpp` is also fragile for a single cell: its float origin can put the
cell and the points in different grid slots by a rounding (at an ego near (100, -100) the
cell lands in slot 0 and the points in slot 1). CenterPoint does not use this filter, which
is why the incidents passed on it.

## The fix

autoware_sim_profile passes detection its own copy of `pointcloud_map_filter.param.yaml`,
upstream's but with `use_dynamic_map_loading: false` (derived by
`scripts/sync_perception_component.sh`, its fourth change). The static loader voxelises the
whole map once from `/map/pointcloud_map`. Same run, same place: the filter's output fell to
1504 points within 80 m, none on the lamp, the ego took the corner, and
`town01_i3_red_light` passed on the clustering stack (it failed on its 300 s limit
without the fix).

Repeated on 2026-10-24 with the fix, each on a fresh clustering stack with i3 first: 6 of 7
passed (5 in a row, then 2 more), plus once inside the regression set. The one failure never
reached the corner: SSv2 gave up waiting for WAITING_FOR_ROUTE because the stack's first NDT
alignment took 105 s, against 6-9 s in every other stack that day, while other users' work
held the host's load near 40 on 48 cores. Not this issue; a start on a loaded host.
