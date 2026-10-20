# 028 — With CenterPoint, an object it has not classified never reaches planning

**Severity**: High
**Component**: Autoware 1.5's perception launch (`use_multi_channel_tracker_merger`),
`src/acb_launch/launch/carla_simulator.launch.xml`
**Status**: Open — `use_multi_channel_tracker_merger:=false` added (default unchanged);
being verified by carla-scenario-bridge roadmap 033 step 4d

## What happened

carla-scenario-bridge roadmap 033 step 4d (2026-10-20) rehearsed the demo tour's incidents
at 40 km/h on `lidar_detection_model:=centerpoint`. A broken-down car standing in the ego's
lane was hit in 2 of 5 runs, and a pedestrian stepping out from behind a van was hit in
every run. Traced per perception stage against the actor's CARLA pose:

| | `clustering/objects` | `centerpoint/objects` | `tracking/objects` |
|---|---|---|---|
| broken-down car, 7 runs | from 67-76 m (UNKNOWN) | from 36-40 m (CAR) | from 28-35 m |
| pedestrian, 6 runs | from 34-37 m (UNKNOWN) | from 17-22 m (PEDESTRIAN) | from 9-12 m, or never |

The car runs tracked from 33 m or more stopped short of it; those tracked from 30 m or less
hit it.

## Why

`autoware_launch`'s perception component defaults `use_multi_channel_tracker_merger` to
true. In lidar mode that switches the lidar merger off and gives `multi_object_tracker`
two channels: the DNN's validated objects (`centerpoint/validation/objects`) and
`detection_by_tracker/objects`. `clustering/objects` then has no subscriber (`ros2 topic
info`), and detection_by_tracker only re-finds objects that are already tracked. So with
CenterPoint, anything CenterPoint has not classified yet does not exist for planning. On
CARLA's lidar CenterPoint classifies cars from ~38 m and people from ~20 m, while
clustering sees both much further out. With `lidar_detection_model:=clustering` (acb's
default) the clustering objects are the DNN channel, so this does not arise.

## The argument

`use_multi_channel_tracker_merger` (carla_simulator.launch.xml, default **true**:
Autoware 1.5's own) is passed to acb's perception component, which already declares it.
With **false**, Autoware's lidar merger feeds a one-channel tracker
(`play_launch dump`, 2026-10-21; the default dump is identical to before but for the
argument itself):

```
centerpoint/objects + clustering/objects
  -> object_association_merger_alpha (priority 2: confidence) -> temporary_merged_objects
  + detection_by_tracker/objects
  -> object_association_merger_beta (priority 0) -> objects_before_filter
  -> object_lanelet_filter -> detection/objects
  -> multi_object_tracker (one channel, detected_objects) -> tracking/objects
```

208 nodes become 211 (the two mergers and the lanelet filter). The merger's first input is
CenterPoint's raw objects, not the validator's output (upstream's lidar_merger default).

## Closing it

Verified when the incidents that failed pass on a stack with the argument false, and the
cyclist still reaches `blind_spot` as BICYCLE (033 step 4d, option b).
