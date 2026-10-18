# 026 — With `perception:=false` the stack never leaves PLANNING

**Severity**: High
**Component**: `src/acb_launch/launch/carla_simulator.launch.xml`,
`src/acb_launch/launch/components/{tier4_perception_component,perception}.launch.xml`,
`scripts/sync_perception_component.sh`
**Status**: Fixed (`perception:=false` drops object recognition only)

## What happened

Issue 023's live check (2026-10-17): ego stack with `GROUND_TRUTH_OBJECTS=true` and
`perception:=false`, csb `scenarios/gap/town01_entity_bodies.xosc`. acb published the
ground-truth objects correctly, but SSv2 ended in `AutowareError`: Autoware stayed in
`PLANNING` and never reached `WAITING_FOR_ENGAGE`. Ground-truth mode with
`perception:=false` was in no regression suite.

`PLANNING` is the AD API's state for "route set, autonomous mode not available"; the
availability is `/autoware/modes/autonomous` of the diagnostic graph, which ANDs
`/autoware/perception` and `/autoware/planning` among others.

## Hypotheses

`carla_simulator.launch.xml` gated the whole perception component on `perception`. With it
false, acb's ground-truth publisher replaces `object_recognition/objects`, but nothing
replaced the rest of what that component publishes: the obstacle pointcloud, the occupancy
grid, and traffic light fusion/arbiter (which carry acb's V2X signals to planning).

## Reproduction (2026-10-18)

Same command, same scenario: `AutowareError ... The current Autoware state is PLANNING`
(50 nodes, 120 composables against the default stack's 62 / 127). From that stack's
`play_log/ego/<run>/`:

- `behavior_path_planner`: `waiting for occupancy_grid`, 23 times, and nothing else -- no
  path, so no trajectory.
- `diagnostics.csv`, every sample of the run: `topic_state_monitor_obstacle_segmentation_pointcloud`
  ERROR (in the diagnostic graph's `/autoware/perception`), `..._traffic_light_recognition_traffic_signals`
  ERROR (component state monitor), `..._scenario_planning_trajectory` ERROR (`/autoware/planning`);
  `..._object_recognition_objects` OK (acb's ground truth).
- On the default stack, `/perception/obstacle_segmentation/pointcloud` is read by
  `behavior_velocity_planner`, `motion_velocity_planner`, `planning_validator`, AEB,
  `collision_detector`, `costmap_generator` and the topic monitor;
  `/perception/occupancy_grid_map/map` by `behavior_path_planner`, both velocity planners;
  `/perception/traffic_light_recognition/traffic_signals` (published by the crosswalk
  estimator after fusion and the arbiter) by the planners and the monitor.

So three hypotheses held at once: the pointcloud fails the diagnostic graph, the missing
occupancy grid stops `behavior_path_planner` (hence no trajectory, failing planning), and
the signals never reach planning.

## Fix

`perception:=false` now means "no object recognition": the perception component is launched
either way, and only detection, tracking and prediction (or upstream's empty-objects
publisher) are left out. Obstacle segmentation, the occupancy grid and traffic light fusion
keep running. `publish_ground_truth_objects:=true` turns object recognition off by itself
too, so ground truth never shares `object_recognition/objects` with the LiDAR pipeline.

upstream's `perception.launch.xml` has no switch for object recognition alone, so
`scripts/sync_perception_component.sh` now also generates acb_launch's copy of it with the
two object-recognition groups gated on an `object_recognition` argument (default true; the
comparison is case-blind because the value carla_simulator passes is an `$(eval)`, which
yields `True`/`False` -- a first version compared against `'true'` and silently dropped
object recognition from the default stack), and the component copy includes it.
`carla_perception:=false` still hands perception back to autoware.launch.xml unchanged.

Guarded by `src/acb_bridge/tests/launch_profile.rs` (written first, seen failing),
including a check that no comment in these launch files contains `--`, which play_launch's
parser rejects.

## Verification (2026-10-18)

- `GROUND_TRUTH_OBJECTS=true EGO_LAUNCH_ARGS=perception:=false just ego-av`: 56 nodes, 124
  composables. On it `town01_entity_bodies` PASS (40 s), `town01_ego_drive` PASS (46 s),
  `town01_engage_state` PASS (46 s), `town01_traffic_light` PASS (186 s: the signals reach
  planning through fusion). The four monitors above are OK while a vehicle exists (ERROR
  only between episodes), and `behavior_path_planner` logged no `waiting for occupancy_grid`.
- Default stack: 62 nodes, 127 composables, as before; the six starters, see carla-scenario-bridge
  roadmap 032's follow-up for the numbers.
