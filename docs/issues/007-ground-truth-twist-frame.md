# 007 — Ground-truth odometry twist is world-frame under `base_link`

**Severity**: Low
**Component**: `src/acb_bridge/src/autoware.rs`, `tick` and `publish_ground_truth`
**Status**: Fixed

## What is wrong

`/carla/ground_truth/odom` is a `nav_msgs/Odometry` with
`header.frame_id: "map"` and `child_frame_id: "base_link"`. The ROS convention for
`Odometry` is that the **pose is in `header.frame_id` and the twist is in
`child_frame_id`** — that is the whole reason the message carries two frames.

The bridge filled the twist from CARLA's world-frame linear and angular velocity, merely
Y-flipped into ROS handedness. It is a map-frame twist published in a `base_link` field.

## Why it matters

Only for consumers of the debug topic — the same-named field on
`/localization/kinematic_state` is produced by Autoware's EKF, not by this bridge, unless
`publish_direct_localization` is on. But this topic exists precisely so that localization
error can be measured against truth, and comparing a body-frame estimate against a
world-frame truth silently reports a large error that is entirely an artifact of the
frame mismatch.

With `publish_direct_localization: true` the same message goes to
`/localization/kinematic_state`, where control and planning read it — so there the frame
error is real.

## Fix

Rotate both linear and angular velocity into the vehicle body frame before publishing,
using CARLA's own `Rotation::inverse_rotate_vector`, then apply the Y-flip. The pose is
untouched: it was already correct in `map`.
