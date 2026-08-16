# 010 — `/vehicle/status/actuation_status` is never published

**Severity**: Low
**Component**: `src/acb_bridge/src/vehicle_control.rs`
**Status**: Fixed

## What is wrong

`autoware.rs` carried a `topic_actuation_status` string
(`"vehicle/status/actuation_status"`) that nothing published to, and it subscribed to
`control/command/actuation_cmd` into a dead `ArcSwap`. The topic — a
`tier4_vehicle_msgs/ActuationStatusStamped` of normalized accel, brake and steer — has no
publisher at all.

## Why it matters

It is the feedback half of the `raw_vehicle_cmd_converter` loop. Autoware's
`accel_brake_map_calibrator` and the actuation-based control path both read it; without
it, a stack configured for actuation control has an open loop and the calibrator has no
input. Nothing in the *current* CARLA profile uses actuation control — the bridge
subscribes to `autoware_control_msgs/Control` and converts to throttle/brake itself — so
today the missing topic costs nothing.

It is a two-line publisher and CARLA already holds exactly these three normalized values
in `VehicleControl`, so leaving the interface half-implemented is not worth the saving.

## Fix

`publish_status` publishes `ActuationStatusStamped` from the applied `VehicleControl`:
`accel_status = throttle`, `brake_status = brake`, `steer_status = steer`, all in CARLA's
own `[0,1]` / `[-1,1]` normalization, which is what the message is defined in.

`vehicle_bridge.rs` — the 472-line `#[allow(dead_code)]` alternative control module that
also implements this — remains unused and is a separate decision (gap 2 in
`docs/roadmap/9-gap-analysis.md`).
