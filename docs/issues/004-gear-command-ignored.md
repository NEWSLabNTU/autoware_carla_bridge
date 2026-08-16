# 004 — Gear command ignored; GearReport hardcoded to DRIVE

**Severity**: High
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `src/acb_bridge/src/autoware.rs`
**Status**: Fixed

## What is wrong

Autoware publishes `/control/command/gear_cmd`
(`autoware_vehicle_msgs/GearCommand`) and expects the vehicle interface to apply it and
report the resulting gear on `/vehicle/status/gear_status`. The bridge did neither:

- `autoware.rs` subscribed to `control/command/gear_cmd`, stored the message in an
  `ArcSwap`, and marked the accessor `#[allow(dead_code)]`. Nothing ever read it.
- `vehicle_control.rs` published `GearReport { report: DRIVE }` unconditionally.
- Reverse was inferred instead from the sign of the commanded velocity:
  `if cmd.longitudinal.velocity < -0.01 { control.reverse = true }`.

## Why it matters

Any maneuver that needs reverse — `start_planner`'s pull-out, `goal_planner`'s
pull-over, and the parking scenarios in general — is driven by the gear command, not by
the velocity sign. Autoware also gates on the *report*: `vehicle_cmd_gate` and the
operation-mode logic compare commanded gear against reported gear, and a report frozen at
DRIVE means the shift never appears to complete.

The velocity-sign heuristic happens to agree with the gear command in normal reverse
driving, so forward-only scenarios never exposed it.

## Fix

The gear command subscription moves from `autoware.rs` (where it was dead) into
`vehicle_control.rs`, alongside the control command it belongs with:

- `GearCommand::REVERSE` / `REVERSE_2` set `VehicleControl.reverse`.
- `GearCommand::PARK` engages the handbrake and holds the vehicle.
- `GearCommand::NEUTRAL` releases throttle and brake without engaging reverse.
- `GearReport.report` echoes the applied gear, defaulting to DRIVE before the first
  command arrives — Autoware's own convention for a vehicle that boots in drive.

The velocity-sign heuristic is kept as a fallback for when no gear command has been
received yet, so a stack without `vehicle_cmd_gate` still reverses.

## Related

The dead `_sub_gear_cmd` and `current_gear_cmd` fields are removed from `autoware.rs`.
Leaving a second subscriber on the same topic is harmless on the wire but is exactly the
kind of "wired but unused" trap the `/clock` regression came from.
