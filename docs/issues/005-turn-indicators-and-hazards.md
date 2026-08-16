# 005 — Turn indicators and hazard lights go nowhere

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`, `src/acb_bridge/src/autoware.rs`
**Status**: Fixed

## What is wrong

Autoware's vehicle interface contract has four topics for lights:

| Direction | Topic | Type |
|---|---|---|
| command | `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/TurnIndicatorsCommand` |
| command | `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/HazardLightsCommand` |
| status | `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/TurnIndicatorsReport` |
| status | `/vehicle/status/hazard_lights_status` | `autoware_vehicle_msgs/HazardLightsReport` |

The bridge subscribed to both commands in `autoware.rs`, stored them in `ArcSwap` fields
marked `#[allow(dead_code)]`, and published neither status. CARLA's blinkers were never
touched, and Autoware never saw its own indicator request acknowledged.

## Why it matters

Two things depend on it:

- **Scenario fidelity.** A lane change or turn is invisible in CARLA's rendering and to
  any other client watching the actor's light state, including scenario checks that
  assert an indicator was used before a maneuver.
- **The status topics are part of the interface.** `autoware_vehicle_cmd_gate` and the
  ADAPI vehicle-status endpoints read them. A missing publisher is not an error anywhere
  — the topic simply has no data, and everything downstream reports "unknown" forever.

CARLA supports both directly: `Vehicle::set_light_state` with
`VehicleLightState::LEFT_BLINKER` / `RIGHT_BLINKER`, exposed by carla-rust since
`0.12.0`.

## Fix

Both subscriptions move into `vehicle_control.rs` and are applied to CARLA's light state
on each command, together with brake and reverse lights derived from the control command
itself (CARLA does not drive those automatically for an externally controlled vehicle).
The two status topics are published every cycle from the state the bridge last applied.

Mapping:

| Autoware | CARLA |
|---|---|
| `TurnIndicatorsCommand::ENABLE_LEFT` | `LEFT_BLINKER` |
| `TurnIndicatorsCommand::ENABLE_RIGHT` | `RIGHT_BLINKER` |
| `TurnIndicatorsCommand::DISABLE` | neither |
| `HazardLightsCommand::ENABLE` | `LEFT_BLINKER │ RIGHT_BLINKER` |
| brake > 0 | `BRAKE` |
| reverse | `REVERSE` |

`NO_COMMAND` (0) leaves the previous state alone, per the message's own semantics.

Hazards win over indicators while active, which is what a real vehicle does and what
`HazardLightsReport` consumers expect.
