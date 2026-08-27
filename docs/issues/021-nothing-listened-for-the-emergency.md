# 021 — Autoware declares an emergency and the vehicle never hears it

**Severity**: High
**Component**: `src/acb_bridge/src/vehicle_control.rs`
**Status**: Fixed; verified by declaring one mid-drive

## Symptom

On a running stack, with the ego driving:

```
$ ros2 topic info /control/command/emergency_cmd
Type: tier4_vehicle_msgs/msg/VehicleEmergencyStamped
Publisher count: 1
Subscription count: 0
```

`vehicle_cmd_gate` publishes the emergency state and **nothing consumes it**. Autoware can
decide the vehicle must stop immediately -- an MRM, an autonomous-emergency-braking trigger, a
failed validator -- and the vehicle carries on with whatever the ordinary control command last
said.

Nothing reports this. The topic exists, it is published, and the count of subscribers is the
only place the gap is visible.

## Why the ordinary control command is not enough

The emergency travels on its own topic precisely so that it still means something when the
control command cannot be trusted. Deriving the emergency from the control command -- treating
a large deceleration request as one -- would reinstate exactly the coupling the separate
channel exists to avoid.

## Fix

`VehicleControlBridge` subscribes to `/control/command/emergency_cmd`. While the flag is set
the control command is overridden: full brake, no throttle, the handbrake once stopped, since
CARLA's automatic transmission idle-creeps at zero throttle and an emergency stop that creeps
is not one. Hazard lights come on with it, which is what a real vehicle does and what makes the
state visible in the simulation.

The decision is taken before any pedal-map work, and the flag defaults to false so that a bench
setup without the topic is not held stationary.

## Verified

Declared mid-drive on a live scenario, reading the applied control back from CARLA:

```
before: speed 2.02 m/s
  t=0.0s speed  2.10  throttle 0.27 brake 0.00 handbrake False
  t=1.0s speed  0.00  throttle 0.00 brake 1.00 handbrake True
  ...
  t=8.0s speed  0.00  throttle 0.00 brake 1.00 handbrake True
```

Stopped inside a second and held for the duration. Releasing it returns control:

```
Autoware declared an emergency; braking fully until it clears
Emergency cleared; returning to commanded control
```

after which the ego drove away at 4.18 m/s. `scripts/test_emergency.py` reproduces it.

## Not done: control mode switching

`/vehicle/status/control_mode` is still reported as a constant `AUTONOMOUS`, and there is no
handler for a mode-change request. `autoware_vehicle_msgs/srv/ControlModeCommand` exists and
`autoware_command_mode_switcher` and `autoware_operation_mode_transition_manager` reference it,
but no such service appears in this stack's graph -- nothing is calling one, so serving one
would be writing an interface against a caller that does not exist here. It should be built
when something asks for it, against that caller.
