# 021 — The emergency topic has no subscriber, and should not have one yet

**Severity**: Medium
**Component**: `src/acb_bridge/src/vehicle_control.rs`
**Status**: Handler implemented and verified, but **off by default** — on this stack the flag
is not an actionable emergency

## What was observed

On a running stack, with the ego driving:

```
$ ros2 topic info /control/command/emergency_cmd
Type: tier4_vehicle_msgs/msg/VehicleEmergencyStamped
Publisher count: 1
Subscription count: 0
```

`vehicle_cmd_gate` publishes an emergency state and nothing consumes it. That looks like a
plain safety gap: Autoware decides the vehicle must stop, and the vehicle never hears.

A handler was written on that reading. It works, and `scripts/test_emergency.py` demonstrates
it: declared mid-drive, the ego went from 2.10 m/s to a standstill inside a second, held with
the handbrake for eight, then released cleanly and drove away at 4.18 m/s.

## Why it is off anyway

Before leaving it enabled, how often the flag is actually raised was measured. It is raised
constantly:

```
  stopped (<0.3 m/s)   n=1814  emergency true 1605 (88.5%)
  creeping             n=  45  emergency true   28 (62.2%)
  driving (>1 m/s)     n=  69  emergency true   20 (29.0%)
```

And what Autoware commands while the flag is set settles it. Pairing each control command with
the emergency state current at the time:

```
  emergency=True   n= 46  accel median +0.430  min -0.444  max +1.177
  emergency=False  n=347  accel median +0.444  min -1.666  max +1.906
```

**While the emergency flag is set, Autoware is commanding acceleration**, and commanding it no
differently from when the flag is clear. This is not a stack asking the vehicle to stop. Acting
on the flag means slamming full brakes against a normal control command in 29% of driving
samples, which is a worse behaviour than ignoring it — and it is what shipping this handler
enabled did, briefly, before this measurement was taken.

`vehicle_cmd_gate` is configured with `use_emergency_handling: true` and
`emergency_acceleration: -2.4`. A stack in a genuine emergency would therefore be commanding
about -2.4 m/s^2 on `control_cmd`, not +0.43. Whatever this flag is tracking here -- a missing
system-emergency heartbeat, an unengaged operation mode -- it is not that.

## What is in the tree

The subscription and the override stay, behind `honor_emergency_cmd`, default `false`. The
bridge logs at startup that it is not acting on the topic, and why. When set:

- full brake, no throttle, handbrake once stopped (CARLA's automatic transmission idle-creeps
  at zero throttle, and an emergency stop that creeps is not one)
- hazard lights on, which is what a real vehicle does and what makes the state visible
- decided before any pedal-map work, since the emergency travels on its own topic precisely so
  that it holds when the control command cannot be trusted

## What would justify turning it on

A stack where the flag correlates with an actual emergency: `control_cmd` carrying roughly
`emergency_acceleration` while it is set, and the flag clear during ordinary driving. The two
measurements above are the test — re-run them before enabling it, rather than assuming the
topic means what its name says.

## Not done: control mode switching

`/vehicle/status/control_mode` is reported as a constant `AUTONOMOUS`.
`autoware_vehicle_msgs/srv/ControlModeCommand` exists, and `autoware_command_mode_switcher` and
`autoware_operation_mode_transition_manager` reference it, but no such service appears in this
stack's graph. Nothing is calling one, so serving one would be writing an interface against a
caller that does not exist here.
