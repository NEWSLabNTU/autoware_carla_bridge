# 025 — A long-lived ego stack steers differently from a fresh one

**Severity**: Medium
**Component**: `src/acb_launch/launch/carla_simulator.launch.xml`,
`src/acb_launch/config/control/trajectory_follower/lateral/mpc.param.yaml`,
`src/acb_launch/launch/components/tier4_control_component.launch.xml`,
`scripts/sync_control_component.sh`
**Status**: Fixed (MPC steering offset removal off in the CARLA profile)

## What happened

carla-scenario-bridge roadmap 032, finding N4 (2026-10-16): `rear_contact` runs r3-r6 came
after ~75 min of one ego stack and failed; after the shove the ego's yaw differed (min yaw
168.0-168.3 deg against 169.8-172.6 deg in passing runs) and it reached the lamp pole.
A restarted ego stack passed r7-r9. Nothing was traced; 032 recorded it as "yaw drift of a
long-lived ego stack, acb/Autoware".

## Hypotheses, in the order they were tested

The stack keeps running across scenarios; every episode respawns the ego, re-attaches
acb's sensors and re-initialises localization at the start pose. So anything that differs
between minute 5 and minute 75 is state that survives those three resets.

1. **Localization** drifts from CARLA's heading (NDT/EKF, EKF yaw bias, a gyro bias).
2. **acb** keeps per-episode state across `VehicleLost` / respawn (IMU, odometry, steering
   dynamics, clocks).
3. **Control** keeps state: Autoware's controller is never restarted. Candidate found by
   reading Autoware 0.48.0 (`autoware_mpc_lateral_controller`, `steering_offset.cpp`):
   MPC's automatic steering offset removal, on in autoware_launch's `mpc.param.yaml`
   (`average_num: 1000`, `update_vel_threshold: 5.56`, `update_steer_threshold: 0.035`,
   `steering_offset_limit: 0.02`).

## Measurement

A probe in the ego domain (carla-scenario-bridge session scratch, data under
`/mnt/wd/aeon/csb_yaw_drift/`) logged every 0.1 s: `/localization/kinematic_state` against
acb's `/carla/ground_truth/odom` (paired by stamp), MPC's
`/control/trajectory_follower/controller_node_exe/output/estimated_steer_offset`, the EKF's
`estimated_yaw_bias`, steering report and command. One ego stack, 2026-10-18.

**Localization does not drift.** Over four rounds of the six starters (40 min, 24 episodes)
the moving ego's heading error, median per run, stayed 0.014-0.077 deg and its lateral
error -0.04..+0.005 m, with no trend across episodes; the EKF's yaw bias is re-initialised
every episode (per-run max 0.0006-0.014 rad, no growth). Hypotheses 1 and 2 are out: acb's
IMU orientation, gyro and steering conversion are stateless (the steering dynamics filter is
off by default), and an acb drift would show up in this error.

**MPC's steering offset grows with stack age and never resets.** Each cycle with |v| >
5.56 m/s and |steering| < 0.035 rad pushes `steer_reported - atan(wz * L / v)` into a
1000-sample window that starts as 1000 zeros; MPC adds the average (clamped to 0.02 rad) to
every command and subtracts it from the steering it believes it has. On CARLA almost the only
such cycles are transients: in the starters, the shoved ego in `rear_contact` (v 5.7-7.3,
steering +0.010..+0.021 while the collision yaws it at -0.03..-0.095 rad/s: samples
+0.035..+0.054) and one frame of `simulator_autopilot`. The estimate after each round:
0.00058, 0.00110, 0.00157 rad -- monotone, from collision samples, not from any steering
bias (CARLA's steering has none, and acb reports the commanded angle).

**The offset changes the post-shove yaw.** `rear_contact` with its pre-032 3 s exit delay
(the N4 configuration), 14 runs on that aged stack and 14 on a fresh one with the fix:

| stack | offset over the runs (rad) | min yaw after the shove, deg | trend |
|---|---|---|---|
| aged, offset removal on | 0.0027 -> 0.0111 (+0.0006/run) | mean 173.19, sd 0.94, 171.79..174.56 | +0.164 deg/run; 252 deg per rad of offset, r = 0.74 |
| fixed, offset removal off | 0 throughout | mean 171.60, sd 0.80, 170.21..173.47 | +0.022 deg/run |

At 252 deg/rad, an offset of -0.012 rad moves the post-shove yaw by -3 deg: from the fresh
stack's ~171.6 to ~168.6, which is N4's failing band (168.0-168.3). This session's
starters drove the estimate positive; the sign 032's stack reached after 75 min of other
scenarios (gap and bench scenarios, AWF use cases) was not recorded and cannot be recovered,
so the direction in N4 is inferred, not measured. The mechanism, its growth with age and its
effect on exactly this manoeuvre are measured.

## Root cause

Autoware's MPC steering offset estimator (an Autoware feature, working as designed for a real
vehicle whose steering sensor has a mechanical offset) runs on CARLA where there is no offset
to learn. It learns the stack's history instead -- collisions and transients -- keeps it for
the controller's lifetime across every episode, and applies it to every command. A
long-lived stack therefore steers differently from a fresh one, by up to 0.02 rad of tire
angle.

## Fix

The CARLA profile turns `steering_offset.enable_auto_steering_offset_removal` off.
upstream hard-codes the lateral parameter file inside `tier4_control_component.launch.xml`
and MPC reads the flag only at construction, so neither a launch argument nor `ros2 param
set` reaches it. As with perception (`sync_perception_component.sh`), acb_launch carries
generated copies: `scripts/sync_control_component.sh` writes the component with the lateral
parameter file as an argument, and `mpc.param.yaml` identical to upstream's but for the flag.
`carla_simulator.launch.xml` launches control from that copy (`carla_control`, default
true; `carla_control:=false` restores upstream's control through autoware.launch.xml), the
same pattern as `carla_localization`, `carla_perception`, `carla_system`. Run
`scripts/sync_control_component.sh --check` after an Autoware upgrade.

Guarded by `src/acb_bridge/tests/launch_profile.rs` (written first, seen failing): the flag
is false, the copy equals upstream otherwise, and the launch file wires it in.

## Verification

- Live: `ros2 param get /control/trajectory_follower/controller_node_exe
  steering_offset.enable_auto_steering_offset_removal` -> False; the estimated offset reads
  0 for the stack's whole life (table above); the default stack's node and composable counts
  are unchanged (62 nodes, 127 composables), i.e. control is launched once.
- Long run on the final build (with issue 026's fix), one default ego stack, 2026-10-18:
  the six starters five times over 43 min, **30/30 PASS**, then `rear_contact` with the 3 s
  exit delay 8 times at stack age 44-50 min, **8/8 PASS**. The estimated offset read 0.00000
  in every run. Localization stayed where it was on the unfixed stack (moving heading error
  0.012-0.149 deg median per run, lateral -0.041..+0.005 m, no trend), and the post-shove
  min yaw was 171.51..173.85 deg (mean 172.64, sd 0.84): the fresh stack's band, with no
  upward or downward trend over the stack's age (-0.27 deg/run across the 8, noise).
