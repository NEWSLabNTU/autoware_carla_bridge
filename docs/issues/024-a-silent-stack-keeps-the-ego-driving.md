# 024 — A silent planner or controller leaves the ego driving

**Severity**: High
**Component**: `src/acb_launch/config/system/component_state_monitor/topics.yaml`,
`src/acb_bridge/src/vehicle_control.rs`, `src/acb_bridge/src/command_watchdog.rs`
**Status**: Fixed (monitor timeouts restored for planning and control; command watchdog)

## What happened

carla-scenario-bridge run `i3 ov_2` (2026-10-17, incident 3 "red light", default clustering
stack, roadmap 042 there): `behavior_path_planner` aborted (autoware_universe#12460) 100 ms
after the scenario set the route, and play_launch reloaded it. On that loaded host the reload
took 20 s. During those 20 s the ego engaged, drove off, accelerated at a steady ~0.95 m/s^2
to 16.7 m/s on a 15 km/h route, ran the red light and hit the guardrail at the T junction
(contact impulse 15217, -18 m/s^2).

Times are wall clock (the ROS logs and `play_log/ego/2026-10-17_07-30-36/diagnostics.csv`);
the simulation ran at ~1x, sim = wall - 1792194834.8 s.

| wall (s) | sim (s) | event | evidence |
|---|---|---|---|
| 838.99 | 4.2 | route set (`set_waypoint_route`) | mission_planner log |
| 839.09 | 4.3 | `behavior_path_planner` killed by SIGABRT | container log, play_launch.log |
| 839.18 | 4.4 | agent: `engaged` -- autonomous mode was still *available* | acb_agent log |
| 840.03 | 5.2 | MRM `NORMAL -> MRM_OPERATING`, cancelled 100 ms later; repeats every 1 s for the whole window | mrm_handler log; vehicle_cmd_gate `Emergency!` at the same instants |
| 840.0 | 5.2 | `controller_node_exe` `control_state` 2 (STOPPED) -> 0 (DRIVE) | diagnostics.csv |
| 842.3 | 7.5 | the ego starts moving | CARLA trace |
| 843.1 | 8.3 | `control_validation_over_velocity` ERROR, continuously from here; `control_validation_acceleration_error` stays OK, i.e. the ego accelerates *as commanded* | diagnostics.csv |
| 841.9 ... 857.2 | | `Subscribed trajectory is timed out` every 3 s | operation_mode_transition_manager log |
| 840 ... 862 | | `/control/command/control_cmd` and `/control/trajectory_follower/control_cmd` at 20 Hz, both monitors OK | diagnostics.csv |
| 858.8 | 24.0 | peak speed 16.7 m/s | CARLA trace |
| 859.03 | 24.2 | `behavior_path_planner` constructed again | container log |
| 859.5 | 24.7 | first new trajectory; `topic_state_monitor_scenario_planning_trajectory` reaches only WARN ("timeout 30.00 [s]"); planning_validator "deceleration is too high" | diagnostics.csv, planning_validator log |
| 861.6 | 26.8 | contact with the guardrail | CARLA trace |

So the commands never stopped. Autoware's `trajectory_follower` kept tracking the last
trajectory it had -- the previous episode's, which ended near (396, -166) heading north, some 260 m from
the new start and at right angles to it -- in DRIVE, and kept commanding acceleration; `vehicle_cmd_gate`
passed it through (its nominal limit is 5 m/s^2); the bridge applied it, and
`control_validator` confirms the ego accelerated as commanded. A steady ~1 m/s^2 is the
velocity smoother's `max_acc`, consistent with the PID's feed-forward reading the planned
acceleration of one frozen trajectory point while the ego drives away from it.

## Why nothing stopped it

1. **`trajectory_follower` has no input timeout.** `Controller::processData` checks that a
   trajectory, odometry and steering have *ever* arrived, not how old they are
   (autoware_universe 0.48 `controller_node.cpp`); `timeout_thr_sec` applies to the
   controller's own outputs. Autoware leaves trajectory staleness to the system monitors.
2. **The monitor that would have caught it was set to 30 s.** `topic_state_monitor_
   scenario_planning_trajectory` is what turns a silent planner into an ERROR in
   `/autoware/planning`, autonomous mode unavailable, and a sustained MRM emergency stop.
   6fefaa6 relaxed every topic in this file from 3 s to 30 s (and the rate floors from 1 Hz
   to 0.1 Hz) so that perception and localization topics sagging on a contended host would
   not keep autonomous mode unavailable. The planning trajectory and both control commands
   went along with them. Their monitors run on the node clock, i.e. simulation time, so the
   sub-realtime argument never applied to them: a paused simulation does not age them.
3. **What did fire only flickered.** With no trajectory arriving, `planning_validator`'s
   diagnostic updater re-publishes its last OK diagnosis once a second, and the diagnostic
   graph's 1 s unit timeout races that period: the planning unit goes stale for ~100 ms
   every second. That is the 1 Hz, 100 ms MRM pulse in the logs: `mrm_handler` operates the
   emergency stop and cancels it as soon as the unit comes back, and the gate brakes for
   100 ms out of every 1000. Reproduced exactly on a healthy stack by stopping the planner
   (below).

The bridge had a gap of its own, which this run did not exercise but a slightly different
one would: it applies `/control/command/control_cmd` when one arrives and does nothing
otherwise, and CARLA keeps the last `VehicleControl` applied to an actor until another one
replaces it. If `vehicle_cmd_gate` itself dies, stops or is reloaded, the ego keeps the last
throttle for as long as the simulation runs. An MRM cannot help there -- it reaches the
vehicle through the very gate that is gone. A drive-by-wire vehicle interface times its
commands out (pacmod_interface: `command_timeout_ms`, 1000 by default); this one did not.

## Fix

1. `topics.yaml`: `scenario_planning_trajectory` warn 2 Hz / error 0.5 Hz / timeout 2.5 s;
   `trajectory_follower_control_cmd` and `control_command_control_cmd` warn 5 Hz / error
   1 Hz / timeout 2.5 s -- autoware_launch's own CARLA profile
   (`config/simulator/carla/component_state_monitor/topics.yaml`), except the trajectory
   error floor, kept at 0.5 Hz because the trajectory was measured as low as 1.6 Hz (sim
   time) on a healthy, loaded stack. Map, localization and perception stay relaxed.
2. `acb_bridge` command watchdog (`command_watchdog.rs`, `VehicleControlBridge::
   watch_commands`): past `control_cmd_timeout_s` (default 1.0) of simulation time without
   a `/control/command/control_cmd`, release the throttle, brake at 2.5 m/s^2 (Autoware's
   MRM emergency stop target) through the pedal maps, hold the wheel where it was, turn the
   hazard lights on, and hold with the handbrake once stopped. The next command hands the
   vehicle back and turns off only the hazards the watchdog turned on. Ages are taken on
   the CARLA frame clock on both sides: a command is stamped with the frame being processed
   when the executor drains it, the check runs after the drain, so a paused simulation and a
   stalled bridge loop age nothing. MANUAL disarms it. 0 disables.

## Verified

On a healthy stack (town01_i3, 2026-10-17), stopping processes of a stack started for the
purpose: `SIGSTOP` to `behavior_path_planner`'s isolated component process stands in for its
20 s reload (a `SIGABRT` reloads in 3 s on an idle host), `SIGSTOP` to `vehicle_cmd_gate`'s
for a dead gate. A recorder logged the control, trajectory, MRM and operation-mode topics.

| case | before | after |
|---|---|---|
| planner stopped at `set_goal`, 30 s | engaged on the stale trajectory; MRM flickered 1 Hz / 100 ms for all 30 s (54 transitions), never a sustained stop | sustained MRM 2.6 s after the planner stopped (7 transitions in 33 s) |
| planner stopped 3 s into the drive, 40 s | no MRM; followed the frozen trajectory to its end, stopped there at -3.4 m/s^2 ~10 s later | MRM emergency stop at 2.6 s: -2.5 m/s^2, 3.4 m/s to rest by 5.5 s |
| gate stopped 15 s while starting | last command (+0.49 m/s^2) held: 0 -> 3.55 m/s with no command | watchdog trip at 1.0 s, held at rest, handed back on SIGCONT |
| gate stopped 15 s at 4.1 m/s | (held throttle, as above) | watchdog trip at 1.0 s, 4.1 m/s to rest in ~2 s, handed back on SIGCONT (16.2 s logged) |

The `i3 ov_2` acceleration itself -- the PID driving toward a far, stale trajectory -- depends
on what the previous episode left behind and did not recur in three tries with other previous
trajectories; the fix does not depend on it. Unit tests: `command_watchdog::tests` (9),
`vehicle_control::tests::watchdog_stop_*` (2). Runs and recorder: carla-scenario-bridge
`docs/roadmap/042-autoware-overlay.md`, "Follow-up".
