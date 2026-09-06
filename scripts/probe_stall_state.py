#!/usr/bin/env python3
"""Catch a managed ego that spawns and never drives, and say what state it was in.

The failure recurs about one run in three and always scores the same: peak 0.00 m/s,
travelled 0.0 m, and cross-track 235.545 m. That number is not noise -- the distance from
the ego's spawn at (320.0, -55.9) to the goal at (88.4, -100.0) is 235.8 m -- so the ego is
sitting at its spawn while the trajectory it is being scored against lies at the goal. A
route that belongs to some other run, in other words.

What is not known is which of these is true when it happens:
  * routing never accepted a new route, and what is published is the previous one
  * a route was set but planning produced no trajectory the vehicle could follow
  * the operation mode never reached AUTONOMOUS, so nothing was ever asked to move

So watch, and when the ego has been present but stationary for `--stall` seconds, dump the
routing state, the operation mode, the trajectory's extent, and the pose. Then keep
watching, because the interesting question is whether any of it changes afterwards.

    ROS_DOMAIN_ID=1 stall_watch.py [--stall 45] [--minutes 8]
"""

import argparse
import math
import sys
import time

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ROUTE = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}
MODE = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stall", type=float, default=45.0)
    ap.add_argument("--minutes", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("stall_watch_%d" % int(time.time()))
    st = {"speed": None, "pose": None, "route": None, "mode": None,
          "traj": [], "moved": False, "still_since": None, "dumped": 0}

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_speed(m):
        st["speed"] = m.longitudinal_velocity
        if abs(m.longitudinal_velocity) > 0.5:
            st["moved"] = True
            st["still_since"] = None
        elif st["still_since"] is None:
            st["still_since"] = time.monotonic()

    def on_odom(m):
        p = m.pose.pose.position
        st["pose"] = (p.x, p.y)

    def on_traj(m):
        st["traj"] = [(p.pose.position.x, p.pose.position.y) for p in m.points]

    node.create_subscription(VelocityReport, "/vehicle/status/velocity_status", on_speed, 10)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 10)
    node.create_subscription(Trajectory, "/planning/scenario_planning/trajectory", on_traj, 1)
    node.create_subscription(RouteState, "/api/routing/state",
                             lambda m: st.__setitem__("route", m.state), latched)
    node.create_subscription(OperationModeState, "/api/operation_mode/state",
                             lambda m: st.__setitem__("mode", m.mode), latched)

    end = time.monotonic() + args.minutes * 60
    last_dump = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        stalled = (st["still_since"] is not None
                   and now - st["still_since"] > args.stall
                   and st["speed"] is not None)
        if stalled and now - last_dump > 30.0:
            last_dump = now
            st["dumped"] += 1
            traj = st["traj"]
            pose = st["pose"]
            head = traj[0] if traj else None
            tail = traj[-1] if traj else None
            d_head = math.dist(pose, head) if (pose and head) else None
            d_tail = math.dist(pose, tail) if (pose and tail) else None
            print(f"STALL #{st['dumped']} after {now - st['still_since']:.0f}s stationary "
                  f"(ego has {'' if st['moved'] else 'NEVER '}moved this run)", flush=True)
            print(f"STALL   pose={pose} speed={st['speed']:.3f}", flush=True)
            print(f"STALL   route={ROUTE.get(st['route'], st['route'])} "
                  f"mode={MODE.get(st['mode'], st['mode'])}", flush=True)
            print(f"STALL   trajectory: {len(traj)} points, first={head}, last={tail}",
                  flush=True)
            print(f"STALL   distance pose->trajectory start {d_head if d_head is None else round(d_head, 1)} m, "
                  f"pose->trajectory end {d_tail if d_tail is None else round(d_tail, 1)} m",
                  flush=True)
    if st["dumped"] == 0:
        print(f"STALL none: ego {'moved' if st['moved'] else 'was never seen moving'}; "
              f"no stall longer than {args.stall:.0f}s", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
