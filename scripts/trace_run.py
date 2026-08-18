#!/usr/bin/env python3
"""Trace a whole scenario run: Autoware state and CARLA ground truth, once a second.

Started before the scenario and left running past its end, so the moment the ego stops is
inside the trace rather than guessed at.
"""
import sys, time
import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
from autoware_control_msgs.msg import Control

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 260
OPMODE = {0: "UNK", 1: "STOP", 2: "AUTO", 3: "LOCAL", 4: "REMOTE"}
ROUTE = {0: "UNK", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}

class T(Node):
    def __init__(self):
        super().__init__("trace_run")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.d = {}
        self.create_subscription(Clock, "/clock",
            lambda m: self.d.__setitem__("clock", m.clock.sec + m.clock.nanosec/1e9), 1)
        self.create_subscription(OperationModeState, "/api/operation_mode/state",
            lambda m: self.d.__setitem__("mode", m), latched)
        self.create_subscription(RouteState, "/api/routing/state",
            lambda m: self.d.__setitem__("route", m.state), latched)
        self.create_subscription(Odometry, "/localization/kinematic_state",
            lambda m: self.d.__setitem__("pose", m.pose.pose.position), 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
            lambda m: self.d.__setitem__("vel", m.longitudinal_velocity), 1)
        self.create_subscription(SteeringReport, "/vehicle/status/steering_status",
            lambda m: self.d.__setitem__("steer", m.steering_tire_angle), 1)
        self.create_subscription(Control, "/control/command/control_cmd",
            lambda m: self.d.__setitem__("cmd", m), 1)

client = carla.Client("localhost", 2000); client.set_timeout(10.0)

def carla_state():
    try:
        w = client.get_world()
        out = []
        for a in w.get_actors().filter("vehicle.*"):
            t = a.get_transform().location
            v = a.get_velocity()
            spd = (v.x**2 + v.y**2 + v.z**2) ** 0.5
            out.append((a.attributes.get("role_name", "?"), t.x, t.y, spd))
        return out
    except Exception:
        return []

rclpy.init()
n = T()
t0 = time.time()
last = 0
print(f"{'t':>4} {'clock':>12} {'mode':>5} {'ok':>5} {'route':>7} {'ax':>7} {'ay':>7} "
      f"{'vel':>6} {'steer':>6} {'cmd_a':>6} | CARLA vehicles")
while time.time() - t0 < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - last < 1.0:
        continue
    last = time.time()
    d = n.d
    m, p, c = d.get("mode"), d.get("pose"), d.get("cmd")
    cars = " ".join(f"{r}({x:.0f},{y:.0f},{s:.1f})" for r, x, y, s in carla_state())
    print(f"{int(time.time()-t0):4d} {d.get('clock', float('nan')):12.2f} "
          f"{OPMODE.get(getattr(m,'mode',None),'-'):>5} "
          f"{str(getattr(m,'is_autonomous_mode_available','-')):>5} "
          f"{ROUTE.get(d.get('route'),'-'):>7} "
          f"{(p.x if p else float('nan')):7.1f} {(p.y if p else float('nan')):7.1f} "
          f"{d.get('vel', float('nan')):6.2f} {d.get('steer', float('nan')):6.3f} "
          f"{getattr(getattr(c,'longitudinal',None),'acceleration',float('nan')):6.2f} | {cars}",
          flush=True)
rclpy.shutdown()
