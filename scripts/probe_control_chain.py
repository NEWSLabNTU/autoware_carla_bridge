"""Trace the control chain across a stall.

The failure this exists for: the ego accelerates to ~4 m/s, tracks its trajectory to a
few centimetres, travels a short distance, stops, and no node reports a fault. Each stage
below separates a different culprit:

  traj_v   what the planner asks for at the ego's own position -- 0 means planning decided
           to stop, and the question moves upstream to why
  tf_acc   what the trajectory follower asks for
  cmd_*    what leaves vehicle_cmd_gate, i.e. what the bridge is actually given
  speed    what the vehicle reports doing

A stall with traj_v > 0 and cmd_acc > 0 is a bridge or physics problem; with traj_v == 0
it is a planning decision; with traj_v > 0 but cmd_acc <= 0 the gate or a controller is
holding it.
"""
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_adapi_v1_msgs.msg import OperationModeState, MrmState

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0

MODE = {0: "?", 1: "STOP", 2: "AUTO", 3: "LOCAL", 4: "REMOTE"}
MRM_STATE = {0: "?", 1: "NORMAL", 2: "MRM_OPERATING", 3: "MRM_SUCCEEDED", 4: "MRM_FAILED"}
MRM_BEHAVIOR = {0: "?", 1: "NONE", 2: "PULL_OVER", 3: "COMFORT_STOP", 4: "EMERGENCY_STOP"}


class Probe(Node):
    def __init__(self):
        super().__init__('stall_probe')
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST)
        state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)
        self.pos = None; self.speed = None
        self.traj = []; self.traj_n = 0
        self.tf = None; self.cmd = None; self.cmd_n = 0
        self.mode = None; self.ctrl_enabled = None
        self.mrm = None
        self.create_subscription(Odometry, '/localization/kinematic_state', self._odom, best)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status',
                                 self._vel, best)
        self.create_subscription(Trajectory, '/planning/scenario_planning/trajectory',
                                 self._traj, 1)
        self.create_subscription(Control, '/control/trajectory_follower/control_cmd',
                                 self._tf, 1)
        self.create_subscription(Control, '/control/command/control_cmd', self._cmd, 1)
        self.create_subscription(OperationModeState, '/api/operation_mode/state',
                                 self._mode, state)
        # MRM is published VOLATILE here, so the transient-local profile used for the
        # other state topics silently matches nothing.
        self.create_subscription(MrmState, '/system/fail_safe/mrm_state', self._mrm, best)

    def _odom(self, m): self.pos = m.pose.pose.position
    def _vel(self, m): self.speed = m.longitudinal_velocity
    def _traj(self, m):
        self.traj_n += 1
        self.traj = [(p.pose.position.x, p.pose.position.y, p.longitudinal_velocity_mps)
                     for p in m.points]
    def _tf(self, m): self.tf = m
    def _cmd(self, m): self.cmd = m; self.cmd_n += 1
    def _mode(self, m):
        self.mode = m.mode; self.ctrl_enabled = m.is_autoware_control_enabled
    def _mrm(self, m): self.mrm = m

    def target_speed(self):
        """Planned speed at the trajectory point nearest the ego."""
        if not self.traj or self.pos is None:
            return None
        best, bestd = None, None
        for x, y, v in self.traj:
            d = math.hypot(x - self.pos.x, y - self.pos.y)
            if bestd is None or d < bestd:
                best, bestd = v, d
        return best


rclpy.init()
n = Probe()
start = time.monotonic()
nxt = 0.0
prev_cmd = 0
print("   t  speed traj_v  tf_acc tf_steer cmd_acc cmd_vel cmd_steer cmd/s traj_n "
      "mode ctrl  mrm")
while time.monotonic() - start < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
    t = time.monotonic() - start
    if t >= nxt:
        nxt += 1.0
        tv = n.target_speed()
        rate = n.cmd_n - prev_cmd
        prev_cmd = n.cmd_n
        f = lambda v, w=7, p=3: ("%*.*f" % (w, p, v)) if v is not None else " " * (w - 1) + "-"
        mode = MODE.get(n.mode, str(n.mode)) if n.mode is not None else "-"
        ctrl = ("on" if n.ctrl_enabled else "off") if n.ctrl_enabled is not None else "-"
        mrm = "-"
        if n.mrm is not None:
            mrm = "%s/%s" % (MRM_STATE.get(n.mrm.state, n.mrm.state),
                             MRM_BEHAVIOR.get(n.mrm.behavior, n.mrm.behavior))
        stalled = (n.speed is not None and abs(n.speed) < 0.1
                   and tv is not None and tv > 0.5)
        print("%5.0f %s %s %s %s %s %s %s %5d %6d %5s %4s %s%s"
              % (t, f(n.speed, 6, 2), f(tv, 6, 2),
                 f(n.tf.longitudinal.acceleration if n.tf else None),
                 f(n.tf.lateral.steering_tire_angle if n.tf else None, 8),
                 f(n.cmd.longitudinal.acceleration if n.cmd else None),
                 f(n.cmd.longitudinal.velocity if n.cmd else None),
                 f(n.cmd.lateral.steering_tire_angle if n.cmd else None, 9),
                 rate, n.traj_n, mode, ctrl, mrm,
                 "  <-- STALLED" if stalled else ""))
        sys.stdout.flush()
rclpy.shutdown()
