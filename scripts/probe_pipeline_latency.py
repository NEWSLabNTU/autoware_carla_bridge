#!/usr/bin/env python3
"""Where does the command-to-wheel delay actually come from?

CARLA's steering actuator tracks any Autoware command within one tick (issue 016), so a
~478 ms command-to-wheel delay has to come from the pipeline or from the measurement. The
original figure correlated a ROS topic stamped in SIM time against wheel angles sampled by
a CARLA client in WALL time; a constant offset between those clocks is indistinguishable
from a constant lag.

This measures all three things in one place:

  1. clock offset  -- ROS /clock against CARLA's own snapshot timestamp. A constant offset
                      here shows up as a fake lag in any cross-clock correlation.
  2. real-time factor -- sim seconds per wall second, both for /clock and for CARLA.
  3. command -> wheel lag, correlated entirely in CARLA SIM time, which is the only
     apples-to-apples comparison available.
"""
import math, sys, time
import carla, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from autoware_control_msgs.msg import Control

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 120

class P(Node):
    def __init__(self):
        super().__init__("probe_pipeline_latency")
        self.clock = None          # ROS /clock, seconds
        self.clock_wall = None     # wall time when it arrived
        self.cmds = []             # (ros_sim_stamp, wall, steering_tire_angle)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(Control, "/control/command/control_cmd", self.on_cmd, 10)

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec / 1e9
        self.clock_wall = time.time()

    def on_cmd(self, m):
        t = m.stamp.sec + m.stamp.nanosec / 1e9
        self.cmds.append((t, time.time(), m.lateral.steering_tire_angle))

client = carla.Client("localhost", 2000); client.set_timeout(20.0)
world = client.get_world()

def hero():
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == "hero":
            return a
    return None

rclpy.init()
n = P()
print("waiting for a hero vehicle and a control stream...", flush=True)
v = None
t0 = time.time()
while time.time() - t0 < 90:
    rclpy.spin_once(n, timeout_sec=0.1)
    v = v or hero()
    if v is not None and len(n.cmds) > 5:
        break
if v is None:
    print("no hero vehicle; run a scenario first"); raise SystemExit(1)

wheels, offsets, rtf = [], [], []
start_wall = time.time()
snap0 = world.get_snapshot().timestamp.elapsed_seconds
clock0 = None
while time.time() - start_wall < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.005)
    snap = world.get_snapshot()
    sim = snap.timestamp.elapsed_seconds
    try:
        fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    except Exception:
        break
    ang = -math.radians(0.5 * (fl + fr))       # CARLA right-positive -> ROS left-positive
    wheels.append((sim, time.time(), ang))
    if n.clock is not None:
        offsets.append(n.clock - sim)
        if clock0 is None:
            clock0 = n.clock
    time.sleep(0.02)

wall = time.time() - start_wall
print(f"\nsamples: {len(wheels)} wheel, {len(n.cmds)} commands, over {wall:.1f} s wall\n")

if offsets:
    offsets.sort()
    med = offsets[len(offsets)//2]
    print("1. CLOCK OFFSET  ros /clock minus CARLA elapsed_seconds")
    print(f"   median {med:+.3f} s   range {min(offsets):+.3f} .. {max(offsets):+.3f} s")
    print(f"   -> a cross-clock correlation would read this as {abs(med)*1000:.0f} ms of fake lag\n")

sim_span = wheels[-1][0] - wheels[0][0]
print("2. REAL-TIME FACTOR")
print(f"   CARLA sim advanced {sim_span:.1f} s in {wall:.1f} s wall  -> RTF {sim_span/wall:.3f}")
if clock0 is not None and n.clock is not None:
    print(f"   ROS /clock advanced {n.clock - clock0:.1f} s            -> RTF {(n.clock-clock0)/wall:.3f}")

# 3. Lag, correlated purely in sim time.
def resample(series, idx, val, grid):
    out, j = [], 0
    for g in grid:
        while j + 1 < len(series) and series[j + 1][idx] <= g:
            j += 1
        out.append(series[j][val])
    return out

if len(n.cmds) > 20 and len(wheels) > 20:
    lo = max(n.cmds[0][0], wheels[0][0]); hi = min(n.cmds[-1][0], wheels[-1][0])
    if hi - lo > 5:
        step = 0.05
        grid = [lo + i * step for i in range(int((hi - lo) / step))]
        c = resample(n.cmds, 0, 2, grid)
        wv = resample(wheels, 0, 2, grid)
        print("\n3. COMMAND -> WHEEL LAG, correlated in CARLA sim time")
        best, bestrms = None, None
        for shift in range(0, 21):
            pairs = [(c[i], wv[i + shift]) for i in range(len(grid) - shift)]
            if len(pairs) < 10: break
            rms = math.sqrt(sum((a - b) ** 2 for a, b in pairs) / len(pairs))
            if bestrms is None or rms < bestrms:
                best, bestrms = shift, rms
            if shift <= 12 and shift % 2 == 0:
                print(f"   shift {shift*step*1000:5.0f} ms  rms {rms:.4f} rad")
        print(f"   best shift {best*step*1000:.0f} ms (rms {bestrms:.4f} rad)")
rclpy.shutdown()
