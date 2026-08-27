#!/usr/bin/env python3
"""Check the longitudinal tracking metric against CARLA's own state before trusting it.

score_longitudinal.py derives delivered acceleration by differentiating the bridge's velocity
report at ~20 Hz. That is two inferences deep -- the report might not match CARLA, and the
differentiation might not match a real acceleration -- and it produced a gain of 1.6 to 2.6,
which would mean the ego routinely delivers far more acceleration than Autoware asks for.
Before chasing that, check both inferences against the server.

Records three streams live and compares them:
  * CARLA ground truth: the ego actor's speed and acceleration, read as a passive observer
  * the bridge's own /vehicle/status/velocity_status
  * Autoware's /control/command/control_cmd

Never ticks. During a scenario the tick belongs to carla_scenario_bridge.
"""
import math, os, statistics, sys, threading, time
import carla
import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0

truth = []      # (wall_t, speed, accel_longitudinal)
cmds = []       # (wall_t, requested accel)
reports = []    # (wall_t, reported speed)
stop = threading.Event()


def carla_thread():
    c = carla.Client("localhost", 2000)
    c.set_timeout(20.0)
    w = c.get_world()
    ego = None
    while not stop.is_set():
        try:
            w.wait_for_tick(seconds=5.0)
        except RuntimeError:
            continue
        if ego is None or not ego.is_alive:
            ego = next((a for a in w.get_actors().filter("vehicle.*")
                        if a.attributes.get("role_name") == "hero"), None)
            if ego is None:
                continue
        try:
            v, a, tf = ego.get_velocity(), ego.get_acceleration(), ego.get_transform()
        except RuntimeError:
            continue
        speed = math.hypot(v.x, v.y)
        # Project acceleration onto the heading: the longitudinal component is what a
        # longitudinal controller is asking for, not the magnitude.
        yaw = math.radians(tf.rotation.yaw)
        a_long = a.x * math.cos(yaw) + a.y * math.sin(yaw)
        truth.append((time.time(), speed, a_long))


# ROS first: bringing up a CARLA client on another thread before rclpy.init() has left the
# middleware unable to create a node ("rmw handle is invalid").
rclpy.init()
n = rclpy.create_node("validate_longitudinal_%d" % (os.getpid()))

t = threading.Thread(target=carla_thread, daemon=True)
t.start()
n.create_subscription(Control, "/control/command/control_cmd",
                      lambda m: cmds.append((time.time(), m.longitudinal.acceleration)), 50)
n.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
                      lambda m: reports.append((time.time(), m.longitudinal_velocity)), 50)
t0 = time.time()
while time.time() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.1)
stop.set()
time.sleep(0.5)
rclpy.shutdown()

print("samples: carla %d, reports %d, commands %d" % (len(truth), len(reports), len(cmds)))
if len(truth) < 50 or len(reports) < 20:
    raise SystemExit("not enough data")


def nearest(series, t, window):
    near = [(abs(s[0] - t),) + s[1:] for s in series if abs(s[0] - t) < window]
    return min(near) if near else None


# 1. Does the bridge report the speed CARLA has?
diffs = [abs(sp - n_[1]) for t_r, sp in reports if (n_ := nearest(truth, t_r, 0.05))]
if diffs:
    print("\n1. reported speed vs CARLA speed")
    print("   |difference|  median %.4f   p95 %.4f  m/s  (n=%d)"
          % (statistics.median(diffs), sorted(diffs)[int(0.95 * len(diffs))], len(diffs)))

# 2. Does differentiating the report reproduce CARLA's acceleration?
derived = []
for i in range(4, len(reports)):
    (ta, va), (tb, vb) = reports[i - 4], reports[i]
    if tb - ta > 1e-3:
        derived.append((0.5 * (ta + tb), (vb - va) / (tb - ta)))
pairs = [(d, n_[1]) for t_d, d in derived if (n_ := nearest([(x[0], x[2]) for x in truth], t_d, 0.05))]
if pairs:
    err = [d - g for d, g in pairs]
    print("\n2. differentiated report vs CARLA acceleration")
    print("   |error|  median %.3f   p95 %.3f  m/s^2  (n=%d)"
          % (statistics.median([abs(e) for e in err]),
             sorted(abs(e) for e in err)[int(0.95 * len(err))], len(err)))
    print("   CARLA accel spread: sd %.3f   derived spread: sd %.3f"
          % (statistics.pstdev([g for _, g in pairs]), statistics.pstdev([d for d, _ in pairs])))

# 3. The honest gain: requested against CARLA's actual acceleration.
LAG = 0.30
gpairs = []
for t_c, a_req in cmds:
    n_ = nearest([(x[0], x[2], x[1]) for x in truth], t_c + LAG, 0.10)
    if n_ and n_[2] > 1.0:            # ignore standstill, where the handbrake rules
        gpairs.append((a_req, n_[1]))
if len(gpairs) > 30:
    req = [r for r, _ in gpairs]
    got = [g for _, g in gpairs]
    mr, mg = statistics.fmean(req), statistics.fmean(got)
    var = statistics.pstdev(req) ** 2
    gain = statistics.fmean([(r - mr) * (g - mg) for r, g in gpairs]) / var if var > 1e-9 else float("nan")
    err = [g - r for r, g in gpairs]
    print("\n3. requested vs CARLA-measured acceleration  (n=%d)" % len(gpairs))
    print("   |error|  median %.3f   p90 %.3f  m/s^2"
          % (statistics.median([abs(e) for e in err]),
             sorted(abs(e) for e in err)[int(0.9 * len(err))]))
    print("   bias %+.3f   gain %.3f" % (statistics.fmean(err), gain))
