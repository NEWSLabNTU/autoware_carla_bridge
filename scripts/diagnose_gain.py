#!/usr/bin/env python3
"""Locate what is left of the longitudinal tracking error.

The measured pedal maps cut the error but left a gain near 1.8: the ego delivers about twice
the acceleration *variation* Autoware asks for. The maps no longer explain that -- they are
measured, and the car follows them -- so this asks where the remaining discrepancy lives.

Records four streams and slices them:
  * Autoware's requested acceleration
  * CARLA's delivered longitudinal acceleration
  * the control CARLA actually has applied (throttle, brake, hand brake)
  * speed

Then reports, per regime, how delivered relates to requested, and sweeps the assumed
command-to-delivery lag rather than fixing it -- a wrong lag distorts gain on its own, and 0.3 s
was a guess.
"""
import math, os, statistics, sys, threading, time
import carla
import rclpy
from autoware_control_msgs.msg import Control

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/gain_samples.tsv"

truth, cmds = [], []
stop = threading.Event()

rclpy.init()
n = rclpy.create_node("diagnose_gain_%d" % os.getpid())
n.create_subscription(Control, "/control/command/control_cmd",
                      lambda m: cmds.append((time.time(), m.longitudinal.acceleration)), 50)


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
            v, a, tf, ctl = (ego.get_velocity(), ego.get_acceleration(),
                             ego.get_transform(), ego.get_control())
        except RuntimeError:
            continue
        yaw = math.radians(tf.rotation.yaw)
        truth.append((time.time(), math.hypot(v.x, v.y),
                      a.x * math.cos(yaw) + a.y * math.sin(yaw),
                      ctl.throttle, ctl.brake, 1 if ctl.hand_brake else 0))


t = threading.Thread(target=carla_thread, daemon=True)
t.start()
t0 = time.time()
while time.time() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.1)
stop.set()
time.sleep(0.5)
rclpy.shutdown()

print("carla samples %d, commands %d" % (len(truth), len(cmds)))
if len(truth) < 100 or len(cmds) < 30:
    raise SystemExit("not enough data")


def pair_at(lag):
    out = []
    for t_c, a_req in cmds:
        target = t_c + lag
        near = [(abs(s[0] - target),) + s[1:] for s in truth if abs(s[0] - target) < 0.06]
        if near:
            _, speed, a_got, thr, brk, hb = min(near)
            out.append((a_req, a_got, speed, thr, brk, hb))
    return out


def gain_of(pairs):
    if len(pairs) < 20:
        return float("nan"), float("nan"), 0
    req = [p[0] for p in pairs]
    got = [p[1] for p in pairs]
    mr, mg = statistics.fmean(req), statistics.fmean(got)
    var = statistics.pstdev(req) ** 2
    if var < 1e-9:
        return float("nan"), float("nan"), len(pairs)
    g = statistics.fmean([(r - mr) * (x - mg) for r, x in zip(req, got)]) / var
    med = statistics.median([abs(x - r) for r, x in zip(req, got)])
    return g, med, len(pairs)


print("\nlag sweep (all samples):")
print("   lag    gain   median|err|    n")
best = None
for lag in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0]:
    p = pair_at(lag)
    g, med, cnt = gain_of(p)
    print("  %.1fs  %6.3f     %6.3f   %4d" % (lag, g, med, cnt))
    if p and (best is None or med < best[1]):
        best = (lag, med)

lag = best[0] if best else 0.3
pairs = pair_at(lag)
print("\nbest lag %.1fs; slicing there" % lag)

moving = [p for p in pairs if p[2] > 1.0]
slow = [p for p in pairs if p[2] <= 1.0]
hb_on = [p for p in pairs if p[5] == 1]
hb_off = [p for p in pairs if p[5] == 0]
braking = [p for p in moving if p[0] < -0.05]
driving = [p for p in moving if p[0] > 0.05]

for name, subset in [("all", pairs), ("moving >1 m/s", moving), ("standstill <=1", slow),
                     ("hand brake on", hb_on), ("hand brake off", hb_off),
                     ("moving, request<0", braking), ("moving, request>0", driving)]:
    g, med, cnt = gain_of(subset)
    print("  %-20s gain %6.3f   median|err| %6.3f   n=%4d" % (name, g, med, cnt))

with open(OUT, "w") as f:
    f.write("req\tgot\tspeed\tthrottle\tbrake\thandbrake\n")
    for p in pairs:
        f.write("%.4f\t%.4f\t%.3f\t%.3f\t%.3f\t%d\n" % p)
print("\nsamples -> %s" % OUT)
