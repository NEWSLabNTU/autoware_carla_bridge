#!/usr/bin/env python3
"""Re-measure the ~478 ms command-to-wheel lag on a real Autoware-driven run.

A synthetic square wave through the same path measures a deterministic one tick (100 ms),
so either the original figure is an artifact or the real command stream behaves differently.
The likely trap is that cross-correlating two SMOOTH signals is ill-conditioned: Autoware's
steering command is rate-limited and slowly varying, so a wide range of shifts fit nearly as
well and the argmin is easily moved by noise or drift.

Three estimators on the same samples, so they can be compared directly:

  A  rms-vs-shift on the raw signals          (the original method)
  B  rms-vs-shift on the derivatives          (high-passed: driven by fast changes)
  C  edge timing at the sharpest command steps (model-free, like the square-wave probe)

Also reports how sharply each curve is minimised, which is the thing that says whether the
number means anything.
"""
import math, sys, time
import carla, rclpy
from autoware_control_msgs.msg import Control

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 100
HZ = 50.0

rclpy.init()
n = rclpy.create_node("probe_478")
latest = {"cmd": None}
n.create_subscription(Control, "/control/command/control_cmd",
                      lambda m: latest.__setitem__("cmd", m.lateral.steering_tire_angle), 10)

client = carla.Client("localhost", 2000); client.set_timeout(20.0)
world = client.get_world()

def hero():
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == "hero":
            return a
    return None

print("waiting for hero + control stream...", flush=True)
v, t0 = None, time.time()
while time.time() - t0 < 120:
    rclpy.spin_once(n, timeout_sec=0.02)
    v = v or hero()
    if v is not None and latest["cmd"] is not None:
        break
if v is None or latest["cmd"] is None:
    print(f"not ready (hero={v is not None}, cmd={latest['cmd'] is not None})"); raise SystemExit(1)
print(f"hero {v.id}, sampling {SECONDS}s at {HZ:.0f} Hz", flush=True)

rows = []
start = time.time()
nxt = start
while time.time() - start < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.001)
    try:
        fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    except Exception:
        break
    wheel = -math.radians(0.5 * (fl + fr))
    rows.append((time.time() - start, latest["cmd"], wheel))
    nxt += 1.0 / HZ
    s = nxt - time.time()
    if s > 0:
        time.sleep(s)
    else:
        nxt = time.time()

print(f"\nsamples: {len(rows)} over {rows[-1][0]:.1f}s\n")
cmd = [r[1] for r in rows]
whl = [r[2] for r in rows]
step_ms = 1000.0 / HZ

def sweep(a, b, label):
    out = []
    for sh in range(0, int(1.0 * HZ) + 1):
        pairs = [(a[i], b[i + sh]) for i in range(len(a) - sh)]
        if len(pairs) < 20:
            break
        rms = math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs))
        out.append((sh, rms))
    best = min(out, key=lambda t: t[1])
    worst = max(out, key=lambda t: t[1])
    flat = [s for s, r in out if r <= best[1] * 1.05]
    print(f"{label}")
    print(f"   best shift {best[0]*step_ms:6.0f} ms   rms {best[1]:.5f}")
    print(f"   rms at 0 shift {out[0][1]:.5f}   worst {worst[1]:.5f}")
    print(f"   within 5% of best: {min(flat)*step_ms:.0f}..{max(flat)*step_ms:.0f} ms"
          f"  ({len(flat)} of {len(out)} shifts)")
    depth = (out[0][1] - best[1]) / out[0][1] * 100 if out[0][1] else 0
    print(f"   improvement over zero shift: {depth:.1f}%")
    return best[0] * step_ms

a = sweep(cmd, whl, "A. raw signals (the original method)")
d_cmd = [cmd[i+1] - cmd[i] for i in range(len(cmd)-1)]
d_whl = [whl[i+1] - whl[i] for i in range(len(whl)-1)]
print()
b = sweep(d_cmd, d_whl, "B. derivatives (high-passed)")

print("\nC. edge timing at the sharpest command steps")
edges = sorted(range(len(d_cmd)), key=lambda i: -abs(d_cmd[i]))[:400]
lags = []
used = []
for i in edges:
    if any(abs(i - u) < int(HZ * 0.5) for u in used):
        continue
    if abs(d_cmd[i]) < 0.002:
        continue
    used.append(i)
    target = cmd[i + 1]
    for j in range(i, min(i + int(HZ), len(whl))):
        if abs(whl[j] - target) < max(0.004, abs(d_cmd[i]) * 0.5):
            lags.append((j - i) * step_ms)
            break
    if len(used) >= 25:
        break
if lags:
    lags.sort()
    print(f"   {len(lags)} isolated command steps")
    print(f"   lag  median {lags[len(lags)//2]:.0f} ms   "
          f"p10 {lags[len(lags)//10]:.0f}   p90 {lags[min(len(lags)-1, 9*len(lags)//10)]:.0f}")
else:
    print("   no sharp isolated steps found (command is smooth)")

print(f"\ncommand range: {min(cmd):+.4f} .. {max(cmd):+.4f} rad")
print(f"wheel   range: {min(whl):+.4f} .. {max(whl):+.4f} rad")
rclpy.shutdown()
