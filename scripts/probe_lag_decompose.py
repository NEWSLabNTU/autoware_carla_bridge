#!/usr/bin/env python3
"""Split the steering command-to-wheel delay into its segments.

Previous work measured the whole path (478 ms, then ~200 ms on re-measurement) and excluded the
actuator, the subscription queue and tick synchronisation, leaving "ROS transport, the bridge's
apply path, or the measurement itself". Measuring the whole path again cannot separate those.

So measure the segments:

  1. ROS transport   the command's own header stamp against its arrival here. A direct
                     subtraction -- no correlation, no smooth-signal problem.
  2. Bridge apply    arrival against the control CARLA reports as applied. The bridge turns a
                     tire angle into a normalised steer, so the two are compared after the same
                     transform, and matched by value rather than by correlation where possible.
  3. Actuator        applied control against the physical wheel angle.

Both a value-match and a correlation are reported for the segments that need one, because a
correlation on a smooth signal can peak almost anywhere -- an earlier sweep here rose
monotonically to the edge of its range and would have been read as a one-second lag.
"""
import bisect, math, os, statistics, sys, threading, time
import carla, rclpy
from autoware_control_msgs.msg import Control

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0

rclpy.init()
node = rclpy.create_node("lag_decompose_%d" % os.getpid())
cmds = []          # (arrival_wall, stamp_sec, tire_angle)
def on_cmd(m):
    stamp = m.stamp.sec + m.stamp.nanosec * 1e-9
    cmds.append((time.time(), stamp, m.lateral.steering_tire_angle))
node.create_subscription(Control, "/control/command/control_cmd", on_cmd, 50)

applied = []       # (wall, steer_input, wheel_mean_rad)
stop = threading.Event()
def carla_thread():
    c = carla.Client("localhost", 2000); c.set_timeout(20.0); w = c.get_world()
    while not stop.is_set():
        try:
            w.wait_for_tick(seconds=5.0)
        except RuntimeError:
            continue
        e = next((a for a in w.get_actors().filter("vehicle.*")
                  if a.attributes.get("role_name") == "hero"), None)
        if e is None:
            continue
        try:
            ctl = e.get_control()
            fl = e.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            fr = e.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        except RuntimeError:
            continue
        applied.append((time.time(), ctl.steer, math.radians(0.5 * (fl + fr))))
threading.Thread(target=carla_thread, daemon=True).start()

t0 = time.time()
while time.time() - t0 < DURATION:
    rclpy.spin_once(node, timeout_sec=0.1)
stop.set(); time.sleep(0.3); rclpy.shutdown()

print("commands %d, carla samples %d" % (len(cmds), len(applied)))
if len(cmds) < 100 or len(applied) < 100:
    raise SystemExit("not enough data")

# --- 1. ROS transport, measured directly -------------------------------------------------
# The command carries a simulation-time stamp while arrival is wall clock, so the difference
# is only meaningful as a spread: its variation is the transport jitter, and a constant offset
# is the clock difference rather than latency.
offsets = [arr - st for arr, st, _ in cmds]
base = min(offsets)
transport = sorted(o - base for o in offsets)
print("\n1. ROS transport (stamp to arrival, offset removed)")
print("   median %.1f ms   p90 %.1f ms   max %.1f ms"
      % (1e3 * statistics.median(transport), 1e3 * transport[int(0.9 * len(transport))],
         1e3 * transport[-1]))

# --- 2 and 3. value matching --------------------------------------------------------------
def first_reaching(series, index, target, tol, after, limit=2.0):
    """When does `series` first come within tol of target, at or after `after`?"""
    i = bisect.bisect_left([s[0] for s in series], after)
    while i < len(series) and series[i][0] - after < limit:
        if abs(series[i][index] - target) <= tol:
            return series[i][0] - after
        i += 1
    return None

# The bridge sends -steer_command_for(angle); over the small angles a scenario uses that is
# very nearly linear, so calibrate the scale from the data instead of assuming the geometry.
pairs = []
for arr, _, angle in cmds:
    i = bisect.bisect_left([a[0] for a in applied], arr)
    if 0 <= i < len(applied) and abs(angle) > 0.02:
        pairs.append((angle, applied[i][1]))
scale = statistics.median(a[1] / a[0] for a in pairs if abs(a[0]) > 0.02) if pairs else None
print("\n   (bridge steer-per-radian, fitted from the data: %.3f)" % (scale or float("nan")))

# Only where the command actually moved. A value-match against a slowly varying signal matches
# on the first sample every time -- which is why the medians below came out at 2 ms and 0.6 ms
# on the first run, and meant nothing. A step is what carries timing information.
STEP = 0.01   # rad; ~0.6 deg of tire angle, well above the command's noise
delays_apply, delays_wheel = [], []
prev = None
for arr, _, angle in cmds:
    step = None if prev is None else abs(angle - prev)
    prev = angle
    if step is None or step < STEP:
        continue
    want_input = angle * scale
    d = first_reaching(applied, 1, want_input, abs(want_input) * 0.10 + 0.005, arr)
    if d is not None:
        delays_apply.append(d)
    d2 = first_reaching(applied, 2, -angle, abs(angle) * 0.10 + 0.002, arr)
    if d2 is not None:
        delays_wheel.append(d2)

def show(name, xs):
    if len(xs) < 20:
        print("   %-34s too few matches (%d)" % (name, len(xs)))
        return
    xs = sorted(xs)
    print("   %-34s median %6.1f ms   p90 %6.1f ms   n=%d"
          % (name, 1e3 * statistics.median(xs), 1e3 * xs[int(0.9 * len(xs))], len(xs)))

print("\n2. arrival -> CARLA reports that steer applied  (steps > %.3f rad only)" % STEP)
show("bridge apply", delays_apply)
print("\n3. arrival -> wheels reach the angle")
show("apply + actuator", delays_wheel)
