"""How much information does the steering command carry for timing a delay?"""
import os, statistics, sys, time
import rclpy
from autoware_control_msgs.msg import Control
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
rclpy.init(); n = rclpy.create_node("cmd_spec_%d" % os.getpid())
c = []
n.create_subscription(Control, "/control/command/control_cmd",
                      lambda m: c.append((time.time(), m.lateral.steering_tire_angle)), 50)
t0 = time.time()
while time.time() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()
print("commands %d over %.0f s (%.1f Hz)" % (len(c), DURATION, len(c) / DURATION))
if len(c) < 50:
    raise SystemExit("not enough")
steps = [abs(c[i][1] - c[i-1][1]) for i in range(1, len(c))]
steps_s = sorted(steps)
print("per-message change in commanded tire angle (rad):")
print("  median %.5f   p90 %.5f   p99 %.5f   max %.5f"
      % (statistics.median(steps_s), steps_s[int(.9*len(steps_s))],
         steps_s[int(.99*len(steps_s))], steps_s[-1]))
for thr in (0.002, 0.005, 0.01, 0.02, 0.05):
    print("  steps > %.3f rad: %4d of %d (%.1f%%)"
          % (thr, sum(1 for x in steps if x > thr), len(steps),
             100.0*sum(1 for x in steps if x > thr)/len(steps)))
rng = max(x[1] for x in c) - min(x[1] for x in c)
print("full command range over the run: %.4f rad" % rng)
