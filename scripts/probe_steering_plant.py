"""Measure the steering plant Autoware's lateral controller is closing a loop around.

Issue 016's stall is preceded by a diverging steering oscillation, with localization,
the steering scale and the command path all excluded. What is left is the loop's gain and
delay against the real plant, and neither has been measured directly on this stack.

Records, paired in time:
  cmd      commanded steering_tire_angle, from /control/command/control_cmd (rad)
  wheels   what CARLA's front wheels actually did (rad, sign-flipped to ROS)
  yaw      the vehicle's yaw rate (rad/s), the thing the controller ultimately wants
  speed    because steering authority scales with it

Then reports the delay that best aligns command to wheels, and the gain between them.
Cross-correlation is only meaningful if the command actually has edges, so the spread of
the command is reported too: a smooth signal correlates well at every lag and the peak
means nothing (this is the trap issue 009 fell into).
"""
import sys, math, time, threading
import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from autoware_control_msgs.msg import Control

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
ROLE = sys.argv[2] if len(sys.argv) > 2 else "hero"

samples = []          # (t, cmd_rad, wheel_rad, yaw_rate, speed)
latest_cmd = {"v": None}
lock = threading.Lock()


class CmdSub(Node):
    def __init__(self):
        super().__init__('steer_plant')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Control, '/control/command/control_cmd', self._cmd, qos)

    def _cmd(self, m):
        with lock:
            latest_cmd["v"] = m.lateral.steering_tire_angle


def ros_thread(node, stop):
    while not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


rclpy.init()
node = CmdSub()
stop = threading.Event()
t = threading.Thread(target=ros_thread, args=(node, stop), daemon=True)
t.start()

client = carla.Client('localhost', 2000)
client.set_timeout(30.0)
world = client.get_world()

start = time.monotonic()
while time.monotonic() - start < DURATION:
    try:
        world.wait_for_tick(30.0)
    except Exception:
        continue
    ego = None
    for a in world.get_actors():
        if a.type_id.startswith('vehicle.') and a.attributes.get('role_name') == ROLE:
            ego = a
            break
    if ego is None:
        continue
    with lock:
        cmd = latest_cmd["v"]
    if cmd is None:
        continue
    try:
        fl = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    except Exception:
        continue
    # CARLA reports degrees, positive to the right; ROS is radians, positive left.
    wheel = -math.radians((fl + fr) / 2.0)
    av = ego.get_angular_velocity()          # deg/s in CARLA, z up but left-handed
    yaw_rate = -math.radians(av.z)
    v = ego.get_velocity()
    speed = math.hypot(v.x, v.y)
    samples.append((time.monotonic() - start, cmd, wheel, yaw_rate, speed))

stop.set()
rclpy.shutdown()

moving = [s for s in samples if s[4] > 1.0]
print("samples %d (%d while moving above 1 m/s)" % (len(samples), len(moving)))
if len(moving) < 50:
    print("not enough moving samples to say anything")
    sys.exit(0)

cmds = [s[1] for s in moving]
whls = [s[2] for s in moving]
dt = (moving[-1][0] - moving[0][0]) / max(1, len(moving) - 1)
print("mean sample period %.3f s" % dt)

# Does the command have edges at all? Without them a lag peak is meaningless.
steps = [abs(cmds[i] - cmds[i - 1]) for i in range(1, len(cmds))]
steps_sorted = sorted(steps)
print("command step per sample: median %.5f rad, p95 %.5f, max %.5f"
      % (steps_sorted[len(steps_sorted) // 2],
         steps_sorted[int(0.95 * (len(steps_sorted) - 1))], steps_sorted[-1]))
print("command range %.4f to %.4f rad" % (min(cmds), max(cmds)))


def corr(a, b):
    n = min(len(a), len(b))
    if n < 10:
        return 0.0
    ma = sum(a[:n]) / n
    mb = sum(b[:n]) / n
    va = sum((x - ma) ** 2 for x in a[:n])
    vb = sum((x - mb) ** 2 for x in b[:n])
    if va <= 0 or vb <= 0:
        return 0.0
    cov = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
    return cov / math.sqrt(va * vb)


print("\nlag(s)  corr(cmd -> wheels)")
best = (None, -2.0)
for shift in range(0, min(40, len(moving) // 4)):
    c = corr(cmds[:len(cmds) - shift] if shift else cmds, whls[shift:])
    if shift * dt <= 2.0:
        print("  %.2f    %.4f" % (shift * dt, c))
    if c > best[1]:
        best = (shift * dt, c)
print("best alignment at %.2f s (corr %.4f)" % best)

# Gain at the best alignment: how much wheel angle per unit command.
shift = int(round(best[0] / dt)) if dt > 0 else 0
pairs = [(cmds[i], whls[i + shift]) for i in range(len(cmds) - shift)
         if abs(cmds[i]) > 0.01]
if pairs:
    ratios = sorted(w / c for c, w in pairs)
    print("wheels/command at that lag: median %.3f over %d samples"
          % (ratios[len(ratios) // 2], len(ratios)))
