#!/usr/bin/env python3
"""Is the residual managed/unmanaged tracking gap a command-bandwidth artefact?

Three explanations for the gap are already eliminated by measurement: the metric's window
length, the clock's rate and regularity, and stamp-to-clock coupling. What has never been
controlled for is BANDWIDTH.

The harness scores tracking as |commanded acceleration at t| - |delivered acceleration at
t+0.30|, where "delivered" is a derivative taken over roughly 333 ms of velocity samples.
That derivative is a low-pass filter. The command is not filtered at all. An unmanaged ego
issues control_cmd at 21 Hz and a managed one at 11, so the unmanaged command carries
frequency content the derivative physically cannot show -- and the difference between a
fast signal and a slow one is counted as tracking error whether or not the vehicle is
following any worse.

So compute the same error three ways over one recording:

  raw       what the harness does: instantaneous command vs smoothed delivery
  matched   command averaged over the SAME window as the derivative, then compared --
            the two signals now carry the same bandwidth
  decimated command subsampled to ~11 Hz first, the managed rate, then compared raw. A
            crude check that reaches the same place by a different route.

If `matched` closes the gap and `raw` does not, the residual is measurement. If the gap
survives matching, it is the vehicle.

Traces are written to CSV so this can be re-analysed without another run.

    ROS_DOMAIN_ID=3 bw_probe.py [seconds] --label unmanaged --csv out.csv
"""

import argparse
import statistics
import sys
import time

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport

LAG_S = 0.30
SPAN = 4            # the harness's derivative span, in velocity samples


def derive(speeds, span=SPAN):
    """Delivered acceleration, and the window each estimate actually covers."""
    out = []
    for i in range(span, len(speeds)):
        (t0, v0), (t1, v1) = speeds[i - span], speeds[i]
        if t1 - t0 > 1e-3:
            out.append((0.5 * (t0 + t1), (v1 - v0) / (t1 - t0), v1, t1 - t0))
    return out


def smooth_commands(commands, window):
    """Average each command over `window` seconds centred on it."""
    if not commands:
        return []
    out = []
    j0 = 0
    for i, (t, _) in enumerate(commands):
        lo, hi = t - window / 2.0, t + window / 2.0
        while j0 < len(commands) and commands[j0][0] < lo:
            j0 += 1
        j = j0
        acc = []
        while j < len(commands) and commands[j][0] <= hi:
            acc.append(commands[j][1])
            j += 1
        out.append((t, statistics.mean(acc) if acc else commands[i][1]))
    return out


def decimate(commands, target_hz):
    """Keep roughly `target_hz` of the command stream, evenly in time."""
    if not commands:
        return []
    step = 1.0 / target_hz
    out = [commands[0]]
    for t, a in commands[1:]:
        if t - out[-1][0] >= step:
            out.append((t, a))
    return out


def score(commands, derived):
    errors = []
    for t_cmd, requested in commands:
        target = t_cmd + LAG_S
        near = [(abs(t - target), a, v) for t, a, v, _ in derived if abs(t - target) < 0.10]
        if not near:
            continue
        _, delivered, speed = min(near)
        if speed > 1.0:
            errors.append(abs(delivered - requested))
    if len(errors) < 30:
        return len(errors), None
    return len(errors), statistics.median(errors)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seconds", nargs="?", type=float, default=150.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--settle", type=float, default=500.0)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("bw_probe_%d" % int(time.time()))
    commands: list[tuple[float, float]] = []
    speeds: list[tuple[float, float]] = []
    moving = {"yes": False}

    def on_cmd(m):
        commands.append((time.time(), m.longitudinal.acceleration))

    def on_speed(m):
        speeds.append((time.time(), m.longitudinal_velocity))
        if m.longitudinal_velocity > 1.0:
            moving["yes"] = True

    node.create_subscription(Control, "/control/command/control_cmd", on_cmd, 50)
    node.create_subscription(
        VelocityReport, "/vehicle/status/velocity_status", on_speed, 50)

    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline and not moving["yes"]:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not moving["yes"]:
        print(f"BW {args.label} ego never moved within {args.settle:.0f}s")
        return 1

    commands.clear(); speeds.clear()
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    derived = derive(speeds)
    if not derived:
        print(f"BW {args.label} no usable velocity samples")
        return 1
    window = statistics.median(d[3] for d in derived)

    cmd_hz = (len(commands) - 1) / (commands[-1][0] - commands[0][0]) if len(commands) > 1 else 0
    spd_hz = (len(speeds) - 1) / (speeds[-1][0] - speeds[0][0]) if len(speeds) > 1 else 0

    n_raw, e_raw = score(commands, derived)
    n_mat, e_mat = score(smooth_commands(commands, window), derived)
    n_dec, e_dec = score(decimate(commands, 11.0), derived)

    def fmt(v):
        return "n/a" if v is None else f"{v:.3f}"

    print(f"BW {args.label} control_cmd {cmd_hz:.2f} Hz, velocity {spd_hz:.2f} Hz, "
          f"derivative window {window * 1e3:.0f} ms")
    print(f"BW {args.label} raw       {fmt(e_raw)} m/s^2 over {n_raw} samples")
    print(f"BW {args.label} matched   {fmt(e_mat)} m/s^2 over {n_mat} samples "
          f"(command averaged over the same {window * 1e3:.0f} ms)")
    print(f"BW {args.label} decimated {fmt(e_dec)} m/s^2 over {n_dec} samples "
          f"(command subsampled to ~11 Hz)")

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("kind,t,value\n")
            for t, a in commands:
                f.write(f"cmd,{t:.6f},{a:.6f}\n")
            for t, v in speeds:
                f.write(f"speed,{t:.6f},{v:.6f}\n")
        print(f"BW {args.label} traces written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
