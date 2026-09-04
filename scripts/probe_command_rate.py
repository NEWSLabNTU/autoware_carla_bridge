#!/usr/bin/env python3
"""Is the remaining managed/unmanaged tracking gap real, or an artefact of the metric?

After the per-frame /clock fix, an unmanaged ego measures 0.110 m/s^2 of longitudinal
error against a managed ego's 0.062 on the same build. Two things could produce that:

  1. The metric. `acceptance.py::longitudinal_error` derives delivered acceleration over a
     span of FOUR VELOCITY SAMPLES, not a fixed time. The window's duration is therefore
     whatever the message rate makes it, and a shorter window is a noisier derivative. If
     the two modes publish at different rates, they are not being measured the same way.
  2. The controller. Autoware's timers run on simulation time, and the two modes see
     different clock granularity -- 100 ms steps from SSv2, 50 ms from the bridge. A
     controller that recomputes twice as often can demand changes faster than the vehicle
     follows, which is a real difference and not a measurement one.

So report the rates, and the same error computed both ways: the harness's 4-sample window,
and a fixed-duration window that cannot depend on rate. If the gap survives the fixed
window, it is not the metric.

    ROS_DOMAIN_ID=3 rate_probe.py [seconds] [--label unmanaged]
"""

import argparse
import statistics
import sys
import time

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport

LAG_S = 0.30          # command to delivery; the harness's figure, swept once
FIXED_WINDOW_S = 0.30  # rate-independent differentiation window


def rate_of(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def error_sample_window(commands, speeds, span: int = 4) -> tuple[int, float | None]:
    """The harness's own calculation: differentiate over `span` samples."""
    derived = []
    for i in range(span, len(speeds)):
        (t0, v0), (t1, v1) = speeds[i - span], speeds[i]
        if t1 - t0 > 1e-3:
            derived.append((0.5 * (t0 + t1), (v1 - v0) / (t1 - t0), v1))
    return _pair(commands, derived)


def error_fixed_window(commands, speeds, window: float = FIXED_WINDOW_S):
    """The same thing, but differentiating over a fixed DURATION.

    For each sample, find the earlier sample closest to `window` seconds back. The result
    cannot depend on how fast the topic publishes, which is the point.
    """
    derived = []
    for i in range(len(speeds)):
        t1, v1 = speeds[i]
        j = None
        for k in range(i - 1, -1, -1):
            if t1 - speeds[k][0] >= window:
                j = k
                break
        if j is None:
            continue
        t0, v0 = speeds[j]
        derived.append((0.5 * (t0 + t1), (v1 - v0) / (t1 - t0), v1))
    return _pair(commands, derived)


def _pair(commands, derived):
    errors = []
    for t_cmd, requested in commands:
        target = t_cmd + LAG_S
        near = [(abs(t - target), a, v) for t, a, v in derived if abs(t - target) < 0.10]
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
    ap.add_argument("--settle", type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("rate_probe_%d" % int(time.time()))
    commands: list[tuple[float, float]] = []
    speeds: list[tuple[float, float]] = []
    cmd_stamps: list[float] = []
    spd_stamps: list[float] = []

    def on_cmd(m):
        now = time.time()
        commands.append((now, m.longitudinal.acceleration))
        cmd_stamps.append(now)

    def on_speed(m):
        now = time.time()
        speeds.append((now, m.longitudinal_velocity))
        spd_stamps.append(now)

    node.create_subscription(Control, "/control/command/control_cmd", on_cmd, 20)
    node.create_subscription(
        VelocityReport, "/vehicle/status/velocity_status", on_speed, 20)

    # Wait for the ego to be MOVING before the window opens; a standstill contributes no
    # tracking samples and would make the measurement about the wait, not the driving.
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if speeds and speeds[-1][1] > 1.0:
            break
    if not (speeds and speeds[-1][1] > 1.0):
        print(f"RATE {args.label} ego never moved within {args.settle:.0f}s")
        return 1

    commands.clear(); speeds.clear(); cmd_stamps.clear(); spd_stamps.clear()
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    n4, e4 = error_sample_window(commands, speeds)
    nf, ef = error_fixed_window(commands, speeds)
    print(f"RATE {args.label} control_cmd {rate_of(cmd_stamps):.2f} Hz "
          f"({len(cmd_stamps)} msgs), velocity_status {rate_of(spd_stamps):.2f} Hz "
          f"({len(spd_stamps)} msgs)")
    if len(spd_stamps) > 5:
        gaps = [b - a for a, b in zip(spd_stamps, spd_stamps[1:])]
        print(f"RATE {args.label} velocity gap median {statistics.median(gaps) * 1e3:.1f} ms "
              f"-> the harness's 4-sample window spans "
              f"{4 * statistics.median(gaps) * 1e3:.0f} ms")
    print(f"RATE {args.label} longitudinal 4-sample window: "
          f"{'n/a' if e4 is None else f'{e4:.3f}'} m/s^2 over {n4} samples")
    print(f"RATE {args.label} longitudinal fixed {FIXED_WINDOW_S:.2f}s window: "
          f"{'n/a' if ef is None else f'{ef:.3f}'} m/s^2 over {nf} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
