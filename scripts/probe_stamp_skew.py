#!/usr/bin/env python3
"""How far ahead of /clock are the messages Autoware is asked to act on?

Phase 013's remaining question. A managed ego tracks longitudinally at 0.038 m/s^2 and an
unmanaged one at ~0.16 on the same build, and neither the metric nor the clock's rate or
regularity explains it -- an unmanaged clock made identical to SSv2's tracked three times
WORSE, not better.

What still differs is who drives the ticks. In a managed run SSv2 both publishes /clock and
advances CARLA, so a message's stamp and the clock move together by construction. In an
unmanaged run the bridge publishes a clock derived from frames it merely observes, while
SSv2 advances CARLA from another domain -- nothing couples the two except that both watch
the same server.

So measure the coupling directly: for each message, stamp minus the newest /clock value at
the moment it arrived.

  positive -> the message is stamped in the FUTURE relative to the clock Autoware is on.
              Autoware's message filters hold or drop such data ("extrapolation into the
              future"), which is the shape of issue 016's cascade.
  negative -> stamped in the past, i.e. the clock has already moved on. Ordinary; some of
              it is unavoidable quantisation, up to one clock period.

Quantisation is the floor on interpreting this: between two /clock messages the newest
value is stale by up to one period (50 ms at 20 Hz, 100 ms at 10 Hz), so a median skew
around minus half a period is what a perfectly coupled stream looks like. What matters is
whether one mode sits far from that, and whether any mass lands on the positive side.

    ROS_DOMAIN_ID=3 skew_probe.py [seconds] [--label unmanaged]
"""

import argparse
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from autoware_vehicle_msgs.msg import VelocityReport


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def summarise(label: str, name: str, skews: list[float], period_ms: float | None) -> None:
    if len(skews) < 20:
        print(f"SKEW {label} {name}: only {len(skews)} samples")
        return
    ms = [s * 1e3 for s in skews]
    ms.sort()
    ahead = sum(1 for v in ms if v > 0)
    p95 = ms[int(0.95 * (len(ms) - 1))]
    p05 = ms[int(0.05 * (len(ms) - 1))]
    extra = ""
    if period_ms:
        # A perfectly coupled stream sits near minus half a clock period, purely from the
        # newest clock value being stale.
        extra = f", expected ~{-period_ms / 2:.0f} ms from quantisation alone"
    print(f"SKEW {label} {name}: n={len(ms)} median {statistics.median(ms):+.1f} ms, "
          f"mean {statistics.mean(ms):+.1f}, p05 {p05:+.1f}, p95 {p95:+.1f}, "
          f"min {ms[0]:+.1f}, max {ms[-1]:+.1f}{extra}")
    print(f"SKEW {label} {name}: {ahead}/{len(ms)} ({100.0 * ahead / len(ms):.1f}%) stamped "
          f"AHEAD of the clock")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seconds", nargs="?", type=float, default=120.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--settle", type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("skew_probe_%d" % int(time.time()))
    clock = {"sim": None}
    clock_gaps: list[float] = []
    last_clock_wall = {"t": None}
    skews: dict[str, list[float]] = {"velocity_status": [], "kinematic_state": [], "gnss": []}
    moving = {"yes": False}

    def on_clock(msg: Clock):
        clock["sim"] = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.monotonic()
        if last_clock_wall["t"] is not None:
            clock_gaps.append(now - last_clock_wall["t"])
        last_clock_wall["t"] = now

    def record(name):
        def cb(msg):
            if clock["sim"] is None:
                return
            skews[name].append(stamp_seconds(msg.header) - clock["sim"])
        return cb

    def on_velocity(msg):
        if msg.longitudinal_velocity > 1.0:
            moving["yes"] = True
        if clock["sim"] is not None:
            skews["velocity_status"].append(stamp_seconds(msg.header) - clock["sim"])

    node.create_subscription(
        Clock, "/clock", on_clock,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))
    node.create_subscription(
        VelocityReport, "/vehicle/status/velocity_status", on_velocity, 20)
    node.create_subscription(
        Odometry, "/localization/kinematic_state", record("kinematic_state"), 20)
    node.create_subscription(
        PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", record("gnss"), 20)

    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline and not moving["yes"]:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not moving["yes"]:
        print(f"SKEW {args.label} ego never moved within {args.settle:.0f}s")
        return 1

    for v in skews.values():
        v.clear()
    clock_gaps.clear()
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    period_ms = statistics.median(clock_gaps) * 1e3 if len(clock_gaps) > 5 else None
    if period_ms:
        print(f"SKEW {args.label} clock period {period_ms:.1f} ms "
              f"({1000.0 / period_ms:.2f} Hz)")
    for name in ("velocity_status", "kinematic_state", "gnss"):
        summarise(args.label, name, skews[name], period_ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
