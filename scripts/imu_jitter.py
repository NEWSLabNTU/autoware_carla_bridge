#!/usr/bin/env python3
"""Measure IMU inter-arrival jitter, which gyro_odometer times out on.

On a failing later run the first real fault is `gyro_odometer: IMU msg is timeout.
imu_dt: 0.2208[sec], tolerance 0.2[sec]`, some 30 s before any control or planning error.
Average rate looks healthy on both runs (~10.5 Hz), so the problem is spacing, not
throughput. acb_bridge publishes this topic, so the gaps are ours. See acb docs/issues/016.
"""
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 60
TOL = 0.2

rclpy.init()
n = rclpy.create_node("imu_jitter")
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
gaps, state = [], {"last": None, "last_stamp": None}
stamp_gaps = []

def on_imu(m):
    now = time.time()
    if state["last"] is not None:
        gaps.append(now - state["last"])
    state["last"] = now
    st = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
    if state["last_stamp"] is not None:
        stamp_gaps.append(st - state["last_stamp"])
    state["last_stamp"] = st

n.create_subscription(Imu, "/sensing/imu/imu_data", on_imu, qos)
start = time.time()
while time.time() - start < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.02)

def report(label, g):
    if len(g) < 5:
        print(f"  {label}: too few samples ({len(g)})")
        return
    g2 = sorted(g)
    over = sum(1 for x in g if x > TOL)
    print(f"  {label}: n={len(g)}  median {g2[len(g2)//2]*1000:6.1f} ms  "
          f"p95 {g2[int(0.95*len(g2))]*1000:6.1f}  max {g2[-1]*1000:7.1f}  "
          f"over {TOL}s: {over} ({100*over/len(g):.1f}%)")

print(f"IMU inter-arrival over {SECONDS}s (tolerance {TOL}s):")
report("wall-clock arrival", gaps)
report("header stamp delta", stamp_gaps)
rclpy.shutdown()
