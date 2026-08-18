#!/usr/bin/env python3
"""Dump perceived objects in the ego's own frame, to see what it swerves around.

Issue 016: a failing run shows a burst of 18-40 objects at the instant the ego leaves its
lane, where a passing run never sees more than 3. This prints each object's position
relative to the ego, its class, size and existence probability, so the burst can be
identified -- the ego's own body, the kerb, or the parked background AV.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects, DetectedObjects

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 240
BURST = int(sys.argv[2]) if len(sys.argv) > 2 else 5

LABEL = {0: "UNKNOWN", 1: "CAR", 2: "TRUCK", 3: "BUS", 4: "TRAILER",
         5: "MOTORCYCLE", 6: "BICYCLE", 7: "PEDESTRIAN"}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class D(Node):
    def __init__(self):
        super().__init__("dump_objects")
        self.d = {}
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 lambda m: self.d.__setitem__("odom", m), 1)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                                 lambda m: self.d.__setitem__("objs", m), 1)
        # The stage before tracking and prediction, to see whether the burst is born in
        # detection or invented downstream.
        for topic in ("/perception/object_recognition/detection/objects",
                      "/perception/object_recognition/detection/clustering/objects"):
            self.create_subscription(
                DetectedObjects, topic,
                (lambda t: lambda m: self.d.__setitem__(t, m))(topic), sensor)


def describe(obj, ex, ey, eyaw, predicted):
    k = obj.kinematics
    pose = (k.initial_pose_with_covariance.pose if predicted
            else k.pose_with_covariance.pose)
    p = pose.position
    dx, dy = p.x - ex, p.y - ey
    lon = dx * math.cos(eyaw) + dy * math.sin(eyaw)
    lat = -dx * math.sin(eyaw) + dy * math.cos(eyaw)
    cls = max(obj.classification, key=lambda c: c.probability, default=None)
    label = LABEL.get(cls.label, str(cls.label)) if cls else "-"
    prob = cls.probability if cls else float("nan")
    s = obj.shape.dimensions
    return (f"    lon={lon:6.1f} lat={lat:6.1f} d={math.hypot(lon, lat):5.1f} "
            f"{label:<10} p={prob:4.2f} exist={obj.existence_probability:4.2f} "
            f"size=({s.x:4.1f},{s.y:4.1f},{s.z:4.1f})")


rclpy.init()
n = D()
t0 = time.time()
last = 0.0
while time.time() - t0 < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - last < 1.0:
        continue
    last = time.time()
    odom, objs = n.d.get("odom"), n.d.get("objs")
    if odom is None or objs is None:
        continue
    ep = odom.pose.pose.position
    eyaw = yaw_of(odom.pose.pose.orientation)
    counts = []
    for t in ("/perception/object_recognition/detection/objects",
              "/perception/object_recognition/detection/clustering/objects"):
        m = n.d.get(t)
        counts.append(f"{t.rsplit('/', 2)[-2]}={len(m.objects) if m else '-'}")
    t = int(time.time() - t0)
    print(f"t={t:4d} ego({ep.x:7.1f},{ep.y:7.1f}) yaw={math.degrees(eyaw):7.1f} "
          f"tracked={len(objs.objects):3d} {' '.join(counts)}", flush=True)
    if len(objs.objects) >= BURST:
        for o in objs.objects:
            print(describe(o, ep.x, ep.y, eyaw, True), flush=True)
rclpy.shutdown()
