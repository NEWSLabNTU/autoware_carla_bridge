#!/usr/bin/env python3
"""Check the acb_bridge vehicle interface against CARLA ground truth, live.

Run inside the ego's ROS domain while a scenario is driving. Subscribes to the
/vehicle/status/* topics and the IMU, samples CARLA directly for the same quantities,
and reports whether they agree.

  python3 check_vehicle_interface.py [--seconds 30] [--carla-port 2000]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport,
)
from sensor_msgs.msg import Imu
from tier4_vehicle_msgs.msg import ActuationStatusStamped

import carla


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class Checker(Node):
    def __init__(self, port):
        super().__init__("acb_vehicle_interface_check")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.latest = {}
        self.counts = {}

        def sub(topic, msg_type, key, q=qos):
            def cb(msg, key=key):
                self.latest[key] = msg
                self.counts[key] = self.counts.get(key, 0) + 1
            self.create_subscription(msg_type, topic, cb, q)
            self.counts.setdefault(key, 0)

        sub("/vehicle/status/velocity_status", VelocityReport, "velocity")
        sub("/vehicle/status/steering_status", SteeringReport, "steering")
        sub("/vehicle/status/gear_status", GearReport, "gear")
        sub("/vehicle/status/control_mode", ControlModeReport, "control_mode")
        sub("/vehicle/status/turn_indicators_status", TurnIndicatorsReport, "turn")
        sub("/vehicle/status/hazard_lights_status", HazardLightsReport, "hazard")
        sub("/vehicle/status/actuation_status", ActuationStatusStamped, "actuation")
        sub("/control/command/gear_cmd", GearCommand, "gear_cmd")
        sub("/sensing/imu/carla/imu_link/imu_raw", Imu, "imu", sensor_qos)

        self.client = carla.Client("localhost", port)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()
        self.vehicle = None
        self.imu = None

    def find_imu(self, vehicle):
        if self.imu is not None:
            try:
                self.imu.get_transform()
                return self.imu
            except RuntimeError:
                self.imu = None
        for actor in self.world.get_actors().filter("sensor.other.imu"):
            if actor.parent is not None and actor.parent.id == vehicle.id:
                self.imu = actor
                return actor
        return None

    def find_hero(self):
        if self.vehicle is not None:
            try:
                self.vehicle.get_transform()
                return self.vehicle
            except RuntimeError:
                self.vehicle = None
        # Re-fetch the world every poll: a map reload starts a NEW episode, and a handle
        # from before it keeps answering from the dead one -- the same trap acb_bridge's
        # wait_for_vehicle documents.
        try:
            self.world = self.client.get_world()
        except RuntimeError:
            return None
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == "hero":
                self.vehicle = actor
                return actor
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--carla-port", type=int, default=2000)
    args = parser.parse_args()

    rclpy.init()
    node = Checker(args.carla_port)

    samples = []
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        vehicle = node.find_hero()
        if vehicle is None or "velocity" not in node.latest:
            continue

        try:
            tf = vehicle.get_transform()
            v = vehicle.get_velocity()
            av = vehicle.get_angular_velocity()
            fwd = tf.get_forward_vector()
            right = tf.get_right_vector()

            truth_lon = v.x * fwd.x + v.y * fwd.y + v.z * fwd.z
            truth_lat = -(v.x * right.x + v.y * right.y + v.z * right.z)
            truth_heading_rate = -math.radians(av.z)
            truth_ros_yaw = -math.radians(tf.rotation.yaw)

            report = node.latest["velocity"]
            row = {
                "lon_report": report.longitudinal_velocity,
                "lon_truth": truth_lon,
                "lat_report": report.lateral_velocity,
                "lat_truth": truth_lat,
                "rate_report": report.heading_rate,
                "rate_truth": truth_heading_rate,
                "speed": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
            }
            if "imu" in node.latest:
                row["imu_yaw"] = quat_yaw(node.latest["imu"].orientation)
                row["yaw_truth"] = truth_ros_yaw
                # The IMU is mounted with a yaw offset -- base_link -> carla/imu_link is
                # RPY [-3.141, -0.015, 3.105] in this sensor kit -- so CARLA's compass,
                # and therefore the published orientation, is the SENSOR's heading, not
                # the vehicle's. Comparing against the vehicle shows a ~3.105 rad error
                # that is entirely the mount. The sensor actor's own transform is the
                # right reference.
                imu_actor = node.find_imu(vehicle)
                if imu_actor is not None:
                    row["imu_yaw_sensor_truth"] = -math.radians(
                        imu_actor.get_transform().rotation.yaw
                    )
            if "steering" in node.latest:
                row["steer_report"] = node.latest["steering"].steering_tire_angle
                fl = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
                fr = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
                row["steer_truth"] = -math.radians(0.5 * (fl + fr))
        except RuntimeError:
            # The scenario despawned the ego mid-sample; wait for the next spawn.
            node.vehicle = None
            continue
        samples.append(row)

    print("=== topic liveness ===")
    for key in sorted(node.counts):
        state = "OK " if node.counts[key] else "NONE"
        print(f"  {state} {key:12s} {node.counts[key]:6d} msgs")

    if not samples:
        print("\nNo samples collected — is the hero vehicle spawned and the bridge running?")
        rclpy.shutdown()
        return 1

    moving = [s for s in samples if s["speed"] > 0.5]
    print(f"\n=== {len(samples)} samples, {len(moving)} while moving (>0.5 m/s) ===")

    def report_error(name, key_r, key_t, unit, source=None):
        rows = [s for s in (source or moving) if key_r in s and key_t in s]
        if not rows:
            print(f"  {name:22s} no samples")
            return
        errs = [abs(s[key_r] - s[key_t]) for s in rows]
        worst = max(errs)
        mean = sum(errs) / len(errs)
        print(f"  {name:22s} mean |err| {mean:8.4f} {unit}  worst {worst:8.4f} {unit}"
              f"   (n={len(rows)})")

    def angle_error(name, key_r, key_t):
        rows = [s for s in samples if key_r in s and key_t in s]
        if not rows:
            print(f"  {name:22s} no samples")
            return
        errs = [abs((s[key_r] - s[key_t] + math.pi) % (2 * math.pi) - math.pi) for s in rows]
        print(f"  {name:22s} mean |err| {sum(errs)/len(errs):8.4f} rad  "
              f"worst {max(errs):8.4f} rad   (n={len(rows)})")

    print("\n=== VelocityReport vs CARLA body-frame truth ===")
    report_error("longitudinal", "lon_report", "lon_truth", "m/s")
    report_error("lateral", "lat_report", "lat_truth", "m/s")
    report_error("heading_rate", "rate_report", "rate_truth", "rad/s")

    print("\n=== IMU orientation vs CARLA yaw ===")
    angle_error("imu yaw vs base_link", "imu_yaw", "yaw_truth")
    angle_error("imu yaw vs sensor tf", "imu_yaw", "imu_yaw_sensor_truth")
    yaws = [s["imu_yaw"] for s in samples if "imu_yaw" in s]
    if yaws:
        print(f"  imu yaw spread        {max(yaws) - min(yaws):8.4f} rad "
              f"(a constant orientation would be 0)")

    print("\n=== SteeringReport vs measured wheel angle ===")
    report_error("steering_tire_angle", "steer_report", "steer_truth", "rad", source=samples)

    print("\n=== last reported states ===")
    for key in ("gear", "turn", "hazard", "control_mode"):
        msg = node.latest.get(key)
        if msg is None:
            print(f"  {key:12s} (never received)")
        else:
            field = "report" if hasattr(msg, "report") else "mode"
            print(f"  {key:12s} {getattr(msg, field)}")
    if "gear_cmd" in node.latest:
        print(f"  gear_cmd     {node.latest['gear_cmd'].command} "
              f"(gear_status should match)")
    if "actuation" in node.latest:
        s = node.latest["actuation"].status
        print(f"  actuation    accel={s.accel_status:.3f} brake={s.brake_status:.3f} "
              f"steer={s.steer_status:.3f}")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
