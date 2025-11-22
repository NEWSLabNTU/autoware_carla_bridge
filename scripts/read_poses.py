#!/usr/bin/env python3
"""Read initial pose and goal pose from RViz and save to file."""

import argparse
import json
import sys
from dataclasses import dataclass
from typing import Optional
from pathlib import Path


@dataclass
class Pose:
    """Pose with position and orientation (quaternion)."""
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    def to_dict(self):
        return {
            'position': {
                'x': self.x,
                'y': self.y,
                'z': self.z,
            },
            'orientation': {
                'x': self.qx,
                'y': self.qy,
                'z': self.qz,
                'w': self.qw,
            }
        }


def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    import math

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def main():
    parser = argparse.ArgumentParser(description="Read initial and goal poses from Autoware")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output JSON file (default: poses.json in script directory)")
    args = parser.parse_args()

    # Default output path is in the same directory as this script
    if args.output is None:
        output_path = Path(__file__).parent / "poses.json"
    else:
        output_path = Path(args.output)

    args.output = output_path

    # Import ROS 2 after parsing args
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSPresetProfiles
        from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    except ImportError as e:
        print(f"Error: Failed to import ROS 2 Python libraries: {e}", file=sys.stderr)
        print("Make sure ROS 2 is sourced: source /opt/ros/humble/setup.bash", file=sys.stderr)
        sys.exit(1)

    # Initialize ROS 2
    rclpy.init()

    # Create node
    node = Node('pose_reader')

    # Store poses
    initial_pose: Optional[Pose] = None
    goal_pose: Optional[Pose] = None

    def initial_pose_callback(msg):
        nonlocal initial_pose
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        initial_pose = Pose(
            x=pos.x, y=pos.y, z=pos.z,
            qx=orient.x, qy=orient.y, qz=orient.z, qw=orient.w
        )
        node.get_logger().info(f"✓ Received initial pose: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        print(f"✓ Received initial pose: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")

    def goal_pose_callback(msg, source="unknown"):
        nonlocal goal_pose
        pos = msg.pose.position
        orient = msg.pose.orientation
        goal_pose = Pose(
            x=pos.x, y=pos.y, z=pos.z,
            qx=orient.x, qy=orient.y, qz=orient.z, qw=orient.w
        )
        node.get_logger().info(f"✓ Received goal pose from {source}: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        print(f"✓ Received goal pose from {source}: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")

    try:
        # Use default QoS preset for system topics (matches RViz)
        initial_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribe to goal pose from mission planning
        goal_sub = node.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal',
            lambda msg: goal_pose_callback(msg, "mission_planning"),
            QoSPresetProfiles.SERVICES_DEFAULT.value
        )

        print("Subscriptions created successfully")
        print("Waiting for initial pose and goal pose...")
        print()
        print("Please set poses in RViz:")
        print("  1. Click '2D Pose Estimate' and click+drag on the map")
        print("  2. Click '2D Goal Pose' and click+drag on the map")
        print()

        # Spin until we receive both poses
        while not (initial_pose and goal_pose):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Convert to output format
        output = {}

        if initial_pose:
            roll, pitch, yaw = euler_from_quaternion(
                initial_pose.qx, initial_pose.qy, initial_pose.qz, initial_pose.qw
            )
            import math
            output['initial_pose'] = {
                **initial_pose.to_dict(),
                'euler': {
                    'roll': math.degrees(roll),
                    'pitch': math.degrees(pitch),
                    'yaw': math.degrees(yaw),
                }
            }
            print(f"\nInitial Pose:")
            print(f"  Position: ({initial_pose.x:.3f}, {initial_pose.y:.3f}, {initial_pose.z:.3f})")
            print(f"  Quaternion: ({initial_pose.qx:.4f}, {initial_pose.qy:.4f}, {initial_pose.qz:.4f}, {initial_pose.qw:.4f})")
            print(f"  Euler (deg): roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f}")

        if goal_pose:
            roll, pitch, yaw = euler_from_quaternion(
                goal_pose.qx, goal_pose.qy, goal_pose.qz, goal_pose.qw
            )
            import math
            output['goal_pose'] = {
                **goal_pose.to_dict(),
                'euler': {
                    'roll': math.degrees(roll),
                    'pitch': math.degrees(pitch),
                    'yaw': math.degrees(yaw),
                }
            }
            print(f"\nGoal Pose:")
            print(f"  Position: ({goal_pose.x:.3f}, {goal_pose.y:.3f}, {goal_pose.z:.3f})")
            print(f"  Quaternion: ({goal_pose.qx:.4f}, {goal_pose.qy:.4f}, {goal_pose.qz:.4f}, {goal_pose.qw:.4f})")
            print(f"  Euler (deg): roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f}")

        # Write to file
        with open(args.output, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"\n✓ Poses saved to {args.output}")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
