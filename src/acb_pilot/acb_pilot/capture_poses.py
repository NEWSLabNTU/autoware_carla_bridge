"""
Capture initial pose and goal pose from RViz and save them to a YAML file.

Subscribes to the same topics that Autoware's RViz adaptors use:
  - /initialpose (geometry_msgs/PoseWithCovarianceStamped) - "2D Pose Estimate" tool
  - /planning/mission_planning/goal (geometry_msgs/PoseStamped) - "2D Nav Goal" tool
  - /rviz/routing/rough_goal (geometry_msgs/PoseStamped) - Autoware RoutePanel

Workflow:
  1. Start this node
  2. In RViz, click "2D Pose Estimate" and place the initial pose on the map
  3. In RViz, click "2D Nav Goal" and place the goal pose on the map
  4. The node saves both poses to a YAML file and exits

ROS Parameters:
    output_file (str, required): Path to output YAML file

Usage:
    ros2 run acb_pilot capture_poses --ros-args -p output_file:=/path/to/poses.yaml

The output file is read by auto_drive:
    ros2 run acb_pilot auto_drive --ros-args -p poses_file:=/path/to/poses.yaml
"""

import sys
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def pose_to_dict(position, orientation):
    """Convert position + orientation to a flat dict."""
    return {
        "x": float(position.x),
        "y": float(position.y),
        "z": float(position.z),
        "qx": float(orientation.x),
        "qy": float(orientation.y),
        "qz": float(orientation.z),
        "qw": float(orientation.w),
    }


def covariance_to_dict(covariance):
    """Extract the 3 diagonal entries we care about (x, y, yaw)."""
    return {
        "xx": float(covariance[0]),
        "yy": float(covariance[7]),
        "yaw_yaw": float(covariance[35]),
    }


class CapturePosesNode(Node):
    """Captures initial pose and goal pose from RViz topics."""

    def __init__(self):
        super().__init__("capture_poses")

        # Declare ROS parameter
        self.declare_parameter("output_file", "")
        self.output_file = (
            self.get_parameter("output_file").get_parameter_value().string_value
        )
        if not self.output_file:
            self.get_logger().fatal("Required parameter 'output_file' not set")
            raise SystemExit(1)

        self.initial_pose = None
        self.initial_covariance = None
        self.goal_pose = None

        # /initialpose - RViz "2D Pose Estimate"
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._on_initial_pose,
            10,
        )

        # /planning/mission_planning/goal - RViz "2D Nav Goal"
        self.create_subscription(
            PoseStamped,
            "/planning/mission_planning/goal",
            self._on_goal_pose,
            10,
        )

        # /rviz/routing/rough_goal - Autoware RoutePanel
        self.create_subscription(
            PoseStamped,
            "/rviz/routing/rough_goal",
            self._on_goal_pose,
            10,
        )

        self.get_logger().info("Waiting for poses from RViz...")
        self.get_logger().info("  1. Use '2D Pose Estimate' to set initial pose")
        self.get_logger().info("  2. Use '2D Nav Goal' to set goal pose")
        self.get_logger().info(f"  Output: {self.output_file}")

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.initial_pose = pose_to_dict(pose.position, pose.orientation)
        self.initial_covariance = covariance_to_dict(msg.pose.covariance)
        self.get_logger().info(
            f"Initial pose captured: "
            f"({self.initial_pose['x']:.2f}, {self.initial_pose['y']:.2f}, "
            f"{self.initial_pose['z']:.2f})"
        )
        self._check_complete()

    def _on_goal_pose(self, msg: PoseStamped):
        pose = msg.pose
        self.goal_pose = pose_to_dict(pose.position, pose.orientation)
        self.get_logger().info(
            f"Goal pose captured: "
            f"({self.goal_pose['x']:.2f}, {self.goal_pose['y']:.2f}, "
            f"{self.goal_pose['z']:.2f})"
        )
        self._check_complete()

    def _check_complete(self):
        if self.initial_pose and self.goal_pose:
            self.get_logger().info("Both poses captured!")

    def is_complete(self) -> bool:
        return self.initial_pose is not None and self.goal_pose is not None

    def to_dict(self) -> dict:
        data = {
            "initial_pose": self.initial_pose,
            "goal_pose": self.goal_pose,
        }
        if self.initial_covariance:
            data["initial_covariance"] = self.initial_covariance
        return data

    def save(self):
        """Save captured poses to the output file."""
        data = self.to_dict()
        with open(self.output_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Poses saved to {self.output_file}")


def main():
    rclpy.init()
    node = CapturePosesNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.is_complete():
                break

        if not node.is_complete():
            node.get_logger().error("Interrupted before both poses were captured")
            sys.exit(1)

        node.save()

    except KeyboardInterrupt:
        # Save whatever we have if at least one pose was captured
        if node.initial_pose or node.goal_pose:
            node.save()
            node.get_logger().info("Interrupted - saved partial poses")
        else:
            node.get_logger().info("Interrupted - no poses captured")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
