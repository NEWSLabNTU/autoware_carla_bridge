#!/usr/bin/env python3
"""
Autonomous driving script for Autoware using standard rclpy API.
Reads poses from poses.json and drives the vehicle autonomously to the goal.

Updated for modern Autoware (2024/2025):
- Uses /api/localization/initialize service
- Uses /api/routing/set_route_points service
- Uses /api/routing/clear_route service
- Uses /api/operation_mode/change_to_autonomous service

IMPORTANT: The initial and goal poses MUST be on connected lanes in the lanelet2 map.
If route planning fails, the poses may not have a valid path between them.
Use RViz's "2D Pose Estimate" and "2D Goal Pose" buttons to select valid poses,
or use get_carla_spawn_points.py to find valid spawn points from CARLA.
"""

import json
import math
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute, ChangeOperationMode, InitializeLocalization
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry


def load_poses():
    """Load poses from JSON file in the same directory as this script."""
    poses_path = Path(__file__).parent / "poses.json"

    if not poses_path.exists():
        print(f"Error: poses.json not found at {poses_path}", file=sys.stderr)
        print("Please run read_poses.py first to capture initial and goal poses", file=sys.stderr)
        sys.exit(1)

    with open(poses_path, 'r') as f:
        poses = json.load(f)

    print(f"✓ Loaded poses from {poses_path}")
    return poses


def calculate_distance(x1, y1, x2, y2):
    """Calculate 2D distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class AutowareDriveNode(Node):
    """ROS 2 node for autonomous driving with Autoware."""

    def __init__(self, poses):
        super().__init__('autoware_drive_node')

        self.poses = poses
        self.initial_pose = poses['initial_pose']
        self.goal_pose = poses['goal_pose']

        # Vehicle state
        self.current_position = None
        self.current_velocity = 0.0
        self.route_state = None
        self.operation_mode = None

        # QoS profiles
        qos_transient_local = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        qos_volatile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Service clients - Modern Autoware API
        self.init_localization_client = self.create_client(
            InitializeLocalization,
            '/api/localization/initialize'
        )

        self.set_route_client = self.create_client(
            SetRoutePoints,
            '/api/routing/set_route_points'
        )

        self.clear_route_client = self.create_client(
            ClearRoute,
            '/api/routing/clear_route'
        )

        self.change_mode_client = self.create_client(
            ChangeOperationMode,
            '/api/operation_mode/change_to_autonomous'
        )

        # Subscribers
        self.velocity_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10
        )

        self.route_state_sub = self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.route_state_callback,
            qos_transient_local
        )

        self.operation_mode_sub = self.create_subscription(
            OperationModeState,
            '/api/operation_mode/state',
            self.operation_mode_callback,
            qos_transient_local
        )

        self.get_logger().info("Autoware Drive Node initialized")

    def velocity_callback(self, msg):
        """Update current velocity."""
        self.current_velocity = msg.longitudinal_velocity

    def odometry_callback(self, msg):
        """Update current position from odometry."""
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }

    def route_state_callback(self, msg):
        """Update route state."""
        self.route_state = msg.state

    def operation_mode_callback(self, msg):
        """Update operation mode."""
        self.operation_mode = msg.mode

    def initialize_localization(self):
        """Initialize localization using the Autoware API service."""
        if not self.init_localization_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Localization initialize service not available")
            return False

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = self.initial_pose['position']['x']
        pose_msg.pose.pose.position.y = self.initial_pose['position']['y']
        pose_msg.pose.pose.position.z = self.initial_pose['position']['z']

        pose_msg.pose.pose.orientation.x = self.initial_pose['orientation']['x']
        pose_msg.pose.pose.orientation.y = self.initial_pose['orientation']['y']
        pose_msg.pose.pose.orientation.z = self.initial_pose['orientation']['z']
        pose_msg.pose.pose.orientation.w = self.initial_pose['orientation']['w']

        # Set covariance (small values indicate high confidence)
        pose_msg.pose.covariance[0] = 0.25  # x variance
        pose_msg.pose.covariance[7] = 0.25  # y variance
        pose_msg.pose.covariance[35] = 0.06853891909122467  # yaw variance

        # Call service
        request = InitializeLocalization.Request()
        request.pose.append(pose_msg)  # pose is an array with max 1 element

        future = self.init_localization_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if result.status.success:
                self.get_logger().info("Localization initialized")
                return True
            else:
                self.get_logger().error(f"Failed to initialize localization: {result.status.message}")
                return False
        else:
            self.get_logger().error("Localization initialization service call failed")
            return False

    def clear_route(self):
        """Clear existing route."""
        if not self.clear_route_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Clear route service not available")
            return False

        request = ClearRoute.Request()
        future = self.clear_route_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if result.status.success:
                self.get_logger().info("Route cleared")
                return True
            else:
                self.get_logger().error(f"Failed to clear route: {result.status.message}")
                return False
        else:
            self.get_logger().error("Clear route service call failed")
            return False

    def set_route(self):
        """Set route to goal pose."""
        if not self.set_route_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set route service not available")
            return False

        from geometry_msgs.msg import Pose

        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'

        # Set goal pose (Pose, not PoseStamped)
        goal = Pose()
        goal.position.x = self.goal_pose['position']['x']
        goal.position.y = self.goal_pose['position']['y']
        goal.position.z = self.goal_pose['position']['z']
        goal.orientation.x = self.goal_pose['orientation']['x']
        goal.orientation.y = self.goal_pose['orientation']['y']
        goal.orientation.z = self.goal_pose['orientation']['z']
        goal.orientation.w = self.goal_pose['orientation']['w']

        request.goal = goal
        request.option.allow_goal_modification = True

        future = self.set_route_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            result = future.result()
            if result.status.success:
                self.get_logger().info("Route set successfully")
                return True
            else:
                self.get_logger().error(f"Failed to set route: {result.status.message}")
                return False
        else:
            self.get_logger().error("Set route service call failed")
            return False

    def set_autonomous_mode(self):
        """Change to autonomous operation mode."""
        if not self.change_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Change to autonomous service not available")
            return False

        # The service takes no parameters (empty request)
        request = ChangeOperationMode.Request()

        future = self.change_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if result.status.success:
                self.get_logger().info("Autonomous mode engaged")
                return True
            else:
                self.get_logger().error(f"Failed to engage autonomous mode: {result.status.message}")
                return False
        else:
            self.get_logger().error("Change operation mode service call failed")
            return False

    def wait_for_route_set(self, timeout_sec=10.0):
        """Wait for route to be set (state == 2)."""
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.route_state == 2:  # ROUTE_SET
                return True
        return False

    def monitor_progress(self):
        """Monitor vehicle progress until goal is reached."""
        goal_x = self.goal_pose['position']['x']
        goal_y = self.goal_pose['position']['y']

        if self.current_position:
            start_x = self.current_position['x']
            start_y = self.current_position['y']
            total_distance = calculate_distance(start_x, start_y, goal_x, goal_y)
        else:
            total_distance = None

        print("\n" + "=" * 60)
        print("MONITORING VEHICLE PROGRESS")
        print("=" * 60)

        start_time = time.time()
        last_update = start_time
        stuck_counter = 0

        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time

                # Update every 2 seconds
                if current_time - last_update >= 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)

                    if self.current_position:
                        curr_x = self.current_position['x']
                        curr_y = self.current_position['y']
                        distance_to_goal = calculate_distance(curr_x, curr_y, goal_x, goal_y)

                        if total_distance and total_distance > 0:
                            progress = ((total_distance - distance_to_goal) / total_distance) * 100
                        else:
                            progress = 0.0

                        print(
                            f"[{elapsed:5.1f}s] Speed: {self.current_velocity:.2f} m/s | "
                            f"Distance: {distance_to_goal:.1f}m | Progress: {progress:.1f}%"
                        )

                        # Check if arrived
                        if self.route_state == 3:  # ARRIVED
                            print("=" * 60)
                            print("\n✓ ARRIVED at goal!")
                            break

                        if distance_to_goal < 5.0:
                            print("=" * 60)
                            print(f"\n✓ Vehicle reached goal! Final distance: {distance_to_goal:.2f} m")
                            break

                        # Check if stuck
                        if abs(self.current_velocity) < 0.1:
                            stuck_counter += 1
                            if stuck_counter > 5:
                                print("⚠ Vehicle may be stuck (speed < 0.1 m/s)")
                        else:
                            stuck_counter = 0
                    else:
                        print(f"[{elapsed:5.1f}s] Waiting for position data...")

                    last_update = current_time

                # Timeout after 5 minutes
                if elapsed > 300:
                    print("\n⚠ Timeout: Did not reach goal within 5 minutes")
                    break

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")


def main():
    print("=" * 60)
    print("AUTOWARE AUTONOMOUS DRIVING")
    print("=" * 60)

    # Load poses
    print("\n1. Loading poses...")
    poses = load_poses()

    initial_pose = poses['initial_pose']
    goal_pose = poses['goal_pose']

    total_distance = calculate_distance(
        initial_pose['position']['x'],
        initial_pose['position']['y'],
        goal_pose['position']['x'],
        goal_pose['position']['y']
    )
    print(f"   Total distance to goal: {total_distance:.2f} meters")

    # Initialize ROS 2
    rclpy.init()

    # Create node
    node = AutowareDriveNode(poses)

    try:
        # Initialize localization
        print("\n2. Initializing localization...")
        if not node.initialize_localization():
            print("   Warning: Localization initialization may have failed")
        print("   Waiting for localization to stabilize...")
        time.sleep(5)

        # Wait for vehicle to spawn and diagnostics to stabilize
        print("   Waiting for vehicle spawn and diagnostics...")
        time.sleep(15)

        # Spin a few times to get initial state
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Clear route
        print("\n3. Clearing existing route...")
        if not node.clear_route():
            print("   Warning: Failed to clear route, continuing anyway...")
        time.sleep(1)

        # Set route
        print("\n4. Setting route to goal...")
        if not node.set_route():
            print("✗ Failed to set route")
            print("\nTroubleshooting:")
            print("  - The initial and goal poses may not be connected in the lanelet2 map")
            print("  - Try using RViz to select poses with '2D Pose Estimate' and '2D Goal Pose'")
            print("  - Or run ./scripts/get_carla_spawn_points.py to find valid CARLA spawn points")
            print("  - Check Autoware logs: just autoware logs -n 50")
            return False

        # Wait for route to be set
        print("   Waiting for route processing...")
        if not node.wait_for_route_set(timeout_sec=10.0):
            print("   Warning: Route may not be fully processed yet")
        else:
            print("   ✓ Route is set and ready")

        # Wait for planning and control pipelines to activate
        print("   Waiting for planning and control to initialize...")
        time.sleep(10)

        # Engage autonomous mode
        print("\n5. Engaging autonomous mode...")
        if not node.set_autonomous_mode():
            print("✗ Failed to engage autonomous mode")
            return False

        print("   ✓ AUTONOMOUS MODE ENGAGED")
        print("   Vehicle is now driving autonomously!")

        time.sleep(2)

        # Monitor progress
        node.monitor_progress()

        print("\n" + "=" * 60)
        print("AUTONOMOUS DRIVING COMPLETE")
        print("=" * 60)

        return True

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return False

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
