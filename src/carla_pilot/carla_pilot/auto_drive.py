"""
Autonomous driving sequence for Autoware.

Initializes localization, sets a route, engages autonomous mode, and waits
until the vehicle arrives at the goal. Uses the Autoware AD API (v1) services
and topics.

ROS Parameters:
    poses_file (str, required): Path to YAML file with initial_pose and goal_pose
    timeout (float, default 120.0): Overall timeout in seconds
    wait_for_arrival (bool, default true): Wait for arrival or exit after engaging

Usage:
    ros2 run carla_pilot auto_drive --ros-args -p poses_file:=/path/to/poses.yaml
    ros2 run carla_pilot auto_drive --ros-args -p poses_file:=/path/to/poses.yaml -p timeout:=180.0
    ros2 run carla_pilot auto_drive --ros-args -p poses_file:=/path/to/poses.yaml -p wait_for_arrival:=false

Requires:
    - ROS 2 with Autoware message packages
    - Autoware planning simulator running
    - A poses YAML file (see config/example_poses.yaml for the format)
"""

import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_adapi_v1_msgs.srv import (
    InitializeLocalization,
    SetRoutePoints,
    ChangeOperationMode,
)
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState


def load_poses(poses_file):
    """Load poses from a YAML file (as produced by capture_poses).

    Required keys: initial_pose, goal_pose.
    Optional key: initial_covariance (defaults to reasonable values).
    """
    with open(poses_file) as f:
        data = yaml.safe_load(f)
    if "initial_pose" not in data:
        raise ValueError(f"Missing 'initial_pose' in {poses_file}")
    if "goal_pose" not in data:
        raise ValueError(f"Missing 'goal_pose' in {poses_file}")
    initial = data["initial_pose"]
    goal = data["goal_pose"]
    cov = data.get("initial_covariance", {"xx": 0.25, "yy": 0.25, "yaw_yaw": 0.0685})
    return initial, goal, cov


def make_covariance_array(cov_dict):
    """Build a 36-element covariance array from the diagonal entries."""
    arr = [0.0] * 36
    arr[0] = cov_dict.get("xx", 0.25)
    arr[7] = cov_dict.get("yy", 0.25)
    arr[35] = cov_dict.get("yaw_yaw", 0.0685)
    return arr


class AutoDriveNode(Node):
    """Automates the autonomous driving sequence via Autoware AD API."""

    def __init__(self):
        super().__init__("auto_drive")

        # Declare ROS parameters
        self.declare_parameter("poses_file", "")
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("wait_for_arrival", True)

        poses_file = self.get_parameter("poses_file").get_parameter_value().string_value
        if not poses_file:
            self.get_logger().fatal("Required parameter 'poses_file' not set")
            raise SystemExit(1)

        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.wait_for_arrival = (
            self.get_parameter("wait_for_arrival").get_parameter_value().bool_value
        )

        # Load poses
        initial, goal, cov = load_poses(poses_file)
        self.get_logger().info(f"Loaded poses from {poses_file}")

        self.initial_pose = initial
        self.goal_pose = goal
        self.covariance_arr = make_covariance_array(cov)
        self.route_state = RouteState.UNKNOWN
        self.op_mode_state = None

        # QoS for state topics (transient local to get last published value)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers for monitoring state
        self.route_state_sub = self.create_subscription(
            RouteState, "/api/routing/state", self._on_route_state, state_qos
        )
        self.op_mode_sub = self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_op_mode_state,
            state_qos,
        )

        # Service clients
        self.init_loc_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.set_route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.change_to_auto_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )

        # Also publish to /initialpose as a fallback (some setups don't have
        # the localization initialize service but do have the topic adaptor)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

    def _on_route_state(self, msg: RouteState):
        self.route_state = msg.state

    def _on_op_mode_state(self, msg: OperationModeState):
        self.op_mode_state = msg

    def _spin_for(self, seconds: float):
        """Spin the node for a duration, processing callbacks."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_service(self, client, name: str, timeout: float = 30.0) -> bool:
        """Wait for a service to become available."""
        self.get_logger().info(f"Waiting for service {name}...")
        if client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"  {name} available")
            return True
        self.get_logger().warn(f"  {name} not available after {timeout}s")
        return False

    def _call_service(self, client, request, name: str, timeout: float = 30.0):
        """Call a service and wait for the response."""
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f"  {name} call timed out after {timeout}s")
            return None
        return future.result()

    def _build_pose_msg(self) -> PoseWithCovarianceStamped:
        """Build PoseWithCovarianceStamped from initial pose data."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_pose["x"]
        msg.pose.pose.position.y = self.initial_pose["y"]
        msg.pose.pose.position.z = self.initial_pose["z"]
        msg.pose.pose.orientation.x = self.initial_pose["qx"]
        msg.pose.pose.orientation.y = self.initial_pose["qy"]
        msg.pose.pose.orientation.z = self.initial_pose["qz"]
        msg.pose.pose.orientation.w = self.initial_pose["qw"]
        msg.pose.covariance = self.covariance_arr
        return msg

    def step1_initialize_localization(self) -> bool:
        """Set the initial pose via AD API and /initialpose topic."""
        self.get_logger().info("=== Step 1: Initialize localization ===")

        pose_msg = self._build_pose_msg()

        # Publish to /initialpose topic (for adaptor-based setups)
        self.get_logger().info("  Publishing to /initialpose topic...")
        for _ in range(5):
            self.initial_pose_pub.publish(pose_msg)
            self._spin_for(0.5)

        # Also try the service API
        if self._wait_for_service(
            self.init_loc_client, "/api/localization/initialize", timeout=5.0
        ):
            req = InitializeLocalization.Request()
            req.pose = [pose_msg]
            resp = self._call_service(
                self.init_loc_client, req, "InitializeLocalization"
            )
            if resp and resp.status.success:
                self.get_logger().info("  Localization initialized via service")
                return True
            elif resp:
                self.get_logger().warn(
                    f"  Service returned: success={resp.status.success}, "
                    f"msg={resp.status.message}"
                )
        else:
            self.get_logger().info(
                "  Service not available, relying on /initialpose topic"
            )

        return True  # Topic was published; localization may still converge

    def step2_wait_for_localization(self, timeout: float = 30.0) -> bool:
        """Wait for localization to converge (operation mode state received)."""
        self.get_logger().info(
            f"=== Step 2: Waiting for localization ({timeout}s max) ==="
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.op_mode_state is not None:
                self.get_logger().info(
                    f"  Operation mode received: mode={self.op_mode_state.mode}"
                )
                return True
        self.get_logger().warn("  Timed out waiting for operation mode state")
        return True  # Continue anyway

    def step3_set_route(self) -> bool:
        """Set the route via AD API."""
        self.get_logger().info("=== Step 3: Set route ===")

        if not self._wait_for_service(
            self.set_route_client, "/api/routing/set_route_points"
        ):
            return False

        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.header.stamp = self.get_clock().now().to_msg()
        req.option.allow_goal_modification = False
        req.goal.position.x = self.goal_pose["x"]
        req.goal.position.y = self.goal_pose["y"]
        req.goal.position.z = self.goal_pose["z"]
        req.goal.orientation.x = self.goal_pose["qx"]
        req.goal.orientation.y = self.goal_pose["qy"]
        req.goal.orientation.z = self.goal_pose["qz"]
        req.goal.orientation.w = self.goal_pose["qw"]
        req.waypoints = []

        resp = self._call_service(self.set_route_client, req, "SetRoutePoints")
        if resp and resp.status.success:
            self.get_logger().info("  Route set successfully")
            return True

        if resp:
            self.get_logger().warn(
                f"  Route failed: success={resp.status.success}, "
                f"code={resp.status.code}, msg={resp.status.message}"
            )
            # Retry once after a delay
            self.get_logger().info("  Retrying in 5s...")
            self._spin_for(5.0)
            resp = self._call_service(self.set_route_client, req, "SetRoutePoints")
            if resp and resp.status.success:
                self.get_logger().info("  Route set on retry")
                return True

        self.get_logger().error("  Failed to set route")
        return False

    def step4_wait_for_route(self, timeout: float = 15.0) -> bool:
        """Wait for route state to become SET."""
        self.get_logger().info(
            f"=== Step 4: Waiting for route to be planned ({timeout}s max) ==="
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.route_state == RouteState.SET:
                self.get_logger().info("  Route state: SET")
                return True
            elif self.route_state == RouteState.ARRIVED:
                self.get_logger().info("  Route state: ARRIVED (already at goal?)")
                return True
        self.get_logger().warn(
            f"  Route state still {self.route_state} after {timeout}s"
        )
        return self.route_state == RouteState.SET

    def step5_engage_autonomous(self) -> bool:
        """Change to autonomous mode via AD API."""
        self.get_logger().info("=== Step 5: Engage autonomous mode ===")

        if not self._wait_for_service(
            self.change_to_auto_client, "/api/operation_mode/change_to_autonomous"
        ):
            return False

        req = ChangeOperationMode.Request()
        resp = self._call_service(
            self.change_to_auto_client, req, "ChangeOperationMode"
        )
        if resp and resp.status.success:
            self.get_logger().info("  Autonomous mode engaged")
            return True

        if resp:
            self.get_logger().warn(
                f"  Engage failed: success={resp.status.success}, "
                f"code={resp.status.code}, msg={resp.status.message}"
            )
            # Retry - may fail if not ready yet
            self.get_logger().info("  Retrying in 3s...")
            self._spin_for(3.0)
            resp = self._call_service(
                self.change_to_auto_client, req, "ChangeOperationMode"
            )
            if resp and resp.status.success:
                self.get_logger().info("  Autonomous mode engaged on retry")
                return True

        self.get_logger().error("  Failed to engage autonomous mode")
        return False

    def step6_wait_for_arrival(self, timeout: float = 120.0) -> bool:
        """Wait until route state becomes ARRIVED or timeout."""
        self.get_logger().info(
            f"=== Step 6: Waiting for arrival ({timeout}s max) ==="
        )
        start = time.monotonic()
        end = start + timeout
        last_log = start
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.monotonic()
            if self.route_state == RouteState.ARRIVED:
                elapsed = now - start
                self.get_logger().info(f"  ARRIVED at goal after {elapsed:.1f}s")
                return True
            # Log progress every 10s
            if now - last_log >= 10.0:
                elapsed = now - start
                mode = self.op_mode_state.mode if self.op_mode_state else "?"
                self.get_logger().info(
                    f"  Driving... {elapsed:.0f}s elapsed, "
                    f"route_state={self.route_state}, op_mode={mode}"
                )
                last_log = now

        self.get_logger().warn(f"  Did not arrive within {timeout}s")
        return False

    def run(self) -> bool:
        """Execute the full autonomous driving sequence."""
        self.get_logger().info(
            f"Auto-drive sequence starting (timeout={self.timeout}s)"
        )
        overall_start = time.monotonic()

        if not self.step1_initialize_localization():
            return False

        if not self.step2_wait_for_localization(timeout=30.0):
            return False

        if not self.step3_set_route():
            return False

        if not self.step4_wait_for_route(timeout=15.0):
            return False

        if not self.step5_engage_autonomous():
            return False

        remaining = self.timeout - (time.monotonic() - overall_start)
        arrived = self.step6_wait_for_arrival(timeout=max(remaining, 10.0))

        total = time.monotonic() - overall_start
        self.get_logger().info(
            f"Auto-drive sequence {'completed' if arrived else 'timed out'} "
            f"in {total:.1f}s"
        )
        return arrived


def main():
    rclpy.init()
    node = AutoDriveNode()

    try:
        if not node.wait_for_arrival:
            # Run steps 1-5 only
            node.step1_initialize_localization()
            node.step2_wait_for_localization(timeout=30.0)
            ok = node.step3_set_route()
            if ok:
                node.step4_wait_for_route(timeout=15.0)
                node.step5_engage_autonomous()
            node.get_logger().info(
                "Autonomous mode engaged, exiting (wait_for_arrival=false)"
            )
        else:
            arrived = node.run()
            if not arrived:
                sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
