"""
Autonomous driving sequence for Autoware.

Waits for Autoware services, then waits for GNSS-driven localization to
complete automatically, sets a route from a per-map goal poses file, engages
autonomous mode, and exits when the vehicle arrives at the goal.

No manual localization initialization is needed - GNSS data flows from CARLA
through gnss_poser to autoware_automatic_pose_initializer_node automatically.

ROS Parameters:
    poses_file (str, required): Path to YAML file with a 'goal_pose' key
    timeout (float, default 600.0): Overall timeout in seconds
    stabilize_seconds (float, default 15.0): Settling time after localization comes
        up, before routing. Pure latency — on a stack driven by a scenario's clock it
        is spent out of that scenario's budget.
    route_timeout (float, default 120.0): How long to keep retrying the route.

Usage:
    ros2 run acb_pilot auto_drive --ros-args -p poses_file:=/path/to/Town01.yaml

Poses file format:
    goal_pose:
      x: 153.6
      y: -36.3
      z: 0.0
      qx: 0.0
      qy: 0.0
      qz: -0.614
      qw: 0.789
"""

import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry

from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    RouteState,
    OperationModeState,
)
from autoware_adapi_v1_msgs.srv import (
    SetRoutePoints,
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
)


_LOC_STATE_NAMES = {
    LocalizationInitializationState.UNKNOWN: "UNKNOWN",
    LocalizationInitializationState.UNINITIALIZED: "UNINITIALIZED",
    LocalizationInitializationState.INITIALIZING: "INITIALIZING",
    LocalizationInitializationState.INITIALIZED: "INITIALIZED",
}


def load_goal_pose(poses_file: str) -> dict:
    with open(poses_file) as f:
        data = yaml.safe_load(f)
    if "goal_pose" not in data:
        raise ValueError(f"Missing 'goal_pose' in {poses_file}")
    return data["goal_pose"]


class AutoDriveNode(Node):
    """Automates the full autonomous driving sequence via Autoware AD API."""

    def __init__(self):
        super().__init__("auto_drive")

        self.declare_parameter("poses_file", "")
        self.declare_parameter("timeout", 600.0)
        # Time to let diagnostics and the planning pipeline settle once localization
        # is up. A parameter because it is pure latency: on a stack driven by a
        # scenario's clock it is spent out of the scenario's own budget.
        self.declare_parameter("stabilize_seconds", 15.0)
        # How long to keep retrying the route before giving up. Routing can fail for
        # a while after a respawn -- the map, the pose and the planner all have to
        # agree first -- so this is a deadline, not a retry count.
        self.declare_parameter("route_timeout", 120.0)

        poses_file = self.get_parameter("poses_file").get_parameter_value().string_value
        if not poses_file:
            self.get_logger().fatal("Required parameter 'poses_file' not set")
            raise SystemExit(1)

        self.goal_pose = load_goal_pose(poses_file)
        self.get_logger().info(f"Loaded goal pose from {poses_file}")

        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.stabilize_seconds = (
            self.get_parameter("stabilize_seconds").get_parameter_value().double_value
        )
        self.route_timeout = (
            self.get_parameter("route_timeout").get_parameter_value().double_value
        )

        # State
        self.localization_state = LocalizationInitializationState.UNKNOWN
        self.route_state = RouteState.UNKNOWN
        self.op_mode_state = None
        # Last pose from the EKF, and when it arrived. The initialization_state topic
        # is latched, so on a stack that has already run once it reports INITIALIZED
        # before this node even subscribes -- see step2 for why that is a lie.
        self.kinematic_pose = None
        self.kinematic_stamp = None

        # QoS for AD API state topics (transient local = get last value on subscribe)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.loc_state_sub = self.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self._on_localization_state,
            state_qos,
        )
        self.route_state_sub = self.create_subscription(
            RouteState,
            "/api/routing/state",
            self._on_route_state,
            state_qos,
        )
        self.op_mode_sub = self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_op_mode_state,
            state_qos,
        )

        self.clear_route_client = self.create_client(
            ClearRoute, "/api/routing/clear_route"
        )
        self.set_route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.change_to_auto_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        self.initialize_localization_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )

        # The EKF output, used to tell a live localization from a latched claim of one.
        self.kinematic_sub = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._on_kinematic_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_localization_state(self, msg: LocalizationInitializationState):
        self.localization_state = msg.state

    def _on_route_state(self, msg: RouteState):
        self.route_state = msg.state

    def _on_op_mode_state(self, msg: OperationModeState):
        self.op_mode_state = msg

    def _on_kinematic_state(self, msg: Odometry):
        self.kinematic_pose = msg.pose.pose
        self.kinematic_stamp = time.monotonic()

    def _request_localization_reinit(self) -> bool:
        """Ask Autoware to localize again from GNSS.

        An empty pose array means "use the GNSS estimate", which is the same path
        the automatic initializer takes on a cold start. Sending it is how the pilot
        forces a pose that belongs to the vehicle in front of it right now.
        """
        if not self.initialize_localization_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                "  /api/localization/initialize not available; "
                "cannot re-localize on this stack"
            )
            return False
        resp = self._call_service(
            self.initialize_localization_client,
            InitializeLocalization.Request(),
            "InitializeLocalization",
        )
        if resp is None:
            return False
        if not resp.status.success:
            self.get_logger().warn(
                f"  Re-initialization refused: {resp.status.message}"
            )
            return False
        self.get_logger().info("  Re-initialization requested (GNSS)")
        return True

    def _spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_service(self, client, name: str, timeout: float = 60.0) -> bool:
        self.get_logger().info(f"  Waiting for {name}...")
        if client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"  {name} ready")
            return True
        self.get_logger().error(f"  {name} not available after {timeout}s")
        return False

    def _call_service(self, client, request, name: str, timeout: float = 30.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f"  {name} call timed out after {timeout}s")
            return None
        return future.result()

    def step1_wait_for_services(self) -> bool:
        """Wait for Autoware AD API services to become callable."""
        self.get_logger().info("=== Step 1: Waiting for Autoware AD API services ===")
        ok = self._wait_for_service(
            self.clear_route_client, "/api/routing/clear_route"
        )
        ok = ok and self._wait_for_service(
            self.set_route_client, "/api/routing/set_route_points"
        )
        ok = ok and self._wait_for_service(
            self.change_to_auto_client, "/api/operation_mode/change_to_autonomous"
        )
        return ok

    def step2_wait_for_localization(self, timeout: float) -> bool:
        """Get a localization that belongs to the vehicle this pilot is driving.

        Localization is normally triggered by GNSS data flowing through gnss_poser ->
        autoware_automatic_pose_initializer_node -> NDT align, and on a cold stack
        waiting for `/api/localization/initialization_state` to read INITIALIZED is
        enough.

        It is not enough on a stack that has already driven once. That topic is
        latched, so it still reads INITIALIZED from the *previous* vehicle -- which
        was destroyed at the end of the last scenario and respawned somewhere else.
        The pilot then skips this step entirely and routes from a pose that belongs
        to nothing, and every attempt comes back "The planned route is empty" with no
        hint of why. Nothing re-triggers the automatic initializer either: it fires on
        the UNINITIALIZED -> INITIALIZED edge, and the state never left INITIALIZED.

        So: if localization already claims to be up when this node starts, ask for it
        again explicitly, and wait for the answer to arrive as a *fresh* EKF pose
        rather than a retained flag.
        """
        self.get_logger().info(
            f"=== Step 2: Waiting for localization (up to {timeout:.0f}s) ==="
        )
        end = time.monotonic() + timeout

        # Let the latched states land before deciding what we are looking at.
        self._spin_for(2.0)
        if self.localization_state == LocalizationInitializationState.INITIALIZED:
            self.get_logger().info(
                "  Localization already reports INITIALIZED — this stack has run "
                "before, so the pose may belong to a vehicle that no longer exists. "
                "Re-initializing."
            )
            self.kinematic_pose = None
            self.kinematic_stamp = None
            self._request_localization_reinit()

        last_log = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            initialized = (
                self.localization_state == LocalizationInitializationState.INITIALIZED
            )
            # A fresh EKF pose is the evidence that the estimate is live and belongs
            # to the vehicle in front of us; the state flag alone is not.
            fresh = self.kinematic_stamp is not None and (
                time.monotonic() - self.kinematic_stamp < 2.0
            )
            if initialized and fresh:
                p = self.kinematic_pose.position
                self.get_logger().info(
                    f"  Localization: INITIALIZED at ({p.x:.2f}, {p.y:.2f})"
                )
                # Let diagnostics and the planning pipeline settle before routing.
                # Without this, engage fails with "target mode not available".
                if self.stabilize_seconds > 0.0:
                    self.get_logger().info(
                        f"  Waiting {self.stabilize_seconds:.0f}s for system to stabilize..."
                    )
                    self._spin_for(self.stabilize_seconds)
                return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                name = _LOC_STATE_NAMES.get(
                    self.localization_state, str(self.localization_state)
                )
                self.get_logger().info(
                    f"  Localization state: {name}"
                    + ("" if fresh else " (no fresh pose yet)")
                )
                last_log = now
        self.get_logger().error(
            f"  Localization did not reach INITIALIZED with a live pose within "
            f"{timeout:.0f}s"
        )
        return False

    def step3_set_route(self) -> bool:
        """Set route to the goal pose via AD API."""
        self.get_logger().info("=== Step 3: Setting route ===")

        # Always clear route first — a stale route from a previous run blocks set_route
        self.get_logger().info("  Clearing any existing route...")
        clear_resp = self._call_service(
            self.clear_route_client, ClearRoute.Request(), "ClearRoute"
        )
        if clear_resp:
            self.get_logger().info(f"  Clear response: success={clear_resp.status.success}")
        # Wait for route state to leave SET/ARRIVED before setting a new one
        self._spin_for(2.0)

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

        # Retry to a deadline rather than a fixed count: after a respawn the pose, the
        # map and the planner take a variable amount of time to agree, and three
        # attempts five seconds apart used to expire while the stack was still
        # settling -- indistinguishable, from the log, from a goal that is genuinely
        # off the routing graph. Halfway through, ask for localization again: a stale
        # pose is the one cause of "The planned route is empty" the pilot can fix.
        deadline = time.monotonic() + self.route_timeout
        reinit_at = time.monotonic() + self.route_timeout / 2.0
        reinitialized = False
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            resp = self._call_service(self.set_route_client, req, "SetRoutePoints")
            if resp and resp.status.success:
                self.get_logger().info(f"  Route set successfully (attempt {attempt})")
                return True
            if resp:
                self.get_logger().warn(
                    f"  Route failed (attempt {attempt}): {resp.status.message}"
                )
            if not reinitialized and time.monotonic() >= reinit_at:
                reinitialized = True
                self.get_logger().info(
                    "  Still no route — asking Autoware to re-localize, in case the "
                    "pose is stale"
                )
                self._request_localization_reinit()
                self._spin_for(10.0)
            self._spin_for(5.0)

        self.get_logger().error(
            f"  Failed to set route within {self.route_timeout:.0f}s ({attempt} attempts). "
            f"If the message was \"The planned route is empty\", check the goal against "
            f"the lanelet2 map: it must be on a subtype=road lanelet, ahead of the "
            f"vehicle along that lanelet's own direction."
        )
        return False

    def step4_wait_for_route(self, timeout: float = 15.0) -> bool:
        """Wait for /api/routing/state = SET."""
        self.get_logger().info(
            f"=== Step 4: Waiting for route to be planned ({timeout}s max) ==="
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.route_state == RouteState.SET:
                self.get_logger().info("  Route state: SET")
                # Wait for planning/control pipeline to activate before engaging
                self.get_logger().info("  Waiting 10s for planning pipeline to activate...")
                self._spin_for(10.0)
                return True
            if self.route_state == RouteState.ARRIVED:
                self.get_logger().info("  Route state: ARRIVED (already at goal)")
                return True
        self.get_logger().warn(
            f"  Route not SET after {timeout}s (state={self.route_state})"
        )
        return self.route_state == RouteState.SET

    def step5_engage_autonomous(self, timeout: float = 300.0) -> bool:
        """Engage autonomous driving mode via AD API.

        First waits until is_autonomous_mode_available = True (trajectory
        pipeline must be producing output), then calls change_to_autonomous.
        """
        self.get_logger().info(f"=== Step 5: Engaging autonomous mode (up to {timeout:.0f}s) ===")
        end = time.monotonic() + timeout

        # Wait until the system signals that autonomous mode is available
        self.get_logger().info("  Waiting for is_autonomous_mode_available...")
        last_log = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if (
                self.op_mode_state is not None
                and self.op_mode_state.is_autonomous_mode_available
            ):
                self.get_logger().info("  Autonomous mode is available")
                break
            now = time.monotonic()
            if now - last_log >= 10.0:
                if self.op_mode_state is not None:
                    s = self.op_mode_state
                    self.get_logger().info(
                        f"  Waiting for autonomous mode: "
                        f"mode={s.mode}, auto_avail={s.is_autonomous_mode_available}, "
                        f"stop_avail={s.is_stop_mode_available}, "
                        f"control={s.is_autoware_control_enabled}"
                    )
                else:
                    self.get_logger().info("  Waiting: no operation_mode/state received yet")
                last_log = now
        else:
            self.get_logger().error(f"  Autonomous mode never became available after {timeout:.0f}s")
            return False

        # Engage with retry — availability can flicker during diagnostic transitions
        req = ChangeOperationMode.Request()
        while time.monotonic() < end:
            resp = self._call_service(self.change_to_auto_client, req, "ChangeOperationMode")
            if resp and resp.status.success:
                self.get_logger().info("  Autonomous mode engaged")
                return True
            if resp:
                self.get_logger().info(f"  Engage attempt failed: {resp.status.message} — retrying in 3s...")
            self._spin_for(3.0)

        self.get_logger().error(f"  Failed to engage autonomous mode after {timeout:.0f}s")
        return False

    def step6_wait_for_arrival(self, timeout: float) -> bool:
        """Wait until /api/routing/state = ARRIVED."""
        self.get_logger().info(
            f"=== Step 6: Driving to goal (up to {timeout:.0f}s) ==="
        )
        start = time.monotonic()
        end = start + timeout
        last_log = start
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.monotonic()
            if self.route_state == RouteState.ARRIVED:
                self.get_logger().info(
                    f"  ARRIVED at goal after {now - start:.1f}s"
                )
                return True
            if now - last_log >= 10.0:
                mode = self.op_mode_state.mode if self.op_mode_state else "?"
                self.get_logger().info(
                    f"  Driving... {now - start:.0f}s elapsed, "
                    f"route_state={self.route_state}, op_mode={mode}"
                )
                last_log = now
        self.get_logger().warn(f"  Did not arrive within {timeout:.0f}s")
        return False

    def run(self) -> bool:
        """Execute the full autonomous driving sequence."""
        self.get_logger().info(f"Auto-drive starting (timeout={self.timeout}s)")
        overall_start = time.monotonic()

        if not self.step1_wait_for_services():
            return False

        elapsed = time.monotonic() - overall_start
        if not self.step2_wait_for_localization(timeout=self.timeout - elapsed):
            return False

        if not self.step3_set_route():
            return False

        if not self.step4_wait_for_route(timeout=15.0):
            return False

        if not self.step5_engage_autonomous():
            return False

        elapsed = time.monotonic() - overall_start
        arrived = self.step6_wait_for_arrival(timeout=max(self.timeout - elapsed, 30.0))

        total = time.monotonic() - overall_start
        self.get_logger().info(
            f"Auto-drive {'completed' if arrived else 'timed out'} in {total:.1f}s"
        )
        return arrived


def main():
    rclpy.init()
    node = AutoDriveNode()

    try:
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
