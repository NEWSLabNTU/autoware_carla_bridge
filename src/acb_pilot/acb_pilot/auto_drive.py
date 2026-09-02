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
    spawn_timeout (float, default 1800.0): How long to wait for a vehicle to drive
        before giving up, measured separately from `timeout` because it is someone
        else's startup time. Bounds re-acquisition too: if the vehicle is despawned
        mid-sequence the pilot starts over, and this is the budget for all attempts.
        See step1b_wait_for_vehicle and run().

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

import math
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

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


# How long GNSS may be quiet before the pilot concludes its vehicle is gone.
VEHICLE_GONE_AFTER = 5.0

# Losing the ego this close to the goal counts as arriving. Wider than the scenario's own
# ReachPositionCondition (6 m in town01_unmanaged.xosc), because the EKF pose the pilot
# last saw lags the CARLA pose the scenario scored.
ARRIVED_IF_LOST_WITHIN = 15.0


class VehicleGone(Exception):
    """The vehicle being driven was despawned; re-acquire and start over."""


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
        # How long to wait for the vehicle to exist before giving up. Separate from
        # `timeout`, and much longer, because it measures someone else's startup:
        # see step1b_wait_for_vehicle.
        self.declare_parameter("spawn_timeout", 1800.0)
        # How far the EKF estimate may sit from the GNSS fix before this node treats
        # localization as not really initialized. A healthy stack measures 1.2-1.3 m,
        # which is the antenna offset rather than error; an estimate left behind at the
        # previous run's goal measures 80-95 m. The limit sits well clear of the healthy
        # figure and well below the fault.
        self.declare_parameter("localization_gap_limit_m", 5.0)

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
        self.spawn_timeout = (
            self.get_parameter("spawn_timeout").get_parameter_value().double_value
        )
        self.localization_gap_limit = (
            self.get_parameter("localization_gap_limit_m")
            .get_parameter_value()
            .double_value
        )

        # State
        self.gnss_stamp = None
        # The GNSS fix itself. It is the only estimate of where the vehicle is that does
        # not come from the filter being checked, so it is what makes a stale EKF pose
        # detectable at all.
        self.gnss_pose = None
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

        # Evidence that the vehicle this pilot drives exists at all. `acb_bridge`
        # publishes GNSS only once it has a vehicle to attach the sensor to, so the
        # first message here is the moment the vehicle appeared. See
        # step1b_wait_for_vehicle for why that has to be timed separately.
        self.gnss_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self._on_gnss,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_localization_state(self, msg: LocalizationInitializationState):
        self.localization_state = msg.state

    def _on_route_state(self, msg: RouteState):
        self.route_state = msg.state

    def _on_op_mode_state(self, msg: OperationModeState):
        self.op_mode_state = msg

    def _on_gnss(self, msg: PoseWithCovarianceStamped):
        self.gnss_stamp = time.monotonic()
        self.gnss_pose = msg.pose.pose

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

    def _localization_gap(self):
        """Planar distance between the EKF estimate and the GNSS fix, in metres.

        Returns None when either estimate is missing, which is not agreement and must
        not be read as such by the caller.
        """
        if self.kinematic_pose is None or self.gnss_pose is None:
            return None
        a, b = self.kinematic_pose.position, self.gnss_pose.position
        return math.hypot(a.x - b.x, a.y - b.y)

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

    def _vehicle_is_present(self) -> bool:
        """Is the vehicle this pilot is driving still there?

        `acb_bridge` publishes GNSS only while it has a vehicle attached, so GNSS going
        quiet means the vehicle was despawned. VEHICLE_GONE_AFTER is generous because the
        topic is not fast and a busy machine can stretch the gap.
        """
        return (
            self.gnss_stamp is not None
            and time.monotonic() - self.gnss_stamp < VEHICLE_GONE_AFTER
        )

    def _distance_to_goal(self) -> float | None:
        """How far the last known EKF pose was from the goal, or None if never seen."""
        if self.kinematic_pose is None:
            return None
        p = self.kinematic_pose.position
        return math.hypot(p.x - self.goal_pose["x"], p.y - self.goal_pose["y"])

    def _abort_if_vehicle_gone(self) -> None:
        """Raise if the vehicle disappeared mid-sequence.

        A scenario ego is despawned and respawned freely -- SSv2 does it at the start of
        every run -- and a pilot that does not notice keeps steering a vehicle that no
        longer exists. That is not hypothetical: an ego stack started while a PREVIOUS
        run's ego was still in the world had its pilot localize to that stale vehicle at
        (104.5, -55.4) instead of the spawn point, route it, engage it, and spend its
        whole 564 s budget on it. The scenario then started, despawned the stale ego and
        spawned the real one, which nothing was left to drive. From outside it looked
        like an ego that drove for ten minutes and never arrived.
        """
        if not self._vehicle_is_present():
            raise VehicleGone()

    def step1b_wait_for_vehicle(self, timeout: float) -> bool:
        """Wait for the vehicle this pilot drives to exist.

        A background AV's vehicle is spawned by its own stack, so it is there before
        this node starts and this step returns immediately. An unmanaged scenario ego
        is not: the ego stack comes up first (~8 minutes), and the vehicle appears only
        when someone later starts a scenario that spawns it. That gap is unbounded --
        it is however long the operator takes -- and it must not be charged to
        `timeout`, which is the budget for localizing and driving.

        Charging it there is what phase 013's "unmanaged ego never localizes" blocker
        actually was. The pilot spent its whole 600 s budget waiting for a vehicle that
        did not exist yet and died 2.5 minutes before the scenario spawned one; nothing
        was wrong with localization, which took 55 s once there was a vehicle to
        localize. So this is a separate deadline, and the drive budget starts after it.

        GNSS is the signal: `acb_bridge` publishes it only once it has attached the
        sensor to a vehicle.
        """
        self.get_logger().info(
            f"=== Step 1b: Waiting for the vehicle to exist (up to {timeout:.0f}s) ==="
        )
        if self.gnss_stamp is not None:
            self.get_logger().info("  Vehicle already present (GNSS is flowing)")
            return True
        end = time.monotonic() + timeout
        last_log = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.gnss_stamp is not None:
                self.get_logger().info("  Vehicle is present (GNSS started flowing)")
                return True
            now = time.monotonic()
            if now - last_log >= 15.0:
                last_log = now
                self.get_logger().info(
                    "  No vehicle yet: nothing is publishing "
                    "/sensing/gnss/pose_with_covariance. Waiting for one to be spawned."
                )
        self.get_logger().error(
            f"  No vehicle appeared within {timeout:.0f}s: nothing ever published "
            f"/sensing/gnss/pose_with_covariance. Either no scenario spawned this "
            f"vehicle, or acb_bridge never attached its sensors."
        )
        return False

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
        last_reinit = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            self._abort_if_vehicle_gone()
            initialized = (
                self.localization_state == LocalizationInitializationState.INITIALIZED
            )
            # A fresh EKF pose is evidence that the estimate is live, but not that it
            # belongs to the vehicle in front of us: an estimate stranded at the
            # previous run's goal goes on publishing at full rate forever, so it is
            # both INITIALIZED and fresh while being 90 m from the car. Only agreement
            # with GNSS -- the one estimate that does not come from this filter --
            # distinguishes the two.
            fresh = self.kinematic_stamp is not None and (
                time.monotonic() - self.kinematic_stamp < 2.0
            )
            gap = self._localization_gap() if fresh else None
            agrees = gap is not None and gap <= self.localization_gap_limit
            if initialized and fresh and not agrees:
                # Re-ask, but no faster than the pipeline can answer: a re-init has to
                # travel through NDT and the EKF before the pose can move.
                if time.monotonic() - last_reinit >= 20.0:
                    last_reinit = time.monotonic()
                    where = "no GNSS fix yet" if gap is None else f"{gap:.1f} m from GNSS"
                    self.get_logger().warn(
                        f"  Localization reports INITIALIZED but the estimate is "
                        f"{where}; re-initializing."
                    )
                    self.kinematic_pose = None
                    self.kinematic_stamp = None
                    self._request_localization_reinit()
                continue
            if initialized and fresh and agrees:
                p = self.kinematic_pose.position
                self.get_logger().info(
                    f"  Localization: INITIALIZED at ({p.x:.2f}, {p.y:.2f}), "
                    f"{gap:.2f} m from GNSS"
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
                gap_note = (
                    ""
                    if not fresh
                    else " (no GNSS fix yet)"
                    if gap is None
                    else f" ({gap:.1f} m from GNSS)"
                )
                self.get_logger().info(
                    f"  Localization state: {name}"
                    + ("" if fresh else " (no fresh pose yet)")
                    + gap_note
                )
                last_log = now
        self.get_logger().error(
            f"  Localization did not reach INITIALIZED with a live pose that agrees "
            f"with GNSS (within {self.localization_gap_limit:.1f} m) in {timeout:.0f}s"
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
            self._abort_if_vehicle_gone()
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
        """Execute the full autonomous driving sequence.

        The sequence restarts if the vehicle is despawned under it. A scenario ego comes
        and goes -- SSv2 despawns and respawns one at the start of every run -- so
        "the vehicle I was driving is gone" is a normal event to recover from, not a
        failure. Without this the pilot drives whatever vehicle happened to exist when it
        started, including one left behind by a previous run, and never notices that the
        real ego arrived later. Bounded by spawn_timeout, so a vehicle that never comes
        back still ends the run rather than looping forever.
        """
        self.get_logger().info(f"Auto-drive starting (timeout={self.timeout}s)")

        if not self.step1_wait_for_services():
            return False

        deadline = time.monotonic() + self.spawn_timeout
        attempt = 0
        while True:
            attempt += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.get_logger().error(
                    f"  No vehicle stayed around long enough to drive, within "
                    f"{self.spawn_timeout:.0f}s"
                )
                return False
            if attempt > 1:
                self.get_logger().info(f"=== Re-acquiring (attempt {attempt}) ===")
                self.kinematic_pose = None
                self.kinematic_stamp = None
                self.gnss_stamp = None
                self.gnss_pose = None

            if not self.step1b_wait_for_vehicle(timeout=remaining):
                return False
            # The drive budget starts once there is a vehicle to drive. Waiting for one
            # is someone else's startup time and is bounded by spawn_timeout instead.
            overall_start = time.monotonic()

            try:
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
                arrived = self.step6_wait_for_arrival(
                    timeout=max(self.timeout - elapsed, 30.0)
                )
            except VehicleGone:
                # A scenario despawns its ego the moment it scores the run, which is
                # BEFORE the pilot can observe route_state=ARRIVED. Losing the vehicle
                # within arriving distance of the goal is that success, not a wrong
                # vehicle -- treating it as the latter left the pilot waiting out its
                # whole spawn_timeout after a run it had just completed.
                gap = self._distance_to_goal()
                if gap is not None and gap <= ARRIVED_IF_LOST_WITHIN:
                    self.get_logger().info(
                        f"  The ego was despawned {gap:.1f} m from the goal: the "
                        "scenario ended the run on arrival."
                    )
                    return True
                where = "unknown position" if gap is None else f"{gap:.1f} m from the goal"
                self.get_logger().warn(
                    f"  The vehicle being driven was despawned at {where}. It was most "
                    "likely not the one this run is about -- waiting for the next one."
                )
                continue

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
