"""The Autoware agent's logic, free of ROS and sockets.

`AgentCore` turns agent-protocol commands into calls on an `AutopilotPort` (Autoware's
ADAPI in production, a fake in the tests), and Autoware's state into the protocol's
neutral state. It is stepped by its owner (agent.py) and answers commands from any thread.

What the scenario used to do through SSv2's concealer, the agent now does on the vehicle
side, against the autopilot's own clock and sensors:

- teleported(pose): wait for two LiDAR scans newer than the current estimate (the first
  after a respawn can still belong to the previous vehicle), initialize localization at
  the pose, and report INITIALIZING until the estimate agrees with it (1 m, 0.2 rad, and
  newer than before). Only then IDLE -- which the relay turns into WAITING_FOR_ROUTE and a
  kinematic state the concealer accepts.
- set_goal: (clear and) set the route; then, once Autoware reports autonomous mode
  available, engage by itself, retrying while availability flickers.
"""

import math
import threading
import time
from dataclasses import dataclass, field

from . import agent_protocol as P

# autoware_adapi_v1_msgs constants (duplicated so this module needs no ROS)
LOC_UNKNOWN, LOC_UNINITIALIZED, LOC_INITIALIZING, LOC_INITIALIZED = 0, 1, 2, 3
ROUTE_UNKNOWN, ROUTE_UNSET, ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING = 0, 1, 2, 3, 4
MODE_UNKNOWN, MODE_STOP, MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE = 0, 1, 2, 3, 4
MRM_UNKNOWN, MRM_NORMAL, MRM_OPERATING, MRM_SUCCEEDED, MRM_FAILED = 0, 1, 2, 3, 4
MRM_BEHAVIORS = {1: "NONE", 2: "EMERGENCY_STOP", 3: "COMFORTABLE_STOP", 4: "PULL_OVER"}
# autoware_vehicle_msgs/TurnIndicatorsCommand
TURN = {1: P.INDICATORS_NONE, 2: P.LEFT, 3: P.RIGHT}

# The same bounds the concealer applied (field_operator_application.cpp).
POSITION_TOLERANCE = 1.0
YAW_TOLERANCE = 0.2
SCANS_REQUIRED = 2
SCAN_WAIT = 10.0
# Localization has this long to agree with a teleported pose; the concealer allowed 10 s
# after WAITING_FOR_ROUTE, but that clock now only starts once the agent reports IDLE.
CONVERGE_TIMEOUT = 30.0
INITIALIZE_ATTEMPTS = 5
ENGAGE_RETRY = 3.0
SETTLE = 3.0  # how long a command waits for Autoware's state to show its effect


@dataclass
class Snapshot:
    """What the autopilot reports, at one instant. Stamps are in the autopilot's time."""

    ready: bool = False  # ADAPI services callable and state topics received
    localization: int = LOC_UNKNOWN
    route: int = ROUTE_UNKNOWN
    mode: int = MODE_UNKNOWN
    control_enabled: bool = False
    autonomous_available: bool = False
    pose: dict | None = None  # localization estimate, protocol pose
    pose_stamp: float = 0.0
    scan_stamp: float = 0.0  # newest localization scan
    scan_count: int = 0
    mrm_state: int = MRM_UNKNOWN
    mrm_behavior: int = 0
    emergency: bool = False
    turn_indicators: int = 0  # 0 = no command
    rtc_status: list | None = None
    rtc: bool = False  # cooperate / cooperate_auto services exist


class AutopilotPort:
    """The autopilot interface the agent drives. Each call returns (ok, message)."""

    def snapshot(self) -> Snapshot:
        raise NotImplementedError

    def initialize(self, pose, stamp):
        raise NotImplementedError

    def set_route(self, goal, waypoints, allow_goal_modification, segments):
        raise NotImplementedError

    def clear_route(self):
        raise NotImplementedError

    def change_to_autonomous(self):
        raise NotImplementedError

    def change_to_stop(self):
        raise NotImplementedError

    def set_velocity_limit(self, mps):
        raise NotImplementedError

    def cooperate(self, module, command, uuid):
        raise NotImplementedError

    def cooperate_auto(self, module, enable):
        raise NotImplementedError


def yaw_of(pose):
    return math.atan2(2.0 * (pose["qw"] * pose["qz"] + pose["qx"] * pose["qy"]),
                      1.0 - 2.0 * (pose["qy"] ** 2 + pose["qz"] ** 2))


def consistent(expected, actual):
    """The concealer's isLocalizationConsistentWith: planar distance and yaw."""
    yaw = math.remainder(yaw_of(actual) - yaw_of(expected), 2.0 * math.pi)
    return (math.hypot(actual["x"] - expected["x"], actual["y"] - expected["y"])
            <= POSITION_TOLERANCE and abs(yaw) <= YAW_TOLERANCE)


def same_pose(a, b, tolerance=0.05):
    return a is not None and b is not None and all(
        abs(a[k] - b[k]) <= tolerance for k in P.POSE_KEYS)


def derive_phase(s: Snapshot, teleporting: bool, stopped: bool) -> str:
    if not s.ready:
        return P.UNAVAILABLE
    if teleporting or s.localization != LOC_INITIALIZED:
        return P.INITIALIZING
    if s.route == ROUTE_ARRIVED:
        return P.ARRIVED
    if s.route in (ROUTE_SET, ROUTE_CHANGING):
        if s.mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE) and s.control_enabled:
            return P.DRIVING
        if stopped:
            return P.STOPPED
        return P.READY if s.autonomous_available else P.PLANNING
    if s.route == ROUTE_UNSET:
        return P.IDLE
    return P.INITIALIZING


@dataclass
class Teleport:
    pose: dict
    before: float  # estimate stamp at the time of the command
    scan_count: int
    started: float
    stage: str = "scans"  # scans -> initialize -> converge
    fresh_scans: int = 0
    attempts: int = 0
    next_try: float = 0.0
    deadline: float = 0.0
    last_scan_count: int = field(default=0)


class AgentCore:
    def __init__(self, port: AutopilotPort, clock=time.monotonic, sleep=time.sleep,
                 logger=None):
        self.port = port
        self.clock = clock
        self.sleep = sleep
        self.log = logger or (lambda text: None)
        self._lock = threading.RLock()
        self.teleport = None
        self.goal = None  # the active set_goal args
        self.stopped = False
        self.detail = ""
        self._last_engage = -math.inf
        self.on_change = lambda: None  # called when the phase may have changed

    # -- state ----------------------------------------------------------------------------
    def phase(self, snapshot=None) -> str:
        s = snapshot or self.port.snapshot()
        with self._lock:
            return derive_phase(s, self.teleport is not None, self.stopped)

    def capabilities(self, snapshot=None):
        s = snapshot or self.port.snapshot()
        return list(P.REQUIRED_COMMANDS) + (list(P.OPTIONAL_COMMANDS) if s.rtc else [])

    def state_message(self) -> dict:
        s = self.port.snapshot()
        if s.emergency:
            fault, behavior, progress = P.EMERGENCY, MRM_BEHAVIORS.get(s.mrm_behavior), None
        elif s.mrm_state in (MRM_OPERATING, MRM_SUCCEEDED, MRM_FAILED):
            fault = P.MINIMAL_RISK_MANEUVER
            behavior = MRM_BEHAVIORS.get(s.mrm_behavior)
            progress = {MRM_OPERATING: P.OPERATING, MRM_SUCCEEDED: P.SUCCEEDED,
                        MRM_FAILED: P.FAILED}[s.mrm_state]
        else:
            fault, behavior, progress = P.FAULT_NONE, None, None
        with self._lock:
            detail = self.detail
        return P.state(self.phase(s), fault=fault, fault_behavior=behavior,
                       fault_progress=progress,
                       turn_indicators=TURN.get(s.turn_indicators),
                       capabilities=self.capabilities(s), detail=detail, pose=s.pose,
                       rtc_status=s.rtc_status if s.rtc else None)

    def _wait_for(self, predicate, timeout=SETTLE):
        """Let Autoware's state show a command's effect before replying (bounded)."""
        end = self.clock() + timeout
        while self.clock() < end:
            if predicate(self.port.snapshot()):
                return True
            self.sleep(0.05)
        return predicate(self.port.snapshot())

    # -- commands -----------------------------------------------------------------------
    def handle(self, name, args):
        """One command: (status, message). Called from any thread."""
        try:
            handler = getattr(self, "_cmd_" + name)
        except AttributeError:
            return P.UNSUPPORTED, f"unknown command {name}"
        status, message = handler(**args)
        self.on_change()
        return status, message

    def _cmd_teleported(self, pose):
        s = self.port.snapshot()
        with self._lock:
            if self.teleport is not None and same_pose(self.teleport.pose, pose):
                return P.OK, "already re-localizing at this pose"
            self.teleport = Teleport(pose=pose, before=s.pose_stamp,
                                     scan_count=s.scan_count, started=self.clock(),
                                     last_scan_count=s.scan_count)
            # Re-initializing localization drops Autoware's route; the goal goes with it.
            self.goal = None
            self.stopped = False
            self.detail = "re-localizing at the teleported pose"
        self.log(f"teleported to ({pose['x']:.2f}, {pose['y']:.2f}, yaw "
                 f"{yaw_of(pose):.2f}): re-localizing")
        return P.OK, ""

    def _cmd_set_goal(self, goal, waypoints=(), allow_goal_modification=False,
                      segments=None):
        s = self.port.snapshot()
        phase = self.phase(s)
        if phase in (P.UNAVAILABLE, P.INITIALIZING):
            return P.FAILED, f"not ready for a goal: {phase}"
        with self._lock:
            if (self.goal is not None and same_pose(self.goal["goal"], goal)
                    and phase in (P.PLANNING, P.READY, P.DRIVING)):
                return P.OK, "already driving to this goal"
        if s.route in (ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING):
            ok, message = self._clear(s)
            if not ok:
                return P.FAILED, f"could not clear the previous route: {message}"
        ok, message = self.port.set_route(goal, list(waypoints), allow_goal_modification,
                                          segments)
        if not ok:
            return P.FAILED, message
        with self._lock:
            self.goal = {"goal": goal, "waypoints": list(waypoints), "segments": segments}
            self.stopped = False
            self.detail = ""
            self._last_engage = -math.inf
        self._wait_for(lambda s: s.route in (ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING))
        self.log(f"goal set: ({goal['x']:.2f}, {goal['y']:.2f}); engaging once ready")
        return P.OK, ""

    def _clear(self, s):
        """Clear Autoware's route, stopping first: the routing API refuses it while
        autonomous."""
        if s.mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE):
            ok, message = self.port.change_to_stop()
            if not ok:
                return False, f"change_to_stop: {message}"
            self._wait_for(lambda s: s.mode == MODE_STOP)
        ok, message = self.port.clear_route()
        if not ok and self.port.snapshot().route != ROUTE_UNSET:
            return False, message
        self._wait_for(lambda s: s.route == ROUTE_UNSET)
        return True, ""

    def _cmd_clear_goal(self):
        with self._lock:
            self.goal = None
        s = self.port.snapshot()
        if s.route == ROUTE_UNSET:
            return P.OK, "no route"
        ok, message = self._clear(s)
        return (P.OK, "") if ok else (P.FAILED, message)

    def _cmd_stop(self):
        with self._lock:
            self.stopped = True
        if self.port.snapshot().mode == MODE_STOP:
            return P.OK, "already stopped"
        ok, message = self.port.change_to_stop()
        if ok:
            self._wait_for(lambda s: s.mode == MODE_STOP)
        return (P.OK, "") if ok else (P.FAILED, message)

    def _cmd_set_speed_limit(self, mps=None):
        if mps is None:
            return P.OK, "no limit"
        ok, message = self.port.set_velocity_limit(float(mps))
        return (P.OK, "") if ok else (P.FAILED, message)

    def _cmd_cooperate(self, module, command, uuid=None):
        if not self.port.snapshot().rtc:
            return P.UNSUPPORTED, "this Autoware offers no RTC commands"
        ok, message = self.port.cooperate(module, command, uuid)
        return (P.OK, "") if ok else (P.FAILED, message)

    def _cmd_cooperate_auto(self, module, enable):
        if not self.port.snapshot().rtc:
            return P.UNSUPPORTED, "this Autoware offers no RTC auto mode"
        ok, message = self.port.cooperate_auto(module, enable)
        return (P.OK, "") if ok else (P.FAILED, message)

    # -- the periodic part ----------------------------------------------------------------
    def step(self):
        """Advance a teleport and engage a ready goal. Call a few times a second."""
        with self._lock:
            teleport = self.teleport
        if teleport is not None:
            self._step_teleport(teleport)
            return
        s = self.port.snapshot()
        with self._lock:
            want = self.goal is not None and not self.stopped
        if want and self.phase(s) == P.READY and self.clock() - self._last_engage >= \
                ENGAGE_RETRY:
            self._last_engage = self.clock()
            ok, message = self.port.change_to_autonomous()
            self.log("engaged" if ok else f"engage refused ({message}); retrying")
            self.on_change()

    def _finish_teleport(self, detail):
        with self._lock:
            self.teleport = None
            self.detail = detail
        self.on_change()

    def _step_teleport(self, t: Teleport):
        now = self.clock()
        s = self.port.snapshot()
        if t.stage == "scans":
            if s.scan_count != t.last_scan_count:
                if s.scan_stamp > t.before:
                    t.fresh_scans += s.scan_count - t.last_scan_count
                t.last_scan_count = s.scan_count
            if t.fresh_scans >= SCANS_REQUIRED or now - t.started >= SCAN_WAIT:
                if t.fresh_scans < SCANS_REQUIRED:
                    self.log(f"only {t.fresh_scans} fresh scan(s) in {SCAN_WAIT:.0f} s; "
                             "initializing localization anyway")
                t.stage, t.next_try = "initialize", now
        if t.stage == "initialize" and now >= t.next_try:
            t.attempts += 1
            ok, message = self.port.initialize(t.pose, s.scan_stamp or None)
            if ok:
                t.stage, t.deadline = "converge", self.clock() + CONVERGE_TIMEOUT
            elif t.attempts >= INITIALIZE_ATTEMPTS:
                self.log(f"localization initialize refused {t.attempts} times: {message}")
                self._finish_teleport(f"localization initialize refused: {message}")
                return
            else:
                t.next_try = now + 2.0
        if t.stage == "converge":
            s = self.port.snapshot()
            if (s.localization == LOC_INITIALIZED and s.pose is not None
                    and s.pose_stamp > t.before and consistent(t.pose, s.pose)):
                self.log(f"localized at the teleported pose after "
                         f"{self.clock() - t.started:.1f} s")
                self._finish_teleport("")
            elif self.clock() >= t.deadline:
                where = ("no estimate" if s.pose is None else
                         f"estimate at ({s.pose['x']:.2f}, {s.pose['y']:.2f})")
                self.log(f"localization did not reach the teleported pose in "
                         f"{CONVERGE_TIMEOUT:.0f} s ({where})")
                self._finish_teleport("localization did not reach the teleported pose "
                                      f"({where})")
