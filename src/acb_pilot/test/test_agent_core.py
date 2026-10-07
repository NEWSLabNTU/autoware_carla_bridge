import math

import pytest

from acb_pilot import agent_core as C
from acb_pilot import agent_protocol as P
from fake_autoware import Clock, FakeAutoware

GOAL = {"x": 50.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
START = {"x": 190.8, "y": -130.1, "z": 0.3, "qx": 0.0, "qy": 0.0,
         "qz": math.sin(1.5708), "qw": math.cos(1.5708)}


@pytest.fixture
def rig():
    clock = Clock()
    autoware = FakeAutoware(clock)
    core = C.AgentCore(autoware, clock=clock, sleep=clock.sleep)
    return clock, autoware, core


def run(clock, core, seconds, dt=0.1):
    for _ in range(int(seconds / dt)):
        clock.advance(dt)
        core.step()


def test_phase_derivation_table():
    s = C.Snapshot(ready=False)
    assert C.derive_phase(s, False, False) == P.UNAVAILABLE
    s = C.Snapshot(ready=True, localization=C.LOC_INITIALIZING)
    assert C.derive_phase(s, False, False) == P.INITIALIZING
    s = C.Snapshot(ready=True, localization=C.LOC_INITIALIZED, route=C.ROUTE_UNSET)
    assert C.derive_phase(s, False, False) == P.IDLE
    assert C.derive_phase(s, True, False) == P.INITIALIZING  # teleport in progress
    s.route, s.mode = C.ROUTE_SET, C.MODE_STOP
    assert C.derive_phase(s, False, False) == P.PLANNING
    s.autonomous_available = True
    assert C.derive_phase(s, False, False) == P.READY
    assert C.derive_phase(s, False, True) == P.STOPPED
    s.mode, s.control_enabled = C.MODE_AUTONOMOUS, True
    assert C.derive_phase(s, False, False) == P.DRIVING
    s.route = C.ROUTE_ARRIVED
    assert C.derive_phase(s, False, False) == P.ARRIVED


def test_scenario_sequence_teleport_goal_engage_arrive(rig):
    clock, autoware, core = rig
    # The concealer's queue: stop, clear_route, initialize (teleported), plan (set_goal).
    assert core.handle(P.STOP, {})[0] == P.OK
    assert core.handle(P.CLEAR_GOAL, {})[0] == P.OK
    assert core.handle(P.TELEPORTED, {"pose": START})[0] == P.OK
    assert core.phase() == P.INITIALIZING
    # A goal before localization agrees is refused; the concealer would retry.
    assert core.handle(P.SET_GOAL, {"goal": GOAL})[0] == P.FAILED

    run(clock, core, 0.35)  # two fresh scans, then initialize
    assert "initialize" in autoware.calls
    assert autoware.init_stamp > 50.0  # stamped with a scan newer than the old estimate
    run(clock, core, 2.0)
    assert core.phase() == P.IDLE
    assert C.consistent(START, core.state_message()["pose"])

    status, _ = core.handle(P.SET_GOAL, {"goal": GOAL, "waypoints": []})
    assert status == P.OK
    assert core.phase() in (P.PLANNING, P.READY)
    run(clock, core, 2.0)
    assert autoware.calls.count("change_to_autonomous") >= 1
    assert core.phase() == P.DRIVING
    autoware.arrive()
    assert core.phase() == P.ARRIVED


def test_teleport_waits_for_fresh_scans_but_not_forever(rig):
    clock, autoware, core = rig
    autoware.publish_scans = False
    core.handle(P.TELEPORTED, {"pose": START})
    run(clock, core, 5.0)
    assert "initialize" not in autoware.calls
    run(clock, core, 5.5)
    assert "initialize" in autoware.calls


def test_teleport_reports_failure_when_localization_does_not_converge(rig):
    clock, autoware, core = rig
    autoware.CONVERGE_TIME = 1e9
    core.handle(P.TELEPORTED, {"pose": START})
    run(clock, core, C.CONVERGE_TIMEOUT + 2.0)
    message = core.state_message()
    assert message["phase"] == P.INITIALIZING  # Autoware itself still initializing
    assert "did not reach" in message["detail"]


def test_teleport_retries_a_refused_initialize(rig):
    clock, autoware, core = rig
    autoware.refuse.add("initialize")
    core.handle(P.TELEPORTED, {"pose": START})
    run(clock, core, 3.0)
    assert autoware.calls.count("initialize") >= 2
    autoware.refuse.clear()
    run(clock, core, 5.0)
    assert core.phase() == P.IDLE


def test_duplicate_commands_are_idempotent(rig):
    clock, autoware, core = rig
    core.handle(P.TELEPORTED, {"pose": START})
    assert "already" in core.handle(P.TELEPORTED, {"pose": START})[1]
    run(clock, core, 3.0)
    core.handle(P.SET_GOAL, {"goal": GOAL})
    calls = autoware.calls.count("set_route")
    assert core.handle(P.SET_GOAL, {"goal": GOAL}) == (P.OK, "already driving to this goal")
    assert autoware.calls.count("set_route") == calls


def test_new_goal_while_driving_stops_clears_and_reroutes(rig):
    clock, autoware, core = rig
    core.handle(P.SET_GOAL, {"goal": GOAL})
    run(clock, core, 2.0)
    assert core.phase() == P.DRIVING
    other = dict(GOAL, x=80.0)
    assert core.handle(P.SET_GOAL, {"goal": other})[0] == P.OK
    assert autoware.calls[-3:] == ["change_to_stop", "clear_route", "set_route"]
    run(clock, core, 2.0)
    assert core.phase() == P.DRIVING and autoware.goal == other


def test_stop_holds_until_the_next_goal(rig):
    clock, autoware, core = rig
    core.handle(P.SET_GOAL, {"goal": GOAL})
    run(clock, core, 2.0)
    assert core.handle(P.STOP, {})[0] == P.OK
    run(clock, core, 5.0)
    assert core.phase() == P.STOPPED  # no re-engage
    assert autoware.calls.count("change_to_autonomous") == 1


def test_engage_retries_while_autoware_refuses(rig):
    clock, autoware, core = rig
    autoware.refuse.add("change_to_autonomous")
    core.handle(P.SET_GOAL, {"goal": GOAL})
    run(clock, core, 8.0)
    assert autoware.calls.count("change_to_autonomous") >= 2
    assert core.phase() == P.READY
    autoware.refuse.clear()
    run(clock, core, 4.0)
    assert core.phase() == P.DRIVING


def test_goal_refused_by_autoware_fails_the_command(rig):
    clock, autoware, core = rig
    autoware.refuse.add("set_route")
    status, message = core.handle(P.SET_GOAL, {"goal": GOAL})
    assert status == P.FAILED and "refused" in message


def test_unavailable_autoware(rig):
    clock, autoware, core = rig
    autoware.s.ready = False
    assert core.phase() == P.UNAVAILABLE
    assert core.handle(P.SET_GOAL, {"goal": GOAL})[0] == P.FAILED


def test_speed_limit(rig):
    clock, autoware, core = rig
    assert core.handle(P.SET_SPEED_LIMIT, {"mps": 8.0})[0] == P.OK
    assert autoware.velocity_limit == 8.0
    assert core.handle(P.SET_SPEED_LIMIT, {"mps": None}) == (P.OK, "no limit")


def test_rtc_unsupported_without_the_services(rig):
    clock, autoware, core = rig
    args = {"module": "INTERSECTION", "command": "ACTIVATE"}
    assert core.handle(P.COOPERATE, args)[0] == P.UNSUPPORTED
    assert core.handle(P.COOPERATE_AUTO, {"module": "CROSSWALK", "enable": True})[0] == \
        P.UNSUPPORTED
    assert P.COOPERATE not in core.state_message()["capabilities"]
    autoware.s.rtc = True
    assert core.handle(P.COOPERATE, args)[0] == P.OK
    assert P.COOPERATE in core.state_message()["capabilities"]


@pytest.mark.parametrize("emergency, mrm, behavior, fault, progress", [
    (False, C.MRM_NORMAL, 1, P.FAULT_NONE, None),
    (False, C.MRM_OPERATING, 2, P.MINIMAL_RISK_MANEUVER, P.OPERATING),
    (False, C.MRM_SUCCEEDED, 3, P.MINIMAL_RISK_MANEUVER, P.SUCCEEDED),
    (True, C.MRM_OPERATING, 2, P.EMERGENCY, None),
])
def test_fault_reporting(rig, emergency, mrm, behavior, fault, progress):
    clock, autoware, core = rig
    autoware.s.emergency, autoware.s.mrm_state, autoware.s.mrm_behavior = (emergency, mrm,
                                                                          behavior)
    message = core.state_message()
    assert message["fault"] == fault
    assert message.get("fault_progress") == progress
    P.validate(message)


def test_state_message_is_valid_protocol(rig):
    clock, autoware, core = rig
    autoware.s.turn_indicators = 2
    message = core.state_message()
    assert message["turn_indicators"] == P.LEFT
    P.decode(P.encode(message))


def test_unknown_command_is_unsupported(rig):
    assert rig[2].handle("fly", {})[0] == P.UNSUPPORTED
