import pathlib

import pytest

from acb_pilot import agent_protocol as P

POSE = {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


@pytest.mark.parametrize("message", [
    P.register("ego", P.COMMANDS, "acb_agent"), P.reply(3, P.OK), P.heartbeat(),
    P.state(P.READY, pose=POSE, turn_indicators=P.RIGHT),
    P.command(1, P.SET_GOAL, goal=POSE, waypoints=[]),
    P.command(2, P.TELEPORTED, pose=POSE),
], ids=lambda m: m["type"])
def test_round_trip(message):
    decoded = P.decode(P.encode(message))
    assert decoded.pop("v") == 1 and decoded == message


def test_copy_matches_the_relay_schema():
    """The agent's copy must not drift from the relay's (when both repos are present)."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        relay = parent / "scenario_agent_relay" / "scenario_agent_relay" / "protocol.py"
        if relay.exists():
            break
    else:
        pytest.skip("the relay's protocol.py is not checked out next to this repo")
    strip = lambda text: text.split('"""', 2)[2]  # noqa: E731 -- skip the docstring
    ours = pathlib.Path(P.__file__).read_text()
    assert strip(ours) == strip(relay.read_text())
