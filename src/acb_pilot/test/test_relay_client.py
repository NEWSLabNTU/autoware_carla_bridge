"""RelayClient against a scripted relay on localhost: registration, state before reply,
and reconnecting when the relay goes away."""

import logging
import socket
import threading
import time

from acb_pilot import agent_core as C
from acb_pilot import agent_protocol as P
from acb_pilot.agent import RelayClient, parse_relay
from fake_autoware import Clock, FakeAutoware


class ScriptedRelay:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(4)
        self.listener.settimeout(5)
        self.port = self.listener.getsockname()[1]

    def accept(self):
        conn, _ = self.listener.accept()
        conn.settimeout(5)
        return Conn(conn)


class Conn:
    def __init__(self, sock):
        self.sock, self.reader, self.pending = sock, P.LineReader(), []

    def send(self, message):
        self.sock.sendall(P.encode(message))

    def recv(self):
        while not self.pending:
            data = self.sock.recv(65536)
            if not data:
                raise ConnectionError
            self.pending += self.reader.feed(data)
        return P.decode(self.pending.pop(0))


def make_client(port):
    clock = Clock()
    core = C.AgentCore(FakeAutoware(clock), clock=clock, sleep=lambda dt: clock.advance(dt))
    client = RelayClient(core, "127.0.0.1", port, "ego", logging.getLogger("test"))
    threading.Thread(target=client.run, daemon=True).start()
    return client


def test_parse_relay():
    assert parse_relay("tcp://sim:6000") == ("sim", 6000)
    assert parse_relay("localhost") == ("localhost", 5560)


def test_register_state_and_command_reply_order():
    relay = ScriptedRelay()
    client = make_client(relay.port)
    try:
        conn = relay.accept()
        hello = conn.recv()
        assert hello["type"] == P.REGISTER and hello["entity"] == "ego"
        assert set(P.REQUIRED_COMMANDS) <= set(hello["capabilities"])
        conn.send(P.registered("ego"))
        assert conn.recv()["type"] == P.STATE
        conn.send(P.command(7, P.STOP))
        seen = []
        while True:
            message = conn.recv()
            seen.append(message["type"])
            if message["type"] == P.REPLY:
                assert message["id"] == 7 and message["status"] == P.OK
                break
        assert seen[-2] == P.STATE  # pushed before the reply
    finally:
        client.stop()


def test_reconnects_when_the_relay_restarts():
    relay = ScriptedRelay()
    client = make_client(relay.port)
    try:
        conn = relay.accept()
        assert conn.recv()["type"] == P.REGISTER
        conn.send(P.registered("ego"))
        conn.sock.close()  # the relay dies
        t0 = time.monotonic()
        conn = relay.accept()  # ... and the agent comes back
        assert conn.recv()["type"] == P.REGISTER
        assert time.monotonic() - t0 < 4.0
    finally:
        client.stop()


def test_silent_relay_is_dropped():
    relay = ScriptedRelay()
    client = make_client(relay.port)
    try:
        conn = relay.accept()
        conn.recv()
        conn.send(P.registered("ego"))
        # Say nothing more (no heartbeats): the agent must give up on this connection
        # after PEER_TIMEOUT and register again.
        second = relay.accept()
        assert second.recv()["type"] == P.REGISTER
    finally:
        client.stop()
