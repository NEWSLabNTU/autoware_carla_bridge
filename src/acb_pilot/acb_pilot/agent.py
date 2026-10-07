"""acb_agent: the Autoware vehicle agent (I3 <-> Autoware ADAPI).

Simulator-neutral: it talks to Autoware's ADAPI in its own ROS domain and to a scenario's
agent relay over TCP, and knows nothing of CARLA. See carla-scenario-bridge's
docs/design/user-workflow.md, "The agent protocol (I3)" and "Vehicle side".

Two modes:

- relay (param `relay`, e.g. tcp://sim-host:5560): register as `entity`, take commands
  from the scenario, report state. Reconnects whenever the relay goes away; runs until
  stopped.
- local (no `relay`): drive to `goal_poses_file`'s goal_pose once Autoware is localized,
  and exit 0 on arrival, 1 on timeout -- a vehicle side testable with no scenario at all.

ROS parameters:
    relay (str, default ""): tcp://host:port of the agent relay; empty = local mode
    entity (str, default "ego"): the scenario entity this vehicle is
    goal_poses_file (str, default ""): local mode's goal (acb_pilot poses format)
    timeout (float, default 600.0): local mode's budget, from start to arrival

Usage:
    ros2 run acb_pilot agent --ros-args -p relay:=tcp://localhost:5560 -p entity:=ego \\
        -p use_sim_time:=true
"""

import socket
import sys
import threading
import time
from urllib.parse import urlparse

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from . import agent_protocol as P
from .agent_core import AgentCore
from .autoware_port import AutowarePort

STATE_PERIOD = 0.2  # state pushes: phase changes at once, pose and heartbeat at 5 Hz
RECONNECT_MIN, RECONNECT_MAX = 1.0, 5.0


def parse_relay(url):
    parsed = urlparse(url if "://" in url else "tcp://" + url)
    if parsed.scheme != "tcp" or not parsed.hostname:
        raise ValueError(f"relay must be tcp://host:port, got {url!r}")
    return parsed.hostname, parsed.port or 5560


class RelayClient:
    """Keeps one registered connection to the relay, with reconnects. Pure sockets."""

    def __init__(self, core, host, port, entity, logger, agent_name="acb_agent"):
        self.core = core
        self.host, self.port = host, port
        self.entity = entity
        self.agent_name = agent_name
        self.log = logger
        self._stop = threading.Event()
        self._sock = None
        self._send_lock = threading.Lock()
        self._last_sent = 0.0
        self._last_phase = None
        self._wake = threading.Event()
        core.on_change = self._wake.set

    def stop(self):
        self._stop.set()
        self._wake.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def run(self):
        delay = RECONNECT_MIN
        while not self._stop.is_set():
            try:
                self._session()
                delay = RECONNECT_MIN
            except (OSError, P.ProtocolError) as e:
                self.log.warning(f"relay {self.host}:{self.port}: {e}; retrying in "
                                 f"{delay:.0f} s")
            finally:
                self._close()
            if self._stop.wait(delay):
                return
            delay = min(delay * 2, RECONNECT_MAX)

    def _close(self):
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _send(self, message):
        data = P.encode(message)
        with self._send_lock:
            if self._sock is None:
                return
            self._sock.sendall(data)
            self._last_sent = time.monotonic()

    def push_state(self):
        message = self.core.state_message()
        self._send(message)
        if message["phase"] != self._last_phase:
            self.log.info(f"phase {self._last_phase} -> {message['phase']}"
                          + (f" ({message['detail']})" if message["detail"] else ""))
            self._last_phase = message["phase"]

    def _session(self):
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.5)
        self._sock = sock
        self._send(P.register(self.entity, self.core.capabilities(), self.agent_name))
        reader = P.LineReader()
        last_received = time.monotonic()
        registered = False
        pusher = threading.Thread(target=self._push_loop, args=(sock,), daemon=True)
        while not self._stop.is_set():
            try:
                data = sock.recv(65536)
            except socket.timeout:
                if time.monotonic() - last_received > P.PEER_TIMEOUT:
                    raise OSError(f"relay silent for {P.PEER_TIMEOUT:.0f} s")
                continue
            if not data:
                raise OSError("relay closed the connection")
            last_received = time.monotonic()
            for line in reader.feed(data):
                message = P.decode(line)
                kind = message["type"]
                if kind == P.REGISTERED:
                    registered = True
                    self.log.info(f"registered with relay {self.host}:{self.port} as "
                                  f"{self.entity!r}")
                    pusher.start()
                elif kind == P.ERROR:
                    raise P.ProtocolError(f"relay refused: {message['reason']}")
                elif kind == P.COMMAND:
                    if not registered:
                        raise P.ProtocolError("command before registration")
                    threading.Thread(target=self._execute, args=(message,),
                                     daemon=True).start()

    def _execute(self, message):
        name, args = message["command"], message.get("args", {})
        self.log.info(f"command {name}")
        try:
            status, text = self.core.handle(name, args)
        except Exception as e:  # a bug must not leave the relay waiting
            status, text = P.FAILED, f"{type(e).__name__}: {e}"
        self.log.info(f"command {name} -> {status}" + (f": {text}" if text else ""))
        try:
            # State first: whatever the command changed is visible to the relay before
            # it answers the scenario.
            self.push_state()
            self._send(P.reply(message["id"], status, text))
        except OSError:
            pass

    def _push_loop(self, sock):
        while not self._stop.is_set() and self._sock is sock:
            try:
                self.push_state()
            except OSError:
                return
            self._wake.wait(STATE_PERIOD)
            self._wake.clear()


class AgentNode(Node):
    def __init__(self):
        super().__init__("acb_agent")
        self.declare_parameter("relay", "")
        self.declare_parameter("entity", "ego")
        self.declare_parameter("goal_poses_file", "")
        self.declare_parameter("timeout", 600.0)
        self.relay = self.get_parameter("relay").value
        self.entity = self.get_parameter("entity").value
        self.goal_file = self.get_parameter("goal_poses_file").value
        self.timeout = float(self.get_parameter("timeout").value)
        self.port = AutowarePort(self)
        logger = self.get_logger()
        self.core = AgentCore(self.port, logger=logger.info)


def run_relay(node, executor_stop):
    host, port = parse_relay(node.relay)
    client = RelayClient(node.core, host, port, node.entity, node.get_logger())
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    node.get_logger().info(f"agent for entity {node.entity!r}: relay tcp://{host}:{port}")
    try:
        while not executor_stop.is_set():
            node.core.step()
            time.sleep(0.1)
    finally:
        client.stop()
    return 0


def run_local(node, executor_stop):
    """No relay: one drive to the goal file's goal_pose, like auto_drive."""
    log = node.get_logger()
    if not node.goal_file:
        log.fatal("Neither `relay` nor `goal_poses_file` is set: nothing to drive to")
        return 1
    with open(node.goal_file) as f:
        goal = yaml.safe_load(f)["goal_pose"]
    goal = {k: float(goal.get(k, 0.0 if k != "qw" else 1.0)) for k in P.POSE_KEYS}
    log.info(f"local mode: goal ({goal['x']:.2f}, {goal['y']:.2f}) from {node.goal_file}")
    end = time.monotonic() + node.timeout
    sent = False
    last_phase = None
    while not executor_stop.is_set() and time.monotonic() < end:
        node.core.step()
        phase = node.core.phase()
        if phase != last_phase:
            log.info(f"phase {phase}")
            last_phase = phase
        if not sent and phase == P.IDLE:
            status, text = node.core.handle(P.SET_GOAL, {"goal": goal})
            sent = status == P.OK
            if not sent:
                log.warning(f"route refused ({text}); retrying")
                time.sleep(5.0)
        elif sent and phase == P.ARRIVED:
            log.info("arrived")
            return 0
        time.sleep(0.1)
    log.error(f"did not arrive within {node.timeout:.0f} s (phase {last_phase})")
    return 1


def main():
    rclpy.init()
    node = AgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    stop = threading.Event()
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 1
    try:
        code = (run_relay if node.relay else run_local)(node, stop)
    except KeyboardInterrupt:
        code = 0
    finally:
        stop.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
