"""acb_agent: the Autoware vehicle agent, on carla-scenario-bridge's agent SDK.

The agent itself is `swb_agent.adapters.autoware.AutowareAgent` (scenario-world-bridge, once
carla-scenario-bridge, roadmap 030): Autoware's ADAPI as a `swb_agent.Agent`,
simulator-neutral. This node runs it with acb's ROS parameters, so the launch file is
unchanged. It needs `swb_agent` installed (scenario-world-bridge's workspace, its release
tarball or the `swb-agent` wheel); a `csb_agent` from before the rename is used when that is
what is installed.

Two modes:

- relay (param `relay`, e.g. tcp://sim-host:5560): register as `entity`, take commands
  from the scenario (protocol v2: a `reset` at each scenario start), report state.
  Reconnects whenever the relay goes away; runs until stopped.
- local (no `relay`): drive to `goal_poses_file`'s goal_pose once Autoware is localized,
  and exit 0 on arrival, 1 on timeout -- a vehicle side testable with no scenario at all.

ROS parameters:
    relay (str, default ""): tcp://host:port of the agent relay; empty = local mode
    entity (str, default "ego"): the scenario entity this vehicle is
    goal_poses_file (str, default ""): local mode's goal (acb_pilot poses format)
    timeout (float, default 600.0): local mode's budget, from start to arrival
    agent_config (str, default ""): a YAML/JSON file of AutowareAgent settings (its
        DEFAULTS: converge_timeout, ready_timeout, scan_wait, ...)

Usage:
    ros2 run acb_pilot agent --ros-args -p relay:=tcp://localhost:5560 -p entity:=ego \\
        -p use_sim_time:=true
"""

import logging
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:  # scenario-world-bridge's agent SDK; csb_agent before its roadmap 046 rename
    import swb_agent as csb_agent
    from swb_agent import protocol as P
    from swb_agent.adapters.autoware import AutowareAgent
    from swb_agent.adapters.autoware_ros import AutowarePort
except ImportError:
    import csb_agent
    from csb_agent import protocol as P
    from csb_agent.adapters.autoware import AutowareAgent
    from csb_agent.adapters.autoware_ros import AutowarePort


class AgentNode(Node):
    def __init__(self):
        super().__init__("acb_agent")
        self.declare_parameter("relay", "")
        self.declare_parameter("entity", "ego")
        self.declare_parameter("goal_poses_file", "")
        self.declare_parameter("timeout", 600.0)
        self.declare_parameter("agent_config", "")
        self.relay = self.get_parameter("relay").value
        self.entity = self.get_parameter("entity").value
        self.goal_file = self.get_parameter("goal_poses_file").value
        self.timeout = float(self.get_parameter("timeout").value)
        path = self.get_parameter("agent_config").value
        config = csb_agent.load_config(path) if path else {}
        config["entity"] = self.entity
        self.agent = AutowareAgent(config, port=AutowarePort(self))


def run_relay(node, stop):
    host, port = csb_agent.parse_relay(node.relay)
    client = csb_agent.Client(node.agent, host, port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    node.get_logger().info(f"agent for entity {node.entity!r}: relay tcp://{host}:{port}")
    try:
        while not stop.wait(0.5):
            pass
    finally:
        client.stop()
    return 0


def run_local(node, stop):
    """No relay: one drive to the goal file's goal_pose, like auto_drive."""
    log = node.get_logger()
    if not node.goal_file:
        log.fatal("Neither `relay` nor `goal_poses_file` is set: nothing to drive to")
        return 1
    with open(node.goal_file) as f:
        goal = yaml.safe_load(f)["goal_pose"]
    goal = {k: float(goal.get(k, 0.0 if k != "qw" else 1.0)) for k in P.POSE_KEYS}
    log.info(f"local mode: goal ({goal['x']:.2f}, {goal['y']:.2f}) from {node.goal_file}")
    agent = node.agent
    end = time.monotonic() + node.timeout
    sent = False
    last_phase = None
    while not stop.is_set() and time.monotonic() < end:
        phase = agent.derived_phase()
        if phase != last_phase:
            log.info(f"phase {phase}")
            last_phase = phase
        if not sent and phase == P.IDLE:
            try:
                agent.on_goal(goal, [])
                sent = True
            except csb_agent.Failed as e:
                log.warning(f"route refused ({e}); retrying")
                time.sleep(5.0)
        elif sent and phase == P.ARRIVED:
            log.info("arrived")
            return 0
        time.sleep(0.1)
    log.error(f"did not arrive within {node.timeout:.0f} s (phase {last_phase})")
    return 1


def main():
    # the SDK logs through `logging` (commands, phases, reconnects): to stderr, as before
    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s] [%(created).6f] [%(name)s]: %(message)s")
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
        node.agent.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
