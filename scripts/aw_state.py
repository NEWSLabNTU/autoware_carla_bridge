#!/usr/bin/env python3
"""Snapshot Autoware's own state, to diff a first run against a later one.

Run 1 passes on every stack; later runs pass about half the time, so the carrier survives a
despawn and respawn and is cleared only by restarting the Autoware nodes. Every experiment in
acb issue 016 so far has looked at the bridge, CARLA, or the interfaces between them. This
looks inside.

Samples diagnostics by node and level, plus the operation-mode, routing and MRM state, over a
window, and prints a summary stable enough to diff between runs.
"""
import sys, time
from collections import Counter, defaultdict

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState, MrmState

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 30
LEVEL = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

rclpy.init()
n = rclpy.create_node("aw_state")
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
diag = defaultdict(Counter)
state = {}
first = {}          # name -> (seconds since probe start, level) of its first non-OK report
T0 = time.time()

def on_diag(m):
    for st in m.status:
        lvl = int.from_bytes(st.level, "little") if isinstance(st.level, bytes) else int(st.level)
        name = LEVEL.get(lvl, str(lvl))
        diag[st.name].update([name])
        # First-mover matters: most of these fire downstream of a car already off its path,
        # so ordering separates a cause from its consequences.
        if lvl >= 2 and st.name not in first:
            first[st.name] = (time.time() - T0, name, st.message[:60])

n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 20)
n.create_subscription(OperationModeState, "/api/operation_mode/state",
                      lambda m: state.__setitem__("mode", m), latched)
n.create_subscription(RouteState, "/api/routing/state",
                      lambda m: state.__setitem__("route", m.state), latched)
n.create_subscription(MrmState, "/api/fail_safe/mrm_state",
                      lambda m: state.__setitem__("mrm", (m.state, m.behavior)), latched)

start = time.time()
while time.time() - start < SECONDS:
    rclpy.spin_once(n, timeout_sec=0.05)

m = state.get("mode")
print(f"operation_mode : {getattr(m, 'mode', '-')}  available={getattr(m, 'is_autonomous_mode_available', '-')}")
print(f"route_state    : {state.get('route', '-')}")
print(f"mrm            : {state.get('mrm', '-')}")
print(f"\ndiagnostics over {SECONDS}s -- nodes reporting anything but OK:")
bad = {k: v for k, v in diag.items() if set(v) - {"OK"}}
if not bad:
    print("  (all OK)")
for name in sorted(bad, key=lambda k: -sum(c for lv, c in diag[k].items() if lv != "OK")):
    counts = " ".join(f"{lv}={c}" for lv, c in sorted(diag[name].items()))
    print(f"  {name[:66]:<66} {counts}")
print("\nfirst ERROR by time since probe start:")
for name, (t, lv, msg) in sorted(first.items(), key=lambda kv: kv[1][0]):
    print(f"  t+{t:6.1f}s  {name[:52]:<52} {msg}")
print(f"\ntotal nodes reporting: {len(diag)}")
rclpy.shutdown()
