#!/usr/bin/env python3
"""Run a scenario against a live stack and judge it, with the reasons attached.

Every check here exists because something in this repository was once decided by hand, or was
decided wrongly. The point is not coverage for its own sake: it is that the same evidence gets
gathered the same way every time, so a regression argues with a number instead of a memory.

What it judges, and why each is here:

* **The scenario's own verdict** (`result.junit.xml`). The thing SSv2 says happened.
* **The ego drove and got somewhere.** A run can pass its scenario while the ego barely moves.
* **No node died.** `traffic_light_multi_camera_fusion` aborted at startup in *every* run for
  at least a fortnight, and nothing noticed until a scenario needed a traffic light: the error
  named a topic, not the parameter it came from, in one process out of ninety.
* **Diagnostics that are not OK.** The cascade behind issue 016 was fifteen non-OK nodes.
* **Longitudinal tracking.** Issue 019 was a factor-of-two error in the pedal conversion that
  no pass/fail verdict ever showed.

Traps this deliberately avoids, each one having produced a confident wrong answer already:

* A despawned CARLA actor keeps reporting `is_alive` and then returns garbage -- position
  (0, 0), a steering value of 5.7e28, handbrake true. That reads exactly like a stalled ego,
  and completed runs were scored as stalls because of it. The ego is looked up every tick and
  its disappearance means the run ended.
* A fresh CARLA client cannot read a world in synchronous mode: its first `GetWorld()` waits
  for a snapshot that only a tick delivers. Everything here that can come from ROS does.
* `DiagnosticStatus.level` is a byte in Humble's Python bindings, not an int.

Usage:
    acceptance.py --scenario path/to.xosc [--domain 1] [--runs 3] [--timeout 180]

Exit status is 0 only if every run passed every check.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry

JUNIT = Path("/tmp/scenario_test_runner/result.junit.xml")
MIN_PEAK_SPEED = 1.0     # m/s; below this the ego never really drove
MIN_DISTANCE = 20.0      # m; and it has to have gone somewhere


def diag_level(status) -> int:
    """DiagnosticStatus.level is a byte in Humble, an int elsewhere."""
    lvl = status.level
    return lvl if isinstance(lvl, int) else int.from_bytes(lvl, "little")


def observe(domain: int, duration: float) -> dict:
    """Watch one run from ROS alone and return what it did."""
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    rclpy.init()
    node = rclpy.create_node("acceptance_%d" % os.getpid())
    seen = {"peak": 0.0, "first": None, "last": None, "samples": 0}
    diags: dict[str, tuple[int, str]] = {}

    def on_velocity(m):
        seen["peak"] = max(seen["peak"], abs(m.longitudinal_velocity))
        seen["samples"] += 1

    def on_odom(m):
        p = m.pose.pose.position
        if seen["first"] is None:
            seen["first"] = (p.x, p.y)
        seen["last"] = (p.x, p.y)

    def on_diag(m):
        for s in m.status:
            level = diag_level(s)
            if level > 0:
                diags[s.name] = (level, s.message)
            else:
                diags.pop(s.name, None)

    node.create_subscription(VelocityReport, "/vehicle/status/velocity_status", on_velocity, 10)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 10)
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)

    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    distance = 0.0
    if seen["first"] and seen["last"]:
        distance = math.dist(seen["first"], seen["last"])
    return {
        "peak_speed": seen["peak"],
        "distance": distance,
        "status_samples": seen["samples"],
        "diagnostics": diags,
    }


def junit_verdict() -> tuple[bool, str]:
    if not JUNIT.exists():
        return False, "no result.junit.xml: the scenario never reported"
    text = JUNIT.read_text()
    if 'failures="0"' in text and 'errors="0"' in text:
        return True, "scenario passed"
    return False, "scenario reported a failure or error"


def dead_nodes(play_log: Path) -> list[str]:
    """Nodes whose stderr holds a fatal error, whatever the launcher reported."""
    dead = []
    runs = sorted((p for p in play_log.glob("*/node") if p.is_dir()),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    if not runs:
        return dead
    for node_dir in runs[0].iterdir():
        err = node_dir / "err"
        if not err.is_file():
            continue
        try:
            text = err.read_text(errors="replace")
        except OSError:
            continue
        for marker in ("terminate called", "what():", "Traceback (most recent call last)"):
            if marker in text:
                first = next((ln for ln in text.splitlines() if marker in ln), marker)
                dead.append("%s: %s" % (node_dir.name, first.strip()[:100]))
                break
    return dead


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--domain", type=int, default=1)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--play-log", default="play_log/ego")
    ap.add_argument("--json", help="write the full result here")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    results = []
    for run in range(1, args.runs + 1):
        print("=== run %d/%d ===" % (run, args.runs), flush=True)

        health = subprocess.run([sys.executable, str(here / "carla_health.py")],
                                capture_output=True, text=True)
        print("  " + health.stdout.strip())
        if health.returncode != 0:
            results.append({"run": run, "ok": False, "why": ["CARLA is not fit to run against"]})
            continue

        subprocess.run(["rm", "-rf", "/tmp/scenario_test_runner"], check=False)
        launched = subprocess.Popen(
            ["setsid", "--fork", "just", "scenario", args.scenario],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        launched.wait()
        time.sleep(20)          # the interpreter has to reach Initialize before anything moves

        seen = observe(args.domain, args.timeout)
        passed, why_verdict = junit_verdict()
        dead = dead_nodes(Path(args.play_log))
        errors = {n: m for n, (lvl, m) in seen["diagnostics"].items() if lvl >= 2}

        why = []
        if not passed:
            why.append(why_verdict)
        if seen["status_samples"] == 0:
            why.append("no vehicle status in domain %d: the bridge never found the ego"
                       % args.domain)
        if seen["peak_speed"] < MIN_PEAK_SPEED:
            why.append("ego never drove (peak %.2f m/s)" % seen["peak_speed"])
        if seen["distance"] < MIN_DISTANCE:
            why.append("ego went nowhere (%.1f m)" % seen["distance"])
        for d in dead:
            why.append("node died -- %s" % d)

        ok = not why
        results.append({"run": run, "ok": ok, "why": why,
                        "peak_speed": round(seen["peak_speed"], 2),
                        "distance": round(seen["distance"], 1),
                        "diagnostic_errors": errors})
        print("  verdict: %s   peak %.2f m/s   travelled %.1f m"
              % ("PASS" if ok else "FAIL", seen["peak_speed"], seen["distance"]))
        for w in why:
            print("    - %s" % w)
        # Reported but not failed: these are noisy on a healthy stack, and a check that cries
        # wolf gets switched off. They are here to be read when something else fails.
        for name, msg in sorted(errors.items())[:5]:
            print("    (diagnostic) %s: %s" % (name[:56], msg[:60]))
        time.sleep(10)

    passed_n = sum(1 for r in results if r["ok"])
    print("\n%d/%d runs passed" % (passed_n, len(results)))
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1))
    return 0 if passed_n == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
