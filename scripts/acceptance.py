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
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry

JUNIT = Path("/tmp/scenario_test_runner/result.junit.xml")
MIN_PEAK_SPEED = 1.0     # m/s; below this the ego never really drove
MIN_DISTANCE = 20.0      # m; and it has to have gone somewhere

# Tracking-quality limits. These are the point of the harness rather than an extra: issue 019
# was a factor-of-two error in the acceleration-to-pedal conversion that no pass/fail verdict
# ever showed, and it lived in the bridge for as long as the bridge did.
#
# The numbers are set from measured runs on a healthy stack, widened so noise cannot fail a
# run. A threshold that fires on a good day gets switched off, and then catches nothing.
# 0.35 is chosen against both ends. Five healthy runs measured 0.064 to 0.142, so this is two
# and a half times the worst good day. Issue 019's defect measured 0.592 before it was fixed,
# so this is well under the thing it exists to catch -- a limit of 0.60, set from the healthy
# runs alone, would have let that regression pass by eight thousandths.
MAX_LONGITUDINAL_ERROR = 0.35   # m/s^2, median |commanded - delivered|
# Bracketed at both ends, like the longitudinal limit. Healthy runs measured 0.029 to 0.289.
# A characterised lateral fault -- `REPORT_MEASURED_STEERING=true`, which issue 009 settled as
# destabilising -- measured 2.187 to 13.277 over six runs. So 1.00 sits three times above the
# worst good day and twice below the mildest bad one, with nothing in between observed.
MAX_CROSS_TRACK = 1.00          # m, median distance from the planned trajectory
LONGITUDINAL_LAG_S = 0.30       # command to delivery; swept once, flat between 0.1 and 0.5
MIN_TRACKING_SAMPLES = 30       # below this the numbers are not worth judging


def diag_level(status) -> int:
    """DiagnosticStatus.level is a byte in Humble, an int elsewhere."""
    lvl = status.level
    return lvl if isinstance(lvl, int) else int.from_bytes(lvl, "little")


def observe(domain: int, duration: float, settle: float = 90.0) -> dict:
    """Watch one run from ROS alone and return what it did.

    Waits for the ego to exist before the measurement window opens. A fixed sleep does not
    work: a scenario takes anywhere from twenty seconds to over a minute to reach the point
    of spawning one, and a window that opens too early scores an empty run as a failure --
    which it did, and the run under it was fine.
    """
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

    # Longitudinal: what was asked for, and what the vehicle then did. Delivered acceleration
    # is differentiated from the bridge's own velocity report rather than read from CARLA --
    # checked against the server, the report matches its speed to a median of 0.0000 m/s and
    # the derivative matches its acceleration to 0.067 m/s^2, and using ROS avoids a fresh
    # CARLA client, which cannot read a synchronous world at all.
    commands: list[tuple[float, float]] = []
    speeds: list[tuple[float, float]] = []
    cross_track: list[float] = []
    traj: dict = {"points": []}

    def on_command(m):
        commands.append((time.time(), m.longitudinal.acceleration))

    def on_speed(m):
        speeds.append((time.time(), m.longitudinal_velocity))

    def on_trajectory(m):
        traj["points"] = [(p.pose.position.x, p.pose.position.y) for p in m.points]

    def on_odom_cross_track(m):
        pts = traj["points"]
        if len(pts) >= 2:
            p = m.pose.pose.position
            cross_track.append(min(math.dist((p.x, p.y), q) for q in pts))

    node.create_subscription(VelocityReport, "/vehicle/status/velocity_status", on_velocity, 10)
    node.create_subscription(VelocityReport, "/vehicle/status/velocity_status", on_speed, 10)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 10)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom_cross_track, 10)
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)
    node.create_subscription(Control, "/control/command/control_cmd", on_command, 20)
    node.create_subscription(
        Trajectory, "/planning/scenario_planning/trajectory", on_trajectory, 1)

    waiting = time.time()
    while seen["samples"] == 0 and time.time() - waiting < settle:
        rclpy.spin_once(node, timeout_sec=0.2)
    if seen["samples"] == 0:
        rclpy.shutdown()
        return {
            "peak_speed": 0.0, "distance": 0.0, "status_samples": 0,
            "diagnostics": diags, "longitudinal": {"samples": 0, "median_error": None},
            "cross_track": None,
        }

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
        "longitudinal": longitudinal_error(commands, speeds),
        "cross_track": median(cross_track) if cross_track else None,
    }


def median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def longitudinal_error(commands, speeds) -> dict:
    """Median |commanded - delivered| acceleration, over samples where the ego was moving.

    Standstill is excluded on purpose. A stopped car asked for -1 m/s^2 delivers 0, and that is
    the car being stopped rather than a tracking failure; counting it measures the metric.
    """
    derived = []
    for i in range(4, len(speeds)):
        (t0, v0), (t1, v1) = speeds[i - 4], speeds[i]
        if t1 - t0 > 1e-3:
            derived.append((0.5 * (t0 + t1), (v1 - v0) / (t1 - t0), v1))
    errors = []
    for t_cmd, requested in commands:
        target = t_cmd + LONGITUDINAL_LAG_S
        near = [(abs(t - target), a, v) for t, a, v in derived if abs(t - target) < 0.10]
        if not near:
            continue
        _, delivered, speed = min(near)
        if speed > 1.0:
            errors.append(abs(delivered - requested))
    if len(errors) < MIN_TRACKING_SAMPLES:
        return {"samples": len(errors), "median_error": None}
    return {"samples": len(errors), "median_error": median(errors)}


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
    ap.add_argument("--timeout", type=float, default=150.0,
                    help="how long to watch once the ego exists")
    ap.add_argument("--settle", type=float, default=120.0,
                    help="how long to wait for the ego to exist before giving up")
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
        seen = observe(args.domain, args.timeout, settle=args.settle)
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

        lon = seen["longitudinal"]
        if lon["median_error"] is None:
            print("    (tracking) too few longitudinal samples (%d) to judge" % lon["samples"])
        elif lon["median_error"] > MAX_LONGITUDINAL_ERROR:
            why.append("longitudinal tracking %.3f m/s^2 above the %.2f limit"
                       % (lon["median_error"], MAX_LONGITUDINAL_ERROR))
        if seen["cross_track"] is not None and seen["cross_track"] > MAX_CROSS_TRACK:
            why.append("cross-track %.3f m above the %.2f limit"
                       % (seen["cross_track"], MAX_CROSS_TRACK))

        ok = not why
        results.append({"run": run, "ok": ok, "why": why,
                        "peak_speed": round(seen["peak_speed"], 2),
                        "distance": round(seen["distance"], 1),
                        "longitudinal": seen["longitudinal"],
                        "cross_track": seen["cross_track"],
                        "diagnostic_errors": errors})
        lon_txt = ("%.3f" % seen["longitudinal"]["median_error"]
                   if seen["longitudinal"]["median_error"] is not None else "n/a")
        xt_txt = "%.3f" % seen["cross_track"] if seen["cross_track"] is not None else "n/a"
        print("  verdict: %s   peak %.2f m/s   travelled %.1f m   longitudinal %s m/s^2   "
              "cross-track %s m"
              % ("PASS" if ok else "FAIL", seen["peak_speed"], seen["distance"],
                 lon_txt, xt_txt))
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
