#!/usr/bin/env python3
"""Measure the ROS -> acb_bridge -> CARLA command path latency with a signal we control.

CARLA's steering actuator tracks any Autoware command within one tick (issue 016), so the
~478 ms command-to-wheel figure has to come from the pipeline. Measuring it through a live
scenario means measuring an input nobody chose; this injects a known square wave on
/control/command/control_cmd instead, drives the ticks itself at the production 10 Hz, and
records the wheel angle per tick. Everything is timed in CARLA sim time.

Requires: acb_bridge running and attached to a hero vehicle, and vehicle_cmd_gate stopped so
nothing else publishes on the topic.
"""
import math, sys, time
import carla, rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control

DT = 0.1                 # production step: the adapter sets fixed_delta_seconds=0.1
PERIOD_TICKS = 20        # square wave half-period, in ticks (2.0 s)
CYCLES = 6
AMP = 0.20               # rad of tire angle

rclpy.init()
n = rclpy.create_node("probe_bridge_latency")
pub = n.create_publisher(Control, "/control/command/control_cmd", 1)

client = carla.Client("localhost", 2000); client.set_timeout(20.0)
world = client.get_world()

def hero():
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == "hero":
            return a
    return None

v = hero()
if v is None:
    print("no hero vehicle in CARLA"); raise SystemExit(1)
print(f"hero actor {v.id}")

orig = world.get_settings()
s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = DT
world.apply_settings(s)

def wheel_rad():
    fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    return -math.radians(0.5 * (fl + fr))     # CARLA right-positive -> ROS left-positive

_next = [time.time()]

def _pace():
    """Hold the tick rate to wall time, as SSv2 does in production (RTF ~1.0)."""
    _next[0] += DT
    slack = _next[0] - time.time()
    if slack > 0:
        time.sleep(slack)
    else:
        _next[0] = time.time()


def publish(angle):
    m = Control()
    now = n.get_clock().now().to_msg()
    m.stamp = now
    m.lateral.stamp = now
    m.lateral.steering_tire_angle = float(angle)
    m.longitudinal.stamp = now
    m.longitudinal.velocity = 0.0
    m.longitudinal.acceleration = 0.0
    pub.publish(m)

try:
    for _ in range(10):
        publish(0.0); world.tick(); _pace()
    cmds, meas = [], []
    tick = 0
    for cyc in range(CYCLES):
        for half in (0, 1):
            target = AMP if half == 0 else -AMP
            for _ in range(PERIOD_TICKS):
                publish(target)
                rclpy.spin_once(n, timeout_sec=0.001)
                world.tick()
                _pace()
                cmds.append(target)
                meas.append(wheel_rad())
                tick += 1
    print(f"\nticks: {tick}  dt: {DT}s  amplitude: {AMP} rad\n")
    print("  shift  ticks     ms     rms(rad)")
    best = None
    for shift in range(0, 11):
        pairs = [(cmds[i], meas[i + shift]) for i in range(len(cmds) - shift)]
        rms = math.sqrt(sum((a - b) ** 2 for a, b in pairs) / len(pairs))
        if best is None or rms < best[1]:
            best = (shift, rms)
        print(f"  {'->' if shift==0 else '  '}     {shift:3d}  {shift*DT*1000:6.0f}  {rms:11.4f}")
    print(f"\nbest alignment: {best[0]} ticks = {best[0]*DT*1000:.0f} ms (rms {best[1]:.4f} rad)")
    # Every transition, so one lucky edge cannot stand in for the run.
    print("\nlatency at each command transition (ticks until the wheel reaches the new value):")
    delays = []
    for k in range(1, len(cmds)):
        if cmds[k] == cmds[k-1]:
            continue
        target = cmds[k]
        d = None
        for j in range(k, min(k + 15, len(meas))):
            if abs(meas[j] - target) < 0.02:
                d = j - k
                break
        delays.append(d)
        seq = " ".join(f"{meas[x]:+.3f}" for x in range(k, min(k+6, len(meas))))
        print(f"  flip at tick {k:3d} -> {target:+.2f} : delay {str(d):>4} ticks   wheel: {seq}")
    good = [d for d in delays if d is not None]
    if good:
        good.sort()
        print(f"\n  transitions: {len(delays)}, resolved: {len(good)}")
        print(f"  delay ticks: min {min(good)} median {good[len(good)//2]} max {max(good)}"
              f"  -> {good[len(good)//2]*DT*1000:.0f} ms median")
finally:
    world.apply_settings(orig)
    rclpy.shutdown()
