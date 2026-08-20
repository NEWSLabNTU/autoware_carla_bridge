#!/usr/bin/env python3
"""Verify that acb_bridge's control latency is a filling subscription queue.

Measured behaviour: the command-to-wheel delay grows 1, 4, 5, 8, 10 ticks and then pins at
exactly 10 -- the rclrs default history depth, which `.reliable()` does not change.

If that reading is right, the delay is set by backlog rather than by any fixed cost, so:

  phase 1  publish every tick        -> backlog grows, delay climbs to the queue depth
  phase 2  stop publishing           -> backlog drains
  phase 3  publish every 4th tick    -> arrival below consumption, delay stays small

A fixed pipeline cost would instead give the same delay in phase 1 and phase 3.
"""
import math, sys, time
import carla, rclpy
from autoware_control_msgs.msg import Control

DT = 0.1
AMP = 0.20

rclpy.init()
n = rclpy.create_node("probe_queue_verify")
pub = n.create_publisher(Control, "/control/command/control_cmd", 1)
client = carla.Client("localhost", 2000); client.set_timeout(20.0)
world = client.get_world()

v = None
for a in world.get_actors().filter("vehicle.*"):
    if a.attributes.get("role_name") == "hero":
        v = a
if v is None:
    print("no hero vehicle"); raise SystemExit(1)

orig = world.get_settings()
s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = DT
world.apply_settings(s)

def wheel():
    fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    return -math.radians(0.5 * (fl + fr))

def publish(a):
    m = Control(); t = n.get_clock().now().to_msg()
    m.stamp = t; m.lateral.stamp = t; m.longitudinal.stamp = t
    m.lateral.steering_tire_angle = float(a)
    pub.publish(m)

def run_phase(name, half_period, every, cycles):
    """Square wave with `half_period` ticks per level, publishing every `every` ticks."""
    print(f"\n--- {name}: publish every {every} tick(s), half-period {half_period} ticks")
    cmds, meas, tick = [], [], 0
    for cyc in range(cycles):
        for half in (0, 1):
            target = AMP if half == 0 else -AMP
            for k in range(half_period):
                if tick % every == 0:
                    publish(target)
                rclpy.spin_once(n, timeout_sec=0.001)
                world.tick()
                cmds.append(target); meas.append(wheel()); tick += 1
    delays = []
    for k in range(1, len(cmds)):
        if cmds[k] == cmds[k-1]:
            continue
        d = None
        for j in range(k, min(k + 25, len(meas))):
            if abs(meas[j] - cmds[k]) < 0.02:
                d = j - k; break
        delays.append(d)
        print(f"    flip tick {k:3d} -> {cmds[k]:+.2f}: delay {str(d):>4} ticks"
              f" ({'n/a' if d is None else f'{d*DT*1000:.0f} ms'})")
    return delays

try:
    for _ in range(10):
        publish(0.0); world.tick()
    d1 = run_phase("PHASE 1  every tick", 12, 1, 4)
    print("\n--- PHASE 2  no commands for 40 ticks (drain)")
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.001); world.tick()
    d3 = run_phase("PHASE 3  every 4th tick", 12, 4, 4)
    def summ(tag, d):
        g = [x for x in d if x is not None]
        if not g: print(f"  {tag}: no resolved transitions"); return
        print(f"  {tag}: first {g[0]}, last {g[-1]}, max {max(g)} ticks")
    print("\nSUMMARY")
    summ("phase 1 (every tick) ", d1)
    summ("phase 3 (every 4th)  ", d3)
    print("\n  queue hypothesis  -> phase 1 climbs to ~10, phase 3 stays small")
    print("  fixed pipeline    -> both phases show the same delay")
finally:
    world.apply_settings(orig)
    rclpy.shutdown()
