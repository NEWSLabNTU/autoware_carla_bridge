#!/usr/bin/env python3
"""Characterise CARLA's steering actuator: first-order lag or slew-rate limit?

A ~478 ms delay was measured between Autoware's steering command and the physical wheel
angle (acb issue 016). The number alone does not say what kind of dynamics produce it, and
the two possibilities call for different fixes:

  first-order lag  -- fixed time constant, step size irrelevant, phase-compensable
  slew-rate limit  -- fixed deg/s, time to target scales with step size, a nonlinearity
                      no amount of phase lead fixes

This applies steer steps of several sizes and records the physical wheel angle every tick,
with no ROS anywhere in the path, so whatever it finds belongs to CARLA rather than to the
bridge.
"""
import carla, math, sys

STEPS = [0.1, 0.2, 0.4, 0.8]
TICKS = 40           # 2.0 s at the production 20 Hz
DT = 0.05

c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world()
orig = w.get_settings()
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = DT
w.apply_settings(s)

bp = w.get_blueprint_library().find("vehicle.tesla.model3")
v = None
for sp in w.get_map().get_spawn_points():
    v = w.try_spawn_actor(bp, sp)
    if v is not None:
        break
if v is None:
    raise SystemExit("no free spawn point")

def wheel():
    fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    return 0.5 * (abs(fl) + abs(fr))

try:
    for _ in range(20):
        w.tick()
    print(f"{'step':>6} {'final':>7} {'t63(ms)':>8} {'t95(ms)':>8} {'peak_rate(deg/s)':>17}")
    rows = []
    for step in STEPS:
        # Settle at zero first so every step starts from the same place.
        for _ in range(30):
            v.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            w.tick()
        trace = []
        for i in range(TICKS):
            v.apply_control(carla.VehicleControl(throttle=0.0, steer=step, brake=1.0))
            w.tick()
            trace.append(wheel())
        final = trace[-1]
        t63 = t95 = None
        for i, a in enumerate(trace):
            if t63 is None and a >= 0.63 * final:
                t63 = (i + 1) * DT * 1000
            if t95 is None and a >= 0.95 * final:
                t95 = (i + 1) * DT * 1000
        rate = max((trace[i] - trace[i - 1]) / DT for i in range(1, len(trace)))
        rows.append((step, final, t63, t95, rate))
        print(f"{step:6.2f} {final:7.2f} {str(t63):>8} {str(t95):>8} {rate:17.1f}")
        print(f"       first 10 ticks: {' '.join(f'{a:.2f}' for a in trace[:10])}")
    print()
    t63s = [r[2] for r in rows if r[2]]
    t95s = [r[3] for r in rows if r[3]]
    rates = [r[4] for r in rows]
    print(f"t63 spread: {min(t63s):.0f}-{max(t63s):.0f} ms   "
          f"t95 spread: {min(t95s):.0f}-{max(t95s):.0f} ms")
    print(f"peak rate spread: {min(rates):.1f}-{max(rates):.1f} deg/s")
    print()
    print("A constant t63 across step sizes means a first-order lag.")
    print("A constant peak rate with t95 growing with step size means a slew-rate limit.")
finally:
    v.destroy()
    w.apply_settings(orig)
