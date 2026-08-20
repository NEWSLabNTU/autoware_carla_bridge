"""Is CARLA's steering rate limit in the input domain or the wheel-angle domain?

The step response's per-tick wheel travel decays slightly (8.41, 7.85, 7.45 ...), which is
what a CONSTANT slew of the normalized steer input looks like after passing through the
Ackermann map, whose gain falls with angle. Invert the map and check.
"""
import carla, math

DT = 0.05
MAX_STEER = math.radians(70.0)
T_OVER_L = 0.5547

def eff(cmd):
    inner = abs(cmd) * MAX_STEER
    if inner <= 1e-9:
        return 0.0
    outer = math.atan(1.0 / (1.0 / math.tan(inner) + T_OVER_L))
    return math.degrees(0.5 * (inner + outer))

def inv(angle_deg):
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if eff(mid) < angle_deg:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

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
try:
    for _ in range(20):
        w.tick()
    for target in (0.8, 1.0):
        for _ in range(30):
            v.apply_control(carla.VehicleControl(steer=0.0, brake=1.0)); w.tick()
        print(f"\nstep to steer={target}")
        print(f"  {'tick':>4} {'wheel_deg':>10} {'implied_cmd':>12} {'d_cmd':>7}")
        prev = 0.0
        for i in range(12):
            v.apply_control(carla.VehicleControl(steer=target, brake=1.0)); w.tick()
            a = 0.5 * (abs(v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
                       + abs(v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)))
            cmd = inv(a)
            print(f"  {i+1:4d} {a:10.2f} {cmd:12.4f} {cmd-prev:7.4f}")
            prev = cmd
finally:
    v.destroy()
    w.apply_settings(orig)
