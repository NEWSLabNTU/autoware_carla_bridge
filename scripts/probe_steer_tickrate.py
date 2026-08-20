"""Is CARLA's 0.125-per-tick steering slew per TICK or per SECOND?

It matters: per tick means the lag is set by fixed_delta_seconds, which we choose, and a
faster simulation step would shrink it proportionally. Per second means the tick rate is
irrelevant and the lag is inherent.
"""
import carla, math

MAX_STEER = math.radians(70.0); T_OVER_L = 0.5547

def eff(cmd):
    inner = abs(cmd) * MAX_STEER
    if inner <= 1e-9: return 0.0
    outer = math.atan(1.0 / (1.0 / math.tan(inner) + T_OVER_L))
    return math.degrees(0.5 * (inner + outer))

def inv(a):
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if eff(mid) < a: lo = mid
        else: hi = mid
    return 0.5 * (lo + hi)

c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world(); orig = w.get_settings()
bp = w.get_blueprint_library().find("vehicle.tesla.model3")

print(f"{'dt(s)':>7} {'Hz':>5} {'ticks_to_full':>14} {'ms_to_full':>11} {'d_cmd/tick':>11} {'d_cmd/s':>9}")
for dt in (0.1, 0.05, 0.025):
    s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = dt
    w.apply_settings(s)
    v = None
    for sp in w.get_map().get_spawn_points():
        v = w.try_spawn_actor(bp, sp)
        if v is not None: break
    if v is None:
        print(f"{dt:7.3f}  no free spawn point"); continue
    try:
        for _ in range(20): w.tick()
        for _ in range(30):
            v.apply_control(carla.VehicleControl(steer=0.0, brake=1.0)); w.tick()
        steps = []
        prev = 0.0
        for i in range(30):
            v.apply_control(carla.VehicleControl(steer=1.0, brake=1.0)); w.tick()
            a = 0.5*(abs(v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
                     + abs(v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)))
            cmd = inv(a); steps.append(cmd - prev); prev = cmd
            if cmd >= 0.999:
                break
        n = len(steps)
        per_tick = max(steps)
        print(f"{dt:7.3f} {1/dt:5.0f} {n:14d} {n*dt*1000:11.0f} {per_tick:11.4f} {per_tick/dt:9.2f}")
    finally:
        v.destroy()
w.apply_settings(orig)
