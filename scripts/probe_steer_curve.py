"""Measure CARLA's command -> tire angle map, including its speed derating."""
import carla, math
c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world()
# Nothing else is ticking this world; take ownership for the probe and hand it back.
_orig = w.get_settings()
_s = w.get_settings(); _s.synchronous_mode = True; _s.fixed_delta_seconds = 0.05
w.apply_settings(_s)
tick = w.tick
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
        tick()
    pc = v.get_physics_control()
    print("steering_curve:", [(p.x, p.y) for p in pc.steering_curve])
    print("wheels max_steer_angle:", [f"{wh.max_steer_angle:.2f}" for wh in pc.wheels])
    print()
    print(f"{'cmd':>6} {'FL':>8} {'FR':>8} {'mean_deg':>9} {'mean_rad':>9} {'rad/cmd':>8}")
    for cmd in [0.1, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        v.apply_control(carla.VehicleControl(throttle=0.0, steer=cmd, brake=1.0))
        for _ in range(25):
            tick()
        fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        mean = (abs(fl) + abs(fr)) / 2.0
        print(f"{cmd:6.2f} {fl:8.2f} {fr:8.2f} {mean:9.2f} {math.radians(mean):9.4f} "
              f"{math.radians(mean)/cmd:8.4f}")
finally:
    v.destroy()
    w.apply_settings(_orig)
