"""Does CARLA's steering_curve derate the commanded wheel angle at speed?

Holds one steer command while the vehicle is driven at several speeds, and reads the
physical wheel angles back. If the curve applies, FR should fall from cmd*70 deg as speed
rises, following steering_curve's [(0,1.0),(20,0.9),(60,0.8),(120,0.7)] in km/h.
"""
import carla, math
c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world()
orig = w.get_settings()
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05
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
    pc = v.get_physics_control()
    wh = pc.wheels
    # Wheel positions are world centimetres at the current pose; use them for geometry.
    import itertools
    fl, fr, rl = wh[0].position, wh[1].position, wh[2].position
    track = math.dist((fl.x, fl.y), (fr.x, fr.y)) / 100.0
    wheelbase = math.dist((fl.x, fl.y), (rl.x, rl.y)) / 100.0
    print(f"track={track:.3f} m  wheelbase={wheelbase:.3f} m  track/wheelbase={track/wheelbase:.4f}")
    print(f"{'cmd':>5} {'kmh':>7} {'FL':>7} {'FR':>7} {'FR/(cmd*70)':>12}")
    CMD = 0.5
    for target_kmh in [0.0, 10.0, 20.0, 40.0, 60.0]:
        tf = v.get_transform()
        fwd = tf.get_forward_vector()
        spd = target_kmh / 3.6
        for _ in range(30):
            v.set_target_velocity(carla.Vector3D(fwd.x * spd, fwd.y * spd, 0.0))
            v.apply_control(carla.VehicleControl(throttle=0.0, steer=CMD, brake=0.0))
            w.tick()
        vel = v.get_velocity()
        kmh = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
        a_fl = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        a_fr = v.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        print(f"{CMD:5.2f} {kmh:7.1f} {a_fl:7.2f} {a_fr:7.2f} {a_fr/(CMD*70.0):12.4f}")
finally:
    v.destroy()
    w.apply_settings(orig)
