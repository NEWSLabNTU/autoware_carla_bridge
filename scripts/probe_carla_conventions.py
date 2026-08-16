#!/usr/bin/env python3
"""Measure CARLA's angular-velocity units and compass convention against ground truth.

Answers two questions the docs disagree on:

  1. Does Actor.get_angular_velocity() return rad/s or deg/s?  Compare it against the
     yaw differentiated over one fixed simulation step, and against the IMU gyroscope,
     which is documented in rad/s.
  2. What is the relation between IMU compass and actor yaw?  Needed to convert compass
     into a ROS map-frame heading.
"""
import math
import sys

import carla

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
DT = 0.05

client = carla.Client("localhost", PORT)
client.set_timeout(30.0)
world = client.get_world()

original = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)

vehicle = None
imu = None
try:
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn)

    imu_bp = world.get_blueprint_library().find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.0")
    # No noise, so the comparison is exact.
    for attr in ("noise_accel_stddev_x", "noise_accel_stddev_y", "noise_accel_stddev_z",
                 "noise_gyro_stddev_x", "noise_gyro_stddev_y", "noise_gyro_stddev_z",
                 "noise_gyro_bias_x", "noise_gyro_bias_y", "noise_gyro_bias_z"):
        if imu_bp.has_attribute(attr):
            imu_bp.set_attribute(attr, "0.0")
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

    latest = {}
    imu.listen(lambda m: latest.update(gyro=m.gyroscope, compass=m.compass,
                                       accel=m.accelerometer))

    world.tick()

    # --- Compass vs yaw, standing still, at four headings ------------------------
    print("=== compass vs actor yaw (stationary) ===")
    print(f"{'carla_yaw':>10} {'compass_rad':>12} {'compass_deg':>12} "
          f"{'compass-yaw':>12} {'pi/2 - compass':>15} {'-yaw (ros)':>11}")
    for yaw_deg in (0.0, -90.0, 90.0, 180.0):
        tf = carla.Transform(spawn.location + carla.Location(z=0.5),
                             carla.Rotation(yaw=yaw_deg))
        vehicle.set_transform(tf)
        for _ in range(4):
            world.tick()
        c = latest.get("compass", float("nan"))
        ros_yaw_from_compass = math.pi / 2 - c
        print(f"{yaw_deg:10.1f} {c:12.4f} {math.degrees(c):12.2f} "
              f"{math.degrees(c) - yaw_deg:12.2f} "
              f"{math.degrees(ros_yaw_from_compass):15.2f} {-yaw_deg:11.2f}")

    # --- Angular velocity units, in a steady turn --------------------------------
    print()
    print("=== angular velocity units (steady left/right turn) ===")
    vehicle.set_transform(carla.Transform(spawn.location + carla.Location(z=0.5),
                                          carla.Rotation(yaw=0.0)))
    for _ in range(10):
        world.tick()
    vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.6))

    prev_yaw = None
    print(f"{'step':>4} {'yaw_deg':>9} {'d_yaw/dt(deg/s)':>16} {'ang_vel.z':>11} "
          f"{'gyro.z(rad/s)':>14} {'angvel/gyro':>12}")
    samples = []
    for step in range(80):
        world.tick()
        yaw = vehicle.get_transform().rotation.yaw
        av = vehicle.get_angular_velocity()
        gyro = latest.get("gyro")
        if prev_yaw is not None and gyro is not None:
            dyaw = (yaw - prev_yaw + 180.0) % 360.0 - 180.0
            rate_deg_s = dyaw / DT
            if step > 40 and abs(rate_deg_s) > 5.0:
                samples.append((rate_deg_s, av.z, gyro.z))
            if step % 10 == 0:
                ratio = av.z / gyro.z if abs(gyro.z) > 1e-6 else float("nan")
                print(f"{step:4d} {yaw:9.2f} {rate_deg_s:16.3f} {av.z:11.4f} "
                      f"{gyro.z:14.4f} {ratio:12.3f}")
        prev_yaw = yaw

    if samples:
        n = len(samples)
        mean_rate_deg = sum(s[0] for s in samples) / n
        mean_av = sum(s[1] for s in samples) / n
        mean_gyro = sum(s[2] for s in samples) / n
        print()
        print(f"over {n} samples in the steady turn:")
        print(f"  differentiated yaw rate : {mean_rate_deg:8.4f} deg/s "
              f"= {math.radians(mean_rate_deg):.4f} rad/s")
        print(f"  get_angular_velocity().z: {mean_av:8.4f}")
        print(f"  imu gyroscope.z         : {mean_gyro:8.4f}  (documented rad/s)")
        print(f"  angular_velocity / differentiated-rad-per-s : "
              f"{mean_av / math.radians(mean_rate_deg):.4f}")
        print(f"  angular_velocity / differentiated-deg-per-s : "
              f"{mean_av / mean_rate_deg:.4f}")
        print(f"  angular_velocity / gyro.z                   : "
              f"{mean_av / mean_gyro:.4f}")
        print()
        verdict = ("deg/s" if abs(mean_av / mean_rate_deg - 1.0) < 0.1
                   else "rad/s" if abs(mean_av / math.radians(mean_rate_deg) - 1.0) < 0.1
                   else "NEITHER -- look at the numbers")
        print(f"  VERDICT: get_angular_velocity() is in {verdict}")
        print(f"  gyro.z sign vs yaw rate: gyro.z={mean_gyro:.4f}, "
              f"yaw rate={math.radians(mean_rate_deg):.4f} rad/s "
              f"({'same' if mean_gyro * mean_rate_deg > 0 else 'opposite'} sign)")

    # --- Body-frame velocity check ------------------------------------------------
    print()
    print("=== world vs body velocity ===")
    v = vehicle.get_velocity()
    tf = vehicle.get_transform()
    fwd = tf.get_forward_vector()
    right = tf.get_right_vector()
    lon = v.x * fwd.x + v.y * fwd.y + v.z * fwd.z
    lat = v.x * right.x + v.y * right.y + v.z * right.z
    speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    print(f"  yaw={tf.rotation.yaw:.1f} deg  world v=({v.x:.3f}, {v.y:.3f}, {v.z:.3f})")
    print(f"  |v|={speed:.3f}  body longitudinal={lon:.3f}  body right={lat:.3f}")
    print(f"  old code would have published longitudinal={speed:.3f}, "
          f"lateral={-v.y:.3f}; correct is longitudinal={lon:.3f}, lateral={-lat:.3f}")

    # --- Steer command vs measured wheel angle ------------------------------------
    print()
    print("=== commanded steer vs measured wheel angle ===")
    phys = vehicle.get_physics_control()
    max_steer = max(w.max_steer_angle for w in phys.wheels)
    print(f"  physics max_steer_angle over wheels: {max_steer:.2f} deg "
          f"= {math.radians(max_steer):.4f} rad")
    for cmd in (0.0, 0.25, 0.6, 1.0):
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=cmd))
        for _ in range(20):
            world.tick()
        fl = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        applied = vehicle.get_control().steer
        print(f"  steer cmd={cmd:.2f} applied={applied:.3f} "
              f"FL={fl:7.2f} deg FR={fr:7.2f} deg  mean={0.5 * (fl + fr):7.2f} deg "
              f"({math.radians(0.5 * (fl + fr)):.4f} rad)")

finally:
    if imu is not None:
        imu.stop()
        imu.destroy()
    if vehicle is not None:
        vehicle.destroy()
    world.apply_settings(original)
