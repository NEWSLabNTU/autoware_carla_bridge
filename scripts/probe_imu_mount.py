#!/usr/bin/env python3
"""Confirm what CARLA's IMU compass is the heading OF, and how to find the sensor actor.

Spawns a vehicle and attaches an IMU with a deliberate yaw offset, the way acb_bridge
mounts `carla/imu_link`. Then checks:

  1. Whether `world.get_actors().filter("sensor.other.imu")` + `actor.parent` finds it
     (the lookup check_vehicle_interface.py's find_imu was using, which returned nothing).
  2. Whether `compass` follows the SENSOR's heading or the VEHICLE's.
  3. Whether `pi/2 - compass` reproduces the sensor's ROS yaw, which is what acb publishes.
"""
import math
import sys

import carla

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
# The mount acb uses for carla/imu_link, read live off TF: RPY [-3.141, -0.015, 3.105].
MOUNT_YAW_ROS = 3.105
MOUNT_ROLL_ROS = -3.141

client = carla.Client("localhost", PORT)
client.set_timeout(30.0)
world = client.get_world()

original = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

vehicle = imu = None
try:
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn)

    imu_bp = world.get_blueprint_library().find("sensor.other.imu")
    for attr in ("noise_gyro_stddev_x", "noise_gyro_stddev_y", "noise_gyro_stddev_z"):
        if imu_bp.has_attribute(attr):
            imu_bp.set_attribute(attr, "0.0")

    # ROS -> CARLA for the mount rotation: roll and yaw flip sign, degrees.
    mount = carla.Transform(
        carla.Location(x=0.9, y=0.0, z=2.0),
        carla.Rotation(roll=math.degrees(-MOUNT_ROLL_ROS),
                       pitch=0.0,
                       yaw=math.degrees(-MOUNT_YAW_ROS)),
    )
    imu = world.spawn_actor(imu_bp, mount, attach_to=vehicle)

    latest = {}
    imu.listen(lambda m: latest.update(compass=m.compass))
    for _ in range(10):
        world.tick()

    print("=== can find_imu's lookup see the sensor? ===")
    found = []
    for actor in world.get_actors().filter("sensor.other.imu"):
        parent = actor.parent
        print(f"  actor {actor.id} type={actor.type_id} "
              f"parent={None if parent is None else parent.id} "
              f"(vehicle is {vehicle.id})")
        if parent is not None and parent.id == vehicle.id:
            found.append(actor)
    print(f"  filter+parent matched: {len(found)}")
    if not found:
        print("  -> this is why find_imu returned None; parent is not resolved here")

    print()
    print("=== compass: whose heading is it? ===")
    print(f"{'vehicle yaw':>12} {'sensor yaw':>11} {'compass':>9} "
          f"{'pi/2-compass':>13} {'-sensor(ros)':>13} {'-vehicle(ros)':>14}")
    for yaw_deg in (0.0, -90.0, 90.0, 180.0):
        vehicle.set_transform(carla.Transform(
            spawn.location + carla.Location(z=0.5), carla.Rotation(yaw=yaw_deg)))
        for _ in range(5):
            world.tick()
        c = latest.get("compass", float("nan"))
        v_yaw = vehicle.get_transform().rotation.yaw
        s_yaw = imu.get_transform().rotation.yaw
        ros_from_compass = math.degrees(math.pi / 2 - c)
        print(f"{v_yaw:12.2f} {s_yaw:11.2f} {math.degrees(c):9.2f} "
              f"{ros_from_compass:13.2f} {-s_yaw:13.2f} {-v_yaw:14.2f}")

    print()
    print("  'pi/2-compass' should equal '-sensor(ros)', not '-vehicle(ros)'.")
    print("  Both columns are wrapped to (-180, 180] below:")

    def wrap(d):
        return (d + 180.0) % 360.0 - 180.0

    err_sensor = err_vehicle = 0.0
    n = 0
    for yaw_deg in (0.0, -90.0, 90.0, 180.0, 45.0, -135.0):
        vehicle.set_transform(carla.Transform(
            spawn.location + carla.Location(z=0.5), carla.Rotation(yaw=yaw_deg)))
        for _ in range(5):
            world.tick()
        c = latest["compass"]
        ros_from_compass = math.degrees(math.pi / 2 - c)
        err_sensor += abs(wrap(ros_from_compass - (-imu.get_transform().rotation.yaw)))
        err_vehicle += abs(wrap(ros_from_compass - (-vehicle.get_transform().rotation.yaw)))
        n += 1
    print(f"  mean |pi/2-compass  -  (-sensor yaw)| = {err_sensor / n:7.3f} deg")
    print(f"  mean |pi/2-compass  -  (-vehicle yaw)| = {err_vehicle / n:7.3f} deg")

finally:
    if imu is not None:
        imu.stop()
        imu.destroy()
    if vehicle is not None:
        vehicle.destroy()
    world.apply_settings(original)
