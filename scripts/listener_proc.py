#!/usr/bin/env python3
# The listener, in its own process, so it can be killed outright.
import sys, time, carla
vid = int(sys.argv[1])
A = carla.Client("localhost", 2000); A.set_timeout(30.0)
wa = A.get_world()
veh = wa.get_actor(vid)
bl = wa.get_blueprint_library()
keep = []
for t in ["sensor.camera.rgb", "sensor.other.gnss", "sensor.other.imu", "sensor.lidar.ray_cast"]:
    bp = bl.find(t)
    if bp.has_attribute("sensor_tick"):
        bp.set_attribute("sensor_tick", "0.0")
    sen = wa.spawn_actor(bp, carla.Transform(carla.Location(z=2.0)), attach_to=veh)
    sen.listen(lambda d: None)
    keep.append(sen)
print("listener ready", flush=True)
while True:
    time.sleep(1)
