#!/usr/bin/env python3
"""Two clients, as in the real stack: A listens to the sensors, B destroys them.

acb attaches and listens; csb owns the ego's lifetime and despawns it. Neither client
alone reproduced the storm, so this tests the split itself.

  repro_two_clients.py [cycles] [stop|nostop]
"""
import os
import re, sys, time
import carla

LOG = os.environ.get("CARLA_LOG", "/tmp/carla.log")
CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 2
MODE = sys.argv[2] if len(sys.argv) > 2 else "nostop"

def offset():
    try:
        with open(LOG, "rb") as f:
            f.seek(0, 2); return f.tell()
    except OSError:
        return 0

def phase(label, start):
    try:
        with open(LOG, "rb") as f:
            f.seek(start); data = f.read()
    except OSError:
        data = b""
    ids = {}
    for m in re.finditer(rb"no stream available with id (\d+)", data):
        k = int(m.group(1)); ids[k] = ids.get(k, 0) + 1
    total = sum(ids.values())
    top = " ".join(f"id{k}={v}" for k, v in sorted(ids.items()))
    print(f"  {label:<28} {len(data)/1024/1024:7.2f} MB  errors={total:<8} {top}")
    return offset()

# A: the listener (acb).  B: the owner/destroyer (csb).
A = carla.Client("localhost", 2000); A.set_timeout(30.0)
B = carla.Client("localhost", 2000); B.set_timeout(30.0)
wa, wb = A.get_world(), B.get_world()

original = wb.get_settings()
s = wb.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05
wb.apply_settings(s)

bl_b = wb.get_blueprint_library()
spawn = wb.get_map().get_spawn_points()[0]
SENSORS = ["sensor.camera.rgb", "sensor.other.gnss", "sensor.other.imu", "sensor.lidar.ray_cast"]

print(f"mode={MODE}  cycles={CYCLES}   (A listens, B destroys)")
try:
    for cycle in range(1, CYCLES + 1):
        print(f"cycle {cycle}")
        mark = offset()

        # B spawns the vehicle, as the scenario runner does.
        vehicle = wb.spawn_actor(bl_b.find("vehicle.tesla.model3"), spawn)
        wb.tick()

        # A finds it and attaches its own sensors, as the bridge does.
        veh_a = wa.get_actor(vehicle.id)
        bl_a = wa.get_blueprint_library()
        sensors_a = []
        for t in SENSORS:
            bp = bl_a.find(t)
            if bp.has_attribute("sensor_tick"):
                bp.set_attribute("sensor_tick", "0.0")
            sen = wa.spawn_actor(bp, carla.Transform(carla.Location(z=2.0)), attach_to=veh_a)
            sen.listen(lambda data: None)
            sensors_a.append(sen)
        for _ in range(20):
            wb.tick()
        mark = phase("A listening, 20 ticks", mark)

        if MODE == "stop":
            for sen in sensors_a:
                sen.stop()
            mark = phase("A stops its sensors", mark)

        # B destroys the vehicle *and its children*, which is destroy_with_children.
        for child in wb.get_actors().filter("sensor.*"):
            if child.parent is not None and child.parent.id == vehicle.id:
                child.destroy()
        vehicle.destroy()
        for _ in range(10):
            wb.tick()
        mark = phase("B destroys children+vehicle", mark)

        for _ in range(60):
            wb.tick()
        mark = phase("60 ticks, nothing alive", mark)

        time.sleep(5)
        mark = phase("5 s paused", mark)
finally:
    wb.apply_settings(original)
