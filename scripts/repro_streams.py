#!/usr/bin/env python3
"""Minimal reproducer for the orphaned-stream error storm.

No ROS, no Autoware: one client, a vehicle, four sensors, torn down two different ways.
Slices the server log per phase so each phase's errors — and the stream ids in them — are
attributable.

  repro_streams.py [cycles] [stop|nostop]
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
            f.seek(0, 2)
            return f.tell()
    except OSError:
        return 0

def slice_since(start):
    """Errors emitted since `start`, as {stream_id: count}."""
    try:
        with open(LOG, "rb") as f:
            f.seek(start)
            data = f.read()
    except OSError:
        return {}, 0
    ids = {}
    for m in re.finditer(rb"no stream available with id (\d+)", data):
        k = int(m.group(1))
        ids[k] = ids.get(k, 0) + 1
    return ids, len(data)

def phase(label, start):
    ids, nbytes = slice_since(start)
    total = sum(ids.values())
    top = " ".join(f"id{k}={v}" for k, v in sorted(ids.items()))
    print(f"  {label:<26} {nbytes/1024/1024:8.2f} MB  errors={total:<9} {top}")
    return offset()

client = carla.Client("localhost", 2000)
client.set_timeout(30.0)
world = client.get_world()
original = world.get_settings()
s = world.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = 0.05
world.apply_settings(s)

bl = world.get_blueprint_library()
spawn = world.get_map().get_spawn_points()[0]
SENSORS = ["sensor.camera.rgb", "sensor.other.gnss", "sensor.other.imu", "sensor.lidar.ray_cast"]

print(f"mode={MODE}  cycles={CYCLES}")
try:
    for cycle in range(1, CYCLES + 1):
        print(f"cycle {cycle}")
        mark = offset()

        vehicle = world.spawn_actor(bl.find("vehicle.tesla.model3"), spawn)
        sensors = []
        for t in SENSORS:
            bp = bl.find(t)
            if bp.has_attribute("sensor_tick"):
                bp.set_attribute("sensor_tick", "0.0")
            sen = world.spawn_actor(bp, carla.Transform(carla.Location(z=2.0)), attach_to=vehicle)
            sen.listen(lambda data: None)
            sensors.append(sen)
        for _ in range(20):
            world.tick()
        mark = phase("spawn+listen+20 ticks", mark)

        if MODE == "stop":
            for sen in sensors:
                sen.stop()
            mark = phase("stop() all sensors", mark)

        for sen in sensors:
            sen.destroy()
        vehicle.destroy()
        for _ in range(10):
            world.tick()
        mark = phase("destroy sensors+vehicle", mark)

        for _ in range(60):          # ~3 s of ticking with nothing attached
            world.tick()
        mark = phase("60 ticks, nothing alive", mark)

        time.sleep(5)                # paused: no ticks at all
        mark = phase("5 s paused, no ticks", mark)
finally:
    world.apply_settings(original)
