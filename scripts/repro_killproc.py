#!/usr/bin/env python3
"""Does killing the listening PROCESS clear the stale sessions?"""
import os, re, signal, subprocess, sys, time
import carla

S = os.environ.get("REPRO_DIR", os.path.dirname(os.path.abspath(__file__)))
LOG = os.environ.get("CARLA_LOG", "/tmp/carla.log")

def offset():
    with open(LOG, "rb") as f:
        f.seek(0, 2); return f.tell()

def rate(label, secs=4):
    a = offset(); time.sleep(secs); b = offset()
    with open(LOG, "rb") as f:
        f.seek(a); data = f.read(b - a)
    n = len(re.findall(rb"no stream available with id", data))
    print(f"  {label:<34} {(b-a)/1024/1024/secs:7.2f} MB/s  {n//secs:>8} errors/s")

B = carla.Client("localhost", 2000); B.set_timeout(30.0)
wb = B.get_world()
original = wb.get_settings()
s = wb.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05
wb.apply_settings(s)
try:
    vehicle = wb.spawn_actor(wb.get_blueprint_library().find("vehicle.tesla.model3"),
                             wb.get_map().get_spawn_points()[0])
    wb.tick()
    proc = subprocess.Popen([sys.executable, f"{S}/listener_proc.py", str(vehicle.id)],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    proc.stdout.readline()          # "listener ready"
    for _ in range(20):
        wb.tick()
    rate("listener alive, sensors alive")

    for child in wb.get_actors().filter("sensor.*"):
        if child.parent is not None and child.parent.id == vehicle.id:
            child.destroy()
    vehicle.destroy()
    for _ in range(10):
        wb.tick()
    rate("sensors destroyed, listener alive")

    proc.send_signal(signal.SIGKILL)
    proc.wait()
    time.sleep(2)
    rate("listener process KILLED")
finally:
    wb.apply_settings(original)
