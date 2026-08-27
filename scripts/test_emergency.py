#!/usr/bin/env python3
"""Declare an emergency mid-drive and watch what the vehicle does, from CARLA."""
import math, os, threading, time
import carla, rclpy
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped

c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world()
ego = None
for _ in range(40):
    w.wait_for_tick(seconds=5.0)
    ego = next((a for a in w.get_actors().filter("vehicle.*")
                if a.attributes.get("role_name") == "hero"), None)
    if ego and math.hypot(ego.get_velocity().x, ego.get_velocity().y) > 2.0:
        break
if ego is None:
    raise SystemExit("no ego")

rclpy.init()
n = rclpy.create_node("emergency_test_%d" % os.getpid())
pub = n.create_publisher(VehicleEmergencyStamped, "/control/command/emergency_cmd", 10)

def speed():
    v = ego.get_velocity(); return math.hypot(v.x, v.y)

print("before: speed %.2f m/s" % speed())
stop = threading.Event()
def spam():
    # The gate publishes false on this topic; publish true faster so the declared state is
    # what the bridge mostly sees.
    m = VehicleEmergencyStamped(); m.emergency = True
    while not stop.is_set():
        pub.publish(m); time.sleep(0.01)
t = threading.Thread(target=spam, daemon=True); t.start()

t0 = time.time()
while time.time() - t0 < 8.0:
    w.wait_for_tick(seconds=5.0)
    if int((time.time()-t0)*2) % 2 == 0:
        ctl = ego.get_control()
        print("  t=%.1fs speed %5.2f  throttle %.2f brake %.2f handbrake %s"
              % (time.time()-t0, speed(), ctl.throttle, ctl.brake, ctl.hand_brake))
        time.sleep(0.5)
stop.set()
print("after 8 s of emergency: speed %.2f m/s" % speed())
rclpy.shutdown()
