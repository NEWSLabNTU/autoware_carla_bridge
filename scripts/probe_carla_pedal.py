"""What pedal actually reaches the ego, and what the ego does about it.

Closes the chain that stall_probe.py opens: if Autoware commands acceleration and the
vehicle does not move, the answer is either the pedal the bridge applied or the physics
underneath it, and neither is visible from ROS.

Sync-mode trap: a client that has observed no tick has no snapshot and reads zero actors,
which looks exactly like an empty world. Always wait for a tick before asking.
"""
import sys, time, carla

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
ROLE = sys.argv[2] if len(sys.argv) > 2 else "hero"

client = carla.Client('localhost', 2000)
client.set_timeout(30.0)
world = client.get_world()

carla_map = world.get_map()
print("   t  actor  speed  throttle  brake   steer  hand gear      x       y  onroad  lane")
start = time.monotonic()
nxt = 0.0
ego = None
while time.monotonic() - start < DURATION:
    try:
        world.wait_for_tick(30.0)
    except RuntimeError:
        continue
    t = time.monotonic() - start
    if t < nxt:
        continue
    nxt += 1.0
    # Re-resolve every sample rather than caching. A handle to a destroyed actor can still
    # report is_alive and then return freed state -- a previous run's ego read back with
    # steer -1.4e21 and gear 6028448, which looks like data and is not.
    ego = None
    for a in world.get_actors():
        if (a.type_id.startswith('vehicle.')
                and a.attributes.get('role_name') == ROLE):
            ego = a
            break
    if ego is None:
        print("%5.0f      -      -         -      -       -     -    -      -       -       -  -" % t)
        sys.stdout.flush()
        continue
    v = ego.get_velocity()
    c = ego.get_control()
    speed = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5
    loc = ego.get_transform().location
    # project_to_road=False returns None when the point is not on a mapped lane at all,
    # which is the difference between "stopped in its lane" and "wedged off the road".
    wp = carla_map.get_waypoint(loc, project_to_road=False)
    onroad = "yes" if wp is not None else "NO"
    lane = str(wp.lane_type).split(".")[-1] if wp is not None else "-"
    print("%5.0f %6d %6.2f  %8.3f %6.3f %7.3f %5s %4d %6.1f %7.1f %7s %s"
          % (t, ego.id, speed, c.throttle, c.brake, c.steer,
             c.hand_brake, c.gear, loc.x, loc.y, onroad, lane))
    sys.stdout.flush()
