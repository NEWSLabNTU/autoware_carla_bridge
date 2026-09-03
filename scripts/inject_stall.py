"""Hold the ego still while Autoware goes on commanding motion.

A controlled version of the fault traced in docs/issues/016: the command path keeps
working and the vehicle does not move. Used to confirm the bridge's stall warning fires
on the real condition rather than only in unit tests. Physics is restored afterwards, and
also on any error, because leaving it off would strand the scenario.
"""
import sys, time, carla

HOLD = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
ROLE = sys.argv[2] if len(sys.argv) > 2 else "hero"

client = carla.Client('localhost', 2000)
client.set_timeout(30.0)
world = client.get_world()


def find_ego():
    """Resolve fresh every time: a handle to a destroyed actor still reports is_alive."""
    for a in world.get_actors():
        if a.type_id.startswith('vehicle.') and a.attributes.get('role_name') == ROLE:
            return a
    return None


# Wait for the ego to be genuinely under way, so the freeze lands mid-drive rather than
# during the standing start the watcher is designed to tolerate.
deadline = time.time() + 240
ego = None
while time.time() < deadline:
    world.wait_for_tick(30.0)
    ego = find_ego()
    if ego is not None:
        v = ego.get_velocity()
        if (v.x * v.x + v.y * v.y) ** 0.5 > 2.0:
            break
    ego = None
if ego is None:
    print("ego never reached 2 m/s; nothing to freeze")
    sys.exit(1)

print("freezing actor %d for %.0fs" % (ego.id, HOLD))
ego.set_simulate_physics(False)
try:
    time.sleep(HOLD)
finally:
    try:
        ego.set_simulate_physics(True)
        print("physics restored")
    except Exception as e:
        print("COULD NOT RESTORE PHYSICS:", e)
