#!/usr/bin/env python3
"""The scene `just vehicle-check` drives: a `hero` on a straight lane, CARLA ticked in real time.

vehicle_check (autoware_sim_vehicle's `autoware_sim_check`) steps the car's acceleration and
brake and sweeps its steering through the vehicle interface acb_bridge serves; it needs a car
with road ahead and a ticking world, nothing else -- no Autoware, no scenario. This spawns the
car at the first spawn point with 160 m of lane ahead turning less than 2 degrees, ticks CARLA
synchronously at the world server's timing (0.05 s, 16 substeps) in real time, and on SIGTERM
destroys the car's sensors, then the car, and puts the world's settings back.

    scripts/vehicle_check_host.py [--port 2000] [--blueprint vehicle.tesla.model3]
"""
import argparse
import signal
import time

import carla

DT = 0.05
STRAIGHT_M = 160.0


def straight_spawn(world):
    """A spawn point with STRAIGHT_M of lane ahead turning less than 2 degrees."""
    m = world.get_map()
    for sp in m.get_spawn_points():
        wp = m.get_waypoint(sp.location)
        yaw0, ok, run = wp.transform.rotation.yaw, True, 0.0
        while run < STRAIGHT_M:
            nxt = wp.next(5.0)
            if len(nxt) != 1:
                ok = False
                break
            wp = nxt[0]
            run += 5.0
            if abs((wp.transform.rotation.yaw - yaw0 + 180) % 360 - 180) > 2.0:
                ok = False
                break
        if ok:
            return sp
    raise SystemExit("no spawn point with a straight lane ahead on this map")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--blueprint", default="vehicle.tesla.model3")
    args = ap.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(30)
    world = client.get_world()
    old = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, DT
    s.substepping, s.max_substep_delta_time, s.max_substeps = True, DT / 16, 16
    world.apply_settings(s)
    bp = world.get_blueprint_library().find(args.blueprint)
    bp.set_attribute("role_name", "hero")
    at = straight_spawn(world)
    at.location.z += 0.3
    car = world.spawn_actor(bp, at)
    print(f"hero {car.id} ({args.blueprint}) at {at.location}", flush=True)
    stop = []
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    signal.signal(signal.SIGINT, lambda *a: stop.append(1))
    try:
        nxt = time.monotonic()
        while not stop:
            world.tick()
            nxt += DT
            time.sleep(max(0.0, nxt - time.monotonic()))
    finally:
        # Children first: a destroyed vehicle's sensors keep ticking, and an orphaned IMU
        # crashes the server.
        for a in world.get_actors():
            if a.parent is not None and a.parent.id == car.id:
                a.destroy()
        car.destroy()
        world.apply_settings(old)
        print("hero gone, settings restored", flush=True)


if __name__ == "__main__":
    main()
