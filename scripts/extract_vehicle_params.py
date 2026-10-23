#!/usr/bin/env python3
"""Measure a CARLA blueprint's geometry for its vehicle spec (sim_vehicle_spec, roadmap 051).

The spec's `geometry` is where Autoware's vehicle_info, the SSv2 catalog's box and axles and
the Isaac rig all come from, so it has to be the car CARLA simulates. This reads the numbers off
a spawned actor and prints them as the spec's `geometry` block, to paste into
sim_vehicle_spec's vehicles/<name>.yaml (then regenerate everything from it).

Sources, and why each:
  wheel_base, wheel_tread   wheel positions in physics_control -- the Ackermann geometry
                            Autoware's steering model uses
  wheel_radius              physics_control wheel radius
  overhangs, height         the actor's bounding box, minus the axle positions, both in the
                            actor's own frame: the axles are not centred on the actor origin
                            (the Model 3's rear axle is 1.386 m behind it, its front 1.618
                            ahead; roadmap 050), so half a wheel base is the wrong offset
  max_steer_angle           see below; this one is deliberately NOT copied by default

Usage:
    extract_vehicle_params.py [vehicle.tesla.model3]   # print the spec's geometry block
    extract_vehicle_params.py --list                   # show available vehicle blueprints

## Why this refuses to run during a scenario

Settling a freshly spawned car needs the simulation to advance. The obvious way is to turn
on synchronous mode and tick, and the first version of this script did exactly that -- on
whatever server it found. That is unsafe here: `carla_world` is the only process
allowed to tick (invariant 1 of docs/design/multi-instance-architecture.md), and a second
ticker corrupts a running scenario. Restoring the settings afterwards makes it worse, since
it would stamp this script's idea of the world onto a run already in progress.

So: if the server is already synchronous, somebody owns the tick and this script stops. In
asynchronous mode the server free-runs and the car settles on its own, so it never touches
world settings at all.

## Why max_steer_angle is reported but not in the block

CARLA drives the inner front wheel to `command * max_steer_angle` and places the outer one
by Ackermann, so the effective bicycle-model angle at full lock is the mean of the two, not
the per-wheel limit (docs/issues/006). For the Tesla that is 1.025 rad, where the workspace
ships 0.70 rad.

That 0.70 is a *planning* limit chosen inside the vehicle's physical capability, not a bad
measurement. Writing the physical value would let the planner ask for tighter turns than it
does today, which is a behaviour change that wants its own testing. The block therefore
leaves max_steer_angle to the spec and prints the physical value for comparison.
"""

import argparse
import math
import sys
import time

DEFAULT_BLUEPRINT = "vehicle.tesla.model3"
# Not exposed by physics_control, so it survives from whatever the file already said.
DEFAULT_WHEEL_WIDTH = 0.235


def measure(world, client, blueprint_id, settle_s, carla):
    """Spawn the blueprint, let it settle, and return its geometry."""
    library = world.get_blueprint_library()
    try:
        bp = library.find(blueprint_id)
    except (IndexError, RuntimeError):
        names = sorted(b.id for b in library.filter("vehicle.*"))
        raise SystemExit(
            "no blueprint '%s'. Available:\n  %s" % (blueprint_id, "\n  ".join(names))
        )

    actor = None
    for sp in world.get_map().get_spawn_points():
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            break
    if actor is None:
        raise SystemExit("every spawn point on this map is occupied")

    try:
        # The server is asynchronous (checked by the caller), so it advances on its own and
        # the car drops onto its suspension without anyone ticking it.
        time.sleep(settle_s)

        pc = actor.get_physics_control()
        steered = [w for w in pc.wheels if w.max_steer_angle > 0.0]
        fixed = [w for w in pc.wheels if w.max_steer_angle <= 0.0]
        if len(steered) != 2 or len(fixed) != 2:
            raise SystemExit(
                "expected 2 steered and 2 fixed wheels, got %d and %d -- this script's "
                "Ackermann assumptions do not hold for '%s'"
                % (len(steered), len(fixed), blueprint_id)
            )

        # Wheel positions are world centimetres at the actor's current pose. Distances
        # between them are pose-independent, which is why no transform is needed.
        def mid(a, b):
            return (
                (a.position.x + b.position.x) / 2.0,
                (a.position.y + b.position.y) / 2.0,
            )

        front = mid(steered[0], steered[1])
        rear = mid(fixed[0], fixed[1])
        wheel_base = math.dist(front, rear) / 100.0
        wheel_tread = (
            math.dist(
                (steered[0].position.x, steered[0].position.y),
                (steered[1].position.x, steered[1].position.y),
            )
            / 100.0
        )
        wheel_radius = steered[0].radius / 100.0

        bb = actor.bounding_box  # extent is half-size, in metres
        length, width, height = bb.extent.x * 2, bb.extent.y * 2, bb.extent.z * 2

        # The axles' x in the actor's own frame (positions are world centimetres once the car
        # has been simulated, which it has after settling).
        tf = actor.get_transform()

        def local_x(world_xy):
            return tf.inverse_transform(carla.Location(world_xy[0] / 100.0, world_xy[1] / 100.0,
                                                       tf.location.z)).x

        front_x, rear_x = local_x(front), local_x(rear)
        front_overhang = (bb.location.x + bb.extent.x) - front_x
        rear_overhang = rear_x - (bb.location.x - bb.extent.x)
        side_overhang = (width - wheel_tread) / 2.0

        max_wheel = math.radians(max(w.max_steer_angle for w in steered))
        outer = math.atan(1.0 / (1.0 / math.tan(max_wheel) + wheel_tread / wheel_base))
        max_steer_effective = 0.5 * (max_wheel + outer)

        return {
            "blueprint": blueprint_id,
            "server": client.get_server_version(),
            "wheel_radius": wheel_radius,
            "wheel_base": wheel_base,
            "wheel_tread": wheel_tread,
            "front_overhang": front_overhang,
            "rear_overhang": rear_overhang,
            "left_overhang": side_overhang,
            "right_overhang": side_overhang,
            "vehicle_height": height,
            "max_steer_physical": max_steer_effective,
            "max_steer_per_wheel": max_wheel,
            "length": length,
            "width": width,
            "front_x": front_x,
            "rear_x": rear_x,
        }
    finally:
        actor.destroy()


def report(m):
    print("# %s on CARLA %s" % (m["blueprint"], m["server"]))
    print("#   bounding box      %.3f x %.3f x %.3f m" % (m["length"], m["width"], m["vehicle_height"]))
    print("#   wheel base        %.3f m (axles at %+.3f and %+.3f m from the actor origin)"
          % (m["wheel_base"], m["front_x"], m["rear_x"]))
    print("#   wheel tread       %.3f m" % m["wheel_tread"])
    print("#   wheel radius      %.3f m" % m["wheel_radius"])
    print(
        "#   steering          %.1f deg per wheel, %.1f deg effective (%.3f rad)"
        % (
            math.degrees(m["max_steer_per_wheel"]),
            math.degrees(m["max_steer_physical"]),
            m["max_steer_physical"],
        )
    )


def geometry_block(m):
    """The spec's `geometry` (sim_vehicle_spec docs/spec-format.md), max_steer_angle aside."""
    rows = ["wheel_base", "wheel_tread", "wheel_radius", "front_overhang", "rear_overhang",
            "left_overhang", "right_overhang", "vehicle_height"]
    return "geometry:\n" + "".join("  %s: %.3f\n" % (k, m[k]) for k in rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blueprint", nargs="?", default=DEFAULT_BLUEPRINT)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--settle", type=float, default=1.5, help="seconds to let the car settle")
    ap.add_argument("--list", action="store_true", help="list vehicle blueprints and exit")
    args = ap.parse_args()

    import carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()

    if args.list:
        for b in sorted(b.id for b in world.get_blueprint_library().filter("vehicle.*")):
            print(b)
        return 0

    if world.get_settings().synchronous_mode:
        print(
            "CARLA is in synchronous mode, so another process owns the tick -- almost\n"
            "certainly carla_world, mid-scenario. Spawning and settling a car\n"
            "here would interfere with that run. Stop the bridge, or wait for the run to\n"
            "finish, and try again.",
            file=sys.stderr,
        )
        return 1

    m = measure(world, client, args.blueprint, args.settle, carla)
    report(m)

    print()
    print("# For sim_vehicle_spec's vehicles/<name>.yaml (wheel_width and max_steer_angle are the")
    print("# spec's own); then regenerate in every repository that holds an output:")
    print(geometry_block(m), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
