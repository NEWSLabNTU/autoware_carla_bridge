#!/usr/bin/env python3
"""Extract Autoware's vehicle_info parameters from a CARLA blueprint.

`vehicle_info.param.yaml` is loaded once at startup and cached by some thirty Autoware
nodes -- control, planning and perception all size the vehicle from it. The values this
workspace shipped were, by their own comment, "defaults from awsim_labs_vehicle (Lexus-like
vehicle)", so Autoware planned and collision-checked for a car it was not driving. This
reads the real numbers off a spawned actor.

Sources, and why each:
  wheel_base, wheel_tread   wheel positions in physics_control -- the Ackermann geometry
                            Autoware's steering model uses
  wheel_radius              physics_control wheel radius
  overhangs, height         the actor's bounding box, minus the axle positions
  max_steer_angle           see below; this one is deliberately NOT copied by default

Usage:
    extract_vehicle_params.py [--blueprint vehicle.tesla.model3]
    extract_vehicle_params.py --write            # update the workspace's param file
    extract_vehicle_params.py --list             # show available vehicle blueprints

## Why this refuses to run during a scenario

Settling a freshly spawned car needs the simulation to advance. The obvious way is to turn
on synchronous mode and tick, and the first version of this script did exactly that -- on
whatever server it found. That is unsafe here: `carla_scenario_bridge` is the only process
allowed to tick (invariant 1 of docs/design/multi-instance-architecture.md), and a second
ticker corrupts a running scenario. Restoring the settings afterwards makes it worse, since
it would stamp this script's idea of the world onto a run already in progress.

So: if the server is already synchronous, somebody owns the tick and this script stops. In
asynchronous mode the server free-runs and the car settles on its own, so it never touches
world settings at all.

## Why max_steer_angle is reported but not written

CARLA drives the inner front wheel to `command * max_steer_angle` and places the outer one
by Ackermann, so the effective bicycle-model angle at full lock is the mean of the two, not
the per-wheel limit (docs/issues/006). For the Tesla that is 1.025 rad, where the workspace
ships 0.70 rad.

That 0.70 is a *planning* limit chosen inside the vehicle's physical capability, not a bad
measurement. Writing the physical value would let the planner ask for tighter turns than it
does today, which is a behaviour change that wants its own testing. `--write` therefore
keeps whatever the file already has and prints the physical value for comparison; pass
`--physical-steer-angle` to write the measured one.
"""

import argparse
import math
import re
import sys
import time
from pathlib import Path

DEFAULT_BLUEPRINT = "vehicle.tesla.model3"
# Not exposed by physics_control, so it survives from whatever the file already said.
DEFAULT_WHEEL_WIDTH = 0.235
PARAM_FILE = (
    Path(__file__).resolve().parents[1]
    / "src/acb_vehicle_launch/acb_vehicle_description/config/vehicle_info.param.yaml"
)


def measure(world, client, blueprint_id, settle_s):
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

        front_overhang = length / 2.0 + bb.location.x - wheel_base / 2.0
        rear_overhang = length / 2.0 - bb.location.x - wheel_base / 2.0
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
        }
    finally:
        actor.destroy()


def report(m):
    print("# %s on CARLA %s" % (m["blueprint"], m["server"]))
    print("#   bounding box      %.3f x %.3f x %.3f m" % (m["length"], m["width"], m["vehicle_height"]))
    print("#   wheel base        %.3f m" % m["wheel_base"])
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


def rewrite(text, m, write_steer):
    """Replace the parameter values in place, leaving every comment as it is."""
    fields = [
        ("wheel_radius", m["wheel_radius"]),
        ("wheel_base", m["wheel_base"]),
        ("wheel_tread", m["wheel_tread"]),
        ("front_overhang", m["front_overhang"]),
        ("rear_overhang", m["rear_overhang"]),
        ("left_overhang", m["left_overhang"]),
        ("right_overhang", m["right_overhang"]),
        ("vehicle_height", m["vehicle_height"]),
    ]
    if write_steer:
        fields.append(("max_steer_angle", m["max_steer_physical"]))

    # The file names the blueprint it was measured from, in a comment. Leaving that stale
    # while rewriting the numbers underneath it would be worse than not writing at all.
    text, n = re.subn(
        r"(Measured from `)vehicle\.[\w.\-]+(`)",
        lambda mo: "%s%s%s" % (mo.group(1), m["blueprint"], mo.group(2)),
        text,
        count=1,
    )
    if n == 0:
        print(
            "# note: no 'Measured from `vehicle.*`' comment to update in the param file",
            file=sys.stderr,
        )

    missing = []
    for key, value in fields:
        pattern = re.compile(r"^(\s*%s:\s*)(-?[\d.]+)" % re.escape(key), re.M)
        text, n = pattern.subn(lambda mo: "%s%.3f" % (mo.group(1), value), text, count=1)
        if n == 0:
            missing.append(key)
    if missing:
        raise SystemExit(
            "these keys are not in the param file, so it was left alone: %s"
            % ", ".join(missing)
        )
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blueprint", nargs="?", default=DEFAULT_BLUEPRINT)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--settle", type=float, default=1.5, help="seconds to let the car settle")
    ap.add_argument("--list", action="store_true", help="list vehicle blueprints and exit")
    ap.add_argument("--write", action="store_true", help="update the workspace's param file")
    ap.add_argument(
        "--physical-steer-angle",
        action="store_true",
        help="also write max_steer_angle; see the note in this script's docstring",
    )
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
            "certainly carla_scenario_bridge, mid-scenario. Spawning and settling a car\n"
            "here would interfere with that run. Stop the bridge, or wait for the run to\n"
            "finish, and try again.",
            file=sys.stderr,
        )
        return 1

    m = measure(world, client, args.blueprint, args.settle)
    report(m)

    if not args.write:
        print()
        print("# Re-run with --write to update")
        print("#   %s" % PARAM_FILE)
        return 0

    if not PARAM_FILE.exists():
        raise SystemExit("no param file at %s" % PARAM_FILE)
    original = PARAM_FILE.read_text()
    updated = rewrite(original, m, args.physical_steer_angle)
    if updated == original:
        print("\n# %s already matches this blueprint" % PARAM_FILE.name)
        return 0
    PARAM_FILE.write_text(updated)
    print("\n# wrote %s" % PARAM_FILE)
    if not args.physical_steer_angle:
        print(
            "# max_steer_angle left as it was: the shipped value is a planning limit, not a\n"
            "# measurement. Pass --physical-steer-angle to write %.3f rad instead."
            % m["max_steer_physical"]
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
