#!/usr/bin/env python3
"""Extract Autoware's vehicle_info parameters from a CARLA blueprint.

`vehicle_info.param.yaml` is loaded once at startup and cached by some thirty Autoware
nodes -- control, planning and perception all size the vehicle from it. The values shipped
in this workspace are, by their own comment, "defaults from awsim_labs_vehicle (Lexus-like
vehicle)", so they describe a different car from the one CARLA is simulating. This reads the
real numbers off a spawned actor.

Sources, and why each:
  wheel_base, wheel_tread   wheel positions in physics_control -- the Ackermann geometry
                            Autoware's steering model uses
  wheel_radius              physics_control wheel radius
  overhangs, height         the actor's bounding box, minus the axle positions
  max_steer_angle           the *effective* bicycle-model angle at full lock, which is the
                            Ackermann mean of the two front wheels and not the per-wheel
                            limit; see acb docs/issues/006

Usage: extract_vehicle_params.py [blueprint]   (default vehicle.tesla.model3)
"""
import math
import sys

import carla

BLUEPRINT = sys.argv[1] if len(sys.argv) > 1 else "vehicle.tesla.model3"

client = carla.Client("localhost", 2000)
client.set_timeout(30.0)
world = client.get_world()
orig = world.get_settings()
s = world.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = 0.05
world.apply_settings(s)

bp = world.get_blueprint_library().find(BLUEPRINT)
actor = None
for sp in world.get_map().get_spawn_points():
    actor = world.try_spawn_actor(bp, sp)
    if actor is not None:
        break
if actor is None:
    raise SystemExit("no free spawn point")

try:
    for _ in range(20):
        world.tick()
    pc = actor.get_physics_control()
    wheels = pc.wheels
    steered = [w for w in wheels if w.max_steer_angle > 0.0]
    fixed = [w for w in wheels if w.max_steer_angle <= 0.0]
    if len(steered) != 2 or len(fixed) != 2:
        raise SystemExit(f"expected 2 steered and 2 fixed wheels, got {len(steered)}/{len(fixed)}")

    # Wheel positions are world centimetres at the current pose.
    def mid(a, b):
        return ((a.position.x + b.position.x) / 2.0, (a.position.y + b.position.y) / 2.0,
                (a.position.z + b.position.z) / 2.0)

    front = mid(steered[0], steered[1])
    rear = mid(fixed[0], fixed[1])
    wheel_base = math.dist(front[:2], rear[:2]) / 100.0
    wheel_tread = math.dist(
        (steered[0].position.x, steered[0].position.y),
        (steered[1].position.x, steered[1].position.y)) / 100.0
    wheel_radius = steered[0].radius / 100.0

    bb = actor.bounding_box               # extent is half-size, in metres
    length, width, height = bb.extent.x * 2, bb.extent.y * 2, bb.extent.z * 2

    # Overhangs: the bounding box reaches beyond each axle.
    front_overhang = length / 2.0 + bb.location.x - wheel_base / 2.0
    rear_overhang = length / 2.0 - bb.location.x - wheel_base / 2.0
    side_overhang = (width - wheel_tread) / 2.0

    # Effective steering: CARLA drives the inner wheel to cmd*max and places the outer by
    # Ackermann, so the bicycle-model angle is the mean of the two, not the per-wheel limit.
    max_wheel = math.radians(max(w.max_steer_angle for w in steered))
    t_over_l = wheel_tread / wheel_base
    outer = math.atan(1.0 / (1.0 / math.tan(max_wheel) + t_over_l))
    max_steer_effective = 0.5 * (max_wheel + outer)

    print(f"# Extracted from {BLUEPRINT} on CARLA {client.get_server_version()}")
    print("/**:")
    print("  ros__parameters:")
    print(f"    wheel_radius: {wheel_radius:.3f}")
    print(f"    wheel_width: {0.235:.3f}      # not exposed by physics_control; unchanged")
    print(f"    wheel_base: {wheel_base:.3f}")
    print(f"    wheel_tread: {wheel_tread:.3f}")
    print(f"    front_overhang: {front_overhang:.3f}")
    print(f"    rear_overhang: {rear_overhang:.3f}")
    print(f"    left_overhang: {side_overhang:.3f}")
    print(f"    right_overhang: {side_overhang:.3f}")
    print(f"    vehicle_height: {height:.3f}")
    print(f"    max_steer_angle: {max_steer_effective:.3f}")
    print()
    print(f"# per-wheel limit {math.degrees(max_wheel):.1f} deg; "
          f"effective bicycle angle at full lock {math.degrees(max_steer_effective):.1f} deg")
    print(f"# bounding box: {length:.3f} x {width:.3f} x {height:.3f} m")
finally:
    actor.destroy()
    world.apply_settings(orig)
