#!/usr/bin/env python3
"""Measure CARLA's pedal-to-acceleration response across speed, for one blueprint.

acb maps Autoware's requested acceleration onto a pedal with a single constant:

    control.throttle = (accel / MAX_ACCEL).clamp(0, 1)     # MAX_ACCEL = 3.0
    control.brake    = (-accel / MAX_ACCEL).clamp(0, 1)    # the same divisor

That assumes the response is linear in the pedal, independent of speed, and identical for the
engine and the brakes. This measures what it actually is.

## Method

Hold one pedal value constant and record (v, dv/dt) every tick while the car ramps through the
speed range on its own. Every sample is then an operating point the vehicle actually passed
through, at a speed that is measured rather than requested.

Two earlier methods were discarded after their output was checked rather than trusted:

- Setting the velocity for each (speed, pedal) point and measuring after a settle window. At a
  requested 25 m/s the car was doing 16-19 by measurement time, and brake response came out
  non-monotonic in the pedal because the car reached standstill inside the window.
- Ramping but teleporting the car back to the start of the straight mid-ramp to keep it on
  road, carrying its speed over. Three brake pedals then produced coast-like numbers -- the
  pedal was not reaching the car across the teleport -- which a single instrumented ramp
  exposed by disagreeing with the map it produced (-6.74 against -5.66 m/s^2 at 14 m/s).

So each ramp now runs once, in one direction, on one stretch of road, with no teleport inside
the measurement. The straight bounds how much of the range a single ramp covers; the car is
reset between ramps, never during one.

Owns the tick deliberately -- it refuses to start if CARLA is already synchronous, because that
means carla_scenario_bridge or another probe owns it.
"""
import json, math, sys, time
import carla

PEDALS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
DT = 0.05
V_TOP = 27.0
# Accelerating away from rest is a real operating point. Braking into a stop is not: the last
# ticks discretise the stop itself and produced apparent decelerations beyond 25 m/s^2.
V_FLOOR_THROTTLE = 0.3
V_FLOOR_BRAKE = 3.0
BLUEPRINT = sys.argv[1] if len(sys.argv) > 1 else "vehicle.tesla.model3"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/longitudinal.json"

# The long straight on Town01: lanelet 6583, y = -129.8 (ROS), x from 325.6 down to 101.4.
X_START, X_END, Y = 322.0, 108.0, 129.8
# Brake ramps accelerate to speed first, and must leave themselves room to stop in.
X_TURNAROUND = 215.0

client = carla.Client("localhost", 2000)
client.set_timeout(30.0)
world = client.get_world()
if world.get_settings().synchronous_mode:
    sys.exit("CARLA is already synchronous: another process owns the tick. Stop it first.")

orig = world.get_settings()
s = world.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = DT
world.apply_settings(s)

bp = world.get_blueprint_library().find(BLUEPRINT)
start, actor = None, None
for offset in range(0, 60, 6):
    t = carla.Transform(carla.Location(x=X_START + offset, y=Y, z=0.5), carla.Rotation(yaw=180.0))
    actor = world.try_spawn_actor(bp, t)
    if actor is not None:
        start = t
        break
if actor is None:
    world.apply_settings(orig)
    sys.exit("every point on the measurement straight is occupied; clear it and retry")


def speed_of():
    v = actor.get_velocity()
    return math.hypot(v.x, v.y)


def apply(throttle, brake, hand_brake=False):
    actor.apply_control(carla.VehicleControl(
        throttle=throttle, brake=brake, steer=0.0, hand_brake=hand_brake))


def reset(entry_speed=0.0):
    """Park at the start of the straight, then set the speed this ramp begins at."""
    actor.set_target_velocity(carla.Vector3D(0, 0, 0))
    actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
    actor.set_transform(start)
    for _ in range(10):
        apply(0.0, 1.0, hand_brake=True)
        world.tick()
    if entry_speed > 0.0:
        # Accelerate up to speed rather than injecting a velocity. Injection leaves a
        # transient that biased the first samples of every brake ramp -- it disagreed with an
        # instrumented ramp by 1.34 m/s^2 at the top of the range, where the naturally
        # accelerated samples agreed to within 0.12.
        for _ in range(600):
            apply(1.0, 0.0)
            world.tick()
            if speed_of() >= entry_speed or actor.get_transform().location.x <= X_TURNAROUND:
                break


def ramp(kind, pedal, rows, from_speed=None):
    """One uninterrupted ramp at a constant pedal, sampling every tick.

    `from_speed` runs the ramp downward from that speed instead of up from rest. Throttle
    needs both directions: a low pedal accelerating from rest never reaches high speed, so
    the only way to learn what it does at 20 m/s is to arrive there and then hold it. Filling
    those cells by extrapolation instead was wrong in the worst way -- it claimed throttle 0.2
    produces +0.01 m/s^2 at 24 m/s, where the truth is heavy deceleration.
    """
    if kind == "throttle":
        reset(0.0 if from_speed is None else from_speed)
        floor = V_FLOOR_THROTTLE
    else:
        # Brake ramps start fast and slow down, so they enter at the top of the range.
        reset(V_TOP)   # capped in practice by X_TURNAROUND
        floor = V_FLOOR_BRAKE

    v_prev = speed_of()
    n = 0
    # A cap, because some ramps have no natural end: throttle 0.0 never moves the car, so
    # neither the speed nor the road-length exit can ever fire.
    for _ in range(1500):
        apply(pedal, 0.0) if kind == "throttle" else apply(0.0, pedal)
        world.tick()
        v = speed_of()
        if actor.get_transform().location.x <= X_END:
            break                      # out of road; the ramp ends rather than teleports
        a = (v - v_prev) / DT
        if v > floor and v_prev > floor:
            rows.append({"kind": kind, "pedal": pedal, "v": round(0.5 * (v + v_prev), 3),
                         "accel": round(a, 4)})
            n += 1
        v_prev = v
        if kind == "throttle" and from_speed is None and v >= V_TOP:
            break
        if kind == "throttle" and from_speed is not None and abs(a) < 0.05 and v < from_speed - 1.0:
            break                      # settled at this pedal's terminal speed
        if kind == "brake" and v <= floor:
            break
    return n


rows = []
t0 = time.time()
try:
    passes = [("throttle", None), ("throttle", V_TOP), ("brake", None)]
    for kind, from_speed in passes:
        for pedal in PEDALS:
            n = ramp(kind, pedal, rows, from_speed)
            got = [r for r in rows if r["kind"] == kind and r["pedal"] == pedal]
            span = (min(r["v"] for r in got), max(r["v"] for r in got)) if got else (0, 0)
            print("%-8s %-4s pedal=%.2f  %4d samples over v = %5.1f .. %5.1f m/s"
                  % (kind, "down" if from_speed else "up", pedal, n, span[0], span[1]),
                  flush=True)
finally:
    actor.destroy()
    world.apply_settings(orig)

with open(OUT, "w") as f:
    json.dump({"blueprint": BLUEPRINT, "dt": DT, "rows": rows}, f)
print("\n%d samples in %.0f s -> %s" % (len(rows), time.time() - t0, OUT))
