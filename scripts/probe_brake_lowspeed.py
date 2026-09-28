#!/usr/bin/env python3
"""Measure the brake map below 5 m/s, where a braking ramp cannot.

Companion to probe_longitudinal.py. That probe holds a brake pedal from the top of the speed
range down to a stop and samples (v, dv/dt) every tick. Above about 5 m/s that is a good
measurement. Below it the ramp is running into its own stop: the last ticks discretise the
stop itself (a car at 0.4 m/s that is at 0 one tick later reads as -8 m/s^2 whatever the
brake is doing), so build_longitudinal_maps.py used to discard every brake sample below
5 m/s and copy the 6 m/s column into 0, 2 and 4. The shipped map then claimed -2.55 m/s^2
at pedal 0 for 0-4 m/s, where the throttle probe measured the same no-pedal row at -0.28,
-0.55 and -1.51. The copy was wrong by up to 2.3 m/s^2, and in the direction that sent a
low-speed stop to neither pedal.

## Method

One short braking run per (pedal, target speed, repeat), each aimed at one speed rather than
sweeping through all of them:

1. Park at the start of the Town01 straight with the handbrake on, then drive up on full
   throttle (not a velocity injection -- see probe_longitudinal.py) to `target + ENTRY_MARGIN`.
2. Apply the brake pedal alone: throttle 0, handbrake off, automatic gearbox, forward gear,
   exactly the fields acb sends. Record sim time and *signed* forward speed every tick until
   the car stops, falls out of the window, or runs out of road.
3. Over the ticks with |v - target| <= WINDOW -- excluding the first SETTLE_TICKS after the
   brake goes on, every tick below V_STOP, and the PRE_STOP_TICKS ticks before the first
   one -- fit dv/dt two ways: `accel`, the slope of v(t), and `accel_at_target`, a line
   through the per-tick (v, dv/dt) pairs evaluated at the target speed. The map uses the
   second (see "Reading a cell at its speed"). A line through ~6-40
   ticks is not sensitive to the last tick's quantisation the way a two-point difference is,
   and the excluded ticks are exactly the ones that carried the artefact.
4. Repeat REPEATS times per cell. The result carries every fit, its residual and its sample
   count, and the raw trace, so a strange cell can be looked at rather than argued about.

## Reading a cell at its speed

The slope of v(t) over the window is the average deceleration over it, and that is not the
deceleration at the target: the response varies with speed, and the car spends more ticks at
the slow end of the window than the fast one, so the average leans low. At pedal 0 the window
mean sat at 1.82 m/s for a 2 m/s target and the slope read -0.75 where the value at 2 m/s is
-0.84. Regressing dv/dt on v and reading the line at the target removes both. Checked against
an independent measurement of the same quantity -- the pedal-0 throttle ramp in
probe_longitudinal.py, read at +-0.15 m/s of each speed -- it agrees to 0.015 m/s^2 at 1, 2, 3
and 4 m/s.

## The onset transient

The brake does not take hold at once, and a first version of this probe (1.5 m/s margin, no
settle exclusion) put its window straight into the transient. Measured at 7 m/s, pedal 1.0,
per tick after the brake goes on:

    straight off full throttle   +5.97 +3.15 -0.65 -3.32 -4.49 -5.11 -5.43 -5.57 -5.61
    after 0.5 s of coasting      -3.23 -4.11 -4.97 -5.65 -5.65 -5.58 -5.49

Two lags, stacked: releasing the throttle still drives the car for ~3 ticks, and the brake
itself then builds over ~4 ticks even from a coast. Neither is the steady response a map cell
stands for (it is dynamics, which Autoware's controller handles as actuation delay), so the
first SETTLE_TICKS = 8 (0.4 s) are dropped, and the entry margin is 2.5 m/s so that the window
still starts after them at pedal 1.0.

Speed is signed along the car's heading. acb and probe_longitudinal.py use |v|, which cannot
tell a stop from a roll back; here a negative speed is visible in the trace. The gear CARLA
reports is recorded with each tick for the same reason.

At pedal 0 the car does not stop at all: CARLA's automatic transmission idle-creeps at zero
throttle and brake (the reason acb holds a commanded standstill with the handbrake), so the
no-pedal row at 1 m/s is a fit across the approach to that creep equilibrium, not braking.

## Timing

Ticks are fixed at DT = 0.05 s of sim time, with physics substeps of 0.05/16 = 3.125 ms --
the timing carla_scenario_bridge applies during a scenario (coordinator.rs `sync_timing`), so
the car measured here is the car the ego drives. `--carla-default-substeps` uses CARLA's
default 10 ms substeps instead, which is what probe_longitudinal.py runs under, to check
that the two agree where they overlap.

## Ownership

Owns the tick and refuses to start if CARLA is already synchronous (that means
carla_scenario_bridge, or another probe, owns it). Spawns its car with role_name
`acb_probe`, never `hero`: a running ego stack's acb_bridge adopts any vehicle named hero.
Destroys only that car, and hands the world back asynchronous.

    probe_brake_lowspeed.py [OUT.json] [--blueprint vehicle.tesla.model3]
                            [--repeats 3] [--pedals 0,0.5,1] [--targets 1,2,3]
                            [--carla-default-substeps]
"""
import argparse, json, math, sys, time
import carla

PEDALS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
TARGETS = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
REPEATS = 3
DT = 0.05
BRIDGE_SUBSTEP = DT / 16          # coordinator.rs: max_substeps clamped to 16
ENTRY_MARGIN = 2.5                # m/s above the target where the brake goes on
SETTLE_TICKS = 8                  # 0.4 s for throttle release and brake build-up
WINDOW = 1.0                      # fit ticks within this many m/s of the target
V_STOP = 0.3                      # below this a tick is the stop, not braking
PRE_STOP_TICKS = 2                # and so are the ticks just before it
MIN_FIT = 4
LOCAL = 0.3                       # the narrow fit, for rows whose response varies with v
AFTER_STOP_TICKS = 20             # keep watching 1 s past the stop
ROLE_NAME = "acb_probe"

# The long straight on Town01 (see probe_longitudinal.py).
X_START, X_END, Y = 322.0, 108.0, 129.8

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("out", nargs="?", default="/tmp/brake_lowspeed.json")
ap.add_argument("--blueprint", default="vehicle.tesla.model3")
ap.add_argument("--repeats", type=int, default=REPEATS)
ap.add_argument("--pedals", default=",".join(str(p) for p in PEDALS))
ap.add_argument("--targets", default=",".join(str(t) for t in TARGETS))
ap.add_argument("--carla-default-substeps", action="store_true")
ap.add_argument("--host", default="localhost")
ap.add_argument("--port", type=int, default=2000)
args = ap.parse_args()
pedals = [float(p) for p in args.pedals.split(",")]
targets = [float(t) for t in args.targets.split(",")]

client = carla.Client(args.host, args.port)
client.set_timeout(30.0)
world = client.get_world()
if world.get_settings().synchronous_mode:
    sys.exit("CARLA is already synchronous: another process owns the tick. Stop it first.")

orig = world.get_settings()
s = world.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = DT
s.substepping = True
if args.carla_default_substeps:
    s.max_substep_delta_time, s.max_substeps = 0.01, 10
else:
    s.max_substep_delta_time, s.max_substeps = BRIDGE_SUBSTEP, 16
world.apply_settings(s)
timing = {"dt": DT, "max_substep_delta_time": s.max_substep_delta_time,
          "max_substeps": s.max_substeps}

bp = world.get_blueprint_library().find(args.blueprint)
bp.set_attribute("role_name", ROLE_NAME)
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
assert actor.attributes.get("role_name") == ROLE_NAME, actor.attributes


def tick():
    world.tick()
    return world.get_snapshot().timestamp.elapsed_seconds


def forward_speed():
    v, f = actor.get_velocity(), actor.get_transform().get_forward_vector()
    return v.x * f.x + v.y * f.y + v.z * f.z


def apply(throttle, brake, hand_brake=False):
    actor.apply_control(carla.VehicleControl(
        throttle=throttle, brake=brake, steer=0.0, hand_brake=hand_brake,
        reverse=False, manual_gear_shift=False))


def line(pts):
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    mv = sum(v for _, v in pts) / n
    stt = sum((t - mt) ** 2 for t, _ in pts)
    slope = sum((t - mt) * (v - mv) for t, v in pts) / stt
    resid = math.sqrt(sum((v - mv - slope * (t - mt)) ** 2 for t, v in pts) / n)
    return slope, mv, resid


def fit(trace, target):
    """Least-squares dv/dt over the window, and the bookkeeping to judge it by.

    `accel` is the fit over the whole +-WINDOW. `accel_local` repeats it over +-LOCAL only:
    where the response is constant in v (any real brake) the two agree, and where it is not
    (pedal 0 approaching the creep speed) the local one is the value *at* the target.
    """
    stop = next((i for i, (_, v, _) in enumerate(trace) if v < V_STOP), len(trace))
    usable = trace[:max(0, stop - PRE_STOP_TICKS)] if stop < len(trace) else trace
    usable = usable[SETTLE_TICKS:]
    pts = [(t, v) for t, v, _ in usable if abs(v - target) <= WINDOW and v >= V_STOP]
    if len(pts) < MIN_FIT:
        return None
    slope, mv, resid = line(pts)
    local = [(t, v) for t, v in pts if abs(v - target) <= LOCAL]
    # dv/dt against v, per tick, over the same ticks; read at the target speed. Consecutive
    # ticks only, so an excluded tick never contributes a difference.
    keep = {t for t, _ in pts}
    av = [(0.5 * (v0 + v1), (v1 - v0) / (t1 - t0))
          for (t0, v0, _), (t1, v1, _) in zip(trace, trace[1:]) if t0 in keep and t1 in keep]
    at_target = None
    if len(av) >= MIN_FIT - 1:
        b, _, _ = line(av)                       # d(accel)/dv
        ma = sum(a for _, a in av) / len(av)
        mvv = sum(v for v, _ in av) / len(av)
        at_target = ma + b * (target - mvv)
    after = trace[stop:]
    return {"accel": round(slope, 4), "v_mean": round(mv, 3), "n": len(pts),
            "accel_at_target": None if at_target is None else round(at_target, 4),
            "resid": round(resid, 5),
            "accel_local": round(line(local)[0], 4) if len(local) >= MIN_FIT else None,
            "n_local": len(local),
            "stopped": stop < len(trace),
            # What the car did once stopped with the brake still held: lowest signed speed
            # (a roll-back would be negative) and the gears CARLA reported.
            "after_stop_min_v": min((v for _, v, _ in after), default=None),
            "gears": sorted({g for _, _, g in trace})}


def run(pedal, target):
    actor.set_target_velocity(carla.Vector3D(0, 0, 0))
    actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
    actor.set_transform(start)
    for _ in range(10):
        apply(0.0, 1.0, hand_brake=True)
        tick()
    entry = target + ENTRY_MARGIN
    for _ in range(600):
        apply(1.0, 0.0)
        tick()
        if forward_speed() >= entry:
            break
    trace = []
    stopped_at = None
    # 40 s cap: pedal 0 at low speed approaches the creep speed and may never stop.
    for i in range(800):
        apply(0.0, pedal)
        t = tick()
        v = forward_speed()
        trace.append((round(t, 4), round(v, 4), actor.get_control().gear))
        if actor.get_transform().location.x <= X_END:
            break
        if stopped_at is None and v < V_STOP:
            stopped_at = i
        if stopped_at is not None and i - stopped_at >= AFTER_STOP_TICKS:
            break          # watched the standstill long enough to see a creep or roll-back
        if stopped_at is None and v < target - WINDOW - 0.3:
            break          # well below the window; nothing further is used
        if i >= 40 and abs(v - trace[i - 40][1]) < 0.02:
            break          # settled at a creep equilibrium; it will not move again
    return trace


runs = []
t0 = time.time()
try:
    for target in targets:
        for pedal in pedals:
            fits = []
            for rep in range(args.repeats):
                trace = run(pedal, target)
                f = fit(trace, target)
                runs.append({"pedal": pedal, "target": target, "rep": rep, "fit": f,
                             "trace": trace})
                fits.append(f)
            got = [f["accel_at_target"] for f in fits if f and f["accel_at_target"] is not None]
            print("target %.1f  pedal %.2f  accel %s  n %s" % (
                target, pedal, " ".join("%+.3f" % a for a in got),
                " ".join(str(f["n"]) if f else "-" for f in fits)), flush=True)
finally:
    actor.destroy()
    orig.synchronous_mode = False
    world.apply_settings(orig)

with open(args.out, "w") as f:
    json.dump({"blueprint": args.blueprint, "timing": timing, "entry_margin": ENTRY_MARGIN,
               "settle_ticks": SETTLE_TICKS,
               "window": WINDOW, "v_stop": V_STOP, "pre_stop_ticks": PRE_STOP_TICKS,
               "runs": runs}, f)
print("\n%d runs in %.0f s -> %s" % (len(runs), time.time() - t0, args.out))
