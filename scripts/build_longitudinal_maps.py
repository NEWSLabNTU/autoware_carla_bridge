#!/usr/bin/env python3
"""Turn probe samples into accel/brake maps in Autoware's convention.

    build_longitudinal_maps.py RAMPS.json OUT_DIR LOWSPEED.json

RAMPS.json comes from probe_longitudinal.py (constant-pedal ramps, every tick a sample) and
LOWSPEED.json from probe_brake_lowspeed.py (short braking runs aimed at one speed each).

## Which data fills which cell

- Accel map: every cell from the throttle ramps.
- Brake map, columns up to LOWSPEED_MAX (6 m/s): from the low-speed probe. A brake ramp
  cannot measure there -- it is running into its own stop -- which is why this script used to
  discard every brake sample below 5 m/s (BRAKE_V_FLOOR) and copy the 6 m/s column into 0, 2
  and 4. That copy claimed -2.55 m/s^2 at pedal 0 for 0-4 m/s, where the car coasts at -0.4 to
  -1.7; the low-speed probe measures those cells directly.
- Brake map, columns above LOWSPEED_MAX: from the brake ramps, whose samples below that speed
  are not used.
- The 0 m/s column of both maps holds the 1 m/s value (ZERO_FROM). Deceleration at standstill
  is not a measurable quantity: a braked car is at rest, and the brake only holds it. Below
  ~0.5 m/s the car goes from moving to rest within one 0.05 s tick under any brake, so there
  is nothing between 1 m/s and rest for a controller to track. Holding the lowest measured
  value is the same rule accel_at() applies outside a table, and it keeps the brake's own
  authority -- which does not fade towards rest, only drag and engine braking do -- rather
  than extrapolating those to zero. Neither probe samples below 0.3 m/s, so the accel map's
  0 m/s cells used to be medians over 0.3-1.5 m/s: the same stand-in, estimated differently,
  which left the two pedal-0 rows 0.14 m/s^2 apart there. One rule for both keeps the rows
  identical, so LongitudinalCalibration::command_for's boundary is a plain lookup.

fill() remains for cells that no data covers, and says which it filled. With both probes run
on the full grid it fills nothing.

## Reading a cell at its speed

A cell is the response *at* its column speed. The median of every sample within +-HALF is
not that: the response varies with speed, and a decelerating ramp spends more ticks at the
slow end of the band, so the median leans towards the slower speed. At pedal 0 it put
-0.555 m/s^2 in the 2 m/s cell where the ramp's own samples at 2.0 +- 0.15 read -0.837. So
where the band has samples on both sides of the column, the cell is a least-squares line
through (v, accel) read at the column; otherwise (the table's edges, or a pedal that stops
short of the column) the median as before.
"""
import json, statistics, sys

# Up to 24 m/s only. The straight is not long enough to hold a steady state above that, so
# samples near the top of a ramp are entry transients rather than response.
SPEEDS = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 11.0, 14.0, 17.0, 20.0, 24.0]
LOWSPEED_MAX = 6.0   # brake columns up to here come from probe_brake_lowspeed.py
ZERO_FROM = 1.0      # the 0 m/s brake column holds this speed's measurement
PLAUSIBLE = 13.0     # m/s^2; nothing this car does exceeds this
HALF = 1.5           # a speed cell gathers samples within this many m/s

src, out_dir, lowspeed_src = sys.argv[1], sys.argv[2], sys.argv[3]
ramps = json.load(open(src))
rows = ramps["rows"]
lowspeed = json.load(open(lowspeed_src))
if ramps.get("timing") != lowspeed.get("timing"):
    sys.exit("the two probes ran under different physics timing: %s vs %s"
             % (ramps.get("timing"), lowspeed.get("timing")))


def line_at(pts, s):
    mv = statistics.fmean(v for v, _ in pts)
    ma = statistics.fmean(a for _, a in pts)
    svv = sum((v - mv) ** 2 for v, _ in pts)
    b = sum((v - mv) * (a - ma) for v, a in pts) / svv if svv > 1e-9 else 0.0
    return lambda x: ma + b * (x - mv)


def at_speed(pts, s):
    """The response at speed `s` from (v, accel) samples near it.

    The line is fitted twice, dropping samples more than 3 MADs (at least 0.2 m/s^2) off the
    first fit: a low throttle breaks away from rest in a lurch -- throttle 0.2 goes through
    1.3-2 m/s at up to +10 m/s^2 before settling -- and those few samples sit at the end of
    the band where a line is most sensitive to them.
    """
    near = [(v, a) for v, a in pts if abs(v - s) <= HALF]
    if len(near) < 3:
        return None
    below, above = sum(v < s for v, _ in near), sum(v > s for v, _ in near)
    if below < 3 or above < 3:
        return statistics.median(a for _, a in near)
    f = line_at(near, s)
    res = [abs(a - f(v)) for v, a in near]
    cut = max(0.2, 3.0 * statistics.median(res))
    kept = [p for p, r in zip(near, res) if r <= cut]
    if len(kept) >= 3:
        f = line_at(kept, s)
    return f(s)


def ramp_grid(kind, speeds):
    pedals = sorted({r["pedal"] for r in rows if r["kind"] == kind})
    table = {}
    for p in pedals:
        pts = [(r["v"], r["accel"]) for r in rows
               if r["kind"] == kind and r["pedal"] == p and abs(r["accel"]) <= PLAUSIBLE]
        if kind == "brake":
            pts = [(v, a) for v, a in pts if v > LOWSPEED_MAX]
        table[p] = {s: c for s in speeds if (c := at_speed(pts, s)) is not None}
    return pedals, table


def lowspeed_cells():
    """{pedal: {speed: accel}} from the low-speed probe: the median over repeats."""
    got = {}
    for r in lowspeed["runs"]:
        f = r["fit"]
        if f and f.get("accel_at_target") is not None:
            got.setdefault(r["pedal"], {}).setdefault(r["target"], []).append(
                f["accel_at_target"])
    return {p: {s: statistics.median(v) for s, v in col.items()} for p, col in got.items()}


def fill(kind, pedals, table):
    """Extend each pedal column to every speed, so the map has no holes to trip a lookup.

    A hole means the car cannot reach that speed on that pedal. Only for those: a cell that
    can be measured is measured (see the module docstring for what the old fill invented).
    """
    for p in pedals:
        col = table[p]
        known = sorted(col)
        for s in SPEEDS:
            if s not in col:
                nearest = min(known, key=lambda k: abs(k - s)) if known else None
                col[s] = col[nearest] if known else 0.0
                print("  %s pedal %.2f at %4.1f m/s: no data, filled from %s"
                      % (kind, p, s, nearest))
    return table


def check_monotonic(kind, pedals, table):
    sign = 1 if kind == "throttle" else -1
    for s in SPEEDS:
        col = [table[p][s] for p in pedals]
        bad = [(pedals[i], pedals[i + 1]) for i in range(len(col) - 1)
               if sign * (col[i + 1] - col[i]) < 0]
        if bad:
            print("  %s at %4.1f m/s is not monotonic in the pedal: %s" % (kind, s, bad))


def write(kind, path, note, pedals, table):
    table = fill(kind, pedals, table)
    check_monotonic(kind, pedals, table)
    with open(path, "w") as f:
        for line in note:
            f.write("# %s\n" % line)
        f.write("# Rows: pedal position. Columns: vehicle speed in m/s.\n")
        f.write("# Cells: measured longitudinal acceleration in m/s^2.\n")
        f.write("default," + ",".join("%.1f" % s for s in SPEEDS) + "\n")
        for p in pedals:
            f.write("%.2f," % p + ",".join("%.3f" % table[p][s] for s in SPEEDS) + "\n")
    print("wrote", path)


t = ramps.get("timing", {})
timing = ("CARLA tick %.3f s, physics substeps of %.4f s (max %d) -- the timing "
          "carla_scenario_bridge uses" % (t.get("dt", 0), t.get("max_substep_delta_time", 0),
                                          t.get("max_substeps", 0)))

pedals, table = ramp_grid("throttle", [s for s in SPEEDS if s > 0.0])
for p in pedals:
    if ZERO_FROM in table[p]:
        table[p][0.0] = table[p][ZERO_FROM]
write("throttle", out_dir + "/accel_map.csv",
      ["Throttle response of %s on CARLA 0.9.16, measured by scripts/probe_longitudinal.py"
       % ramps["blueprint"], timing], pedals, table)

pedals, table = ramp_grid("brake", [s for s in SPEEDS if s > LOWSPEED_MAX])
low = lowspeed_cells()
for p in pedals:
    for s in SPEEDS:
        src_speed = ZERO_FROM if s == 0.0 else s
        if s <= LOWSPEED_MAX and src_speed in low.get(p, {}):
            table[p][s] = low[p][src_speed]
write("brake", out_dir + "/brake_map.csv",
      ["Brake response of %s on CARLA 0.9.16. Columns up to %.0f m/s measured by "
       "scripts/probe_brake_lowspeed.py (0 m/s holds the %.0f m/s value), the rest by "
       "scripts/probe_longitudinal.py" % (ramps["blueprint"], LOWSPEED_MAX, ZERO_FROM), timing],
      pedals, table)
