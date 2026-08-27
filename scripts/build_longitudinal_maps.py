#!/usr/bin/env python3
"""Turn probe_longitudinal.py samples into accel/brake maps in Autoware's convention."""
import json, statistics, sys

# Up to 24 m/s only. The straight is not long enough to hold a steady state above that, so
# samples near the top of a ramp are entry transients rather than response.
SPEEDS = [0.0, 2.0, 4.0, 6.0, 8.0, 11.0, 14.0, 17.0, 20.0, 24.0]
# A brake ramp's final ticks discretise the stop itself and produce apparent decelerations far
# beyond what the car can do; they are not braking measurements.
BRAKE_V_FLOOR = 5.0
PLAUSIBLE = 13.0   # m/s^2; nothing this car does exceeds this
HALF = 1.5   # a speed cell gathers samples within this many m/s

src, out_dir = sys.argv[1], sys.argv[2]
rows = json.load(open(src))["rows"]

def grid(kind):
    pedals = sorted({r["pedal"] for r in rows if r["kind"] == kind})
    table = {}
    for p in pedals:
        pts = [(r["v"], r["accel"]) for r in rows
               if r["kind"] == kind and r["pedal"] == p
               and abs(r["accel"]) <= PLAUSIBLE
               and (kind != "brake" or r["v"] >= BRAKE_V_FLOOR)]
        col = {}
        for s in SPEEDS:
            near = [a for v, a in pts if abs(v - s) <= HALF]
            if len(near) >= 3:
                col[s] = statistics.median(near)
        table[p] = col
    return pedals, table

def fill(pedals, table):
    """Extend each pedal column to every speed, so the map has no holes to trip a lookup.

    A hole means the car cannot reach that speed on that pedal -- throttle 0.2 never gets past
    3.6 m/s. Holding the last measured value there is the honest extrapolation: it says the
    pedal delivers no more than it was last seen to deliver, which is what the inverse map
    needs in order to reject an unreachable request rather than interpolate a fiction.
    """
    for p in pedals:
        col = table[p]
        known = sorted(col)
        if not known:
            col.update({s: 0.0 for s in SPEEDS})
            continue
        for s in SPEEDS:
            if s not in col:
                nearest = min(known, key=lambda k: abs(k - s))
                col[s] = col[nearest]
    return table

def write(kind, path, note):
    pedals, table = grid(kind)
    table = fill(pedals, table)
    with open(path, "w") as f:
        f.write("# %s\n" % note)
        f.write("# Rows: pedal position. Columns: vehicle speed in m/s.\n")
        f.write("# Cells: measured longitudinal acceleration in m/s^2.\n")
        f.write("default," + ",".join("%.1f" % s for s in SPEEDS) + "\n")
        for p in pedals:
            f.write("%.2f," % p + ",".join("%.3f" % table[p][s] for s in SPEEDS) + "\n")
    print("wrote", path)

write("throttle", out_dir + "/accel_map.csv",
      "Throttle response of vehicle.tesla.model3 on CARLA 0.9.16, measured by "
      "scripts/probe_longitudinal.py")
write("brake", out_dir + "/brake_map.csv",
      "Brake response of vehicle.tesla.model3 on CARLA 0.9.16, measured by "
      "scripts/probe_longitudinal.py")
