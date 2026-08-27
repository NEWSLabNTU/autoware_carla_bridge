"""Score one trace for lateral tracking quality over the straight approach.

Outcome for the report_measured_steering A/B (acb issue 016). Pass/fail barely
discriminates here, so this measures the thing the issue is actually about: how well the
ego holds its path while driving down the straight.

Two references, deliberately:
  lat_*  deviation from the lane centre (a fixed map y on this route)
  xt_*   the trajectory's own cross-track error, which needs no lane assumption
Only samples where the ego is actually driving down the straight are counted.
"""
import sys
import statistics

LANE_Y = -129.9
X_HI, X_LO = 188.0, 126.0   # the straight approach, clear of spawn and goal
MIN_SPEED = 0.5

# The trace window is longer than one scenario, so a file can span two drives. Score only
# the FIRST westward traversal: start at the first moving sample inside the window, and stop
# when x climbs well above the running minimum, which is the next run respawning at 190.8.
samples = []
with open(sys.argv[1]) as f:
    for line in f:
        p = line.split()
        if len(p) < 16 or not p[0].isdigit():
            continue
        try:
            ax, ay, vel = float(p[5]), float(p[6]), float(p[7])
            report, cmd, xtrack = float(p[8]), float(p[9]), float(p[13])
        except ValueError:
            continue
        samples.append((ax, ay, vel, xtrack, report, cmd))

rows, arm_gaps, started, xmin = [], [], False, float("inf")
for ax, ay, vel, xtrack, report, cmd in samples:
    if not started:
        if vel >= MIN_SPEED and X_LO <= ax <= X_HI:
            started, xmin = True, ax
        else:
            continue
    if ax > xmin + 5.0:      # respawned: the next run has begun
        break
    xmin = min(xmin, ax)
    if vel >= MIN_SPEED and X_LO <= ax <= X_HI:
        rows.append((ay - LANE_Y, xtrack))
        arm_gaps.append(abs(report - cmd))

if len(rows) < 5:
    print("0\tnan\tnan\tnan\tnan\tnan\tno-data")
    sys.exit(0)

# NOT the arm. Comparing the steering report against the command at 1 Hz measures how fast
# steering happens to be changing, because the two topics publish at 20 Hz and arrive
# asynchronously -- the sampling skew swamps the difference between echoing the command and
# reporting the measured wheel. Measured runs came out at 0.007 and commanded ones at 0.036,
# i.e. backwards. Verify the arm from play_launch's own
# play_log/ego/<dir>/node/acb_bridge/params_files/overrides.yaml instead, which is written at
# launch and is unambiguous. Kept only as a diagnostic of steering activity.
gap = statistics.median(arm_gaps)
arm = f"gap={gap:.4f}"

lat = [abs(a) for a, _ in rows]
xt = [abs(b) for _, b in rows]
lat_sd = statistics.pstdev([a for a, _ in rows])
xt_sd = statistics.pstdev([b for _, b in rows])
lat_s = sorted(lat)
p95 = lat_s[min(len(lat_s) - 1, int(0.95 * len(lat_s)))]
print(f"{len(rows)}\t{lat_sd:.3f}\t{p95:.3f}\t{max(lat):.3f}\t{xt_sd:.3f}\t{max(xt):.3f}\t{arm}")
