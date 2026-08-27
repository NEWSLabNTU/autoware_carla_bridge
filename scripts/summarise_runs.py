#!/usr/bin/env python3
"""Summarise a run table from `run_scenarios.sh`.

Reports run 1 apart from later runs, because they are not comparable: a fresh stack has
passed on every stack measured, while later runs are where regressions show. Rows with no
trace data are excluded from the tracking figures and counted separately -- treating a
harness gap as a failure is how an effect gets invented.
"""
import statistics as st
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "scenario_results.tsv"
rows = []
with open(path) as f:
    header = f.readline()
    for line in f:
        p = line.rstrip("\n").split("\t")
        if len(p) < 10:
            continue
        rows.append({
            "run": p[1], "verdict": p[2], "n": p[3],
            "xt_sd": p[7], "note": p[10] if len(p) > 10 else "",
        })

def summarise(label, sel):
    if not sel:
        return
    usable = [r for r in sel if r["n"] not in ("0", "")]
    passes = sum(1 for r in sel if r["verdict"] == "PASS")
    xt = [float(r["xt_sd"]) for r in usable if r["xt_sd"] not in ("nan", "")]
    line = f"  {label:<12} {passes}/{len(sel)} passed"
    if len(usable) != len(sel):
        line += f"  ({len(sel) - len(usable)} with no data, excluded below)"
    print(line)
    if xt:
        print(f"               xt_sd median {st.median(xt):.3f}  range {min(xt):.3f}-{max(xt):.3f}")

print(f"\n{len(rows)} runs from {path}")
summarise("run 1", [r for r in rows if r["run"] == "1"])
summarise("later runs", [r for r in rows if r["run"] != "1"])
print("\n  Lateral tracking spread is the sensitive measure; verdicts alone have hidden")
print("  a five-fold regression before. Healthy runs have sat near 0.02-0.15;")
print("  departures 0.6-2.4. See docs/issues/016.")
