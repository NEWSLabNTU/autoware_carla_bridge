#!/usr/bin/env python3
"""How many compile jobs acb's `just build` runs at once (scensim issue 121).

colcon's defaults ask for one compiler per core: a package per core at once, make at -jN and
cargo at N jobs, N the CPU count, whatever the host has free. scensim's build measured about
2 GB a C++ compiler (SSv2's); acb's carla-sys compiles LibCarla's C++, and its Rust crates
build, at a job per core the same way. This is scensim's scripts/build_jobs.py, with the same
host setting: the total is SCENSIM_BUILD_JOBS, or else half of MemAvailable at the build's
start at JOB_GB a job, at least 1 and at most the CPU count; colcon runs WORKERS packages at
once (1 under 4 jobs), and each package's make and cargo get the total divided by them.

    eval "$(python3 scripts/build_jobs.py)"   # MAKEFLAGS, CARGO_BUILD_JOBS, COLCON_WORKERS

It prints shell assignments on stdout and a line saying what it chose on stderr. A MAKEFLAGS
with a job count, a CARGO_BUILD_JOBS or a COLCON_DEFAULTS_FILE already in the environment is
kept: that part is not assigned. COLCON_WORKERS is empty then, and the recipe passes no
--parallel-workers."""
import os
import re
import shlex
import sys

JOB_GB = 2.5     # a compiler on SSv2's C++: about 2 GB measured (243, 2026-11-23), and a margin
SHARE = 0.5      # of MemAvailable: the rest stays for what runs beside the build, which grows
WORKERS = 2      # colcon packages at once
ENV = "SCENSIM_BUILD_JOBS"


def mem_available_gb(meminfo="/proc/meminfo"):
    with open(meminfo) as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / (1024 * 1024)
    raise RuntimeError(f"{meminfo} has no MemAvailable")


def plan(cpus, mem_gb, env):
    """(assignments, note): the shell variables to set and a line saying why. Pure."""
    setting = env.get(ENV, "").strip()
    if setting:
        if not setting.isdigit() or int(setting) < 1:
            raise ValueError(f"{ENV}={setting!r}: a whole number of jobs, 1 or more")
        total, why = int(setting), f"{ENV}={setting}"
    else:
        total = max(1, min(cpus, int(SHARE * mem_gb // JOB_GB)))
        why = f"half of {mem_gb:.1f} GB available at {JOB_GB} GB a job, {cpus} CPUs"
    workers = 1 if total < 4 else WORKERS
    per = max(1, total // workers)
    out, kept = {}, []
    if re.search(r"(^|\s)(-j\s*\d+|--jobs(=|\s+)\d+)", env.get("MAKEFLAGS", "")):
        kept.append(f"MAKEFLAGS={env['MAKEFLAGS']}")
    else:  # other flags in it stay
        out["MAKEFLAGS"] = f"{env.get('MAKEFLAGS', '')} -j{per}".strip()
    if env.get("CARGO_BUILD_JOBS"):
        kept.append(f"CARGO_BUILD_JOBS={env['CARGO_BUILD_JOBS']}")
    else:
        out["CARGO_BUILD_JOBS"] = str(per)
    if env.get("COLCON_DEFAULTS_FILE"):
        kept.append(f"COLCON_DEFAULTS_FILE={env['COLCON_DEFAULTS_FILE']}")
        out["COLCON_WORKERS"] = ""
    else:
        out["COLCON_WORKERS"] = str(workers)
    note = (f"build jobs: {total} ({why}): colcon {workers} package(s) at once, "
            f"make and cargo {per} each" + (f"; kept from the environment: {', '.join(kept)}"
                                            if kept else ""))
    return out, note


def main():
    out, note = plan(os.cpu_count() or 1, mem_available_gb(), os.environ)
    print(note, file=sys.stderr)
    for k, v in out.items():
        print(("" if k == "COLCON_WORKERS" else "export ") + f"{k}={shlex.quote(v)}")


if __name__ == "__main__":
    try:
        main()
    except ValueError as e:
        print(f"just build: {e}", file=sys.stderr)
        sys.exit(2)
