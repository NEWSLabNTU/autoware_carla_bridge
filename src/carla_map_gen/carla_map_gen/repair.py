"""Run the lanelet2 repairs on a map, in order (roadmap 087; the order of scensim's download_maps.sh).

    python3 -m carla_map_gen.repair <map dir | maps root> [--xodr FILE | --xodr-dir DIR] [--signal-links TOWN --port N]

Each repair is idempotent and keeps the first `.orig` backup, so this also runs on maps already repaired.
The OpenDRIVE repairs (speed limits, walkways under roads, non-driving lanes) need the town's `.xodr`:
`--xodr` names the file for a single map directory, `--xodr-dir` holds `<Town>.xodr` (or `<Town>/main.xodr`)
for a root of towns. Without either they are skipped. `--signal-links` relinks the lights to the ones a live
CARLA controls (repairs/signal_links.py); it needs the server and is off by default.
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

# (module, needs the xodr) in download_maps.sh's order; stop_lines and light_facing (087, the latter on local coordinates)
STEPS = [
    ("traffic_lights", False),
    ("stop_lines", False),
    ("subtypes", False),
    ("line_types", False),
    ("bound_directions", False),
    ("local_coords", False),
    ("light_facing", False),
    ("duplicate_points", False),
    ("turn_direction", False),
    ("joins", False),
    ("right_of_way", False),
    ("speed_limits", True),
    ("walkways_under_roads", True),
    ("non_driving_lanes", True),
]


def run(module, args):
    cmd = [sys.executable, "-m", f"carla_map_gen.repairs.{module}", *args]
    print(f"== {module}", flush=True)
    rc = subprocess.call(cmd)
    if rc != 0:
        raise SystemExit(f"repair {module} failed (rc {rc}): {' '.join(cmd)}")


def repair_map(root, xodr=None, xodr_dir=None, signal_links=None, host="localhost", port=2000):
    root = Path(root)
    with tempfile.TemporaryDirectory() as tmp:
        if xodr is not None:
            # one map directory: offer its xodr under the directory's name, as find_xodr looks it up
            xodr_dir = Path(tmp)
            (xodr_dir / f"{root.name}.xodr").symlink_to(Path(xodr).resolve())
        for module, needs_xodr in STEPS:
            if needs_xodr:
                if xodr_dir is None:
                    print(f"== {module}: skipped (no --xodr / --xodr-dir)", flush=True)
                    continue
                run(module, [str(root), "--xodr-dir", str(xodr_dir)])
            else:
                run(module, [str(root)])
    if signal_links:
        run("signal_links", [signal_links, "--map", str(root), "--host", host, "--port", str(port)])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", help="a map directory (lanelet2_map.osm inside) or a root of town directories")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--xodr", type=Path, help="the map's OpenDRIVE file (a single map directory)")
    g.add_argument("--xodr-dir", type=Path, help="a directory of <Town>.xodr files (a root of towns)")
    ap.add_argument("--signal-links", metavar="TOWN", help="also relink the lights against a live CARLA running TOWN")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args(argv)
    repair_map(a.root, a.xodr, a.xodr_dir, a.signal_links, a.host, a.port)


if __name__ == "__main__":
    main()
