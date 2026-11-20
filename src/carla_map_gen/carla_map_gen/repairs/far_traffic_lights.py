#!/usr/bin/env python3
"""Keep traffic-light regulatory elements to the lights at their own stop line.

An OpenDRIVE -> Lanelet2 conversion can attach a traffic-light element to a lanelet whose
stop line is nowhere near the light it refers to. On CARLA's Town04 (converted with
CommonRoad Scenario Designer 0.8.5, 2026-10-14) 69 of 128 elements were such, one at the
foot of the highway ramp pointing at a light 400 m away, and Autoware stopped at the merge
waiting for a signal that never comes. On Town03 converted from CARLA 0.9.16's OpenDRIVE
(scensim issue 073, 2026-11-20) 37 of 94 elements refer only to lights beyond 60 m of their
stop line, and 14 more mix their own light with lights at another junction (one, 138167,
refers to its light 16 m away and to two 328 m away). acb publishes the lights within 100 m
of the ego (ground_truth_range_m) and gives an element the most restrictive colour of the
lights it publishes, so a far element is a stop that never ends, and a far light within
range can turn a near green red.

A light is far when its centre is more than FAR metres (default 60) from the centre of its
element's `ref_line`:

- an element whose every light is far is removed: the lanelets stop referencing it, and the
  element goes. Autoware's map validator requires every traffic-light element to have a
  referrer (`traffic_light.missing_referrers`; an unlinked element left in place counted 37 on
  Town03, scensim issue 073);
- an element with near and far lights keeps the near ones and drops the far ones from its
  `refers`.

A light way or stop line that nothing uses any more (it belonged only to a removed element or a
far grouping) is removed, with its nodes no other way uses: the validator would count a light
as `traffic_light.missing_referrers` too. Distances use the nodes' `local_x` / `local_y` tags
(run `repairs/local_coords.py` first if the map has none). Idempotent: a second run finds
nothing.

  python3 -m carla_map_gen.repairs.far_traffic_lights <map dir, root of towns, or .osm>
      [--far 60] [--dry-run]

Roadmap 031, stage 1 (defects to expect); scensim issue 073.
"""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

FAR_M = 60.0


def tags(el):
    return {t.get("k"): t.get("v") for t in el.iter("tag")}


def repair(path, far=FAR_M, dry=False):
    """One map. Returns (removed element ids, lanelet references dropped, trimmed element
    ids, far lights dropped from trimmed elements, ways left unused and removed)."""
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = {}
    for n in root.iter("node"):
        t = tags(n)
        if "local_x" in t and "local_y" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.iter("nd")] for w in root.iter("way")}

    def centre(way_id):
        pts = [nodes[i] for i in ways.get(way_id, []) if i in nodes]
        if not pts:
            return None
        return (sum(x for x, _ in pts) / len(pts), sum(y for _, y in pts) / len(pts))

    unlinked, trimmed = set(), []
    lights_dropped = 0
    referred_before = set()
    elements = [r for r in root.iter("relation") if tags(r).get("subtype") == "traffic_light"]
    for r in elements:
        lights = [m for m in r.findall("member") if m.get("role") == "refers"]
        referred_before.update(m.get("ref") for m in lights)
        lines = [centre(m.get("ref")) for m in r.findall("member") if m.get("role") == "ref_line"]
        if not lines or lines[0] is None or not lights:
            continue
        dist = [(m, centre(m.get("ref"))) for m in lights]
        if any(c is None for _, c in dist):
            continue  # a light without local coordinates: leave the element as it is
        far_ones = [m for m, c in dist if math.dist(c, lines[0]) > far]
        if len(far_ones) == len(lights):
            unlinked.add(r.get("id"))
        elif far_ones:
            for m in far_ones:
                r.remove(m)
            trimmed.append(r.get("id"))
            lights_dropped += len(far_ones)

    dropped = 0
    for r in root.iter("relation"):
        for m in list(r.findall("member")):
            if m.get("role") == "regulatory_element" and m.get("ref") in unlinked:
                r.remove(m)
                dropped += 1
    # the far elements go: a traffic-light element no lanelet refers to fails the validator
    held = set()
    for r in list(root.findall("relation")):
        if r.get("id") in unlinked:
            held |= {m.get("ref") for m in r.findall("member") if m.get("type") == "way"}
            root.remove(r)
    removed = sorted(unlinked)

    used_ways = {m.get("ref") for r in root.iter("relation") for m in r.findall("member")
                 if m.get("type") == "way"}
    # a light way only a far grouping referred to, or a way only a removed element held, belongs
    # to nothing now: removed, with the nodes no other way uses
    gone = sorted(((referred_before - used_ways) | held) - used_ways)
    if gone:
        gone_set = set(gone)
        for w in list(root.findall("way")):
            if w.get("id") in gone_set:
                root.remove(w)
        used = {nd.get("ref") for w in root.iter("way") for nd in w.iter("nd")}
        used |= {m.get("ref") for r in root.iter("relation") for m in r.findall("member")
                 if m.get("type") == "node"}
        candidates = {n for wid in gone_set for n in ways.get(wid, [])}
        for n in list(root.findall("node")):
            if n.get("id") in candidates and n.get("id") not in used:
                root.remove(n)
    if (removed or lights_dropped or gone) and not dry:
        tree.write(path, encoding="UTF-8", xml_declaration=True)
    return removed, dropped, trimmed, lights_dropped, gone


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", help="a map directory, a root of town directories, or a .osm")
    ap.add_argument("--far", type=float, default=FAR_M, help="metres, light to stop line")
    ap.add_argument("--dry-run", action="store_true", help="report, do not write")
    a = ap.parse_args(argv)
    root = Path(a.root)
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    for p in maps:
        removed, dropped, trimmed, lights, gone = repair(p, a.far, a.dry_run)
        name = p.parent.name if p.name == "lanelet2_map.osm" else p.name
        print(f"  {name}: {len(removed)} traffic-light element(s) with every light beyond "
              f"{a.far:g} m of their stop line removed ({dropped} lanelet reference(s)); "
              f"{len(trimmed)} element(s) kept their near lights and dropped {lights} far "
              f"one(s)" + (f"; {len(gone)} way(s) nothing uses any more removed" if gone else "")
              + (" (dry run)" if a.dry_run else ""))


if __name__ == "__main__":
    main()
