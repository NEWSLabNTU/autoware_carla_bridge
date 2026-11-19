"""Make each traffic light linestring face the lanes it controls (roadmap 087).

Autoware reads a light's facing from its linestring's direction: front to back must run the way the
controlled lanelet's left-to-right edge runs at the end nearest the stop line. That is
autoware_lanelet2_map_validator's TrafficLight.CorrectFacing check (traffic_light_facing.cpp), whose
rule this repeats, in the frame Autoware loads (the map's lat/lon through its projector, not the
local_x/local_y tags). A light shared by regulatory elements whose lanes face different ways (crdesigner
shares one among a junction's approaches) cannot face them all: a stub one is split, one bar per element.

crdesigner draws every light as a stub about 0.14 m long, below the resolution of the lat/lon it writes
(about 0.1 m), so its direction is noise: on Town01_Opt the validator called 18 of 36 wrong, and the
local tags and the lat/lon disagreed on some. A light shorter than MIN_LEN is therefore rebuilt as a
BAR_LEN bar centred where it was, along its lanelet's edge (so it faces the lanes by construction), with
its two nodes' lat/lon and local_x/local_y written from the projector. A longer light that faces the
wrong way is reversed. A long light whose lanes disagree is left alone and reported.

    python3 -m carla_map_gen.repairs.light_facing <map dir or lanelet2_map.osm> [--dry-run]
"""

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from carla_map_gen.projection import Projector

MIN_LEN = 0.5  # m: a light shorter than this has no reliable direction
BAR_LEN = 0.8  # m: the rebuilt bar (a three-lamp head is about that wide)


def repair(path, dry=False):
    tree = ET.parse(path)
    root = tree.getroot()
    proj = Projector.for_map(str(path))
    nodes = {n.get("id"): n for n in root.iter("node")}
    xy = {i: proj.project(float(n.get("lat")), float(n.get("lon"))) for i, n in nodes.items()}
    ways = {w.get("id"): w for w in root.iter("way")}
    nds = {i: [nd.get("ref") for nd in w.findall("nd")] for i, w in ways.items()}
    rels = list(root.iter("relation"))
    tags = {r.get("id"): {t.get("k"): t.get("v") for t in r.findall("tag")} for r in rels}
    users = {}
    for r in rels:
        if tags[r.get("id")].get("type") == "lanelet":
            for m in r.findall("member"):
                if m.get("role") == "regulatory_element":
                    users.setdefault(m.get("ref"), []).append(r)

    def edge(ll, ref):
        b = {m.get("role"): m.get("ref") for m in ll.findall("member") if m.get("type") == "way"}
        left, right = nds.get(b.get("left"), []), nds.get(b.get("right"), [])
        if not left or not right or len(ref) < 2:
            return None
        fl, bl, fr, br = xy[left[0]], xy[left[-1]], xy[right[0]], xy[right[-1]]
        r1, r2 = xy[ref[0]], xy[ref[-1]]
        d = lambda a, c: math.dist(a, r1) + math.dist(c, r2)
        a, c = (fl, fr) if min(d(fl, fr), d(fr, fl)) <= min(d(bl, br), d(br, bl)) else (bl, br)
        return (c[0] - a[0], c[1] - a[1])

    found = {}  # light way -> list of edges of the lanes it controls
    for r in rels:
        t = tags[r.get("id")]
        if t.get("type") != "regulatory_element" or t.get("subtype") != "traffic_light":
            continue
        mem = r.findall("member")
        stop = next((m.get("ref") for m in mem if m.get("role") == "ref_line"), None)
        lls = users.get(r.get("id"), [])
        if stop is None or not lls:
            continue
        e = edge(lls[0], nds.get(stop, []))
        if e is None or math.hypot(*e) < 1e-6:
            continue
        for m in mem:
            w = ways.get(m.get("ref")) if m.get("role") == "refers" else None
            if w is None:
                continue
            wt = {k.get("k"): k.get("v") for k in w.findall("tag")}
            if wt.get("type") == "traffic_light" and wt.get("subtype") == "red_yellow_green":
                found.setdefault(w.get("id"), []).append((e, r, m))

    def set_xy(nid, x, y):
        lat, lon = proj.unproject(x, y)
        n = nodes[nid]
        n.set("lat", repr(lat))
        n.set("lon", repr(lon))
        t = {k.get("k"): k for k in n.findall("tag")}
        for k, v in (("local_x", x), ("local_y", y)):
            if k in t:
                t[k].set("v", repr(v))
            else:
                ET.SubElement(n, "tag", k=k, v=repr(v))

    next_id = [max(int(i) for i in list(nodes) + list(ways) + [r.get("id") for r in rels]) + 1]

    def new_id():
        next_id[0] += 1
        return str(next_id[0] - 1)

    def first_index(tag):
        return next((i for i, c in enumerate(list(root)) if c.tag == tag), len(list(root)))

    def unit(e):
        n = math.hypot(*e)
        return (e[0] / n, e[1] / n)

    rebuilt, turned, ambiguous, split = [], [], [], []
    for wid, uses in found.items():
        ids = nds[wid]
        a, b = xy[ids[0]], xy[ids[-1]]
        light = (b[0] - a[0], b[1] - a[1])
        units = [unit(e) for e, _, _ in uses]
        mean = (sum(u[0] for u in units), sum(u[1] for u in units))
        stub = math.hypot(*light) < MIN_LEN and len(ids) == 2 and len(set(ids)) == 2
        cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        h = BAR_LEN / 2
        if any(u[0] * units[0][0] + u[1] * units[0][1] < 0.707 for u in units) or math.hypot(*mean) < 1e-6:
            # its lanes' edges diverge (crdesigner shares one light among approaches). A stub is split: every
            # regulatory element after the first gets its own copy of the light, a bar facing its lanes
            if not stub:
                ambiguous.append(wid)
                continue
            w = ways[wid]
            wtags = [(t.get("k"), t.get("v")) for t in w.findall("tag")]
            ele = {k.get("k"): k.get("v") for k in nodes[ids[0]].findall("tag")}.get("ele")
            for i, ((e, r, m), u) in enumerate(zip(uses, units)):
                if i == 0:
                    set_xy(ids[0], cx - u[0] * h, cy - u[1] * h)
                    set_xy(ids[1], cx + u[0] * h, cy + u[1] * h)
                    continue
                nid = []
                for sx in (-1, 1):
                    n = ET.Element("node", id=new_id(), visible="true", version="1", lat="0", lon="0")
                    root.insert(first_index("way"), n)  # nodes before the ways, as OSM lists them
                    if ele is not None:
                        ET.SubElement(n, "tag", k="ele", v=ele)
                    nodes[n.get("id")] = n
                    set_xy(n.get("id"), cx + sx * u[0] * h, cy + sx * u[1] * h)
                    nid.append(n.get("id"))
                nw = ET.Element("way", id=new_id(), visible="true", version="1")
                root.insert(first_index("relation"), nw)
                for i2 in nid:
                    ET.SubElement(nw, "nd", ref=i2)
                for k, v in wtags:
                    ET.SubElement(nw, "tag", k=k, v=v)
                m.set("ref", nw.get("id"))
            split.append(wid)
            continue
        u = unit(mean)
        if stub:
            set_xy(ids[0], cx - u[0] * h, cy - u[1] * h)
            set_xy(ids[1], cx + u[0] * h, cy + u[1] * h)
            rebuilt.append(wid)
        elif light[0] * u[0] + light[1] * u[1] < 0:
            w = ways[wid]
            refs = w.findall("nd")
            for nd in refs:
                w.remove(nd)
            for i, nd in enumerate(reversed(refs)):
                w.insert(i, nd)
            turned.append(wid)
    if (rebuilt or turned or split) and not dry:
        backup = Path(str(path) + ".orig")
        if not backup.exists():
            shutil.copy(path, backup)
        tree.write(path, encoding="UTF-8", xml_declaration=True)
    return rebuilt, turned, ambiguous, split


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", nargs="?", default="data/carla-autoware-bridge")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    root = Path(a.root)
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    for p in maps:
        rebuilt, turned, ambiguous, split = repair(p, a.dry_run)
        print(f"  {p.parent.name}: {len(rebuilt)} stub light(s) rebuilt as {BAR_LEN} m bars facing their lanes, "
              f"{len(split)} shared stub(s) split into one bar per regulatory element, "
              f"{len(turned)} turned" + (f"; {len(ambiguous)} left alone (lanes disagree): {', '.join(ambiguous)}"
                                         if ambiguous else "") + (" (dry run)" if a.dry_run else ""))


if __name__ == "__main__":
    main()
