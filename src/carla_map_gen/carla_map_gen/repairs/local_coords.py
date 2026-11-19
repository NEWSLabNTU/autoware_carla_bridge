#!/usr/bin/env python3
"""Set every node's `local_x` / `local_y` in a Lanelet2 map to its position in Autoware's map
frame.

Autoware and SSv2 place a node from its lat/lon, through the TransverseMercator the map's
`map_projector_info.yaml` declares (`carla_map_gen.projection`); neither reads the local tags
(Autoware only under a `Local` projector, SSv2 never). scensim reads them: scensim_core's traffic
light table, the other repair_lanelet_*.py scripts, scensim_isaac_world's surface check. So they
must hold the same frame, or scensim and Autoware disagree about where a thing is.

The pack's tags did not. They were CARLA's spherical Mercator on R = 6 378 137 m about
(0, 0), x = R lon, y = R ln(tan(pi/4 + lat/2)), 0.7 % long in y against the TransverseMercator
(2.2 m on Town01's y = -330 road), and 50 of Town10's nodes, on stop lines and signs, had
none. Measured on Town01 (docs/design/ssv2-feature-completeness.md, "Lane position"): the
projected lat/lon is the frame of OpenDRIVE's lanes, the point cloud map and the driven ego;
the tags are not.

The repair sets both tags on every node from `Projector.project(lat, lon)`: one `local_x` and one
`local_y`, the first of each kept and rewritten, any further ones removed, added as the node's first
tags where absent. lat/lon is taken as the truth: a map that `regenerate_light_geometry.py` or
`repairs/signal_links.py` edited before they wrote lat/lon with `Projector.unproject` has its new
nodes' lat/lon wrong; rerun that script first. It reads and writes the map through ElementTree, as
the other repairs do (issue 067: the text edits it made before matched `"/>` only, so on a map an
earlier repair had written back (`" />`) it added a second pair to every node and left the stale one).
Idempotent; the first edit keeps a `.orig` copy (shared with the other repairs, whichever runs first).

    python3 -m carla_map_gen.repairs.local_coords data/carla-autoware-bridge
"""
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from carla_map_gen.projection import Projector

KEYS = ("local_x", "local_y")


def set_local(node, projector):
    """one <node> element with exactly one local_x/local_y, from its lat/lon; whether it changed"""
    lat, lon = node.get("lat"), node.get("lon")
    if lat is None or lon is None:
        return False
    x, y = projector.project(float(lat), float(lon))
    values = {"local_x": "%.4f" % x, "local_y": "%.4f" % y}
    changed = False
    for i, k in enumerate(KEYS):
        tags = [t for t in node.findall("tag") if t.get("k") == k]
        if tags:
            if tags[0].get("v") != values[k]:
                tags[0].set("v", values[k])
                changed = True
            for extra in tags[1:]:
                node.remove(extra)
                changed = True
        else:
            tag = ET.Element("tag", k=k, v=values[k])
            # the pack's layout: one tag per line, at the node's children's indent
            if not len(node):  # a node without tags (<node .../>): open it
                node.text, tag.tail = "\n    ", "\n  "
            elif i == 0:
                tag.tail = node.text
            else:
                tag.tail, node[i - 1].tail = node[i - 1].tail, node.text
            node.insert(i, tag)
            changed = True
    return changed


def repair(path):
    """the number of nodes whose local tags changed"""
    projector = Projector.for_map(path)
    tree = ET.parse(path)
    changed = sum(set_local(n, projector) for n in tree.getroot().iter("node"))
    if changed:
        backup = path.with_suffix(path.suffix + ".orig")
        if not backup.exists():
            shutil.copy2(path, backup)
        tree.write(path, encoding="UTF-8", xml_declaration=True)
    return changed


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m)
        total += n
        print(f"  {m.parent.name}: {n} node(s) given Autoware's local_x/local_y" if n
              else f"  {m.parent.name}: every node's local_x/local_y is Autoware's")
    print(f"{total} node local coordinate(s) repaired")


if __name__ == "__main__":
    main()
