"""Type every traffic light's untyped ref_line as a stop line (roadmap 087).

Autoware's traffic-light module stops the car at the light regulatory element's ref_line, and
autoware_lanelet2_map_validator requires that line to be a `stop_line`
(TrafficLight.RegulatoryElementDetails-003). TUM's pack and crdesigner both write the ref_line
as a bare two-node way with no tags: all 36 of Town01's lights. A ref_line that already has a
type is left alone, whatever it is.

    python3 -m carla_map_gen.repairs.stop_lines <map dir or lanelet2_map.osm> [--dry-run]
"""

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def repair(path, dry=False):
    tree = ET.parse(path)
    root = tree.getroot()
    ways = {w.get("id"): w for w in root.iter("way")}
    fixed = 0
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        for m in rel.findall("member"):
            if m.get("role") != "ref_line" or m.get("type") != "way":
                continue
            w = ways.get(m.get("ref"))
            if w is None or any(t.get("k") == "type" for t in w.findall("tag")):
                continue
            ET.SubElement(w, "tag", k="type", v="stop_line")
            fixed += 1
    if fixed and not dry:
        backup = Path(str(path) + ".orig")
        if not backup.exists():
            shutil.copy(path, backup)
        tree.write(path, encoding="UTF-8", xml_declaration=True)
    return fixed


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", nargs="?", default="data/carla-autoware-bridge")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    root = Path(a.root)
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    for p in maps:
        n = repair(p, a.dry_run)
        print(f"  {p.parent.name}: {n} traffic-light ref_line(s) typed stop_line" + (" (dry run)" if a.dry_run else ""))


if __name__ == "__main__":
    main()
