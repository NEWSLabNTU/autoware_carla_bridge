#!/usr/bin/env python3
"""Compare a generated Lanelet2 OSM file against a reference."""

import argparse
import sys
from collections import Counter
from lxml import etree


def analyze_osm(path):
    """Analyze a Lanelet2 OSM file and return structured stats."""
    tree = etree.parse(path)
    root = tree.getroot()

    stats = {}

    nodes = root.findall("node")
    ways = root.findall("way")
    relations = root.findall("relation")

    stats["nodes"] = len(nodes)
    stats["ways"] = len(ways)
    stats["relations"] = len(relations)
    stats["file_size"] = len(etree.tostring(root))

    # Node attributes (sample first 100)
    local_coords = 0
    mgrs_codes = 0
    for n in nodes[:100]:
        for t in n.findall("tag"):
            if t.get("k") == "local_x":
                local_coords += 1
            if t.get("k") == "mgrs_code":
                mgrs_codes += 1
    stats["local_coords_pct"] = local_coords
    stats["mgrs_codes_pct"] = mgrs_codes

    # Way tag combinations
    way_combos = Counter()
    for w in ways:
        tags = frozenset(
            (t.get("k"), t.get("v")) for t in w.findall("tag")
            if t.get("k") not in ("lane_change",)  # skip for comparison
        )
        way_combos[tags] += 1
    stats["way_combos"] = way_combos

    # Way type distribution
    way_types = Counter()
    for w in ways:
        wtype = ""
        for t in w.findall("tag"):
            if t.get("k") == "type":
                wtype = t.get("v")
        way_types[wtype] += 1
    stats["way_types"] = way_types

    # Relation analysis
    lanelet_subtypes = Counter()
    reg_subtypes = Counter()
    lanelet_locations = Counter()
    for r in relations:
        rtype = ""
        subtype = ""
        location = ""
        for t in r.findall("tag"):
            if t.get("k") == "type":
                rtype = t.get("v")
            if t.get("k") == "subtype":
                subtype = t.get("v")
            if t.get("k") == "location":
                location = t.get("v")

        if rtype == "lanelet":
            lanelet_subtypes[subtype or "(none)"] += 1
            lanelet_locations[location or "(none)"] += 1
        elif rtype == "regulatory_element":
            reg_subtypes[subtype or "(none)"] += 1

    stats["lanelet_subtypes"] = lanelet_subtypes
    stats["reg_subtypes"] = reg_subtypes
    stats["lanelet_locations"] = lanelet_locations

    # TL way details
    tl_subtypes = Counter()
    for w in ways:
        is_tl = False
        sub = ""
        for t in w.findall("tag"):
            if t.get("k") == "type" and t.get("v") == "traffic_light":
                is_tl = True
            if t.get("k") == "subtype":
                sub = t.get("v")
        if is_tl:
            tl_subtypes[sub or "(empty)"] += 1
    stats["tl_subtypes"] = tl_subtypes

    return stats


def print_counter_comparison(label, ref_counter, gen_counter):
    """Print a side-by-side counter comparison."""
    all_keys = sorted(set(list(ref_counter.keys()) + list(gen_counter.keys())))
    if not all_keys:
        print(f"  {label}: (none)")
        return

    print(f"  {label}:")
    for key in all_keys:
        r = ref_counter.get(key, 0)
        g = gen_counter.get(key, 0)
        diff = g - r
        marker = " " if diff == 0 else "*"
        print(f"  {marker} {key:30s}  ref={r:>5}  gen={g:>5}  diff={diff:>+5}")


def main():
    parser = argparse.ArgumentParser(description="Compare Lanelet2 OSM files")
    parser.add_argument("generated", help="Path to generated lanelet2_map.osm")
    parser.add_argument("reference", help="Path to reference lanelet2_map.osm")
    args = parser.parse_args()

    ref = analyze_osm(args.reference)
    gen = analyze_osm(args.generated)

    print("=" * 64)
    print("  Lanelet2 Map Comparison")
    print("=" * 64)
    print(f"  Reference: {args.reference}")
    print(f"  Generated: {args.generated}")

    # Element counts
    print(f"\n--- Element Counts ---")
    for key in ("nodes", "ways", "relations"):
        r, g = ref[key], gen[key]
        diff = g - r
        pct = (diff / r * 100) if r else 0
        marker = " " if diff == 0 else "*"
        print(f" {marker} {key:12s}  ref={r:>10,}  gen={g:>10,}  "
              f"diff={diff:>+8,} ({pct:>+.1f}%)")

    # Coordinate attributes
    print(f"\n--- Node Attributes (sample of 100) ---")
    print(f"  local_x/y:  ref={ref['local_coords_pct']}  gen={gen['local_coords_pct']}")
    print(f"  mgrs_code:  ref={ref['mgrs_codes_pct']}  gen={gen['mgrs_codes_pct']}")

    # Way types
    print(f"\n--- Way Types ---")
    print_counter_comparison("type", ref["way_types"], gen["way_types"])

    # Lanelet subtypes
    print(f"\n--- Lanelet Subtypes ---")
    print_counter_comparison("subtype", ref["lanelet_subtypes"], gen["lanelet_subtypes"])

    # Lanelet locations
    print(f"\n--- Lanelet Locations ---")
    print_counter_comparison("location", ref["lanelet_locations"], gen["lanelet_locations"])

    # Regulatory element subtypes
    print(f"\n--- Regulatory Element Subtypes ---")
    print_counter_comparison("subtype", ref["reg_subtypes"], gen["reg_subtypes"])

    # TL way subtypes
    print(f"\n--- Traffic Light Way Subtypes ---")
    print_counter_comparison("subtype", ref["tl_subtypes"], gen["tl_subtypes"])

    # Summary
    print(f"\n--- Summary ---")
    lanelets_ref = sum(ref["lanelet_subtypes"].values())
    lanelets_gen = sum(gen["lanelet_subtypes"].values())
    print(f"  Total lanelets:  ref={lanelets_ref}  gen={lanelets_gen}  "
          f"{'MATCH' if lanelets_ref == lanelets_gen else 'MISMATCH'}")
    print(f"  Ways:            ref={ref['ways']}  gen={gen['ways']}  "
          f"{'MATCH' if ref['ways'] == gen['ways'] else 'MISMATCH'}")
    print(f"  Relations:       ref={ref['relations']}  gen={gen['relations']}  "
          f"{'MATCH' if ref['relations'] == gen['relations'] else 'MISMATCH'}")

    # Structural diffs (element counts + lanelet subtypes)
    structural_diff = 0
    for key in ("ways", "relations"):
        structural_diff += abs(gen[key] - ref[key])
    for k in set(list(ref["lanelet_subtypes"].keys()) +
                 list(gen["lanelet_subtypes"].keys())):
        structural_diff += abs(
            ref["lanelet_subtypes"].get(k, 0) -
            gen["lanelet_subtypes"].get(k, 0))

    print(f"  Structural diff: {structural_diff}")
    if structural_diff > 0:
        print(f"  (lanelet subtype classification differences are expected "
              f"between converters)")

    sys.exit(0 if structural_diff == 0 else 1)


if __name__ == "__main__":
    main()
