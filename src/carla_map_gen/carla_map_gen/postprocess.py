"""Post-process Lanelet2 OSM files for Autoware compatibility."""

from lxml import etree


def postprocess_osm(osm_path: str) -> None:
    """Post-process Lanelet2 OSM file in-place.

    Fixes:
    1. Traffic light way subtypes: set empty subtype to "red_yellow_green"
    2. Remove empty speed_limit regulatory elements (zero members)
    3. Strip curbstone/traffic_sign way type tags (not in Autoware format)
    4. Remove orphan traffic_sign ways (not referenced by any relation)
    5. Reclassify shoulder lanelets as road (between road and walkway)
    """
    tree = etree.parse(osm_path)
    root = tree.getroot()

    fixed_tl = _fix_traffic_light_subtypes(root)
    removed_sl = _remove_empty_speed_limits(root)
    stripped = _strip_extra_way_types(root)
    removed_orphans = _remove_orphan_ways(root)
    reclassified = _reclassify_shoulder_lanelets(root)

    tree.write(osm_path, xml_declaration=True, encoding="UTF-8")

    print(f"  Post-processing: fixed {fixed_tl} TL subtypes, "
          f"removed {removed_sl} empty speed_limit relations, "
          f"stripped {stripped} way types, "
          f"removed {removed_orphans} orphan ways, "
          f"reclassified {reclassified} shoulder lanelets as road")


def _fix_traffic_light_subtypes(root):
    """Set empty traffic light way subtypes to red_yellow_green."""
    count = 0
    for way in root.findall("way"):
        is_tl = False
        subtype_tag = None
        for tag in way.findall("tag"):
            if tag.get("k") == "type" and tag.get("v") == "traffic_light":
                is_tl = True
            if tag.get("k") == "subtype":
                subtype_tag = tag
        if is_tl and subtype_tag is not None and not subtype_tag.get("v"):
            subtype_tag.set("v", "red_yellow_green")
            count += 1
    return count


def _remove_empty_speed_limits(root):
    """Remove speed_limit relations with zero members and clean up references."""
    ids_to_remove = set()
    for relation in root.findall("relation"):
        is_speed_limit = False
        for tag in relation.findall("tag"):
            if tag.get("k") == "subtype" and tag.get("v") == "speed_limit":
                is_speed_limit = True
                break
        if is_speed_limit and len(relation.findall("member")) == 0:
            ids_to_remove.add(relation.get("id"))
            root.remove(relation)

    # Remove references to deleted relations from lanelet relations
    if ids_to_remove:
        for relation in root.findall("relation"):
            for member in relation.findall("member"):
                if (member.get("ref") in ids_to_remove
                        and member.get("type") == "relation"):
                    relation.remove(member)

    return len(ids_to_remove)


def _strip_extra_way_types(root):
    """Strip curbstone and traffic_sign type/subtype tags from ways.

    CommonRoad generates these way types but the Autoware/TUMFTM reference
    format uses untyped ways for lanelet boundaries. Traffic light ways
    are kept as they're properly referenced by regulatory elements.
    """
    strip_types = {"curbstone", "traffic_sign"}
    count = 0
    for way in root.findall("way"):
        way_type = None
        type_tag = None
        subtype_tag = None
        for tag in way.findall("tag"):
            if tag.get("k") == "type":
                way_type = tag.get("v")
                type_tag = tag
            if tag.get("k") == "subtype":
                subtype_tag = tag
        if way_type in strip_types:
            way.remove(type_tag)
            if subtype_tag is not None:
                way.remove(subtype_tag)
            # Also remove virtual tag if present
            for tag in way.findall("tag"):
                if tag.get("k") == "virtual":
                    way.remove(tag)
            count += 1
    return count


def _remove_orphan_ways(root):
    """Remove ways not referenced by any relation."""
    # Collect all way IDs referenced by relations
    referenced_way_ids = set()
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            if member.get("type") == "way":
                referenced_way_ids.add(member.get("ref"))

    # Also collect ways referenced by other ways (shouldn't happen in OSM, but safe)
    # Remove unreferenced ways that have type tags (don't remove lanelet boundaries)
    count = 0
    for way in list(root.findall("way")):
        if way.get("id") not in referenced_way_ids:
            # Only remove typed ways (traffic_sign, etc.), keep boundary ways
            tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
            if tags:  # has tags = typed way, safe to remove if orphaned
                root.remove(way)
                count += 1
    return count


def _reclassify_shoulder_lanelets(root):
    """Reclassify shoulder lanelets (between road and walkway) as road.

    CommonRoad leaves lanelets between roads and sidewalks untyped (no
    subtype). The TUMFTM reference classifies these as road. We detect
    them by finding untyped lanelets that share a boundary with a road
    lanelet and set subtype=road, location=urban.
    """
    # Build way_id -> [lanelet_relation_id] mapping
    way_to_lanelets = {}
    lanelet_info = {}
    for relation in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        rid = relation.get("id")
        subtype = tags.get("subtype", "")
        lanelet_info[rid] = {"subtype": subtype, "element": relation}
        for member in relation.findall("member"):
            if member.get("type") == "way":
                wid = member.get("ref")
                way_to_lanelets.setdefault(wid, []).append(rid)

    # Find untyped lanelets adjacent to road lanelets
    count = 0
    for rid, info in lanelet_info.items():
        if info["subtype"]:
            continue  # already has a subtype

        # Check if adjacent to a road lanelet
        adj_to_road = False
        for member in info["element"].findall("member"):
            if member.get("type") != "way":
                continue
            wid = member.get("ref")
            for adj_rid in way_to_lanelets.get(wid, []):
                if adj_rid != rid and lanelet_info[adj_rid]["subtype"] == "road":
                    adj_to_road = True
                    break
            if adj_to_road:
                break

        if adj_to_road:
            # Add subtype=road and location=urban
            relation = info["element"]
            relation.append(_make_tag("subtype", "road"))
            relation.append(_make_tag("location", "urban"))
            info["subtype"] = "road"
            count += 1

    return count


def _make_tag(k, v):
    """Create an OSM tag element."""
    tag = etree.Element("tag")
    tag.set("k", k)
    tag.set("v", v)
    return tag
