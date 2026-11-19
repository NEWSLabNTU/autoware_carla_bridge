"""carla_map_gen's projection, georeference and light repairs (roadmap 087).

    python3 -m pytest src/carla_map_gen/test
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_map_gen.projection import MgrsProjector, Projector  # noqa: E402
from carla_map_gen.repairs import light_facing, stop_lines  # noqa: E402


@pytest.mark.parametrize("lat, lon, grid", [(22.9997300995886, 120.227025902544, "51QTF"), (52.5, 13.4, "33UUU"),
                                            (35.68, 139.69, "54SUE"), (-33.9, 151.2, "56HLH")])
def test_mgrs_square_and_round_trip(lat, lon, grid):
    assert MgrsProjector.grid_of(lat, lon) == grid
    p = MgrsProjector.for_grid(grid)
    x, y = p.project(lat, lon)
    assert 0 <= x < 100_000 and 0 <= y < 100_000
    la, lo = p.unproject(x, y)
    assert abs(la - lat) < 1e-9 and abs(lo - lon) < 1e-9


def test_mgrs_matches_utm():
    # UTM 51N of the origin of a map whose OpenDRIVE declared (215743.6, 2546179) for it, 0.4 m off a proper inverse
    e, n = MgrsProjector.for_grid("51QTF").utm(22.9997300995886, 120.227025902544)
    assert abs(e - 215743.196) < 0.01 and abs(n - 2546178.996) < 0.01


def write_map(d, proj="TransverseMercator"):
    """one lanelet running east (left bound north of the right one) whose traffic light refers to a 0.14 m stub
    drawn the wrong way (west), and an untyped ref_line: crdesigner's output in miniature"""
    d.mkdir(parents=True, exist_ok=True)
    (d / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin:\n  latitude: 0.0\n  longitude: 0.0\n"
        "scale_factor: 1.0\n")
    pr = Projector(0.0, 0.0, 1.0)
    pts = {1: (0, 2), 2: (20, 2), 3: (0, -2), 4: (20, -2), 5: (20, 2), 6: (20, -2), 7: (22.07, 3), 8: (21.93, 3)}
    osm = ET.Element("osm", version="0.6")
    for i, (x, y) in pts.items():
        lat, lon = pr.unproject(x, y)
        ET.SubElement(osm, "node", id=str(i), lat=repr(lat), lon=repr(lon))
    for wid, refs, tags in ((10, (1, 2), {}), (11, (3, 4), {}), (12, (5, 6), {}),
                            (13, (7, 8), {"type": "traffic_light", "subtype": "red_yellow_green"})):
        w = ET.SubElement(osm, "way", id=str(wid))
        for r in refs:
            ET.SubElement(w, "nd", ref=str(r))
        for k, v in tags.items():
            ET.SubElement(w, "tag", k=k, v=v)
    reg = ET.SubElement(osm, "relation", id="20")
    ET.SubElement(reg, "member", type="way", ref="12", role="ref_line")
    ET.SubElement(reg, "member", type="way", ref="13", role="refers")
    for k, v in (("type", "regulatory_element"), ("subtype", "traffic_light")):
        ET.SubElement(reg, "tag", k=k, v=v)
    ll = ET.SubElement(osm, "relation", id="21")
    ET.SubElement(ll, "member", type="way", ref="10", role="left")
    ET.SubElement(ll, "member", type="way", ref="11", role="right")
    ET.SubElement(ll, "member", type="relation", ref="20", role="regulatory_element")
    for k, v in (("type", "lanelet"), ("subtype", "road")):
        ET.SubElement(ll, "tag", k=k, v=v)
    ET.ElementTree(osm).write(d / "lanelet2_map.osm", encoding="UTF-8", xml_declaration=True)
    return d / "lanelet2_map.osm"


def test_stop_lines_types_the_bare_ref_line(tmp_path):
    p = write_map(tmp_path / "m")
    assert stop_lines.repair(p) == 1
    w = next(w for w in ET.parse(p).getroot().iter("way") if w.get("id") == "12")
    assert {t.get("k"): t.get("v") for t in w.findall("tag")} == {"type": "stop_line"}
    assert stop_lines.repair(p) == 0  # idempotent


def test_light_facing_rebuilds_a_stub_to_face_its_lane(tmp_path):
    p = write_map(tmp_path / "m")
    rebuilt, turned, ambiguous, split = light_facing.repair(p)
    assert rebuilt == ["13"] and not turned and not ambiguous and not split
    root = ET.parse(p).getroot()
    pr = Projector.for_map(p)
    xy = {n.get("id"): pr.project(float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    a, b = xy["7"], xy["8"]
    # the lane's edge at the stop line runs left (north) to right (south): the light must run that way, 0.8 m long
    assert b[1] < a[1] and abs(math.dist(a, b) - light_facing.BAR_LEN) < 1e-6
    centre = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    assert math.dist(centre, (22.0, 3.0)) < 1e-6
    assert light_facing.repair(p)[:2] == ([], [])  # idempotent


def test_town_georeference_is_the_inertial_frame(tmp_path):
    pytest.importorskip("pyproj")
    from carla_map_gen.georef import GeoRef
    x = tmp_path / "t.xodr"
    x.write_text('<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 '
                 '+datum=WGS84]]></geoReference></header></OpenDRIVE>')
    g = GeoRef(x)
    proj, info, shift = g.projector((100.0, -200.0))
    assert g.is_town and shift == (0.0, 0.0) and "TransverseMercator" in info and "scale_factor: 1.0" in info


def test_utm_georeference_is_mgrs_shifted(tmp_path):
    pytest.importorskip("pyproj")
    from carla_map_gen.georef import GeoRef
    x = tmp_path / "u.xodr"
    x.write_text('<OpenDRIVE><header><geoReference><![CDATA[+proj=utm +zone=51 +datum=WGS84 +units=m +no_defs]]>'
                 '</geoReference><offset x="215743.6" y="2546179.0" z="0" hdg="0"/></header></OpenDRIVE>')
    proj, info, shift = GeoRef(x).projector((0.0, 0.0))
    assert "mgrs_grid: 51QTF" in info
    assert abs(shift[0] - 15743.6) < 1e-3 and abs(shift[1] - 46179.0) < 1e-3


def test_a_rotated_grid_is_refused(tmp_path):
    pytest.importorskip("pyproj")
    from carla_map_gen.georef import GeoRef
    x = tmp_path / "r.xodr"  # a transverse Mercator about the map's own meridian with a false origin: rotated vs UTM
    x.write_text('<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=22.99973 +lon_0=120.227026 +k=1 '
                 '+x_0=215743.6 +y_0=2546179 +datum=WGS84 +units=m +no_defs]]></geoReference></header></OpenDRIVE>')
    with pytest.raises(SystemExit, match="not UTM's"):
        GeoRef(x).projector((0.0, 0.0))
