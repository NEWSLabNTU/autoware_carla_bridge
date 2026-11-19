"""Unit tests for carla_map_gen.projection.py (Autoware's map frame from lat/lon) and
carla_map_gen.repairs.local_coords.py (which writes it into local_x/local_y). No ROS:

    python3 -m pytest src/carla_map_gen/test/test_projection.py
"""

import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # carla_map_gen
from carla_map_gen import projection as ap  # noqa: E402
from carla_map_gen.repairs import local_coords as repair  # noqa: E402

try:
    import pyproj
except ImportError:
    pyproj = None

PROJECTOR_YAML = """projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
"""

# Town01 node 1 as the pack ships it (spherical local tags), one node with none, one
# self-closing node
MAP = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="test">
  <node id="1" action="modify" visible="true" version="1" lat="-1.805613721080238e-07" lon="0.003454829852879983">
    <tag k="local_x" v="384.5899"/>
    <tag k="local_y" v="-0.0201"/>
    <tag k="ele" v="0"/>
  </node>
  <node id="2" lat="-0.003" lon="0.003">
    <tag k="ele" v="5"/>
  </node>
  <node id="3" lat="0.001" lon="-0.002"/>
  <way id="10">
    <nd ref="1"/>
    <nd ref="2"/>
  </way>
</osm>
"""


def tags(text, node_id):
    import xml.etree.ElementTree as ET
    for n in ET.fromstring(text).iter("node"):
        if n.get("id") == node_id:
            return {t.get("k"): t.get("v") for t in n.iter("tag")}


class Projection(unittest.TestCase):
    @unittest.skipIf(pyproj is None, "no pyproj")
    def test_matches_pyproj_transverse_mercator(self):
        rng = random.Random(0)
        for lat0, lon0, k0 in ((0.0, 0.0, 0.9996), (35.0, 139.0, 0.9996), (24.8, 121.0, 1.0)):
            p = ap.Projector(lat0, lon0, k0)
            ref = pyproj.Transformer.from_crs(
                "EPSG:4326", f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k={k0} +ellps=WGS84",
                always_xy=True)
            for _ in range(200):
                lat, lon = lat0 + rng.uniform(-0.03, 0.03), lon0 + rng.uniform(-0.03, 0.03)
                x, y = p.project(lat, lon)
                rx, ry = ref.transform(lon, lat)
                self.assertLess(math.hypot(x - rx, y - ry), 1e-3, (lat0, lon0, lat, lon))

    def test_unproject_inverts_project(self):
        p = ap.Projector()
        for x, y in ((0.0, 0.0), (384.4, -0.02), (-250.0, 330.0), (2000.0, -2000.0)):
            lat, lon = p.unproject(x, y)
            px, py = p.project(lat, lon)
            self.assertLess(math.hypot(px - x, py - y), 1e-6)

    def test_town01_south_road_is_0_7_percent_short_of_the_spherical_tags(self):
        # the pack's local_y for lat -0.003 is R * radians(lat) = -333.95; Autoware's is
        # 2.37 m nearer the origin
        _, y = ap.Projector().project(-0.003, 0.0)
        self.assertAlmostEqual(y, -331.59, places=2)
        self.assertAlmostEqual(-math.radians(0.003) * 6378137.0 - y, -2.37, places=2)

    def test_for_map_reads_the_yaml_beside_the_map(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "map_projector_info.yaml").write_text(
                PROJECTOR_YAML.replace("latitude: 0.0", "latitude: 35.5")
                + "scale_factor: 1.0\n")
            p = ap.Projector.for_map(Path(d) / "lanelet2_map.osm")
            self.assertEqual((p.lat0, p.lon0, p.k0), (35.5, 0.0, 1.0))
            (Path(d) / "map_projector_info.yaml").write_text("projector_type: Local\n")
            with self.assertRaises(SystemExit):
                ap.Projector.for_map(d)


class Repair(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        d = Path(self.dir.name)
        (d / "map_projector_info.yaml").write_text(PROJECTOR_YAML)
        self.osm = d / "lanelet2_map.osm"
        self.osm.write_text(MAP)

    def tearDown(self):
        self.dir.cleanup()

    def test_every_node_gets_autowares_local_coordinates(self):
        self.assertEqual(repair.repair(self.osm), 3)
        text = self.osm.read_text()
        p = ap.Projector()
        for nid, lat, lon in (("1", -1.805613721080238e-07, 0.003454829852879983),
                              ("2", -0.003, 0.003), ("3", 0.001, -0.002)):
            t = tags(text, nid)
            x, y = p.project(lat, lon)
            self.assertEqual((t["local_x"], t["local_y"]), ("%.4f" % x, "%.4f" % y), nid)
        self.assertEqual(tags(text, "1")["local_x"], "384.4361")
        self.assertEqual(tags(text, "2")["ele"], "5")
        # one tag per line in the pack's layout, the new tags first, the way untouched, and the first edit kept
        # the original (issue 067: through ElementTree, as the other repairs)
        self.assertIn('  <way id="10">\n    <nd ref="1" />', text)
        self.assertIn('    <tag k="local_x" v="384.4361" />\n    <tag k="local_y" v="-0.0200" />\n'
                      '    <tag k="ele" v="0" />', text)
        self.assertIn('    <tag k="local_x" v="333.8249" />\n    <tag k="local_y" v="-331.5901" />\n'
                      '    <tag k="ele" v="5" />', text)
        self.assertIn('  <node id="3" lat="0.001" lon="-0.002">\n    <tag k="local_x" v="-222.5499" />\n'
                      '    <tag k="local_y" v="110.5300" />\n  </node>', text)
        self.assertEqual((self.osm.parent / "lanelet2_map.osm.orig").read_text(), MAP)

    def test_a_map_written_by_elementtree_keeps_one_pair(self):
        """issue 067: on a map an earlier repair wrote back (`" />`), and with a stale duplicate pair, one
        local_x/local_y per node remains, at the projector's values"""
        self.osm.write_text(MAP.replace('"/>', '" />').replace(
            '<tag k="ele" v="5"/>', '<tag k="local_x" v="1.0" />\n    <tag k="local_y" v="2.0" />\n    <tag k="ele" v="5" />'))
        repair.repair(self.osm)
        text = self.osm.read_text()
        p = ap.Projector()
        for nid, lat, lon in (("1", -1.805613721080238e-07, 0.003454829852879983),
                              ("2", -0.003, 0.003), ("3", 0.001, -0.002)):
            block = text.split(f'<node id="{nid}"')[1].split("</node>")[0]
            self.assertEqual(block.count('k="local_x"'), 1, nid)
            self.assertEqual(block.count('k="local_y"'), 1, nid)
            x, y = p.project(lat, lon)
            self.assertEqual(tags(text, nid)["local_x"], "%.4f" % x)

    def test_idempotent(self):
        repair.repair(self.osm)
        once = self.osm.read_text()
        self.assertEqual(repair.repair(self.osm), 0)
        self.assertEqual(self.osm.read_text(), once)


if __name__ == "__main__":
    unittest.main()
