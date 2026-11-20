"""Unit tests for carla_map_gen.repairs.far_traffic_lights (roadmap 031; scensim issue 073).

A synthetic map in local_x / local_y metres: three stop lines, each with a lanelet referring to
its traffic-light element, and lights near and far from them. No ROS:

    python3 -m pytest src/carla_map_gen/test/test_repair_far_traffic_lights.py
"""

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # carla_map_gen
from carla_map_gen.repairs import far_traffic_lights as ftl  # noqa: E402


class _Map:
    def __init__(self):
        self.next_id = 1
        self.nodes, self.ways, self.rels = [], [], []

    def _id(self):
        self.next_id += 1
        return str(self.next_id)

    def way(self, pts, kind):
        ids = []
        for x, y in pts:
            nid = self._id()
            self.nodes.append(f'<node id="{nid}" lat="0" lon="0"><tag k="local_x" v="{x}"/>'
                              f'<tag k="local_y" v="{y}"/></node>')
            ids.append(nid)
        wid = self._id()
        nds = "".join(f'<nd ref="{i}"/>' for i in ids)
        self.ways.append(f'<way id="{wid}">{nds}<tag k="type" v="{kind}"/></way>')
        return wid

    def light(self, x, y):
        return self.way([(x, y), (x + 0.8, y)], "traffic_light")

    def element(self, stop_line, lights):
        rid = self._id()
        members = f'<member type="way" ref="{stop_line}" role="ref_line"/>' + "".join(
            f'<member type="way" ref="{w}" role="refers"/>' for w in lights)
        self.rels.append(f'<relation id="{rid}">{members}<tag k="type" v="regulatory_element"/>'
                         f'<tag k="subtype" v="traffic_light"/></relation>')
        return rid

    def lanelet(self, element):
        left, right = self.way([(0, 1), (10, 1)], "line_thin"), self.way([(0, -1), (10, -1)], "line_thin")
        rid = self._id()
        self.rels.append(f'<relation id="{rid}"><member type="way" ref="{left}" role="left"/>'
                         f'<member type="way" ref="{right}" role="right"/>'
                         f'<member type="relation" ref="{element}" role="regulatory_element"/>'
                         f'<tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation>')
        return rid

    def write(self, path):
        Path(path).write_text("<?xml version='1.0' encoding='UTF-8'?>\n<osm version='0.6'>"
                              + "".join(self.nodes + self.ways + self.rels) + "</osm>\n")


def _refs(root, rid, role):
    rel = next(r for r in root.iter("relation") if r.get("id") == rid)
    return [m.get("ref") for m in rel.findall("member") if m.get("role") == role]


class FarTrafficLightsTest(unittest.TestCase):
    def setUp(self):
        m = _Map()
        # A: its own light 15 m from its stop line (near)
        self.stop_a = m.way([(0, 0), (0, 3)], "stop_line")
        self.light_a = m.light(15, 0)
        # B: a junction 300 m away, its own element and light
        self.stop_b = m.way([(300, 0), (300, 3)], "stop_line")
        self.light_b = m.light(310, 0)
        # far_only: a light 330 m from its stop line and nothing else
        self.stop_c = m.way([(0, 100), (0, 103)], "stop_line")
        self.light_c = m.light(330, 100)
        # an element at A mixing A's light, B's light (B refers to it too) and a light only it refers to
        self.light_lonely = m.light(320, 5)
        self.el_a = m.element(self.stop_a, [self.light_a, self.light_b, self.light_lonely])
        self.el_b = m.element(self.stop_b, [self.light_b])
        self.el_c = m.element(self.stop_c, [self.light_c])
        self.ll_a, self.ll_b, self.ll_c = m.lanelet(self.el_a), m.lanelet(self.el_b), m.lanelet(self.el_c)
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "lanelet2_map.osm"
        m.write(self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_far_only_element_is_unlinked_and_kept(self):
        unlinked, dropped, _, _, _ = ftl.repair(self.path)
        self.assertEqual(unlinked, [self.el_c])
        self.assertEqual(dropped, 1)
        root = ET.parse(self.path).getroot()
        self.assertEqual(_refs(root, self.ll_c, "regulatory_element"), [])
        # the element stays, with its light, so its id is still valid for the signal table
        self.assertEqual(_refs(root, self.el_c, "refers"), [self.light_c])

    def test_a_mixed_element_keeps_only_its_near_lights(self):
        _, _, trimmed, lights, _ = ftl.repair(self.path)
        self.assertEqual(trimmed, [self.el_a])
        self.assertEqual(lights, 2)
        root = ET.parse(self.path).getroot()
        self.assertEqual(_refs(root, self.el_a, "refers"), [self.light_a])
        self.assertEqual(_refs(root, self.ll_a, "regulatory_element"), [self.el_a])

    def test_near_elements_are_untouched(self):
        ftl.repair(self.path)
        root = ET.parse(self.path).getroot()
        self.assertEqual(_refs(root, self.el_b, "refers"), [self.light_b])
        self.assertEqual(_refs(root, self.ll_b, "regulatory_element"), [self.el_b])

    def test_a_light_no_element_refers_to_any_more_is_removed_with_its_nodes(self):
        before = ET.parse(self.path).getroot()
        lonely_nodes = {nd.get("ref") for w in before.iter("way") if w.get("id") == self.light_lonely
                        for nd in w.iter("nd")}
        _, _, _, _, orphaned = ftl.repair(self.path)
        self.assertEqual(orphaned, [self.light_lonely])
        root = ET.parse(self.path).getroot()
        ways = {w.get("id") for w in root.iter("way")}
        self.assertNotIn(self.light_lonely, ways)
        self.assertIn(self.light_b, ways, "B still refers to its light")
        self.assertFalse(lonely_nodes & {n.get("id") for n in root.iter("node")})

    def test_a_second_run_finds_nothing(self):
        ftl.repair(self.path)
        text = self.path.read_text()
        self.assertEqual(ftl.repair(self.path), ([], 0, [], 0, []))
        self.assertEqual(self.path.read_text(), text)

    def test_a_dry_run_writes_nothing(self):
        text = self.path.read_text()
        unlinked, _, trimmed, _, orphaned = ftl.repair(self.path, dry=True)
        self.assertEqual((unlinked, trimmed, orphaned), ([self.el_c], [self.el_a], [self.light_lonely]))
        self.assertEqual(self.path.read_text(), text)

    def test_main_takes_a_map_directory(self):
        ftl.main([str(self.path.parent)])
        root = ET.parse(self.path).getroot()
        self.assertEqual(_refs(root, self.ll_c, "regulatory_element"), [])


if __name__ == "__main__":
    unittest.main()
