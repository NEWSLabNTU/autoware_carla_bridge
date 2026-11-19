"""The OpenDRIVE file's georeference, carried into the Lanelet2 map (roadmap 087).

An OpenDRIVE file's inertial (x, y) plus its header's `<offset>` are coordinates in the CRS its
`<geoReference>` names (a PROJ string). carla_map_gen used to write every map with Autoware's
TransverseMercator projector at 0,0, which is right for CARLA's towns only (`+proj=tmerc +lat_0=0
+lon_0=0`); a real place came out with meaningless lat/lon. Here:

- the file is converted with its georeference replaced by the towns' (so crdesigner's lat/lon are a
  TransverseMercator at 0,0 of the inertial frame, which `Projector` inverts exactly), and
- every node's lat/lon is then recomputed from the inertial frame through the file's own CRS (pyproj),
  and its local_x/local_y written in the map frame the projector below defines.

The projector:

- **TransverseMercator at the CRS origin** when the CRS is a transverse Mercator with no false
  origin and the file has no offset (the towns: 0,0, k 1; an OSM2ODR import): the map frame is the
  inertial frame, as the bridges assume (CARLA's world is it with y negated).
- **MGRS** otherwise, when the CRS is UTM or a transverse Mercator (a false origin, an offset): the
  square of the map's centre. The map frame is then the inertial frame shifted: `map = inertial +
  shift` on the square's grid, which only holds as a pure shift when the CRS is UTM (a TM about another
  meridian is rotated against the grid; such a file is refused). The shift is written to
  `map_frame.yaml`, since a bridge that maps CARLA's frame onto the map's must apply it.
"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from carla_map_gen.projection import MgrsProjector, Projector

TOWN_PROJ = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"


def proj_params(proj):
    """{'proj': 'tmerc', 'lat_0': 0.0, ...} of a PROJ string"""
    out = {}
    for tok in proj.split():
        if not tok.startswith("+"):
            continue
        k, _, v = tok[1:].partition("=")
        try:
            out[k] = float(v) if v else True
        except ValueError:
            out[k] = v
    return out


class GeoRef:
    def __init__(self, xodr):
        root = ET.parse(xodr).getroot()
        header = root.find("header")
        g = header.find("geoReference") if header is not None else None
        self.proj = (g.text or "").strip() if g is not None else ""
        off = header.find("offset") if header is not None else None
        self.offset = (float(off.get("x", 0)), float(off.get("y", 0))) if off is not None else (0.0, 0.0)
        if off is not None and abs(float(off.get("hdg", 0) or 0)) > 1e-9:
            raise SystemExit(f"{xodr}: <offset hdg> is not 0; a rotated offset is not handled")
        self.p = proj_params(self.proj) if self.proj else {}

    @property
    def is_town(self):
        """no georeference, or a transverse Mercator at 0,0 with no false origin and no offset"""
        if not self.proj:
            return True
        p = self.p
        return (p.get("proj") == "tmerc" and not any(p.get(k, 0.0) for k in ("lat_0", "lon_0", "x_0", "y_0"))
                and self.offset == (0.0, 0.0))

    @property
    def is_local_tm(self):
        p = self.p
        return (p.get("proj") == "tmerc" and not p.get("x_0", 0.0) and not p.get("y_0", 0.0)
                and self.offset == (0.0, 0.0))

    def to_lonlat(self):
        """a function (x, y) inertial -> (lon, lat) through the file's CRS"""
        from pyproj import CRS, Transformer
        crs = CRS.from_proj4(self.proj)
        t = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
        ox, oy = self.offset
        return lambda x, y: t.transform(x + ox, y + oy)

    def projector(self, centre_xy):
        """(projector, map_projector_info text, shift (dx, dy) map = inertial + shift)"""
        if self.is_town or self.is_local_tm:
            p = self.p
            lat0, lon0, k = (p.get("lat_0", 0.0), p.get("lon_0", 0.0), p.get("k", p.get("k_0", 1.0))) if self.proj \
                else (0.0, 0.0, 1.0)
            text = (f"projector_type: TransverseMercator\nvertical_datum: WGS84\nmap_origin:\n"
                    f"  latitude: {lat0!r}\n  longitude: {lon0!r}\nscale_factor: {k!r}\n")
            return Projector(lat0, lon0, k), text, (0.0, 0.0)
        if self.p.get("proj") not in ("utm", "tmerc"):
            raise SystemExit(f"geoReference {self.proj!r}: only tmerc and utm are handled")
        lon, lat = self.to_lonlat()(*centre_xy)
        grid = MgrsProjector.grid_of(lat, lon)
        mp = MgrsProjector.for_grid(grid)
        # the shift, and a check that it is one: the CRS's grid must be UTM's, not rotated against it
        f = self.to_lonlat()
        pts = [(centre_xy[0] + dx, centre_xy[1] + dy) for dx, dy in ((0, 0), (500, 0), (0, 500), (-500, -500))]
        shifts = [tuple(m - i for m, i in zip(mp.project(*reversed(f(x, y))), (x, y))) for x, y in pts]
        spread = max(math.dist(shifts[0], s) for s in shifts[1:])
        if spread > 0.05:
            raise SystemExit(f"geoReference {self.proj!r}: its grid is not UTM's ({spread:.2f} m apart over 500 m), so "
                             "the map frame would not be a shift of the inertial frame; write the file in UTM (an "
                             "<offset> to the origin) or in a local transverse Mercator")
        text = f"projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {grid}\n"
        return mp, text, shifts[0]


def rewrite_town_proj(src, dst):
    """the file with its georeference replaced by the towns' and no offset (what crdesigner converts)"""
    text = Path(src).read_text(encoding="utf-8")
    text = re.sub(r"<geoReference>.*?</geoReference>", f"<geoReference><![CDATA[{TOWN_PROJ}]]></geoReference>",
                  text, count=1, flags=re.S)
    text = re.sub(r"<offset\b[^>]*/>", "", text, count=1)
    Path(dst).write_text(text, encoding="utf-8")


def relocate(osm, georef, out_dir):
    """recompute every node's lat/lon from its inertial position through the file's CRS, write local_x/local_y
    in the map frame, map_projector_info.yaml and (when the frame is shifted) map_frame.yaml"""
    tree = ET.parse(osm)
    nodes = list(tree.getroot().iter("node"))
    town = Projector(0.0, 0.0, 1.0)  # what crdesigner projected the inertial frame with
    xy = [town.project(float(n.get("lat")), float(n.get("lon"))) for n in nodes]
    if not xy:
        raise SystemExit(f"{osm}: no nodes")
    centre = (sum(p[0] for p in xy) / len(xy), sum(p[1] for p in xy) / len(xy))
    proj, info, shift = georef.projector(centre)
    if georef.is_town:
        f = None  # lat/lon are already the towns' frame
    else:
        f = georef.to_lonlat()
    for n, (x, y) in zip(nodes, xy):
        if f is not None:
            lon, lat = f(x, y)
            n.set("lat", repr(lat))
            n.set("lon", repr(lon))
        else:
            lat, lon = float(n.get("lat")), float(n.get("lon"))
        lx, ly = proj.project(lat, lon)
        tags = {t.get("k"): t for t in n.findall("tag")}
        for k, v in (("local_x", lx), ("local_y", ly)):
            if k in tags:
                tags[k].set("v", repr(v))
            else:
                ET.SubElement(n, "tag", k=k, v=repr(v))
    tree.write(osm, encoding="UTF-8", xml_declaration=True)
    out_dir = Path(out_dir)
    (out_dir / "map_projector_info.yaml").write_text(info)
    frame = out_dir / "map_frame.yaml"
    if shift != (0.0, 0.0):
        frame.write_text(
            "# carla_map_gen (roadmap 087): the map frame against the OpenDRIVE's inertial frame, map = inertial + shift.\n"
            "# CARLA's world is the inertial frame with y negated; a bridge must apply this shift.\n"
            f"shift_x: {shift[0]!r}\nshift_y: {shift[1]!r}\ngeoreference: {georef.proj!r}\n"
            f"offset: [{georef.offset[0]!r}, {georef.offset[1]!r}]\n")
    elif frame.exists():
        frame.unlink()
    return info, shift


def shift_xodr(src, dst, shift):
    """the file with every reference-line geometry moved by shift (its inertial frame made the map frame), for the
    repairs that match lanelets against the OpenDRIVE"""
    tree = ET.parse(src)
    for g in tree.getroot().iter("geometry"):
        g.set("x", repr(float(g.get("x")) + shift[0]))
        g.set("y", repr(float(g.get("y")) + shift[1]))
    tree.write(dst, encoding="utf-8", xml_declaration=True)
