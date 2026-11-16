"""streetlook.py: register mast types to lamp meshes, regional furniture, fallbacks, roof clutter and the prop
manifest / catalogue entries (no network, no CARLA)."""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from coupledcarla import scenery, streetlook

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def cat10():
    return scenery.Catalog("1.0")


def test_parse_mast():
    p = streetlook.parse_mast
    assert p("AL 3,5 CONISCH NIEUW") == ("AL", 3.5, None, 0.0)
    assert p("ST 9 STANDAARD VZ", "ST ENKEL 1,50 M LOS VZ")[:3] == ("ST", 9.0, "single")
    assert p("ST 9 STANDAARD VZ", "ST ENKEL 1,50 M LOS VZ")[3] == pytest.approx(1.5)
    assert p("ST 11 M.V.U. 0,44/0,38 DUB.A/S VZ", "ST DUBBEL 1,50 M L. MET TREKSTANG VZ")[2] == "double"
    assert p("AL 6 NEDAL V.061PT03.6E.000", "AL OPZETSTUK 0,20 M")[2] == "top"
    assert p("BB IN/OP/AAN WAND/GEVEL")[0] == "BB" and p("BB IN/OP/AAN WAND/GEVEL")[1] is None
    assert p("Mast met trap")[0] == "TRAP"
    assert p("GY 3,5 (met deur)")[:2] == ("GY", 3.5)
    assert p("ST 12 V-100.T.609 VZ", "ST ENKEL 100 X 162 VZ 1,5M")[3] == pytest.approx(1.5)
    assert p("", "ST WANDARM OPZET 450X200 VZ")[2] == "wall"
    assert p(None) == (None, None, None, 0.0)


def test_lamps_by_register_type(cat10):
    c = streetlook.Chooser(cat10, "NLD", (51.43, 5.47, 51.45, 5.49))
    lamp = c.lamp
    assert lamp({"ehv:mast": "AL 3,5 CONISCH NIEUW"}, True)[0] == "SM_CC_NL_LampPostTop4"
    assert lamp({"ehv:mast": "AL 3,5 CONISCH NIEUW"}, True)[1] == pytest.approx(0.875)
    assert lamp({"ehv:mast": "AL 6 NEDAL V.061PT03.6E.000", "ehv:arm": "AL OPZETSTUK 0,20 M"}, True)[0] == \
        "SM_CC_NL_LampTopLed6"
    m, s, arm = lamp({"ehv:mast": "ST 9 STANDAARD VZ", "ehv:arm": "ST ENKEL 1,50 M LOS VZ"}, True)
    assert (m, s, arm) == ("SM_CC_NL_LampArm9", 1.0, True)
    assert lamp({"ehv:mast": "ST 15 CONISCH VZ"}, True)[0] == "SM_CC_NL_LampArm12"
    assert lamp({"ehv:mast": "ST 11 M.V.U. 0,44/0,38 DUB.A/S VZ", "ehv:arm": "ST DUBBEL 1,50 M"}, True)[0] == \
        "SM_CC_NL_LampDouble10"
    assert lamp({"ehv:mast": "GY 3,5 (met deur)"}, True)[0] == "SM_CC_NL_LampHeritage4"
    assert lamp({"ehv:mast": "BB IN/OP/AAN WAND/GEVEL"}, True) == "skip"
    assert lamp({"support": "wall_mounted"}, True) == "skip"
    assert lamp({}, True)[0] == "SM_CC_NL_LampTopLed6" and lamp({}, False)[0] == "SM_CC_NL_LampPostTop4"
    # scale stays within the limits (a 30 m mast does not become a 40 m lamp)
    assert lamp({"height": "25"}, True)[1] == streetlook.SCALE_LIMITS[1]
    assert c.counts["lamp_no_mast"] == 2


def test_regions_and_catalogue_versions(cat10):
    assert streetlook.region("NLD") == "NL" and streetlook.region("GBR") == "UK" and streetlook.region("USA") is None
    us = streetlook.Chooser(cat10, "USA")
    assert us.lamp({"ehv:mast": "AL 4"}, True) is None and us.furniture("amenity", "bench", {}) is None
    old = streetlook.Chooser(scenery.Catalog("0.9.16"), "NLD")      # 0.9.16 has none of our meshes: CARLA's props
    assert old.lamp({"ehv:mast": "AL 4"}, True) is None
    assert old.furniture("barrier", "bollard", {}) is None
    uk = streetlook.Chooser(cat10, "GBR")
    assert uk.lamp({}, True)[0] == "SM_CC_UK_LampColumn8" and uk.lamp({}, False) is None
    assert uk.furniture("amenity", "waste_basket", {}) == "SM_CC_UK_BinCast"
    assert streetlook.Chooser(cat10, "JPN").lamp({}, True)[0] == "SM_CC_JP_LampGooseneck8"


def test_dutch_furniture(cat10):
    c = streetlook.Chooser(cat10, "NLD", (51.43, 5.47, 51.45, 5.49))
    f = c.furniture
    assert f("amenity", "bench", {"ehv:model": "Model Eindhoven", "colour": "Donker bruin"}) == \
        "SM_CC_NL_BenchEindhoven_DarkBrown"
    assert f("amenity", "bench", {"colour": "Licht bruin"}) == "SM_CC_NL_BenchEindhoven_LightBrown"
    assert f("amenity", "bench", {"ehv:model": "Picknickset"}) is None
    assert f("amenity", "waste_basket", {"ehv:model": "Bammens Prestige", "colour": "Grijs"}) == \
        "SM_CC_NL_BinRound_Grey"
    assert f("amenity", "waste_basket", {"ehv:model": "Bammens Prestige", "colour": "Groen"}) == \
        "SM_CC_NL_BinRound_Green"
    assert f("amenity", "waste_basket", {"ehv:model": "Grijsen Constructo"}) == "SM_CC_NL_BinSquare_Grey"
    assert f("barrier", "bollard", {}) == "SM_CC_NL_Bollard"
    assert f("amenity", "bicycle_parking", {"capacity": "2"}) == "SM_CC_NL_Nietje_x1"
    assert f("amenity", "bicycle_parking", {"capacity": "40"}) == "SM_CC_NL_Nietje_x6"
    assert f("amenity", "post_box", {}) is None
    ams = streetlook.Chooser(cat10, "NLD", (52.36, 4.88, 52.38, 4.90))
    assert ams.furniture("barrier", "bollard", {}) == "SM_CC_NL_Amsterdammertje"
    # OSM bins without colour: green and grey in about the register's mix, the same each build
    picks = [f("amenity", "waste_basket", {}, uid=i) for i in range(400)]
    grey = picks.count("SM_CC_NL_BinRound_Grey") / len(picks)
    assert 0.25 < grey < 0.5 and picks == [f("amenity", "waste_basket", {}, uid=i) for i in range(400)]


def test_fallbacks_are_stock_meshes(cat10):
    names = [n for n in streetlook.props() if n in cat10]
    assert len(names) == len(streetlook.props()) >= 39
    fb = streetlook.fallbacks(names, cat10)
    assert fb[cat10.path("SM_CC_NL_LampArm9")] == cat10.path("SM_StreetLight06_A")
    assert all(v.startswith("/Game/Carla/") for v in fb.values())
    # markings, decals, roofs and signal heads have no stock stand-in
    assert cat10.path("SM_CC_NL_SharkTeeth_4") not in fb and cat10.path("SM_CC_Any_RoofAC") not in fb


def test_manifest_matches_catalogue_and_generator(cat10):
    sys.path.insert(0, str(ROOT / "tools"))
    sp = pytest.importorskip("street_props")
    data = json.loads((ROOT / "coupledcarla" / "data" / "street_props.json").read_text())
    specs = sp.specs()
    assert len(specs) == len(data["meshes"])
    for name, region, folder, role, make, fallback, note in specs:
        n = sp.mesh_name(region, name)
        m, _ = make()
        ext, off = sp.unreal_bounds(m)
        spec = data["meshes"][n]
        assert spec["ext"] == ext and spec["off"] == off, n       # regenerate with tools/street_props.py --data
        assert spec["path"] == cat10.path(n)
        L, W, H = cat10.size(n)
        assert H == pytest.approx(2 * ext[2])
        assert all(slot in sp.PALETTE for slot in spec["slots"]), n
        assert folder in ("Pole", "Static", "TrafficLight", "RoadLine", "Road", "Building")     # tagger labels
    # the conventions the placement relies on: lamp arms towards +x, bench seat and bin slot towards Unreal -y,
    # signal lenses towards Unreal +y, shark-teeth tips towards +x, everything standing on z = 0
    lamp = data["meshes"]["SM_CC_NL_LampArm9"]
    assert lamp["off"][0] > 0.5 and lamp["light"][0] > 1.5
    head = data["meshes"]["SM_CC_NL_SignalCar"]
    assert all(c[1] > 0 for c in head["lenses"]) and head["off"][1] > 0
    for n, spec in data["meshes"].items():
        assert spec["off"][2] - spec["ext"][2] == pytest.approx(0.0, abs=0.06) or "Signal" in n, n


class FakePlacer:
    def __init__(self):
        self.items, self.bases, self.counts = [], [], {}

    def add(self, kind, mesh, cx, cy, yaw, scale, sink=0.0, tilt=None, z=None, **kw):
        self.items.append((kind, mesh, cx, cy, yaw, scale, z))


def test_roof_clutter_inside_flat_measured_roofs(cat10):
    pytest.importorskip("shapely")
    from shapely.geometry import Point, Polygon
    pl = FakePlacer()
    office = Polygon([(0, 0), (40, 0), (40, 25), (0, 25)])
    blds = [{"poly": office, "h": 18.0, "z": 2.0, "cls": "office", "src": "3dbag", "roof": "flat"},
            {"poly": Polygon([(100, 0), (140, 0), (140, 25), (100, 25)]), "h": 18.0, "z": 0.0, "cls": "office",
             "src": "osm", "roof": "flat"},                                  # an OSM guess: no clutter
            {"poly": Polygon([(200, 0), (240, 0), (240, 25), (200, 25)]), "h": 9.0, "z": 0.0, "cls": "house",
             "src": "3dbag", "roof": "pitched"}]                             # a pitched roof: none
    got = streetlook.roof_clutter(pl, cat10, blds)
    assert got.get("bulkhead") == 1 and got.get("ac", 0) >= 3 and got.get("ac_large") == 1
    inner = office.buffer(-1.0)
    for kind, mesh, x, y, yaw, s, z in pl.items:
        assert inner.contains(Point(x, y)) and z == pytest.approx(20.0)
    # the same placements on every build
    pl2 = FakePlacer()
    streetlook.roof_clutter(pl2, cat10, blds)
    assert pl2.items == pl.items


def test_roof_clutter_needs_the_meshes():
    pytest.importorskip("shapely")
    from shapely.geometry import Polygon
    pl = FakePlacer()
    blds = [{"poly": Polygon([(0, 0), (40, 0), (40, 25), (0, 25)]), "h": 18.0, "z": 0.0, "cls": "office",
             "src": "3dbag", "roof": "flat"}]
    assert streetlook.roof_clutter(pl, scenery.Catalog("0.9.16"), blds) == {} and not pl.items


def test_xodr_lanes(tmp_path):
    x = tmp_path / "m.xodr"
    x.write_text("""<OpenDRIVE><road id="7" length="50"><lanes>
      <laneSection s="0"><left><lane id="1" type="driving"/></left><center><lane id="0" type="none"/></center>
        <right><lane id="-1" type="driving"/><lane id="-2" type="biking"/></right></laneSection>
      <laneSection s="30"><right><lane id="-1" type="driving"/><lane id="-2" type="parking"/></right></laneSection>
    </lanes></road></OpenDRIVE>""")
    assert streetlook.xodr_lanes(x, ("biking",)) == [(7, -2, 0.0, 30.0)]
    assert streetlook.xodr_lanes(x, ("parking",)) == [(7, -2, 30.0, 50.0)]


def test_municipal_cache_version(tmp_path, monkeypatch):
    from coupledcarla import realworld as rw
    path = tmp_path / "municipal.json"
    path.write_text(json.dumps({"elements": [{"tags": {"highway": "street_lamp"}}], "sources": ["eindhoven"]}))
    calls = []

    def fake(bbox):
        calls.append(bbox)
        return [{"type": "node", "id": -1, "lat": 51.44, "lon": 5.47,
                 "tags": {"highway": "street_lamp", "ehv:mast": "AL 4"}}]
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57), fake, "test")])
    got = rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)
    assert calls and got["version"] == rw.MUNICIPAL_VERSION and got["elements"][0]["tags"]["ehv:mast"] == "AL 4"
    calls.clear()
    rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)       # now current: read from the cache
    assert not calls

    def down(bbox):
        raise OSError("portal down")
    path.write_text(json.dumps({"elements": [{"tags": {"natural": "tree"}}], "sources": ["eindhoven"]}))
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57), down, "test")])
    assert rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)["elements"] == [{"tags": {"natural": "tree"}}]


def test_scenery_drops_leaves_out_of_autumn(monkeypatch):
    """Leaf litter (kind "leaves") is spawned only when the trial's season is autumn."""
    data = {"meshes": ["/Game/a.a"], "cell_m": 100.0, "items": [[0, 1.0, 1.0, 0.0, 0.0, 1.0, "leaves"],
                                                                 [0, 2.0, 2.0, 0.0, 0.0, 1.0, "wear"]]}

    class W:
        def get_blueprint_library(self):
            return SimpleNamespace(find=lambda name: None)
    for state, want in (("leaf", 1), ("autumn", 2)):
        sc = scenery.Scenery(None, W(), None, {"radius_m": 0}, log=lambda *a: None,
                             season={"state": state} if state == "autumn" else None, data=data)
        assert sum(len(v) for v in sc.cells.values()) == want


def test_vienna_faces_for_osm_signs(cat10):
    f = streetlook.osm_face
    assert f("give_way", None, "UKR", cat10) == "SM_NL_B6"            # Chernihiv's UA:2.1
    assert f("give_way", None, "USA", cat10) is None and f("give_way", None, "GBR", cat10) is None
    assert f("speed", 50, "UKR", cat10) == "SM_NL_A1_50" and f("speed", 50, "GBR", cat10) == "SM_NL_A1_50"
    assert f("speed", 50, "USA", cat10) is None and f("speed", 55, "FRA", cat10) is None   # no such face
    assert f("give_way", None, "UKR", scenery.Catalog("0.9.16")) is None
    assert f("stop", None, "UKR", cat10) is None and f("give_way", None, None, cat10) is None
