"""The NDW traffic sign register: RVV codes to CARLA meshes, the fetch (monkeypatched, no network), posts,
placement against a fake road, the OSM duplicates and the WKD speed check."""
import json
import math
from collections import defaultdict
from types import SimpleNamespace

import pytest

from coupledcarla import ndwsigns, scenery

C09, C10 = scenery.Catalog("0.9.16"), scenery.Catalog("1.0")


def S(code, k=None, b=90, lat=51.44, lon=5.48, p="L"):
    return {"id": code, "c": code, "k": k, "b": b, "lat": lat, "lon": lon, "p": p, "z": None}


def test_parse_and_shapes():
    assert ndwsigns.parse("G07") == ("G", 7, "")
    assert ndwsigns.parse("G12a") == ("G", 12, "a")
    assert ndwsigns.parse("A1-50") == ("A", 1, "")
    assert ndwsigns.parse("onbekend") is None
    assert ndwsigns.shape_of("B6") == "inv_triangle" and ndwsigns.shape_of("B1") == "diamond"
    assert ndwsigns.shape_of("J24") == "triangle" and ndwsigns.shape_of("E4") == "square"
    assert ndwsigns.shape_of("E1") == "round" and ndwsigns.shape_of("C3") == "rect_h"
    assert ndwsigns.shape_of("G11") == "round" and ndwsigns.shape_of("G13") == "rect_v"
    assert ndwsigns.kind_of(S("A1", "30")) == ("speed", 30)
    assert ndwsigns.kind_of(S("A1-50")) == ("speed", 50)
    assert ndwsigns.kind_of(S("A1")) == ("speed", None)
    assert ndwsigns.kind_of(S("B7"))[0] == "stop" and ndwsigns.kind_of(S("C2"))[0] == "no_entry"


def test_choose_meshes_both_catalogues():
    assert ndwsigns.choose(S("B7"), C09)[::2] == ("SM_stopSign", "real")
    # CARLA's give-way plate says YIELD (the US design): a blank triangle on 1.0, none on 0.9.16 (no blank plates)
    assert ndwsigns.choose(S("B6"), C10, False)[::2] == ("SM_InvertedTriangleShape", "generic")
    assert ndwsigns.choose(S("B6"), C09)[2] == "skipped"
    # CARLA's speed plate is the US "SPEED LIMIT" rectangle: never in the Netherlands (a blank roundel or nothing)
    assert ndwsigns.choose(S("A1", "50"), C09)[2] == "skipped"
    assert ndwsigns.choose(S("A1", "30"), C10, False)[::2] == ("SM_CircleShape", "generic")
    # the posted speed or nothing like it: no 20 km/h plate in CARLA
    assert ndwsigns.choose(S("A1", "20"), C09)[2] == "skipped"
    assert ndwsigns.choose(S("A1", "20"), C10, False)[::2] == ("SM_CircleShape", "generic")
    assert ndwsigns.choose(S("C2"), C09)[2] == "skipped"
    assert ndwsigns.choose(S("C2"), C10, False)[::2] == ("SM_CircleShape", "generic")    # not DO NOT ENTER
    assert ndwsigns.choose(S("E4"), C10, False)[::2] == ("SM_SquareShape", "generic")
    assert ndwsigns.choose(S("J24"), C10, False)[::2] == ("SM_DangerSignShape", "generic")
    assert ndwsigns.choose(S("onbekend"), C10)[2] == "skipped"
    for shape, mesh in ndwsigns.GENERIC[1].items():
        assert mesh in C10, mesh
    cov = ndwsigns.coverage([S("B7"), S("E4"), S("E4"), S("onbekend")], C10, False)
    assert (cov["real"], cov["generic"], cov["skipped"]) == (1, 2, 1)
    assert cov["by_code"]["E4"] == {"generic": 2}


def test_dutch_faces_on_carla_1():
    # CARLA's stop stays; its (US) give-way, speed and no-entry plates and the blank plates give way to faces
    assert ndwsigns.choose(S("B6"), C10)[::2] == ("SM_NL_B6", "face")
    assert ndwsigns.choose(S("C2"), C10)[::2] == ("SM_NL_C2", "face")
    assert ndwsigns.choose(S("A1", "30"), C10)[::2] == ("SM_NL_A1_30", "face")
    assert ndwsigns.choose(S("A1", "20"), C10)[::2] == ("SM_NL_A1_20", "face")
    assert ndwsigns.choose(S("E4"), C10) == ("SM_NL_E4", 0.6, "face")
    assert ndwsigns.choose(S("J24"), C10)[::2] == ("SM_NL_J24", "face")
    assert ndwsigns.choose(S("A4", "15"), C10)[2] == "generic"     # no face with that number: a blank plate
    assert ndwsigns.choose(S("E4"), C09)[2] == "skipped" and ndwsigns.choose(S("A1", "50"), C09)[2] == "skipped"
    zone = dict(S("A1", "30"), z="ZB")
    assert ndwsigns.face_keys(zone) == ["A1-30-ZB", "A1-30"]
    assert ndwsigns.choose(zone, C10)[::2] == ("SM_NL_A1_30_ZB", "face")
    assert ndwsigns.face_keys(dict(S("A2", "30"), z="ZE"))[0] == "A2-30-ZE"
    assert ndwsigns.face_keys(dict(S("E1"), z="ZH")) == ["E1-ZB", "E1", "E1-R"]
    assert ndwsigns.face_keys(S("A1-50")) == ["A1-50"] and ndwsigns.face_keys(S("A1")) == []
    assert ndwsigns.face_keys(S("C17", "11.0")) == ["C17", "C17-R"]
    assert ndwsigns.choose(S("C4"), C10)[::2] == ("SM_NL_C4_R", "face")       # drawn left and right: right
    assert ndwsigns.choose(S("G07"), C10)[::2] == ("SM_NL_G7", "face")
    cov = ndwsigns.coverage([S("B7"), S("E4"), S("G11"), S("onbekend")], C10)
    assert (cov["real"], cov["face"], cov["generic"], cov["skipped"]) == (1, 2, 0, 1)
    # every face is in the 1.0 catalogue with its plate's size, and has a blank plate to fall back to
    for f in ndwsigns.faces().values():
        assert f["mesh"] in C10 and C10.size(f["mesh"]) == C10.size(f["plate"])
    fb = ndwsigns.fallbacks(["SM_NL_E4", "SM_Pole01"], C10)
    assert fb == {C10.path("SM_NL_E4"): C10.path("SM_SquareShape")}
    assert C10.path("SM_NL_E4") == "/Game/CoupledCarla/Signs/NL/Meshes/SM_NL_E4.SM_NL_E4"


class FakeResponse:
    def __init__(self, data):
        self.data = data

    def raise_for_status(self):
        pass

    def json(self):
        return self.data


def _feature(code, lon, lat, **props):
    return {"type": "Feature", "id": code + str(lon), "geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": dict({"rvvCode": code, "status": "PLACED", "bearing": 90, "placement": "L"}, **props)}


def test_fetch_caches_and_keeps_the_box(tmp_path, monkeypatch):
    calls = []

    def get(url, params=None, headers=None, timeout=None):
        calls.append(params)
        return FakeResponse({"features": [_feature("A1", 5.48, 51.44, blackCode="30"),
                                          _feature("B6", 5.60, 51.44)]})     # outside the box
    monkeypatch.setattr(ndwsigns.requests, "get", get)
    path = tmp_path / "ndw_signs.json"
    d = ndwsigns.fetch([51.43, 5.47, 51.45, 5.49], path, {}, log=lambda m: None)
    assert calls[0] == {"bbox": "5.47,51.43,5.49,51.45"}
    assert [s["c"] for s in d["signs"]] == ["A1"] and d["signs"][0]["k"] == "30"
    ndwsigns.fetch([51.43, 5.47, 51.45, 5.49], path, {}, log=lambda m: None)
    assert len(calls) == 1 and json.loads(path.read_text())["signs"][0]["b"] == 90


def test_scenery_falls_back(tmp_path, monkeypatch):
    msgs = []
    mc = {"bbox": [51.43, 5.47, 51.45, 5.49]}

    def fail(*a, **k):
        raise ndwsigns.requests.ConnectionError("offline")
    monkeypatch.setattr(ndwsigns.requests, "get", fail)
    assert scenery._ndw(mc, {}, tmp_path, msgs.append) is None and "OSM signs only" in msgs[0]
    assert scenery._ndw(mc, {"ndw_signs": False}, tmp_path, msgs.append) is None
    assert scenery._ndw({"bbox": [48.85, 2.33, 48.87, 2.36]}, {}, tmp_path, msgs.append) is None   # Paris
    assert len(msgs) == 1


def test_posts():
    signs = [S("B6", lat=0, lon=0), S("D6", lat=0.3, lon=0), S("E1", lat=0, lon=0, b=270), S("C2", lat=5, lon=0)]
    groups = ndwsigns.posts(signs, lambda s: (s["lon"], s["lat"]))
    assert sorted(len(g) for g in groups) == [1, 1, 2]


class FakePlacer:
    def __init__(self, cat, lanes):
        self.cat, self.lanes, self.items, self.counts = cat, lanes, [], defaultdict(int)

    def on_lane(self, x, y, z=0.0):
        return self.lanes(x, y)

    def add(self, kind, mesh, cx, cy, yaw, scale, z=None, **kw):
        self.items.append((kind, mesh, cx, cy, yaw % 360, scale, z))
        self.counts[kind] += 1


class FakeRoads:
    """An east-west road: eastbound (yaw 0) on y > 0 (CARLA's y points south: the right of eastbound traffic)."""
    lefthand = False

    def governed(self, x, y, yaw):
        return 0.0, 1.0, (x, 4.3)


def test_place_stacks_plates_facing_the_traffic():
    cat = C10
    pl = FakePlacer(cat, lambda x, y: abs(y) < 4.0)
    # a give-way and a 30 sign on one post, on the pavement at y = 2 (on a lane: walked off it), read eastwards
    signs = [S("A1", "30", lon=10.0, lat=2.0), S("B6", lon=10.2, lat=2.0), S("onbekend", lon=50, lat=6)]
    placed = ndwsigns.place(pl, cat, signs, lambda s: (s["lon"], s["lat"]), FakeRoads())
    post, post2, top, low = pl.items[:4]
    assert post[1] == post2[1] == cat.m.POST and abs(post[3]) >= 4.0 and post[6] == 1.0
    assert post2[2:4] == post[2:4] and post2[5] == post[5] == 1.0 and post2[6] > post[6]   # a taller post
    assert top[1] == "SM_NL_B6" and low[1] == "SM_NL_A1_30"       # priority sign on top (the Dutch B6, not YIELD)
    assert post[4] == pytest.approx(90.0) and top[4] == pytest.approx(90.0)   # face (+y) to the west: oncoming
    assert top[6] > low[6] and low[6] >= 1.0 + ndwsigns.CLEAR_M - 0.01
    assert top[2] < post[2] and top[3] == post[3]   # plates in front of the post: +y of yaw 90 is -x, the west
    assert pl.counts["sign_ndw"] == 2 and pl.counts["sign_ndw_real"] == 0 and pl.counts["sign_ndw_face"] == 2
    assert pl.counts["sign_ndw_skipped"] == 1
    assert pl.counts["sign_ndw_posts"] == 1 and len(placed) == 2
    assert ndwsigns.near_same(placed, 12.0, 6.0, "give_way") and not ndwsigns.near_same(placed, 12, 6, "stop")
    assert ndwsigns.osm_kind("SM_Stop01", cat.m) == "stop" and ndwsigns.osm_kind("SM_SpeedLimit50", cat.m) == "speed"


# --- Roads against a fake CARLA map: a two-way road along x, lanes -1 (eastbound, y = +1.6) and 1 (westbound) ---

class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class FakeWp:
    def __init__(self, road, lane_id, x):
        self.road, self.lane_id, self.lane_width = road, lane_id, 3.2
        self.lane_type = "Driving"
        self.is_junction = False
        y = 1.6 if lane_id < 0 else -1.6
        if road.oneway:
            y = 0.0
        yaw = 0.0 if lane_id < 0 else 180.0
        r = math.radians(yaw + 90)
        self.transform = SimpleNamespace(location=V(x, y, 0.5), rotation=SimpleNamespace(yaw=yaw),
                                         get_right_vector=lambda: V(math.cos(r), math.sin(r)))

    def get_left_lane(self):
        if self.road.oneway or self.road.separate:
            return None
        return FakeWp(self.road, -self.lane_id, self.transform.location.x)

    def get_right_lane(self):
        return None


class FakeMap:
    def __init__(self, oneway=False, separate=False):
        self.oneway, self.separate = oneway, separate

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        if abs(loc.y) > 30:
            return None
        return FakeWp(self, -1 if (loc.y >= 0 or self.oneway) else 1, loc.x)


FakeCarla = SimpleNamespace(Location=lambda x=0.0, y=0.0, z=0.0: V(x, y, z),
                            LaneType=SimpleNamespace(Driving="Driving"))


def test_roads_governed():
    roads = ndwsigns.Roads(FakeMap(), FakeCarla)
    yaw, z, side = roads.governed(5.0, 4.0, 0.0)        # south pavement, read eastwards: the eastbound lane
    assert yaw == 0.0 and z == 0.5 and side == pytest.approx((5.0, 4.0))
    yaw, z, side = roads.governed(5.0, -4.0, 180.0)     # north pavement, read westwards
    assert yaw == 180.0 and side[1] == pytest.approx(-4.0)
    yaw, z, side = roads.governed(5.0, -4.0, 170.0)     # a bearing rounded to the compass: the lane's own yaw
    assert yaw == 180.0
    # a no-entry sign at the end of a one-way street running east: read by westbound traffic, which has no lane
    yaw, z, side = ndwsigns.Roads(FakeMap(oneway=True), FakeCarla).governed(5.0, -2.5, 180.0)
    assert yaw == 180.0 and side[1] == pytest.approx(-2.4)       # beside the lane, on the traffic's right
    assert ndwsigns.Roads(FakeMap(), FakeCarla).governed(5.0, 100.0, 0.0) is None
    # SUMO's OpenDRIVE: each direction a road of its own (no left lane); a sign on the far side, read westwards
    yaw, z, side = ndwsigns.Roads(FakeMap(separate=True), FakeCarla).governed(5.0, 4.0, 180.0)
    assert yaw == 180.0
    yaw, z, side = ndwsigns.Roads(FakeMap(separate=True), FakeCarla).governed(5.0, -4.0, 180.0)
    assert yaw == 180.0 and side[1] == pytest.approx(-4.0)


# --- speed check ---------------------------------------------------------------------------------------------

class FakeEdge:
    def __init__(self, shape, kmh):
        self.shape, self.kmh = shape, kmh

    def getFunction(self):
        return ""

    def allows(self, vc):
        return True

    def getShape(self):
        return self.shape

    def getSpeed(self):
        return self.kmh / 3.6


class FakeNet:
    def __init__(self, edges):
        self.edges = edges

    def convertLonLat2XY(self, lon, lat):
        return lon, lat

    def getEdges(self):
        return self.edges


def test_speed_check():
    pytest.importorskip("shapely")
    net = FakeNet([FakeEdge([(0, 0), (100, 0)], 50), FakeEdge([(100, 1), (0, 1)], 30),
                   FakeEdge([(0, 50), (100, 50)], 50)])
    segs = [{"id": 1, "v": 30, "d": "B", "g": [[0, 0.5], [100, 0.5]]},
            {"id": 2, "v": None, "d": "B", "g": [[0, 50], [100, 50]]}]     # a cycle path: no speed
    res = ndwsigns.speed_check(net, segs)
    assert res["edges"] == 3 and res["matched"] == 2 and res["agree"] == 1 and res["disagree"] == 1
    assert res["pairs"] == {(50, 30): 1}
    # a section holding only against its geometry matches only the westbound edge
    res = ndwsigns.speed_check(net, [{"id": 1, "v": 30, "d": "T", "g": [[0, 0.5], [100, 0.5]]}])
    assert res["matched"] == 1 and res["agree"] == 1


class NoLaneRoads:
    """No lane of the sign's own in reach: a street the map leaves out, `d` metres from the nearest driving lane."""
    lefthand = False

    def __init__(self, d):
        self.d = d

    def governed(self, x, y, yaw):
        return None

    def distance(self, x, y):
        return self.d


def test_place_off_the_map_roads():
    pl = FakePlacer(C10, lambda x, y: False)
    pl.ground = lambda x, y: 2.0
    ndwsigns.place(pl, C10, [S("G11", b=0, lon=3.0, lat=7.0)], lambda s: (s["lon"], s["lat"]), NoLaneRoads(15.0))
    post, post2, plate = pl.items
    assert post[2:4] == (3.0, 7.0) and post[6] == 2.0          # where the register has it, on the ground
    assert plate[6] == pytest.approx(2.0 + ndwsigns.CLEAR_M)   # its bottom 2.2 m up: a taller post
    assert post[4] == pytest.approx(0.0)       # read northwards (yaw -90): faces south, the traffic coming
    assert plate[1] == "SM_NL_G11" and pl.counts["sign_ndw_offroad"] == 1
    pl = FakePlacer(C10, lambda x, y: False)
    ndwsigns.place(pl, C10, [S("G11", b=0)], lambda s: (s["lon"], s["lat"]), NoLaneRoads(ndwsigns.NEAR_M + 1))
    assert pl.items == [] and pl.counts["sign_ndw_far"] == 1


def test_scenery_spawns_the_blank_plate_without_our_content(monkeypatch, tmp_path):
    """A server without the CoupledCarla content refuses a face mesh: the same blank plate goes in its place, and
    from then on without asking again."""
    import sys
    import types
    m = types.ModuleType("carla")
    m.Location = lambda x=0.0, y=0.0, z=0.0: SimpleNamespace(x=x, y=y, z=z)
    m.Rotation = lambda pitch=0.0, yaw=0.0, roll=0.0: SimpleNamespace(pitch=pitch, yaw=yaw, roll=roll)
    m.Transform = lambda loc, rot: SimpleNamespace(location=loc, rotation=rot)
    m.command = SimpleNamespace(SpawnActor=lambda bp, tf: SimpleNamespace(attrs=dict(bp.attrs), tf=tf))
    monkeypatch.setitem(sys.modules, "carla", m)

    class BP:
        attrs = {}

        def set_attribute(self, k, v):
            self.attrs = dict(self.attrs, **{k: v})

    class Client:
        tried = []

        def apply_batch_sync(self, cmds, _due=False):
            self.tried += [c.attrs["mesh_path"] for c in cmds]
            return [SimpleNamespace(actor_id=0 if c.attrs["mesh_path"].startswith("/Game/CoupledCarla") else i + 1,
                                    error="failed to load mesh" if c.attrs["mesh_path"].startswith("/Game/Coupled")
                                    else "") for i, c in enumerate(cmds)]

    world = SimpleNamespace(get_blueprint_library=lambda: SimpleNamespace(find=lambda k: BP()))
    face, pole = C10.path("SM_NL_E4"), C10.path("SM_Pole01")
    data = {"meshes": [pole, face], "cell_m": 100.0, "fallbacks": ndwsigns.fallbacks(["SM_NL_E4"], C10),
            "items": [[0, 1.0, 1.0, 0.0, 0.0, 1.0, "sign"], [1, 1.0, 1.0, 2.5, 90.0, 2.0, "sign"],
                      [1, 250.0, 1.0, 2.5, 90.0, 2.0, "sign"]]}
    msgs, client = [], Client()
    sc = scenery.Scenery(client, world, tmp_path, {"radius_m": 100.0}, msgs.append, data=data)
    sc.update([SimpleNamespace(x=0.0, y=0.0)])
    plate = C10.path("SM_SquareShape")
    assert client.tried == [pole, face, plate] and sc.n == 2 and any("blank plates" in s for s in msgs)
    client.tried = []
    sc.update([SimpleNamespace(x=250.0, y=0.0)])        # the next cell: the plate at once
    assert client.tried == [plate]
