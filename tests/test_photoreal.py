"""coupledcarla.photoreal + odroutline: Google's Photorealistic 3D Tiles over a baked level (no network, no CARLA,
no key: the HTTP layer is mocked and nothing reaches Google)."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from coupledcarla import config, look, odroutline, photoreal, validate

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import photoreal_check  # noqa: E402

GEO = "+proj=tmerc +lat_0=51.443 +lon_0=5.4815 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs"


def xodr(roads, geo=GEO):
    return (f"<?xml version='1.0'?><OpenDRIVE><header revMajor='1' revMinor='4'><geoReference><![CDATA[{geo}]]>"
            f"</geoReference></header>{''.join(roads)}</OpenDRIVE>")


def lane(i, typ, w):
    return f"<lane id='{i}' type='{typ}'><width sOffset='0' a='{w}' b='0' c='0' d='0'/></lane>"


def road(rid, geom, length, left=(), right=(), offset=0.0, junction="-1", elev=10.0):
    lefts = "".join(lane(i + 1, t, w) for i, (t, w) in enumerate(left))
    rights = "".join(lane(-(i + 1), t, w) for i, (t, w) in enumerate(right))
    return (f"<road id='{rid}' length='{length}' junction='{junction}'><planView>{geom}</planView>"
            f"<elevationProfile><elevation s='0' a='{elev}' b='0' c='0' d='0'/></elevationProfile>"
            f"<lanes><laneOffset s='0' a='{offset}' b='0' c='0' d='0'/><laneSection s='0'>"
            f"<left>{lefts}</left><center><lane id='0' type='none'/></center><right>{rights}</right>"
            f"</laneSection></lanes></road>")


STRAIGHT = road(1, "<geometry s='0' x='0' y='0' hdg='0' length='100'><line/></geometry>", 100,
                left=[("driving", 3.5), ("sidewalk", 2.0)], right=[("driving", 3.5), ("parking", 2.5), ("sidewalk", 2.0)])


# --------------------------------------------------------------------------------------------- OpenDRIVE

def test_reference_line_geometries():
    arc = odroutline.Geometry(__import__("xml.etree.ElementTree", fromlist=["x"]).fromstring(
        "<geometry s='0' x='0' y='0' hdg='0' length='31.4159265'><arc curvature='0.1'/></geometry>"))
    x, y, h = arc.at(arc.length)      # half a circle of radius 10
    assert x == pytest.approx(0.0, abs=1e-4) and y == pytest.approx(20.0, abs=1e-4) and h == pytest.approx(math.pi)
    sp = odroutline.Geometry(__import__("xml.etree.ElementTree", fromlist=["x"]).fromstring(
        "<geometry s='0' x='0' y='0' hdg='0' length='20'><spiral curvStart='0' curvEnd='0.1'/></geometry>"))
    x, y, h = sp.at(20)
    assert h == pytest.approx(1.0)                    # heading = 0.5 * 0.1 * 20
    assert 17.5 < x < 19.5 and 5.5 < y < 7.0          # an Euler spiral bends gently left
    pp = odroutline.Geometry(__import__("xml.etree.ElementTree", fromlist=["x"]).fromstring(
        "<geometry s='0' x='5' y='5' hdg='1.5707963' length='10'>"
        "<paramPoly3 aU='0' bU='10' cU='0' dU='0' aV='0' bV='0' cV='0' dV='0' pRange='normalized'/></geometry>"))
    x, y, h = pp.at(10)
    assert x == pytest.approx(5.0, abs=1e-5) and y == pytest.approx(15.0, abs=1e-5)


def test_lane_outlines_carriageway_and_road():
    roads, geo = odroutline.parse(xodr([STRAIGHT]))
    assert geo.startswith("+proj=tmerc")
    from shapely.geometry import Polygon
    cw = odroutline.lane_outlines(roads[0], odroutline.CARRIAGEWAY, step=5)
    # left: driving (one run); right: driving + parking (one run of 6 m)
    areas = sorted(round(Polygon(p).area) for p in cw)
    assert areas == [350, 600]
    full = odroutline.lane_outlines(roads[0], odroutline.CARRIAGEWAY | odroutline.SIDEWALK, step=5)
    assert sorted(round(Polygon(p).area) for p in full) == [550, 800]
    sw = odroutline.lane_outlines(roads[0], {"sidewalk"}, step=5)     # the outer runs only
    assert sorted(round(Polygon(p).area) for p in sw) == [200, 200]


def test_lane_offset_shifts_the_lanes():
    r = road(1, "<geometry s='0' x='0' y='0' hdg='0' length='10'><line/></geometry>", 10,
             left=[("driving", 3.0)], offset=2.0)
    p = odroutline.lane_outlines(odroutline.parse(xodr([r]))[0][0], {"driving"}, step=5)[0]
    ys = sorted({round(v[1], 6) for v in p})
    assert ys == [2.0, 5.0]


# --------------------------------------------------------------------------------------------- carve

def test_dehole_keeps_area_without_holes():
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    sq = Polygon([(0, 0), (100, 0), (100, 100), (0, 100)], [[(20, 20), (40, 20), (40, 40), (20, 40)],
                                                            [(60, 60), (80, 60), (80, 80), (60, 80)]])
    parts = photoreal.dehole(sq)
    assert all(not p.interiors for p in parts)
    assert sum(p.area for p in parts) == pytest.approx(sq.area)
    assert unary_union(parts).symmetric_difference(sq).area < 1e-6


def test_carve_rings_from_a_grid_has_no_holes(tmp_path):
    # four roads around a block: their union is a ring with a hole, which Cesium's polygons cannot hold
    rs = [road(1, "<geometry s='0' x='0' y='0' hdg='0' length='100'><line/></geometry>", 100, [("driving", 3)], [("driving", 3)]),
          road(2, "<geometry s='0' x='100' y='0' hdg='1.5707963' length='100'><line/></geometry>", 100, [("driving", 3)], [("driving", 3)]),
          road(3, "<geometry s='0' x='100' y='100' hdg='3.1415927' length='100'><line/></geometry>", 100, [("driving", 3)], [("driving", 3)]),
          road(4, "<geometry s='0' x='0' y='100' hdg='-1.5707963' length='100'><line/></geometry>", 100, [("driving", 3)], [("driving", 3)])]
    f = tmp_path / "grid.xodr"
    f.write_text(xodr(rs))
    rings, geo, stats = photoreal.carve_rings(f)
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    assert stats["rings"] == len(rings) >= 2
    u = unary_union([Polygon(r) for r in rings])
    assert u.area == pytest.approx(stats["area_m2"], rel=0.01)
    assert 4 * 100 * 6 * 0.9 < u.area < 4 * 106 * 6       # four 6 m wide roads, corners overlapping
    assert all(Polygon(r).is_valid and not Polygon(r).interiors for r in rings)


def test_footprints_from_manifest_and_buildings_json(tmp_path):
    man = tmp_path / "manifest.json"
    man.write_text(json.dumps({"buildings": [{"rings_ue": json.dumps([[[0, 0], [1000, 0], [1000, -1000], [0, -1000]]])}]}))
    fp = photoreal.load_footprints(man)
    assert fp == [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]]       # UE cm, y south -> m, y north
    bj = tmp_path / "buildings.json"
    bj.write_text(json.dumps([{"h": 9, "p": [[0, 0], [10, 0], [10, -10], [0, -10]]}]))
    assert photoreal.load_footprints(bj) == [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]]
    assert photoreal.load_footprints(tmp_path / "missing.json") == []
    rings = photoreal.footprint_rings(fp, grow=0.5)
    from shapely.geometry import Polygon
    assert Polygon(rings[0]).area == pytest.approx(11 * 11, rel=0.02)


# --------------------------------------------------------------------------------------------- datum

@pytest.fixture
def offline_geoid(monkeypatch):
    monkeypatch.setattr(photoreal, "geoid_height", lambda lat, lon, vd, network=True:
                        {"NAP": 43.93, "EGM96": 44.23, "EGM2008": 44.13, "ellipsoid": 0.0}[vd])


def test_georef_eindhoven(offline_geoid):
    g = photoreal.georef(GEO, epoch=2026.9)
    assert g["vdatum"] == "NAP" and g["frame"] == "ETRS89"
    e, n, u = g["shift_m"]
    assert math.hypot(e, n) == pytest.approx(0.90, abs=0.05)      # ETRS89 drifts north-east ~2.5 cm a year
    assert 30 < math.degrees(math.atan2(e, n)) < 65 and abs(u) < 0.05
    assert g["height"] == pytest.approx(43.93, abs=0.05)
    assert g["lat"] > 51.443 and g["lon"] > 5.4815
    assert "geoidgrids" not in g["proj"]


def test_georef_options(offline_geoid):
    g = photoreal.georef(GEO.replace("51.443", "40.75").replace("5.4815", "-73.98"),
                         {"vdatum": "EGM96", "shift_m": [1.0, -2.0, 0.5]})
    assert g["frame"] == "given" and g["height"] == pytest.approx(44.23 + 0.5)
    with pytest.raises(photoreal.PhotorealError):
        photoreal.georef("+proj=utm +zone=31")


def test_georef_without_grid_fails_plainly(monkeypatch):
    monkeypatch.setattr(photoreal, "geoid_height", lambda *a, **k: None)
    with pytest.raises(photoreal.PhotorealError, match="geoid grid"):
        photoreal.georef(GEO)


def test_cesium_placement_error_is_small(offline_geoid):
    g = photoreal.georef(GEO, epoch=2026.9)
    for x, y in ((0, 0), (800, 0), (0, -800), (800, 800)):
        horiz, vert = photoreal.placement_error(g, x, y, 15.0)
        d = math.hypot(x, y)
        assert horiz < 0.02                                       # tmerc vs the tangent plane: millimetres
        assert vert == pytest.approx(-d * d / (2 * 6371000), abs=0.01)   # the earth curving away: 5 cm at 800 m


def test_projector_round_trip(offline_geoid):
    g = photoreal.georef(GEO, {"shift_m": [0, 0, 0]})
    lon, lat = photoreal.Projector(g).lonlat([(0, 0)])[0]
    assert (lat, lon) == pytest.approx((51.443, 5.4815), abs=1e-7)
    lon2, lat2 = photoreal.Projector(g).lonlat([(0, 111.2)])[0]
    assert (lat2 - lat) * 111200 == pytest.approx(111.2, rel=0.01)


def test_level_data(tmp_path, offline_geoid):
    (tmp_path / "twins").mkdir()
    (tmp_path / "twins" / "T.xodr").write_text(xodr([STRAIGHT]))
    cfg = {"_root": str(tmp_path), "name": "t",
           "map": {"name": "t", "opendrive": "twins/T.xodr", "carla_level": "T", "photoreal": "google"}}
    man = tmp_path / "m.json"
    man.write_text(json.dumps({"buildings": [{"rings_ue": [[[0, 1000], [1000, 1000], [1000, 2000], [0, 2000]]]}]}))
    d = photoreal.level_data(cfg, footprints=man)
    assert d["level"] == "T" and d["hide"] == ["building", "terrain"]
    assert len(d["carve"]) == 1 and len(d["footprints"]) == 1     # both sides meet at the reference line
    assert all(5.48 < p[0] < 5.483 and 51.44 < p[1] < 51.444 for r in d["carve"] for p in r)
    out = photoreal.prepare(cfg, pack=tmp_path / "pack", footprints=man)
    assert out == tmp_path / "pack" / "Content" / "Config" / "Photoreal" / "T.json"
    assert json.loads(out.read_text())["origin"]["height"] == pytest.approx(43.93, abs=0.05)


# --------------------------------------------------------------------------------------------- settings

def test_settings_and_role_options():
    assert photoreal.settings({}) is None and photoreal.settings({"photoreal": False}) is None
    s = photoreal.settings({"photoreal": "google"})
    assert s["carve"] == "carriageway" and s["day_cap"] == 50
    opts = dict(kv.split("=") for kv in photoreal.role_options(s).split(";"))
    assert opts["relit"] == "0" and opts["hide"] == "building,terrain" and "url" not in opts
    assert dict(kv.split("=") for kv in photoreal.role_options(s, night=True).split(";"))["relit"] == "1"   # light auto
    s2 = photoreal.settings({"photoreal": {"light": "baked", "hide": "building", "carve": "none"}})
    o2 = dict(kv.split("=") for kv in photoreal.role_options(s2, night=True).split(";"))
    assert o2["relit"] == "0" and o2["hide"] == "building" and o2["carve"] == "0"


def test_problems():
    base = {"carla_level": "EhvCentre", "opendrive": "twins/EhvCentre.xodr"}
    assert photoreal.problems({"map": dict(base, photoreal="google")}) == []
    assert photoreal.problems({"map": dict(base)}) == []
    msgs = [m for _, m in photoreal.problems({"map": {"photoreal": {"carve": "all", "day_cap": 0, "colour": 1,
                                                                   "hide": ["roofs"], "month_cap": 5000}}})]
    text = " | ".join(msgs)
    for frag in ("unknown key 'colour'", "carve 'all'", "day_cap 0", "hide: unknown roofs", "above the 1,000 free",
                 "map.carla_level", "map.opendrive"):
        assert frag in text


def test_experiment_validates():
    cfg = config.load(ROOT / "experiments" / "eindhoven_twin_photoreal.yaml")
    assert config.load(ROOT / "experiments" / "eindhoven_twin.yaml")["map"]["photoreal"] is False    # default off
    assert photoreal.settings(cfg["map"])["provider"] == "google"
    import yaml
    raw = yaml.safe_load((ROOT / "experiments" / "eindhoven_twin_photoreal.yaml").read_text())
    rep, _ = validate.check(raw, cfg, roads=False)
    assert not [e for e in rep.errors if "photoreal" in str(e)]


# --------------------------------------------------------------------------------------------- key, ledger

def test_read_key_env_file_and_status(tmp_path):
    assert photoreal.read_key(env={}, cfgdir=tmp_path) == (None, None)
    (tmp_path / "google_maps_key").write_text("# my key\nAIzaFAKEKEY123\n")
    (tmp_path / "google_maps_key").chmod(0o644)
    assert photoreal.read_key(env={}, cfgdir=tmp_path) == ("AIzaFAKEKEY123", "file")
    assert photoreal.read_key(env={"GOOGLE_MAPS_API_KEY": " ENVKEY "}, cfgdir=tmp_path) == ("ENVKEY", "env")
    st = photoreal.key_status(env={}, cfgdir=tmp_path)
    assert "AIzaFAKEKEY123" not in st and "14 characters" in st and "chmod 600" in st
    assert photoreal.config_dir({"COUPLED_CARLA_CONFIG_DIR": str(tmp_path)}) == tmp_path


def test_scrub():
    assert "SECRET" not in photoreal.scrub("GET https://x/root.json?key=SECRET&session=1")
    assert "SECRET" not in photoreal.scrub("{'X-Goog-Api-Key': 'SECRET'}")
    assert photoreal.scrub("a SECRET b", "SECRET") == "a *** b"


class Clock:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t


def test_usage_rollover_and_caps(tmp_path):
    clock = Clock(time.mktime((2026, 11, 15, 12, 0, 0, 0, 0, -1)))
    u = photoreal.Usage(tmp_path / "u.json", clock=clock)
    assert u.allows(2, 1000) == (True, "")
    u.count_root("EhvCentre", run="r1")
    u.count_root("EhvCentre", run="r1")
    d = u.load()
    assert (d["day_roots"], d["month_roots"], d["total_roots"]) == (2, 2, 2)
    assert d["runs"] == [{"start": "r1", "map": "EhvCentre", "by": "client", "roots": 2}]
    ok, why = u.allows(2, 1000)
    assert not ok and "today's cap of 2" in why
    clock.t += 86400                                    # the next day: the daily count starts again
    assert u.load()["day_roots"] == 0 and u.load()["month_roots"] == 2
    assert not u.allows(10, 2)[0]
    clock.t = time.mktime((2026, 12, 1, 12, 0, 0, 0, 0, -1))    # a new month
    assert u.load()["month_roots"] == 0 and u.load()["total_roots"] == 2


# --------------------------------------------------------------------------------------------- HTTP

class Resp:
    def __init__(self, code, body, headers=None):
        self.status_code, self.content, self.headers = code, body, headers or {}


class FakeHTTP:
    """Google's 3D Tiles API as far as the session goes: root.json (max-age 0) whose child URLs carry the session,
    a nested tileset whose links do not, tiles with max-age 14400."""

    def __init__(self, status=200):
        self.calls = []
        self.status = status
        self.n_root = 0

    def get(self, url, headers=None, timeout=None):
        self.calls.append((url, dict(headers or {})))
        path = url.split("?")[0]
        if self.status != 200:
            return Resp(self.status, json.dumps({"error": {"message": "API key not valid. key=LEAK"}}).encode())
        if path.endswith("/v1/3dtiles/root.json"):
            self.n_root += 1
            sess = f"S{self.n_root}"
            return Resp(200, json.dumps({"root": {"children": [
                {"content": {"uri": f"/v1/3dtiles/datasets/A/files/sub.json?session={sess}"}},
                {"content": {"uri": f"/v1/3dtiles/datasets/A/files/t.glb?session={sess}"}}]}}).encode(),
                {"Cache-Control": "max-age=0"})
        if path.endswith(".json"):
            return Resp(200, json.dumps({"root": {"content": {"uri": "u.glb"}}}).encode(),
                        {"Cache-Control": "private, max-age=14400"})
        return Resp(200, b"GLB", {"Cache-Control": "private, max-age=14400, must-revalidate"})


def session(tmp_path, http, clock=None, **kw):
    clock = clock or Clock(1e9)
    return photoreal.TileSession("FAKEKEY", http=http, usage=photoreal.Usage(tmp_path / "u.json", clock=clock),
                                 clock=clock, map_name="T", **kw), clock


def test_one_root_request_per_session(tmp_path):
    http = FakeHTTP()
    s, clock = session(tmp_path, http)
    s.root()
    s.get("/v1/3dtiles/datasets/A/files/t.glb?session=S1")
    s.get("/v1/3dtiles/datasets/A/files/sub.json?session=S1")
    s.get("u.glb", "https://tile.googleapis.com/v1/3dtiles/datasets/A/files/sub.json?session=S1")
    for _ in range(20):                                    # every tick or tile reuses the session
        s.root()
    assert http.n_root == 1 and s.roots == 1
    assert photoreal.Usage(tmp_path / "u.json", clock=clock).load()["day_roots"] == 1
    # the nested tileset's relative link got the session, the key went in the header, never in a URL
    urls = [u for u, _ in http.calls]
    assert any(u.endswith("files/u.glb?session=S1") for u in urls)
    assert all("FAKEKEY" not in u and "key=" not in u for u in urls)
    assert all(h.get("X-Goog-Api-Key") == "FAKEKEY" for _, h in http.calls)
    assert "FAKEKEY" not in repr(s)


def test_session_renewed_after_three_hours(tmp_path):
    http = FakeHTTP()
    s, clock = session(tmp_path, http)
    s.root()
    clock.t += 2.5 * 3600
    s.root()
    assert http.n_root == 1
    clock.t += 0.5 * 3600                                  # past 3 h less the margin: a new (billed) session
    s.get("/v1/3dtiles/datasets/A/files/t.glb")
    assert http.n_root == 2 and http.calls[-1][0].endswith("t.glb?session=S2")


def test_cache_follows_max_age_in_memory_only(tmp_path):
    http = FakeHTTP()
    s, clock = session(tmp_path, http)
    for _ in range(3):
        s.get("/v1/3dtiles/datasets/A/files/t.glb")
    assert sum(1 for u, _ in http.calls if "t.glb" in u) == 1
    clock.t += 14401                                       # max-age over: fetched again
    s.get("/v1/3dtiles/datasets/A/files/t.glb")
    assert sum(1 for u, _ in http.calls if "t.glb" in u) == 2
    assert list(tmp_path.iterdir()) == [tmp_path / "u.json"]    # nothing written but the ledger


def test_caps_refuse_before_any_request(tmp_path):
    http = FakeHTTP()
    s, clock = session(tmp_path, http, day_cap=1)
    s.root()
    s2, _ = session(tmp_path, http, clock=clock, day_cap=1)
    with pytest.raises(photoreal.PhotorealError, match="today's cap"):
        s2.root()
    assert http.n_root == 1


def test_errors_never_carry_the_key(tmp_path):
    s, _ = session(tmp_path, FakeHTTP(status=403))
    with pytest.raises(photoreal.PhotorealError) as e:
        s.root()
    assert "403" in str(e.value) and "LEAK" not in str(e.value) and "FAKEKEY" not in str(e.value)

    class Boom:
        def get(self, url, headers=None, timeout=None):
            raise OSError(f"connection reset {url} {headers}")

    s, _ = session(tmp_path, Boom())
    with pytest.raises(photoreal.PhotorealError) as e:
        s.root()
    assert "FAKEKEY" not in str(e.value)


def test_key_in_url_fallback(tmp_path):
    http = FakeHTTP()
    s, _ = session(tmp_path, http, key_in="url")
    s.root()
    assert "key=FAKEKEY" in http.calls[0][0] and "X-Goog-Api-Key" not in http.calls[0][1]
    s2, _ = session(tmp_path, FakeHTTP(status=403), key_in="url")
    with pytest.raises(photoreal.PhotorealError) as e:
        s2.root()
    assert "FAKEKEY" not in str(e.value)


def test_no_key_no_session():
    with pytest.raises(photoreal.PhotorealError):
        photoreal.TileSession(None, http=FakeHTTP())


# --------------------------------------------------------------------------------------------- measure

def test_glb_decode_and_area(tmp_path, offline_geoid):
    g = photoreal.georef(GEO, {"shift_m": [0, 0, 0]})
    files = photoreal_check.mock_files({"lat": g["lat"], "lon": g["lon"], "height": g["height"]})
    local = photoreal.decode_glb_positions(files["/tile.glb"])
    assert local.shape == (4, 3) and np.allclose(sorted(set(local[:, 2])), [10.0])    # y-up -> z-up: 10 m up

    class MockSession:
        root_url = "http://127.0.0.1/root.json"
        roots = 1

        def root(self):
            return json.loads(files["/root.json"])

        def get(self, uri, base=None):
            return files["/" + uri.split("?")[0].split("/")[-1]]

    enu, tiles = photoreal.fetch_area(MockSession(), g, (0, 0), 100)
    assert tiles == 1 and enu.shape == (4, 3)
    assert np.allclose(enu[:, 2], 10.0, atol=0.01) and np.allclose(np.abs(enu[:, :2]), 50.0, atol=0.01)
    _, none = photoreal.fetch_area(MockSession(), g, (3000, 0), 100)     # out of the box: not fetched
    assert none == 0


def test_ground_offset_and_footprint_shift():
    rng = np.random.default_rng(1)
    street = np.c_[rng.uniform(0, 50, 4000), rng.uniform(0, 50, 4000), np.full(4000, 15.6)]
    cars = np.c_[rng.uniform(0, 50, 300), rng.uniform(0, 50, 300), np.full(300, 17.0)]
    roads = [(x, y, 15.0) for x in range(2, 48, 4) for y in range(2, 48, 4)]
    dz, iqr, n = photoreal.ground_offset(np.vstack([street, cars]), roads)
    assert dz == pytest.approx(0.6, abs=0.01) and n > 100
    fp = [[(10, 10), (30, 10), (30, 30), (10, 30)]]
    walls = np.c_[rng.uniform(10, 30, 3000) + 1.5, rng.uniform(10, 30, 3000) - 1.0, rng.uniform(20, 30, 3000)]
    sh = photoreal.footprint_shift(walls, fp, ground_z=15.0)
    assert sh["east_m"] == pytest.approx(1.5, abs=0.26) and sh["north_m"] == pytest.approx(-1.0, abs=0.26)
    assert sh["inside_after"] > sh["inside_before"]


# --------------------------------------------------------------------------------------------- run time

class FakeBP:
    def __init__(self, id_):
        self.id, self.attrs = id_, {"role_name": ""}

    def has_attribute(self, k):
        return k in self.attrs

    def set_attribute(self, k, v):
        self.attrs[k] = v


class FakeActor:
    def __init__(self):
        self.id, self.is_alive = 7, True

    def destroy(self):
        self.is_alive = False


class FakeWorld:
    def __init__(self, bps):
        self.bps, self.spawned = bps, []

    def get_blueprint_library(self):
        w = self

        class Lib:
            def filter(self, pat):
                return [b for b in w.bps if b.id == pat]
        return Lib()

    def try_spawn_actor(self, bp, tf):
        a = FakeActor()
        self.spawned.append((bp, a))
        return a

    def get_actor(self, i):
        return next((a for _, a in self.spawned if a.id == i and a.is_alive), None)


@pytest.fixture
def fake_carla(monkeypatch):
    class T:
        def __init__(self, *a):
            pass
    monkeypatch.setitem(sys.modules, "carla", type(sys)("carla"))
    sys.modules["carla"].Transform = T


def test_attach_off_missing_and_on(fake_carla):
    logs = []
    tc = {"map": {"carla_level": "EhvCentre", "photoreal": "google"}}
    assert photoreal.attach(FakeWorld([]), {"map": {"carla_level": "EhvCentre"}}, log_fn=logs.append) is None
    assert logs == []                                               # off: silent
    assert photoreal.attach(FakeWorld([]), tc, log_fn=logs.append) is None
    assert "has no blueprint.cc_photoreal_google" in logs[-1]
    assert photoreal.attach(FakeWorld([FakeBP(photoreal.BLUEPRINT)]), {"map": {"photoreal": "google"}},
                            log_fn=logs.append) is None and "map.carla_level" in logs[-1]
    w = FakeWorld([FakeBP(photoreal.BLUEPRINT)])
    h = photoreal.attach(w, tc, night=True, log_fn=logs.append)
    bp, actor = w.spawned[0]
    opts = dict(kv.split("=") for kv in bp.attrs["role_name"].split(";"))
    assert opts["relit"] == "1" and "key" not in opts and opts["key_in"] == "header"     # never the key itself
    assert h.check() and "on" in logs[-1]
    actor.is_alive = False                                          # the server removed it (no key there)
    assert not h.check() and "did not start" in logs[-1]
    assert h.info()["state"] == "off"


def test_attribution_and_stamp(tmp_path):
    assert photoreal.attribution_text({"attribution": "auto"}, cfgdir=tmp_path) == "Google"
    (tmp_path / "google_tiles_credits.txt").write_text("Landsat / Copernicus; Aerodata")
    assert photoreal.attribution_text({"attribution": "auto"}, cfgdir=tmp_path) == "Google · Landsat / Copernicus; Aerodata"
    assert photoreal.attribution_text({"attribution": "Map data ©2026 Google"}) == "Map data ©2026 Google"
    img = np.full((360, 640, 3), 128, np.uint8)
    out = photoreal.stamp(img, "Google", logo=tmp_path / "none.png")
    assert out.shape == img.shape
    assert (out[-20:, -60:] != 128).any() and (out[:300, :500] == 128).all()


def test_save_frame_stamps_while_on(tmp_path, monkeypatch):
    monkeypatch.setattr(look, "ATTRIBUTION", "Google")
    img = np.full((120, 200, 3), 128, np.uint8)
    look.save_frame(img, tmp_path / "f.png")
    from PIL import Image
    a = np.asarray(Image.open(tmp_path / "f.png"))
    assert (a[-12:, -40:] != 128).any()
    monkeypatch.setattr(look, "ATTRIBUTION", None)
    look.save_frame(img, tmp_path / "g.png")
    assert (np.asarray(Image.open(tmp_path / "g.png")) == 128).all()


def test_pack_registers_the_blueprint():
    bp = json.loads((photoreal.PACK / "Content" / "Config" / "BlueprintParameters.json").read_text())
    e = next(b for b in bp["Blueprints"] if b["Name"] == "cc_photoreal_google")
    assert e["Path"].endswith("_C")          # CARLA appends _C to paths without it: the native class carries it
