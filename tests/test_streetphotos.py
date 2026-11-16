"""coupledcarla/streetphotos.py without a network, a GPU or CARLA: Mapillary vector tiles and records, the cache and
the polite HTTP client, the extra metrics (edge alignment, differences in words, spot and map scores), the street
objects of scenery.json against photo evidence, the correction pass, and poles found in a panorama's labels."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from coupledcarla import realism as R
from coupledcarla import streetphotos as SP

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))


# ---------------------------------------------------------------- a minimal protobuf writer for test tiles

def _vint(v):
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        out.append(b | (0x80 if v else 0))
        if not v:
            return bytes(out)


def _field(num, wt, payload):
    if wt == 0:
        return _vint(num << 3) + _vint(payload)
    return _vint(num << 3 | 2) + _vint(len(payload)) + payload


def _zz(v):
    return (v << 1) ^ (v >> 31)


def _tile(layer, points, extent=4096):
    """A vector tile with one layer of point features: points = [(x, y, {key: value})]."""
    keys, vals, feats = [], [], b""
    for i, (x, y, props) in enumerate(points):
        tags = []
        for k, v in props.items():
            if k not in keys:
                keys.append(k)
            if v not in vals:
                vals.append(v)
            tags += [keys.index(k), vals.index(v)]
        geom = [(1 << 3) | 1, _zz(x), _zz(y)]
        f = _field(1, 0, i + 1) + _field(2, 2, b"".join(_vint(t) for t in tags)) + _field(3, 0, 1) + \
            _field(4, 2, b"".join(_vint(g) for g in geom))
        feats += _field(2, 2, f)

    def val(v):
        if isinstance(v, str):
            return _field(1, 2, v.encode())
        if isinstance(v, bool):
            return _field(7, 0, int(v))
        return _field(5, 0, v)
    body = _field(15, 0, 2) + _field(1, 2, layer.encode()) + feats + b"".join(_field(3, 2, k.encode()) for k in keys)
    body += b"".join(_field(4, 2, val(v)) for v in vals) + _field(5, 0, extent)
    return _field(3, 2, body)


def test_vector_tile_decoding_and_tile_coordinates():
    data = _tile("image", [(100, 200, {"id": 42, "is_pano": True, "sequence_id": "abc", "captured_at": 1700000000000}),
                           (4000, 50, {"id": 43, "is_pano": False})])
    layers = SP.decode_mvt(data)
    extent, rows = layers["image"]
    assert extent == 4096 and len(rows) == 2
    assert rows[0]["parts"] == [[(100, 200)]]
    assert rows[0]["props"] == {"id": 42, "is_pano": True, "sequence_id": "abc", "captured_at": 1700000000000}
    assert rows[1]["props"]["is_pano"] is False
    # tile_lonlat and tile_of agree, and a box covers the tiles around its corners
    z, x, y = 14, *SP.tile_of(51.4433, 5.4797, 14)
    lat, lon = SP.tile_lonlat(z, x, y, 2048, 2048)
    assert SP.tile_of(lat, lon, 14) == (x, y)
    assert abs(lat - 51.4433) < 0.02 and abs(lon - 5.4797) < 0.03
    tiles = SP.tiles_for_bbox([51.436, 5.47, 51.45, 5.493])
    assert (14, x, y) in tiles and len(tiles) == len(set(tiles))


def test_mapillary_points_in_box_from_cached_tiles(tmp_path):
    z, x, y = 14, *SP.tile_of(51.4433, 5.4797, 14)
    cache = SP.Cache(tmp_path)
    for layer, rows in (("mly1_public", [(2048, 2048, {"id": 7, "sequence_id": "s", "captured_at": 86400000 * 3})]),
                        ("mly_map_feature_point", [(2048, 2048, {"id": 9, "value": "object--street-light",
                                                                 "first_seen_at": 1, "last_seen_at": 2})]),
                        ("mly_map_feature_traffic_sign", [])):
        name = "image" if layer == "mly1_public" else "point" if "point" in layer else "traffic_sign"
        cache.get(f"mapillary/tiles/{layer}/{z}/{x}/{y}.pbf", lambda r=rows, n=name: _tile(n, r))

    class NoHttp:
        def get(self, *a, **k):
            raise AssertionError("cached tiles: no request")
    m = SP.Mapillary(tmp_path, token="MLY|test", http=NoHttp(), log=lambda s: None)
    lat, lon = SP.tile_lonlat(z, x, y, 2048, 2048)
    box = [lat - 0.001, lon - 0.001, lat + 0.001, lon + 0.001]
    assert [p["id"] for p in m.points("mly1_public", box)] == [7]
    assert m.camera_points(box) == [(pytest.approx(lat), pytest.approx(lon), 3)]
    feats = m.features(box)
    assert feats[0]["value"] == "object--street-light" and SP.mly_class(feats[0]["value"]) == "lamp"
    assert m.points("mly1_public", [lat + 0.002, lon, lat + 0.003, lon + 0.001]) == []    # outside the box


def test_mapillary_needs_a_token(monkeypatch):
    monkeypatch.setattr(SP, "mapillary_token", lambda: None)
    with pytest.raises(SP.FetchError, match="Dashboard"):
        SP.Mapillary("/tmp/none")


def test_mapillary_record_pose_and_fov():
    x = {"id": "1", "camera_type": "perspective", "camera_parameters": [0.8, 0.0, 0.0], "width": 2048, "height": 1536,
         "compass_angle": 10.0, "computed_compass_angle": 372.5, "geometry": {"coordinates": [5.0, 51.0]},
         "computed_geometry": {"coordinates": [5.001, 51.001]}, "captured_at": 1700000000000,
         "creator": {"username": "someone"}, "thumb_2048_url": "https://x/y.jpg", "sequence": "s1"}
    r = SP.mapillary_record(x)
    assert r["heading"] == pytest.approx(12.5)          # the computed (structure from motion) heading, wrapped
    assert (r["lat"], r["lon"]) == (51.001, 5.001)
    assert r["fov"] == pytest.approx(math.degrees(2 * math.atan(1 / 1.6)))
    assert r["projection"] == "pinhole" and r["kind"] == "mapillary" and r["licence"] == "CC BY-SA 4.0"
    assert r["time"] == "2023-11-14T22:13:20Z" and r["author"] == "someone"
    pano = SP.mapillary_record(dict(x, camera_type="spherical"))
    assert pano["projection"] == "equirect" and pano["fov"] == 360.0 and pano["kind"] == "mly_pano"
    assert SP.mapillary_record(dict(x, camera_type="fisheye")) is None
    assert SP.mapillary_record({k: v for k, v in x.items() if "compass" not in k}) is None
    assert R.HEIGHTS["mly_pano"] > R.HEIGHTS["mapillary"]
    assert "mapillary" in R.SOURCES and "mapillary" in R.FETCHERS


def test_cache_makes_once_and_stays_inside(tmp_path):
    c = SP.Cache(tmp_path)
    calls = []
    assert c.get("a/b.bin", lambda: calls.append(1) or b"x") == b"x"
    assert c.get("a/b.bin", lambda: calls.append(1) or b"y") == b"x"
    assert calls == [1]
    assert c.json("j.json", lambda: {"k": 1}) == {"k": 1}
    with pytest.raises(ValueError):
        c.path("../escape")


def test_http_retries_and_never_shows_the_url():
    class Resp:
        def __init__(self, code):
            self.status_code = code

    class Session:
        def __init__(self, codes):
            self.codes = list(codes)
            self.seen = []

        def get(self, url, params=None, headers=None, timeout=None):
            self.seen.append(params)
            return Resp(self.codes.pop(0))
    s = Session([429, 503, 200])
    h = SP.Http(session=s, sleep=lambda t: None)
    assert h.get("https://tiles.mapillary.com/x", {"access_token": "MLY|secret"}).status_code == 200
    assert len(s.seen) == 3
    h = SP.Http(session=Session([403]), sleep=lambda t: None)
    with pytest.raises(SP.FetchError) as e:
        h.get("https://tiles.mapillary.com/x", {"access_token": "MLY|secret"}, what="Mapillary tile")
    assert "secret" not in str(e.value) and "403" in str(e.value)


# ---------------------------------------------------------------- metrics

def _scene(w=120, h=90, building_left=True, road=0.5):
    g = np.full((h, w), R.GROUPS.index("sky"), np.uint8)
    g[h // 2:] = R.GROUPS.index("sidewalk")
    g[int(h * (1 - road / 2)):] = R.GROUPS.index("road")
    if building_left:
        g[10:h // 2, :w // 3] = R.GROUPS.index("building")
    return g


def test_boundary_f():
    a = _scene()
    assert SP.boundary_f(a, a) == pytest.approx(1.0)
    shifted = np.roll(a, 12, axis=0)
    assert SP.boundary_f(a, shifted, tol=2) < 0.5
    down1 = np.concatenate([a[:1], a[:-1]])               # one pixel lower
    assert SP.boundary_f(a, down1, tol=2) == pytest.approx(1.0)
    flat = np.zeros_like(a)
    assert SP.boundary_f(flat, flat) != SP.boundary_f(flat, flat)        # NaN: no outlines in either


def test_differences_in_words():
    real, render = _scene(building_left=True, road=0.3), _scene(building_left=False, road=0.9)
    texts = [t for _, t in SP.differences(real, render, 90.0, R.GROUPS)]
    assert any(t.startswith("missing building on the left") for t in texts)
    assert any(t.startswith("road too wide") for t in texts)
    assert SP.differences(real, real, 90.0, R.GROUPS) == []
    trees = render.copy()
    trees[5:40, 90:] = R.GROUPS.index("vegetation")
    assert any("trees or greenery on the right" in t for _, t in SP.differences(render, trees, 90.0, R.GROUPS))


def test_spot_and_map_scores():
    assert SP.spot_score({"al_miou_coarse": 0.4, "boundary_f": 0.6, "dino_patch": float("nan")}) == pytest.approx(0.5)
    assert math.isnan(SP.spot_score({}))
    rows = [{"spot_score": 0.4,
             "differences": "missing building on the left (photo 50%, render 0%); road too wide (x)"},
            {"spot_score": 0.6,
             "differences": "missing building on the right (photo 40%, render 0%); skyline 6 deg too "
                            "low: buildings or trees lower than in the photo, or missing"},
            {"spot_score": float("nan"), "differences": ""}]
    m = SP.map_score(rows)
    assert m["score"] == 50.0 and m["spots"] == 2
    assert m["differences"] == {"missing building": 2, "road too wide": 1, "skyline too low": 1}


# ---------------------------------------------------------------- street objects

def _scenery():
    meshes = ["/Game/a/SM_Streetlight3_group3.SM_Streetlight3_group3", "/Game/a/SM_RoadSigns01.SM_RoadSigns01",
              "/Game/a/SM_stopSign.SM_stopSign", "/Game/a/SM_StreetBench.SM_StreetBench",
              "/Game/a/SM_Maple.SM_Maple"]
    items = [[0, 0.0, 0.0, 1.0, 90.0, 1.0, "lamp"],
             [0, 100.0, 0.0, 1.0, 90.0, 1.0, "lamp"],
             [1, 50.0, 10.0, 1.0, 0.0, 1.0, "sign"],
             [2, 50.3, 10.2, 2.0, 0.0, 1.0, "sign"],
             [3, 20.0, 5.0, 1.0, 45.0, 1.0, "furniture"],
             [4, 30.0, 5.0, 1.0, 0.0, 1.0, "tree"],
             None]
    return {"meshes": meshes, "items": items}


def test_scenery_objects():
    objs = SP.scenery_objects(_scenery())
    by = {(o["cls"], round(o["x"])): o for o in objs}
    assert set(by) == {("lamp", 0), ("lamp", 100), ("sign", 50), ("bench", 20)}
    assert by[("sign", 50)]["items"] == [2, 3]          # the post and its plate are one sign


def test_merge_and_match():
    pts = [{"x": 0.0, "y": 0.0}, {"x": 0.5, "y": 0.0}, {"x": 10.0, "y": 0.0}]
    merged = SP.merge_points(pts, 1.5)
    assert [m["n"] for m in merged] == [2, 1]
    pairs = SP.match([{"x": 0.0, "y": 0.0}, {"x": 9.0, "y": 0.0}], [{"x": 1.0, "y": 0.0}, {"x": 2.0, "y": 0.0}], 3.0)
    assert pairs == [(0, 0, 1.0)]


def _ev(cls, x, y, span_days=10, i="e"):
    return {"cls": cls, "x": x, "y": y, "id": i, "first_seen": 0, "last_seen": span_days * 86400000}


def test_compare_objects_rules_and_protection():
    objs = SP.scenery_objects(_scenery())
    many = [(x, y, d) for d in range(5) for x in np.linspace(-5, 110, 60) for y in (0.0, 4.0)]
    obs = SP.Observations(many)
    ev = [_ev("lamp", 3.0, 0.0, i="moved"),            # 3 m from the lamp at 0: a move
          _ev("lamp", 60.0, -2.0, i="new"),            # nothing near: an add
          _ev("lamp", 80.0, 0.0, span_days=0, i="once")]       # seen on one day only: no add
    stats, corr = SP.compare_objects(ev, objs, obs)
    acts = {(c["action"], c["cls"]) for c in corr}
    assert ("move", "lamp") in acts and ("add", "lamp") in acts
    assert ("remove", "lamp") in acts                   # the lamp at 100: photographed often, never seen
    assert not any(c.get("evidence") == "once" for c in corr)
    assert stats["lamp"]["matched"] == 1 and stats["lamp"]["evidence"] == 3
    _, corr = SP.compare_objects(ev, objs, obs, protect={"lamp"})
    assert not any(c["cls"] == "lamp" for c in corr)
    # an object nobody photographed is never removed
    _, corr = SP.compare_objects([], objs, SP.Observations([]))
    assert corr == []
    assert SP.summarise_corrections([{"action": "add", "cls": "lamp"}, {"action": "add", "cls": "lamp"}]) == \
        {"lamp": {"add": 2}}


def test_apply_corrections():
    data = _scenery()
    corr = [{"action": "move", "cls": "lamp", "items": [0], "from": [0.0, 0.0], "to": [3.0, 0.0]},
            {"action": "remove", "cls": "lamp", "items": [1], "from": [100.0, 0.0]},
            {"action": "add", "cls": "lamp", "to": [60.0, -2.0]},
            {"action": "add", "cls": "lamp", "to": [70.0, 0.0]}]
    n = len(data["items"])
    counts = SP.apply_corrections(data, corr, ("move", "add", "remove"), ground=lambda x, y: 0.1 * x,
                                  on_lane=lambda x, y: x == 70.0, log=lambda s: None)
    assert counts == {"move": 1, "add": 1, "remove": 1, "refused_on_lane": 1}
    assert data["items"][0][1:3] == [3.0, 0.0] and data["items"][0][3] == pytest.approx(1.3)
    assert data["items"][1] is None
    new = data["items"][n]          # a copy of the nearest lamp (the moved one: 1.3 m at ground 0.3) at ground 6.0
    assert new[0] == 0 and new[1:3] == [60.0, -2.0] and new[6] == "lamp" and new[3] == pytest.approx(7.0)
    # only the asked actions
    data = _scenery()
    assert SP.apply_corrections(data, corr, ("add",), log=lambda s: None)["move"] == 0


def test_correct_map_writes_scenery_and_keeps_the_old(tmp_path):
    (tmp_path / "scenery.json").write_text(json.dumps(_scenery()))
    (tmp_path / "meta.json").write_text(json.dumps({"scenery": {}}))
    cams = [(x, 0.0, d) for d in range(5) for x in np.linspace(80, 120, 30)]
    SP.save_evidence(tmp_path, "mapillary", [_ev("lamp", 60.0, -2.0)], cams)
    res = SP.correct_map(tmp_path, ("mapillary",), apply=True, log=lambda s: None)
    assert res["mapillary"]["summary"]["lamp"] == {"add": 1, "remove": 1}
    data = json.loads((tmp_path / "scenery.json").read_text())
    assert data["items"][1] is None and data["items"][-1][1:3] == [60.0, -2.0]
    assert (tmp_path / "scenery.before_photos.json").exists()
    assert json.loads((tmp_path / "meta.json").read_text())["scenery"]["photo_corrections_mapillary"]["add"] == 1


def test_protected_classes(tmp_path):
    (tmp_path / "real").mkdir()
    (tmp_path / "real" / "municipal.json").write_text(json.dumps(
        {"elements": [{"tags": {"highway": "street_lamp"}}]}))
    (tmp_path / "meta.json").write_text(json.dumps({"scenery": {"sign_ndw_real": 3}}))
    assert SP.protected_classes(tmp_path) == {"lamp", "sign"}
    assert SP.protected_classes(tmp_path / "nowhere") == set()


def test_validation_figures_on_a_toy_register():
    truth = [{"cls": "lamp", "x": float(x), "y": 0.0, "items": [i]} for i, x in enumerate(range(0, 400, 25))]
    perfect = [_ev("lamp", o["x"], 0.0, i=str(k)) for k, o in enumerate(truth)]
    obs = SP.Observations([(x, 0.0, d) for d in range(5) for x in np.linspace(0, 400, 200)])
    v = SP.validate_corrections(truth, perfect, obs, shift_m=3.0, drop=0.3, extra=0.3, seed=3)
    assert v["register"] == len(truth)
    assert v["after"]["missing"] <= v["before"]["missing"]          # perfect evidence fills the gaps
    assert v["removes"] == v["removes_of_made_up"]                  # and removes only made-up objects
    assert v["moves_closer"] == v["moves"]


def test_poles_in_a_panorama():
    h, w = 512, 1024
    lab = np.full((h, w), 10, np.uint8)          # sky
    lab[h // 2:] = 0                             # road below the horizon
    c = w // 2 + w // 8                          # 45 deg right of the centre
    foot = h // 2 + 40                           # 40 px = 14.06 deg below the horizon
    lab[100:foot + 1, c - 2:c + 2] = SP.CS_POLE
    poles = SP.pano_poles(lab, cam_h=2.0)
    assert len(poles) == 1
    p = poles[0]
    assert p["az"] == pytest.approx(45.0, abs=0.6)
    el = (foot + 1) / h * 180.0 - 90.0
    assert p["dist"] == pytest.approx(2.0 / math.tan(math.radians(el)), rel=1e-6)
    assert p["height"] > 5.0 and SP.pole_class(p) == "lamp"
    hidden = lab.copy()
    hidden[foot + 1:foot + 8, c - 4:c + 4] = 13           # a car at its foot: not counted
    assert SP.pano_poles(hidden, 2.0) == []
    # north (0 deg) moves to -y in CARLA, east to +x
    assert SP.offset_xy(0.0, 0.0, 0.0, 0.0, 10.0) == pytest.approx((0.0, -10.0))
    assert SP.offset_xy(0.0, 0.0, 90.0, 0.0, 10.0) == pytest.approx((10.0, 0.0))


def test_photo_cache_folder_of_a_run(tmp_path):
    import realism_check as RC
    out, mdir = tmp_path / "run", tmp_path / "maps" / "m"
    assert RC.photos_dir(out) == out
    d = RC.set_photos_dir(out, mdir)
    assert d == (mdir / "real" / "streetphotos").resolve() and RC.photos_dir(out) == d
    legacy = tmp_path / "old"
    (legacy / "panos").mkdir(parents=True)
    assert RC.set_photos_dir(legacy, mdir) == legacy and RC.photos_dir(legacy) == legacy
