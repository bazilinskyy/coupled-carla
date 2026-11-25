"""coupledcarla/screens.py: big screens and vertical shop signs on facades (no CARLA); the snap's facade rule."""
from shapely.geometry import Point, box

from coupledcarla import screens
from tools import snap_scenery as ss


class _Cat:
    """The catalogue's lookups: the round-3 street props with their bounding boxes (tools/street_props.py)."""
    def __init__(self):
        self.by_name = {f"SM_CC_Any_Screen_{k}": (0, {"off": [0, -0.06, 0], "ext": [2.04, 0.06, 1.165]})
                        for k in "ABCD"}
        self.by_name.update({f"SM_CC_Any_ScreenTall_{k}": (0, {"off": [0, -0.06, 0], "ext": [1.165, 0.06, 2.04]})
                             for k in "ABCD"})
        self.by_name.update({f"SM_CC_JP_SignV_{k}": (0, {"off": [0, -0.425, 0], "ext": [0.061, 0.425, 1.8]})
                             for k in ("izakaya", "ramen", "sushi", "shokudo", "cafe", "karaoke", "kusuri", "biyo",
                                       "shoten", "fuku", "mise")})

    def __contains__(self, name):
        return name in self.by_name


class _Pl:
    def __init__(self):
        self.items, self.counts = [], {}

    def ground(self, x, y):
        return 10.0

    def add(self, kind, mesh, x, y, yaw, scale, z=None, prov=None):
        it = {"kind": kind, "mesh": mesh, "x": x, "y": y, "yaw": yaw, "scale": scale, "z": z, "prov": prov}
        self.items.append(it)
        return it


class _Geo:
    def to_carla(self, lat, lon):
        return lon, lat


def _blds():
    # a 30 m tall block west of a square and a 40 m tower east of it, a low kiosk north
    return [{"poly": box(-60, -20, -20, 20), "h": 30.0}, {"poly": box(20, -10, 40, 10), "h": 40.0},
            {"poly": box(-5, 30, 5, 34), "h": 4.0}]


def test_sign_words_and_floors():
    assert screens.sign_key({"amenity": "restaurant", "cuisine": "ramen;japanese"}) == "ramen"
    assert screens.sign_key({"amenity": "bar"}) == "izakaya" and screens.sign_key({"shop": "books"}) == "shoten"
    assert screens.sign_key({"shop": "electronics"}) == "mise"
    assert screens.floor_of({"level": "3"}) == 3 and screens.floor_of({"level": "-1"}) == 0
    assert screens.floor_of({"addr:floor": "2;3"}) == 2 and screens.floor_of({}) == 0


def test_vertical_signs_hang_on_the_facade_at_their_floor():
    pl, cat, blds = _Pl(), _Cat(), _blds()
    pts = [(-18.0, 0.0, "cafe", {"name": "x", "amenity": "cafe", "level": "2"}),
           (-18.0, 0.5, "shop", {"name": "y", "shop": "books"}),           # same wall: stepped along it
           (0.0, 31.0, "shop", {"name": "z", "shop": "books"}),             # the 4 m kiosk: too low for a sign
           (-18.0, 5.0, "shop", {"shop": "books"})]                         # no name: no sign
    n = screens.vertical_signs(pl, cat, blds, screens._tree(blds), pts)
    assert n == 2
    a, b = pl.items
    assert a["mesh"] == "SM_CC_JP_SignV_cafe" and b["mesh"] == "SM_CC_JP_SignV_shoten" and a["kind"] == "facade"
    for it in (a, b):        # the board's centre out of the wall (x = -20), facing along +x
        assert -19.6 < it["x"] < -19.4 and not blds[0]["poly"].contains(Point(it["x"], it["y"]))
    assert abs(a["y"] - b["y"]) >= 1.2
    assert a["z"] - 10.0 == 3.0 + 3.4 * 2 and b["z"] - 10.0 == 3.0     # bottom: 3 m + its floor


def test_screens_around_a_place_face_it_from_tall_walls():
    pl, cat, blds = _Pl(), _Cat(), _blds()
    n = screens.place_screens(pl, cat, blds, screens._tree(blds), 0.0, 0.0, 90.0, 6)
    assert n == 2
    for it in pl.items:
        b = next(b for b in blds if b["poly"].exterior.distance(Point(it["x"], it["y"])) < 0.5)
        assert b["h"] >= screens.HOT_MIN_H and not b["poly"].contains(Point(it["x"], it["y"]))
        assert 10.0 + 6.0 <= it["z"] and it["z"] + 4.0 * it["scale"] * 9 / 16 <= 10.0 + b["h"] - 1.5 + 1e-6
        assert it["prov"] == ("inferred", "screens")
    assert sorted(round(it["x"]) for it in pl.items) == [-20, 20]      # the walls facing the place, not the far sides
    # a wall facing away, or a place out of reach: none
    pl2 = _Pl()
    assert screens.place_screens(pl2, cat, blds, screens._tree(blds), 0.0, 300.0, 90.0, 6) == 0 and not pl2.items


def test_the_listed_places():
    names = {p["name"] for p in screens.places()}
    assert {"Shibuya scramble crossing", "Times Square"} <= names


def test_mapped_screens_and_the_gates(tmp_path):
    import json
    (tmp_path / "real").mkdir()
    (tmp_path / "real" / "osm_scenery.json").write_text(json.dumps({"elements": [
        {"type": "node", "id": 1, "lat": 0.0, "lon": 21.0, "tags": {"advertising": "screen", "width": "12"}},
        {"type": "node", "id": 2, "lat": 0.0, "lon": -21.0, "tags": {"advertising": "billboard"}}]}))
    blds, cat = _blds(), _Cat()
    pl = _Pl()
    said = screens.add_to_scenery(pl, cat, blds, [], "NL", {"carla_level": "X", "bbox": [5, 5, 6, 6]}, {}, tmp_path,
                                  _Geo())
    assert len(pl.items) == 1 and pl.items[0]["mesh"].startswith("SM_CC_Any_Screen") and said == ["1 mapped screens"]
    assert abs(pl.items[0]["scale"] - 12.0 / screens.BASE_W) < 1e-9 and pl.items[0]["prov"] == ("register", "osm")
    for mc, sc in (({}, {}), ({"carla_level": "X"}, {"screens": False})):     # not a twin, or turned off
        pl = _Pl()
        assert screens.add_to_scenery(pl, cat, blds, [], "JP", mc, sc, tmp_path, _Geo()) == [] and not pl.items


class _SnapCat:
    by_name = {"Cube": (0, {"off": [0, 0, 0.5], "ext": [0.5, 0.5, 0.5]}),
               "Screen": (1, {"off": [0, -0.06, 0], "ext": [2.0, 0.06, 1.1]})}

    def __contains__(self, name):
        return name in self.by_name


def test_facade_items_keep_their_height_over_the_moved_ground():
    # the terrain model planned everything 19.5 m too high (Shibuya): the benches go onto the ground, the screen on
    # the wall moves with them, its own ray (hitting the building) is not asked
    lvl = lambda x, y, z0, z1: sorted(((z, lb) for z, lb in ((0.0, "sidewalks"), (40.0, "buildings"))  # noqa: E731
                                       if min(z0, z1) <= z <= max(z0, z1)), reverse=True)
    items = [[0, float(x), 0.0, 19.5, 0, 1.0, "furniture"] for x in range(0, 20, 2)] + \
            [[1, 5.0, 1.0, 19.5 + 20.0, 90, 2.0, "facade"]]
    d = {"meshes": ["/Game/X/Cube.Cube", "/Game/X/Screen.Screen"], "items": items}
    res = ss.snap_items(d, _SnapCat(), lvl)
    assert all(it[3] == 0.0 for it in d["items"][:-1])
    assert d["items"][-1][3] == 20.0 and res["facade_items_moved"] == 1


def test_round3_props_and_their_textures(tmp_path):
    """The screens' and signs' meshes face out of the wall; their textures come from git without a CJK font."""
    from tools import street_props as sp
    sp.textures3(tmp_path)          # copies unreal/StreetProps/textures (no font needed)
    for k in sp.SCREEN_KEYS:
        assert (tmp_path / f"T_CC_screen_{k}.png").exists()
    for k in sp.SIGNV_WORDS:
        assert (tmp_path / f"T_CC_signv_{k}.png").exists()
    pal = sp.PALETTE["Screen_A"]
    assert pal[3] == "screen_a" and pal[4] > 0 and pal[6] > 0          # texture, emissive, a vertical pan
    m, extra = sp.blade_sign("SignV_ramen")
    v = sp.np.concatenate([sp.np.concatenate(p[0]).reshape(-1, 3) for p in m.parts.values()])
    assert v[:, 1].min() >= -1e-6 and abs(v[:, 1].max() - extra["out"]) < 1e-6     # all out of the wall (+y)
    m, extra = sp.screen("Screen_A")
    normals = sp.np.concatenate(m.parts["Screen_A"][1]).reshape(-1, 3)
    assert (normals[:, 1] > 0).all() and abs(extra["width"] - sp.screen.__defaults__[0]) < 1e-9   # faces out
