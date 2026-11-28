"""signage.py and tools/signage_art.py: generated, place-authentic shop signs and ads (no CARLA, no network).

What is checked: every Japanese character drawn is in the bundled font subsets (no missing-glyph boxes), no real brand
is named, the catalogue matches the street-props manifest, faces draw, and the placement puts fascias, banners, signs
and billboards where they belong (on the right wall, at the right height, never inside a building)."""
import collections
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

pytest.importorskip("shapely")
from shapely.geometry import Point, Polygon, box  # noqa: E402

from coupledcarla import scenery, screens, signage, streetlook  # noqa: E402
import signage_art as art  # noqa: E402

# well-known brands, chains and marks that must never appear in generated content (a guard, not a full list)
BRANDS = ["coca", "pepsi", "sony", "uniqlo", "apple", "iphone", "samsung", "nike", "adidas", "toyota", "honda",
          "mcdonald", "starbucks", "tsutaya", "docomo", "softbank", "lawson", "familymart", "7-eleven", "seven-eleven",
          "nintendo", "suntory", "kirin", "asahi", "netflix", "disney", "h&m", "zara", "albert heijn", "jumbo",
          "hema", "mercadona", "migros", "coop", "k-market", "alko", "kruidvat", "etos", "ichiran", "matsumoto",
          "big echo", "shibuya 109", "allerhande", "tsubakiya", "aozora", "día a día", "alhambra",
          "weidegoud", "edelweiss", "shear genius", "petal pushers", "109", "don quijote", "ドン・キホーテ", "マツモトキヨシ", "ユニクロ", "セブン",
          "ローソン", "ファミリーマート", "スターバックス", "マクドナルド", "times square", "broadway"]


def all_texts():
    out = list(art.jp_texts())
    for loc, kinds in art.FASCIA.items():
        for faces in kinds.values():
            out += [t for f in faces for t in f]
    for boards in art.ABOARD.values():
        out += [t for b in boards for t in b]
    for ads in art.ADS.values():
        out += [t for a in ads for t in a[:3]]
    for sets in art.COLUMN.values():
        out += [t for s in sets for p in s for t in p]
    return out


def _cmap(path, index=None):
    from fontTools.ttLib import TTFont
    return set(TTFont(str(path)).getBestCmap())


def test_japanese_glyphs_are_in_the_subsets():
    pytest.importorskip("fontTools")
    chars = {c for t in art.jp_texts() for c in t if c != " "}
    for key, path in art.JP_FONTS.items():
        have = _cmap(path)
        missing = sorted(c for c in chars if ord(c) not in have)
        assert not missing, f"{path.name} lacks {''.join(missing)}: run tools/signage_fonts.py"


def test_latin_glyphs_are_in_dejavu():
    pytest.importorskip("fontTools")
    mpl = pytest.importorskip("matplotlib")
    ttf = Path(mpl.__file__).parent / "mpl-data" / "fonts" / "ttf"
    chars = {c for t in all_texts() if not art._has_jp(t) for c in t if c != " "}
    for key in ("sans", "serif", "serif_it", "light"):
        have = _cmap(ttf / art.LATIN_FONTS[key])
        assert not sorted(c for c in chars if ord(c) not in have), key
    have = _cmap(ttf / art.LATIN_FONTS["times"])      # STIX: the faces it draws (fascias)
    fascia = {c for f in art.FASCIA.values() for faces in f.values() for t in faces for s in t for c in s if c != " "}
    assert not sorted(c for c in fascia if ord(c) not in have)


def test_no_real_brand_is_named():
    import re
    for t in all_texts():
        low = t.lower()
        for b in BRANDS:        # a whole word ("coop", not the "coop" in "bioscoop"); Japanese: anywhere
            pat = re.escape(b) if art._has_jp(b) else r"(?<![a-zà-ÿ])" + re.escape(b) + r"(?![a-zà-ÿ])"
            assert not re.search(pat, low), (b, t)


def test_catalogue_matches_the_manifest_and_palette():
    sp = pytest.importorskip("street_props")
    import json
    data = json.loads((ROOT / "coupledcarla" / "data" / "street_props.json").read_text())
    items = art.catalog()
    names = [f"SM_CC_{it['loc']}_{it['name']}" for it in items]
    assert len(names) == len(set(names)) and len({it["tex"] for it in items}) == len(items)
    for it, n in zip(items, names):
        assert n in data["meshes"], n          # regenerate: python tools/street_props.py OUT --data
        assert it["slot"] in sp.PALETTE and sp.PALETTE[it["slot"]][3] == it["tex"]
        assert (len(sp.PALETTE[it["slot"]]) > 4) == bool(it["lit"])
    # every locale has a fascia for every trade, JP its full set
    for loc in ("NL", "ES", "DE", "FI", "EN"):
        assert signage.variants(set(names), loc, "Fascia", "shop") == [f"SM_CC_{loc}_Fascia_shop_1",
                                                                        f"SM_CC_{loc}_Fascia_shop_2"]
    for fam in ("Kanban_ramen", "Tenant", "Nobori", "Stand", "Billboard", "BillboardTall", "RoofBillboard",
                "Screen_A", "Mupi"):
        assert any(n.startswith(f"SM_CC_JP_{fam}") for n in names), fam


def test_faces_draw(tmp_path):
    pytest.importorskip("PIL")
    pytest.importorskip("matplotlib")
    import numpy as np
    from PIL import Image
    kinds = {}
    for it in art.catalog():
        kinds.setdefault((it["draw"][0], it["loc"] == "JP"), it)
    paths = art.textures(tmp_path, list(kinds.values()))
    for p in paths:
        a = np.asarray(Image.open(p).convert("L"), float)
        assert a.std() > 12, p.name                 # not a blank board
        w, h = Image.open(p).size
        assert w & (w - 1) == 0 and h & (h - 1) == 0, p.name      # power-of-two sides: Unreal mips them
    assert Image.open(tmp_path / "T_CC_sig_jp_screen_a.png").size == (1024, 2048)


def test_locale_and_trade():
    assert signage.locale("jpn") == "JP" and signage.locale("CHE") == "DE" and signage.locale("USA") == "EN"
    assert signage.locale("RWA") is None and signage.locale(None) is None
    assert signage.trade({"amenity": "restaurant", "cuisine": "ramen;japanese"}) == "ramen"
    assert signage.trade({"shop": "shoes"}) == "fashion" and signage.trade({"shop": "car_parts"}) == "shop"


# --- placement -----------------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def cat10():
    return scenery.Catalog("1.0")


class Placer:
    def __init__(self, cat):
        self.cat, self.items, self.mesh_index = cat, [], {}
        self.counts = collections.defaultdict(int)
        self.ground = lambda x, y: 10.0

    def add(self, kind, mesh, cx, cy, yaw, scale, sink=0.0, tilt=None, z=None, prov=None, **kw):
        it = {"kind": kind, "mesh": mesh, "x": cx, "y": cy, "yaw": yaw, "scale": scale, "z": z, "prov": prov}
        self.items.append(it)
        return it


def _face_dir(yaw):
    return math.cos(math.radians(yaw - 90)), math.sin(math.radians(yaw - 90))


def test_fascias_on_their_walls_facing_out(cat10):
    pl = Placer(cat10)
    shop = Polygon([(0, 0), (20, 0), (20, 12), (0, 12)])
    blds = [{"poly": shop, "h": 15.0}]
    pts = [(8.0, -3.0, "cafe", {"amenity": "cafe"}), (9.0, -3.0, "shop", {"shop": "bakery"}),
           (30.0, 30.0, "shop", {"shop": "books"})]          # far from any facade
    placed = signage.fascias(pl, cat10, blds, screens._tree(blds), pts, "NL")
    assert len(placed) == 2 and len(pl.items) == 2
    a, b = pl.items
    assert {a["mesh"].rsplit("_", 1)[0], b["mesh"].rsplit("_", 1)[0]} == {"SM_CC_NL_Fascia_cafe",
                                                                           "SM_CC_NL_Fascia_bakery"}
    for it in pl.items:
        assert it["kind"] == "facade" and it["z"] == pytest.approx(10.0 + 2.9)
        assert -0.2 < it["y"] < 0 and not shop.contains(Point(it["x"], it["y"]))
        assert _face_dir(it["yaw"])[1] == pytest.approx(-1.0, abs=1e-6)     # out of the wall, south
    assert abs(a["x"] - b["x"]) >= 3.2 + signage.FASCIA_GAP - 1e-6          # side by side, not overlapping


def test_japanese_kanban_at_the_shops_floor_and_pavement_signs(cat10):
    pl = Placer(cat10)
    blk = Polygon([(0, 0), (30, 0), (30, 15), (0, 15)])
    blds = [{"poly": blk, "h": 20.0}]
    pts = [(5.0, -2.0, "cafe", {"amenity": "restaurant", "cuisine": "ramen"}),
           (15.0, -2.0, "cafe", {"amenity": "bar", "level": "2"}),
           (25.0, -2.0, "shop", {"shop": "hairdresser", "level": "9"})]       # above the roof: no sign
    tree = screens._tree(blds)
    placed = signage.fascias(pl, cat10, blds, tree, pts, "JP")
    assert [p[4] for p in placed] == ["ramen", "izakaya"]
    ramen, bar = pl.items
    assert ramen["mesh"].startswith("SM_CC_JP_Kanban_ramen_") and ramen["z"] == pytest.approx(10.0 + 2.6)
    assert bar["mesh"].startswith("SM_CC_JP_Kanban_izakaya_") and bar["z"] == pytest.approx(10.0 + 2.6 + 3.4 * 2)
    n0 = len(pl.items)
    got = signage.pavement_signs(pl, cat10, placed, "JP")
    assert got.get("stand", 0) >= 1                      # the upstairs bar advertises at the street
    for it in pl.items[n0:]:
        assert it["kind"] == "furniture" and it["z"] is None and -1.0 < it["y"] < -0.3
        assert not blk.contains(Point(it["x"], it["y"]))
    n = signage.tenant_signs(pl, cat10, blds, tree, pts, {})
    assert n == 1
    t = pl.items[-1]
    assert t["mesh"].startswith("SM_CC_JP_Tenant_") and t["z"] == pytest.approx(10.0 + signage.TENANT_BOTTOM)
    top = t["z"] + 2 * cat10.by_name[t["mesh"]][1]["ext"][2] * t["scale"]
    assert top <= 10.0 + 20.0 - 0.3 + 1e-6


def test_billboards_on_tall_dense_walls_and_roofs(cat10):
    pl = Placer(cat10)
    tall = {"poly": box(0, 0, 30, 20), "h": 40.0}
    low = {"poly": box(60, 0, 90, 20), "h": 18.0}
    blds = [tall, low]
    pts = [(x, -3.0, "shop", {"shop": "clothes"}) for x in range(0, 95, 6)]
    tree = screens._tree(blds)
    dens = signage.Density(pts)
    cands = signage._wall_candidates(blds, tree, dens, 15.0, 8)
    assert cands and all(c[1] in (0, 1) for c in cands)
    n = signage.wall_billboards(pl, cat10, blds, tree, cands, "JP", 10)
    assert n == 2
    for it in pl.items:
        b = tall if it["x"] < 45 else low
        assert b["poly"].exterior.distance(Point(it["x"], it["y"])) < 0.5 and not b["poly"].contains(
            Point(it["x"], it["y"]))
        hs = 2 * cat10.by_name[it["mesh"]][1]["ext"][2] * it["scale"]
        assert it["z"] >= 10.0 + 6.0 and it["z"] + hs <= 10.0 + b["h"] - 1.0 + 1e-6
    pl2 = Placer(cat10)
    signage.ROOF["most"], old = 5, signage.ROOF["most"]
    try:
        blds2 = [{"poly": box(100 * k, 0, 100 * k + 25, 18), "h": 20.0} for k in range(12)]
        pts2 = [(100 * k + x, -3.0, "shop", {}) for k in range(12) for x in range(0, 25, 3)]
        n = signage.roof_billboards(pl2, cat10, blds2, screens._tree(blds2), signage.Density(pts2))
    finally:
        signage.ROOF["most"] = old
    assert n >= 1
    for it in pl2.items:
        b = blds2[int(it["x"] // 100)]
        assert b["poly"].contains(Point(it["x"], it["y"])) and it["z"] == pytest.approx(10.0 + 20.0)


def test_add_to_scenery_gates_and_screens_in_the_local_language(cat10):
    blds = [{"poly": box(-60, -20, -20, 20), "h": 30.0}, {"poly": box(20, -10, 40, 10), "h": 40.0}]
    pts = [(-18.0, 0.0, "cafe", {"name": "x", "amenity": "cafe"})]
    for iso3, mc, sc in (("JPN", {}, {}), ("JPN", {"carla_level": "X"}, {"signage": False}), ("RWA", {"carla_level": "X"}, {})):
        pl = Placer(cat10)
        assert signage.add_to_scenery(pl, cat10, blds, pts, iso3, mc, sc) == ([], False) and not pl.items
    pl = Placer(cat10)
    said, fascias = signage.add_to_scenery(pl, cat10, blds, pts, "JPN", {"carla_level": "X"}, {})
    assert fascias and said and pl.counts["signage_fascia"] == 1
    pl = Placer(cat10)
    n = screens.place_screens(pl, cat10, [dict(b) for b in blds], screens._tree(blds), 0.0, 0.0, 90.0, 6, loc="JP")
    assert n == 2 and all(it["mesh"].startswith("SM_CC_JP_Screen") for it in pl.items)
    pl = Placer(cat10)
    screens.place_screens(pl, cat10, [dict(b) for b in blds], screens._tree(blds), 0.0, 0.0, 90.0, 6, loc="NL")
    assert all(it["mesh"].startswith("SM_CC_Any_Screen") for it in pl.items)     # no Dutch screen set: generic


def test_mapped_advertising_shows_local_content(cat10):
    look = streetlook.Chooser(cat10, "ESP", (40.9, -5.7, 41.0, -5.6))
    assert look.furniture("advertising", "poster_box", {}, uid=7).startswith("SM_CC_ES_Mupi_")
    assert look.furniture("advertising", "column", {}, uid=7).startswith("SM_CC_ES_AdColumn_")
    assert look.furniture("advertising", "billboard", {}, uid=7).startswith("SM_CC_ES_BillboardStand_")
    rw = streetlook.Chooser(cat10, "RWA", (0, 0, 1, 1))
    assert rw.furniture("advertising", "column", {}, uid=7).startswith("SM_CC_Any_AdColumn_")
    assert rw.furniture("advertising", "poster_box", {}, uid=7) is None        # CARLA's own light box
