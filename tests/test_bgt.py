"""The Dutch BGT: surface kinds, the fetch (monkeypatched, no network), ground tiles and furniture merging."""
import json

import pytest

from coupledcarla import bgt


def test_surface_kinds():
    assert bgt.surface_kind("begroeidterreindeel", None, "groenvoorziening") == "green"
    assert bgt.surface_kind("begroeidterreindeel", None, "loofbos") == "green"
    assert bgt.surface_kind("waterdeel", None, None) == "water"
    assert bgt.surface_kind("wegdeel", "voetpad", "open verharding") == "open"
    assert bgt.surface_kind("wegdeel", "fietspad", "gesloten verharding") == "closed"
    assert bgt.surface_kind("wegdeel", "parkeervlak", "half verhard") == "unpaved"
    assert bgt.surface_kind("wegdeel", "rijbaan lokale weg", "gesloten verharding") == "road"
    assert bgt.surface_kind("wegdeel", "OV-baan", "gesloten verharding") == "road"
    assert bgt.surface_kind("ondersteunendwegdeel", "berm", "groenvoorziening") == "green"
    assert bgt.surface_kind("onbegroeidterreindeel", None, "erf") == "open"
    assert bgt.surface_kind("onbegroeidterreindeel", None, "zand") == "unpaved"
    assert bgt.surface_kind("onbegroeidterreindeel", None, "something new") is None


def test_plus_tags():
    assert bgt.tags_of("paal", "lichtmast") == {"highway": "street_lamp"}
    assert bgt.tags_of("kast", None) == {"man_made": "street_cabinet"}
    assert bgt.tags_of("kast", "elektrakast") == {"man_made": "street_cabinet"}    # any kast
    assert bgt.tags_of("paal", "verkeersregelinstallatiepaal") is None              # SUMO draws the lights
    assert bgt.tags_of("put", "brandkraan / -put") is None                          # underground in NL
    assert bgt.kind_of({"amenity": "bench", "backrest": "yes"}) == "amenity=bench"
    assert bgt.kind_of({"amenity": "cafe"}) is None


class FakeGeo:
    """Lon/lat are metres here: x = lon, y = lat."""

    def to_carla(self, lat, lon):
        return lon, lat


def _sq(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]


shapely = pytest.importorskip("shapely")


def test_classify_tiles():
    data = {"surfaces": [
        {"c": "wegdeel", "f": "rijbaan lokale weg", "v": "gesloten verharding", "r": [_sq(0, 0, 40, 10)]},
        {"c": "wegdeel", "f": "voetpad", "v": "open verharding", "r": [_sq(0, 10, 40, 16)]},
        {"c": "begroeidterreindeel", "f": None, "v": "groenvoorziening", "r": [_sq(0, 16, 40, 40)]},
        {"c": "begroeidterreindeel", "f": None, "v": "loofbos", "r": [_sq(100, 100, 140, 140)]},
        {"c": "onbegroeidterreindeel", "f": None, "v": "onbekend", "r": [_sq(200, 200, 240, 240)]},
    ]}
    s = bgt.Surfaces(data, FakeGeo())
    assert len(s) == 4 and len(s.wood) == 1 and s.wood[0][1] == "wood"
    kind, share = s.classify(20, 20, 40)          # road left out: 6 m of pavement, 24 m of green
    assert kind == "green" and share == pytest.approx(24 / 30)
    assert s.classify(20, 5, 10) == (None, 0.0)   # only road
    assert s.classify(500, 500, 40) == (None, 0.0)  # no BGT here (outside the fetched box): OSM decides
    assert s.classify(220, 220, 40) == (None, 0.0)  # an unknown look is not a kind


def test_merge_replaces_nearby_osm():
    data = {"objects": [
        {"c": "straatmeubilair", "y": "bank", "lon": 0.0, "lat": 0.0},
        {"c": "straatmeubilair", "y": "bank", "lon": 0.3, "lat": 0.0},        # a duplicate 0.3 m away
        {"c": "straatmeubilair", "y": "fietsenrek", "lon": 50.0, "lat": 0.0},
        {"c": "straatmeubilair", "y": "fietsenrek", "lon": 52.0, "lat": 0.0},  # within 4 m of the last rack
        {"c": "straatmeubilair", "y": "fietsenrek", "lon": 57.0, "lat": 0.0},
        {"c": "paal", "y": "lichtmast", "lon": 100.0, "lat": 0.0},
        {"c": "vegetatieobject_punt", "y": "boom", "lon": 150.0, "lat": 0.0},
        {"c": "put", "y": "kolk", "lon": 160.0, "lat": 0.0},
    ]}
    osm = [
        {"type": "node", "id": 1, "lat": 0.0, "lon": 2.0, "tags": {"amenity": "bench"}},       # replaced
        {"type": "node", "id": 2, "lat": 0.0, "lon": 10.0, "tags": {"amenity": "bench"}},      # kept
        {"type": "node", "id": 3, "lat": 0.0, "lon": 1.0, "tags": {"amenity": "waste_basket"}},  # other kind
        {"type": "node", "id": 4, "lat": 0.0, "lon": 101.0, "tags": {"highway": "street_lamp",
                                                                    "source": "eindhoven-openbare-verlichting"}},
        {"type": "way", "id": 5, "tags": {"leisure": "park"}, "geometry": []},
    ]
    els, st = bgt.merge(osm, data, FakeGeo(), skip_lamps=True, skip_trees=False)
    ids = {e["id"] for e in els}
    assert ids >= {2, 3, 4, 5} and 1 not in ids
    added = [e for e in els if e["id"] < 0]
    kinds = sorted(bgt.kind_of(e["tags"]) for e in added)
    assert kinds == ["amenity=bench", "amenity=bicycle_parking", "amenity=bicycle_parking", "natural=tree"]
    assert st["replaced"] == {"amenity=bench": 1} and st["skipped_register"] == 1 and st["too_close"] == 2
    assert all(e["tags"]["source"].startswith("bgt-") for e in added)


def _page(features, nxt=None):
    links = [{"rel": "self", "href": "x"}] + ([{"rel": "next", "href": nxt}] if nxt else [])
    return {"type": "FeatureCollection", "features": features, "links": links}


def test_fetch_pages_filters_and_caches(tmp_path, monkeypatch):
    pytest.importorskip("pyproj")
    # RD New around Eindhoven station; a 10 m square of green, its copy from another quarter, a viaduct deck
    sq = [[161400.0, 383600.0], [161410.0, 383600.0], [161410.0, 383610.0], [161400.0, 383610.0],
          [161400.0, 383600.0]]
    green = {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [sq]},
             "properties": {"lokaal_id": "G1", "fysiek_voorkomen": "groenvoorziening", "relatieve_hoogteligging": 0}}
    deck = {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [sq]},
            "properties": {"lokaal_id": "W9", "functie": "voetpad", "fysiek_voorkomen": "open verharding",
                           "relatieve_hoogteligging": 1}}
    bench = {"type": "Feature", "geometry": {"type": "Point", "coordinates": [161432.4, 383677.3]},
             "properties": {"lokaal_id": "S1", "plus_type": "bank", "relatieve_hoogteligging": 0}}
    calls = []

    class Resp:
        def __init__(self, d):
            self.d = d

        def raise_for_status(self):
            pass

        def json(self):
            return self.d

    def fake_get(url, params=None, headers=None, timeout=None):
        calls.append((url, params))
        assert headers["User-Agent"]
        if params is not None:
            assert params["filter"] == "eind_registratie IS NULL" and params["crs"].endswith("28992")
        coll = url.split("/collections/")[1].split("/")[0]
        if coll == "begroeidterreindeel":
            return Resp(_page([green], nxt=url + "?cursor=2") if params else _page([]))
        if coll == "wegdeel":
            return Resp(_page([deck]))
        if coll == "straatmeubilair":
            return Resp(_page([bench]))
        return Resp(_page([]))

    monkeypatch.setattr(bgt.requests, "get", fake_get)
    path = tmp_path / "real" / "bgt.json"
    data = bgt.fetch([51.436, 5.47, 51.45, 5.493], path, {"User-Agent": "test"}, log=lambda m: None)
    assert len(data["surfaces"]) == 1 and data["surfaces"][0]["v"] == "groenvoorziening"   # one copy, no deck
    assert data["counts"] == {"begroeidterreindeel": 1, "straatmeubilair": 1}
    lon, lat = data["surfaces"][0]["r"][0][0]
    assert 5.47 < lon < 5.49 and 51.43 < lat < 51.45
    assert data["objects"] == [{"c": "straatmeubilair", "y": "bank", "lon": pytest.approx(5.4797, abs=1e-3),
                                "lat": pytest.approx(51.4422, abs=1e-3)}]
    assert json.loads(path.read_text())["counts"] == data["counts"]

    def offline(*a, **k):
        raise AssertionError("the cache should answer")
    monkeypatch.setattr(bgt.requests, "get", offline)
    assert bgt.fetch([51.436, 5.47, 51.45, 5.493], path, {"User-Agent": "test"})["counts"] == data["counts"]


def test_scenery_falls_back(tmp_path, monkeypatch):
    from coupledcarla import scenery
    msgs = []
    mc = {"bbox": [51.436, 5.47, 51.45, 5.493]}

    def broken(*a, **k):
        raise RuntimeError("PDOK down")
    monkeypatch.setattr(bgt, "fetch", broken)
    assert scenery._bgt(mc, {}, tmp_path, msgs.append) is None and "BGT failed" in msgs[-1]
    assert scenery._bgt(mc, {"bgt": False}, tmp_path, msgs.append) is None and len(msgs) == 1
    assert scenery._bgt({"bbox": [45.75, 4.82, 45.76, 4.84]}, {}, tmp_path, msgs.append) is None   # Lyon
    assert len(msgs) == 1


def test_tile_cover():
    from coupledcarla.scenery import tile_cover
    assert tile_cover((80.0, 80.0, 0.0), 40.0) == [((0.0, 0.0), 0.5)]              # grass: one plane
    assert tile_cover((1.0, 1.0, 1.0), 20.0) == [((0.0, 0.0), 20.0)]               # paving cube
    two = tile_cover((11.7, 19.4, 0.0), 40.0)                                      # concrete: two side by side
    s = two[0][1]
    assert len(two) == 2 and s == pytest.approx(40 / 19.4)
    assert two[0][0][0] - 11.7 * s / 2 == pytest.approx(-20) and two[-1][0][0] + 11.7 * s / 2 == pytest.approx(20)
    three = tile_cover((34.8, 80.0, 0.0), 40.0)                                    # earth: three
    assert len(three) == 3 and all(dy == 0.0 for (_, dy), _ in three)
