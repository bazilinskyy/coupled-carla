"""The map maker (coupledcarla/mapmaker.py, mapmaker_web.py): geocoding, guesses, experiment files, progress, errors and
the local web page. No network, no SUMO, no CARLA."""
import json
import math
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest
import yaml

from coupledcarla import mapmaker as mm
from coupledcarla import mapmaker_web as web

REPO = Path(__file__).resolve().parent.parent

NOMINATIM_SEOUL = {
    "lat": "37.5634", "lon": "126.9846", "category": "place", "type": "quarter", "addresstype": "quarter",
    "name": "명동", "display_name": "명동, 중구, 서울, 대한민국",
    "namedetails": {"name": "명동", "name:en": "Myeong-dong"},
    "address": {"quarter": "명동", "city": "Seoul", "country": "South Korea", "country_code": "kr"},
    "extratags": {},
}
NOMINATIM_SALAMANCA = {
    "lat": "40.9650", "lon": "-5.6640", "category": "highway", "type": "pedestrian", "addresstype": "road",
    "name": "Plaza Mayor", "display_name": "Plaza Mayor, Salamanca, Castilla y León, España",
    "namedetails": {"name": "Plaza Mayor"},
    "address": {"road": "Plaza Mayor", "city": "Salamanca", "country": "Spain", "country_code": "es"},
    "extratags": {},
}


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    """A cache and a project folder of our own, and no network."""
    monkeypatch.setattr(mm, "CACHE", tmp_path / "cache" / "geocode.json")
    root = tmp_path / "proj"
    (root / "experiments" / "cities").mkdir(parents=True)
    (root / "experiments" / "cities.yaml").write_text((REPO / "experiments" / "cities.yaml").read_text())
    monkeypatch.setattr(mm, "ROOT", root)
    monkeypatch.setattr(mm, "PLACES", root / "experiments" / "places")
    monkeypatch.setattr(mm, "_last", [0.0])

    def no_network(url, params, timeout=20):
        raise AssertionError(f"unexpected request to {url}")
    monkeypatch.setattr(mm, "_fetch_json", no_network)
    return root


def fake_fetch(monkeypatch, answers, calls=None):
    """answers: {url substring: answer or exception}."""
    def fetch(url, params, timeout=20):
        if calls is not None:
            calls.append((url, dict(params)))
        for key, ans in answers.items():
            if key in url:
                if isinstance(ans, Exception):
                    raise ans
                return ans(params) if callable(ans) else ans
        raise AssertionError(url)
    monkeypatch.setattr(mm, "_fetch_json", fetch)


# ---------------------------------------------------------------- parsing and guesses

def test_parse_coords():
    assert mm.parse_coords("51.44, 5.48") == (51.44, 5.48)
    assert mm.parse_coords("51.44 5.48") == (51.44, 5.48)
    assert mm.parse_coords("-33.93;18.42") == (-33.93, 18.42)
    assert mm.parse_coords("33.9 S, 18.4 E") == (-33.9, 18.4)
    assert mm.parse_coords("40.75N 73.98W") == (40.75, -73.98)
    assert mm.parse_coords("Markt, Eindhoven") is None
    assert mm.parse_coords("91, 5") is None and mm.parse_coords("") is None
    assert mm.parse_coords("1600 Pennsylvania Avenue") is None


def test_slugify():
    assert mm.slugify("Plaza Mayor, Salamanca") == "plaza_mayor_salamanca"
    assert mm.slugify("São Paulo (Sé)") == "sao_paulo_se"
    assert mm.slugify("Straße am Großmarkt") == "strasse_am_grossmarkt"
    assert mm.slugify("명동") == "place"                       # no Latin letters: the fallback
    assert mm.slugify("渋谷", fallback="shibuya") == "shibuya"
    assert mm.slugify("5th Avenue") == "place_5th_avenue"     # a name, not a number
    assert len(mm.slugify("x" * 100)) <= 40


def test_sizes_and_box():
    assert mm.size_km("small") == 1.0 and mm.size_km("Medium") == 1.4 and mm.size_km("large") == 2.0
    assert mm.size_km("1.2") == 1.2 and mm.size_km("1.8 km") == 1.8 and mm.size_km(3) == 3.0
    for bad in ("huge", "0.1", "9"):
        with pytest.raises(mm.MapMakerError) as e:
            mm.size_km(bad)
        assert e.value.advice
    assert mm.size_name(1.4) == "medium" and mm.size_name(1.2) == "1.2 km"
    s, w, n, e = mm.bbox(51.44, 5.48, 1.4)
    assert abs((n - s) * 111.32 - 1.4) < 0.01
    assert abs((e - w) * 111.32 * math.cos(math.radians(51.44)) - 1.4) < 0.01
    assert abs((s + n) / 2 - 51.44) < 1e-4 and abs((w + e) / 2 - 5.48) < 1e-4
    from coupledcarla import cities             # the same box as the 19 cities
    assert mm.bbox(-33.9346, 18.8610, 1.4) == cities.bbox(-33.9346, 18.8610, 1.4)
    assert mm.estimate_minutes(2.0) > mm.estimate_minutes(1.4) > mm.estimate_minutes(1.0) >= 1


def test_driving_side():
    for cc in ("GB", "IE", "JP", "IN", "KE", "ZA", "AU", "NZ", "TH", "MT", "CY", "jm", "VI", "WS"):
        assert mm.driving_side(cc) == "left", cc
    for cc in ("US", "NL", "FR", "KR", "CN", "BR", "EG", "RW", "AS", "VU", None, ""):
        assert mm.driving_side(cc) == "right", cc
    codes = {"Egypt": "EG", "Rwanda": "RW", "South Africa": "ZA", "Japan": "JP", "Taiwan": "TW", "Laos": "LA",
             "United Kingdom": "GB", "France": "FR", "Belgium": "BE", "USA": "US", "Brazil": "BR", "Uruguay": "UY",
             "Argentina": "AR", "Australia": "AU", "New Zealand": "NZ", "New Caledonia": "NC"}
    for c in yaml.safe_load((REPO / "experiments" / "cities.yaml").read_text())["cities"]:
        assert mm.driving_side(codes[c["country"]]) == c["drive"], c["name"]
        assert mm.CONTINENT[codes[c["country"]]] == c["continent"], c["name"]


def test_town_class_matches_the_cities():
    for c in yaml.safe_load((REPO / "experiments" / "cities.yaml").read_text())["cities"]:
        assert mm.town_class(c["population"])[0] == c["size"], c["name"]
    assert mm.town_class(None, "village")[0] == "small"
    assert mm.town_class(None, "city")[0] == "medium"
    assert mm.town_class(None, None) == ("medium", "size unknown, a medium city assumed")
    assert mm.parse_population("9,586,195") == 9586195
    assert mm.parse_population("1 234 567") == 1234567
    assert mm.parse_population("12000;12500") == 12000
    assert mm.parse_population("~5000") == 5000
    assert mm.parse_population("unknown") is None and mm.parse_population(None) is None


def test_traffic_follows_cities_yaml_and_box():
    cat = yaml.safe_load((REPO / "experiments" / "cities.yaml").read_text())
    t = mm.traffic_for("medium", 1.4)
    assert t["cars_per_hour"] == cat["size_traffic"]["medium"]["cars_per_hour"]
    assert t["bikes_per_hour"] == cat["defaults"]["traffic"]["bikes_per_hour"]
    big = mm.traffic_for("large", 2.0)
    assert big["cars_per_hour"] > cat["size_traffic"]["large"]["cars_per_hour"]
    small = mm.traffic_for("small", 1.0)
    assert small["cars_per_hour"] < cat["size_traffic"]["small"]["cars_per_hour"] and small["buses_per_hour"] >= 1


# ---------------------------------------------------------------- geocoding

def test_search_nominatim_non_latin(monkeypatch):
    calls = []
    fake_fetch(monkeypatch, {"nominatim.openstreetmap.org/search": [NOMINATIM_SEOUL]}, calls)
    res = mm.search("명동, 서울")
    p = res[0]
    assert p["name"] == "Myeong-dong, Seoul" and p["country_code"] == "KR" and p["country"] == "South Korea"
    assert p["lat"] == 37.5634 and p["settlement"] == "Seoul" and p["source"] == "OpenStreetMap Nominatim"
    url, params = calls[0]
    assert params["q"] == "명동, 서울" and params["format"] == "jsonv2" and params["accept-language"] == "en"
    assert not any("@" in str(v) for v in params.values())          # nothing personal
    assert mm.search("명동,   서울")[0]["name"] == p["name"] and len(calls) == 1   # cached (same text)
    assert json.loads(mm.CACHE.read_text())


def test_search_falls_back_to_photon(monkeypatch):
    photon = {"features": [{"geometry": {"coordinates": [-5.664, 40.965]},
                            "properties": {"name": "Plaza Mayor", "city": "Salamanca", "country": "Spain",
                                           "countrycode": "ES", "osm_key": "highway", "type": "street"}}]}
    fake_fetch(monkeypatch, {"nominatim": ConnectionError("down"), "photon.komoot.io/api": photon})
    p = mm.search("Plaza Mayor, Salamanca")[0]
    assert p["name"] == "Plaza Mayor, Salamanca" and p["country_code"] == "ES" and p["lon"] == -5.664
    assert p["source"].startswith("Photon")


def test_search_errors_in_plain_words(monkeypatch):
    fake_fetch(monkeypatch, {"nominatim": [], "photon": {"features": []}})
    with pytest.raises(mm.MapMakerError) as e:
        mm.search("Xyzzy Plugh")
    assert "could not find" in str(e.value) and "spelling" in e.value.advice
    fake_fetch(monkeypatch, {"nominatim": ConnectionError("x"), "photon": ConnectionError("y")})
    with pytest.raises(mm.MapMakerError) as e:
        mm.search("Somewhere else")
    assert "did not answer" in str(e.value) and "internet" in e.value.advice
    with pytest.raises(mm.MapMakerError):
        mm.search("   ")


def test_coordinates_are_reverse_geocoded(monkeypatch):
    rev = dict(NOMINATIM_SALAMANCA, lat="40.96501", lon="-5.66399")
    fake_fetch(monkeypatch, {"nominatim.openstreetmap.org/reverse": rev})
    p = mm.search("40.965, -5.664")[0]
    assert (p["lat"], p["lon"]) == (40.965, -5.664) and p["country_code"] == "ES"
    fake_fetch(monkeypatch, {"nominatim": ConnectionError("x"), "photon": ConnectionError("y")})
    p = mm.reverse(10.0, 20.0)                 # offline: the bare point, country unknown
    assert p["country_code"] is None and p["source"] == "coordinates"
    monkeypatch.setattr(mm, "nearby_places", lambda lat, lon: None)
    assert mm.plan(found=mm.reverse(45.0, -30.0))["slug"] == "point_45_000n_30_000w"


def test_population_from_the_town(monkeypatch):
    def search(params):
        if params.get("featureType") == "settlement":
            return [{"lat": "37.5666", "lon": "126.9782", "extratags": {"population": "9,586,195"}}]
        return [NOMINATIM_SEOUL]
    fake_fetch(monkeypatch, {"nominatim.openstreetmap.org/search": search})
    p = mm.plan("명동, 서울")
    assert p["population"] == 9586195 and p["town"] == "large" and p["drive"] == "right"
    assert p["continent"] == "Asia" and p["slug"] == "myeong_dong_seoul"
    assert p["traffic"]["cars_per_hour"] == 3000


def test_population_of_a_city_state_but_not_a_far_namesake(monkeypatch):
    delhi = {"lat": "28.6315", "lon": "77.2167", "category": "place", "type": "suburb", "addresstype": "suburb",
             "name": "Connaught Place", "display_name": "Connaught Place, New Delhi, Delhi, India",
             "address": {"suburb": "Connaught Place", "city": "New Delhi", "state_district": "New Delhi",
                         "state": "Delhi", "country": "India", "country_code": "in"}}
    towns = {"New Delhi, India": [{"lat": "28.6139", "lon": "77.2090", "extratags": {"population": "249998"}}],
             "Delhi, India": [{"lat": "28.6517", "lon": "77.2219", "extratags": {"population": "16787941"}}],
             "Spokane, United States": [{"lat": "47.66", "lon": "-117.42", "extratags": {"population": "228989"}}],
             "Washington, United States": [{"lat": "38.89", "lon": "-77.03", "extratags": {"population": "689545"}}]}

    def search(params):
        if params.get("featureType") == "settlement":
            return towns.get(params["q"], [])
        return [delhi]
    fake_fetch(monkeypatch, {"nominatim.openstreetmap.org/search": search})
    p = mm.plan("Connaught Place, New Delhi")
    assert p["population"] == 16787941 and p["town"] == "large" and p["drive"] == "left"
    spokane = {"name": "Riverfront Park", "lat": 47.661, "lon": -117.42, "country": "United States",
               "country_code": "US", "settlement": "Spokane", "areas": ["Spokane County", "Washington"],
               "population": None, "kind": "park", "full": "x", "source": "test"}
    assert mm.settlement_population(spokane) == 228989          # Washington, D.C. is 3,000 km away


def test_a_big_city_nearby_counts(monkeypatch):
    shibuya = {"name": "Shibuya Scramble Crossing", "lat": 35.6595, "lon": 139.7005, "country": "Japan",
               "country_code": "JP", "settlement": "Shibuya", "areas": [], "population": None, "kind": "junction",
               "full": "x", "source": "test"}
    cities = {"elements": [{"lat": 35.6894, "lon": 139.6917, "tags": {"name": "Tokyo", "population": "14094034"}},
                           {"lat": 35.85, "lon": 139.65, "tags": {"name": "Saitama", "population": "1300000"}}]}
    fake_fetch(monkeypatch, {"interpreter": cities, "nominatim": [
        {"lat": "35.66", "lon": "139.70", "extratags": {"population": "227850"}}]})
    assert mm.settlement_population(shibuya) == 14094034
    assert mm.town_class(mm.settlement_population(shibuya))[0] == "large"
    kinsale = {"name": "Kinsale", "lat": 51.7059, "lon": -8.5222, "country": "Ireland", "country_code": "IE",
               "settlement": "Kinsale", "areas": ["County Cork"], "population": None, "full": "x", "source": "t"}
    monkeypatch.setattr(mm, "CACHE", mm.CACHE.with_name("other.json"))
    fake_fetch(monkeypatch, {"interpreter": {"elements": [{"lat": 51.8985, "lon": -8.4756,
                                                           "tags": {"population": "210000"}}]},
                             "nominatim": [{"lat": "51.7059", "lon": "-8.5222", "extratags": {"population": "5281"}}]})
    assert mm.settlement_population(kinsale) == 5281          # Cork, 22 km away, does not reach it


def test_a_town_found_as_its_boundary_is_centred_on_the_town(monkeypatch):
    hallstatt = {"lat": "47.5347939", "lon": "13.5988875", "category": "boundary", "type": "administrative",
                 "addresstype": "city", "osm_type": "relation", "osm_id": 80744, "name": "Hallstatt",
                 "display_name": "Hallstatt, Bezirk Gmunden, Upper Austria, Austria",
                 "address": {"city": "Hallstatt", "state": "Upper Austria", "country": "Austria",
                             "country_code": "at"}, "extratags": {"population": "740"}}
    centre = {"elements": [{"type": "node", "lat": 47.5622, "lon": 13.6493,
                            "tags": {"place": "village", "name": "Hallstatt"}},
                           {"type": "node", "lat": 47.55, "lon": 13.62, "tags": {"place": "hamlet", "name": "Lahn"}}]}
    calls = []
    fake_fetch(monkeypatch, {"nominatim": [hallstatt], "interpreter": lambda params: centre if "rel(80744)" in
                             params["data"] else {"elements": []}}, calls)
    found = mm.search("Hallstatt")[0]
    assert found["boundary"] == 80744 and found["population"] == 740
    p = mm.plan(found=found, size="small")
    assert (p["lat"], p["lon"]) == (47.5622, 13.6493) and p["town"] == "small"
    assert mm.town_centre(dict(found, boundary=None)) is None


def test_a_clicked_point_is_named_after_its_town(monkeypatch):
    rev = {"lat": "51.7059", "lon": "-8.5222", "category": "highway", "type": "secondary", "addresstype": "road",
           "name": "Pearse Street", "display_name": "Pearse Street, Kinsale Urban, County Cork, Ireland",
           "address": {"road": "Pearse Street", "city_district": "Kinsale Urban", "county": "County Cork",
                       "country": "Ireland", "country_code": "ie"}}
    places = {"elements": [{"lat": 51.7066, "lon": -8.5223, "tags": {"place": "town", "name": "Kinsale",
                                                                      "population": "5281"}},
                           {"lat": 51.8985, "lon": -8.4756, "tags": {"place": "city", "name": "Cork",
                                                                      "population": "210000"}}]}
    fake_fetch(monkeypatch, {"nominatim.openstreetmap.org/reverse": rev, "interpreter": places})
    p = mm.plan(lat=51.7059, lon=-8.5222, size="small")
    assert p["label"] == "Kinsale" and p["slug"] == "kinsale" and p["population"] == 5281 and p["town"] == "small"
    assert p["drive"] == "left"


# ---------------------------------------------------------------- the experiment file

def place(**kw):
    p = {"name": "Kinsale", "full": "Kinsale, County Cork, Ireland", "lat": 51.7059, "lon": -8.5222,
         "country": "Ireland", "country_code": "IE", "settlement": "Kinsale", "kind": "town", "population": 5281,
         "source": "OpenStreetMap Nominatim"}
    p.update(kw)
    return p


def test_experiment_file(isolated):
    p = mm.plan(size="small", found=place(), pedestrians=True)
    assert p["drive"] == "left" and p["town"] == "small" and p["km"] == 1.0 and p["slug"] == "kinsale"
    path = mm.write_experiment(p, query="Kinsale")
    text = path.read_text(encoding="utf-8")
    assert path == isolated / "experiments" / "places" / "kinsale.yaml"
    assert text.startswith("# Kinsale, Ireland: a 1 km square (small)")
    assert "left-hand traffic" in text and "python -m coupledcarla.run experiments/places/kinsale.yaml" in text
    exp = yaml.safe_load(text)
    assert exp["name"] == "kinsale" and exp["map"]["lefthand"] is True and exp["map"]["bbox"] == p["bbox"]
    assert exp["city"] == {"name": "Kinsale", "country": "Ireland", "continent": "Europe", "size": "small",
                           "population": 5281, "lat": 51.7059, "lon": -8.5222, "drive": "left",
                           "country_code": "IE"}
    assert exp["traffic"]["walkers"] == {"count": "auto"}
    assert exp["participants"][0]["role"] == "driver"
    from coupledcarla import config                      # a valid experiment for the rest of coupled-carla
    cfg = config.load(path)
    assert cfg["map"]["name"] == "kinsale" and cfg["map"]["lefthand"] and cfg["traffic"]["walkers"]["radius_m"]


def test_unicode_and_warnings(isolated):
    p = mm.plan(size=3, found=place(name="Plaza Mayor, Salamanca", country="Spain", country_code="ES",
                                     population=None, kind="road", settlement="Salamanca"))
    assert p["drive"] == "right" and p["warnings"] and "very large" in p["warnings"][0]
    p2 = mm.plan(found=place(country=None, country_code=None, name="Somewhere"))
    assert any("country" in w for w in p2["warnings"]) and p2["drive"] == "right"
    t = mm.experiment_text(mm.plan(found=place(name="Zürich, Paradeplatz")), query="Zürich")
    assert "Zürich" in t and "\\u" not in t
    bad = mm.experiment_text(mm.plan(found=place(), name="x\nserver: {launch: false}"), query="a\nb: 1")
    assert all(line.startswith("#") for line in bad.split("\nname:")[0].splitlines())
    assert yaml.safe_load(bad).get("server") is None and yaml.safe_load(bad).get("b") is None


def test_unique_names(isolated):
    p = mm.plan(found=place())
    mm.write_experiment(p)
    assert mm.plan(found=place())["slug"] == "kinsale"                     # the same box: the same map
    assert mm.plan(found=place(lat=51.70), size="small")["slug"] == "kinsale"     # never built: reused
    (isolated / "maps" / "kinsale").mkdir(parents=True)
    (isolated / "maps" / "kinsale" / "meta.json").write_text(json.dumps({"bbox": p["bbox"]}))
    assert mm.plan(found=place())["slug"] == "kinsale"                     # built, the same box: the same map
    assert mm.plan(found=place(lat=51.70), size="small")["slug"] == "kinsale_2"   # another box: a new name
    (isolated / "experiments" / "cities" / "leuven.yaml").write_text("name: leuven\n")
    assert mm.plan(found=place(name="Leuven"))["slug"] == "leuven_2"       # never a city's name


# ---------------------------------------------------------------- progress and errors

LOG = """[mapbuild    3.1s] Overpass https://overpass-api.de/api/interpreter failed: 502 Server Error
[mapbuild    6.3s] OSM from https://overpass.openstreetmap.fr/api/interpreter: 0.2 MB
[realworld   14.2s] elevation (terrarium): 1.1 to 64.8 m
[mapbuild   14.2s] elevation on 1806 road nodes (0 on bridges or in tunnels levelled)
Success.
[mapbuild   14.6s] pruning: 340 car edges -> 201 kept (72 dead-end nodes, core of 83 junctions); 0 bike/foot-only
[mapbuild   14.8s] OpenDRIVE: 1007 lane links, 5372 waypoints, 1 lane ends, 3 traffic-light signals
[realworld   28.0s] Overture 2026-10-21.0: 1280 buildings (0 with height)
[mapbuild   28.4s] scenery: 1576 building meshes for 1274 footprints (overture)
[canopy] 1431 x 1433 px from 1 of 1 tiles (031312013) in 5 s
[mapbuild   84.5s] scenery: 1781 trees (26 street trees (OSM and registers), 0 in rows); 595 sized by the canopy
[mapbuild   84.6s] scenery: 45 pieces of street furniture, 515 lamps
[mapbuild   84.6s] scenery: 6 road signs from OSM
[mapbuild   85.1s] scenery: 6840 ground tiles of 20 m (808 lowered under the road surface)
[mapbuild  126.4s] parked: 1374 kerb stalls (beside_road 179), 603 in car parks
[transit      34.3s] 4 OSM bus routes in the box; 5 matched onto the net (3 line numbers), 1 stops
[transit      42.9s] GTFS mdb-2636 (Bus Éireann): 1 of 3 lines with a timetable
[mapbuild  191.9s] done: /home/x/maps/calib_kinsale
"""


def test_progress_in_plain_words(tmp_path):
    (tmp_path / "map.osm").write_text('<osm><way><tag k="highway" v="primary"/></way>'
                                      '<way><tag k="highway" v="residential"/></way></osm>')
    events = []
    pr = mm.Progress(events.append, tmp_path)
    pr.set("streets", "running", "")
    for line in LOG.splitlines():
        pr.feed(line)
    done = {e["step"]: e["text"] for e in events if e["state"] == "done"}
    assert done["streets"] == "done, 2 streets and paths"
    assert "busy" in next(e["text"] for e in events if e["step"] == "streets" and e["state"] == "running" and e["text"])
    assert done["roads"] == "done, 1,007 lane connections, 3 traffic lights"
    assert done["buildings"] == "done, 1,274 buildings" and done["trees"] == "done, 1,781 trees"
    assert done["furniture"] == "done, 515 lamps, 45 benches, bins and other street furniture, 6 road signs"
    assert done["parked"].startswith("done, 1,374") and done["ground"] == "done, 6,840 ground tiles"
    assert done["buses"] == "done, 3 bus lines (1 with a real timetable)"
    assert all(pr.state[s] == "done" for s in mm._ORDER[:-1])
    lines = []
    pr2 = mm.Progress(mm.printer(type("S", (), {"write": lambda self, s: lines.append(s), "flush": lambda self: 0})()),
                      tmp_path)
    pr2.set("streets", "running", "")
    for line in LOG.splitlines():
        pr2.feed(line)
    out = "".join(lines)
    assert "Downloading the streets from OpenStreetMap…" in out and "Planting the trees…" in out


def test_explain_failure():
    msg, advice = mm.explain_failure("RuntimeError: no Overpass server answered")
    assert "did not answer" in msg and "minutes" in advice
    msg, advice = mm.explain_failure("NameResolutionError ... Max retries\nRuntimeError: no Overpass server answered")
    assert "internet" in msg
    msg, advice = mm.explain_failure("pruning: 0 car edges -> 0 kept (0 dead-end nodes, core of 0 junctions)")
    assert "no usable roads" in msg and "sea" in advice
    msg, advice = mm.explain_failure("ModuleNotFoundError: No module named 'sumolib'")
    assert "sumolib" in msg and "setup" in advice
    assert "memory" in mm.explain_failure("", -9)[0]
    assert "went wrong" in mm.explain_failure("Traceback ... ValueError: odd")[0]


def test_build_refuses_an_empty_area(isolated, monkeypatch, tmp_path):
    from coupledcarla import config
    monkeypatch.setattr(config, "map_dir", lambda cfg: tmp_path / "maps" / cfg["map"]["name"])
    monkeypatch.setattr(mm, "check_roads", lambda box: 0)
    monkeypatch.setattr(mm, "nearby_places", lambda lat, lon: None)
    exp = mm.write_experiment(mm.plan(found=place(name="Middle of the Atlantic", lat=30.0, lon=-40.0,
                                                  country=None, country_code=None)))
    events = []
    with pytest.raises(mm.MapMakerError) as e:
        mm.build(exp, events.append)
    assert "No roads found here" in str(e.value) and "town" in e.value.advice
    assert events[-1]["state"] == "failed"
    assert not exp.exists() and not (tmp_path / "maps" / exp.stem).exists()    # nothing left behind
    assert mm.built_maps() == []


def test_summary(tmp_path):
    d = tmp_path / "m"
    d.mkdir()
    (d / "meta.json").write_text(json.dumps({
        "name": "m", "lefthand": True, "buildings": 3, "bbox": [1, 2, 3, 4], "built": "2026-11-15 12:00:00",
        "xodr": {"lane_links": 10, "tl_signals": 2}, "elevation": {"source": "terrarium", "min_m": 1.0, "max_m": 65.0},
        "scenery": {"tree": 7, "lamp": 2, "furniture": 1, "sign": 0, "parked_kerb_stalls": 4},
        "transit": {"refs": 3, "with_timetable": 1},
        "sources": {"overture": {"what": "Overture Maps", "licence": "ODbL 1.0 / CDLA Permissive 2.0"}}}))
    (d / "buildings.json").write_text(json.dumps([{"h": 5.0, "p": []}, {"h": 9.0, "p": []}, {"h": 30.0, "p": []}]))
    s = mm.summary(d, {"city": {"name": "Kinsale", "country": "Ireland", "country_code": "IE"}, "traffic": {}})
    assert s["drive"] == "left" and s["buildings"] == 3 and s["building_height_m"] == [5.0, 9.0, 30.0]
    assert s["trees"] == 7 and s["bus_lines"] == 3 and s["relief_m"] == 64 and s["parking_places"] == 4
    assert {"osm", "overture", "nominatim"} <= set(s["sources"])
    assert s["warnings"] == [] or s["roads_km"] < 2.0              # no net here: sumolib absent or roads_km 0
    t = mm.summary_text(s)
    assert "Kinsale, Ireland" in t and "the tallest 30 m" in t and "ODbL 1.0" in t and "64 m of hills" in t


def test_carla_status(monkeypatch, tmp_path):
    monkeypatch.setattr(mm.sys, "platform", "darwin")
    st = mm.carla_status()
    assert not st["ok"] and "Mac" in st["why"]
    monkeypatch.setattr(mm.sys, "platform", "linux")
    monkeypatch.delenv("CARLA_ROOT", raising=False)
    monkeypatch.setattr(mm, "find_carla", lambda: None)
    st = mm.carla_status()
    assert not st["ok"] and "0.9.16" in st["how"]
    cmd = mm.drive_command(mm.ROOT / "experiments" / "places" / "x.yaml")
    assert cmd[1:] == ["-m", "coupledcarla.run", str(Path("experiments/places/x.yaml"))]
    venv = mm.ROOT / ".venv" / "bin" / "python"
    assert mm.shown_command([str(venv)] + cmd[1:]).startswith(str(Path(".venv/bin/python")) + " -m coupledcarla.run")
    assert mm.shown_command(["/usr/bin/python3", "-m", "x"]) == "python -m x"


# ---------------------------------------------------------------- the web page

@pytest.fixture
def server(monkeypatch):
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), BaseHTTPRequestHandler)
    port = httpd.server_address[1]
    httpd.RequestHandlerClass = web.make_handler(web.Jobs(), port)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    yield f"http://127.0.0.1:{port}"
    httpd.shutdown()
    httpd.server_close()


def request(url, data=None, headers=None):
    h = dict(headers or {})
    body = None
    if data is not None:
        body = json.dumps(data).encode()
        h.setdefault("Content-Type", "application/json")
    req = urllib.request.Request(url, data=body, headers=h)
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read(), r.headers.get("Content-Type")
    except urllib.error.HTTPError as err:
        return err.code, err.read(), err.headers.get("Content-Type")


def test_web_page_and_api(server, monkeypatch):
    code, body, ctype = request(server + "/")
    assert code == 200 and "text/html" in ctype
    page = body.decode()
    assert "Build this map" in page and "leaflet@1.9.4" in page and "integrity=" in page
    assert "if (!r.ok) throw data" in page          # a failed job (with an "error" field) is shown, not retried
    assert "OpenStreetMap</a> contributors" in page
    monkeypatch.setattr(mm, "search", lambda q, limit=5: [place(name=q)])
    code, body, _ = request(server + "/api/search?q=Kinsale")
    assert code == 200 and json.loads(body)["results"][0]["name"] == "Kinsale"
    code, body, _ = request(server + "/api/job")
    assert json.loads(body) == {"state": "idle"}
    code, body, _ = request(server + "/api/maps")
    assert json.loads(body) == {"maps": []}
    monkeypatch.setattr(mm, "reverse", lambda lat, lon: place(lat=lat, lon=lon))
    code, body, _ = request(server + "/api/plan", {"lat": 51.7, "lon": -8.52, "size": "large"})
    p = json.loads(body)
    assert code == 200 and p["drive"] == "left" and p["km"] == 2.0 and p["lat"] == 51.7
    code, body, _ = request(server + "/api/plan", {"found": place(), "size": "small"})
    assert json.loads(body)["km"] == 1.0
    code, body, _ = request(server + "/api/plan", {"size": "huge", "found": place()})
    assert code == 400 and json.loads(body)["advice"]
    code, body, _ = request(server + "/api/plan", {"lat": 95, "lon": 0})
    assert code == 400


def test_web_refuses_other_sites(server):
    assert request(server + "/", headers={"Host": "evil.example:80"})[0] == 403
    assert request(server + "/api/plan", {"lat": 1, "lon": 2}, {"Origin": "https://evil.example"})[0] == 403
    req = urllib.request.Request(server + "/api/build", data=b"lat=1", headers={"Content-Type": "text/plain"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req, timeout=10)
    assert e.value.code == 403
    assert request(server + "/maps/../../etc/passwd")[0] == 404
    assert request(server + "/maps/x/preview.png")[0] == 404
    assert request(server + "/api/drive", {"name": "../x"})[0] == 400


def test_web_build_runs_one_at_a_time(server, monkeypatch):
    gate = threading.Event()

    def slow_build(exp, emit, force=False):
        emit({"step": "streets", "title": "x", "state": "running", "text": "busy", "at": 0})
        gate.wait(10)
        return {"label": "Kinsale"}
    monkeypatch.setattr(mm, "build", slow_build)
    code, body, _ = request(server + "/api/build", {"found": place()})
    assert code == 200 and json.loads(body)["job"]
    code, body, _ = request(server + "/api/build", {"found": place()})
    assert code == 409 and "already" in json.loads(body)["error"]
    j = json.loads(request(server + "/api/job")[1])
    assert j["state"] == "running" and j["steps"][1]["text"] == "busy" and j["plan"]["slug"] == "kinsale"
    gate.set()
    for _ in range(50):
        j = json.loads(request(server + "/api/job")[1])
        if j["state"] != "running":
            break
        threading.Event().wait(0.1)
    assert j["state"] == "done" and j["summary"] == {"label": "Kinsale"}
    assert (mm.PLACES / "kinsale.yaml").exists()


def test_search_ranks_the_city_before_its_region():
    from coupledcarla.mapmaker import rank_places
    res = [{"name": "Chernihiv Oblast", "addresstype": "state", "category": "boundary", "place_rank": 8},
           {"name": "Chernihiv", "addresstype": "city", "category": "boundary", "place_rank": 16},
           {"name": "Chernihiv Raion", "addresstype": "district", "category": "boundary", "place_rank": 12}]
    assert [r["name"] for r in rank_places(res)] == ["Chernihiv", "Chernihiv Oblast", "Chernihiv Raion"]
