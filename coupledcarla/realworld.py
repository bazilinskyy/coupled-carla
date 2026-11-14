"""Real-world data for a map's bounding box, from free services that need no account or key.

Everything is cached in maps/<name>/real/, so a map is fetched once and then builds offline. Each fetch says
where its data came from (SOURCES), and meta.json lists what a map was built from.

- OpenStreetMap (Overpass API): street trees with species and height, tree rows, parks, woods and grass, water,
  street lamps, benches, bins, bus shelters, bollards, post boxes, bicycle racks, fountains, ad columns, stop,
  give-way and speed-limit signs.
- Overture Maps buildings (S3, GeoParquet via DuckDB): footprints conflated from OSM, Microsoft and Google ML
  buildings and Esri, with height and floor counts where known.
- National building heights replacing guessed ones: 3DBAG (Netherlands, lidar), IGN BD TOPO (France),
  NYC Open Data building footprints (New York City), MLIT PLATEAU (Japan, the cities it covers).
- Elevation: AHN 0.5 m DTM from PDOK (Netherlands) or the AWS Terrain Tiles (Terrarium, worldwide; SRTM, 3DEP,
  national lidar where available).
- Open-Meteo: hourly weather for the place and time of a trial (forecast and ERA5 archive back to 1940).
"""
import io
import json
import math
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import requests

UA = {"User-Agent": "coupled-carla/0.2 (+https://github.com/bazilinskyy/coupled-carla)"}
OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.openstreetmap.fr/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter",
            "https://overpass.private.coffee/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter"]
OVERTURE_RELEASE = "2026-10-21.0"
OVERTURE = "s3://overturemaps-us-west-2/release/{rel}/theme=buildings/type=building/*"
THREEDBAG = "https://api.3dbag.nl/collections/pand/items"
AHN_WCS = "https://service.pdok.nl/rws/ahn/wcs/v1_0"
TERRARIUM = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
METEO = "https://api.open-meteo.com/v1/forecast"
METEO_ARCHIVE = "https://archive-api.open-meteo.com/v1/archive"

SOURCES = {
    "osm": ("OpenStreetMap contributors via the Overpass API", "ODbL 1.0"),
    "overture": ("Overture Maps Foundation buildings (OSM, Microsoft, Google, Esri)", "ODbL 1.0 / CDLA Permissive 2.0"),
    "3dbag": ("3DBAG by the 3D geoinformation group, TU Delft (BAG + AHN)", "CC BY 4.0"),
    "bdtopo": ("IGN BD TOPO buildings (heights, use) via the Géoplateforme WFS", "Licence Ouverte 2.0 (Etalab)"),
    "nyc": ("NYC Open Data Building Footprints (roof heights, OTI)", "NYC Open Data terms of use"),
    "plateau": ("MLIT Project PLATEAU 3D city models (LOD1 heights), as GeoParquet by Pacific Spatial Solutions "
                "(Flateau, Source Cooperative)", "CC BY 4.0 (PLATEAU terms of use)"),
    "ahn": ("AHN4 DTM 0.5 m via PDOK (Rijkswaterstaat)", "CC0 1.0"),
    "bgt": ("Basisregistratie Grootschalige Topografie (BGT, with IMGeo plus objects) via the PDOK OGC API "
            "(Kadaster and the bronhouders)", "CC0 1.0"),
    "ndw": ("NDW Verkeersborden: the national traffic sign register of the Dutch road authorities "
            "(Nationaal Dataportaal Wegverkeer)", "CC0 1.0"),
    "wkd": ("Rijkswaterstaat Wegkenmerkendatabase: maximum speeds per NWB road section", "CC0 1.0"),
    "terrarium": ("AWS Terrain Tiles (Mapzen Terrarium: SRTM, 3DEP, national lidar, ...)", "see their attribution"),
    "open-meteo": ("Open-Meteo.com weather API (national weather services, ERA5)", "CC BY 4.0"),
    "eindhoven": ("Gemeente Eindhoven open data: trees (bomen) and street lights (openbare verlichting)",
                  "public domain"),
    "nyc": ("NYC Parks 2015 Street Tree Census", "NYC Open Data terms of use"),
    "canopy": ("Meta and WRI High Resolution Canopy Height Maps (Tolan et al. 2024), AWS open data", "CC BY 4.0"),
    "gtfs": ("GTFS timetables of the local transit agencies, found via the Mobility Database catalogue",
             "per feed (see transit.json)"),
}


def log(msg, t0=[time.time()]):
    print(f"[realworld {time.time() - t0[0]:6.1f}s] {msg}", flush=True)


# --- OpenStreetMap -------------------------------------------------------------------------------------------

def overpass_json(query, timeout=100):
    for attempt in range(2):
        for url in OVERPASS:
            try:
                r = requests.post(url, data={"data": query}, headers=UA, timeout=timeout)
                r.raise_for_status()
                return r.json()
            except (requests.RequestException, ValueError) as err:
                log(f"Overpass {url} failed: {err}")
        time.sleep(15)
    raise RuntimeError("no Overpass server answered")


def osm_scenery(bbox, path):
    """Trees, green and water areas and street furniture in the box (Overpass JSON with geometry and tags)."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    query = f"""[out:json][timeout:180];
(
  node["natural"="tree"]{b};
  way["natural"="tree_row"]{b};
  way["natural"~"^(wood|scrub|grassland|heath|water)$"]{b};
  relation["natural"~"^(wood|water)$"]{b};
  way["landuse"~"^(grass|forest|meadow|village_green|recreation_ground|cemetery|allotments|farmland|orchard|flowerbed|basin|reservoir)$"]{b};
  way["leisure"~"^(park|garden|pitch|playground|dog_park|common)$"]{b};
  relation["leisure"="park"]{b};
  way["waterway"="riverbank"]{b};
  node["highway"~"^(street_lamp|bus_stop)$"]{b};
  node["amenity"~"^(bench|waste_basket|post_box|bicycle_parking|fountain|drinking_water|telephone|vending_machine|atm|recycling|shelter|waste_disposal)$"]{b};
  node["leisure"="picnic_table"]{b};
  node["barrier"~"^(bollard|planter)$"]{b};
  node["emergency"="fire_hydrant"]{b};
  node["advertising"~"^(column|billboard|board|poster_box)$"]{b};
  node["public_transport"="platform"]["bus"="yes"]{b};
  node["man_made"="street_cabinet"]{b};
  node["highway"~"^(stop|give_way)$"]{b};
  node["traffic_sign"]{b};
);
out geom tags;"""
    data = overpass_json(query)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    log(f"OSM scenery: {len(data.get('elements', []))} features")
    return data


# --- municipal registers (trees, street lights) ------------------------------------------------------------------

EHV = "https://data.eindhoven.nl/api/explore/v2.1/catalog/datasets/{}/exports/json"
EHV_HEIGHTS = {"0-6": 4.5, "6-9": 7.5, "9-12": 10.5, "12-15": 13.5, "15-18": 16.5, "18-24": 21.0, ">24": 26.0}
NYC_TREES = "https://data.cityofnewyork.us/resource/uvpi-gqnh.json"


def _ehv(bbox):
    s, w, n, e = bbox
    where = f"in_bbox(geo_point_2d,{s},{w},{n},{e})"
    out = []
    trees = requests.get(EHV.format("bomen"), params={"where": where, "select": "boomsoort,hoogte,geo_point_2d"},
                         headers=UA, timeout=180).json()
    for i, t in enumerate(trees):
        g = t.get("geo_point_2d") or {}
        if "lat" not in g:
            continue
        latin = (t.get("boomsoort") or "").strip()
        tags = {"natural": "tree", "genus": latin.split(" ")[0] if latin else "", "species": latin,
                "source": "eindhoven-bomen"}
        if EHV_HEIGHTS.get(t.get("hoogte")):
            tags["height"] = str(EHV_HEIGHTS[t["hoogte"]])
        out.append({"type": "node", "id": -1_000_000 - i, "lat": g["lat"], "lon": g["lon"], "tags": tags})
    lights = requests.get(EHV.format("data-openbare-verlichting"),
                          params={"where": where, "select": "geo_point_2d,lph_armatuur,soort_lamp,kleur_lamp"},
                          headers=UA, timeout=180).json()
    seen = set()
    for i, lt in enumerate(lights):
        g = lt.get("geo_point_2d") or {}
        if "lat" not in g:
            continue
        key = (round(g["lat"], 6), round(g["lon"], 6))      # a mast with two lamps is listed twice
        if key in seen:
            continue
        seen.add(key)
        tags = {"highway": "street_lamp", "source": "eindhoven-openbare-verlichting",
                "lamp_type": lt.get("soort_lamp") or "", "light:colour": lt.get("kleur_lamp") or ""}
        if lt.get("lph_armatuur"):
            tags["height"] = f"{float(lt['lph_armatuur']) / 100:.1f}"      # light point height, cm
        out.append({"type": "node", "id": -2_000_000 - i, "lat": g["lat"], "lon": g["lon"], "tags": tags})
    return out


def _nyc_trees(bbox):
    s, w, n, e = bbox
    rows = requests.get(NYC_TREES, params={
        "$select": "spc_latin,tree_dbh,latitude,longitude", "$limit": 50000,
        "$where": f"latitude between {s} and {n} AND longitude between {w} and {e} AND status='Alive'"},
        headers=UA, timeout=180).json()
    out = []
    for i, t in enumerate(rows):
        try:
            lat, lon, dbh = float(t["latitude"]), float(t["longitude"]), float(t.get("tree_dbh") or 0)
        except (KeyError, ValueError):
            continue
        latin = (t.get("spc_latin") or "").strip()
        # no heights in the census: a street tree's height from its trunk diameter (inches), capped
        h = min(25.0, 2.0 + 0.55 * dbh) if dbh else None
        tags = {"natural": "tree", "genus": latin.split(" ")[0] if latin else "", "species": latin,
                "source": "nyc-street-tree-census-2015"}
        if h:
            tags["height"] = f"{h:.1f}"
        out.append({"type": "node", "id": -3_000_000 - i, "lat": lat, "lon": lon, "tags": tags})
    return out


def _in_box(bbox, s, w, n, e):
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    return s <= lat <= n and w <= lon <= e


MUNICIPAL = [   # (name, area (s, w, n, e) the register covers, fetch, what it has)
    ("eindhoven", (51.38, 5.38, 51.50, 5.57), _ehv, "Gemeente Eindhoven open data: bomen, openbare verlichting"),
    ("nyc", (40.49, -74.27, 40.92, -73.68), _nyc_trees, "NYC Parks 2015 Street Tree Census"),
]


def municipal_scenery(bbox, path):
    """Trees and street lights from the city's own registers, as OSM-like nodes ({elements, sources}); empty where
    no register is known. Cached in path."""
    if path.exists():
        return json.loads(path.read_text())
    els, used = [], []
    for name, area, fetch, what in MUNICIPAL:
        if _in_box(bbox, *area):
            try:
                got = fetch(bbox)
            except Exception as err:  # noqa: BLE001 - the register is down: OSM alone
                log(f"{name} register failed ({err})")
                continue
            els += got
            used.append(name)
            log(f"{what}: {sum(1 for x in got if x['tags'].get('natural') == 'tree')} trees, "
                f"{sum(1 for x in got if x['tags'].get('highway') == 'street_lamp')} street lights")
    data = {"elements": els, "sources": used}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return data


def osm_buildings(bbox, path):
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    data = overpass_json(f'[out:json][timeout:180];(way["building"]({s},{w},{n},{e});'
                         f'relation["building"]({s},{w},{n},{e}););out geom tags;')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    return data


# --- buildings: Overture and 3DBAG ---------------------------------------------------------------------------

def overture_buildings(bbox, path, release=OVERTURE_RELEASE):
    """[{ring: [[lon, lat], ...], height, floors, cls, src}] from Overture Maps, or None without DuckDB."""
    if path.exists():
        return json.loads(path.read_text())
    try:
        import duckdb
    except ImportError:
        log("Overture skipped: pip install duckdb")
        return None
    s, w, n, e = bbox
    con = duckdb.connect()
    con.execute("INSTALL httpfs; LOAD httpfs; INSTALL spatial; LOAD spatial; SET s3_region='us-west-2';")
    q = f"""SELECT height, num_floors, class, subtype, sources[1].dataset AS src, ST_AsGeoJSON(geometry) AS g
            FROM read_parquet('{OVERTURE.format(rel=release)}', hive_partitioning=1)
            WHERE bbox.xmin < {e} AND bbox.xmax > {w} AND bbox.ymin < {n} AND bbox.ymax > {s}"""
    out = []
    for height, floors, cls, sub, src, g in con.execute(q).fetchall():
        geom = json.loads(g)
        polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom["coordinates"]
        for poly in polys:
            out.append({"ring": [[round(x, 7), round(y, 7)] for x, y in poly[0]], "height": height,
                        "floors": floors, "cls": cls or sub, "src": src})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    log(f"Overture {release}: {len(out)} buildings ({sum(b['height'] is not None for b in out)} with height)")
    return out


def _rd_bbox(bbox):
    from pyproj import Transformer
    to_rd = Transformer.from_crs("EPSG:4326", "EPSG:28992", always_xy=True)
    s, w, n, e = bbox
    xs, ys = zip(*(to_rd.transform(lon, lat) for lat, lon in ((s, w), (s, e), (n, w), (n, e))))
    return min(xs), min(ys), max(xs), max(ys)


def in_netherlands(bbox):
    """A rough test (3DBAG and AHN cover the Netherlands only; the API answers empty elsewhere)."""
    s, w, n, e = bbox
    lat, lon = (s + n) / 2, (w + e) / 2
    if not (50.75 <= lat <= 53.6 and 3.3 <= lon <= 7.25):
        return False
    # Belgium and Germany inside that box: south of 51.5 only the Limburg/Brabant strip is Dutch
    if lat < 51.25 and not (5.6 <= lon <= 6.1 and lat > 50.75):
        return False
    return True


def threedbag_buildings(bbox, path):
    """[{ring: [[lon, lat]...], height, ground, roof, id}] from 3DBAG (lidar heights), [] outside NL."""
    if path.exists():
        return json.loads(path.read_text())
    if not in_netherlands(bbox):
        return []
    from pyproj import Transformer
    to_ll = Transformer.from_crs("EPSG:28992", "EPSG:4326", always_xy=True)
    url, params, out = THREEDBAG, {"bbox": ",".join(f"{v:.1f}" for v in _rd_bbox(bbox)), "limit": 100}, []
    while url:
        for attempt in range(3):
            try:
                r = requests.get(url, params=params, timeout=120, headers=UA)
                r.raise_for_status()
                page = r.json()
                break
            except (requests.RequestException, ValueError) as err:
                log(f"3DBAG failed ({err}), retrying")
                time.sleep(5)
        else:
            raise RuntimeError("3DBAG did not answer")
        tf = page["metadata"]["transform"]
        scale, translate = np.array(tf["scale"]), np.array(tf["translate"])
        for f in page["features"]:
            verts = np.asarray(f["vertices"], dtype=np.float64) * scale + translate
            for oid, obj in f["CityObjects"].items():
                if obj.get("type") != "Building":
                    continue
                a = obj.get("attributes", {})
                roof = a.get("b3_h_dak_70p") or a.get("b3_h_70p") or a.get("b3_h_dak_50p")
                ground = a.get("b3_h_maaiveld")
                ring = None
                for g in obj.get("geometry", []):
                    if str(g.get("lod")) == "0":
                        b = g["boundaries"]
                        ring = b[0][0] if g["type"] == "MultiSurface" else b[0][0][0]
                if ring is None:      # no LoD0: the lowest ring of a part's LoD1.2 solid
                    for child in obj.get("children", []):
                        part = f["CityObjects"].get(child, {})
                        for g in part.get("geometry", []):
                            if g["type"] == "Solid" and ring is None:
                                rings = [s[0] for s in g["boundaries"][0]]
                                ring = min(rings, key=lambda r: float(np.mean(verts[r, 2])))
                if ring is None or roof is None or ground is None:
                    continue
                lon, lat = to_ll.transform(verts[ring, 0], verts[ring, 1])
                out.append({"ring": [[round(x, 7), round(y, 7)] for x, y in zip(lon, lat)],
                            "height": round(float(roof) - float(ground), 2), "ground": round(float(ground), 2),
                            "roof": a.get("b3_dak_type"), "id": oid})
        url = next((l["href"] for l in page.get("links", []) if l.get("rel") == "next"), None)
        params = None
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    log(f"3DBAG: {len(out)} buildings with lidar heights")
    return out


BDTOPO_WFS = "https://data.geopf.fr/wfs/ows"
NYC_FOOTPRINTS = "https://data.cityofnewyork.us/resource/5zhs-2jue.geojson"
BDTOPO_USE = {"Résidentiel": "residential", "Commercial et services": "commercial", "Industriel": "industrial",
              "Religieux": "church", "Agricole": "farm", "Sportif": "commercial", "Annexe": "garage"}


def _get(url, params, tries=4):
    for attempt in range(tries):
        try:
            r = requests.get(url, params=params, headers=UA, timeout=120)
            r.raise_for_status()
            return r.json()
        except (requests.RequestException, ValueError) as err:
            log(f"{url} failed ({err}), retrying")
            time.sleep(10 * (attempt + 1))
    raise RuntimeError(f"{url} did not answer")


def _rings(geom):
    polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom["coordinates"]
    return [[[round(x, 7), round(y, 7)] for x, y, *_ in poly[0]] for poly in polys]


def bdtopo_buildings(bbox, path):
    """France: [{ring, height, cls}] from IGN BD TOPO (photogrammetric heights), [] elsewhere."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    if not (41.3 <= (s + n) / 2 <= 51.1 and -5.2 <= (w + e) / 2 <= 9.6):
        return []
    out, start = [], 0
    while True:
        page = _get(BDTOPO_WFS, {"SERVICE": "WFS", "VERSION": "2.0.0", "REQUEST": "GetFeature",
                                 "TYPENAMES": "BDTOPO_V3:batiment", "outputFormat": "application/json",
                                 "BBOX": f"{s},{w},{n},{e},urn:ogc:def:crs:EPSG::4326", "COUNT": 1000,
                                 "STARTINDEX": start})
        for f in page.get("features", []):
            p = f["properties"]
            if f.get("geometry") is None or p.get("hauteur") is None:
                continue
            for ring in _rings(f["geometry"]):
                out.append({"ring": ring, "height": float(p["hauteur"]), "cls": BDTOPO_USE.get(p.get("usage_1"))})
        if len(page.get("features", [])) < 1000:
            break
        start += 1000
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    log(f"BD TOPO: {len(out)} buildings with heights")
    return out


def nyc_buildings(bbox, path):
    """New York City: [{ring, height}] from NYC Open Data (roof heights in feet), [] elsewhere."""
    if path.exists():
        return json.loads(path.read_text())
    s, w, n, e = bbox
    if not (40.49 <= (s + n) / 2 <= 40.92 and -74.27 <= (w + e) / 2 <= -73.68):
        return []
    data = _get(NYC_FOOTPRINTS, {"$where": f"within_box(the_geom, {n}, {w}, {s}, {e})", "$limit": 50000})
    out = []
    for f in data.get("features", []):
        h = f["properties"].get("height_roof")
        if f.get("geometry") is None or h is None:
            continue
        for ring in _rings(f["geometry"]):
            out.append({"ring": ring, "height": round(float(h) * 0.3048, 2), "cls": None})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":")))
    log(f"NYC Open Data: {len(out)} buildings with roof heights")
    return out


FLATEAU = "https://data.source.coop/pacificspatial/flateau/parquet/"
FLATEAU_LIST = "https://s3.us-west-2.amazonaws.com/us-west-2.opendata.source.coop"
GSI_REVERSE = "https://mreversegeocoder.gsi.go.jp/reverse-geocoder/LonLatToAddress"
PLATEAU_USE = {"住宅": "house", "共同住宅": "apartments", "店舗等併用住宅": "retail", "店舗等併用共同住宅": "retail",
               "作業所併用住宅": "residential", "業務施設": "office", "商業施設": "commercial",
               "商業系複合施設": "commercial", "宿泊施設": "hotel", "文教厚生施設": "school",
               "官公庁施設": "government", "防衛施設": "government", "運輸倉庫施設": "warehouse", "工場": "industrial",
               "供給処理施設": "service", "農林漁業用施設": "farm"}


def in_japan(bbox):
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    return 24.0 <= lat <= 45.6 and 122.9 <= lon <= 146.0


def _jp_municipalities(bbox, n=5):
    """JIS municipality codes under the corners and an n x n grid of the box (GSI's reverse geocoder, no key)."""
    s, w, north, e = bbox
    codes = []
    pts = [(s, w), (s, e), (north, w), (north, e)] + [
        (s + (north - s) * (i + 0.5) / n, w + (e - w) * (j + 0.5) / n) for i in range(n) for j in range(n)]
    session = requests.Session()
    for lat, lon in pts:
        for attempt in range(4):    # the geocoder drops connections when asked quickly
            try:
                r = session.get(GSI_REVERSE, params={"lat": f"{lat:.6f}", "lon": f"{lon:.6f}"}, headers=UA,
                                timeout=30)
                r.raise_for_status()
                break
            except requests.RequestException:
                if attempt == 3:
                    raise
                time.sleep(2 * (attempt + 1))
        time.sleep(0.2)
        code = ((r.json() or {}).get("results") or {}).get("muniCd")
        if code and code not in codes:
            codes.append(code)
    return codes


def _plateau_files(codes):
    """Flateau footprint files for the municipalities (a designated city's wards are in the city's file)."""
    import re
    out = []
    for code in codes:
        for c in dict.fromkeys([code, code[:3] + "00"]):
            r = requests.get(FLATEAU_LIST, params={"list-type": "2", "prefix": f"pacificspatial/flateau/parquet/{c}_"},
                             headers=UA, timeout=60)
            r.raise_for_status()
            keys = sorted(k for k in re.findall(r"<Key>([^<]+)</Key>", r.text) if k.endswith("_building_lod0.parquet"))
            if keys:
                url = FLATEAU + keys[-1].rsplit("/", 1)[-1]    # the newest year
                if url not in out:
                    out.append(url)
                break
    return out


def _plateau_rows(url, bbox):
    """(height LOD1, measured height, usage, storeys, geojson, id) of the footprints in the box (DuckDB)."""
    import duckdb
    s, w, n, e = bbox
    con = duckdb.connect()
    con.execute("SET enable_progress_bar=false; INSTALL httpfs; LOAD httpfs; INSTALL spatial; LOAD spatial;")
    return con.execute(f"""SELECT cal_height_m, measured_height, usage, storeys_above_ground, ST_AsGeoJSON(GEOM),
                                  gml_id FROM read_parquet('{url}')
                           WHERE cal_xmax > {w} AND cal_xmin < {e} AND cal_ymax > {s} AND cal_ymin < {n}""").fetchall()


def plateau_buildings(bbox, path):
    """Japan: [{ring, height, cls, measured, id}] from MLIT PLATEAU (aerial-survey heights of the LOD1 model), []
    outside Japan or the cities PLATEAU covers. `height` is the LOD1 block's height (a typical roof height, as
    3DBAG's 70th percentile), `measured` PLATEAU's measuredHeight (to the highest point)."""
    if path.exists():
        return json.loads(path.read_text())
    if not in_japan(bbox):
        return []
    try:
        import duckdb  # noqa: F401
    except ImportError:
        log("PLATEAU skipped: pip install duckdb")
        return []
    urls = _plateau_files(_jp_municipalities(bbox))
    out = []
    for url in urls:
        for lod1, measured, use, floors, g, gid in _plateau_rows(url, bbox):
            ok = lambda v: v is not None and 1.0 < v < 700    # noqa: E731 - PLATEAU uses 0 / 9999 / -9999 for none
            h = lod1 if ok(lod1) else measured if ok(measured) else (3.2 * floors + 1.5 if floors else None)
            for ring in _rings(json.loads(g)):
                out.append({"ring": ring, "height": round(h, 2) if h else None, "cls": PLATEAU_USE.get(use),
                            "measured": round(measured, 2) if ok(measured) else None, "id": gid})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, separators=(",", ":"), ensure_ascii=False))
    log(f"PLATEAU: {len(out)} buildings from {', '.join(u.rsplit('/', 1)[-1] for u in urls) or 'no covered city'}")
    return out


# --- elevation -----------------------------------------------------------------------------------------------

class Dem:
    """A ground elevation grid (metres above sea level) that samples at lat/lon."""

    def __init__(self, z, kind, meta):
        self.z, self.kind, self.meta = z.astype(np.float32), kind, meta
        self._to_rd = None

    def _pixel(self, lat, lon):
        lat, lon = np.asarray(lat, dtype=np.float64), np.asarray(lon, dtype=np.float64)
        m = self.meta
        if self.kind == "merc":
            n = 2.0 ** m["zoom"] * 256
            px = (lon + 180.0) / 360.0 * n - m["x0"]
            py = (1.0 - np.log(np.tan(np.radians(lat)) + 1.0 / np.cos(np.radians(lat))) / math.pi) / 2.0 * n - m["y0"]
            return px, py
        if self._to_rd is None:
            from pyproj import Transformer
            self._to_rd = Transformer.from_crs("EPSG:4326", "EPSG:28992", always_xy=True)
        x, y = self._to_rd.transform(lon, lat)
        return (np.asarray(x) - m["x0"]) / m["res"], (m["y1"] - np.asarray(y)) / m["res"]

    def sample(self, lat, lon):
        """Bilinear elevation at lat/lon (scalars or arrays)."""
        px, py = self._pixel(lat, lon)
        h, w = self.z.shape
        px = np.clip(px - 0.5, 0, w - 1.001)
        py = np.clip(py - 0.5, 0, h - 1.001)
        x0, y0 = np.floor(px).astype(int), np.floor(py).astype(int)
        fx, fy = px - x0, py - y0
        z = self.z
        v = (z[y0, x0] * (1 - fx) * (1 - fy) + z[y0, x0 + 1] * fx * (1 - fy) + z[y0 + 1, x0] * (1 - fx) * fy
             + z[y0 + 1, x0 + 1] * fx * fy)
        return float(v) if np.ndim(v) == 0 else v

    def save(self, path):
        np.savez_compressed(path, z=self.z, kind=self.kind, meta=json.dumps(self.meta))

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        return cls(d["z"], str(d["kind"]), json.loads(str(d["meta"])))


def _fill_holes(z):
    """Fill NaN cells (DTM gaps under buildings and water) from their neighbours."""
    z = z.copy()
    for _ in range(400):
        bad = np.isnan(z)
        if not bad.any():
            break
        p = np.pad(z, 1, constant_values=np.nan)
        nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        cnt = (~np.isnan(nb)).sum(0)
        mean = np.where(cnt > 0, np.nansum(nb, 0) / np.maximum(cnt, 1), np.nan)
        z[bad] = mean[bad]
    return np.where(np.isnan(z), np.nanmean(z), z)


def _box_blur(z, r):
    if r <= 0:
        return z
    k = 2 * r + 1
    p = np.pad(z, r, mode="edge")
    c = p.cumsum(0).cumsum(1)
    c = np.pad(c, ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


def _ahn(bbox, res=2.0):
    import tifffile   # AHN tiles are float32 with the floating-point predictor: also needs imagecodecs
    x0, y0, x1, y1 = _rd_bbox(bbox)
    x0, y0, x1, y1 = math.floor(x0) - 20, math.floor(y0) - 20, math.ceil(x1) + 20, math.ceil(y1) + 20
    r = requests.get(AHN_WCS, params={"SERVICE": "WCS", "VERSION": "2.0.1", "REQUEST": "GetCoverage",
                                      "COVERAGEID": "dtm_05m", "FORMAT": "image/tiff",
                                      "SCALEFACTOR": str(0.5 / res),
                                      "SUBSET": [f"x({x0},{x1})", f"y({y0},{y1})"]}, headers=UA, timeout=300)
    r.raise_for_status()
    z = tifffile.imread(io.BytesIO(r.content)).astype(np.float64)
    z[(z > 1e30) | (z < -1e3)] = np.nan
    res_x = (x1 - x0) / z.shape[1]
    z = _box_blur(_fill_holes(z), 1)
    return Dem(z, "rd", {"x0": x0, "y1": y1, "res": res_x})


def _terrarium(bbox, zoom=15):
    from PIL import Image
    s, w, n, e = bbox

    def tile_xy(lat, lon):
        nt = 2 ** zoom
        x = (lon + 180.0) / 360.0 * nt
        y = (1.0 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2.0 * nt
        return x, y

    tx0, ty0 = (int(v) for v in tile_xy(n, w))
    tx1, ty1 = (int(v) for v in tile_xy(s, e))
    mosaic = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            r = requests.get(TERRARIUM.format(z=zoom, x=tx, y=ty), headers=UA, timeout=60)
            r.raise_for_status()
            a = np.asarray(Image.open(io.BytesIO(r.content)).convert("RGB"), dtype=np.float64)
            mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = \
                a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768
    # most of the world is SRTM (30 m, a surface model): smooth out building and tree bumps
    return Dem(_box_blur(mosaic, 4), "merc", {"zoom": zoom, "x0": tx0 * 256, "y0": ty0 * 256})


def dem(bbox, path, source="auto"):
    """Dem for the box (cached as dem.npz), from AHN in the Netherlands, else the AWS Terrain Tiles."""
    if path.exists():
        d = Dem.load(path)
        return d, d.meta.get("source", d.kind)
    d = None
    if source in ("auto", "ahn") and in_netherlands(bbox):
        try:
            d = _ahn(bbox)
            d.meta["source"] = "ahn"
        except Exception as err:  # noqa: BLE001 - missing tifffile/imagecodecs or PDOK down: fall back
            log(f"AHN failed ({err}); using the AWS Terrain Tiles")
    if d is None:
        d = _terrarium(bbox)
        d.meta["source"] = "terrarium"
    path.parent.mkdir(parents=True, exist_ok=True)
    d.save(path)
    s, w, n, e = bbox
    grid = d.sample(*np.meshgrid(np.linspace(s, n, 30), np.linspace(w, e, 30)))
    log(f"elevation ({d.meta['source']}): {grid.min():.1f} to {grid.max():.1f} m")
    return d, d.meta["source"]


# --- weather and sun -----------------------------------------------------------------------------------------

HOURLY = ["temperature_2m", "relative_humidity_2m", "precipitation", "rain", "snowfall", "cloud_cover",
          "wind_speed_10m", "wind_direction_10m", "is_day", "snow_depth"]   # snow_depth (m): logged only (no snow in CARLA)


def weather_at(lat, lon, when, cache_dir=None):
    """Open-Meteo's hourly weather around `when` (a local naive datetime at the place, or 'now').

    Returns (record, local datetime, utc offset seconds). The record holds the hour's values plus
    precipitation over the 3 hours before (for wet roads) and visibility (forecast API only).
    """
    now = when in (None, "now")
    day = datetime.now(timezone.utc).date() if now else when.date()
    key = f"weather_{lat:.3f}_{lon:.3f}_{day.isoformat()}.json"
    cached = Path(cache_dir) / key if cache_dir else None
    if cached and cached.exists() and not now:
        data = json.loads(cached.read_text())
    else:
        start = (day - timedelta(days=1)).isoformat()
        params = {"latitude": lat, "longitude": lon, "hourly": ",".join(HOURLY), "timezone": "auto",
                  "start_date": start, "end_date": day.isoformat()}
        recent = (datetime.now(timezone.utc).date() - day).days < 80
        if recent:
            params["hourly"] += ",visibility"
            url = METEO
        else:
            url = METEO_ARCHIVE
        r = requests.get(url, params=params, headers=UA, timeout=60)
        r.raise_for_status()
        data = r.json()
        if cached:
            cached.parent.mkdir(parents=True, exist_ok=True)
            cached.write_text(json.dumps(data))
    off = data.get("utc_offset_seconds", 0)
    if now:
        local = datetime.now(timezone.utc).replace(tzinfo=None) + timedelta(seconds=off)
    else:
        local = when
    times = data["hourly"]["time"]
    stamp = local.replace(minute=0, second=0, microsecond=0).strftime("%Y-%m-%dT%H:%M")
    i = times.index(stamp) if stamp in times else len(times) - 1
    rec = {k: v[i] for k, v in data["hourly"].items() if k != "time"}
    rec["precipitation_3h"] = sum(x or 0 for x in data["hourly"]["precipitation"][max(0, i - 3):i + 1])
    rec["time"] = times[i]
    rec["timezone"] = data.get("timezone")
    return rec, local, off


def sun_position(lat, lon, utc):
    """Solar altitude and compass azimuth (degrees, 0 = north, 90 = east) at a UTC datetime (NOAA formulas)."""
    jd = utc.timestamp() / 86400.0 + 2440587.5
    t = (jd - 2451545.0) / 36525.0
    l0 = (280.46646 + t * (36000.76983 + 0.0003032 * t)) % 360
    m = 357.52911 + t * (35999.05029 - 0.0001537 * t)
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)
    c = (math.sin(math.radians(m)) * (1.914602 - t * (0.004817 + 0.000014 * t))
         + math.sin(math.radians(2 * m)) * (0.019993 - 0.000101 * t) + math.sin(math.radians(3 * m)) * 0.000289)
    true_long = l0 + c
    omega = 125.04 - 1934.136 * t
    app_long = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    eps0 = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60
    eps = eps0 + 0.00256 * math.cos(math.radians(omega))
    decl = math.degrees(math.asin(math.sin(math.radians(eps)) * math.sin(math.radians(app_long))))
    y = math.tan(math.radians(eps / 2)) ** 2
    eq_time = 4 * math.degrees(y * math.sin(2 * math.radians(l0)) - 2 * e * math.sin(math.radians(m))
                               + 4 * e * y * math.sin(math.radians(m)) * math.cos(2 * math.radians(l0))
                               - 0.5 * y * y * math.sin(4 * math.radians(l0))
                               - 1.25 * e * e * math.sin(2 * math.radians(m)))
    minutes = utc.hour * 60 + utc.minute + utc.second / 60
    tst = (minutes + eq_time + 4 * lon) % 1440
    ha = tst / 4 - 180
    zen = math.degrees(math.acos(max(-1.0, min(1.0, math.sin(math.radians(lat)) * math.sin(math.radians(decl))
                                               + math.cos(math.radians(lat)) * math.cos(math.radians(decl))
                                               * math.cos(math.radians(ha))))))
    alt = 90 - zen
    az = math.degrees(math.atan2(math.sin(math.radians(ha)),
                                 math.cos(math.radians(ha)) * math.sin(math.radians(lat))
                                 - math.tan(math.radians(decl)) * math.cos(math.radians(lat))))
    return alt, (az + 180) % 360


def carla_weather_fields(rec, alt, az):
    """Open-Meteo hour + sun position -> carla.WeatherParameters fields (see README for the mapping)."""
    rain = (rec.get("rain") or 0) + (rec.get("snowfall") or 0) * 0.7          # mm/h
    vis = rec.get("visibility")
    fog = 0.0 if vis is None or vis >= 10000 else min(100.0, 100.0 * (1 - vis / 10000.0) ** 1.5)
    return {
        # 100 % cover -> 60: above about 60 CARLA 0.9.16 renders an OpenDRIVE world's sky dark grey to black
        # (sky brightness 66, 18 and 1 of 255 at 60, 80 and 90, measured)
        "cloudiness": 0.6 * float(rec.get("cloud_cover") or 0),
        "precipitation": min(100.0, rain * 20.0),
        "precipitation_deposits": min(100.0, (rec.get("precipitation_3h") or 0) * 15.0),
        "wetness": min(100.0, (rec.get("precipitation_3h") or 0) * 25.0),
        "wind_intensity": min(100.0, (rec.get("wind_speed_10m") or 0) * 2.0),
        "fog_density": fog,
        "fog_distance": 0.0 if vis is None else min(150.0, vis / 20.0),
        "sun_altitude_angle": alt,
        # CARLA's sun azimuth is the yaw the light travels along (0: light towards +x = east, so the sun in the
        # west; measured with a pole's shadow): compass bearing of the sun + 90
        "sun_azimuth_angle": (az + 90.0) % 360.0,
    }
