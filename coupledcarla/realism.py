"""How close a map's render is to the real street: real street-level photos with position and heading (open, keyless
sources), the same views rendered in CARLA, and their comparison (semantic layout, skyline, image statistics).

The CARLA side is in tools/realism_check.py (render); everything here runs without CARLA, SUMO or a GPU:
- fetch():   photo metadata in the map's box from Panoramax, KartaView and Wikimedia Commons, cached in real/
- select():  the views to compare: one per place (thinned), daytime only, 360-degree panoramas cut into forward views
- reproject(): equirectangular or pinhole image -> pinhole view at any yaw/pitch (crops and the alignment search)
- compare_labels(), image_stats(): the metrics; Segmenter: a Cityscapes model (transformers, CPU or GPU)
"""
import csv
import json
import math
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

SOURCES = {
    "panoramax": ("Panoramax, the open federated street-level imagery commons (api.panoramax.xyz)", "CC BY-SA 4.0"),
    "kartaview": ("KartaView street-level photos (api.kartaview.org, keyless v2 API)", "CC BY-SA 4.0"),
    "commons": ("Wikimedia Commons geotagged photos with a camera heading", "per file (CC BY, CC BY-SA, CC0, PD)"),
    "mapillary": ("Mapillary street-level imagery (tiles + graph API; needs the token mapillary_token)", "CC BY-SA 4.0"),
}
# Camera height above the ground (m) by how the photo was taken; the panoramas of the box are from a pole on a
# cyclist's backpack, the phones are held by pedestrians, KartaView's are phones/dashcams behind a windscreen.
# Mapillary: panoramas mostly from car roofs (2.2 m), perspective images mostly dashcams and phones behind a windscreen.
HEIGHTS = {"pano": 2.1, "action": 1.1, "phone": 1.5, "kartaview": 1.4, "commons": 1.6, "dashcam": 1.4,
           "mly_pano": 2.2, "mapillary": 1.4}
DEFAULT_FOV = {"phone": 69.0, "kartaview": 65.0, "commons": 60.0, "mapillary": 70.0}
MIN_SUN = 5.0          # degrees: photos with the sun lower than this (dusk, night, tunnels at night) are left out

# Cityscapes train ids -> groups compared. Dynamic road users (person, rider, motorcycle, bicycle) differ by the
# minute and are ignored in both images; vehicles are kept (parked cars are part of the street), but reported apart.
CITYSCAPES = ["road", "sidewalk", "building", "wall", "fence", "pole", "traffic light", "traffic sign", "vegetation",
              "terrain", "sky", "person", "rider", "car", "truck", "bus", "train", "motorcycle", "bicycle"]
GROUPS = ["road", "sidewalk", "building", "pole_sign", "vegetation", "sky", "vehicle"]
IGNORE = 255
_GROUP_OF = {0: 0, 1: 1, 2: 2, 3: 2, 4: 2, 5: 3, 6: 3, 7: 3, 8: 4, 9: 4, 10: 5, 13: 6, 14: 6, 15: 6, 16: 6}
CS_TO_GROUP = np.full(256, IGNORE, np.uint8)
for _k, _g in _GROUP_OF.items():
    CS_TO_GROUP[_k] = _g
# CARLA semantic tags (0.9.14+ and 1.0, Cityscapes-like) -> groups: the render's own ground truth
_CARLA_GROUP = {1: 0, 24: 0, 27: 0, 2: 1, 3: 2, 4: 2, 5: 2, 26: 2, 28: 2, 6: 3, 7: 3, 8: 3, 9: 4, 10: 4, 25: 1,
                11: 5, 14: 6, 15: 6, 16: 6, 17: 6}
CARLA_TO_GROUP = np.full(256, IGNORE, np.uint8)
for _k, _g in _CARLA_GROUP.items():
    CARLA_TO_GROUP[_k] = _g
COARSE = np.full(256, IGNORE, np.uint8)      # groups -> ground, building, vegetation, sky (pole/vehicle: none)
COARSE[[0, 1, 2, 3, 4, 5, 6]] = [0, 0, 1, 254, 2, 3, 254]
PALETTE = np.array([[128, 64, 128], [244, 35, 232], [70, 70, 70], [220, 220, 0], [107, 142, 35], [70, 130, 180],
                    [0, 0, 142]], np.uint8)


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- fetching (cached in the map's real/ folder)

def _get(url, params=None, timeout=60):
    import requests
    from .realworld import UA
    r = requests.get(url, params=params, headers=UA, timeout=timeout)
    r.raise_for_status()
    return r


def _bearing(lat1, lon1, lat2, lon2):
    p1, p2, dl = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def metres(lat1, lon1, lat2, lon2):
    k = 111320.0
    return math.hypot((lat2 - lat1) * k, (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2)))


def fov_from_f35(f35, w, h):
    """Horizontal field of view (deg) from the 35 mm equivalent focal length (diagonal convention)."""
    diag = math.hypot(w, h)
    return math.degrees(2 * math.atan(43.27 * (w / diag) / (2.0 * float(f35))))


def _ratio(text):
    """'3570003/14258' or '12.5' -> float, None if not a number."""
    try:
        if isinstance(text, str) and "/" in text:
            a, b = text.split("/")[:2]
            return float(a) / float(b)
        return float(text)
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def panoramax_record(f):
    """A Panoramax STAC item -> record (or None)."""
    p = f.get("properties") or {}
    az = p.get("view:azimuth")
    if az is None:
        return None
    io = p.get("pers:interior_orientation") or {}
    model = " ".join(x for x in (io.get("camera_manufacturer"), io.get("camera_model")) if x) or None
    fov = io.get("field_of_view")
    exif = p.get("exif") or {}
    lon, lat = f["geometry"]["coordinates"][:2]
    pano = fov == 360
    w, h = (io.get("sensor_array_dimensions") or [None, None])[:2]
    w = w or _ratio(exif.get("Exif.Photo.PixelXDimension"))
    h = h or _ratio(exif.get("Exif.Photo.PixelYDimension"))
    if not pano and not fov:
        f35 = _ratio(exif.get("Exif.Photo.FocalLengthIn35mmFilm"))
        fov = fov_from_f35(f35, w, h) if f35 and w and h else None
    action = bool(model and ("GoPro" in model or "HERO" in model)) and not pano
    kind = "pano" if pano else ("action" if action else "phone")
    assets = f.get("assets") or {}
    nxt = [lk for lk in f.get("links", []) if lk.get("rel") == "next" and "geometry" in lk]
    return {
        "source": "panoramax", "src_id": f["id"], "lat": lat, "lon": lon, "heading": float(az) % 360.0,
        "pitch": float(p.get("pers:pitch") or 0.0), "roll": float(p.get("pers:roll") or 0.0),
        "fov": float(fov) if fov else None, "projection": "equirect" if pano else "pinhole", "kind": kind,
        "width": w, "height": h, "time": p.get("datetime"), "camera": model,
        "accuracy_m": p.get("quality:horizontal_accuracy"),
        "url": (assets.get("hd") or {}).get("href"), "url_sd": (assets.get("sd") or {}).get("href"),
        "page": f"https://api.panoramax.xyz/#focus=pic&pic={f['id']}",
        "licence": "CC BY-SA 4.0" if "BY-SA" in str(p.get("license") or "CC-BY-SA-4.0") else p.get("license"),
        "author": ", ".join(pr.get("name", "") for pr in f.get("providers") or []) or p.get("geovisio:producer"),
        "sequence": f.get("collection"),
        "next": nxt[0]["geometry"]["coordinates"] if nxt else None,
    }


def fetch_panoramax(bbox, get=_get):
    """All Panoramax pictures in bbox [s, w, n, e]: the search returns at most 500, so full tiles are split."""
    s, w, n, e = bbox
    out = {}

    def tile(w_, s_, e_, n_, depth=0):
        feats = get("https://api.panoramax.xyz/api/search",
                    {"bbox": f"{w_:.6f},{s_:.6f},{e_:.6f},{n_:.6f}", "limit": 500}).json()["features"]
        if len(feats) >= 500 and depth < 6:
            mx, my = (w_ + e_) / 2, (s_ + n_) / 2
            for b in ((w_, s_, mx, my), (mx, s_, e_, my), (w_, my, mx, n_), (mx, my, e_, n_)):
                tile(*b, depth=depth + 1)
        else:
            for f in feats:
                out[f["id"]] = f
    tile(w, s, e, n)
    recs = [panoramax_record(f) for f in out.values()]
    return [r for r in recs if r]


def kartaview_record(x):
    head = _ratio(x.get("heading"))
    if head is None:
        return None
    fov = _ratio(x.get("fieldOfView")) or None
    sphere = str(x.get("projection", "")).upper() == "SPHERE"
    return {
        "source": "kartaview", "src_id": str(x["id"]), "lat": float(x["lat"]), "lon": float(x["lng"]),
        "heading": head % 360.0, "pitch": 0.0, "roll": 0.0, "fov": 360.0 if sphere else fov,
        "projection": "equirect" if sphere else "pinhole", "kind": "pano" if sphere else "kartaview",
        "width": _ratio(x.get("width")), "height": _ratio(x.get("height")),
        "time": (x.get("shotDate") or "").replace(" ", "T")[:19] or None,     # local time (no zone given)
        "camera": None, "accuracy_m": _ratio(x.get("gpsAccuracy")),
        "url": x.get("fileurlProc") or x.get("imageProcUrl"), "url_sd": x.get("imageLthUrl"),
        "page": f"https://kartaview.org/details/{x.get('sequenceId')}/{x.get('sequenceIndex')}",
        "licence": "CC BY-SA 4.0", "author": None, "sequence": str(x.get("sequenceId")), "next": None,
    }


def fetch_kartaview(bbox, get=_get):
    """KartaView photos in bbox: the keyless v2 API lists photos around a point (radius in m, 150 per page)."""
    s, w, n, e = bbox
    out = {}
    rows, cols = 7, 7
    for i in range(rows):
        for j in range(cols):
            lat, lng = s + (n - s) * (i + 0.5) / rows, w + (e - w) * (j + 0.5) / cols
            for page in range(1, 50):
                res = get("https://api.kartaview.org/2.0/photo/",
                          {"lat": f"{lat:.6f}", "lng": f"{lng:.6f}", "radius": 250, "itemsPerPage": 150,
                           "page": page}).json()["result"]
                for x in res.get("data") or []:
                    out[str(x["id"])] = x
                if not res.get("hasMoreData"):
                    break
    recs = [kartaview_record(x) for x in out.values() if s <= float(x["lat"]) <= n and w <= float(x["lng"]) <= e]
    return [r for r in recs if r]


_COMPASS = {"N": 0, "NNE": 22.5, "NE": 45, "ENE": 67.5, "E": 90, "ESE": 112.5, "SE": 135, "SSE": 157.5, "S": 180,
            "SSW": 202.5, "SW": 225, "WSW": 247.5, "W": 270, "WNW": 292.5, "NW": 315, "NNW": 337.5}


def compass(text):
    if text is None:
        return None
    t = str(text).strip().upper()
    if t in _COMPASS:
        return float(_COMPASS[t])
    return _ratio(t)


def commons_record(page, pov=None, sdc_heading=None):
    """A Commons file page (imageinfo + wikitext) -> record if it has a camera heading."""
    import re
    ii = (page.get("imageinfo") or [{}])[0]
    if ii.get("mime") != "image/jpeg":
        return None
    md = {m["name"]: m["value"] for m in ii.get("commonmetadata") or [] if "name" in m}
    em = {k: (v or {}).get("value") for k, v in (ii.get("extmetadata") or {}).items()}
    text = (((page.get("revisions") or [{}])[0].get("slots") or {}).get("main") or {}).get("*", "")
    m = re.search(r"\{\{\s*(?:Location|Camera location)(?:\s+dec)?\s*\|[^}]*?heading:([A-Za-z0-9.]+)", text, re.I)
    head = sdc_heading if sdc_heading is not None else (compass(m.group(1)) if m else None)
    if head is None:
        head = _ratio(md.get("GPSImgDirection"))
    if head is None:
        return None
    if pov is None:
        c = (page.get("coordinates") or [{}])[0]
        if "lat" not in c:
            return None
        pov = (c["lat"], c["lon"])
    w, h = ii.get("width"), ii.get("height")
    f35 = _ratio(md.get("FocalLengthIn35mmFilm"))
    fov = fov_from_f35(f35, w, h) if f35 and w and h else None
    date = md.get("DateTimeOriginal") or em.get("DateTimeOriginal") or ""
    date = re.sub(r"^(\d{4}):(\d\d):(\d\d)[ T](\d\d:\d\d(:\d\d)?).*", r"\1-\2-\3T\4", str(date).strip())
    lic = em.get("LicenseShortName")
    author = re.sub(r"<[^>]+>", "", em.get("Artist") or "").strip() or None
    return {
        "source": "commons", "src_id": str(page["pageid"]), "lat": float(pov[0]), "lon": float(pov[1]),
        "heading": float(head) % 360.0, "pitch": 0.0, "roll": 0.0, "fov": fov, "projection": "pinhole",
        "kind": "commons", "width": w, "height": h, "time": date if re.match(r"\d{4}-\d\d-\d\dT", date) else None,
        "camera": md.get("Model"), "accuracy_m": None, "url": ii.get("thumburl") or ii.get("url"), "url_sd": None,
        "page": ii.get("descriptionurl") or f"https://commons.wikimedia.org/?curid={page['pageid']}",
        "licence": lic, "author": author, "sequence": None, "next": None, "title": page.get("title"),
    }


def _api_json(get, url, params, tries=4):
    """A MediaWiki API answer; one that carries an error (maxlag, rate limit) instead of the result is asked again."""
    for k in range(tries):
        j = get(url, params).json()
        if "error" not in j:
            return j
        time.sleep(2.0 * 2 ** k)
    raise RuntimeError(f"Commons API: {j['error'].get('code')}")


def fetch_commons(bbox, get=_get):
    s, w, n, e = bbox
    api = "https://commons.wikimedia.org/w/api.php"
    ids = {}

    def ask(params):
        return _api_json(get, api, params)

    def tile(s_, w_, n_, e_, depth=0):
        g = ask({"action": "query", "list": "geosearch", "gsbbox": f"{n_}|{w_}|{s_}|{e_}", "gsnamespace": 6,
                   "gslimit": 500, "gsprimary": "all", "format": "json"})["query"]["geosearch"]
        if len(g) >= 500 and depth < 5:
            ms, mw = (s_ + n_) / 2, (w_ + e_) / 2
            for b in ((s_, w_, ms, mw), (s_, mw, ms, e_), (ms, w_, n_, mw), (ms, mw, n_, e_)):
                tile(*b, depth=depth + 1)
        else:
            for x in g:
                ids[x["pageid"]] = x
    tile(s, w, n, e)
    out, keys = [], list(ids)
    for k in range(0, len(keys), 50):
        chunk = keys[k:k + 50]
        sdc = {}
        ents = ask({"action": "wbgetentities", "ids": "|".join(f"M{i}" for i in chunk),
                    "format": "json"}).get("entities") or {}
        for mid, ent in ents.items():
            for st in ((ent.get("statements") or {}).get("P1259") or []):     # coordinates of the point of view
                dv = (st.get("mainsnak") or {}).get("datavalue", {}).get("value")
                head = None
                for q in (st.get("qualifiers") or {}).get("P7787", []):         # heading
                    head = float(q["datavalue"]["value"]["amount"])
                if dv:
                    sdc[int(mid[1:])] = ((dv["latitude"], dv["longitude"]), head)
        pages = ask({"action": "query", "pageids": "|".join(map(str, chunk)),
                     "prop": "imageinfo|revisions|coordinates", "iiprop": "url|size|mime|commonmetadata|extmetadata",
                     "iiurlwidth": 1600, "rvprop": "content", "rvslots": "main", "format": "json"})["query"]["pages"]
        for pid, p in pages.items():
            pov, head = sdc.get(int(pid), (None, None))
            r = commons_record(p, pov, head)
            if r and s <= r["lat"] <= n and w <= r["lon"] <= e:
                out.append(r)
    return out


def fetch_mapillary(bbox, cache_dir=None):
    from . import streetphotos
    return streetphotos.fetch_mapillary(bbox, cache_dir)


FETCHERS = {"panoramax": fetch_panoramax, "kartaview": fetch_kartaview, "commons": fetch_commons,
            "mapillary": fetch_mapillary}


def fetch(mdir, bbox, sources=("panoramax", "kartaview", "commons"), refresh=False, log=log, fetchers=None):
    """Records of every source, each cached as real/streetview_<source>.json. A failing source logs one line and
    gives what its cache has (or nothing)."""
    fetchers = fetchers or FETCHERS
    out = []
    for src in sources:
        path = Path(mdir) / "real" / f"streetview_{src}.json"
        recs = None
        if path.exists() and not refresh:
            recs = json.loads(path.read_text())["records"]
        else:
            try:
                t0 = time.time()
                recs = fetchers[src](bbox, Path(mdir) / "real" / "streetphotos") if src == "mapillary" and \
                    fetchers[src] is fetch_mapillary else fetchers[src](bbox)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps({"source": SOURCES[src][0], "licence": SOURCES[src][1], "bbox": bbox,
                                            "fetched": datetime.now().strftime("%Y-%m-%d %H:%M"),
                                            "records": recs}))
                log(f"{src}: {len(recs)} photos with a heading in the box ({time.time() - t0:.0f} s)")
            except Exception as err:  # noqa: BLE001 - offline or the service changed: the cache or nothing
                recs = json.loads(path.read_text())["records"] if path.exists() else []
                log(f"{src}: fetch failed ({err}); {len(recs)} cached photos used")
        out += recs
    return out


# ---------------------------------------------------------------- choosing the views

def utc_time(text, lon):
    """ISO time (with zone, or naive = local solar-ish time) -> aware UTC datetime, or None."""
    if not text:
        return None
    try:
        t = datetime.fromisoformat(str(text).replace("Z", "+00:00"))
    except ValueError:
        return None
    if t.tzinfo is None:       # KartaView and EXIF times are local: Europe/Amsterdam is UTC+1/+2
        return (t - timedelta(hours=_nl_offset(t))).replace(tzinfo=timezone.utc) if 3 <= lon <= 8 else \
            (t - timedelta(hours=lon / 15.0)).replace(tzinfo=timezone.utc)
    return t.astimezone(timezone.utc)


def _nl_offset(t):
    """Central European (summer) time offset in hours for a naive local datetime."""
    y = t.year

    def last_sunday(month):
        d = datetime(y, month, 31)
        return d - timedelta(days=(d.weekday() + 1) % 7)
    start, end = last_sunday(3).replace(hour=2), last_sunday(10).replace(hour=3)
    return 2 if start <= t < end else 1


def local_time(utc, lon):
    """UTC -> the place's local clock (CET/CEST in the Netherlands and its neighbours, solar time elsewhere)."""
    if 3 <= lon <= 8:
        guess = utc.replace(tzinfo=None) + timedelta(hours=1)
        return utc.replace(tzinfo=None) + timedelta(hours=_nl_offset(guess))
    return utc.replace(tzinfo=None) + timedelta(hours=lon / 15.0)


def select(records, bbox, spacing_m=20.0, pano_offsets=(0.0,), max_views=0, include_action=False, min_sun=MIN_SUN,
           size=1024, log=log):
    """The views to render and compare: in the box, with a heading, daytime; one per spacing_m per source and heading
    sector (so a sequence shot every 3 m does not dominate); each panorama gives one view per offset from its
    heading. Returns view dicts sorted along a nearest-neighbour path (the renderer streams less)."""
    from .realworld import sun_position
    s, w, n, e = bbox
    kept, dropped = [], {"outside": 0, "no_time": 0, "night": 0, "action_cam": 0, "near_another": 0}
    for r in sorted(records, key=lambda r: (r["source"], r.get("sequence") or "", r.get("time") or "", r["src_id"])):
        if not (s <= r["lat"] <= n and w <= r["lon"] <= e):
            dropped["outside"] += 1
            continue
        if r["kind"] == "action" and not include_action:     # handlebar GoPros: strong roll, fisheye
            dropped["action_cam"] += 1
            continue
        utc = utc_time(r.get("time"), r["lon"])
        if utc is None:
            dropped["no_time"] += 1
            sun = None
        else:
            sun = sun_position(r["lat"], r["lon"], utc)[0]
            if sun < min_sun:
                dropped["night"] += 1
                continue
        offsets = pano_offsets if r["projection"] == "equirect" else (0.0,)
        for off in offsets:
            head = (r["heading"] + off) % 360.0
            near = [k for k in kept if k["source"] == r["source"] and abs((k["heading"] - head + 180) % 360 - 180) < 45
                    and metres(k["lat"], k["lon"], r["lat"], r["lon"]) < spacing_m]
            if near:
                dropped["near_another"] += 1
                continue
            hfov = 90.0 if r["projection"] == "equirect" else float(r.get("fov") or DEFAULT_FOV.get(r["kind"], 60.0))
            if r["projection"] == "equirect":
                wpx, hpx = size, int(size * 9 / 16)
            else:
                ar = (r["height"] / r["width"]) if r.get("width") and r.get("height") else 0.75
                wpx, hpx = size, int(round(size * ar))
            vid = f"{r['source'][:3]}_{r['src_id'][:18]}" + (f"_{int(round(off)) % 360:03d}" if len(offsets) > 1 else "")
            kept.append({
                "id": vid.replace("-", ""), "source": r["source"], "src_id": r["src_id"], "kind": r["kind"],
                "lat": round(r["lat"], 7), "lon": round(r["lon"], 7), "heading": round(head, 2),
                "pitch": round(r.get("pitch") or 0.0, 2), "roll": round(r.get("roll") or 0.0, 2),
                "hfov": round(hfov, 2), "width": wpx, "height": hpx, "height_m": HEIGHTS.get(r["kind"], 1.6),
                "projection": r["projection"], "src_heading": round(r["heading"], 2),
                "utc": utc.strftime("%Y-%m-%dT%H:%M:%SZ") if utc else "",
                "local_time": local_time(utc, r["lon"]).strftime("%Y-%m-%d %H:%M") if utc else "",
                "sun_alt": round(sun, 1) if sun is not None else "", "url": r.get("url") or "",
                "url_sd": r.get("url_sd") or "", "page": r.get("page") or "", "licence": r.get("licence") or "",
                "author": r.get("author") or "", "camera": r.get("camera") or "", "sequence": r.get("sequence") or "",
                "accuracy_m": r.get("accuracy_m") or "",
            })
    if max_views and len(kept) > max_views:
        step = len(kept) / max_views
        kept = [kept[int(i * step)] for i in range(max_views)]
    log(f"views: {len(kept)} kept; left out: " + ", ".join(f"{v} {k.replace('_', ' ')}" for k, v in dropped.items()))
    return route_order(kept)


def daytime(records, min_sun=MIN_SUN):
    """The records shot with the sun at least min_sun degrees up (records without a time are left out)."""
    from .realworld import sun_position
    out = []
    for r in records:
        utc = utc_time(r.get("time"), r["lon"])
        if utc is not None and sun_position(r["lat"], r["lon"], utc)[0] >= min_sun:
            out.append(r)
    return out


def route_order(views):
    """Greedy nearest-neighbour order starting at the most western view."""
    if not views:
        return views
    left = list(views)
    cur = min(left, key=lambda v: v["lon"])
    out = [cur]
    left.remove(cur)
    while left:
        cur = min(left, key=lambda v: metres(cur["lat"], cur["lon"], v["lat"], v["lon"]))
        out.append(cur)
        left.remove(cur)
    return out


VIEW_FIELDS = ["id", "source", "src_id", "kind", "lat", "lon", "heading", "pitch", "roll", "hfov", "width", "height",
               "height_m", "projection", "src_heading", "utc", "local_time", "sun_alt", "url", "url_sd", "page",
               "licence", "author", "camera", "sequence", "accuracy_m"]
NUM = {"lat", "lon", "heading", "pitch", "roll", "hfov", "height_m", "src_heading"}


def write_views(path, views):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        wr = csv.DictWriter(f, VIEW_FIELDS, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(views)


def read_views(path):
    out = []
    with Path(path).open(newline="") as f:
        for r in csv.DictReader(f):
            for k in NUM:
                r[k] = float(r[k])
            r["width"], r["height"] = int(r["width"]), int(r["height"])
            out.append(r)
    return out


# ---------------------------------------------------------------- camera geometry

def pinhole_rays(w, h, hfov, yaw=0.0, pitch=0.0, roll=0.0):
    """Unit view directions (h, w, 3) as (right, up, forward) in the frame of the nominal camera, for a pinhole camera
    turned by yaw (right positive), pitch (up positive) and roll (clockwise positive), degrees."""
    f = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    u = (np.arange(w) + 0.5 - w / 2.0) / f
    v = (h / 2.0 - (np.arange(h) + 0.5)) / f
    uu, vv = np.meshgrid(u, v)
    d = np.stack([uu, vv, np.ones_like(uu)], -1)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    rz = np.array([[math.cos(r), math.sin(r), 0], [-math.sin(r), math.cos(r), 0], [0, 0, 1]])     # roll
    rx = np.array([[1, 0, 0], [0, math.cos(p), math.sin(p)], [0, -math.sin(p), math.cos(p)]])     # pitch up
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])     # yaw right
    rot = ry @ rx @ rz
    return d @ rot.T


def _sample(img, xs, ys, nearest):
    hs, ws = img.shape[:2]
    if nearest:
        xi = np.clip(np.floor(xs).astype(int), 0, ws - 1)
        yi = np.clip(np.floor(ys).astype(int), 0, hs - 1)
        return img[yi, xi]
    x0 = np.floor(xs - 0.5)
    y0 = np.floor(ys - 0.5)
    fx, fy = xs - 0.5 - x0, ys - 0.5 - y0
    x0, y0 = x0.astype(int), y0.astype(int)
    a = img.astype(np.float32)

    def px(yy, xx):
        return a[np.clip(yy, 0, hs - 1), np.clip(xx, 0, ws - 1)]
    if a.ndim == 3:
        fx, fy = fx[..., None], fy[..., None]
    out = (px(y0, x0) * (1 - fx) * (1 - fy) + px(y0, x0 + 1) * fx * (1 - fy) + px(y0 + 1, x0) * (1 - fx) * fy
           + px(y0 + 1, x0 + 1) * fx * fy)
    return out


def reproject(src, src_proj, dst_w, dst_h, dst_hfov, yaw=0.0, pitch=0.0, roll=0.0, src_hfov=None, src_yaw=0.0,
              src_pitch=0.0, nearest=False, fill=0):
    """A pinhole view (dst_w x dst_h, dst_hfov) of a source image, turned by yaw/pitch/roll from the source's nominal
    direction. src_proj 'equirect': the centre column looks along the nominal heading (src_yaw = heading of the
    centre column minus the view's heading). 'pinhole': a camera with src_hfov looking along src_yaw/src_pitch.
    Pixels the source does not cover get `fill`. nearest=True for label maps."""
    d = pinhole_rays(dst_w, dst_h, dst_hfov, yaw, pitch, roll)
    hs, ws = src.shape[:2]
    if src_proj == "equirect":
        az = np.degrees(np.arctan2(d[..., 0], d[..., 2])) - src_yaw
        el = np.degrees(np.arcsin(np.clip(d[..., 1], -1, 1)))
        xs = ((az / 360.0 + 0.5) % 1.0) * ws
        ys = (0.5 - el / 180.0) * hs
        valid = np.ones(xs.shape, bool)
    else:
        if src_yaw or src_pitch:      # rays into the source camera's frame
            y, p = math.radians(src_yaw), math.radians(src_pitch)
            ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
            rx = np.array([[1, 0, 0], [0, math.cos(p), math.sin(p)], [0, -math.sin(p), math.cos(p)]])
            d = d @ (ry @ rx)
        f = (ws / 2.0) / math.tan(math.radians(src_hfov) / 2.0)
        z = d[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            xs = ws / 2.0 + f * d[..., 0] / z
            ys = hs / 2.0 - f * d[..., 1] / z
        valid = (z > 1e-6) & (xs >= 0) & (xs < ws) & (ys >= 0) & (ys < hs)
        xs, ys = np.where(valid, xs, 0), np.where(valid, ys, 0)
    out = _sample(src, xs, ys, nearest)
    if src_proj != "equirect":
        out = np.where(valid[..., None] if out.ndim == 3 else valid, out, fill)
    return out.astype(src.dtype) if nearest else np.clip(out, 0, 255).astype(np.uint8)


def wide_size(width, height, hfov, yaw_margin=15.0, pitch_margin=8.0, max_hfov=150.0):
    """The render's (w, h, hfov): wide enough that the view turned by up to +-yaw_margin and +-pitch_margin (both at
    once, corners included) stays inside it, at the view's pixel density."""
    f = (width / 2.0) / math.tan(math.radians(hfov) / 2.0)
    tx = ty = 0.0
    for dy in (-yaw_margin, 0.0, yaw_margin):
        for dp in (-pitch_margin, 0.0, pitch_margin):
            d = pinhole_rays(width, height, hfov, dy, dp)
            edge = np.concatenate([d[0], d[-1], d[:, 0], d[:, -1]])
            tx = max(tx, float(np.max(np.abs(edge[:, 0] / edge[:, 2]))))
            ty = max(ty, float(np.max(np.abs(edge[:, 1] / edge[:, 2]))))
    tx = min(tx, math.tan(math.radians(max_hfov) / 2))
    w, h = int(math.ceil(2 * f * tx)) + 2, int(math.ceil(2 * f * ty)) + 2
    w, h = w + w % 2, h + h % 2
    return w, h, math.degrees(2 * math.atan((w / 2.0) / f))


# ---------------------------------------------------------------- metrics

def groups_of(labels, table=CS_TO_GROUP):
    return table[labels]


def fractions(g):
    valid = g != IGNORE
    n = max(1, int(valid.sum()))
    return {name: float((g == i).sum()) / n for i, name in enumerate(GROUPS)}


def skyline(g, hfov):
    """Per column: elevation (deg, camera frame) of the highest non-sky pixel; NaN where the column is all sky or
    ignored at the top. Pinhole geometry."""
    h, w = g.shape
    f = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    sky = g == GROUPS.index("sky")
    other = (g != IGNORE) & ~sky
    first = np.where(other.any(0), other.argmax(0), h)
    top_ignored = g[0] == IGNORE
    el = np.degrees(np.arctan((h / 2.0 - (first + 0.5)) / f))
    el = np.where(first >= h, np.nan, el)
    return np.where(top_ignored & ~sky[0], np.nan, el)


def compare_labels(real, render, hfov):
    """Group maps (same size) -> metrics: pixel accuracy, IoU per group and their mean, the total variation distance
    of the group areas (0 = same composition, 1 = disjoint), skyline mean absolute difference (deg)."""
    valid = (real != IGNORE) & (render != IGNORE)
    n = max(1, int(valid.sum()))
    out = {"pix_acc": float(((real == render) & valid).sum()) / n}
    ious = []
    fr, fd = fractions(np.where(valid, real, IGNORE)), fractions(np.where(valid, render, IGNORE))
    for i, name in enumerate(GROUPS):
        a, b = (real == i) & valid, (render == i) & valid
        union = int((a | b).sum())
        iou = float((a & b).sum()) / union if union else float("nan")
        out[f"iou_{name}"] = iou
        if union >= 0.005 * n:
            ious.append(iou)
        out[f"real_{name}"] = fr[name]
        out[f"render_{name}"] = fd[name]
    out["miou"] = float(np.mean(ious)) if ious else float("nan")
    out["layout_tv"] = 0.5 * sum(abs(fr[k] - fd[k]) for k in GROUPS)
    # coarse: ground (road + sidewalk: Dutch red cycle paths and brick pavements swap between the two), building,
    # vegetation, sky; the robust score (the alignment search uses it)
    cr, cd = COARSE[real], COARSE[render]
    out["pix_acc_coarse"] = float(((cr == cd) & valid).sum()) / n
    cious = []
    for i in range(4):
        a, b = (cr == i) & valid, (cd == i) & valid
        union = int((a | b).sum())
        if union >= 0.005 * n:
            cious.append(float((a & b).sum()) / union)
    out["miou_coarse"] = float(np.mean(cious)) if cious else float("nan")
    sa, sb = skyline(real, hfov), skyline(render, hfov)
    both = ~np.isnan(sa) & ~np.isnan(sb)
    out["skyline_mae_deg"] = float(np.mean(np.abs(sa[both] - sb[both]))) if both.any() else float("nan")
    out["skyline_corr"] = float(np.corrcoef(sa[both], sb[both])[0, 1]) if both.sum() > 10 and \
        np.std(sa[both]) > 1e-6 and np.std(sb[both]) > 1e-6 else float("nan")
    return out


def image_stats(rgb, mask=None):
    """Simple appearance statistics of an RGB uint8 image (optionally over a mask): luminance mean/contrast,
    saturation, colourfulness (Hasler and Suesstrunk), edge density."""
    a = rgb.astype(np.float32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    lum = 0.2126 * r + 0.7152 * g + 0.0722 * b
    mx, mn = a.max(-1), a.min(-1)
    sat = np.where(mx > 0, (mx - mn) / np.maximum(mx, 1e-6), 0)
    rg, yb = r - g, 0.5 * (r + g) - b
    m = np.ones(lum.shape, bool) if mask is None else mask
    if m.sum() < 10:
        m = np.ones(lum.shape, bool)
    gx = np.zeros_like(lum)
    gy = np.zeros_like(lum)
    gx[:, 1:-1] = lum[:, 2:] - lum[:, :-2]
    gy[1:-1] = lum[2:] - lum[:-2]
    edges = np.hypot(gx, gy) > 40.0
    return {"lum_mean": float(lum[m].mean()), "lum_std": float(lum[m].std()), "sat_mean": float(sat[m].mean()),
            "colourfulness": float(math.hypot(rg[m].std(), yb[m].std()) + 0.3 * math.hypot(rg[m].mean(), yb[m].mean())),
            "edge_density": float(edges[m].mean())}


def colour_hist_sim(a, b, mask=None, bins=8):
    """Histogram intersection (0..1) of two RGB images' 8x8x8 colour histograms."""
    def hist(x):
        q = (x.astype(np.int32) * bins // 256)
        idx = (q[..., 0] * bins + q[..., 1]) * bins + q[..., 2]
        idx = idx[mask] if mask is not None and mask.sum() >= 10 else idx.ravel()
        hh = np.bincount(idx, minlength=bins ** 3).astype(np.float64)
        return hh / max(1.0, hh.sum())
    return float(np.minimum(hist(a), hist(b)).sum())


def _score(m):
    return m["miou_coarse"] if not math.isnan(m["miou_coarse"]) else -1.0


def align(real_g, wide_g, hfov, w, h, wide_hfov, yaw_range=12.0, pitch_range=6.0, step=3.0, center=(0.0, 0.0)):
    """Photo with a fixed frame (a pinhole photo): the yaw/pitch offset (deg) of the render view, cut from the wider
    render, that best matches the photo's layout (coarse mIoU). It separates pose errors of the photo (heading,
    unknown pitch) from differences of the twin. Returns (heading error = the offset: the photo looks that much
    right of / above its stated direction, dpitch, render group map there, metrics). The search runs around
    `center`; ranges 0 take `center` as it is (an alignment reused from an earlier round)."""
    best = None
    for dy in center[0] + np.arange(-yaw_range, yaw_range + 1e-6, step):
        for dp in center[1] + np.arange(-pitch_range, pitch_range + 1e-6, step):
            g = reproject(wide_g, "pinhole", w, h, hfov, yaw=float(dy), pitch=float(dp), src_hfov=wide_hfov,
                          nearest=True, fill=IGNORE)
            m = compare_labels(real_g, g, hfov)
            if best is None or _score(m) > best[0]:
                best = (_score(m), float(dy), float(dp), g, m)
    return best[1], best[2], best[3], best[4]


def align_pano(pano_g, render_g, hfov, w, h, src_yaw, yaw_range=30.0, pitch_range=6.0, step=3.0, pitch0=0.0,
               center=(0.0, 0.0)):
    """Panorama photo: the render stays at the stated pose and the photo is cut at yaw/pitch offsets from its labelled
    panorama (no margin needed, so a wider yaw range: the panoramas' headings come from the GPS track). Returns
    (heading error: the true direction of the stated heading's column is stated + this, dpitch, the photo's group map
    there, metrics, (dy, dp) of the cut). The search runs around the cut `center`; ranges 0 take it as it is."""
    best = None
    for dy in center[0] + np.arange(-yaw_range, yaw_range + 1e-6, step):
        for dp in center[1] + np.arange(-pitch_range, pitch_range + 1e-6, step):
            g = reproject(pano_g, "equirect", w, h, hfov, yaw=float(dy), pitch=pitch0 + float(dp), src_yaw=src_yaw,
                          nearest=True)
            m = compare_labels(g, render_g, hfov)
            if best is None or _score(m) > best[0]:
                best = (_score(m), float(dy), float(dp), g, m)
    return -best[1], -best[2], best[3], best[4], (best[1], best[2])


# ---------------------------------------------------------------- segmentation model

class Segmenter:
    """Cityscapes semantic segmentation with a transformers model (default SegFormer-B2, 19 classes)."""

    def __init__(self, model="nvidia/segformer-b2-finetuned-cityscapes-1024-1024", device=None, width=1024):
        import torch
        from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
        self.torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.proc = AutoImageProcessor.from_pretrained(model)
        self.model = AutoModelForSemanticSegmentation.from_pretrained(model).to(self.device).eval()
        self.width = width
        self.name = model

    def __call__(self, rgb, width=None):
        """uint8 (h, w, 3) -> Cityscapes train ids (h, w) uint8, inferred at `width` px (panoramas: 2048 x 1024,
        Cityscapes' own frame size)."""
        from PIL import Image
        h, w = rgb.shape[:2]
        width = width or self.width
        im = Image.fromarray(rgb)
        if w != width:
            im = im.resize((width, int(round(h * width / w))), Image.BILINEAR)
        inp = self.proc(images=im, return_tensors="pt", do_resize=False).to(self.device)
        with self.torch.no_grad():
            logits = self.model(**inp).logits
            logits = self.torch.nn.functional.interpolate(logits, size=(h, w), mode="bilinear", align_corners=False)
        return logits.argmax(1)[0].to("cpu").numpy().astype(np.uint8)


def colourise(g):
    out = np.zeros(g.shape + (3,), np.uint8)
    for i in range(len(GROUPS)):
        out[g == i] = PALETTE[i]
    return out
