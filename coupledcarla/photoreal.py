"""Google Photorealistic 3D Tiles in a baked CARLA 1.0 level, under the twin's own roads (map.photoreal: google).

Opt-in, off by default. The photogrammetry mesh of Google's Map Tiles API is streamed into the level by Cesium for
Unreal on the CARLA server; the CoupledPhotoreal plugin (unreal/CoupledPhotoreal, actor blueprint.cc_photoreal_google)
places it and this module prepares and switches it:

- Roads: CARLA's OpenDRIVE roads (smooth surface, lane markings) stay. The photo mesh is cut out above the twin's
  carriageway (or also its sidewalks, `carve: road`) by a Cesium polygon clipping overlay: carve_rings() turns the
  twin's OpenDRIVE lanes (odroutline.py) into hole-free polygons (Cesium's cartographic polygons have no holes).
- Buildings and ground: the twin's 3DBAG buildings and its photo ground are hidden where the photo mesh covers them
  (visibility only: collision, LiDAR and navigation still use them, so cars drive on CARLA's roads and walkers on its
  sidewalks). At night the twin's buildings come back (their windows light up) and the photo mesh is cut along the
  building footprints instead (night_buildings).
- Datum: Google's mesh is in WGS84 (an ITRF realisation) with ellipsoidal heights; the twin is ETRS89 (Dutch RD
  data) with NAP heights. georef() puts the level's origin into Google's frame: ellipsoidal height = NAP + the
  NLGEO2018 quasi-geoid height (43.93 m at the Eindhoven twin's origin; EGM96/EGM2008 elsewhere), plus the
  ETRS89 -> ITRF2020 drift at the current epoch (0.90 m to the north-east in 2026). `height_offset_m` and `shift_m`
  take a measured correction (`python -m coupledcarla.photoreal measure`, which costs one root request).
- Billing (Google's pages, 2026-11-15): Photorealistic 3D Tiles is an Enterprise SKU; 1,000 billable events a month
  free, then USD 6.00 per 1,000; a root tileset request is the billable event and opens a session of up to three
  hours of tile requests; 10,000 root requests a day per project. Every level load spawns one actor, which sends one
  root request (the server counts it in ~/.config/coupled-carla/google_tiles_usage.json and refuses beyond
  `day_cap` / `month_cap`, default 50 a day and the 1,000 free a month). Nothing is requested per tick or per tile.
- Terms: tiles are streamed, never stored beyond their Cache-Control max-age (TileSession keeps them in memory only),
  never baked into packages or content packs; rendered views and saved frames carry Google's attribution (stamp()).
- Key: the server machine's own, at run time: $GOOGLE_MAPS_API_KEY or ~/.config/coupled-carla/google_maps_key
  (mode 600). It is sent as the X-Goog-Api-Key header, never in a URL, a log line, a file of ours or over CARLA's RPC.
  Without a key (or the plugin, or the level data) the run goes on with the normal twin after one log line.

    python -m coupledcarla.photoreal prepare experiments/eindhoven_twin_photoreal.yaml   # level data -> content pack
    python -m coupledcarla.photoreal status                                              # key found? usage
    python -m coupledcarla.photoreal measure experiments/eindhoven_twin_photoreal.yaml   # 1 root: datum, alignment
"""
import argparse
import copy
import json
import math
import os
import re
import sys
import time
from pathlib import Path
from urllib.parse import urljoin, urlsplit, urlunsplit, parse_qsl, urlencode

SOURCES = {
    "google_3dtiles": ("Google Maps Platform Map Tiles API, Photorealistic 3D Tiles (needs the user's own API key "
                       "and billing)", "Google Maps Platform Terms of Service: streamed only, no storage beyond "
                       "Cache-Control, no redistribution, attribution and logo on screen"),
    "cesium_unreal": ("Cesium for Unreal v2.31.0 (CesiumGS), built for CARLA's UE 5.8 fork", "Apache License 2.0"),
    "nlgeo2018": ("NLGEO2018 quasi-geoid (NSGI) via PROJ's grid CDN, for NAP -> ETRS89 ellipsoidal heights",
                  "CC0 / public (PROJ-data)"),
}

DEFAULTS = {
    "provider": "google",
    "carve": "carriageway",      # cut the photo mesh above the twin's "carriageway" lanes, "road" (with sidewalks),
                                 # or "none"
    "hide": ["building", "terrain"],    # twin meshes hidden under the photo mesh (visibility only; collision stays):
                                 # building, terrain, ground, sidewalk, road, roadline, vegetation
    "night_buildings": True,     # at night show the twin's buildings (lit windows), cut the photo mesh at footprints
    "light": "auto",             # "baked": the photos as they are (unlit, their own sun and shadows); "relit": lit by
                                 # CARLA's sun and lamps (shadows doubled by day); "auto": baked by day, relit at
                                 # night (so street lamps light the photo streets). Fixed per level load: changing
                                 # it makes Cesium reload the tileset, i.e. another billed root request
    "max_sse": 16,               # Cesium's maximum screen-space error (px): lower = finer tiles, more traffic
    "day_cap": 50,               # root requests a day this machine allows itself (also set a Cloud quota cap)
    "month_cap": 1000,           # ... a calendar month (1,000 are free; above that USD 6 per 1,000)
    "run_cap": 2,                # root requests one level load may cause (the first, plus one renewal after
                                 # session_h); a tileset that reloads beyond it is stopped
    "session_h": 2.9,            # renew the tile session after this many hours (a session lasts up to three)
    "shift_m": "auto",           # [east, north, up] from the level's datum to Google's, or "auto": ETRS89/NAD83 ->
                                 # ITRF2020 at the current epoch for Europe/the US, 0 elsewhere
    "height_offset_m": 0.0,      # measured: how much higher the photo mesh stands than the twin (m); lowers the mesh
    "vdatum": "auto",            # the level's heights: "NAP" (the Netherlands), "EGM96", "EGM2008", "ellipsoid", or
                                 # "auto" (NAP inside the Netherlands, else the OpenDRIVE geoReference's geoid)
    "carve_grow_m": 0.0,         # widen the road cut (m) when the photo kerbs stand inside the twin's roads
    "key_in": "header",          # how the key goes to Google: "header" (X-Goog-Api-Key, never in a URL or log) or
                                 # "url" (?key=, a fallback should the header be refused; then in Cesium's logs)
    "attribution": "auto",       # text stamped on saved frames: "auto" = "Google" plus the credits the server
                                 # wrote (~/.config/coupled-carla/google_tiles_credits.txt on the same machine)
}
CARVE = ("carriageway", "road", "none")
HIDE = ("building", "terrain", "ground", "sidewalk", "road", "roadline", "vegetation", "none")
LIGHT = ("baked", "relit", "auto")
VDATUM = ("auto", "NAP", "EGM96", "EGM2008", "ellipsoid")
DAILY_QUOTA = 10000             # Google's per-project limit of root requests a day

GOOGLE_ROOT = "https://tile.googleapis.com/v1/3dtiles/root.json"
SESSION_S = 3 * 3600.0          # a root request opens a session of up to three hours
KEY_ENV = "GOOGLE_MAPS_API_KEY"
BLUEPRINT = "blueprint.cc_photoreal_google"
PACK = Path(__file__).resolve().parent / "data" / "carla_packs" / "CoupledCarla"
NL_BOX = (50.6, 3.2, 53.7, 7.3)     # s, w, n, e
EU_BOX = (34.0, -11.0, 71.5, 32.0)
US_BOX = (24.0, -125.0, 49.5, -66.0)
R_EARTH = 6371000.0


class PhotorealError(RuntimeError):
    pass


def log(msg):
    print(f"[photoreal] {msg}", flush=True)


# --------------------------------------------------------------------------------------------------- settings

def settings(map_cfg):
    """The photoreal settings of a config's map section (DEFAULTS filled in), or None when it is off."""
    v = (map_cfg or {}).get("photoreal")
    if v in (None, False, "", "off", "none", "false"):
        return None
    if v is True:
        v = {"provider": "google"}
    elif isinstance(v, str):
        v = {"provider": v}
    elif not isinstance(v, dict):
        raise PhotorealError(f"map.photoreal: {v!r} is not google, false or a mapping")
    out = copy.deepcopy(DEFAULTS)
    out.update(v)
    if isinstance(out["hide"], str):
        out["hide"] = [x.strip() for x in out["hide"].split(",") if x.strip()]
    return out


def problems(cfg):
    """[(level, message)] for map.photoreal of a loaded experiment (validate.py prints them)."""
    mc = cfg.get("map") or {}
    raw = mc.get("photoreal")
    try:
        s = settings(mc)
    except PhotorealError as err:
        return [("error", str(err))]
    if s is None:
        return []
    out = []
    if isinstance(raw, dict):
        for k in raw:
            if k not in DEFAULTS:
                out.append(("error", f"unknown key {k!r} (known: {', '.join(sorted(DEFAULTS))})"))
    if s["provider"] != "google":
        out.append(("error", f"provider {s['provider']!r}: only google"))
    if s["carve"] not in CARVE:
        out.append(("error", f"carve {s['carve']!r} is not one of {', '.join(CARVE)}"))
    bad = [h for h in s["hide"] if h not in HIDE]
    if bad:
        out.append(("error", f"hide: unknown {', '.join(bad)} (known: {', '.join(HIDE)})"))
    if s["light"] not in LIGHT:
        out.append(("error", f"light {s['light']!r} is not one of {', '.join(LIGHT)}"))
    if s["key_in"] not in ("header", "url"):
        out.append(("error", f"key_in {s['key_in']!r} is not header or url"))
    if s["vdatum"] not in VDATUM:
        out.append(("error", f"vdatum {s['vdatum']!r} is not one of {', '.join(VDATUM)}"))
    for k, lo, hi in (("day_cap", 1, DAILY_QUOTA), ("month_cap", 1, 31 * DAILY_QUOTA), ("run_cap", 1, 10)):
        if not isinstance(s[k], int) or isinstance(s[k], bool) or not lo <= s[k] <= hi:
            out.append(("error", f"{k} {s[k]!r}: a whole number {lo}..{hi}"))
    if isinstance(s["month_cap"], int) and s["month_cap"] > 1000:
        out.append(("warn", f"month_cap {s['month_cap']} is above the 1,000 free root requests a month "
                            f"(USD 6.00 per 1,000 above it)"))
    if not _num(s["max_sse"]) or s["max_sse"] <= 0:
        out.append(("error", f"max_sse {s['max_sse']!r}: a positive number of pixels"))
    if not _num(s["session_h"]) or not 0 <= s["session_h"] <= 3:
        out.append(("error", f"session_h {s['session_h']!r}: hours, 0..3 (a session lasts up to three)"))
    if s["shift_m"] != "auto" and not (isinstance(s["shift_m"], (list, tuple)) and len(s["shift_m"]) == 3
                                       and all(_num(x) for x in s["shift_m"])):
        out.append(("error", f"shift_m {s['shift_m']!r}: auto or [east, north, up] in metres"))
    if not _num(s["height_offset_m"]) or abs(s["height_offset_m"]) > 100:
        out.append(("error", f"height_offset_m {s['height_offset_m']!r}: metres"))
    if not _num(s["carve_grow_m"]) or not -2 <= s["carve_grow_m"] <= 5:
        out.append(("error", f"carve_grow_m {s['carve_grow_m']!r}: metres, -2..5"))
    if not mc.get("carla_level"):
        out.append(("error", "needs a baked CARLA 1.0 level (map.carla_level) whose buildings it can hide"))
    if not mc.get("opendrive") and s["carve"] != "none":
        out.append(("error", "the road cut needs the level's OpenDRIVE (map.opendrive), or carve: none"))
    return out


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def role_options(s, night=False):
    """The actor's options as its role_name ("k=v;k=v"; CCPhotorealTiles.cpp reads them). Never the key."""
    relit = s["light"] == "relit" or (s["light"] == "auto" and night)
    hide = [h for h in s["hide"] if h != "none"] or ["none"]
    opts = {"carve": 0 if s["carve"] == "none" else 1, "hide": ",".join(hide), "relit": int(relit),
            "night": int(bool(s["night_buildings"])), "sse": s["max_sse"], "day_cap": s["day_cap"],
            "month_cap": s["month_cap"], "run_cap": s["run_cap"], "session_h": s["session_h"],
            "dz": s["height_offset_m"], "key_in": s["key_in"]}
    if s.get("test_url"):                 # tools/photoreal_check.py: a local mock server, never Google
        opts["url"] = s["test_url"]
        if s.get("test_refresh_s"):
            opts["test_refresh_s"] = s["test_refresh_s"]
    return ";".join(f"{k}={v}" for k, v in opts.items())


# -------------------------------------------------------------------------------------------------------- key

def config_dir(env=None):
    """~/.config/coupled-carla (Windows: %USERPROFILE%\\.config\\coupled-carla), where the key and ledger live."""
    env = os.environ if env is None else env
    if env.get("COUPLED_CARLA_CONFIG_DIR"):     # tests; shared lab machines (the plugin reads it too)
        return Path(env["COUPLED_CARLA_CONFIG_DIR"])
    home = env.get("USERPROFILE") if os.name == "nt" else env.get("HOME")
    return Path(home or Path.home()) / ".config" / "coupled-carla"


def read_key(env=None, cfgdir=None):
    """(key, source) on this machine: $GOOGLE_MAPS_API_KEY, else the first line of config_dir()/google_maps_key;
    (None, None) without one. Never print the key."""
    env = os.environ if env is None else env
    k = (env.get(KEY_ENV) or "").strip()
    if k:
        return k, "env"
    f = Path(cfgdir or config_dir(env)) / "google_maps_key"
    try:
        for line in f.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                return line, "file"
    except OSError:
        pass
    return None, None


def key_status(env=None, cfgdir=None):
    """One line about the key on this machine, without the key."""
    cfgdir = Path(cfgdir or config_dir(env))
    key, src = read_key(env, cfgdir)
    if key is None:
        return f"no Google Maps API key ({KEY_ENV} unset, {cfgdir / 'google_maps_key'} missing)"
    msg = f"Google Maps API key found ({'$' + KEY_ENV if src == 'env' else cfgdir / 'google_maps_key'}, " \
          f"{len(key)} characters)"
    if src == "file" and os.name != "nt":
        mode = (cfgdir / "google_maps_key").stat().st_mode & 0o777
        if mode & 0o077:
            msg += f"; the file is mode {mode:o}: chmod 600 it"
    return msg


def scrub(text, key=None):
    """`text` with the key (and any key=... query value) blanked, for messages and logs."""
    text = re.sub(r"(?i)(key=)[^&\s\"']+", r"\1***", str(text))
    text = re.sub(r"(?i)(x-goog-api-key['\"]?\s*[:=]\s*['\"]?)[^\s'\",}]+", r"\1***", text)
    if key:
        text = text.replace(key, "***")
    return text


# ----------------------------------------------------------------------------------------------------- ledger

class Usage:
    """Root requests on this machine: today, this month (local time), in total, and per run, in
    config_dir()/google_tiles_usage.json, the same file the CoupledPhotoreal plugin writes."""

    def __init__(self, path=None, clock=time.time):
        self.path = Path(path) if path else config_dir() / "google_tiles_usage.json"
        self.clock = clock

    def load(self):
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(d, dict):
                d = {}
        except (OSError, ValueError):
            d = {}
        now = time.localtime(self.clock())
        month, day = time.strftime("%Y-%m", now), time.strftime("%Y-%m-%d", now)
        if d.get("month") != month:
            d["month"], d["month_roots"] = month, 0
        if d.get("day") != day:
            d["day"], d["day_roots"] = day, 0
        d.setdefault("total_roots", 0)
        d.setdefault("runs", [])
        return d

    def allows(self, day_cap, month_cap):
        """(True, "") or (False, why)."""
        d = self.load()
        if d["day_roots"] >= day_cap:
            return False, f"today's cap of {day_cap} root requests is reached ({self.path})"
        if d["month_roots"] >= month_cap:
            return False, f"this month's cap of {month_cap} root requests is reached ({self.path})"
        return True, ""

    def count_root(self, map_name="", by="client", run=None):
        d = self.load()
        for k in ("day_roots", "month_roots", "total_roots"):
            d[k] = int(d.get(k, 0)) + 1
        run = run or time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.clock()))
        for r in d["runs"]:
            if r.get("start") == run and r.get("by") == by and r.get("map") == map_name:
                r["roots"] = int(r.get("roots", 0)) + 1
                break
        else:
            d["runs"].append({"start": run, "map": map_name, "by": by, "roots": 1})
        d["runs"] = d["runs"][-200:]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)
        return d

    def summary(self):
        d = self.load()
        return (f"root requests: {d['day_roots']} today, {d['month_roots']} in {d['month']} "
                f"(1,000 a month free), {d['total_roots']} in total ({self.path})")


# ------------------------------------------------------------------------------------------- HTTP (tools only)

def _max_age(headers):
    cc = ""
    for k, v in (headers or {}).items():
        if k.lower() == "cache-control":
            cc = v
    if re.search(r"no-store|no-cache", cc):
        return 0.0
    m = re.search(r"max-age=(\d+)", cc)
    return float(m.group(1)) if m else 0.0


def _session_of(url):
    return dict(parse_qsl(urlsplit(url).query)).get("session")


def _with_session(url, session):
    """`url` carrying the session parameter (child URLs from the root carry it; nested tilesets' links may not)."""
    if not session or _session_of(url):
        return url
    p = urlsplit(url)
    q = parse_qsl(p.query) + [("session", session)]
    return urlunsplit((p.scheme, p.netloc, p.path, urlencode(q), p.fragment))


def _without_session(url):
    p = urlsplit(url)
    q = [(k, v) for k, v in parse_qsl(p.query) if k not in ("session", "key")]
    return urlunsplit((p.scheme, p.netloc, p.path, urlencode(q), ""))


class TileSession:
    """A Map Tiles API 3D session for tools (measure, check): exactly one root request (billed) per session of up
    to three hours; tiles under it carry the session. The key goes in the X-Goog-Api-Key header. Responses are kept
    in memory for their Cache-Control max-age only (never on disk). `http` is a requests-like object (get(url,
    headers=, timeout=) -> response with status_code, headers, content), so tests mock it."""

    def __init__(self, key, http=None, usage=None, day_cap=50, month_cap=1000, clock=time.time, map_name="",
                 root_url=GOOGLE_ROOT, renew_margin_s=300.0, key_in="header"):
        if not key:
            raise PhotorealError("no Google Maps API key")
        if http is None:
            import requests
            http = requests.Session()
        self._key = key
        self.http = http
        self.usage = usage or Usage(clock=clock)
        self.caps = (day_cap, month_cap)
        self.clock = clock
        self.map_name = map_name
        self.root_url = root_url
        self.renew_margin_s = renew_margin_s
        self.key_in = key_in
        self.roots = 0
        self.requests = 0
        self.bytes = 0
        self._root = None
        self._root_t = None
        self._session = None
        self._cache = {}

    def __repr__(self):
        return f"TileSession(roots={self.roots}, requests={self.requests})"   # never the key

    def _headers(self):
        from .realworld import UA
        return {"X-Goog-Api-Key": self._key, **UA} if self.key_in == "header" else dict(UA)

    def _get(self, url):
        self.requests += 1
        if self.key_in == "url":
            p = urlsplit(url)
            url = urlunsplit((p.scheme, p.netloc, p.path, urlencode(parse_qsl(p.query) + [("key", self._key)]), ""))
        try:
            r = self.http.get(url, headers=self._headers(), timeout=60)
        except Exception as err:  # noqa: BLE001 - network errors carry the URL, never the key (header)
            raise PhotorealError(scrub(f"request failed: {err}", self._key)) from None
        body = r.content or b""
        self.bytes += len(body)
        if r.status_code != 200:
            msg = body[:300].decode("utf-8", "replace")
            try:
                msg = json.loads(body)["error"]["message"]
            except (ValueError, KeyError, TypeError):
                pass
            raise PhotorealError(scrub(f"HTTP {r.status_code} for {_without_session(url)}: {msg}", self._key))
        return body, dict(r.headers or {})

    def root(self):
        """The root tileset JSON: requested once per session (billed), reused until the session runs out."""
        now = self.clock()
        if self._root is not None and now - self._root_t < SESSION_S - self.renew_margin_s:
            return self._root
        ok, why = self.usage.allows(*self.caps)
        if not ok:
            raise PhotorealError(why)
        self.usage.count_root(self.map_name, by="tools")    # counted before it goes out: a failed one may be billed
        self.roots += 1
        body, _ = self._get(self.root_url)
        self._root = json.loads(body)
        self._root_t = now
        self._session = None
        for uri in _content_uris(self._root.get("root") or {}):
            self._session = _session_of(uri)
            if self._session:
                break
        self._cache = {}
        return self._root

    def get(self, uri, base=None):
        """A tile or nested tileset (bytes), `uri` relative to `base` (default the root's URL)."""
        self.root()
        url = _with_session(urljoin(base or self.root_url, uri), self._session or _session_of(base or ""))
        now = self.clock()
        hit = self._cache.get(url)
        if hit and hit[0] > now:
            return hit[1]
        body, headers = self._get(url)
        age = _max_age(headers)
        if age > 0:
            self._cache[url] = (now + age, body)
        return body


def _content_uris(node):
    out = []
    c = node.get("content") or {}
    u = c.get("uri") or c.get("url")
    if u:
        out.append(u)
    for ch in node.get("children") or []:
        out.extend(_content_uris(ch))
    return out


# ---------------------------------------------------------------------------------------------- georeference

def _proj_params(georef):
    return dict(re.findall(r"\+(\w+)=(\S+)", georef or ""))


def clean_proj(georef):
    """The OpenDRIVE geoReference without geoid grids or vertical units (horizontal maths only)."""
    return re.sub(r"\s*\+geoidgrids=\S+|\s*\+vunits=\S+", "", (georef or "").strip())


def _inside(box, lat, lon):
    s, w, n, e = box
    return s <= lat <= n and w <= lon <= e


def geoid_height(lat, lon, vdatum, network=True):
    """Height of the vertical datum's zero above the WGS84/GRS80 ellipsoid (m) at lat, lon, from PROJ's grids (fetched
    from cdn.proj.org into PROJ's user cache when `network`). None when the grid cannot be had."""
    if vdatum == "ellipsoid":
        return 0.0
    try:
        import pyproj
        from pyproj import Transformer
        if network:
            pyproj.network.set_network_enabled(True)
        if vdatum == "NAP":
            rd = Transformer.from_crs("EPSG:4258", "EPSG:28992", always_xy=True).transform(lon, lat)
            t = Transformer.from_crs("EPSG:7415", "EPSG:7931", always_xy=True)   # RD New + NAP -> ETRS89 3D
            h = t.transform(rd[0], rd[1], 0.0)[2]
        else:
            vcrs = {"EGM96": "EPSG:5773", "EGM2008": "EPSG:3855"}[vdatum]
            h = -Transformer.from_crs("EPSG:4979", f"EPSG:4326+{vcrs.split(':')[1]}", always_xy=True).transform(
                lon, lat, 0.0)[2]
        if h is None or not math.isfinite(h) or abs(h) > 150:
            return None
        return float(h)
    except Exception:  # noqa: BLE001 - no grid (offline): the caller says so
        return None


def frame_shift(lat, lon, frame, epoch):
    """[east, north, up] metres from `frame` ("ETRS89", "NAD83", "WGS84") to ITRF2020 at `epoch` (decimal year),
    with PROJ's time-dependent Helmert transformations (no grids). None when PROJ cannot do it."""
    if frame == "WGS84":
        return [0.0, 0.0, 0.0]
    src = {"ETRS89": "EPSG:4937", "NAD83": "EPSG:6319"}[frame]
    try:
        from pyproj import Geod, Transformer
        t = Transformer.from_crs(src, "EPSG:9989", always_xy=True)
        lon2, lat2, h2, _ = t.transform(lon, lat, 0.0, epoch)
        if not all(math.isfinite(v) for v in (lon2, lat2, h2)):
            return None
        az, _, d = Geod(ellps="GRS80").inv(lon, lat, lon2, lat2)
        return [d * math.sin(math.radians(az)), d * math.cos(math.radians(az)), h2]
    except Exception:  # noqa: BLE001
        return None


def decimal_year(t=None):
    tm = time.gmtime(t if t is not None else time.time())
    return tm.tm_year + (tm.tm_yday - 0.5) / (366.0 if tm.tm_year % 4 == 0 else 365.0)


def georef(georef_str, s=None, epoch=None, network=True):
    """Where the level's (0, 0, 0) is in Google's frame: {lat, lon, height (ellipsoidal), geoid_m, vdatum, frame,
    shift_m, epoch, proj}. Raises PhotorealError when the height datum cannot be resolved (offline without the grid:
    set vdatum: ellipsoid and height_offset_m by hand)."""
    s = dict(DEFAULTS, **(s or {}))
    p = _proj_params(georef_str)
    if p.get("proj") != "tmerc" or "lat_0" not in p or "lon_0" not in p:
        raise PhotorealError(f"geoReference {georef_str!r}: expected +proj=tmerc with +lat_0 and +lon_0")
    if float(p.get("x_0", 0)) or float(p.get("y_0", 0)):
        raise PhotorealError("geoReference with a false easting/northing: not supported")
    lat0, lon0 = float(p["lat_0"]), float(p["lon_0"])
    vd = s["vdatum"]
    if vd == "auto":
        if _inside(NL_BOX, lat0, lon0):
            vd = "NAP"
        else:
            g = (p.get("geoidgrids") or "").lower()
            vd = "EGM2008" if "egm08" in g or "egm2008" in g else ("EGM96" if "egm96" in g else "EGM96")
    n = geoid_height(lat0, lon0, vd, network=network)
    if n is None:
        raise PhotorealError(f"no {vd} geoid grid here (offline?): run prepare with the network once, or set "
                             f"map.photoreal.vdatum: ellipsoid and height_offset_m")
    epoch = epoch or decimal_year()
    if s["shift_m"] == "auto":
        frame = "ETRS89" if _inside(EU_BOX, lat0, lon0) else ("NAD83" if _inside(US_BOX, lat0, lon0) else "WGS84")
        shift = frame_shift(lat0, lon0, frame, epoch)
        if shift is None:
            log(f"no {frame} -> ITRF2020 transformation in PROJ; no horizontal shift")
            shift, frame = [0.0, 0.0, 0.0], "WGS84"
    else:
        frame, shift = "given", [float(x) for x in s["shift_m"]]
    lat, lon = offset_deg(lat0, lon0, shift[0], shift[1])
    return {"lat": lat, "lon": lon, "height": n + shift[2], "geoid_m": n, "vdatum": vd, "frame": frame,
            "shift_m": [round(x, 4) for x in shift], "epoch": round(epoch, 3), "lat0": lat0, "lon0": lon0,
            "proj": clean_proj(georef_str)}


def offset_deg(lat, lon, east, north):
    """lat, lon moved by east/north metres (WGS84 radii of curvature)."""
    a, e2 = 6378137.0, 6.69437999014e-3
    sl = math.sin(math.radians(lat))
    m = a * (1 - e2) / (1 - e2 * sl * sl) ** 1.5
    nn = a / math.sqrt(1 - e2 * sl * sl)
    return lat + math.degrees(north / m), lon + math.degrees(east / (nn * math.cos(math.radians(lat))))


class Projector:
    """Level x, y (OpenDRIVE: x east, y north, metres) -> longitude, latitude in Google's frame."""

    def __init__(self, g):
        from pyproj import Transformer
        self.g = g
        self.t = Transformer.from_crs(g["proj"], "EPSG:4326", always_xy=True)
        self.shift = g["shift_m"]

    def lonlat(self, pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        lons, lats = self.t.transform(xs, ys)
        out = []
        for lon, lat in zip(lons, lats):
            la, lo = offset_deg(lat, lon, self.shift[0], self.shift[1])
            out.append((round(lo, 8), round(la, 8)))
        return out


def _ecef(lat, lon, h):
    a, e2 = 6378137.0, 6.69437999014e-3
    la, lo = math.radians(lat), math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo), (n + h) * math.cos(la) * math.sin(lo),
            (n * (1 - e2) + h) * math.sin(la))


def placement_error(g, x, y, z=0.0):
    """How Cesium's flat east-north-up placement of the level differs from the true position of the level's point
    x, y (OpenDRIVE metres), z (metres in the level's datum): (horizontal m, vertical m). Cesium places the level as
    a plane tangent at the origin, the twin was built on a map projection with heights above the geoid, so the photo
    mesh appears lower in the level by about d^2 / 2R (5 cm at 800 m) plus the geoid's slope."""
    from pyproj import Transformer
    lon, lat = Transformer.from_crs(g["proj"], "EPSG:4326", always_xy=True).transform(x, y)
    lat, lon = offset_deg(lat, lon, g["shift_m"][0], g["shift_m"][1])
    true = _ecef(lat, lon, z + g["height"])         # geoid assumed flat over the level (checked apart)
    la0, lo0 = math.radians(g["lat"]), math.radians(g["lon"])
    o = _ecef(g["lat"], g["lon"], g["height"])
    e = (-math.sin(lo0), math.cos(lo0), 0.0)
    nvec = (-math.sin(la0) * math.cos(lo0), -math.sin(la0) * math.sin(lo0), math.cos(la0))
    u = (math.cos(la0) * math.cos(lo0), math.cos(la0) * math.sin(lo0), math.sin(la0))
    d = [true[i] - o[i] for i in range(3)]
    te, tn, tu = (sum(d[i] * v[i] for i in range(3)) for v in (e, nvec, u))
    return math.hypot(te - x, tn - y), tu - z


# ------------------------------------------------------------------------------------------------ road carve

def dehole(poly):
    """A shapely Polygon split into polygons without holes (Cesium's cartographic polygons have none): cut through
    the first hole with a vertical line, recursively."""
    from shapely.geometry import box
    if not poly.interiors:
        return [poly]
    minx, miny, maxx, maxy = poly.bounds
    cx = poly.interiors[0].centroid.x
    out = []
    for half in (box(minx - 1, miny - 1, cx, maxy + 1), box(cx, miny - 1, maxx + 1, maxy + 1)):
        part = poly.intersection(half)
        for g in getattr(part, "geoms", [part]):
            if g.geom_type == "Polygon" and g.area > 0.01:
                out.extend(dehole(g))
    return out


def carve_rings(xodr, carve="carriageway", step=1.0, simplify=0.1, grow=0.0):
    """Hole-free rings (open lists of (x, y), OpenDRIVE metres) covering the lanes cut out of the photo mesh, the
    geoReference string and stats."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from . import odroutline
    types = set(odroutline.CARRIAGEWAY) | (set(odroutline.SIDEWALK) if carve == "road" else set())
    polys, georef_str = odroutline.outlines(xodr, types, step)
    shapes = []
    for p in polys:
        if len(p) < 3:
            continue
        g = Polygon(p)
        if not g.is_valid:
            g = g.buffer(0)
        if not g.is_empty and g.area > 0.05:
            shapes.append(g)
    merged = unary_union(shapes)
    if grow:
        merged = merged.buffer(grow, join_style=2)
    merged = merged.simplify(simplify, preserve_topology=True)
    pieces = []
    for g in getattr(merged, "geoms", [merged]):
        if g.geom_type == "Polygon" and g.area > 0.05:
            pieces.extend(dehole(g))
    rings = [list(p.exterior.coords)[:-1] for p in pieces]
    stats = {"lane_polygons": len(shapes), "area_m2": round(merged.area, 1), "rings": len(rings),
             "points": sum(len(r) for r in rings), "carve": carve}
    return rings, georef_str, stats


def footprint_rings(buildings, grow=0.5, simplify=0.2):
    """Hole-free rings of building footprints (lists of (x, y) in the level's metres), grown a little so the photo
    mesh's walls are cut too."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    shapes = []
    for ring in buildings:
        if len(ring) >= 3:
            g = Polygon(ring)
            g = g if g.is_valid else g.buffer(0)
            if not g.is_empty:
                shapes.append(g.buffer(grow, join_style=2))
    if not shapes:
        return []
    merged = unary_union(shapes).simplify(simplify, preserve_topology=True)
    pieces = []
    for g in getattr(merged, "geoms", [merged]):
        if g.geom_type == "Polygon":
            pieces.extend(dehole(g))
    return [list(p.exterior.coords)[:-1] for p in pieces]


def load_footprints(path):
    """Building footprints in the level's metres (x east, y north) from a carla-digitaltwins bake manifest
    (ue/manifest.json: buildings[].rings_ue, UE cm with y south) or a map's buildings.json (records with "p": CARLA
    x, y south, metres). [] when the file is missing or has none."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return []
    out = []
    if isinstance(data, dict) and isinstance(data.get("buildings"), list):     # bake manifest
        for b in data["buildings"]:
            rings = b.get("rings_ue")
            if isinstance(rings, str):
                rings = json.loads(rings)
            if rings and len(rings[0]) >= 3:
                out.append([(float(x) / 100.0, -float(y) / 100.0) for x, y in rings[0]])
    elif isinstance(data, list):                                               # buildings.json
        for b in data:
            ring = b.get("p") if isinstance(b, dict) else None
            if ring and len(ring) >= 3:
                out.append([(float(x), -float(y)) for x, y in ring])
    return out


# ------------------------------------------------------------------------------------------------- prepare

def level_data(cfg, xodr=None, network=True, footprints=None):
    """footprints: a bake manifest or buildings.json path (load_footprints), a list of rings, or None: the map
    folder's buildings.json when it has one."""
    """The level's photoreal data (origin, carve rings and footprints in Google's lon/lat, the hide list), as the
    plugin reads it from <pack>/Content/Config/Photoreal/<Level>.json."""
    from . import config as _config
    mc = cfg["map"]
    s = settings(mc) or settings({"photoreal": "google"})
    src = Path(xodr or mc.get("opendrive") or "")
    if not src.is_absolute():
        src = Path(cfg["_root"]) / src
    if not src.exists():
        raise PhotorealError(f"no OpenDRIVE at {src} (map.opendrive)")
    from . import odroutline
    if s["carve"] != "none":
        rings, georef_str, stats = carve_rings(src, s["carve"], grow=s["carve_grow_m"])
    else:
        _, georef_str = odroutline.parse(src)
        rings, stats = [], {"carve": "none"}
    g = georef(georef_str, s, network=network)
    proj = Projector(g)
    if footprints is None:
        footprints = _config.map_dir(cfg) / "buildings.json"
    if isinstance(footprints, (str, Path)):
        footprints = load_footprints(footprints)
    fp = footprint_rings(footprints) if s["night_buildings"] else []
    return {
        "version": 1,
        "level": mc.get("carla_level"),
        "made": time.strftime("%Y-%m-%d %H:%M:%S"),
        "opendrive": src.name,
        "origin": {"lon": round(g["lon"], 9), "lat": round(g["lat"], 9), "height": round(g["height"], 3)},
        "datum": {k: g[k] for k in ("geoid_m", "vdatum", "frame", "shift_m", "epoch", "lat0", "lon0", "proj")},
        "hide": [h for h in s["hide"] if h != "none"],
        "carve": [[list(p) for p in proj.lonlat(r)] for r in rings],
        "footprints": [[list(p) for p in proj.lonlat(r)] for r in fp],
        "stats": dict(stats, footprints=len(fp)),
    }


def pack_file(level, pack=PACK):
    return Path(pack) / "Content" / "Config" / "Photoreal" / f"{level}.json"


def prepare(cfg, xodr=None, pack=PACK, network=True, footprints=None):
    """Write the level data into the content pack (and the map folder). Returns the path."""
    from . import config as _config
    data = level_data(cfg, xodr, network=network, footprints=footprints)
    level = data["level"]
    if not level:
        raise PhotorealError("map.carla_level is not set")
    out = pack_file(level, pack)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    mdir = _config.map_dir(cfg)
    if mdir.exists():
        (mdir / "photoreal.json").write_text(json.dumps({k: v for k, v in data.items() if k not in
                                                         ("carve", "footprints")}, indent=1), encoding="utf-8")
    st = data["stats"]
    log(f"{out}: origin {data['origin']['lat']:.7f}, {data['origin']['lon']:.7f}, {data['origin']['height']:.2f} m "
        f"ellipsoidal ({data['datum']['vdatum']} + {data['datum']['geoid_m']:.2f} m, {data['datum']['frame']} shift "
        f"{data['datum']['shift_m']}); {st.get('rings', 0)} carve rings ({st.get('points', 0)} points, "
        f"{st.get('area_m2', 0):.0f} m2), {st['footprints']} footprints; {out.stat().st_size / 1e6:.1f} MB")
    return out


# ---------------------------------------------------------------------------------------------- run time

class Handle:
    """The spawned blueprint.cc_photoreal_google actor of one level load."""

    def __init__(self, world, actor, s, night, log_fn=log):
        self.world, self.actor, self.s, self.night, self.log = world, actor, s, night, log_fn
        self.state = "spawned"

    def check(self):
        """After a few ticks: still there = streaming; gone = the server stopped it (no key there, a cap, no level
        data, a load error: its log says which). Logs once; returns True while it streams."""
        alive = False
        try:
            alive = self.world.get_actor(self.actor.id) is not None and self.actor.is_alive
        except RuntimeError:
            alive = False
        new = "on" if alive else "off"
        if new != self.state:
            if alive:
                self.log("Google Photorealistic 3D Tiles on: one root request (counted on the server machine in "
                         "~/.config/coupled-carla/google_tiles_usage.json)")
            else:
                self.log("the server did not start the photorealistic tiles (no key on the server machine, a cap "
                         "reached, no level data in its content pack, or a load error: see the server log); the "
                         "twin is shown as it is")
            self.state = new
        return alive

    def info(self):
        return {"photoreal": self.s["provider"], "state": self.state, "night": self.night,
                "options": role_options(self.s, self.night)}

    def destroy(self):
        try:
            if self.actor.is_alive:
                self.actor.destroy()
        except RuntimeError:
            pass


def attach(world, tc, night=False, log_fn=log):
    """Spawn the photoreal actor for this level load when map.photoreal is on. None when off or not possible
    (one log line); the run goes on with the normal twin either way."""
    mc = tc.get("map") or {}
    try:
        s = settings(mc)
    except PhotorealError as err:
        log_fn(f"{err}; off")
        return None
    if s is None:
        return None
    if not mc.get("carla_level"):
        log_fn("map.photoreal needs a baked CARLA 1.0 level (map.carla_level); off")
        return None
    lib = world.get_blueprint_library()
    try:
        bps = lib.filter(BLUEPRINT)
        bp = next((b for b in bps if b.id == BLUEPRINT), None)
    except RuntimeError:
        bp = None
    if bp is None:
        log_fn(f"this server has no {BLUEPRINT} (Cesium for Unreal and CoupledPhotoreal plugins, and the "
               f"CoupledCarla content pack: unreal/README.md); the twin is shown as it is")
        return None
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", role_options(s, night))
    import carla
    actor = world.try_spawn_actor(bp, carla.Transform())
    if actor is None:
        log_fn(f"could not spawn {BLUEPRINT}; the twin is shown as it is")
        return None
    return Handle(world, actor, s, night, log_fn)


# ---------------------------------------------------------------------------------------------- attribution

def attribution_text(s=None, cfgdir=None):
    """The line stamped on saved frames: Google's name plus the data credits the server last saw."""
    a = (s or DEFAULTS).get("attribution", "auto")
    if a and a != "auto":
        return str(a)
    try:
        credits = (Path(cfgdir or config_dir()) / "google_tiles_credits.txt").read_text(encoding="utf-8").strip()
    except OSError:
        credits = ""
    credits = re.sub(r"\s+", " ", credits)
    if credits and not credits.lower().startswith("google"):
        return f"Google · {credits}"
    return credits or "Google"


def stamp(rgb, text, logo=None):
    """`rgb` (H x W x 3 uint8) with the attribution at the bottom right: Google's logo when a copy of it is at
    config_dir()/google_maps_logo.png (not shipped: Google's brand asset), else the text alone."""
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    im = Image.fromarray(np.ascontiguousarray(rgb)).convert("RGB")
    d = ImageDraw.Draw(im)
    size = max(11, im.height // 48)
    try:
        font = ImageFont.load_default(size=size)
    except TypeError:   # Pillow < 10.1
        font = ImageFont.load_default()
    x0, y0, x1, y1 = d.textbbox((0, 0), text, font=font)
    w, h = x1 - x0, y1 - y0
    pad = max(3, size // 3)
    x, y = im.width - w - 2 * pad, im.height - h - 2 * pad
    logo = logo or (config_dir() / "google_maps_logo.png")
    try:
        lg = Image.open(logo).convert("RGBA")
        lg = lg.resize((max(1, int(lg.width * (h * 1.6) / lg.height)), max(1, int(h * 1.6))))
        im.paste(lg, (max(0, x - lg.width - pad), max(0, y - int(h * 0.3))), lg)
    except (OSError, ValueError):
        pass
    d.rectangle((x - pad, y - pad + y0, im.width, im.height), fill=(0, 0, 0))
    d.text((x, y), text, fill=(255, 255, 255), font=font)
    return np.asarray(im)


# ------------------------------------------------------------------------------------------------- measure

def decode_glb_positions(glb):
    """ECEF vertex positions (N x 3 float64) of a glTF binary tile (Google's: plain floats, node matrices, y-up)."""
    import struct
    import numpy as np
    if glb[:4] != b"glTF":
        raise PhotorealError("not a glb")
    off, js, bin_ = 12, None, b""
    while off + 8 <= len(glb):
        ln, ty = struct.unpack_from("<II", glb, off)
        chunk = glb[off + 8:off + 8 + ln]
        if ty == 0x4E4F534A:
            js = json.loads(chunk)
        elif ty == 0x004E4942:
            bin_ = chunk
        off += 8 + ln
    if js is None:
        raise PhotorealError("glb without json")
    yup = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)

    def node_matrix(n):
        if "matrix" in n:
            return np.array(n["matrix"], dtype=float).reshape(4, 4).T
        t = n.get("translation", [0, 0, 0])
        qx, qy, qz, qw = n.get("rotation", [0, 0, 0, 1])
        sx, sy, sz = n.get("scale", [1, 1, 1])
        r = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                      [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                      [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        m = np.eye(4)
        m[:3, :3] = r * np.array([sx, sy, sz])
        m[:3, 3] = t
        return m

    out = []
    scene = js.get("scenes", [{}])[js.get("scene", 0)]
    stack = [(i, yup) for i in scene.get("nodes", [])]
    while stack:
        i, parent = stack.pop()
        node = js["nodes"][i]
        m = parent @ node_matrix(node)
        if "mesh" in node:
            for prim in js["meshes"][node["mesh"]]["primitives"]:
                acc = js["accessors"][prim["attributes"]["POSITION"]]
                bv = js["bufferViews"][acc["bufferView"]]
                start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
                stride = bv.get("byteStride", 12)
                n = acc["count"]
                pos = np.ndarray((n, 3), dtype="<f4", buffer=bin_, offset=start, strides=(stride, 4)).astype(float)
                out.append(pos @ m[:3, :3].T + m[:3, 3])
        for c in node.get("children", []):
            stack.append((c, m))
    return np.vstack(out) if out else np.zeros((0, 3))


def ecef_to_enu(points, g):
    """N x 3 ECEF -> N x 3 east, north, up (m) of the level's frame (Cesium's placement at g's origin)."""
    import numpy as np
    la0, lo0 = math.radians(g["lat"]), math.radians(g["lon"])
    o = np.array(_ecef(g["lat"], g["lon"], g["height"]))
    r = np.array([[-math.sin(lo0), math.cos(lo0), 0.0],
                  [-math.sin(la0) * math.cos(lo0), -math.sin(la0) * math.sin(lo0), math.cos(la0)],
                  [math.cos(la0) * math.cos(lo0), math.cos(la0) * math.sin(lo0), math.sin(la0)]])
    return (np.asarray(points) - o) @ r.T


def _obb_hits(box, centre, radius):
    """Does a 3D Tiles bounding box (12 numbers, ECEF) come within `radius` of `centre` (ECEF)?"""
    import numpy as np
    c = np.array(box[:3])
    axes = [np.array(box[3 + 3 * i:6 + 3 * i]) for i in range(3)]
    d = np.array(centre) - c
    q = c.copy()
    for a in axes:
        ln = np.linalg.norm(a)
        if ln == 0:
            continue
        u = a / ln
        q += u * max(-ln, min(ln, float(d @ u)))
    return float(np.linalg.norm(np.array(centre) - q)) <= radius


def fetch_area(session, g, centre_xy=(0.0, 0.0), radius=150.0, max_error=2.0, max_tiles=400):
    """ENU vertices (m, in the level's frame) of the finest tiles (geometricError <= max_error) within `radius` of
    a level point. Walks nested tilesets under the one session; applies tile transforms (Google's have none)."""
    import numpy as np
    lon, lat = Projector(g).lonlat([centre_xy])[0]
    centre = np.array(_ecef(lat, lon, g["height"]))
    root = session.root()
    pts, tiles = [], 0
    stack = [(root["root"], session.root_url, np.eye(4))]
    while stack and tiles < max_tiles:
        node, base, parent = stack.pop()
        m = parent @ (np.array(node["transform"], dtype=float).reshape(4, 4).T if "transform" in node else np.eye(4))
        box = (node.get("boundingVolume") or {}).get("box")
        if box:
            c = m[:3, :3] @ np.array(box[:3]) + m[:3, 3]
            axes = [m[:3, :3] @ np.array(box[3 + 3 * i:6 + 3 * i]) for i in range(3)]
            if not _obb_hits([*c, *axes[0], *axes[1], *axes[2]], centre, radius):
                continue
        content = node.get("content") or {}
        uri = content.get("uri") or content.get("url")
        children = node.get("children") or []
        fine = node.get("geometricError", 1e9) <= max_error
        if uri:
            path = urlsplit(urljoin(base, uri)).path
            if path.endswith(".json"):
                sub = json.loads(session.get(uri, base))
                stack.append((sub["root"], urljoin(base, uri), m))
                continue
            if fine or not children:
                p = decode_glb_positions(session.get(uri, base))
                pts.append(ecef_to_enu(p @ m[:3, :3].T + m[:3, 3], g))
                tiles += 1
                continue
        stack.extend((c, base, m) for c in children)
    return (np.vstack(pts) if pts else np.zeros((0, 3))), tiles


def ground_offset(mesh_enu, road_pts, cell=1.0):
    """How much higher the photo mesh's street is than the twin's road (m): per road point the lowest mesh vertex
    within `cell` metres horizontally (cars and trees stand above the street), then the median over the points.
    road_pts: (x, y, z) in the level's frame. Returns (median, IQR, n)."""
    import numpy as np
    if len(mesh_enu) == 0 or len(road_pts) == 0:
        return None, None, 0
    keys = np.floor(mesh_enu[:, :2] / cell).astype(int)
    lowest = {}
    for (kx, ky), z in zip(map(tuple, keys), mesh_enu[:, 2]):
        if z < lowest.get((kx, ky), 1e9):
            lowest[(kx, ky)] = z
    diffs = []
    for x, y, z in road_pts:
        k = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        if k in lowest:
            diffs.append(lowest[k] - z)
    if not diffs:
        return None, None, 0
    q1, med, q3 = np.percentile(diffs, [25, 50, 75])
    return float(med), float(q3 - q1), len(diffs)


def footprint_shift(mesh_enu, footprints, ground_z, search=3.0, step=0.25, min_height=4.0):
    """The horizontal shift (east, north, m) that best puts the photo mesh's walls and roofs (vertices more than
    min_height above the ground) inside the twin's building footprints, by grid search within +-search m; with the
    share inside before and after. footprints: rings in the level's metres."""
    import numpy as np
    from shapely import contains_xy
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    if len(mesh_enu) == 0 or not footprints:
        return None
    high = mesh_enu[mesh_enu[:, 2] > ground_z + min_height]
    if len(high) < 50:
        return None
    if len(high) > 20000:
        high = high[np.random.default_rng(0).choice(len(high), 20000, replace=False)]
    area = unary_union([Polygon(r).buffer(0) for r in footprints if len(r) >= 3])
    best, base = (0.0, 0.0, -1.0), None
    n = int(round(search / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            dx, dy = i * step, j * step
            share = float(np.mean(contains_xy(area, high[:, 0] - dx, high[:, 1] - dy)))
            if i == 0 and j == 0:
                base = share
            if share > best[2]:
                best = (dx, dy, share)
    return {"east_m": best[0], "north_m": best[1], "inside_before": round(base, 3), "inside_after": round(best[2], 3),
            "vertices": int(len(high))}


def road_points(xodr, step=10.0):
    """(x, y, z) on the twin's driving lane centres every `step` m (z from the OpenDRIVE elevation)."""
    import xml.etree.ElementTree as ET
    from . import odroutline
    roads, _ = odroutline.parse(xodr)
    root = ET.parse(xodr).getroot()
    elev = {r.get("id"): odroutline._records(r.find("elevationProfile"), "elevation")
            for r in root.findall("road") if r.find("elevationProfile") is not None}
    out = []
    for rd in roads:
        if rd.junction != "-1":
            continue
        n = max(1, int(rd.length // step))
        for i in range(n + 1):
            s = min(rd.length, i * step)
            x, y, h = rd.ref(s)
            out.append((x, y, odroutline._eval_records(elev.get(rd.id, []), s)))
    return out


def measure(cfg, session, radius=150.0, points=None, log_fn=log, footprints=None):
    """Alignment of the photo mesh against the twin around each point (level metres; default the origin): height
    offset of the streets, horizontal offset of the buildings. Uses the session's one root request."""
    from . import config as _config
    mc = cfg["map"]
    s = settings(mc) or settings({"photoreal": "google"})
    src = Path(mc["opendrive"])
    if not src.is_absolute():
        src = Path(cfg["_root"]) / src
    from . import odroutline
    _, georef_str = odroutline.parse(src)
    g = georef(georef_str, s)
    roads = road_points(src)
    fps = load_footprints(footprints or _config.map_dir(cfg) / "buildings.json")
    results = []
    for cx, cy in points or [(0.0, 0.0)]:
        mesh, tiles = fetch_area(session, g, (cx, cy), radius)
        near = [p for p in roads if math.hypot(p[0] - cx, p[1] - cy) <= radius]
        dz, iqr, n = ground_offset(mesh, near)
        ground = (sum(p[2] for p in near) / len(near)) if near else 0.0
        fp = [r for r in fps if math.hypot(r[0][0] - cx, r[0][1] - cy) <= radius]
        sh = footprint_shift(mesh, fp, ground + (dz or 0.0))
        res = {"x": cx, "y": cy, "tiles": tiles, "vertices": int(len(mesh)), "street_dz_m": dz, "street_dz_iqr_m": iqr,
               "road_points": n, "buildings": len(fp), "shift": sh}
        log_fn(json.dumps(res))
        results.append(res)
    return {"origin": {k: g[k] for k in ("lat", "lon", "height", "geoid_m", "vdatum", "frame", "shift_m")},
            "points": results, "session": {"roots": session.roots, "requests": session.requests,
                                           "megabytes": round(session.bytes / 1e6, 1)}}


# -------------------------------------------------------------------------------------------------------- cli

def main(argv=None):
    from . import config as _config
    ap = argparse.ArgumentParser(prog="python -m coupledcarla.photoreal", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prepare", help="write the level data (origin, road carve, footprints) into the content pack")
    p.add_argument("experiment")
    p.add_argument("--xodr", help="the level's OpenDRIVE (default map.opendrive)")
    p.add_argument("--pack", default=str(PACK), help="content pack folder (default the repo's CoupledCarla pack)")
    p.add_argument("--offline", action="store_true", help="no PROJ grid download (needs vdatum: ellipsoid)")
    p.add_argument("--footprints", help="building footprints for the night cut: a carla-digitaltwins ue/manifest.json "
                                        "or a buildings.json (default the map folder's buildings.json)")
    sub.add_parser("status", help="is there a key on this machine; root requests used")
    m = sub.add_parser("measure", help="ONE billed root request: street height offset and building shift")
    m.add_argument("experiment")
    m.add_argument("--radius", type=float, default=150.0)
    m.add_argument("--at", action="append", default=[], help="x,y in the level's metres (repeatable; default 0,0)")
    m.add_argument("--yes", action="store_true", help="really send the root request")
    m.add_argument("--out", help="write the result JSON here")
    m.add_argument("--footprints", help="as for prepare (the building shift needs footprints)")
    a = ap.parse_args(argv)
    if a.cmd == "status":
        print(key_status())
        print(Usage().summary())
        return 0
    cfg = _config.load(a.experiment)
    if a.cmd == "prepare":
        prepare(cfg, a.xodr, Path(a.pack), network=not a.offline, footprints=a.footprints)
        return 0
    key, _ = read_key()
    if key is None:
        print(key_status())
        return 1
    if not a.yes:
        print("measure sends one billed root request (a session of up to three hours); add --yes")
        print(Usage().summary())
        return 1
    s = settings(cfg["map"]) or DEFAULTS
    sess = TileSession(key, day_cap=s["day_cap"], month_cap=s["month_cap"], map_name=cfg["map"].get("carla_level") or "",
                       key_in=s["key_in"])
    pts = [tuple(float(v) for v in t.split(",")) for t in a.at] or None
    res = measure(cfg, sess, a.radius, pts, footprints=a.footprints)
    text = json.dumps(res, indent=1)
    if a.out:
        Path(a.out).write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
