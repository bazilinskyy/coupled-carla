"""The city's own sounds where they are: an ambient bed that fits the place and the hour, from recordings in
coupledcarla/data/sounds/ambience (soundsamples.clip) placed at real places and heard through the soundscape's
propagation (distance, air, ground, buildings' occlusion, a facade reflection, the street's reverberation).

Places of a map (places(), cached in maps/<name>/sound_places.json, CARLA coordinates):
- cafes and restaurants, shops: Overture Maps places (real/overture_places.json, food_and_drink, shopping);
- fountains, water, parks and woods: the map's OSM scenery (real/osm_scenery.json);
- churches (bells) and construction sites: OSM (amenity=place_of_worship with religion=christian,
  landuse=construction, building=construction), one Overpass query cached in real/osm_sound.json;
- tunnels and passages under buildings: the map's OSM ways with tunnel=yes|building_passage (map.osm).

What plays (PlaceBed.update, each with a level at a reference distance, gated by the local hour and season):
- terrace voices and cutlery at the nearest cafe within 40 m (8-23 h), people talking around a pedestrian (by the
  number of walkers within 20 m), construction at the nearest site within 150 m (working days 7-17 h), a fountain
  within 40 m, church bells striking the hour (once per hour, 7-22 h) from the nearest church within 800 m;
- crickets on warm nights (May-September, mirrored south of the equator, at least 13 C) near green, cicadas on hot
  days (25 C, within 45 degrees of the equator) near trees, leaves in the wind (from 3 m/s) among trees;
- inside a tunnel or a passage under a building: a long, louder reverberant tail.
Missing recordings or places leave that part out (one log line).
"""
import json
import math
from pathlib import Path

import numpy as np

from . import acoustics as ac
from . import soundsamples, synth

OSM_SOUND_QUERY = """[out:json][timeout:120];
(
  node["amenity"="place_of_worship"]["religion"="christian"]{b};
  way["amenity"="place_of_worship"]["religion"="christian"]{b};
  way["building"~"^(church|cathedral|chapel)$"]{b};
  way["landuse"="construction"]{b};
  way["building"="construction"]{b};
);
out center tags;"""


def _lw(lp, d):
    """A-weighted sound power of a source giving `lp` dB(A) at `d` m (hemispherical, as soundscape._lw_from_lp)."""
    return lp + 20.0 * math.log10(d) + 8.0


# category -> (clip, dB(A) at the reference distance, reference distance m, audible radius m)
LEVELS = {"cafe": ("cafe", 56.0, 5.0, 40.0), "construction": ("construction", 72.0, 20.0, 150.0),
          "fountain": ("fountain", 62.0, 3.0, 40.0), "church": ("church_bell", 88.0, 25.0, 800.0)}


def _centre(el):
    if "center" in el:
        return el["center"]["lat"], el["center"]["lon"]
    if "lat" in el:
        return el["lat"], el["lon"]
    g = el.get("geometry") or []
    if g:
        return sum(p["lat"] for p in g) / len(g), sum(p["lon"] for p in g) / len(g)
    return None


def _radius(el):
    g = el.get("geometry") or []
    if len(g) < 3:
        return 10.0
    lat0 = sum(p["lat"] for p in g) / len(g)
    xs = [p["lon"] * 111320.0 * math.cos(math.radians(lat0)) for p in g]
    ys = [p["lat"] * 110540.0 for p in g]
    a = 0.5 * abs(sum(xs[i] * ys[i - 1] - xs[i - 1] * ys[i] for i in range(len(g))))
    return max(5.0, math.sqrt(a / math.pi))


def fetch_osm_sound(bbox, path, log=print):
    """Churches and construction sites in the box (Overpass, realworld's mirrors), cached; None offline."""
    if path.exists():
        return json.loads(path.read_text())
    try:
        from .realworld import overpass_json
        s, w, n, e = bbox
        data = overpass_json(OSM_SOUND_QUERY.format(b=f"({s},{w},{n},{e})"), timeout=120)
    except Exception as e:  # noqa: BLE001 - offline: no bells or construction
        log(f"sound: no OSM churches or construction sites ({type(e).__name__}: {e})")
        return None
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":")))
    return data


def places(map_dir, log=print, fetch=True, to_xy=None):
    """{category: [[x, y(, r)], ...], "tunnel": [[x0, y0, x1, y1], ...]} of a map folder, cached in
    sound_places.json; {} without a map. to_xy(lat, lon) -> (x, y) (default: geo.Geo of geo.net.xml)."""
    mdir = Path(map_dir)
    cache = mdir / "sound_places.json"
    if cache.exists():
        try:
            return json.loads(cache.read_text())
        except ValueError:
            pass
    meta = {}
    try:
        meta = json.loads((mdir / "meta.json").read_text())
    except (OSError, ValueError):
        pass
    if to_xy is None:
        try:
            from .geo import Geo
            to_xy = Geo(mdir / "geo.net.xml").to_carla
        except Exception as e:  # noqa: BLE001 - no net (or no sumolib): no places
            log(f"sound: no places for the ambient bed ({type(e).__name__}: {e})")
            return {}
    out = {k: [] for k in ("cafe", "shop", "church", "construction", "fountain", "water", "green", "tunnel")}
    real = mdir / "real"

    def add(cat, lat, lon, r=None):
        x, y = to_xy(lat, lon)
        out[cat].append([round(x, 1), round(y, 1)] + ([round(r, 1)] if r is not None else []))

    try:
        for rec in json.loads((real / "overture_places.json").read_text()):
            cat = {"food_and_drink": "cafe", "shopping": "shop"}.get(rec[3])
            if cat:
                add(cat, rec[0], rec[1])
    except (OSError, ValueError, IndexError, TypeError):
        pass
    try:
        for el in json.loads((real / "osm_scenery.json").read_text()).get("elements", []):
            t = el.get("tags") or {}
            c = _centre(el)
            if c is None:
                continue
            if t.get("amenity") == "fountain":
                add("fountain", *c)
            elif t.get("natural") == "water" or t.get("waterway") == "riverbank":
                add("water", *c, _radius(el))
            elif t.get("leisure") in ("park", "garden", "common") or t.get("landuse") in ("forest", "meadow",
                                                                                           "grass", "village_green") \
                    or t.get("natural") in ("wood", "scrub", "grassland", "heath"):
                if el.get("type") != "node":
                    add("green", *c, _radius(el))
    except (OSError, ValueError):
        pass
    osm = fetch_osm_sound(meta["bbox"], real / "osm_sound.json", log) if (fetch and meta.get("bbox")) else \
        (json.loads((real / "osm_sound.json").read_text()) if (real / "osm_sound.json").exists() else None)
    for el in (osm or {}).get("elements", []):
        t = el.get("tags") or {}
        c = _centre(el)
        if c is None:
            continue
        if t.get("landuse") == "construction" or t.get("building") == "construction":
            add("construction", *c)
        else:
            add("church", *c)
    import xml.etree.ElementTree as ET
    try:
        nodes, ways = {}, []
        for _ev, el in ET.iterparse(str(mdir / "map.osm")):
            if el.tag == "node":
                nodes[el.get("id")] = (float(el.get("lat")), float(el.get("lon")))
            elif el.tag == "way":
                tags = {k.get("k"): k.get("v") for k in el.findall("tag")}
                if tags.get("tunnel") in ("yes", "building_passage") and "highway" in tags:
                    ways.append([nd.get("ref") for nd in el.findall("nd")])
                el.clear()
        for refs in ways:
            pts = [to_xy(*nodes[r]) for r in refs if r in nodes]
            for a, b in zip(pts, pts[1:]):
                out["tunnel"].append([round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1)])
    except (OSError, ET.ParseError):
        pass
    try:
        cache.write_text(json.dumps(out, separators=(",", ":")))
    except OSError:
        pass
    log("sound: places for the ambient bed: " + ", ".join(f"{len(v)} {k}" for k, v in out.items() if v))
    return out


def _seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L = dx * dx + dy * dy
    u = 0.0 if L <= 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
    return math.hypot(px - (x0 + u * dx), py - (y0 + u * dy))


class PlaceBed:
    """The ambient bed's voices in a soundscape (see the module doc). update() once per tick after the vehicles."""

    def __init__(self, scape, pl, cfg=None, log=print):
        self.sc, self.log = scape, log
        self.cfg = cfg or {}
        self.pl = {k: np.asarray(v, float) for k, v in (pl or {}).items() if len(v)}
        self.clips = {}
        self.voices = {}
        self.bell_hour = None
        self.in_tunnel = False
        self.info = {}
        for name in ("cafe", "construction", "fountain", "walla", "crickets", "cicadas", "leaves", "church_bell"):
            c = soundsamples.clip(name, log)
            if c is not None:
                self.clips[name] = c

    def _near(self, cat, x, y, radius):
        a = self.pl.get(cat)
        if a is None or not len(a):
            return None
        d = np.hypot(a[:, 0] - x, a[:, 1] - y)
        i = int(np.argmin(d))
        return (a[i, 0], a[i, 1], float(d[i])) if d[i] <= radius else None

    def _loop(self, key, clip, seed):
        v = self.voices.get(key)
        if v is None:
            buf = self.clips[clip]
            v = synth.ShapedMono(buf, loop=True, start=(seed * 7919) % len(buf), kinds=("direct", "refl", "reverb"))
            self.voices[key] = self.sc.mixer.add(("place", key), v)
        return v

    def _bed(self, key, clip, dba, az, inside, il, seed):
        """A non-positional loop (voices around, insects, leaves) at `dba` at the ears (outside), None: silent."""
        if clip not in self.clips:
            return
        v = self.voices.get(key)
        if v is None:
            buf = self.clips[clip]
            v = synth.ShapedMono(buf, loop=True, start=(seed * 7919) % len(buf), kinds=("direct",))
            v.paths[0].az = az
            self.voices[key] = self.sc.mixer.add(("place", key), v)
        if dba is None:
            v.paths[0].w = np.zeros(v.K, np.float32)
            return
        with np.errstate(divide="ignore"):
            s = 10.0 * np.log10(np.maximum(v.share, 1e-12))
        bd = s - ac.a_weighted(s) + dba - (il if inside else 0.0)
        v.paths[0].w = v.weights(bd, self.sc.ref) * self.sc._on
        self.info[key] = dba

    def update(self, t, lx, ly, lz, walkers=(), inside=False, il=None, trees=0.4):
        sc, env = self.sc, self.sc.env
        il = np.zeros(ac.NB) if il is None else il
        h = env.hour(t)
        day = env.local.weekday() if env.local is not None else 2
        month = env.local.month if env.local is not None else 6
        if env.lat is not None and env.lat < 0:
            month = (month + 5) % 12 + 1
        self.info = {}
        # positional sources: the nearest of each kind within its radius, through the soundscape's propagation
        gates = {"cafe": 8 <= h < 23, "construction": day < 5 and 7 <= h < 17, "fountain": not (month in (12, 1, 2))}
        for cat, ok in gates.items():
            clip, lp, dref, radius = LEVELS[cat]
            near = self._near(cat, lx, ly, radius) if ok and clip in self.clips else None
            v = self.voices.get(cat)
            if near is None:
                if v is not None:
                    sc._apply(v, np.full(ac.NB, -200.0), sc._propagation(("place", cat), lx, ly, lz, 1.0, t), 0.0)
                continue
            v = self._loop(cat, clip, len(self.voices) + 1)
            lw = _lw(lp, dref) - (6.0 if cat == "cafe" and not 11 <= h < 22 else 0.0)
            with np.errstate(divide="ignore"):
                s = 10.0 * np.log10(np.maximum(v.share, 1e-12))
            p = sc._propagation(("place", cat), near[0], near[1], lz - 0.5, 1.5, t, refl=1)
            sc._apply(v, s - ac.a_weighted(s) + lw, p, sc._on)
            self.info[cat] = round(ac.a_weighted(s - ac.a_weighted(s) + lw - p.att[0]), 1)
        # church bells: the hour struck (1-12) once per hour from 7 to 22 h at the nearest church
        if "church_bell" in self.clips and self.pl.get("church") is not None and env.local is not None:
            near = self._near("church", lx, ly, LEVELS["church"][3])
            hr = int(h)
            v = self.voices.get("bells")
            if v is None:
                v = synth.EventVoice(self.clips["church_bell"], seed=9, jitter=0.02, gain_db_sd=0.5,
                                     kinds=("direct", "refl", "reverb"))
                v.left = 0
                self.voices["bells"] = sc.mixer.add(("place", "bells"), v)
            frac = h - hr
            if near is not None and 7 <= hr < 23 and frac < 0.01 and self.bell_hour != hr:
                self.bell_hour = hr
                v.left = (hr - 1) % 12 + 1
                v.next = 0.0
                sc.on_cue("place", "church_bells", f"{v.left} strikes at {near[2]:.0f} m")
            if v.left > 0:
                if v.period is None:
                    v.period = 2.4
                elif v.events >= v.left:
                    v.period, v.left, v.events = None, 0, 0
            if near is not None:
                p = sc._propagation(("place", "bells"), near[0], near[1], lz + 25.0, 30.0, t, refl=1)
                with np.errstate(divide="ignore"):
                    s = 10.0 * np.log10(np.maximum(v.share, 1e-12))
                sc._apply(v, s - ac.a_weighted(s) + _lw(*LEVELS["church"][1:3]), p, sc._on)
        # beds around the listener
        n = len(walkers)
        self._bed("walla", "walla", 44.0 + 10.0 * math.log10(n) if n >= 2 and not inside else None, -0.4, inside,
                  il, 3)
        green = self._near("green", lx, ly, 80.0) is not None or trees > 0.3
        night = env.night or not (6 <= h < 21)
        cr = night and month in (5, 6, 7, 8, 9) and env.temp >= 13.0 and green and env.rain < 0.3
        self._bed("crickets", "crickets", 40.0 + min(6.0, (env.temp - 13.0) / 2.0) if cr else None, 0.5, inside,
                  il, 5)
        ci = not night and 9 <= h < 19 and env.temp >= 25.0 and (env.lat is None or abs(env.lat) < 45) and \
            trees > 0.2 and env.rain < 0.3
        self._bed("cicadas", "cicadas", 48.0 if ci else None, -0.6, inside, il, 7)
        lv = None
        if env.wind >= 3.0 and trees > 0.15:
            lv = 30.0 + 25.0 * math.log10(env.wind / 3.0) + 10.0 * math.log10(min(1.0, trees) / 0.4 + 1e-3)
        self._bed("leaves", "leaves", lv, 0.0, inside, il, 11)
        # tunnels and passages: a long, louder tail
        tun = False
        for x0, y0, x1, y1 in self.pl.get("tunnel", ()):
            if abs(x0 - lx) < 200 and _seg_dist(lx, ly, x0, y0, x1, y1) < 5.0:
                tun = True
                break
        if tun != self.in_tunnel:
            self.in_tunnel = tun
            sc.on_cue("place", "tunnel", "in" if tun else "out")
        return self.info

