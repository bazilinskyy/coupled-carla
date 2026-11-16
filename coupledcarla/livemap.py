"""The session dashboard's live map and live controls (session.py).

In the trial's host process (run.py, with a session): LiveFeed sends the positions of everything on the map a few
times a second (participants, scripted cars and walkers, SUMO cars, bicycles and buses, the pedestrian crowd, drones
with their height, traffic-light states) and carries out the experimenter's live commands: an eHMI state for the AVs
and the weather and time of day (CARLA's weather presets, the sun of that place and hour from realworld.py).

In the session process: map_layers() turns a built map's roads and buildings (the trial viewer's layers,
export_trial.py) into lat/lon for the page, and aerial() picks an open aerial photo for the place where one exists.

CARLA (x, y) -> lat/lon uses an affine fit to the map's projection (geo.net.xml) over the map's box: over a few km
the projection is linear to well under a decimetre, and the page then needs no projection code.
"""
import math
import threading
import time

# carla.WeatherParameters presets (CARLA 0.9.16 and 1.0)
WEATHER_PRESETS = ("ClearNoon", "CloudyNoon", "WetNoon", "WetCloudyNoon", "SoftRainNoon", "MidRainyNoon",
                   "HardRainNoon", "ClearSunset", "CloudySunset", "WetSunset", "WetCloudySunset", "SoftRainSunset",
                   "MidRainSunset", "HardRainSunset", "ClearNight", "CloudyNight", "WetNight", "WetCloudyNight",
                   "SoftRainNight", "MidRainyNight", "HardRainNight", "DustStorm")

# What the map shows (the page's legend uses the same keys)
KINDS = {"participant": "participant", "av": "AV (eHMI)", "agent": "scripted car", "sumo": "SUMO car",
         "bike": "bicycle", "bus": "bus", "walker": "AI walker", "crowd": "crowd pedestrian", "drone": "AI drone"}

# Open aerial photos by country, keyless (checked 2026-11-15). Boxes are rough: the country name of the experiment's
# city block wins when it has one. Elsewhere the page uses OpenStreetMap's tiles.
AERIAL = [
    {"country": "Netherlands", "boxes": [(51.25, 3.36, 53.56, 7.23), (50.75, 5.55, 51.25, 6.25)],
     "name": "Aerial photo (PDOK)", "max": 19,
     "url": "https://service.pdok.nl/hwh/luchtfotorgb/wmts/v1_0/Actueel_orthoHR/EPSG:3857/{z}/{x}/{y}.jpeg",
     "attribution": 'Luchtfoto &copy; <a href="https://www.pdok.nl">PDOK</a> (CC BY 4.0)'},
    {"country": "France", "boxes": [(42.3, -4.8, 50.0, 2.5), (43.0, 2.5, 49.4, 7.7), (41.3, 8.5, 43.1, 9.6)],
     "name": "Aerial photo (IGN)", "max": 19,
     "url": "https://data.geopf.fr/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=ORTHOIMAGERY.ORTHOPHOTOS"
            "&STYLE=normal&TILEMATRIXSET=PM&FORMAT=image/jpeg&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}",
     "attribution": 'Orthophotos &copy; <a href="https://geoservices.ign.fr">IGN</a> (Licence Ouverte 2.0)'},
    {"country": "United States", "boxes": [(24.4, -125.0, 49.4, -66.9)], "name": "Aerial photo (USGS)", "max": 16,
     "url": "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{z}/{y}/{x}",
     "attribution": 'Imagery: <a href="https://www.usgs.gov">USGS</a> The National Map (public domain)'},
    {"country": "Japan", "boxes": [(24.0, 122.9, 45.6, 145.9)], "name": "Aerial photo (GSI)", "max": 18,
     "url": "https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg",
     "attribution": '<a href="https://maps.gsi.go.jp/development/ichiran.html">地理院タイル</a> (GSI Japan)'},
]


def aerial(bbox, country=None):
    """The open aerial photo layer for a map box [s, w, n, e] ({name, url, attribution, max}), or None."""
    if not bbox:
        return None
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    for a in AERIAL:
        if country and country == a["country"]:
            return {k: a[k] for k in ("name", "url", "attribution", "max")}
    if country:
        known = {a["country"] for a in AERIAL}
        if country not in known:
            return None
    for a in AERIAL:
        if any(s <= lat <= n and w <= lon <= e for s, w, n, e in a["boxes"]):
            return {k: a[k] for k in ("name", "url", "attribution", "max")}
    return None


def fit_affine(to_latlon, x0, y0, x1, y1, n=5):
    """(a, b, c, d, e, f) with lat = a x + b y + c and lon = d x + e y + f, least squares on an n x n grid of CARLA
    points over the box (x0, y0)-(x1, y1); to_latlon(x, y) is the exact conversion (geo.Geo.to_latlon)."""
    import numpy as np
    pts = [(x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * j / (n - 1)) for i in range(n) for j in range(n)]
    a = np.array([[x, y, 1.0] for x, y in pts])
    ll = np.array([to_latlon(x, y) for x, y in pts])
    lat = np.linalg.lstsq(a, ll[:, 0], rcond=None)[0]
    lon = np.linalg.lstsq(a, ll[:, 1], rcond=None)[0]
    return tuple(float(v) for v in (*lat, *lon))


def to_ll(aff, x, y):
    a, b, c, d, e, f = aff
    return round(a * x + b * y + c, 7), round(d * x + e * y + f, 7)


def geo_affine(geo, bbox):
    """The affine of a map: CARLA box from the map's [s, w, n, e] corners."""
    xs, ys = zip(*(geo.to_carla(lat, lon) for lat in (bbox[0], bbox[2]) for lon in (bbox[1], bbox[3])))
    return fit_affine(geo.to_latlon, min(xs), min(ys), max(xs), max(ys))


def feature(aff, kind, ident, label, x, y, yaw, speed_ms, extra=None):
    """One thing on the map, compact for the page: [kind, id, label, lat, lon, yaw (CARLA degrees: 0 east, 90
    south, so the page rotates clockwise from east), km/h, extra or null]."""
    lat, lon = to_ll(aff, x, y)
    return [kind, str(ident), label, lat, lon, round(float(yaw) % 360.0, 1), round(3.6 * float(speed_ms), 1),
            extra or None]


def light_state(char):
    """SUMO signal character -> g / y / r / o (off)."""
    c = (char or "o").lower()
    return "g" if c == "g" else "y" if c in ("y", "u") else "r" if c in ("r", "s") else "o"


def _speed(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class _CaptureWorld:
    """run.set_weather on a stand-in world: it reads the current weather and keeps what would be set, so the slow
    part (Open-Meteo for the time zone) runs in a thread and the run loop applies the result in one call."""

    def __init__(self, weather):
        self.weather, self.out = weather, None

    def get_weather(self):
        return self.weather

    def set_weather(self, w):
        self.out = w


class LiveFeed:
    """Created by run_trial when its control channel is a session's (coupling.RelayControl); tick() every tick."""

    def __init__(self, control, geo, tc, mdir, parts, agents, signals, step, log=print, tools=None):
        self.control, self.tc, self.mdir, self.parts, self.agents, self.signals, self.log = \
            control, tc, mdir, parts, agents, signals, log
        self.tools = tools
        bb = tc["map"]["bbox"]
        self.aff = geo_affine(geo, bb)
        self.latlon = ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
        self.every = max(1, round(0.2 / step))           # 5 Hz
        self.heads = []
        for tls, idxs, loc, _yaw in getattr(signals, "heads", []):
            self.heads.append((tls, idxs, *to_ll(self.aff, loc.x, loc.y)))
        self.lights, self.n_live = [], 0
        self.states = ["off"] + [s for s in ((tc.get("ehmi") or {}).get("states") or
                                             {"driving": 0, "yielding": 0, "go": 0}) if s != "off"]
        w = tc["simulation"].get("weather")
        self.weather = {"preset": w if isinstance(w, str) else "custom", "time": tc["simulation"].get("time")}
        self._pending, self._busy = None, False
        self.fps, self._fps_t, self._fps_n = None, time.time(), 0
        self.riders = {p.agent.id: p.role for p in parts if getattr(p, "agent", None) is not None}

    # -- commands -------------------------------------------------------------------------------------------------
    def ehmi_agents(self):
        return [a for a in self.agents if hasattr(a, "vehicle") and (a.ehmi_mode != "off" or a.id in self.riders)]

    def command(self, cmd, t, frame, logger, world):
        c = cmd.get("cmd")
        if c == "ehmi":
            state = str(cmd.get("state"))
            target = cmd.get("agent") or None
            if state not in self.states:
                self.log(f"dashboard: no eHMI state {state!r} in this experiment")
                return
            cars = [a for a in self.agents if hasattr(a, "vehicle") and a.id == target] if target else \
                self.ehmi_agents()
            for a in cars:
                if a.ehmi_mode in ("auto", "off"):
                    a.ehmi_mode = "manual"          # it would switch back by itself otherwise
                a.set_ehmi(state, "dashboard")
        elif c == "weather" and not self._busy:
            self._weather(cmd, world)

    def _weather(self, cmd, world):
        preset = cmd.get("preset") if cmd.get("preset") in WEATHER_PRESETS else None
        tod = cmd.get("time") or None
        if not preset and not tod:
            return
        sc = {"weather": preset or {}, "time": tod, "sun_altitude": None}
        cap = _CaptureWorld(world.get_weather())
        self._busy = True

        def work():
            try:
                from .run import set_weather
                info = set_weather(cap, sc, self.latlon if tod else None, self.mdir / "real", self.log)
                self._pending = (cap.out, info, preset, tod)
            except Exception as err:  # noqa: BLE001 - the weather stays as it was
                self.log(f"weather change failed: {err}")
                self._busy = False
        threading.Thread(target=work, daemon=True).start()

    def _apply_weather(self, t, frame, logger, world):
        w, info, preset, tod = self._pending
        self._pending, self._busy = None, False
        if w is None:
            return
        world.set_weather(w)
        try:
            from .bridge.synchronization import BridgeHelper
            lights = info.get("lights") or {"low_beam": bool(info.get("night"))}
            BridgeHelper.night = bool(lights.get("low_beam"))     # SUMO cars' headlights and rear lights
            BridgeHelper.fog = bool(lights.get("fog_rear"))
        except ImportError:
            pass
        self.weather = {"preset": preset or self.weather["preset"], "time": tod or self.weather["time"],
                        "sun_altitude": round(w.sun_altitude_angle, 1)}
        logger.event(t, frame, "weather", "", f"{preset or 'same weather'}"
                                              f"{' at ' + tod if tod else ''}, sun {w.sun_altitude_angle:.0f} deg "
                                              f"(dashboard)")

    # -- the feed -------------------------------------------------------------------------------------------------
    def tick(self, t, snap, n, logger, world, sync=None, types=None, crowd=None, drone_world=None):
        tools = self.tools
        if tools is not None and tools.other:
            cmds, tools.other[:] = list(tools.other), []
            for cmd in cmds:
                self.command(cmd, t, snap.frame, logger, world)
        if self._pending is not None:
            self._apply_weather(t, snap.frame, logger, world)
        self._fps_n += 1
        if n % self.every:
            return
        now = time.time()
        if now - self._fps_t >= 1.0:
            self.fps, self._fps_t, self._fps_n = round(self._fps_n / (now - self._fps_t), 1), now, 0
        if self.n_live % 5 == 0:                          # lights once a second
            self.lights = self.light_states()
        self.n_live += 1
        self.control.live({"t": round(t, 2), "frame": snap.frame, "fps": self.fps,
                           "features": self.collect(snap, sync, types or {}, crowd, drone_world),
                           "lights": self.lights, "weather": self.weather,
                           "ehmi": {"states": self.states,
                                    "agents": [{"id": a.id, "state": a.ehmi, "rider": self.riders.get(a.id)}
                                               for a in self.ehmi_agents()]}})

    def light_states(self):
        out, states = [], {}
        for tls, idxs, lat, lon in self.heads:
            try:
                st = self.signals._state(states, tls, idxs)
            except Exception:  # noqa: BLE001 - SUMO closing
                return out
            out.append([lat, lon, light_state(st)])
        return out

    def collect(self, snap, sync, types, crowd, drone_world):
        aff, rows, taken = self.aff, [], set()

        def at(actor_id):
            a = snap.find(actor_id)
            if a is None:
                return None
            tf = a.get_transform()
            return tf.location.x, tf.location.y, tf.rotation.yaw, _speed(a.get_velocity())

        tools = self.tools
        for p in self.parts:
            if p.kind == "drone" or p.actor is None:
                continue
            got = at(p.actor.id)
            if got is None:
                continue
            extra = {"k": p.kind}
            if getattr(p, "agent", None) is not None:
                extra["e"] = p.agent.ehmi
            if tools is not None and p.role in getattr(tools, "tor", {}):
                extra["a"] = tools.tor[p.role].state
            rows.append(feature(aff, "participant", p.actor.id, p.role, *got, extra))
            taken.add(p.actor.id)
        for a in self.agents:
            actor = getattr(a, "vehicle", None) or getattr(a, "walker", None)
            if actor is None or actor.id in taken:
                continue
            got = at(actor.id)
            if got is None:
                continue
            if hasattr(a, "vehicle"):
                kind = "av" if (a.ehmi_mode != "off" or a.id in self.riders) else "agent"
                rows.append(feature(aff, kind, actor.id, a.id, *got, {"e": a.ehmi} if kind == "av" else None))
            else:
                rows.append(feature(aff, "walker", actor.id, a.id, *got))
        if sync is not None:
            for sid, cid in list(sync.sumo2carla_ids.items()):
                got = at(cid)
                if got is not None:
                    kind = {"car": "sumo", "bike": "bike", "bus": "bus"}.get((types.get(cid) or ("car",))[0], "sumo")
                    rows.append(feature(aff, kind, cid, sid, *got))
        for w in getattr(crowd, "walkers", None) or []:
            got = at(w.id)
            if got is not None:
                rows.append(feature(aff, "crowd", w.id, "", *got))
        if drone_world is not None:
            rows += self.drones(drone_world)
        return rows

    def drones(self, dw):
        from .drones import to_carla
        f, out = dw.fleet, []
        pilots = {p.idx: p for p in dw.pilots}
        for i in range(f.n):
            c = to_carla(f.pos[i])
            p = pilots.get(i)
            ground = p.ground if p is not None else f.ground[i]
            v = f.vel[i]
            yaw = math.degrees(math.atan2(-float(v[1]), float(v[0]))) if math.hypot(v[0], v[1]) > 0.5 else 0.0
            speed = math.sqrt(float(v[0]) ** 2 + float(v[1]) ** 2 + float(v[2]) ** 2)
            extra = {"h": round(float(c[2] - ground), 1)}
            if p is not None:
                extra["k"] = "drone"
                out.append(feature(self.aff, "participant", dw.names[i], p.role, c[0], c[1], yaw, speed, extra))
            else:
                extra["m"] = dw.kinds[i]
                out.append(feature(self.aff, "drone", dw.names[i], dw.names[i], c[0], c[1], yaw, speed, extra))
        return out


def map_layers(map_dir, country=None):
    """A built map's roads (lanes, with their width) and buildings in lat/lon, its box and the aerial photo layer,
    for the session page. Needs sumolib (reads geo.net.xml)."""
    import json
    from pathlib import Path
    from .export_trial import map_layers as viewer_layers
    from .geo import Geo
    map_dir = Path(map_dir)
    data = viewer_layers(map_dir)
    bbox = data.get("bbox") or json.loads((map_dir / "meta.json").read_text())["bbox"]
    aff = geo_affine(Geo(map_dir / "geo.net.xml"), bbox)

    def line(pts):
        return [list(to_ll(aff, x, y)) for x, y in pts]
    roads = [[lane["k"], lane["w"], line(lane["p"])] for lane in data["lanes"]]
    buildings = [line(b["p"]) for b in data["buildings"] if len(b.get("p") or []) >= 3]
    return {"map": map_dir.name, "bbox": bbox, "lefthand": data.get("lefthand", False), "roads": roads,
            "buildings": buildings, "aerial": aerial(bbox, country)}
