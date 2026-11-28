"""Cyclists where the driver will see them: a rolling window that keeps the place's share of bikes in the traffic
around each driving participant.

SUMO's bikes ride the whole map (cycling.py: 70 % along the cycle network), so the place's bike rate (CROWD's
bicycles per motor vehicle, crowdtraffic.py) holds over the map but not in a driver's few hundred metres: the
Eindhoven twin clip (2026-11-28, 4,600 cars and 1,040 bikes an hour, bikes 20 % of the vehicles on the map) had no
cyclist in the chase camera's 20 s. The window, like the crowd's (crowdwindow.py):

- covers, for a driver with a planned route (the SUMO driver: route_xy), the bike lanes within `band_m` of its
  route from `behind_m` behind it to `ahead_m` ahead; for any other driving participant, those within `radius_m`;
- every `check_s` counts the SUMO vehicles in it; when the bikes fall short of the place's bikes per motor vehicle
  times the motor vehicles there, it adds SUMO bikes on its far part (`spawn_min_m` .. `ahead_m` ahead, never
  nearer than `spawn_min_m` to a participant or camera), on cycle tracks first (lanes for bikes only, `track_weight`
  times as likely), each with a route to a bike edge 300-1500 m away;
- takes its own bikes off again once they are `drop_m` from every participant (out of sight), and keeps at most
  `max_bikes` of them.

So cyclists appear where they are seen at the place's rate, on every map: nothing here knows a place. Settings:
traffic.cycling.window (config.py). The pure parts (bike_lanes, target, candidates) need no SUMO or CARLA.
"""
import math
import random

DEFAULTS = {
    "enabled": "auto",       # "auto": on when a participant drives and the place's bike rate is known (CROWD)
    "ahead_m": 250.0,        # m of a driver's route ahead that the window covers
    "behind_m": 30.0,        # m around the car that stay in it
    "band_m": 40.0,          # m either side of the route (cycle tracks beside the road, side streets)
    "radius_m": 150.0,       # without a route: around the car
    "spawn_min_m": 70.0,     # new bikes at least this far from any participant or camera (the bridge hides 60 m)
    "check_s": 2.0,          # s of simulation between checks
    "max_bikes": 40,         # the window's own bikes at once
    "drop_m": 320.0,         # its bikes are taken off this far from every participant
    "track_weight": 3.0,     # how much likelier a lane for bikes only (a cycle track) is than a shared one
    "ratio": "auto",         # bikes per motor vehicle in the window, or "auto": the place's (crowdtraffic.py)
}
ANCHOR_KINDS = ("driver", "av_passenger")


def settings(tc):
    """traffic.cycling.window over DEFAULTS with "enabled" resolved for the trial (and the ratio it keeps)."""
    tr = tc.get("traffic") or {}
    cfg = {**DEFAULTS, **((tr.get("cycling") or {}).get("window") or {})}
    ratio = cfg["ratio"] if cfg["ratio"] != "auto" else tr.get("_bikes_per_motor")
    cfg["ratio"] = float(ratio) if ratio is not None else None
    en = cfg["enabled"]
    if en == "auto":
        kinds = [p.get("kind", "driver") for p in (tc.get("participants") or []) if isinstance(p, dict)]
        en = any(k in ANCHOR_KINDS for k in kinds) and bool(cfg["ratio"]) and bool(tr.get("bikes_per_hour"))
    cfg["enabled"] = bool(en) and en not in ("off", "false") and bool(cfg["ratio"])
    return cfg


def target(ratio, motors):
    """Bikes the window should hold for `motors` motor vehicles in it at `ratio` bikes per motor vehicle."""
    return int(math.floor(ratio * motors + 0.5)) if ratio and motors > 0 else 0


def bike_lanes(net, offset, min_len=15.0):
    """[(lane id, edge id, lane index, length, CARLA x, CARLA y, bikes only)] of the net's lanes bikes may ride
    (sumolib net; offset: BridgeHelper.offset, CARLA x = SUMO x - off x, CARLA y = -(SUMO y - off y))."""
    out = []
    for e in net.getEdges():
        if e.getFunction() == "internal":
            continue
        for lane in e.getLanes():
            if not lane.allows("bicycle") or lane.getLength() < min_len:
                continue
            shape = lane.getShape()
            sx, sy = shape[len(shape) // 2]
            only = not lane.allows("passenger")
            out.append((lane.getID(), e.getID(), lane.getIndex(), lane.getLength(), sx - offset[0],
                        -(sy - offset[1]), only))
    return out


def in_zone(x, y, anchor, cfg, spawn=False):
    """Whether CARLA (x, y) lies in an anchor's window ({"x", "y", "yaw", "ri", "s"}); spawn: in its far part."""
    ri = anchor.get("ri")
    d = math.hypot(x - anchor["x"], y - anchor["y"])
    if ri is not None and ri.length > 0:
        s0 = anchor.get("s", 0.0)
        if spawn:
            return d >= cfg["spawn_min_m"] and ri.inside(x, y, s0 + cfg["spawn_min_m"], s0 + cfg["ahead_m"],
                                                          cfg["band_m"])
        return d <= cfg["behind_m"] or ri.inside(x, y, s0, s0 + cfg["ahead_m"], cfg["band_m"])
    if d > cfg["radius_m"]:
        return False
    if not spawn:
        return True
    a = math.radians(anchor.get("yaw", 0.0))
    return d >= cfg["spawn_min_m"] and (x - anchor["x"]) * math.cos(a) + (y - anchor["y"]) * math.sin(a) >= 0.0


def candidates(lanes, anchor, cfg, viewers=()):
    """Indices of the bike lanes in the anchor's spawn zone, not within spawn_min_m of any viewer (x, y)."""
    out = []
    for i, ln in enumerate(lanes):
        x, y = ln[4], ln[5]
        if not in_zone(x, y, anchor, cfg, spawn=True):
            continue
        if any(math.hypot(x - vx, y - vy) < cfg["spawn_min_m"] for vx, vy in viewers):
            continue
        out.append(i)
    return out


class BikeWindow:
    """Keeps the place's share of SUMO bikes around the driving participants (see the module doc). anchors:
    [{"actor", "route": callable() -> CARLA polyline or None}]; viewers: callable() -> [(x, y)] of everyone who looks
    (participants, film cameras); poses: the bridge's {carla id: (x, y, ...)}; ids: callable() -> {carla id: SUMO id}.
    """

    def __init__(self, anchors, lanes, cfg, bike_types, viewers, poses, ids, log=print, seed=1, traci=None,
                 offset=(0.0, 0.0)):
        self.anchors, self.lanes, self.cfg = anchors, lanes, cfg
        self.offset = offset            # BridgeHelper.offset: CARLA x = SUMO x - off x, y = -(SUMO y - off y)
        self.bike_types = list(bike_types.items()) if isinstance(bike_types, dict) else [(b, 1) for b in bike_types]
        self.viewers, self.poses, self.ids = viewers, poses, ids
        self.log, self.rng = log, random.Random(seed + 211)
        self.traci = traci
        self.last, self.last_log, self.n = None, 0.0, 0
        self.mine = set()
        self.pending = {}       # own bikes added, not shown in CARLA yet: {SUMO id: t added}
        self._t = 0.0
        self.added = self.dropped = self.failed = 0
        self.seen = []          # (t, bikes, motors, target) per check
        self._routes = {}
        self.by_edge = {}
        for ln in lanes:
            self.by_edge.setdefault(ln[1], ln)

    def _tr(self):
        if self.traci is None:
            import traci
            self.traci = traci
        return self.traci

    def _anchor(self, n, a):
        from .crowdwindow import RouteIndex
        try:
            tf = a["actor"].get_transform()
        except RuntimeError:
            return None
        x, y = tf.location.x, tf.location.y
        ri = None
        if a.get("route") is not None:
            try:
                poly = a["route"]() or []
            except Exception:  # noqa: BLE001 - no route: the radius
                poly = []
            if len(poly) >= 2:
                key = (len(poly), tuple(poly[0]), tuple(poly[-1]))
                got = self._routes.get(n)
                if got is None or got[0] != key:
                    got = (key, RouteIndex(poly))
                    self._routes[n] = got
                ri = got[1]
        return {"x": x, "y": y, "yaw": tf.rotation.yaw, "ri": ri, "s": ri.locate(x, y) if ri is not None else 0.0}

    def step(self, t):
        if self.last is not None and t - self.last < self.cfg["check_s"]:
            return
        self.last = self._t = t
        anchors = [an for an in (self._anchor(n, a) for n, a in enumerate(self.anchors)) if an is not None]
        if not anchors:
            return
        ids = self.ids()
        bikes = motors = 0
        for cid, p in list(self.poses.items()):
            sid = ids.get(cid, "")
            if sid.startswith("ego_") or not any(in_zone(p[0], p[1], an, self.cfg) for an in anchors):
                continue
            if sid.startswith("bike"):
                bikes += 1
            else:
                motors += 1
        shown = set(ids.values())       # bikes added in the last 20 s that CARLA does not show yet count too
        self.pending = {v: t0 for v, t0 in self.pending.items() if v not in shown and t - t0 < 20.0}
        bikes += len(self.pending)
        want = target(self.cfg["ratio"], motors)
        self.seen.append((round(t, 1), bikes, motors, want))
        self._drop(anchors)
        if want > bikes and len(self.mine) < int(self.cfg["max_bikes"]):
            viewers = [(an["x"], an["y"]) for an in anchors] + list(self.viewers() or [])
            for _ in range(min(want - bikes, int(self.cfg["max_bikes"]) - len(self.mine), 4)):
                an = self.rng.choice(anchors)
                self._add(an, viewers)
        if t - self.last_log >= 30.0:
            self.last_log = t
            self.log(f"bike window: t {t:.0f} s, {bikes} bikes for {motors} motor vehicles in the drivers' windows "
                     f"(the place's {self.cfg['ratio']:.3f} per motor vehicle wants {want}); {self.added} added, "
                     f"{self.dropped} taken off" + (f", {self.failed} not placed" if self.failed else ""))

    def _drop(self, anchors):
        tr = self._tr()
        try:
            live = set(tr.vehicle.getIDList())
        except Exception:  # noqa: BLE001
            return
        self.pending = {v: t0 for v, t0 in self.pending.items() if v in live or self._t - t0 < 20.0}
        self.mine = {v for v in self.mine if v in live or v in self.pending}    # not inserted yet: still ours
        for vid in list(self.mine):
            if vid not in live:
                continue
            try:
                sx, sy = tr.vehicle.getPosition(vid)
            except Exception:  # noqa: BLE001
                continue
            off = self.offset
            x, y = sx - off[0], -(sy - off[1])
            if all(math.hypot(x - an["x"], y - an["y"]) > self.cfg["drop_m"] for an in anchors):
                try:
                    tr.vehicle.remove(vid)
                    self.mine.discard(vid)
                    self.pending.pop(vid, None)
                    self.dropped += 1
                except Exception:  # noqa: BLE001
                    pass

    def _add(self, anchor, viewers, tries=6):
        cand = candidates(self.lanes, anchor, self.cfg, viewers)
        if not cand:
            self.failed += 1
            return False
        w = [self.cfg["track_weight"] if self.lanes[i][6] else 1.0 for i in cand]
        tr = self._tr()
        for _ in range(tries):
            ln = self.lanes[self.rng.choices(cand, w)[0]]
            dest = self._destination(ln)
            bp = self.rng.choices([b for b, _ in self.bike_types], [float(v) for _, v in self.bike_types])[0]
            try:
                route = list(tr.simulation.findRoute(ln[1], dest, bp).edges) if dest else []
                if not route or route[0] != ln[1]:
                    route = [ln[1]]
                self.n += 1
                vid, rid = f"bikewin{self.n}", f"bikewin_r{self.n}"
                tr.route.add(rid, route)
                tr.vehicle.add(vid, rid, typeID=bp, departLane=str(ln[2]),
                               departPos=f"{self.rng.uniform(2.0, max(2.5, ln[3] - 2.0)):.1f}", departSpeed="max")
                tr.vehicle.setColor(vid, self._colour())
                self.mine.add(vid)
                self.pending[vid] = self._t
                self.added += 1
                return True
            except Exception:  # noqa: BLE001 - SUMO would not take it there: another lane
                continue
        self.failed += 1
        return False

    def _destination(self, ln):
        x, y = ln[4], ln[5]
        far = [l for l in self.lanes if 300.0 <= math.hypot(l[4] - x, l[5] - y) <= 1500.0]
        return self.rng.choice(far)[1] if far else None

    def _colour(self):
        from .fleet import BIKE_COLORS, COLORS
        names = [k for k in BIKE_COLORS if k in COLORS]
        r, g, b = COLORS[self.rng.choices(names, [BIKE_COLORS[k] for k in names])[0]]
        return (int(r), int(g), int(b), 255)

    def summary(self):
        s = self.seen
        n = len(s) or 1
        return {"added": self.added, "dropped": self.dropped, "failed": self.failed, "checks": len(s),
                "bikes_mean": round(sum(x[1] for x in s) / n, 2), "motors_mean": round(sum(x[2] for x in s) / n, 2),
                "target_mean": round(sum(x[3] for x in s) / n, 2), "ratio": self.cfg["ratio"]}


def make(tc, parts, sync, net, offset, viewers, log=print):
    """A BikeWindow for the trial's driving participants (run.py), or None (off, nobody drives, no bike rate)."""
    cfg = settings(tc)
    if not cfg["enabled"]:
        return None
    anchors = []
    for p in parts:
        if p.kind not in ANCHOR_KINDS or getattr(p, "actor", None) is None:
            continue
        route = getattr(getattr(p, "input", None), "route_xy", None)
        anchors.append({"actor": p.actor, "route": (lambda f=route: f(1000.0)) if callable(route) else None})
    if not anchors:
        return None
    lanes = bike_lanes(net, offset)
    if not lanes:
        return None

    def ids():
        return {cid: sid for sid, cid in sync.sumo2carla_ids.items()}
    w = BikeWindow(anchors, lanes, cfg, (tc.get("traffic") or {}).get("bike_types") or {}, viewers, sync.poses,
                   ids, log, int((tc.get("traffic") or {}).get("seed", 1)), offset=offset)
    log(f"bike window: around {len(anchors)} driving participant(s), {len(lanes)} bike lanes "
        f"({sum(1 for ln in lanes if ln[6])} for bikes only), {cfg['ratio']:.3f} bikes per motor vehicle")
    return w
