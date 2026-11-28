"""The crowd where the driver will see it: a rolling window of walkers around each driving participant.

The crowd (crowdpeds.py) is planned once, at the start, with at most `max_count` walkers. A car that drives off
leaves that crowd behind; planned along the whole route ahead instead, the capped crowd is spread so thin that the
driver meets too few of them (Eindhoven twin drive, 2026-11-27: 80 of 80 walkers over ~1.5 km of route, 10 walkers a
minute met against CROWD's 15.8). The window keeps the crowd's density (walkers per kerb-km, from CROWD's pedestrians
per minute of the place) only where the driver is and is about to be:

- a driver with a planned route (the SUMO driver: `route_xy`): the kerbs within `band_m` of its route from where it
  is to `ahead_m` ahead, plus everything within `behind_m` of the car;
- any other driving participant (a person at the wheel, CARLA's autopilot, an AV passenger's car): the kerbs within
  `radius_m` (the crowd's radius) of the car.

The walkers are planned in the window at the start. Whenever one falls out of every window (far behind the car, off
along a side street) where no participant (and no film camera) sees it, and a window has fewer walkers than its kerb
length calls for, it is taken off the street and a new walker is planned at the window's far end (the last
`spawn_band_m` of it, never nearer than `spawn_min_m` to a participant, out of sight), with the old one's looks
(group, child, phone, wheelchair). Walkers near a participant on foot are never moved; groups move together; the
living city's own walkers are left alone. So the cap buys walkers only where they can be met, on every map and
twin: nothing here knows a place.

Settings: traffic.walkers.window (config.py). Pure parts (RouteIndex, initial_mask, zone) need no CARLA.
"""
import math
import random

DEFAULTS = {
    "enabled": "auto",       # "auto": on when a participant drives (driver, av_passenger) and the crowd is CROWD's;
                             # false: the crowd planned once around the start and along the route (before 2026-11-28)
    "ahead_m": 300.0,        # m of a driver's route ahead that the window covers
    "behind_m": 50.0,        # m around the car that stay in its window (the walkers it just passed)
    "band_m": 50.0,          # m either side of the route (the side streets a driver sees down)
    "spawn_band_m": 100.0,   # new walkers appear in the last spawn_band_m of the window (ahead_m - 100 .. ahead_m)
    "spawn_min_m": 60.0,     # never nearer than this to a participant (and only where nobody sees it)
    "check_s": 1.0,          # s of simulation between checks
    "route_max_m": 6000.0,   # m of the route the crowd's kerbs are prepared for (more: slower set-up)
}
ANCHOR_KINDS = ("driver", "av_passenger")
ROUTE_STEP_M = 5.0           # m between the points of a route's index
CELL_M = 25.0                # grid cell for kerb samples and route points
HIDE_M = 45.0                # without the living city: unseen beyond this from every participant (livingcity.HIDE_M)
SLACK_M = 15.0               # a walker leaves a window only this far beyond its edge (no churn at the edge)


def settings(tc):
    """traffic.walkers.window merged over DEFAULTS, with "enabled" resolved to True/False for the trial's
    participants (auto: on when one of them drives)."""
    wc = (tc.get("traffic") or {}).get("walkers") or {}
    cfg = {**DEFAULTS, **(wc.get("window") or {})}
    en = cfg.get("enabled", "auto")
    if en == "auto":
        kinds = [p.get("kind", "driver") for p in (tc.get("participants") or []) if isinstance(p, dict)]
        en = any(k in ANCHOR_KINDS for k in kinds) and wc.get("behaviour", "auto") not in ("off", False, None)
    cfg["enabled"] = bool(en) and en not in ("off", "false")
    for k in DEFAULTS:
        if k != "enabled":
            cfg[k] = float(cfg[k])
    return cfg


# --- geometry without CARLA ------------------------------------------------------------------------------------------

def resample(poly, step=ROUTE_STEP_M):
    """[(x, y, s)]: points every `step` m along the polyline (its first and last point included), s = m along it."""
    pts = [(float(x), float(y)) for x, y in (poly or [])]
    if not pts:
        return []
    out, s, carry = [(pts[0][0], pts[0][1], 0.0)], 0.0, 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        d = step - carry
        while d <= seg:
            f = d / seg
            out.append((ax + f * (bx - ax), ay + f * (by - ay), s + d))
            d += step
        carry = (carry + seg) % step if seg else carry
        s += seg
    if (out[-1][0], out[-1][1]) != pts[-1]:
        out.append((pts[-1][0], pts[-1][1], s))
    return out


class RouteIndex:
    """A route (CARLA x, y polyline) as points every ROUTE_STEP_M with their distance along it, in a grid."""

    def __init__(self, poly, step=ROUTE_STEP_M):
        self.pts = resample(poly, step)
        self.length = self.pts[-1][2] if self.pts else 0.0
        self.grid = {}
        for n, (x, y, _s) in enumerate(self.pts):
            self.grid.setdefault((int(x // CELL_M), int(y // CELL_M)), []).append(n)

    def near(self, x, y, max_d):
        """[(s, d)] of the route points within max_d of (x, y)."""
        r = int(max_d // CELL_M) + 1
        gx, gy = int(x // CELL_M), int(y // CELL_M)
        out = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for n in self.grid.get((gx + dx, gy + dy), ()):
                    px, py, s = self.pts[n]
                    d = math.hypot(px - x, py - y)
                    if d <= max_d:
                        out.append((s, d))
        return out

    def locate(self, x, y):
        """m along the route of the car at (x, y): of the points within 3 m of the nearest distance, the first (a
        route that comes back past its start does not move the car ahead)."""
        if not self.pts:
            return 0.0
        d2 = [(px - x) ** 2 + (py - y) ** 2 for px, py, _ in self.pts]
        lim = (math.sqrt(min(d2)) + 3.0) ** 2
        return next(self.pts[n][2] for n, v in enumerate(d2) if v <= lim)

    def project(self, x, y, max_d):
        """(s, d) of the route point nearest to (x, y) within max_d, else None."""
        got = self.near(x, y, max_d)
        return min(got, key=lambda sd: sd[1]) if got else None

    def inside(self, x, y, s_lo, s_hi, band):
        """Whether (x, y) lies within `band` of the route and its nearest route point between s_lo and s_hi m."""
        got = self.project(x, y, band)
        return got is not None and s_lo <= got[0] <= s_hi


class Samples:
    """The crowd's kerb samples (crowdpeds.Segment sides, geo.net x, y) in CARLA x, y, in a grid."""

    def __init__(self, segments):
        self.items = []                 # (seg, side, i, x, y)
        self.grid = {}
        self.step = {}
        self.add(segments)

    def add(self, segments, first=0):
        """Adds segments that are numbered from `first` in the crowd's list."""
        for k, s in enumerate(segments, start=first):
            for side in (0, 1):
                for i, (x, y) in enumerate(s.sides[side]):
                    self.grid.setdefault((int(x // CELL_M), int(-y // CELL_M)), []).append(len(self.items))
                    self.items.append((k, side, i, x, -y))
            self.step[k] = s.step

    def around(self, x, y, r):
        """Indices of the samples within r of (x, y)."""
        n = int(r // CELL_M) + 1
        gx, gy = int(x // CELL_M), int(y // CELL_M)
        out = []
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                for j in self.grid.get((gx + dx, gy + dy), ()):
                    _k, _sd, _i, sx, sy = self.items[j]
                    if (sx - x) ** 2 + (sy - y) ** 2 <= r * r:
                        out.append(j)
        return out

    def along(self, ri, s_lo, s_hi, band):
        """Indices of the samples within band of route `ri` between s_lo and s_hi m along it."""
        cand = set()
        for px, py, s in ri.pts:
            if s_lo <= s <= s_hi:
                cand.update(self.around(px, py, band))
        return {j for j in cand if ri.inside(self.items[j][3], self.items[j][4], s_lo, s_hi, band)}

    def km(self, idx):
        return sum(self.step[self.items[j][0]] for j in idx) / 1000.0


def zone(samples, anchor, cfg, radius, spawn=False):
    """Sample indices of an anchor's window (spawn: only where new walkers may appear: the window's far end).
    anchor: {"x", "y", "yaw" (deg), "ri" (RouteIndex or None), "s" (m along it)}."""
    x, y, ri = anchor["x"], anchor["y"], anchor.get("ri")
    if ri is not None and ri.length > 0:
        s0, hi = anchor.get("s", 0.0), anchor.get("s", 0.0) + cfg["ahead_m"]
        if not spawn:
            return samples.along(ri, s0, hi, cfg["band_m"]) | set(samples.around(x, y, cfg["behind_m"]))
        far = min(hi, ri.length)          # a route ending sooner: its last spawn_band_m
        lo = max(s0 + cfg["spawn_min_m"], far - cfg["spawn_band_m"])
        return {j for j in samples.along(ri, lo, far, cfg["band_m"])
                if math.hypot(samples.items[j][3] - x, samples.items[j][4] - y) >= cfg["spawn_min_m"]}
    near = samples.around(x, y, radius)
    if not spawn:
        return set(near)
    hx, hy = math.cos(math.radians(anchor.get("yaw", 0.0))), math.sin(math.radians(anchor.get("yaw", 0.0)))
    lo = max(cfg["spawn_min_m"], radius - cfg["spawn_band_m"])
    out = set()
    for j in near:
        dx, dy = samples.items[j][3] - x, samples.items[j][4] - y
        if math.hypot(dx, dy) >= lo and dx * hx + dy * hy >= 0.0:     # the far edge, ahead of the car
            out.add(j)
    return out


def in_window(x, y, anchor, cfg, radius):
    """Whether a walker at (x, y) is still in the anchor's window (SLACK_M beyond its edges counts as in)."""
    ri = anchor.get("ri")
    if math.hypot(x - anchor["x"], y - anchor["y"]) <= (cfg["behind_m"] if ri is not None and ri.length > 0
                                                         else radius) + SLACK_M:
        return True
    if ri is None or ri.length <= 0:
        return False
    s0 = anchor.get("s", 0.0)
    return ri.inside(x, y, s0 - SLACK_M, s0 + cfg["ahead_m"] + SLACK_M, cfg["band_m"] + SLACK_M)


def initial_mask(segments, anchors, keep_xy, cfg, radius):
    """[[side 0 bools, side 1 bools] per segment]: the kerb samples where the crowd is planned at the start: the
    anchors' windows plus everything within radius + 30 m of a participant that is not one (on foot, a drone)."""
    smp = Samples(segments)
    on = set()
    for a in anchors:
        on |= zone(smp, a, cfg, radius)
    for x, y in keep_xy:
        on.update(smp.around(x, y, radius + 30.0))
    mask = [[[False] * len(s.sides[0]), [False] * len(s.sides[1])] for s in segments]
    for j in on:
        k, side, i, _x, _y = smp.items[j]
        mask[k][side][i] = True
    return mask


# --- the CARLA side ---------------------------------------------------------------------------------------------------

class CrowdWindow:
    """Keeps a CrowdSim's walkers in the drivers' windows (see the module doc). anchors: participants that drive,
    each {"actor": CARLA actor, "route": callable() -> CARLA x, y polyline or None}; keep: actors on foot (or flying)
    whose surroundings keep their walkers. density: walkers per kerb-km the windows are filled to."""

    def __init__(self, sim, segments, att, params, anchors, keep, cfg, radius, density, duration_s, log=print, seed=1,
                 extend=None):
        self.sim, self.segments, self.att, self.params = sim, segments, att, params
        self.extend = extend or {}      # crowdpeds.prepare's: all the map's kerbs, places, count sites
        self.seg_ids = {s.id for s in segments}
        self.added = 0
        self.anchors, self.keep, self.cfg, self.radius = anchors, keep, cfg, float(radius)
        self.density, self.duration = float(density), float(duration_s)
        self.samples = Samples(segments)
        self.rng = random.Random(seed + 101)
        self.log = log
        self.last, self.last_log = None, 0.0
        self.recycled = self.failed = 0
        self._routes = {}       # anchor index -> (polyline key, RouteIndex)

    # -- poses and windows
    def _anchor(self, n, a):
        try:
            tf = a["actor"].get_transform()
        except RuntimeError:
            return None
        x, y = tf.location.x, tf.location.y
        ri = None
        if a.get("route") is not None:
            try:
                poly = a["route"]() or []
            except Exception:  # noqa: BLE001 - a route that cannot be read: the radius around the car
                poly = []
            if len(poly) >= 2:
                key = (len(poly), tuple(poly[0]), tuple(poly[-1]))
                got = self._routes.get(n)
                if got is None or got[0] != key:
                    got = (key, RouteIndex(poly))
                    self._routes[n] = got
                    self._extend(got[1])
                ri = got[1]
        an = {"x": x, "y": y, "yaw": tf.rotation.yaw, "ri": ri}
        an["s"] = ri.locate(x, y) if ri is not None else 0.0
        return an

    def _extend(self, ri):
        """Kerbs along a route that runs past the prepared ones (a SUMO driver's next random destination, a planned
        route that was short): the map's segments within band + 30 m of it that the crowd lacks, weighted as at the
        start (crowdpeds.attraction). The crowd's segment list grows in place (the walkers' legs index it)."""
        segs_all = self.extend.get("segments")
        if not segs_all or ri.length <= 0:
            return
        from .crowdpeds import attraction, crop
        centres = [(x, -y) for x, y, _s in ri.pts[::12]] + [(ri.pts[-1][0], -ri.pts[-1][1])]   # every 60 m, net
        new = [s for s in crop([s for s in segs_all if s.id not in self.seg_ids], centres, self.cfg["band_m"] + 30.0)]
        if not new:
            return
        first = len(self.segments)
        self.segments.extend(new)
        self.att.extend(attraction(new, self.extend.get("pois") or [], self.extend.get("counts") or []))
        self.samples.add(new, first)
        self.seg_ids.update(s.id for s in new)
        self.added += len(new)

    @staticmethod
    def _poses(actors):
        out = []
        for a in actors:
            try:
                tf = a.get_transform()
                out.append((tf.location.x, tf.location.y, tf.rotation.yaw))
            except RuntimeError:
                continue
        return out

    def _unseen(self, x, y, viewers):
        if self.sim.hidden is not None:      # the living city's: participants, free and film cameras
            try:
                return bool(self.sim.hidden(x, y, None))
            except Exception:  # noqa: BLE001 - when in doubt, not here
                return False
        return unseen(x, y, viewers)

    # -- the step
    def step(self, t):
        if self.last is not None and t - self.last < self.cfg["check_s"]:
            return
        self.last = t
        anchors = [an for an in (self._anchor(n, a) for n, a in enumerate(self.anchors)) if an is not None]
        if not anchors:
            return
        keep = self._poses(self.keep)
        keep_xy = [(x, y) for x, y, _ in keep]
        viewers = [(an["x"], an["y"], an["yaw"]) for an in anchors] + keep
        st = self.sim.state
        followers = {}
        for k, s in enumerate(st):
            if s["alive"] and s.get("follow") is not None:
                followers.setdefault(s["follow"][0], []).append(k)
        counts = [0] * len(anchors)
        out = []
        for k, s in enumerate(st):
            if not s["alive"] or s.get("follow") is not None or s.get("tag", "crowd") != "crowd":
                continue
            x, y = s["pos"]
            inside = [n for n, an in enumerate(anchors) if in_window(x, y, an, self.cfg, self.radius)]
            for n in inside:
                counts[n] += 1 + len(followers.get(k, ()))
            if inside or any(math.hypot(x - kx, y - ky) <= self.radius + 30.0 for kx, ky in keep_xy):
                continue
            if s.get("crossing") or s.get("leaving") is not None:
                continue
            out.append(k)
        if out:
            self._recycle(t, out, anchors, counts, followers, viewers)
        if t - self.last_log >= 30.0:
            self.last_log = t
            self.log(f"crowd window: t {t:.0f} s, {sum(counts)} walkers in the drivers' windows, "
                     f"{self.recycled} moved ahead so far" + (f" ({self.failed} could not be placed)"
                                                               if self.failed else "")
                     + (f", {self.added} kerb stretches added along the route" if self.added else ""))

    def _recycle(self, t, out, anchors, counts, followers, viewers):
        zones = [zone(self.samples, an, self.cfg, self.radius) for an in anchors]
        want = [self.density * self.samples.km(z) for z in zones]
        for k in out:
            group = [k] + followers.get(k, [])
            # the window that lacks the most walkers for its kerb length; none lacks any: keep it in reserve
            n = max(range(len(anchors)), key=lambda m: want[m] - counts[m])
            if want[n] - counts[n] < 0.5:
                break
            if want[n] - counts[n] < len(group) - 0.5:      # a group larger than the gap: a smaller one may fit
                continue
            if not all(self._unseen(*self.sim.state[g]["pos"], viewers) for g in group):
                continue
            if self._place(t, group, anchors[n], viewers):
                counts[n] += len(group)
                self.recycled += len(group)
            else:
                self.failed += 1

    def _place(self, t, group, anchor, viewers, tries=8):
        """Plans the group's leader anew at the window's far end, spawns it (and its group beside it) and takes the
        old ones off the street. False when no unseen spot took a walker."""
        from .crowdpeds import plan_from
        cand = list(zone(self.samples, anchor, self.cfg, self.radius, spawn=True))
        if not cand:
            return False
        weights = [self.att[k][side][i] + 1e-9 for k, side, i, _x, _y in (self.samples.items[j] for j in cand)]
        sim, st = self.sim, self.sim.state
        lead = st[group[0]]
        for _ in range(tries):
            j = self.rng.choices(cand, weights=weights)[0]
            k, side, i, x, y = self.samples.items[j]
            if min((math.hypot(x - vx, y - vy) for vx, vy, _ in viewers), default=1e9) < self.cfg["spawn_min_m"]:
                continue
            if not self._unseen(x, y, viewers):
                continue
            w = plan_from(self.segments, self.params, k, side, i, max(30.0, self.duration - t), self.rng)
            new = {key: v for key, v in (lead.get("plan") or {}).items()
                   if key not in ("seg", "side", "i", "legs", "xy", "yaw", "follow", "speed")}
            new.update(w)
            kl = sim.add(new)
            if kl is None:
                continue
            seg = self.segments[k]
            for m, g in enumerate(group[1:], start=1):       # the group beside its leader, a kerb sample apart
                old = st[g].get("plan") or {}
                f = st[g].get("follow") or (None, 0.6, 1.0)
                ii = max(0, min(seg.n - 1, i - m if i - m >= 0 else i + m))
                fw = {key: v for key, v in old.items() if key not in ("seg", "side", "i", "legs", "xy", "yaw")}
                fw.update({"seg": k, "side": side, "i": ii, "legs": list(w["legs"]), "speed": w["speed"],
                           "follow": (kl, f[1], f[2])})
                sim.add(fw)
            for g in reversed(group):
                sim.remove(g, "recycled", force=True)
            return True
        return False


def unseen(x, y, viewers, hide_m=HIDE_M):
    """Whether (x, y) is out of every viewer's sight (viewers: (x, y, yaw deg)): farther than hide_m, or behind it
    (over 110 degrees off its heading), as livingcity.hidden without the cameras and occluders."""
    for vx, vy, yaw in viewers:
        dx, dy = x - vx, y - vy
        d = math.hypot(dx, dy)
        if d > hide_m:
            continue
        a = math.radians(yaw)
        if d > 2.0 and (dx * math.cos(a) + dy * math.sin(a)) / d < -0.34:
            continue
        return False
    return True


def anchors_at_start(parts, cfg):
    """The driving participants' windows at the start ({"x", "y", "yaw", "ri", "s"}, for initial_mask) and the
    positions of the others [(x, y)]."""
    anchors, keep = [], []
    for p in parts:
        a = getattr(p, "actor", None)
        if a is None:
            continue
        try:
            tf = a.get_transform()
        except RuntimeError:
            continue
        x, y = tf.location.x, tf.location.y
        if p.kind not in ANCHOR_KINDS:
            keep.append((x, y))
            continue
        route = getattr(getattr(p, "input", None), "route_xy", None)
        poly = route(cfg["route_max_m"]) if callable(route) else []
        ri = RouteIndex(poly) if len(poly or []) >= 2 else None
        anchors.append({"x": x, "y": y, "yaw": tf.rotation.yaw, "ri": ri, "s": ri.locate(x, y) if ri else 0.0})
    return anchors, keep


def make(sim, segments, att, params, parts, cfg, radius, density, duration_s, log=print, seed=1, extend=None):
    """A CrowdWindow over the trial's participants (run.py's Participant: kind, actor, input), or None when no
    participant drives."""
    anchors, keep = [], []
    for p in parts:
        if getattr(p, "actor", None) is None:
            continue
        if p.kind in ANCHOR_KINDS:
            route = getattr(getattr(p, "input", None), "route_xy", None)
            mx = cfg["route_max_m"]
            anchors.append({"actor": p.actor, "route": (lambda f=route: f(mx)) if callable(route) else None})
        else:
            keep.append(p.actor)
    if not anchors:
        return None
    return CrowdWindow(sim, segments, att, params, anchors, keep, cfg, radius, density, duration_s, log, seed, extend)
