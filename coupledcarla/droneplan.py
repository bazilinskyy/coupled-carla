"""Where AI drones fly: the obstacles of a map (building footprints with their heights, trees) as a height field, and
the missions coupled-sim's DroneSpawner flies, planned on it:

- delivery: an industrial drone takes off from its depot, climbs to a cruise height `clearance` above the highest roof
  or tree under its route (sampled in a corridor `corridor_m` wide, plus `layer_m` per drone, so that crossing routes
  do not meet), flies to the address at its cruise speed (accelerating and braking smoothly), descends to drop the
  parcel (lands, or hovers at `winch_m` and lowers it on a tether, as Wing's drones do), and flies back; then the next
  address. Addresses are given as lat/lon or picked from the map's buildings (houses and flats first), each with an
  open drop point beside the building; the depot likewise (given, or open ground near the area's centre).
- patrol: a loop of waypoints (given, or a circle) at `clearance` above the roofs under it.
- swarm: Vasarhelyi et al. (2018) flocking at a height above the roofs of the area, migrating round a loop of points;
  buildings that reach up to the flock act through shill agents at their walls.
- hover (inspection): fly to a point at a height and hold there facing a target, or orbit it.

Everything runs in the model's frame (x east, y north, z up, metres); building records come in CARLA's (x east,
y south), as buildings.json has them, and are flipped on loading. No CARLA here: drones.py hands in the records and
a ground-height function.
"""
import math
import random

import numpy as np

from . import dronemodel as dm

HOME_KINDS = ("house", "residential", "apartments", "detached", "semidetached_house", "terrace", "bungalow")


class Obstacles:
    """Roof (and tree-top) heights on a grid of `cell` metres. records: [{"p": [[x, y], ...] CARLA, "top": absolute
    top z}] (buildings) and trees [(x, y CARLA, top z, radius)]."""

    def __init__(self, buildings=(), trees=(), cell=2.0, margin=60.0):
        self.cell = float(cell)
        polys = []
        for b in buildings:
            pts = np.asarray(b["p"], float)
            if len(pts) < 3:
                continue
            pts = np.column_stack([pts[:, 0], -pts[:, 1]])        # CARLA y south -> north
            polys.append((pts, float(b["top"]), b.get("k")))
        self.polys = polys
        trees = [(float(x), -float(y), float(top), float(r)) for x, y, top, r in trees]
        self.trees = trees
        xs = [p[0][:, 0] for p in polys] + [np.array([t[0] for t in trees])] if (polys or trees) else [np.zeros(1)]
        ys = [p[0][:, 1] for p in polys] + [np.array([t[1] for t in trees])] if (polys or trees) else [np.zeros(1)]
        xs, ys = np.concatenate(xs), np.concatenate(ys)
        self.x0, self.y0 = float(xs.min()) - margin, float(ys.min()) - margin
        nx = int(math.ceil((float(xs.max()) + margin - self.x0) / self.cell)) + 1
        ny = int(math.ceil((float(ys.max()) + margin - self.y0) / self.cell)) + 1
        self.top = np.full((ny, nx), -np.inf)
        self.owner = np.full((ny, nx), -1, int)      # building index per cell (drop points, shills)
        for i, (pts, top, _k) in enumerate(polys):
            self._fill(pts, top, i)
        for x, y, top, r in trees:
            self._disc(x, y, r, top)
        self._dilated = {}
        # buckets of building indices (100 m) for wall queries
        self.buckets = {}
        for i, (pts, _t, _k) in enumerate(polys):
            for bx in range(int(pts[:, 0].min() // 100), int(pts[:, 0].max() // 100) + 1):
                for by in range(int(pts[:, 1].min() // 100), int(pts[:, 1].max() // 100) + 1):
                    self.buckets.setdefault((bx, by), []).append(i)

    # -- rasterising ----------------------------------------------------------------------------------------------
    def _ij(self, x, y):
        return (np.floor((np.asarray(y) - self.y0) / self.cell).astype(int),
                np.floor((np.asarray(x) - self.x0) / self.cell).astype(int))

    def _fill(self, pts, top, idx):
        """Cells whose centres are inside the polygon (even-odd rule)."""
        i0, j0 = self._ij(pts[:, 0].min(), pts[:, 1].min())
        i1, j1 = self._ij(pts[:, 0].max(), pts[:, 1].max())
        i0, j0 = max(0, int(i0)), max(0, int(j0))
        i1, j1 = min(self.top.shape[0] - 1, int(i1)), min(self.top.shape[1] - 1, int(j1))
        if i1 < i0 or j1 < j0:
            return
        cy = self.y0 + (np.arange(i0, i1 + 1) + 0.5) * self.cell
        cx = self.x0 + (np.arange(j0, j1 + 1) + 0.5) * self.cell
        X, Y = np.meshgrid(cx, cy)
        inside = np.zeros(X.shape, bool)
        xa, ya = pts[:, 0], pts[:, 1]
        xb, yb = np.roll(xa, -1), np.roll(ya, -1)
        for k in range(len(xa)):
            if ya[k] == yb[k]:
                continue
            crosses = ((ya[k] > Y) != (yb[k] > Y)) & \
                (X < (xb[k] - xa[k]) * (Y - ya[k]) / (yb[k] - ya[k]) + xa[k])
            inside ^= crosses
        if not inside.any():      # smaller than a cell: its centroid's cell
            ci, cj = self._ij(xa.mean(), ya.mean())
            if 0 <= ci < self.top.shape[0] and 0 <= cj < self.top.shape[1]:
                self.top[ci, cj] = max(self.top[ci, cj], top)
                self.owner[ci, cj] = idx
            return
        sub = self.top[i0:i1 + 1, j0:j1 + 1]
        osub = self.owner[i0:i1 + 1, j0:j1 + 1]
        higher = inside & (top > sub)
        sub[higher] = top
        osub[inside] = idx

    def _disc(self, x, y, r, top):
        n = int(math.ceil(r / self.cell))
        ci, cj = self._ij(x, y)
        for di in range(-n, n + 1):
            for dj in range(-n, n + 1):
                if (di * self.cell) ** 2 + (dj * self.cell) ** 2 <= r * r + 1e-9:
                    i, j = ci + di, cj + dj
                    if 0 <= i < self.top.shape[0] and 0 <= j < self.top.shape[1]:
                        self.top[i, j] = max(self.top[i, j], top)

    # -- queries ----------------------------------------------------------------------------------------------------
    def dilated(self, radius):
        """The top grid with each cell the highest within `radius` (a square of that half-width, one cell more: the
        grid's quantisation errs on the safe side)."""
        k = max(0, int(math.ceil(radius / self.cell)) + 1)
        if k not in self._dilated:
            g = self.top.copy()
            for axis in (0, 1):
                out = g.copy()
                for s in range(1, k + 1):
                    out = np.maximum(out, np.roll(g, s, axis))
                    out = np.maximum(out, np.roll(g, -s, axis))
                g = out
            self._dilated[k] = g
        return self._dilated[k]

    def top_at(self, x, y, radius=0.0):
        """Highest obstacle top within radius of (x, y) (model frame); -inf where there is none."""
        g = self.dilated(radius) if radius > 0 else self.top
        scalar = np.ndim(x) == 0
        i, j = self._ij(np.atleast_1d(x), np.atleast_1d(y))
        inside = (i >= 0) & (i < g.shape[0]) & (j >= 0) & (j < g.shape[1])
        out = np.full(i.shape, -np.inf)
        out[inside] = g[i[inside], j[inside]]
        return float(out[0]) if scalar else out

    def corridor_top(self, a, b, radius=10.0):
        """Highest top within `radius` of the straight line a -> b (x, y)."""
        a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        n = max(2, int(np.linalg.norm(b - a) / self.cell) + 2)
        t = np.linspace(0.0, 1.0, n)
        pts = a + (b - a) * t[:, None]
        return float(np.max(self.top_at(pts[:, 0], pts[:, 1], radius)))

    def area_top(self, c, r):
        """Highest top within r of c."""
        xs = np.arange(c[0] - r, c[0] + r + self.cell, self.cell)
        X, Y = np.meshgrid(xs, np.arange(c[1] - r, c[1] + r + self.cell, self.cell))
        m = (X - c[0]) ** 2 + (Y - c[1]) ** 2 <= r * r
        return float(np.max(self.top_at(X[m], Y[m]))) if m.any() else -np.inf

    def is_open(self, x, y, radius=4.0):
        """No building or tree within radius."""
        return self.top_at(x, y, radius) == -np.inf

    def walls(self, p, radius, above_z):
        """Shill agents for a drone at p: the nearest point of each building wall within radius whose top is above
        `above_z`, with a velocity pointing from the wall towards the drone (out of the building)."""
        out = []
        bx, by = int(p[0] // 100), int(p[1] // 100)
        seen = set()
        for kx in (bx - 1, bx, bx + 1):
            for ky in (by - 1, by, by + 1):
                for i in self.buckets.get((kx, ky), ()):
                    if i in seen:
                        continue
                    seen.add(i)
                    pts, top, _k = self.polys[i]
                    if top < above_z:
                        continue
                    q = nearest_on_ring(pts, p[:2])
                    d = math.hypot(p[0] - q[0], p[1] - q[1])
                    if d <= radius and d > 1e-6:
                        out.append(((q[0], q[1], p[2]), ((p[0] - q[0]) / d, (p[1] - q[1]) / d, 0.0)))
        return out


def nearest_on_ring(pts, p):
    a = pts
    b = np.roll(pts, -1, axis=0)
    ab = b - a
    t = np.clip(np.sum((p - a) * ab, axis=1) / np.maximum(1e-12, np.sum(ab * ab, axis=1)), 0.0, 1.0)
    q = a + ab * t[:, None]
    d = np.sum((q - p) ** 2, axis=1)
    return q[int(np.argmin(d))]


def open_point_near(obs, x, y, rng, radius=40.0, clear=4.0, tries=60):
    """An open point (no building or tree within `clear`) near (x, y), nearest first among random tries."""
    if obs.is_open(x, y, clear):
        return (x, y)
    best = None
    for _ in range(tries):
        a, r = rng.uniform(0, 2 * math.pi), radius * math.sqrt(rng.random())
        px, py = x + r * math.cos(a), y + r * math.sin(a)
        if obs.is_open(px, py, clear):
            d = math.hypot(px - x, py - y)
            if best is None or d < best[0]:
                best = (d, px, py)
    return (best[1], best[2]) if best else None


def drop_point_for(obs, bi, rng, clear=3.0):
    """An open point beside building bi (in its garden or on the street in front), for a parcel."""
    pts, _top, _k = obs.polys[bi]
    c = pts.mean(axis=0)
    for _ in range(40):
        k = rng.randrange(len(pts) - 1 if len(pts) > 1 else 1)
        a, b = pts[k], pts[(k + 1) % len(pts)]
        m = a + (b - a) * rng.uniform(0.2, 0.8)
        out = m - c
        n = np.linalg.norm(out)
        if n < 1e-6:
            continue
        for dist in (5.0, 8.0, 12.0):
            q = m + out / n * dist
            if obs.is_open(q[0], q[1], clear):
                return (float(q[0]), float(q[1]))
    return None


def pick_addresses(obs, n, rng, centre, radius):
    """n buildings within radius of centre, homes first (OSM/3DBAG classes), each with a drop point:
    [(building index, (x, y))]."""
    cands = []
    for i, (pts, _top, k) in enumerate(obs.polys):
        c = pts.mean(axis=0)
        if math.hypot(c[0] - centre[0], c[1] - centre[1]) <= radius:
            cands.append((0 if (k or "") in HOME_KINDS else 1, rng.random(), i))
    cands.sort()
    out = []
    for _, _, i in cands:
        dp = drop_point_for(obs, i, rng)
        if dp:
            out.append((i, dp))
        if len(out) >= n:
            break
    return out


# -- missions ----------------------------------------------------------------------------------------------------

class Mission:
    """Base: one or more fleet indices, a kind and a phase; step(t, dt) sets their commands."""
    kind = "mission"

    def __init__(self, name, fleet, idx):
        self.name, self.fleet, self.idx = name, fleet, list(idx)
        self.phase = "parked"
        self.events = []         # (event name, detail) for events.csv

    def set_phase(self, phase, detail=""):
        if phase != self.phase:
            self.phase = phase
            self.events.append(("drone_phase", f"{self.name} {phase} {detail}".strip()))

    @staticmethod
    def heading(v, yaw):
        return math.atan2(v[1], v[0]) if math.hypot(v[0], v[1]) > 1.0 else yaw


class Delivery(Mission):
    """Depot -> address -> depot for each address in turn (then again from the first, with loop)."""
    kind = "delivery"

    def __init__(self, name, fleet, i, depot, drops, cruise_z, ground, start_s=0.0, dwell_s=10.0, cruise=12.0,
                 drop="land", winch_m=7.0, loop=True):
        super().__init__(name, fleet, [i])
        self.depot, self.drops = depot, list(drops)       # (x, y, ground z)
        self.cruise_z = list(cruise_z)                    # per leg (address k): absolute cruise height
        self.ground = ground
        self.wait_until, self.dwell, self.cruise = float(start_s), float(dwell_s), float(cruise)
        self.drop, self.winch_m, self.loop = drop, float(winch_m), loop
        self.k, self.outbound, self.route = 0, True, None
        self.delivered = 0

    def step(self, t, dt):
        f, i = self.fleet, self.idx[0]
        if self.k >= len(self.drops):
            f.set_command(i, "off")
            self.set_phase("done")
            return
        p, v, yaw = f.pos[i], f.vel[i], float(f.yaw()[i])
        if self.route is None:
            if t < self.wait_until:
                f.set_command(i, "off" if f.on_ground[i] else "position", pos=p.copy(), yaw=yaw)
                if f.on_ground[i]:
                    self.set_phase("on the ground" if self.delivered or not self.outbound else "parked")
                return
            a = self.depot if self.outbound else self.drops[self.k]
            b = self.drops[self.k] if self.outbound else self.depot
            z = self.cruise_z[self.k]
            end_z = b[2] + (self.winch_m if (self.drop == "winch" and self.outbound) else 0.3)
            pts = [(a[0], a[1], z), (b[0], b[1], z), (b[0], b[1], end_z)]
            self.route = dm.Waypoints(pts, cruise=self.cruise, climb=4.0)
            self.set_phase("delivering" if self.outbound else "returning", f"cruise {z:.1f} m")
        tgt, vel = self.route.step(p, dt)
        if self.route.done:
            b = self.drops[self.k] if self.outbound else self.depot
            hovering = self.drop == "winch" and self.outbound
            if hovering or f.on_ground[i]:
                if self.outbound:
                    self.delivered += 1
                    self.events.append(("drone_delivery", f"{self.name} parcel {self.delivered} at "
                                                          f"{b[0]:.1f},{b[1]:.1f} ({'winch' if hovering else 'landed'})"))
                    self.set_phase("dropping")
                self.route = None
                self.wait_until = t + self.dwell
                if not self.outbound:
                    self.k += 1
                    if self.k >= len(self.drops) and self.loop:
                        self.k = 0
                self.outbound = not self.outbound
                f.set_command(i, "position" if hovering else "off", pos=p.copy(), yaw=yaw)
                return
            tgt = (tgt[0], tgt[1], b[2] - 0.5)      # the last point is just above the ground: let it settle
        f.set_command(i, "position", pos=tgt, vel=vel, yaw=self.heading(vel, yaw))


class Patrol(Mission):
    kind = "patrol"

    def __init__(self, name, fleet, i, loop_pts, speed=8.0, start_s=0.0):
        super().__init__(name, fleet, [i])
        p0 = fleet.pos[i].copy()
        pts = [(p0[0], p0[1], loop_pts[0][2])] + [tuple(p) for p in loop_pts] + [tuple(loop_pts[0])]
        self.route = dm.Waypoints(pts, cruise=speed, loop_from=1)
        self.start_s = float(start_s)

    def step(self, t, dt):
        f, i = self.fleet, self.idx[0]
        if t < self.start_s:
            f.set_command(i, "off")
            return
        p, yaw = f.pos[i], float(f.yaw()[i])
        tgt, vel = self.route.step(p, dt)
        self.set_phase("patrol")
        f.set_command(i, "position", pos=tgt, vel=vel, yaw=self.heading(vel, yaw))


class Hover(Mission):
    """Inspection: climb to a point and hold, facing `look` (x, y), or orbit `look` at the point's distance."""
    kind = "hover"

    def __init__(self, name, fleet, i, point, look=None, orbit_speed=0.0, start_s=0.0):
        super().__init__(name, fleet, [i])
        self.point, self.look, self.orbit, self.start_s = tuple(point), look, float(orbit_speed), float(start_s)
        p0 = fleet.pos[i].copy()
        self.route = dm.Waypoints([(p0[0], p0[1], point[2]), self.point], cruise=6.0)

    def step(self, t, dt):
        f, i = self.fleet, self.idx[0]
        if t < self.start_s:
            f.set_command(i, "off")
            return
        p, yaw = f.pos[i], float(f.yaw()[i])
        if not self.route.done:
            tgt, vel = self.route.step(p, dt)
            self.set_phase("transit")
            f.set_command(i, "position", pos=tgt, vel=vel, yaw=self.heading(vel, yaw))
            return
        self.set_phase("orbit" if self.orbit else "hover")
        tgt, vel = self.point, (0.0, 0.0, 0.0)
        if self.look is not None and self.orbit:
            r = math.hypot(self.point[0] - self.look[0], self.point[1] - self.look[1])
            a = math.atan2(p[1] - self.look[1], p[0] - self.look[0]) + self.orbit / max(r, 1.0) * dt * 10
            tgt = (self.look[0] + r * math.cos(a), self.look[1] + r * math.sin(a), self.point[2])
            vel = (-math.sin(a) * self.orbit, math.cos(a) * self.orbit, 0.0)
        face = math.atan2(self.look[1] - p[1], self.look[0] - p[0]) if self.look is not None else yaw
        f.set_command(i, "position", pos=tgt, vel=vel, yaw=face)


class Swarm(Mission):
    """Vasarhelyi flocking round a loop of migration points at height z (absolute)."""
    kind = "swarm"

    def __init__(self, name, fleet, idx, route, z, params, obstacles=None, start_s=0.0, stagger_s=0.4,
                 others=lambda: ([], [])):
        super().__init__(name, fleet, idx)
        self.route, self.z, self.k = [tuple(p) for p in route], float(z), dm.flock_params(params)
        self.obs, self.leg = obstacles, 0
        self.start = [float(start_s) + stagger_s * n for n in range(len(idx))]
        self.member_phase = ["take-off"] * len(idx)
        self.others = others          # () -> (positions, velocities) of other drones it keeps away from

    def step(self, t, dt):
        f = self.fleet
        flock = [n for n, ph in enumerate(self.member_phase) if ph == "flock"]
        for n, i in enumerate(self.idx):
            if self.member_phase[n] != "take-off":
                continue
            if t < self.start[n]:
                f.set_command(i, "off")
                continue
            p = f.pos[i]
            f.set_command(i, "position", pos=(p[0], p[1], self.z), yaw=float(f.yaw()[i]))
            if abs(p[2] - self.z) < 2.0:
                self.member_phase[n] = "flock"
        if not flock:
            self.set_phase("take-off")
            return
        self.set_phase("flock")
        ids = [self.idx[n] for n in flock]
        pos, vel = f.pos[ids], f.vel[ids]
        centroid = pos.mean(axis=0)
        goal = self.route[self.leg % len(self.route)]
        if math.hypot(centroid[0] - goal[0], centroid[1] - goal[1]) < 25.0:
            self.leg += 1
            self.events.append(("drone_phase", f"{self.name} leg {self.leg}"))
            goal = self.route[self.leg % len(self.route)]
        op, ov = self.others()
        allp = np.vstack([pos] + ([np.asarray(op, float)] if len(op) else []))
        allv = np.vstack([vel] + ([np.asarray(ov, float)] if len(ov) else []))
        goals = np.full((len(allp), 3), np.nan)
        goals[:len(ids)] = (goal[0], goal[1], 0.0)
        goals[:len(ids), 2] = pos[:, 2]
        shills = [self.obs.walls(p, 30.0, self.z - 5.0) if self.obs is not None else [] for p in pos]
        want = dm.flock_velocities(allp, allv, goals, self.k, shills, members=list(range(len(ids))))
        for row, i in enumerate(ids):
            w = want[row].copy()
            w[2] = 0.8 * (self.z - pos[row, 2])      # and its height
            f.set_command(i, "velocity", vel=w, yaw=self.heading(w, float(f.yaw()[i])))


def separation(fleet, idx, others_pos=(), radius=10.0, gain=0.4, skip=()):
    """Velocity nudges (len(idx), 3) keeping AI drones `idx` apart from each other and from other drones (the
    pilots'): linear repulsion below `radius`, Vasarhelyi's repulsion term. Drones in `skip` are not nudged."""
    idx = list(idx)
    pts = fleet.pos[idx]
    extra = np.asarray(others_pos, float).reshape(-1, 3)
    allp = np.vstack([pts, extra]) if len(extra) else pts
    out = np.zeros((len(idx), 3))
    for r, i in enumerate(idx):
        if i in skip or fleet.on_ground[i]:
            continue
        d = pts[r] - allp
        dist = np.sqrt(np.sum(d * d, axis=1))
        m = (dist > 1e-3) & (dist < radius)
        if m.any():
            out[r] = np.sum((gain * (radius - dist[m]) / dist[m])[:, None] * d[m], axis=0)
    return out


def avoid_obstacles(fleet, idx, obs, lookahead_s=2.0, margin=4.0, radius=5.0):
    """A safety layer: drones whose position `lookahead_s` ahead would be within `margin` of an obstacle top (within
    `radius`) climb over it: returns {index: minimum height}."""
    out = {}
    if obs is None:
        return out
    for i in idx:
        if fleet.on_ground[i]:
            continue
        p, v = fleet.pos[i], fleet.vel[i]
        ahead = p + v * lookahead_s
        top = max(obs.top_at(p[0], p[1], radius), obs.top_at(ahead[0], ahead[1], radius))
        if top > -np.inf and p[2] < top + margin and math.hypot(v[0], v[1]) > 0.5:
            out[i] = top + margin
    return out


def loop_points(centre, radius, n=24, phase=0.0):
    return [(centre[0] + radius * math.cos(phase + 2 * math.pi * k / n),
             centre[1] + radius * math.sin(phase + 2 * math.pi * k / n)) for k in range(n)]


def make_rng(seed):
    return random.Random(seed)
