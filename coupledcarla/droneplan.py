"""Where AI drones fly: the obstacles of a map (building footprints with their heights, trees) as a height field, and
the missions coupled-sim's DroneSpawner flies, planned on it:

- delivery: an industrial drone takes off from its depot, climbs to its cruise height, flies to the address at its
  cruise speed (accelerating and braking smoothly, slowing in turns), descends to drop the parcel (lands, or hovers at
  `winch_m` and lowers it on a tether, as Wing's drones do), and flies back; then the next address. Addresses are
  given as lat/lon or picked from the map's buildings (houses and flats first), each with an open drop point beside
  the building; the depot likewise (given, or open ground near the area's centre). Routes (Router): see below.
- patrol: a loop of waypoints (given, or a circle) in the cruise band, round what reaches into it.
- swarm: Vasarhelyi et al. (2018) flocking at a height above the roofs of the area, migrating round a loop of points;
  buildings that reach up to the flock act through shill agents at their walls.
- hover (inspection): fly to a point at a height and hold there facing a target, or orbit it.

Routes (Router, routing mode "band"), as delivery drones fly them: in a fixed cruise band above the ground (Wing
about 45 m, Manna 50-80 m, Amazon's MK30 55-115 m; under the 120 m of EU 2019/947), in a layer by the route's heading
so that opposite directions never share a height (the heading-altitude rule of Doole, Ellerbroek, Knoop & Hoekstra
2021, "Constrained urban airspace design for large-scale drone-based delivery traffic", Aerospace 8(2) 38; Amazon's
outbound and inbound bands). Buildings and trees that reach into the band, less a vertical clearance, are obstacles,
inflated by a lateral clearance; both clearances default to SORA's operational volume (flight geography plus
contingency volume: GNSS and position-holding error, map error, the distance flown in the reaction time and the
stopping distance v^2 / (2 g tan(tilt)); vertically the altimeter error, the climb in the reaction time and v^2 / 2g;
JARUS SORA, as detailed in FOCA's GM SORA, ISS 02, section 13.3.2), so that no obstacle is inside the volume the
drone may use when something goes wrong. A* on an 8-connected grid finds the way round, line-of-sight shortcuts make
it any-angle (A* with post-smoothing, compared with Theta* by Daniel, Nash, Koenig & Felner 2010, JAIR 39), and
corners become arcs of radius v^2 / a_turn (smaller, and flown slower, where the obstacles leave no room), with
speed limits that the guidance brakes for. Higher cruise heights above the band are tried too, and the cheapest in
energy (dronemodel.steady_power: climb, descent and distance) wins: a drone climbs over a tower only when the way round
is much longer. No-fly areas (given polygons, OSM aerodromes, helipads and military land) are obstacles at every
height. Routing mode "over" keeps the old way: `clearance` above the highest roof or tree under the straight route.

Everything runs in the model's frame (x east, y north, z up, metres); building records come in CARLA's (x east,
y south), as buildings.json has them, and are flipped on loading. No CARLA here: drones.py hands in the records and
a ground-height function.
"""
import heapq
import json
import math
import random
import time
from dataclasses import dataclass, field

import numpy as np

from . import dronemodel as dm
from .config import DEFAULTS

ROUTING = DEFAULTS["drones"]["routing"]

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


# -- 3D routes ---------------------------------------------------------------------------------------------------

def routing(*overrides):
    """ROUTING with the given dicts merged over it (None skipped)."""
    rc = dict(ROUTING)
    for o in overrides:
        rc.update(o or {})
    return rc


def margins(af, speed, rc=None):
    """(lateral, vertical) clearance (m) a drone of airframe af keeps from obstacles at cruise speed `speed`: half
    the flight geography (SORA: at least 3 characteristic dimensions wide and high) plus the contingency volume of
    SORA's detailed approach (FOCA GM SORA 13.3.2): S_CV = S_GNSS + S_pos + S_map + V0 t_R + V0^2 / (2 g tan theta)
    for a multirotor's stop, H_CV = H_baro + V_climb t_R + V0^2 / 2g. Numbers in rc override either."""
    rc = rc or ROUTING
    cd = 2.0 * (af.arm + af.rotor_radius)              # characteristic dimension: rotor tip to rotor tip
    t = float(rc.get("reaction_s", 1.0))
    tan = math.tan(min(af.max_tilt, math.radians(45.0)))
    lat = 1.5 * cd + float(rc.get("gnss_m", 3.0)) + float(rc.get("position_hold_m", 3.0)) + \
        float(rc.get("map_error_m", 1.0)) + speed * t + speed * speed / (2.0 * dm.G * tan)
    ver = 1.5 * cd + float(rc.get("altimeter_m", 1.0)) + af.max_climb * t + speed * speed / (2.0 * dm.G)
    if isinstance(rc.get("lateral_clearance_m"), (int, float)):
        lat = float(rc["lateral_clearance_m"])
    if isinstance(rc.get("vertical_clearance_m"), (int, float)):
        ver = float(rc["vertical_clearance_m"])
    return lat, ver


def leg_energy_wh(af, horizontal_m, climb_m, descent_m, speed, vrate=4.0):
    """Energy (Wh) of a leg: climb, cruise and descent at steady speeds (dronemodel.steady_power)."""
    e = dm.steady_power(af, speed) * horizontal_m / max(speed, 0.1)
    e += dm.steady_power(af, 0.0, vrate) * max(0.0, climb_m) / vrate
    e += dm.steady_power(af, 0.0, -vrate) * max(0.0, descent_m) / vrate
    return e / 3600.0


def path_length(xy):
    xy = np.asarray(xy, float)
    return float(np.sum(np.linalg.norm(np.diff(xy[:, :2], axis=0), axis=1))) if len(xy) > 1 else 0.0


def _dilate(mask, r):
    """Cells whose centre is within r cells of a True cell's centre (a disc, row by row with running sums)."""
    if r <= 0 or not mask.any():
        return mask.copy()
    ny, nx = mask.shape
    out = np.zeros_like(mask)
    c = np.concatenate([np.zeros((ny, 1), np.int32), np.cumsum(mask, axis=1, dtype=np.int32)], axis=1)
    cols = np.arange(nx)
    rows_any = mask.any(axis=1)
    R = int(math.floor(r))
    for dy in range(-R, R + 1):
        w = int(math.floor(math.sqrt(max(0.0, r * r - dy * dy))))
        lo, hi = np.clip(cols - w, 0, nx), np.clip(cols + w + 1, 0, nx)
        if dy >= 0:
            src = slice(dy, ny)
            dst = slice(0, ny - dy)
        else:
            src = slice(0, ny + dy)
            dst = slice(-dy, ny)
        if not rows_any[src].any():
            continue
        out[dst] |= (c[src][:, hi] - c[src][:, lo]) > 0
    return out


def _raster(polys, shape, x0, y0, cell):
    """Cells whose centres are inside any of the polygons (model x, y)."""
    m = np.zeros(shape, bool)
    for pts in polys:
        pts = np.asarray(pts, float)
        if len(pts) < 3:
            continue
        j0 = max(0, int((pts[:, 0].min() - x0) // cell))
        j1 = min(shape[1] - 1, int((pts[:, 0].max() - x0) // cell))
        i0 = max(0, int((pts[:, 1].min() - y0) // cell))
        i1 = min(shape[0] - 1, int((pts[:, 1].max() - y0) // cell))
        if i1 < i0 or j1 < j0:
            continue
        X, Y = np.meshgrid(x0 + (np.arange(j0, j1 + 1) + 0.5) * cell, y0 + (np.arange(i0, i1 + 1) + 0.5) * cell)
        inside = np.zeros(X.shape, bool)
        xa, ya = pts[:, 0], pts[:, 1]
        xb, yb = np.roll(xa, -1), np.roll(ya, -1)
        for k in range(len(xa)):
            if ya[k] != yb[k]:
                inside ^= ((ya[k] > Y) != (yb[k] > Y)) & (X < (xb[k] - xa[k]) * (Y - ya[k]) / (yb[k] - ya[k]) + xa[k])
        m[i0:i1 + 1, j0:j1 + 1] |= inside
    return m


@dataclass
class Leg:
    """A planned leg: 3D points from above the start (at the cruise height) to above the end, a speed limit per point
    (None: none), the cruise height, horizontal length and straight distance (m), estimated energy (Wh), how it was
    planned and how long that took."""
    points: list
    limits: list
    z: float
    length: float
    straight: float
    energy_wh: float
    how: str = "band"
    ms: float = 0.0
    tried: list = field(default_factory=list)     # (cruise z, length or None, energy or None) per height tried

    @property
    def detour(self):
        return self.length / max(self.straight, 1e-6)


@dataclass
class Loop:
    """A planned patrol loop: transit points from the start to the loop, then the loop (closed), with speed limits."""
    transit: list
    transit_limits: list
    points: list
    limits: list
    z: float
    length: float
    how: str = "band"
    ms: float = 0.0


class Router:
    """Routes in 3D over the obstacle height field: a cruise band, detours round what reaches into it, climbing over
    only when that saves energy. obs: Obstacles; rc: routing options (ROUTING); no_fly: polygons (model x, y)."""

    def __init__(self, obs, rc=None, no_fly=(), log=None):
        self.obs, self.rc, self.log = obs, routing(rc), log or (lambda m: None)
        f = max(1, int(round(float(self.rc.get("cell_m", 4.0)) / obs.cell)))
        self.cell = obs.cell * f
        ny, nx = obs.top.shape
        top = np.full((-(-ny // f) * f, -(-nx // f) * f), -np.inf)
        top[:ny, :nx] = obs.top
        self.top = top.reshape(top.shape[0] // f, f, top.shape[1] // f, f).max(axis=(1, 3))
        self.x0, self.y0 = obs.x0, obs.y0
        self.no_fly = [np.asarray(p, float) for p in no_fly if len(p) >= 3]
        if self.no_fly:        # the grid covers the no-fly areas and 150 m round them too
            allp = np.vstack(self.no_fly)
            c = self.cell
            ny, nx = self.top.shape
            pl = max(0, int(math.ceil((self.x0 - (allp[:, 0].min() - 150.0)) / c)))
            pb = max(0, int(math.ceil((self.y0 - (allp[:, 1].min() - 150.0)) / c)))
            pr = max(0, int(math.ceil((allp[:, 0].max() + 150.0 - (self.x0 + nx * c)) / c)))
            pt = max(0, int(math.ceil((allp[:, 1].max() + 150.0 - (self.y0 + ny * c)) / c)))
            self.top = np.pad(self.top, ((pb, pt), (pl, pr)), constant_values=-np.inf)
            self.x0, self.y0 = self.x0 - pl * c, self.y0 - pb * c
        self.no_fly_core = _raster(self.no_fly, self.top.shape, self.x0, self.y0, self.cell)
        self._masks = {}
        self._nf = {}

    # -- grid -------------------------------------------------------------------------------------------------------
    def ij(self, x, y):
        return (int(math.floor((y - self.y0) / self.cell)), int(math.floor((x - self.x0) / self.cell)))

    def xy(self, i, j):
        return (self.x0 + (j + 0.5) * self.cell, self.y0 + (i + 0.5) * self.cell)

    def inside(self, i, j):
        return 0 <= i < self.top.shape[0] and 0 <= j < self.top.shape[1]

    def _no_fly_mask(self, lateral):
        b = self.rc.get("no_fly_buffer_m", "auto")
        b = float(b) if isinstance(b, (int, float)) else lateral
        key = round(b, 1)
        if key not in self._nf:
            self._nf[key] = _dilate(self.no_fly_core, (b + self.cell * 0.7072) / self.cell) \
                if self.no_fly_core.any() else self.no_fly_core
        return self._nf[key]

    def masks(self, z, lateral, vertical):
        """(blocked, core) at cruise height z: core where an obstacle reaches above z - vertical, blocked where one
        is within `lateral` (plus half a cell's diagonal for the grid), or a no-fly area."""
        key = (round(z, 1), round(lateral, 1), round(vertical, 1))
        if key not in self._masks:
            core = self.top > z - vertical
            blocked = _dilate(core, (lateral + self.cell * 0.7072) / self.cell) | self._no_fly_mask(lateral)
            if len(self._masks) > 24:
                self._masks.pop(next(iter(self._masks)))
            self._masks[key] = (blocked, core | self.no_fly_core)
        return self._masks[key]

    def _relaxed(self, blocked, core, ends, lateral):
        """A copy of blocked opened round a leg's end that lies within the clearance of something tall (a drone climbs
        and lowers its parcel beside a building): the cells near the end that are no closer to an obstacle than the
        end itself, so that the drone leaves the end without coming nearer to anything."""
        out = None
        r = int(math.ceil((lateral + 2 * self.cell) / self.cell))
        rr = r + int(math.ceil((lateral + self.cell) / self.cell))
        for (x, y) in ends:
            i, j = self.ij(x, y)
            if not self.inside(i, j) or not blocked[i, j]:
                continue
            if out is None:
                out = blocked.copy()
            i0, i1 = max(0, i - r), min(blocked.shape[0], i + r + 1)
            j0, j1 = max(0, j - r), min(blocked.shape[1], j + r + 1)
            k0, k1 = max(0, i - rr), min(blocked.shape[0], i + rr + 1)
            l0, l1 = max(0, j - rr), min(blocked.shape[1], j + rr + 1)
            ci, cj = np.nonzero(core[k0:k1, l0:l1])
            I, J = np.mgrid[i0:i1, j0:j1]
            disc = (I - i) ** 2 + (J - j) ** 2 <= r * r
            if len(ci):
                d2 = np.min((I[..., None] - (ci + k0)) ** 2 + (J[..., None] - (cj + l0)) ** 2, axis=-1)
                d_end = d2[i - i0, j - j0]
                disc &= np.sqrt(d2) >= math.sqrt(d_end) - 0.5
            sub = out[i0:i1, j0:j1]
            sub[disc & ~core[i0:i1, j0:j1]] = False
        return blocked if out is None else out

    def free(self, m, p, q):
        """No blocked cell on the segment p -> q (sampled at a quarter cell; outside the grid is free)."""
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(d / (self.cell * 0.25)) + 2)
        t = np.linspace(0.0, 1.0, n)
        x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
        i = np.floor((y - self.y0) / self.cell).astype(int)
        j = np.floor((x - self.x0) / self.cell).astype(int)
        ok = (i >= 0) & (i < m.shape[0]) & (j >= 0) & (j < m.shape[1])
        return not m[i[ok], j[ok]].any()

    def astar(self, m, a, b, margin_cells=40):
        """Cell path a -> b (model x, y) on the 8-connected grid without blocked cells (no corner cutting), in a
        window round the two (then the whole grid); None if there is none."""
        sa, sb = self.ij(*a), self.ij(*b)
        ny, nx = m.shape
        sa = (min(max(sa[0], 0), ny - 1), min(max(sa[1], 0), nx - 1))
        sb = (min(max(sb[0], 0), ny - 1), min(max(sb[1], 0), nx - 1))
        span = max(abs(sa[0] - sb[0]), abs(sa[1] - sb[1]))
        for pad in (max(margin_cells, span // 2), max(ny, nx)):
            i0, i1 = max(0, min(sa[0], sb[0]) - pad), min(ny, max(sa[0], sb[0]) + pad + 1)
            j0, j1 = max(0, min(sa[1], sb[1]) - pad), min(nx, max(sa[1], sb[1]) + pad + 1)
            path = self._astar(m[i0:i1, j0:j1].copy(), (sa[0] - i0, sa[1] - j0), (sb[0] - i0, sb[1] - j0))
            if path is not None:
                return [(i + i0, j + j0) for i, j in path]
            if i0 == 0 and j0 == 0 and i1 == ny and j1 == nx:
                break
        return None

    @staticmethod
    def _astar(sub, s, g):
        sub[s] = False
        sub[g] = False
        ny, nx = sub.shape
        W = nx
        flat = sub.ravel()
        gs, gg = s[0] * W + s[1], g[0] * W + g[1]
        cost = np.full(ny * nx, np.inf)
        parent = np.full(ny * nx, -1, np.int64)
        done = np.zeros(ny * nx, bool)
        cost[gs] = 0.0
        gi, gj = g
        r2 = math.sqrt(2.0)

        def h(i, j):
            di, dj = abs(i - gi), abs(j - gj)
            return (di + dj) + (r2 - 2.0) * min(di, dj)
        heap = [(h(*s), 0.0, gs)]
        steps = ((0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                 (1, 1, r2), (1, -1, r2), (-1, 1, r2), (-1, -1, r2))
        while heap:
            _f, c, u = heapq.heappop(heap)
            if done[u]:
                continue
            if u == gg:
                break
            done[u] = True
            ui, uj = divmod(u, W)
            for di, dj, w in steps:
                vi, vj = ui + di, uj + dj
                if vi < 0 or vj < 0 or vi >= ny or vj >= nx:
                    continue
                v = vi * W + vj
                if flat[v] or done[v]:
                    continue
                if di and dj and (flat[ui * W + vj] or flat[vi * W + uj]):
                    continue
                nc = c + w
                if nc < cost[v]:
                    cost[v] = nc
                    parent[v] = u
                    heapq.heappush(heap, (nc + h(vi, vj), nc, v))
        if not np.isfinite(cost[gg]):
            return None
        out, u = [], gg
        while u != -1:
            out.append(divmod(int(u), W))
            u = parent[u]
        return out[::-1]

    def path2d(self, m, a, b):
        """Any-angle path a -> b (x, y lists) avoiding blocked cells: straight if free, else A* then line-of-sight
        shortcuts (each point joined to the farthest one it sees)."""
        a, b = (float(a[0]), float(a[1])), (float(b[0]), float(b[1]))
        if self.free(m, a, b):
            return [a, b]
        cells = self.astar(m, a, b)
        if cells is None:
            return None
        pts = [a] + [self.xy(i, j) for i, j in cells[1:-1]] + [b]
        out, k = [a], 0
        while k < len(pts) - 1:
            j = k + 1
            while j + 1 < len(pts) and self.free(m, pts[k], pts[j + 1]):
                j += 1
            out.append(pts[j])
            k = j
        return out

    def smooth(self, m, pts, speed, closed=False):
        """Corners of a polyline as arcs of radius speed^2 / turn_accel (smaller where the arc would meet a blocked
        cell or the segments are short), each arc point with the speed limit sqrt(turn_accel r): (points, limits)."""
        a_turn = float(self.rc.get("turn_accel_mps2", 2.0))
        P = [np.asarray(p, float) for p in pts]
        if closed and len(P) > 1 and np.linalg.norm(P[0] - P[-1]) < 1e-6:
            P = P[:-1]
        n = len(P)
        if n < 3:
            return [tuple(p) for p in P], [None] * n
        r0 = speed * speed / a_turn
        corners = range(n) if closed else range(1, n - 1)
        arcs = {}
        for k in corners:
            p0, p1, p2 = P[k - 1], P[k], P[(k + 1) % n]
            lin, lout = np.linalg.norm(p1 - p0), np.linalg.norm(p2 - p1)
            if lin < 1e-6 or lout < 1e-6:
                continue
            u1, u2 = (p1 - p0) / lin, (p2 - p1) / lout
            th = math.acos(max(-1.0, min(1.0, float(np.dot(u1, u2)))))
            if th < math.radians(3.0):
                arcs[k] = None                  # nearly straight: drop the point
                continue
            tmax = 0.5 * min(lin, lout)
            r = min(r0, tmax / math.tan(th / 2.0))
            side = 1.0 if (u1[0] * u2[1] - u1[1] * u2[0]) > 0 else -1.0
            nrm = side * np.array([-u1[1], u1[0]])
            best = (p1, None)                    # a sharp corner (flown slowly) if no arc fits
            while r >= 2.0:
                t = r * math.tan(th / 2.0)
                t1, c = p1 - u1 * t, p1 - u1 * t + nrm * r
                a0 = math.atan2(t1[1] - c[1], t1[0] - c[0])
                ns = max(2, int(math.ceil(th / math.radians(15.0))))
                arc = [c + r * np.array([math.cos(a0 + side * th * s / ns), math.sin(a0 + side * th * s / ns)])
                       for s in range(ns + 1)]
                if all(self.free(m, arc[s], arc[s + 1]) for s in range(ns)):
                    best = (arc, min(speed, math.sqrt(a_turn * r)))
                    break
                r *= 0.5
            arcs[k] = best
        out, lim = [], []
        for k in range(n):
            if k in arcs:
                if arcs[k] is None:
                    continue
                arc, v = arcs[k]
                if v is None:      # no arc fits: stop-and-turn speed at the corner
                    out.append(tuple(arc))
                    lim.append(1.5)
                else:
                    out.extend(tuple(q) for q in arc)
                    lim.extend([v] * len(arc))
            else:
                out.append(tuple(P[k]))
                lim.append(None)
        return out, lim

    # -- legs -------------------------------------------------------------------------------------------------------
    def layer_offset(self, bearing, k=0):
        lo, hi = self.band()
        lm = float(self.rc.get("layer_m", 4.0))
        n = max(1, int((hi - lo) / lm + 1e-6) + 1) if lm > 0 else 1
        if self.rc.get("layers", "heading") == "heading":
            idx = int(((math.degrees(bearing) % 360.0) / 360.0) * n) % n
        else:
            idx = int(k) % n
        return idx * lm

    def band(self):
        lo, hi = (list(self.rc.get("band_m") or [40.0, 60.0]) + [None])[:2]
        lo = float(lo)
        hi = float(hi) if hi is not None else lo
        return lo, max(lo, hi)

    def over_leg(self, a, b, af, speed, end_z, clearance=20.0, corridor=10.0, layer=0.0):
        """The old way: straight, `clearance` above the highest top within `corridor` of the line."""
        t0 = time.perf_counter()
        top = self.obs.corridor_top(a, b, corridor)
        z = max(top, max(a[2], b[2])) + clearance + layer
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        e = leg_energy_wh(af, d, z - a[2], z - end_z, speed)
        return Leg([(a[0], a[1], z), (b[0], b[1], z)], [None, None], z, d, d, e, "over",
                   (time.perf_counter() - t0) * 1000.0)

    def leg(self, a, b, af, speed, end_z=None, k=0, clearance=20.0, corridor=10.0, layer_m=None):
        """The leg from a to b ((x, y, ground z) each), ending at height end_z (default b's ground): Leg."""
        a = (float(a[0]), float(a[1]), float(a[2]))
        b = (float(b[0]), float(b[1]), float(b[2]))
        end_z = b[2] if end_z is None else float(end_z)
        rc = self.rc
        if rc.get("mode", "band") == "over":
            lm = float(rc.get("layer_m", 4.0) if layer_m is None else layer_m)
            return self.over_leg(a, b, af, speed, end_z, clearance, corridor, lm * k)
        t0 = time.perf_counter()
        lateral, vertical = margins(af, speed, rc)
        ground = max(a[2], b[2])
        lo, hi = self.band()
        bearing = math.atan2(b[1] - a[1], b[0] - a[0])
        z_band = ground + lo + self.layer_offset(bearing, k)
        cap = ground + float(rc.get("max_altitude_m", 120.0))
        straight = math.hypot(b[0] - a[0], b[1] - a[1])
        heights = [min(z_band, cap)]
        if rc.get("climb_over", True):
            step = max(1.0, float(rc.get("over_step_m", 10.0)))
            z = ground + hi + step
            while z <= cap + 1e-6:
                heights.append(z)
                z += step
            zs = self.obs.corridor_top(a, b, lateral) + vertical      # where the straight line is clear
            if ground + hi + 0.5 < zs <= cap:
                heights.append(zs)
        heights = sorted(set(round(h, 2) for h in heights))
        pen = max(1.0, float(rc.get("over_penalty", 1.0)))
        best, tried = None, []
        for z in heights:
            above = z > ground + hi + 0.5
            floor = leg_energy_wh(af, straight, z - a[2], z - end_z, speed) * (pen if above else 1.0)
            if best is not None and floor >= best[0]:
                break                     # even straight it would cost more: higher is no better
            blocked, core = self.masks(z, lateral, vertical)
            m = self._relaxed(blocked, core, (a[:2], b[:2]), lateral)
            path = self.path2d(m, a, b)
            if path is None:
                tried.append((z, None, None))
                continue
            xy, lim = self.smooth(m, path, speed)
            L = path_length(xy)
            e = leg_energy_wh(af, L, z - a[2], z - end_z, speed)
            tried.append((z, L, e))
            score = e * (pen if above else 1.0)
            if best is None or score < best[0]:
                best = (score, z, xy, lim, L, e, "band" if not above else "over the band")
            if L <= straight * 1.001:
                break
        if best is None:
            leg = self.over_leg(a, b, af, speed, end_z, vertical, lateral)
            leg.how, leg.tried = "fallback (no way round)", tried
            leg.z = min(leg.z, cap)
            leg.points = [(p[0], p[1], leg.z) for p in leg.points]
            self.log(f"drones: no route {a[0]:.0f},{a[1]:.0f} -> {b[0]:.0f},{b[1]:.0f} in the band or over it "
                     f"below {cap - ground:.0f} m; flying straight at {leg.z - ground:.0f} m")
            return leg
        _s, z, xy, lim, L, e, how = best
        pts = [(x, y, z) for x, y in xy]
        lim = [None] + list(lim[1:-1]) + [None]
        return Leg(pts, lim, z, L, straight, e, how, (time.perf_counter() - t0) * 1000.0, tried)

    def loop(self, xy, start, af, speed, z=None, k=0):
        """A patrol loop through the points xy (closed) at height z (default: the band, layer k by drone) from start
        (x, y, ground z): points that lie in a blocked area move to the nearest open cell, each side goes round what
        reaches up to it, the loop is smoothed. If a side has no way round, the next height up is tried."""
        t0 = time.perf_counter()
        rc = self.rc
        lateral, vertical = margins(af, speed, rc)
        ground = float(start[2])
        lo, _hi = self.band()
        lm = float(rc.get("layer_m", 4.0))
        z = ground + lo + lm * int(k) if z is None else float(z)
        cap = ground + float(rc.get("max_altitude_m", 120.0))
        step = max(1.0, float(rc.get("over_step_m", 10.0)))
        while True:
            blocked, core = self.masks(z, lateral, vertical)
            m = self._relaxed(blocked, core, (start[:2],), lateral)
            pts = []
            for p in xy:
                q = self.nearest_open(m, p)
                if q is not None and (not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > self.cell):
                    pts.append(q)
            ring = []
            ok = len(pts) >= 2
            for p, q in zip(pts, pts[1:] + pts[:1]):
                if not ok:
                    break
                seg = self.path2d(m, p, q)
                if seg is None:
                    ok = False
                    break
                ring.extend(seg[:-1])
            transit = self.path2d(m, start[:2], pts[0]) if ok else None
            if ok and transit is not None:
                break
            if z + step > cap + 1e-6:
                self.log(f"drones: patrol loop has no way round at any height up to {cap - ground:.0f} m; "
                         f"flying it as given at {z - ground:.0f} m")
                ring, transit = [tuple(p[:2]) for p in xy], [tuple(start[:2]), tuple(xy[0][:2])]
                pts = ring
                break
            z += step
        loop_xy, loop_lim = self.smooth(m, ring + ring[:1], speed, closed=True)
        tr_xy, tr_lim = self.smooth(m, transit, speed)
        L = path_length(loop_xy + loop_xy[:1])
        return Loop([(x, y, z) for x, y in tr_xy[1:-1]], list(tr_lim[1:-1]),
                    [(x, y, z) for x, y in loop_xy], list(loop_lim), z, L,
                    "band" if z <= ground + _hi + 1e-6 else "over the band", (time.perf_counter() - t0) * 1000.0)

    def nearest_open(self, m, p, radius=200.0):
        """p if its cell is open, else the nearest open cell centre within radius (None if none)."""
        i, j = self.ij(p[0], p[1])
        if not self.inside(i, j) or not m[i, j]:
            return (float(p[0]), float(p[1]))
        r = int(math.ceil(radius / self.cell))
        i0, i1 = max(0, i - r), min(m.shape[0], i + r + 1)
        j0, j1 = max(0, j - r), min(m.shape[1], j + r + 1)
        I, J = np.nonzero(~m[i0:i1, j0:j1])
        if not len(I):
            return None
        d = (I + i0 - i) ** 2 + (J + j0 - j) ** 2
        n = int(np.argmin(d))
        return self.xy(int(I[n] + i0), int(J[n] + j0))


def osm_no_fly(bbox, path, timeout=25):
    """Aerodromes, heliports, helipads and military land in the box from OSM (Overpass), as rings of [lat, lon];
    cached in `path` (real/osm_nofly.json). A helipad drawn as a point becomes a 30 m circle. Fails soft: []."""
    from pathlib import Path
    path = Path(path)
    if path.exists():
        try:
            return json.loads(path.read_text())
        except ValueError:
            pass
    s, w, n, e = bbox
    b = f"({s},{w},{n},{e})"
    query = (f'[out:json][timeout:{timeout}];(nwr["aeroway"~"^(aerodrome|heliport|helipad)$"]{b};'
             f'way["landuse"="military"]{b};relation["landuse"="military"]{b};way["military"]{b};);out geom;')
    try:
        import requests
        from .realworld import OVERPASS, UA
        data = None
        for url in OVERPASS:
            try:
                r = requests.post(url, data={"data": query}, headers=UA, timeout=timeout + 5)
                r.raise_for_status()
                data = r.json()
                break
            except (requests.RequestException, ValueError):
                continue
        if data is None:
            raise RuntimeError("no Overpass server answered")
    except Exception as err:  # noqa: BLE001 - offline: no OSM no-fly areas
        print(f"drones: OSM no-fly areas not fetched ({type(err).__name__}: {err})")
        return []
    rings = []
    for el in data.get("elements", []):
        if el["type"] == "node":
            lat, lon = el["lat"], el["lon"]
            dl = 30.0 / 111320.0
            rings.append([[lat + dl * math.sin(a), lon + dl * math.cos(a) / math.cos(math.radians(lat))]
                          for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)])
        elif el["type"] == "way" and el.get("geometry"):
            rings.append([[g["lat"], g["lon"]] for g in el["geometry"]])
        elif el["type"] == "relation":
            for mem in el.get("members", []):
                if mem.get("role") == "outer" and mem.get("geometry"):
                    rings.append([[g["lat"], g["lon"]] for g in mem["geometry"]])
    rings = [r for r in rings if len(r) >= 3]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rings))
    return rings


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
    """Depot -> address -> depot for each address in turn (then again from the first, with loop). Each trip flies
    straight at cruise_z[k], or the planned legs[k] = (out, back) (Router.leg) when given."""
    kind = "delivery"

    def __init__(self, name, fleet, i, depot, drops, cruise_z, ground, start_s=0.0, dwell_s=10.0, cruise=12.0,
                 drop="land", winch_m=7.0, loop=True, legs=None):
        super().__init__(name, fleet, [i])
        self.depot, self.drops = depot, list(drops)       # (x, y, ground z)
        self.legs = list(legs) if legs else None
        self.cruise_z = list(cruise_z) if cruise_z is not None else [lg[0].z for lg in self.legs]
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
            end_z = b[2] + (self.winch_m if (self.drop == "winch" and self.outbound) else 0.3)
            if self.legs:
                leg = self.legs[self.k][0 if self.outbound else 1]
                z = leg.z
                pts = list(leg.points) + [(b[0], b[1], end_z)]
                lim = list(leg.limits) + [None]
            else:
                z = self.cruise_z[self.k]
                pts = [(a[0], a[1], z), (b[0], b[1], z), (b[0], b[1], end_z)]
                lim = None
            self.route = dm.Waypoints(pts, cruise=self.cruise, climb=4.0, limits=lim)
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

    """A loop flown for ever; with a planned Loop (Router.loop): its transit from the start, then its smoothed ring
    with the speed limits of its arcs."""

    def __init__(self, name, fleet, i, loop_pts, speed=8.0, start_s=0.0, limits=None, transit=(),
                 transit_limits=None):
        super().__init__(name, fleet, [i])
        p0 = fleet.pos[i].copy()
        transit = [tuple(p) for p in transit]
        pts = [(p0[0], p0[1], loop_pts[0][2])] + transit + [tuple(p) for p in loop_pts] + [tuple(loop_pts[0])]
        lim = None
        if limits is not None:
            lim = [None] + list(transit_limits or [None] * len(transit)) + list(limits) + [limits[0]]
        self.route = dm.Waypoints(pts, cruise=speed, loop_from=1 + len(transit), limits=lim)
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
