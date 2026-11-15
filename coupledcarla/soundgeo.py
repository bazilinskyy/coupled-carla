"""The map's buildings for the sound: occlusion of a source by buildings (ISO 9613-2 barrier attenuation over the
roofs and around the sides), first-order reflections off the nearest facades (image sources), and the street
canyon around the listener (width and height) for the reverberant tail.

The footprints come from maps/<name>/buildings.json (CARLA x, y; height h; base z), the same file the drones and the
trial viewer use; for a baked level without one, drones.py's cached drone_obstacles.json. Facade edges are kept in
a grid of `cell` metres, so a ray only tests the edges of the cells it crosses (one vectorised test per ray: about
30 us). No CARLA here.
"""
import json
import math
from pathlib import Path

import numpy as np

from . import acoustics as ac

C = 343.0


class Buildings:
    def __init__(self, records, cell=16.0):
        """records: [{"p": [[x, y], ...] CARLA, "h": height m, "z": base z}]"""
        self.cell = float(cell)
        segs, bid, tops, polys = [], [], [], []
        for b in records:
            pts = np.asarray(b.get("p") or [], float)
            if len(pts) < 3:
                continue
            if np.allclose(pts[0], pts[-1]):
                pts = pts[:-1]
            if len(pts) < 3:
                continue
            # counter-clockwise in a right-handed (x, -y) frame: compute the signed area in CARLA's frame and keep
            # an outward normal per edge
            area = 0.5 * float(np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]))
            if abs(area) < 4.0:
                continue
            if area < 0:
                pts = pts[::-1]
            i = len(polys)
            polys.append(pts)
            tops.append(float(b.get("z") or 0.0) + float(b.get("h") or 8.0))
            q = np.roll(pts, -1, axis=0)
            segs.append(np.hstack([pts, q]))
            bid.append(np.full(len(pts), i))
        self.polys = polys
        self.tops = np.asarray(tops, float)
        self.n = len(polys)
        if not segs:
            self.seg = np.zeros((0, 4))
            self.bid = np.zeros(0, int)
            self.grid = {}
            return
        self.seg = np.vstack(segs)
        self.bid = np.concatenate(bid)
        d = self.seg[:, 2:] - self.seg[:, :2]
        self.length = np.hypot(d[:, 0], d[:, 1])
        # outward normal of a counter-clockwise polygon (positive area in x, y): (dy, -dx)
        self.normal = np.column_stack([d[:, 1], -d[:, 0]]) / np.maximum(self.length, 1e-9)[:, None]
        grid = {}
        c = self.cell
        lo = np.floor(np.minimum(self.seg[:, :2], self.seg[:, 2:]) / c).astype(int)
        hi = np.floor(np.maximum(self.seg[:, :2], self.seg[:, 2:]) / c).astype(int)
        for k in range(len(self.seg)):
            for gx in range(lo[k, 0], hi[k, 0] + 1):
                for gy in range(lo[k, 1], hi[k, 1] + 1):
                    grid.setdefault((gx, gy), []).append(k)
        self.grid = {key: np.asarray(v, int) for key, v in grid.items()}

    # -- loading ------------------------------------------------------------------------------------------------
    @classmethod
    def from_map(cls, mdir, log=print):
        """Buildings of a map folder, or None (no file: no occlusion, one log line)."""
        mdir = Path(mdir)
        for name in ("buildings.json", "drone_obstacles.json"):
            p = mdir / name
            if p.exists():
                try:
                    recs = json.loads(p.read_text())
                    b = cls(recs)
                    log(f"sound: {b.n} buildings from {name} for occlusion and reflections")
                    return b
                except (OSError, ValueError) as e:
                    log(f"sound: cannot read {p} ({e}); no building occlusion")
                    return None
        log(f"sound: no buildings.json in {mdir}; no building occlusion")
        return None

    # -- rays -----------------------------------------------------------------------------------------------------
    def _cells(self, x0, y0, x1, y1):
        c = self.cell
        n = int(max(abs(x1 - x0), abs(y1 - y0)) / (0.5 * c)) + 1
        xs = np.linspace(x0, x1, n + 1) / c
        ys = np.linspace(y0, y1, n + 1) / c
        keys = set(zip(np.floor(xs).astype(int).tolist(), np.floor(ys).astype(int).tolist()))
        # the diagonal neighbours a thin line can cross between samples
        extra = set()
        for gx, gy in keys:
            extra.update(((gx + 1, gy), (gx, gy + 1), (gx - 1, gy), (gx, gy - 1)))
        return keys | extra

    def edges_on(self, x0, y0, x1, y1):
        ks = [self.grid[k] for k in self._cells(x0, y0, x1, y1) if k in self.grid]
        return np.unique(np.concatenate(ks)) if ks else np.zeros(0, int)

    def hits(self, s, r):
        """Where the segment s-r (2D) crosses facade edges: (ts along s->r 0..1, edge indices), sorted by t."""
        if not len(self.seg):
            return np.zeros(0), np.zeros(0, int)
        e = self.edges_on(s[0], s[1], r[0], r[1])
        if not len(e):
            return np.zeros(0), e
        P = self.seg[e]
        px, py = s[0], s[1]
        rx, ry = r[0] - px, r[1] - py
        qx, qy = P[:, 0], P[:, 1]
        sx, sy = P[:, 2] - qx, P[:, 3] - qy
        den = rx * sy - ry * sx
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((qx - px) * sy - (qy - py) * sx) / den
            u = ((qx - px) * ry - (qy - py) * rx) / den
        ok = (np.abs(den) > 1e-12) & (t > 1e-6) & (t < 1 - 1e-6) & (u >= 0) & (u <= 1)
        order = np.argsort(t[ok])
        return t[ok][order], e[ok][order]

    def occlusion(self, s, r, hs=0.05, hr=1.5):
        """Per-band attenuation (dB) of the direct path s -> r (CARLA x, y, z) by the buildings in between: the
        energetic sum of the path over the roofs (double diffraction, C3 for the thickness) and the two paths around
        the sides (round the convex hull of the blocking buildings), ISO 9613-2 7.4. Zero when nothing blocks.
        Returns (attenuation per band, number of buildings in the way)."""
        t, e = self.hits(s, r)
        if not len(t):
            return np.zeros(ac.NB), 0
        bids = np.unique(self.bid[e])
        d2 = math.hypot(r[0] - s[0], r[1] - s[1])
        d3 = math.dist(s, r)
        zs, zr = s[2] + 0.0, r[2] + 0.0
        top = float(self.tops[bids].max())
        # over the roofs: from the first wall crossed to the last, at the highest roof
        t0, t1 = float(t[0]), float(t[-1])
        ztop = top + 0.0
        za, zb = zs + (zr - zs) * t0, zs + (zr - zs) * t1
        if ztop <= max(za, zb):
            over = np.zeros(ac.NB)        # the line passes above the roofs
            Tover = np.ones(ac.NB)
        else:
            a1 = math.hypot(t0 * d2, ztop - zs)
            e_ = (t1 - t0) * d2
            a2 = math.hypot((1 - t1) * d2, ztop - zr)
            z = a1 + e_ + a2 - d3
            over = ac.barrier_attenuation(z, double=True, e=e_)
            Tover = 10 ** (-over / 10)
        # around the sides: tangent vertices of the blocking buildings' hull seen from s and from r
        pts = np.vstack([self.polys[i] for i in bids])
        ux, uy = (r[0] - s[0]) / max(d2, 1e-9), (r[1] - s[1]) / max(d2, 1e-9)
        T = Tover.copy()
        for side in (1.0, -1.0):
            rel_s = pts - np.array([s[0], s[1]])
            rel_r = pts - np.array([r[0], r[1]])
            cross_s = side * (ux * rel_s[:, 1] - uy * rel_s[:, 0])
            if not np.any(cross_s > 0):
                continue
            # the vertex seen at the widest angle from s on this side, and from r
            ang_s = np.arctan2(cross_s, ux * rel_s[:, 0] + uy * rel_s[:, 1])
            cross_r = side * (ux * rel_r[:, 1] - uy * rel_r[:, 0])
            ang_r = np.arctan2(cross_r, -(ux * rel_r[:, 0] + uy * rel_r[:, 1]))
            vs, vr = pts[int(np.argmax(ang_s))], pts[int(np.argmax(ang_r))]
            if np.max(ang_s) <= 0 or np.max(ang_r) <= 0:
                continue
            path = math.hypot(*(vs - s[:2])) + math.hypot(*(vr - vs)) + math.hypot(*(r[:2] - vr))
            z = path - d2
            ev = math.hypot(*(vr - vs))
            dz = ac.barrier_attenuation(z, double=ev > 0.5, e=ev)
            # the longer way round also spreads further
            T += 10 ** (-(dz + 20 * math.log10(path / max(d2, 1e-9))) / 10)
        att = -10 * np.log10(np.maximum(T, 10 ** -2.5))
        return np.minimum(att, 25.0), len(bids)

    def blocked(self, s, r):
        return len(self.hits(s, r)[0]) > 0

    # -- facades near the listener --------------------------------------------------------------------------------
    def facades_near(self, x, y, radius=40.0, k=8, min_len=3.0):
        """Up to k facade edges within `radius` of (x, y) facing it, nearest first: (indices, distances)."""
        if not len(self.seg):
            return np.zeros(0, int), np.zeros(0)
        c = self.cell
        r = int(math.ceil(radius / c))
        gx, gy = int(math.floor(x / c)), int(math.floor(y / c))
        ks = [self.grid[(i, j)] for i in range(gx - r, gx + r + 1) for j in range(gy - r, gy + r + 1)
              if (i, j) in self.grid]
        if not ks:
            return np.zeros(0, int), np.zeros(0)
        e = np.unique(np.concatenate(ks))
        e = e[self.length[e] >= min_len]
        P = self.seg[e]
        a, b = P[:, :2], P[:, 2:]
        ab = b - a
        tt = np.clip(np.einsum("ij,ij->i", np.array([x, y]) - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-9), 0, 1)
        q = a + ab * tt[:, None]
        dist = np.hypot(q[:, 0] - x, q[:, 1] - y)
        facing = np.einsum("ij,ij->i", np.array([x, y]) - q, self.normal[e]) > 0
        ok = facing & (dist <= radius)
        e, dist = e[ok], dist[ok]
        order = np.argsort(dist)[:k]
        return e[order], dist[order]

    def reflections(self, s, r, edges, n=2, check=True):
        """First-order image sources of s off the given facade edges, as heard at r: [(path length, reflection point
        (x, y), edge)] of the n shortest valid ones (the specular point on the edge, both on its outer side, legs
        clear of other buildings when check)."""
        if not len(edges):
            return []
        P = self.seg[edges]
        a = P[:, :2]
        nrm = self.normal[edges]
        sp = np.array(s[:2], float)
        rp = np.array(r[:2], float)
        ds = np.einsum("ij,ij->i", sp - a, nrm)        # source in front of the facade (> 0)
        dr = np.einsum("ij,ij->i", rp - a, nrm)
        img = sp - 2.0 * ds[:, None] * nrm
        # where img -> r crosses the facade line
        with np.errstate(divide="ignore", invalid="ignore"):
            k = ds / (ds + dr)
        hit = img + (rp - img) * k[:, None]
        ab = P[:, 2:] - a
        u = np.einsum("ij,ij->i", hit - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-9)
        ok = (ds > 0.2) & (dr > 0.2) & (u > 0.02) & (u < 0.98)
        L = np.hypot(rp[0] - img[:, 0], rp[1] - img[:, 1])
        out = []
        for j in np.argsort(np.where(ok, L, np.inf)):
            if not ok[j] or len(out) >= n:
                break
            h = hit[j]
            if check:
                # each leg must be clear; nudge the reflection point off the wall so the wall itself is not hit
                hp = h + nrm[j] * 0.05
                if self.blocked((sp[0], sp[1]), hp) or self.blocked(hp, (rp[0], rp[1])):
                    continue
            out.append((float(math.hypot(L[j], s[2] - r[2])), (float(h[0]), float(h[1])), int(edges[j])))
        return out

    def canyon(self, x, y, heading_rad, reach=60.0):
        """(width m, height m, sides 0..2) of the street around (x, y): rays to the left and right of the street
        direction (the nearest facade's direction if any, else the heading) up to `reach`."""
        e, dist = self.facades_near(x, y, radius=reach, k=1)
        if len(e):
            d = self.seg[e[0], 2:] - self.seg[e[0], :2]
            ang = math.atan2(d[1], d[0])
        else:
            ang = heading_rad
        nx, ny = -math.sin(ang), math.cos(ang)
        widths, heights = [], []
        for side in (1.0, -1.0):
            tx, ty = x + side * nx * reach, y + side * ny * reach
            t, ed = self.hits((x, y), (tx, ty))
            if len(t):
                widths.append(float(t[0]) * reach)
                heights.append(float(self.tops[self.bid[ed[0]]]))
        if not widths:
            return None, 0.0, 0
        w = sum(widths) if len(widths) == 2 else 2.0 * widths[0] + 20.0
        return max(4.0, w), float(np.mean(heights)), len(widths)


def canyon_reverb(width, height, sides, alpha_facade=0.1, alpha_ground=0.03):
    """(RT60 per octave band in s, reverberant level re the source power in dB) of a street canyon: Sabine on a
    segment three widths long whose open top and ends absorb fully, with air absorption (ISO 9613-1 at 15 C, 70 %).
    The diffuse-field level 10 lg(4/R) is lowered by 8 dB: an open canyon is far from diffuse and the first
    reflections are rendered on their own (Kang 2000, JASA 107(3): street reverberation is short-lived and
    non-diffuse). Open space (no facades): a short weak tail."""
    if not width or sides == 0:
        return np.full(ac.NB, 0.35), -40.0
    W, H = float(width), max(3.0, float(height))
    Lseg = 3.0 * W
    V = W * H * Lseg
    fac = (2.0 if sides == 2 else 1.0) * H * Lseg
    S = fac + W * Lseg + W * Lseg + 2 * W * H + (0.0 if sides == 2 else H * Lseg)
    A = fac * alpha_facade + W * Lseg * alpha_ground + W * Lseg + 2 * W * H + (0.0 if sides == 2 else H * Lseg)
    m = ac.air_absorption(15.0, 70.0) / (10 * math.log10(math.e))       # power attenuation per m
    rt = 0.161 * V / (A + 4 * m * V)
    abar = A / S
    R = A / max(1e-3, 1.0 - abar)
    lev = 10 * math.log10(4.0 / R) - 8.0
    return np.clip(rt, 0.2, 2.5), lev
