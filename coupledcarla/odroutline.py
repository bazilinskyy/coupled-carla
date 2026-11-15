"""Lane outlines of an OpenDRIVE file as polygons in its own x, y (metres, x east, y north).

For the photorealistic tiles (photoreal.py): the photogrammetry's road surface is cut out where the twin's roads are,
so CARLA's roads (lane markings, a smooth surface) show through. Implements what that needs of OpenDRIVE 1.4-1.8: the
reference line (line, arc, spiral, poly3, paramPoly3 with arcLength or normalized ranges), laneOffset, lane sections
and lane width polynomials. Lane heights are not used (the cut is a vertical prism). `border` records and lane
heights are ignored; a lane without width records counts as 0 m wide.
"""
import math
import xml.etree.ElementTree as ET

# lane types cut out of the photo mesh by `carve` (photoreal.DEFAULTS): the carriageway is everything a vehicle uses;
# "road" adds the sidewalks (the twin's own pavement shows instead of the photo's)
CARRIAGEWAY = {"driving", "parking", "biking", "bidirectional", "bus", "taxi", "hov", "shoulder", "restricted",
               "entry", "exit", "onRamp", "offRamp", "connectingRamp", "stop", "special1", "special2", "special3",
               "roadWorks", "tram", "rail", "slipLane"}
SIDEWALK = {"sidewalk", "walking", "curb"}


def _poly(c, ds):
    a, b, cc, d = c
    return a + ds * (b + ds * (cc + ds * d))


def _records(parent, tag, keys=("a", "b", "c", "d"), s_key="s"):
    """Sorted (s, (a, b, c, d)) records of child elements `tag`."""
    out = []
    for e in parent.findall(tag):
        out.append((float(e.get(s_key, 0.0)), tuple(float(e.get(k, 0.0)) for k in keys)))
    out.sort(key=lambda r: r[0])
    return out


def _eval_records(recs, s):
    """The polynomial record in force at s (the last one starting at or before s), evaluated."""
    if not recs:
        return 0.0
    cur = recs[0]
    for r in recs:
        if r[0] <= s + 1e-9:
            cur = r
        else:
            break
    return _poly(cur[1], s - cur[0])


class Geometry:
    """One planView geometry record."""

    def __init__(self, e):
        self.s = float(e.get("s"))
        self.x = float(e.get("x"))
        self.y = float(e.get("y"))
        self.hdg = float(e.get("hdg"))
        self.length = float(e.get("length"))
        child = next(iter(e), None)
        self.kind = child.tag if child is not None else "line"
        a = {k: float(v) for k, v in (child.attrib.items() if child is not None else ()) if k != "pRange"}
        self.p = a
        self.prange = child.get("pRange", "normalized") if child is not None else None

    def at(self, ds):
        """x, y, heading at distance ds (0..length) along this record."""
        ds = min(max(ds, 0.0), self.length)
        k = self.kind
        if k == "line":
            return self.x + ds * math.cos(self.hdg), self.y + ds * math.sin(self.hdg), self.hdg
        if k == "arc":
            c = self.p.get("curvature", 0.0)
            if abs(c) < 1e-12:
                return self.x + ds * math.cos(self.hdg), self.y + ds * math.sin(self.hdg), self.hdg
            h = self.hdg + c * ds
            return (self.x + (math.sin(h) - math.sin(self.hdg)) / c,
                    self.y - (math.cos(h) - math.cos(self.hdg)) / c, h)
        if k == "spiral":
            return self._spiral(ds)
        if k in ("poly3", "paramPoly3"):
            return self._param(ds)
        raise ValueError(f"unknown OpenDRIVE geometry {k}")

    def _spiral(self, ds):
        """Curvature linear from curvStart to curvEnd: integrate the heading (Simpson, <= 0.25 m steps)."""
        c0, c1 = self.p.get("curvStart", 0.0), self.p.get("curvEnd", 0.0)
        dc = (c1 - c0) / self.length if self.length > 0 else 0.0

        def heading(t):
            return self.hdg + c0 * t + 0.5 * dc * t * t

        n = max(2, int(math.ceil(ds / 0.25)))
        n += n % 2
        h = ds / n
        sx = sy = 0.0
        for i in range(n + 1):
            w = 1 if i in (0, n) else (4 if i % 2 else 2)
            th = heading(i * h)
            sx += w * math.cos(th)
            sy += w * math.sin(th)
        return self.x + sx * h / 3.0, self.y + sy * h / 3.0, heading(ds)

    def _param(self, ds):
        g = self.p
        if self.kind == "poly3":
            # v(u) = a + b u + c u^2 + d u^3 along the local u axis; u found from arc length by stepping
            u = self._u_for_arclength(ds, lambda t: (t, _poly((g.get("a", 0), g.get("b", 0), g.get("c", 0),
                                                                 g.get("d", 0)), t)))
            lu, lv = u, _poly((g.get("a", 0), g.get("b", 0), g.get("c", 0), g.get("d", 0)), u)
            du, dv = 1.0, g.get("b", 0) + 2 * g.get("c", 0) * u + 3 * g.get("d", 0) * u * u
        else:
            cu = (g.get("aU", 0), g.get("bU", 0), g.get("cU", 0), g.get("dU", 0))
            cv = (g.get("aV", 0), g.get("bV", 0), g.get("cV", 0), g.get("dV", 0))
            p = ds if self.prange == "arcLength" else (ds / self.length if self.length > 0 else 0.0)
            lu, lv = _poly(cu, p), _poly(cv, p)
            du = cu[1] + 2 * cu[2] * p + 3 * cu[3] * p * p
            dv = cv[1] + 2 * cv[2] * p + 3 * cv[3] * p * p
        ch, sh = math.cos(self.hdg), math.sin(self.hdg)
        x = self.x + lu * ch - lv * sh
        y = self.y + lu * sh + lv * ch
        h = self.hdg + (math.atan2(dv, du) if (du or dv) else 0.0)
        return x, y, h

    def _u_for_arclength(self, ds, f, step=0.25):
        if ds <= 0:
            return 0.0
        acc, t = 0.0, 0.0
        px, py = f(0.0)
        while acc < ds and t < 10 * self.length + 1:
            t2 = t + step
            x, y = f(t2)
            seg = math.hypot(x - px, y - py)
            if acc + seg >= ds:
                return t + step * (ds - acc) / seg if seg > 0 else t2
            acc += seg
            t, px, py = t2, x, y
        return t


class Road:
    def __init__(self, e):
        self.id = e.get("id")
        self.junction = e.get("junction", "-1")
        self.length = float(e.get("length"))
        self.geoms = sorted((Geometry(g) for g in e.findall("planView/geometry")), key=lambda g: g.s)
        lanes = e.find("lanes")
        self.offsets = _records(lanes, "laneOffset") if lanes is not None else []
        self.sections = []
        secs = sorted(lanes.findall("laneSection") if lanes is not None else [], key=lambda x: float(x.get("s", 0)))
        for i, sec in enumerate(secs):
            s0 = float(sec.get("s", 0.0))
            s1 = float(secs[i + 1].get("s")) if i + 1 < len(secs) else self.length
            sides = {}
            for side, sign in (("left", 1), ("right", -1)):
                lanes_ = []
                for ln in sec.findall(f"{side}/lane"):
                    lid = int(ln.get("id"))
                    lanes_.append((abs(lid), ln.get("type", "none"),
                                   _records(ln, "width", s_key="sOffset")))
                lanes_.sort(key=lambda t: t[0])   # from the centre outwards
                sides[sign] = lanes_
            self.sections.append((s0, s1, sides))

    def ref(self, s):
        """x, y, heading of the reference line at s."""
        g = self.geoms[0]
        for cand in self.geoms:
            if cand.s <= s + 1e-9:
                g = cand
            else:
                break
        return g.at(s - g.s)


def parse(xodr):
    """(roads, geoReference string) of an OpenDRIVE file path or its text."""
    text = xodr
    if not str(xodr).lstrip().startswith("<"):
        with open(xodr, encoding="utf-8") as f:
            text = f.read()
    root = ET.fromstring(text.encode("utf-8") if isinstance(text, str) else text)
    geo = root.find("header/geoReference")
    georef = (geo.text or "").strip() if geo is not None else ""
    return [Road(r) for r in root.findall("road") if r.find("planView/geometry") is not None], georef


def _samples(s0, s1, step):
    n = max(1, int(math.ceil((s1 - s0) / step)))
    return [s0 + (s1 - s0) * i / n for i in range(n + 1)]


def lane_outlines(road, types, step=1.0):
    """Polygons (lists of (x, y), open rings) of each run of neighbouring lanes whose type is in `types`, per lane
    section and side. Lanes of other types between them split the run."""
    out = []
    for s0, s1, sides in road.sections:
        ss = _samples(s0, s1, step)
        for sign, lanes in sides.items():
            if not lanes:
                continue
            # border offsets (from the reference line, signed: + left) at each s, lane by lane outwards
            runs, cur = [], None
            for idx, (_, typ, _) in enumerate(lanes):
                if typ in types:
                    if cur is None:
                        cur = [idx, idx]
                    else:
                        cur[1] = idx
                else:
                    if cur is not None:
                        runs.append(cur)
                    cur = None
            if cur is not None:
                runs.append(cur)
            if not runs:
                continue
            borders = []    # per s: [t_0 (inner of lane 0), t_1, ...]
            for s in ss:
                t = _eval_records(road.offsets, s)
                ts = [t]
                for _, _, widths in lanes:
                    w = max(0.0, _eval_records(widths, s - s0))
                    t += sign * w
                    ts.append(t)
                borders.append(ts)
            refs = [road.ref(s) for s in ss]
            for a, b in runs:
                inner, outer = [], []
                for (x, y, h), ts in zip(refs, borders):
                    nx, ny = -math.sin(h), math.cos(h)   # left normal
                    ti, to = ts[a], ts[b + 1]
                    inner.append((x + nx * ti, y + ny * ti))
                    outer.append((x + nx * to, y + ny * to))
                if max(abs(ts[b + 1] - ts[a]) for ts in borders) < 0.05:
                    continue    # zero-width (a lane that only exists on paper here)
                out.append(inner + outer[::-1])
    return out


def outlines(xodr, types, step=1.0):
    """All lane-run polygons of the file for lane `types`, with the geoReference string."""
    roads, georef = parse(xodr)
    polys = []
    for r in roads:
        polys.extend(lane_outlines(r, types, step))
    return polys, georef
