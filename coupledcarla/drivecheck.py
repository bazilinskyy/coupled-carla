"""Measures of how realistic the driving is, from SUMO's outputs (tools/traffic_check.py) and a trial's frames.csv
(tools/drive_check.py). Plain Python: no SUMO binary, no CARLA, so the measures are unit-tested.

SUMO side (per vehicle class): accelerations and jerk from the FCD (10 Hz), time gaps to the leader (gap / speed,
moving traffic only), free-flow speed / limit, vehicles standing inside junctions, red-light entries (a vehicle
entering a junction's internal lane over a link that shows red), plus SUMO's own counts (teleports, collisions,
emergency braking) from its statistic output.

CARLA side: oriented bounding boxes of the vehicles logged in frames.csv every frame: vehicle-vehicle overlaps,
vehicle-walker contacts, and the participants' own accelerations, jerk and events (stuck, reset_to_lane, collisions).
"""
import bisect
import math
import xml.etree.ElementTree as ET


# -- small statistics ---------------------------------------------------------------------------------------------

def pct(values, p):
    """The p-th percentile (0-100) of values by linear interpolation, None for no values."""
    v = sorted(values)
    if not v:
        return None
    k = (len(v) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def share(values, test):
    v = list(values)
    return sum(1 for x in v if test(x)) / len(v) if v else None


def summary(values, digits=2):
    v = list(values)
    if not v:
        return {"n": 0}
    r = lambda x: round(x, digits) if x is not None else None   # noqa: E731
    return {"n": len(v), "mean": r(sum(v) / len(v)), "p50": r(pct(v, 50)), "p95": r(pct(v, 95)), "max": r(max(v))}


# -- SUMO FCD ---------------------------------------------------------------------------------------------------------

class FcdStats:
    """Accumulates FCD records (one dict per vehicle per sample) into the measures above.

    lane_speed: lane id -> speed limit (m/s); kind_of(type id) -> "car" | "bus" | "bike". A sample counts for the
    time gap when the vehicle moves faster than 5 m/s with a leader within 60 m (car following, not queues), and for
    free-flow speed when it has no leader within 80 m on a normal lane, moving.
    """

    def __init__(self, lane_speed, kind_of, period=0.1):
        self.lane_speed, self.kind_of, self.period = lane_speed, kind_of, period
        self.acc, self.jerk, self.gap, self.free, self.free_mps = {}, {}, {}, {}, {}
        self.last = {}                  # vehicle -> (time, accel)
        self.junction_stop = {}         # vehicle -> seconds standing on an internal lane in a row
        self.junction_waits = []        # completed stops inside junctions longer than 5 s (s)
        self.samples = 0

    def add(self, t, rec):
        vid, kind = rec["id"], self.kind_of(rec.get("type", ""))
        speed, lane = float(rec.get("speed", 0.0)), rec.get("lane", "")
        self.samples += 1
        a = rec.get("acceleration")
        if a is not None:
            a = float(a)
            self.acc.setdefault(kind, []).append(a)
            prev = self.last.get(vid)
            if prev is not None and abs(t - prev[0] - self.period) < 1e-6:
                self.jerk.setdefault(kind, []).append((a - prev[1]) / self.period)
            self.last[vid] = (t, a)
        gap, leader = rec.get("leaderGap"), rec.get("leaderID")
        has_leader = leader not in (None, "") and gap not in (None, "") and float(gap) >= 0
        if has_leader and speed > 5.0 and float(gap) < 60.0:
            self.gap.setdefault(kind, []).append(float(gap) / speed)
        limit = self.lane_speed.get(lane)
        if limit and not lane.startswith(":") and speed > 1.0 and (not has_leader or float(gap) > 80.0):
            self.free.setdefault(kind, []).append(speed / limit)
            self.free_mps.setdefault(kind, []).append(speed)
        if lane.startswith(":") and speed < 0.1:
            self.junction_stop[vid] = self.junction_stop.get(vid, 0.0) + self.period
        elif vid in self.junction_stop:
            w = self.junction_stop.pop(vid)
            if w > 5.0:
                self.junction_waits.append(w)

    def result(self):
        out = {}
        for kind in sorted(set(self.acc) | set(self.gap) | set(self.free)):
            acc = self.acc.get(kind, [])
            jerk = [abs(j) for j in self.jerk.get(kind, [])]
            gaps = self.gap.get(kind, [])
            free = self.free.get(kind, [])
            out[kind] = {
                "abs_accel": summary(abs(a) for a in acc),
                "decel_over_4.5_share": round(share(acc, lambda a: a < -4.5) or 0.0, 5),
                "decel_over_7_share": round(share(acc, lambda a: a < -7.0) or 0.0, 5),
                "abs_jerk": summary(jerk),
                "jerk_over_10_share": round(share(jerk, lambda j: j > 10.0) or 0.0, 5),
                "time_gap_s": summary(gaps),
                "time_gap_under_1s_share": round(share(gaps, lambda g: g < 1.0) or 0.0, 4),
                "time_gap_under_0.5s_share": round(share(gaps, lambda g: g < 0.5) or 0.0, 4),
                "free_speed_over_limit": summary(free, 3),
                "free_speed_mps": summary(self.free_mps.get(kind, []), 2),
                "free_speed_over_110pct_share": round(share(free, lambda r: r > 1.1) or 0.0, 4),
            }
        waits = self.junction_waits + [w for w in self.junction_stop.values() if w > 5.0]
        out["junction_stops_over_5s"] = len(waits)
        out["junction_stop_s"] = summary(waits, 1)
        return out


def iter_fcd(path):
    """(time, record dict) for each vehicle of each timestep of a SUMO FCD file (plain or .gz), streaming."""
    import gzip
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rb") as f:
        t = None
        for ev, el in ET.iterparse(f, events=("start", "end")):
            if ev == "start" and el.tag == "timestep":
                t = float(el.get("time"))
            elif ev == "end" and el.tag == "vehicle":
                yield t, dict(el.attrib)
                el.clear()
            elif ev == "end" and el.tag == "timestep":
                el.clear()


def sumo_statistics(path):
    """SUMO's --statistic-output as a flat dict: teleports (total, jam, yield, wrongLane), safety (collisions,
    emergencyStops, emergencyBraking), vehicles (loaded, inserted, running, waiting), trip means."""
    root = ET.parse(path).getroot()
    out = {}
    for tag, prefix in (("teleports", "teleports"), ("safety", ""), ("vehicles", "vehicles"),
                        ("vehicleTripStatistics", "trip")):
        el = root.find(tag)
        if el is None:
            continue
        for k, v in el.attrib.items():
            key = (f"{prefix}_{k}" if prefix and k != "total" else (prefix or k)) if prefix else k
            try:
                out[key] = float(v) if "." in v else int(v)
            except ValueError:
                out[key] = v
    return out


def collisions(path):
    """Counts of SUMO's --collision-output by type (collision, frontal, side, junction, ...) and by vehicle pair
    class, and the list of (time, collider, victim, type)."""
    rows = []
    for el in ET.parse(path).getroot().iter("collision"):
        rows.append((float(el.get("time")), el.get("collider"), el.get("victim"), el.get("type", "")))
    kinds = {}
    for r in rows:
        kinds[r[3]] = kinds.get(r[3], 0) + 1
    return {"n": len(rows), "by_type": kinds, "rows": rows[:50]}


def link_state_on_entry(links, internal_lane):
    """The state character of the link from a normal lane into internal_lane, from traci.lane.getLinks(lane,
    extended=True) tuples (approached lane, hasPrio, isOpen, hasFoe, approached internal lane, state, direction,
    length); None when no link leads there."""
    for link in links:
        if len(link) > 5 and link[4] == internal_lane:
            return link[5]
    return None


TLS_STATES = set("GgrRyYuoO")      # link states of signalised links (priority junctions: M, m, =, s, w)


def is_red(state):
    return state in ("r", "R")


# -- CARLA frames.csv ---------------------------------------------------------------------------------------------

def obb_corners(x, y, yaw_deg, hx, hy):
    """Corners of a box centred at x, y with half lengths hx (forward) and hy, turned by yaw (CARLA degrees)."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def obb_overlap(a, b, shrink=0.0):
    """Whether two oriented boxes (x, y, yaw_deg, hx, hy) overlap (separating axis test), each first shrunk by
    `shrink` metres on every side (a tolerance for bounding boxes that are larger than the body)."""
    pa = obb_corners(a[0], a[1], a[2], max(0.0, a[3] - shrink), max(0.0, a[4] - shrink))
    pb = obb_corners(b[0], b[1], b[2], max(0.0, b[3] - shrink), max(0.0, b[4] - shrink))
    for yaw in (a[2], a[2] + 90.0, b[2], b[2] + 90.0):
        ux, uy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        da = [px * ux + py * uy for px, py in pa]
        db = [px * ux + py * uy for px, py in pb]
        if max(da) < min(db) or max(db) < min(da):
            return False
    return True


def point_in_obb(px, py, box, margin=0.0):
    """Whether a point lies within an oriented box (x, y, yaw_deg, hx, hy) grown by margin."""
    x, y, yaw, hx, hy = box
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    dx, dy = px - x, py - y
    f, l = dx * c + dy * s, -dx * s + dy * c
    return abs(f) <= hx + margin and abs(l) <= hy + margin


class Grid:
    """A 2D hash of boxes by cell, for overlap checks among a few hundred actors per frame."""

    def __init__(self, cell=10.0):
        self.cell, self.cells = cell, {}

    def add(self, key, x, y):
        self.cells.setdefault((int(x // self.cell), int(y // self.cell)), []).append(key)

    def near(self, x, y):
        cx, cy = int(x // self.cell), int(y // self.cell)
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                yield from self.cells.get((cx + i, cy + j), ())


def frame_conflicts(vehicles, walkers, shrink=0.15, walker_margin=0.1):
    """Overlapping vehicle pairs and vehicle-walker contacts in one frame. vehicles: {id: (x, y, yaw, hx, hy)},
    walkers: {id: (x, y)}. Returns (set of (id, id) vehicle pairs, set of (vehicle, walker))."""
    g = Grid()
    for vid, b in vehicles.items():
        g.add(vid, b[0], b[1])
    pairs, hits = set(), set()
    for vid, b in vehicles.items():
        for o in g.near(b[0], b[1]):
            if str(o) <= str(vid):          # each pair once, never with itself
                continue
            ob = vehicles[o]
            if (b[0] - ob[0]) ** 2 + (b[1] - ob[1]) ** 2 > (b[3] + ob[3] + 1.0) ** 2:
                continue
            if obb_overlap(b, ob, shrink):
                pairs.add((vid, o))
    for wid, (x, y) in walkers.items():
        for vid in g.near(x, y):
            if point_in_obb(x, y, vehicles[vid], walker_margin + 0.25):     # a walker's radius is ~0.25 m
                hits.add((vid, wid))
    return pairs, hits


def episodes(frames_with_keys, gap_frames=25):
    """Number of distinct episodes per key: a key seen in frames that are at most gap_frames apart counts once.
    frames_with_keys: iterable of (frame, key)."""
    last, n = {}, {}
    for frame, key in sorted(frames_with_keys):
        if key not in last or frame - last[key] > gap_frames:
            n[key] = n.get(key, 0) + 1
        last[key] = frame
    return sum(n.values())


def series_jerk(times, speeds_xy):
    """|longitudinal accel| and |jerk| of one actor from its logged velocities (vx, vy) per time; the accel is the
    rate of change of speed, smoothed over 0.1 s, so 50 Hz logging noise does not dominate."""
    sp = [math.hypot(vx, vy) for vx, vy in speeds_xy]
    acc, jerk = [], []
    if len(times) < 3:
        return acc, jerk
    # resample on 0.1 s
    t0, t1 = times[0], times[-1]
    grid = [t0 + 0.1 * i for i in range(int((t1 - t0) / 0.1) + 1)]
    vs = []
    for tg in grid:
        k = min(max(1, bisect.bisect_left(times, tg)), len(times) - 1)
        ta, tb = times[k - 1], times[k]
        w = (tg - ta) / (tb - ta) if tb > ta else 0.0
        vs.append(sp[k - 1] + (sp[k] - sp[k - 1]) * w)
    acc = [(vs[i + 1] - vs[i]) / 0.1 for i in range(len(vs) - 1)]
    jerk = [(acc[i + 1] - acc[i]) / 0.1 for i in range(len(acc) - 1)]
    return [abs(a) for a in acc], [abs(j) for j in jerk]


# -- bodies and turns from the SUMO FCD (x, y, angle) ---------------------------------------------------------------

def fcd_box(rec, length, width):
    """(x, y, yaw, hx, hy) of a vehicle's body from an FCD record (front bumper x, y in the net; navigation angle:
    0 north, clockwise) in a frame with y pointing south like CARLA's (y flipped, yaw = angle - 90)."""
    yaw = float(rec["angle"]) - 90.0
    fx, fy = float(rec["x"]), -float(rec["y"])
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (fx - c * length / 2.0, fy - s * length / 2.0, yaw, length / 2.0, width / 2.0)


class BodyStats:
    """Side clearance of large vehicles and lateral accelerations, from FCD records with x, y, angle.

    add_step(t, recs) once per FCD time step; size_of(type id) -> (length, width, large) or None. A large vehicle's
    side clearance counts against bodies beside it (their extents along its axis overlap) while one of the two moves;
    lateral acceleration per vehicle from its body centre over 0.2 s (clipcheck.dynamics)."""

    def __init__(self, size_of, near_m=0.5):
        self.size_of, self.near_m = size_of, near_m
        self.min_clear = {}            # (kind pair) -> min clearance
        self.close = []                # (t, a, b, clearance) under near_m
        self.series = {}               # vehicle -> [(t, x, y, yaw)]

    def add_step(self, t, recs):
        boxes, big, moving = {}, [], {}
        for r in recs:
            sz = self.size_of(r.get("type", ""))
            if sz is None or "x" not in r or "angle" not in r:
                continue
            b = fcd_box(r, sz[0], sz[1])
            boxes[r["id"]] = b
            moving[r["id"]] = float(r.get("speed", 0.0)) > 0.5
            self.series.setdefault(r["id"], []).append((t, b[0], b[1], b[2]))
            if sz[2]:
                big.append(r["id"])
        if not big:
            return
        g = Grid(15.0)
        for vid, b in boxes.items():
            g.add(vid, b[0], b[1])
        from .clipcheck import box_clearance, side_by_side
        for a in big:
            ba = boxes[a]
            for o in g.near(ba[0], ba[1]):
                if o == a or not (moving[a] or moving[o]):
                    continue
                bo = boxes[o]
                if (bo[0] - ba[0]) ** 2 + (bo[1] - ba[1]) ** 2 > 225 or not side_by_side(ba, bo):
                    continue
                d = box_clearance(ba, bo)
                key = "large-large" if o in big else "large-other"
                if key not in self.min_clear or d < self.min_clear[key]:
                    self.min_clear[key] = d
                if d < self.near_m and (o not in big or a < o):
                    self.close.append((t, a, o, round(d, 2), key))

    def result(self, period=0.1):
        from .clipcheck import dynamics
        lat_max = []
        for s in self.series.values():
            runs, cur = [], []
            for p in s:
                if cur and p[0] - cur[-1][0] > 1.5 * period:
                    runs.append(cur)
                    cur = []
                cur.append(p)
            runs.append(cur)
            m = 0.0
            for run in runs:
                for d in dynamics(run):
                    if d[3] > 2.0:
                        m = max(m, d[1])
            lat_max.append(m)
        eps = {k: episodes([(round(c[0] / period), (c[1], c[2])) for c in self.close if c[4] == k], gap_frames=10)
               for k in ("large-large", "large-other")}
        return {"min_side_clearance_m": {k: round(v, 2) for k, v in self.min_clear.items()},
                "large_close_episodes": eps,
                "large_close_examples": [c for c in self.close if c[4] == "large-large"][:6] +
                                        [c for c in self.close if c[4] == "large-other"][:4],
                "vehicle_max_lat_acc": summary(lat_max), "vehicles_over_4_mps2": sum(1 for v in lat_max if v > 4.0)}



# -- targets: what real traffic shows ---------------------------------------------------------------------------------
# (path in traffic_check's summary, low, high, source). A path is keys joined by "/"; a None bound is open.
TARGETS = [
    ("fcd/car/abs_accel/p95", None, 2.5, "urban drivers' longitudinal accelerations: 95th percentile under ~2-2.5 "
     "m/s^2 (Bokare & Maurya 2017, Transp. Res. Procedia 25; Wang et al. 2004 naturalistic data)"),
    ("fcd/car/decel_over_4.5_share", None, 0.005, "hard braking (> 4.5 m/s^2) is rare in normal driving "
     "(< 0.5 % of time; naturalistic studies, e.g. Bagdadi & Varhelyi 2011)"),
    ("fcd/car/abs_jerk/p95", None, 2.5, "jerk: comfortable driving stays under ~2 m/s^3 (Bae et al. 2019; "
     "Huang & Wang 2004); 2.5 allows for SUMO's 0.1 s sampling"),
    ("fcd/car/time_gap_s/p50", 1.0, 2.2, "median time gap in urban car following 1-2 s (Treiber & Kesting 2013, "
     "table 11.2: T 1.0-1.5 s; observed medians 1.3-2 s)"),
    ("fcd/car/time_gap_under_0.5s_share", None, 0.05, "time gaps under 0.5 s: a few percent at most"),
    ("fcd/car/free_speed_over_limit/p50", 0.85, 1.08, "free-flow speed / limit in cities ~0.9-1.05 "
     "(speeds.py: PBOT spot speeds; NL enforcement)"),
    ("fcd/bike/free_speed_mps/p50", 4.2, 5.4, "Dutch cyclists 15-19 km/h (KiM Cycling Facts 2023; Fietsberaad: "
     "pedelecs 18.7 km/h)"),
    ("fcd/bodies/vehicle_max_lat_acc/p50", None, 4.0, "a vehicle's sharpest turn: lateral acceleration of human "
     "drivers mostly under 3-4 m/s^2 (Xu et al. 2015)"),
    ("fcd/bodies/large_close_episodes/large-large", None, 0, "buses never pass each other closer than 0.5 m"),
    ("sumo/teleports", None, 1, "no vehicle teleported (vanishes and reappears)"),
    ("collisions/n", None, 0, "SUMO collisions"),
    ("red_light_share_by_class/car", None, 0.01, "cars entering on red: rare (red-light cameras, EU studies "
     "~1 % of entries)"),
    ("red_light_share_by_class/bike", 0.1, 0.4, "Dutch cyclists crossing on red: ~24 % of adults "
     "(SWOV, observations in The Hague)"),
]


def _get(d, path):
    for k in path.split("/"):
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def against_targets(summary, targets=TARGETS):
    """[{metric, value, low, high, ok, source}] for the measures present in a traffic_check summary."""
    out = []
    for path, lo, hi, src in targets:
        v = _get(summary, path)
        if v is None:
            continue
        ok = (lo is None or v >= lo) and (hi is None or v <= hi)
        out.append({"metric": path, "value": v, "low": lo, "high": hi, "ok": ok, "source": src})
    return out


def targets_table(rows):
    lines = ["| Measure | Value | Target | OK | Source |", "|---|---|---|---|---|"]
    for r in rows:
        tgt = (f"{r['low']}-{r['high']}" if r["low"] is not None and r["high"] is not None else
               f">= {r['low']}" if r["low"] is not None else f"<= {r['high']}")
        lines.append(f"| {r['metric']} | {r['value']} | {tgt} | {'yes' if r['ok'] else 'NO'} | {r['source']} |")
    return "\n".join(lines)
