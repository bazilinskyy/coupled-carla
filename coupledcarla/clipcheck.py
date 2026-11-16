"""Clip QA: does a demo clip's trial show anything that looks wrong on screen? (tools/clip_check.py, make_videos.py)

Everything comes from the trial's logs, so it runs without CARLA or SUMO, and the measures are unit-tested:
frames.csv (every actor near the participants, every tick), the film camera's camera.csv (its pose every tick),
events.csv (spawns with their SUMO ids), sumo/sumo.log (teleports) and sumo/sumo_collisions.xml (SUMO's collisions,
written by trials since this module), and the map's buildings.json.

Checks, each reported with its times (trial time) and whether it falls into the camera window (start_s ..
start_s + duration_s) and into view (the film camera's frustum within VIEW_M, or within VIEW_M of a viewed
participant for station clips):
  overlap     two vehicle bodies overlap (oriented boxes from data/blueprint_sizes_*.json, shrunk by OVERLAP_SHRINK
              for boxes that are larger than the body; a bus's box with mirrors is capped at its body width) or a
              walker stands inside a vehicle
  building    a vehicle inside a building footprint (buildings.json)
  swerve      a participant's car, or a vehicle in view, with lateral acceleration beyond human driving
              (LAT_ACC m/s^2, from positions over 0.2 s at speed; a vehicle in view blocks from TRAFFIC_LAT_ACC) or a
              heading jump (HEADING_JUMP deg in 0.1 s)
  jump        an actor moving farther in one tick than its speed allows (JUMP_MARGIN m), or appearing / vanishing in
              view closer than POP_M (a SUMO teleport, a level-of-detail spawn in plain sight)
  teleport    SUMO teleported a vehicle ("waited too long", jam, wrong lane), from sumo.log
  sumo_collision  SUMO's own collision output
  jitter      the camera shakes: its pose minus a smooth fit over 7 frames (Savitzky-Golay, order 2) beyond
              JITTER_M in position or JITTER_DEG in yaw or pitch, at the clip's frame times
Metrics: the participants' max lateral acceleration and yaw rate in the window, the minimum clearance between a bus
and another vehicle body, the camera jitter (RMS and max residual), counts per check.

A problem blocks encoding (make_videos.py, unless --force) when it is in the window and, for overlap, building, jump
and teleport, in view; swerves of a participant and camera jitter always count; SUMO collisions are reported (their
visible part is an overlap).
"""
import bisect
import csv
import json
import math
import re
from pathlib import Path

from . import drivecheck

LAT_ACC = 4.0           # m/s^2: urban drivers stay under ~3-4 (e.g. Xu et al. 2015, comfort in curves): reported
TRAFFIC_LAT_ACC = 6.0   # m/s^2: a vehicle in view (not a participant) blocks a clip from here (a sharp turn shows)
BUILDING_VIEW_M = 50.0  # a vehicle inside a building blocks a clip when in view this close
HEADING_JUMP = 45.0     # deg within 0.1 s
SWERVE_MIN_SPEED = 2.0  # m/s: slower than this, turning on the spot is normal
JUMP_MARGIN = 0.3       # m beyond speed * dt in one tick
POP_M = 40.0            # appearing / vanishing closer than this in view
VIEW_M = 80.0           # what lies farther than this is not counted as seen
OVERLAP_SHRINK = 0.15
BODY_WIDTH = {"vehicle.mitsubishi.fusorosa": 2.55}     # boxes with mirrors -> body width (m); others: drivers.MAX_WIDTH
JITTER_M = 0.02         # camera position residual (m)
JITTER_DEG = 0.15       # camera yaw / pitch residual (deg)
# a drone's own camera (FPV, rigid on the frame) turns with the drone: the onset of a turn is a yaw acceleration of
# ~600 deg/s^2 for 0.1 s in the drone model, 0.19 deg off a 7-frame quadratic (Eindhoven drones clip, 9.6 s); that
# is the drone turning, not shaking, so drone cameras are held to 0.3 deg
JITTER_DEG_DRONE = 0.3
SG7 = (-2.0, 3.0, 6.0, 7.0, 6.0, 3.0, -2.0)            # Savitzky-Golay, 7 points, order 2 (/21)
BLOCKING = ("overlap", "building", "jump", "teleport")    # sumo_collision: reported; what of it shows is an overlap


# -- reading ----------------------------------------------------------------------------------------------------------

def read_frames(path, t0, t1, every=1):
    """{trial time: {actor id: row}} for t0 <= t <= t1 (rows: kind, role, type, x, y, z, yaw, pitch, vx, vy, speed),
    every n-th logged frame."""
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            t = float(r["timestamp"])
            if t < t0 or t > t1:
                continue
            if every > 1 and int(r["frame"]) % every:
                continue
            vx, vy = float(r["vel_x"] or 0), float(r["vel_y"] or 0)
            out.setdefault(round(t, 3), {})[r["actor_id"]] = {
                "kind": r["kind"], "role": r["role"], "type": r["type_id"], "x": float(r["pos_x"]),
                "y": float(r["pos_y"]), "z": float(r["pos_z"]), "yaw": float(r["rot_yaw"]),
                "speed": math.hypot(vx, vy), "vx": vx, "vy": vy}
    return out


def read_camera(path):
    """[(t, x, y, z, yaw, pitch)] from a film camera.csv."""
    with open(path, newline="") as f:
        return [tuple(float(r[k]) for k in ("t", "x", "y", "z", "yaw", "pitch"))
                for r in csv.DictReader(f, delimiter=";")]


def sumo_ids(events_csv):
    """{SUMO vehicle id: CARLA actor id} from the spawn events."""
    out = {}
    try:
        with open(events_csv, newline="") as f:
            for r in csv.DictReader(f, delimiter=";"):
                if r["event"] == "spawn" and "sumo:" in r["detail"]:
                    out[r["detail"].split("sumo:")[-1].strip()] = r["actor_id"]
    except OSError:
        pass
    return out


TELEPORT = re.compile(r"Teleporting vehicle '([^']+)'; ([^,]+), lane='([^']*)', time=([0-9]+(?:\.[0-9]+)?)")


def sumo_teleports(log_text):
    """[(time, vehicle, reason, lane)] from SUMO's log."""
    return [(float(m.group(4)), m.group(1), m.group(2), m.group(3)) for m in TELEPORT.finditer(log_text)]


def load_buildings(path):
    """[(xmin, ymin, xmax, ymax, polygon)] of buildings.json (CARLA x, y), [] when missing."""
    try:
        raw = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return []
    out = []
    for b in raw:
        p = b.get("p") if isinstance(b, dict) else None
        if not p or len(p) < 3:
            continue
        xs, ys = [q[0] for q in p], [q[1] for q in p]
        out.append((min(xs), min(ys), max(xs), max(ys), [(q[0], q[1]) for q in p]))
    return out


# -- geometry -----------------------------------------------------------------------------------------------------

def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L = ax * ax + ay * ay
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L))
    return math.hypot(p[0] - a[0] - u * ax, p[1] - a[1] - u * ay)


def box_clearance(a, b):
    """Distance between two oriented boxes (x, y, yaw, hx, hy); 0 when they overlap."""
    if drivecheck.obb_overlap(a, b):
        return 0.0
    pa, pb = drivecheck.obb_corners(*a), drivecheck.obb_corners(*b)
    d = min(_seg_dist(p, pb[i], pb[(i + 1) % 4]) for p in pa for i in range(4))
    return min(d, min(_seg_dist(p, pa[i], pa[(i + 1) % 4]) for p in pb for i in range(4)))


def side_by_side(a, b):
    """Whether box b lies beside box a (their extents along a's axis overlap): a lateral, not a nose-to-tail gap."""
    ux, uy = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    proj = [(px - a[0]) * ux + (py - a[1]) * uy for px, py in drivecheck.obb_corners(*b)]
    return min(proj) < a[3] and max(proj) > -a[3]


def in_view(cam, x, y, fov_deg, view_m=VIEW_M, margin_deg=10.0):
    """Whether (x, y) lies within the camera's horizontal field of view and view_m. cam: (t, x, y, z, yaw, pitch)."""
    dx, dy = x - cam[1], y - cam[2]
    d = math.hypot(dx, dy)
    if d > view_m:
        return False
    if d < 3.0:
        return True
    off = abs((math.degrees(math.atan2(dy, dx)) - cam[4] + 180.0) % 360.0 - 180.0)
    return off <= fov_deg / 2.0 + margin_deg


def savgol_residual(values):
    """values minus their 7-point Savitzky-Golay (order 2) fit; the three samples at each end are 0."""
    n = len(values)
    out = [0.0] * n
    for i in range(3, n - 3):
        fit = sum(c * values[i + k - 3] for k, c in enumerate(SG7)) / 21.0
        out[i] = values[i] - fit
    return out


def unwrap(degs):
    out, off = [], 0.0
    for i, a in enumerate(degs):
        if i:
            d = a + off - out[-1]
            off -= 360.0 * round(d / 360.0)
        out.append(a + off)
    return out


# -- checks -------------------------------------------------------------------------------------------------------

def box_of(row, sizes, max_width):
    s = sizes.get(row["type"])
    if not s:
        return None
    w = BODY_WIDTH.get(row["type"], min(s["width"], max_width.get(_vclass(row), s["width"])))
    # the box's centre lies centre_x in front of the actor's origin (measured with the sizes)
    cx = s.get("centre_x", 0.0)
    c, sn = math.cos(math.radians(row["yaw"])), math.sin(math.radians(row["yaw"]))
    return (row["x"] + c * cx, row["y"] + sn * cx, row["yaw"], s["length"] / 2.0, w / 2.0)


def _vclass(row):
    return {"bus": "bus", "bike": "bicycle"}.get(row["kind"], "passenger")


def is_large(row, sizes):
    s = sizes.get(row["type"])
    return row["kind"] == "bus" or bool(s and s["length"] > 7.0)


def dynamics(series):
    """Lateral acceleration (m/s^2), yaw rate (deg/s) and speed per sample of one actor: series [(t, x, y, yaw)] at
    an even step; velocity and acceleration from positions 0.1 s either side. [(t, lat_acc, yaw_rate, speed)]."""
    if len(series) < 3:
        return []
    dt = (series[-1][0] - series[0][0]) / (len(series) - 1) or 0.02
    k = max(1, int(round(0.1 / dt)))
    yaws = unwrap([s[3] for s in series])
    out = []
    for i in range(2 * k, len(series) - 2 * k):
        def vel(j):
            return ((series[j + k][1] - series[j - k][1]) / (2 * k * dt),
                    (series[j + k][2] - series[j - k][2]) / (2 * k * dt))
        v0, v1, v2 = vel(i - k), vel(i), vel(i + k)
        ax, ay = (v2[0] - v0[0]) / (2 * k * dt), (v2[1] - v0[1]) / (2 * k * dt)
        sp = math.hypot(*v1)
        lat = abs(v1[0] * ay - v1[1] * ax) / sp if sp > 0.5 else 0.0
        yr = (yaws[i + k] - yaws[i - k]) / (2 * k * dt)
        out.append((series[i][0], lat, yr, sp))
    return out


def _episodes(items, gap, severity=None):
    """[(t, key, detail)] -> [(t_first, t_last, key, detail)] grouped per key with gaps <= gap; the detail kept is
    the worst by severity(detail) (default: the first)."""
    out, open_ = [], {}
    for t, key, detail in sorted(items, key=lambda x: (str(x[1]), x[0])):
        cur = open_.get(key)
        if cur is not None and t - cur[1] <= gap:
            cur[1] = t
            if severity is not None and severity(detail) > severity(cur[3]):
                cur[3] = detail
        else:
            cur = open_[key] = [t, t, key, detail]
            out.append(cur)
    return [tuple(e) for e in sorted(out)]


class Report:
    def __init__(self, t0, t1):
        self.t0, self.t1 = t0, t1
        self.problems = []
        self.metrics = {}

    def add(self, check, t, t_end, what, detail="", visible=True, blocking=None):
        in_window = t_end >= self.t0 and t <= self.t1
        block = in_window and (visible if blocking is None else blocking)
        self.problems.append({"check": check, "t": round(t, 2), "t_end": round(t_end, 2), "what": what,
                              "detail": detail, "in_window": in_window, "in_view": bool(visible),
                              "blocking": bool(block)})

    def blocking(self):
        return [p for p in self.problems if p["blocking"]]

    def as_dict(self):
        counts = {}
        for p in self.problems:
            if p["in_window"]:
                counts[p["check"]] = counts.get(p["check"], 0) + 1
        return {"window": [self.t0, self.t1], "ok": not self.blocking(), "blocking": len(self.blocking()),
                "counts_in_window": counts, "metrics": self.metrics, "problems": self.problems}


def check_trial(trial, t0, t1, camera="main", fov=None, map_dir=None, viewers=(), sizes=None, every=2,
                margin_s=1.0):
    """The QA report (Report.as_dict()) of one trial's camera window t0..t1 (trial time).

    camera: the film camera's name (film/<camera>/camera.csv) or None (stations: viewers are the participants' roles
    whose surroundings are seen); fov: the camera's horizontal fov (default: from config.yaml); map_dir: for
    buildings.json (default: maps/<name> next to the trial's repo); every: check bodies on every n-th tick."""
    from . import drivers
    trial = Path(trial)
    sizes = sizes if sizes is not None else drivers.sizes()
    rep = Report(t0, t1)
    cfg = {}
    try:
        import yaml
        cfg = yaml.safe_load((trial / "config.yaml").read_text()) or {}
    except (OSError, ImportError, ValueError):
        pass
    step = float((cfg.get("simulation") or {}).get("step", 0.02))
    film = cfg.get("film") or {}
    if fov is None:
        cams = ([film["camera"]] if film.get("camera") else []) + list(film.get("cameras") or [])
        c0 = next((c for c in cams if c.get("name", "main") == camera), cams[0] if cams else {})
        fov = float(c0.get("fov", film.get("fov", 90.0)))
    eye_role = None        # the walker a first-person follow camera rides on (offset within 1.5 m)
    for c in ([film["camera"]] if film.get("camera") else []) + list(film.get("cameras") or []):
        off = c.get("offset") or [0, 0, 0]
        if c.get("name", "main") == camera and c.get("kind") == "follow" and \
                sum(float(v) ** 2 for v in off) ** 0.5 <= 1.5:
            eye_role = c.get("target")
    lo, hi = t0 - margin_s, t1 + margin_s
    frames = read_frames(trial / "frames.csv", lo, hi)
    times = sorted(frames)
    cam_rows = []
    cam_path = trial / "film" / (camera or "main") / "camera.csv"
    if camera and cam_path.exists():
        cam_rows = read_camera(cam_path)
    cam_t = [c[0] for c in cam_rows]

    def cam_at(t):
        if not cam_rows:
            return None
        i = min(max(0, bisect.bisect_left(cam_t, t)), len(cam_rows) - 1)
        return cam_rows[i] if abs(cam_rows[i][0] - t) < 0.1 else None

    def visible(t, x, y):
        c = cam_at(t)
        if c is not None:
            return in_view(c, x, y, fov)
        if camera and cam_rows:
            return False       # outside the camera's recorded span
        rows = frames.get(t) or {}
        eyes = [r for r in rows.values() if r["kind"] == "participant" and (not viewers or r["role"] in viewers)]
        return any(math.hypot(r["x"] - x, r["y"] - y) <= VIEW_M for r in eyes)

    # bodies: overlaps, walker contacts, bus clearance, buildings
    from . import inbuildings      # a twin's footprints with its passages under buildings cut out, else buildings.json
    fp = inbuildings.footprints_path(map_dir) if map_dir else None
    blds = load_buildings(fp) if fp else []
    bgrid = {}
    for i, b in enumerate(blds):
        for gx in range(int(b[0] // 20), int(b[2] // 20) + 1):
            for gy in range(int(b[1] // 20), int(b[3] // 20) + 1):
                bgrid.setdefault((gx, gy), []).append(i)
    over, contact, inside = [], [], []
    clear = {"min_m": None}
    tick = 0
    for t in times:
        tick += 1
        if every > 1 and tick % every:
            continue
        rows = frames[t]
        veh, walk = {}, {}
        for aid, r in rows.items():
            if r["type"].startswith("walker.") and r["z"] > -50:
                walk[aid] = (r["x"], r["y"])
            elif r["type"].startswith("vehicle.") and r["z"] > -10.0:      # a held car waits 30 m below the road
                b = box_of(r, sizes, drivers.MAX_WIDTH)
                if b:
                    veh[aid] = b
        pairs, hits = drivecheck.frame_conflicts(veh, walk, shrink=OVERLAP_SHRINK)
        for a, b in pairs:
            ra, rb = rows[a], rows[b]
            over.append((t, (a, b), f"{ra['kind']} {a} ({ra['type']}) and {rb['kind']} {b} ({rb['type']})"))
        for v, w in hits:
            contact.append((t, (v, w), f"walker {w} inside {rows[v]['kind']} {v} ({rows[v]['type']})"))
        for aid, r in rows.items():          # large vehicles: the closest body beside them
            if aid not in veh or not is_large(r, sizes):
                continue
            for o, ob in veh.items():
                if o == aid or o in walk:
                    continue
                if (ob[0] - veh[aid][0]) ** 2 + (ob[1] - veh[aid][1]) ** 2 > 225:
                    continue
                if max(r["speed"], rows[o]["speed"]) < 0.5 or not side_by_side(veh[aid], ob):
                    continue        # standing still, or nose to tail in a queue: not a lateral gap
                d = box_clearance(veh[aid], ob)
                if t0 <= t <= t1 and (clear["min_m"] is None or d < clear["min_m"]):
                    clear.update(min_m=round(d, 2), t=t, pair=[aid, o], kinds=[r["kind"], rows[o]["kind"]])
        if bgrid:
            for aid, b in veh.items():
                cs = drivecheck.obb_corners(b[0], b[1], b[2], max(0.0, b[3] - 0.3), max(0.0, b[4] - 0.3))
                for gi in bgrid.get((int(b[0] // 20), int(b[1] // 20)), ()):
                    bx = blds[gi]
                    if not (bx[0] <= b[0] <= bx[2] and bx[1] <= b[1] <= bx[3]):
                        continue
                    if sum(point_in_polygon(px, py, bx[4]) for px, py in cs) >= 2:
                        inside.append((t, aid, f"{rows[aid]['kind']} {aid} ({rows[aid]['type']}) inside a building"))
                        break
    gap = 0.5
    for t, t_e, key, what in _episodes(over, gap):
        r = frames[t][key[0]]
        rep.add("overlap", t, t_e, what, visible=visible(t, r["x"], r["y"]))
    for t, t_e, key, what in _episodes(contact, gap):
        r = frames[t][key[0]]
        rep.add("overlap", t, t_e, what, visible=visible(t, r["x"], r["y"]))
    for t, t_e, key, what in _episodes(inside, gap):
        r = frames[t][key]
        c = cam_at(t)
        near = c is None or math.hypot(r["x"] - c[1], r["y"] - c[2]) <= BUILDING_VIEW_M
        rep.add("building", t, t_e, what, visible=visible(t, r["x"], r["y"]) and near)
    rep.metrics["large_vehicle_min_clearance"] = clear

    # motion: swerves, heading and position jumps, pop-ins
    series = {}
    for t in times:
        for aid, r in frames[t].items():
            series.setdefault(aid, []).append((t, r))
    swerves, jumps, pops = [], [], []
    parts = {}
    for aid, s in series.items():
        r0 = s[0][1]
        is_part = r0["kind"] in ("participant", "agent") and r0["type"].startswith("vehicle.")
        if r0["type"].startswith("vehicle."):
            pts = [(t, r["x"], r["y"], r["yaw"]) for t, r in s if r["z"] > -10.0]
            # one contiguous run per actor (gaps: out of logging range, or held)
            runs, cur = [], []
            for p in pts:
                if cur and p[0] - cur[-1][0] > 1.5 * step:
                    runs.append(cur)
                    cur = []
                cur.append(p)
            if cur:
                runs.append(cur)
            for run in runs:
                dyn = dynamics(run)
                if is_part:
                    w = [d for d in dyn if t0 <= d[0] <= t1 and d[3] > SWERVE_MIN_SPEED]
                    pm = parts.setdefault(r0["role"] or aid, {"max_lat_acc": 0.0, "max_yaw_rate": 0.0})
                    for d in w:
                        if d[1] > pm["max_lat_acc"]:
                            pm.update(max_lat_acc=round(d[1], 2), t_lat=d[0])
                        if abs(d[2]) > pm["max_yaw_rate"]:
                            pm.update(max_yaw_rate=round(abs(d[2]), 1), t_yaw=d[0])
                for d in dyn:
                    if d[3] > SWERVE_MIN_SPEED and d[1] > LAT_ACC:
                        x, y = _xy_at(s, d[0])
                        swerves.append((d[0], aid, (is_part, x, y, d[1], d[3])))
                k = max(1, int(round(0.1 / step)))
                ys = unwrap([p[3] for p in run])
                for i in range(k, len(run)):
                    if abs(ys[i] - ys[i - k]) > HEADING_JUMP and _speed_at(s, run[i][0]) > 0.5:
                        swerves.append((run[i][0], aid, (is_part, run[i][1], run[i][2], None, ys[i] - ys[i - k])))
        # position jumps (vehicles and walkers); a first-person camera's own walker is not seen: what its jumps do
        # to the picture is the camera's jitter
        if r0["role"] and r0["role"] == eye_role:
            continue
        for (ta, ra), (tb, rb) in zip(s, s[1:]):
            dt = tb - ta
            if dt > 1.5 * step or ra["z"] < -10.0 or rb["z"] < -10.0:
                continue
            d = math.hypot(rb["x"] - ra["x"], rb["y"] - ra["y"])
            allowed = max(ra["speed"], rb["speed"]) * dt + JUMP_MARGIN
            if d > allowed:
                jumps.append((tb, aid, (rb["x"], rb["y"], d, allowed - JUMP_MARGIN, r0["kind"], r0["type"])))
        # appearing / vanishing in view
        if r0["kind"] in ("participant",):
            continue
        first, last = s[0], s[-1]
        if first[0] > lo + step and math.hypot(first[1]["x"] - _cam_xy(cam_at(first[0]), frames, first[0])[0],
                                                first[1]["y"] - _cam_xy(cam_at(first[0]), frames, first[0])[1]) < POP_M:
            pops.append((first[0], aid, ("appears", first[1])))
        if last[0] < times[-1] - step and math.hypot(last[1]["x"] - _cam_xy(cam_at(last[0]), frames, last[0])[0],
                                                      last[1]["y"] - _cam_xy(cam_at(last[0]), frames, last[0])[1]) < POP_M:
            pops.append((last[0], aid, ("vanishes", last[1])))
    for t, t_e, aid, (is_part, x, y, lat, extra) in _episodes(swerves, 0.5, lambda d: 99.0 if d[3] is None else d[3]):
        r = series[aid][0][1]
        what = (f"{r['role'] or r['kind']} {aid} ({r['type']}): lateral acceleration {lat:.1f} m/s^2 at "
                f"{extra * 3.6:.0f} km/h" if lat is not None else
                f"{r['role'] or r['kind']} {aid} ({r['type']}): heading jumped {extra:.0f} deg in 0.1 s")
        vis = visible(t, x, y)
        hard = lat is None or lat > TRAFFIC_LAT_ACC
        rep.add("swerve", t, t_e, what, visible=vis or is_part, blocking=(is_part or (vis and hard)))
    for t, t_e, aid, (x, y, d, sp, kind, typ) in _episodes(jumps, 0.3, lambda d: d[2]):
        rep.add("jump", t, t_e, f"{kind} {aid} ({typ}) moved {d:.2f} m in one tick (speed allows {sp:.2f} m)",
                visible=visible(t, x, y))
    for t, aid, (how, r) in pops:
        rep.add("jump", t, t, f"{r['kind']} {aid} ({r['type']}) {how} {POP_M:.0f} m or closer to the camera",
                visible=visible(t, r["x"], r["y"]))
    rep.metrics["participants"] = parts

    # SUMO: teleports and collisions
    ids = sumo_ids(trial / "events.csv")
    log = trial / "sumo" / "sumo.log"
    tele = sumo_teleports(log.read_text(errors="replace")) if log.exists() else []
    n_tele_win = 0
    for t, vid, why, lane in tele:
        aid = ids.get(vid)
        pos = _pos_of(series.get(aid), t) if aid else None
        vis = pos is not None and visible(_near_time(times, t), *pos)
        if lo <= t <= hi:
            n_tele_win += 1
        rep.add("teleport", t, t, f"SUMO teleported {vid} ({why}) from lane {lane}"
                + (f", CARLA actor {aid}" if aid else ""), visible=vis)
    coll = trial / "sumo" / "sumo_collisions.xml"
    n_coll = 0
    if coll.exists():
        try:
            rows = drivecheck.collisions(coll)["rows"]
        except Exception:  # noqa: BLE001 - a run that ended early leaves an unclosed file
            rows = []
        for t, a, b, kind in rows:
            n_coll += 1
            aid = ids.get(a) or ids.get(b)
            pos = _pos_of(series.get(aid), t) if aid else None
            vis = pos is not None and visible(_near_time(times, t), *pos)
            # SUMO's collision test touches boxes with no gap; what a viewer sees of it is the bodies' overlap,
            # which the overlap check blocks on (a bike grazing a bike in SUMO blocked the rush-hour clip)
            rep.add("sumo_collision", t, t, f"SUMO collision ({kind}) {a} with {b}", visible=vis, blocking=False)
    rep.metrics["sumo"] = {"teleports_total": len(tele), "teleports_near_window": n_tele_win,
                           "collisions_total": n_coll if coll.exists() else None}

    # camera jitter at the clip's frame times
    if cam_rows:
        fps = float(film.get("fps", 25.0))
        grid = [c for c in cam_rows if t0 - 1e-6 <= c[0] <= t1 + 1e-6 and
                abs((c[0] - t0) * fps - round((c[0] - t0) * fps)) < 0.25]
        if len(grid) >= 9:
            res = {}
            for i, name in ((1, "x"), (2, "y"), (3, "z"), (4, "yaw"), (5, "pitch")):
                vals = unwrap([g[i] for g in grid]) if i >= 4 else [g[i] for g in grid]
                res[name] = savgol_residual(vals)
            pos_r = [math.sqrt(res["x"][k] ** 2 + res["y"][k] ** 2 + res["z"][k] ** 2) for k in range(len(grid))]
            ang_r = [max(abs(res["yaw"][k]), abs(res["pitch"][k])) for k in range(len(grid))]
            core = slice(3, len(grid) - 3)
            rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0   # noqa: E731
            rep.metrics["camera_jitter"] = {
                "frames": len(grid), "pos_rms_cm": round(100 * rms(pos_r[core]), 2),
                "pos_max_cm": round(100 * max(pos_r[core]), 2), "ang_rms_deg": round(rms(ang_r[core]), 3),
                "ang_max_deg": round(max(ang_r[core]), 3),
                "frames_over": sum(1 for p, a in zip(pos_r, ang_r) if p > JITTER_M or a > JITTER_DEG)}  # at 0.15
            kinds = {c.get("name", "main"): c.get("kind") for c in
                     ([film["camera"]] if film.get("camera") else []) + list(film.get("cameras") or [])}
            ang_lim = JITTER_DEG_DRONE if kinds.get(camera) == "drone" else JITTER_DEG
            rep.metrics["camera_jitter"]["ang_limit_deg"] = ang_lim
            bad = [(grid[k][0], "camera", (pos_r[k], ang_r[k])) for k in range(len(grid))
                   if pos_r[k] > JITTER_M or ang_r[k] > ang_lim]
            for t, t_e, _, (p, a) in _episodes(bad, 0.2):
                rep.add("jitter", t, t_e, f"camera shakes: {100 * p:.1f} cm, {a:.2f} deg off a smooth path",
                        visible=True)
    return rep.as_dict()


def _xy_at(s, t):
    for tt, r in s:
        if tt >= t:
            return r["x"], r["y"]
    return s[-1][1]["x"], s[-1][1]["y"]


def _speed_at(s, t):
    for tt, r in s:
        if tt >= t:
            return r["speed"]
    return s[-1][1]["speed"]


def _pos_of(s, t):
    if not s:
        return None
    best = min(s, key=lambda p: abs(p[0] - t))
    return (best[1]["x"], best[1]["y"]) if abs(best[0] - t) < 1.0 else None


def _near_time(times, t):
    if not times:
        return t
    i = min(max(0, bisect.bisect_left(times, t)), len(times) - 1)
    return times[i]


def _cam_xy(cam, frames, t):
    if cam is not None:
        return cam[1], cam[2]
    rows = frames.get(t) or {}
    eyes = [r for r in rows.values() if r["kind"] == "participant" and r["z"] > -10]
    if eyes:
        return eyes[0]["x"], eyes[0]["y"]
    return 1e9, 1e9


def summary_lines(name, rep):
    """A short text report of one clip."""
    m = rep["metrics"]
    head = f"{name}: {'OK' if rep['ok'] else 'PROBLEMS'} (window {rep['window'][0]:g}-{rep['window'][1]:g} s; " \
           f"{rep['blocking']} blocking)"
    lines = [head]
    for role, p in (m.get("participants") or {}).items():
        lines.append(f"  {role}: max lateral acceleration {p['max_lat_acc']:.1f} m/s^2, max yaw rate "
                     f"{p['max_yaw_rate']:.0f} deg/s")
    c = m.get("large_vehicle_min_clearance") or {}
    if c.get("min_m") is not None:
        lines.append(f"  large vehicle min clearance {c['min_m']:.2f} m at {c['t']:.1f} s ({'-'.join(c['kinds'])})")
    j = m.get("camera_jitter")
    if j:
        lines.append(f"  camera jitter: position rms {j['pos_rms_cm']:.2f} cm, max {j['pos_max_cm']:.1f} cm; angle rms "
                     f"{j['ang_rms_deg']:.3f} deg, max {j['ang_max_deg']:.2f} deg; {j['frames_over']} frames over")
    s = m.get("sumo") or {}
    lines.append(f"  SUMO: {s.get('teleports_total', 0)} teleports ({s.get('teleports_near_window', 0)} near the window), "
                 f"collisions {s.get('collisions_total')}")
    for p in rep["problems"]:
        if p["in_window"]:
            flag = "BLOCK" if p["blocking"] else ("view " if p["in_view"] else "     ")
            span = f"{p['t']:.2f}" + (f"-{p['t_end']:.2f}" if p["t_end"] > p["t"] else "")
            lines.append(f"  [{flag}] {p['check']:14s} {span:>13s} s  {p['what']}")
    return lines
