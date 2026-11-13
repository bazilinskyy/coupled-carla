"""Measures of each trial's pedestrian-vehicle encounters, read from its frames.csv and events.csv.

    python -m coupledcarla.analysis logs/eindhoven_crossing                     # every trial of an experiment
    python -m coupledcarla.analysis logs/eindhoven_crossing/20261113-174013_trial1 --all-walkers --poi-frames

The host runs it after every trial. Per trial it writes <trial>/analysis/:

  encounters.csv  one row per pedestrian-vehicle pair that came within --radius metres of each other (see FIELDS)
  poi.csv         per participant and scripted agent, per point of interest: closest distance and when, first
                  time within the POI's radius
  frames_poi.csv  (--poi-frames) participant positions in each POI's reference frame, as coupled-sim's
                  reference frames: x forward along the POI's heading, y to its left, metres
  timeline.png    gap, speeds and time to collision per encounter, with eHMI changes, stops and eye contact
                  (when matplotlib is installed)

and for an experiment folder also <experiment>/analysis.csv, every trial's encounters in one table.

Geometry: a vehicle is a box (half length and width by blueprint, see DIMS) around its logged position, a
pedestrian a disc of PED_R. Time to collision (TTC) assumes both keep their current velocity, and counts only
while the vehicle moves (above 0.5 m/s), so a pedestrian passing a stopped car is not a conflict. The
post-encroachment time (PET) is the shortest time between the pedestrian being somewhere and the vehicle's box
covering that spot, sampled at 10 Hz; positive when the pedestrian was there first.
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

PED_R = 0.3                       # m
MOVING = 0.5                      # m/s: below this a vehicle counts as stopped
TTC_HORIZON = 10.0                # s
# keyword in the blueprint id -> (half length, half width) in m; the first match wins, else a car
DIMS = [
    (("fusorosa", "carlacola", "european_hgv", "firetruck", "fuso", "bus"), (5.5, 1.3)),
    (("sprinter", "volkswagen.t2", "ambulance"), (2.9, 1.0)),
    (("crossbike", "century", "omafiets", "bike", "harley", "kawasaki", "yamaha", "vespa", "ninja", "low_rider",
      "zx125"), (1.0, 0.4)),
]
CAR = (2.35, 0.95)

FIELDS = [
    "trial", "pedestrian", "pedestrian_id", "vehicle", "vehicle_id", "vehicle_type",
    "t_first", "t_last",                          # s: the pair's shared time in the log
    "min_gap_m", "t_min_gap",                     # closest distance between the pedestrian disc and the vehicle box
    "min_ttc_s", "t_min_ttc",                     # empty when no collision course within TTC_HORIZON
    "pet_s", "first",                             # PET (s, positive: pedestrian first); who passed the spot first
    "t_in_path", "veh_speed_in_path_kmh", "veh_dist_in_path_m", "ehmi_in_path",
    # the first time the pedestrian is in the vehicle's path (within its width + 0.5 m, ahead, within 60 m):
    # the vehicle's speed, its distance (front to pedestrian) and its eHMI state then
    "stopped", "t_stop", "stop_gap_m",            # the vehicle stopped with the pedestrian ahead (within 30 m, 8 m aside)
    "eye_contact_s", "t_eye_contact",             # total eye contact (MutualGaze events) and its first start
    "ehmi_states",                                # the vehicle's eHMI changes, "t:state" separated by |
    "collision",
]


def dims(type_id, kind=""):
    t = type_id.lower()
    if kind == "bike":
        return DIMS[2][1]
    for keys, d in DIMS:
        if any(k in t for k in keys):
            return d
    return CAR


class Track:
    """One actor's rows in frames.csv, as numpy arrays (time in s, CARLA metres, yaw in rad, velocity in m/s)."""

    def __init__(self, aid, kind, role, type_id, rows):
        self.id, self.kind, self.role, self.type_id = aid, kind, role, type_id
        a = np.array(rows, dtype=np.float64)
        order = np.argsort(a[:, 1], kind="stable")
        a = a[order]
        _, first = np.unique(a[:, 1], return_index=True)   # one row per frame
        a = a[first]
        self.t, self.frame, self.x, self.y = a[:, 0], a[:, 1].astype(np.int64), a[:, 2], a[:, 3]
        self.yaw, self.vx, self.vy = np.radians(a[:, 4]), a[:, 5], a[:, 6]
        self.lat, self.lon = a[:, 7], a[:, 8]
        self.speed = np.hypot(self.vx, self.vy)
        self.ehmi = None

    @property
    def name(self):
        return self.role or f"{self.kind}{self.id}"

    @property
    def is_vehicle(self):
        return self.type_id.startswith("vehicle.")

    @property
    def is_walker(self):
        return self.type_id.startswith("walker.")


def load_tracks(trial_dir):
    rows, meta, ehmi = {}, {}, {}
    with open(Path(trial_dir) / "frames.csv", newline="") as f:
        r = csv.reader(f, delimiter=";")
        head = next(r)
        ix = {k: i for i, k in enumerate(head)}
        cols = [ix[k] for k in ("timestamp", "frame", "pos_x", "pos_y", "rot_yaw", "vel_x", "vel_y", "lat", "lon")]
        ie = ix.get("ehmi")
        for row in r:
            if len(row) < len(head):
                continue
            aid = int(row[ix["actor_id"]])
            try:
                vals = [float(row[c]) if row[c] != "" else math.nan for c in cols]
            except ValueError:
                continue
            rows.setdefault(aid, []).append(vals)
            if aid not in meta:
                meta[aid] = (row[ix["kind"]], row[ix["role"]], row[ix["type_id"]])
            if ie is not None and row[ie]:
                ehmi.setdefault(aid, {})[int(row[ix["frame"]])] = row[ie]
    tracks = {}
    for aid, rs in rows.items():
        tr = Track(aid, *meta[aid], rs)
        if aid in ehmi:
            tr.ehmi = ehmi[aid]
        tracks[aid] = tr
    return tracks


def load_events(trial_dir):
    out = []
    p = Path(trial_dir) / "events.csv"
    if not p.exists():
        return out
    with open(p, newline="") as f:
        for row in csv.DictReader(f, delimiter=";"):
            try:
                out.append((float(row["timestamp"]), int(row["frame"] or 0), row["event"], row["actor_id"],
                            row["detail"] or ""))
            except (TypeError, ValueError):
                continue
    return out


def retime(events, tracks):
    """Put tracks and events on one clock, trial time from the first logged frame.

    Logs before 2026-11-14 wrote the server's elapsed time in frames.csv (and for traffic_light events) but trial
    time for the other events; events carry their frame number, so each event takes its frame's time.
    """
    if not tracks:
        return events
    t0 = min(float(tr.t[0]) for tr in tracks.values())
    at = {}
    for tr in tracks.values():
        tr.t = tr.t - t0
        at.update(zip(tr.frame.tolist(), tr.t.tolist()))
    return [(at.get(f, t), f, ev, aid, detail) for t, f, ev, aid, detail in events]


def eye_contact(events, t_end):
    """{frozenset(name, name): (total s, first start)} from eye_contact_start / _end events."""
    open_, total = {}, {}
    for t, _, ev, _, detail in events:
        if not ev.startswith("eye_contact_"):
            continue
        m = re.match(r"(\S+)<->(\S+)", detail)
        if not m:
            continue
        key = frozenset(m.groups())
        if ev == "eye_contact_start":
            open_.setdefault(key, t)
            total.setdefault(key, [0.0, t])
        elif key in open_:
            total[key][0] += t - open_.pop(key)
    for key, t0 in open_.items():
        total[key][0] += t_end - t0
    return {k: (round(v[0], 2), v[1]) for k, v in total.items()}


def in_vehicle_frame(veh, i, px, py):
    """Points (px, py) in the frame of vehicle sample(s) i: (forward, lateral)."""
    c, s = np.cos(veh.yaw[i]), np.sin(veh.yaw[i])
    dx, dy = px - veh.x[i], py - veh.y[i]
    return dx * c + dy * s, -dx * s + dy * c


def ttc_box(px, py, vx, vy, a, b):
    """Time until a point at (px, py) moving with (vx, vy) enters the box |x| <= a, |y| <= b (arrays; inf if never)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1, tx2 = (-a - px) / vx, (a - px) / vx
        ty1, ty2 = (-b - py) / vy, (b - py) / vy
    still_x, still_y = np.abs(vx) < 1e-9, np.abs(vy) < 1e-9
    inside_x, inside_y = np.abs(px) <= a, np.abs(py) <= b
    lo_x = np.where(still_x, np.where(inside_x, -np.inf, np.inf), np.minimum(tx1, tx2))
    hi_x = np.where(still_x, np.where(inside_x, np.inf, -np.inf), np.maximum(tx1, tx2))
    lo_y = np.where(still_y, np.where(inside_y, -np.inf, np.inf), np.minimum(ty1, ty2))
    hi_y = np.where(still_y, np.where(inside_y, np.inf, -np.inf), np.maximum(ty1, ty2))
    enter, leave = np.maximum(lo_x, lo_y), np.minimum(hi_x, hi_y)
    hit = (enter <= leave) & (leave >= 0)
    return np.where(hit, np.maximum(enter, 0.0), np.inf)


def pet(ped, veh, a, b, step_s=0.1):
    """(PET s, who first) for the pedestrian's positions covered by the vehicle's box at some time, or (None, "")."""
    def thin(tr):
        dt = np.median(np.diff(tr.t)) if len(tr.t) > 1 else step_s
        return np.arange(0, len(tr.t), max(1, int(round(step_s / max(dt, 1e-6)))))
    ip, iv = thin(ped), thin(veh)
    if not len(ip) or not len(iv):
        return None, ""
    fx, fy = in_vehicle_frame(veh, iv[:, None], ped.x[ip][None, :], ped.y[ip][None, :])
    covered = (np.abs(fx) <= a) & (np.abs(fy) <= b)
    if not covered.any():
        return None, ""
    dt = veh.t[iv][:, None] - ped.t[ip][None, :]
    dt = np.where(covered, dt, np.nan)
    k = np.nanargmin(np.abs(dt))
    v = float(dt.flat[k])
    return round(v, 2), ("pedestrian" if v > 0 else "vehicle")


def encounter(ped, veh, events, eye, radius):
    """The FIELDS of one pedestrian-vehicle pair, or None if they never came within `radius` m."""
    common, ip, iv = np.intersect1d(ped.frame, veh.frame, return_indices=True)
    if len(common) < 2:
        return None
    d = np.hypot(ped.x[ip] - veh.x[iv], ped.y[ip] - veh.y[iv])
    if np.nanmin(d) > radius:
        return None
    hl, hw = dims(veh.type_id, veh.kind)
    t = veh.t[iv]
    fx, fy = in_vehicle_frame(veh, iv, ped.x[ip], ped.y[ip])
    gap = np.maximum(np.hypot(np.maximum(np.abs(fx) - hl, 0), np.maximum(np.abs(fy) - hw, 0)) - PED_R, 0)
    c, s = np.cos(veh.yaw[iv]), np.sin(veh.yaw[iv])
    rvx, rvy = ped.vx[ip] - veh.vx[iv], ped.vy[ip] - veh.vy[iv]
    ttc = ttc_box(fx, fy, rvx * c + rvy * s, -rvx * s + rvy * c, hl + PED_R, hw + PED_R)
    ttc = np.where((veh.speed[iv] > MOVING) & (ttc <= TTC_HORIZON), ttc, np.inf)
    out = {"pedestrian": ped.name, "pedestrian_id": ped.id, "vehicle": veh.name, "vehicle_id": veh.id,
           "vehicle_type": veh.type_id, "t_first": round(float(t[0]), 2), "t_last": round(float(t[-1]), 2)}
    k = int(np.nanargmin(gap))
    out.update(min_gap_m=round(float(gap[k]), 2), t_min_gap=round(float(t[k]), 2))
    if np.isfinite(ttc).any():
        k = int(np.argmin(ttc))
        out.update(min_ttc_s=round(float(ttc[k]), 2), t_min_ttc=round(float(t[k]), 2))
    out["pet_s"], out["first"] = pet(ped, veh, hl + PED_R, hw + PED_R)
    in_path = (np.abs(fy) <= hw + PED_R + 0.5) & (fx > 0) & (fx < hl + 60)
    if in_path.any():
        k = int(np.argmax(in_path))
        out.update(t_in_path=round(float(t[k]), 2), veh_speed_in_path_kmh=round(3.6 * float(veh.speed[iv][k]), 1),
                   veh_dist_in_path_m=round(max(float(fx[k]) - hl, 0.0), 1))
        if veh.ehmi is not None:
            out["ehmi_in_path"] = veh.ehmi.get(int(common[k]), "")
    ahead = (fx > 0) & (fx < hl + 30) & (np.abs(fy) < 8)
    moved = np.maximum.accumulate(veh.speed[iv] > 2.0)
    stop = ahead & moved & (veh.speed[iv] < MOVING)
    out["stopped"] = int(stop.any())
    if stop.any():
        k = int(np.argmax(stop))
        out.update(t_stop=round(float(t[k]), 2), stop_gap_m=round(float(gap[k]), 1))
    e = eye.get(frozenset((ped.name, veh.name)))
    if e:
        out.update(eye_contact_s=e[0], t_eye_contact=round(e[1], 2))
    states = []
    for et, _, ev, aid, detail in events:
        if ev == "ehmi" and aid == str(veh.id):
            m = re.match(r"\S+ (\S+)", detail)
            states.append(f"{et:.2f}:{m.group(1) if m else detail}")
    out["ehmi_states"] = "|".join(states)
    out["collision"] = int(any(ev == "collision" and (
        (detail.startswith(veh.name + " hit") and aid == str(ped.id)) or
        (detail.startswith(ped.name + " hit") and aid == str(veh.id))) for _, _, ev, aid, detail in events))
    out["_series"] = (t, gap, ttc, 3.6 * veh.speed[iv], 3.6 * ped.speed[ip])
    return out


def enu(lat, lon, lat0, lon0):
    """Local east/north metres around (lat0, lon0) (equirectangular: exact enough over a few km)."""
    r = 6371008.8
    return (np.radians(lon - lon0) * r * math.cos(math.radians(lat0)), np.radians(lat - lat0) * r)


def poi_tables(tracks, pois, frames_poi=False):
    """(poi rows, frames_poi rows) for participants and scripted agents."""
    rows, frames = [], []
    actors = [tr for tr in tracks.values() if tr.kind in ("participant", "agent")]
    for poi in pois or []:
        lat0, lon0 = float(poi["lat"]), float(poi["lon"])
        h = math.radians(float(poi.get("heading", 0.0)))           # compass degrees: 0 north, 90 east
        fwd, left = (math.sin(h), math.cos(h)), (-math.cos(h), math.sin(h))
        radius = float(poi.get("radius_m", 10.0))
        for tr in actors:
            e, n = enu(tr.lat, tr.lon, lat0, lon0)
            d = np.hypot(e, n)
            if not np.isfinite(d).any():
                continue
            k = int(np.nanargmin(d))
            within = d <= radius
            rows.append({"poi": poi.get("name", ""), "actor": tr.name, "actor_id": tr.id,
                         "min_dist_m": round(float(d[k]), 2), "t_min_dist": round(float(tr.t[k]), 2),
                         "t_within": round(float(tr.t[int(np.argmax(within))]), 2) if within.any() else ""})
            if frames_poi and tr.kind == "participant":
                x, y = e * fwd[0] + n * fwd[1], e * left[0] + n * left[1]
                for i in range(len(tr.t)):
                    frames.append([f"{tr.t[i]:.3f}", int(tr.frame[i]), tr.id, tr.name, poi.get("name", ""),
                                   f"{x[i]:.3f}", f"{y[i]:.3f}", f"{d[i]:.3f}"])
    return rows, frames


def plot(encounters, events, path, trial_name):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    def interest(e):     # conflicts first (collision course or crossed paths), human pedestrians first, then by gap
        conflict = e.get("min_ttc_s") is not None or (e.get("pet_s") is not None and abs(e["pet_s"]) <= 10)
        return (not e.get("collision"), not conflict, e["pedestrian"] != "pedestrian", e["min_gap_m"])
    enc = sorted(encounters, key=interest)
    enc = ([e for e in enc if not interest(e)[1]] or enc)[:6]      # only the conflicts when there are any
    if not enc:
        return False
    fig, axes = plt.subplots(len(enc), 1, figsize=(11, 2.6 * len(enc)), sharex=True, squeeze=False)
    for ax, e in zip(axes[:, 0], enc):
        t, gap, ttc, vs, ps = e["_series"]
        ax.plot(t, gap, color="#3b6fb6", lw=1.6, label="gap (m)")
        ax.set_ylabel("gap m / TTC s")
        fin = np.isfinite(ttc)
        ax.scatter(t[fin], ttc[fin], s=3, color="#c0392b", label="TTC (s)")
        ax.set_ylim(0, max(12.0, min(60.0, float(np.nanmax(gap)))))
        ax2 = ax.twinx()
        ax2.plot(t, vs, color="#555", lw=1.0, ls="--", label=f"{e['vehicle']} km/h")
        ax2.plot(t, ps, color="#2e8b57", lw=1.0, ls=":", label=f"{e['pedestrian']} km/h")
        ax2.set_ylabel("km/h")
        for et, _, ev, aid, detail in events:
            if ev == "ehmi" and aid == str(e["vehicle_id"]):
                ax.axvline(et, color="#e67e22", lw=0.8)
                ax.text(et, ax.get_ylim()[1] * 0.92, " " + detail.split(" (")[0].split(" ", 1)[-1], fontsize=7,
                        color="#e67e22")
            elif ev in ("agent_stops", "agent_goes") and aid == str(e["vehicle_id"]):
                ax.axvline(et, color="#888", lw=0.8, ls=":")
        start = None
        for et, _, ev, _, detail in events:
            if ev.startswith("eye_contact_") and {e["pedestrian"], e["vehicle"]} == set(detail.split(" ")[0].split("<->")):
                if ev == "eye_contact_start":
                    start = et
                elif start is not None:
                    ax.axvspan(start, et, color="#f1c40f", alpha=0.15, lw=0)
                    start = None
        title = f"{e['pedestrian']} vs {e['vehicle']}: min gap {e['min_gap_m']} m"
        if e.get("min_ttc_s") is not None:
            title += f", min TTC {e['min_ttc_s']} s"
        if e.get("pet_s") is not None:
            title += f", PET {e['pet_s']} s ({e['first']} first)"
        ax.set_title(title, fontsize=9, loc="left")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper right", ncol=4)
    axes[-1, 0].set_xlabel("trial time (s); orange: eHMI, dotted: agent stops/goes, yellow: eye contact")
    fig.suptitle(trial_name, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def _write(path, fields, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, delimiter=";", extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if r.get(k) is None else r.get(k)) for k in fields})


def trial_index(trial_dir):
    m = re.search(r"_trial(\d+)$", Path(trial_dir).name)
    return int(m.group(1)) if m else ""


def analyse_trial(trial_dir, radius=30.0, all_walkers=False, poi_frames=False, plots=True):
    """Analyse one trial folder; returns its encounter rows (without the plot series)."""
    trial_dir = Path(trial_dir)
    tracks = load_tracks(trial_dir)
    events = retime(load_events(trial_dir), tracks)
    t_end = max((tr.t[-1] for tr in tracks.values()), default=0.0)
    eye = eye_contact(events, t_end)
    peds = [tr for tr in tracks.values() if tr.is_walker and (
        tr.kind in ("participant", "agent") or all_walkers)]
    vehs = [tr for tr in tracks.values() if tr.is_vehicle]
    enc = []
    for p in peds:
        for v in vehs:
            e = encounter(p, v, events, eye, radius)
            if e:
                e["trial"] = trial_index(trial_dir)
                enc.append(e)
    enc.sort(key=lambda e: (e["pedestrian"], e["t_first"]))
    out = trial_dir / "analysis"
    out.mkdir(exist_ok=True)
    _write(out / "encounters.csv", FIELDS, enc)
    cfg_path = trial_dir / "config.yaml"
    pois = []
    if cfg_path.exists():
        import yaml
        pois = (yaml.safe_load(cfg_path.read_text()) or {}).get("pois") or []
    prow, pframes = poi_tables(tracks, pois, poi_frames)
    if prow:
        _write(out / "poi.csv", ["poi", "actor", "actor_id", "min_dist_m", "t_min_dist", "t_within"], prow)
    if pframes:
        with open(out / "frames_poi.csv", "w", newline="") as f:
            w = csv.writer(f, delimiter=";")
            w.writerow(["timestamp", "frame", "actor_id", "actor", "poi", "x_forward", "y_left", "distance"])
            w.writerows(pframes)
    if plots:
        plot(enc, events, out / "timeline.png", trial_dir.name)
    return [{k: v for k, v in e.items() if k != "_series"} for e in enc]


def trial_dirs(path):
    path = Path(path)
    if (path / "frames.csv").exists():
        return [path]
    return sorted(p for p in path.iterdir() if (p / "frames.csv").exists())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help="a trial folder or an experiment's log folder")
    ap.add_argument("--radius", type=float, default=30.0, help="pairs closer than this (m) at some time")
    ap.add_argument("--all-walkers", action="store_true", help="also the crowd and other unscripted walkers")
    ap.add_argument("--poi-frames", action="store_true", help="write frames_poi.csv")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)
    dirs = trial_dirs(args.path)
    if not dirs:
        sys.exit(f"no trial folders (with frames.csv) in {args.path}")
    allrows = []
    for d in dirs:
        rows = analyse_trial(d, args.radius, args.all_walkers, args.poi_frames, not args.no_plots)
        for r in rows:
            r["trial_dir"] = d.name
        allrows += rows
        print(f"{d.name}: {len(rows)} encounters")
        for r in rows:
            bits = [f"gap {r['min_gap_m']} m"]
            if r.get("min_ttc_s") is not None:
                bits.append(f"TTC {r['min_ttc_s']} s")
            if r.get("pet_s") is not None:
                bits.append(f"PET {r['pet_s']} s ({r['first']} first)")
            if r.get("stopped"):
                bits.append(f"stopped at {r['t_stop']} s, {r['stop_gap_m']} m away")
            if r.get("eye_contact_s"):
                bits.append(f"eye contact {r['eye_contact_s']} s")
            if r.get("collision"):
                bits.append("COLLISION")
            print(f"  {r['pedestrian']} / {r['vehicle']}: " + ", ".join(bits))
    if len(dirs) > 1:
        _write(Path(args.path) / "analysis.csv", ["trial_dir"] + FIELDS, allrows)
        print(f"wrote {Path(args.path) / 'analysis.csv'}")


if __name__ == "__main__":
    main()
