"""What a trial's camera saw of the traffic: vehicles by class and walkers in its view, against the place's CROWD mix.

Reads a trial's frames.csv (the AI actors within logging.radius_m of the participants, and of the drones and the
ground their cameras film) and puts a camera where the clip's camera was:

  --camera drone     a pilot's gimbal camera (drones.csv: the pilot's pose and gimbal pitch)
  --camera follow    a chase camera behind --role (offset back / up in metres, looking along its heading)

Every --every seconds from --start for --duration it counts the actors inside the camera's frustum (--fov
horizontal, 16:9) nearer than --max-m: the mean in view per class and the unique ones per minute, and prints the
shares beside the place's CROWD shares (crowdtraffic.plan_for from the trial's config.yaml). Occlusion by
buildings is not modelled: these are the actors the camera could see.

    python tools/traffic_seen.py logs/<exp>/<stamp>_trial0 --camera drone --start 25 --duration 20
    python tools/traffic_seen.py logs/<exp>/<stamp>_trial0 --camera follow --role driver --back 8.5 --up 3.4
"""
import argparse
import collections
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CLASSES = ("car", "taxi", "van", "truck", "bus", "motorcycle", "bicycle", "emergency", "walker", "parked")
CROWD_OF = {"car": "cars", "taxi": "cars", "van": "trucks", "truck": "trucks", "bus": "buses",
            "motorcycle": "motorcycles", "bicycle": "bicycles", "emergency": "cars", "walker": "persons"}


def vehicle_class(type_id):
    t = type_id.lower()
    if t.startswith("walker."):
        return "walker"
    if any(k in t for k in ("ambulance", "police", "dodgecop", "firetruck")):
        return "emergency"
    if any(k in t for k in ("omafiets", "crossbike", "century")):
        return "bicycle"
    if any(k in t for k in ("vespa", "yamaha", "kawasaki", "harley")):
        return "motorcycle"
    if any(k in t for k in ("fusorosa", "fuso")):
        return "bus"
    if any(k in t for k in ("carlacola", "european_hgv")):
        return "truck"
    if any(k in t for k in ("sprinter", "volkswagen.t2")):
        return "van"
    if "taxi" in t or "ford.crown" in t:
        return "taxi"
    return "car" if t.startswith("vehicle.") else None


def in_view(cam, pt, fov_deg=90.0, aspect=16 / 9, max_m=300.0):
    """Whether pt (x, y, z) is inside the frustum of cam (x, y, z, yaw deg, pitch deg), nearer than max_m."""
    x, y, z, yaw, pitch = cam
    dx, dy, dz = pt[0] - x, pt[1] - y, pt[2] - z
    d = math.sqrt(dx * dx + dy * dy + dz * dz)
    if d < 1.0 or d > max_m:
        return False
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    fwd = (cp * cy, cp * sy, sp)                       # CARLA: x forward, y right, z up; pitch positive up
    right = (-sy, cy, 0.0)
    up = (-sp * cy, -sp * sy, cp)
    f = dx * fwd[0] + dy * fwd[1] + dz * fwd[2]
    if f <= 0:
        return False
    r = dx * right[0] + dy * right[1] + dz * right[2]
    u = dx * up[0] + dy * up[1] + dz * up[2]
    th = math.tan(math.radians(fov_deg / 2))
    return abs(r / f) <= th and abs(u / f) <= th / aspect


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f, delimiter=";"))


def cameras_drone(trial, role="pilot"):
    out = {}
    for r in read(Path(trial) / "drones.csv"):
        if r["role"] != role or r["kind"] != "drone":
            continue
        t = round(float(r["timestamp"]), 2)
        # drones.csv heading is the compass heading of the CARLA yaw (drones.py: heading = -degrees(yaw) on ENU)
        out[t] = (float(r["pos_x"]), float(r["pos_y"]), float(r["pos_z"]), float(r["heading"]),
                  float(r["gimbal_pitch"] or -15.0))
    return out


def cameras_follow(rows, role, back, up, look_up):
    out = {}
    for r in rows:
        if r.get("role") != role:
            continue
        yaw = float(r["rot_yaw"])
        x = float(r["pos_x"]) - back * math.cos(math.radians(yaw))
        y = float(r["pos_y"]) - back * math.sin(math.radians(yaw))
        z = float(r["pos_z"]) + up
        pitch = -math.degrees(math.atan2(up - look_up, back + 12.0))
        out[round(float(r["timestamp"]), 2)] = (x, y, z, yaw, pitch)
    return out


def count(rows, cams, start, duration, every=1.0, fov=90.0, max_m=300.0, parked=()):
    """{class: mean in view}, {class: unique per minute}, samples. parked: [(x, y, z)] of the trial's parked cars
    (parked.csv; a dashcam's tracks count them too)."""
    by_t = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") in ("participant", "agent", "drone", "drone_ai"):
            continue
        by_t[round(float(r["timestamp"]), 2)].append(r)
    times = sorted(t for t in by_t if start <= t <= start + duration)
    cam_times = sorted(cams)
    if not times or not cam_times:
        return {}, {}, 0
    mean, uniq, n, next_t = collections.Counter(), collections.defaultdict(set), 0, None
    for t in times:
        if next_t is not None and t < next_t:
            continue
        next_t = t + every
        ct = min(cam_times, key=lambda c: abs(c - t))
        cam = cams[ct]
        n += 1
        for r in by_t[t]:
            c = vehicle_class(r["type_id"])
            if c is None:
                continue
            pt = (float(r["pos_x"]), float(r["pos_y"]), float(r["pos_z"]) + (0.9 if c == "walker" else 0.7))
            if in_view(cam, pt, fov, max_m=max_m):
                mean[c] += 1
                uniq[c].add(r["actor_id"])
        for k, (px, py, pz) in enumerate(parked):
            if abs(px - cam[0]) <= max_m and abs(py - cam[1]) <= max_m and in_view(cam, (px, py, pz + 0.7), fov,
                                                                                  max_m=max_m):
                mean["parked"] += 1
                uniq["parked"].add(k)
    minutes = max(duration, every) / 60.0
    return ({c: mean[c] / n for c in CLASSES}, {c: len(uniq[c]) / minutes for c in CLASSES}, n)


def crowd_target(trial):
    """The place's CROWD plan (crowdtraffic.plan_for) from the trial's config.yaml, or None."""
    try:
        import yaml
        sys.path.insert(0, str(ROOT))
        from coupledcarla import crowdpeds, crowdtraffic
        cfg = yaml.safe_load((Path(trial) / "config.yaml").read_text())
        bbox = (cfg.get("map") or {}).get("bbox")
        if not bbox:
            meta = ROOT / "maps" / cfg["map"]["name"] / "meta.json"
            bbox = json.loads(meta.read_text())["bbox"]
        iso3, _ = crowdpeds.locate(cfg, bbox, None)
        from coupledcarla.transit import trial_time
        when = trial_time((cfg.get("simulation") or {}).get("time"))
        return crowdtraffic.plan_for(iso3, (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2, when,
                                     crowdtraffic.settings(cfg))
    except Exception as err:  # noqa: BLE001 - the counts without the target
        print(f"(no CROWD target: {type(err).__name__}: {err})", file=sys.stderr)
        return None


def report(mean, per_min, n, target=None, parked=None):
    lines = [f"samples {n}"]
    veh = [c for c in CLASSES if c not in ("walker", "parked")]
    tot_m = sum(per_min[c] for c in veh) or 1e-9
    lines.append("class        in view (mean)  unique/min  share of vehicles" + ("  CROWD share" if target else ""))
    seen = (target or {}).get("seen", {})
    for c in CLASSES:
        share = f"{100 * per_min[c] / tot_m:6.1f}%" if c not in ("walker", "parked") else "      "
        tgt = ""
        if target and c in ("car", "van", "bus", "motorcycle", "bicycle"):
            key = {"car": "cars", "van": "trucks", "bus": "buses", "motorcycle": "motorcycles",
                   "bicycle": "bicycles"}[c]
            tgt = f"  {100 * seen[key]:6.1f}%" + (" (cars+taxis+service)" if c == "car" else
                                                  " (vans+lorries)" if c == "van" else "")
        lines.append(f"{c:12s} {mean[c]:8.1f}  {per_min[c]:10.1f}  {share}{tgt}")
    motor = sum(per_min[c] for c in veh if c != "bicycle")
    lines.append(f"motor vehicles per minute in view: moving {motor:.1f}, with the parked cars {motor + per_min['parked']:.1f} "
                 f"(a dashcam's tracks include parked cars)")
    if target:
        lines.append(f"CROWD per minute (dashcam): persons {target['rates']['persons']:.1f}, motor vehicles "
                     f"{sum(target['rates'][k] for k in ('cars', 'trucks', 'buses', 'motorcycles')):.1f}, "
                     f"bicycles {target['rates']['bicycles']:.1f} ({', '.join(f'{a} {b} h' for a, b in target['levels'][:2])})")
    if parked is not None:
        lines.append(f"parked cars in the trial: {parked}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trial")
    ap.add_argument("--camera", choices=["drone", "follow"], default="follow")
    ap.add_argument("--role", default=None)
    ap.add_argument("--back", type=float, default=8.5)
    ap.add_argument("--up", type=float, default=3.4)
    ap.add_argument("--look-up", type=float, default=1.5)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--every", type=float, default=1.0)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--max-m", type=float, default=300.0)
    ap.add_argument("--json", help="also write the numbers here")
    a = ap.parse_args()
    rows = read(Path(a.trial) / "frames.csv")
    if a.camera == "drone":
        cams = cameras_drone(a.trial, a.role or "pilot")
    else:
        cams = cameras_follow(rows, a.role or "driver", a.back, a.up, a.look_up)
    parked_xy = []
    pc = Path(a.trial) / "parked.csv"
    if pc.exists():
        parked_xy = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in read(pc)]
    mean, per_min, n = count(rows, cams, a.start, a.duration, a.every, a.fov, a.max_m, parked_xy)
    if not n:
        sys.exit("no frames in the window (or no camera)")
    try:
        parked = json.loads((Path(a.trial) / "summary.json").read_text()).get("parked_cars")
    except (OSError, ValueError):
        parked = None
    target = crowd_target(a.trial)
    print(report(mean, per_min, n, target, parked))
    if a.json:
        Path(a.json).write_text(json.dumps({"mean": mean, "per_min": per_min, "samples": n, "parked": parked,
                                            "target": target}, indent=1))


if __name__ == "__main__":
    main()
