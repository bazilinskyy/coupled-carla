"""How realistic was the driving in a CARLA trial? Measures from its frames.csv and events.csv.

    python tools/drive_check.py logs/<experiment>/<time>_trial0 [more trial folders] [--every 5] [--from 0]

Per trial: the participants' cars (stuck, reset_to_lane, respawn, left_road events; collisions by what they hit;
|accel| and |jerk| from their logged speeds), the vehicles logged near the participants (pairs whose bounding boxes
overlap, from coupledcarla/data/blueprint_sizes_*.json, counted as episodes), vehicle-walker contacts (a walker inside
a vehicle's box), heading jumps (> 45 deg in 0.1 s) and position jumps (faster than 90 km/h) of vehicles, and SUMO's
yields to walkers (sumo_yields, sumo_emergency_stop). Prints one JSON per trial and the totals.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import drivecheck, drivers  # noqa: E402


def check(folder, every=5, t_from=0.0):
    folder = Path(folder)
    sizes = drivers.sizes()
    ev = {}
    hit_kinds = {}
    with open(folder / "events.csv", newline="") as f:
        for row in csv.DictReader(f, delimiter=";"):
            if float(row["timestamp"]) < t_from:
                continue
            ev[row["event"]] = ev.get(row["event"], 0) + 1
            if row["event"] == "collision":
                what = row["detail"].split(" hit ")[-1]
                cat = what.split(".")[0] if what else "?"
                who = row["detail"].split(" hit ")[0]
                hit_kinds[f"{who}->{cat}"] = hit_kinds.get(f"{who}->{cat}", 0) + 1
    part = {}
    overlaps, contacts = [], []
    frame_rows, cur = [], None
    last, jumps = {}, {"heading": [], "position": []}     # vehicle -> (frame, x, y, yaw); sync smoothness

    def flush(rows):
        if not rows:
            return
        frame = int(rows[0]["frame"])
        veh, walk = {}, {}
        for r in rows:
            x, y = float(r["pos_x"]), float(r["pos_y"])
            if r["type_id"].startswith("walker."):
                walk[r["actor_id"]] = (x, y)
            elif r["type_id"].startswith("vehicle."):
                s = sizes.get(r["type_id"])
                if s:
                    w = min(s["width"], drivers.MAX_WIDTH["bus"])
                    veh[r["actor_id"]] = (x, y, float(r["rot_yaw"]), s["length"] / 2.0, w / 2.0)
        for aid, (x, y, yaw, _hl, _hw) in veh.items():
            prev = last.get(aid)
            if prev is not None and frame - prev[0] == every:
                dt = every * 0.02
                if abs((yaw - prev[3] + 180.0) % 360.0 - 180.0) > 45.0:        # more than 450 deg/s
                    jumps["heading"].append((frame, aid))
                if ((x - prev[1]) ** 2 + (y - prev[2]) ** 2) ** 0.5 > 25.0 * dt + 1.0:   # faster than 90 km/h + 1 m
                    jumps["position"].append((frame, aid))
            last[aid] = (frame, x, y, yaw)
        pairs, hits = drivecheck.frame_conflicts(veh, walk)
        overlaps.extend((frame, p) for p in pairs)
        contacts.extend((frame, h) for h in hits)

    with open(folder / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            t = float(r["timestamp"])
            if t < t_from:
                continue
            if r["kind"] == "participant" and r["type_id"].startswith("vehicle."):
                p = part.setdefault(r["role"], {"t": [], "v": []})
                p["t"].append(t)
                p["v"].append((float(r["vel_x"]), float(r["vel_y"])))
            fr = int(r["frame"])
            if fr != cur:
                if cur is not None and cur % every == 0:
                    flush(frame_rows)
                frame_rows, cur = [], fr
            frame_rows.append(r)
        if cur is not None and cur % every == 0:
            flush(frame_rows)
    out = {"trial": str(folder), "events": {k: ev.get(k, 0) for k in (
        "stuck", "reset_to_lane", "respawn", "left_road", "collision", "sumo_yields", "sumo_emergency_stop")},
        "collisions_by_kind": hit_kinds,
        "vehicle_overlap_episodes": drivecheck.episodes(overlaps, gap_frames=every * 5),
        "walker_contact_episodes": drivecheck.episodes(contacts, gap_frames=every * 5),
        "heading_jumps": drivecheck.episodes(jumps["heading"], gap_frames=every * 5),
        "position_jumps": drivecheck.episodes(jumps["position"], gap_frames=every * 5),
        "walker_contacts": len({h for _, h in contacts})}
    for role, p in part.items():
        acc, jerk = drivecheck.series_jerk(p["t"], p["v"])
        sp = [(vx * vx + vy * vy) ** 0.5 for vx, vy in p["v"]]
        out[role] = {"abs_accel": drivecheck.summary(acc), "abs_jerk": drivecheck.summary(jerk),
                     "mean_speed_kmh": round(3.6 * sum(sp) / max(1, len(sp)), 1),
                     "standing_share": round(drivecheck.share(sp, lambda v: v < 0.3) or 0.0, 3)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", nargs="+")
    ap.add_argument("--every", type=int, default=5, help="check overlaps every n-th frame")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="ignore the trial's first seconds")
    a = ap.parse_args()
    res = [check(t, a.every, a.t_from) for t in a.trials]
    for r in res:
        print(json.dumps(r, indent=1))
    if len(res) > 1:
        tot = {k: sum(r["events"][k] for r in res) for k in res[0]["events"]}
        tot["vehicle_overlap_episodes"] = sum(r["vehicle_overlap_episodes"] for r in res)
        for k in ("walker_contact_episodes", "heading_jumps", "position_jumps"):
            tot[k] = sum(r[k] for r in res)
        print(json.dumps({"totals": tot}, indent=1))


if __name__ == "__main__":
    main()
