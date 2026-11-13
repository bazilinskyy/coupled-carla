"""Export a trial for the browser viewer (viewer/index.html): roads, buildings, signals and trajectories.

    python -m coupledcarla.export_trial logs/eindhoven_centre/<time>_trial0      # writes trial.json there
    python -m coupledcarla.export_trial --map eindhoven_centre                     # map only (no trial)

Coordinates are CARLA metres (x east, y south); the viewer maps them to three.js (x, z) with y up.
"""
import argparse
import csv
import json
from pathlib import Path

import sumolib
import yaml

ROOT = Path(__file__).resolve().parent.parent


def map_layers(map_dir):
    geo = sumolib.net.readNet(str(map_dir / "geo.net.xml"), withInternal=False)
    lanes = []
    for edge in geo.getEdges():
        for lane in edge.getLanes():
            kind = "bike" if lane.allows("bicycle") and not lane.allows("passenger") else \
                "walk" if lane.allows("pedestrian") and not lane.allows("passenger") else "road"
            if kind == "walk":
                continue
            lanes.append({"k": kind, "w": round(lane.getWidth(), 2),
                          "p": [[round(x, 1), round(-y, 1)] for x, y in lane.getShape()]})
    buildings = json.loads((map_dir / "buildings.json").read_text()) if (map_dir / "buildings.json").exists() else []
    meta = json.loads((map_dir / "meta.json").read_text())
    return {"map": meta.get("name"), "bbox": meta.get("bbox"), "lefthand": meta.get("lefthand", False),
            "lanes": lanes, "buildings": buildings}


def trial_layers(trial_dir, every_participant=0.1, every_ai=0.2):
    tracks, last = {}, {}
    with open(trial_dir / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            t, aid, kind = float(r["timestamp"]), r["actor_id"], r["kind"]
            step = every_participant if kind in ("participant", "agent") else every_ai
            if aid in last and t - last[aid] < step - 1e-6:
                continue
            last[aid] = t
            tr = tracks.setdefault(aid, {"kind": kind, "type": r["type_id"], "role": r["role"], "s": []})
            sample = [round(t, 2), round(float(r["pos_x"]), 2), round(float(r["pos_y"]), 2),
                      round(float(r["rot_yaw"]), 1), round(float(r["speed_kmh"]), 1)]
            if kind in ("participant", "agent"):
                sample += [r["next_tl_state"] or "", r["next_tl_dist"] or "", r.get("ehmi") or ""]
            tr["s"].append(sample)
    t0 = min((tr["s"][0][0] for tr in tracks.values()), default=0.0)
    events = []
    with open(trial_dir / "events.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if r["event"] not in ("spawn",):
                # events.csv times are seconds since the trial started; frames.csv has simulator time
                events.append([round(float(r["timestamp"]) + t0, 2), r["event"], r["detail"][:120]])
    return {"t0": t0, "tracks": tracks, "events": events}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("trial", nargs="?")
    ap.add_argument("--map")
    args = ap.parse_args()
    if args.trial:
        trial = Path(args.trial)
        cfg = yaml.safe_load((trial / "config.yaml").read_text())
        data = map_layers(ROOT / "maps" / cfg["map"]["name"])
        data["experiment"] = cfg["name"]
        from .ai import DEFAULT_EHMI
        data["ehmi"] = (cfg.get("ehmi") or {}).get("states") or DEFAULT_EHMI
        data["city"] = cfg.get("city")
        data.update(trial_layers(trial))
        out = trial / "trial.json"
    else:
        data = map_layers(ROOT / "maps" / args.map)
        out = ROOT / "maps" / args.map / "map.json"
    out.write_text(json.dumps(data, separators=(",", ":")))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
