"""Simulated pedestrian crossings against CROWD's real ones (coupledcarla/crowdpeds.py calibration, checked in trials).

    python tools/crowd_validate.py logs/*/*_trial0 [--json out.json]

For each trial with crowd walkers (traffic.walkers; events.csv "crowd_crosses"): the place's CROWD values
(data/crowd_pedestrians.json: city, else country) and what the walkers did in CARLA:
  speed      walking speed while crossing (frames.csv: the walker's mean speed over its first 3 s on the road after
             it stepped out) against CROWD's median crossing speed and quartiles of the place
  mid-block  share of the crossings away from a marked crossing against CROWD's jaywalking share (crossings without
             a crosswalk or pedestrian light, crowd-jaywalking)
  wait       how long walkers waited at the kerb (CROWD does not measure this: reported, no target)
Gaps are reported as simulated - real, with the number of crossings behind each figure. No CARLA, no network.
"""
import argparse
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

CROSS = re.compile(r"^(\d+) (marked|mid-block) ([0-9.]+) m after ([0-9.]+) s")


def crowd_place(cfg):
    """(label, {speed_mps, speed_q25, speed_q75, jaywalk_share}) of the trial's place from CROWD's table."""
    from coupledcarla import crowdpeds
    d = json.loads((ROOT / "coupledcarla" / "data" / "crowd_pedestrians.json").read_text())
    bbox = (cfg.get("map") or {}).get("bbox")
    if not bbox:
        return None, {}
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    best = None
    for loc in d["localities"]:
        dist = math.hypot((loc["lat"] - lat) * 111.0, (loc["lon"] - lon) * 111.0 * math.cos(math.radians(lat)))
        if dist < 25 and (best is None or dist < best[0]):
            best = (dist, loc)
    country = d["countries"].get(best[1]["iso3"]) if best else None
    if country is None:
        try:
            iso3 = crowdpeds.country_of(lat, lon)          # optional helper; absent: world values
            country = d["countries"].get(iso3)
        except Exception:  # noqa: BLE001
            country = None
    out, label = {}, []
    for src, name in ((best[1] if best else None, "city"), (country, "country"), (d["world"], "world")):
        if not src:
            continue
        for k in ("speed_mps", "speed_q25", "speed_q75", "jaywalk_share"):
            if k not in out and src.get(k) is not None:
                out[k] = src[k]
                label.append(f"{k} from {name} {src.get('name', '')}".strip())
    return (best[1]["name"] if best else (country or {}).get("name", "world")), out


def trial_crossings(trial):
    """[{t, actor, kind, width, wait, speed}] of a trial's crowd crossings (speed from frames.csv)."""
    trial = Path(trial)
    rows = []
    with open(trial / "events.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if r["event"] != "crowd_crosses":
                continue
            m = CROSS.match(r["detail"])
            if m:
                rows.append({"t": float(r["timestamp"]), "actor": m.group(1), "kind": m.group(2),
                             "width": float(m.group(3)), "wait": float(m.group(4))})
    if not rows:
        return rows
    want = {r["actor"]: [] for r in rows}
    with open(trial / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if r["actor_id"] in want:
                want[r["actor_id"]].append((float(r["timestamp"]), float(r["pos_x"]), float(r["pos_y"])))
    for c in rows:
        pts = [p for p in want[c["actor"]] if c["t"] <= p[0] <= c["t"] + 3.0]
        if len(pts) > 10 and pts[-1][0] - pts[0][0] > 1.0:
            d = sum(math.dist(a[1:], b[1:]) for a, b in zip(pts, pts[1:]))
            c["speed"] = round(d / (pts[-1][0] - pts[0][0]), 3)
    return rows


def compare(trial):
    import yaml
    cfg = yaml.safe_load((Path(trial) / "config.yaml").read_text()) or {}
    rows = trial_crossings(trial)
    if not rows:
        return None
    place, real = crowd_place(cfg)
    speeds = sorted(r["speed"] for r in rows if r.get("speed"))
    sim = {"crossings": len(rows), "mid_block_share": round(sum(r["kind"] == "mid-block" for r in rows) / len(rows), 3),
           "wait_s_median": round(statistics.median(r["wait"] for r in rows), 1)}
    if speeds:
        q = statistics.quantiles(speeds, n=4) if len(speeds) >= 4 else [speeds[0], statistics.median(speeds), speeds[-1]]
        sim.update(speed_mps=round(statistics.median(speeds), 3), speed_q25=round(q[0], 3), speed_q75=round(q[2], 3),
                   speed_n=len(speeds))
    gaps = {k: round(sim[k] - real[k], 3) for k in ("speed_mps", "speed_q25", "speed_q75") if k in sim and k in real}
    if "jaywalk_share" in real:
        gaps["mid_block_vs_jaywalk_share"] = round(sim["mid_block_share"] - real["jaywalk_share"], 3)
    return {"trial": str(trial), "map": (cfg.get("map") or {}).get("name"), "crowd_place": place, "crowd": real,
            "simulated": sim, "gap": gaps}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("trials", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    out = []
    print("| Trial | Map | CROWD place | Crossings | Speed sim (q25-q75) | Speed CROWD (q25-q75) | Mid-block sim | "
          "Jaywalk CROWD | Wait median |")
    print("|---|---|---|---|---|---|---|---|---|")
    for t in a.trials:
        try:
            r = compare(t)
        except (OSError, ValueError) as err:
            print(f"<!-- {t}: {err} -->")
            continue
        if r is None:
            continue
        out.append(r)
        s, c = r["simulated"], r["crowd"]
        sp = f"{s.get('speed_mps', '-')} ({s.get('speed_q25', '-')}-{s.get('speed_q75', '-')})"
        cp = f"{c.get('speed_mps', '-')} ({c.get('speed_q25', '-')}-{c.get('speed_q75', '-')})"
        print(f"| {Path(t).parent.name}/{Path(t).name} | {r['map']} | {r['crowd_place']} | {s['crossings']} | {sp} | "
              f"{cp} | {s['mid_block_share']} | {c.get('jaywalk_share', '-')} | {s['wait_s_median']} s |")
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
