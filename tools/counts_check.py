"""Check how well SUMO's car flows match the open traffic counts on a map: a SUMO-only run of a trial (no CARLA).

    python tools/counts_check.py experiments/cities/new_york_midtown.yaml [--time "2026-11-17 08:00"]
        [--counts auto|off] [--cars-per-hour N|auto] [--out DIR]

Prepares the trial's traffic (coupledcarla.traffic.prepare) with the counts on or off, runs SUMO for the trial with
an edgeData output, and compares the simulated hourly flow on each counted edge (vehicles entering it after a
warm-up, scaled to an hour) with the counted flow: GEH per edge, the share under GEH 5, and the teleports and
vehicles SUMO could not insert (gridlock signs). Writes summary.json in the output folder.
"""
import argparse
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config, counts, traffic, transit  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--time", default=None, help='simulation.time, e.g. "2026-11-17 08:00" (default: as configured)')
    ap.add_argument("--counts", default="auto")
    ap.add_argument("--cars-per-hour", default=None)
    ap.add_argument("--warmup", type=float, default=120.0)
    ap.add_argument("--duration", type=float, default=None, help="trial length in s (default: the first trial's)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cfg = config.load(a.experiment)
    cfg = config.trial_config(cfg, cfg["trials"][0])
    if a.time:
        cfg["simulation"]["time"] = a.time
    cfg["traffic"]["counts"] = False if a.counts in ("off", "false") else a.counts
    if a.cars_per_hour:
        cfg["traffic"]["cars_per_hour"] = a.cars_per_hour if a.cars_per_hour == "auto" else float(a.cars_per_hour)
    map_dir = config.map_dir(cfg)
    duration = a.duration or cfg["trials"][0].get("duration_s", 300)
    work = Path(a.out or ROOT / "scratch" / f"counts_{cfg['map']['name']}_{a.counts}")
    work.mkdir(parents=True, exist_ok=True)
    sumocfg = traffic.prepare(cfg, map_dir, work, duration, log=print)
    end = int(duration) + 60
    # no FCD (10 Hz positions of every vehicle are the bridge's business), an edgeData output instead
    text = re.sub(r"\s*<(fcd-output|fcd-output\.geo|fcd-output\.signals|device\.fcd\.period)[^>]*/>", "",
                  sumocfg.read_text())
    (work / "edgedata.add.xml").write_text(
        f'<additional>\n  <edgeData id="check" file="edgedata.out.xml" begin="{a.warmup:g}" end="{end}"/>\n'
        '</additional>\n')
    if "<additional-files" in text:
        text = re.sub(r'<additional-files value="([^"]*)"', r'<additional-files value="\1,edgedata.add.xml"', text)
    else:
        text = text.replace("</input>", '  <additional-files value="edgedata.add.xml"/>\n  </input>')
    (work / "check.sumocfg").write_text(text)
    out = subprocess.run(["sumo", "-c", "check.sumocfg", "--end", str(end), "--lateral-resolution", "0.25",
                          "--no-step-log", "--duration-log.statistics", "--step-length", "0.1"],
                         cwd=work, capture_output=True, text=True)
    log = out.stdout + out.stderr
    (work / "sumo.log").write_text(log)

    # the counted flows of this trial, as traffic.prepare used them
    bbox = json.loads((map_dir / "meta.json").read_text())["bbox"]
    when = transit.trial_time(cfg["simulation"].get("time"), ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2),
                              map_dir / "real")
    tc = dict(cfg["traffic"], counts="auto")
    _, flows, _, summ = counts.plan(tc, map_dir, bbox, when, log=lambda m: None)
    flows = flows or {}
    sim = {}
    for iv in ET.parse(work / "edgedata.out.xml").getroot().iter("interval"):
        for e in iv.iter("edge"):
            sim[e.get("id")] = float(e.get("entered", 0)) + float(e.get("departed", 0))   # what a counter sees
    span = end - a.warmup
    rows = []
    for eid, c in sorted(flows.items()):
        m = sim.get(eid, 0.0) * 3600.0 / span
        rows.append({"edge": eid, "counted": round(c), "simulated": round(m), "geh": round(counts.geh(m, c), 2)})
    gehs = [r["geh"] for r in rows]

    def stat(key):
        m = re.search(key + r":?\s*([\d.]+)", log)
        return float(m.group(1)) if m else None
    tele = re.search(r"Teleports: (\d+)(.*)", log)
    import sumolib
    net = sumolib.net.readNet(str(map_dir / "map.net.xml"), withInternal=False)
    lengths = [sum(net.getEdge(x).getLength() for x in r.get("edges").split())
               for r in ET.parse(work / "car.rou.xml").getroot().iter("route")]
    summary = {
        "experiment": cfg["name"], "counts": a.counts, "cars_per_hour": cfg["traffic"]["cars_per_hour"],
        "when": when.isoformat(), "plan": summ, "counted_edges": len(rows),
        "geh_under_5": round(sum(1 for g in gehs if g < 5) / len(gehs), 3) if gehs else None,
        "geh_median": sorted(gehs)[len(gehs) // 2] if gehs else None,
        "counted_veh_h": sum(r["counted"] for r in rows), "simulated_veh_h": sum(r["simulated"] for r in rows),
        "inserted": stat("Inserted"), "running": stat("Running"), "waiting": stat("Waiting"),
        "teleports": int(tele.group(1)) if tele else 0, "teleport_kinds": tele.group(2).strip() if tele else "",
        "car_routes": len(lengths), "car_mean_route_m": round(sum(lengths) / len(lengths)) if lengths else None,
        "mean_speed": stat("Speed"), "edges": rows,
    }
    (work / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "edges"}, indent=1))
    for r in rows:
        print(f"  {r['edge']:>10} counted {r['counted']:5d}  simulated {r['simulated']:5d}  GEH {r['geh']:5.1f}")


if __name__ == "__main__":
    main()
