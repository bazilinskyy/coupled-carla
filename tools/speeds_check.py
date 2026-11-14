"""Check SUMO's car speeds against the open speed measurements on a map: SUMO-only runs (no CARLA), before and after.

    python tools/speeds_check.py experiments/cities/portland_or.yaml [--time "2026-11-17 12:00"]
        [--duration 900] [--warmup 300] [--out DIR]

Runs the trial's traffic (coupledcarla.traffic.prepare) three times: as before (netconvert's limits, speedFactor
N(1.0, 0.1)), with the urban default limits only, and with the limits and the measured speed factor
(speeds.plan). A spot site (PBOT count, detector) gets an induction loop across its edge at its place: the
time-mean speed of the cars passing it, as the real detector measures; a segment site (NDW floating car data) is
compared with the space-mean speed on its edge (edgeData). Per run: the mean absolute percentage error (MAPE) and
the median ratio simulated / measured over the sites with at least 5 simulated cars, for the trial's hour and day
type. Writes summary.json in the output folder.

--calibrate: a SUMO car slows down at the junctions on its edge, so simulated spot speeds sit below the desired
speed that the measured free-flow speed gives: the speed factor that makes the simulated speeds match the measured
ones (median ratio 1) is found by a secant step between the "limits" run (factor 1) and the "after" run (the
measured factor), checked in a fourth run ("calibrated"), and with --write stored in
coupledcarla/data/speed_calibration.json, which speeds.plan then uses for this map.
"""
import argparse
import json
import re
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config, counts, speeds, traffic, transit  # noqa: E402

CALIBRATION = ROOT / "coupledcarla" / "data" / "speed_calibration.json"

RUNS = {"before": {"speed_limits": False, "speed_factor": False},
        "limits": {"speed_limits": "auto", "speed_factor": False},
        "after": {"speed_limits": "auto", "speed_factor": "auto"}}


def run(cfg, map_dir, work, duration, warmup, matched, net):
    work.mkdir(parents=True, exist_ok=True)
    sumocfg = traffic.prepare(cfg, map_dir, work, duration, log=lambda m: print("   ", m))
    end = int(duration) + 60
    text = re.sub(r"\s*<(fcd-output|fcd-output\.geo|fcd-output\.signals|device\.fcd\.period)[^>]*/>", "",
                  sumocfg.read_text())
    rows = ['<additional>', f'  <edgeData id="check" file="edgedata.out.xml" begin="{warmup:g}" end="{end}"/>']
    loops = {}
    for site, eid, _, _ in matched:
        if site.get("kind") != "spot":
            continue
        x, y = net.convertLonLat2XY(site["lon"], site["lat"])
        for lane in net.getEdge(eid).getLanes():
            if not lane.allows("passenger"):
                continue
            pos, _ = lane.getClosestLanePosAndDist((x, y))
            pos = min(max(pos, 1.0), lane.getLength() - 1.0)
            lid = f"loop{len(loops)}"
            loops[lid] = site["id"]
            rows.append(f'  <inductionLoop id="{lid}" lane="{lane.getID()}" pos="{pos:.1f}" period="{end}" '
                        f'file="loops.out.xml"/>')
    rows.append("</additional>")
    (work / "check.add.xml").write_text("\n".join(rows) + "\n")
    if "<additional-files" in text:
        text = re.sub(r'<additional-files value="([^"]*)"', r'<additional-files value="\1,check.add.xml"', text)
    else:
        text = text.replace("</input>", '  <additional-files value="check.add.xml"/>\n  </input>')
    (work / "check.sumocfg").write_text(text)
    for seed in (None, 1, 2):        # SUMO (1.2x, sublane model) now and then dies mid-run: run it again
        out = subprocess.run(["sumo", "-c", "check.sumocfg", "--end", str(end), "--lateral-resolution", "0.25",
                              "--no-step-log", "--duration-log.statistics", "--step-length", "0.1"]
                             + (["--seed", str(seed)] if seed else []), cwd=work, capture_output=True, text=True)
        (work / "sumo.log").write_text(out.stdout + out.stderr)
        if out.returncode == 0:
            break
        print(f"    SUMO exited with {out.returncode}; again with --seed {(seed or 0) + 1}")
    edge = {}
    for iv in ET.parse(work / "edgedata.out.xml").getroot().iter("interval"):
        for e in iv.iter("edge"):
            if e.get("speed") is not None:
                edge[e.get("id")] = (float(e.get("speed")) * 3.6, float(e.get("entered", 0)))
    spot = {}
    if loops and (work / "loops.out.xml").exists():
        for iv in ET.parse(work / "loops.out.xml").getroot().iter("interval"):
            sid = loops.get(iv.get("id"))
            n, v = float(iv.get("nVehContrib", 0)), float(iv.get("speed", -1))
            if sid and n > 0 and v >= 0:
                tot, cnt = spot.get(sid, (0.0, 0.0))
                spot[sid] = (tot + v * 3.6 * n, cnt + n)
    sim = {}
    for site, eid, _, _ in matched:
        if site.get("kind") == "spot" and site["id"] in spot:
            tot, n = spot[site["id"]]
            sim[site["id"]] = (tot / n, n)
        elif site.get("kind") == "segment" and eid in edge:
            sim[site["id"]] = edge[eid]
    m = re.search(r"Speed:\s*([\d.]+)", out.stdout + out.stderr)
    return sim, (float(m.group(1)) if m else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--time", default=None, help='simulation.time, e.g. "2026-11-17 12:00" (default: as configured)')
    ap.add_argument("--duration", type=float, default=900.0)
    ap.add_argument("--warmup", type=float, default=300.0)
    ap.add_argument("--runs", default="before,limits,after")
    ap.add_argument("--calibrate", action="store_true", help="fit the factor in SUMO (needs the limits and after runs)")
    ap.add_argument("--write", action="store_true", help="store the calibrated factor in data/speed_calibration.json")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cfg = config.load(a.experiment)
    cfg = config.trial_config(cfg, cfg["trials"][0])
    if a.time:
        cfg["simulation"]["time"] = a.time
    map_dir = config.map_dir(cfg)
    bbox = json.loads((map_dir / "meta.json").read_text())["bbox"]
    when = transit.trial_time(cfg["simulation"].get("time"), ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2),
                              map_dir / "real")
    day, hour = transit.day_type(when), when.hour
    data = speeds.load(map_dir, bbox)
    if not data:
        sys.exit("no open speed measurements cover this map")
    import sumolib
    net, edges = counts.net_edges(map_dir)
    geo = sumolib.net.readNet(str(map_dir / "geo.net.xml"), withInternal=False, withPedestrianConnections=False)
    limits = speeds.implicit_limits(net, geo, speeds.country_of(cfg, bbox))
    matched = speeds.match_sites(data["sites"], net, edges, limits)
    meas = {s["id"]: speeds.measured(s, day, hour) for s, *_ in matched}
    out_dir = Path(a.out or ROOT / "scratch" / f"speeds_{cfg['map']['name']}")
    summary = {"experiment": cfg["name"], "source": data["source"], "when": when.isoformat(), "day": day,
               "hour": hour, "sites": len(data["sites"]), "matched": len(matched),
               "with_measurement": sum(1 for v in meas.values() if v), "runs": {}}
    print(f"{cfg['name']}: {data['what']}: {len(matched)} of {len(data['sites'])} sites matched, "
          f"{summary['with_measurement']} measured at {day} {hour:02d}:00")
    per_site = {sid: {"measured": v} for sid, v in meas.items()}
    names = a.runs.split(",")
    direct = speeds.fit(matched, data["source"])
    if a.calibrate and not direct and len(matched) >= speeds.MIN_SITES:
        direct = {"mean": 0.8, "sd": 0.1, "min": 0.55, "max": 1.05}   # segment speeds give no factor: a probe
    if a.calibrate and direct:
        names += [n for n in ("limits", "after") if n not in names] + ["calibrated"]
    runs = dict(RUNS)
    if direct:      # the measured factor itself (not a calibration already stored for the map)
        runs["after"] = {"speed_limits": "auto", "speed_factor": speeds.distribution(direct)}
    for name in names:
        if name == "calibrated":
            rl, ra = summary["runs"]["limits"]["median_sim_over_measured"], summary["runs"]["after"]["median_sim_over_measured"]
            f0, f1 = speeds.DEFAULT["mean"], direct["mean"]
            fc = f1 if ra == rl else f1 + (1.0 - ra) * (f1 - f0) / (ra - rl)
            fc = round(min(1.5, max(0.5, fc)), 3)
            sd = round(direct["sd"] * fc / f1, 3)
            cal = {"mean": fc, "sd": sd, "min": round(max(0.4, fc - 2.5 * sd), 2), "max": round(min(2.0, fc + 2.5 * sd), 2)}
            runs["calibrated"] = {"speed_limits": "auto", "speed_factor": speeds.distribution(cal)}
            summary["calibrated"] = cal
        c = json.loads(json.dumps(cfg))
        c["traffic"].update(runs[name])
        print(f"run {name}: {runs[name]}")
        sim, mean_speed = run(c, map_dir, out_dir / name, a.duration, a.warmup, matched, net)
        errs, ratios = [], []
        for sid, (v, n) in sim.items():
            per_site.setdefault(sid, {})[name] = round(v, 1)
            m = meas.get(sid)
            if m and n >= 5:
                errs.append(abs(v - m) / m)
                ratios.append(v / m)
        res = {"sites": len(errs), "mape": round(100 * statistics.mean(errs), 1) if errs else None,
               "median_sim_over_measured": round(statistics.median(ratios), 3) if ratios else None,
               "mean_speed_kmh": round(mean_speed * 3.6, 1) if mean_speed else None}
        summary["runs"][name] = res
        print(f"  {name}: MAPE {res['mape']}% over {res['sites']} sites, simulated / measured median "
              f"{res['median_sim_over_measured']}, network mean speed {res['mean_speed_kmh']} km/h")
    if a.write and summary.get("calibrated") and summary["runs"].get("calibrated", {}).get("mape") is not None:
        book = json.loads(CALIBRATION.read_text()) if CALIBRATION.exists() else {}
        book[cfg["map"]["name"]] = {**summary["calibrated"], "source": data["source"], "when": when.isoformat(),
                                    "sites": summary["runs"]["calibrated"]["sites"],
                                    "measured_factor": direct["mean"],
                                    "mape": {k: v["mape"] for k, v in summary["runs"].items()},
                                    "sim_over_measured": {k: v["median_sim_over_measured"]
                                                          for k, v in summary["runs"].items()}}
        CALIBRATION.write_text(json.dumps(book, indent=1, sort_keys=True) + "\n")
        print(f"calibrated speed factor for {cfg['map']['name']} written to {CALIBRATION}")
    summary["per_site"] = per_site
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps({k: v for k, v in summary.items() if k != "per_site"}, indent=1))


if __name__ == "__main__":
    main()
