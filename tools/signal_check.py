"""Traffic lights before and after: a SUMO-only run (no CARLA) per traffic.signals mode, measured at the signals.

    python tools/signal_check.py experiments/eindhoven_centre.yaml [--modes sumo,dutch] [--duration 300]
        [--warmup 60] [--seed 42] [--set traffic.cars_per_hour=2400] [--out DIR]

For each mode: traffic.prepare (the trial's demand, the same seed), SUMO through TraCI at the bridge's step and
options (drivers.SUMO_ARGS: sublane model, junction collisions counted), and per signalised approach lane every
second its halting vehicles. Reports per mode:
- queues: mean and 95th percentile of halting vehicles per signalised lane, the longest queue (vehicles);
- waits: per vehicle class, the mean waiting time and time loss of the trips that ended (tripinfo), and the mean
  wait per stop at a red light;
- red-light stops: vehicles coming to a stop within 60 m of a stop line whose light is red or amber, and red-light
  entries (a vehicle entering a junction over a red link);
- SUMO's teleports, collisions and emergency brakes;
- green conflicts: link pairs of a junction a phase shows green at once while they cross or merge and neither
  yields (junctions.green_conflicts on the net SUMO runs: 0 is wanted).
Writes summary.json per mode and compare.json in the output folder and prints the comparison.

    python tools/signal_check.py --trial logs/<experiment>/<trial> [...]

measures the same from CARLA trials' SUMO outputs (with simulation.traffic_lights.log: true also whether the drawn
heads showed SUMO's states).
"""
import argparse
import json
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config, drivecheck, junctions, traffic  # noqa: E402


def _set(cfg, item):
    key, _, value = item.partition("=")
    d = cfg
    parts = key.split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = yaml.safe_load(value)


def green_conflicts(net_file):
    net = junctions.read_net(net_file)
    nodes = {net.edges[c["from"]]["to"] for c in net.conns if c.get("tl") and not c["from"].startswith(":")}
    pairs = sum(len(junctions.green_conflicts(net, j)) for j in sorted(nodes))
    return {"signalised_nodes": len(nodes), "lights": len(net.tls), "green_conflicts": pairs}


def run_mode(a, mode, out):
    cfg = config.load(a.experiment)
    cfg = config.trial_config(cfg, cfg["trials"][0])
    if a.time:
        cfg["simulation"]["time"] = a.time
    for item in a.set or []:
        _set(cfg, item)
    cfg["traffic"]["seed"] = a.seed
    cfg["traffic"]["signals"] = mode
    work = out / mode
    work.mkdir(parents=True, exist_ok=True)
    map_dir = config.map_dir(cfg)
    sumocfg = traffic.prepare(cfg, map_dir, work, a.duration, log=print)
    text = re.sub(r"\s*<(fcd-output[^>]*|device\.fcd\.period[^>]*)/>", "", sumocfg.read_text())
    (work / "check.sumocfg").write_text(text)
    net_file = re.search(r'<net-file value="([^"]+)"', text).group(1)
    static = green_conflicts(net_file)

    import sumolib
    import traci
    from coupledcarla.drivers import SUMO_ARGS
    bikes, bus = set(cfg["traffic"]["bike_types"]), cfg["traffic"]["bus_type"]

    def kind_of(type_id):
        t = type_id.split("@")[0]
        return "bike" if t in bikes or "bike" in t else "bus" if t == bus or "bus" in t else "car"
    cmd = [sumolib.checkBinary("sumo"), "-c", str(work / "check.sumocfg"), "--step-length", str(a.step),
           *SUMO_ARGS, "--collision-output", str(work / "collisions.xml"),
           "--statistic-output", str(work / "statistics.xml"), "--tripinfo-output", str(work / "tripinfo.xml"),
           "--end", str(a.duration), "--no-step-log", "--log", str(work / "sumo.log")]
    t0 = time.time()
    traci.start(cmd)
    tls = traci.trafficlight.getIDList()
    lanes = sorted({ln for t in tls for ln in traci.trafficlight.getControlledLanes(t)})
    queues = {ln: [] for ln in lanes}
    VAR_L, VAR_S = traci.constants.VAR_LANE_ID, traci.constants.VAR_SPEED
    prev_lane, moving, stops, red_wait = {}, {}, [], {}
    reds, entries, ambers = [], 0, 0
    every = max(1, int(round(1.0 / a.step)))
    n = 0
    while traci.simulation.getTime() < a.duration:
        traci.simulationStep()
        n += 1
        now = traci.simulation.getTime()
        for vid in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(vid, [VAR_L, VAR_S])
        for vid in traci.simulation.getArrivedIDList():
            prev_lane.pop(vid, None)
            moving.pop(vid, None)
        res = traci.vehicle.getAllSubscriptionResults()
        for vid, r in res.items():
            lane, v = r.get(VAR_L, ""), r.get(VAR_S, 0.0)
            prev = prev_lane.get(vid)
            prev_lane[vid] = lane
            if prev and lane.startswith(":") and not prev.startswith(":") and now > a.warmup:
                st = drivecheck.link_state_on_entry(traci.lane.getLinks(prev, True), lane)
                if st is not None and st in drivecheck.TLS_STATES:
                    entries += 1
                    if drivecheck.is_red(st):
                        reds.append((round(now, 2), vid, prev, lane))
                    elif st in ("y", "Y"):
                        ambers += 1
            if n % every:
                continue
            was = moving.get(vid, True)
            if v > 1.0:
                moving[vid] = True
                if vid in red_wait:
                    stops.append(now - red_wait.pop(vid))
            elif v < 0.1 and was:
                moving[vid] = False
                if now > a.warmup and not lane.startswith(":"):
                    nxt = traci.vehicle.getNextTLS(vid)
                    if nxt and nxt[0][2] < 60.0 and nxt[0][3] in "rRyYs":
                        red_wait[vid] = now
        if n % every == 0 and now > a.warmup:
            for ln in lanes:
                queues[ln].append(traci.lane.getLastStepHaltingNumber(ln))
    traci.close()
    wall = time.time() - t0

    allq = [q for v in queues.values() for q in v]
    trips = {}
    if (work / "tripinfo.xml").exists():
        for el in drivecheck.ET.parse(work / "tripinfo.xml").getroot().iter("tripinfo"):
            if float(el.get("depart")) < a.warmup:
                continue
            k = kind_of(el.get("vType", ""))
            d = trips.setdefault(k, {"n": 0, "wait": 0.0, "loss": 0.0, "duration": 0.0})
            d["n"] += 1
            d["wait"] += float(el.get("waitingTime"))
            d["loss"] += float(el.get("timeLoss"))
            d["duration"] += float(el.get("duration"))
    for d in trips.values():
        for f in ("wait", "loss", "duration"):
            d[f] = round(d[f] / max(1, d["n"]), 1)
    log = (work / "sumo.log").read_text(errors="replace")
    col = drivecheck.collisions(work / "collisions.xml")
    out_d = {
        "mode": mode, "duration_s": a.duration, "warmup_s": a.warmup, "seed": a.seed, "wall_s": round(wall, 1),
        "net": static, "lights": len(tls), "signalised_lanes": len(lanes),
        "queue_mean_veh": round(sum(allq) / max(1, len(allq)), 3),
        "queue_p95_veh": drivecheck.pct(allq, 95) if allq else 0,
        "queue_max_veh": max(allq) if allq else 0,
        "queue_lane_mean_max_veh": round(max((sum(v) / len(v) for v in queues.values() if v), default=0), 2),
        "trips": trips,
        "red_light_stops": len(stops) + len(red_wait),
        "red_light_stop_wait_mean_s": round(sum(stops) / len(stops), 1) if stops else None,
        "red_light_entries": len(reds), "amber_entries": ambers, "tls_entries": entries,
        "red_light_examples": reds[:5],
        "teleports": len(re.findall(r"Teleporting vehicle", log)),
        "emergency_braking": len(re.findall(r"emergency braking", log)),
        "collisions": {k: v for k, v in col.items() if k != "rows"},
        "sumo": drivecheck.sumo_statistics(work / "statistics.xml"),
    }
    (work / "summary.json").write_text(json.dumps(out_d, indent=1))
    return out_d


def analyse_trial(trial, warmup=60.0):
    """The same measures from a CARLA trial's SUMO outputs (sumo/: trial.sumocfg, sumo_fcd.xml.gz every 0.1 s,
    sumo_tripinfo.xml, tls_states.xml, sumo.log), plus the participants' collisions (summary.json) and whether the
    drawn heads showed SUMO's states (signal_heads.py)."""
    import gzip
    from bisect import bisect_right
    import signal_heads
    trial = Path(trial)
    sd = trial / "sumo"
    text = (sd / "trial.sumocfg").read_text()
    net = junctions.read_net(re.search(r'<net-file value="([^"]+)"', text).group(1))
    lane_links = {}                     # signalised approach lane -> [(tls, link index)]
    for c in net.conns:
        if c.get("tl") and not c["from"].startswith(":"):
            lane_links.setdefault(f"{c['from']}_{c['fromLane']}", []).append((c["tl"], int(c["linkIndex"])))
    rec = signal_heads.sumo_record(sd / "tls_states.xml") if (sd / "tls_states.xml").exists() else {}

    def red(lane, t):
        for tls, k in lane_links[lane]:
            times, states = rec.get(tls, ([], []))
            i = bisect_right(times, t + 1e-6) - 1
            if i < 0 or k >= len(states[i]) or states[i][k] not in "rRyYs":
                return False
        return True
    queues, halted, stops = {}, {}, 0
    with gzip.open(sd / "sumo_fcd.xml.gz", "rt") as f:
        t = None
        for ev, el in ET.iterparse(f, events=("start", "end")):
            if ev == "start" and el.tag == "timestep":
                t = float(el.get("time"))
                if abs(t - round(t)) > 1e-6:
                    t = None                # one sample a second
                count = {}
            elif ev == "end" and el.tag == "vehicle" and t is not None and t >= warmup:
                lane, v = el.get("lane", ""), float(el.get("speed", 0))
                if lane in lane_links:
                    vid = el.get("id")
                    if v < 0.1:
                        count[lane] = count.get(lane, 0) + 1
                        near = net.lanes[lane]["length"] - float(el.get("pos", 0)) < 60.0
                        if not halted.get(vid) and near and red(lane, t):
                            stops += 1
                        halted[vid] = True
                    elif v > 1.0:
                        halted[vid] = False
            elif ev == "end" and el.tag == "timestep":
                if t is not None and t >= warmup:
                    for ln in lane_links:
                        queues.setdefault(ln, []).append(count.get(ln, 0))
                el.clear()
    allq = [q for v in queues.values() for q in v]
    trips = {}
    if (sd / "sumo_tripinfo.xml").exists():
        for el in ET.parse(sd / "sumo_tripinfo.xml").getroot().iter("tripinfo"):
            if float(el.get("depart")) < warmup:
                continue
            tid = el.get("vType", "")
            k = "bike" if "bike" in tid or "gazelle" in tid or "diamondback" in tid or "bh." in tid else \
                "bus" if "bus" in tid or "fusorosa" in tid else "car"
            d = trips.setdefault(k, {"n": 0, "wait": 0.0, "loss": 0.0})
            d["n"] += 1
            d["wait"] += float(el.get("waitingTime"))
            d["loss"] += float(el.get("timeLoss"))
    for d in trips.values():
        d["wait"], d["loss"] = round(d["wait"] / max(1, d["n"]), 1), round(d["loss"] / max(1, d["n"]), 1)
    log = (sd / "sumo.log").read_text(errors="replace") if (sd / "sumo.log").exists() else ""
    summ = json.loads((trial / "summary.json").read_text()) if (trial / "summary.json").exists() else {}
    heads = signal_heads.compare(trial) if (trial / "signal_heads.csv").exists() and rec else None
    return {"trial": str(trial), "lights": len(net.tls), "signalised_lanes": len(lane_links),
            "queue_mean_veh": round(sum(allq) / max(1, len(allq)), 3),
            "queue_p95_veh": drivecheck.pct(allq, 95) if allq else 0, "queue_max_veh": max(allq) if allq else 0,
            "red_light_stops": stops, "trips": trips,
            "teleports": len(re.findall(r"Teleporting vehicle", log)),
            "sumo_collisions": len(re.findall(r"collision with", log)),
            "emergency_braking": len(re.findall(r"emergency braking", log)),
            "participant_collisions": summ.get("collisions"), "sumo_vehicles_spawned":
            summ.get("sumo_vehicles_spawned"), "rtf": summ.get("rtf"),
            "heads": {k: heads[k] for k in ("samples", "agree", "share", "agree_at_switch_step", "heads_lights")}
            if heads else None}


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--trial":     # a CARLA trial's outputs
        sys.path.insert(0, str(ROOT / "tools"))
        print(json.dumps({t: analyse_trial(t) for t in sys.argv[2:]}, indent=1))
        return
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--modes", default="sumo,dutch")
    ap.add_argument("--time")
    ap.add_argument("--set", action="append")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--warmup", type=float, default=60.0)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out")
    a = ap.parse_args()
    out = Path(a.out or ROOT / "scratch" / f"signals_{Path(a.experiment).stem}")
    out.mkdir(parents=True, exist_ok=True)
    res = {m: run_mode(a, m, out) for m in a.modes.split(",")}
    keys = ["lights", "signalised_lanes", "queue_mean_veh", "queue_p95_veh", "queue_max_veh",
            "queue_lane_mean_max_veh", "red_light_stops", "red_light_stop_wait_mean_s", "red_light_entries",
            "amber_entries", "teleports", "emergency_braking"]
    table = {k: {m: r.get(k) for m, r in res.items()} for k in keys}
    table["green_conflicts"] = {m: r["net"]["green_conflicts"] for m, r in res.items()}
    table["collisions"] = {m: r["collisions"] for m, r in res.items()}
    table["trips"] = {m: r["trips"] for m, r in res.items()}
    (out / "compare.json").write_text(json.dumps(table, indent=1))
    print(json.dumps(table, indent=1))


if __name__ == "__main__":
    main()
