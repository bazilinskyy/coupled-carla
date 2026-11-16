"""How realistic is a trial's SUMO traffic? A SUMO-only run (no CARLA) with the bridge's own SUMO options, measured.

    python tools/traffic_check.py experiments/eindhoven_centre.yaml [--clip eindhoven_rush_hour] [--duration 300]
        [--time "2026-06-17 17:15"] [--set traffic.cars_per_hour=3600] [--out DIR] [--seed 42]

Prepares the trial's traffic (coupledcarla.traffic.prepare), runs SUMO through TraCI at the bridge's step (0.02 s,
sublane model, junction collision checks) with collisions only warned (so each one is counted, not hidden by a
teleport), and reports (coupledcarla/drivecheck.py): SUMO's teleports, collisions, emergency braking; red-light
entries; vehicles standing inside junctions; accelerations, jerk, time gaps and free-flow speed / limit per vehicle
class; lane changes per vehicle-km. Writes summary.json in the output folder and prints it.
"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config, drivecheck, traffic  # noqa: E402


def _set(cfg, item):
    key, _, value = item.partition("=")
    d = cfg
    parts = key.split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = yaml.safe_load(value)


def experiment(a, work):
    if a.clip:
        sys.path.insert(0, str(ROOT / "tools"))
        import make_videos
        clip = make_videos.find_clip(make_videos.load_clips(), a.clip)
        exp = make_videos.derive_experiment(clip, {"port": 2000})
        exp.pop("film", None)
        path = work / "experiment.yaml"
        path.write_text(yaml.safe_dump(exp, sort_keys=False))
        cfg = config.load(path)
        cfg["_root"] = str(ROOT)
    else:
        cfg = config.load(a.experiment)
    cfg = config.trial_config(cfg, cfg["trials"][0])
    if a.time:
        cfg["simulation"]["time"] = a.time
    for item in a.set or []:
        _set(cfg, item)
    if a.seed is not None:
        cfg["traffic"]["seed"] = a.seed
    return cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment", nargs="?")
    ap.add_argument("--clip", help="a clip of experiments/videos/clips.yaml (its experiment, time and settings)")
    ap.add_argument("--time")
    ap.add_argument("--set", action="append", help="key.path=value (YAML), e.g. traffic.cars_per_hour=3600")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--warmup", type=float, default=60.0, help="seconds not measured at the start")
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--seed", type=int)
    ap.add_argument("--out")
    ap.add_argument("--sumo-args", help="SUMO options instead of the bridge's (drivers.SUMO_ARGS), one string")
    a = ap.parse_args()
    if not (a.experiment or a.clip):
        ap.error("an experiment or --clip")
    work = Path(a.out or ROOT / "scratch" / f"traffic_{a.clip or Path(a.experiment).stem}")
    work.mkdir(parents=True, exist_ok=True)
    cfg = experiment(a, work)
    map_dir = config.map_dir(cfg)
    t0 = time.time()
    sumocfg = traffic.prepare(cfg, map_dir, work, a.duration, log=print)
    text = re.sub(r"\s*<(fcd-output\.geo|fcd-output\.signals)[^>]*/>", "", sumocfg.read_text())
    text = text.replace("sumo_fcd.xml.gz", "check_fcd.xml.gz")
    (work / "check.sumocfg").write_text(text)

    import sumolib
    import traci
    net_file = re.search(r'<net-file value="([^"]+)"', text).group(1)
    net = sumolib.net.readNet(net_file, withInternal=True, withPedestrianConnections=False)
    lane_speed = {ln.getID(): ln.getSpeed() for e in net.getEdges() for ln in e.getLanes()}
    bikes, bus = set(cfg["traffic"]["bike_types"]), cfg["traffic"]["bus_type"]

    def kind_of(type_id):
        t = type_id.split("@")[0]
        return "bike" if t in bikes or "bike" in t else "bus" if t == bus or "bus" in t else "car"

    from coupledcarla.drivers import SUMO_ARGS       # the bridge's options: every collision counted, not teleported
    cmd = [sumolib.checkBinary("sumo"), "-c", str(work / "check.sumocfg"), "--step-length", str(a.step),
           *(a.sumo_args.split() if a.sumo_args is not None else SUMO_ARGS),
           "--collision-output", str(work / "collisions.xml"),
           "--statistic-output", str(work / "statistics.xml"),
           "--lanechange-output", str(work / "lanechanges.xml"),
           "--fcd-output.acceleration", "--fcd-output.max-leader-distance", "100",
           "--fcd-output.attributes", "id,type,speed,lane,acceleration,leaderID,leaderGap,x,y,angle,pos,posLat",
           "--end", str(a.duration), "--no-step-log", "--log", str(work / "sumo.log")]
    traci.start(cmd)
    lanes, reds, entries, ambers = {}, [], 0, 0
    VAR = traci.constants.VAR_LANE_ID
    while traci.simulation.getTime() < a.duration:
        traci.simulationStep()
        now = traci.simulation.getTime()
        for vid in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(vid, [VAR])
        for vid in traci.simulation.getArrivedIDList():
            lanes.pop(vid, None)
        for vid, res in traci.vehicle.getAllSubscriptionResults().items():
            lane = res.get(VAR, "")
            prev = lanes.get(vid)
            lanes[vid] = lane
            if prev and lane.startswith(":") and not prev.startswith(":") and now > a.warmup:
                state = drivecheck.link_state_on_entry(traci.lane.getLinks(prev, True), lane)
                if state is None or state not in drivecheck.TLS_STATES:
                    continue      # not a signalised link
                entries += 1
                if drivecheck.is_red(state):
                    reds.append((round(now, 2), vid, prev, lane))
                elif state in ("y", "Y"):
                    ambers += 1
    traci.close()
    wall = time.time() - t0

    stats = drivecheck.FcdStats(lane_speed, kind_of)
    from coupledcarla import drivers
    vt = {}
    for tf in [p for p in work.glob("*.xml") if p.name.endswith((".rou.xml", "types.add.xml"))]:
        for el in drivecheck.ET.parse(tf).getroot().iter("vType"):
            vt[el.get("id")] = (float(el.get("length", 5.0)), float(el.get("width", 1.8)), el.get("vClass", ""))

    def size_of(type_id):           # CARLA's bodies (what is seen), large: buses and anything over 7 m
        t = type_id.split("@")[0].replace("ego:", "")
        length, width, vclass = vt.get(type_id) or vt.get(t) or (4.5, 1.8, "")
        width = drivers.BODY_WIDTH.get(t, width)
        return length, width, vclass == "bus" or length > 7.0

    bodies = drivecheck.BodyStats(size_of)
    step_t, step_recs = None, []
    for t, rec in drivecheck.iter_fcd(work / "check_fcd.xml.gz"):
        if t >= a.warmup:
            stats.add(t, rec)
            if t != step_t:
                if step_recs:
                    bodies.add_step(step_t, step_recs)
                step_t, step_recs = t, []
            step_recs.append(rec)
    if step_recs:
        bodies.add_step(step_t, step_recs)
    fcd = stats.result()
    fcd["bodies"] = bodies.result()
    lc = sum(1 for _ in drivecheck.ET.parse(work / "lanechanges.xml").getroot().iter("change")) \
        if (work / "lanechanges.xml").exists() else None
    log = (work / "sumo.log").read_text(errors="replace")
    out = {
        "experiment": a.clip or a.experiment, "duration_s": a.duration, "warmup_s": a.warmup, "step": a.step,
        "seed": cfg["traffic"]["seed"], "wall_s": round(wall, 1),
        "sumo": drivecheck.sumo_statistics(work / "statistics.xml"),
        "collisions": {k: v for k, v in drivecheck.collisions(work / "collisions.xml").items() if k != "rows"},
        "log_emergency_braking": len(re.findall(r"emergency braking", log)),
        "log_teleports": len(re.findall(r"Teleporting vehicle", log)),
        "tls_entries": entries, "red_light_entries": len(reds), "amber_entries": ambers,
        "red_light_examples": reds[:10],
        "lane_changes": lc,
        "fcd": fcd,
    }
    (work / "summary.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
