"""SUMO-only check of a demo clip's traffic (no CARLA): the clip's experiment as tools/make_videos.py derives it,
its SUMO demand (traffic.prepare), SUMO run headless to the end of the clip, and the vehicles by class within
--radius m of the clip's place (its first participant's spawn, route start or drone flight, or --centre) during the
clip's window, against the place's CROWD mix (crowdtraffic.py).

    python tools/traffic_mix_check.py twin_eindhoven [--radius 200] [--root <another checkout>] [--json out.json]

--root runs another checkout's code, clips and experiments (e.g. main before a change) on this checkout's maps/.
"""
import argparse
import collections
import gzip
import json
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]


def _km(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    return 6371000.0 * math.hypot(x, math.radians(lat2 - lat1))


def clip_centre(tc):
    for p in tc.get("participants") or []:
        bot = (p.get("bot") or {}).get("path")
        if p.get("kind") == "drone" and bot:
            return sum(q[0] for q in bot) / len(bot), sum(q[1] for q in bot) / len(bot)
        if (p.get("route") or [])[:1]:
            return tuple(map(float, p["route"][0][:2]))
        sp = p.get("spawn") or {}
        if "lat" in sp:
            return float(sp["lat"]), float(sp["lon"])
    bb = tc["map"]["bbox"]
    return (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2


def classify(vtype):
    sys.path.insert(0, str(HERE / "tools"))
    from traffic_seen import vehicle_class
    return vehicle_class(vtype.split("@")[0].split("#")[0] + ("taxi" if "#taxi" in vtype else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clip")
    ap.add_argument("--root", default=str(HERE))
    ap.add_argument("--radius", type=float, default=200.0)
    ap.add_argument("--centre", help="lat,lon")
    ap.add_argument("--work", default=None)
    ap.add_argument("--json")
    a = ap.parse_args()
    root = Path(a.root).resolve()
    sys.path.insert(0, str(root))
    sys.path.insert(1, str(root / "tools"))
    import yaml
    import make_videos as mv
    from coupledcarla import config, traffic
    clip = mv.find_clip(mv.load_clips(root / "experiments" / "videos" / "clips.yaml"), a.clip)
    exp = mv.derive_experiment(clip, {}, root=root)
    work = Path(a.work or tempfile.mkdtemp(prefix=f"mix_{a.clip}_"))
    work.mkdir(parents=True, exist_ok=True)
    ef = root / "experiments" / f"_mixcheck_{a.clip}.yaml"
    ef.write_text(yaml.safe_dump(exp, sort_keys=False, allow_unicode=True))
    try:
        cfg = config.load(ef)
    finally:
        ef.unlink()
    cfg["_root"] = str(HERE)                      # maps/ of this checkout
    tc = config.trial_config(cfg, cfg["trials"][0])
    mdir = config.map_dir(tc)
    tc["map"]["bbox"] = tc["map"].get("bbox") or json.loads((mdir / "meta.json").read_text())["bbox"]
    start, dur = float(clip["start_s"]), float(clip["duration_s"])
    lines = []
    cfg_file = traffic.prepare(tc, mdir, work / "sumo", start + dur + 5, lambda m: lines.append(str(m)))
    import sumo
    fcd = work / "fcd.xml.gz"
    run = subprocess.run([str(Path(sumo.SUMO_HOME) / "bin" / "sumo"), "-c", str(cfg_file), "--end", str(start + dur),
                          "--fcd-output", str(fcd), "--device.fcd.period", "1", "--fcd-output.geo", "true",
                          "--no-step-log", "--no-warnings", "--duration-log.statistics", "true",
                          "--tripinfo-output", "/dev/null", "--collision-output", "/dev/null",
                          "--stop-output", "/dev/null", "--lateral-resolution", "0.25"],
                         check=True, cwd=str(work / "sumo"), capture_output=True, text=True)
    stats = {k: int(v) for k, v in re.findall(r"(Teleports|Jam|Inserted|Running|Waiting|Collisions): (\d+)",
                                               run.stdout + run.stderr)}
    clat, clon = map(float, a.centre.split(",")) if a.centre else clip_centre(tc)
    seen, inside, total, steps, t = collections.defaultdict(set), collections.Counter(), collections.Counter(), 0, None
    at_start = 0
    for line in gzip.open(fcd, "rt"):
        m = re.search(r'<timestep time="([\d.]+)"', line)
        if m:
            t = float(m.group(1))
            if start <= t <= start + dur:
                steps += 1
            continue
        m = re.search(r'<vehicle id="([^"]+)" x="([-\d.]+)" y="([-\d.]+)".*? type="([^"]+)"', line)
        if not m or t is None:
            continue
        vid, lon, lat, vtype = m.group(1), float(m.group(2)), float(m.group(3)), m.group(4)
        if t == 0.0:
            at_start += 1
        if not (start <= t <= start + dur):
            continue
        c = classify(vtype) or "car"
        total[c] += 1
        if _km(clat, clon, lat, lon) <= a.radius:
            inside[c] += 1
            seen[c].add(vid)
    steps = max(1, steps)
    import importlib.util
    from coupledcarla import crowdpeds
    from coupledcarla.transit import trial_time
    spec = importlib.util.spec_from_file_location("_here_crowdtraffic", HERE / "coupledcarla" / "crowdtraffic.py")
    crowdtraffic = importlib.util.module_from_spec(spec)       # this checkout's (--root may predate it)
    spec.loader.exec_module(crowdtraffic)
    bb = tc["map"]["bbox"]
    iso3, _ = crowdpeds.locate(tc, bb, mdir / "real" / "crowdpeds_country.json")
    target = crowdtraffic.plan_for(iso3, (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2,
                                   trial_time(tc["simulation"].get("time"), None))
    classes = ["car", "taxi", "van", "truck", "bus", "motorcycle", "bicycle", "emergency"]
    uniq = {c: len(seen[c]) for c in classes}
    tot_u = sum(uniq.values()) or 1
    res = {"clip": a.clip, "root": str(root), "centre": [clat, clon], "radius_m": a.radius,
           "window": [start, start + dur], "at_t0_on_map": at_start, "sumo": stats,
           "mean_in_area": {c: round(inside[c] / steps, 2) for c in classes},
           "unique_in_area": uniq, "mean_on_map": {c: round(total[c] / steps, 1) for c in classes},
           "share_unique": {c: round(uniq[c] / tot_u, 3) for c in classes},
           "crowd_seen": {k: round(v, 3) for k, v in target["seen"].items()}, "crowd_levels": target["levels"][:2],
           "log": [ln for ln in lines if re.match(r"(traffic of the place|car stream|cars:|bikes|warm-up|fleet|"
                                                  r"public transport|counts)", ln)]}
    print(json.dumps(res, indent=1, ensure_ascii=False))
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
