"""Plans the CROWD-calibrated pedestrian crowd of experiments without a server and prints what it chose.

    python tools/crowdpeds_check.py experiments/eindhoven_centre.yaml experiments/cities/cairo.yaml \
        [--radius 250] [--duration 300] [--count auto] [--night] [--offline] [--sumo]

For each map: the country and where each parameter came from, the walkers planned around the map's centre, how many
crossings they make in the trial (marked, at junctions, mid-block) and the walked crossing rate, and how much more
the planned walkers start near places (Overture/OSM, transit stops, pedestrian counts) than walkers spread evenly
along the kerbs, as the crowd was before. With --sumo, a SUMO-only run of the trial's traffic (traffic.prepare, FCD
every 0.5 s) and the planned walkers replayed through it (crowdpeds.replay): how long they wait at the kerb for a
gap and which gaps they accept (SUMO's cars do not yield to them in this replay; in a trial they do).
"""
import argparse
import json
import random
import subprocess
import xml.etree.ElementTree as ET
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import config, crowdpeds, traffic  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


def sumo_frames(cfg, mdir, net, duration, centre, radius=500.0):
    """SUMO-only run of the trial's traffic: [(t, [(x, y, vx, vy)])] in CARLA coordinates near the centre."""
    import math
    work = ROOT / "scratch" / f"crowdpeds_{cfg['map']['name']}"
    sumocfg = traffic.prepare(cfg, mdir, work, duration, log=lambda m: None)
    subprocess.run(["sumo", "-c", sumocfg.name, "--end", str(int(duration) + 1), "--step-length", "0.1",
                    "--no-step-log", "--fcd-output", "crowd_fcd.xml", "--fcd-output.geo", "true",
                    "--device.fcd.period", "0.5", "--lateral-resolution", "0.25"],
                   cwd=work, capture_output=True, text=True)
    frames, cur, t = [], None, None
    for ev, el in ET.iterparse(str(work / "crowd_fcd.xml"), events=("start", "end")):
        if ev == "start" and el.tag == "timestep":
            t, cur = float(el.get("time")), []
        elif ev == "end" and el.tag == "vehicle" and cur is not None:
            x, y = net.convertLonLat2XY(float(el.get("x")), float(el.get("y")))
            if math.hypot(x - centre[0], y - centre[1]) <= radius:
                a, v = math.radians(float(el.get("angle"))), float(el.get("speed"))
                cur.append((x, -y, v * math.sin(a), -v * math.cos(a)))
            el.clear()
        elif ev == "end" and el.tag == "timestep":
            frames.append((t, cur))
            el.clear()
    return frames


def near_places(segs, walkers_or_starts, pois_xy, r=50.0):
    """Median number of places within r metres of the starting points."""
    out = []
    for k, side, i in walkers_or_starts:
        x, y = segs[k].sides[side][i]
        out.append(sum(1 for px, py, _ in pois_xy if (px - x) ** 2 + (py - y) ** 2 <= r * r))
    out.sort()
    return out[len(out) // 2] if out else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiments", nargs="+")
    ap.add_argument("--radius", type=float, default=250.0)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--count", default="auto")
    ap.add_argument("--night", action="store_true")
    ap.add_argument("--offline", action="store_true", help="use only cached real-world data")
    ap.add_argument("--sumo", action="store_true", help="replay the walkers through a SUMO-only run of the traffic")
    args = ap.parse_args()
    import sumolib
    rows = []
    for exp in args.experiments:
        cfg = config.load(exp)
        cfg = config.trial_config(cfg, cfg["trials"][0])
        mdir = config.map_dir(cfg)
        cfg["traffic"]["walkers"] = {**(cfg["traffic"].get("walkers") or {}), "count": args.count,
                                     "radius_m": args.radius}
        net = sumolib.net.readNet(str(mdir / "geo.net.xml"), withInternal=False, withPedestrianConnections=False)
        s, w, n, e = cfg["map"]["bbox"]
        centre = net.convertLonLat2XY((w + e) / 2, (s + n) / 2)
        params, segs, walkers, stats = crowdpeds.prepare(cfg, mdir, net, [centre], args.duration, args.night,
                                                         fetch=not args.offline)
        real = mdir / "real"
        pois = []
        for f in ("overture_places.json", "osm_places.json"):
            p = real / f
            if p.exists():
                pois = json.loads(p.read_text())
                break
        pois += crowdpeds.transit_stops(real)
        pxy = [(*net.convertLonLat2XY(lon, lat), wt) for lat, lon, wt, _ in pois]
        rng = random.Random(1)
        cum = [(k, side) for k, sg in enumerate(segs) for side in (0, 1) for _ in range(sg.n)]
        uniform = []
        for _ in range(max(1, len(walkers))):
            k, side = rng.choice(cum)
            uniform.append((k, side, rng.randrange(segs[k].n)))
        rep = None
        if args.sumo:
            frames = sumo_frames(cfg, mdir, net, args.duration, centre)
            rep = crowdpeds.replay(segs, walkers, frames, args.duration)
            rep = {"vehicles": round(sum(len(f[1]) for f in frames) / max(1, len(frames)), 1), "crossings": rep}
        rows.append((cfg["name"], params, stats, near_places(segs, [(x["seg"], x["side"], x["i"]) for x in walkers], pxy),
                     near_places(segs, uniform, pxy), rep))
    print()
    print("| Map | Country (how) | Pedestrians/min (from) | Walkers | Speed m/s (from) | Crossings/km target | "
          "Mid-block share target (from) | Crossings in 5 min: marked / junction / mid-block | Mid-block share planned | "
          "Crossings/km planned | Places (count sites) | OSM crossings on the kerbs | Places within 50 m of a start: planned vs even |")
    print("|" + "---|" * 13)
    for name, p, st, near, even, _ in rows:
        print(f"| {name} | {p['country_name']} ({p['located_by']}) | {p['persons_per_min']} ({p['persons_per_min_from']}) | "
              f"{st['walkers']} on {st['kerb_km']} km of kerb | {p['speed_median']} ± {p['speed_sd']} ({p['speed_from']}) | "
              f"{p['crossings_per_km']} | {p['jaywalk_share']:.3f} ({p['jaywalk_from']}) | "
              f"{st['crosswalk']} / {st['junction']} / {st['midblock']} | {st['midblock_share']} | {st['crossings_per_km']} | "
              f"{st['places']} ({st['count_sites']}) | {st['segments_with_crossing']} of {st['segments']} segments |"
              f" {near} vs {even} |")
    if args.sumo:
        print()
        print("| Map | SUMO vehicles within 500 m (mean) | Crossings replayed | Waited at the kerb | Median wait, s "
              "(marked / junction / mid-block) | Longest wait, s | Median accepted gap, s | Still waiting at the end |")
        print("|" + "---|" * 8)

        def med(v):
            v = sorted(v)
            return round(v[len(v) // 2], 1) if v else "-"
        for name, p, st, near, even, rep in rows:
            c = [x for x in rep["crossings"] if not x["censored"]]
            by = {k: med([x["wait"] for x in c if x["kind"] == k]) for k in ("crosswalk", "junction", "midblock")}
            print(f"| {name} | {rep['vehicles']} | {len(c)} | {sum(1 for x in c if x['wait'] > 0)} | "
                  f"{by['crosswalk']} / {by['junction']} / {by['midblock']} | "
                  f"{max([x['wait'] for x in c], default=0):.1f} | {med([x['gap'] for x in c if x['gap'] is not None])} | "
                  f"{sum(1 for x in rep['crossings'] if x['censored'])} |")
        (ROOT / "scratch").mkdir(exist_ok=True)
        (ROOT / "scratch" / "crowdpeds_check.json").write_text(json.dumps(
            [{"map": r[0], "params": r[1], "stats": r[2], "replay": r[5]} for r in rows], indent=1))


if __name__ == "__main__":
    main()
