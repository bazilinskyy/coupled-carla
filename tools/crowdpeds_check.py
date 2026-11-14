"""Plans the CROWD-calibrated pedestrian crowd of experiments without a server and prints what it chose.

    python tools/crowdpeds_check.py experiments/eindhoven_centre.yaml experiments/cities/cairo.yaml \
        [--radius 250] [--duration 300] [--count auto] [--night] [--offline]

For each map: the country and where each parameter came from, the walkers planned around the map's centre, how many
crossings they make in the trial (marked, at junctions, mid-block) and the walked crossing rate, and how much more
the planned walkers start near places (Overture/OSM, transit stops, pedestrian counts) than walkers spread evenly
along the kerbs, as the crowd was before.
"""
import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import config, crowdpeds  # noqa: E402


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
    args = ap.parse_args()
    import sumolib
    rows = []
    for exp in args.experiments:
        cfg = config.load(exp)
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
                import json
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
        rows.append((cfg["name"], params, stats, near_places(segs, [(x["seg"], x["side"], x["i"]) for x in walkers], pxy),
                     near_places(segs, uniform, pxy)))
    print()
    print("| Map | Country (how) | Pedestrians/min (from) | Walkers | Speed m/s (from) | Crossings/km target | "
          "Mid-block share target (from) | Crossings in 5 min: marked / junction / mid-block | Mid-block share planned | "
          "Crossings/km planned | Places (count sites) | OSM crossings on the kerbs | Places within 50 m of a start: planned vs even |")
    print("|" + "---|" * 13)
    for name, p, st, near, even in rows:
        print(f"| {name} | {p['country_name']} ({p['located_by']}) | {p['persons_per_min']} ({p['persons_per_min_from']}) | "
              f"{st['walkers']} on {st['kerb_km']} km of kerb | {p['speed_median']} ± {p['speed_sd']} ({p['speed_from']}) | "
              f"{p['crossings_per_km']} | {p['jaywalk_share']:.3f} ({p['jaywalk_from']}) | "
              f"{st['crosswalk']} / {st['junction']} / {st['midblock']} | {st['midblock_share']} | {st['crossings_per_km']} | "
              f"{st['places']} ({st['count_sites']}) | {st['segments_with_crossing']} of {st['segments']} segments |"
              f" {near} vs {even} |")


if __name__ == "__main__":
    main()
