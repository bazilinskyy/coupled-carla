"""Gap acceptance of the crowd with and without perception: the planned walkers of each experiment replayed through
the same SUMO-only traffic (crowdpeds.replay), once with the crowd's gap check on every car's true position and
speed (HCM critical gap) and once with embodiedped's walkers that only know what they have seen (embodied-human's
eyes, peripheral vision and reaction times). Needs embodied-human installed; SUMO; no CARLA.

    python tools/embodiedped_check.py experiments/eindhoven_crowd.yaml experiments/cities/new_york_midtown.yaml \\
        --duration 300 [--seeds 3] [--json scratch/embodiedped_check.json]

Per map and model: crossings, median wait at the kerb, the accepted gap (time until the next car reached the
crossing line when the walker stepped out) and how it depends on that car's speed (published: people accept shorter
time gaps from faster cars, judging by distance: Lobjois & Cavallo 2007, Petzoldt 2014; the HCM rule has no such
effect), crossings with an accepted gap under t_c and under the time it takes to cross (a car would have to brake),
and the model's cost per decision.
"""
import argparse
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import config, crowdpeds, embodiedped, traffic  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SPEED_BINS = ((0.0, 30.0), (30.0, 45.0), (45.0, 200.0))      # km/h of the car that set the accepted gap


def sumo_frames(cfg, mdir, net, duration, centre, radius=500.0, period=0.1, seed=None):
    """SUMO-only run of the trial's traffic: [(t, [(x, y, vx, vy)], [ids])] in CARLA coordinates near the centre."""
    work = ROOT / "scratch" / f"embodiedped_{cfg['map']['name']}_{seed}"
    if seed is not None:
        cfg["traffic"]["seed"] = seed
    sumocfg = traffic.prepare(cfg, mdir, work, duration, log=lambda m: None)
    subprocess.run(["sumo", "-c", sumocfg.name, "--end", str(int(duration) + 1), "--step-length", "0.1",
                    "--no-step-log", "--fcd-output", "fcd.xml", "--fcd-output.geo", "true",
                    "--device.fcd.period", str(period), "--lateral-resolution", "0.25"],
                   cwd=work, capture_output=True, text=True, check=False)
    frames, cur, ids, t, num = [], None, None, None, {}
    for ev, el in ET.iterparse(str(work / "fcd.xml"), events=("start", "end")):
        if ev == "start" and el.tag == "timestep":
            t, cur, ids = float(el.get("time")), [], []
        elif ev == "end" and el.tag == "vehicle" and cur is not None:
            x, y = net.convertLonLat2XY(float(el.get("x")), float(el.get("y")))
            if math.hypot(x - centre[0], y - centre[1]) <= radius and not el.get("type", "").startswith(
                    ("vehicle.gazelle", "vehicle.bh.", "vehicle.diamondback")):      # bicycles: not judged here
                a, v = math.radians(float(el.get("angle"))), float(el.get("speed"))
                cur.append((x, -y, v * math.sin(a), -v * math.cos(a)))
                ids.append(num.setdefault(el.get("id"), len(num)))
            el.clear()
        elif ev == "end" and el.tag == "timestep":
            frames.append((t, cur, ids))
            el.clear()
    return frames


def med(v):
    v = sorted(v)
    return round(v[len(v) // 2], 2) if v else None


def pct(v, q):
    v = sorted(v)
    return round(v[min(len(v) - 1, int(q * len(v)))], 2) if v else None


def slope(xs, ys):
    """Least-squares slope of ys on xs."""
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return round(sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx, 4) if sxx else None


def summarise(rows, kinds=None):
    """Crossings of the given kinds (midblock, crosswalk, junction; None: all)."""
    done = [r for r in rows if not r["censored"] and (kinds is None or r["kind"] in kinds)]
    rows = [r for r in rows if kinds is None or r["kind"] in kinds]
    judged = [r for r in done if r["gap"] is not None and r["gap"] < 30.0]
    out = {"crossings": len(done), "censored": len(rows) - len(done), "waited": sum(1 for r in done if r["wait"] > 0),
           "wait_median_s": med([r["wait"] for r in done]), "wait_p90_s": pct([r["wait"] for r in done], 0.9),
           "with_car_approaching": len(judged), "gap_median_s": med([r["gap"] for r in judged]),
           "gap_p15_s": pct([r["gap"] for r in judged], 0.15),
           "gap_under_tc": sum(1 for r in judged if r["gap"] < r["t_c"]),
           "gap_under_crossing_time": sum(1 for r in judged if r["gap"] < r["cross_s"]),
           # seconds of accepted gap per 10 km/h of the car's speed (negative: shorter gaps from faster cars)
           "gap_per_10kmh_s": (lambda s: round(10 * s / 3.6, 3) if s is not None else None)(
               slope([r["gap_speed"] for r in judged], [r["gap"] for r in judged]))}
    for lo, hi in SPEED_BINS:
        g = [r["gap"] for r in judged if lo <= 3.6 * r["gap_speed"] < hi]
        out[f"gap_median_{int(lo)}_{int(hi)}kmh_s"] = med(g)
        out[f"n_{int(lo)}_{int(hi)}kmh"] = len(g)
        out[f"under_tc_{int(lo)}_{int(hi)}kmh"] = sum(1 for r in judged if lo <= 3.6 * r["gap_speed"] < hi
                                                      and r["gap"] < r["t_c"])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiments", nargs="+")
    ap.add_argument("--radius", type=float, default=250.0)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--count", default="auto")
    ap.add_argument("--seeds", type=int, default=1, help="SUMO demand seeds (traffic.seed 1..N)")
    ap.add_argument("--dt", type=float, default=0.08, help="decision step, s (the crowd decides every 4 ticks)")
    ap.add_argument("--set", action="append", default=[], help="perception option, e.g. --set prior_speed_kmh=50")
    ap.add_argument("--midblock", action="store_true", help="every crossing mid-block (jaywalk_share 1): pure gap "
                    "acceptance, no car that could stop is ignored")
    ap.add_argument("--offline", action="store_true")
    ap.add_argument("--json", default=str(ROOT / "scratch" / "embodiedped_check.json"))
    args = ap.parse_args()
    import sumolib
    over = {k: float(v) for k, v in (s.split("=", 1) for s in args.set)}
    results = []
    for exp in args.experiments:
        cfg = config.load(exp)
        cfg = config.trial_config(cfg, cfg["trials"][0])
        mdir = config.map_dir(cfg)
        cfg["traffic"]["walkers"] = {**(cfg["traffic"].get("walkers") or {}), "count": args.count,
                                     "radius_m": args.radius, **({"jaywalk_share": 1.0} if args.midblock else {})}
        net = sumolib.net.readNet(str(mdir / "geo.net.xml"), withInternal=False, withPedestrianConnections=False)
        s, w, n, e = cfg["map"]["bbox"]
        centre = net.convertLonLat2XY((w + e) / 2, (s + n) / 2)
        _params, segs, walkers, stats = crowdpeds.prepare(cfg, mdir, net, [centre], args.duration,
                                                          fetch=not args.offline, log=lambda m: None)
        rows = {"hcm": [], "embodied": []}
        cost = []
        for seed in range(1, args.seeds + 1):
            frames = sumo_frames(cfg, mdir, net, args.duration, centre, seed=seed)
            rows["hcm"] += crowdpeds.replay(segs, walkers, frames, args.duration, dt=args.dt)
            m = embodiedped.EmbodiedCrossing({**embodiedped.DEFAULTS, "model": "embodied", "seed": seed, **over})
            t0 = time.perf_counter()
            calls = [0]
            decide = m.decide

            def counted(*a, **k):
                calls[0] += 1
                return decide(*a, **k)
            m.decide = counted
            rows["embodied"] += crowdpeds.replay(segs, walkers, frames, args.duration, dt=args.dt, decider=m)
            cost.append((time.perf_counter() - t0, calls[0], m.counts))
            veh = round(sum(len(f[1]) for f in frames) / max(1, len(frames)), 1)
        res = {"map": cfg["name"], "walkers": len(walkers), "planned_crossings": stats["crossings"],
               "vehicles_within_500m": veh, "seeds": args.seeds, "options": over,
               "hcm": summarise(rows["hcm"]), "embodied": summarise(rows["embodied"]),
               "hcm_midblock": summarise(rows["hcm"], ("midblock",)),
               "embodied_midblock": summarise(rows["embodied"], ("midblock",)),
               "embodied_us_per_decision": round(1e6 * sum(c[0] for c in cost) / max(1, sum(c[1] for c in cost)), 1),
               "embodied_counts": cost[-1][2]}
        res["rows"] = rows
        results.append(res)
        print(json.dumps({k: v for k, v in res.items() if k != "rows"}), flush=True)
    Path(args.json).parent.mkdir(parents=True, exist_ok=True)
    pooled = {name: summarise([x for r in results for x in r["rows"][name.split("_")[0]]],
                              ("midblock",) if name.endswith("midblock") else None)
              for name in ("hcm", "embodied", "hcm_midblock", "embodied_midblock")}
    results.append({"map": "all", **pooled})
    Path(args.json).write_text(json.dumps(results, indent=1))
    print()
    print("| Map | Model | Crossings (car approaching) | Gaps under t_c by car speed <30 / 30-45 / >45 km/h |")
    print("|---|---|---|---|")
    for r in results:
        for name in ("hcm_midblock", "embodied_midblock"):
            x = r[name]
            print(f"| {r['map']} | {name} | {x['crossings']} ({x['with_car_approaching']}) | "
                  + " / ".join(f"{x[f'under_tc_{int(lo)}_{int(hi)}kmh']} of {x[f'n_{int(lo)}_{int(hi)}kmh']}"
                               for lo, hi in SPEED_BINS) + " |")
    print()
    print("| Map | Model | Crossings (car approaching) | Median wait, s | Accepted gap, median (P15), s | "
          "Median gap by car speed <30 / 30-45 / >45 km/h, s | Gap per +10 km/h, s | Under t_c | Under crossing time |")
    print("|" + "---|" * 9)
    for r in results:
        for name in ("hcm", "embodied", "hcm_midblock", "embodied_midblock"):
            x = r[name]
            print(f"| {r['map']} | {name} | {x['crossings']} ({x['with_car_approaching']}) | {x['wait_median_s']} | "
                  f"{x['gap_median_s']} ({x['gap_p15_s']}) | {x['gap_median_0_30kmh_s']} / {x['gap_median_30_45kmh_s']} / "
                  f"{x['gap_median_45_200kmh_s']} (n {x['n_0_30kmh']} / {x['n_30_45kmh']} / {x['n_45_200kmh']}) | "
                  f"{x['gap_per_10kmh_s']} | {x['gap_under_tc']} | {x['gap_under_crossing_time']} |")


if __name__ == "__main__":
    main()
