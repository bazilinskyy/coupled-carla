"""Sums a trial's busy and scramble crossings (coupledcarla/scramble.py): per walk phase the walkers waiting at its
start, the crossers, those on the road in view of the first camera or participant, crossing times, stragglers,
late runners, red-light crossers and photo stops.

    python tools/crossing_report.py logs/<experiment>/<stamp>_trial0 [--json]
"""
import argparse
import csv
import json
import sys
from pathlib import Path


def _q(vals, f):
    vals = sorted(vals)
    return round(vals[min(len(vals) - 1, int(f * len(vals)))], 1) if vals else None


def report(trial):
    trial = Path(trial)
    ph = list(csv.DictReader(open(trial / "crossing_phases.csv"))) if (trial / "crossing_phases.csv").exists() else []
    cr = list(csv.DictReader(open(trial / "crossings.csv"))) if (trial / "crossings.csv").exists() else []
    out = {}
    for kind in ("scramble", "busy"):
        p = [r for r in ph if r["kind"] == kind]
        c = [r for r in cr if r["kind"] == kind]
        if not p and not c:
            continue
        num = (lambda rows, k: [float(r[k]) for r in rows if r.get(k) not in ("", None)])
        d = {"phases": len(p),
             "crossers_per_phase": [int(r["crossers"]) for r in p],
             "diagonal_per_phase": [int(r["diagonal"]) for r in p],
             "queued_at_start": [int(r["queued_at_start"]) for r in p],
             "in_view_max": [int(r["in_view_max"]) for r in p],
             "in_view_mean": num(p, "in_view_mean"),
             "late_runners": sum(int(r["late_runners"]) for r in p),
             "crossings": len(c),
             "red_crossers": sum(r["red"] == "True" for r in c),
             "stragglers_ran": sum(r["ran"] == "True" for r in c),
             "photo_stops": sum(r["photo"] == "True" for r in c)}
        for leg in sorted({r["leg"] for r in c}):
            s = [float(r["s"]) for r in c if r["leg"] == leg and r["red"] != "True"]
            ln = [float(r["length_m"]) for r in c if r["leg"] == leg]
            d[f"crossing_s_{leg}"] = {"n": len(s), "p10": _q(s, 0.1), "median": _q(s, 0.5), "p90": _q(s, 0.9),
                                      "length_m_median": _q(ln, 0.5)}
        out[kind] = d
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("trial")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    r = report(a.trial)
    if a.json:
        print(json.dumps(r, indent=1))
        return 0
    if not r:
        print("no crossing crowds in this trial")
        return 1
    for kind, d in r.items():
        print(f"{kind}: {d['phases']} walk phases, {d['crossings']} crossings")
        print(f"  crossers per phase {d['crossers_per_phase']} (diagonal {d['diagonal_per_phase']})")
        print(f"  waiting at the phase's start {d['queued_at_start']}")
        print(f"  on the road in view: max {d['in_view_max']}, mean {d['in_view_mean']}")
        for k, v in d.items():
            if k.startswith("crossing_s_"):
                print(f"  crossing time {k[11:]}: {v}")
        print(f"  stragglers who ran {d['stragglers_ran']}, late runners {d['late_runners']}, red-light crossers "
              f"{d['red_crossers']}, photo stops {d['photo_stops']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
