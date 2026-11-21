"""Did a participant stop at red lights? From a trial's frames.csv (next_tl, next_tl_dist, next_tl_state per tick).

    python tools/redlight_check.py TRIAL_DIR [--role driver]

A red approach: the participant within 30 m of a light showing red (r/R) or amber; a stop: speed under 1 km/h within
15 m of a red light; a red run: the light ahead changes (the stop line was passed) while the last state seen was red
within 6 m and the speed was over 5 km/h. Prints the counts and each red run with its time.
"""
import argparse
import csv
from pathlib import Path


def check(trial_dir, role="driver"):
    rows = []
    with open(Path(trial_dir) / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if r["role"] == role:
                rows.append(r)
    approaches, stops, runs = set(), set(), []
    prev = None
    for r in rows:
        tl, st = r.get("next_tl") or "", r.get("next_tl_state") or ""
        d = float(r["next_tl_dist"]) if r.get("next_tl_dist") else None
        v = float(r["speed_kmh"] or 0.0)
        if tl and d is not None and st[:1] in "rRyY" and d <= 30.0:
            approaches.add(tl)
        if tl and d is not None and st[:1] in "rR" and d <= 15.0 and v < 1.0:
            stops.add(tl)
        if prev is not None:
            ptl, pst = prev.get("next_tl") or "", prev.get("next_tl_state") or ""
            pd = float(prev["next_tl_dist"]) if prev.get("next_tl_dist") else None
            pv = float(prev["speed_kmh"] or 0.0)
            if ptl and ptl != tl and pst[:1] in "rR" and pd is not None and pd <= 6.0 and pv > 5.0:
                runs.append((float(prev["timestamp"]), ptl, pd, pv))
        prev = r
    return {"ticks": len(rows), "red_or_amber_approaches": len(approaches), "stops_at_red": len(stops),
            "red_runs": runs}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trial_dir")
    ap.add_argument("--role", default="driver")
    a = ap.parse_args()
    out = check(a.trial_dir, a.role)
    print(f"{out['ticks']} ticks: {out['red_or_amber_approaches']} lights approached at red/amber, "
          f"{out['stops_at_red']} stopped at red, {len(out['red_runs'])} red runs")
    for t, tl, d, v in out["red_runs"]:
        print(f"  red run at t={t:.1f} s: light {tl}, {d:.1f} m before it at {v:.0f} km/h")


if __name__ == "__main__":
    main()
