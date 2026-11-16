"""Actors inside buildings in a trial: every vehicle and walker position of frames.csv against the map's footprints.

    python tools/trial_buildings.py logs/shibuya_twin/20261116-075022_trial0          # whole trial
    python tools/trial_buildings.py TRIAL --start 30 --duration 20                     # a clip's window
    python tools/trial_buildings.py TRIAL [TRIAL ...] --json report.json

Run it on a clip's trial before publishing the clip. For a baked twin the footprints are the twin's solid footprints
after its road clearance cut (maps/<name>/twin_buildings.json, from carla-digitaltwins' <Name>.buildings.json), so
vehicles in a passage cut into a building do not count; other maps use maps/<name>/buildings.json. Exit status 1
when an actor stands inside a building (coupledcarla/inbuildings.py). tools/clip_check.py (feat-traffic) does the
vehicle half of this per clip window; this one also checks walkers and whole trials.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import inbuildings  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trials", nargs="+")
    ap.add_argument("--map-dir", help="maps/<name> (default: from the trial's config.yaml)")
    ap.add_argument("--footprints", help="a footprint file instead of the map's")
    ap.add_argument("--start", type=float), ap.add_argument("--duration", type=float)
    ap.add_argument("--every", type=int, default=1, help="check every n-th frame")
    ap.add_argument("--json", help="write the reports here")
    a = ap.parse_args(argv)
    t0 = a.start
    t1 = a.start + a.duration if a.start is not None and a.duration is not None else None
    reports, bad = {}, []
    for trial in a.trials:
        if not (Path(trial) / "frames.csv").exists():
            print(f"{trial}: no frames.csv")
            bad.append(trial)
            continue
        rep = inbuildings.scan_trial(trial, map_dir=a.map_dir and Path(a.map_dir), footprints=a.footprints,
                                     t0=t0, t1=t1, every=a.every)
        reports[trial] = rep
        print("\n".join(inbuildings.summary_lines(trial, rep)), flush=True)
        if rep["footprints"] is None:
            print("    no footprint file found (give --map-dir or --footprints)")
            bad.append(trial)
        elif rep["episodes"]:
            bad.append(trial)
    if a.json:
        Path(a.json).write_text(json.dumps(reports, indent=1))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
