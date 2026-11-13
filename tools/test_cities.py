"""Smoke-test every city: a short headless autopilot trial each, results as a Markdown table.

    python tools/test_cities.py [--duration 40] [--only lyon] > docs/city_tests.md
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=40)
    ap.add_argument("--only", action="append")
    args = ap.parse_args()
    cat = yaml.safe_load((ROOT / "experiments" / "cities.yaml").read_text())
    exps = [("eindhoven_centre", ROOT / "experiments" / "eindhoven_centre.yaml", "Eindhoven", "Europe", "medium", "right")]
    for c in cat["cities"]:
        exps.append((c["slug"], ROOT / "experiments" / "cities" / f"{c['slug']}.yaml", c["name"], c["continent"],
                     c["size"], c["drive"]))
    rows = []
    for slug, exp, name, continent, size, drive in exps:
        if args.only and slug not in args.only:
            continue
        t0 = time.time()
        log_path = ROOT / "logs" / f"test_{slug}.log"
        log_path.parent.mkdir(exist_ok=True)
        with open(log_path, "w") as log_file:   # a file, not a pipe: an orphaned server cannot block us
            proc = subprocess.Popen([sys.executable, "-m", "coupledcarla.run", str(exp), "--trial", "0", "--input",
                                     "autopilot", "--headless", "--view", "chase", "--duration",
                                     str(args.duration), "--frames-every", "10"], cwd=ROOT, stdout=log_file,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            try:
                proc.wait(timeout=args.duration * 5 + 240)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                # the CARLA server runs in its own process group
                subprocess.call(["pkill", "-9", "-f", "CarlaUE4-Linux-Shipping.*carla-rpc-port=2000"])
                time.sleep(5)
                log_file.write("\nTIMEOUT\n")
        out = log_path.read_text(errors="replace")
        trials = sorted((ROOT / "logs" / slug).glob("*_trial0"))
        sfile = trials[-1] / "summary.json" if trials else None
        summary = json.loads(sfile.read_text()) if sfile and sfile.exists() else {}
        meta = json.loads((ROOT / "maps" / slug / "meta.json").read_text())
        events = (trials[-1] / "events.csv").read_text() if trials else ""
        ok = proc.returncode == 0 and summary.get("sim_s", 0) >= args.duration - 0.5
        rows.append([name, continent, size, drive, meta["xodr"]["lane_links"], meta.get("buildings", 0),
                     summary.get("sumo_vehicles_spawned", "–"), summary.get("rtf", "–"),
                     events.count(";left_road;"), events.count(";stuck;"), events.count(";reset_to_lane;"),
                     summary.get("collisions", "–"),
                     "ok" if ok else "FAILED"])
        print(f"{slug}: {rows[-1][-1]} in {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
        if not ok:
            print(out[-2500:], file=sys.stderr)
    print(f"Smoke test, {time.strftime('%Y-%m-%d')}: {args.duration:.0f} s headless autopilot trial per map, "
          "CARLA 0.9.16 at Epic, 50 Hz, on lin4080 (RTX 4080 SUPER shared with another job).\n")
    print("| Map | Continent | Size | Traffic | Lane links | Buildings | SUMO vehicles shown | Real-time factor "
          "| Left road | Stuck | Put back on lane | Collisions | Result |")
    print("|" + "---|" * 13)
    for r in rows:
        print("| " + " | ".join(str(x) for x in r) + " |")


if __name__ == "__main__":
    main()
