"""Smoke-test the CARLA towns: a short headless autopilot trial in each, with SUMO traffic, the town's traffic lights
run by SUMO, parked cars, a crowd and CARLA's AI walkers; results as a Markdown table.

    python tools/test_towns.py [--duration 40] [--port 2000] [--only Town03_Opt] > docs/town_tests.md

Each town's experiment is written to logs/town_tests/<town>.yaml (map.carla_town; the map is built on first use in
maps/<town>/). The server is started by the run on --port and stopped after each town; only a server on that port
is ever killed.
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
sys.path.insert(0, str(ROOT))

from coupledcarla import towns  # noqa: E402

TOWNS_0916 = [f"{t}_Opt" for t in ("Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD")]


def experiment(town, port, duration, walkers=20, ai=20):
    """The smoke-test experiment of a town (a dict for YAML)."""
    return {"name": f"town_test_{town}", "map": {"carla_town": town},
            "server": {"port": port, "tm_port": port + 6000},
            "simulation": {"weather": "ClearNoon"},
            "traffic": {"walkers": {"count": walkers, "ai": {"count": ai}}},
            "participants": [{"role": "driver", "vehicle": "vehicle.lincoln.mkz_2020", "input": "autopilot",
                              "view": "chase"}],
            "logging": {"carla_recorder": False},
            "trials": [{"duration_s": duration}]}


def mem_available_gb():
    """MemAvailable of /proc/meminfo in GB (Linux), else None."""
    try:
        for line in open("/proc/meminfo"):
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1048576
    except OSError:
        return None
    return None


def wait_for_memory(need_gb, log=print):
    """Waits until the machine has need_gb available (a shared machine: other servers come and go)."""
    while need_gb and (mem_available_gb() or need_gb) < need_gb:
        log(f"waiting for {need_gb:.0f} GB of free memory ({mem_available_gb():.1f} GB now)")
        time.sleep(120)


def run_one(exp, log_path, duration, port):
    with open(log_path, "w") as log_file:   # a file, not a pipe: an orphaned server cannot block us
        proc = subprocess.Popen([sys.executable, "-m", "coupledcarla.run", str(exp), "--trial", "0", "--input",
                                 "autopilot", "--headless", "--view", "chase", "--duration", str(duration),
                                 "--frames-every", "10"], cwd=ROOT, stdout=log_file, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            proc.wait(timeout=duration * 5 + 400)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
            log_file.write("\nTIMEOUT\n")
        # only the server this run started (its port); the server runs in its own process group
        subprocess.call(["pkill", "-9", "-f", f"(CarlaUE4|CarlaUnreal)-Linux-Shipping.*carla-rpc-port={port}( |$)"])
        time.sleep(5)
    return proc


def row(town, meta, summary, events, log_text, ok, objs):
    tl = summary.get("town_lights") or {}
    sc = meta.get("scenery") or {}
    n_ai = next((int(line.split("town: ")[1].split()[0]) for line in log_text.splitlines()
                 if "AI walkers on the navigation mesh" in line), 0)
    return [town, meta["xodr"]["lane_links"], f"{tl.get('synced', '–')} of {tl.get('carla', '–')}",
            sc.get("parked_kerb_stalls", 0), summary.get("parked_cars", "–"), (objs or {}).get("buildings", "–"),
            n_ai or 0, summary.get("sumo_vehicles_spawned", "–"), summary.get("rtf", "–"),
            events.count(";left_road;"), events.count(";stuck;"),
            events.count(";reset_to_lane;") + events.count(";respawn;"), summary.get("collisions", "–"),
            "ok" if ok else "FAILED"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=40)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", action="append")
    ap.add_argument("--towns", nargs="*", default=TOWNS_0916)
    ap.add_argument("--need-free-gb", type=float, default=0, help="wait for this much available memory per town")
    args = ap.parse_args()
    rows = []
    folder = ROOT / "logs" / "town_tests"
    folder.mkdir(parents=True, exist_ok=True)
    for town in args.towns:
        if args.only and town not in args.only:
            continue
        t0 = time.time()
        exp = folder / f"{town}.yaml"
        exp.write_text(yaml.safe_dump(experiment(town, args.port, args.duration), sort_keys=False))
        log_path = folder / f"{town}.log"
        for attempt in range(2):   # CARLA 0.9.16 now and then segfaults while starting: try once more
            wait_for_memory(args.need_free_gb, lambda m: print(f"{town}: {m}", file=sys.stderr, flush=True))
            proc = run_one(exp, log_path, args.duration, args.port)
            if "waiting for the simulator" not in log_path.read_text(errors="replace"):
                break
            print(f"{town}: the CARLA server crashed at start, retrying", file=sys.stderr, flush=True)
        out = log_path.read_text(errors="replace")
        trials = sorted((ROOT / "logs" / f"town_test_{town}").glob("*_trial0"))
        sfile = trials[-1] / "summary.json" if trials else None
        summary = json.loads(sfile.read_text()) if sfile and sfile.exists() else {}
        mdir = ROOT / "maps" / town
        meta = json.loads((mdir / "meta.json").read_text()) if (mdir / "meta.json").exists() else {"xodr": {}}
        objs = json.loads((mdir / "town_objects.json").read_text()) if (mdir / "town_objects.json").exists() else {}
        events = (trials[-1] / "events.csv").read_text() if trials and (trials[-1] / "events.csv").exists() else ""
        ok = proc.returncode == 0 and summary.get("sim_s", 0) >= args.duration - 0.5
        rows.append(row(town, meta, summary, events, out, ok, objs))
        print(f"{town}: {rows[-1][-1]} in {time.time() - t0:.0f}s", file=sys.stderr, flush=True)
        if not ok:
            print(out[-2500:], file=sys.stderr)
    print(f"Smoke test of the CARLA towns, {time.strftime('%Y-%m-%d')}: {args.duration:.0f} s headless autopilot "
          "trial per town, CARLA 0.9.16 at Epic, 50 Hz, on lin4080 (RTX 4080 SUPER, shared with three other CARLA "
          "servers), with SUMO traffic, the town's traffic lights run by SUMO, parked cars, a crowd of 20 and 20 "
          "CARLA AI walkers.\n")
    print("| Town | Lane links | Traffic lights run by SUMO | Parking stalls | Parked cars shown | Buildings (sound, "
          "drones) | AI walkers | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane "
          "| Collisions | Result |")
    print("|" + "---|" * 14)
    for r in rows:
        print("| " + " | ".join(str(x) for x in r) + " |")
    print("\n" + "\n".join(f"- {t}: {towns.info(t)[0]}" for t in args.towns if not args.only or t in args.only))


if __name__ == "__main__":
    main()
