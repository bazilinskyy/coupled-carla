"""Walkers and cyclists a moving participant 'sees' per minute in a trial, against CROWD's pedestrians per minute.

CROWD (crowd-dataset/crowd) counts persons per minute of dashcam video. A trial's driving participant is such a
camera: this counts the unique walkers (and SUMO bikes) within --range m in a forward cone of +-60 degrees while it
moves faster than 2 m/s, from the trial's frames.csv, and prints them per minute of moving time beside the CROWD rate
of the nearest locality (else country, continent, world; coupledcarla/data/crowd_pedestrians.json).

    python tools/crowd_seen.py logs/<experiment>/<stamp>_trial0 [--range 40] [--role driver]
"""
import argparse
import collections
import csv
import json
import math
import sys
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / "coupledcarla" / "data" / "crowd_pedestrians.json"


def seen_per_minute(rows, role=None, rng=40.0, half_deg=60.0, vmin=2.0):
    """(moving minutes, walkers seen, bikes seen) from frames.csv rows (dicts): unique ids in the forward cone."""
    by_t = collections.defaultdict(list)
    for r in rows:
        by_t[float(r["timestamp"])].append(r)
    moving, walkers, bikes, prev = 0.0, set(), set(), None
    for t in sorted(by_t):
        part = [r for r in by_t[t] if r["kind"] == "participant" and (role is None or r.get("role") == role)]
        if not part:
            prev = t
            continue
        p = part[0]
        px, py, yaw = float(p["pos_x"]), float(p["pos_y"]), math.radians(float(p["rot_yaw"]))
        if prev is not None and float(p["speed_kmh"]) / 3.6 > vmin:
            moving += t - prev
            for r in by_t[t]:
                if r["kind"] not in ("walker", "bike"):
                    continue
                dx, dy = float(r["pos_x"]) - px, float(r["pos_y"]) - py
                d = math.hypot(dx, dy)
                if 0.5 <= d <= rng and abs((math.degrees(math.atan2(dy, dx) - yaw) + 180) % 360 - 180) <= half_deg:
                    (walkers if r["kind"] == "walker" else bikes).add(r["actor_id"])
        prev = t
    return moving / 60.0, len(walkers), len(bikes)


def crowd_rate(lat, lon, data=None):
    """(persons per minute, where from): the nearest CROWD locality within 30 km with >= 5 hours of footage, else the
    country of the nearest locality (within 500 km), else the world (as crowdpeds.behaviour, roughly)."""
    d = data or json.loads(DATA.read_text())
    near, good = None, None
    for loc in d.get("localities", []):
        if loc.get("persons_per_min") is None:
            continue
        km = 6371.0 * math.hypot(math.radians(loc["lat"] - lat),
                                 math.radians(loc["lon"] - lon) * math.cos(math.radians(lat)))
        if near is None or km < near[0]:
            near = (km, loc)
        if loc.get("hours", 0) >= 5 and km <= 30 and (good is None or km < good[0]):
            good = (km, loc)
    if good:
        return good[1]["persons_per_min"], f"locality {good[1]['name']} ({good[0]:.0f} km)"
    country = (d.get("countries") or {}).get(near[1].get("iso3")) if near and near[0] <= 500 else None
    if country and country.get("persons_per_min") is not None:
        return country["persons_per_min"], f"country {country.get('name')}"
    return d["world"]["persons_per_min"], "world"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("trial", help="a trial folder with frames.csv")
    ap.add_argument("--range", type=float, default=40.0)
    ap.add_argument("--role", default=None, help="the participant (default: the first one logged)")
    a = ap.parse_args(argv)
    rows = list(csv.DictReader(open(Path(a.trial) / "frames.csv"), delimiter=";"))
    m, w, b = seen_per_minute(rows, a.role, a.range)
    if m <= 0:
        print("the participant did not move")
        return 1
    part = next(r for r in rows if r["kind"] == "participant")
    rate, src = crowd_rate(float(part["lat"]), float(part["lon"]))
    print(f"moving {m * 60:.0f} s: walkers seen {w} ({w / m:.1f}/min), bikes {b} ({b / m:.1f}/min); "
          f"CROWD {rate:.1f} persons/min ({src})" + ("  (under a minute: a rough figure)" if m < 1 else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
