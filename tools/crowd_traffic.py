"""Builds coupledcarla/data/crowd_traffic.json, the per-place traffic table of coupledcarla/crowdtraffic.py (the
vehicle mix and density of SUMO's demand), from CROWD's published outputs (City Road Observations With Dashcams,
crowd-dataset/crowd; data under CC BY 4.0):

  --mapping      CROWD's mapping.csv (localities, segments, time of day, the vehicle the footage is filmed from)
  --detections   CROWD's analysis_data/detection_counts.csv.gz (unique YOLO tracks per segment, confidence >= 0.7:
                 Persons, Cars, Bicycles, Motorcycles, Buses, Trucks)

It only reads them. Rates per minute are computed as CROWD's analysis.py and tools/crowd_table.py do (sum of tracks /
sum of processed minutes, processed minutes = end - 1 s - start), over the footage of every viewpoint, as CROWD's
README tables and crowd_pedestrians.json (--viewpoints 0,1,2 keeps only some: CROWD's vehicle_type codes, 0 car,
1 bus, 2 truck, 3 motorcycle, 4 bicycle, ...). Footage filmed from a bicycle rides on the cycle paths and sees more
cyclists than a car's dashcam; Coupled CARLA's cameras (chase, drone, pedestrian) see both, so all of it counts.

Each level (world, continent, country, locality) keeps its processed minutes and its tracks per minute of every
class, over all footage and by day / night (time_of_day 0 / 1; a part with under an hour of footage is left out).

    python tools/crowd_traffic.py --mapping .../mapping.csv --detections .../detection_counts.csv.gz \
        --crowd-version "crowd-dataset/crowd 354ea3b6 (2026-11-27)"
"""
import argparse
import csv
import gzip
import json
import statistics
from collections import defaultdict
from pathlib import Path

try:
    from tools.crowd_table import _nested, _processed_s
except ImportError:          # run as python tools/crowd_traffic.py
    from crowd_table import _nested, _processed_s

OUT = Path(__file__).resolve().parent.parent / "coupledcarla" / "data" / "crowd_traffic.json"
CLASSES = ("persons", "cars", "bicycles", "motorcycles", "buses", "trucks")
COLUMNS = ("Persons", "Cars", "Bicycles", "Motorcycles", "Buses", "Trucks")
MIN_PART_MIN = 60.0                     # day / night rates only from at least an hour of footage


def detections(path):
    out = {}
    with gzip.open(path, "rt") as f:
        for r in csv.DictReader(f):
            out[(r["video"], int(r["start"]))] = tuple(int(r.get(c) or 0) for c in COLUMNS)
    return out


def accumulate(rows, counts, viewpoints=None):
    """Per locality id: {"minutes", "segments", class: tracks, and the same with _day / _night}."""
    per = defaultdict(lambda: defaultdict(float))
    seen = set()
    for row in rows:
        vids, starts, ends, tods, vtypes = (_nested(row["videos"]), _nested(row["start_time"]),
                                            _nested(row["end_time"]), _nested(row["time_of_day"]),
                                            _nested(row.get("vehicle_type", "")))
        for i, vid in enumerate(vids):
            try:
                vt = int(float(vtypes[i])) if i < len(vtypes) else 0
            except (TypeError, ValueError):
                vt = 0
            if viewpoints is not None and vt not in viewpoints:
                continue
            for j, (st, et) in enumerate(zip(starts[i] if i < len(starts) else [], ends[i] if i < len(ends) else [])):
                key = (vid, int(st))
                c = counts.get(key)
                if c is None or key in seen:
                    continue
                seen.add(key)
                m = _processed_s(st, et) / 60.0
                if m <= 0:
                    continue
                tod = int(tods[i][j]) if i < len(tods) and j < len(tods[i]) else 0
                part = "night" if tod else "day"
                p = per[row["id"]]
                for k, v in zip(CLASSES + ("minutes", "segments"), c + (m, 1)):
                    p[k] += v
                    p[f"{k}_{part}"] += v
    return per


def rates(acc):
    """{"minutes", "segments", "per_min": {class: rate}, "day": {...}, "night": {...}} of an accumulator."""
    def one(sfx):
        m = acc.get(f"minutes{sfx}", 0.0)
        return {"minutes": round(m, 1), "segments": int(acc.get(f"segments{sfx}", 0)),
                "per_min": {k: round(acc.get(f"{k}{sfx}", 0.0) / m, 3) for k in CLASSES}} if m > 0 else None
    out = one("")
    if out is None:
        return None
    for part in ("day", "night"):
        r = one(f"_{part}")
        if r and r["minutes"] >= MIN_PART_MIN:
            out[part] = r
    return out


def build(args):
    rows = list(csv.DictReader(open(args.mapping, encoding="utf-8")))
    vp = {int(v) for v in args.viewpoints.split(",")} if args.viewpoints else None
    per_loc = accumulate(rows, detections(args.detections), vp)
    acc_c, acc_k, acc_w = defaultdict(lambda: defaultdict(float)), defaultdict(lambda: defaultdict(float)), \
        defaultdict(float)
    info, localities = {}, []
    for r in rows:
        acc = per_loc.get(r["id"])
        if not acc:
            continue
        info.setdefault(r["iso3"], {"name": r["country"], "continent": r["continent"]})
        for k, v in acc.items():
            acc_c[r["iso3"]][k] += v
            acc_k[r["continent"]][k] += v
            acc_w[k] += v
        try:
            lat, lon = float(r["lat"]), float(r["lon"])
        except ValueError:
            continue
        if acc["minutes"] < args.min_locality_minutes:
            continue
        e = {"name": r["locality"], "iso3": r["iso3"], "lat": round(lat, 4), "lon": round(lon, 4)}
        if r.get("state"):
            e["state"] = r["state"]
        e.update(rates(acc))
        localities.append(e)
    countries = {iso3: {**info[iso3], **rates(acc)} for iso3, acc in sorted(acc_c.items())}
    # the reference a place's density is measured against: the median locality with at least two hours of footage
    # (the world's pooled rate is dominated by a few cities with thousands of hours, New York's 1,900 among them)
    ref = [l for l in localities if l["minutes"] >= 120]
    median = {"localities": len(ref)}
    for part in (None, "day", "night"):
        vals = [(l[part] if part else l)["per_min"] for l in ref if (l.get(part) if part else l)]
        if vals:
            median[part or "all"] = {k: round(statistics.median(v[k] for v in vals), 3) for k in CLASSES}
    continents = {k: rates(acc) for k, acc in sorted(acc_k.items())}
    return {
        "version": 1,
        "source": {
            "dataset": "CROWD: City Road Observations With Dashcams (crowd-dataset/crowd)",
            "crowd_version": args.crowd_version,
            "licence": "CC BY 4.0 (CROWD DATA_LICENSE.md); YouTube videos are not redistributed",
            "citation": "Alam, M. S., Bazilinska, O., & Bazilinskyy, P. (2026). A global dataset of continuous urban "
                        "dashcam driving. arXiv:2604.01044",
            "notes": "per_min: unique YOLO (yolo11x) tracks with confidence >= 0.7 per processed minute of footage "
                     f"({'viewpoints ' + args.viewpoints if args.viewpoints else 'every viewpoint'}; COCO classes person, car, bicycle, motorcycle, bus, truck; "
                     "'truck' also takes vans and pickups, 'motorcycle' scooters and mopeds; parked vehicles count "
                     "too); day / night from CROWD's time_of_day label, given where a level has an hour of each.",
        },
        "classes": list(CLASSES),
        "world": rates(acc_w),
        "median_locality": median,
        "continents": continents,
        "countries": countries,
        "localities": sorted(localities, key=lambda l: (l["iso3"], l["name"])),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mapping", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--crowd-version", default="")
    ap.add_argument("--viewpoints", default="", help="CROWD vehicle_type codes to keep (default: all)")
    ap.add_argument("--min-locality-minutes", type=float, default=30.0)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    data = build(args)
    Path(args.out).write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    w = data["world"]
    print(f"{args.out}: world {w['minutes'] / 60:.0f} h, {len(data['countries'])} countries, "
          f"{len(data['localities'])} localities")


if __name__ == "__main__":
    main()
