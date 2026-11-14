"""Builds coupledcarla/data/crowd_pedestrians.json, the pedestrian table of coupledcarla/crowdpeds.py, from CROWD's
published outputs (City Road Observations With Dashcams, crowd-dataset/crowd; data under CC BY 4.0):

  --mapping      CROWD's mapping.csv (localities, segments, time of day)
  --detections   CROWD's analysis_data/detection_counts.csv.gz (unique YOLO tracks per segment, confidence >= 0.7)
  --countries    CROWD's analysis_data/countries.csv (country names -> ISO2)
  --speeds       crowd-city's crowd_crossing_speed_by_city.csv (median crossing speed in m/s per city, Waymo-calibrated)
  --jaywalking   crowd-jaywalking's results folder (one folder per run with per_person_results.csv and
                 per_video_results.csv: crossings in front of the dashcam checked by a VLM for a marked crosswalk or
                 a pedestrian light)

It only reads them. Rates per minute are computed as CROWD's analysis.py does (sum of tracks / sum of processed
minutes, processed minutes = end - 1 s - start), so the numbers match the CROWD README's tables.

    python tools/crowd_table.py --mapping .../mapping.csv --detections .../detection_counts.csv.gz \
        --countries .../countries.csv --speeds .../crowd_crossing_speed_by_city.csv \
        --jaywalking .../results_jaywalking --crowd-version "crowd-dataset/crowd 14204a2d (2026-11-14)"
"""
import argparse
import csv
import gzip
import json
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "coupledcarla" / "data" / "crowd_pedestrians.json"

# runs in the order they were made: a segment in several runs counts once, from the latest
JAYWALK_RUNS = ["crowd_jaywalking_nl", "crowd_jaywalking_bgd", "crowd_jaywalking_rus_jpn", "crowd_capitals",
                "six_countries", "korea_vietnam", "six_countries_100h"]
# CROWD country names that countries.csv spells differently (ISO2)
ISO2_EXTRA = {"Bahamas": "BS", "Bonaire, Sint Eustatius and Saba": "BQ", "Cabo Verde": "CV",
              "Congo (Congo-Brazzaville)": "CG", "Congo (Democratic Republic)": "CD", "Czechia": "CZ",
              "Côte d'Ivoire": "CI", "Gambia": "GM", "Micronesia": "FM", "Palestine": "PS", "Saint Helena": "SH",
              "Saint Martin": "MF", "Sao Tome and Principe": "ST"}


def _nested(cell):
    """'[[0,100],[5]]' -> [[0, 100], [5]]; '[a,b]' -> ['a', 'b']."""
    s = (cell or "").strip()
    if not s or s == "[]":
        return []
    if s.startswith("[["):
        return [[float(x) for x in re.findall(r"-?[\d.]+", inner)] for inner in re.findall(r"\[([^\[\]]*)\]", s[1:-1])]
    return [t.strip() for t in s.strip("[]").split(",") if t.strip()]


def _processed_s(st, et):
    st, et = int(st), int(et)
    end = et - 1 if et - 1 > st else et
    return max(0, end - st)


def detections(mapping_rows, det_path):
    """Per locality id: persons, cars, motorcycles, minutes, by day (0) and night (1)."""
    counts = {}
    with gzip.open(det_path, "rt") as f:
        for r in csv.DictReader(f):
            counts[(r["video"], int(r["start"]))] = (int(r["Persons"]), int(r["Cars"]),
                                                     int(r.get("Motorcycles") or 0))
    per = defaultdict(lambda: defaultdict(float))
    seen = set()
    for row in mapping_rows:
        vids, starts, ends, tods = (_nested(row["videos"]), _nested(row["start_time"]), _nested(row["end_time"]),
                                    _nested(row["time_of_day"]))
        for i, vid in enumerate(vids):
            for j, (st, et) in enumerate(zip(starts[i] if i < len(starts) else [], ends[i] if i < len(ends) else [])):
                key = (vid, int(st))
                c = counts.get(key)
                if c is None or key in seen:
                    continue
                seen.add(key)
                m = _processed_s(st, et) / 60.0
                tod = int(tods[i][j]) if i < len(tods) and j < len(tods[i]) else 0
                p = per[row["id"]]
                for k, v in (("persons", c[0]), ("cars", c[1]), ("motorcycles", c[2]), ("minutes", m)):
                    p[k] += v
                    p[f"{k}_{'night' if tod else 'day'}"] += v
    return per


def _rates(acc):
    out = {"hours": round(acc["minutes"] / 60, 1)}
    if acc["minutes"] > 0:
        out["persons_per_min"] = round(acc["persons"] / acc["minutes"], 2)
        out["cars_per_min"] = round(acc["cars"] / acc["minutes"], 2)
    for tod in ("day", "night"):
        if acc[f"minutes_{tod}"] >= 60:            # at least an hour of each, as CROWD's day/night figure
            out[f"persons_per_min_{tod}"] = round(acc[f"persons_{tod}"] / acc[f"minutes_{tod}"], 2)
    return out


def speeds(path):
    """crowd-city's per-city crossing speeds: {(locality, state, country): {...}}."""
    out = {}
    for r in csv.DictReader(open(path, encoding="utf-8")):
        if r.get("motion_unit") != "m/s" or not r.get("median_crossing_speed_mps"):
            continue
        out[(r["locality"], r.get("state") or "", r["country"])] = {
            "speed_mps": round(float(r["median_crossing_speed_mps"]), 3),
            "speed_q25": round(float(r["crossing_speed_mps_q25"]), 3),
            "speed_q75": round(float(r["crossing_speed_mps_q75"]), 3),
            "speed_n": int(r["reliable_metric_speed_tracks"]),
            # share of the persons tracked in front of the dashcam that crossed its road
            "crossing_tracks": int(r["detected_crossing_tracks"]),
            "person_tracks": int(r["person_tracks"]),
        }
    return out


def jaywalking(folder):
    """Per country: crossings at a marked crosswalk or pedestrian light (COMPLIANT), without (JAYWALKING), the
    analysed hours and the runs used. A segment in several runs counts once, from the latest run."""
    folder = Path(folder)
    seg_run = {}
    for run in JAYWALK_RUNS:
        p = folder / run / "per_video_results.csv"
        if p.exists():
            for r in csv.DictReader(open(p, encoding="utf-8")):
                seg_run[(r["video_id"], r["start_second"])] = (run, r)
    out = defaultdict(lambda: {"compliant": 0, "jaywalking": 0, "hours": 0.0, "runs": set()})
    for (vid, st), (run, r) in seg_run.items():
        end = float(r["effective_end_second"] or r["end_second"])
        out[r["country"]]["hours"] += (end - float(st)) / 3600
    for run in JAYWALK_RUNS:
        p = folder / run / "per_person_results.csv"
        if not p.exists():
            continue
        for r in csv.DictReader(open(p, encoding="utf-8")):
            if seg_run.get((r["video_id"], r["start_second"]), (None,))[0] != run:
                continue
            if r["predicted_crossing"] != "True":
                continue
            lab = r["jaywalking_label"]
            if lab in ("COMPLIANT", "JAYWALKING"):
                c = out[r["country"]]
                c[lab.lower()] += 1
                c["runs"].add(run)
    return out


def _wmedian(pairs):
    """Weighted median of (value, weight)."""
    pairs = sorted(p for p in pairs if p[1] > 0)
    total, acc = sum(w for _, w in pairs), 0.0
    for v, w in pairs:
        acc += w
        if acc >= total / 2:
            return v
    return None


def build(args):
    rows = list(csv.DictReader(open(args.mapping, encoding="utf-8")))
    iso2 = {r["country"]: r["iso2"] for r in csv.DictReader(open(args.countries, encoding="utf-8"))}
    iso2.update(ISO2_EXTRA)
    per_loc = detections(rows, args.detections)
    spd = speeds(args.speeds)
    jw = jaywalking(args.jaywalking) if args.jaywalking else {}

    country_acc = defaultdict(lambda: defaultdict(float))
    cont_acc = defaultdict(lambda: defaultdict(float))
    world_acc = defaultdict(float)
    country_info, n_loc = {}, defaultdict(int)
    localities = []
    for r in rows:
        acc = per_loc.get(r["id"])
        iso3 = r["iso3"]
        country_info.setdefault(iso3, {"name": r["country"], "iso2": iso2.get(r["country"], ""),
                                       "continent": r["continent"]})
        try:
            lat, lon = float(r["lat"]), float(r["lon"])
        except ValueError:
            continue
        entry = {"name": r["locality"], "iso3": iso3, "lat": round(lat, 4), "lon": round(lon, 4)}
        if r.get("state"):
            entry["state"] = r["state"]
        if acc:
            n_loc[iso3] += 1
            for k, v in acc.items():
                country_acc[iso3][k] += v
                cont_acc[r["continent"]][k] += v
                world_acc[k] += v
            entry.update(_rates(acc))
        s = spd.get((r["locality"], r.get("state") or "", r["country"])) or (
            spd.get((r["locality"], "", r["country"])) if not any(
                k[0] == r["locality"] and k[2] == r["country"] and k[1] for k in spd) else None)
        if s:
            entry.update(s)
        if acc or s:
            localities.append(entry)

    def speed_summary(items):
        """Across cities: median speed, quartiles and crossing share, weighted by tracks."""
        items = [s for s in items if s.get("speed_n")]
        if not items:
            return {}
        return {"speed_mps": _wmedian([(s["speed_mps"], s["speed_n"]) for s in items]),
                "speed_q25": _wmedian([(s["speed_q25"], s["speed_n"]) for s in items]),
                "speed_q75": _wmedian([(s["speed_q75"], s["speed_n"]) for s in items]),
                "speed_n": sum(s["speed_n"] for s in items), "speed_cities": len(items),
                "crossing_share": round(sum(s["crossing_tracks"] for s in items) /
                                        max(1, sum(s["person_tracks"] for s in items)), 4)}

    countries = {}
    for iso3, info in sorted(country_info.items()):
        c = dict(info)
        if iso3 in country_acc:
            c.update(_rates(country_acc[iso3]))
            c["localities"] = n_loc[iso3]
        c.update(speed_summary([l for l in localities if l["iso3"] == iso3]))
        j = jw.get(info["name"])
        if j and j["compliant"] + j["jaywalking"]:
            n = j["compliant"] + j["jaywalking"]
            c.update({"jaywalk_share": round(j["jaywalking"] / n, 4), "jaywalk_n": n,
                      "jaywalk_hours": round(j["hours"], 1),
                      "crossings_per_hour": round(n / j["hours"], 1) if j["hours"] else None,
                      "jaywalk_runs": sorted(j["runs"])})
        if len(c) > 3:
            countries[iso3] = c
    continents = {}
    for cont, acc in sorted(cont_acc.items()):
        isos = [k for k, v in country_info.items() if v["continent"] == cont]
        continents[cont] = {**_rates(acc), **speed_summary([l for l in localities if l["iso3"] in isos])}
    world = {**_rates(world_acc), **speed_summary(localities)}
    # the reference for a place's crossing share: the median city (the pooled share is dominated by a few dense
    # cities where most tracked pedestrians stand or walk along the road)
    shares = sorted(l["crossing_tracks"] / l["person_tracks"] for l in localities if l.get("person_tracks"))
    world["crossing_share_city_median"] = round(statistics.median(shares), 4) if shares else None

    # jaywalking where CROWD has not measured it: logit(share) ~ log(pedestrians per minute) over the measured
    # countries (persons per minute is CROWD's broadest measure; it rises with informal, walking-heavy streets)
    pts = [(math.log(c["persons_per_min"]), c["jaywalk_share"], c["jaywalk_n"]) for c in countries.values()
           if "jaywalk_share" in c and c.get("persons_per_min") and c["jaywalk_n"] >= 50]
    model = None
    if len(pts) >= 4:
        xs = [x for x, _, _ in pts]
        # Laplace-smoothed shares, so 0 of 218 is not -inf
        ys = [math.log((s * n + 0.5) / (n - s * n + 0.5)) for _, s, n in pts]
        ws = [math.sqrt(n) for _, _, n in pts]
        sw = sum(ws)
        mx, my = sum(w * x for w, x in zip(ws, xs)) / sw, sum(w * y for w, y in zip(ws, ys)) / sw
        b = sum(w * (x - mx) * (y - my) for w, x, y in zip(ws, xs, ys)) / sum(w * (x - mx) ** 2 for w, x in zip(ws, xs))
        a = my - b * mx
        res = [y - (a + b * x) for x, y in zip(xs, ys)]
        model = {"form": "logit(jaywalk_share) = a + b * ln(persons_per_min)", "a": round(a, 3), "b": round(b, 3),
                 "countries": len(pts), "residual_sd": round(statistics.pstdev(res), 2)}
    world["jaywalk_share"] = round(statistics.median(c["jaywalk_share"] for c in countries.values()
                                                     if "jaywalk_share" in c), 4) if jw else None

    places = defaultdict(set)
    for r in rows:
        try:
            places[r["iso3"]].add((round(float(r["lat"]), 2), round(float(r["lon"]), 2)))
        except ValueError:
            pass
    places = {k: sorted(v) for k, v in sorted(places.items())}
    data = {
        "version": 1,
        "source": {
            "dataset": "CROWD: City Road Observations With Dashcams (crowd-dataset/crowd)",
            "crowd_version": args.crowd_version,
            "speeds_version": args.speeds_version,
            "jaywalking_version": args.jaywalking_version,
            "licence": "CC BY 4.0 (CROWD DATA_LICENSE.md); YouTube videos are not redistributed",
            "citation": "Alam, M. S., Bazilinska, O., & Bazilinskyy, P. (2026). A global dataset of continuous urban "
                        "dashcam driving. arXiv:2604.01044",
            "notes": "persons/cars per minute: unique YOLO (yolo11x) tracks with confidence >= 0.7 per processed minute "
                     "of footage, as CROWD's README tables; speed: crowd-city's Waymo-calibrated median crossing speed "
                     "per city (m/s, q25-q75), country = track-weighted median over its cities; crossing_share: "
                     "crossing tracks / person tracks of crowd-city; jaywalk_share: crowd-jaywalking's crossings "
                     "without a visible marked crosswalk or pedestrian light / crossings with a VLM verdict.",
        },
        "jaywalk_model": model,
        "world": world,
        "continents": continents,
        "countries": countries,
        "localities": sorted([l for l in localities if l.get("hours", 0) >= args.min_locality_hours or "speed_mps" in l],
                             key=lambda l: (l["iso3"], l["name"])),
        # every CROWD locality, for finding a map's country offline: {iso3: [[lat, lon], ...]} (0.01 degree)
        "places": places,
    }
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mapping", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--countries", required=True)
    ap.add_argument("--speeds", required=True)
    ap.add_argument("--jaywalking")
    ap.add_argument("--crowd-version", default="")
    ap.add_argument("--speeds-version", default="")
    ap.add_argument("--jaywalking-version", default="")
    ap.add_argument("--min-locality-hours", type=float, default=2.0)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    data = build(args)
    Path(args.out).write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    print(f"{args.out}: {len(data['countries'])} countries, {len(data['localities'])} localities, "
          f"{sum(map(len, data['places'].values()))} places; jaywalking model {data['jaywalk_model']}")


if __name__ == "__main__":
    main()
