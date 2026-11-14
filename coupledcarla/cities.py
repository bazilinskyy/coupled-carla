"""Write one experiment file per city of experiments/cities.yaml and build their maps.

    python -m coupledcarla.cities                 # write experiments/cities/*.yaml and build every map
    python -m coupledcarla.cities --only lyon     # one city
    python -m coupledcarla.cities --no-build      # only write the experiment files
"""
import argparse
import math
import time
import traceback
from pathlib import Path

import yaml

from . import config, mapbuild

ROOT = Path(__file__).resolve().parent.parent


def bbox(lat, lon, size_km):
    half = size_km / 2.0
    dlat = half / 111.32
    dlon = half / (111.32 * math.cos(math.radians(lat)))
    return [round(lat - dlat, 5), round(lon - dlon, 5), round(lat + dlat, 5), round(lon + dlon, 5)]


def experiment(c, cat):
    traffic = dict(cat["defaults"]["traffic"])
    traffic.update(cat["size_traffic"].get(c["size"], {}))
    traffic.update(c.get("traffic", {}))
    return {
        "name": c["slug"],
        "city": {k: c[k] for k in ("name", "country", "continent", "size", "population", "lat", "lon", "drive")},
        "map": {"bbox": bbox(c["lat"], c["lon"], cat["size_km"]), "lefthand": c["drive"] == "left",
                **c.get("map", {})},
        "traffic": traffic,
        "participants": [{"role": "driver", "vehicle": "vehicle.lincoln.mkz_2020", "input": "keyboard",
                          "view": "driver"}],   # no spawn: a random spawn point of the map
        "trials": [{"duration_s": 300}],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", action="append")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    cat = yaml.safe_load((ROOT / "experiments" / "cities.yaml").read_text())
    out = ROOT / "experiments" / "cities"
    out.mkdir(exist_ok=True)
    results = {}
    for c in cat["cities"]:
        if args.only and c["slug"] not in args.only:
            continue
        path = out / f"{c['slug']}.yaml"
        path.write_text(f"# {c['name']}, {c['country']} ({c['continent']}, {c['size']}, ~{c['population']:,} "
                        f"people, {c['drive']}-hand traffic). Generated from experiments/cities.yaml.\n"
                        + yaml.safe_dump(experiment(c, cat), sort_keys=False, allow_unicode=True))
        if args.no_build:
            continue
        try:
            mapbuild.build(config.load(path), force=args.force)
            results[c["slug"]] = "ok"
        except Exception as err:  # noqa: BLE001 - one city failing should not stop the others
            traceback.print_exc()
            results[c["slug"]] = f"FAILED: {err}"
        time.sleep(5)   # be gentle with the public Overpass servers
    for k, v in results.items():
        print(f"{k:20s} {v}")


if __name__ == "__main__":
    main()
