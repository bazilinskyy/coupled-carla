"""Collect the plate spots a CARLA 1.0 server measured on vehicles without plate material into
coupledcarla/data/plate_spots.json, which coupled-carla sends with the country (plates.value_for) to servers that
cannot read their meshes (a cooked package keeps no CPU copy of them).

The server (LicensePlates.cpp) logs, for each such vehicle it spawns with a license_plate attribute,
    LogCarla: Licence plates: body spots vehicle.lincoln.mkz measured;rear=-231.4,0.0,52.0,-0.998,0.000,0.060;front=...

    python tools/plate_spots.py <server log> [<server log> ...] [--out coupledcarla/data/plate_spots.json]

Spots already in the file are kept unless a log measured the vehicle again.
"""
import argparse
import json
import re
import sys
from pathlib import Path

LINE = re.compile(r"Licence plates: body spots (\S+) measured((?:;(?:rear|front)=[-0-9.,]+)+)")
SPOT = re.compile(r"(rear|front)=([-0-9.,]+)")
OUT = Path(__file__).resolve().parents[1] / "coupledcarla" / "data" / "plate_spots.json"


def parse(text):
    """{blueprint id: {"rear": [x, y, z, nx, ny, nz], "front": [...]}} of a server log's measured spots."""
    out = {}
    for bp, spots in LINE.findall(text):
        sides = {}
        for side, nums in SPOT.findall(spots):
            v = [float(n) for n in nums.split(",") if n]
            if len(v) == 6:
                sides[side] = v
        if sides:
            out[bp] = sides
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args(argv)
    try:
        data = json.loads(Path(a.out).read_text())
    except (OSError, ValueError):
        data = {}
    spots = dict(data.get("spots") or {})
    new = {}
    for log in a.logs:
        new.update(parse(Path(log).read_text(errors="replace")))
    spots.update(new)
    data = {"about": "plate spots (cm, the body mesh's component space: x forward, z up; centre then outward normal) "
                     "of CARLA 1.0 vehicles without plate material, measured by the server (LicensePlates.cpp "
                     "BodySpots); tools/plate_spots.py", "spots": dict(sorted(spots.items()))}
    lines = [f" {json.dumps(bp)}: {json.dumps(sides, sort_keys=True)}" for bp, sides in data["spots"].items()]
    Path(a.out).write_text('{"about": %s,\n"spots": {\n%s\n}}\n' % (json.dumps(data["about"]), ",\n".join(lines)))
    print(f"{len(new)} vehicles measured, {len(spots)} in {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
