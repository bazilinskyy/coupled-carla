"""Stand a baked level's scenery on the level's own ground (a running CARLA server with the level is needed).

    python tools/snap_scenery.py experiments/eindhoven_twin.yaml [--port 2000]

scenery.py places trees, bushes, lamps and furniture on a baked level (map.carla_level) at the terrain model's
heights (AHN in the Netherlands, base 0), which the level's ground follows only roughly: on the Eindhoven twin the
two agree within 0.3 m at 58% of the objects, but differ by more than 1 m at 12% (ramps, embankments, the station
platforms). This tool loads the level, casts a ray down at every such object and moves it so that its bottom
stands on the level's ground (the ground-like hit nearest the planned height, so an object under a bridge stays
under it). Road signs stand on the OpenDRIVE road height already and are left alone. It rewrites scenery.json and
records what it did there ("snapped"); run it again after each `mapbuild --scenery` of the map.
"""
import argparse
import json
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, run, scenery  # noqa: E402

GROUND = {"terrain", "ground", "sidewalks", "roads", "roadlines"}
SINK = {"tree": 0.1}        # as scenery.build sinks them
KINDS = ("tree", "bush", "lamp", "furniture")
CELL = 200.0                # the level streams its ground in around the spectator: one cell at a time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--max-shift", type=float, default=4.0, help="leave objects that would move more (m)")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    if not cfg["map"].get("carla_level"):
        sys.exit("only for a baked level (map.carla_level)")
    mdir = config.map_dir(cfg)
    f = mdir / "scenery.json"
    data = json.loads(f.read_text())
    cat = scenery.Catalog(data["catalog"])
    client = carla.Client(cfg["server"]["host"], args.port or cfg["server"]["port"])
    client.set_timeout(60.0)
    world = run.load_world(client, mdir / "map.xodr", cfg["map"])
    spectator = world.get_spectator()
    sync = world.get_settings().synchronous_mode
    tick = world.tick if sync else world.wait_for_tick
    cells = defaultdict(list)
    for i, it in enumerate(data["items"]):
        if it[6] in KINDS and len(it) <= 7:
            cells[(int(it[1] // CELL), int(it[2] // CELL))].append(i)
    shifts, missed, skipped = [], 0, 0
    t0 = time.time()
    for (cx, cy), idx in sorted(cells.items()):
        x0, y0 = (cx + 0.5) * CELL, (cy + 0.5) * CELL
        spectator.set_transform(carla.Transform(carla.Location(x=x0, y=y0, z=100.0), carla.Rotation(pitch=-90)))
        for _ in range(60):     # until the ground under the cell's centre has streamed in (at most ~6 s)
            tick()
            if any(str(h.label).split(".")[-1].lower() in GROUND for h in world.cast_ray(
                    carla.Location(x=x0, y=y0, z=400.0), carla.Location(x=x0, y=y0, z=-200.0))):
                break
            time.sleep(0.1)
        for i in idx:
            it = data["items"][i]
            name = data["meshes"][it[0]].rsplit(".", 1)[-1]
            m = cat.by_name[name][1] if name in cat else None
            if m is None:
                continue
            sink = SINK.get(it[6], 0.0)
            bottom = it[3] + it[5] * (m["off"][2] - m["ext"][2])
            planned = bottom + sink
            hits = [h.location.z for h in world.cast_ray(carla.Location(x=it[1], y=it[2], z=planned + 30.0),
                                                         carla.Location(x=it[1], y=it[2], z=planned - 30.0))
                    if str(h.label).split(".")[-1].lower() in GROUND]
            if not hits:
                missed += 1
                continue
            g = min(hits, key=lambda z: abs(z - planned))
            d = g - planned
            if abs(d) > args.max_shift:
                skipped += 1
                continue
            it[3] = round(it[3] + d, 2)
            shifts.append(d)
    a = sorted(abs(d) for d in shifts)
    data["snapped"] = {"level": cfg["map"]["carla_level"], "items": len(shifts), "no_ground": missed,
                       "left_alone": skipped, "median_shift_m": round(statistics.median(shifts), 2) if shifts else 0,
                       "p95_abs_shift_m": round(a[int(0.95 * (len(a) - 1))], 2) if a else 0,
                       "when": time.strftime("%Y-%m-%d %H:%M")}
    f.write_text(json.dumps(data, separators=(",", ":")))
    print(f"snapped {len(shifts)} objects to the level's ground in {time.time() - t0:.0f} s "
          f"({missed} without ground under them, {skipped} would move > {args.max_shift} m): {data['snapped']}")


if __name__ == "__main__":
    main()
