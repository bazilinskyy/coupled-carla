"""Stand a baked level's scenery on the level's own ground (a running CARLA server with the level is needed).

    python tools/snap_scenery.py experiments/eindhoven_twin.yaml [--port 2000]

scenery.py places trees, bushes, lamps and furniture on a baked level (map.carla_level) at the terrain model's
heights (AHN in the Netherlands, base 0), which the level's ground follows only roughly: on the Eindhoven twin the
two agree within 0.3 m at 58% of the objects, but differ by more than 1 m at 12% (ramps, embankments, the station
platforms). This tool loads the level, casts a ray down at every such object and moves it so that its bottom
stands on the level's ground (the ground-like hit nearest the planned height, so an object under a bridge stays
under it). Road signs stand on the OpenDRIVE road height already and are left alone. It rewrites scenery.json and
records what it did there ("snapped"); run it again after each `mapbuild --scenery` of the map.

The parked cars (parked.json, parking.py) are snapped the same way: rays at the four wheels of each car, the car's
bottom set to the highest of their ground hits (the wheel that touches first), and its window glass moved with it.
"""
import argparse
import json
import math
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, parking, run, scenery  # noqa: E402

GROUND = {"terrain", "ground", "sidewalks", "roads", "roadlines"}
SINK = {"tree": 0.1}        # as scenery.build sinks them
KINDS = ("tree", "bush", "lamp", "furniture")
CELL = 200.0                # the level streams its ground in around the spectator: one cell at a time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--max-shift", type=float, default=4.0, help="leave objects that would move more (m)")
    ap.add_argument("--parked-only", action="store_true", help="snap only parked.json (scenery.json as it is)")
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
    def stream(cx, cy):
        x0, y0 = (cx + 0.5) * CELL, (cy + 0.5) * CELL
        spectator.set_transform(carla.Transform(carla.Location(x=x0, y=y0, z=100.0), carla.Rotation(pitch=-90)))
        for _ in range(60):     # until the ground under the cell's centre has streamed in (at most ~6 s)
            tick()
            if any(str(h.label).split(".")[-1].lower() in GROUND for h in world.cast_ray(
                    carla.Location(x=x0, y=y0, z=400.0), carla.Location(x=x0, y=y0, z=-200.0))):
                break
            time.sleep(0.1)

    cells = defaultdict(list)
    for i, it in enumerate(data["items"]):
        if it[6] in KINDS and len(it) <= 7 and not args.parked_only:
            cells[(int(it[1] // CELL), int(it[2] // CELL))].append(i)
    shifts, missed, skipped = [], 0, 0
    t0 = time.time()
    for (cx, cy), idx in sorted(cells.items()):
        stream(cx, cy)
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
    if not args.parked_only:
        f.write_text(json.dumps(data, separators=(",", ":")))
    print(f"snapped {len(shifts)} objects to the level's ground in {time.time() - t0:.0f} s "
          f"({missed} without ground under them, {skipped} would move > {args.max_shift} m): {data['snapped']}")
    pf = mdir / "parked.json"
    if pf.exists():
        snap_parked(pf, world, stream, args.max_shift, cfg["map"]["carla_level"])


def snap_parked(pf, world, stream, max_shift, level):
    """Stand parked.json's cars on the level's ground: the bottom of each car's box at the highest ground hit under
    its four wheels (near the planned height); a car's glass slot (the one after it, flagged) moves with it."""
    pdata = json.loads(pf.read_text())
    cat = scenery.Catalog(pdata["catalog"])
    parking.car_meshes(cat)
    slots = pdata["slots"]
    cells = defaultdict(list)
    for i, sl in enumerate(slots):
        if len(sl) <= 8:
            cells[(int(sl[1] // CELL), int(sl[2] // CELL))].append(i)
    shifts, missed, skipped, spread = [], 0, 0, []
    t0 = time.time()
    for (cx, cy), idx in sorted(cells.items()):
        stream(cx, cy)
        for i in idx:
            sl = slots[i]
            name = pdata["meshes"][sl[0]].rsplit(".", 1)[-1]
            if name not in cat:
                continue
            m = cat.by_name[name][1]
            planned = sl[3] + m["off"][2] - m["ext"][2]
            r = math.radians(sl[4])
            ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
            ox, oy = sl[1] + m["off"][0] * ux + m["off"][1] * vx, sl[2] + m["off"][0] * uy + m["off"][1] * vy
            gs = []
            for a, b in ((0.65, 0.75), (0.65, -0.75), (-0.65, 0.75), (-0.65, -0.75)):    # the wheels
                x, y = ox + a * m["ext"][0] * ux + b * m["ext"][1] * vx, oy + a * m["ext"][0] * uy + b * m["ext"][1] * vy
                hits = [h.location.z for h in world.cast_ray(carla.Location(x=x, y=y, z=planned + 20.0),
                                                             carla.Location(x=x, y=y, z=planned - 20.0))
                        if str(h.label).split(".")[-1].lower() in GROUND]
                if hits:
                    gs.append(min(hits, key=lambda z: abs(z - planned)))
            if not gs:
                missed += 1
                continue
            d = max(gs) - planned
            if abs(d) > max_shift:
                skipped += 1
                continue
            spread.append(max(gs) - min(gs))
            sl[3] = round(sl[3] + d, 2)
            if i + 1 < len(slots) and len(slots[i + 1]) > 8:      # its window glass
                slots[i + 1][3] = sl[3]
            shifts.append(d)
    a = sorted(abs(d) for d in shifts)
    sp = sorted(spread)
    pdata["snapped"] = {"level": level, "cars": len(shifts), "no_ground": missed, "left_alone": skipped,
                        "median_shift_m": round(statistics.median(shifts), 2) if shifts else 0,
                        "p95_abs_shift_m": round(a[int(0.95 * (len(a) - 1))], 2) if a else 0,
                        "p95_wheel_spread_m": round(sp[int(0.95 * (len(sp) - 1))], 2) if sp else 0,
                        "when": time.strftime("%Y-%m-%d %H:%M")}
    pf.write_text(json.dumps(pdata, separators=(",", ":")))
    print(f"snapped {len(shifts)} parked cars in {time.time() - t0:.0f} s: {pdata['snapped']}")


if __name__ == "__main__":
    main()
