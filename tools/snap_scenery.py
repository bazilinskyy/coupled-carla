"""Stand a baked level's scenery on the level's own ground (a running CARLA server with the level is needed).

    python tools/snap_scenery.py experiments/eindhoven_twin.yaml [--port 2000]

scenery.py places trees, bushes, lamps and furniture on a baked level (map.carla_level) at the terrain model's
heights (AHN in the Netherlands, base 0), which the level's ground follows only roughly: on the Eindhoven twin the
two agree within 0.3 m at 58% of the objects, but differ by more than 1 m at 12% (ramps, embankments, the station
platforms). This tool loads the level, casts a ray down at every such object and moves it so that its bottom
stands on the level's ground (the ground-like hit nearest the planned height, so an object under a bridge stays
under it). Where the planned heights are off by a whole area (a twin whose level stands on another terrain than
the map's), each object moves by its area's median offset first; one with nothing at all above or below it (past
the level's edge) is left out, and so is a rooftop unit with no roof of the level under it. Road signs stand on the
OpenDRIVE road height already and are left alone. It rewrites scenery.json and
records what it did there ("snapped"); run it again after each `mapbuild --scenery` of the map.

The parked cars (parked.json, parking.py) are snapped the same way: rays at the four wheels of each car from 3 m
above its planned bottom (a bridge or deck higher than that stays above it), the car's bottom set to the highest of
their first ground hits (the wheel that touches first), and its window glass moved with it. A car whose wheels' ground
differs by more than 0.3 m (on a kerb, a mound or a step of the level's ground), whose middle stands on ground more
than 0.15 m above its wheels, or whose box a building, wall, pole or raised ground cuts (rays along its diagonals at
0.3 and 1.0 m) is taken out of parked.json.
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
SOFT = {"vegetation", "none", "unlabeled", "other", "sky"}     # leaves, invisible volumes: not in a parked car's way
SINK = {"tree": 0.1}        # as scenery.build sinks them
KINDS = ("tree", "bush", "lamp", "furniture")
ROOF = "roof"               # rooftop units (streetlook.py): onto the level's own roof below them, or left out
BUILDING = {"buildings", "building"}
ROOF_MAX_SHIFT = 25.0       # m: a roof further from the planned one is another building's (or none): left out
REACH = 300.0               # m: a ray this far up and down that hits nothing at all: no level there
AREA = 100.0                # m: objects in one such square share a planned-height offset
CELL = 200.0                # the level streams its ground in around the spectator: one cell at a time


def drop_items(data, drop):
    """Take items out of scenery.json and renumber what points at them: lamp_night and leaf rows start with an
    item index, prov["items"] has one entry per item."""
    new = {}
    for i in range(len(data["items"])):
        if i not in drop:
            new[i] = len(new)
    data["items"] = [it for i, it in enumerate(data["items"]) if i not in drop]
    for key in ("lamp_night", "leaf"):
        if data.get(key):
            data[key] = [[new[r[0]]] + list(r[1:]) for r in data[key] if r[0] in new]
    pv = data.get("prov")
    if isinstance(pv, dict) and isinstance(pv.get("items"), list):
        pv["items"] = [v for i, v in enumerate(pv["items"]) if i not in drop]


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
        if ((it[6] in KINDS and len(it) <= 7) or it[6] == ROOF) and not args.parked_only:
            cells[(int(it[1] // CELL), int(it[2] // CELL))].append(i)
    shifts, missed, skipped = [], 0, 0
    roof_shifts, roof_drop, void = [], set(), set()
    cand = {}                   # item -> (planned bottom, ground hits): placed after the pass, by its area's offset
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
            if it[6] == ROOF:
                # the scenery's roofs come from OSM/Overture heights; a twin's level has its own buildings (3DBAG,
                # PLATEAU, Catastro): the unit goes on the highest building surface under it, or out where the
                # level has no roof there (Shibuya 2026-11-24: units floating 5-30 m over the PLATEAU roofs)
                tops = [h.location.z for h in world.cast_ray(carla.Location(x=it[1], y=it[2], z=planned + 80.0),
                                                             carla.Location(x=it[1], y=it[2], z=planned - 80.0))
                        if str(h.label).split(".")[-1].lower() in BUILDING]
                if not tops or abs(max(tops) - planned) > ROOF_MAX_SHIFT:
                    roof_drop.add(i)
                    continue
                d = max(tops) - planned
                it[3] = round(it[3] + d, 2)
                roof_shifts.append(d)
                continue
            allhits = world.cast_ray(carla.Location(x=it[1], y=it[2], z=planned + REACH),
                                     carla.Location(x=it[1], y=it[2], z=planned - REACH))
            if not allhits:         # nothing at all above or below: past the level's edge, it would float in the void
                void.add(i)
                continue
            hits = [h.location.z for h in allhits if str(h.label).split(".")[-1].lower() in GROUND]
            if not hits:
                missed += 1
                continue
            cand[i] = (planned, hits)
    # the planned heights may be off by a whole area (a twin whose level stands on another terrain than the map's,
    # Salamanca 2026-11-24: 4-30 m, so 844 objects were left floating or buried): each object moves to the ground
    # nearest its planned height plus the median offset of its area's objects, and is left alone only when that
    # ground is more than max_shift from where its neighbours put it (under a bridge, on a deck)
    near = {i: min(h, key=lambda z, p=p: abs(z - p)) - p for i, (p, h) in cand.items()}
    area = defaultdict(list)
    for i, d in near.items():
        it = data["items"][i]
        area[(int(it[1] // AREA), int(it[2] // AREA))].append(d)
    offs = {k: statistics.median(v) for k, v in area.items()}
    for i, (planned, hits) in cand.items():
        it = data["items"][i]
        off = offs[(int(it[1] // AREA), int(it[2] // AREA))]
        g = min(hits, key=lambda z: abs(z - planned - off))
        d = g - planned
        if abs(d - off) > args.max_shift:
            skipped += 1
            continue
        it[3] = round(it[3] + d, 2)
        shifts.append(d)
    if roof_drop or void:
        drop_items(data, roof_drop | void)
    a = sorted(abs(d) for d in shifts)
    ra = sorted(abs(d) for d in roof_shifts)
    data["snapped"] = {"roof_units": len(roof_shifts), "roof_units_left_out": len(roof_drop),
                       "roof_p95_abs_shift_m": round(ra[int(0.95 * (len(ra) - 1))], 2) if ra else 0,
                       "level": cfg["map"]["carla_level"], "items": len(shifts), "no_ground": missed,
                       "left_alone": skipped, "past_the_level_left_out": len(void),
                       "median_shift_m": round(statistics.median(shifts), 2) if shifts else 0,
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
    shifts, missed, skipped, spread, drop, blocked = [], 0, 0, [], set(), 0
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
                hits = [h.location.z for h in world.cast_ray(carla.Location(x=x, y=y, z=planned + 3.0),
                                                             carla.Location(x=x, y=y, z=planned - 20.0))
                        if str(h.label).split(".")[-1].lower() in GROUND]
                if hits:
                    gs.append(hits[0])      # the first ground from above: what the wheel stands on
            if not gs:
                missed += 1
                continue
            d = max(gs) - planned
            if abs(d) > max_shift:
                skipped += 1
                continue
            spread.append(max(gs) - min(gs))
            if len(gs) < 4 or max(gs) - min(gs) > 0.3:
                drop.add(i)
                continue
            bottom = max(gs)
            mid = [h.location.z for h in world.cast_ray(carla.Location(x=ox, y=oy, z=bottom + 3.0),
                                                        carla.Location(x=ox, y=oy, z=bottom - 3.0))
                   if str(h.label).split(".")[-1].lower() in GROUND]
            cut = bool(mid) and mid[0] > bottom + 0.15
            corners = [(ox + a * 0.95 * m["ext"][0] * ux + b * 0.95 * m["ext"][1] * vx,
                        oy + a * 0.95 * m["ext"][0] * uy + b * 0.95 * m["ext"][1] * vy)
                       for a, b in ((1, 1), (-1, -1), (1, -1), (-1, 1))]
            for hz in (0.3, 1.0):
                for (x0, y0), (x1, y1) in ((corners[0], corners[1]), (corners[2], corners[3])):
                    for h in world.cast_ray(carla.Location(x=x0, y=y0, z=bottom + hz),
                                            carla.Location(x=x1, y=y1, z=bottom + hz)):
                        if str(h.label).split(".")[-1].lower() not in SOFT:
                            cut = True
            if cut:         # a building, wall or post in the car, or raised ground under its middle
                drop.add(i)
                blocked += 1
                continue
            sl[3] = round(sl[3] + d, 2)
            if i + 1 < len(slots) and len(slots[i + 1]) > 8:      # its window glass
                slots[i + 1][3] = sl[3]
            shifts.append(d)
    a = sorted(abs(d) for d in shifts)
    sp = sorted(spread)
    keep = []
    for i, sl in enumerate(slots):      # cars on uneven ground go, with their glass
        if i in drop or (len(sl) > 8 and i - 1 in drop):
            continue
        keep.append(sl)
    pdata["slots"] = keep
    pdata["snapped"] = {"level": level, "cars": len(shifts), "uneven_dropped": len(drop) - blocked,
                        "obstructed_dropped": blocked, "no_ground": missed,
                        "left_alone": skipped,
                        "median_shift_m": round(statistics.median(shifts), 2) if shifts else 0,
                        "p95_abs_shift_m": round(a[int(0.95 * (len(a) - 1))], 2) if a else 0,
                        "p95_wheel_spread_m": round(sp[int(0.95 * (len(sp) - 1))], 2) if sp else 0,
                        "when": time.strftime("%Y-%m-%d %H:%M")}
    pf.write_text(json.dumps(pdata, separators=(",", ":")))
    print(f"snapped {len(shifts)} parked cars in {time.time() - t0:.0f} s: {pdata['snapped']}")


if __name__ == "__main__":
    main()
