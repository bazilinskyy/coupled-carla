"""Check a map's parked cars (parked.json) on a running CARLA server, without rendering (works on -nullrhi).

    python tools/parked_check.py experiments/eindhoven_twin.yaml [--port 2010] [--radius 400] [--no-load]

Loads the experiment's world as the run loop does, then for every stall within --radius of the first participant's
spawn (all stalls, not only those taken at the trial's hour), cell by cell (a baked World Partition level streams its
ground in only around the spectator, which is moved over each 200 m cell first):
- lanes: the car's footprint (its corners, edge middles and centre) against the CARLA map: how many cars stand on a
  driving (or bus, bidirectional) lane, a cycle lane or a sidewalk lane, strictly and with the footprint 10 cm in;
- level geometry, before the cars are spawned: vertical rays at 15 points of the footprint find raised ground under
  the car (a kerb or pavement more than 8 cm above its bottom) and anything else inside its box (buildings, walls,
  poles); horizontal rays along its sides and diagonals at 0.3 m and 1.0 m above its bottom find walls and posts
  between the points;
- spawning: how many cars (and their window glass) the server refuses, and the bottom of each car's bounding box
  minus the ground under its wheels.
"""
import argparse
import json
import math
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, parking, run, scenery  # noqa: E402
from coupledcarla.geo import Geo  # noqa: E402

GROUND = {"roads", "sidewalks", "terrain", "ground", "roadlines", "water", "railtrack", "bridge"}
SOFT = {"vegetation", "none", "unlabeled", "other", "sky"}       # leaves, invisible volumes: not an obstacle
CELL = 200.0


def label(h):
    return str(h.label).split(".")[-1].lower()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--radius", type=float, default=400.0)
    ap.add_argument("--no-load", action="store_true", help="use the world the server has loaded")
    ap.add_argument("--out", help="write per-car rows here (json)")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    mdir = config.map_dir(cfg)
    geo = Geo(mdir / "geo.net.xml")
    data = json.loads((mdir / "parked.json").read_text())
    cat = scenery.Catalog(data["catalog"])
    parking.car_meshes(cat)
    client = carla.Client(cfg["server"]["host"], args.port or cfg["server"]["port"])
    client.set_timeout(60.0)
    world = client.get_world() if args.no_load else run.load_world(client, mdir / "map.xodr", cfg["map"])
    sync = world.get_settings().synchronous_mode
    tick = world.tick if sync else world.wait_for_tick
    cmap = world.get_map()
    sp = cfg["participants"][0]["spawn"]
    cx, cy = geo.to_carla(sp["lat"], sp["lon"])
    spectator = world.get_spectator()

    def stream(i, j):
        x0, y0 = (i + 0.5) * CELL, (j + 0.5) * CELL
        spectator.set_transform(carla.Transform(carla.Location(x=x0, y=y0, z=100.0), carla.Rotation(pitch=-90)))
        for _ in range(60):     # until the ground under the cell's centre has streamed in (at most ~6 s)
            tick()
            if any(label(h) in GROUND for h in world.cast_ray(carla.Location(x=x0, y=y0, z=400.0),
                                                             carla.Location(x=x0, y=y0, z=-200.0))):
                break
            time.sleep(0.1)
        for _ in range(10):
            tick()
    slots = data["slots"]
    sel = [i for i, s in enumerate(slots) if len(s) <= 8 and math.hypot(s[1] - cx, s[2] - cy) <= args.radius]
    print(f"server {client.get_server_version()}, map {cmap.name}, parked.json {data.get('catalog')} "
          f"({'snapped ' + str(data['snapped']) if 'snapped' in data else 'not snapped'}); "
          f"{len(sel)} of {sum(1 for s in slots if len(s) <= 8)} stalls within {args.radius:.0f} m")
    LT = carla.LaneType
    kinds = {"driving": LT.Driving | LT.Bidirectional | LT.Restricted, "biking": LT.Biking, "sidewalk": LT.Sidewalk}
    rows, labels = [], Counter()
    cells = defaultdict(list)
    for i in sel:
        cells[(int(slots[i][1] // CELL), int(slots[i][2] // CELL))].append(i)
    bp = world.get_blueprint_library().find("static.prop.mesh")
    refused, gaps, by_i = Counter(), [], {}
    t0 = time.time()
    for cell, idx in sorted(cells.items()):
        stream(*cell)
        check_cell(world, cmap, cat, data, slots, idx, kinds, rows, labels, by_i)
        spawn_cell(client, world, tick, bp, cat, data, slots, idx, refused, gaps, by_i)
    print(f"checked {len(rows)} cars in {len(cells)} cells in {time.time() - t0:.0f} s")
    report(rows, labels, kinds, refused, gaps, args.out)


def check_cell(world, cmap, cat, data, slots, idx, kinds, rows, labels, by_i):
    """Lanes under each car and the level's geometry inside its box (before the cars are spawned)."""
    for i in idx:
        s = slots[i]
        name = data["meshes"][s[0]].rsplit(".", 1)[-1]
        m = cat.by_name[name][1]
        r = math.radians(s[4])
        ux, uy, vx, vy = math.cos(r), math.sin(r), -math.sin(r), math.cos(r)
        ox, oy = s[1] + m["off"][0] * ux + m["off"][1] * vx, s[2] + m["off"][0] * uy + m["off"][1] * vy
        ex, ey = m["ext"][0], m["ext"][1]
        bottom = s[3] + m["off"][2] - m["ext"][2]
        top = s[3] + m["off"][2] + m["ext"][2]

        def pt(a, b, inset=0.0):
            return (ox + a * (ex - inset) * ux + b * (ey - inset) * vx, oy + a * (ex - inset) * uy + b * (ey - inset) * vy)
        probe = [(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)]
        row = {"i": i, "mesh": name, "x": s[1], "y": s[2], "src": data["sources"][s[7]]}
        for k, lt in kinds.items():
            for tag, inset in (("", 0.0), ("_in10", 0.1)):
                row[k + tag] = any(cmap.get_waypoint(carla.Location(x=x, y=y, z=s[3] + 0.5), project_to_road=False,
                                                     lane_type=lt) is not None
                                   for x, y in (pt(a, b, inset) for a, b in probe))
        # level geometry inside the car's box (nothing spawned yet)
        raised, inside = 0.0, set()
        for a in (-1, -0.5, 0, 0.5, 1):
            for b in (-1, 0, 1):
                x, y = pt(a, b, 0.05)
                for h in world.cast_ray(carla.Location(x=x, y=y, z=top + 3.0), carla.Location(x=x, y=y, z=bottom - 1.0)):
                    lab = label(h)
                    labels[lab] += 1
                    if lab in GROUND:
                        raised = max(raised, h.location.z - bottom)
                        break       # the ground: what is below it does not matter
                    if lab not in SOFT and bottom + 0.15 < h.location.z < top:
                        inside.add(lab)
        for hz in (0.3, 1.0):
            z = bottom + hz
            corners = [pt(a, b, 0.05) for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
            for (x0, y0), (x1, y1) in list(zip(corners, corners[1:] + corners[:1])) + [(corners[0], corners[2]),
                                                                                       (corners[1], corners[3])]:
                for h in world.cast_ray(carla.Location(x=x0, y=y0, z=z), carla.Location(x=x1, y=y1, z=z)):
                    lab = label(h)
                    if lab not in SOFT and (lab not in GROUND or h.location.z > bottom + 0.15):
                        inside.add(lab + "_h")
        row["raised_m"] = round(raised, 2)
        row["inside"] = sorted(inside)
        rows.append(row)
        by_i[i] = row


def spawn_cell(client, world, tick, bp, cat, data, slots, idx, refused, gaps, by_i):
    """Spawn the cell's cars (and their glass); the bottom of each car's box minus the ground under its wheels."""
    cmds, owner = [], []
    for i in idx:
        for j in (i, i + 1):
            if j == i + 1 and not (j < len(slots) and len(slots[j]) > 8):
                continue
            s = slots[j]
            bp.set_attribute("mesh_path", data["meshes"][s[0]])
            bp.set_attribute("scale", "1")
            cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(x=s[1], y=s[2], z=s[3]),
                                                                     carla.Rotation(yaw=s[4]))))
            owner.append((i, j != i))
    res = []
    for k in range(0, len(cmds), 500):
        res += client.apply_batch_sync(cmds[k:k + 500], False)
    tick()
    refused.update(("glass" if g else "car") for (i, g), r in zip(owner, res) if r.error)
    for (i, g), r in zip(owner, res):
        if g or r.error:
            continue
        a = world.get_actor(r.actor_id)
        tf, bb = a.get_transform(), a.bounding_box
        c = bb.location
        verts = [tf.transform(carla.Location(x=c.x + p * bb.extent.x, y=c.y + q * bb.extent.y, z=c.z + w * bb.extent.z))
                 for p in (-1, 1) for q in (-1, 1) for w in (-1, 1)]
        bottom = min(v.z for v in verts)
        s = slots[i]
        m = cat.by_name[data["meshes"][s[0]].rsplit(".", 1)[-1]][1]
        r_ = math.radians(s[4])
        zs = []
        for p, q in ((0.65, 0.75), (0.65, -0.75), (-0.65, 0.75), (-0.65, -0.75)):
            x = tf.location.x + p * m["ext"][0] * math.cos(r_) - q * m["ext"][1] * math.sin(r_)
            y = tf.location.y + p * m["ext"][0] * math.sin(r_) + q * m["ext"][1] * math.cos(r_)
            for h in world.cast_ray(carla.Location(x=x, y=y, z=bottom + 5.0), carla.Location(x=x, y=y, z=bottom - 5.0)):
                if label(h) in GROUND:
                    zs.append(h.location.z)
                    break
        if zs:
            by_i[i]["gap_m"] = round(bottom - max(zs), 3)
            gaps.append(bottom - max(zs))
    client.apply_batch_sync([carla.command.DestroyActor(r.actor_id) for r in res if not r.error], False)
    tick()


def report(rows, labels, kinds, refused, gaps, out):
    n = len(rows)
    print(f"ray hit labels (inside footprints): {dict(labels.most_common(12))}")
    for k in kinds:
        print(f"on a {k} lane: {sum(r[k] for r in rows)} of {n} (footprint 10 cm in: {sum(r[k + '_in10'] for r in rows)})")
    print(f"raised ground under the car > 0.08 m (kerb or pavement): {sum(r['raised_m'] > 0.08 for r in rows)}, "
          f"> 0.15 m: {sum(r['raised_m'] > 0.15 for r in rows)}")
    ins = Counter(lab for r in rows for lab in r["inside"])
    print(f"cars with level geometry inside their box: {sum(1 for r in rows if r['inside'])} ({dict(ins)})")
    print(f"refused spawns: {dict(refused) or 'none'}")
    if gaps:
        g = sorted(gaps)
        print(f"bottom minus ground under the highest wheel: median {statistics.median(g):.3f} m, p5 "
              f"{g[int(0.05 * (len(g) - 1))]:.3f}, p95 {g[int(0.95 * (len(g) - 1))]:.3f}; floating > 0.1 m: "
              f"{sum(x > 0.1 for x in g)}, sunk < -0.1 m: {sum(x < -0.1 for x in g)} (of {len(g)})")
    by_src = Counter(r["src"] for r in rows)
    print(f"stalls by source: {dict(by_src)}")
    if out:
        Path(out).write_text(json.dumps(rows))


if __name__ == "__main__":
    main()
