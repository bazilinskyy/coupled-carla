"""Check a map's scenery geometry on a running CARLA server, without rendering (works on a -nullrhi server).

    python tools/scenery_check.py experiments/eindhoven_centre.yaml [--port 2010] [--radius 300] [--no-load]

Loads the experiment's world (as the run loop does), spawns the scenery placements within --radius of the first
participant's spawn (or --lat/--lon), and for every mesh compares the bottom of its bounding box with the ground
under it (rays cast down through it, ignoring vegetation, roofs and the like; the spectator is moved there first,
as a baked World Partition level streams its ground in only around it). Prints, per kind, how many spawned, how
many the server refused (a mesh path it does not have), and how far bottoms are from the ground: a floating tree
shows as a large positive gap, a buried one as a large negative gap.
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
from coupledcarla import config, run  # noqa: E402
from coupledcarla.geo import Geo  # noqa: E402

GROUND = {"roads", "sidewalks", "terrain", "ground", "water", "roadlines", "railtrack", "bridge", "none",
          "unlabeled", "other"}


def ground_below(world, x, y, z_top, z_bottom):
    """(z, label) of the highest ground-like hit of a vertical ray, or (None, None)."""
    hits = world.cast_ray(carla.Location(x=x, y=y, z=z_top), carla.Location(x=x, y=y, z=z_bottom))
    for h in hits:      # ordered from the start of the ray
        label = str(h.label).split(".")[-1].lower()
        if label in GROUND:
            return h.location.z, label
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--radius", type=float, default=300.0)
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--no-load", action="store_true", help="use the world the server has loaded")
    ap.add_argument("--kinds", default="tree,bush,lamp,furniture,sign,building,landmark,ground,water")
    ap.add_argument("--out", help="write the per-mesh rows here (json)")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    mdir = config.map_dir(cfg)
    geo = Geo(mdir / "geo.net.xml")
    data = json.loads((mdir / "scenery.json").read_text())
    client = carla.Client(cfg["server"]["host"], args.port or cfg["server"]["port"])
    client.set_timeout(60.0)
    print(f"server {client.get_server_version()}, scenery catalogue {data.get('catalog')}, "
          f"{len(data['items'])} placements")
    world = client.get_world() if args.no_load else run.load_world(client, mdir / "map.xodr", cfg["map"], slab=True)
    if args.lat is not None:
        cx, cy = geo.to_carla(args.lat, args.lon)
    else:
        sp = cfg["participants"][0]["spawn"]
        cx, cy = geo.to_carla(sp["lat"], sp["lon"])
    # a baked level (World Partition) streams its ground and roads in only around the spectator
    world.get_spectator().set_transform(carla.Transform(carla.Location(x=cx, y=cy, z=80.0), carla.Rotation(pitch=-90)))
    for _ in range(20):
        world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
    time.sleep(8.0)
    kinds = set(args.kinds.split(","))
    items = [it for it in data["items"] if it[6] in kinds
             and math.hypot(*(a - b for a, b in zip(it[8] if len(it) > 8 else it[1:3], (cx, cy)))) <= args.radius]
    bp = world.get_blueprint_library().find("static.prop.mesh")
    cmds = []
    for it in items:
        bp.set_attribute("mesh_path", data["meshes"][it[0]])
        bp.set_attribute("scale", str(it[5]))
        pitch, roll = (it[7] if len(it) > 7 else (0.0, 0.0))
        cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(x=it[1], y=it[2], z=it[3]),
                                                                 carla.Rotation(pitch=pitch, yaw=it[4], roll=roll))))
    res = []
    for i in range(0, len(cmds), 500):
        res += client.apply_batch_sync(cmds[i:i + 500], False)
    world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
    failed = Counter(data["meshes"][it[0]] for it, r in zip(items, res) if r.error)
    print(f"{len(items)} placements within {args.radius:.0f} m of ({cx:.0f}, {cy:.0f}): "
          f"{sum(1 for r in res if not r.error)} spawned, {sum(failed.values())} refused "
          f"({len(failed)} meshes{': ' + ', '.join(p.rsplit('.', 1)[-1] for p in failed) if failed else ''})")
    gaps, errs, rows, labels = defaultdict(list), defaultdict(list), [], Counter()
    from coupledcarla import scenery
    scen = scenery.Catalog(data["catalog"])
    cat = {p: m for p, m in json.loads((scenery.DATA / f"meshes_{data['catalog']}.json").read_text())["meshes"].items()}
    ids = []
    for it, r in zip(items, res):
        if r.error:
            continue
        ids.append(r.actor_id)
        if it[6] in ("ground", "water"):
            continue
        a = world.get_actor(r.actor_id)
        tf = a.get_transform()
        bb = a.bounding_box
        # the server scales bounding_box.extent with the actor's scale but not bounding_box.location (measured on
        # CARLA 1.0), so the box is rebuilt here from the scaled centre
        c = bb.location
        verts = [tf.transform(carla.Location(x=c.x * it[5] + i * bb.extent.x, y=c.y * it[5] + j * bb.extent.y,
                                             z=c.z * it[5] + k * bb.extent.z))
                 for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
        bottom = min(v.z for v in verts)
        # where scenery.json meant the bottom to be: the catalogue's pivot offset and extent at the item's scale
        # (yaw-only items); a difference means the server places or scales the mesh differently
        m = cat.get(data["meshes"][it[0]])
        planned = it[3] + it[5] * (m["off"][2] - m["ext"][2]) if m and len(it) <= 7 else None
        if planned is not None:
            errs[it[6]].append(bottom - planned)
        # the ground under the mesh: vertical rays through it, the first ground-like hit (trunks, roofs skipped)
        zs = []
        for dx, dy in ((0, 0), (0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5)):
            z, lab = ground_below(world, tf.location.x + dx, tf.location.y + dy, bottom + 60.0, bottom - 40.0)
            if z is not None:
                zs.append(z)
                labels[lab] += 1
        if not zs:
            gaps[it[6] + "_noground"].append(0.0)
            continue
        g = statistics.median(zs)
        kind = it[6]
        if kind == "sign" and data["meshes"][it[0]].rsplit(".", 1)[-1] != scen.m.POST:
            kind = "sign_plate"         # the plate on top of its post, high by design
        gaps[kind].append(bottom - g)
        rows.append({"kind": it[6], "mesh": data["meshes"][it[0]].rsplit(".", 1)[-1], "x": round(tf.location.x, 1),
                     "y": round(tf.location.y, 1), "bottom": round(bottom, 2), "ground": round(g, 2),
                     "gap": round(bottom - g, 2), "planned": None if planned is None else round(planned, 2)})
    print(f"ground hits by label: {dict(labels)}")
    print("bottom of each mesh's bounding box minus where scenery.json placed it (catalogue offsets at scale):")
    for k, v in sorted(errs.items()):
        v = sorted(v)
        print(f"  {k:12s} {len(v):5d} median {statistics.median(v):6.2f}  min {v[0]:6.2f}  max {v[-1]:6.2f}")
    print("bottom of each mesh's bounding box minus the ground under it (ground tiles, roads or the level):")
    print(f"{'kind':12s} {'n':>5s} {'median':>7s} {'p5':>7s} {'p95':>7s} {'>0.3m up':>9s} {'<-1m down':>9s}")
    for k, v in sorted(gaps.items()):
        v = sorted(v)
        p = (lambda q: v[min(len(v) - 1, int(q * len(v)))])
        print(f"{k:12s} {len(v):5d} {statistics.median(v):7.2f} {p(0.05):7.2f} {p(0.95):7.2f} "
              f"{sum(1 for g in v if g > 0.3):9d} {sum(1 for g in v if g < -1.0):9d}")
    if args.out:
        Path(args.out).write_text(json.dumps(rows))
    client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)


if __name__ == "__main__":
    main()
