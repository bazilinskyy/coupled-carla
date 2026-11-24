"""Stand a baked level's scenery on the level's own ground (a running CARLA server with the level is needed).

    python tools/snap_scenery.py experiments/eindhoven_twin.yaml [--port 2000]
    python tools/snap_scenery.py experiments/salamanca_twin.yaml --export <twin export>/ue   # no server


scenery.py places trees, bushes, lamps and furniture on a baked level (map.carla_level) at the terrain model's
heights (AHN in the Netherlands, base 0), which the level's ground follows only roughly: on the Eindhoven twin the
two agree within 0.3 m at 58% of the objects, but differ by more than 1 m at 12% (ramps, embankments, the station
platforms). This tool loads the level, casts a ray down at every such object and moves it so that its bottom
stands on the level's ground (snap_items): with one ground surface under it, that one, however far (the terrain
model of a non-NL twin is 4-20 m off and that changes within 100 m); with several (a bridge, a deck), the one
nearest the planned height plus the offset its unambiguous neighbours moved by, so an object under a bridge stays
under it. Flat things (wear, puddles, leaves, markings) are moved the same way when they float or sink, at their
kind's usual lift. An object that would reach into a building above its ground (a tree under an arcade), one with
nothing at all above or below it (past the level's edge) and a rooftop unit with no roof of the level under it are
left out. Road signs stand on the OpenDRIVE road height already and are left alone. It rewrites scenery.json and
records what it did there ("snapped"); run it again after each `mapbuild --scenery` of the map. With `--export` it
needs no server: it casts the rays at the twin export the level was baked from (carla-digitaltwins, manifest.json and
its meshes), which gives the same result as the server (Salamanca, 24 Nov: 3,307 on the ground and 398 off, both).

The parked cars (parked.json, parking.py) are snapped the same way: rays at the four wheels of each car from 3 m
above its planned bottom (a bridge or deck higher than that stays above it), the car's bottom set to the highest of
their first ground hits (the wheel that touches first), and its window glass moved with it. A car whose wheels' ground
differs by more than 0.3 m (on a kerb, a mound or a step of the level's ground), whose middle stands on ground more
than 0.15 m above its wheels, or whose box a building, wall, pole or raised ground cuts (rays along its diagonals at
0.3 and 1.0 m) is taken out of parked.json. Cars need horizontal rays: only with a server.
"""
import argparse
import json
import math
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, parking, scenery  # noqa: E402

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


LAYER_GAP = 2.0             # m: ground hits closer than this are one surface (a kerb, a ramp); further: decks, bridges
LOCAL = 30.0                # m: unambiguous neighbours this close give a multi-layer object its offset
FLAT = ("wear", "puddle", "leaves", "marking")      # flat things lying on the ground: moved only when > ON_GROUND off
SURFACE = ("surface",)      # road surfaces laid over the road (streetlook: red asphalt on Dutch cycle paths)
SURFACE_LIFT = 0.01         # m: they lie this far over the level's road (not in it, not over it)
SURFACE_ON = 0.01           # m: closer than this to that height they stay
ON_GROUND = 0.2             # m: an object this close to the ground under it counts as standing on it
UNDER = 0.5                 # m: a building surface more than this above an object's ground and below its top cuts it


def label(hit):
    return str(hit.label).split(".")[-1].lower()


def layers(zs, gap=LAYER_GAP):
    """Ground heights (any order) grouped into surfaces, lowest first: a gap of more than `gap` starts a new one."""
    zs = sorted(zs)
    out = [[zs[0]]] if zs else []
    for z in zs[1:]:
        (out[-1].append(z) if z - out[-1][-1] <= gap else out.append([z]))
    return out


def snap_items(data, cat, ray, stream=lambda cx, cy: None, max_shift=4.0):
    """Stand scenery.json's items on the level's ground (in place) and return the summary for data["snapped"].

    ray(x, y, z_from, z_to) -> [(z, label), ...]: every surface of the level on that vertical, top first.
    - trees, bushes, lamps, furniture (KINDS): onto the ground under them. With one ground surface under an object
      (no bridge, no deck) that is where it goes, however far (Salamanca 2026-11-24: the terrain model is 4-20 m
      off within 100 m, so 399 objects with nothing ambiguous under them were left floating or buried). With several,
      the one nearest the planned height plus the offset of the unambiguous objects within LOCAL m (else of its AREA
      m square); left alone when that is more than max_shift away. One whose top would reach into a building
      surface above its ground (a tree under an arcade or a passage) is left out.
    - flat things (FLAT: wear, puddles, leaves, markings) the same, moved only when more than ON_GROUND off, and
      kept at their kind's usual lift over the ground.
    - road surfaces (SURFACE: red asphalt strips) onto the level's road, SURFACE_LIFT over it: they come from the
      OpenDRIVE's lane heights (else the terrain model's), and a twin's baked road lies elsewhere (TUeCampus
      2026-11-24: 413 of 608 strips more than 0.1 m above or in the road, the map's OpenDRIVE being the level's).
    - rooftop units (ROOF) onto the highest building surface under them, or left out; everything with no level
      above or below it (past the level's edge) is left out; the rest (signs on the OpenDRIVE heights) stays."""
    cells = defaultdict(list)
    for i, it in enumerate(data["items"]):
        cells[(int(it[1] // CELL), int(it[2] // CELL))].append(i)
    missed, skipped, far = 0, 0, 0
    roof_shifts, roof_drop, void, under = [], set(), set(), set()
    cand = {}       # item -> (planned bottom, ground hits, building hits, height)
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
            moved = (it[6] in KINDS and len(it) <= 7) or it[6] in FLAT or it[6] in SURFACE
            if not moved and it[6] != ROOF:
                # signs (on the OpenDRIVE's heights) and the rest stay where they are, unless the level ends
                # before them (Salamanca 2026-11-24: signs on roads of map.xodr past the level floated in the void)
                if not ray(it[1], it[2], planned + REACH, planned - REACH):
                    void.add(i)
                continue
            if it[6] == ROOF:
                # the scenery's roofs come from OSM/Overture heights; a twin's level has its own buildings (3DBAG,
                # PLATEAU, Catastro): the unit goes on the highest building surface under it, or out where the
                # level has no roof there (Shibuya 2026-11-24: units floating 5-30 m over the PLATEAU roofs)
                tops = [z for z, lb in ray(it[1], it[2], planned + 80.0, planned - 80.0) if lb in BUILDING]
                if not tops or abs(max(tops) - planned) > ROOF_MAX_SHIFT:
                    roof_drop.add(i)
                    continue
                d = max(tops) - planned
                it[3] = round(it[3] + d, 2)
                roof_shifts.append(d)
                continue
            allhits = ray(it[1], it[2], planned + REACH, planned - REACH)
            if not allhits:         # nothing at all above or below: past the level's edge, it would float in the void
                void.add(i)
                continue
            hits = [z for z, lb in allhits if lb in GROUND]
            if not hits:
                missed += 1
                continue
            cand[i] = (planned, hits, [z for z, lb in allhits if lb in BUILDING],
                       2.0 * m["ext"][2] * it[5] - sink)
    # the planned heights may be off by a whole area (a twin whose level stands on another terrain than the map's):
    # the objects with one ground surface under them say by how much, around each object and per AREA square
    # (flat things come from the OpenDRIVE's heights, the rest from the terrain model's: each has its own offsets)
    near = {i: min(h, key=lambda z, p=p: abs(z - p)) - p for i, (p, h, _b, _t) in cand.items()}
    for i, (p, h, _b, _t) in cand.items():      # flat things and road surfaces: from the visible top of that layer
        if data["items"][i][6] in FLAT + SURFACE:
            g = p + near[i]
            near[i] = max(next(lay for lay in layers(h) if g in lay)) - p
    sure = {i for i, (_p, h, _b, _t) in cand.items() if len(layers(h)) == 1}
    area, local = defaultdict(list), defaultdict(list)
    for i, d in near.items():
        it = data["items"][i]
        grp = it[6] in FLAT + SURFACE
        area[(grp, int(it[1] // AREA), int(it[2] // AREA))].append(d)
        if i in sure:
            local[(grp, int(it[1] // LOCAL), int(it[2] // LOCAL))].append((it[1], it[2], d))
    offs = {k: statistics.median(v) for k, v in area.items()}

    def offset(it):
        grp, kx, ky = it[6] in FLAT + SURFACE, int(it[1] // LOCAL), int(it[2] // LOCAL)
        ds = [d for dx in (-1, 0, 1) for dy in (-1, 0, 1) for x, y, d in local.get((grp, kx + dx, ky + dy), ())
              if (x - it[1]) ** 2 + (y - it[2]) ** 2 <= LOCAL ** 2]
        return statistics.median(ds) if ds else offs[(grp, int(it[1] // AREA), int(it[2] // AREA))]
    lift = {}           # a flat kind's usual height over the ground (leaves lie a little above it)
    for k in FLAT:
        v = [near[i] for i in cand if data["items"][i][6] == k and abs(near[i]) <= ON_GROUND]
        lift[k] = statistics.median(v) if v else 0.0
    lift.update({k: -SURFACE_LIFT for k in SURFACE})     # (ground - object, as near: over it is negative)
    shifts, flat, surf = [], 0, 0
    for i, (planned, hits, bldg, height) in cand.items():
        it = data["items"][i]
        flat_kind, surf_kind = it[6] in FLAT, it[6] in SURFACE
        if flat_kind and abs(near[i]) <= ON_GROUND:
            continue                            # lies on the ground already
        if surf_kind and abs(near[i] + SURFACE_LIFT) <= SURFACE_ON:
            continue                            # lies on the road already
        off = offset(it)
        g = min(hits, key=lambda z: abs(z - planned - off))
        if abs(g - planned - off) > max_shift:
            if i not in sure:                   # several surfaces and none where its neighbours put it: unsure
                skipped += 1
                continue
            g = max(hits)                       # one surface: its top, what the object stands on
            far += 1
        if flat_kind or surf_kind:
            # the visible top of that surface: a road and the ground plane 0.35 m under it are one layer, and a
            # strip or a puddle on the hidden plane would be under the road
            g = max(next(lay for lay in layers(hits) if g in lay))
        if not (flat_kind or surf_kind) and any(g + UNDER < z < g + height for z in bldg):
            under.add(i)                        # would reach into the building above it
            continue
        d = g - planned - (lift[it[6]] if flat_kind or surf_kind else 0.0)
        it[3] = round(it[3] + d, 3 if surf_kind else 2)
        shifts.append(d)
        flat += flat_kind
        surf += surf_kind
    if roof_drop or void or under:
        drop_items(data, roof_drop | void | under)
    a = sorted(abs(d) for d in shifts)
    ra = sorted(abs(d) for d in roof_shifts)
    return {"roof_units": len(roof_shifts), "roof_units_left_out": len(roof_drop),
            "roof_p95_abs_shift_m": round(ra[int(0.95 * (len(ra) - 1))], 2) if ra else 0,
            "items": len(shifts), "flat_items_moved": flat, "surfaces_moved": surf, "far_on_the_only_ground": far, "no_ground": missed,
            "left_alone": skipped, "past_the_level_left_out": len(void), "under_a_building_left_out": len(under),
            "median_shift_m": round(statistics.median(shifts), 2) if shifts else 0,
            "p95_abs_shift_m": round(a[int(0.95 * (len(a) - 1))], 2) if a else 0,
            "when": time.strftime("%Y-%m-%d %H:%M")}


EXPORT_LABEL = {"ground": "ground", "groundplane": "ground", "verge": "terrain", "boundary": "static",
                "curb": "sidewalks", "riser": "sidewalks", "building_3dbag": "buildings", "roof": "buildings",
                "landmark": "buildings"}
SEMANTIC_LABEL = {"road": "roads", "sidewalk": "sidewalks", "terrain": "terrain", "building": "buildings",
                  "roadline": "roadlines", "static": "static", "vegetation": "vegetation"}


def _glb_triangles(path):
    """Triangles (n, 3, 3) of a GLB in CARLA's frame: glTF (x, y, z) m -> (x, z, y) (glTF z = -north = CARLA y)."""
    import struct
    import numpy as np
    b = Path(path).read_bytes()
    jl = struct.unpack_from("<I", b, 12)[0]
    g = json.loads(b[20:20 + jl])
    blob = b[28 + jl:]
    def acc(k):
        a = g["accessors"][k]
        v = g["bufferViews"][a["bufferView"]]
        dt = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}[a["componentType"]]
        w = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
        o = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        return np.frombuffer(blob, dt, a["count"] * w, o).reshape(a["count"], w) if w > 1 else \
            np.frombuffer(blob, dt, a["count"], o)
    out = []
    for mesh in g.get("meshes", []):
        for pr in mesh["primitives"]:
            p = acc(pr["attributes"]["POSITION"]).astype(float)[:, [0, 2, 1]]
            f = acc(pr["indices"]).astype(np.int64).reshape(-1, 3) if "indices" in pr else \
                np.arange(len(p)).reshape(-1, 3)
            out.append(p[f])
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


class ExportGround:
    """The level's surfaces from the twin export it was baked from (manifest.json + meshes/*.glb, carla-digitaltwins
    TwinModel): vertical rays without a server. Labels follow the export's kinds and semantics as the baked level's
    ray hits name them (its ground planes answer as ground)."""
    BIN = 4.0

    def __init__(self, ue):
        import numpy as np
        ue = Path(ue)
        man = json.loads((ue / "manifest.json").read_text())
        kinds = man.get("kinds", {})
        tris, labs = [], []
        for a in man["assets"]:
            t = _glb_triangles(ue / a["file"])
            if not len(t):
                continue
            sem = (a.get("semantic") or kinds.get(a["kind"], {}).get("semantic") or "static").lower()
            tris.append(t)
            labs += [EXPORT_LABEL.get(a["kind"], SEMANTIC_LABEL.get(sem, sem))] * len(t)
        self.t, self.lab = np.concatenate(tris), labs
        lo = np.floor(self.t[:, :, :2].min(1) / self.BIN).astype(int)
        hi = np.floor(self.t[:, :, :2].max(1) / self.BIN).astype(int)
        self.bins = defaultdict(list)
        for i in range(len(self.t)):
            for bx in range(lo[i, 0], hi[i, 0] + 1):
                for by in range(lo[i, 1], hi[i, 1] + 1):
                    self.bins[(bx, by)].append(i)

    def ray(self, x, y, z0, z1):
        out = []
        for i in self.bins.get((math.floor(x / self.BIN), math.floor(y / self.BIN)), ()):
            (ax, ay, az), (bx, by, bz), (cx, cy, cz) = self.t[i]
            v0x, v0y, v1x, v1y = bx - ax, by - ay, cx - ax, cy - ay
            det = v0x * v1y - v0y * v1x
            if abs(det) < 1e-9:
                continue
            px, py = x - ax, y - ay
            u, w = (px * v1y - py * v1x) / det, (v0x * py - v0y * px) / det
            if u >= -1e-6 and w >= -1e-6 and u + w <= 1 + 1e-6:
                z = az + u * (bz - az) + w * (cz - az)
                if min(z0, z1) <= z <= max(z0, z1):
                    out.append((round(float(z), 3), self.lab[i]))
        return sorted(set(out), reverse=True)     # a ray along an edge meets both its triangles: once


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--max-shift", type=float, default=4.0, help="leave objects that would move more (m)")
    ap.add_argument("--parked-only", action="store_true", help="snap only parked.json (scenery.json as it is)")
    ap.add_argument("--export", type=Path, default=None,
                    help="no server: ray-cast the twin export the level was baked from (its folder with "
                         "manifest.json); parked.json is left as it is")
    ap.add_argument("--map-dir", type=Path, default=None, help="the map folder (default: the experiment's)")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    if not cfg["map"].get("carla_level"):
        sys.exit("only for a baked level (map.carla_level)")
    mdir = args.map_dir or config.map_dir(cfg)
    f = mdir / "scenery.json"
    data = json.loads(f.read_text())
    cat = scenery.Catalog(data["catalog"])
    t0 = time.time()
    if args.export:
        ground = ExportGround(args.export)
        world, stream, ray = None, (lambda cx, cy: None), ground.ray
    else:
        import carla
        from coupledcarla import run
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
                if any(label(h) in GROUND for h in world.cast_ray(
                        carla.Location(x=x0, y=y0, z=400.0), carla.Location(x=x0, y=y0, z=-200.0))):
                    break
                time.sleep(0.1)

        def ray(x, y, z0, z1):
            return [(h.location.z, label(h)) for h in world.cast_ray(carla.Location(x=x, y=y, z=z0),
                                                                      carla.Location(x=x, y=y, z=z1))]
    if not args.parked_only:
        res = snap_items(data, cat, ray, stream, args.max_shift)
        data["snapped"] = {"level": cfg["map"]["carla_level"], **res,
                           "from": (f"export {args.export.resolve().parent.name}/{args.export.name}" if args.export
                                    else "server")}
        f.write_text(json.dumps(data, separators=(",", ":")))
        print(f"snapped {res['items']} objects to the level's ground in {time.time() - t0:.0f} s "
              f"({res['no_ground']} without ground under them, {res['left_alone']} would move > {args.max_shift} m): "
              f"{data['snapped']}")
    pf = mdir / "parked.json"
    if pf.exists() and world is not None:
        snap_parked(pf, world, stream, args.max_shift, cfg["map"]["carla_level"])
    elif pf.exists():
        print("parked.json left as it is (its cars need horizontal rays: a server)")


def snap_parked(pf, world, stream, max_shift, level):
    """Stand parked.json's cars on the level's ground: the bottom of each car's box at the highest ground hit under
    its four wheels (near the planned height); a car's glass slot (the one after it, flagged) moves with it."""
    import carla
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
