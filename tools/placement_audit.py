"""Audit where coupled-carla puts things, on a running CARLA server: floating, sunk, tilted, duplicated, on the road,
inside buildings, facing the wrong way.

    python tools/placement_audit.py experiments/eindhoven_centre.yaml --port 2700 [--radius 300] [--out DIR]
        [--frames 12] [--carla-lights] [--scenery FILE] [--parked FILE] [--live] [--lat .. --lon ..]

Loads the experiment's world as the run loop does (run.load_world: CARLA's own traffic lights left out unless
--carla-lights) and spawns, within --radius of the first participant (or --lat/--lon):
- every scenery.json placement (buildings, trees, bushes, lamps, furniture and bus shelters, signs, shared bikes,
  ground and water tiles; --scenery for another file, e.g. one built before a fix),
- every parked car stall of parked.json (all stalls, not only those taken at an hour; --parked for another file),
- the traffic-light heads signals.py builds from SUMO's programs (SUMO is started on map.net.xml, no traffic),
plus the actors CARLA itself makes from the OpenDRIVE (traffic lights, stop and yield signs). --live instead audits
the actors of a running trial (vehicles, walkers, landed drones, everything spawned), without loading anything.

For every object it compares the bottom of its bounding box with the ground under it (vertical rays through its
footprint; hits on other objects are skipped): floating over 0.15 m, sunk more than 0.3 m (buildings: a corner in
the air, or buried by over 1.5 m); tilted over 8 degrees (upright things); a duplicate of the same kind closer than
placement.DUPLICATE_M; standing on a driving lane or in a junction's paved area ("on_drivable"); inside a building's
box; parked cars in each other or in another object; sign plates facing away from their traffic ("backwards"),
parked cars at the kerb against the traffic. It also casts rays down onto the driving lanes (every 2 m, three across
each lane, up to 2.5 m above the road) and reports what stands on them ("blocks_road": a sign in a junction, a
ground tile above the road). Writes report.json and report.txt (counts per kind and problem, the worst examples) to
--out, with camera frames of the worst examples (a red box around each) unless --frames 0 or the server renders
nothing (-nullrhi).
"""
import argparse
import json
import math
import queue
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, placement, run, scenery  # noqa: E402
from coupledcarla.geo import Geo  # noqa: E402

# what an object may stand on (hits on other objects are skipped by their labels; parts of lamps are labelled None).
# Some ground tiles have no collision (SM_Square_61, the concrete), so rays miss them: the tiles' top faces are
# taken from their actors as well (TileTops)
GROUND = {"roads", "sidewalks", "terrain", "ground", "roadlines", "water", "railtrack", "bridge"}
ROAD = {"roads", "roadlines"}
STANDING = {"tree", "bush", "lamp", "furniture", "shelter", "sign_post", "parked", "shared", "signal_pole",
            "building", "landmark", "vehicle", "walker", "carla_sign"}
UPRIGHT = {"tree", "lamp", "furniture", "shelter", "sign_post", "parked", "shared", "signal_pole", "signal",
           "sign_plate", "carla_light", "carla_sign"}
POLES = {"lamp", "sign_post", "signal_pole", "tree"}    # stand at their pivot; a small base for overlaps
ON_ROAD_KINDS = {"tree", "bush", "lamp", "furniture", "shelter", "sign_post", "parked", "shared", "signal_pole",
                 "building", "landmark"}
FRAME_PROBLEMS = ["blocks_road", "on_drivable", "carla_generated", "floating", "sunk", "duplicate", "in_building",
                  "backwards", "against_traffic", "overlap", "tilted"]


UNATTRIBUTED = []        # where the road scan hit something it could not tell (label, x, y, height, road, junction)
OWN_TICK = [False]       # this tool runs the world in synchronous mode (not in --live)


def step(world):
    world.tick() if OWN_TICK[0] else world.wait_for_tick()


def label(h):
    return str(h.label).split(".")[-1].lower()


def ground_at(world, x, y, z_top, z_bottom, none_below=None):
    """(z, label) of the first ground-like hit of a vertical ray from z_top down to z_bottom, or (None, None).
    CARLA labels its cycle lanes' and junctions' surfaces None, as it does parts of some lamps: a None hit counts
    as ground below none_below (an object's bottom + 1.5 m) only."""
    for h in world.cast_ray(carla.Location(x=x, y=y, z=z_top), carla.Location(x=x, y=y, z=z_bottom)):
        lab = label(h)
        if lab in GROUND or (lab == "none" and none_below is not None and h.location.z < none_below):
            return h.location.z, lab
    return None, None


def box_of(actor, scale=1.0, cat=None):
    """World corners of an actor's bounding box (8 carla.Location): from the mesh catalogue's box (cat: {"off",
    "ext"} at scale 1) when given, else the server's. The server scales extent with the actor's scale but not
    bounding_box.location (scenery_check.py), so the centre is scaled here. On 0.9.16 a static mesh's
    bounding_box is not its own box turned with it: a parked car at yaw 85 came out with its length across the
    street (every car in a row "overlapping" its neighbours), a tilted ground tile 0.6 m too high, so the
    catalogue's box is used wherever there is one."""
    tf = actor.get_transform()
    if cat is not None:
        (cx, cy, cz), (ex, ey, ez) = [v * scale for v in cat["off"]], [v * scale for v in cat["ext"]]
    else:
        bb = actor.bounding_box
        cx, cy, cz = bb.location.x * scale, bb.location.y * scale, bb.location.z * scale
        ex, ey, ez = bb.extent.x, bb.extent.y, bb.extent.z
    return [tf.transform(carla.Location(x=cx + i * ex, y=cy + j * ey, z=cz + k * ez))
            for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]


def record(kind, actor, mesh=None, scale=1.0, extra=None, verts=None, cat=None):
    tf = actor.get_transform() if actor is not None else None
    verts = verts or box_of(actor, scale, cat)
    bottom, top = min(v.z for v in verts), max(v.z for v in verts)
    xs, ys = [v.x for v in verts], [v.y for v in verts]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    # the footprint: the four bottom corners (or the box's xy hull for a tilted one) in order around the centre
    pts = sorted({(round(v.x, 3), round(v.y, 3)) for v in verts}, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    if len(pts) > 4:
        from shapely.geometry import MultiPoint
        pts = list(MultiPoint(pts).convex_hull.exterior.coords)[:-1]
    r = {"kind": kind, "id": actor.id if actor is not None else None, "mesh": mesh,
         "x": round(cx, 2), "y": round(cy, 2), "z": round(bottom, 2), "bottom": round(bottom, 2),
         "top": round(top, 2), "yaw": round(tf.rotation.yaw, 1) if tf else None,
         "tilt": round(placement.tilt_of(tf.rotation.pitch, tf.rotation.roll), 1) if tf else 0.0,
         "foot": pts, "problems": [], "detail": {},
         "pivot": [round(tf.location.x, 2), round(tf.location.y, 2)] if tf else None}
    if kind in POLES and tf is not None:     # the overlap check uses the pole's base, not an arm over the road
        px, py = tf.location.x, tf.location.y
        r["base_foot"] = [(px - 0.25, py - 0.25), (px + 0.25, py - 0.25), (px + 0.25, py + 0.25), (px - 0.25, py + 0.25)]
    r.update(extra or {})
    return r


def spawn_items(client, world, meshes, items, catalog_kind):
    """Spawn scenery-like items ([mesh index, x, y, z, yaw, scale, kind, (pitch, roll)]); [(item, actor id)]."""
    bp = world.get_blueprint_library().find("static.prop.mesh")
    cmds = []
    for it in items:
        bp.set_attribute("mesh_path", meshes[it[0]])
        bp.set_attribute("scale", str(it[5]))
        pitch, roll = (it[7] if len(it) > 7 and isinstance(it[7], list) else (0.0, 0.0))
        cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(x=it[1], y=it[2], z=it[3]),
                                                                 carla.Rotation(pitch=pitch, yaw=it[4], roll=roll))))
    res = []
    for i in range(0, len(cmds), 500):
        res += client.apply_batch_sync(cmds[i:i + 500], False)
    step(world)        # (synchronous mode) the client knows the new actors after a tick
    refused = Counter(meshes[it[0]].rsplit(".", 1)[-1] for it, r in zip(items, res) if r.error)
    if refused:
        print(f"  {catalog_kind}: {sum(refused.values())} refused ({', '.join(f'{m} {n}' for m, n in refused.most_common(6))})")
    return [(it, r.actor_id) for it, r in zip(items, res) if not r.error]


def start_signals(world, cfg, mdir, style, scenery_file=None):
    """signals.Signals for the map (SUMO started on map.net.xml with no traffic), or None without SUMO."""
    try:
        import sumolib
        import traci
        from coupledcarla.signals import Signals
    except ImportError as err:
        print(f"  signals: not audited ({err})")
        return None
    net_file = mdir / "map.net.xml"
    traci.start(["sumo", "-n", str(net_file), "--no-step-log", "true", "--no-warnings", "true"])
    net = sumolib.net.readNet(str(net_file))
    geo = sumolib.net.readNet(str(mdir / "geo.net.xml"), withInternal=False)
    mo, go = net.getLocationOffset(), geo.getLocationOffset()
    import inspect      # (an older Signals, to audit the code before a fix, knows no ground surface)
    kw = {}
    if "surface" in inspect.signature(Signals.__init__).parameters and hasattr(placement, "ground_surface"):
        kw["surface"] = placement.ground_surface(scenery_file or mdir / "scenery.json")
    return Signals(world, net, (mo[0] - go[0], mo[1] - go[1]), style=style,
                   lefthand=bool(cfg["map"].get("lefthand")), log=lambda m: print("  " + m), **kw)


def nearest_lane(cmap, x, y, z=0.0, reach=12.0):
    wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=z), project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return None
    loc = wp.transform.location
    return wp if math.hypot(loc.x - x, loc.y - y) <= reach else None


class TileTops:
    """The top faces of the spawned ground tiles: z at a point, from each tile actor's transform and box."""

    def __init__(self, tiles):
        from shapely.geometry import MultiPoint
        from shapely.strtree import STRtree
        self.faces, polys = [], []
        for actor, scale, cat in tiles:
            tf = actor.get_transform()
            if cat is not None:
                (cx, cy, cz), ez = [v * scale for v in cat["off"]], cat["ext"][2] * scale
            else:
                bb = actor.bounding_box
                cx, cy, cz, ez = bb.location.x * scale, bb.location.y * scale, bb.location.z * scale, bb.extent.z
            top = tf.transform(carla.Location(x=cx, y=cy, z=cz + ez))
            n = tf.get_up_vector()
            verts = box_of(actor, scale, cat)
            polys.append(MultiPoint([(v.x, v.y) for v in verts]).convex_hull)
            self.faces.append((top.x, top.y, top.z, n.x, n.y, n.z))
        self.polys = polys
        self.tree = STRtree(polys) if polys else None

    def z(self, x, y):
        if self.tree is None:
            return None
        from shapely.geometry import Point
        p, best = Point(x, y), None
        for i in self.tree.query(p):
            i = int(i)
            if not self.polys[i].contains(p):
                continue
            x0, y0, z0, nx, ny, nz = self.faces[i]
            if nz < 0.5:
                continue
            v = z0 - (nx * (x - x0) + ny * (y - y0)) / nz
            best = v if best is None else max(best, v)
        return best


def check_ground(world, recs, tops=None):
    for r in recs:
        if r["kind"] not in STANDING:
            continue
        cx, cy = r["x"], r["y"]
        if r["kind"] in POLES and r.get("pivot"):      # a pole stands at its pivot (a lamp's box is mostly its arm)
            cx, cy = r["pivot"]
            pts = [(cx, cy)] + [(cx + dx, cy + dy) for dx, dy in ((0.2, 0), (-0.2, 0), (0, 0.2), (0, -0.2))]
        else:
            pts = [(cx, cy)] + [(cx + 0.6 * (px - cx), cy + 0.6 * (py - cy)) for px, py in r["foot"][:4]]
        zs = []
        for px, py in pts:
            z, lab = ground_at(world, px, py, r["top"] + 1.0, r["bottom"] - 30.0, r["bottom"] + 1.5)
            t = tops.z(px, py) if tops is not None else None
            if t is not None and t <= r["top"] + 1.0 and (z is None or t > z):
                z, lab = t, "tile"
            if z is not None:
                zs.append(z)
                r["detail"].setdefault("ground_labels", []).append(lab)
        if not zs:
            r["problems"].append("no_ground")
            continue
        if r["kind"] in ("building", "landmark"):
            # a corner in the air is floating (a building on a slope); buried by more than 1.5 m is sunk
            r["ground"] = round(min(zs), 2)
            r["gap"] = round(r["bottom"] - min(zs), 2)
            if r["gap"] > placement.FLOAT_M:
                r["problems"].append("floating")
            elif r["bottom"] - max(zs) < -1.5 - 0.3:
                r["problems"].append("sunk")
                r["gap"] = round(r["bottom"] - max(zs), 2)
            continue
        g = statistics.median(zs) if r["kind"] not in ("parked", "vehicle", "shelter") else max(zs)
        r["ground"], r["gap"] = round(g, 2), round(r["bottom"] - g, 2)
        r["problems"] += placement.classify(r["gap"], 0.0, upright=False)


def check_tilt(recs):
    for r in recs:
        if r["kind"] in UPRIGHT and r["tilt"] > placement.TILT_DEG:
            r["problems"].append("tilted")
            r["detail"]["tilt"] = r["tilt"]


def check_drivable(cmap, junctions, recs):
    road = carla.LaneType.Driving | carla.LaneType.Bidirectional
    for r in recs:
        if r["kind"] not in ON_ROAD_KINDS:
            continue
        loc = carla.Location(x=r["x"], y=r["y"], z=r["bottom"] + 0.5)
        if r["kind"] in ("building", "landmark"):
            continue     # a building is checked by its rays (blocks_road); its centre is never on a lane
        lane = cmap.get_waypoint(loc, project_to_road=False, lane_type=road)
        j = junctions.which(r["x"], r["y"]) if junctions is not None else None
        if lane is not None or j is not None:
            r["problems"].append("on_drivable")
            r["detail"]["on_drivable"] = "junction" if j is not None else f"lane {lane.road_id}/{lane.lane_id}"


def check_in_building(recs):
    from shapely.geometry import Point, Polygon
    from shapely.strtree import STRtree
    blds = [Polygon(r["foot"]).buffer(-0.5) for r in recs if r["kind"] in ("building", "landmark") and len(r["foot"]) >= 3]
    blds = [b for b in blds if not b.is_empty]
    if not blds:
        return
    tree = STRtree(blds)
    for r in recs:
        if r["kind"] not in ("tree", "lamp", "sign_post", "furniture", "shelter", "parked", "signal_pole", "shared"):
            continue
        p = Point(r["x"], r["y"])
        if any(blds[int(i)].contains(p) for i in tree.query(p)):
            r["problems"].append("in_building")


def check_facing(cmap, recs, lefthand, oneway_ok=True):
    for r in recs:
        if r["kind"] == "sign_plate" and r.get("face") is not None:
            wp = nearest_lane(cmap, r["x"], r["y"], r["bottom"])
            if wp is None:
                continue
            wp = placement.approach(wp)
            err = placement.facing_error(r["face"], wp.transform.rotation.yaw)
            r["detail"]["facing_error"] = round(err, 1)
            if err > 120.0:
                r["problems"].append("backwards")
        elif r["kind"] == "parked" and r.get("kerb"):
            wp = nearest_lane(cmap, r["x"], r["y"], r["bottom"], reach=6.0)
            if wp is None or wp.is_junction:
                continue
            d = abs((r["yaw"] - wp.transform.rotation.yaw + 180.0) % 360.0 - 180.0)
            if d > 150.0:       # parallel and nose against the traffic of the lane beside it
                other = wp.get_left_lane() if not lefthand else wp.get_right_lane()
                two_way = other is not None and other.lane_id * wp.lane_id < 0 and \
                    other.lane_type == carla.LaneType.Driving
                if two_way or not oneway_ok:
                    r["problems"].append("against_traffic")
                    r["detail"]["facing_error"] = round(d, 1)


def kerb_drops(world, cmap, centre, radius, tops, step=4.0):
    """How far the ground falls beside the roads: at every outermost lane edge (each side, every `step` m), the
    road's height minus the ground 1.5 m beyond it (rays and the ground tiles' tops). Returns the drops (m)."""
    drops = []
    for wp in cmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        loc = wp.transform.location
        if math.hypot(loc.x - centre[0], loc.y - centre[1]) > radius:
            continue
        rv = wp.transform.get_right_vector()
        for side, nxt in ((1.0, wp.get_right_lane()), (-1.0, wp.get_left_lane())):
            if nxt is not None and nxt.lane_type in (carla.LaneType.Driving, carla.LaneType.Biking,
                                                     carla.LaneType.Parking, carla.LaneType.Bidirectional,
                                                     carla.LaneType.Sidewalk):
                continue
            if nxt is not None and nxt.lane_type == carla.LaneType.Driving:
                continue
            d = side * (wp.lane_width / 2 + 1.5)
            x, y = loc.x + rv.x * d, loc.y + rv.y * d
            if cmap.get_waypoint(carla.Location(x=x, y=y, z=loc.z), project_to_road=False,
                                 lane_type=carla.LaneType.Driving) is not None:
                continue        # another road right there
            z, _lab = ground_at(world, x, y, loc.z + 3.0, loc.z - 30.0)
            t = tops.z(x, y) if tops is not None else None
            if t is not None and t <= loc.z + 3.0 and (z is None or t > z):
                z = t
            if z is not None:
                drops.append(loc.z - z)
    return drops


def scan_roads(world, cmap, centre, radius, recs, step=2.0):
    """Rays down onto the driving lanes: what stands on them below 2.5 m. Marks the record a hit falls in
    ("blocks_road") and returns {label: count} of hits in no record (CARLA's own actors, the road mesh)."""
    from shapely.geometry import Point, Polygon
    from shapely.strtree import STRtree
    cand = [k for k, r in enumerate(recs) if r["kind"] not in ("sign_plate", "signal") and len(r["foot"]) >= 3]
    polys = [Polygon(recs[k]["foot"]).buffer(0.25) for k in cand]
    tree = STRtree(polys) if polys else None
    unattributed, n_rays, hits = Counter(), 0, 0
    marked = set()
    for wp in cmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving:
            continue
        loc = wp.transform.location
        if math.hypot(loc.x - centre[0], loc.y - centre[1]) > radius:
            continue
        rv = wp.transform.get_right_vector()
        for off in (-0.35, 0.0, 0.35):
            x, y = loc.x + rv.x * off * wp.lane_width, loc.y + rv.y * off * wp.lane_width
            n_rays += 1
            got = world.cast_ray(carla.Location(x=x, y=y, z=loc.z + 2.5), carla.Location(x=x, y=y, z=loc.z - 0.6))
            for h in got:
                lab = label(h)
                if lab in ROAD:
                    break
                if h.location.z < loc.z + (0.15 if lab == "none" else 0.05):
                    break       # (CARLA's junction surfaces, labelled None, lie up to 0.1 m over the lane centre)
                if lab in ("vegetation",):      # leaves over the road are fine, a trunk is not: attributed below
                    pass
                p = Point(h.location.x, h.location.y)
                owner = None
                if tree is not None:       # the object the hit is in; a ground tile only when nothing else is
                    owners = [cand[int(i)] for i in tree.query(p) if polys[int(i)].contains(p) and
                              recs[cand[int(i)]]["bottom"] - 0.3 <= h.location.z <= recs[cand[int(i)]]["top"] + 0.3
                              and not (lab == "none" and recs[cand[int(i)]]["kind"] in ("ground", "water"))]
                    # (a ground tile is Terrain or Sidewalks; None over a lane is CARLA's own junction surface)
                    owners.sort(key=lambda k: recs[k]["kind"] in ("ground", "water"))
                    owner = owners[0] if owners else None
                if owner is None:
                    if lab != "vegetation":
                        unattributed[lab] += 1
                        hits += 1
                        if len(UNATTRIBUTED) < 40:
                            UNATTRIBUTED.append([lab, round(h.location.x, 1), round(h.location.y, 1),
                                                 round(h.location.z - loc.z, 2), wp.road_id, wp.is_junction])
                    break
                r = recs[owner]
                if r["kind"] == "tree" and h.location.z > r["bottom"] + 2.0:
                    break       # its crown over the road
                hits += 1
                if owner not in marked:
                    marked.add(owner)
                    r["problems"].append("blocks_road")
                    r["detail"]["blocks_road"] = [lab, round(h.location.z - loc.z, 2)]
                break
    print(f"  road scan: {n_rays} rays on the driving lanes, {hits} hits on things, {len(marked)} objects block a lane,"
          f" unattributed {dict(unattributed)}")
    return dict(unattributed), n_rays


def carla_own(world):
    """Records for the actors CARLA made from the OpenDRIVE: traffic lights (their heads) and signs."""
    out = []
    for a in world.get_actors().filter("traffic.*"):
        if a.type_id.startswith("traffic.traffic_light"):
            boxes = a.get_light_boxes()
            verts = [v for b in boxes for v in b.get_world_vertices(carla.Transform())] or box_of(a)
            r = record("carla_light", a, a.type_id, verts=verts)
            r["detail"]["heads"] = len(boxes)
        elif a.type_id.startswith("traffic.") and a.type_id != "traffic.unknown":
            r = record("carla_sign", a, a.type_id)
        else:
            continue
        r["problems"].append("carla_generated")
        out.append(r)
    return out


def live_records(world):
    out = []
    for a in world.get_actors():
        t = a.type_id
        kind = ("vehicle" if t.startswith("vehicle.") else "walker" if t.startswith("walker.pedestrian") else
                "static" if t.startswith("static.") else None)
        if kind is None:
            continue
        out.append(record(kind, a, t))
    return out


def take_frames(world, recs, out, n, cmap):
    """Camera frames of the worst examples: from the street (the nearest lane, eye height) and from above, with a
    red box drawn around the object."""
    picks, seen = [], set()

    def pick(prob, r):
        if r["id"] not in seen and len(picks) < n:
            seen.add(r["id"])
            picks.append((prob, r))
    for prob in FRAME_PROBLEMS:     # the worst of each problem first
        for r in placement.worst(recs, prob, 1):
            pick(prob, r)
    for prob in ("floating", "sunk", "duplicate", "blocks_road", "overlap"):     # then a typical one per kind
        kinds = sorted({r["kind"] for r in recs if prob in r["problems"]})
        for kind in kinds:
            some = placement.worst([r for r in recs if r["kind"] == kind], prob, 10 ** 6)
            if some:
                pick(prob + "_typical", some[len(some) // 2])
    if not picks:
        return []
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "1280")
    bp.set_attribute("image_size_y", "720")
    bp.set_attribute("fov", "70")
    for k, v in (("motion_blur_intensity", "0"), ("bloom_intensity", "0.3")):
        if bp.has_attribute(k):
            bp.set_attribute(k, v)
    fdir = out / "frames"
    fdir.mkdir(parents=True, exist_ok=True)
    saved = []
    for i, (prob, r) in enumerate(picks):
        target = carla.Location(x=r["x"], y=r["y"], z=(r["bottom"] + min(r["top"], r["bottom"] + 6.0)) / 2)
        foot = r["foot"] or [(r["x"], r["y"])]
        world.debug.draw_box(carla.BoundingBox(target, carla.Vector3D(
            max(0.3, max(abs(px - r["x"]) for px, _ in foot)), max(0.3, max(abs(py - r["y"]) for _, py in foot)),
            max(0.3, (min(r["top"], r["bottom"] + 6.0) - r["bottom"]) / 2))), carla.Rotation(),
            thickness=0.06, color=carla.Color(255, 0, 0), life_time=6.0)
        wp = nearest_lane(cmap, r["x"], r["y"], r["bottom"], reach=40.0)
        views = []
        if wp is not None:
            base = wp.transform.location
            dx, dy = r["x"] - base.x, r["y"] - base.y
            d = math.hypot(dx, dy)
            if d < 9.0:     # step back along the lane so the object is in view, not under the camera
                f = wp.transform.get_forward_vector()
                base = carla.Location(x=r["x"] - f.x * 11.0, y=r["y"] - f.y * 11.0, z=base.z)
            views.append(("street", carla.Location(x=base.x, y=base.y, z=base.z + 1.7)))
        views.append(("above", carla.Location(x=r["x"] - 9.0, y=r["y"] - 9.0, z=r["bottom"] + 14.0)))
        for name, at in views:
            v = target - at
            yaw = math.degrees(math.atan2(v.y, v.x))
            pitch = math.degrees(math.atan2(v.z, math.hypot(v.x, v.y)))
            cam = world.spawn_actor(bp, carla.Transform(at, carla.Rotation(pitch=pitch, yaw=yaw)))
            q = queue.Queue()
            cam.listen(q.put)
            img = None
            try:
                for _ in range(12):     # let the view stream in and anti-aliasing settle
                    step(world)
                    img = q.get(timeout=10.0)
            except queue.Empty:
                pass
            cam.stop()
            cam.destroy()
            if img is None:
                print("  frames: the server renders nothing (-nullrhi?); no frames")
                return saved
            f = fdir / f"{i:02d}_{prob}_{r['kind']}_{name}.png"
            img.save_to_disk(str(f))
            saved.append({"file": f.name, "problem": prob, "kind": r["kind"], "mesh": r.get("mesh"),
                          "x": r["x"], "y": r["y"], "gap": r.get("gap"), "detail": r["detail"].get(prob)})
    return saved


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--radius", type=float, default=300.0)
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--out", default=None, help="report folder (default logs/placement_audit/<map>)")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--carla-lights", action="store_true", help="keep CARLA's own traffic lights (as before the fix)")
    ap.add_argument("--scenery", help="audit this scenery.json instead of the map's")
    ap.add_argument("--parked", help="audit this parked.json instead of the map's")
    ap.add_argument("--style", default="head", help="signal style to audit (head or marker)")
    ap.add_argument("--live", action="store_true", help="audit the actors of a running trial")
    ap.add_argument("--no-road-scan", action="store_true")
    ap.add_argument("--keep", action="store_true", help="leave the spawned objects in the world (to look around)")
    args = ap.parse_args()

    cfg = config.load(args.experiment)
    mdir = config.map_dir(cfg)
    geo = Geo(mdir / "geo.net.xml")
    out = Path(args.out or Path("logs") / "placement_audit" / cfg["map"]["name"])
    out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(cfg["server"]["host"], args.port or cfg["server"]["port"])
    client.set_timeout(120.0)
    t0 = time.time()
    if args.lat is not None:
        cx, cy = geo.to_carla(args.lat, args.lon)
    else:       # the first participant's spawn, else the first point of interest, else the map's middle
        sp = next((p["spawn"] for p in cfg.get("participants") or [] if p.get("spawn")), None) or \
            next(iter(cfg.get("pois") or []), None)
        bb = cfg["map"]["bbox"]
        cx, cy = geo.to_carla(sp["lat"], sp["lon"]) if sp else geo.to_carla((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
    if args.live:
        world = client.get_world()
    else:
        import inspect      # (an older run.load_world, to audit the code before a fix, keeps CARLA's lights)
        kw = {"keep_lights": args.carla_lights} if "keep_lights" in inspect.signature(run.load_world).parameters else {}
        world = run.load_world(client, mdir / "map.xodr", cfg["map"], slab=True, **kw)
    # synchronous mode: the server then answers ray casts at once instead of once per rendered frame (25x faster)
    # (not in --live: the trial's host ticks its world)
    settings = world.get_settings()
    old_settings = (settings.synchronous_mode, settings.fixed_delta_seconds)
    if not args.live:
        OWN_TICK[0] = True
        settings.synchronous_mode, settings.fixed_delta_seconds = True, 0.05
        world.apply_settings(settings)
    step(world)
    try:
        audit(args, cfg, mdir, client, world, out, cx, cy, t0)
    finally:
        if not args.live:
            try:
                step(world)
            except RuntimeError:
                pass
            settings = world.get_settings()
            settings.synchronous_mode, settings.fixed_delta_seconds = old_settings
            world.apply_settings(settings)


def audit(args, cfg, mdir, client, world, out, cx, cy, t0):
    cmap = world.get_map()
    print(f"server {client.get_server_version()}, map {cfg['map']['name']}, centre ({cx:.0f}, {cy:.0f}), "
          f"radius {args.radius:.0f} m")
    world.get_spectator().set_transform(carla.Transform(carla.Location(x=cx, y=cy, z=120.0), carla.Rotation(pitch=-90)))
    junctions = None if cfg["map"].get("carla_level") else placement.JunctionAreas.of_map(cmap, carla, log=print)
    recs, signals, spawned, tiles = [], None, [], []

    def near(it):
        x, y = it[8] if len(it) > 8 and isinstance(it[8], list) else it[1:3]
        # ground tiles a little further out: the objects at the edge stand on them
        return math.hypot(x - cx, y - cy) <= args.radius + (60.0 if len(it) > 6 and it[6] in ("ground", "water") else 0.0)

    if args.live:
        recs = live_records(world)
    else:
        sfile = Path(args.scenery) if args.scenery else mdir / "scenery.json"
        data = json.loads(sfile.read_text())
        cat = scenery.Catalog(data["catalog"])
        boxes = json.loads((scenery.DATA / f"meshes_{data['catalog']}.json").read_text())["meshes"]
        try:        # the parked cars' boxes (the server's bounding_box of a static mesh is not its own box)
            boxes.update(json.loads((scenery.DATA / "parked_cars.json").read_text()).get(data["catalog"], {}))
        except (OSError, ValueError):
            pass
        ms = cat.m
        shelters = set(ms.SHELTER)
        items = [it for it in data["items"] if near(it)]
        for it, aid in spawn_items(client, world, data["meshes"], items, "scenery"):
            spawned.append(aid)
            name = data["meshes"][it[0]].rsplit(".", 1)[-1]
            kind = it[6]
            extra = {}
            if kind == "sign":
                kind = "sign_post" if name == ms.POST else "sign_plate"
                if kind == "sign_plate" and "noenter" not in name.lower() and "donoenter" not in name.lower():
                    extra["face"] = (it[4] - ms.SIGN_YAW + 90.0) % 360.0
            elif kind == "furniture" and name in shelters:
                kind = "shelter"
            a = world.get_actor(aid)
            box = boxes.get(data["meshes"][it[0]])
            if kind in ("ground", "water"):
                tiles.append((a, it[5], box))
            recs.append(record(kind, a, name, it[5], extra, cat=box))   # ground tiles: for the road scan
        pfile = Path(args.parked) if args.parked else mdir / "parked.json"
        if pfile.exists():
            pdata = json.loads(pfile.read_text())
            srcs = pdata.get("sources") or []
            slots = [s for s in pdata["slots"] if len(s) <= 8 and near(s)]
            items = [[s[0], s[1], s[2], s[3], s[4], 1.0, "parked"] for s in slots]
            by_pos = {(s[1], s[2], s[4]): s for s in slots}
            for it, aid in spawn_items(client, world, pdata["meshes"], items, "parked"):
                spawned.append(aid)
                s = by_pos.get((it[1], it[2], it[4]))
                src = srcs[s[7]] if s is not None and len(s) > 7 and s[7] < len(srcs) else ""
                recs.append(record("parked", world.get_actor(aid), pdata["meshes"][it[0]].rsplit(".", 1)[-1], 1.0,
                                   {"kerb": not str(src).startswith("lot"), "src": src},
                                   cat=boxes.get(pdata["meshes"][it[0]])))
        signals = start_signals(world, cfg, mdir, args.style, sfile)
        step(world)
        if signals is not None:
            for a in signals.actors:
                if math.hypot(a.get_location().x - cx, a.get_location().y - cy) > args.radius:
                    continue
                recs.append(record("signal_part", a, a.attributes.get("mesh_path", "").rsplit(".", 1)[-1] or None,
                                   float(a.attributes.get("scale", 1.0))))
            # the heads themselves, as points (their backboards are among the actors above)
            for tls, idxs, loc, yaw in signals.heads:
                if math.hypot(loc.x - cx, loc.y - cy) <= args.radius:
                    recs.append({"kind": "signal", "id": None, "mesh": tls, "x": round(loc.x, 2), "y": round(loc.y, 2),
                                 "z": round(loc.z, 2), "bottom": round(loc.z, 2), "top": round(loc.z + 3.0, 2),
                                 "yaw": round(yaw, 1), "tilt": 0.0, "foot": [], "problems": [], "detail": {}})
        recs += [r for r in carla_own(world) if math.hypot(r["x"] - cx, r["y"] - cy) <= args.radius]
    # a taller sign post is a second post standing on the first (ndwsigns.place): it does not stand on the ground
    posts = {}
    for r in recs:
        if r["kind"] == "sign_post":
            posts.setdefault((round(r["x"], 1), round(r["y"], 1)), []).append(r)
    for group in posts.values():
        for r in sorted(group, key=lambda r: r["bottom"])[1:]:
            r["kind"] = "sign_post_upper"
    # signal pole meshes: told apart by height (the pole is 3 m tall, the backboard under 1 m)
    for r in recs:
        if r["kind"] == "signal_part":
            r["kind"] = "signal_pole" if r["top"] - r["bottom"] > 1.5 else "signal_board"
    for _ in range(5):
        step(world)
    print(f"  {len(recs)} objects to audit ({time.time() - t0:.0f} s)")
    tops = TileTops(tiles) if tiles else None
    check_ground(world, recs, tops)
    drops = [] if args.live else kerb_drops(world, cmap, (cx, cy), args.radius, tops)
    kerb = {"points": len(drops), "over_0.4m": sum(d > 0.4 for d in drops), "over_1m": sum(d > 1.0 for d in drops),
            "median": round(statistics.median(drops), 2) if drops else None,
            "p95": round(sorted(drops)[int(0.95 * len(drops))], 2) if drops else None}
    print(f"  ground beside the roads: {kerb}")
    check_tilt(recs)
    if not args.live:
        check_drivable(cmap, junctions, recs)
        check_in_building(recs)
        check_facing(cmap, recs, bool(cfg["map"].get("lefthand")))
        placement.duplicates(recs)
        placement.overlaps(recs, {"parked"}, {"parked", "building", "landmark", "lamp", "sign_post", "shelter",
                                              "signal_pole"}, min_area=0.3)
    unattributed, n_rays = ({}, 0) if args.no_road_scan else scan_roads(world, cmap, (cx, cy), args.radius, recs)
    per, tot = placement.summary(recs)
    frames = take_frames(world, recs, out, args.frames, cmap) if args.frames else []
    examples = {p: [{k: r.get(k) for k in ("kind", "mesh", "x", "y", "bottom", "ground", "gap", "detail")}
                    for r in placement.worst(recs, p, 6)] for p in sorted(tot)}
    for e in examples.values():
        for r in e:
            r["detail"] = {k: v for k, v in (r["detail"] or {}).items() if k != "ground_labels"}
    rep = {"experiment": args.experiment, "map": cfg["map"]["name"], "centre": [round(cx, 1), round(cy, 1)],
           "radius_m": args.radius, "server": client.get_server_version(), "carla_lights": args.carla_lights,
           "scenery": args.scenery, "parked": args.parked, "live": args.live, "objects": len(recs),
           "per_kind": per, "problems": tot, "road_rays": n_rays, "road_unattributed": unattributed,
           "road_unattributed_at": UNATTRIBUTED, "kerb_drop": kerb,
           "examples": examples, "frames": frames, "seconds": round(time.time() - t0, 1)}
    (out / "report.json").write_text(json.dumps(rep, indent=1))
    (out / "records.json").write_text(json.dumps(recs, separators=(",", ":")))
    lines = [f"placement audit: {cfg['map']['name']} within {args.radius:.0f} m of ({cx:.0f}, {cy:.0f}), "
             f"{len(recs)} objects, {rep['seconds']} s"]
    probs = sorted(tot)
    lines.append(f"{'kind':14s} {'n':>6s} " + " ".join(f"{p[:12]:>12s}" for p in probs))
    for k in sorted(per):
        lines.append(f"{k:14s} {per[k]['n']:6d} " + " ".join(f"{per[k].get(p, 0):12d}" for p in probs))
    lines.append(f"{'total':14s} {len(recs):6d} " + " ".join(f"{tot[p]:12d}" for p in probs))
    lines.append(f"road scan: {n_rays} rays, hits on nothing spawned here: {unattributed}")
    lines.append(f"ground 1.5 m beyond the outermost lanes, under the road by: {kerb}")
    for p, ex in examples.items():
        lines.append(f"worst {p}:")
        for r in ex[:4]:
            lines.append(f"  {r['kind']:12s} {str(r['mesh'])[:34]:34s} ({r['x']:.1f}, {r['y']:.1f}) "
                         f"gap {r.get('gap')} {r['detail'] or ''}")
    (out / "report.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    if spawned and not args.keep:
        client.apply_batch_sync([carla.command.DestroyActor(i) for i in spawned], False)
    if signals is not None:
        if not args.keep:
            signals.destroy()
        import traci
        traci.close()


if __name__ == "__main__":
    main()
