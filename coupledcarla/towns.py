"""CARLA's standard maps: Town01-Town07 and Town10HD (and their _Opt variants) on CARLA 0.9.16, the _Opt towns and
Mine_01 on CARLA 1.0, driven like any coupled-carla map.

    map: {carla_town: Town10HD_Opt}

picks a town. The level is loaded by name (map.carla_level, as for the Eindhoven twin) and SUMO's net is built from
the town's own OpenDRIVE, cached in maps/<town>/: the file from CARLA_ROOT (0.9.16's
CarlaUE4/Content/Carla/Maps/OpenDrive, a 1.0 source tree's Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive), else
world.get_map().to_opendrive() of the server (a CARLA 1.0 package keeps its OpenDRIVE inside the pak), started for
the purpose when nothing listens on server.host:port. mapbuild.build_from_opendrive then does what it does for
twins: geo.net.xml and map.net.xml from the same file (without guessed traffic lights: a town's lights are its own).

What each feature means in a town (overrides() sets the defaults; anything the experiment sets itself wins):

  SUMO traffic       cars, bikes and random buses on the town's lanes (the town has no bus lines; cars_per_hour per
                     town from TOWNS, bikes fewer: CARLA's towns are North American)
  traffic lights     the town's own CARLA traffic-light actors show SUMO's signal states (the bridge syncs them by
                     OpenDRIVE id, as CARLA's own SUMO co-simulation); none are drawn (traffic_lights.style none)
  parked cars        in the OpenDRIVE's parking lanes (Town03, Town05, Town07) and on 2-3 m kerb shoulders of town
                     streets (Town03, Town05, Town10HD; the 3.5 m hard shoulders of highways stay empty),
                     clear of crosswalks, signals and junctions (parking.py, map.scenery.park_shoulders_m)
  crowd              crowdpeds.py's calibrated crowd on the kerbs (no OSM places or crossings: the town's crosswalks
                     come from its OpenDRIVE), plus traffic.walkers.ai: CARLA's own AI walkers on the town's
                     navigation mesh (WalkerAIController), which wander the pavements and cross at crosswalks
  weather and sun    presets; real weather and the sun of map.place (a [lat, lon] and simulation.time; default the
                     town's geoReference, 0 N 0 E in the Gulf of Guinea)
  night              the town's own street and building lights (LightManager), simulation.town_lights
  sound              building occlusion and reflections from the town's buildings (environment objects' boxes,
                     buildings.json, written on the first trial)
  drones             roofs from the same buildings.json, tree tops from the vegetation boxes (trees.json)
  scenery            off: the town has its own buildings, trees, furniture, signs and ground (and no OSM data)
  real-world data    off: no terrain model, OSM bus lines, counts, register fleet, cycle routes or urban speed limits

CARLA (x, y) of a town and lat/lon (geo.net.xml, from the town's tmerc geoReference) convert with
`python -m coupledcarla.towns latlon Town03 x y`; experiment points may also be given as {x, y} dicts (resolve_xy).
"""
import argparse
import json
import math
import os
import re
import shutil
import socket
from pathlib import Path

from .geo import nearest_waypoint

# name -> (what it is, cars entering per hour, bikes per hour). CARLA 0.9.16 has each with and without _Opt (the
# layered version: same roads, map layers that can be unloaded); CARLA 1.0 ships the _Opt towns and Mine_01.
TOWNS = {
    "Town01": ("a small town with a river and bridges, T-junctions", 500, 60),
    "Town02": ("a small residential and commercial town, T-junctions", 400, 60),
    "Town03": ("the largest urban map: a roundabout, an underpass, ramps, a large junction", 1600, 120),
    "Town04": ("a small town in the mountains inside a figure-of-eight highway", 2400, 60),
    "Town05": ("a squared grid town with multi-lane roads, a bridge and a highway", 2000, 120),
    "Town06": ("long highways with many entrances and exits, Michigan lefts", 2000, 0),
    "Town07": ("a rural village with narrow roads, barns and few traffic lights", 350, 40),
    "Town10HD": ("downtown: avenues, skyscrapers and a waterfront", 900, 120),
    "Mine_01": ("an open-pit mine (CARLA 1.0)", 150, 0),
}
CARLA_10 = ["Town01_Opt", "Town02_Opt", "Town03_Opt", "Town04_Opt", "Town05_Opt", "Town06_Opt", "Town07_Opt",
            "Town10HD_Opt", "Mine_01"]
XODR_DIRS = ["CarlaUE4/Content/Carla/Maps/OpenDrive",                 # 0.9.x release and source
             "Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive",       # 1.0 source tree
             "CarlaUnreal/Content/Carla/Maps/OpenDrive"]              # a 1.0 package that ships loose files
PARK_SHOULDERS_M = [2.0, 3.0]   # kerb shoulders this wide are parking strips (highway hard shoulders: 3.5 m)


def base_name(name):
    """'Town10HD_Opt' -> 'Town10HD'; anything else unchanged."""
    return re.sub(r"_Opt$", "", str(name or ""))


def is_town(name):
    """A CARLA standard map name (Town01..Town15 with or without _Opt, Mine_01, ...)."""
    return bool(name) and bool(re.fullmatch(r"(Town\d+\w*|Mine_\d+)", str(name)))


def town_of(cfg):
    """The experiment's map.carla_town, or None."""
    return ((cfg or {}).get("map") or {}).get("carla_town") or None


def info(name):
    """(description, cars per hour, bikes per hour) of a town; unknown towns get a middling default."""
    return TOWNS.get(base_name(name), ("a CARLA map", 800, 60))


def overrides(raw):
    """Defaults for an experiment on a town (merged under the experiment's own settings by config.load); {} when the
    experiment names no town. raw: the experiment file as written."""
    town = town_of(raw)
    if not town:
        return {}
    if not is_town(town):
        raise ValueError(f"map.carla_town: {town!r} is not a CARLA town name (Town01..Town07, Town10HD, *_Opt, "
                         f"Mine_01)")
    _desc, cars, bikes = info(town)
    off = {k: False for k in ("buildings", "trees", "furniture", "guess_lamps", "signs", "municipal", "ground",
                              "bgt", "canopy", "ndw_signs", "shared")}
    return {
        "map": {"name": town, "carla_level": town, "elevation": False, "transit": False, "buildings": False,
                "overture": False, "national_buildings": False, "urban_speeds": False,
                "scenery": dict(off, parked=True, park_shoulders_m=list(PARK_SHOULDERS_M))},
        "simulation": {"traffic_lights": {"style": "none"}},
        "traffic": {"cars_per_hour": cars, "bikes_per_hour": bikes, "bus_min_distance_m": 300, "counts": False, "speed_limits": False,
                    "speed_factor": False, "cycling": {"routes": False, "counts": False, "dock_weight": 0},
                    "walkers": {"places": False}},
        # crowdpeds.locate: CARLA's towns are modelled on North American towns (else the town's geoReference,
        # which lies in the sea, gives the world's numbers)
        "city": {"country": "USA"},
    }


# --- the town's OpenDRIVE -------------------------------------------------------------------------------------------

def local_opendrive(town, roots=None):
    """The town's .xodr under CARLA_ROOT (0.9.x or a 1.0 source tree), or None."""
    roots = roots if roots is not None else [os.environ.get("CARLA_ROOT")]
    for root in roots:
        if not root:
            continue
        for d in XODR_DIRS:
            p = Path(root).expanduser() / d / f"{town}.xodr"
            if p.exists() and p.stat().st_size > 0:
                return p
    return None


def _port_open(host, port):
    try:
        with socket.create_connection((host, int(port)), timeout=2.0):
            return True
    except OSError:
        return False


def server_opendrive(sc, town, log=print):
    """The town's OpenDRIVE from a CARLA server (world.get_map().to_opendrive()): the one listening on
    server.host:port, else (server.launch and CARLA_ROOT) one started for this and stopped again."""
    import carla
    server = None
    if not _port_open(sc["host"], sc["port"]):
        if not (sc.get("launch", True) and os.environ.get("CARLA_ROOT")):
            raise RuntimeError(f"no OpenDRIVE for {town}: not under CARLA_ROOT and no CARLA server on "
                               f"{sc['host']}:{sc['port']} (start one, set CARLA_ROOT, or set map.opendrive)")
        from .server import Server
        log(f"{town}: starting CARLA once to read the town's OpenDRIVE")
        server = Server(sc, log)
    try:
        client = server.start() if server else carla.Client(sc["host"], int(sc["port"]))
        client.set_timeout(600.0)
        world = client.get_world()
        if not world.get_map().name.endswith(town):
            world = client.load_world(town)
        text = world.get_map().to_opendrive()
        client.set_timeout(60.0)
        return text
    finally:
        if server is not None:
            server.stop()


def opendrive_file(cfg, out, log=print):
    """maps/<town>/town.xodr, copied from CARLA_ROOT or read from a server once (map.opendrive, when set, is used as
    it is by mapbuild)."""
    town = town_of(cfg)
    out = Path(out)
    dest = out / "town.xodr"
    if dest.exists() and dest.stat().st_size > 0:
        return dest
    out.mkdir(parents=True, exist_ok=True)
    src = local_opendrive(town)
    if src is not None:
        shutil.copyfile(src, dest)
        log(f"{town}: OpenDRIVE from {src}")
    else:
        dest.write_text(server_opendrive(cfg["server"], town, log), encoding="utf-8")
        log(f"{town}: OpenDRIVE from the server ({dest.stat().st_size / 1e6:.1f} MB)")
    return dest


def missing_level(client, level, err):
    """The message for a level the server does not have: its levels, and where CARLA's other towns come from."""
    try:
        have = sorted(m.rsplit("/", 1)[-1] for m in client.get_available_maps())
    except RuntimeError:
        have = []
    hint = ""
    if base_name(level) in ("Town06", "Town07"):
        hint = " Town06 and Town07 come with CARLA 0.9.16's AdditionalMaps package (import it into the release)."
    elif is_town(level) and not level.endswith("_Opt"):
        hint = " CARLA 1.0 ships only the _Opt towns."
    return f"{err}: this server has {', '.join(have) or 'no levels listed'}.{hint}"


def georeference(text):
    """The <geoReference> projection string of an OpenDRIVE text, or None."""
    m = re.search(r"<geoReference>\s*(?:<!\[CDATA\[)?\s*([^<\]]*?)\s*(?:\]\]>)?\s*</geoReference>", text)
    return m.group(1).strip() if m else None


def geo_origin(text):
    """(lat_0, lon_0) of a geoReference (CARLA's towns: tmerc at 0, 0), or None."""
    ref = georeference(text) or ""
    lat, lon = re.search(r"\+lat_0=([-0-9.e+]+)", ref), re.search(r"\+lon_0=([-0-9.e+]+)", ref)
    return (float(lat.group(1)), float(lon.group(1))) if lat and lon else None


# --- map build: parked cars only -----------------------------------------------------------------------------------

def build_scenery(cfg, out, meta, cmap=None, server_major=None, log=print):
    """A town's scenery: none of coupled-carla's meshes (the town has its own; an empty scenery.json in the catalogue
    of the CARLA in use keeps the run loop's streaming as on other maps), and parked cars (parking.py, baked-level
    rules: the OpenDRIVE's parking lanes and park_shoulders_m-wide shoulders). Recorded in meta."""
    import carla
    from types import SimpleNamespace
    from . import parking, scenery
    out = Path(out)
    mc = cfg["map"]
    sc = mc.get("scenery") or {}
    version = scenery.catalog_version(sc.get("catalog", "auto"), server_major)
    if version is None:
        raise ValueError(f"no mesh catalogue for CARLA {server_major}.x")
    cat = scenery.Catalog(version)
    if cmap is None:
        cmap = carla.Map(mc["name"], (out / "map.xodr").read_text(encoding="utf-8"))
    (out / "scenery.json").write_text(json.dumps(
        {"version": 1, "catalog": cat.version, "cell_m": scenery.CELL, "meshes": [], "items": [], "counts": {},
         "sources": [], "leaf": {}, "town": town_of(cfg), "snapped": "opendrive"}, separators=(",", ":")))
    counts = {}
    if sc.get("parked", True):
        def ground(x, y):
            wp = nearest_waypoint(cmap, x, y)
            return wp.transform.location.z if wp is not None else 0.0
        bbox = mc.get("bbox") or meta.get("bbox")
        counts = parking.build(out, dict(cfg, map=dict(mc, bbox=bbox)), None, cat, cmap, ground,
                               SimpleNamespace(items=[], mesh_index={}), [], log) or {}
        pj = out / "parked.json"
        if pj.exists():     # a town's OpenDRIVE heights are its level's road surface: nothing to snap
            data = json.loads(pj.read_text())
            data["snapped"] = "opendrive"
            pj.write_text(json.dumps(data, separators=(",", ":")))
    meta["scenery"] = counts
    meta["buildings"] = meta.get("buildings", 0)
    meta.setdefault("sources", {})
    meta["town"] = town_of(cfg)
    return meta


# --- run time ------------------------------------------------------------------------------------------------------

def _label(carla, *names):
    for n in names:
        if hasattr(carla.CityObjectLabel, n):
            return getattr(carla.CityObjectLabel, n)
    return None


def footprints(objs, on_road=None, max_area=40000.0):
    """Environment objects' bounding boxes as buildings.json records [{"h", "z", "k", "s", "p"}] (CARLA x, y):
    the box's corners at its base, its height. Boxes under 2 m tall or 4 m2, over max_area (merged meshes) or whose
    centre is on a road (on_road(x, y) -> bool: a bridge or a merged block) are left out."""
    out = []
    for o in objs:
        bb = o.bounding_box
        yaw = math.radians(bb.rotation.yaw)
        ex, ey, ez = abs(bb.extent.x), abs(bb.extent.y), abs(bb.extent.z)
        if 2 * ez < 2.0 or 4 * ex * ey < 4.0 or 4 * ex * ey > max_area:
            continue
        cx, cy = bb.location.x, bb.location.y
        if on_road is not None and on_road(cx, cy):
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [[round(cx + a * ex * c - b * ey * s, 2), round(cy + a * ex * s + b * ey * c, 2)]
               for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        h = 2 * ez
        kind = "house" if h < 12.0 and 4 * ex * ey < 400.0 else "commercial"
        out.append({"h": round(h, 1), "z": round(bb.location.z - ez, 2), "k": kind, "s": "carla", "p": pts})
    return out


def tree_tops(objs, min_h=2.5, max_r=8.0):
    """Vegetation boxes as [x, y, top z, radius] (drones.py's trees)."""
    out = []
    for o in objs:
        bb = o.bounding_box
        h = 2 * abs(bb.extent.z)
        if h < min_h:
            continue
        r = min(max_r, max(0.5, max(abs(bb.extent.x), abs(bb.extent.y))))
        out.append([round(bb.location.x, 2), round(bb.location.y, 2), round(bb.location.z + abs(bb.extent.z), 2),
                    round(r, 2)])
    return out


def environment(world, mdir, server_version, log=print):
    """buildings.json (sound occlusion, drones, viewer) and trees.json (drones) from the loaded town's environment
    objects, written once per CARLA version (town_objects.json says which). Returns {"buildings": n, "trees": n,
    "parked_layer": n} or None when the server gives nothing."""
    import carla
    mdir = Path(mdir)
    stamp = mdir / "town_objects.json"
    if stamp.exists() and (mdir / "buildings.json").exists():
        got = json.loads(stamp.read_text())
        if got.get("carla") == server_version:
            return got
    cmap = world.get_map()

    def on_road(x, y):
        return cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=False,
                                 lane_type=carla.LaneType.Driving) is not None
    try:
        bl, veg, car = (_label(carla, "Buildings", "Building"), _label(carla, "Vegetation"),
                        _label(carla, "Car", "Vehicles"))
        blds = footprints(world.get_environment_objects(bl), on_road) if bl is not None else []
        trees = tree_tops(world.get_environment_objects(veg)) if veg is not None else []
        parked = len(world.get_environment_objects(car)) if car is not None else 0
    except RuntimeError as err:
        log(f"town: environment objects not read ({err}); no building occlusion or drone obstacles")
        return None
    (mdir / "buildings.json").write_text(json.dumps(blds, separators=(",", ":")))
    (mdir / "trees.json").write_text(json.dumps(trees, separators=(",", ":")))
    got = {"carla": server_version, "buildings": len(blds), "trees": len(trees), "parked_layer": parked}
    stamp.write_text(json.dumps(got))
    log(f"town: {len(blds)} buildings and {len(trees)} trees from the level's environment objects "
        f"(buildings.json, trees.json); {parked} static cars of its own")
    return got


def own_parked(world, tc, mdir, log=print):
    """Whether Coupled CARLA places its parked cars in this town (True) or the level keeps its own static ones
    (False). simulation.parked.town_layer: "auto" places ours where the map has stalls and then unloads the _Opt
    towns' ParkedVehicles layer (so cars do not stand in each other); a full (non-_Opt) level, whose parked cars
    cannot be unloaded, keeps them when it has any; true keeps the level's cars and places none of ours; false
    unloads the layer and places ours."""
    import carla
    pc = (tc.get("simulation") or {}).get("parked") or {}
    want = pc.get("town_layer", "auto")
    pj, objs = Path(mdir) / "parked.json", Path(mdir) / "town_objects.json"
    stalls = bool(json.loads(pj.read_text()).get("slots")) if pj.exists() else False
    level_cars = json.loads(objs.read_text()).get("parked_layer", 0) if objs.exists() else 0
    layered = str(tc["map"].get("carla_level", "")).endswith("_Opt")
    if want is True:
        return False
    ours = stalls and (want is False or layered or not level_cars)
    if ours and layered:
        try:
            world.unload_map_layer(carla.MapLayer.ParkedVehicles)
            log("town: the level's own parked cars unloaded (Coupled CARLA's parked cars stand in their place)")
        except (RuntimeError, AttributeError) as err:
            log(f"town: parked-car layer not unloaded ({err})")
    elif not ours and level_cars:
        log(f"town: the level's own {level_cars} parked cars")
    return ours


def lights(world, night, mode="auto", log=print):
    """The town's street and building lights (LightManager): on at night and off by day ("auto"), or "on" / "off";
    False leaves them to the server. Returns the number switched on."""
    import carla
    if mode is False or mode in ("none", "server"):
        return 0
    on = night if mode in (None, "auto", True) else mode == "on"
    try:
        lm = world.get_lightmanager()
        groups = [g for g in (getattr(carla.LightGroup, "Street", None), getattr(carla.LightGroup, "Building", None))
                  if g is not None]
        lamps = [lt for g in groups for lt in lm.get_all_lights(g)]
        if mode not in (None, "auto", True):
            lm.set_day_night_cycle(False)
        (lm.turn_on if on else lm.turn_off)(lamps)
    except (RuntimeError, AttributeError) as err:
        log(f"town: lights not switched ({err})")
        return 0
    if on:
        log(f"town: {len(lamps)} street and building lights on")
    return len(lamps) if on else 0


def snap_kerbs(cmap, segments, log=print, reach=6.0):
    """The crowd's kerb lines (crowdpeds segments, geo x, y) moved onto the town's pavements: each point to the
    centre of the nearest sidewalk lane within reach metres. The SUMO net's kerb offset does not know a town's walls
    and sunken roads (in Town03's underpass a walker stepped off the wall's top). Returns the points moved."""
    import carla
    n = tot = 0
    for seg in segments:
        for side in seg.sides:
            for i, (x, y) in enumerate(side):
                tot += 1
                wp = nearest_waypoint(cmap, x, -y, carla.LaneType.Sidewalk)
                if wp is None:
                    continue
                loc = wp.transform.location
                if math.hypot(loc.x - x, loc.y + y) <= reach:
                    side[i] = (loc.x, -loc.y)
                    n += 1
    log(f"crowd: {n} of {tot} kerb points on the town's pavements")
    return n


def off_road(cmap, loc, tol=2.5):
    """True when a car at loc is on no driving lane (a town has kerbs and real heights: the z < -0.25 test of the
    generated worlds' ground plane does not apply). tol: metres beyond the lane centre still counted as on it."""
    import carla
    wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return True
    d = wp.transform.location
    return math.hypot(d.x - loc.x, d.y - loc.y) > wp.lane_width / 2 + tol or abs(d.z - loc.z) > 4.0


class NavWalkers:
    """CARLA's own AI walkers (WalkerAIController) on the level's navigation mesh: they walk the pavements to random
    destinations, cross at crosswalks (and, with cross_share, elsewhere) and get a new destination when they arrive.
    Only levels with a navigation mesh (CARLA's towns, baked twins); OpenDRIVE-generated worlds serve none.

    traffic.walkers.ai: {count, radius_m (around the participants at the start; 0: anywhere), speed_mps, run_share,
    cross_share, seed}"""

    def __init__(self, client, world, cfg, centres, log=print):
        import carla
        import random
        self.world, self.log = world, log
        self.walkers, self.controllers, self._road, self._t_next = [], [], [], 0.0
        n = int(cfg.get("count", 0))
        if n <= 0:
            return
        rng = random.Random(int(cfg.get("seed", 3)))
        radius = float(cfg.get("radius_m", 150.0))
        lib = world.get_blueprint_library()
        bps = [bp for bp in lib.filter("walker.pedestrian.*")]
        ctl_bp = lib.find("controller.ai.walker")
        world.set_pedestrians_seed(int(cfg.get("seed", 3)))
        spots = []
        for _ in range(n * 40):
            loc = world.get_random_location_from_navigation()
            if loc is None:
                break
            if radius <= 0 or not centres or any(math.hypot(loc.x - c.x, loc.y - c.y) <= radius for c in centres):
                spots.append(loc)
            if len(spots) >= n:
                break
        if not spots:
            log("town: no navigation mesh on this level; no AI walkers")
            return
        cmds = []
        speeds = []
        base = float(cfg.get("speed_mps", 1.35))
        from . import walkerlooks
        for loc in spots:
            bp = walkerlooks.choose_blueprint(lib, rng, bps)      # the place's look (walkerlooks.py), else uniform
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            run = rng.random() < float(cfg.get("run_share", 0.05))
            speeds.append(base * (2.2 if run else rng.uniform(0.8, 1.2)))
            cmds.append(carla.command.SpawnActor(bp, carla.Transform(loc + carla.Location(z=1.0))))
        ids = [r.actor_id for r in client.apply_batch_sync(cmds) if not r.error]
        self.speeds = dict(zip(ids, speeds))
        self.walkers = [a for a in world.get_actors(ids)]
        self.ctl_bp = ctl_bp
        self.cross = float(cfg.get("cross_share", 0.05))

    def start(self):
        """Controllers for the walkers (after a tick: an actor has its place only then), started towards a random
        destination."""
        import carla
        if not self.walkers:
            return
        self.controllers = [self.world.spawn_actor(self.ctl_bp, carla.Transform(), attach_to=w) for w in self.walkers]
        self.world.set_pedestrians_cross_factor(self.cross)

    def go(self):
        """Second step, after another tick: start each controller and send it off."""
        for c, w in zip(self.controllers, self.walkers):
            c.start()
            dest = self.world.get_random_location_from_navigation()
            if dest is not None:
                c.go_to_location(dest)
            c.set_max_speed(self.speeds.get(w.id, 1.35))
        if self.controllers:
            self.log(f"town: {len(self.controllers)} AI walkers on the navigation mesh")

    def step(self, t, cmap):
        """Every 0.5 s: which walkers are on a driving lane (SUMO and scripted cars yield to them); every 10 s a new
        destination for a few that stand still (arrived)."""
        import carla
        if not self.walkers or t < self._t_next:
            return
        self._t_next = t + 0.5
        road = []
        for w in self.walkers:
            loc = w.get_location()
            if cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                road.append(w)
        self._road = road
        if int(t * 2) % 20 == 0:
            sent = 0
            for c, w in zip(self.controllers, self.walkers):
                v = w.get_velocity()
                if v.x * v.x + v.y * v.y < 0.01 and sent < 3:
                    dest = self.world.get_random_location_from_navigation()
                    if dest is not None:
                        c.go_to_location(dest)
                        sent += 1

    def on_road(self):
        return list(self._road)

    def destroy(self):
        for c in self.controllers:
            try:
                c.stop()
                c.destroy()
            except RuntimeError:
                pass
        for w in self.walkers:
            try:
                w.destroy()
            except RuntimeError:
                pass
        self.walkers, self.controllers = [], []


def resolve_xy(tc, geo):
    """Points given in CARLA coordinates become lat, lon with the map's geo (geo.Geo) in the participants, agents,
    points of interest, drones and film sections (in a CARLA town people know places by their CARLA x, y):
    {x: .., y: ..} dicts (heading, height_m, ... kept) become {lat, lon}, and a key ending in _xy holding [x, y] or
    a list of [x, y(, height)] becomes the key without _xy in [lat, lon(, height)] (route_xy -> route, path_xy ->
    path, at_xy -> at). Returns how many were converted."""
    n = [0]

    def pt(p):
        lat, lon = geo.to_latlon(float(p[0]), float(p[1]))
        return [round(lat, 8), round(lon, 8), *p[2:]]

    def walk(v):
        if isinstance(v, dict):
            for k in [k for k in v if isinstance(k, str) and k.endswith("_xy")]:
                pts = v.pop(k)          # route_xy, path_xy, at_xy, ...: [x, y(, more)] or a list of them
                single = bool(pts) and isinstance(pts[0], (int, float))
                v[k[:-3]] = pt(pts) if single else [pt(p) for p in pts]
                n[0] += 1
            if "x" in v and "y" in v and "lat" not in v and isinstance(v["x"], (int, float)) \
                    and isinstance(v["y"], (int, float)):
                lat, lon = geo.to_latlon(float(v["x"]), float(v["y"]))
                v["lat"], v["lon"] = round(lat, 8), round(lon, 8)
                del v["x"], v["y"]
                n[0] += 1
            for k in list(v):
                walk(v[k])
        elif isinstance(v, list):
            for item in v:
                walk(item)
    for key in ("participants", "agents", "pois", "drones", "film"):
        walk(tc.get(key))
    return n[0]


# --- command line --------------------------------------------------------------------------------------------------

def crosswalks(xodr_path):
    """The crosswalks of a town's OpenDRIVE: [{"x", "y", "length", "road"}] (CARLA x, y of each crosswalk's centre,
    its length across the road)."""
    import carla
    from . import parking
    text = Path(xodr_path).read_text(encoding="utf-8")
    cmap = carla.Map("town", text)
    pts = parking._xodr_points(xodr_path, cmap)
    out = []
    for poly in pts["crossing"]:
        c = poly.centroid
        x0, y0, x1, y1 = poly.bounds
        out.append({"x": round(c.x, 1), "y": round(c.y, 1), "length": round(max(x1 - x0, y1 - y0), 1)})
    return out


def main():
    ap = argparse.ArgumentParser(description="CARLA towns in coupled-carla")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="the towns and what they are")
    for name in ("latlon", "xy"):
        p = sub.add_parser(name, help="CARLA x, y -> lat, lon (latlon) or back (xy), for a built town map")
        p.add_argument("town")
        p.add_argument("a", type=float)
        p.add_argument("b", type=float)
    p = sub.add_parser("crosswalks", help="crosswalks of a built town map (CARLA x, y and lat, lon)")
    p.add_argument("town")
    args = ap.parse_args()
    root = Path(__file__).resolve().parent.parent
    if args.cmd == "list":
        for name, (desc, cars, bikes) in TOWNS.items():
            print(f"{name:10s} {desc}; {cars} cars/h, {bikes} bikes/h")
        print(f"CARLA 1.0: {', '.join(CARLA_10)}")
        return
    mdir = root / "maps" / args.town
    if not (mdir / "geo.net.xml").exists():
        raise SystemExit(f"{mdir} is not built: python -m coupledcarla.mapbuild <an experiment with "
                         f"map.carla_town: {args.town}>")
    from .geo import Geo
    geo = Geo(mdir / "geo.net.xml")
    if args.cmd == "latlon":
        lat, lon = geo.to_latlon(args.a, args.b)
        print(f"{lat:.8f} {lon:.8f}")
    elif args.cmd == "xy":
        x, y = geo.to_carla(args.a, args.b)
        print(f"{x:.2f} {y:.2f}")
    else:
        for c in crosswalks(mdir / "map.xodr"):
            lat, lon = geo.to_latlon(c["x"], c["y"])
            print(f"x {c['x']:8.1f}  y {c['y']:8.1f}  lat {lat:.8f}  lon {lon:.8f}  length {c['length']:.1f} m")


if __name__ == "__main__":
    main()
