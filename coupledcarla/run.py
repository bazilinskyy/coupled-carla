"""Run an experiment's trials (the host): CARLA world + SUMO traffic + participants + AI agents + logging.

    python -m coupledcarla.run experiments/eindhoven_centre.yaml [--trial 0] [--input keyboard] [--headless]
    python -m coupledcarla.run experiments/eindhoven_crossing.yaml        # coupled: waits for its stations

Each tick: apply the inputs of host-local participants and step the AI agents, then step SUMO and CARLA
together through the bridge (SUMO moves its vehicles in CARLA; CARLA-driven cars, human or AI, are mirrored
into SUMO, so traffic reacts to them), let SUMO traffic yield to pedestrians, draw traffic lights and eHMIs,
handle the stations' messages, log the frame and draw the host's view. With simulation.real_time the loop
sleeps so that one simulated second takes one wall-clock second.

Participants (coupled-sim's roles) are `driver` (a manual car), `pedestrian` (a walker), `av_passenger`
(rides in a scripted AV and sets its eHMI) and `drone` (flies a quadrotor; drones.py, with the AI drones). Each is either host-local (input keyboard / wheel / autopilot /
bot on this machine) or `station: true`: a human at a station (python -m coupledcarla.station) that renders
its own view and applies its own controls; the host waits for the stations before a trial starts.
"""
import argparse
import json
import random
import time
from pathlib import Path

import carla
import yaml

from . import analysis, compat, config, crowdpeds, crowdwindow, drivermix, inputs, livingcity, look, mapbuild, \
    nightlights, parking, photoreal, reactions, realworld, season, soundrec, study, towns, traffic
from .ai import DEFAULT_EHMI, MutualGaze, PathWalker, ScriptedCar, ScriptedWalker, SumoYield, draw_ehmi
from . import ehmi_design, ehmi_inside, farring, levels, levelxodr, livemap, multigpu, placement, plates, \
    staticactors, stationux, walkerlooks
from .bridge.synchronization import BridgeHelper, CarlaSimulation, SimulationSynchronization, SumoSimulation
from .coupling import ControlServer
from .display import View
from .drones import DroneView, DroneWorld
from .ships import ShipWorld
from .geo import Geo, ground, heading_to_yaw, nearest_waypoint
from .logger import TrialLogger
from .routing import SafeGlobalRoutePlanner
from .scenery import Scenery
from .server import Server
from .side import Side
from .signals import Signals
from .studytools import HostStudy


class Laps:
    """Wall time per part of the loop (simulation.profile prints it every 10 s and puts the trial's totals in
    summary.json). laps(name) adds the time since the previous lap or start() to `name`."""

    def __init__(self):
        self.acc, self.total = {}, {}
        self.t = time.perf_counter()

    def start(self):
        self.t = time.perf_counter()

    def __call__(self, name):
        now = time.perf_counter()
        self.acc[name] = self.acc.get(name, 0.0) + now - self.t
        self.t = now

    def line(self, n):
        return "  ".join(f"{k} {1000 * v / n:.2f}" for k, v in self.acc.items()) + \
            f"  = {1000 * sum(self.acc.values()) / n:.1f}"

    def reset(self):
        for k, v in self.acc.items():
            self.total[k] = self.total.get(k, 0.0) + v
        self.acc = {}

    def totals(self, n):
        self.reset()
        return {k: round(1000 * v / max(1, n), 3) for k, v in self.total.items()}


class Log:
    def __init__(self):
        self.t0 = time.time()

    def __call__(self, msg):
        print(f"[{time.time() - self.t0:7.1f}s] {msg}", flush=True)


def load_world(client, xodr, mc, slab=False, keep_lights=False):
    """The trial's world: a baked level by name, else generated from the OpenDRIVE (without its traffic lights
    unless keep_lights: placement.carla_xodr; SUMO's lights are drawn by signals.py)."""
    if mc.get("carla_level"):   # a baked level (its own roads, buildings and ground): load it by name
        client.set_timeout(600.0)
        try:
            world = client.load_world(mc["carla_level"])
        except RuntimeError as err:     # say which levels this server has (a CARLA town may need AdditionalMaps)
            raise RuntimeError(towns.missing_level(client, mc["carla_level"], err)) from err
        client.set_timeout(60.0)
        return world
    p = mc["carla"]
    params = carla.OpendriveGenerationParameters(
        vertex_distance=p["vertex_distance"], max_road_length=p["max_road_length"], wall_height=p["wall_height"],
        additional_width=p["additional_width"], smooth_junctions=p["smooth_junctions"],
        enable_mesh_visibility=True, enable_pedestrian_navigation=p["pedestrian_navigation"])
    client.set_timeout(600.0)
    water = None
    try:        # the scenery's water bodies: a patched CARLA 1.0 lowers its generated terrain under them
        water = json.loads((Path(xodr).parent / "scenery.json").read_text()).get("water")
    except (OSError, ValueError):
        pass
    text, n = placement.carla_xodr(Path(xodr).read_text(encoding="utf-8"), keep_lights, water)
    if water:
        print(f"[run] OpenDRIVE for CARLA: {len(water)} water bodies as userData (CARLA 1.0 terrain under water)",
              flush=True)
    if n:
        print(f"[run] OpenDRIVE for CARLA: {n} traffic-light signals left out (SUMO's are drawn)", flush=True)
    world = client.generate_opendrive_world(text, params)
    client.set_timeout(60.0)
    if p["ground"]:
        # A 3 km plane (with collision) 0.3 m under the road surface (roads dip a little in places): the
        # OpenDRIVE-only world has no terrain, so a car leaving the road mesh would otherwise fall out of it.
        bp = world.get_blueprint_library().find("static.prop.mesh")
        bounds = [w.transform.location for w in world.get_map().generate_waypoints(50.0)]
        cx = sum(b.x for b in bounds) / len(bounds)
        cy = sum(b.y for b in bounds) / len(bounds)
        # with scenery: a 20 km paving cube whose top is the ground beyond the scenery tiles, out to the horizon (the
        # void beyond a smaller one shows as a black band under a cloudy sky); else a 3 km plane. A packaged CARLA 1.0
        # server may not have cooked the cube ("invalid actor description"): then the plane, then nothing.
        # The map's far landscape ring (farring.py: the real land to the horizon, cooked into the server) replaces both.
        ring = farring.spawn(world, Path(xodr).parent, carla) if slab else 0
        choices = ([("/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube", "20000", -0.45 - 10000.0)]
                   if slab and not ring else [])
        if not ring:
            choices.append(("/Engine/BasicShapes/Plane.Plane", "3000", -0.3))
        for mesh, scale, z in choices:
            bp.set_attribute("mesh_path", mesh)
            bp.set_attribute("scale", scale)
            if world.try_spawn_actor(bp, carla.Transform(carla.Location(x=cx, y=cy, z=z))) is not None:
                break
            print(f"[run] ground {mesh.rsplit('/', 1)[-1]} not on this server; trying the next", flush=True)
    return world


def _local_time(text):
    from datetime import datetime
    if text in (None, "now"):
        return "now"
    text = str(text).strip()
    for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M", "%Y-%m-%d"):
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            pass
    hh, mm = (int(v) for v in text.split(":")[:2])
    return datetime.now().replace(hour=hh, minute=mm, second=0, microsecond=0)


def set_weather(world, sc, latlon=None, cache_dir=None, log=print):
    """Apply the weather; with simulation.time the sun at that place and time, with weather "real" also
    Open-Meteo's weather of that hour. Returns what was applied (written to weather.json)."""
    from datetime import timedelta
    w = sc["weather"]
    info = {"weather": w}
    real = w == "real"
    rec = None
    if isinstance(w, str) and not real:
        weather = getattr(carla.WeatherParameters, w)
    else:
        weather = world.get_weather()
        for k, v in ({} if real else w).items():
            setattr(weather, k, v)
    if (real or sc.get("time")) and latlon:
        when = _local_time(sc.get("time") or "now")
        lat, lon = latlon
        try:
            rec, local, off = realworld.weather_at(lat, lon, when, cache_dir)
        except Exception as err:  # noqa: BLE001 - offline: the sun from the clock alone
            log(f"Open-Meteo failed ({err}); sun position only")
            rec, local, off = None, (when if when != "now" else None), None
        from datetime import datetime, timezone
        if off is not None:
            utc = local - timedelta(seconds=off)
        else:      # no time zone from Open-Meteo: solar time from the longitude
            utc = (local - timedelta(hours=lon / 15.0)) if local is not None else \
                datetime.now(timezone.utc).replace(tzinfo=None)
            local = local or utc + timedelta(hours=lon / 15.0)
        alt, az = realworld.sun_position(lat, lon, utc.replace(tzinfo=timezone.utc))
        fields = realworld.carla_weather_fields(rec or {}, alt, az, float(sc.get("cloud_scale") or 0.6))
        if not real or rec is None:
            fields = {k: fields[k] for k in ("sun_altitude_angle", "sun_azimuth_angle")}
        for k, v in fields.items():
            setattr(weather, k, v)
        info.update(local_time=local.strftime("%Y-%m-%d %H:%M"), sun_altitude=round(alt, 2),
                    sun_azimuth_compass=round(az, 2), open_meteo=rec, carla=fields)
        log(f"{'real weather' if real and rec else 'sun'} for {local:%Y-%m-%d %H:%M} local: sun {alt:.1f} deg up "
            f"at {az:.0f} deg" + (f", clouds {rec['cloud_cover']}%, rain {rec.get('rain')} mm/h, wind "
                                 f"{rec.get('wind_speed_10m')} km/h, {rec.get('temperature_2m')} C"
                                 if real and rec else ""))
        if real and rec and (rec.get("snow_depth") or 0) >= 0.01:
            log(f"{rec['snow_depth'] * 100:.0f} cm of snow on the ground that hour: CARLA cannot show snow")
    if sc.get("sun_altitude") is not None:
        weather.sun_altitude_angle = sc["sun_altitude"]
    world.set_weather(weather)
    info["night"] = weather.sun_altitude_angle < 3.0
    # cars' lights by the Dutch rules: dipped beam and rear lights at night, in fog and in rain (nightlights.py)
    info["lights"] = nightlights.weather_lights(weather, rec if real else None, sc.get("headlights", "auto"))
    return info


def spawn_vehicle(world, geo, p, log):
    cmap = world.get_map()
    bp = compat.find(world, p.get("vehicle", "vehicle.lincoln.mkz_2020"), "passenger")
    bp.set_attribute("role_name", p["role"])
    if p.get("color") and bp.has_attribute("color"):     # "r,g,b": a taxi's livery, a company car's paint
        bp.set_attribute("color", str(p["color"]))
    plates.tag(bp)
    sp = p.get("spawn") or {}
    if "lat" in sp:
        x, y = geo.to_carla(sp["lat"], sp["lon"])
        wp = nearest_waypoint(cmap, x, y, carla.LaneType.Driving)
        log(f"{p['role']}: asked for {sp['lat']:.5f},{sp['lon']:.5f} -> lane {wp.road_id}/{wp.lane_id} "
            f"{wp.transform.location.distance(carla.Location(x=x, y=y, z=wp.transform.location.z)):.1f} m away")
        candidates = [wp.transform]
        if "heading" in sp:
            # the driving lane across the road facing the heading best, the nearest of those (the opposite
            # direction lies to the right on a left-hand map, and a road can have several lanes each way)
            yaw = heading_to_yaw(sp["heading"])
            here = carla.Location(x=x, y=y, z=wp.transform.location.z)
            lanes = compat.road_lanes(wp, carla.LaneType.Driving) or [wp]
            wp = min(lanes, key=lambda w: (round(abs((w.transform.rotation.yaw - yaw + 180) % 360 - 180) / 45.0),
                                           w.transform.location.distance(here)))
            candidates = [wp.transform]
        candidates += [w.transform for w in wp.next(15.0) + wp.previous(15.0)]
    else:
        # CARLA's spawn points of an OpenDRIVE-generated world do not always face along the lane, so start on
        # a random driving-lane waypoint (outside junctions), which does.
        wps = [w for w in cmap.generate_waypoints(20.0)
               if w.lane_type == carla.LaneType.Driving and not w.is_junction]
        random.Random(p.get("seed", 0)).shuffle(wps)
        candidates = [w.transform for w in wps[:50]]
    for tf in candidates:
        tf.location.z += 0.6
        actor = world.try_spawn_actor(bp, tf)
        if actor:
            return actor
    raise RuntimeError(f"could not spawn {p['role']}")


def spawn_pedestrian(world, geo, p):
    bp = compat.find(world, p.get("blueprint", "walker.pedestrian.0001"))
    bp.set_attribute("role_name", p["role"])
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    sp = p["spawn"]
    x, y = geo.to_carla(sp["lat"], sp["lon"])
    yaw = heading_to_yaw(sp.get("heading", 0.0))
    loc = ground(world.get_map(), x, y) + carla.Location(z=1.2)
    return world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=yaw)))


def to_carla_path(geo, latlons):
    return [list(geo.to_carla(lat, lon)) for lat, lon in latlons]


class Participant:
    def __init__(self, p):
        self.cfg, self.role, self.kind = p, p["role"], p.get("kind", "driver")
        self.station = bool(p.get("station", False))
        self.actor = self.agent = self.input = self.bot = None
        self.ctl, self.light, self.light_key = None, None, None
        self.ehmi_schedule = []

    def record(self):
        return {"actor": self.actor, "role": self.role, "kind": "participant", "ctl": self.ctl,
                "light": self.light, "ehmi": self.agent.ehmi if self.agent else ""}


def welcome_for(part, geo, tc):
    """What a station needs to know about its role (sent on hello)."""
    bot = dict(part.cfg.get("bot") or {})
    if "path" in bot and part.kind != "drone":
        bot["path"] = to_carla_path(geo, bot["path"])
    if "path" in bot and part.kind == "drone":      # [lat, lon, height]: the host flies a station's bot drone
        bot["path"] = [list(p) for p in bot["path"]]
    out = {"kind": part.kind, "view": part.cfg.get("view"), "bot": bot,
           "ehmi_states": list((tc.get("ehmi") or {}).get("states", DEFAULT_EHMI)),
           "step": tc["simulation"]["step"], "experiment": tc["name"],
           "camera_profile": tc["simulation"].get("camera_profile", "Dashcam"),
           "capture": tc["simulation"].get("capture"),
           "camera_attributes": dict((tc.get("film") or {}).get("attributes") or {})}   # demo videos (film.py)
    # sound, in-car displays, haptics and designed eHMIs (stationux.py), when the experiment sets them
    out.update(stationux.welcome_extras(tc, part.cfg, part.kind, Path(tc["_path"]).parent,
                                        default_states=list(DEFAULT_EHMI), default_ehmi=DEFAULT_EHMI))
    out.update(multigpu.welcome(tc["server"], part.role))     # secondaries: relay and camera placement
    if part.cfg.get("view") == "vr" or part.cfg.get("vr"):     # a VR station (vr.py): the experiment's and own `vr`
        out["vr"] = config._merge(tc.get("vr") or {}, part.cfg.get("vr") or {})
    if part.kind == "drone":      # gamepad / RC axes and buttons for the station's stick input (droneinput.py)
        out["drone_input"] = {k: part.cfg[k] for k in ("axes", "joystick", "deadband", "expo", "buttons")
                              if k in part.cfg}
    return out


def signal_region(tc, mdir, major):
    """The region whose signal heads the drawn traffic lights use (signals.py): CARLA 1.0 with the street content
    (streetlook.py) in the Netherlands, the UK or Japan, unless map.scenery.street_props is off; else None."""
    if major < 1 or (tc["map"].get("scenery") or {}).get("street_props", "auto") in (False, "off", None):
        return None
    try:
        from . import crowdpeds, streetlook
        bbox = tc["map"].get("bbox") or json.loads((mdir / "meta.json").read_text())["bbox"]
        iso3 = crowdpeds.locate(tc, bbox, mdir / "real" / "crowdpeds_country.json")[0]
        return streetlook.region(iso3)
    except Exception:  # noqa: BLE001 - country unknown: CARLA's heads
        return None


def run_trial(cfg, trial_idx, args, control=None, dashboard=None):
    log = Log()
    trial = cfg["trials"][trial_idx]
    tc = config.trial_config(cfg, trial)
    if getattr(args, "local", False):    # also a trial's or condition's own participants list (main sees only the top's)
        _make_local(tc["participants"])
    if getattr(args, "station_roles", None) is not None:    # the session dashboard says who has a station
        _set_stations(tc["participants"], args.station_roles)
    for p in tc["participants"]:
        if args.input and not p.get("station") and p.get("kind", "driver") == "driver":
            p["input"] = args.input
        if args.headless:
            p["window"] = False
            if not args.input and not p.get("station") and p.get("input", "keyboard") in ("keyboard", "wheel"):
                p["input"] = "autopilot"     # nobody to steer without a window
        if args.view:
            p["view"] = args.view
    mdir = config.map_dir(tc)
    if not (mdir / "meta.json").exists():
        mapbuild.build(tc)
    geo = Geo(mdir / "geo.net.xml")
    if not tc["map"].get("bbox"):   # a map built from an OpenDRIVE file: its box is in meta.json
        tc["map"]["bbox"] = json.loads((mdir / "meta.json").read_text())["bbox"]
    plates.setup(tc, mdir, log)                # country number plates on the vehicles (plates.py)
    walkerlooks.setup(tc, mdir, log)           # walkers that look like the people of the place (walkerlooks.py)
    look.resolve_auto(tc, log)                 # camera_profile "auto": the place's fitted look (look.py)
    town = tc["map"].get("carla_town")       # a CARLA standard map (towns.py)
    if towns.resolve_xy(tc, geo):            # points given as CARLA {x, y}
        log("points given as CARLA x, y converted to lat, lon")
    step = tc["simulation"]["step"]
    duration = float(trial.get("duration_s", 300))
    ehmi_states = (tc.get("ehmi") or {}).get("states", DEFAULT_EHMI)
    ehmi_slots = tuple((tc.get("ehmi") or {}).get("slots", ["front"]))
    ehmi_designed = ehmi_design.drawers(ehmi_design.for_agents(tc, Path(tc["_path"]).parent, log))
    ehmi_render = (tc.get("ehmi") or {}).get("render", "world")     # designed eHMIs: world | station | both
    # an AV passenger's own car: the panels it would see from behind are drawn by the stations (ehmi_inside.py)
    ehmi_cabin = ehmi_inside.host_plan(tc, ehmi_design.for_agents(tc, Path(tc["_path"]).parent, lambda m: None),
                                       lambda a: a.get("ehmi_slots") or ehmi_slots)
    for aid, what in ehmi_cabin.items():
        if "placement" in what and aid in ehmi_designed:
            d = ehmi_designed[aid]
            world_pl = ehmi_inside.split(d.placements)[0]
            ehmi_designed[aid] = ehmi_design.DesignedEhmi(d.design, world_pl, d.offset) if world_pl else None
        log(f"eHMI of {aid}: its {', '.join(what.get('placement') or what.get('slots'))} drawn by the stations "
            f"(ehmi.inside: {ehmi_inside.settings(tc)[0]} for its passenger)")
    coupling = tc.get("coupling") or {}

    stamp = time.strftime("%Y%m%d-%H%M%S")
    # with a participant number: <time>_P07_trial<i>, i the trial's number in the file (study.plan)
    who = f"_P{trial['_participant']:02d}" if trial.get("_participant") else ""
    out = Path(tc["_root"]) / tc["logging"]["dir"] / tc["name"] / f"{stamp}{who}_trial{trial.get('_index', trial_idx)}"
    out.mkdir(parents=True, exist_ok=True)
    plan_info = {k[1:]: trial[k] for k in ("_participant", "_position", "_index", "_block", "_condition") if k in trial}
    (out / "config.yaml").write_text(yaml.safe_dump({**{k: v for k, v in tc.items() if not k.startswith("_")},
                                                     "study": plan_info}, sort_keys=False, allow_unicode=True))
    log(f"trial {trial_idx} -> {out}")

    cfg_file = traffic.prepare(tc, mdir, out / "sumo", duration, log)
    log("SUMO demand ready")
    parts = [Participant(p) for p in tc["participants"]]
    by_role = {p.role: p for p in parts}
    state = {"state": "loading", "trial": trial_idx}
    tools = HostStudy(tc, trial, parts, out, log, control, dashboard, cfg["trials"])   # responses, take-overs, dashboard
    if control is not None:
        control.on_hello = lambda role: (
            {**welcome_for(by_role[role], geo, tc), **tools.welcome(by_role[role]), **state,
             "actor_id": _actor_id(by_role.get(role))}
            if role in by_role else {"error": f"no participant with role {role}"})

    server = Server(tc["server"], log)
    client = server.start()
    look.DEBUG_COMPRESS = compat.server_major(client) == 0     # debug colours (traffic lights, eHMIs): look.py
    sync = view = recorder = world = logger = drone_world = ship_world = film = host_audio = photo = navw = city = react = None
    own_logs = []
    collisions, agents = [], []
    summary = {"trial": trial_idx, "ticks": 0}
    try:
        # scenery.json names the meshes of one CARLA version's catalogue: rebuilt first when made for the other
        has_scenery = tc["map"].get("scenery", {}).get("enabled", True) and \
            mapbuild.scenery_for_server(tc, mdir, compat.server_major(client), log)
        if tc["map"].get("standin"):   # the roads around the start only, generated in seconds (standin.py)
            from .standin import standin_xodr
            world = load_world(client, standin_xodr(mdir, tc, log), dict(tc["map"], carla_level=None),
                               slab=has_scenery, keep_lights=tc["simulation"]["tls_manager"] == "carla")
        else:
            world = load_world(client, mdir / "map.xodr", tc["map"], slab=has_scenery,
                               keep_lights=tc["simulation"]["tls_manager"] == "carla")
        if tc["map"].get("carla_level"):     # a baked level's OpenDRIVE fetched once, not per get_map() (compat.py)
            compat.cache_maps(carla)
        if not tc["map"].get("standin"):
            # a baked level: is map.xodr (SUMO's net, the scenery, the paving) the level's own OpenDRIVE?
            # stops if not
            levelxodr.check(world, mdir, tc, log)
        if town:      # the town's buildings and trees from its environment objects (sound, drones)
            towns.environment(world, mdir, client.get_server_version(), log)
        wx = set_weather(world, tc["simulation"], config.place(tc), mdir / "real", log)
        if town:      # the town's own street and building lights at night
            towns.lights(world, wx["night"], tc["simulation"].get("town_lights", "auto"), log)
        (out / "weather.json").write_text(json.dumps(wx, indent=2))
        tc["_sound_env"] = soundrec.sound_env(wx, mdir, world.get_weather(),
                                              (tc.get("sound") or {}).get("weather")).as_dict()   # stations' sound
        BridgeHelper.night = wx["lights"]["low_beam"]         # SUMO cars: dipped beam and rear lights
        BridgeHelper.fog = wx["lights"]["fog_rear"]
        if wx["lights"]["low_beam"]:
            log(f"cars drive with dipped beam and rear lights ({wx['lights']['why']})"
                + ("; rear fog lights on" if wx["lights"]["fog_rear"] else ""))
        photo = photoreal.attach(world, tc, wx["night"], log)    # map.photoreal: Google's 3D tiles (photoreal.py)
        look.ATTRIBUTION = photoreal.attribution_text(photo.s) if photo else None
        if tc["server"].get("offscreen", True):
            # nobody sees the spectator of an off-screen server, but the server still renders its view: park it
            # high up, looking at the sky, so it costs nothing (it rendered the whole city's scenery otherwise)
            world.get_spectator().set_transform(carla.Transform(carla.Location(z=3000.0), carla.Rotation(pitch=90.0)))
        log(f"world ready: {len(world.get_map().get_spawn_points())} spawn points")
        try:        # CARLA's stock (US) sign actors -> the place's own signs (localsigns.py)
            from . import localsigns
            summary["stock_signs"] = localsigns.replace(client, world, plates.iso3(), log,
                                                        tc["simulation"].get("stock_signs", "auto"))
        except Exception as err:  # noqa: BLE001 - the trial runs with the server's signs
            log(f"signs: stock signs left as they are ({type(err).__name__}: {err})")
        level_stream = bool(tc["map"].get("carla_level"))

        from .bridge import pipeline
        sumo_ratio = pipeline.ratio_for(tc["simulation"].get("sumo_step"), step)   # CARLA ticks per SUMO step
        sumo = SumoSimulation(str(cfg_file), step * sumo_ratio)
        carla_sim = CarlaSimulation(tc["server"]["host"], tc["server"]["port"], step)
        # drawings, traffic-light states and the spectator on a connection of their own (see side.py)
        side = Side(tc["server"]["host"], tc["server"]["port"])
        carla_sim.use_side(side.world)
        focus_actors = []   # SUMO actors appear in CARLA only near these (the participants)

        def focus_points():
            """The participants and where the pilots' drone cameras look on the ground (drones.look_points)."""
            pts = [a.get_location() for a in focus_actors]
            if drone_world is not None:
                pts += drone_world.look_points()
            return pts
        sync = SimulationSynchronization(
            sumo, carla_sim, tc["simulation"]["tls_manager"], sync_vehicle_lights=True,
            sync_vehicle_color=True,     # each SUMO car's colour (traffic.py: from the vehicle register) on its CARLA car
            tm_port=tc["server"]["tm_port"] if tc["simulation"].get("traffic_manager") else None,
            radius=tc["simulation"]["lod_radius_m"],
            animate_radius=tc["simulation"].get("animate_radius_m", 0.0),   # wheels roll, cyclists pedal (animate.py)
            smooth_paths=tc["simulation"].get("smooth_paths", True),        # no swerves at SUMO's kinks (animate.py)
            driver_mix_seed=(tc["traffic"].get("seed", 42) if tc["traffic"].get("driver_mix", True) else None),
            red_running=drivermix.red_running(tc),
            depart_hide_m=tc["simulation"].get("depart_hide_m", 60.0),   # no car appears out of nowhere nearby
            focus=lambda: focus_points() or None)
        if town:      # the town's traffic-light actors, run by SUMO through the bridge
            both = sync.sumo.traffic_light_ids & sync.carla.traffic_light_ids
            summary["town_lights"] = {"carla": len(sync.carla.traffic_light_ids), "sumo": len(sync.sumo.traffic_light_ids),
                                      "synced": len(both)}
            log(f"traffic lights: {len(both)} of the town's {len(sync.carla.traffic_light_ids)} signals run by SUMO")
        # The bridge assumes CARLA = SUMO - netOffset, true for CARLA's own towns. Our OpenDRIVE carries a UTM
        # georeference, so map.net.xml's netOffset is the whole UTM shift (~-671 km, -5701 km) and every SUMO
        # car would land thousands of km away (and moving the participant there crashes SUMO). The real shift
        # is the difference between map.net.xml's and geo.net.xml's offsets (CARLA x,y = geo.net x,-y).
        mo, go = sumo.net.getLocationOffset(), geo.net.getLocationOffset()
        BridgeHelper.offset = (mo[0] - go[0], mo[1] - go[1])
        # stacked roads: where SUMO's lane heights are CARLA's, a car off its lane's height is put back (levels.py)
        carla_sim._cmap = world.get_map()
        carla_sim.check_level = levels.check_heights(carla_sim._cmap, sumo.net, BridgeHelper.offset, log=log)
        from . import stoplines     # the stop lines before crosswalks in the net SUMO runs (traffic.prepare)
        signals = Signals(world, sumo.net, BridgeHelper.offset, step=step, debug=side.debug,
                          style=(tc["simulation"].get("traffic_lights") or {}).get("style", "head"),
                          lefthand=bool(tc["map"].get("lefthand")), log=log,
                          surface=placement.ground_surface(mdir / "scenery.json") if has_scenery else None,
                          region=signal_region(tc, mdir, compat.server_major(client)),
                          stop_offsets=stoplines.read_offsets(getattr(sumo, "net_file", None)))
        if signals.style != "none":
            log(f"traffic lights: {len(signals.heads)} signal heads from SUMO ({signals.style}"
                f"{', meshes' if signals.meshes else ''}) drawn near participants")
        carla_sim._kinds.update({a.id: False for a in signals.actors})     # static: never looked up per tick
        if (tc["simulation"].get("traffic_lights") or {}).get("log"):   # drawn states, for tools/signal_heads.py
            import traci
            signals.start_log(out / "signal_heads.csv", traci.simulation.getTime)
        from . import signaldata      # real phases replayed exactly, or a live feed, through TraCI (signaldata.py)
        sig_driver = signaldata.driver(out / "sumo", log)
        try:
            summary["signals"] = json.loads((out / "sumo" / "signals.json").read_text())
        except (OSError, ValueError):
            pass
        # city-sized maps: SUMO every few ticks, in a thread, asked only about what is near (bridge/pipeline.py)
        sync.configure_pipeline(sumo_ratio, tc["simulation"].get("sumo_thread"), tc["simulation"].get("sumo_near"),
                                tc["simulation"].get("sumo_near_m"))
        if sync.pipelined:
            sync.hooks.append(lambda: signals.prefetch([a.get_location() for a in focus_actors]))
            log(f"SUMO: a step of {step * sumo_ratio:g} s every {sumo_ratio} tick(s), cars interpolated between"
                f"{'; in a thread while CARLA ticks' if sync.threaded else ''}"
                f"{'; asked only about vehicles and lights near the participants' if sync.near else ''}")
        world.tick()

        # AI agents (scripted cars and walkers); an AV passenger rides in one of them
        planner = SafeGlobalRoutePlanner(world.get_map(), 2.0) if tc.get("agents") else None
        for a in tc.get("agents") or []:
            if a.get("kind", "car") == "car":
                ag = ScriptedCar(world, geo, a, planner)
                log(f"agent {ag.id}: {ag.vehicle.type_id}, route {ag.route_length():.0f} m, eHMI {ag.ehmi_mode}")
            else:
                ag = ScriptedWalker(world, geo, a)
                log(f"agent {ag.id}: walker, {len(a['path'])} points from t = {a.get('start_s', 0)} s")
            agents.append(ag)
        agent_by_id = {a.id: a for a in agents}
        # drones: participants of kind drone and the AI drones of `drones.missions` (drones.py)
        if any(p.kind == "drone" for p in parts) or (tc.get("drones") or {}).get("missions"):
            drone_world = DroneWorld(world, client, geo, tc, mdir, out, log, side.debug)
        if (tc.get("ships") or {}).get("fleet"):      # AI ships (ships.py)
            ship_world = ShipWorld(world, client, geo, tc, mdir, out, log)

        def watch_collisions(actor, role):
            col = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                    carla.Transform(), attach_to=actor)
            col.listen(lambda e: collisions.append((e.timestamp, e.frame, role, e.other_actor.id,
                                                    e.other_actor.type_id)))

        for i, part in enumerate(parts):
            p = part.cfg
            if part.kind == "driver":   # an automated driver's car starts on its route
                part.actor = tools.spawn(world, geo, part, planner) or spawn_vehicle(world, geo, p, log)
                if not part.station:
                    kind = p.get("input", "keyboard")
                    if kind == "autopilot" and (p.get("autopilot") or tc["simulation"].get("autopilot")) == "sumo" \
                            and not (getattr(tools, "aspec", None) or {}).get(part.role):   # take-over bots: BasicAgent
                        kind = "sumo"           # SUMO drives it, with the traffic's rules (sumodriver.py)
                    part.input = inputs.make(kind, part.actor)
            elif part.kind == "pedestrian":
                part.actor = spawn_pedestrian(world, geo, p)
                if not part.station and (p.get("input") == "bot" or p.get("bot")):
                    bot = p.get("bot") or {}
                    pts = [ground(world.get_map(), x, y) for x, y in to_carla_path(geo, bot.get("path", []))]
                    part.bot = PathWalker(part.actor, pts, float(bot.get("speed", 1.4)), float(bot.get("start_s", 0)))
            elif part.kind == "drone":
                part.actor = drone_world.add_pilot(part)
            elif part.kind == "av_passenger":
                part.agent = agent_by_id[p["agent"]]
                part.actor = part.agent.vehicle
                if part.agent.ehmi_mode == "off":
                    part.agent.ehmi_mode = "manual"
                if not part.station:
                    part.ehmi_schedule = sorted((p.get("bot") or {}).get("ehmi", []))
            else:
                raise ValueError(f"unknown participant kind {part.kind}")
            focus_actors.append(part.actor)
            if part.kind not in ("av_passenger", "drone"):      # a drone's crashes: drones.py
                watch_collisions(part.actor, part.role)
            if not part.station and i == 0 and part.kind == "driver":
                window = p.get("window", part.input.needs_display)
                if window or tc["logging"]["frames_every_s"] > 0:
                    view = View(world, part.actor, p.get("view", "driver"), window=window, save_dir=out,
                                save_every_s=tc["logging"]["frames_every_s"],
                                profile=tc["simulation"].get("camera_profile", "Dashcam"),
                                capture=tc["simulation"].get("capture"),
                                lefthand=bool((tc.get("map") or {}).get("lefthand")))
            if not part.station and part.kind == "drone":
                window = p.get("window", p.get("input", "keyboard") in ("keyboard", "gamepad", "rc"))
                drone_world.local_input(part.drone, window and (i == 0))
                if i == 0 and (window or tc["logging"]["frames_every_s"] > 0):
                    view = DroneView(drone_world, part.drone, window=window, save_dir=out,
                                     save_every_s=tc["logging"]["frames_every_s"],
                                     profile=tc["simulation"].get("camera_profile", "Dashcam"),
                                     capture=tc["simulation"].get("capture"))
            log(f"{part.role}: {part.kind} {part.actor.type_id} "
                f"{'at a station' if part.station else 'on the host (' + str(p.get('input', 'bot')) + ')'}")
        for ag in agents:
            if isinstance(ag, ScriptedCar):
                watch_collisions(ag.vehicle, ag.id)

        if wx["lights"]["low_beam"]:      # headlights on after dark, in fog and rain (SUMO cars: BridgeHelper.night)
            lights = nightlights.light_bits(wx["lights"], carla)
            for a in [p.actor for p in parts if p.actor.type_id.startswith("vehicle.")] + \
                     [ag.vehicle for ag in agents if isinstance(ag, ScriptedCar)]:
                a.set_light_state(carla.VehicleLightState(int(a.get_light_state()) | int(lights)))
        world.tick()      # actors have their spawn locations only after a tick
        scenery = parked = None
        if has_scenery:
            t_s = time.time()
            seas, seas_info = season.for_trial(tc, mdir, world, log)   # leaf state; lamps lit on 1.0 at night
            (out / "season.json").write_text(json.dumps(seas_info, indent=2))
            scenery = Scenery(client, world, mdir, tc["simulation"].get("scenery") or {}, log,
                              on_spawn=lambda ids: carla_sim._kinds.update({i: False for i in ids}), season=seas)
            from . import gbfs   # a trial at time "now": today's shared bikes and scooters
            gbfs.refresh_scenery(scenery, tc, mdir, geo, world.get_map(), log)
            scenery.update([a.get_location() for a in focus_actors])
            try:        # measured / register / inferred / generated (provenance.py), for the analysis
                from . import provenance
                pv = scenery.provenance([(a.get_location().x, a.get_location().y) for a in focus_actors])
                if tc["map"].get("carla_level"):
                    bb = tc["map"].get("bbox") or [0, 0, 0, 0]
                    cc = (tc.get("city") or {}).get("country_code") or provenance.country_at(
                        (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
                    pv["twin_layers"] = provenance.twin_layers(cc)
                (out / "provenance.json").write_text(json.dumps(pv, indent=1))
                log("provenance: " + ", ".join(f"{k} {v}" for k, v in sorted(pv["by_tag"].items()))
                    + f"; generated within 50 m of the participants {pv['near']['generated_share']:.0%}")
            except Exception as err:  # noqa: BLE001 - the log is a convenience, the trial runs without it
                log(f"provenance: not written ({err})")
            # a server that omits static props (upstream since f5fb36aed, or our tree with the flag) needs a client
            # wheel from the same tree, or world.get_actors() misses the scenery: checked once
            staticactors.check_after_spawn(world, [i for ids in scenery.loaded.values() for i in ids][:20],
                                           tc["server"], log)
            log(f"scenery: {scenery.n} meshes within {scenery.radius:.0f} m of the participants "
                f"({time.time() - t_s:.1f} s)" + (f"; {scenery.n_lights} lamps with light within "
                                                  f"{scenery.lights_radius:.0f} m" if scenery.light_bp else "")
                + (f", {scenery.n_lamp_specs} with their register colour and lumens" if scenery.n_lamp_specs else "")
                + scenery.failed_summary())
            if not town or towns.own_parked(world, tc, mdir, log):     # a town may keep its own parked cars
                parked = parking.scenery(client, world, mdir, tc, log,
                                         on_spawn=lambda ids: carla_sim._kinds.update({i: False for i in ids}))
            if parked is not None:
                parked.update(focus_points())
                log(f"parked: {parked.n} cars within {parked.radius:.0f} m of the participants")
        for part in parts:   # BasicAgent planned from (0, 0, 0) before the first tick: plan from the spawn lane
            if hasattr(part.input, "attach"):     # the SUMO driver: its route from the spawn lane (sumodriver.py)
                try:
                    part.input.attach(sync, sumo.net, geo, part.cfg, BridgeHelper.offset, log)
                except RuntimeError as err:       # e.g. a spawn on a lane SUMO's net has not got: CARLA's autopilot
                    log(f"{err}; CARLA's BasicAgent drives instead")
                    part.input = inputs.make("autopilot", part.actor)
            if hasattr(part.input, "restart"):
                part.input.restart(world.get_map().get_waypoint(part.actor.get_location()))
        walkers_all = [p.actor for p in parts if p.kind == "pedestrian"] + \
                      [a.walker for a in agents if isinstance(a, ScriptedWalker)]
        # the pedestrian crowd: calibrated with CROWD for the map's country (crowdpeds.py), or with behaviour off
        # ai.Crowd beside the roads; its walkers join walkers_now (cars yield to them) only while on the road
        livingcity.apply_crowd_default(tc, livingcity.settings(tc), log)   # a living city has a crowd
        crowd_centres = [p.actor.get_location() for p in parts]
        if drone_world is not None:   # a bot pilot's crowd covers its flight and the ground its camera films
            fly = drone_world.path_points()
            if fly:
                crowd_centres += [carla.Location(x, y, 0.0) for x, y in fly]
                log(f"crowd: also along the drone flights and their camera views ({len(fly)} points)")
        wcfg = crowdwindow.settings(tc)     # a rolling window keeps the crowd around the drivers (crowdwindow.py)
        reach = crowdpeds.route_reach(duration, wcfg["route_max_m"] if wcfg["enabled"] else
                                      crowdpeds.ROUTE_AHEAD_MAX_M)
        ahead = [p.input.route_xy(reach) for p in parts if callable(getattr(p.input, "route_xy", None))]
        if any(ahead):      # a driver leaves the start: its crowd's kerbs also cover the route it will drive
            extra = crowdpeds.route_centres(ahead)
            crowd_centres += [carla.Location(x, y, 0.0) for x, y in extra]
            log(f"crowd: also along the drivers' routes ahead ({len(extra)} points, {reach:.0f} m at most)")
        crowd = crowdpeds.make_crowd(world, geo, tc, mdir, crowd_centres, wx["night"], duration, log,
                                     parts=parts if wcfg["enabled"] else None)
        wai = (tc["traffic"].get("walkers") or {}).get("ai") or {}
        if int(wai.get("count") or 0) > 0 and tc["map"].get("carla_level"):   # CARLA's AI walkers (navigation mesh)
            navw = towns.NavWalkers(client, world, wai, [p.actor.get_location() for p in parts], log)
            world.tick()
            navw.start()
            world.tick()
            navw.go()
        cmap = world.get_map()
        sumo_yield = SumoYield()
        if crowd is not None:     # SUMO's speeds: CARLA reads 0 m/s for a SUMO vehicle placed with physics off
            crowd.velocities = sync.velocity
            crowd.poses = sync.poses
            if hasattr(crowd, "signals"):     # cross at signals with the light (pedsignals.py)
                from .pedsignals import PedSignals
                crowd.signals = PedSignals.from_sumo(sumo.net, BridgeHelper.offset)
            if hasattr(crowd, "start_crossings"):     # crowds at busy and scramble crossings (scramble.py)
                crowd.start_crossings(
                    tc, mdir, geo, [(c.x, c.y) for c in crowd_centres], queue=carla_sim.queue,
                    on_spawn=lambda ids: carla_sim._kinds.update({i: False for i in ids}),   # bicycles: no SUMO twin
                    viewers=lambda: [r.camera for r in (film.rigs if film is not None else [])     # what is
                                     if getattr(r, "camera", None) is not None] + list(focus_actors),   # in view
                    log=log)
        # Controls and light states of the host's participants, agents and crowd go with the tick's batch (one
        # call per tick; on CARLA 1.0 several separate ones stall the tick, see side.py).
        queue = carla_sim.queue
        walker_cmd = carla.command.ApplyWalkerControl
        for part in parts:
            if hasattr(part.input, "send"):
                part.input.send = queue
            if part.bot is not None:
                part.bot.apply = lambda c, i=part.actor.id: queue(walker_cmd(i, c))
        for ag in agents:
            if isinstance(ag, ScriptedCar):
                bl = nightlights.BrakeLight(ag.vehicle, queue)      # its brake lights follow its control
                ag.apply = lambda c, i=ag.vehicle.id, bl=bl: (queue(carla.command.ApplyVehicleControl(i, c)),
                                                             bl.update(c, carla))
            else:
                ag.follow.apply = lambda c, i=ag.walker.id: queue(walker_cmd(i, c))
        for f in getattr(crowd, "follow", None) or []:        # ai.Crowd (behaviour off)
            f.apply = lambda c, i=f.walker.id: queue(walker_cmd(i, c))
        if crowd is not None and hasattr(crowd, "send"):      # crowdpeds.CrowdSim
            crowd.send = lambda actor, c: queue(walker_cmd(actor.id, c))
        # the living city (livingcity.py): activities, kerbside life and reactions to the participants, seeded, logged
        city = react = None
        try:
            city = livingcity.make(tc, world, client, geo, mdir, crowd if hasattr(crowd, "add") else None, sync,
                                   carla_sim, parked, parts, duration, wx, log)
            if city is not None:
                # what anyone sees: the participants, free cameras (focus_actors) and the film's cameras; the city's
                # walkers and cars appear and vanish only out of their sight (clip QA)
                city.viewers = lambda: list(focus_actors) + [r.camera for r in (film.rigs if film is not None else [])
                                                             if getattr(r, "camera", None) is not None]
                if city.crowd is not None:
                    city.crowd.hidden = city.hidden
                log(f"living city: {city.start()}")
                react = reactions.make(city, parts, sync, city.crowd, cmap, sumo_yield)
        except Exception as err:  # noqa: BLE001 - the trial without it rather than no trial
            log(f"living city: not started ({type(err).__name__}: {err})")
            city = react = None

        if drone_world is not None:      # obstacles, wind and AI missions once the participants stand
            drone_world.queue = queue
            drone_world.start([a.get_location() for a in focus_actors])
        if ship_world is not None:
            ship_world.queue = queue
            ship_world.start()
        if tc["logging"]["carla_recorder"]:
            recorder = client.start_recorder(f"coupledcarla_{tc['name']}_{stamp}_trial{trial_idx}.log", True)
        logger = TrialLogger(out, geo, tc["logging"], step)
        own_logs = [x for x in [p.input for p in parts] + [crowd] if hasattr(x, "open_log")]
        for x in own_logs:      # inputs and the crowd with logs of their own (myodriver.py, embodiedped.py)
            x.open_log(out)
        logger.velocities = sync.velocity        # SUMO vehicles' speeds in frames.csv (see above)
        if drone_world is not None:              # AI actors near the drones and where their cameras look, too
            logger.extra_centres = lambda: [a.get_location() for a in focus_actors] + drone_world.look_points()
        fc = tc.get("film") or {}
        if fc.get("camera") or fc.get("cameras"):       # demo-video cameras on a sim-time grid (tools/make_videos.py)
            from .film import Film
            fc = dict(fc)
            fc["camera_profile"], why = look.film_profile(tc)     # CARLA 1.0; 0.9.x ignores it
            log(f"film: camera profile {fc['camera_profile']} ({why})"
                + ("; it must be installed on the server (tools/install_profiles.py)" if look.ours(fc["camera_profile"])
                   else ""))
            names = {**{p.role: p.actor for p in parts},
                     **{a.id: getattr(a, "vehicle", None) or getattr(a, "walker", None) for a in agents}}
            film = Film(fc, world, geo, out, log, names.get, step, queue, drone_world,
                        focus=focus_actors)
            film.hold(parts)
        sc_ = tc.get("sound") or {}
        if parts and (getattr(args, "record_audio", False) or sc_.get("record")):
            # the first participant's soundscape in simulation time, into audio.wav (soundrec.py)
            try:
                host_audio = soundrec.HostAudio(tc, out, parts[0], mdir, wx, step, carla_sim.world, log, agents=[
                    a for a in agents if isinstance(a, ScriptedCar)], drone_world=drone_world, signals=signals,
                    net=sumo.net)
            except Exception as err:  # noqa: BLE001 - a trial without its audio rather than no trial
                log(f"sound: no audio recording ({type(err).__name__}: {err})")
                host_audio = None

        # stations: wait for them, then tell each its actor and start
        station_roles = [p.role for p in parts if p.station]
        if station_roles:
            if control is None:
                raise RuntimeError("participants with station: true need the control channel (coupling.port)")
            missing = control.wait_for(station_roles, float(coupling.get("wait_for_stations_s", 600)))
            if missing:
                log(f"starting without stations: {', '.join(sorted(missing))}")
                logger.event(0.0, 0, "stations_missing", "", ",".join(sorted(missing)))
        # a secondary that connected after the world settings switched the primary to 1/20 s (multigpu.py)
        guard = multigpu.guard_settings(tc["server"], step, log)
        if guard is not None:
            guard(world, force=True)
        state["agents"] = {ag.id: ag.vehicle.id for ag in agents if isinstance(ag, ScriptedCar)}
        if not tools.ready():          # the dashboard ended the trial (or the session) before it started
            duration = 0.0
        tools.started()
        if control is not None:      # this trial's settings, before the stations start (see ControlServer.rewelcome)
            control.rewelcome(station_roles)
        state.update(state="running")
        for part in parts:
            if control is not None:
                control.send(part.role, {"type": "state", **state, "actor_id": part.actor.id})
        logger.event(0.0, 0, "trial_start", "", json.dumps({"trial": trial_idx, "stations": station_roles,
                                                            "agents": [a.id for a in agents], **plan_info}))

        types = {}
        # with a session dashboard: positions for its live map, and its eHMI and weather commands (livemap.py)
        live = livemap.LiveFeed(control, geo, tc, mdir, parts, agents, signals, step, log, tools) \
            if hasattr(control, "live") else None
        prof = {"sync": 0.0, "log": 0.0, "ticks0": 0}
        laps = Laps()
        sync.laps = laps
        profile = bool(tc["simulation"].get("profile"))
        was_off, stuck_since, station_stats, station_gaze = {}, {}, {}, {}
        gaze = MutualGaze(logger, **(tc.get("eye_contact") or {}))
        sync_at = sorted(float(x) for x in coupling.get("visual_sync_at_s", []))
        t_wall0, sim0, next_status, seen_col, t = time.time(), None, 0.0, 0, 0.0
        tools.bind(logger, queue, station_stats, lambda at: sync_at.insert(0, at), view)
        # snapshots from the connection that ticks (carla_sim's): another connection's get_snapshot() right after
        # the tick was still the frame before in 58 % of ticks (lin4080, 0.9.16), so frames.csv repeated frames
        snap = carla_sim.world.get_snapshot()
        while True:
            tools.hold()                  # paused on the dashboard
            t_tick = time.time()
            if guard is not None:
                guard(world)              # every 5 s
            if photo is not None and photo.state == "spawned" and t >= 2.0:
                photo.check()             # gone: the server had no key, a cap or no level data (its log says)
                (out / "photoreal.json").write_text(json.dumps(photo.info(), indent=2))
            laps.start()
            events = view.events() if view else []
            for part in parts:            # host-local participants
                if part.station:
                    continue
                if part.input is not None:
                    ctl = part.input.control(part.actor, events)
                    if tools.intercept(part, ctl, t, events):    # the automation drives (take-over watched)
                        continue
                    if ctl is not None:
                        queue(carla.command.ApplyVehicleControl(part.actor.id, ctl))
                        part.ctl = ctl
                        if not hasattr(part.input, "_send_lights"):      # the bot driver: brake lights from here
                            if getattr(part, "brake_light", None) is None:
                                part.brake_light = nightlights.BrakeLight(part.actor, queue)
                            part.brake_light.update(ctl, carla)
                elif part.bot is not None:
                    part.bot.step(t)
                while part.ehmi_schedule and part.ehmi_schedule[0][0] <= t:
                    part.agent.set_ehmi(part.ehmi_schedule.pop(0)[1], part.role)
            laps("inputs")
            if navw is not None:
                navw.step(t, cmap)
            walkers_now = walkers_all + (crowd.on_road() if crowd is not None else []) + \
                (navw.on_road() if navw is not None else [])
            for ag in agents:
                ag.step(t, step, walkers_now)
            tools.step(t, step, walkers_now, snap.frame)      # automated drivers' cars, dashboard commands
            if crowd is not None and hasattr(crowd, "tick"):       # bicycles pushed by crossing walkers
                crowd.tick()
            if crowd is not None and summary["ticks"] % 4 == 0:     # a walker keeps its last control
                crowd.step(t)
                if getattr(crowd, "window", None) is not None:    # walkers left behind reappear ahead
                    crowd.window.step(t)
            laps("agents")
            if drone_world is not None:   # the flight model, crash checks, the drones' and cameras' transforms
                drone_world.step(t, step, events, snap)
                drone_world.broadcast(control, summary["ticks"])
                laps("drones")
            if ship_world is not None:    # the ship model and the ships' transforms
                ship_world.step(t, step)
                laps("ships")
            t_a = time.time()
            sync.tick()
            if sig_driver is not None:
                import traci
                sig_driver.step(traci.simulation.getTime(), traci)
            t_b = time.time()
            prof["sync"] += t_b - t_a
            laps.start()      # sync.tick() timed its own parts
            # every walker near a SUMO vehicle, on the road or about to step onto it (yielding.py predicts them)
            sumo_yield.step(sync, world, walkers_all + list(getattr(crowd, "walkers", None) or
                                                            (crowd.on_road() if crowd is not None else [])), step)
            laps("yield")
            if city is not None:
                city.step(t, types)
                if react is not None and summary["ticks"] % 5 == 0:
                    react.step(t, 5 * step, types)
                laps("living_city")
            signals.draw([a.get_location() for a in focus_actors])
            laps("signals")
            if scenery is not None and summary["ticks"] % 50 == 25:
                scenery.update([a.get_location() for a in focus_actors])
            if parked is not None and summary["ticks"] % 50 == 35:
                parked.update(focus_points())
            if photo is not None and photo.s.get("cameras") == "spectator" and summary["ticks"] % 5 == 0:
                # Cesium streams the photo tiles for the spectator's view: keep it where the saved or shown view is
                # (with cameras: sensors the plugin hands CARLA's camera sensors to Cesium instead)
                # (a film camera, the drone pilot's camera, the participant's view), else just behind the participant
                cams = ([r.camera for r in film.rigs if r.camera is not None] if film is not None else []) + \
                    ([drone_world.view.camera] if drone_world is not None and getattr(drone_world, "view", None)
                     is not None and getattr(drone_world.view, "camera", None) is not None else []) + \
                    ([view.camera] if view is not None and getattr(view, "camera", None) is not None else [])
                try:
                    tf = cams[0].get_transform() if cams else None
                    if tf is None and focus_actors:
                        a = focus_actors[0].get_transform()
                        fwd = a.get_forward_vector()
                        tf = carla.Transform(a.location - fwd * 8.0 + carla.Location(z=3.0), a.rotation)
                    if tf is not None:
                        side.spectator.set_transform(tf)
                except RuntimeError:
                    pass
            elif level_stream and focus_actors and summary["ticks"] % 50 == 0:
                # a baked World Partition level streams its cells (ground, roads, buildings, with their collision)
                # around the spectator: keep it over the first participant, high up and looking at the sky
                loc = focus_actors[0].get_location()
                side.spectator.set_transform(carla.Transform(carla.Location(x=loc.x, y=loc.y, z=loc.z + 60.0),
                                                             carla.Rotation(pitch=90.0)))
            laps("scenery")
            adopted_ids = {c: s_ for s_, c in sync.adopted.items() if s_ in sync.sumo2carla_ids}   # in SUMO now
            for part in parts:
                if part.actor.type_id.startswith("vehicle."):
                    sid = sync.carla2sumo_ids.get(part.actor.id) or adopted_ids.get(part.actor.id)
                    part.light = signals.next_light(sid) if sid else None
                    if hasattr(part.input, "light"):
                        part.input.light = part.light
            for ag in agents:
                if isinstance(ag, ScriptedCar):
                    sid = sync.carla2sumo_ids.get(ag.vehicle.id)
                    ag.light = signals.next_light(sid) if sid else None
                    if summary["ticks"] % 2 == 0 and ag.id in ehmi_designed:
                        if ehmi_render != "station" and ehmi_designed[ag.id] is not None:
                            ehmi_design.draw_world(side.debug, ag.vehicle, ag.ehmi, ehmi_designed[ag.id], t,
                                                   life=step * 3)
                    elif summary["ticks"] % 2 == 0 and ag.ehmi not in ("", "off"):
                        here = ag.vehicle.get_location()
                        near = [w.get_location() for w in walkers_all]
                        nearest_walker = min(near, key=lambda l: l.distance(here)) if near else None  # not look/gaze
                        if nearest_walker is not None and nearest_walker.distance(here) > 40.0:
                            nearest_walker = None
                        draw_ehmi(side.debug, ag.vehicle, ag.ehmi, ehmi_states, life=step * 3,
                                  slots=tuple(ag.cfg.get("ehmi_slots") or ehmi_slots),
                                  look_at=(nearest_walker + carla.Location(z=0.7)
                                           if nearest_walker is not None else None),
                                  text_slots=ehmi_inside.world_slots(ag.cfg.get("ehmi_slots") or ehmi_slots)
                                  if ag.id in ehmi_cabin else None)
            if summary["ticks"] % 5 == 0:
                gaze.update(t, snap.frame, parts, agents, station_gaze)
            laps("next_light_ehmi")
            snap = carla_sim.world.get_snapshot()
            if control is not None:          # every tick, for the stations' latency measurement
                control.broadcast({"type": "tick", "frame": carla_sim.tick_frame, "t": carla_sim.tick_wall})
            sim_t = snap.timestamp.elapsed_seconds
            sim0 = sim_t if sim0 is None else sim0
            if logger.sim0 is None:
                logger.sim0 = sim0
            t = sim_t - sim0
            summary["ticks"] += 1
            if film is not None:
                film.tick(t, snap)

            # stations
            if control is not None:
                for msg in control.poll():
                    kind, role = msg.get("type"), msg.get("role")
                    if drone_world is not None and drone_world.on_message(msg):    # a pilot's sticks, camera
                        continue
                    if tools.on_message(msg, t, snap.frame):     # responses, take-over, engage
                        continue
                    if kind == "ehmi" and role in by_role and by_role[role].agent:
                        by_role[role].agent.set_ehmi(msg.get("state", "off"), role)
                    elif kind == "gaze":
                        station_gaze[role] = msg.get("yaw")
                    elif kind == "sync":
                        sync_at.insert(0, t)
                    elif kind == "stats":
                        # everything the station reports (labcheck's images, ready_s, camera_hz, ux flags, ...)
                        stats = {"role": role, **{k: v for k, v in msg.items() if k not in ("type", "role")}}
                        station_stats[role] = stats
                        logger.event(t, snap.frame, "station_stats", "", json.dumps(stats))
                    elif kind in ("hello", "bye"):
                        logger.event(t, snap.frame, f"station_{'joined' if kind == 'hello' else 'left'}", "", role)
                        if kind == "hello" and role in by_role:
                            control.send(role, {"type": "state", **state, "actor_id": by_role[role].actor.id})
            while sync_at and sync_at[0] <= t:
                sync_at.pop(0)
                if control is not None:
                    control.broadcast({"type": "flash", "frame": snap.frame, "t": t})
                logger.event(t, snap.frame, "visual_sync", "", "red bar on every station for 1 s")

            # log
            ai = {}
            sumo_signals = {}
            for sid, cid in sync.sumo2carla_ids.items():
                if sid in sync.adopted:      # a participant's car driven by SUMO: logged as the participant
                    continue
                try:
                    sa = sync.sumo.get_actor(sid)
                except KeyError:             # not inserted yet (a living-city car handed to SUMO)
                    continue
                if cid not in types:
                    a = world.get_actor(cid)
                    tid = a.type_id if a else ""
                    kind = compat.traffic_kind(tid, sa.vclass if sa else None, tc["traffic"]["bike_types"],
                                               tc["traffic"]["bus_type"])
                    types[cid] = (kind, tid)
                    logger.event(t, snap.frame, "spawn", cid, f"{kind} {tid} sumo:{sid}")
                ai[cid] = types[cid]
                sumo_signals[cid] = sa.signals if sa else 0
            for w in (crowd.walkers if crowd is not None else []) + (navw.walkers if navw is not None else []):
                ai[w.id] = ("walker", w.type_id)
            for vid, v in (city.vehicles.items() if city is not None else ()):    # vans, parking cars (not SUMO's)
                if v.get("actor") is not None and vid not in ai:
                    ai[vid] = ("car", v["actor"].type_id)
            for part in parts:   # an event whenever the light a participant faces changes
                key = (part.light[0], part.light[2]) if part.light else None
                if key != part.light_key:
                    if key:
                        logger.event(t, snap.frame, "next_light", part.actor.id,
                                     f"{part.role} {key[0]} {key[1]} at {part.light[1]:.1f} m")
                    part.light_key = key
            records = [p.record() for p in parts if p.kind not in ("av_passenger", "drone")]
            for ag in agents:
                if isinstance(ag, ScriptedCar):
                    records.append({"actor": ag.vehicle, "role": ag.id, "kind": "agent", "ctl": None,
                                    "light": ag.light, "ehmi": ag.ehmi})
                else:
                    records.append({"actor": ag.walker, "role": ag.id, "kind": "agent"})
                for name, detail in ag.events:
                    logger.event(t, snap.frame, name, ag.vehicle.id if isinstance(ag, ScriptedCar) else ag.walker.id,
                                 detail)
                    if name == "ehmi" and control is not None:   # stations: auditory eHMIs, station-drawn designs
                        control.broadcast({"type": "ehmi_state", "actor_id": ag.vehicle.id, "state": ag.ehmi})
                ag.events.clear()
            if control is not None and summary["ticks"] % 100 == 1:
                control.broadcast({"type": "ehmi_state", "all": {str(ag.vehicle.id): ag.ehmi for ag in agents
                                                                 if isinstance(ag, ScriptedCar)}})
            for part in parts:
                for name, detail in getattr(part.input, "events", ()):
                    logger.event(t, snap.frame, name, part.actor.id, detail)
                if getattr(part.input, "events", None):
                    part.input.events.clear()
            for name, detail in sumo_yield.events:
                logger.event(t, snap.frame, name, "", detail)
            sumo_yield.events.clear()
            for name, detail in (crowd.events if crowd is not None else ()):
                logger.event(t, snap.frame, name, "", detail)
            if crowd is not None:
                crowd.events.clear()
            if city is not None:
                for name, aid, detail in city.events:
                    logger.event(t, snap.frame, name, aid, detail)
                city.events.clear()
                for se in city.sound_events:      # horns, bells, hazards, doors: the sound helper renders them
                    if control is not None:
                        control.broadcast({"type": "sound_event", **se})
                    if host_audio is not None and hasattr(host_audio, "sound_event"):
                        host_audio.sound_event(se)
                city.sound_events.clear()
            sumo_driven = set(sync.adopted.values())
            for r in records:     # the light states the bridge read this frame (saves a round trip each)
                lights = carla_sim.frame_lights.get(r["actor"].id)
                if lights is None and r["actor"].id in sumo_driven:   # SUMO drives it: the lights the bridge set
                    lights = carla_sim.current_lights(r["actor"].id)  # (get_light_state is a server round trip)
                r["lights"] = lights
            laps("stations_events")
            logger.log(snap, records, ai, sumo_signals, [])
            if drone_world is not None:
                drone_world.log_frame(logger, t, snap.frame)
            if ship_world is not None:
                ship_world.log_frame(logger, t, snap.frame)
            tools.log(t, snap.frame)
            if live is not None:
                live.tick(t, snap, summary["ticks"], logger, world, sync, types, crowd, drone_world)
            laps("log")
            if host_audio is not None:
                host_audio.tick(t, snap, carla_sim.frame_lights)
                laps("audio")
            prof["log"] += time.time() - t_b
            for c in collisions[seen_col:]:
                logger.event(t, c[1], "collision", c[3], f"{c[2]} hit {c[4]}")
            seen_col = len(collisions)

            first = parts[0] if parts else None
            if view and first:
                v = first.actor.get_velocity()
                lt = first.light
                view.render([f"{3.6 * (v.x ** 2 + v.y ** 2) ** 0.5:5.1f} km/h", f"t {t:6.1f} s",
                             f"light {lt[2]} in {lt[1]:.0f} m" if lt else "", *tools.hud_lines(first)])
            if t >= next_status:
                rtf = t / max(1e-6, time.time() - t_wall0)
                n = max(1, summary["ticks"] - prof["ticks0"])
                speeds = "  ".join(f"{p.role} {3.6 * (p.actor.get_velocity().x ** 2 + p.actor.get_velocity().y ** 2) ** 0.5:4.1f}"
                                   if p.kind != "drone" else f"{p.role} {drone_world.speed_kmh(p):4.1f}"
                                   for p in parts) or "-"
                log(f"t {t:6.1f}s  sumo {len(sync.sumo2carla_ids):3d}  km/h: {speeds}  RTF {rtf:.2f}  "
                    f"rows {logger.rows}  ms/tick sync {1000 * prof['sync'] / n:.1f} log {1000 * prof['log'] / n:.1f}")
                if profile:
                    log("  ms/tick " + laps.line(n))
                laps.reset()
                prof.update(sync=0.0, log=0.0, ticks0=summary["ticks"])
                next_status += 10.0
            if t >= duration or tools.abort or any(getattr(p.input, "quit", False) for p in parts) or \
                    (drone_world is not None and drone_world.quit):
                break
            for part in parts:            # recovery for host-local cars (the autopilot, or a human pressing R)
                ctl_in = part.input
                if part.kind != "driver" or ctl_in is None or tools.drives(part):
                    continue
                veh = part.actor
                if getattr(ctl_in, "inserted", True) is False:  # the SUMO driver waiting below the road (hold)
                    continue
                # down on the ground plane; a town has kerbs and real heights: off every driving lane; a map whose
                # roads have real heights (a baked level, map.elevation): off its lane's height or tipped (levels.py)
                if town:
                    off_road = towns.off_road(cmap, veh.get_location())
                elif carla_sim.check_level:
                    if summary["ticks"] % 10 == 0 or veh.id not in was_off:
                        tfv = veh.get_transform()
                        wpv = cmap.get_waypoint(tfv.location, project_to_road=True, lane_type=carla.LaneType.Driving)
                        off_road = levels.tipped(tfv.rotation.pitch, tfv.rotation.roll) or (
                            wpv is not None and levels.off_level(tfv.location.z, wpv.transform.location.z))
                    else:
                        off_road = was_off[veh.id]
                else:
                    off_road = veh.get_location().z < -0.25
                if off_road and not was_off.get(veh.id):
                    logger.event(t, snap.frame, "left_road", veh.id, part.role)
                was_off[veh.id] = off_road
                v = veh.get_velocity()
                ctl = part.ctl or carla.VehicleControl()
                if isinstance(ctl_in, inputs.Autopilot) and ctl.throttle > 0.3 and (v.x ** 2 + v.y ** 2) < 0.1:
                    stuck_since.setdefault(veh.id, t)
                    if t - stuck_since[veh.id] > 3.0:
                        logger.event(t, snap.frame, "stuck", veh.id, part.role)
                        ctl_in.stuck = True
                        stuck_since.pop(veh.id)
                else:
                    stuck_since.pop(veh.id, None)
                if getattr(ctl_in, "stuck", False) or getattr(ctl_in, "reset", False) or \
                        (off_road and isinstance(ctl_in, inputs.Autopilot)):
                    wp = world.get_map().get_waypoint(veh.get_location(), project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                    tf = wp.transform
                    tf.location.z += 0.4
                    veh.set_target_velocity(carla.Vector3D())
                    veh.set_transform(tf)
                    ctl_in.stuck = ctl_in.reset = False
                    logger.event(t, snap.frame, "reset_to_lane", veh.id, f"{part.role} road {wp.road_id} lane {wp.lane_id}")
                    if hasattr(ctl_in, "restart") and not ctl_in.restart(wp):
                        # nowhere to go from this lane (it ends): start again on a random lane
                        cands = [w for w in world.get_map().generate_waypoints(20.0)
                                 if w.lane_type == carla.LaneType.Driving and not w.is_junction]
                        w = random.choice(cands)
                        tf = w.transform
                        tf.location.z += 0.4
                        veh.set_transform(tf)
                        ctl_in.restart(w)
                        logger.event(t, snap.frame, "respawn", veh.id, f"{part.role} road {w.road_id} lane {w.lane_id}")
            laps("rest")
            if tc["simulation"]["real_time"]:
                time.sleep(max(0.0, step - (time.time() - t_tick)))
        wall = time.time() - t_wall0
        if profile:
            summary["ms_per_tick"] = laps.totals(summary["ticks"])
        if carla_sim.level_teleports:
            summary["level_teleports"] = carla_sim.level_teleports     # put back onto their level or upright
        summary.update({"sim_s": round(t, 2), "wall_s": round(wall, 2), "rtf": round(t / max(wall, 1e-6), 3),
                        "rows": logger.rows, "collisions": len(collisions), "agents": len(agents),
                        "stations": station_roles, "sumo_vehicles_spawned": len(types),
                        "station_stats": station_stats, "parked_cars": parked.n if parked is not None else 0})
        if drone_world is not None:
            summary["drones"] = drone_world.summary()
        if ship_world is not None:
            summary["ships"] = ship_world.summary()
        if scenery is not None:
            summary.update({"scenery_meshes": scenery.n, "scenery_refused": scenery.n_failed})
            if scenery.n_failed:        # which meshes this server lacks (a CARLA package without the content packs)
                (out / "scenery_refused.json").write_text(json.dumps(
                    {"refused": scenery.n_failed, "meshes": dict(scenery.failed.most_common()),
                     "reasons": dict(scenery.fail_reasons)}, indent=1))
        logger.event(t, snap.frame, "trial_end", "", json.dumps(summary))
        log(f"trial done: {summary}")
    finally:
        tools.close(summary)
        look.ATTRIBUTION = None
        if photo is not None:
            photo.destroy()
        if film is not None:
            film.close()
        state.update(state="ended")
        if control is not None:
            control.broadcast({"type": "state", **state})
            # give the stations a moment to let go of the server before it stops (see station.stop)
            waiting, t_end = set(control.connected()) & {p.role for p in parts if p.station}, time.time() + 5.0
            while waiting and time.time() < t_end:
                for msg in control.poll():
                    if msg.get("type") in ("ended_ack", "bye"):
                        waiting.discard(msg.get("role"))
                time.sleep(0.05)
            time.sleep(0.5)
        if drone_world is not None:
            drone_world.close()
        if ship_world is not None:
            ship_world.close()
        if navw is not None:
            navw.destroy()
        if city is not None:
            try:
                info = city.summary()
                info["reactions"] = {k: reactions.rate(city.lc, k) for k in reactions.RATES}
                info["reaction_sources"] = {k: reactions.SOURCES[reactions.RATES[k][2]] for k in reactions.RATES}
                (out / "living_city.json").write_text(json.dumps(info, indent=2, default=str))
                summary["living_city"] = info["counts"]
                city.close()
            except Exception as err:  # noqa: BLE001
                log(f"living city summary: {err}")
        if logger is not None:
            logger.close()
        for x in own_logs:
            try:
                x.close()
            except Exception as err:  # noqa: BLE001 - the trial's own logs are written
                log(f"{type(x).__name__} log: {err}")
        if recorder is not None:
            client.stop_recorder()
        if view:
            view.destroy()
        if sync:
            try:
                sync.close()
            except Exception as err:  # noqa: BLE001
                log(f"bridge close: {err}")
        server.stop()
        if host_audio is not None:
            host_audio.close(summary)
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        if tc["logging"].get("analyse", True) and (out / "frames.csv").exists():
            try:
                rows = analysis.analyse_trial(out)
                log(f"analysis: {len(rows)} pedestrian-vehicle encounters in {out / 'analysis'}")
            except Exception as err:  # noqa: BLE001 - the trial's logs are written; analysis can be re-run
                log(f"analysis failed ({err}); run python -m coupledcarla.analysis {out}")
    return summary


def _make_local(participants):
    """--local: every station participant runs on the host as its bot (a driver on the autopilot)."""
    for p in participants:
        if p.get("station"):
            p["station"] = False
            if p.get("kind", "driver") == "driver":
                p["input"] = "autopilot"


def _set_stations(participants, roles):
    """--station-roles: these roles have a station (station: true), every other station participant runs on the
    host as its bot (as with --local); the session dashboard passes the roles the experimenter gave out."""
    roles = set(roles)
    for p in participants:
        if p["role"] in roles:
            p["station"] = True
        elif p.get("station"):
            _make_local([p])


def _needs_control(cfg, local=False):
    """Whether any trial has a station participant (a trial or condition may bring its own participants list)."""
    if local:
        return False
    return any(p.get("station") for t in cfg["trials"] for p in config.trial_config(cfg, t)["participants"]) or \
        any(p.get("station") for p in cfg["participants"])


def _actor_id(part):
    return part.actor.id if part is not None and part.actor is not None else None


def main():
    import signal
    import sys
    # A plain kill (SIGTERM) runs the clean-up too, so the CARLA server and SUMO do not outlive the runner.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--trial", type=int, action="append", help="trial index (repeatable); default: all")
    ap.add_argument("--input", choices=["autopilot", "keyboard", "wheel"], help="override host-local drivers' input")
    ap.add_argument("--headless", action="store_true", help="no window (autopilot input)")
    ap.add_argument("--duration", type=float, help="override each trial's duration_s")
    ap.add_argument("--frames-every", type=float, help="save the first participant's view every N s")
    ap.add_argument("--record-audio", action="store_true",
                    help="write the first participant's soundscape, in simulation time, to audio.wav in the trial's "
                         "log folder (sound.record; see soundrec.py)")
    ap.add_argument("--quality", choices=["Low", "Epic"], help="override server.quality")
    ap.add_argument("--view", choices=["driver", "chase", "hood"], help="override every participant's view")
    ap.add_argument("--step", type=float, help="override simulation.step (s)")
    ap.add_argument("--local", action="store_true",
                    help="run every station participant on the host (their bot), for tests without stations")
    ap.add_argument("--participant", type=int, default=0,
                    help="participant number: the trials in this participant's counterbalanced order (design.order)")
    ap.add_argument("--order", choices=study.ORDERS, help="override design.order")
    ap.add_argument("--dashboard", action="store_true", help="serve the experimenter dashboard (dashboard.port)")
    ap.add_argument("--dashboard-host", help="listen there instead of 127.0.0.1 (other machines; needs the token)")
    ap.add_argument("--dashboard-port", type=int)
    # run by the session dashboard (session.py): stations through its link, its commands and live map
    ap.add_argument("--control-relay", help=argparse.SUPPRESS)          # 127.0.0.1:PORT of the session's link
    ap.add_argument("--relay-token", help=argparse.SUPPRESS)
    ap.add_argument("--station-roles", type=lambda v: [r for r in v.split(",") if r],
                    help="comma-separated roles that have a station; other station roles run as bots on the host")
    ap.add_argument("--server-host", help="override server.host (a CARLA server on another machine)")
    ap.add_argument("--server-port", type=int, help="override server.port")
    ap.add_argument("--no-launch", action="store_true", help="use a running CARLA server (server.launch: false)")
    ap.add_argument("--real-time", choices=["on", "off"], help="override simulation.real_time")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    from . import validate
    report, _ = validate.check(yaml.safe_load(Path(args.experiment).read_text()), cfg, roads=False, participants=[])
    for level, where, msg in report.items:
        print(f"[validate] {level}  {where}: {msg}  (python -m coupledcarla.validate {args.experiment})", flush=True)
    # the trials in this participant's order (conditions merged; --trial picks positions in this list)
    cfg["trials"] = study.plan(cfg, args.participant, args.order)
    if args.participant and not args.control_relay:      # the session dashboard writes the session file itself
        stamp = time.strftime("%Y%m%d-%H%M%S")
        sdir = Path(cfg["_root"]) / cfg["logging"]["dir"] / cfg["name"]
        sdir.mkdir(parents=True, exist_ok=True)
        import csv
        rows = study.session_rows(cfg["trials"])
        with open(sdir / f"session_P{args.participant:02d}_{stamp}.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]), delimiter=";")
            w.writeheader()
            w.writerows(rows)
        print(f"participant {args.participant}: " + " | ".join(study.label(t) for t in cfg["trials"]), flush=True)
    if args.quality:
        cfg["server"]["quality"] = args.quality
    if args.server_host:
        cfg["server"]["host"] = args.server_host
    if args.server_port:
        cfg["server"]["port"] = args.server_port
    if args.no_launch:
        cfg["server"]["launch"] = False
    if args.real_time:
        cfg["simulation"]["real_time"] = args.real_time == "on"
    if args.step:
        cfg["simulation"]["step"] = args.step
    if args.frames_every:
        cfg["logging"]["frames_every_s"] = args.frames_every
    if args.duration:
        for t in cfg["trials"]:
            t["duration_s"] = args.duration
    if args.local:
        _make_local(cfg["participants"])
    control = None
    dashboard = None
    dc = cfg.get("dashboard") or {}
    if args.control_relay:            # the session dashboard owns the stations and the controls
        from .coupling import RelayControl
        control = RelayControl(args.control_relay, args.relay_token or "", Log())
        dashboard = control.dashboard
    elif _needs_control(cfg, args.local):
        control = ControlServer(int((cfg.get("coupling") or {}).get("port", 2100)), Log())
    if dashboard is None and (args.dashboard or args.dashboard_host or dc.get("enabled")):
        from .dashboard import Dashboard
        dashboard = Dashboard(args.dashboard_host or dc.get("host", "127.0.0.1"),
                              args.dashboard_port or int(dc.get("port", 8780)), Log())
    try:
        for i in args.trial or range(len(cfg["trials"])):
            summary = run_trial(cfg, i, args, control, dashboard)
            if summary.get("aborted") == "session stopped":
                break
    finally:
        if control is not None:
            control.close()
        if dashboard is not None:
            dashboard.update(state="session over")
            dashboard.close()


if __name__ == "__main__":
    main()
