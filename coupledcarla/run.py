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

from . import analysis, compat, config, crowdpeds, inputs, mapbuild, parking, realworld, season, soundrec, study, traffic
from .ai import DEFAULT_EHMI, MutualGaze, PathWalker, ScriptedCar, ScriptedWalker, SumoYield, draw_ehmi
from . import ehmi_design, multigpu, stationux
from .bridge.synchronization import BridgeHelper, CarlaSimulation, SimulationSynchronization, SumoSimulation
from .coupling import ControlServer
from .display import View
from .drones import DroneView, DroneWorld
from .geo import Geo, ground, heading_to_yaw
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


def load_world(client, xodr, mc, slab=False):
    if mc.get("carla_level"):   # a baked level (its own roads, buildings and ground): load it by name
        client.set_timeout(600.0)
        world = client.load_world(mc["carla_level"])
        client.set_timeout(60.0)
        return world
    p = mc["carla"]
    params = carla.OpendriveGenerationParameters(
        vertex_distance=p["vertex_distance"], max_road_length=p["max_road_length"], wall_height=p["wall_height"],
        additional_width=p["additional_width"], smooth_junctions=p["smooth_junctions"],
        enable_mesh_visibility=True, enable_pedestrian_navigation=p["pedestrian_navigation"])
    client.set_timeout(600.0)
    world = client.generate_opendrive_world(Path(xodr).read_text(encoding="utf-8"), params)
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
        choices = ([("/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube", "20000", -0.45 - 10000.0)] if slab else [])
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
        fields = realworld.carla_weather_fields(rec or {}, alt, az)
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
    return info


def spawn_vehicle(world, geo, p, log):
    cmap = world.get_map()
    bp = compat.find(world, p.get("vehicle", "vehicle.lincoln.mkz_2020"), "passenger")
    bp.set_attribute("role_name", p["role"])
    sp = p.get("spawn") or {}
    if "lat" in sp:
        x, y = geo.to_carla(sp["lat"], sp["lon"])
        wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
        log(f"{p['role']}: asked for {sp['lat']:.5f},{sp['lon']:.5f} -> lane {wp.road_id}/{wp.lane_id} "
            f"{wp.transform.location.distance(carla.Location(x=x, y=y, z=wp.transform.location.z)):.1f} m away")
        candidates = [wp.transform]
        if "heading" in sp:
            yaw = heading_to_yaw(sp["heading"])
            opp = compat.left_lane(wp)
            if opp and abs((opp.transform.rotation.yaw - yaw + 180) % 360 - 180) < \
                    abs((wp.transform.rotation.yaw - yaw + 180) % 360 - 180):
                candidates = [opp.transform]
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
           "camera_attributes": dict((tc.get("film") or {}).get("attributes") or {})}   # demo videos (film.py)
    # sound, in-car displays, haptics and designed eHMIs (stationux.py), when the experiment sets them
    out.update(stationux.welcome_extras(tc, part.cfg, part.kind, Path(tc["_path"]).parent,
                                        default_states=list(DEFAULT_EHMI)))
    out.update(multigpu.welcome(tc["server"], part.role))     # secondaries: relay and camera placement
    if part.kind == "drone":      # gamepad / RC axes and buttons for the station's stick input (droneinput.py)
        out["drone_input"] = {k: part.cfg[k] for k in ("axes", "joystick", "deadband", "expo", "buttons")
                              if k in part.cfg}
    return out


def run_trial(cfg, trial_idx, args, control=None, dashboard=None):
    log = Log()
    trial = cfg["trials"][trial_idx]
    tc = config.trial_config(cfg, trial)
    if getattr(args, "local", False):    # also a trial's or condition's own participants list (main sees only the top's)
        _make_local(tc["participants"])
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
    step = tc["simulation"]["step"]
    duration = float(trial.get("duration_s", 300))
    ehmi_states = (tc.get("ehmi") or {}).get("states", DEFAULT_EHMI)
    ehmi_slots = tuple((tc.get("ehmi") or {}).get("slots", ["front"]))
    ehmi_designed = ehmi_design.drawers(ehmi_design.for_agents(tc, Path(tc["_path"]).parent, log))
    ehmi_render = (tc.get("ehmi") or {}).get("render", "world")     # designed eHMIs: world | station | both
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
    sync = view = recorder = world = logger = drone_world = film = host_audio = None
    collisions, agents = [], []
    summary = {"trial": trial_idx, "ticks": 0}
    try:
        # scenery.json names the meshes of one CARLA version's catalogue: rebuilt first when made for the other
        has_scenery = tc["map"].get("scenery", {}).get("enabled", True) and \
            mapbuild.scenery_for_server(tc, mdir, compat.server_major(client), log)
        world = load_world(client, mdir / "map.xodr", tc["map"], slab=has_scenery)
        bb = tc["map"]["bbox"]
        wx = set_weather(world, tc["simulation"], ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2), mdir / "real", log)
        (out / "weather.json").write_text(json.dumps(wx, indent=2))
        tc["_sound_env"] = soundrec.sound_env(wx, mdir, world.get_weather(),
                                              (tc.get("sound") or {}).get("weather")).as_dict()   # stations' sound
        BridgeHelper.night = wx["night"]
        if tc["server"].get("offscreen", True):
            # nobody sees the spectator of an off-screen server, but the server still renders its view: park it
            # high up, looking at the sky, so it costs nothing (it rendered the whole city's scenery otherwise)
            world.get_spectator().set_transform(carla.Transform(carla.Location(z=3000.0), carla.Rotation(pitch=90.0)))
        log(f"world ready: {len(world.get_map().get_spawn_points())} spawn points")
        level_stream = bool(tc["map"].get("carla_level"))

        sumo = SumoSimulation(str(cfg_file), step)
        carla_sim = CarlaSimulation(tc["server"]["host"], tc["server"]["port"], step)
        # drawings, traffic-light states and the spectator on a connection of their own (see side.py)
        side = Side(tc["server"]["host"], tc["server"]["port"])
        carla_sim.use_side(side.world)
        focus_actors = []   # SUMO actors appear in CARLA only near these (the participants)
        sync = SimulationSynchronization(
            sumo, carla_sim, tc["simulation"]["tls_manager"], sync_vehicle_lights=True,
            sync_vehicle_color=True,     # each SUMO car's colour (traffic.py: from the vehicle register) on its CARLA car
            tm_port=tc["server"]["tm_port"] if tc["simulation"].get("traffic_manager") else None,
            radius=tc["simulation"]["lod_radius_m"],
            focus=lambda: [a.get_location() for a in focus_actors] or None)
        # The bridge assumes CARLA = SUMO - netOffset, true for CARLA's own towns. Our OpenDRIVE carries a UTM
        # georeference, so map.net.xml's netOffset is the whole UTM shift (~-671 km, -5701 km) and every SUMO
        # car would land thousands of km away (and moving the participant there crashes SUMO). The real shift
        # is the difference between map.net.xml's and geo.net.xml's offsets (CARLA x,y = geo.net x,-y).
        mo, go = sumo.net.getLocationOffset(), geo.net.getLocationOffset()
        BridgeHelper.offset = (mo[0] - go[0], mo[1] - go[1])
        signals = Signals(world, sumo.net, BridgeHelper.offset, step=step, debug=side.debug)
        log(f"traffic lights: {len(signals.heads)} signalled lanes from SUMO drawn near participants")
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
                    part.input = inputs.make(p.get("input", "keyboard"), part.actor)
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
                                profile=tc["simulation"].get("camera_profile", "Dashcam"))
            if not part.station and part.kind == "drone":
                window = p.get("window", p.get("input", "keyboard") in ("keyboard", "gamepad", "rc"))
                drone_world.local_input(part.drone, window and (i == 0))
                if i == 0 and (window or tc["logging"]["frames_every_s"] > 0):
                    view = DroneView(drone_world, part.drone, window=window, save_dir=out,
                                     save_every_s=tc["logging"]["frames_every_s"],
                                     profile=tc["simulation"].get("camera_profile", "Dashcam"))
            log(f"{part.role}: {part.kind} {part.actor.type_id} "
                f"{'at a station' if part.station else 'on the host (' + str(p.get('input', 'bot')) + ')'}")
        for ag in agents:
            if isinstance(ag, ScriptedCar):
                watch_collisions(ag.vehicle, ag.id)

        if wx["night"]:      # headlights on after dark (SUMO cars: BridgeHelper.night)
            lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position
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
            log(f"scenery: {scenery.n} meshes within {scenery.radius:.0f} m of the participants "
                f"({time.time() - t_s:.1f} s)" + (f"; {scenery.n_lights} lamps with light within "
                                                  f"{scenery.lights_radius:.0f} m" if scenery.light_bp else ""))
            parked = parking.scenery(client, world, mdir, tc, log,
                                     on_spawn=lambda ids: carla_sim._kinds.update({i: False for i in ids}))
            if parked is not None:
                parked.update([a.get_location() for a in focus_actors])
                log(f"parked: {parked.n} cars within {parked.radius:.0f} m of the participants")
        for part in parts:   # BasicAgent planned from (0, 0, 0) before the first tick: plan from the spawn lane
            if hasattr(part.input, "restart"):
                part.input.restart(world.get_map().get_waypoint(part.actor.get_location()))
        walkers_all = [p.actor for p in parts if p.kind == "pedestrian"] + \
                      [a.walker for a in agents if isinstance(a, ScriptedWalker)]
        # the pedestrian crowd: calibrated with CROWD for the map's country (crowdpeds.py), or with behaviour off
        # ai.Crowd beside the roads; its walkers join walkers_now (cars yield to them) only while on the road
        crowd = crowdpeds.make_crowd(world, geo, tc, mdir, [p.actor.get_location() for p in parts], wx["night"],
                                     duration, log)
        sumo_yield = SumoYield()
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
                ag.apply = lambda c, i=ag.vehicle.id: queue(carla.command.ApplyVehicleControl(i, c))
            else:
                ag.follow.apply = lambda c, i=ag.walker.id: queue(walker_cmd(i, c))
        for f in getattr(crowd, "follow", None) or []:        # ai.Crowd (behaviour off)
            f.apply = lambda c, i=f.walker.id: queue(walker_cmd(i, c))
        if crowd is not None and hasattr(crowd, "send"):      # crowdpeds.CrowdSim
            crowd.send = lambda actor, c: queue(walker_cmd(actor.id, c))

        if drone_world is not None:      # obstacles, wind and AI missions once the participants stand
            drone_world.queue = queue
            drone_world.start([a.get_location() for a in focus_actors])
        if tc["logging"]["carla_recorder"]:
            recorder = client.start_recorder(f"coupledcarla_{tc['name']}_{stamp}_trial{trial_idx}.log", True)
        logger = TrialLogger(out, geo, tc["logging"], step)
        fc = tc.get("film") or {}
        if fc.get("camera") or fc.get("cameras"):       # demo-video cameras on a sim-time grid (tools/make_videos.py)
            from .film import Film
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
                elif part.bot is not None:
                    part.bot.step(t)
                while part.ehmi_schedule and part.ehmi_schedule[0][0] <= t:
                    part.agent.set_ehmi(part.ehmi_schedule.pop(0)[1], part.role)
            laps("inputs")
            walkers_now = walkers_all + crowd.on_road() if crowd is not None else walkers_all
            for ag in agents:
                ag.step(t, step, walkers_now)
            tools.step(t, step, walkers_now, snap.frame)      # automated drivers' cars, dashboard commands
            if crowd is not None and summary["ticks"] % 4 == 0:     # a walker keeps its last control
                crowd.step(t)
            laps("agents")
            if drone_world is not None:   # the flight model, crash checks, the drones' and cameras' transforms
                drone_world.step(t, step, events, snap)
                drone_world.broadcast(control, summary["ticks"])
                laps("drones")
            t_a = time.time()
            sync.tick()
            t_b = time.time()
            prof["sync"] += t_b - t_a
            laps.start()      # sync.tick() timed its own parts
            sumo_yield.step(sync, world, walkers_now)
            laps("yield")
            signals.draw([a.get_location() for a in focus_actors])
            laps("signals")
            if scenery is not None and summary["ticks"] % 50 == 25:
                scenery.update([a.get_location() for a in focus_actors])
            if parked is not None and summary["ticks"] % 50 == 35:
                parked.update([a.get_location() for a in focus_actors])
            if level_stream and focus_actors and summary["ticks"] % 50 == 0:
                # a baked World Partition level streams its cells (ground, roads, buildings, with their collision)
                # around the spectator: keep it over the first participant, high up and looking at the sky
                loc = focus_actors[0].get_location()
                side.spectator.set_transform(carla.Transform(carla.Location(x=loc.x, y=loc.y, z=loc.z + 60.0),
                                                             carla.Rotation(pitch=90.0)))
            laps("scenery")
            for part in parts:
                if part.actor.type_id.startswith("vehicle."):
                    sid = sync.carla2sumo_ids.get(part.actor.id)
                    part.light = signals.next_light(sid) if sid else None
                    if hasattr(part.input, "light"):
                        part.input.light = part.light
            for ag in agents:
                if isinstance(ag, ScriptedCar):
                    sid = sync.carla2sumo_ids.get(ag.vehicle.id)
                    ag.light = signals.next_light(sid) if sid else None
                    if summary["ticks"] % 2 == 0 and ag.id in ehmi_designed:
                        if ehmi_render != "station":
                            ehmi_design.draw_world(side.debug, ag.vehicle, ag.ehmi, ehmi_designed[ag.id], t,
                                                   life=step * 3)
                    elif summary["ticks"] % 2 == 0 and ag.ehmi not in ("", "off"):
                        here = ag.vehicle.get_location()
                        near = [w.get_location() for w in walkers_all]
                        look = min(near, key=lambda l: l.distance(here)) if near else None
                        if look is not None and look.distance(here) > 40.0:
                            look = None
                        draw_ehmi(side.debug, ag.vehicle, ag.ehmi, ehmi_states, life=step * 3,
                                  slots=tuple(ag.cfg.get("ehmi_slots") or ehmi_slots),
                                  look_at=look + carla.Location(z=0.7) if look is not None else None)
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
                sa = sync.sumo.get_actor(sid)
                if cid not in types:
                    a = world.get_actor(cid)
                    tid = a.type_id if a else ""
                    kind = compat.traffic_kind(tid, sa.vclass if sa else None, tc["traffic"]["bike_types"],
                                               tc["traffic"]["bus_type"])
                    types[cid] = (kind, tid)
                    logger.event(t, snap.frame, "spawn", cid, f"{kind} {tid} sumo:{sid}")
                ai[cid] = types[cid]
                sumo_signals[cid] = sa.signals if sa else 0
            if crowd is not None:
                for w in crowd.walkers:
                    ai[w.id] = ("walker", w.type_id)
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
            for name, detail in sumo_yield.events:
                logger.event(t, snap.frame, name, "", detail)
            sumo_yield.events.clear()
            for name, detail in (crowd.events if crowd is not None else ()):
                logger.event(t, snap.frame, name, "", detail)
            if crowd is not None:
                crowd.events.clear()
            for r in records:     # the light states the bridge read this frame (saves a round trip each)
                r["lights"] = carla_sim.frame_lights.get(r["actor"].id)
            laps("stations_events")
            logger.log(snap, records, ai, sumo_signals, [])
            if drone_world is not None:
                drone_world.log_frame(logger, t, snap.frame)
            tools.log(t, snap.frame)
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
                off_road = veh.get_location().z < -0.25          # down on the ground plane
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
        summary.update({"sim_s": round(t, 2), "wall_s": round(wall, 2), "rtf": round(t / max(wall, 1e-6), 3),
                        "rows": logger.rows, "collisions": len(collisions), "agents": len(agents),
                        "stations": station_roles, "sumo_vehicles_spawned": len(types),
                        "station_stats": station_stats, "parked_cars": parked.n if parked is not None else 0})
        if drone_world is not None:
            summary["drones"] = drone_world.summary()
        logger.event(t, snap.frame, "trial_end", "", json.dumps(summary))
        log(f"trial done: {summary}")
    finally:
        tools.close(summary)
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
        if logger is not None:
            logger.close()
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
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    from . import validate
    report, _ = validate.check(yaml.safe_load(Path(args.experiment).read_text()), cfg, roads=False, participants=[])
    for level, where, msg in report.items:
        print(f"[validate] {level}  {where}: {msg}  (python -m coupledcarla.validate {args.experiment})", flush=True)
    # the trials in this participant's order (conditions merged; --trial picks positions in this list)
    cfg["trials"] = study.plan(cfg, args.participant, args.order)
    if args.participant:
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
    if _needs_control(cfg, args.local):
        control = ControlServer(int((cfg.get("coupling") or {}).get("port", 2100)), Log())
    dashboard = None
    dc = cfg.get("dashboard") or {}
    if args.dashboard or args.dashboard_host or dc.get("enabled"):
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
