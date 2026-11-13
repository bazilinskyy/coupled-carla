"""Run an experiment's trials (the host): CARLA world + SUMO traffic + participants + AI agents + logging.

    python -m coupledcarla.run experiments/eindhoven_centre.yaml [--trial 0] [--input keyboard] [--headless]
    python -m coupledcarla.run experiments/eindhoven_crossing.yaml        # coupled: waits for its stations

Each tick: apply the inputs of host-local participants and step the AI agents, then step SUMO and CARLA
together through the bridge (SUMO moves its vehicles in CARLA; CARLA-driven cars, human or AI, are mirrored
into SUMO, so traffic reacts to them), let SUMO traffic yield to pedestrians, draw traffic lights and eHMIs,
handle the stations' messages, log the frame and draw the host's view. With simulation.real_time the loop
sleeps so that one simulated second takes one wall-clock second.

Participants (coupled-sim's roles) are `driver` (a manual car), `pedestrian` (a walker) and `av_passenger`
(rides in a scripted AV and sets its eHMI). Each is either host-local (input keyboard / wheel / autopilot /
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

from . import analysis, compat, config, inputs, mapbuild, traffic
from .ai import DEFAULT_EHMI, Crowd, MutualGaze, PathWalker, ScriptedCar, ScriptedWalker, SumoYield, draw_ehmi
from .bridge.synchronization import BridgeHelper, CarlaSimulation, SimulationSynchronization, SumoSimulation
from .coupling import ControlServer
from .display import View
from .geo import Geo, heading_to_yaw
from .logger import TrialLogger
from .routing import SafeGlobalRoutePlanner
from .server import Server
from .signals import Signals


class Log:
    def __init__(self):
        self.t0 = time.time()

    def __call__(self, msg):
        print(f"[{time.time() - self.t0:7.1f}s] {msg}", flush=True)


def load_world(client, xodr, mc):
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
        bp.set_attribute("mesh_path", "/Engine/BasicShapes/Plane.Plane")
        bp.set_attribute("scale", "3000")
        bounds = [w.transform.location for w in world.get_map().generate_waypoints(50.0)]
        cx = sum(b.x for b in bounds) / len(bounds)
        cy = sum(b.y for b in bounds) / len(bounds)
        world.spawn_actor(bp, carla.Transform(carla.Location(x=cx, y=cy, z=-0.3)))
    return world


def set_weather(world, sc):
    w = sc["weather"]
    if isinstance(w, str):
        weather = getattr(carla.WeatherParameters, w)
    else:
        weather = world.get_weather()
        for k, v in w.items():
            setattr(weather, k, v)
    if sc.get("sun_altitude") is not None:
        weather.sun_altitude_angle = sc["sun_altitude"]
    world.set_weather(weather)


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
    return world.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=1.2), carla.Rotation(yaw=yaw)))


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
    if "path" in bot:
        bot["path"] = to_carla_path(geo, bot["path"])
    return {"kind": part.kind, "view": part.cfg.get("view"), "bot": bot,
            "ehmi_states": list((tc.get("ehmi") or {}).get("states", DEFAULT_EHMI)),
            "step": tc["simulation"]["step"], "experiment": tc["name"]}


def run_trial(cfg, trial_idx, args, control=None):
    log = Log()
    trial = cfg["trials"][trial_idx]
    tc = config.trial_config(cfg, trial)
    for p in tc["participants"]:
        if args.input and not p.get("station") and p.get("kind", "driver") == "driver":
            p["input"] = args.input
        if args.headless:
            p["window"] = False
        if args.view:
            p["view"] = args.view
    mdir = config.map_dir(tc)
    if not (mdir / "meta.json").exists():
        mapbuild.build(tc)
    geo = Geo(mdir / "geo.net.xml")
    step = tc["simulation"]["step"]
    duration = float(trial.get("duration_s", 300))
    ehmi_states = (tc.get("ehmi") or {}).get("states", DEFAULT_EHMI)
    ehmi_slots = tuple((tc.get("ehmi") or {}).get("slots", ["front"]))
    coupling = tc.get("coupling") or {}

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(tc["_root"]) / tc["logging"]["dir"] / tc["name"] / f"{stamp}_trial{trial_idx}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.yaml").write_text(yaml.safe_dump({k: v for k, v in tc.items() if not k.startswith("_")},
                                                     sort_keys=False, allow_unicode=True))
    log(f"trial {trial_idx} -> {out}")

    cfg_file = traffic.prepare(tc, mdir, out / "sumo", duration)
    log("SUMO demand ready")
    parts = [Participant(p) for p in tc["participants"]]
    by_role = {p.role: p for p in parts}
    state = {"state": "loading", "trial": trial_idx}
    if control is not None:
        control.on_hello = lambda role: (
            {**welcome_for(by_role[role], geo, tc), **state, "actor_id": _actor_id(by_role.get(role))}
            if role in by_role else {"error": f"no participant with role {role}"})

    server = Server(tc["server"], log)
    client = server.start()
    sync = view = recorder = world = logger = None
    collisions, agents = [], []
    summary = {"trial": trial_idx, "ticks": 0}
    try:
        world = load_world(client, mdir / "map.xodr", tc["map"])
        set_weather(world, tc["simulation"])
        log(f"world ready: {len(world.get_map().get_spawn_points())} spawn points")

        sumo = SumoSimulation(str(cfg_file), step)
        carla_sim = CarlaSimulation(tc["server"]["host"], tc["server"]["port"], step)
        focus_actors = []   # SUMO actors appear in CARLA only near these (the participants)
        sync = SimulationSynchronization(
            sumo, carla_sim, tc["simulation"]["tls_manager"], sync_vehicle_lights=True,
            tm_port=tc["server"]["tm_port"] if tc["simulation"].get("traffic_manager") else None,
            radius=tc["simulation"]["lod_radius_m"],
            focus=lambda: [a.get_location() for a in focus_actors] or None)
        # The bridge assumes CARLA = SUMO - netOffset, true for CARLA's own towns. Our OpenDRIVE carries a UTM
        # georeference, so map.net.xml's netOffset is the whole UTM shift (~-671 km, -5701 km) and every SUMO
        # car would land thousands of km away (and moving the participant there crashes SUMO). The real shift
        # is the difference between map.net.xml's and geo.net.xml's offsets (CARLA x,y = geo.net x,-y).
        mo, go = sumo.net.getLocationOffset(), geo.net.getLocationOffset()
        BridgeHelper.offset = (mo[0] - go[0], mo[1] - go[1])
        signals = Signals(world, sumo.net, BridgeHelper.offset, step=step)
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

        def watch_collisions(actor, role):
            col = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                    carla.Transform(), attach_to=actor)
            col.listen(lambda e: collisions.append((e.timestamp, e.frame, role, e.other_actor.id,
                                                    e.other_actor.type_id)))

        for i, part in enumerate(parts):
            p = part.cfg
            if part.kind == "driver":
                part.actor = spawn_vehicle(world, geo, p, log)
                if not part.station:
                    part.input = inputs.make(p.get("input", "keyboard"), part.actor)
            elif part.kind == "pedestrian":
                part.actor = spawn_pedestrian(world, geo, p)
                if not part.station and (p.get("input") == "bot" or p.get("bot")):
                    bot = p.get("bot") or {}
                    pts = [carla.Location(x=x, y=y) for x, y in to_carla_path(geo, bot.get("path", []))]
                    part.bot = PathWalker(part.actor, pts, float(bot.get("speed", 1.4)), float(bot.get("start_s", 0)))
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
            if part.kind != "av_passenger":
                watch_collisions(part.actor, part.role)
            if not part.station and i == 0 and part.kind == "driver":
                window = p.get("window", part.input.needs_display)
                if window or tc["logging"]["frames_every_s"] > 0:
                    view = View(world, part.actor, p.get("view", "driver"), window=window, save_dir=out,
                                save_every_s=tc["logging"]["frames_every_s"])
            log(f"{part.role}: {part.kind} {part.actor.type_id} "
                f"{'at a station' if part.station else 'on the host (' + str(p.get('input', 'bot')) + ')'}")
        for ag in agents:
            if isinstance(ag, ScriptedCar):
                watch_collisions(ag.vehicle, ag.id)

        world.tick()
        for part in parts:   # BasicAgent planned from (0, 0, 0) before the first tick: plan from the spawn lane
            if hasattr(part.input, "restart"):
                part.input.restart(world.get_map().get_waypoint(part.actor.get_location()))
        walkers_all = [p.actor for p in parts if p.kind == "pedestrian"] + \
                      [a.walker for a in agents if isinstance(a, ScriptedWalker)]
        wc = tc["traffic"].get("walkers") or {}
        crowd = None
        if int(wc.get("count", 0)) > 0:
            crowd = Crowd(world, [p.actor.get_location() for p in parts], int(wc["count"]),
                          float(wc.get("radius_m", 120)), int(wc.get("seed", 1)), log=log)
            # not in walkers_all: they stay on the sidewalks, so no car has to yield to them
        sumo_yield = SumoYield()

        if tc["logging"]["carla_recorder"]:
            recorder = client.start_recorder(f"coupledcarla_{tc['name']}_{stamp}_trial{trial_idx}.log", True)
        logger = TrialLogger(out, geo, tc["logging"], step)

        # stations: wait for them, then tell each its actor and start
        station_roles = [p.role for p in parts if p.station]
        if station_roles:
            if control is None:
                raise RuntimeError("participants with station: true need the control channel (coupling.port)")
            missing = control.wait_for(station_roles, float(coupling.get("wait_for_stations_s", 600)))
            if missing:
                log(f"starting without stations: {', '.join(sorted(missing))}")
                logger.event(0.0, 0, "stations_missing", "", ",".join(sorted(missing)))
        state.update(state="running")
        for part in parts:
            if control is not None:
                control.send(part.role, {"type": "state", **state, "actor_id": part.actor.id})
        logger.event(0.0, 0, "trial_start", "", json.dumps({"trial": trial_idx, "stations": station_roles,
                                                            "agents": [a.id for a in agents]}))

        types = {}
        prof = {"sync": 0.0, "log": 0.0, "ticks0": 0}
        was_off, stuck_since, station_stats, station_gaze = {}, {}, {}, {}
        gaze = MutualGaze(logger, **(tc.get("eye_contact") or {}))
        sync_at = sorted(float(x) for x in coupling.get("visual_sync_at_s", []))
        t_wall0, sim0, next_status, seen_col, t = time.time(), None, 0.0, 0, 0.0
        snap = world.get_snapshot()
        while True:
            t_tick = time.time()
            events = view.events() if view else []
            for part in parts:            # host-local participants
                if part.station:
                    continue
                if part.input is not None:
                    ctl = part.input.control(part.actor, events)
                    if ctl is not None:
                        part.actor.apply_control(ctl)
                        part.ctl = ctl
                elif part.bot is not None:
                    part.bot.step(t)
                while part.ehmi_schedule and part.ehmi_schedule[0][0] <= t:
                    part.agent.set_ehmi(part.ehmi_schedule.pop(0)[1], part.role)
            for ag in agents:
                ag.step(t, step, walkers_all)
            if crowd is not None and summary["ticks"] % 4 == 0:     # a walker keeps its last control
                crowd.step(t)
            t_a = time.time()
            sync.tick()
            t_b = time.time()
            prof["sync"] += t_b - t_a
            sumo_yield.step(sync, world, walkers_all)
            signals.draw([a.get_location() for a in focus_actors])
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
                    if summary["ticks"] % 2 == 0 and ag.ehmi not in ("", "off"):
                        here = ag.vehicle.get_location()
                        near = [w.get_location() for w in walkers_all]
                        look = min(near, key=lambda l: l.distance(here)) if near else None
                        if look is not None and look.distance(here) > 40.0:
                            look = None
                        draw_ehmi(world.debug, ag.vehicle, ag.ehmi, ehmi_states, life=step * 3,
                                  slots=tuple(ag.cfg.get("ehmi_slots") or ehmi_slots),
                                  look_at=look + carla.Location(z=0.7) if look is not None else None)
            if summary["ticks"] % 5 == 0:
                gaze.update(t, snap.frame, parts, agents, station_gaze)
            snap = world.get_snapshot()
            if control is not None:          # every tick, for the stations' latency measurement
                control.broadcast({"type": "tick", "frame": carla_sim.tick_frame, "t": carla_sim.tick_wall})
            sim_t = snap.timestamp.elapsed_seconds
            sim0 = sim_t if sim0 is None else sim0
            if logger.sim0 is None:
                logger.sim0 = sim0
            t = sim_t - sim0
            summary["ticks"] += 1

            # stations
            if control is not None:
                for msg in control.poll():
                    kind, role = msg.get("type"), msg.get("role")
                    if kind == "ehmi" and role in by_role and by_role[role].agent:
                        by_role[role].agent.set_ehmi(msg.get("state", "off"), role)
                    elif kind == "gaze":
                        station_gaze[role] = msg.get("yaw")
                    elif kind == "sync":
                        sync_at.insert(0, t)
                    elif kind == "stats":
                        stats = {k: msg.get(k) for k in ("role", "rtt_ms", "fps", "frames", "host", "latency")}
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
                if cid not in types:
                    a = world.get_actor(cid)
                    tid = a.type_id if a else ""
                    kind = ("bike" if tid in tc["traffic"]["bike_types"] else
                            "bus" if tid == tc["traffic"]["bus_type"] else "car")
                    types[cid] = (kind, tid)
                    logger.event(t, snap.frame, "spawn", cid, f"{kind} {tid} sumo:{sid}")
                ai[cid] = types[cid]
                sa = sync.sumo.get_actor(sid)
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
            records = [p.record() for p in parts if p.kind != "av_passenger"]
            for ag in agents:
                if isinstance(ag, ScriptedCar):
                    records.append({"actor": ag.vehicle, "role": ag.id, "kind": "agent", "ctl": None,
                                    "light": ag.light, "ehmi": ag.ehmi})
                else:
                    records.append({"actor": ag.walker, "role": ag.id, "kind": "agent"})
                for name, detail in ag.events:
                    logger.event(t, snap.frame, name, ag.vehicle.id if isinstance(ag, ScriptedCar) else ag.walker.id,
                                 detail)
                ag.events.clear()
            for name, detail in sumo_yield.events:
                logger.event(t, snap.frame, name, "", detail)
            sumo_yield.events.clear()
            logger.log(snap, records, ai, sumo_signals, [])
            prof["log"] += time.time() - t_b
            for c in collisions[seen_col:]:
                logger.event(t, c[1], "collision", c[3], f"{c[2]} hit {c[4]}")
            seen_col = len(collisions)

            first = parts[0] if parts else None
            if view and first:
                v = first.actor.get_velocity()
                lt = first.light
                view.render([f"{3.6 * (v.x ** 2 + v.y ** 2) ** 0.5:5.1f} km/h", f"t {t:6.1f} s",
                             f"light {lt[2]} in {lt[1]:.0f} m" if lt else ""])
            if t >= next_status:
                rtf = t / max(1e-6, time.time() - t_wall0)
                n = max(1, summary["ticks"] - prof["ticks0"])
                speeds = "  ".join(f"{p.role} {3.6 * (p.actor.get_velocity().x ** 2 + p.actor.get_velocity().y ** 2) ** 0.5:4.1f}"
                                   for p in parts) or "-"
                log(f"t {t:6.1f}s  sumo {len(sync.sumo2carla_ids):3d}  km/h: {speeds}  RTF {rtf:.2f}  "
                    f"rows {logger.rows}  ms/tick sync {1000 * prof['sync'] / n:.1f} log {1000 * prof['log'] / n:.1f}")
                prof.update(sync=0.0, log=0.0, ticks0=summary["ticks"])
                next_status += 10.0
            if t >= duration or any(getattr(p.input, "quit", False) for p in parts):
                break
            for part in parts:            # recovery for host-local cars (the autopilot, or a human pressing R)
                ctl_in = part.input
                if part.kind != "driver" or ctl_in is None:
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
            if tc["simulation"]["real_time"]:
                time.sleep(max(0.0, step - (time.time() - t_tick)))
        wall = time.time() - t_wall0
        summary.update({"sim_s": round(t, 2), "wall_s": round(wall, 2), "rtf": round(t / max(wall, 1e-6), 3),
                        "rows": logger.rows, "collisions": len(collisions), "agents": len(agents),
                        "stations": station_roles, "sumo_vehicles_spawned": len(types),
                        "station_stats": station_stats})
        logger.event(t, snap.frame, "trial_end", "", json.dumps(summary))
        log(f"trial done: {summary}")
    finally:
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
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        if tc["logging"].get("analyse", True) and (out / "frames.csv").exists():
            try:
                rows = analysis.analyse_trial(out)
                log(f"analysis: {len(rows)} pedestrian-vehicle encounters in {out / 'analysis'}")
            except Exception as err:  # noqa: BLE001 - the trial's logs are written; analysis can be re-run
                log(f"analysis failed ({err}); run python -m coupledcarla.analysis {out}")
    return summary


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
    ap.add_argument("--quality", choices=["Low", "Epic"], help="override server.quality")
    ap.add_argument("--view", choices=["driver", "chase", "hood"], help="override every participant's view")
    ap.add_argument("--step", type=float, help="override simulation.step (s)")
    ap.add_argument("--local", action="store_true",
                    help="run every station participant on the host (their bot), for tests without stations")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
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
        for p in cfg["participants"]:
            if p.get("station"):
                p["station"] = False
                if p.get("kind", "driver") == "driver":
                    p["input"] = "autopilot"
    control = None
    if any(p.get("station") for p in cfg["participants"]):
        control = ControlServer(int((cfg.get("coupling") or {}).get("port", 2100)), Log())
    try:
        for i in args.trial or range(len(cfg["trials"])):
            run_trial(cfg, i, args, control)
    finally:
        if control is not None:
            control.close()


if __name__ == "__main__":
    main()
