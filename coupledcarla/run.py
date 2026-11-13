"""Run an experiment's trials: CARLA world + SUMO traffic + participants + logging, in one loop.

    python -m coupledcarla.run experiments/eindhoven_centre.yaml [--trial 0] [--input keyboard] [--headless]

Each tick: read each participant's input and apply it, then step SUMO and CARLA together through the bridge
(SUMO moves its vehicles in CARLA; CARLA-driven participants are mirrored into SUMO, so traffic reacts to
them), then log the frame and draw the view. With simulation.real_time the loop sleeps so that one simulated
second takes one wall-clock second.
"""
import argparse
import json
import random
import time
from pathlib import Path

import carla
import yaml

from . import config, inputs, mapbuild, traffic
from .bridge.synchronization import BridgeHelper, CarlaSimulation, SimulationSynchronization, SumoSimulation
from .display import View
from .geo import Geo, heading_to_yaw
from .logger import TrialLogger
from .signals import Signals
from .server import Server


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
        # OpenDRIVE-only world has no terrain,
        # so a car leaving the road mesh would otherwise fall out of the world.
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


def spawn_participant(world, geo, p, log):
    cmap = world.get_map()
    bp = world.get_blueprint_library().find(p.get("vehicle", "vehicle.lincoln.mkz_2020"))
    bp.set_attribute("role_name", p.get("role", "driver"))
    sp = p.get("spawn") or {}
    if "lat" in sp:
        x, y = geo.to_carla(sp["lat"], sp["lon"])
        wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
        log(f"{p['role']}: asked for {sp['lat']:.5f},{sp['lon']:.5f} -> lane {wp.road_id}/{wp.lane_id} "
            f"{wp.transform.location.distance(carla.Location(x=x, y=y, z=wp.transform.location.z)):.1f} m away")
        candidates = [wp.transform]
        if "heading" in sp:
            yaw = heading_to_yaw(sp["heading"])
            opp = wp.get_left_lane()
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


def spawn_walkers(world, n, seed):
    if n <= 0:
        return [], random.Random(seed)
    rng = random.Random(seed)
    lib = world.get_blueprint_library()
    walker_bps = lib.filter("walker.pedestrian.*")
    batch = []
    for _ in range(n):
        loc = world.get_random_location_from_navigation()
        if loc:
            bp = rng.choice(walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            batch.append(carla.command.SpawnActor(bp, carla.Transform(loc + carla.Location(z=1))))
    return batch, rng


def run_trial(cfg, trial_idx, args):
    log = Log()
    trial = cfg["trials"][trial_idx]
    tc = config.trial_config(cfg, trial)
    for p in tc["participants"]:
        if args.input:
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

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(tc["_root"]) / tc["logging"]["dir"] / tc["name"] / f"{stamp}_trial{trial_idx}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.yaml").write_text(yaml.safe_dump({k: v for k, v in tc.items() if not k.startswith("_")},
                                                     sort_keys=False, allow_unicode=True))
    log(f"trial {trial_idx} -> {out}")

    cfg_file = traffic.prepare(tc, mdir, out / "sumo", duration)
    log("SUMO demand ready")
    server = Server(tc["server"], log)
    client = server.start()
    sync = view = recorder = world = logger = None
    actors, walker_ctrls, collisions = [], [], []
    summary = {"trial": trial_idx, "ticks": 0}
    try:
        world = load_world(client, mdir / "map.xodr", tc["map"])
        set_weather(world, tc["simulation"])
        log(f"world ready: {len(world.get_map().get_spawn_points())} spawn points")

        sumo = SumoSimulation(str(cfg_file), step)
        carla_sim = CarlaSimulation(tc["server"]["host"], tc["server"]["port"], step)
        focus_actors = []   # participants' vehicles; SUMO actors appear in CARLA only near them
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
        log(f"SUMO->CARLA shift {BridgeHelper.offset[0]:.2f}, {BridgeHelper.offset[1]:.2f} m")
        signals = Signals(world, sumo.net, BridgeHelper.offset, step=step)
        log(f"traffic lights: {len(signals.heads)} signalled lanes from SUMO drawn near participants")
        world.tick()

        parts = []
        for i, p in enumerate(tc["participants"]):
            veh = spawn_participant(world, geo, p, log)
            actors.append(veh)
            ctl_in = inputs.make(p.get("input", "keyboard"), veh)
            window = p.get("window", ctl_in.needs_display) and i == 0
            if window or (i == 0 and tc["logging"]["frames_every_s"] > 0):
                view = View(world, veh, p.get("view", "driver"), window=window, save_dir=out,
                            save_every_s=tc["logging"]["frames_every_s"])
            col = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                    carla.Transform(), attach_to=veh)
            col.listen(lambda e, role=p["role"]: collisions.append((e.timestamp, e.frame, role, e.other_actor.id,
                                                                    e.other_actor.type_id)))
            actors.append(col)
            parts.append([veh, p["role"], ctl_in, carla.VehicleControl(), None])
            focus_actors.append(veh)
            log(f"{p['role']}: {veh.type_id} at {veh.get_location()} with {p.get('input')}")

        world.tick()
        for part in parts:   # BasicAgent planned from (0, 0, 0) before the first tick: plan from the spawn lane
            if hasattr(part[2], "restart"):
                part[2].restart(world.get_map().get_waypoint(part[0].get_location()))
        batch, rng = spawn_walkers(world, tc["traffic"]["walkers"], tc["traffic"]["seed"])
        results = client.apply_batch_sync(batch, True)
        walkers = [r.actor_id for r in results if not r.error]
        errors = sorted({r.error for r in results if r.error})
        if len(walkers) < tc["traffic"]["walkers"]:
            log(f"pedestrians: {len(batch)} navigation points for {tc['traffic']['walkers']} asked, "
                f"{len(walkers)} spawned; errors: {errors[:3]}")
        ctrl_bp = world.get_blueprint_library().find("controller.ai.walker")
        walker_ctrls = [r.actor_id for r in client.apply_batch_sync(
            [carla.command.SpawnActor(ctrl_bp, carla.Transform(), w) for w in walkers], True) if not r.error]
        world.tick()
        world.set_pedestrians_cross_factor(0.05)
        for cid in walker_ctrls:
            c = world.get_actor(cid)
            c.start()
            c.go_to_location(world.get_random_location_from_navigation())
            c.set_max_speed(1.2 + rng.random() * 0.5)
        walker_ids = {w: ("walker", world.get_actor(w).type_id) for w in walkers}
        log(f"{len(walkers)} pedestrians walking")

        if tc["logging"]["carla_recorder"]:
            recorder = client.start_recorder(f"coupledcarla_{tc['name']}_{stamp}_trial{trial_idx}.log", True)
        logger = TrialLogger(out, geo, tc["logging"], step)
        tls = []   # CARLA light actors are inert in OpenDRIVE worlds; lights are logged from SUMO (signals.py)
        logger.event(0.0, 0, "trial_start", "", json.dumps({"trial": trial_idx, "traffic_lights": len(tls)}))

        types = {}
        prof = {"sync": 0.0, "log": 0.0, "ticks0": 0}
        was_off, stuck_since = {}, {}
        t_wall0, sim0, next_status, seen_col = time.time(), None, 0.0, 0
        while True:
            t_tick = time.time()
            events = view.events() if view else []
            for part in parts:
                veh, role, ctl_in = part[:3]
                ctl = ctl_in.control(veh, events)
                if ctl is not None:
                    veh.apply_control(ctl)
                    part[3] = ctl
            t_a = time.time()
            sync.tick()
            t_b = time.time()
            prof["sync"] += t_b - t_a
            signals.draw([p[0].get_location() for p in parts])
            for part in parts:
                sid = sync.carla2sumo_ids.get(part[0].id)
                part[4] = signals.next_light(sid) if sid else None
                if hasattr(part[2], "light"):
                    part[2].light = part[4]
            snap = world.get_snapshot()
            sim_t = snap.timestamp.elapsed_seconds
            sim0 = sim_t if sim0 is None else sim0
            t = sim_t - sim0
            summary["ticks"] += 1

            ai = dict(walker_ids)
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
            for part in parts:   # an event whenever the light a participant faces changes
                key = (part[4][0], part[4][2]) if part[4] else None
                if key != (part[5] if len(part) > 5 else None):
                    if key:
                        logger.event(t, snap.frame, "next_light", part[0].id,
                                     f"{part[1]} {key[0]} {key[1]} at {part[4][1]:.1f} m")
                    part[5:] = [key]
            logger.log(snap, [(p[0], p[1], p[3], p[4]) for p in parts], ai, sumo_signals, tls)
            prof["log"] += time.time() - t_b
            for c in collisions[seen_col:]:
                logger.event(t, c[1], "collision", c[3], f"{c[2]} hit {c[4]}")
            seen_col = len(collisions)

            if view:
                v = parts[0][0].get_velocity()
                lt = parts[0][4]
                view.render([f"{3.6 * (v.x ** 2 + v.y ** 2) ** 0.5:5.1f} km/h", f"t {t:6.1f} s",
                             f"light {lt[2]} in {lt[1]:.0f} m" if lt else ""])
            if t >= next_status:
                v = parts[0][0].get_velocity() if parts else carla.Vector3D()
                rtf = t / max(1e-6, time.time() - t_wall0)
                n = max(1, summary["ticks"] - prof["ticks0"])
                log(f"t {t:6.1f}s  sumo vehicles {len(sync.sumo2carla_ids):3d}  "
                    f"ego {3.6 * (v.x ** 2 + v.y ** 2) ** 0.5:5.1f} km/h  RTF {rtf:.2f}  rows {logger.rows}  "
                    f"ms/tick sync {1000 * prof['sync'] / n:.1f} log {1000 * prof['log'] / n:.1f}")
                prof.update(sync=0.0, log=0.0, ticks0=summary["ticks"])
                lt = parts[0][4] if parts else None
                if lt:
                    log(f"           next light {lt[0]} {lt[2]} in {lt[1]:.0f} m")
                next_status += 10.0
            if t >= duration or any(p[2].quit for p in parts):
                break
            for part in parts:
                veh, ctl_in = part[0], part[2]
                off_road = veh.get_location().z < -0.25          # down on the ground plane
                if off_road and not was_off.get(veh.id):
                    logger.event(t, snap.frame, "left_road", veh.id, part[1])
                was_off[veh.id] = off_road
                # an autopilot pressing the throttle without moving (wedged on a road edge) counts as stuck
                v = veh.get_velocity()
                if isinstance(ctl_in, inputs.Autopilot) and part[3].throttle > 0.3 and (v.x ** 2 + v.y ** 2) < 0.1:
                    stuck_since.setdefault(veh.id, t)
                    if t - stuck_since[veh.id] > 3.0:
                        logger.event(t, snap.frame, "stuck", veh.id, part[1])
                        ctl_in.stuck = True
                        stuck_since.pop(veh.id)
                else:
                    stuck_since.pop(veh.id, None)
                # Put the car back on the nearest lane, facing along it: the autopilot when it left the road or
                # ran out of lane, a human by pressing R.
                if getattr(ctl_in, "stuck", False) or getattr(ctl_in, "reset", False) or \
                        (off_road and isinstance(ctl_in, inputs.Autopilot)):
                    wp = world.get_map().get_waypoint(veh.get_location(), project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                    tf = wp.transform
                    tf.location.z += 0.4
                    veh.set_target_velocity(carla.Vector3D())
                    veh.set_transform(tf)
                    ctl_in.stuck = ctl_in.reset = False
                    if hasattr(ctl_in, "restart") and not ctl_in.restart(wp):
                        ctl_in.stuck = True   # nowhere to go from here: try again next tick
                    logger.event(t, snap.frame, "reset_to_lane", veh.id, f"{part[1]} road {wp.road_id} lane {wp.lane_id}")
            if parts and parts[0][0].get_location().z < -5:
                logger.event(t, snap.frame, "fell_off_map", parts[0][0].id)
                log("participant fell off the map")
                break
            if tc["simulation"]["real_time"]:
                time.sleep(max(0.0, step - (time.time() - t_tick)))
        wall = time.time() - t_wall0
        summary.update({"sim_s": round(t, 2), "wall_s": round(wall, 2), "rtf": round(t / wall, 3),
                        "rows": logger.rows, "collisions": len(collisions), "pedestrians": len(walkers),
                        "sumo_vehicles_spawned": len(types)})
        logger.event(t, snap.frame, "trial_end", "", json.dumps(summary))
        log(f"trial done: {summary}")
    finally:
        if logger is not None:
            logger.close()
        if recorder is not None:
            client.stop_recorder()
        if view:
            view.destroy()
        for cid in walker_ctrls if world else []:
            c = world.get_actor(cid)
            if c:
                c.stop()
        if sync:
            try:
                sync.close()
            except Exception as err:  # noqa: BLE001
                log(f"bridge close: {err}")
        server.stop()
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main():
    import signal
    import sys
    # A plain kill (SIGTERM) runs the clean-up too, so the CARLA server and SUMO do not outlive the runner.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--trial", type=int, action="append", help="trial index (repeatable); default: all")
    ap.add_argument("--input", choices=["autopilot", "keyboard", "wheel"], help="override every participant's input")
    ap.add_argument("--headless", action="store_true", help="no window (autopilot input)")
    ap.add_argument("--duration", type=float, help="override each trial's duration_s")
    ap.add_argument("--frames-every", type=float, help="save the first participant's view every N s")
    ap.add_argument("--quality", choices=["Low", "Epic"], help="override server.quality")
    ap.add_argument("--view", choices=["driver", "chase", "hood"], help="override every participant's view")
    ap.add_argument("--step", type=float, help="override simulation.step (s)")
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
    for i in args.trial or range(len(cfg["trials"])):
        run_trial(cfg, i, args)


if __name__ == "__main__":
    main()
