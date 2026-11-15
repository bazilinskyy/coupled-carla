"""Drones in a coupled-carla trial: a participant who flies one (kind `drone`) and AI drones (the `drones` section),
all flown by the quadrotor model of dronemodel.py (coupled-sim's, ported) and integrated on the host.

Why on the host: the drone's motion is not CARLA physics but the flight model's, so whoever integrates it has to
send its pose to CARLA every tick, and the host already sends one batch per tick (CarlaSimulation.queue). On the
host the drone, its crash checks against CARLA geometry, the AI drones that keep away from it, the logs and the
station's gimbal camera all use the same state of the same tick: the camera is moved in the tick's batch, so the
picture and the drone never disagree. A station sends its sticks over the control channel every frame (round trip
0.3-23 ms on the lab LAN, README), so a stick input reaches the drone at the next host tick: at most one tick
(20 ms) plus half a round trip later, the same order as a driver station's control reaching CARLA's physics.

Looks (`drones.visual`):
- `blueprint` (CARLA 1.0 with coupled-carla's content pack): BP_CCDrone_* actors (blueprint.cc_drone_consumer,
  _industrial and their _parked variants), a detailed quadrotor with spinning propellers (blur discs and blades
  turned in the material), red and green navigation lights and a white anti-collision strobe; no collision. A
  landed drone shows its parked variant (propellers still, lights dim) and the flying one waits under the ground.
- `mesh`: the same mesh (SM_CCDrone_*) as a static.prop.mesh with a mass (Movable), physics and collisions then
  switched off so ApplyTransform moves it (any server that has the mesh cooked).
- `shapes` (CARLA 0.9.16, which has no drone and whose static.prop.mesh props cannot be moved unless spawned with a
  mass): a body and four motors from /Engine/BasicShapes moved as one, plus arms, rotor discs and lights drawn as
  debug lines near the participants.
- `auto`: the first of these the server has. `none`: no actors (logs only).
"""
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

from . import dronemodel as dm
from . import droneplan as plan
from .droneinput import BotPilot, Hold, Sticks, VIEWS, pilot_command

PACK_BLUEPRINT = "blueprint.cc_drone_{}"
MESH_PATH = "/Game/CoupledCarla/Drones/SM_CCDrone_{0}.SM_CCDrone_{0}"
GROUND_LABELS = {"Roads", "Sidewalks", "Ground", "Terrain", "RoadLines", "Water", "Rail", "Bridge", "Static", "NONE",
                 "Other"}
LANDABLE = GROUND_LABELS | {"Buildings", "Walls", "GuardRail"}      # a drone can set down on them (roofs too)
HIDE_BELOW_M = 50.0          # a hidden variant waits this far under the drone
LOG_COLUMNS = ["timestamp", "frame", "drone", "kind", "role", "mission", "airframe", "phase", "mode", "pos_x",
               "pos_y", "pos_z", "lat", "lon", "alt_agl", "vel_x", "vel_y", "vel_z", "speed_kmh", "heading", "pitch",
               "roll", "rotor1", "rotor2", "rotor3", "rotor4", "blade_passing_hz", "thrust_ratio", "power_w",
               "battery", "stick_climb", "stick_yaw", "stick_forward", "stick_lateral", "gimbal_pitch", "view",
               "crashed", "actor_id"]


# -- frames ------------------------------------------------------------------------------------------------------

def to_carla(p):
    """Model (x east, y north, z up) -> CARLA (x east, y south, z up)."""
    return float(p[0]), -float(p[1]), float(p[2])


def from_carla(x, y, z=0.0):
    return np.array([x, -y, z], float)


def yaw_from_heading(heading_deg):
    """Compass heading (0 north, 90 east) -> model yaw (rad from east towards north)."""
    return math.radians(90.0 - float(heading_deg))


def rotation_deg(r):
    """CARLA (pitch, yaw, roll) in degrees of a model rotation matrix r (columns body x, y, z)."""
    f, l, u = r[:, 0], r[:, 1], r[:, 2]
    yaw = math.degrees(math.atan2(-f[1], f[0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, f[2]))))
    roll = math.degrees(math.atan2(l[2], u[2]))
    return pitch, yaw, roll


def wind_at(w10, height_agl, z0=1.0):
    """Mean wind (m/s) at a height from the wind at 10 m, by the log profile over a city (roughness z0 = 1 m)."""
    h = max(z0 * 1.5, float(height_agl))
    return float(w10) * math.log(h / z0) / math.log(10.0 / z0)


def sound_level(lwa, thrust_ratio, d):
    """A-weighted level (dB) at distance d (m) of a drone with sound power lwa (dB re 1 pW) in hover, over the ground
    (hemispherical spreading: Lp = Lw - 20 lg r - 8, ISO 3744's free field over a reflecting plane), scaled with the
    thrust as rotor loading noise is with the tip speed (+25 lg T/T_hover, i.e. 50 lg of the rotor speed)."""
    t = max(0.05, float(thrust_ratio))
    return lwa + max(-15.0, min(10.0, 25.0 * math.log10(t))) - 20.0 * math.log10(max(1.0, d)) - 8.0


# -- obstacles from the map --------------------------------------------------------------------------------------

def load_obstacles(mdir, tc, ground_z, log=print, geo=None):
    """plan.Obstacles from the map's buildings (buildings.json; for a baked level without one, the scenery's
    building sources, cached in drone_obstacles.json) and trees (scenery.json with the mesh catalogue's sizes).
    ground_z(x, y) (CARLA) gives the ground of a building without a base height. Fails soft (no obstacles)."""
    mdir = Path(mdir)
    recs = []
    src = "none"
    try:
        path = mdir / "buildings.json"
        cache = mdir / "drone_obstacles.json"
        if path.exists():
            raw = json.loads(path.read_text())
            src = "buildings.json"
        elif cache.exists():
            raw = json.loads(cache.read_text())
            src = "drone_obstacles.json"
        else:
            raw = []
            if geo is not None:
                from . import scenery
                blds, used = scenery.building_records(tc["map"], mdir / "real", geo, None, 0.0, None, log)
                raw = [{"h": b["h"], "z": None, "k": b["cls"],
                        "p": [[round(x, 2), round(y, 2)] for x, y in b["poly"].exterior.coords]} for b in blds]
                src = "+".join(used)
            for b in raw:
                if b.get("z") is None:
                    c = np.mean(np.asarray(b["p"], float), axis=0)
                    b["z"] = round(float(ground_z(c[0], c[1])), 2)
            if raw:
                cache.write_text(json.dumps(raw, separators=(",", ":")))
        for b in raw:
            recs.append({"p": b["p"], "top": float(b.get("z") or 0.0) + float(b.get("h") or 8.0), "k": b.get("k")})
    except Exception as err:  # noqa: BLE001 - no building data: drones plan without it
        log(f"drones: no building data ({type(err).__name__}: {err}); cruise heights from clearance alone")
        recs = []
    trees = []
    try:
        sj = mdir / "scenery.json"
        if sj.exists():
            from .scenery import Catalog
            s = json.loads(sj.read_text())
            cat = Catalog(s.get("catalog", "0.9.16"))
            meshes = s["meshes"]
            for it in s["items"]:
                if it[6] != "tree":
                    continue
                try:
                    L, W, H = cat.size(meshes[it[0]])
                except Exception:  # noqa: BLE001
                    L, W, H = 6.0, 6.0, 12.0
                sc = float(it[5])
                trees.append((it[1], it[2], it[3] + H * sc, max(1.0, max(L, W) * sc / 2)))
    except Exception as err:  # noqa: BLE001
        log(f"drones: trees not read ({type(err).__name__}: {err})")
    obs = plan.Obstacles(recs, trees)
    log(f"drones: obstacles {len(recs)} buildings ({src}), {len(trees)} trees, highest "
        f"{np.max(obs.top) if np.isfinite(np.max(obs.top)) else 0:.1f} m")
    return obs


# -- looks ---------------------------------------------------------------------------------------------------------

class Visual:
    """The actors that show one drone and their per-tick transforms (into the tick's batch)."""

    def __init__(self, mode, actors, parked=None, parts=None, size=1.0):
        self.mode, self.actors, self.parked, self.parts, self.size = mode, actors, parked, parts or [], size
        self.showing_parked = None

    @property
    def actor(self):
        return self.actors[0] if self.actors else None

    def commands(self, carla, pos_c, rot_deg, landed):
        """carla.command.ApplyTransform list for this tick."""
        out = []
        if not self.actors:
            return out
        x, y, z = pos_c
        pitch, yaw, roll = rot_deg
        tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
        hidden = carla.Transform(carla.Location(x=x, y=y, z=z - HIDE_BELOW_M), carla.Rotation(yaw=yaw))
        if self.parked is not None:
            show_parked = bool(landed)
            if show_parked:
                out.append(carla.command.ApplyTransform(self.parked.id, tf))
                if self.showing_parked is not True:
                    out.append(carla.command.ApplyTransform(self.actors[0].id, hidden))
            else:
                out.append(carla.command.ApplyTransform(self.actors[0].id, tf))
                if self.showing_parked is not False:
                    out.append(carla.command.ApplyTransform(self.parked.id, hidden))
            self.showing_parked = show_parked
            return out
        if self.mode == "shapes":
            cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            for actor, (dx, dy, dz) in zip(self.actors, self.parts):
                out.append(carla.command.ApplyTransform(actor.id, carla.Transform(
                    carla.Location(x=x + dx * cy - dy * sy, y=y + dx * sy + dy * cy, z=z + dz),
                    carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))))
            return out
        out.append(carla.command.ApplyTransform(self.actors[0].id, tf))
        return out

    def destroy(self, carla, client):
        ids = [a.id for a in self.actors] + ([self.parked.id] if self.parked is not None else [])
        if ids:
            try:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
            except RuntimeError:
                pass


def available_mode(world, wanted, log=print):
    """The look this server can show: blueprint, mesh, shapes or none."""
    lib = world.get_blueprint_library()
    if wanted in ("none", None, False):
        return "none"
    if wanted in ("auto", "blueprint"):
        if lib.filter(PACK_BLUEPRINT.format("consumer")):
            return "blueprint"
        if wanted == "blueprint":
            log("drones: no blueprint.cc_drone_* on this server (CARLA 1.0 with the content pack and "
                "/Game/CoupledCarla/Drones cooked); trying the mesh")
    if wanted in ("auto", "blueprint", "mesh"):
        try:
            bp = lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", MESH_PATH.format("Consumer"))
            if wanted == "mesh" or _mesh_exists(world, bp):
                return "mesh"
        except (IndexError, RuntimeError):
            pass
    return "shapes"


def _mesh_exists(world, bp):
    """Spawns the mesh once far away; an actor without a mesh (not cooked) has no extent."""
    import carla
    a = world.try_spawn_actor(bp, carla.Transform(carla.Location(z=-500.0)))
    if a is None:
        return False
    try:
        e = a.bounding_box.extent
        ok = e.x > 0.01 or e.y > 0.01
    finally:
        a.destroy()
    return ok


def spawn_visual(world, mode, af, at_c, yaw_deg, role, log=print):
    """Actors for one drone in mode (blueprint | mesh | shapes | none)."""
    import carla
    lib = world.get_blueprint_library()
    x, y, z = at_c
    tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw_deg))
    name = af.name.lower()

    def kinematic(actor):
        actor.set_simulate_physics(False)
        actor.set_collisions(False)
        return actor
    try:
        if mode == "blueprint":
            bp = lib.find(PACK_BLUEPRINT.format(name))
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", role)
            fly = world.spawn_actor(bp, tf)
            parked = None
            if lib.filter(PACK_BLUEPRINT.format(name + "_parked")):
                pb = lib.find(PACK_BLUEPRINT.format(name + "_parked"))
                parked = world.spawn_actor(pb, carla.Transform(carla.Location(x=x, y=y, z=z - HIDE_BELOW_M), tf.rotation))
            return Visual(mode, [fly], parked, size=af.size)
        if mode == "mesh":
            bp = lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", MESH_PATH.format(af.name))
            bp.set_attribute("mass", "1.0")
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", role)
            return Visual(mode, [kinematic(world.spawn_actor(bp, tf))], size=af.size)
        if mode == "shapes":
            s = af.size
            parts, actors = [(0.0, 0.0, 0.06 * s)], []
            for k, a in enumerate(dm.ANGLES):      # motors at the arm tips (model x forward, y left -> CARLA y right)
                parts.append((af.arm * math.cos(a), -af.arm * math.sin(a), 0.07 * s))
            for k, (dx, dy, dz) in enumerate(parts):
                bp = lib.find("static.prop.mesh")
                if k == 0:
                    bp.set_attribute("mesh_path", "/Engine/BasicShapes/Cube.Cube")
                    bp.set_attribute("scale", f"{0.16 * s:.3f}")
                else:
                    bp.set_attribute("mesh_path", "/Engine/BasicShapes/Cylinder.Cylinder")
                    bp.set_attribute("scale", f"{0.04 * s:.3f}")
                bp.set_attribute("mass", "1.0")
                actors.append(kinematic(world.spawn_actor(bp, carla.Transform(
                    carla.Location(x=x + dx, y=y + dy, z=z + dz), tf.rotation))))
            return Visual(mode, actors, parts=parts, size=s)
    except (RuntimeError, IndexError) as err:
        log(f"drones: {mode} look for {role} failed ({err}); no actor")
    return Visual("none", [], size=af.size)


def draw_wireframe(debug, carla, af, pos_c, r_c, t, life, lights=True):
    """Arms, rotor discs, landing legs and navigation lights as debug lines (the 0.9.16 look and a fallback)."""
    x, y, z = pos_c
    fwd, right, up = r_c

    def at(dx, dy, dz):
        return carla.Location(x=x + fwd[0] * dx + right[0] * dy + up[0] * dz,
                              y=y + fwd[1] * dx + right[1] * dy + up[1] * dz,
                              z=z + fwd[2] * dx + right[2] * dy + up[2] * dz)
    dark, grey = carla.Color(30, 30, 34), carla.Color(150, 150, 155)
    s = af.size
    th = 0.012 * s
    for k, a in enumerate(dm.ANGLES):
        tx, ty = af.arm * math.cos(a), -af.arm * math.sin(a)
        debug.draw_line(at(0, 0, 0.06 * s), at(tx, ty, 0.07 * s), thickness=th, color=dark, life_time=life)
        pts = [at(tx + af.rotor_radius * math.cos(b), ty + af.rotor_radius * math.sin(b), 0.09 * s)
               for b in np.linspace(0, 2 * math.pi, 9)]
        for p, q in zip(pts[:-1], pts[1:]):
            debug.draw_line(p, q, thickness=0.004 * s, color=grey, life_time=life)
        if lights:
            col = carla.Color(255, 20, 10) if k in (0, 3) else carla.Color(20, 255, 40)
            debug.draw_point(at(tx, ty, 0.055 * s), size=0.025 * s, color=col, life_time=life)
    if lights and (t % 1.0) < 0.08:       # anti-collision strobe: a flash a second
        debug.draw_point(at(0, 0, 0.1 * s), size=0.05 * s, color=carla.Color(255, 255, 255), life_time=life)


# -- the drones of a trial -----------------------------------------------------------------------------------------

class Pilot:
    """A participant's drone."""

    def __init__(self, part, idx, af, home, home_ground, yaw):
        cfg = part.cfg
        self.part, self.idx, self.af = part, idx, af
        self.role = part.role
        self.mode = cfg.get("mode", "position")
        self.view = cfg.get("view", "fpv") if cfg.get("view", "fpv") in VIEWS else "fpv"
        self.gimbal = float(cfg.get("gimbal_pitch_deg", -12.0))
        self.hold = Hold()
        self.home, self.home_ground = np.asarray(home, float), float(home_ground)
        gf = cfg.get("geofence") or {}
        self.fence_r = float(gf.get("radius_m", 0) or 0)
        self.fence_alt = float(gf.get("max_altitude_m", 120.0) or 0)     # EU open category: 120 m
        self.sticks = Sticks()
        self.sticks_at = None          # wall time of a station's last sticks
        self.crashed = False
        self.crashes = 0
        self.rth = False
        self.fenced = False
        self.battery_warned = set()
        self.ground = home_ground
        self.camera_id = None          # a station's camera, moved by the host
        self.cam_pos = None            # chase camera, smoothed
        self.input = None              # host-local PygameInput or BotPilot
        self.los = None                # the pilot's eye position (CARLA) for the line-of-sight view
        self.was_ground = True
        self.yaw0 = yaw
        self.quit = False


class DroneWorld:
    """Pilot drones (participants of kind `drone`) and AI drones (the trial's `drones.missions`), the flight model
    stepping them all, their looks, crash checks, logs and the stations' cameras and sound. The run loop calls
    add_pilot() while spawning participants, start() after the first tick, step() every tick before the CARLA tick
    and log_frame() after the frame's logging."""

    def __init__(self, world, client, geo, tc, mdir, out, log=print, side_debug=None):
        import carla
        self.carla, self.world, self.client, self.geo, self.tc = carla, world, client, geo, tc
        self.cfg = tc.get("drones") or {}
        self.mdir, self.out, self.log = Path(mdir), Path(out), log
        self.debug = side_debug
        self.fleet = dm.Fleet()
        self.gusts = dm.Dryden(0, seed=int(self.cfg.get("seed", 1)))
        self.visuals, self.names, self.kinds = [], [], []      # per fleet index
        self.missions, self.pilots = [], []
        self.mission_of = {}
        self.queue = None
        self.events = []        # (name, actor id, detail) for events.csv
        self.mode = available_mode(world, self.cfg.get("visual", "auto"), log)
        self.obs = None
        self.cmap = world.get_map()
        self.w10, self.wind_dir = 0.0, 270.0
        self.prev = np.zeros((0, 3))
        self.log_f = None
        self.log_every = max(1, round(float(self.cfg.get("log_every_s", 0.1)) / tc["simulation"]["step"]))
        self.ticks = 0
        self.cost = 0.0
        self.bb_cache = {}      # actor id -> (type_id, extent) or None, for crashes into vehicles and walkers
        self.view = None
        self.cameras = []       # observer cameras (drones.cameras)
        self.ground_cache = {}
        log(f"drones: look {self.mode}")

    # -- set-up ---------------------------------------------------------------------------------------------------
    def surface(self, x, y, z_from, depth=600.0):
        """The highest surface a drone can set down on below (x, y, z_from) (CARLA): the ground, a road or a roof,
        not a tree's canopy, a pole or a car (those are obstacles: flying into them is a crash). None if CARLA has
        nothing there (a World Partition cell that is not loaded)."""
        carla = self.carla
        try:
            hits = self.world.cast_ray(carla.Location(x=x, y=y, z=z_from), carla.Location(x=x, y=y, z=z_from - depth))
        except (RuntimeError, AttributeError):
            hits = None
        if hits is None:
            try:
                lp = self.world.ground_projection(carla.Location(x=x, y=y, z=z_from), depth)
                return lp.location.z if lp is not None else None
            except (RuntimeError, AttributeError):
                return None
        land = [h.location.z for h in hits if str(h.label).split(".")[-1] in LANDABLE]
        return max(land) if land else None

    def ground_z(self, x, y):
        """Ground (or roof) height at CARLA (x, y): the landable surface from 300 m up, else the nearest lane."""
        key = (round(x, 1), round(y, 1))
        if key in self.ground_cache:
            return self.ground_cache[key]
        carla = self.carla
        z = self.surface(x, y, 300.0)
        if z is None:
            wp = self.cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True,
                                        lane_type=carla.LaneType.Any)
            z = wp.transform.location.z if wp is not None else 0.0
        self.ground_cache[key] = z
        return z

    def road_z(self, x, y):
        carla = self.carla
        wp = self.cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Any)
        return wp.transform.location.z if wp is not None else 0.0

    def _add(self, af, pos_m, yaw, name, kind, role):
        i = self.fleet.add(af, pos_m, yaw)
        self.gusts.resize(self.fleet.n)
        pc = to_carla(pos_m)
        vis = spawn_visual(self.world, self.mode, af, (pc[0], pc[1], pc[2] + 0.02), -math.degrees(yaw), role,
                           self.log) if self.mode != "none" else Visual("none", [], size=af.size)
        self.visuals.append(vis)
        self.names.append(name)
        self.kinds.append(kind)
        self.prev = np.vstack([self.prev, np.asarray(pos_m, float)[None]])
        aid = vis.actor.id if vis.actor is not None else ""
        self.events.append(("drone_spawn", aid, f"{name} {kind} {af.name} at {pc[0]:.1f},{pc[1]:.1f},{pc[2]:.1f}"))
        return i

    def add_pilot(self, part):
        """The participant's drone at its spawn point (on the ground, heading from spawn.heading); returns its
        actor (None with visual none)."""
        cfg = part.cfg
        af = dm.preset(cfg.get("airframe", "consumer"))
        sp = cfg.get("spawn") or {}
        if "lat" in sp:
            x, y = self.geo.to_carla(sp["lat"], sp["lon"])
        else:
            x, y = 0.0, 0.0
        g = self.ground_z(x, y)
        yaw = yaw_from_heading(sp.get("heading", 0.0))
        i = self._add(af, from_carla(x, y, g), yaw, part.role, "drone", part.role)
        if self.visuals[i].actor is None:       # the run loop needs an actor for a participant: the 0.9 shapes
            self.visuals[i] = spawn_visual(self.world, "shapes", af, (x, y, g + 0.02), -math.degrees(yaw),
                                           part.role, self.log)
        p = Pilot(part, i, af, from_carla(x, y, g), g, yaw)
        # the pilot stands `pilot_offset_m` behind the take-off point (line of sight), eyes at 1.7 m
        pp = cfg.get("pilot_position") or {}
        if "lat" in pp:
            lx, ly = self.geo.to_carla(pp["lat"], pp["lon"])
        else:
            off = float(cfg.get("pilot_offset_m", 3.0))
            lx, ly = x - off * math.cos(-yaw), y - off * math.sin(-yaw)
        p.los = (lx, ly, self.ground_z(lx, ly) + 1.7)
        self.fleet.ground[i] = g
        if not part.station:
            src = cfg.get("input", "keyboard")
            # a headless run makes keyboard participants "autopilot" (run.py): for a drone that is its bot
            if src in ("bot", "autopilot") or (cfg.get("bot") and src not in ("keyboard", "gamepad", "rc")):
                p.input = self._bot(cfg.get("bot") or {}, af)
        self.pilots.append(p)
        part.drone = p
        return self.visuals[i].actor

    def _bot(self, bot, af):
        path = []
        for pt in bot.get("path") or []:
            x, y = self.geo.to_carla(pt[0], pt[1])
            h = float(pt[2]) if len(pt) > 2 else 20.0
            path.append((x, -y, self.road_z(x, y) + h if not bot.get("absolute") else h))     # above the street
        return BotPilot(path, float(bot.get("speed", 6.0)), float(bot.get("start_s", 2.0)), bot.get("sticks"),
                        bool(bot.get("land", False)), af.max_speed, af.max_climb,
                        after_crash=bot.get("after_crash", "stop"))

    def local_input(self, pilot, window):
        """A keyboard, gamepad or RC transmitter at the host's window for a host-local pilot; without a window
        (--headless) its bot, if it has one."""
        cfg = pilot.part.cfg
        src = cfg.get("input", "keyboard")
        if pilot.input is not None:
            return
        if window and src in ("keyboard", "gamepad", "rc"):
            from .droneinput import PygameInput
            pilot.input = PygameInput(src, cfg)
        elif cfg.get("bot"):
            pilot.input = self._bot(cfg["bot"], pilot.af)
            self.log(f"drones: {pilot.role} flown by its bot (no window)")
        else:
            self.log(f"drones: {pilot.role} has no input (no window, no bot): it stays on the ground")

    def speed_kmh(self, part):
        p = getattr(part, "drone", None)
        return 3.6 * float(np.linalg.norm(self.fleet.vel[p.idx])) if p is not None else None

    def start(self, participants_locs=()):
        """After the first tick: the obstacles, the wind and the AI missions."""
        t0 = time.time()
        self.obs = load_obstacles(self.mdir, self.tc, self.road_z, self.log, self.geo)
        w = self.cfg.get("wind_mps", "auto")
        self.w10 = self._wind(w)
        self.wind_dir = float(self.cfg.get("wind_from_deg", self.wind_dir))
        centre = self._centre(participants_locs)
        rng = plan.make_rng(int(self.cfg.get("seed", 1)))
        for m in self.cfg.get("missions") or []:
            try:
                self._mission(m, centre, rng)
            except Exception as err:  # noqa: BLE001 - one bad mission must not stop the trial
                self.log(f"drones: mission {m.get('id', m.get('kind'))} not flown ({type(err).__name__}: {err})")
                self.events.append(("drone_mission_failed", "", f"{m.get('id', m.get('kind'))}: {err}"))
        for c in self.cfg.get("cameras") or []:
            self._observer(c)
        n_ai = sum(1 for k in self.kinds if k != "drone")
        self.log(f"drones: {len(self.pilots)} piloted, {n_ai} AI in {len(self.missions)} missions; wind "
                 f"{self.w10:.1f} m/s at 10 m from {self.wind_dir:.0f} deg ({time.time() - t0:.1f} s)")
        self.log_f = open(self.out / "drones.csv", "w", newline="")
        self.log_w = csv.writer(self.log_f, delimiter=";")
        self.log_w.writerow(LOG_COLUMNS)

    def _wind(self, w):
        if isinstance(w, (int, float)):
            return float(w)
        try:   # weather.json of the trial: Open-Meteo's wind with weather real
            wx = json.loads((self.out / "weather.json").read_text())
            rec = wx.get("open_meteo") or {}
            if rec.get("wind_speed_10m") is not None:
                if rec.get("wind_direction_10m") is not None:
                    self.wind_dir = float(rec["wind_direction_10m"])
                return float(rec["wind_speed_10m"]) / 3.6
        except (OSError, ValueError):
            pass
        try:   # CARLA's wind intensity (0-100, a look): 100 as a fresh breeze, 10 m/s
            return float(self.world.get_weather().wind_intensity) / 10.0
        except (RuntimeError, AttributeError):
            return 0.0

    def _centre(self, locs):
        c = self.cfg.get("centre")
        if isinstance(c, dict) and "lat" in c:
            x, y = self.geo.to_carla(c["lat"], c["lon"])
            return from_carla(x, y)
        if self.pilots:
            return self.pilots[0].home.copy()
        if locs:
            l0 = locs[0]
            return from_carla(l0.x, l0.y)
        bb = self.tc["map"].get("bbox")
        if bb:
            x, y = self.geo.to_carla((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
            return from_carla(x, y)
        return np.zeros(3)

    def _point(self, spec, fallback=None):
        """{lat, lon} or [lat, lon] -> model (x, y, ground z)."""
        if spec is None or spec == "auto":
            return fallback
        lat, lon = (spec["lat"], spec["lon"]) if isinstance(spec, dict) else (spec[0], spec[1])
        x, y = self.geo.to_carla(lat, lon)
        return np.array([x, -y, self.ground_z(x, y)])

    def _open_ground(self, xy, rng, radius=40.0):
        q = plan.open_point_near(self.obs, xy[0], xy[1], rng, radius) or (xy[0], xy[1])
        x, y = float(q[0]), float(q[1])
        return np.array([x, y, self.ground_z(x, -y)])

    def _mission(self, m, centre, rng):
        kind = m.get("kind", "delivery")
        mid = m.get("id") or f"{kind}{len(self.missions) + 1}"
        af = dm.preset(m.get("airframe", "industrial" if kind == "delivery" else "consumer"))
        clear = float(m.get("clearance_m", 20.0))
        corridor = float(m.get("corridor_m", 10.0))
        c = self._point(m.get("centre"), centre)
        radius = float(m.get("radius_m", 250.0))
        start = float(m.get("start_s", 0.0))
        if kind == "delivery":
            n = int(m.get("count", 1))
            depot = self._point(m.get("depot"), None)
            if depot is None:
                depot = self._open_ground(c[:2], rng, 60.0)
            given = [self._point(a) for a in (m.get("addresses") or [])]
            for k in range(n):
                drops = list(given)
                if not drops:
                    picks = plan.pick_addresses(self.obs, int(m.get("addresses_per_drone", 3)), rng, c[:2], radius)
                    drops = [np.array([xy[0], xy[1], self.ground_z(xy[0], -xy[1])]) for _, xy in picks]
                if not drops:
                    drops = [self._open_ground((c[0] + rng.uniform(-radius, radius) * 0.7,
                                                c[1] + rng.uniform(-radius, radius) * 0.7), rng)]
                pad = depot + np.array([3.0 * math.cos(2.4 * k), 3.0 * math.sin(2.4 * k), 0.0]) * (k > 0)
                pad[2] = self.ground_z(pad[0], -pad[1])
                cruise = []
                for d in drops:
                    top = self.obs.corridor_top(pad, d, corridor)
                    base = max(pad[2], d[2])
                    cruise.append(max(top, base) + clear + float(m.get("layer_m", 4.0)) * (k + len(self.missions)))
                name = f"{mid}_{k}" if n > 1 else mid
                i = self._add(af, pad, rng.uniform(-math.pi, math.pi), name, "delivery", name)
                ms = plan.Delivery(name, self.fleet, i, tuple(pad), [tuple(d) for d in drops], cruise, None,
                                   start + float(m.get("stagger_s", 6.0)) * k, float(m.get("dwell_s", 10.0)),
                                   float(m.get("speed", 12.0)), m.get("drop", "land"), float(m.get("winch_m", 7.0)),
                                   bool(m.get("loop", True)))
                self._register(ms)
                self.log(f"drones: {name} from {pad[0]:.0f},{-pad[1]:.0f} to {len(drops)} addresses, cruise "
                         f"{', '.join(f'{z - pad[2]:.0f}' for z in cruise)} m above the depot")
        elif kind in ("patrol", "hover"):
            n = int(m.get("count", 1))
            for k in range(n):
                name = f"{mid}_{k}" if n > 1 else mid
                start_pt = self._point(m.get("start"), None)
                if start_pt is None:
                    start_pt = self._open_ground((c[0] + rng.uniform(-30, 30), c[1] + rng.uniform(-30, 30)), rng)
                i = self._add(af, start_pt, rng.uniform(-math.pi, math.pi), name, kind, name)
                if kind == "patrol":
                    if m.get("route"):
                        xy = [self._point(p)[:2] for p in m["route"]]
                    else:
                        xy = plan.loop_points(c, radius * 0.45, 24, rng.uniform(0, 2 * math.pi))
                    alt = m.get("altitude_m")
                    tops = [self.obs.corridor_top(a, b, corridor) for a, b in zip(xy, xy[1:] + xy[:1])]
                    z = (start_pt[2] + float(alt)) if alt is not None else \
                        max(max(tops), start_pt[2]) + max(clear, 35.0) + 3.0 * k
                    ms = plan.Patrol(name, self.fleet, i, [(p[0], p[1], z) for p in xy], float(m.get("speed", 8.0)),
                                     start + 2.0 * k)
                else:
                    pt = m.get("point") or {}
                    p = self._point(pt, c)
                    h = float(pt.get("height_m", 30.0)) if isinstance(pt, dict) else 30.0
                    roof = self.obs.top_at(p[0], p[1], 8.0)
                    z = max(p[2] + h, (roof + 8.0) if roof > -np.inf else -np.inf)
                    look = self._point(m.get("look"))
                    ms = plan.Hover(name, self.fleet, i, (p[0] + 4.0 * k, p[1], z),
                                    tuple(look[:2]) if look is not None else None, float(m.get("orbit_speed", 0.0)),
                                    start + 2.0 * k)
                self._register(ms)
        elif kind == "swarm":
            n = int(m.get("count", 8))
            launch = self._point(m.get("start"), None)
            if launch is None:
                launch = self._open_ground(c[:2], rng, 60.0)
            ids, names = [], []
            for k in range(n):        # a grid of take-off points 3 m apart
                p = launch + np.array([3.0 * (k % 4) - 4.5, 3.0 * (k // 4), 0.0])
                p[2] = self.ground_z(p[0], -p[1])
                name = f"{mid}_{k}"
                ids.append(self._add(af, p, 0.0, name, "swarm", name))
                names.append(name)
            if m.get("route"):
                route = [tuple(self._point(p)[:2]) for p in m["route"]]
            else:
                route = plan.loop_points(c, radius * 0.55, 8)
            alt = m.get("altitude_m")
            z = (launch[2] + float(alt)) if alt is not None else \
                max(self.obs.area_top(c, radius * 0.6), launch[2]) + max(clear, 30.0)
            pilots = self.pilots

            def others():
                return ([self.fleet.pos[p.idx] for p in pilots], [self.fleet.vel[p.idx] for p in pilots])
            ms = plan.Swarm(mid, self.fleet, ids, route, z, m.get("flocking", "coupled_sim"), self.obs, start,
                            float(m.get("stagger_s", 0.4)), others)
            self._register(ms)
            self.log(f"drones: swarm {mid} of {n} at {z - launch[2]:.0f} m round {len(route)} points "
                     f"({m.get('flocking', 'coupled_sim')} flocking)")
        else:
            raise ValueError(f"unknown drone mission kind {kind!r}")

    def _register(self, ms):
        self.missions.append(ms)
        for i in ms.idx:
            self.mission_of[i] = ms

    def _observer(self, c):
        """A fixed observer camera (a person on the ground looking at a drone or a point), saving frames."""
        carla = self.carla
        x, y = self.geo.to_carla(c["lat"], c["lon"])
        z = self.ground_z(x, y) + float(c.get("height_m", 1.7))
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        size = c.get("size", [1280, 720])
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(c.get("fov", 70)))
        from .compat import setup_camera
        every = float(c.get("every_s", self.tc["logging"].get("frames_every_s") or 1.0))
        keep = setup_camera(bp, every, self.tc["simulation"].get("camera_profile", "Dashcam"), 400)
        cam = self.world.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z)))
        name = c.get("name", f"observer{len(self.cameras) + 1}")
        d = self.out / f"{name}_frames"
        d.mkdir(parents=True, exist_ok=True)
        state = {"kept": None}

        def on_image(img, d=d, keep=keep or every, state=state):
            if state["kept"] is not None and img.timestamp - state["kept"] < keep - 1e-6:
                return
            state["kept"] = img.timestamp
            img.save_to_disk(str(d / f"{img.frame:08d}.jpg"))
        from .compat import listen
        listen(cam, on_image, self.log, f"drone camera {name}")
        look = c.get("look", "nearest")
        self.cameras.append({"cam": cam, "pos": (x, y, z), "look": look, "name": name})

    # -- per tick -------------------------------------------------------------------------------------------------
    def on_message(self, msg):
        """A station's message about its drone: sticks (drone_input) or its camera (drone_camera). True if handled."""
        kind = msg.get("type")
        if kind not in ("drone_input", "drone_camera"):
            return False
        p = next((p for p in self.pilots if p.role == msg.get("role")), None)
        if p is None:
            return True
        if kind == "drone_camera":
            p.camera_id = msg.get("camera_id")
            return True
        if msg.get("bot") and p.input is None:      # a station without a human: the host flies its bot
            p.input = self._bot(p.part.cfg.get("bot") or {}, p.af)
            return True
        p.sticks = Sticks.from_list(msg.get("s"), msg.get("g", 0.0), msg.get("e") or ())
        p.sticks_at = time.time()
        for e in p.sticks.events:
            self._pilot_event(p, e)
        return True

    def _pilot_event(self, p, e):
        aid = self._aid(p.idx)
        if e == "view":
            p.view = VIEWS[(VIEWS.index(p.view) + 1) % len(VIEWS)]
            self.events.append(("drone_view", aid, f"{p.role} {p.view}"))
        elif e == "mode":
            p.mode = "attitude" if p.mode == "position" else "position"
            p.hold.holding = False
            self.events.append(("drone_mode", aid, f"{p.role} {p.mode}"))
        elif e == "home" and not self.fleet.on_ground[p.idx]:
            p.rth = not p.rth
            self.events.append(("drone_rth", aid, f"{p.role} {'on' if p.rth else 'off'}"))
        elif e == "quit":
            p.quit = True

    def _aid(self, i):
        a = self.visuals[i].actor
        return a.id if a is not None else ""

    @property
    def quit(self):
        return any(p.quit for p in self.pilots)

    def step(self, t, dt, events=(), snap=None):
        """Commands for every drone, the flight model over dt, crash checks, and the tick's transforms queued."""
        t0 = time.perf_counter()
        f = self.fleet
        if f.n == 0:
            return
        self.prev = f.pos.copy()
        vel_before = f.vel.copy()
        # wind: the mean (log profile at each drone's height) plus Dryden gusts
        agl = f.pos[:, 2] - f.ground
        frm = math.radians(self.wind_dir)
        dirv = np.array([-math.sin(frm), -math.cos(frm), 0.0])       # blowing towards
        means = np.array([wind_at(self.w10, h) for h in agl])[:, None] * dirv
        air = np.linalg.norm(f.vel - means, axis=1)
        gust = self.gusts.step(dt, np.maximum(1.0, agl), 0.78 * self.w10, air) if self.w10 > 0 else 0.0
        f.wind[:] = means + gust
        for p in self.pilots:
            self._pilot_command(p, t, dt, events)
        ai = [i for i, k in enumerate(self.kinds) if k != "drone"]
        if ai:
            self._ai_commands(t, dt, ai)
        f.step(dt)
        self.acc = (f.vel - vel_before) / dt
        for p in self.pilots:
            self._crash_check(p, t, snap)
        self._ai_contacts(t, ai)
        if self.queue is not None:
            self._transforms(t, dt)
        self.ticks += 1
        self.cost += time.perf_counter() - t0

    def _pilot_command(self, p, t, dt, events):
        f, i, af = self.fleet, p.idx, p.af
        pos, vel = f.pos[i].copy(), f.vel[i].copy()
        yaw = float(f.yaw()[i])
        # sticks: the host's window, a bot, or the station's (centred if they stop coming: hold the position)
        if p.input is not None:
            if isinstance(p.input, BotPilot):
                p.sticks = p.input.sticks(t, pos, vel, yaw)
            else:
                p.sticks = p.input.poll(events)
                for e in p.sticks.events:
                    self._pilot_event(p, e)
        elif p.part.station and p.sticks_at is not None and time.time() - p.sticks_at > 0.5:
            if p.sticks.as_list() != [0, 0, 0, 0]:
                self.events.append(("drone_link_lost", self._aid(i), p.role))
            p.sticks = Sticks()
        p.gimbal = max(-90.0, min(20.0, p.gimbal + 45.0 * p.sticks.gimbal * dt))
        # the ground (or roof) under the drone, from CARLA
        c = to_carla(pos)
        z = self.surface(c[0], c[1], c[2] + 0.3, 500.0)
        if z is not None:
            p.ground = z
        f.ground[i] = p.ground
        on_ground = bool(f.on_ground[i])
        if p.crashed:
            f.set_command(i, "off")
            if on_ground:
                p.crashed = False
                self.events.append(("drone_crash_down", self._aid(i), f"{p.role} on the ground, ready"))
            return
        bat = float(f.battery()[i])
        for level in (0.2, 0.1):
            if bat <= level and level not in p.battery_warned:
                p.battery_warned.add(level)
                self.events.append(("drone_battery", self._aid(i), f"{p.role} {100 * bat:.0f}%"
                                    + (" auto landing" if level == 0.1 else " low")))
        if p.rth and not on_ground:
            if not p.sticks.centred:
                p.rth = False
                self.events.append(("drone_rth", self._aid(i), f"{p.role} cancelled by the sticks"))
            else:
                self._rth(p, pos, yaw)
                return
        cmd = pilot_command(p.sticks, pos, vel, yaw, on_ground, af, p.mode, p.hold, agl=pos[2] - p.ground)
        if bat <= 0.1 and not on_ground:       # forced landing, as consumer drones do
            cmd = {"mode": "velocity", "vel": (0.0, 0.0, -1.5), "yaw_rate": cmd.get("yaw_rate", 0.0)}
        self._fence(p, cmd, pos)
        f.set_command(i, **cmd)

    def _rth(self, p, pos, yaw):
        """Return to home: climb to 30 m above home (or stay higher), fly back, descend and land."""
        f, i = self.fleet, p.idx
        safe = p.home_ground + max(30.0, float(p.part.cfg.get("rth_altitude_m", 30.0)))
        d = math.hypot(pos[0] - p.home[0], pos[1] - p.home[1])
        if d > 2.0:
            z = max(pos[2], safe) if d > 5.0 else pos[2]
            if pos[2] < safe - 1.0 and d > 5.0:
                f.set_command(i, "position", pos=(pos[0], pos[1], z), yaw=yaw)
            else:
                f.set_command(i, "position", pos=(p.home[0], p.home[1], z),
                              yaw=math.atan2(p.home[1] - pos[1], p.home[0] - pos[0]))
        else:
            f.set_command(i, "velocity", vel=(0.0, 0.0, -1.5), yaw=yaw)

    def _fence(self, p, cmd, pos):
        """Keeps the command inside the geofence (radius from home, height above home); logs each breach once."""
        hit = False
        if p.fence_alt > 0 and pos[2] - p.home_ground >= p.fence_alt:
            if cmd.get("mode") == "velocity" and cmd["vel"][2] > 0:
                cmd["vel"] = (cmd["vel"][0], cmd["vel"][1], min(0.0, p.home_ground + p.fence_alt - pos[2]))
                hit = True
            if cmd.get("mode") == "attitude" and cmd.get("climb", 0) > 0:
                cmd["climb"] = 0.0
                hit = True
            if cmd.get("mode") == "position":
                q = list(cmd["pos"])
                q[2] = min(q[2], p.home_ground + p.fence_alt)
                cmd["pos"] = tuple(q)
        if p.fence_r > 0:
            dx, dy = pos[0] - p.home[0], pos[1] - p.home[1]
            d = math.hypot(dx, dy)
            if d >= p.fence_r and cmd.get("mode") == "velocity":
                ux, uy = dx / d, dy / d
                v = cmd["vel"]
                out = v[0] * ux + v[1] * uy
                if out > 0:
                    cmd["vel"] = (v[0] - out * ux, v[1] - out * uy, v[2])
                    hit = True
        if hit and not p.fenced:
            self.events.append(("drone_geofence", self._aid(p.idx), f"{p.role} at the fence "
                                f"({pos[2] - p.home_ground:.0f} m up, {math.hypot(*(pos[:2] - p.home[:2])):.0f} m out)"))
        p.fenced = hit

    def _ai_commands(self, t, dt, ai):
        f = self.fleet
        for i in ai:      # the ground under AI drones: their landing points' when near them, the road's elsewhere
            if self.ticks % 5 == 0 or self.ticks < 2:
                c = to_carla(f.pos[i])
                ms = self.mission_of.get(i)
                pts = []
                if isinstance(ms, plan.Delivery):
                    pts = [ms.depot] + ms.drops
                near = [q for q in pts if math.hypot(q[0] - f.pos[i, 0], q[1] - f.pos[i, 1]) < 6.0]
                f.ground[i] = near[0][2] if near else self.road_z(c[0], c[1])
        for ms in self.missions:
            ms.step(t, dt)
            for name, detail in ms.events:
                i0 = ms.idx[0]
                self.events.append((name, self._aid(i0), detail))
            ms.events.clear()
        # keep apart from each other and from the pilots' drones; climb over anything ahead
        swarm = {i for ms in self.missions if isinstance(ms, plan.Swarm) for i in ms.idx}
        others = [f.pos[p.idx] for p in self.pilots]
        nudge = plan.separation(f, ai, others, radius=float(self.cfg.get("separation_m", 10.0)), skip=swarm)
        floor = plan.avoid_obstacles(f, ai, self.obs)
        for row, i in enumerate(ai):
            if f.mode[i] == dm.OFF:
                continue
            if np.any(nudge[row]):
                f.cmd_vel[i] += nudge[row]
            if i in floor and f.cmd_pos[i, 2] < floor[i]:
                f.cmd_pos[i, 2] = floor[i]
                if f.mode[i] == dm.VELOCITY:
                    f.cmd_vel[i, 2] = max(f.cmd_vel[i, 2], 2.0)

    def _crash(self, i, what, speed, t, stop_at=None):
        f = self.fleet
        name = self.names[i]
        for p in self.pilots:
            if p.idx == i and isinstance(p.input, BotPilot):
                p.input.crashed = True
        if stop_at is not None:
            f.place(i, stop_at, (0.0, 0.0, min(0.0, float(f.vel[i, 2]))))
        f.set_command(i, "off")
        self.events.append(("drone_crash", self._aid(i), f"{name} hit {what} at {3.6 * speed:.0f} km/h"))
        self.log(f"drones: {name} crashed into {what} at {3.6 * speed:.0f} km/h (t {t:.1f} s)")

    def _crash_check(self, p, t, snap):
        """The step from the previous position to this one against CARLA's geometry (a ray of the drone's size
        ahead), vehicles and walkers (bounding boxes) and other drones; hitting the ground fast is a crash too."""
        f, i, af = self.fleet, p.idx, p.af
        a, b = self.prev[i], f.pos[i].copy()
        v = f.vel[i].copy()
        speed = float(np.linalg.norm(v))
        radius = af.arm + af.rotor_radius * 0.5
        landed_now = bool(f.on_ground[i]) and not p.was_ground
        p.was_ground = bool(f.on_ground[i])
        if landed_now:
            impact = float(np.linalg.norm(self.prev_vel[i])) if hasattr(self, "prev_vel") else 0.0
            self.events.append(("drone_landed", self._aid(i), f"{p.role} at {impact:.1f} m/s"))
            if impact > 3.0 and not p.crashed:
                p.crashes += 1
                self._crash(i, "the ground (hard landing)", impact, t)
                p.crashed = True
                return
        if p.crashed or f.on_ground[i]:
            return
        step = b - a
        n = float(np.linalg.norm(step))
        if n > 1e-4:
            d = step / n
            ca, cb = to_carla(a), to_carla(b + d * radius)
            carla = self.carla
            try:
                hits = self.world.cast_ray(carla.Location(*ca), carla.Location(*cb))
            except (RuntimeError, AttributeError):
                hits = []
            slow = abs(v[2]) < 3.0 and math.hypot(v[0], v[1]) < 2.0
            for h in hits:
                label = str(h.label).split(".")[-1]
                landing = h.location.z <= p.ground + 0.15 and (label in GROUND_LABELS or
                                                                (label in LANDABLE and d[2] < -0.5))
                if landing and slow:
                    continue        # touching down (on the ground, or on a roof from above)
                hit_m = from_carla(h.location.x, h.location.y, h.location.z)
                back = hit_m - d * radius
                p.crashes += 1
                p.crashed = True
                what = f"{label} (hard landing at {abs(v[2]):.1f} m/s)" if landing else label
                self._crash(i, what, speed, t, stop_at=(back[0], back[1], max(b[2], p.ground)))
                return
        hit = self._vehicle_hit(b, radius, snap)
        if hit:
            p.crashes += 1
            p.crashed = True
            self._crash(i, hit, speed, t, stop_at=a)
            return
        for j in range(f.n):
            if j == i:
                continue
            r2 = radius + f.airframes[j].arm + f.airframes[j].rotor_radius * 0.5
            if np.linalg.norm(f.pos[j] - b) < r2 and not f.on_ground[j]:
                p.crashes += 1
                p.crashed = True
                self._crash(i, f"drone {self.names[j]}", speed, t, stop_at=a)
                if self.kinds[j] != "drone":
                    self._crash(j, f"drone {self.names[i]}", float(np.linalg.norm(f.vel[j])), t)
                return

    def _vehicle_hit(self, b, radius, snap):
        """A vehicle or walker whose bounding box (grown by the drone's radius) holds the drone; its type or None."""
        if snap is None:
            return None
        c = to_carla(b)
        near = []
        for a in snap:
            loc = a.get_transform().location
            if abs(loc.x - c[0]) < 12 and abs(loc.y - c[1]) < 12 and abs(loc.z - c[2]) < 8:
                near.append(a)
        new = [a.id for a in near if a.id not in self.bb_cache]
        if new:
            try:
                for actor in self.world.get_actors(new):
                    tid = actor.type_id
                    self.bb_cache[actor.id] = (tid, actor.bounding_box) if tid.startswith(("vehicle.", "walker.")) \
                        else None
            except RuntimeError:
                pass
            for k in new:
                self.bb_cache.setdefault(k, None)
        for a in near:
            info = self.bb_cache.get(a.id)
            if info is None:
                continue
            tid, bb = info
            tf = a.get_transform()
            yaw = math.radians(tf.rotation.yaw)
            dx, dy = c[0] - tf.location.x, c[1] - tf.location.y
            lx = dx * math.cos(yaw) + dy * math.sin(yaw) - bb.location.x
            ly = -dx * math.sin(yaw) + dy * math.cos(yaw) - bb.location.y
            lz = c[2] - tf.location.z - bb.location.z
            e = bb.extent
            if abs(lx) < e.x + radius and abs(ly) < e.y + radius and abs(lz) < e.z + radius:
                return tid
        return None

    def _ai_contacts(self, t, ai):
        """AI drones that touch each other (should not happen; logged as crashes)."""
        f = self.fleet
        flying = [i for i in ai if not f.on_ground[i] and f.mode[i] != dm.OFF]
        for k, i in enumerate(flying):
            for j in flying[k + 1:]:
                r = f.airframes[i].arm + f.airframes[j].arm
                if np.linalg.norm(f.pos[i] - f.pos[j]) < r:
                    self._crash(i, f"drone {self.names[j]}", float(np.linalg.norm(f.vel[i])), t)
                    self._crash(j, f"drone {self.names[i]}", float(np.linalg.norm(f.vel[j])), t)
        self.prev_vel = f.vel.copy()

    def _transforms(self, t, dt):
        carla, f = self.carla, self.fleet
        mats = f.matrices()
        landed = f.on_ground
        for i, vis in enumerate(self.visuals):
            pc = to_carla(f.pos[i])
            rot = rotation_deg(mats[i])
            for c in vis.commands(carla, pc, rot, landed[i] and f.rotors[i].max() < 1.0):
                self.queue(c)
        if self.debug is not None and (self.mode == "shapes" or self.cfg.get("wireframe")) and self.ticks % 2 == 0:
            focus = [to_carla(f.pos[p.idx]) for p in self.pilots] + [c["pos"] for c in self.cameras]
            for i in range(f.n):
                pc = to_carla(f.pos[i])
                if focus and min(math.dist(pc, q) for q in focus) > float(self.cfg.get("wireframe_radius_m", 120)):
                    continue
                r = mats[i]
                fwd = (r[0, 0], -r[1, 0], r[2, 0])
                right = (-r[0, 1], r[1, 1], -r[2, 1])
                up = (r[0, 2], -r[1, 2], r[2, 2])
                draw_wireframe(self.debug, carla, f.airframes[i], pc, (fwd, right, up), t, dt * 2.6)
        for p in self.pilots:
            cam = p.camera_id if p.part.station else (self.view.camera.id if self.view is not None and
                                                      self.view.pilot is p and self.view.camera else None)
            if cam is not None:
                self.queue(carla.command.ApplyTransform(cam, self.camera_transform(p, dt)))
        for c in self.cameras:
            tgt = self._look_target(c)
            if tgt is not None:
                x, y, z = c["pos"]
                yaw = math.degrees(math.atan2(tgt[1] - y, tgt[0] - x))
                pitch = math.degrees(math.atan2(tgt[2] - z, math.hypot(tgt[0] - x, tgt[1] - y)))
                self.queue(carla.command.ApplyTransform(c["cam"].id, carla.Transform(
                    carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw))))

    def _look_target(self, c):
        f = self.fleet
        look = c["look"]
        if f.n == 0:
            return None
        if isinstance(look, str) and look in self.names:
            return to_carla(f.pos[self.names.index(look)])
        if isinstance(look, dict) and "lat" in look:
            x, y = self.geo.to_carla(look["lat"], look["lon"])
            return (x, y, self.ground_z(x, y) + float(look.get("height_m", 20.0)))
        pts = [to_carla(f.pos[i]) for i in range(f.n) if not f.on_ground[i]] or [to_carla(f.pos[0])]
        return min(pts, key=lambda q: math.dist(q, c["pos"]))

    def camera_transform(self, p, dt):
        """The pilot's camera: the gimbal under the nose (level, turning with the drone, pitch from the gimbal),
        the pilot's eyes on the ground looking at the drone (line of sight), or a chase camera behind and above."""
        carla, f = self.carla, self.fleet
        i, s = p.idx, p.af.size
        pos = f.pos[i]
        yaw = float(f.yaw()[i])
        c = to_carla(pos)
        cy, sy = math.cos(yaw), math.sin(yaw)
        heading = -math.degrees(yaw)
        if p.view == "los":
            lx, ly, lz = p.los
            pitch = math.degrees(math.atan2(c[2] - lz, math.hypot(c[0] - lx, c[1] - ly)))
            return carla.Transform(carla.Location(x=lx, y=ly, z=lz),
                                   carla.Rotation(pitch=pitch, yaw=math.degrees(math.atan2(c[1] - ly, c[0] - lx))))
        if p.view == "chase":
            back = np.array([pos[0] - cy * 1.6 * s * 1.3, pos[1] - sy * 1.6 * s * 1.3, pos[2] + 0.5 * s])
            p.cam_pos = back if p.cam_pos is None else p.cam_pos + (back - p.cam_pos) * (1 - math.exp(-12 * dt))
            q = to_carla(p.cam_pos)
            look = (c[0], c[1], c[2] + 0.1 * s)
            pitch = math.degrees(math.atan2(look[2] - q[2], math.hypot(look[0] - q[0], look[1] - q[1])))
            return carla.Transform(carla.Location(*q), carla.Rotation(
                pitch=pitch, yaw=math.degrees(math.atan2(look[1] - q[1], look[0] - q[0]))))
        g = (pos[0] + cy * 0.13 * s, pos[1] + sy * 0.13 * s, pos[2] + 0.02 * s)     # the gimbal camera's lens
        q = to_carla(g)
        return carla.Transform(carla.Location(*q), carla.Rotation(pitch=p.gimbal, yaw=heading))

    # -- what the pilot sees and the stations hear ----------------------------------------------------------------
    def hud(self, p):
        f, i = self.fleet, p.idx
        agl = float(f.pos[i, 2] - p.ground)
        kmh = 3.6 * float(np.linalg.norm(f.vel[i]))
        home = math.hypot(*(f.pos[i, :2] - p.home[:2]))
        bat = float(f.battery()[i])
        mins = float(f.endurance_s()[i]) / 60.0
        lines = [f"ALT {agl:5.1f} m  ({f.pos[i, 2] - p.home_ground:5.1f} m above home)   {kmh:5.1f} km/h",
                 f"BAT {100 * bat:3.0f}% ({mins:4.1f} min)   HOME {home:5.0f} m   "
                 f"{'P-GPS' if p.mode == 'position' else 'ATTI'}   {p.view.upper()}   GIMBAL {p.gimbal:+.0f}"]
        warn = [w for w, on in (("CRASHED", p.crashed), ("RETURNING HOME", p.rth), ("GEOFENCE", p.fenced),
                                ("LOW BATTERY", bat <= 0.2), ("LANDED", bool(f.on_ground[i]) and not p.crashed))
                if on]
        if warn:
            lines.append("  ".join(warn))
        return lines

    def broadcast(self, control, ticks):
        """Every 5th tick: the HUD to each pilot's station, the drones' sound sources to every station."""
        if control is None or ticks % 5:
            return
        f = self.fleet
        if f.n == 0:
            return
        for p in self.pilots:
            if p.part.station:
                control.send(p.role, {"type": "drone_hud", "lines": self.hud(p), "ears": list(p.los)})
        control.broadcast({"type": "drones", "d": self.sound_sources()})

    def sound_sources(self):
        """The drones as sound sources for soundscape.py: [id, x, y, z, vx, vy, vz (CARLA), blade-passing Hz, thrust
        ratio, sound power dB(A), blades] (the stations get them every 5th tick, the host's recorder every tick)."""
        f = self.fleet
        bpf, thr = f.blade_passing_hz(), f.thrust_ratio()
        d = []
        for i in range(f.n):
            c = to_carla(f.pos[i])
            v = f.vel[i]
            d.append([self._aid(i) or -1 - i, round(c[0], 2), round(c[1], 2), round(c[2], 2), round(float(v[0]), 2),
                      round(-float(v[1]), 2), round(float(v[2]), 2), round(float(bpf[i]), 1), round(float(thr[i]), 3),
                      f.airframes[i].sound_power_dba, int(f.airframes[i].blades)])
        return d

    # -- logging ----------------------------------------------------------------------------------------------------
    def records(self):
        """frames.csv rows: (role, kind, type id, actor id, carla.Transform, velocity, acceleration) per drone."""
        carla, f = self.carla, self.fleet
        out = []
        mats = f.matrices()
        for i in range(f.n):
            c = to_carla(f.pos[i])
            pitch, yaw, roll = rotation_deg(mats[i])
            v = f.vel[i]
            kind = "drone" if self.kinds[i] == "drone" else "drone_ai"
            a = self.acc[i] if getattr(self, "acc", None) is not None and len(self.acc) == f.n else np.zeros(3)
            out.append((self.names[i], kind, f"drone.{f.airframes[i].name.lower()}", self._aid(i) or -1 - i,
                        carla.Transform(carla.Location(*c), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)),
                        carla.Vector3D(float(v[0]), -float(v[1]), float(v[2])),
                        carla.Vector3D(float(a[0]), -float(a[1]), float(a[2]))))
        return out

    def log_frame(self, logger, t, frame):
        """drones.csv (pilots every frame, AI drones every log_every_s) and the frame rows and events."""
        for name, aid, detail in self.events:
            logger.event(t, frame, name, aid, detail)
        self.events.clear()
        f = self.fleet
        if f.n == 0 or self.log_f is None:
            return
        logger.drone_rows(t, frame, self.records())
        ai_now = self.ticks % self.log_every == 0
        mats = f.matrices()
        bpf, thr, bat = f.blade_passing_hz(), f.thrust_ratio(), f.battery()
        pil = {p.idx: p for p in self.pilots}
        for i in range(f.n):
            p = pil.get(i)
            if p is None and not ai_now:
                continue
            c = to_carla(f.pos[i])
            lat, lon = self.geo.to_latlon(c[0], c[1])
            pitch, yaw, roll = rotation_deg(mats[i])
            v = f.vel[i]
            ms = self.mission_of.get(i)
            st = p.sticks if p is not None else None
            ground = p.ground if p is not None else f.ground[i]
            self.log_w.writerow([
                f"{t:.3f}", frame, self.names[i], self.kinds[i], p.role if p else "", ms.name if ms else "",
                f.airframes[i].name, ms.phase if ms else ("crashed" if p and p.crashed else
                                                          "landed" if f.on_ground[i] else "flying"),
                p.mode if p else dm.MODE_NAMES[int(f.mode[i])], f"{c[0]:.3f}", f"{c[1]:.3f}", f"{c[2]:.3f}",
                f"{lat:.7f}", f"{lon:.7f}", f"{c[2] - ground:.2f}", f"{v[0]:.3f}", f"{-v[1]:.3f}", f"{v[2]:.3f}",
                f"{3.6 * float(np.linalg.norm(v)):.2f}", f"{yaw:.2f}", f"{pitch:.2f}", f"{roll:.2f}",
                *(f"{w:.1f}" for w in f.rotors[i]), f"{bpf[i]:.1f}", f"{thr[i]:.3f}", f"{f.power_w[i]:.1f}",
                f"{bat[i]:.4f}", *((f"{x:.3f}" for x in st.as_list()) if st else ("", "", "", "")),
                f"{p.gimbal:.1f}" if p else "", p.view if p else "", int(p.crashed) if p else "", self._aid(i)])

    def summary(self):
        f = self.fleet
        dl = sum(ms.delivered for ms in self.missions if isinstance(ms, plan.Delivery))
        return {"piloted": len(self.pilots), "ai": sum(1 for k in self.kinds if k != "drone"),
                "missions": len(self.missions), "look": self.mode, "deliveries": dl,
                "crashes": {p.role: p.crashes for p in self.pilots},
                "ms_per_tick": round(1000 * self.cost / max(1, self.ticks), 3),
                "battery_left": {self.names[i]: round(float(b), 3) for i, b in enumerate(f.battery())}}

    def close(self):
        if self.log_f is not None:
            self.log_f.close()
            self.log_f = None
        for c in self.cameras:
            try:
                c["cam"].stop()
                c["cam"].destroy()
            except RuntimeError:
                pass


class DroneView:
    """A host-local pilot's camera: the gimbal, line-of-sight or chase view (moved by DroneWorld every tick, not
    attached: the gimbal stays level), in a pygame window with the HUD, or saved every frames_every_s."""

    def __init__(self, drones, pilot, window=True, size=(1280, 720), fov=84, save_dir=None, save_every_s=0.0,
                 profile="Dashcam"):
        import carla
        from .compat import setup_camera
        world = drones.world
        self.drones, self.pilot, self.window, self.save_dir = drones, pilot, window, save_dir
        self.latest = None
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(fov))
        self.keep_every = setup_camera(bp, save_every_s if (not window and save_every_s > 0) else 0.0, profile, 400)
        self._kept = None
        self.camera = world.spawn_actor(bp, drones.camera_transform(pilot, 0.02))
        from .compat import listen
        listen(self.camera, self._on_image, drones.log, f"drone view {pilot.role}")
        self.screen = None
        if window:
            import pygame
            pygame.init()
            self.screen = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame.display.set_caption("coupled-carla: drone")
            self.font = pygame.font.SysFont("monospace", 20, bold=True)
        drones.view = self
        self.carla = carla

    def _on_image(self, image):
        if self.window:
            arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
            self.latest = arr[:, :, 2::-1]
        elif self.save_dir:
            if self.keep_every:
                if self._kept is not None and image.timestamp - self._kept < self.keep_every - 1e-6:
                    return
                self._kept = image.timestamp
            image.save_to_disk(str(Path(self.save_dir) / f"drone_{self.pilot.role}_{image.frame:08d}.jpg"))

    def events(self):
        if not self.window:
            return []
        import pygame
        return pygame.event.get()

    def render(self, hud_lines=()):
        if not self.window or self.latest is None:
            return
        import pygame
        self.screen.blit(pygame.surfarray.make_surface(self.latest.swapaxes(0, 1)), (0, 0))
        lines = self.drones.hud(self.pilot) + [x for x in hud_lines[1:2] if x]
        for k, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (255, 255, 255)), (16, 12 + 24 * k))
        pygame.display.flip()

    def destroy(self):
        try:
            self.camera.stop()
            self.camera.destroy()
        except RuntimeError:
            pass
        if self.window:
            import pygame
            pygame.quit()


# -- checks (validate.py) ----------------------------------------------------------------------------------------

MISSION_KEYS = {"kind", "id", "count", "airframe", "start_s", "stagger_s", "speed", "clearance_m", "corridor_m",
                "layer_m", "centre", "radius_m", "depot", "addresses", "addresses_per_drone", "dwell_s", "drop",
                "winch_m", "loop", "route", "altitude_m", "start", "point", "look", "orbit_speed", "flocking"}
MISSION_KINDS = ("delivery", "patrol", "swarm", "hover")
PILOT_KEYS = {"airframe", "mode", "geofence", "gimbal_pitch_deg", "pilot_position", "pilot_offset_m",
              "rth_altitude_m", "axes", "joystick", "deadband", "expo", "buttons"}
PILOT_INPUTS = {"keyboard", "gamepad", "rc", "bot", "autopilot"}
CAMERA_KEYS = {"name", "lat", "lon", "height_m", "look", "every_s", "fov", "size"}


def check_config(tc, where, duration, error, warn, points):
    """Problems in a trial's drones section and drone participants: error(where, msg), warn(where, msg);
    points(where, [[lat, lon], ...], kind) collects points for the map checks."""
    dc = tc.get("drones") or {}
    if dc.get("visual", "auto") not in ("auto", "blueprint", "mesh", "shapes", "none"):
        error(f"{where} drones.visual", f"{dc.get('visual')!r} is not auto, blueprint, mesh, shapes or none")
    ids = []
    for m in dc.get("missions") or []:
        if not isinstance(m, dict):
            error(f"{where} drones.missions", f"{m!r} is not a mapping")
            continue
        mw = f"{where} drone mission {m.get('id', m.get('kind'))}"
        for k in m:
            if k not in MISSION_KEYS:
                error(mw, f"unknown key {k!r}")
        kind = m.get("kind", "delivery")
        if kind not in MISSION_KINDS:
            error(mw, f"kind {kind!r} is not one of {', '.join(MISSION_KINDS)}")
        ids.append(m.get("id"))
        if m.get("airframe", "consumer").lower() not in dm.PRESETS:
            error(mw, f"airframe {m['airframe']!r} is not consumer or industrial")
        if m.get("drop", "land") not in ("land", "winch"):
            error(mw, f"drop {m.get('drop')!r} is not land or winch")
        fl = m.get("flocking")
        if fl is not None:
            try:
                dm.flock_params(fl)
            except (KeyError, TypeError) as e:
                error(mw, f"flocking {fl!r}: {e}")
        for key in ("depot", "centre", "start", "point", "look"):
            v = m.get(key)
            if isinstance(v, dict) and ("lat" in v) != ("lon" in v):
                error(f"{mw} {key}", "needs both lat and lon")
            elif isinstance(v, dict) and "lat" in v:
                points(f"{mw} {key}", [[v["lat"], v["lon"]]], "any")
        for key in ("addresses", "route"):
            if m.get(key) is not None:
                pts = m[key]
                if not isinstance(pts, list) or not all(isinstance(p, (list, tuple)) and len(p) >= 2 for p in pts):
                    error(f"{mw} {key}", "needs a list of [lat, lon]")
                else:
                    points(f"{mw} {key}", [list(p[:2]) for p in pts], "any")
        if kind == "patrol" and m.get("route") is not None and len(m["route"]) < 2:
            error(mw, "a patrol route needs at least 2 points")
        if isinstance(m.get("start_s"), (int, float)) and duration and m["start_s"] >= duration:
            warn(mw, f"starts at {m['start_s']} s, after the trial's end")
        if isinstance(m.get("count"), int) and m["count"] > 60:
            warn(mw, f"{m['count']} drones: each costs little in the model but one actor and a transform per tick")
    named = [i for i in ids if i]
    if len(set(named)) != len(named):
        error(f"{where} drones.missions", "mission ids are not unique")
    for c in dc.get("cameras") or []:
        if not isinstance(c, dict) or "lat" not in c or "lon" not in c:
            error(f"{where} drones.cameras", f"{c!r} needs lat and lon")
            continue
        for k in c:
            if k not in CAMERA_KEYS:
                error(f"{where} drones.cameras", f"unknown key {k!r}")
        points(f"{where} drones camera {c.get('name', '')}", [[c["lat"], c["lon"]]], "any")
    for p in tc.get("participants") or []:
        if not isinstance(p, dict) or p.get("kind") != "drone":
            continue
        pw = f"{where} participant {p.get('role')}"
        if p.get("input") is not None and p["input"] not in PILOT_INPUTS:
            error(pw, f"input {p['input']!r} is not one of {', '.join(sorted(PILOT_INPUTS - {'autopilot'}))}")
        if p.get("view") is not None and p["view"] not in VIEWS:
            error(pw, f"view {p['view']!r} is not one of {', '.join(VIEWS)}")
        if p.get("mode", "position") not in ("position", "attitude"):
            error(pw, f"mode {p.get('mode')!r} is not position or attitude")
        if str(p.get("airframe", "consumer")).lower() not in dm.PRESETS:
            error(pw, f"airframe {p['airframe']!r} is not consumer or industrial")
        if not (isinstance(p.get("spawn"), dict) and "lat" in p["spawn"]):
            error(pw, "a drone needs spawn: {lat, lon} (its take-off point)")
        gf = p.get("geofence") or {}
        if not isinstance(gf, dict) or set(gf) - {"radius_m", "max_altitude_m"}:
            error(pw, "geofence takes {radius_m, max_altitude_m}")
        elif (gf.get("max_altitude_m") or 0) > 120:
            warn(pw, f"geofence max_altitude_m {gf['max_altitude_m']} is above the EU open category's 120 m")
        bot = p.get("bot") or {}
        if bot.get("path"):
            if not all(isinstance(q, (list, tuple)) and len(q) >= 2 for q in bot["path"]):
                error(f"{pw} bot", "path needs [lat, lon, height_m] points")
            else:
                points(f"{pw} bot path", [list(q[:2]) for q in bot["path"]], "any")
