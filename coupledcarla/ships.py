"""AI ships in a coupled-carla trial (the `ships` section), sailed by the MMG model of shipmodel.py and integrated on
the host, like the drones (drones.py): the host steps every ship each tick and sends its pose to CARLA in the tick's
batch (ApplyTransform), so the picture, the logs and the other actors all see the same state of the same tick.

First use: ships seen from the shore (docs/ships.md). A ship follows a route of (lat, lon) points with the model's
autopilot at an ordered speed; current, wind and waves come from `ships.environment` (wind "auto": the trial's real
weather, as for the drones). The water stays flat in CARLA: the waves act on the ship's course and speed only.

Looks (`ships.visual`):
- `mesh`: the hull from tools/ship_mesh.py (SM_CCShip_<Hull>, imported by tools/ue_ship_assets.py into
  /Game/CoupledCarla/Ships) as a static.prop.mesh with a mass (Movable), physics and collisions then switched off so
  ApplyTransform moves it. Its origin is amidships on the waterline.
- `shapes` (any server, e.g. CARLA 0.9.16): the hull as a row of boxes from /Engine/BasicShapes (only uniform scale
  is possible, so the boxes are a beam wide and reach below the water), the deckhouse as a box and the funnel as a
  cylinder, moved as one.
- `auto`: mesh if the server has it, else shapes. `none`: no actors (logs only).
"""
import csv
import json
import math
import time
from pathlib import Path

from . import shipmodel as sm

MESH_PATH = "/Game/CoupledCarla/Ships/SM_CCShip_{0}.SM_CCShip_{0}"
CUBE = "/Engine/BasicShapes/Cube.Cube"
CYLINDER = "/Engine/BasicShapes/Cylinder.Cylinder"
SHIP_KEYS = {"id", "hull", "route", "speed_kn", "start_s", "advance_s", "at_end"}
ENV_KEYS = {"current_mps", "current_to_deg", "wind_mps", "wind_from_deg", "wave_hs", "wave_tp", "wave_from_deg"}
AT_END = {"stop", "loop", "remove"}
LOG_COLUMNS = ["timestamp", "frame", "ship", "hull", "pos_x", "pos_y", "pos_z", "lat", "lon", "heading_deg",
               "speed_kn", "sog_kn", "cog_deg", "drift_deg", "yaw_rate_dps", "rudder_deg", "revolutions_rpm",
               "air_u_kn", "air_v_kn", "wave_fx_kN", "wave_fy_kN", "waypoint", "state", "actor_id"]


# -- frames ------------------------------------------------------------------------------------------------------

def earth_from_carla(x, y):
    """CARLA (x east, y south) -> the model's earth frame (x north, y east), same origin."""
    return -float(y), float(x)


def route_points(geo, route):
    """[[lat, lon], ...] -> [(north, east), ...] in the model's earth frame (metres from CARLA's origin)."""
    out = []
    for p in route:
        x, y = geo.to_carla(float(p[0]), float(p[1]))
        out.append(earth_from_carla(x, y))
    return out


def point_in_ring(x, y, ring):
    """Whether (x, y) is inside the closed ring [[x, y], ...] (even-odd)."""
    inside = False
    j = len(ring) - 1
    for i in range(len(ring)):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


# -- looks -------------------------------------------------------------------------------------------------------

def shape_parts(hull):
    """The `shapes` look: (mesh path, uniform scale in m, dx forward, dy starboard, dz up) per part, from amidships on
    the waterline. Boxes a beam wide along the hull (their tops at the deck), a deckhouse over the stern and a funnel
    behind it."""
    L, B, fb = hull.lpp, hull.beam, hull.freeboard
    n = max(2, math.ceil(L / B))
    span = L - B
    parts = [(CUBE, B, -span / 2 + k * span / (n - 1), 0.0, fb - B / 2) for k in range(n)]
    house = min(0.45 * B, 0.09 * L)                       # deckhouse: a cube, about 4 decks high on a VLCC
    hx = -L / 2 + 0.06 * L + house / 2
    parts.append((CUBE, house, hx, 0.0, fb + house / 2))
    funnel = 0.4 * house
    parts.append((CYLINDER, funnel, hx - house / 2 - funnel / 2, 0.0, fb + house + funnel / 2 - 0.3 * funnel))
    return parts


class ShipVisual:
    """The actors that show one ship and their per-tick transforms."""

    def __init__(self, mode, actors, parts=None):
        self.mode, self.actors, self.parts = mode, actors, parts or []

    @property
    def actor(self):
        return self.actors[0] if self.actors else None

    def commands(self, carla, x, y, z, yaw_deg):
        out = []
        if not self.actors:
            return out
        rot = carla.Rotation(yaw=yaw_deg)
        if self.mode == "shapes":
            cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
            for actor, (_, _, dx, dy, dz) in zip(self.actors, self.parts):
                out.append(carla.command.ApplyTransform(actor.id, carla.Transform(
                    carla.Location(x=x + dx * cy - dy * sy, y=y + dx * sy + dy * cy, z=z + dz), rot)))
            return out
        out.append(carla.command.ApplyTransform(self.actors[0].id, carla.Transform(carla.Location(x=x, y=y, z=z), rot)))
        return out

    def destroy(self, carla, client):
        ids = [a.id for a in self.actors]
        if ids and client is not None:
            try:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
            except RuntimeError:
                pass
        self.actors = []


def _mesh_exists(world, path):
    """Spawns the mesh once far below; an actor without a cooked mesh has no extent."""
    import carla
    try:
        bp = world.get_blueprint_library().find("static.prop.mesh")
        bp.set_attribute("mesh_path", path)
    except (IndexError, RuntimeError):
        return False
    a = world.try_spawn_actor(bp, carla.Transform(carla.Location(z=-2000.0)))
    if a is None:
        return False
    try:
        e = a.bounding_box.extent
        return e.x > 0.01 or e.y > 0.01
    finally:
        a.destroy()


def available_mode(world, wanted, hull_name, log=print):
    if wanted in ("none", None, False):
        return "none"
    if wanted in ("auto", "mesh"):
        if _mesh_exists(world, MESH_PATH.format(hull_name)):
            return "mesh"
        if wanted == "mesh":
            log(f"ships: no SM_CCShip_{hull_name} on this server (tools/ship_mesh.py, tools/ue_ship_assets.py and "
                "the content pack); using shapes")
    try:
        world.get_blueprint_library().find("static.prop.mesh")
    except (IndexError, RuntimeError):
        return "none"
    return "shapes"


def spawn_visual(world, mode, hull, at, yaw_deg, role, log=print):
    import carla
    lib = world.get_blueprint_library()
    x, y, z = at

    def kinematic(actor):
        actor.set_simulate_physics(False)
        actor.set_collisions(False)
        return actor
    try:
        if mode == "mesh":
            bp = lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", MESH_PATH.format(hull.name))
            bp.set_attribute("mass", "1.0")
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", role)
            return ShipVisual(mode, [kinematic(world.spawn_actor(
                bp, carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw_deg))))])
        if mode == "shapes":
            parts, actors = shape_parts(hull), []
            cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
            for mesh, scale, dx, dy, dz in parts:
                bp = lib.find("static.prop.mesh")
                bp.set_attribute("mesh_path", mesh)
                bp.set_attribute("scale", f"{scale:.3f}")
                bp.set_attribute("mass", "1.0")
                actors.append(kinematic(world.spawn_actor(bp, carla.Transform(
                    carla.Location(x=x + dx * cy - dy * sy, y=y + dx * sy + dy * cy, z=z + dz),
                    carla.Rotation(yaw=yaw_deg)))))
            return ShipVisual(mode, actors, parts)
    except (RuntimeError, IndexError) as err:
        log(f"ships: {mode} look for {role} failed ({err}); no actor")
    return ShipVisual("none", [])


# -- the ships ---------------------------------------------------------------------------------------------------

class AIShip:
    def __init__(self, spec, hull, route, ship, z):
        self.spec, self.hull, self.route, self.ship, self.z = spec, hull, route, ship, z
        self.name = str(spec.get("id") or hull.name.lower())
        self.start_s = float(spec.get("start_s", 0.0))
        self.at_end = spec.get("at_end", "stop")
        self.speed_kn = float(spec.get("speed_kn") or hull.service_speed_kn)
        self.visual = None
        self.state = "waiting"          # waiting | sailing | stopping | removed
        self.sailed_m = 0.0


class ShipWorld:
    """AI ships of the trial's `ships.fleet`: the model stepping them, their looks and logs. The run loop calls
    start() after the first tick, step() every tick before the CARLA tick and log_frame() after the frame's logging."""

    def __init__(self, world, client, geo, tc, mdir, out, log=print):
        import carla
        self.carla, self.world, self.client, self.geo, self.tc = carla, world, client, geo, tc
        self.cfg = tc.get("ships") or {}
        self.mdir, self.out, self.log = Path(mdir), Path(out), log
        self.ships = []
        self.queue = None
        self.events = []
        self.env = None
        self.modes = {}
        self.log_f = None
        self.log_every = max(1, round(float(self.cfg.get("log_every_s", 0.5)) / tc["simulation"]["step"]))
        self.ticks, self.cost = 0, 0.0

    # -- set-up -----------------------------------------------------------------------------------------------
    def environment(self):
        e = dict(self.cfg.get("environment") or {})
        wind, wind_from = e.get("wind_mps", "auto"), e.get("wind_from_deg", 270.0)
        if not isinstance(wind, (int, float)):
            wind, wind_from = self._weather_wind(wind_from)
        return sm.Environment(current_mps=float(e.get("current_mps", 0.0)),
                              current_to_deg=float(e.get("current_to_deg", 0.0)),
                              wind_mps=float(wind), wind_from_deg=float(wind_from),
                              wave_hs=float(e.get("wave_hs", 0.0)), wave_tp=float(e.get("wave_tp", 0.0)),
                              wave_from_deg=float(e.get("wave_from_deg", wind_from)))

    def _weather_wind(self, wind_from):
        """Open-Meteo's wind of the trial (weather.json, weather real), else CARLA's wind_intensity / 10."""
        try:
            rec = json.loads((self.out / "weather.json").read_text()).get("open_meteo") or {}
            if rec.get("wind_speed_10m") is not None:
                return float(rec["wind_speed_10m"]) / 3.6, float(rec.get("wind_direction_10m", wind_from))
        except (OSError, ValueError):
            pass
        try:
            return float(self.world.get_weather().wind_intensity) / 10.0, wind_from
        except (RuntimeError, AttributeError):
            return 0.0, wind_from

    def _probe(self, x, y):
        """What a ray from 500 m down to -200 m hits at CARLA (x, y): [(label, z), ...]."""
        carla = self.carla
        try:
            hits = self.world.cast_ray(carla.Location(x=x, y=y, z=500.0), carla.Location(x=x, y=y, z=-200.0)) or []
        except (RuntimeError, AttributeError):
            hits = []
        return [(str(h.label).split(".")[-1], h.location.z) for h in hits]

    def scenery_level(self, points):
        """The level of the scenery's water body (scenery.json "water", the flat water tiles' top) under the routes'
        points: the median over the points that lie in one, else None."""
        try:
            bodies = json.loads((self.mdir / "scenery.json").read_text()).get("water") or []
        except (OSError, ValueError):
            return None
        levels = []
        for x, y in points:
            for b in bodies:
                inside = sum(1 for ring in b.get("rings") or [] if len(ring) >= 3 and point_in_ring(x, y, ring))
                if inside % 2:
                    levels.append(float(b["level"]))
                    break
        return float(sorted(levels)[len(levels) // 2]) if levels else None

    def water_level(self, points):
        """One waterline height for the map (CARLA z): `ships.water_z` if set, else the level of the scenery's water
        under the routes (its flat tiles), else the median of the Water hits of rays there, else the median of the
        highest hits, else 0. One level for every ship: a single probe can hit a quay, a buoy or a bank (2026-11-27,
        Hoek van Holland: +1.32 and -0.45 m under two ships)."""
        z = self.cfg.get("water_z", "auto")
        if isinstance(z, (int, float)):
            return float(z)
        lv = self.scenery_level(points)
        if lv is not None:
            self.log(f"ships: water level {lv:.2f} m (the scenery's water under the routes)")
            return lv
        water, top, seen = [], [], {}
        for x, y in points:
            hits = self._probe(x, y)
            for label, hz in hits:
                seen[label] = seen.get(label, 0) + 1
            water += [hz for label, hz in hits if label == "Water"]
            if hits:
                top.append(max(hz for _, hz in hits))
        pick = water or top
        level = float(sorted(pick)[len(pick) // 2]) if pick else 0.0
        self.log(f"ships: water level {level:.2f} m from {len(water)} Water hits of {len(points)} points "
                 f"(labels hit: {', '.join(f'{k} {v}' for k, v in sorted(seen.items())) or 'none'})")
        return level

    def start(self):
        t0 = time.time()
        self.env = self.environment()
        pts = []
        for spec in self.cfg.get("fleet") or []:
            try:
                pts += [(e, -n) for n, e in route_points(self.geo, spec.get("route") or [])]
            except (TypeError, ValueError, IndexError):
                pass
        self.level = self.water_level(pts)
        for spec in self.cfg.get("fleet") or []:
            try:
                self._add(spec)
            except Exception as err:  # noqa: BLE001 - one bad ship must not stop the trial
                self.log(f"ships: {spec.get('id', spec.get('hull'))} not sailed ({type(err).__name__}: {err})")
                self.events.append(("ship_failed", "", f"{spec.get('id', spec.get('hull'))}: {err}"))
        e = self.env
        self.log(f"ships: {len(self.ships)} ({', '.join(sorted(set(self.modes.values()))) or 'no look'}); wind "
                 f"{e.wind_mps:.1f} m/s from {e.wind_from_deg:.0f} deg, current {e.current_mps:.1f} m/s to "
                 f"{e.current_to_deg:.0f} deg, waves Hs {e.wave_hs:.1f} m Tp {e.wave_tp:.1f} s "
                 f"({time.time() - t0:.1f} s)")
        self.log_f = open(self.out / "ships.csv", "w", newline="")
        self.log_w = csv.writer(self.log_f, delimiter=";")
        self.log_w.writerow(LOG_COLUMNS)

    def _add(self, spec):
        hull = sm.preset(spec.get("hull", "kvlcc2"))
        route = route_points(self.geo, spec["route"])
        if len(route) < 2:
            raise ValueError("a route needs at least two points")
        (n0, e0), (n1, e1) = route[0], route[1]
        psi = math.atan2(e1 - e0, n1 - n0)
        ship = sm.Ship.at_speed(hull, float(spec.get("speed_kn") or hull.service_speed_kn), x=n0, y=e0, psi=psi,
                                env=self.env, wave_every_s=1.0)
        ship.autopilot = sm.Autopilot(waypoints=route[1:], reach_m=0.5 * hull.lpp)
        s = AIShip(spec, hull, route, ship, self.level)
        adv = float(spec.get("advance_s", 0.0))
        while ship.t < adv and not ship.autopilot.done:          # already under way when the trial starts
            ship.step(1.0)
        ship.t = 0.0
        if hull.name not in self.modes:
            self.modes[hull.name] = available_mode(self.world, self.cfg.get("visual", "auto"), hull.name, self.log)
        self.ships.append(s)

    def _spawn(self, s):
        x, y, yaw = sm.to_carla(s.ship.x, s.ship.y, s.ship.psi)
        mode = self.modes.get(s.hull.name, "none")
        s.visual = spawn_visual(self.world, mode, s.hull, (x, y, s.z), yaw, s.name, self.log) \
            if mode != "none" else ShipVisual("none", [])
        s.state = "sailing"
        self.events.append(("ship_spawn", self._aid(s),
                            f"{s.name} {s.hull.name} {s.visual.mode} at {x:.1f},{y:.1f},{s.z:.1f} "
                            f"{s.speed_kn:.1f} kn"))

    # -- per tick ---------------------------------------------------------------------------------------------
    def step(self, t, dt):
        t0 = time.perf_counter()
        for s in self.ships:
            if s.state == "waiting":
                if t < s.start_s:
                    continue
                self._spawn(s)
            if s.state == "removed":
                continue
            x0, y0 = s.ship.x, s.ship.y
            s.ship.step(dt)
            s.sailed_m += math.hypot(s.ship.x - x0, s.ship.y - y0)
            if s.state == "sailing" and s.ship.autopilot.done:
                self._route_end(s, t)
        if self.queue is not None:
            carla = self.carla
            for s in self.ships:
                if s.visual is not None and s.state != "removed":
                    x, y, yaw = sm.to_carla(s.ship.x, s.ship.y, s.ship.psi)
                    for c in s.visual.commands(carla, x, y, s.z, yaw):
                        self.queue(c)
        self.ticks += 1
        self.cost += time.perf_counter() - t0

    def _route_end(self, s, t):
        self.events.append(("ship_arrived", self._aid(s), f"{s.name} after {s.sailed_m:.0f} m, {s.at_end}"))
        if s.at_end == "loop":
            (n0, e0), (n1, e1) = s.route[0], s.route[1]
            sh = s.ship
            sh.x, sh.y, sh.psi = n0, e0, math.atan2(e1 - e0, n1 - n0)
            sh.u, sh.vm, sh.r, sh.delta, sh.rudder_cmd = s.speed_kn * sm.KNOT, 0.0, 0.0, 0.0, 0.0
            sh.autopilot = sm.Autopilot(waypoints=s.route[1:], reach_m=0.5 * s.hull.lpp)
        elif s.at_end == "remove":
            s.visual.destroy(self.carla, self.client)
            s.state = "removed"
        else:                           # stop: engine stopped, the last heading held while she slows down
            s.ship.order_speed(0.0)
            s.ship.autopilot = sm.Autopilot(heading=s.ship.psi)
            s.state = "stopping"

    def _aid(self, s):
        a = s.visual.actor if s.visual is not None else None
        return a.id if a is not None else ""

    # -- logs -------------------------------------------------------------------------------------------------
    def records(self):
        """frames.csv rows: (role, kind, type id, actor id, transform, velocity, acceleration) per ship on show."""
        carla = self.carla
        out = []
        for k, s in enumerate(self.ships):
            if s.state in ("waiting", "removed"):
                continue
            x, y, yaw = sm.to_carla(s.ship.x, s.ship.y, s.ship.psi)
            vn, ve = s.ship.velocity_ne()
            out.append((s.name, "ship_ai", f"ship.{s.hull.name.lower()}", self._aid(s) or -1000 - k,
                        carla.Transform(carla.Location(x=x, y=y, z=s.z), carla.Rotation(yaw=yaw)),
                        carla.Vector3D(float(ve), float(-vn), 0.0), carla.Vector3D(0.0, 0.0, 0.0)))
        return out

    def log_frame(self, logger, t, frame):
        for name, aid, detail in self.events:
            logger.event(t, frame, name, aid, detail)
        self.events.clear()
        if self.log_f is None or not self.ships:
            return
        logger.drone_rows(t, frame, self.records())
        if self.ticks % self.log_every:
            return
        kn = 1.0 / sm.KNOT
        for s in self.ships:
            if s.state in ("waiting", "removed"):
                continue
            sh = s.ship
            x, y, yaw = sm.to_carla(sh.x, sh.y, sh.psi)
            lat, lon = self.geo.to_latlon(x, y)
            vn, ve = sh.velocity_ne()
            au, av = sm.to_body(*self.env.wind_ne(), sh.psi)      # the air's velocity in the body frame
            self.log_w.writerow([
                f"{t:.3f}", frame, s.name, s.hull.name, f"{x:.2f}", f"{y:.2f}", f"{s.z:.2f}", f"{lat:.7f}",
                f"{lon:.7f}", f"{math.degrees(sh.psi) % 360:.2f}", f"{sh.speed * kn:.2f}",
                f"{sh.speed_over_ground * kn:.2f}", f"{math.degrees(sh.course) % 360:.2f}",
                f"{math.degrees(math.atan2(-sh.vm, sh.u)) if sh.u > 0.01 else 0.0:.2f}",
                f"{math.degrees(sh.r):.3f}", f"{math.degrees(sh.delta):.2f}", f"{60 * sh.n:.1f}",
                f"{au * kn:.2f}", f"{av * kn:.2f}", f"{sh.wave_force[0] / 1e3:.1f}",
                f"{sh.wave_force[1] / 1e3:.1f}", sh.autopilot.index if sh.autopilot else "", s.state, self._aid(s)])

    def summary(self):
        return {"ships": len(self.ships), "looks": dict(self.modes),
                "sailed_m": {s.name: round(s.sailed_m, 1) for s in self.ships},
                "arrived": [s.name for s in self.ships if s.state in ("stopping", "removed")],
                "ms_per_tick": round(1000 * self.cost / max(1, self.ticks), 3)}

    def close(self):
        if self.log_f is not None:
            self.log_f.close()
            self.log_f = None
        for s in self.ships:
            if s.visual is not None:
                s.visual.destroy(self.carla, self.client)


# -- config checks (validate.py) ---------------------------------------------------------------------------------

def check_config(tc, where, error, warn, points):
    cfg = tc.get("ships") or {}
    if not cfg:
        return
    w = f"{where} ships"
    if cfg.get("visual", "auto") not in ("auto", "mesh", "shapes", "none"):
        error(f"{w} visual", f"{cfg.get('visual')!r} is not auto, mesh, shapes or none")
    for k in set(cfg.get("environment") or {}) - ENV_KEYS:
        error(f"{w} environment", f"unknown key {k!r} ({', '.join(sorted(ENV_KEYS))})")
    for i, spec in enumerate(cfg.get("fleet") or []):
        ws = f"{w} fleet[{i}]"
        if not isinstance(spec, dict):
            error(ws, "needs a mapping")
            continue
        for k in set(spec) - SHIP_KEYS:
            error(ws, f"unknown key {k!r} ({', '.join(sorted(SHIP_KEYS))})")
        try:
            sm.preset(spec.get("hull", "kvlcc2"))
        except ValueError as err:
            error(f"{ws} hull", str(err))
        route = spec.get("route") or []
        if len(route) < 2:
            error(f"{ws} route", "needs at least two [lat, lon] points")
        else:
            points(f"{ws} route", route, "any")     # on water: only the map box is checked
        if spec.get("at_end", "stop") not in AT_END:
            error(f"{ws} at_end", f"{spec.get('at_end')!r} is not {', '.join(sorted(AT_END))}")
        if float(spec.get("speed_kn", 1.0)) <= 0:
            error(f"{ws} speed_kn", "must be above 0")
