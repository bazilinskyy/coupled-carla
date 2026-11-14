"""AI participants (coupled-sim's AI cars and AI pedestrians), eHMI drawing, and pedestrian yielding in SUMO.

ScriptedCar   follows a route through lat/lon points (planned on the lanes with SafeGlobalRoutePlanner) with
              CARLA's BasicAgent, at a speed profile set by triggers along the route, like coupled-sim's
              SpeedSettings on waypoints: speed_kmh, accel (m/s2), yield_s (stop and wait), blinker, ehmi.
              It stops for SUMO red/yellow lights and, with yield_to_pedestrians, for pedestrians in its path
              (human or AI). It can carry an eHMI, set by its triggers, by an AV-passenger station, or `auto`
              (yielding while it brakes for a pedestrian, driving otherwise).
ScriptedWalker walks a path of lat/lon points with CARLA WalkerControl (no navigation mesh needed): start_s,
              speed, loop. The same path follower drives a pedestrian station's bot.
draw_ehmi()   the eHMI as a light bar across the front of the car and a text above it, from world.debug, so it
              shows in every station's camera.
SumoYield     makes SUMO vehicles stop for pedestrians in their path (SUMO itself only yields to persons on
              crossings, and CARLA pedestrians are not in SUMO).
"""
import math
import random
import time
import zlib

import carla
import traci

from . import compat
from .geo import ground

DEFAULT_EHMI = {
    "off": {"text": "", "color": [0, 0, 0]},
    "driving": {"text": "DRIVING", "color": [255, 170, 0]},
    "yielding": {"text": "I'M STOPPING", "color": [0, 190, 255]},
    "go": {"text": "GO AHEAD", "color": [0, 230, 120]},
}


def _rel(tf, loc):
    """(forward, right) of loc in the frame of transform tf."""
    yaw = math.radians(tf.rotation.yaw)
    dx, dy = loc.x - tf.location.x, loc.y - tf.location.y
    return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)


# A stroke font (polylines on a 4 x 6 grid): CARLA draws debug *text* only in the spectator view, not in camera
# sensors, so eHMI text is drawn with debug *lines*, which every station's camera sees.
GLYPHS = {
    "A": [[(0, 0), (0, 4), (2, 6), (4, 4), (4, 0)], [(0, 3), (4, 3)]],
    "B": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (0, 0)]],
    "C": [[(4, 6), (0, 6), (0, 0), (4, 0)]],
    "D": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 1), (3, 0), (0, 0)]],
    "E": [[(4, 6), (0, 6), (0, 0), (4, 0)], [(0, 3), (3, 3)]],
    "F": [[(4, 6), (0, 6), (0, 0)], [(0, 3), (3, 3)]],
    "G": [[(4, 6), (0, 6), (0, 0), (4, 0), (4, 3), (2, 3)]],
    "H": [[(0, 0), (0, 6)], [(4, 0), (4, 6)], [(0, 3), (4, 3)]],
    "I": [[(2, 0), (2, 6)], [(1, 0), (3, 0)], [(1, 6), (3, 6)]],
    "J": [[(4, 6), (4, 0), (0, 0), (0, 2)]],
    "K": [[(0, 0), (0, 6)], [(4, 6), (0, 3), (4, 0)]],
    "L": [[(0, 6), (0, 0), (4, 0)]],
    "M": [[(0, 0), (0, 6), (2, 3), (4, 6), (4, 0)]],
    "N": [[(0, 0), (0, 6), (4, 0), (4, 6)]],
    "O": [[(0, 0), (0, 6), (4, 6), (4, 0), (0, 0)]],
    "P": [[(0, 0), (0, 6), (4, 6), (4, 3), (0, 3)]],
    "Q": [[(0, 0), (0, 6), (4, 6), (4, 0), (0, 0)], [(2, 2), (4, 0)]],
    "R": [[(0, 0), (0, 6), (4, 6), (4, 3), (0, 3), (4, 0)]],
    "S": [[(4, 6), (0, 6), (0, 3), (4, 3), (4, 0), (0, 0)]],
    "T": [[(0, 6), (4, 6)], [(2, 6), (2, 0)]],
    "U": [[(0, 6), (0, 0), (4, 0), (4, 6)]],
    "V": [[(0, 6), (2, 0), (4, 6)]],
    "W": [[(0, 6), (1, 0), (2, 3), (3, 0), (4, 6)]],
    "X": [[(0, 0), (4, 6)], [(0, 6), (4, 0)]],
    "Y": [[(0, 6), (2, 3), (4, 6)], [(2, 3), (2, 0)]],
    "Z": [[(0, 6), (4, 6), (0, 0), (4, 0)]],
    "'": [[(2, 6), (2, 4.5)]],
    "!": [[(2, 6), (2, 2)], [(2, 0.6), (2, 0)]],
    "-": [[(1, 3), (3, 3)]],
}


def stroke_text(debug, centre, along, up, text, height, colour, life, thickness=0.03):
    """Draws text with lines, centred on `centre`, letters advancing along `along` (unit vectors)."""
    unit = height / 6.0
    advance = 5.5 * unit
    width = advance * len(text) - 1.5 * unit
    origin = centre - along * (width / 2) - up * (height / 2)
    for i, ch in enumerate(text.upper()):
        base = origin + along * (i * advance)
        for stroke in GLYPHS.get(ch, []):
            pts = [base + along * (x * unit) + up * (y * unit) for x, y in stroke]
            for a, b in zip(pts, pts[1:]):
                debug.draw_line(a, b, thickness=thickness, color=colour, life_time=life)


SLOTS = ("front", "rear", "roof", "road", "eyes")


def _facing(d):
    """Letters advance to the right of a reader looking along d (CARLA's yaw turns clockwise seen from above)."""
    return carla.Location(-d.y, d.x, 0.0)


def draw_ehmi(debug, vehicle, state, states, life, slots=("front",), look_at=None):
    """Draws the eHMI of `vehicle` in its current state on the chosen slots:

    front  light bar across the front, the state's text above it (read by someone in front of the car)
    rear   the same on the back (read by a cyclist or driver behind)
    roof   a light ring round the roof and the text on all four sides of it (read from anywhere)
    road   the text projected on the road 3 m ahead, readable from in front
    eyes   two eyes on the front whose pupils follow `look_at` (a location: the nearest pedestrian), coloured
           by the state: the "eyes on a car" concept for eye contact with an automated vehicle
    """
    spec = states.get(state) or {}
    rgb = spec.get("color", [0, 0, 0])
    if not state or state == "off" or rgb == [0, 0, 0]:
        return
    tf = vehicle.get_transform()
    ext = vehicle.bounding_box.extent
    fwd, right = tf.get_forward_vector(), tf.get_right_vector()
    fwd, right = carla.Location(fwd.x, fwd.y, 0.0), carla.Location(right.x, right.y, 0.0)
    up = carla.Location(z=1.0)
    colour = carla.Color(*rgb)
    dim = carla.Color(*(int(c * 0.55) for c in rgb))     # debug lines glow (bloom); text a little dimmer stays legible
    text = spec.get("text", "")

    def bar(a, b, thick=0.12):
        debug.draw_line(a, b, thickness=thick, color=colour, life_time=life)

    for slot in slots:
        if slot in ("front", "rear"):
            d = fwd if slot == "front" else fwd * -1.0
            c = tf.location + d * (ext.x + 0.06) + carla.Location(z=ext.z * 1.05)
            bar(c - right * (ext.y * 0.8), c + right * (ext.y * 0.8))
            if text:
                stroke_text(debug, c + up * 0.42 - d * 0.05, _facing(d * -1.0), up, text, height=0.32,
                            colour=dim, life=life, thickness=0.035)
        elif slot == "roof":
            z = carla.Location(z=2 * ext.z + 0.08)
            corners = [tf.location + fwd * (sx * ext.x * 0.45) + right * (sy * ext.y * 0.85) + z
                       for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
            for a, b in zip(corners, corners[1:] + corners[:1]):
                bar(a, b, 0.08)
            if text:
                for d, half in ((fwd, ext.x * 0.45), (fwd * -1.0, ext.x * 0.45),
                                (right, ext.y * 0.85), (right * -1.0, ext.y * 0.85)):
                    stroke_text(debug, tf.location + z + up * 0.22 + d * (half + 0.02), _facing(d * -1.0), up,
                                text, height=0.22, colour=dim, life=life, thickness=0.03)
        elif slot == "road" and text:
            c = tf.location + fwd * (ext.x + 3.0) + carla.Location(z=0.06)    # a vehicle's origin is at road level
            # read by someone in front looking back along -fwd: the top of the letters is towards the car
            stroke_text(debug, c, _facing(fwd * -1.0), fwd * -1.0, text, height=0.55, colour=dim, life=life,
                        thickness=0.035)
        elif slot == "eyes":
            c = tf.location + fwd * (ext.x + 0.05) + carla.Location(z=ext.z * 1.0)
            target = None
            if look_at is not None:
                target = carla.Location(look_at.x - c.x, look_at.y - c.y, look_at.z - c.z)
                n = math.sqrt(target.x ** 2 + target.y ** 2 + target.z ** 2)
                target = target * (1.0 / n) if n > 1e-6 else None
            for side in (-1, 1):
                eye = c + right * (side * ext.y * 0.45)
                r = 0.16
                pts = [eye + right * (r * math.cos(a)) + up * (r * math.sin(a))
                       for a in (2 * math.pi * k / 16 for k in range(17))]
                for a, b in zip(pts, pts[1:]):
                    bar(a, b, 0.04)
                # pupil: shifted inside the eye towards the target (or straight ahead)
                dx = (target.x * right.x + target.y * right.y) if target else 0.0
                dz = target.z if target else 0.0
                pupil = eye + right * (0.08 * dx) + up * (0.08 * dz) + fwd * 0.02
                debug.draw_point(pupil, size=0.09, color=carla.Color(20, 20, 20), life_time=life)
                debug.draw_point(pupil + fwd * 0.01, size=0.05, color=colour, life_time=life)


class MutualGaze:
    """Logs eye contact between pedestrians and vehicles: both face each other within `angle_deg` and closer than
    `distance_m`. A pedestrian faces where its station's camera looks (the station sends its yaw), else where the
    walker faces; a vehicle faces along its heading (no head tracking yet). Events eye_contact_start / _end with
    the pair, the distance and both angles."""

    def __init__(self, logger, angle_deg=20.0, distance_m=30.0):
        self.logger, self.angle, self.dist = logger, float(angle_deg), float(distance_m)
        self.on = {}

    @staticmethod
    def _off(yaw, a, b):
        want = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        return abs((want - yaw + 180) % 360 - 180)

    def update(self, t, frame, parts, agents, station_gaze):
        peds = [(p.role, p.actor, station_gaze.get(p.role)) for p in parts if p.kind == "pedestrian"]
        if not peds:
            return
        cars = [(a.id, a.vehicle) for a in agents if isinstance(a, ScriptedCar)]
        cars += [(p.role, p.actor) for p in parts if p.actor.type_id.startswith("vehicle.")]
        seen, unique = set(), []
        for name, actor in cars:          # an AV passenger's actor is the AV agent's car: count it once
            if actor.id not in seen:
                seen.add(actor.id)
                unique.append((name, actor))
        cars = unique
        for prole, pact, pyaw in peds:
            try:
                ptf = pact.get_transform()
            except RuntimeError:
                continue
            pyaw = ptf.rotation.yaw if pyaw is None else pyaw
            for crole, cact in cars:
                try:
                    ctf = cact.get_transform()
                except RuntimeError:
                    continue
                d = ptf.location.distance(ctf.location)
                a_p = self._off(pyaw, ptf.location, ctf.location)
                a_c = self._off(ctf.rotation.yaw, ctf.location, ptf.location)
                now = d < self.dist and a_p < self.angle and a_c < self.angle
                key = (prole, crole)
                if now != self.on.get(key, False):
                    self.on[key] = now
                    self.logger.event(t, frame, "eye_contact_start" if now else "eye_contact_end", "",
                                      f"{prole}<->{crole} {d:.1f} m, pedestrian {a_p:.0f} deg, vehicle {a_c:.0f} deg")


class ScriptedCar:
    kind = "agent"

    def __init__(self, world, geo, cfg, planner=None):
        from .routing import SafeGlobalRoutePlanner   # needs CARLA's PythonAPI (agents); stations do not
        import agents.navigation.basic_agent as basic_agent
        self.cfg, self.id = cfg, cfg["id"]
        self.world, cmap = world, world.get_map()
        pts = [carla.Location(*geo.to_carla(lat, lon)) for lat, lon in cfg["route"]]
        start = cmap.get_waypoint(pts[0], project_to_road=True, lane_type=carla.LaneType.Driving)
        bp = compat.find(world, cfg.get("vehicle", "vehicle.tesla.model3"), "passenger")
        bp.set_attribute("role_name", self.id)
        if bp.has_attribute("color") and cfg.get("color"):
            bp.set_attribute("color", cfg["color"])
        tf = start.transform
        tf.location.z += 0.5
        self.vehicle = world.spawn_actor(bp, tf)
        self.apply = self.vehicle.apply_control   # the runner sends it with the tick's batch instead (run.py)
        self.planner = planner or SafeGlobalRoutePlanner(cmap, 2.0)
        plan = []
        for a, b in zip([start.transform.location] + pts[1:-1], pts[1:]):
            seg = self.planner.trace_route(a, b)
            plan += seg[1:] if plan else seg
        self.plan = plan
        self.base_speed = float(cfg.get("speed_kmh", 40))
        self.agent = basic_agent.BasicAgent(self.vehicle, target_speed=self.base_speed,
                                            opt_dict={"max_brake": 0.8}, map_inst=cmap, grp_inst=self.planner)
        self.agent.ignore_traffic_lights(True)          # lights come from SUMO (signals.py)
        self.agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
        self.target = 0.0
        self.desired = self.base_speed
        self.accel = float(cfg.get("accel", 2.0))
        self.yield_until = None
        self.light = None
        self.ehmi_mode = cfg.get("ehmi", "off")         # off | auto | manual (a passenger sets it)
        self.ehmi = "driving" if self.ehmi_mode == "auto" else "off"
        self.braking_for = None
        self.clear_since = None
        self.triggers = []
        for trg in cfg.get("triggers", []):
            if "at" in trg:
                loc = carla.Location(*geo.to_carla(*trg["at"]))
                idx = min(range(len(plan)), key=lambda i: plan[i][0].transform.location.distance(loc)) if plan else 0
            else:
                idx = self._index_at_m(trg.get("at_m", 0.0))
            self.triggers.append([idx, dict(trg), False])
        self.progress = 0
        self.events = []                                 # (name, detail) for the log, drained by the runner

    def _index_at_m(self, metres):
        acc = 0.0
        for i in range(1, len(self.plan)):
            acc += self.plan[i - 1][0].transform.location.distance(self.plan[i][0].transform.location)
            if acc >= metres:
                return i
        return len(self.plan) - 1

    def route_length(self):
        from .routing import route_length
        return route_length(self.plan)

    def step(self, t, dt, walkers=()):
        v = self.vehicle.get_velocity()
        speed = math.hypot(v.x, v.y)
        loc = self.vehicle.get_location()
        # progress along the plan (search a little ahead of the last index)
        best = min(range(self.progress, min(len(self.plan), self.progress + 40)),
                   key=lambda i: self.plan[i][0].transform.location.distance(loc), default=self.progress)
        self.progress = best
        for trg in self.triggers:
            idx, spec, fired = trg
            if not fired and self.progress >= idx:
                trg[2] = True
                self.events.append(("trigger", f"{self.id} {spec}"))
                if "speed_kmh" in spec:
                    self.desired = float(spec["speed_kmh"])
                if "accel" in spec:
                    self.accel = float(spec["accel"])
                if spec.get("yield_s"):
                    self.yield_until = t + float(spec["yield_s"])
                if "blinker" in spec:
                    ls = {"left": carla.VehicleLightState.LeftBlinker, "right": carla.VehicleLightState.RightBlinker,
                          "hazard": carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker}
                    self.vehicle.set_light_state(carla.VehicleLightState(ls.get(spec["blinker"], 0)))
                if "ehmi" in spec:
                    self.set_ehmi(spec["ehmi"], "trigger")
        # speed profile: ramp the target speed at the configured acceleration
        step_kmh = self.accel * 3.6 * dt
        self.target += max(-step_kmh * 2, min(step_kmh, self.desired - self.target))
        self.agent.set_target_speed(max(0.0, self.target))
        ctl = self.agent.run_step()
        stop, reason = False, None
        ext_x = self.vehicle.bounding_box.extent.x
        stop_gap = None
        if self.yield_until is not None:
            if t < self.yield_until:
                stop, reason = True, "yield_s"
            else:
                self.yield_until = None
        # pedestrians in the path; once stopped for one, hold until the (wider) path has been clear for 0.8 s
        if self.cfg.get("yield_to_pedestrians", True):
            tf = self.vehicle.get_transform()
            holding = (self.braking_for or "").startswith("pedestrian")
            reach = 6.0 + speed * 1.2 + speed ** 2 / (2 * 3.5) + (8.0 if holding else 0.0)
            width = 3.3 if holding else 2.6
            seen, seen_fwd = None, None
            for w in walkers:
                fwd, side = _rel(tf, w.get_location())
                if 0.5 < fwd < reach and abs(side) < width and (seen_fwd is None or fwd < seen_fwd):
                    seen, seen_fwd = w, fwd
            if seen is not None:
                self.clear_since = None
                stop, reason = True, f"pedestrian {seen.id}"
                stop_gap = seen_fwd - ext_x - 4.0     # come to rest about 4 m short of the pedestrian
            elif holding:
                self.clear_since = t if self.clear_since is None else self.clear_since
                if t - self.clear_since < 0.8:
                    stop, reason = True, self.braking_for
        # SUMO traffic light ahead
        if self.light and self.light[2] in "rRyYs":
            if self.light[1] < 4.0 + speed * 1.0 + speed ** 2 / (2 * 3.5):
                stop, reason = True, f"light {self.light[0]} {self.light[2]}"
        if stop:
            if stop_gap is not None and speed > 0.3:
                # brake just enough to stop at the gap (comfortable), harder when it gets short
                need = speed ** 2 / (2 * max(0.5, stop_gap))
                brake = 0.0 if need < 0.8 else min(1.0, 0.08 + need / 7.0)
                ctl = carla.VehicleControl(throttle=0.0, brake=brake)
            else:
                ctl = carla.VehicleControl(throttle=0.0, brake=min(1.0, 0.25 + speed / 12.0) if speed > 0.3 else 1.0)
            self.target = min(self.target, speed * 3.6)
        if reason != self.braking_for:
            if reason:
                self.events.append(("agent_stops", f"{self.id} for {reason}"))
            elif self.braking_for:
                self.events.append(("agent_goes", f"{self.id}"))
            self.braking_for = reason
        if self.ehmi_mode == "auto":
            self.set_ehmi("yielding" if (reason or "").startswith("pedestrian") else "driving", "auto")
        if self.agent.done() and self.cfg.get("at_end", "stop") == "stop":
            ctl = carla.VehicleControl(throttle=0.0, brake=1.0)
        self.apply(ctl)
        return ctl

    def set_ehmi(self, state, by):
        if state != self.ehmi:
            self.ehmi = state
            self.events.append(("ehmi", f"{self.id} {state} (by {by})"))


class PathWalker:
    """Follows a list of carla.Locations with WalkerControl; used by ScriptedWalker and pedestrian bots."""

    def __init__(self, walker, points, speed=1.4, start_s=0.0, loop=False):
        self.walker, self.points, self.speed = walker, points, speed
        self.apply = walker.apply_control   # the runner sends it with the tick's batch instead (run.py)
        self.start_s, self.loop, self.i, self.done = start_s, loop, 0, False

    def step(self, t):
        if self.done or t < self.start_s or not self.points:
            self.apply(carla.WalkerControl(speed=0.0))
            return
        loc = self.walker.get_location()
        target = self.points[self.i]
        dx, dy = target.x - loc.x, target.y - loc.y
        d = math.hypot(dx, dy)
        if d < 0.8:
            self.i += 1
            if self.i >= len(self.points):
                if self.loop:
                    self.i = 0
                else:
                    self.done = True
                    self.apply(carla.WalkerControl(speed=0.0))
                    return
            return
        self.apply(carla.WalkerControl(direction=carla.Vector3D(dx / d, dy / d, 0.0),
                                                      speed=self.speed))


class ScriptedWalker:
    kind = "walker"

    def __init__(self, world, geo, cfg):
        self.cfg, self.id = cfg, cfg["id"]
        cmap = world.get_map()
        pts = [ground(cmap, *geo.to_carla(lat, lon)) for lat, lon in cfg["path"]]
        want = cfg.get("blueprint", "walker.pedestrian.*")
        bps = sorted(world.get_blueprint_library().filter(want), key=lambda b: b.id) if "*" in want else []
        bp = bps[zlib.crc32(self.id.encode()) % len(bps)] if bps else compat.find(world, want)
        bp.set_attribute("role_name", self.id)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        self.walker = world.spawn_actor(bp, carla.Transform(pts[0] + carla.Location(z=1.2)))
        self.follow = PathWalker(self.walker, pts[1:], float(cfg.get("speed", 1.3)), float(cfg.get("start_s", 0)),
                                 bool(cfg.get("loop", False)))
        self.events = []

    def step(self, t, dt, walkers=()):
        self.follow.step(t)


class Crowd:
    """Background pedestrians walking up and down beside the roads near the participants: on the sidewalk lanes
    when the OpenDRIVE has them, else 1.5 m beyond the outermost lane (on the ground plane, where the
    participants' pedestrians walk too). They never step onto the road, so neither SUMO nor the scripted cars
    yield to them, and SUMO does not know them.

    Not CARLA's walker AI: CARLA 0.9.16 never serves a navigation mesh for an OpenDRIVE world (it writes
    OpenDriveMap.bin, but get_random_location_from_navigation() stays None, even after reloading the world), and
    walkers fall through its generated sidewalk meshes. So each follows a path with PathWalker, out and back.

    traffic.walkers: {count: 0, radius_m: 120, seed: 1}
    """

    def __init__(self, world, centres, count, radius_m=120.0, seed=1, log=print, path_m=150.0, kerb_m=1.5):
        rng = random.Random(seed)
        self.walkers, self.follow = [], []
        if count <= 0:
            return
        cmap = world.get_map()

        def outer(w):
            """The kerbside lane of w's side of the road, and whether it is a sidewalk."""
            side = w
            for _ in range(6):
                nxt = compat.right_lane(side)
                if nxt is None or nxt.lane_id * w.lane_id <= 0 or nxt.lane_type in (
                        carla.LaneType.NONE, carla.LaneType.Border, carla.LaneType.Median):
                    return side, False
                side = nxt
                if side.lane_type == carla.LaneType.Sidewalk:
                    return side, True
            return side, False

        def spot(w, sidewalk):
            if sidewalk:
                return w.transform.location
            r = w.transform.get_right_vector()
            off = w.lane_width / 2 + kerb_m
            return w.transform.location + carla.Location(r.x * off, r.y * off, 0.0)

        starts = []
        for w in cmap.generate_waypoints(8.0):
            if w.lane_type != carla.LaneType.Driving or w.is_junction:
                continue
            if centres and min(w.transform.location.distance(c) for c in centres) > radius_m:
                continue
            starts.append(outer(w))
        rng.shuffle(starts)
        bps = sorted(world.get_blueprint_library().filter("walker.pedestrian.*"), key=lambda b: b.id)
        on_sidewalks = 0
        for start, sidewalk in starts:
            if len(self.walkers) >= count:
                break
            pts, w = [spot(start, sidewalk)], start
            for _ in range(int(path_m / 2.5)):
                nxt = [n for n in w.next(2.5) if n.lane_type == start.lane_type and not n.is_junction]
                if not nxt:
                    break
                w = nxt[0]
                pts.append(spot(w, sidewalk))
            if len(pts) < 8:
                continue
            bp = rng.choice(bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            bp.set_attribute("role_name", "crowd")
            i0 = rng.randrange(len(pts) // 2)
            walker = world.try_spawn_actor(bp, carla.Transform(pts[i0] + carla.Location(z=1.0)))
            if walker is None:
                continue
            path = pts[i0 + 1:] + pts[::-1] + pts[1:i0 + 1]      # on to the end, back, and round again
            self.walkers.append(walker)
            self.follow.append(PathWalker(walker, path, rng.uniform(1.0, 1.6), rng.uniform(0, 3), loop=True))
            on_sidewalks += sidewalk
        log(f"crowd: {len(self.walkers)} walkers beside the roads within {radius_m:.0f} m "
            f"({on_sidewalks} on sidewalk lanes)")

    def step(self, t):
        for f in self.follow:
            try:
                f.step(t)
            except RuntimeError:
                pass


class SumoYield:
    """SUMO vehicles (shown in CARLA) stop for pedestrians in their path and drive on when it is clear."""

    def __init__(self, every=2):
        self.held = set()
        self.every, self._tick = every, 0
        self.events = []

    def step(self, sync, world, walkers):
        self._tick += 1
        if self._tick % self.every or not walkers:
            if not walkers and self.held:
                for sid in list(self.held):
                    self._release(sid)
            return
        wl = [w.get_location() for w in walkers]
        for sid, cid in list(sync.sumo2carla_ids.items()):
            actor = world.get_actor(cid)
            if actor is None:
                continue
            tf = actor.get_transform()
            if all(tf.location.distance(w) > 60 for w in wl):
                if sid in self.held:
                    self._release(sid)
                continue
            v = actor.get_velocity()
            speed = math.hypot(v.x, v.y)
            reach = 6.0 + speed * 1.0 + speed ** 2 / (2 * 3.0)
            hit = any(0.5 < f < reach and abs(s) < 2.2 for f, s in (_rel(tf, w) for w in wl))
            if hit and sid not in self.held:
                try:
                    traci.vehicle.setSpeed(sid, 0.0)      # SUMO brakes within its decel limits
                    self.held.add(sid)
                    self.events.append(("sumo_yields", f"{sid} (carla {cid})"))
                except traci.TraCIException:
                    pass
            elif not hit and sid in self.held:
                self._release(sid)

    def _release(self, sid):
        try:
            traci.vehicle.setSpeed(sid, -1)
        except traci.TraCIException:
            pass
        self.held.discard(sid)
