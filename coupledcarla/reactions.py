"""How the living city's people and drivers react to the participant and to each other (living_city.reactions).

  horns         a SUMO driver behind a participant who does not move off (light green or none, nothing ahead)
                honks within 12 s with probability honk_p12 (Doob & Gross 1968: 84 % and 50 %; Jann & Coutts 2017:
                79 %), after a grace time; a driver whose lane the participant cuts into (time headway under 0.8 s or
                a needed deceleration over 3 m/s^2) honks with honk_cut_in_p; drivers queued behind a stopped van or
                taxi honk now and then; a driver held by a pedestrian participant standing in the road honks like a
                blocked one. Event "horn" (the sound helper renders it) with the reason and kinematics.
  courtesy      a SUMO driver in the lane the participant signals into lets them in with courtesy_p: slows down
                and flashes its headlights ("lc_yield_flash").
  looks         a pedestrian standing (at the kerb, a stop, a shop window, a bench) turns the head to an approaching
                vehicle within 30 m and 6 s (look_p: Rasouli, Kotseruba & Tsotsos 2017: over 90 % look before
                crossing; texting walkers at odds / 4.0: Thompson et al. 2013); "lc_look", with "participant" when the
                vehicle is a participant's.
  hurry / back  a crossing pedestrian with a vehicle under 3 s away speeds up by hurry_factor (Gorrini et al. 2017:
                1.35 m/s crossing after 0.94 m/s assessing traffic); one who just stepped out with a vehicle under
                step_back_tta_s away steps back to the kerb; a horn within 15 m startles walkers the same way.
  bells         a SUMO cyclist with a pedestrian in its path within 10 m rings with bell_p per encounter
                (Hatfield & Prabhakharan 2016: cyclists seldom warn).
  near misses   any participant and road user, and SUMO vehicles and crossing walkers, closing with a time to
                collision under near_miss_ttc_s (1.5 s, the usual critical TTC: van der Horst 1990) or passing within
                0.5 m: "near_miss" with both ids, kinds, distance, TTC, speeds, decelerations and the reactions of
                the last 2 s.
All decisions draw from random generators seeded by living_city.seed, the reaction and the actors involved.
"""
import math

from .livingcity import rng_for

SOURCES = {
    "doob1968": "Doob AN, Gross AE (1968). Status of frustrator as an inhibitor of horn-honking responses. Journal of "
                "Social Psychology 76(2):213-218 (84 % / 50 % honked within 12 s); Jann B, Coutts E (2017). Social "
                "status and peer-punishment: findings from two road traffic field experiments. Bern working paper "
                "27 (79 % within 12 s, Bern)",
    "rasouli2017": "Rasouli A, Kotseruba I, Tsotsos JK (2017). Agreeing to cross: how drivers and pedestrians "
                   "communicate. IEEE Intelligent Vehicles Symposium; arXiv:1702.03555 (over 90 % looked)",
    "thompson2013": "Thompson LL et al. (2013). Injury Prevention 19(4):232-237 (texters' odds of not looking 4.0)",
    "gorrini2017": "Gorrini A, Crociani L, Vizzari G, Bandini S (2017). Observation results on pedestrian-vehicle "
                   "interactions at non-signalized intersections towards simulation. TGF 2017; arXiv:1711.00728",
    "hatfield2016": "Hatfield J, Prabhakharan P (2016). An investigation of behaviour and attitudes relevant to the "
                    "user safety of pedestrian/cyclist shared paths. Transportation Research F 40:35-47",
    "vanderhorst1990": "van der Horst ARA (1990). A time-based analysis of road user behaviour in normal and critical "
                       "encounters. PhD thesis, TU Delft (TTC under 1.5 s critical)",
    "assumption": "an assumption of this model (no open measurement found); set it in the experiment",
}

RATES = {
    "honk_p12": (0.7, "probability within 12 s", "doob1968", "measured",
                 "a driver blocked by a car that does not move off honks within 12 s"),
    "honk_grace_s": (2.0, "s", "assumption", "assumption", "no honk in the first seconds of a blocking"),
    "honk_cut_in_p": (0.3, "per cut-in", "assumption", "assumption",
                      "a driver the participant cuts in front of honks (no open measurement found)"),
    "honk_queue_per_s": (0.01, "per s", "assumption", "assumption",
                         "a driver queued behind a stopped van or taxi honks"),
    "courtesy_p": (0.3, "per signalled lane change", "assumption", "assumption",
                   "a driver lets the participant in (could not verify a share; Kondyli & Elefteriadou 2009 found "
                   "drivers prefer changing lanes to slowing)"),
    "look_p": (0.9, "per approaching vehicle", "rasouli2017", "measured",
               "a standing pedestrian looks at an approaching vehicle"),
    "phone_look_odds": (4.0, "odds ratio", "thompson2013", "measured", "texters look less"),
    "hurry_factor": (1.44, "x speed", "gorrini2017", "measured",
                     "crossing speed after assessing traffic (1.35 / 0.94 m/s)"),
    "step_back_tta_s": (1.5, "s", "assumption", "assumption",
                        "a pedestrian who just stepped out steps back for a vehicle this close in time"),
    "bell_p": (0.2, "per encounter", "hatfield2016", "assumption", "cyclists seldom ring (no share published)"),
    "near_miss_ttc_s": (1.5, "s", "vanderhorst1990", "measured", "time to collision of a near miss"),
}

DEFAULTS = {"enabled": True, "horns": True, "courtesy": True, "looks": True, "hurry": True, "bells": True,
            "near_misses": True}

RADIUS = {"bike": 0.5, "car": 1.0, "bus": 1.3, "walker": 0.3}     # discs for near misses (half widths + a margin)
LEFT_BLINKER, RIGHT_BLINKER = 2, 1          # bits of carla.VehicleLightState: LeftBlinker = 32, RightBlinker = 16
GREEN = ("G", "g")


def rate(lc, name):
    return (lc.get("rates") or {}).get(name, RATES[name][0])


def honk_hazard(p12, grace_s):
    """Hazard (1/s) after the grace time so that P(honk within 12 s of the blocking) = p12."""
    return -math.log(max(1e-6, 1.0 - p12)) / max(0.5, 12.0 - grace_s)


def look_probability(p, phone=False, odds_ratio=4.0):
    """A walker's probability to look: p, or for a texting walker the odds of looking divided by odds_ratio."""
    if not phone:
        return p
    o = p / max(1e-6, 1.0 - p) / odds_ratio
    return o / (1.0 + o)


def ttc(ax, ay, avx, avy, bx, by, bvx, bvy, radius):
    """(time to collision s or None, distance m): when, moving on as now, the two discs (combined radius) first
    touch; None when they pass clear of each other (an oncoming car in its own lane) or move apart."""
    px, py = bx - ax, by - ay
    vx, vy = bvx - avx, bvy - avy
    d = math.hypot(px, py)
    if d <= radius:
        return 0.0, d
    a = vx * vx + vy * vy
    b = 2.0 * (px * vx + py * vy)
    c = px * px + py * py - radius * radius
    disc = b * b - 4.0 * a * c
    if a < 1e-6 or b >= 0.0 or disc < 0.0:
        return None, d
    return (-b - math.sqrt(disc)) / (2.0 * a), d


def closest(ax, ay, avx, avy, bx, by, bvx, bvy):
    """(time s, distance m) of the closest approach of two road users moving on as now; time None when they are
    moving apart."""
    px, py = bx - ax, by - ay
    vx, vy = bvx - avx, bvy - avy
    a = vx * vx + vy * vy
    if a < 1e-6:
        return None, math.hypot(px, py)
    t = -(px * vx + py * vy) / a
    if t <= 0.0:
        return None, math.hypot(px, py)
    return t, math.hypot(px + vx * t, py + vy * t)


def behind(pose_x, pose_y, yaw_deg, ox, oy, oyaw, reach=15.0, lat=1.7):
    """Distance (m) of (ox, oy) heading oyaw behind a vehicle at (pose_x, pose_y) heading yaw_deg in the same
    direction and lane, or None."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    dx, dy = ox - pose_x, oy - pose_y
    along, side = dx * c + dy * s, -dx * s + dy * c
    if -reach <= along <= -2.0 and abs(side) <= lat and abs(((oyaw - yaw_deg + 180) % 360) - 180) < 35:
        return -along
    return None


class Reactions:
    """Runner hook: step(t, ...) every few ticks; writes its events and sounds through the LivingCity."""

    def __init__(self, city, parts, sync, crowd, cmap, signals_light=None, sumo_yield=None):
        self.city, self.parts, self.sync, self.crowd, self.cmap = city, parts, sync, crowd, cmap
        self.lc = city.lc
        self.opts = {**DEFAULTS, **(self.lc.get("reactions") or {})}
        self.sumo_yield = sumo_yield
        self.block = {}          # participant role -> {"since", "follower", "last_honk", "honks"}
        self.lane = {}           # participant role -> (road, lane)
        self.courtesy = {}       # (role, sumo id) -> decided
        self.looking = {}        # crowd index -> (vehicle id, since)
        self.decided = set()
        self.bell_t = {}
        self.pairs = {}          # near-miss pair -> last logged t
        self.recent = []         # (t, actor id, what) of reactions, for the near-miss log
        self.speeds = {}         # actor id -> [(t, speed)] for decelerations
        self.t = 0.0
        self.queue_wait = {}

    # -- helpers --
    def _sumo_ids(self):
        return {cid: sid for sid, cid in self.sync.sumo2carla_ids.items() if sid not in self.sync.adopted}

    def _honk(self, cid, reason, **kin):
        actor = self.city.world.get_actor(cid)
        self.city.ev("horn", cid, reason=reason, **kin)
        if actor is not None:
            self.city.sound("horn", actor, reason=reason)
            loc = actor.get_location()
            self._startle(loc.x, loc.y)
        self.recent.append((self.t, cid, "horn"))

    def _startle(self, x, y, reach=15.0):
        """Walkers within reach of a horn: crossing ones hurry, standing ones look at it."""
        if self.crowd is None:
            return
        for k, st in enumerate(self.crowd.state):
            if not st["alive"]:
                continue
            px, py = st["pos"]
            if math.hypot(px - x, py - y) > reach:
                continue
            if st["crossing"]:
                st["hurry"], st["hurry_until"] = _rate_h(self.lc), self.t + 3.0
            elif st["standing"] is not None and k not in self.looking:
                self._look(k, st, x, y, None)

    def _part_state(self, p):
        try:
            tf = p.actor.get_transform()
            v = p.actor.get_velocity()
        except RuntimeError:
            return None
        return tf, v, math.hypot(v.x, v.y)

    # -- the participant blocking traffic, cutting in, signalling --
    def _blocking(self, t, p, tf, speed, poses, ids, wp=None):
        rng = rng_for(self.lc, "horn", p.role, int(t))
        b = self.block.setdefault(p.role, {"since": None, "last": -1e9, "honks": 0})
        x, y, yaw = tf.location.x, tf.location.y, tf.rotation.yaw
        go = speed < 0.5
        light = getattr(p, "light", None)
        green = light is not None and light[1] < 40.0 and light[2] in GREEN
        if go and light is not None and light[1] < 40.0 and not green:
            go = False                         # waiting at red or yellow: nobody honks
        if go and not green and wp is not None and _near_junction(wp):
            go = False                         # waiting to enter or cross a junction (give way): nobody honks
        if go:     # something standing in front of the participant (a queue, a walker): nobody honks either
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            ext = p.actor.bounding_box.extent.x if hasattr(p.actor, "bounding_box") else 2.4
            for cid, (ox, oy, _yaw, _sp, hl, *_r) in poses.items():
                along, side = (ox - x) * c + (oy - y) * s, -(ox - x) * s + (oy - y) * c
                if 0 < along < ext + hl + 6.0 and abs(side) < 1.8:
                    go = False
                    break
            if go and self.crowd is not None:
                for st in self.crowd.state:
                    if st["alive"]:
                        wx, wy = st["pos"]
                        along, side = (wx - x) * c + (wy - y) * s, -(wx - x) * s + (wy - y) * c
                        if 0 < along < ext + 8.0 and abs(side) < 2.0:
                            go = False
                            break
        follower = None
        if go:
            for cid, (ox, oy, oyaw, osp, *_r) in poses.items():
                d = behind(x, y, yaw, ox, oy, oyaw)
                if d is not None and osp < 0.5 and cid in ids:
                    follower = (cid, d)
                    break
        if not (go and follower):
            b["since"] = None
            return
        if b["since"] is None:
            b["since"] = t
            self.city.ev("lc_blocked", follower[0], target=p.role, gap_m=round(follower[1], 1),
                         light=(light[2] if light else None))
        waited = t - b["since"]
        grace = rate(self.lc, "honk_grace_s")
        if waited < grace or t - b["last"] < 4.0:
            return
        lam = honk_hazard(rate(self.lc, "honk_p12"), grace) * (0.5 if b["honks"] else 1.0)
        if rng.random() < lam * self.dt:
            b["last"], b["honks"] = t, b["honks"] + 1
            self._honk(follower[0], "blocked", target=p.role, waited_s=round(waited, 1),
                       gap_m=round(follower[1], 1), light=(light[2] if light else None))

    def _lanes(self, t, p, tf, speed, poses, ids, wp):
        import carla
        if wp is None:
            return
        key = (wp.road_id, wp.lane_id)
        old = self.lane.get(p.role)
        self.lane[p.role] = key
        x, y, yaw = tf.location.x, tf.location.y, tf.rotation.yaw
        # signalling: a driver in the target lane may let the participant in
        try:
            lights = int(p.actor.get_light_state())
        except RuntimeError:
            lights = 0
        import carla as _c
        left = bool(lights & int(_c.VehicleLightState.LeftBlinker))
        right = bool(lights & int(_c.VehicleLightState.RightBlinker))
        if self.opts.get("courtesy", True) and left != right:
            nb = wp.get_left_lane() if left else wp.get_right_lane()
            if nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * wp.lane_id > 0:
                nx, ny = nb.transform.location.x, nb.transform.location.y
                c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                near = None          # the driver nearest behind (or beside) in the target lane decides
                for cid, (ox, oy, oyaw, osp, *_r) in poses.items():
                    sid = ids.get(cid)
                    if sid is None:
                        continue
                    along = (ox - x) * c + (oy - y) * s
                    lat = -(ox - nx) * s + (oy - ny) * c
                    if -30.0 < along < 3.0 and abs(lat) < 1.6 and abs(((oyaw - yaw + 180) % 360) - 180) < 35 and \
                            (near is None or along > near[2]):
                        near = (cid, sid, along, osp)
                if near is not None and (p.role, near[1]) not in self.courtesy:
                    cid, sid, along, osp = near
                    rng = rng_for(self.lc, "courtesy", p.role, sid)
                    yes = rng.random() < rate(self.lc, "courtesy_p")
                    self.courtesy[(p.role, sid)] = yes
                    if yes:
                        self._yield_flash(cid, sid, osp, p.role, -along)
        # a lane change: the driver now behind in the new lane, if cut off, may honk
        if old is not None and key != old and old[0] == key[0] and self.opts.get("horns", True):
            for cid, (ox, oy, oyaw, osp, hl, *_r) in poses.items():
                d = behind(x, y, yaw, ox, oy, oyaw, reach=30.0, lat=1.5)
                if d is None or cid not in ids:
                    continue
                gap = max(0.1, d - hl - p.actor.bounding_box.extent.x)
                thw = gap / max(0.1, osp)
                need = max(0.0, osp - speed) ** 2 / (2.0 * gap) if osp > speed else 0.0
                if thw < 0.8 or need > 3.0:
                    rng = rng_for(self.lc, "cut_in", p.role, cid, int(t))
                    honk = rng.random() < rate(self.lc, "honk_cut_in_p")
                    self.city.ev("lc_cut_in", cid, target=p.role, gap_m=round(gap, 1), thw_s=round(thw, 2),
                                 decel_needed=round(need, 2), v_follower=round(osp, 1), v_participant=round(speed, 1),
                                 honk=honk)
                    if honk:
                        self._honk(cid, "cut_in", target=p.role, thw_s=round(thw, 2), decel_needed=round(need, 2))
                break

    def _yield_flash(self, cid, sid, speed, role, gap):
        import traci
        try:
            traci.vehicle.slowDown(sid, max(0.0, speed - 3.0), 3.0)
            traci.vehicle.setSignals(sid, 64 | 16)              # SUMO high beam (+ low beam): a flash
        except traci.TraCIException:
            return
        self.city.at(self.t + 0.6, _signals_reset, sid)
        self.city.at(self.t + 4.0, _speed_reset, sid)
        self.city.ev("lc_yield_flash", cid, target=role, gap_m=round(gap, 1), speed=round(speed, 1))
        self.recent.append((self.t, cid, "yield_flash"))

    def _queue_honks(self, t, poses, ids):
        """SUMO drivers queued behind a living-city vehicle standing in the lane (a double-parked van, a taxi)."""
        stands = [(v.get("actor"), cid) for cid, v in self.city.vehicles.items()
                  if (v["kind"] == "delivery" and v.get("double")) or (v["kind"] == "pickup" and v.get("stopped"))]
        for actor, cid in stands:
            pose = poses.get(cid)
            if pose is None and actor is not None:
                try:
                    tf = actor.get_transform()
                    pose = (tf.location.x, tf.location.y, tf.rotation.yaw, 0.0)
                except RuntimeError:
                    continue
            if pose is None:
                continue
            for ocid, (ox, oy, oyaw, osp, *_r) in poses.items():
                if ocid == cid or ocid not in ids:
                    continue
                d = behind(pose[0], pose[1], pose[2], ox, oy, oyaw, reach=12.0)
                if d is None or osp > 0.5:
                    self.queue_wait.pop(ocid, None)
                    continue
                since = self.queue_wait.setdefault(ocid, t)
                rng = rng_for(self.lc, "queue_honk", ocid, int(t))
                if t - since > 10.0 and rng.random() < rate(self.lc, "honk_queue_per_s") * self.dt:
                    self._honk(ocid, "queued", behind=cid, waited_s=round(t - since, 1))

    def _ped_blocking(self, t):
        """A pedestrian participant standing in the road holds a SUMO car (yielding.py) for over 4 s: it may honk."""
        held = getattr(self.sumo_yield, "held", None) or {}
        peds = [p for p in self.parts if p.kind == "pedestrian"]
        if not held or not peds:
            return
        ids = {sid: cid for sid, cid in self.sync.sumo2carla_ids.items()}
        lam = honk_hazard(rate(self.lc, "honk_p12"), rate(self.lc, "honk_grace_s"))
        for sid, since in held.items():
            cid = ids.get(sid)
            pose = (self.sync.poses or {}).get(cid)
            if cid is None or pose is None or pose[3] > 0.3:
                continue
            for p in peds:
                try:
                    loc = p.actor.get_location()
                except RuntimeError:
                    continue
                c, s = math.cos(math.radians(pose[2])), math.sin(math.radians(pose[2]))
                along = (loc.x - pose[0]) * c + (loc.y - pose[1]) * s
                if 0 < along < pose[4] + 6.0 and abs(-(loc.x - pose[0]) * s + (loc.y - pose[1]) * c) < 2.0:
                    b = self.block.setdefault(("ped", sid), {"since": t, "last": -1e9})
                    rng = rng_for(self.lc, "ped_horn", sid, int(t))
                    if t - b["since"] > 4.0 and t - b["last"] > 5.0 and rng.random() < lam * self.dt:
                        b["last"] = t
                        self._honk(cid, "pedestrian_blocking", target=p.role, waited_s=round(t - b["since"], 1))

    # -- pedestrians --
    def _vehicles(self):
        """[(id, x, y, vx, vy, is participant)] of the vehicles: SUMO's (poses, velocities) and the participants'."""
        out = []
        vel = getattr(self.sync, "velocity", {}) or {}
        mine = {p.actor.id for p in self.parts}          # a participant driven by SUMO is in the poses too
        for cid, (x, y, *_r) in (self.sync.poses or {}).items():
            if cid in mine:
                continue
            vx, vy = vel.get(cid, (0.0, 0.0))
            out.append((cid, x, y, vx, vy, False))
        for p in self.parts:
            if not p.actor.type_id.startswith("vehicle."):
                continue
            try:
                loc, v = p.actor.get_location(), p.actor.get_velocity()
            except RuntimeError:
                continue
            out.append((p.actor.id, loc.x, loc.y, v.x, v.y, p.role))
        return out

    def _look(self, k, st, x, y, vid, part=None, d=None, tta=None):
        actor = self.crowd.actors[k]
        px, py = st["pos"]
        hx, hy = st["heading"]
        body = math.degrees(math.atan2(hy, hx))
        bearing = math.degrees(math.atan2(y - py, x - px))
        turn = ((bearing - body + 180) % 360) - 180
        if self.city.poser.look(actor, turn):
            self.looking[k] = (vid, self.t)
            self.city.ev("lc_look", actor.id, vehicle=vid, participant=part, turn_deg=round(turn),
                         d=round(d, 1) if d is not None else None, tta=round(tta, 1) if tta is not None else None)

    def _peds(self, t, vehicles):
        if self.crowd is None:
            return
        looks, hurry = self.opts.get("looks", True), self.opts.get("hurry", True)
        foc = self.city._focus
        for k, st in enumerate(self.crowd.state):
            if not st["alive"]:
                if k in self.looking:
                    self.looking.pop(k)
                continue
            px, py = st["pos"]
            if foc and all((px - c.x) ** 2 + (py - c.y) ** 2 > 80.0 ** 2 for c in foc):
                continue
            hx, hy = st["heading"]
            wvx, wvy = hx * st["cmd"], hy * st["cmd"]
            best = None          # the vehicle coming closest soonest: passing within 10 m in under 6 s
            for vid, x, y, vx, vy, part in vehicles:
                d = math.hypot(x - px, y - py)
                if d > 30.0:
                    continue
                tt, dmin = closest(px, py, wvx, wvy, x, y, vx, vy)
                if tt is not None and tt < 6.0 and dmin < 10.0 and (best is None or tt < best[1]):
                    best = (vid, tt, d, x, y, part, dmin)
            # looks: standing walkers turn their head to the approaching vehicle, and back once it is gone
            if looks:
                cur = self.looking.get(k)
                if cur is not None and (best is None or best[0] != cur[0]) and t - cur[1] > 1.0:
                    self.looking.pop(k)
                    if st["standing"] is not None:
                        self.city.poser.unlook(self.crowd.actors[k])
                if best is not None and st["standing"] is not None and k not in self.looking and \
                        (k, best[0]) not in self.decided:
                    self.decided.add((k, best[0]))
                    rng = rng_for(self.lc, "look", k, best[0])
                    p = look_probability(rate(self.lc, "look_p"), st.get("phone"), rate(self.lc, "phone_look_odds"))
                    if rng.random() < p:
                        self._look(k, st, best[3], best[4], best[0], best[5], best[2], best[1])
            if not hurry or best is None:
                continue
            if st["crossing"] and best[1] < 3.0 and best[6] < 3.0 and t >= st["hurry_until"]:
                route = st["route"][st["leg"]] if st["leg"] < len(st["route"]) else None
                (ax, ay) = route[1][0] if route and route[1] else (px, py)
                if route and route[0] == "cross" and math.hypot(px - ax, py - ay) < 1.2 and \
                        best[1] < rate(self.lc, "step_back_tta_s"):
                    st["pt"], st["crossing"], st["wait_from"] = 0, False, t
                    st["back_until"] = t + 0.8
                    self.city.ev("lc_step_back", self.crowd.actors[k].id, vehicle=best[0], tta=round(best[1], 2),
                                 d=round(best[2], 1), participant=best[5])
                    self.recent.append((t, self.crowd.actors[k].id, "step_back"))
                else:
                    st["hurry"], st["hurry_until"] = rate(self.lc, "hurry_factor"), t + 3.0
                    self.city.ev("lc_hurry", self.crowd.actors[k].id, vehicle=best[0], tta=round(best[1], 2),
                                 d=round(best[2], 1), participant=best[5])
                    self.recent.append((t, self.crowd.actors[k].id, "hurry"))

    def _bells(self, t, types):
        from .yielding import first_conflict
        walkers = []
        if self.crowd is not None:
            for st in self.crowd.state:
                if st["alive"]:
                    hx, hy = st["heading"]
                    walkers.append((st["pos"][0], st["pos"][1], hx * st["cmd"], hy * st["cmd"]))
        for p in self.parts:
            if p.kind == "pedestrian":
                try:
                    loc, v = p.actor.get_location(), p.actor.get_velocity()
                    walkers.append((loc.x, loc.y, v.x, v.y))
                except RuntimeError:
                    pass
        if not walkers:
            return
        for cid, pose in (self.sync.poses or {}).items():
            if (types.get(cid) or ("",))[0] != "bike" or pose[3] < 2.0 or t - self.bell_t.get(cid, -1e9) < 10.0:
                continue
            d = first_conflict(pose, 0.0, walkers)
            if d is None or d > 10.0:
                continue
            self.bell_t[cid] = t
            rng = rng_for(self.lc, "bell", cid, int(t))
            if rng.random() < rate(self.lc, "bell_p"):
                actor = self.city.world.get_actor(cid)
                self.city.ev("bell", cid, walker_m=round(d, 1), speed=round(pose[3], 1))
                if actor is not None:
                    self.city.sound("bell", actor)
                self.recent.append((t, cid, "bell"))

    # -- near misses --
    def _near_misses(self, t, vehicles, types):
        thr = rate(self.lc, "near_miss_ttc_s")
        users = [(vid, x, y, vx, vy, "participant:" + part if part else (types.get(vid) or ("vehicle",))[0],
                  RADIUS.get((types.get(vid) or ("car",))[0], 1.0)) for vid, x, y, vx, vy, part in vehicles]
        if self.crowd is not None:
            for k, st in enumerate(self.crowd.state):
                if st["alive"] and (st["crossing"] or st["standing"] in ("kerb",)):
                    hx, hy = st["heading"]
                    users.append((self.crowd.actors[k].id, st["pos"][0], st["pos"][1], hx * st["cmd"],
                                  hy * st["cmd"], "walker", 0.3))
        for p in self.parts:
            if p.kind == "pedestrian":
                try:
                    loc, v = p.actor.get_location(), p.actor.get_velocity()
                    users.append((p.actor.id, loc.x, loc.y, v.x, v.y, "participant:" + p.role, 0.3))
                except RuntimeError:
                    pass
        for uid, x, y, vx, vy, *_ in users:
            hist = self.speeds.setdefault(uid, [])
            hist.append((t, math.hypot(vx, vy)))
            while hist and t - hist[0][0] > 1.2:
                hist.pop(0)
        parts_ids = {u[0] for u in users if str(u[5]).startswith("participant")}
        n = len(users)
        for i in range(n):
            a = users[i]
            for j in range(i + 1, n):
                b = users[j]
                if a[0] not in parts_ids and b[0] not in parts_ids and "walker" not in (a[5], b[5]):
                    continue                  # SUMO-SUMO pairs: SUMO's own collision log covers them
                if a[5] == "walker" and b[5] == "walker":
                    continue
                if abs(a[1] - b[1]) > 20 or abs(a[2] - b[2]) > 20:
                    continue
                tt, d = ttc(a[1], a[2], a[3], a[4], b[1], b[2], b[3], b[4], a[6] + b[6])
                rel = math.hypot(a[3] - b[3], a[4] - b[4])
                if not ((tt is not None and tt < thr and d < 20.0) or (d - a[6] - b[6] < 0.5 and rel > 1.5)):
                    continue
                key = (min(a[0], b[0]), max(a[0], b[0]))
                if t - self.pairs.get(key, -1e9) < 5.0:
                    continue
                self.pairs[key] = t
                reacts = [what for (rt, aid, what) in self.recent if t - rt <= 2.0 and aid in key]
                self.city.ev("near_miss", a[0], other=b[0], kind_a=a[5], kind_b=b[5], d=round(d, 2),
                             ttc=round(tt, 2) if tt is not None else None, v_a=round(math.hypot(a[3], a[4]), 2),
                             v_b=round(math.hypot(b[3], b[4]), 2), rel_v=round(rel, 2),
                             decel_a=_decel(self.speeds.get(a[0])), decel_b=_decel(self.speeds.get(b[0])),
                             reactions=reacts, x=round(a[1], 1), y=round(a[2], 1))
        self.recent = [r for r in self.recent if t - r[0] <= 3.0]

    # -- the tick --
    def step(self, t, dt, types):
        self.t, self.dt = t, dt
        poses = self.sync.poses or {}
        ids = self._sumo_ids()
        if self.opts.get("horns", True) or self.opts.get("courtesy", True):
            for p in self.parts:
                if not p.actor.type_id.startswith("vehicle.") or p.kind == "av_passenger":
                    continue
                got = self._part_state(p)
                if got is None:
                    continue
                tf, v, speed = got
                import carla
                wp = self.cmap.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
                if self.opts.get("horns", True):
                    self._blocking(t, p, tf, speed, poses, ids, wp)
                self._lanes(t, p, tf, speed, poses, ids, wp)
            if self.opts.get("horns", True):
                self._queue_honks(t, poses, ids)
                self._ped_blocking(t)
        vehicles = self._vehicles()
        self._peds(t, vehicles)
        if self.opts.get("bells", True):
            self._bells(t, types)
        if self.opts.get("near_misses", True):
            self._near_misses(t, vehicles, types)


def _near_junction(wp, reach=10.0):
    """Whether a waypoint is in a junction or one is within reach metres ahead on its lane."""
    try:
        if wp.is_junction:
            return True
        return any(n.is_junction for n in wp.next(reach)) or any(n.is_junction for n in wp.next(reach / 2))
    except RuntimeError:
        return False


def _rate_h(lc):
    return rate(lc, "hurry_factor")


def _decel(hist):
    """Deceleration (m/s^2, positive when slowing) over the last second of speed samples, or None."""
    if not hist or len(hist) < 2 or hist[-1][0] - hist[0][0] < 0.2:
        return None
    return round((hist[0][1] - hist[-1][1]) / (hist[-1][0] - hist[0][0]), 2)


def _signals_reset(sid):
    import traci
    try:
        traci.vehicle.setSignals(sid, -1)
    except traci.TraCIException:
        pass


def _speed_reset(sid):
    import traci
    try:
        traci.vehicle.setSpeed(sid, -1)
    except traci.TraCIException:
        pass


def make(city, parts, sync, crowd, cmap, sumo_yield=None):
    if city is None or not (city.lc.get("reactions") or {}).get("enabled", True):
        return None
    return Reactions(city, parts, sync, crowd, cmap, sumo_yield=sumo_yield)


__all__ = ["Reactions", "RATES", "SOURCES", "DEFAULTS", "make", "honk_hazard", "look_probability", "ttc", "behind"]
