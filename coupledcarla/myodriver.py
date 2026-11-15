"""A musculoskeletal driver: MyoSuite muscles press the pedals and turn the wheel of a car in a trial.

`input: myosuite` on a driver participant (run on the host, no station). It wraps Mark Colley's myosuite-carla
(https://github.com/M-Colley/myosuite-carla, Apache-2.0), used as an installed package, no code copied:

  body       MyoSuite's seated full body (myo_sim myofullbody, 416 muscles; `preset: fast` keeps the 114 that
             drive and steps MuJoCo at 2 ms) in front of a cockpit with spring-loaded pedals and a steering wheel
  driver     myosuite-carla's ReflexDriver: two-point visual steering (Salvucci & Gray, 2004), the Intelligent
             Driver Model for speed and the car ahead (Treiber et al., 2000), a 0.2 s perception delay; it turns
             what the driver sees into muscle excitations, never into vehicle controls
  controls   what the pedals and the wheel measure (travel, with deadzones and brake priority) becomes the
             car's carla.VehicleControl; CARLA moves the car
  loads      the car's acceleration and rotation load the body (d'Alembert forces) and the wheel's
             self-aligning torque, as in myosuite-carla

Coupled CARLA's side, every tick: the lane ahead (near and far points on CARLA's waypoints), the speed to keep, the
SUMO car ahead in the lane (the bridge's poses) and a red or amber light ahead (SUMO's next light, set by the
runner as for the autopilot; a light to stop at is a standing car for the IDM) go into the reflex driver; the
measured controls go back with the tick's batch. <trial>/myodriver.csv has a row per tick (intent, measured
controls, mechanism travel and contact forces, the largest driving-muscle force, lane offset, speed, the time the
body took), <trial>/myodriver_muscles.csv every driving muscle's activation and force every `muscles_every` ticks,
and <trial>/myodriver_summary.json lane keeping, speed keeping and cost per tick.

The body and its cockpit are myosuite-carla's generic rig, not a fitted person, and its reflex driver's gains were
tuned on CARLA 0.10's Nissan Patrol (CARLA 1.0 steers linearly, 0.10 through a squared curve): `steer_gains` and
`target_speed_kmh` are the knobs. Needs, on the host only and best in a separate venv: mujoco>=3.6,
gymnasium<1.3, shapely, MyoSuite's dev branch (myo_sim) and myosuite-carla. Without them the runner logs why and
CARLA's BasicAgent drives the car instead.
"""
import csv
import json
import math
import time

DEFAULTS = {
    "preset": "fast",            # fast: 2 ms physics and the 114 driving muscles (myosuite-carla's benchmarked
                                 # preset, 2.5x faster); full: 1 ms physics and all 416 muscles
    "target_speed_kmh": 30.0,    # speed kept on a free road
    "near_m": 8.0,               # near point on the lane ahead (two-point steering), m; nearer at low speed
    "near_s": 1.0,               # ... at most this many seconds ahead at the current speed, but at least 4 m
    "far_s": 2.0,                # far point: this many seconds ahead at the current speed (at least near_m + 2 m)
    "steer_gains": [6.0, 2.0, 5.0],   # [k_far, k_near, k_integral] of two-point steering: myosuite-carla's (6, 2, 1)
                                 # with the integral gain raised to 5 (on its vehicle surrogate: lane RMS in a 60 m
                                 # bend at 50 km/h 2.1 -> 0.9 m, in a 15 m bend at 30 km/h 1.8 -> 1.5 m); None: (6, 2, 1)
    "steer_exponent": 2.0,       # the measured wheel position goes to CARLA as sign(s) |s|^e: the reflex driver's gains
                                 # were identified on CARLA 0.10, which steers by the square of the input (exponent
                                 # 1.97); CARLA 0.9.16 and 1.0 steer linearly (1: pass the wheel through unchanged)
    "perception_delay_s": 0.2,   # delay of what the driver perceives (ReflexDriver)
    "speed_trim": 0.1,           # 1/s: a slow integral of the speed error on a free road added to the reflex driver's
                                 # desired acceleration (a glance at the speedometer; its throttle map was fitted on
                                 # another car: without it 25 km/h for 30 on 0.9.16's Lincoln); 0: off
    "curve_lat_accel": 2.0,      # drivers slow for bends to keep the lateral acceleration under this, m/s^2
                                 # (assumed; Reymond et al. 2001), braking at most 1.5 m/s^2 before them; 0: off
    "lead_range_m": 60.0,        # SUMO vehicles ahead in the lane within this distance are followed (IDM)
    "stop_for_lights": True,     # a red or amber light ahead is a standing car at its stop line for the IDM
    "strength": 1.0,             # peak-force scale of every muscle (0.5: sarcopenia, MyoSuite's apply_sarcopenia)
    "motor_noise": 0.0,          # signal-dependent motor noise, fraction of the command (MyoSuite's MotorNoiseCfg)
    "fatigue": False,            # MyoSuite's 3CC-r muscle fatigue every physics step
    "vehicle_loads": True,       # the car's acceleration and rotation load the body
    "seed": 0,                   # motor noise
    "log_every": 1,              # myodriver.csv every N ticks (0: off)
    "muscles_every": 25,         # myodriver_muscles.csv every N ticks (0: off)
    "reset_off_lane_m": 3.5,     # further than this from the lane centre: back onto the lane (reset_to_lane event)
    "stuck_s": 4.0,              # throttle on and no movement this long (on a kerb, against a car): back onto the lane
}
PLAN_STEP_M = 2.0                # waypoint spacing of the lane plan
PLAN_AHEAD_M = 120.0             # the plan reaches at least this far ahead
ACC_CLIP = 15.0                  # m/s^2: more (a crash, a teleport's one-frame spike) does not reach the body
SETTLE_S = 0.5                   # after a reset the body sits at rest this long before the car's motion loads it
LANE_HALF_M = 1.8                # a SUMO vehicle within this lateral distance of the plan is in the lane


def options(cfg):
    """The participant's `myosuite` options over DEFAULTS; unknown keys raise ValueError."""
    got = dict((cfg or {}).get("myosuite") or {})
    bad = sorted(set(got) - set(DEFAULTS))
    if bad:
        raise ValueError(f"myosuite: unknown option(s) {', '.join(bad)}")
    return {**DEFAULTS, **got}


# --- geometry on the lane plan (plain Python, tested without CARLA) ---------------------------------------------------

def view_angle(x, y, yaw_deg, px, py):
    """Angle (rad) to point (px, py) from a car at (x, y) heading yaw_deg; positive to the right (CARLA's
    left-handed x, y), as two-point steering uses it."""
    h = math.radians(yaw_deg)
    fwd = (px - x) * math.cos(h) + (py - y) * math.sin(h)
    right = -(px - x) * math.sin(h) + (py - y) * math.cos(h)
    return math.atan2(right, max(fwd, 1e-6))


class LanePlan:
    """A polyline of lane-centre points [(x, y)] the car follows, with arc lengths."""

    def __init__(self, pts):
        self.pts = list(pts)
        self.s = [0.0]
        for (x0, y0), (x1, y1) in zip(self.pts, self.pts[1:]):
            self.s.append(self.s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.i = 0          # segment the car was last projected on

    @property
    def length(self):
        return self.s[-1]

    def extend(self, pts):
        for x, y in pts:
            x0, y0 = self.pts[-1]
            self.pts.append((x, y))
            self.s.append(self.s[-1] + math.hypot(x - x0, y - y0))

    def project(self, x, y, window=40):
        """(arc length, signed lateral offset: + right of the plan) of (x, y), searching forward from the last
        segment (the car does not go back)."""
        best = None
        lo, hi = max(0, self.i - 2), min(len(self.pts) - 1, self.i + window)
        for k in range(lo, max(lo + 1, hi)):
            if k + 1 >= len(self.pts):
                break
            (x0, y0), (x1, y1) = self.pts[k], self.pts[k + 1]
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy or 1e-9
            u = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
            cx, cy = x0 + u * dx, y0 + u * dy
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if best is None or d2 < best[0]:
                L = math.sqrt(L2)
                side = (-(x - x0) * dy + (y - y0) * dx) / L      # CARLA: + right of the direction of travel
                best = (d2, k, self.s[k] + u * L, side)
        if best is None:
            return self.s[-1], 0.0
        self.i = best[1]
        return best[2], best[3]

    def point(self, s):
        """The point at arc length s (clamped to the plan)."""
        s = max(0.0, min(self.s[-1], s))
        k = max(0, min(len(self.s) - 2, self.i))
        while k > 0 and self.s[k] > s:
            k -= 1
        while k + 1 < len(self.s) - 1 and self.s[k + 1] < s:
            k += 1
        if len(self.pts) == 1:
            return self.pts[0]
        seg = self.s[k + 1] - self.s[k] or 1e-9
        u = (s - self.s[k]) / seg
        (x0, y0), (x1, y1) = self.pts[k], self.pts[k + 1]
        return x0 + u * (x1 - x0), y0 + u * (y1 - y0)

    def drop_behind(self, keep_m=20.0):
        """Forget points more than keep_m behind the car (the plan grows ahead)."""
        s_car = self.s[self.i] if self.i < len(self.s) else 0.0
        k = 0
        while k + 1 < len(self.s) and self.s[k + 1] < s_car - keep_m:
            k += 1
        if k > 0:
            self.pts, s0 = self.pts[k:], self.s[k]
            self.s = [v - s0 for v in self.s[k:]]
            self.i -= k


def lead_on_plan(plan, s_car, poses, lane_half=LANE_HALF_M, reach=60.0, ego_half_len=2.3):
    """(bumper gap m, speed m/s) of the nearest vehicle ahead on the plan, from the bridge's poses {id: (x, y, yaw,
    speed, half length, half width)}; None when none is within reach."""
    best = None
    for x, y, _yaw, speed, hl, _hw in (poses or {}).values():
        k0 = plan.i
        s, off = plan.project(x, y, window=int(reach / PLAN_STEP_M) + 10)
        plan.i = k0                          # projecting another car must not move the car's own segment
        ahead = s - s_car
        if 0.0 < ahead <= reach and abs(off) <= lane_half:
            gap = ahead - hl - ego_half_len
            if best is None or gap < best[0]:
                best = (max(0.1, gap), float(speed))
    return best


def curve_speed(plan, s_car, reach, a_lat, decel=1.5, step=2.0, span=6.0):
    """Highest speed from which the car can still slow (at decel) for every bend on the plan within reach to take it
    at lateral acceleration a_lat: min over points ahead of sqrt(a_lat / curvature + 2 decel distance). Curvature
    from the headings of the chords either side of each point. None: no bend that limits the speed."""
    best, d = None, 0.0
    while d <= reach:
        s0 = s_car + d
        (x0, y0), (x1, y1), (x2, y2) = plan.point(s0 - span / 2), plan.point(s0), plan.point(s0 + span / 2)
        h1, h2 = math.atan2(y1 - y0, x1 - x0), math.atan2(y2 - y1, x2 - x1)
        turn = abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
        seg = math.hypot(x1 - x0, y1 - y0) + math.hypot(x2 - x1, y2 - y1)
        if seg > 0.5 * span and turn > 1e-3:
            v = math.sqrt(a_lat * 0.5 * seg / turn + 2.0 * decel * d)    # the chords' headings: half a span apart
            best = v if best is None else min(best, v)
        d += step
    return best


def light_as_lead(light, speed, ego_half_len=2.3):
    """A red or amber light ahead (runner: (tls id, distance m, state)) as a standing car at its stop line, (gap,
    speed 0), or None: green, none, or amber too close to stop (go on, as the autopilot does)."""
    if not light or light[2] not in "rRyYs":
        return None
    dist = float(light[1])
    if light[2] in "yY" and dist < speed * 1.0:
        return None
    return max(0.1, dist - ego_half_len - 1.0), 0.0


class Stats:
    """Lane keeping, speed keeping and cost per tick for myodriver_summary.json."""

    def __init__(self):
        self.n = 0
        self.sum_off2 = self.max_off = 0.0
        self.speed_err = []
        self.ms = []
        self.resets = 0

    def add(self, offset, speed, target, ms, free_road):
        self.n += 1
        self.sum_off2 += offset * offset
        self.max_off = max(self.max_off, abs(offset))
        if free_road:
            self.speed_err.append(speed - target)
        self.ms.append(ms)

    def summary(self):
        ms = sorted(self.ms)

        def pct(v, q):
            return round(v[min(len(v) - 1, int(q * len(v)))], 2) if v else None
        se = self.speed_err
        return {"ticks": self.n, "lane_offset_rms_m": round(math.sqrt(self.sum_off2 / self.n), 3) if self.n else None,
                "lane_offset_max_m": round(self.max_off, 3), "resets": self.resets,
                "free_road_ticks": len(se),
                "speed_error_mean_mps": round(sum(se) / len(se), 3) if se else None,
                "speed_error_rms_mps": round(math.sqrt(sum(e * e for e in se) / len(se)), 3) if se else None,
                "body_ms_mean": round(sum(ms) / len(ms), 2) if ms else None, "body_ms_p50": pct(ms, 0.5),
                "body_ms_p95": pct(ms, 0.95)}


# --- the input ------------------------------------------------------------------------------------------------------

class MyoDriver:
    """inputs.make("myosuite", vehicle). attach() (the runner, before the first tick) builds the body; control()
    returns the car's VehicleControl each tick, from the muscles."""
    needs_display = False

    def __init__(self, vehicle, **_):
        self.vehicle = vehicle
        self.light, self.quit, self.stuck, self.reset = None, False, False, False
        self.events = []
        self.opt = dict(DEFAULTS)
        self.body = self.reflex = self.plan = self.cmap = None
        self.poses = None
        self.sample = None
        self.stats = Stats()
        self._csv = self._mcsv = None
        self._files = []
        self.out = None
        self.tick = 0
        self.t = 0.0
        self._stuck_s = 0.0
        self._reset_t = -1e9
        self._settle_until = 0.0
        self._acc_i = 0.0
        self.build_s = None

    # the runner calls attach(sync, net, geo, participant config, offset, log) for inputs that have it (sumodriver)
    def attach(self, sync, net, geo, p, offset, log=print):
        self.opt = options(p)
        o = self.opt
        try:
            import mujoco  # noqa: F401
            from myosuite_carla import FullBodyDriverConfig, build_fullbody_driver
            from myosuite_carla.driver_models import (IdmConfig, ReflexDriver, ReflexDriverConfig,
                                                      TwoPointSteeringConfig)
        except ImportError as err:
            raise RuntimeError(f"input myosuite needs mujoco, MyoSuite (dev) and myosuite-carla on the host ({err})") \
                from err
        world = self.vehicle.get_world()
        step = float(world.get_settings().fixed_delta_seconds or 0.02)
        kw = {"control_dt": step, "strength": float(o["strength"]), "fatigue": bool(o["fatigue"]),
              "motor_noise_signal": float(o["motor_noise"]), "vehicle_loads": bool(o["vehicle_loads"])}
        t0 = time.time()
        try:
            import warnings
            cfg = FullBodyDriverConfig.fast(**kw) if o["preset"] == "fast" else FullBodyDriverConfig(**kw)
            with warnings.catch_warnings():      # MuJoCo's "spec with an empty prefix" on every compile
                warnings.simplefilter("ignore", UserWarning)
                self.body = build_fullbody_driver(cfg)
        except Exception as err:  # noqa: BLE001 - e.g. a step that is no whole number of physics steps, no myo_sim
            raise RuntimeError(f"myosuite: no musculoskeletal driver ({type(err).__name__}: {err})") from err
        self.body.reset(seed=int(o["seed"]))
        self.build_s = time.time() - t0
        gains = o["steer_gains"]
        steering = TwoPointSteeringConfig(*map(float, gains)) if gains else TwoPointSteeringConfig()
        self.reflex = ReflexDriver(step, ReflexDriverConfig(perception_delay=float(o["perception_delay_s"]),
                                                            steering=steering, idm=IdmConfig()))
        self.step_s = step
        self.poses = getattr(sync, "poses", None)
        self.cmap = world.get_map()
        self.restart(self.cmap.get_waypoint(self.vehicle.get_location()))
        import mujoco as mj
        self._muscle_ids = list(self.body._driving_muscles)
        self._muscle_names = [mj.mj_id2name(self.body.model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in self._muscle_ids]
        n_mus = int((self.body.model.actuator_dyntype == mj.mjtDyn.mjDYN_MUSCLE).sum())
        log(f"myosuite: musculoskeletal driver built in {self.build_s:.1f} s ({o['preset']}: {n_mus} muscles, "
            f"{1000 * cfg.timestep:.0f} ms physics, {len(self._muscle_ids)} driving muscles logged), "
            f"{o['target_speed_kmh']:.0f} km/h")
        self.events.append(("myosuite_driver", f"{o['preset']} body, {n_mus} muscles, built in {self.build_s:.1f} s"))

    def restart(self, wp):
        """Plan along the lane from waypoint wp (at the start, and after the runner put the car back on a lane)."""
        if wp is None:
            return False
        pts, w = [], wp
        pts.append((w.transform.location.x, w.transform.location.y))
        self._last_wp = w
        self.plan = LanePlan(pts)
        self._grow()
        if self.reflex is not None:
            self.reflex.reset()
        if self.body is not None and self.tick:   # put back on the lane: the driver takes the wheel again
            self.body.reset()                     # (a teleport's jolt had thrown the hands off it: CARLA 1.0, 15 Nov)
            self.sample = None
            self._settle_until = self.t + SETTLE_S
        return self.plan.length > 10.0

    def _grow(self):
        """Extend the plan along the lane, straight on at junctions (the successor whose heading changes least)."""
        w, new = self._last_wp, []
        while self.plan.length + len(new) * PLAN_STEP_M < (self.plan.s[self.plan.i] + PLAN_AHEAD_M):
            nxt = w.next(PLAN_STEP_M)
            if not nxt:
                break
            yaw = w.transform.rotation.yaw
            w = min(nxt, key=lambda c: abs((c.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0))
            new.append((w.transform.location.x, w.transform.location.y))
        self._last_wp = w
        if new:
            self.plan.extend(new)

    def open_log(self, out):
        """The runner: the trial's folder, once the logs start."""
        self.out = out
        if self.opt["log_every"]:
            f = open(out / "myodriver.csv", "w", newline="")
            self._files.append(f)
            self._csv = csv.writer(f, delimiter=";")
            self._csv.writerow(["timestamp", "tick", "speed_mps", "target_speed_mps", "lane_offset_m", "near_angle_rad",
                                "far_angle_rad", "lead_gap_m", "lead_speed_mps", "intent_right_foot",
                                "intent_left_foot", "intent_steer", "throttle", "brake", "steer", "throttle_travel_m",
                                "brake_travel_m", "steering_travel_rad", "carla_steer", "throttle_contact_n", "brake_contact_n",
                                "steering_contact_n", "largest_muscle_n", "desired_accel_mps2", "body_ms"])
        if self.opt["muscles_every"]:
            f = open(out / "myodriver_muscles.csv", "w", newline="")
            self._files.append(f)
            self._mcsv = csv.writer(f, delimiter=";")
            self._mcsv.writerow(["timestamp", "tick"] + [f"act_{n}" for n in self._muscle_names] +
                                [f"force_{n}" for n in self._muscle_names])

    def control(self, vehicle, events=()):
        import carla
        from myosuite_carla.vehicle_loads import VehicleMotion
        o, plan = self.opt, self.plan
        tf = vehicle.get_transform()
        v, acc, w = vehicle.get_velocity(), vehicle.get_acceleration(), vehicle.get_angular_velocity()
        x, y, yaw = tf.location.x, tf.location.y, tf.rotation.yaw
        speed = math.hypot(v.x, v.y)
        if plan.length - plan.s[plan.i] < PLAN_AHEAD_M:
            self._grow()
        s_car, offset = plan.project(x, y)
        plan.drop_behind()
        s_car, offset = plan.project(x, y)
        near = max(4.0, min(float(o["near_m"]), speed * float(o["near_s"])))
        far = max(near + 2.0, speed * float(o["far_s"]))
        a_near = view_angle(x, y, yaw, *plan.point(s_car + near))
        a_far = view_angle(x, y, yaw, *plan.point(s_car + far))
        lead = lead_on_plan(plan, s_car, self.poses, reach=float(o["lead_range_m"]))
        stop = light_as_lead(self.light, speed) if o["stop_for_lights"] else None
        if stop is not None and (lead is None or stop[0] < lead[0]):
            lead = stop
        target = float(o["target_speed_kmh"]) / 3.6
        if o["curve_lat_accel"]:
            v_bend = curve_speed(plan, s_car, max(30.0, 4.0 * speed), float(o["curve_lat_accel"]))
            if v_bend is not None:
                target = min(target, max(2.0, v_bend))
        last = self.sample.command if self.sample is not None else None
        state = {"speed_mps": speed, "target_speed_mps": target, "near_angle_rad": a_near, "far_angle_rad": a_far,
                 "lead_present": 1.0 if lead else 0.0, "gap_m": lead[0] if lead else 100.0,
                 "closing_speed_mps": speed - lead[1] if lead else 0.0,
                 "input_steer": last.steer if last else 0.0, "input_throttle": last.throttle if last else 0.0,
                 "input_brake": last.brake if last else 0.0}
        t0 = time.perf_counter()
        trim = float(o["speed_trim"])
        if trim and lead is None and abs(target - speed) < 3.0:
            self._acc_i = max(-1.0, min(1.0, self._acc_i + trim * (target - speed) * self.step_s))
        intent = self.reflex.act(state, acceleration_offset=self._acc_i)
        if o["vehicle_loads"] and self.t >= self._settle_until:
            clip = (lambda a: max(-ACC_CLIP, min(ACC_CLIP, a)))
            self.body.set_vehicle_motion(VehicleMotion(acceleration=(clip(acc.x), clip(acc.y), clip(acc.z)),
                                                       angular_velocity=(w.x, w.y, w.z),
                                                       rotation=(tf.rotation.pitch, yaw, tf.rotation.roll)))
        self.body.advance(intent)
        self.sample = self.body.mapper.sample(self.body.data)
        ms = 1000.0 * (time.perf_counter() - t0)
        cmd = self.sample.command
        self.stats.add(offset, speed, target, ms, free_road=lead is None and target >= float(o["target_speed_kmh"]) / 3.6)
        self.tick += 1
        self.t += self.step_s
        if self._csv is not None and self.tick % int(o["log_every"]) == 0:
            m = self.body.measurement()
            self._csv.writerow([f"{self.t:.3f}", self.tick, f"{speed:.3f}", f"{target:.2f}", f"{offset:.3f}",
                                f"{a_near:.4f}", f"{a_far:.4f}", f"{lead[0]:.2f}" if lead else "",
                                f"{lead[1]:.2f}" if lead else "", f"{intent.right_foot:.4f}",
                                f"{intent.left_foot:.4f}", f"{intent.steer:.4f}", f"{cmd.throttle:.4f}",
                                f"{cmd.brake:.4f}", f"{cmd.steer:.4f}", f"{m['throttle_travel']:.5f}",
                                f"{m['brake_travel']:.5f}", f"{m['steering_travel']:.4f}",
                                f"{self.to_carla_steer(cmd.steer):.4f}",
                                f"{m['throttle_contact_newtons']:.2f}", f"{m['brake_contact_newtons']:.2f}",
                                f"{m['steering_contact_newtons']:.2f}", f"{m['largest_muscle_newtons']:.1f}",
                                f"{self.reflex.demand[0]:.3f}", f"{ms:.2f}"])
        if self._mcsv is not None and self.tick % int(o["muscles_every"]) == 0:
            d = self.body.data
            self._mcsv.writerow([f"{self.t:.3f}", self.tick] +
                                [f"{d.act[self.body.model.actuator_actadr[i]]:.4f}" for i in self._muscle_ids] +
                                [f"{d.actuator_force[i]:.1f}" for i in self._muscle_ids])
        if abs(offset) > float(o["reset_off_lane_m"]) and self.t - self._reset_t > 1.0:     # once per reset
            self._reset_t = self.t
            self.reset = True          # the runner puts the car back on the nearest lane and calls restart()
            self.stats.resets += 1
            self.events.append(("myosuite_off_lane", f"{offset:+.1f} m from the lane centre at {3.6 * speed:.0f} km/h"))
        if cmd.throttle > 0.25 and speed < 0.3:
            self._stuck_s += self.step_s
            if self._stuck_s >= float(o["stuck_s"]):
                self.stuck, self._stuck_s = True, 0.0    # the runner resets the car onto the lane, as for the autopilot
                self.stats.resets += 1
                self.events.append(("myosuite_stuck", f"throttle {cmd.throttle:.2f}, no movement for {o['stuck_s']} s, "
                                    f"{offset:+.1f} m from the lane centre"))
        else:
            self._stuck_s = 0.0
        return carla.VehicleControl(throttle=float(cmd.throttle), steer=self.to_carla_steer(cmd.steer),
                                    brake=float(cmd.brake), hand_brake=bool(cmd.hand_brake), reverse=bool(cmd.reverse))

    def to_carla_steer(self, s):
        e = float(self.opt["steer_exponent"])
        return float(math.copysign(abs(s) ** e, s)) if e != 1.0 else float(s)

    def close(self):
        """The runner, at the end of the trial: the summary, and the CSVs closed."""
        for f in self._files:
            f.close()
        self._files = []
        if self.out is not None and self.body is not None:
            s = {**self.stats.summary(), "preset": self.opt["preset"], "build_s": round(self.build_s or 0.0, 1),
                 "control_dt_s": self.step_s, "options": self.opt,
                 "source": "myosuite-carla (M. Colley, Apache-2.0) with MyoSuite myo_sim myofullbody"}
            (self.out / "myodriver_summary.json").write_text(json.dumps(s, indent=2))
