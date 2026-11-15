"""Crowd walkers that cross on what they see: eyes, peripheral vision and reaction times from embodied-human.

traffic.walkers.perception.model: embodied replaces the crowd's gap check (crowdpeds.gap_ok, which knows every car's
true position and speed) with one per walker that only knows what that walker has seen. The senses and timing are
Mark Colley's embodied-human (https://github.com/M-Colley/embodied-human, Apache-2.0; used as an installed package,
no code copied); the crossing rule and the gaze policy around them are this module's:

  eyes        embodied-human's main-sequence eyes: log-normal saccade latency (median 200 ms, Saslow 1967),
              duration 2.2 ms/deg + 21 ms (Carpenter 1988), nothing seen during a saccade; per walker the latency
              varies as in its Population (BASELINE_VARIATIONS)
  seeing      embodied-human's object vision (EccentricityNoise): what a walker sees of a car is noisier the further
              it is from where the walker looks, SD x (1 + eccentricity / 2 deg) (Strasburger et al. 2011), nothing
              beyond 90 deg, nothing during a saccade, one percept per car per fixation. While a car's optical
              expansion (car width x speed / distance^2) is above `looming_threshold` (0.003 rad/s, Hoffmann &
              Mortimer 1994) the walker sees its time to arrival (SD `tta_weber` x it); below, it sees only the
              distance (SD `distance_weber` x it) and assumes the car drives at `prior_speed_kmh`: people judge far
              and fast cars by distance (Oxley et al. 2005; Lobjois & Cavallo 2007)
  belief      per car a time to arrival and a distance with their uncertainty (a Kalman filter: run down between
              looks, fused with each look by its noise)
  gaze        at the kerb the walker looks at a car it has not seen well enough to judge, else along the road to
              the side it has not seen for longest (to the nearest car there when it knows of one), holding each
              fixation at least `fixation_s`
  decision    the crowd's own criterion, so that only perception differs: step out when, for every car the walker
              believes approaches, the believed time to reach the crossing is at least the HCM critical gap t_c
              (crowdpeds.critical_gap: width / speed + 2 s, shrinking while waiting), or at a marked crossing the car
              can still stop; every such car seen well enough (the believed gap's relative SD at most `max_rel_sd`,
              else the walker looks at it first); and both sides looked at within `recheck_s`
  reaction    from that decision to the first step: embodied-human's TWIN response time to a seen signal (Diederich
              & Colonius 2015); a car that turns up meanwhile cancels it

What it logs: events crowd_notices (a walker first fixates a car: distance and time away) and crowd_crosses with
the believed and the true gap; <trial>/crowd_gaze.csv every walker at the kerb every decision step (gaze direction
in the world, saccade, side looked at, believed and true smallest gap, decision). The crowd's gaze is in the log
only: CARLA's walkers do not turn their heads here.

Without embodied-human installed (pip install embodied-human from its repository; numpy and scipy only) the crowd
keeps crowdpeds.gap_ok, with one log line.
"""
import csv
import math
import zlib

DEFAULTS = {
    "model": "off",              # "embodied": crossing decisions from what each walker sees (embodiedped.py, needs
                                 # embodied-human); "off": the crowd's gap check on every car's true position
    "seed": 1,
    "distance_weber": 0.10,      # foveal SD of a seen distance, fraction of it (assumed)
    "tta_weber": 0.15,           # foveal SD of a time to arrival seen from a car's looming, fraction of it (assumed)
    "looming_threshold": 0.003,  # rad/s optical expansion below which a car's approach is not seen
    "prior_speed_kmh": 30.0,     # speed assumed for a car whose approach is not seen yet (assumed: urban 30 zones)
    "prior_sd": 0.2,             # SD of that assumed speed, fraction of it (assumed)
    "max_rel_sd": 0.35,          # a car is judged once the believed gap's relative SD is at most this (assumed)
    "recheck_s": 1.5,            # both sides must have been looked at within this time before stepping out
    "fixation_s": 0.3,           # shortest fixation before the gaze moves on (assumed)
    "side_gaze_deg": 85.0,       # where the walker looks for a side with no known car: down the road
    "reach_m": 150.0,            # cars further along the road than this are not judged (assumed)
    "log_gaze": True,            # <trial>/crowd_gaze.csv
}
CAR_WIDTH_M = 1.8                # optical expansion of an approaching car
SOURCES = {
    "embodied_human": ("embodied-human, M. Colley: main-sequence eyes, eccentricity noise, TWIN response, "
                       "Population variation (github.com/M-Colley/embodied-human)", "Apache-2.0"),
}


def options(wcfg):
    """traffic.walkers.perception over DEFAULTS ("embodied" alone is short for {model: embodied})."""
    p = (wcfg or {}).get("perception") or {}
    if isinstance(p, str):
        p = {"model": p}
    return {**DEFAULTS, **p}


def make(tc, events=None, log=print, eh=None):
    """The trial's decider, or None (perception off, or embodied-human missing: the crowd's gap check)."""
    o = options((tc.get("traffic") or {}).get("walkers"))
    if o["model"] in ("off", None, False):
        return None
    if o["model"] != "embodied":
        log(f"crowd: unknown perception model {o['model']!r}; the crowd's gap check instead")
        return None
    try:
        return EmbodiedCrossing(o, events, eh=eh)
    except ImportError as err:
        log(f"crowd: perception model embodied needs embodied-human ({err}); the crowd's gap check instead")
        return None


def embodied_human_api():
    """The parts of embodied-human used here (ImportError when it is not installed)."""
    import types
    import embodied_human
    from embodied_human.population import BASELINE_VARIATIONS, Population
    from embodied_human.response import Signal
    from embodied_human.vision.objects import Item
    return types.SimpleNamespace(create=embodied_human.create, Population=Population,
                                 BASELINE_VARIATIONS=BASELINE_VARIATIONS, Signal=Signal, Item=Item,
                                 version=getattr(embodied_human, "__version__", "?"))


def approach(a, b, x, y, vx, vy, reach=120.0):
    """A vehicle against the crossing line a-b: (on_crossing, distance to the line m, closing speed m/s, azimuth
    deg from the walker at a facing b, + right; None when it does not head for the crossing within reach)."""
    ax, ay = a
    ux, uy = b[0] - ax, b[1] - ay
    L = math.hypot(ux, uy) or 1.0
    ux, uy = ux / L, uy / L
    along = (x - ax) * ux + (y - ay) * uy
    off = -(x - ax) * uy + (y - ay) * ux
    if -2.0 <= along <= L + 2.0 and abs(off) < 2.5:
        return True, 0.0, 0.0, 0.0
    v = math.hypot(vx, vy)
    if v < 0.5:
        return None
    closing = -(vx * -uy + vy * ux) * (1 if off > 0 else -1)
    if closing <= 0.3 or abs(off) > reach:
        return None
    tta = abs(off) / closing
    hit = along + (vx * ux + vy * uy) * tta
    if not -3.0 <= hit <= L + 3.0:
        return None
    # direction from the walker, in the walker's frame (x forward along the crossing, y to its right: CARLA)
    fwd, right = (x - ax) * ux + (y - ay) * uy, -(x - ax) * uy + (y - ay) * ux
    return False, abs(off), closing, math.degrees(math.atan2(right, fwd))


class _Walker:
    """One walker's senses (embodied-human modules) and belief."""

    def __init__(self, eh, cfg, seed):
        import numpy as np
        self.eyes = eh.create("eyes", cfg.eyes.name, **dict(cfg.eyes.params))
        self.objects = eh.create("objects", cfg.objects.name, **dict(cfg.objects.params))
        self.response = eh.create("response", cfg.response.name, **dict(cfg.response.params))
        streams = np.random.SeedSequence(seed).spawn(3)
        for m, s in zip((self.eyes, self.objects, self.response), streams):
            m.reset(np.random.default_rng(s))
        self.belief = {}         # vehicle id -> [d, var_d, v, var_v, seen_speed, t]
        self.seen_side = {}      # side (-1 left, +1 right) -> last time fixated
        self.noticed = set()
        self.fix_since = 0.0
        self.t = None
        self.go_at = None
        self.crossing = None
        self.looks = 0


class EmbodiedCrossing:
    """decide() at the kerb, in place of crowdpeds.gap_ok."""

    def __init__(self, opts, events=None, eh=None):
        self.eh = eh if eh is not None else embodied_human_api()
        self.Item, self.Signal = self.eh.Item, self.eh.Signal
        self.population = self.eh.Population(variations=self.eh.BASELINE_VARIATIONS)
        self.o = dict(opts)
        self.events = events if events is not None else []
        self.walkers = {}
        self.last = {}           # the last crossing: believed and true gap, looks
        self._csv = self._file = None
        self.counts = {"decisions": 0, "crossed": 0, "cancelled": 0, "noticed": 0, "saccades": 0}

    def walker(self, key):
        w = self.walkers.get(key)
        if w is None:
            import numpy as np
            seed = zlib.crc32(f"{self.o['seed']}:{key}".encode())
            cfg = self.population.sample(np.random.default_rng(seed))
            w = self.walkers[key] = _Walker(self.eh, cfg, seed)
        return w

    # --- the decision ---------------------------------------------------------------------------------------------

    def decide(self, key, t, a, b, t_c, vehicles, ids, marked=False, decel=3.0, react=1.0):
        """Whether walker `key` steps out now from a to cross to b. vehicles: [(x, y, vx, vy)] (CARLA), ids their
        actor ids (the same car keeps its id from call to call); t_c the critical gap (crowdpeds.critical_gap)."""
        o, w = self.o, self.walker(key)
        if w.crossing != (a, b) or w.t is None or t < w.t or t - w.t > 2.0:     # a new wait at a kerb
            w.crossing, w.belief, w.seen_side, w.go_at, w.t = (a, b), {}, {}, None, t
            w.eyes.reset(w.eyes.rng, gaze=(0.0, 0.0))
            w.fix_since, w.looks = t, 0
        dt = max(0.0, t - w.t)
        w.t = t
        eye = w.eyes.step(t)
        gaze = eye.gaze[0]
        # what is there (true), and what the walker sees of it
        cars, blocked = {}, False
        for (x, y, vx, vy), vid in zip(vehicles, ids):
            ap = approach(a, b, x, y, vx, vy, o["reach_m"])
            if ap is None:
                continue
            if ap[0]:
                blocked = True           # a vehicle on the crossing: right in front, no need to judge it
                continue
            cars[vid] = ap[1:]
        self._predict(w, dt)
        # one percept per car per fixation (noise is drawn once per look, not averaged away within it)
        fix = eye.fixation_onset
        items = {}
        for vid, (d, v, az) in cars.items():
            if w.belief.get(vid, (None,) * 7)[6] == fix:
                continue
            items[f"d{vid}"] = self.Item((az, 0.0), d, max(0.5, o["distance_weber"] * d))
            if CAR_WIDTH_M * v / max(1.0, d) ** 2 >= o["looming_threshold"]:
                items[f"t{vid}"] = self.Item((az, 0.0), d / v, o["tta_weber"] * d / v)
        seen = w.objects.observe(items, eye.gaze, suppressed=eye.in_saccade) if items else {}
        for vid, (d, v, az) in cars.items():
            od = seen.get(f"d{vid}")
            if od is not None:
                self._fuse(w, vid, od, seen.get(f"t{vid}"), t, fix)
                if od.eccentricity < 10.0 and vid not in w.noticed:
                    w.noticed.add(vid)
                    self.counts["noticed"] += 1
                    self.events.append(("crowd_notices", f"{key} saw {vid} {d:.0f} m away, "
                                        f"{d / max(0.1, v):.1f} s, {3.6 * v:.0f} km/h"))
        if not eye.in_saccade:
            for side in (-1, 1):
                if side * gaze > 0.5 * o["side_gaze_deg"]:
                    w.seen_side[side] = t
        for vid in [k for k in w.belief if k not in cars]:     # gone past, turned off, or out of reach
            del w.belief[vid]
        # where to look next: a car whose believed gap might or might not do (uncertainty that matters for the
        # decision, as in Senders' uncertainty sampling), else the side not seen for longest
        if not w.eyes.busy and t - max(w.fix_since, eye.fixation_onset) >= o["fixation_s"]:
            unsure = [(b_[0], vid) for vid, b_ in w.belief.items() if self._rel_sd(b_) > o["max_rel_sd"]]
            if unsure and abs(cars[min(unsure)[1]][2] - gaze) >= 5.0:
                target = cars[min(unsure)[1]][2]
            else:
                side = min((-1, 1), key=lambda s: w.seen_side.get(s, -1e9))
                known = [(b_[0], vid) for vid, b_ in w.belief.items() if math.copysign(1, cars[vid][2]) == side]
                target = cars[min(known)[1]][2] if known else side * o["side_gaze_deg"]
            if abs(target - gaze) >= 1.0:
                w.eyes.command((target, 0.0), t)
                w.looks += 1
                self.counts["saccades"] += 1
            w.fix_since = t
        # the decision: the crowd's criterion on the walker's belief
        fresh = all(t - w.seen_side.get(s, -1e9) <= o["recheck_s"] for s in (-1, 1))
        believed = self._smallest_gap(w)
        ok = fresh and not blocked and all(self._acceptable(b_, t_c, marked, decel, react) for b_ in w.belief.values())
        true_gap = min((d / v for d, v, _az in cars.values()), default=None)
        go = False
        if ok:
            if w.go_at is None:
                self.counts["decisions"] += 1
                r = w.response.respond([self.Signal("visual", t, w.response.visual_latency(0.0))])
                w.go_at = r.time
            go = t >= w.go_at
        elif w.go_at is not None:
            self.counts["cancelled"] += 1
            w.go_at = None
        if self._csv is not None:
            self._csv.writerow([f"{t:.2f}", key, f"{gaze:.1f}", int(eye.in_saccade),
                                "" if abs(gaze) < 0.5 * o["side_gaze_deg"] else ("left" if gaze < 0 else "right"),
                                len(cars), len(w.belief), "" if believed is None else f"{believed:.2f}",
                                "" if true_gap is None else f"{true_gap:.2f}", f"{t_c:.2f}",
                                "go" if go else ("deciding" if w.go_at is not None else "wait")])
        if go:
            self.counts["crossed"] += 1
            self.last = {"believed_gap": believed, "true_gap": true_gap, "looks": w.looks}
            w.crossing, w.go_at = None, None
        return go

    def _predict(self, w, dt):
        """Between looks a believed gap runs down at 1 s/s and the distance at the believed speed; both grow less
        certain (assumed process noise: 0.3 s of gap and 1 m of distance per second)."""
        if dt <= 0:
            return
        for b_ in w.belief.values():
            v = max(0.3, b_[2] / max(0.1, b_[0]))
            b_[0] -= dt
            b_[1] += (0.3 * dt) ** 2 + 0.3 ** 2 * dt
            b_[2] -= v * dt
            b_[3] += 1.0 * dt

    def _fuse(self, w, vid, od, ot, t, fix):
        """A look at car vid: its gap from its looming (ot) when that is above threshold, else from its seen
        distance at the assumed speed; fused with the belief by their variances (a Kalman update)."""
        o = self.o
        prior_v = o["prior_speed_kmh"] / 3.6
        if ot is not None:
            g, var = ot.value, ot.sd ** 2
        else:
            g = max(0.0, od.value) / prior_v
            var = (od.sd / prior_v) ** 2 + (g * o["prior_sd"]) ** 2
        b_ = w.belief.get(vid)
        if b_ is None:
            w.belief[vid] = [g, var, od.value, od.sd ** 2, ot is not None, t, fix]
            return
        k = b_[1] / (b_[1] + var)
        b_[0], b_[1] = b_[0] + k * (g - b_[0]), (1 - k) * b_[1]
        k = b_[3] / (b_[3] + od.sd ** 2)
        b_[2], b_[3] = b_[2] + k * (od.value - b_[2]), (1 - k) * b_[3]
        b_[4], b_[5], b_[6] = b_[4] or ot is not None, t, fix

    @staticmethod
    def _rel_sd(b_):
        """Relative SD of the believed gap."""
        return math.sqrt(b_[1]) / max(0.1, b_[0])

    def _acceptable(self, b_, t_c, marked, decel, react):
        if self._rel_sd(b_) > self.o["max_rel_sd"]:
            return False                 # not seen well enough to judge yet
        if b_[0] >= t_c:
            return True
        d = max(0.0, b_[2])
        v = d / max(0.1, b_[0])
        return bool(marked and d > v * react + v * v / (2 * decel) + 3.0)

    @staticmethod
    def _smallest_gap(w):
        gaps = [b_[0] for b_ in w.belief.values()]
        return min(gaps) if gaps else None

    # --- logs -----------------------------------------------------------------------------------------------------

    def open_log(self, out):
        if not self.o["log_gaze"]:
            return
        self._file = open(out / "crowd_gaze.csv", "w", newline="")
        self._csv = csv.writer(self._file, delimiter=";")
        self._csv.writerow(["timestamp", "walker", "gaze_deg", "saccade", "side", "cars_approaching", "cars_believed",
                            "believed_gap_s", "true_gap_s", "critical_gap_s", "state"])

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = self._csv = None

    def summary(self):
        return {**self.counts, "walkers": len(self.walkers), "options": self.o}
