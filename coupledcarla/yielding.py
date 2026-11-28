"""SUMO vehicles brake for the CARLA walkers in their path (participants, scripted walkers, the crowd).

SUMO does not know the walkers (it has no sidewalks or crossings here), so the runner checks every SUMO vehicle near a
walker each few ticks and commands its speed through TraCI. Before (ai.SumoYield) the check read the vehicle's speed
from CARLA, where a SUMO vehicle placed with physics off reads 0 m/s: its look-ahead was always the 6 m of a standing
car, and a car at 50 km/h, told to stop 6 m before a pedestrian with SUMO's 4.5 m/s^2, hit the pedestrian (the Tokyo
clip). Now:

- poses and speeds come from SUMO (the bridge's poses), walkers' positions and velocities from CARLA;
- the vehicle's path is an arc at its current turn rate; a walker conflicts when it stands in the path's corridor
  (half the vehicle's width + CORRIDOR_M) within the look-ahead, or will be there when the vehicle arrives (walker
  moving at its velocity, up to PREDICT_S ahead);
- the vehicle brakes to stop STOP_GAP_M before the walker: at the deceleration that needs, at least COMFORT and,
  if that is more than the vehicle's planned deceleration, as hard as needed up to its emergency deceleration
  (SUMO's speed mode then lets it brake harder than decel; logged as sumo_emergency_stop);
- it waits until its path has been clear for CLEAR_S (a walker leaving the look-ahead for a moment did not start
  it again: the stop/go flicker of README's lessons) and then drives on.
"""
import math

import traci

CORRIDOR_M = 0.9       # beyond the vehicle's half width, each side
STOP_GAP_M = 2.5       # stop this far before the walker
COMFORT = 2.0          # m/s^2: the gentlest deceleration used for a walker ahead
EMERGENCY = 8.0        # m/s^2: the hardest (cars; SUMO's own emergencyDecel caps it per type)
PREDICT_S = 4.0        # how far ahead a walker's motion is extrapolated
CLEAR_S = 1.0          # the path must be clear this long before the vehicle drives on
NEAR_M = 70.0          # vehicles farther than this from every walker are not checked


def look_ahead(speed, decel=COMFORT):
    """Metres ahead of the front a vehicle at speed checks: 1 s of reaction plus a stop at decel, plus 8 m."""
    return 8.0 + speed * 1.0 + speed * speed / (2.0 * decel)


def path_points(x, y, yaw_deg, yaw_rate_deg, speed, front, length, step=1.0):
    """[(s, px, py)]: points along the vehicle's path from its front bumper (s = 0) to `length` m, on an arc at its
    current turn rate (straight when standing)."""
    pts = []
    yaw = math.radians(yaw_deg)
    k = math.radians(yaw_rate_deg) / speed if speed > 1.0 else 0.0      # curvature 1/m
    k = max(-0.2, min(0.2, k))                                          # no tighter than a 5 m radius
    px, py = x + front * math.cos(yaw), y + front * math.sin(yaw)
    s = 0.0
    while s <= length + 1e-9:
        pts.append((s, px, py))
        px += step * math.cos(yaw)
        py += step * math.sin(yaw)
        yaw += k * step
        s += step
    return pts


def first_conflict(pose, yaw_rate_deg, walkers):
    """Distance (m, from the front bumper) to the first walker in the vehicle's path, or None.

    pose: (x, y, yaw_deg, speed, half_length, half_width) in CARLA coordinates; walkers: [(x, y, vx, vy)].
    """
    x, y, yaw, speed, hl, hw = pose
    reach = look_ahead(speed)
    near = [w for w in walkers if (w[0] - x) ** 2 + (w[1] - y) ** 2 <= (reach + hl + 6.0) ** 2]
    if not near:
        return None
    half = hw + CORRIDOR_M
    for s, px, py in path_points(x, y, yaw, yaw_rate_deg, speed, hl, reach):
        t = s / max(speed, 1.0)
        for wx, wy, vx, vy in near:
            if (wx - px) ** 2 + (wy - py) ** 2 <= (half - 0.3) ** 2:      # in the corridor now
                return s
            tp = min(t, PREDICT_S)
            qx, qy = wx + vx * tp, wy + vy * tp
            if (qx - px) ** 2 + (qy - py) ** 2 <= half * half and math.hypot(vx, vy) > 0.2:
                return s                                                  # there when the vehicle arrives
    return None


GRID_M = 20.0


def near_walkers(grid, x, y, reach):
    """The walkers of grid {(cell x, cell y): [(x, y, vx, vy)]} (GRID_M cells) within reach m of (x, y)."""
    r2, n = reach * reach, int(reach // GRID_M) + 1
    gx, gy = int(x // GRID_M), int(y // GRID_M)
    return [w for dx in range(-n, n + 1) for dy in range(-n, n + 1) for w in grid.get((gx + dx, gy + dy), ())
            if (w[0] - x) ** 2 + (w[1] - y) ** 2 <= r2]


def braking(speed, dist, decel=3.4, emergency=EMERGENCY):
    """(deceleration to use m/s^2, emergency?) to stop STOP_GAP_M before a walker dist m ahead of the front, or
    (0, False) when it can wait: further than a comfortable stop needs."""
    room = dist - STOP_GAP_M
    if speed < 0.1:
        return COMFORT, False                     # standing: stay
    if room <= 0.05:
        return emergency, True
    need = speed * speed / (2.0 * room)
    if need < 0.6 * COMFORT:
        return 0.0, False
    a = max(COMFORT, need)
    return min(a, emergency), a > decel


class SumoYield:
    """Runner hook: step(sync, world, walkers) every tick; events [(name, detail)] for the log."""

    def __init__(self, every=2):
        self.every, self._tick = every, 0
        self.held = {}             # sumo id -> time the path was last blocked
        self.hard = set()          # sumo ids with the max-decel check off (speed mode 27)
        self.events = []
        self.t = 0.0
        self._yaw = {}

    def step(self, sync, world, walkers, dt=0.02):
        self._tick += 1
        self.t += dt
        if self._tick % self.every:
            return
        wl = []
        for w in walkers:
            try:
                loc, vel = w.get_location(), w.get_velocity()
            except RuntimeError:
                continue
            wl.append((loc.x, loc.y, vel.x, vel.y))
        poses = getattr(sync, "poses", {})
        grid = {}                  # walkers by 20 m cell: a crowd of hundreds (scramble.py) checked only near a car
        for w in wl:
            grid.setdefault((int(w[0] // GRID_M), int(w[1] // GRID_M)), []).append(w)
        ids = {cid: sid for sid, cid in sync.sumo2carla_ids.items()}
        for sid in [s for s in self.held if s not in sync.sumo2carla_ids]:   # gone from CARLA (or SUMO)
            self._release(sid)
        for cid, pose in poses.items():
            sid = ids.get(cid)
            if sid is None:
                continue
            x, y, yaw, speed = pose[:4]
            prev = self._yaw.get(cid, yaw)
            self._yaw[cid] = yaw
            rate = ((yaw - prev + 180.0) % 360.0 - 180.0) / (dt * self.every)
            near = near_walkers(grid, x, y, NEAR_M)
            if not near:
                if sid in self.held:
                    self._release(sid)
                continue
            d = first_conflict(pose, rate, near)
            if d is None:
                if sid in self.held:
                    if self.t - self.held[sid] >= CLEAR_S:
                        self._release(sid)
                    else:       # not clear for long enough: keep slowing (a SUMO speed command stays until changed)
                        self._command(sid, speed, COMFORT, dt * self.every)
                continue
            a, emergency = braking(speed, d)
            if a <= 0.0 and sid not in self.held:
                continue
            a = a or COMFORT
            try:
                if sid not in self.held:
                    self.events.append(("sumo_yields", f"{sid} (carla {cid}) {speed:.1f} m/s, walker {d:.1f} m"))
                if emergency and sid not in self.hard:
                    traci.vehicle.setSpeedMode(sid, 27)          # may brake harder than its decel
                    self.hard.add(sid)
                    self.events.append(("sumo_emergency_stop", f"{sid} {a:.1f} m/s^2, walker {d:.1f} m"))
                self._command(sid, speed, a, dt * self.every)
                self.held[sid] = self.t
            except traci.TraCIException:
                self.held.pop(sid, None)
        self._yaw = {c: v for c, v in self._yaw.items() if c in poses}

    def _command(self, sid, speed, decel, period):
        """Slows from speed at decel over the period until the next check (slowDown spreads it over the steps; a
        setSpeed would take the whole drop in the first step: 16 m/s^2 for 8 every two ticks); standing: holds."""
        try:
            target = max(0.0, speed - decel * period)
            if speed < 0.1 or target <= 0.0:
                traci.vehicle.setSpeed(sid, 0.0)
            else:
                traci.vehicle.slowDown(sid, target, period)
        except traci.TraCIException:
            self.held.pop(sid, None)

    def _release(self, sid):
        try:
            traci.vehicle.setSpeed(sid, -1)
            if sid in self.hard:
                traci.vehicle.setSpeedMode(sid, 31)
        except traci.TraCIException:
            pass
        self.held.pop(sid, None)
        self.hard.discard(sid)
