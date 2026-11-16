"""coupled-carla: SUMO vehicles near a camera move in CARLA as real vehicles, so their wheels roll and steer, cyclists
pedal and motorbike riders ride.

The bridge used to place every SUMO vehicle with ApplyTransform and physics off. CARLA 0.9.16 then reports 0 m/s for
it, its wheels never turn, and a bike's pedals (and with them the rider's legs: the rider follows the pedals) only
move when the bike's own physics drives it with throttle (measured: physics off, the pedal bone stays at its angle;
physics on with a target velocity alone, it stays too; with throttle as well, it turns). So near a participant
(simulation.animate_radius_m) a SUMO vehicle has physics on and follows its SUMO pose by velocity: each tick a
target velocity (SUMO's speed along its heading plus the position error over FOLLOW_S), a target yaw rate (the
heading error over FOLLOW_S plus SUMO's own turn rate), and a control with a little throttle while it moves (the
bike's pedalling animation) and the steering angle of its turn (the front wheels). Off by more than TELEPORT_M or
TELEPORT_DEG (a bump, a collision, a SUMO jump) it is put back with a transform. Farther away it is placed as before.
On lin4080 the follow tracked within 0.1 m at 5 m/s (bike, car, bus).

Pure functions here; the bridge (synchronization.py) turns them into carla commands.
"""
import math

FOLLOW_S = 0.3        # time constant of the position and heading correction (s)
MAX_CORRECTION = 3.0  # m/s at most added to SUMO's velocity for the position error
TELEPORT_M = 1.2      # farther off than this: put back with a transform
TELEPORT_DEG = 25.0
THROTTLE = 0.3        # while moving: the bike's pedalling (and a car's drive wheels)
MAX_STEER_DEG = 60.0  # steer 1.0 as a front-wheel angle (CARLA cars: 60-70 deg)
UNSTABLE_TELEPORTS = 2        # this many put-backs within UNSTABLE_TICKS: placed with physics off instead ...
UNSTABLE_TICKS = 150
UNSTABLE_PAUSE_TICKS = 500    # ... for this many ticks (10 s at 50 Hz)
FLIP_DEG = 60.0               # a SUMO heading that jumps more than this in one tick is a flip, not a turn


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def follow(target, current, speed, yaw_rate_deg, wheelbase, vz=0.0, dz=None, tilt=None):
    """Commands to follow a SUMO pose with physics on.

    target, current: (x, y, yaw_deg) in CARLA coordinates (the SUMO pose as CARLA's, the actor's now); speed (m/s)
    and yaw_rate_deg (deg/s) from SUMO; wheelbase (m) for the steering angle; vz: the actor's vertical speed now
    (gravity keeps it on the road); dz: SUMO's height less the actor's (given only where the net's heights match
    CARLA's roads, levels.py), tilt: the actor's (pitch, roll). Off by more than levels.LEVEL_M in height (on
    another level of a stacked road, or knocked up onto something) or tipped beyond levels.MAX_TILT_DEG it is put
    back too. Returns {"teleport": bool, "why": reason or None, "velocity": (vx, vy, vz), "angular_z": deg/s,
    "throttle", "steer", "brake"}.
    """
    from coupledcarla import levels  # pylint: disable=import-outside-toplevel
    ex, ey = target[0] - current[0], target[1] - current[1]
    err = math.hypot(ex, ey)
    eyaw = wrap_deg(target[2] - current[2])
    why = "position" if err > TELEPORT_M else "heading" if abs(eyaw) > TELEPORT_DEG else \
        "level" if (dz is not None and abs(dz) > levels.LEVEL_M) else \
        "tipped" if (tilt is not None and levels.tipped(*tilt)) else None
    teleport = why is not None
    yaw = math.radians(target[2])
    k = min(1.0, MAX_CORRECTION / (err / FOLLOW_S)) if err > 1e-6 else 0.0
    vx = speed * math.cos(yaw) + ex / FOLLOW_S * k
    vy = speed * math.sin(yaw) + ey / FOLLOW_S * k
    if teleport:
        eyaw = 0.0
    wz = yaw_rate_deg + eyaw / FOLLOW_S
    moving = speed > 0.3
    if moving and abs(yaw_rate_deg) > 0.05:
        delta = math.degrees(math.atan(wheelbase * math.radians(yaw_rate_deg) / speed))
    else:
        delta = 0.0
    steer = max(-1.0, min(1.0, delta / MAX_STEER_DEG))
    return {"teleport": teleport, "why": why, "velocity": (vx, vy, vz), "angular_z": wz,
            "throttle": THROTTLE if moving else 0.0, "steer": steer, "brake": 0.0 if moving else 1.0}


def steady_yaw(prev, yaw, speed):
    """The heading to show: SUMO's, unless it jumped by more than FLIP_DEG since the last tick while moving (SUMO
    turns a vehicle round on some short junction lanes for a second: a bus pointed backwards at 30 km/h)."""
    if prev is not None and speed > 0.5 and abs(wrap_deg(yaw - prev)) > FLIP_DEG:
        return prev
    return yaw


def wheelbase(length):
    """A vehicle's wheelbase from its length (cars about 0.6 of it; a bike 1.1 m of 1.8 m)."""
    return max(0.9, 0.6 * length)


def yaw_rate(prev_yaw, yaw, dt):
    """deg/s between two SUMO headings (CARLA degrees) dt seconds apart."""
    return wrap_deg(yaw - prev_yaw) / dt if dt > 0 else 0.0


def velocity(speed, yaw_deg):
    """(vx, vy) in CARLA coordinates of a vehicle driving at speed along yaw_deg."""
    r = math.radians(yaw_deg)
    return speed * math.cos(r), speed * math.sin(r)


# -- smoothed paths ---------------------------------------------------------------------------------------------------
# SUMO moves a vehicle's front bumper along its lane's centre line, internal (junction) lanes included, and turns its
# body to the chord of its front and back positions. Where that line has a kink (netconvert's junction shapes: a lane
# that crosses two lanes inside a 6 m junction turned SUMO's heading 77 -> 27 -> 74 deg, so the participant's car on
# the SUMO driver swerved at 12 m/s, Eindhoven rush hour) a CARLA car that copies the pose jerks sideways. A
# PathSmoother drives a kinematic bicycle (reference: the rear axle) after SUMO's pose instead: it steers at the
# point SUMO's rear axle has reached (pure pursuit), trailing it by LAG_S of travel, with its curvature limited to a
# lateral acceleration a human driver accepts (A_LAT) and changed no faster than a hand turns the wheel (K_RATE), so a
# kink becomes a curve a few metres long. Farther off SUMO's path the limits open up (to A_LAT_HARD at HARD_M off);
# beyond SNAP_M (a SUMO teleport, a reroute) it starts again at SUMO's pose.

A_LAT = 3.5          # m/s^2: comfortable lateral acceleration (urban drivers mostly stay under 3-4: Xu et al. 2015)
A_LAT_HARD = 8.0     # m/s^2 allowed when far off SUMO's path (traffic stays close to its SUMO pose: other
                     # vehicles keep their gaps to that)
HARD_M = 1.0         # lateral offset from SUMO's path at which the hard limit applies (linear from 0.4 m)
K_MAX = 0.22         # 1/m: the tightest turn (radius 4.5 m)
K_RATE = 0.12        # 1/m per s: how fast the curvature changes (about 20 deg/s at the front wheels)
LAG_S = 0.15         # the smoothed car trails SUMO's by this much travel time
LONG_S = 0.6         # time constant of the along-path correction
SNAP_M = 3.0         # farther off than this: start again at SUMO's pose
# a participant's car on the SUMO driver (the camera's subject): gentler, and may fall further behind SUMO's path
PARTICIPANT = {"a_lat": 3.0, "a_lat_hard": 3.5, "hard_m": 3.0, "snap_m": 8.0, "k_rate": 0.08, "lag_s": 0.3}
REAR_AXLE = 0.3      # rear axle behind the centre as a share of the length (cars, bikes); buses REAR_AXLE_LONG
REAR_AXLE_LONG = 0.25


def rear_offset(length):
    return (REAR_AXLE_LONG if length > 7.0 else REAR_AXLE) * length


class PathSmoother:
    """One vehicle's smoothed pose. step((x, y, yaw) of SUMO's body centre in CARLA coordinates, speed, length, dt)
    -> (x, y, yaw, yaw rate deg/s, speed) of the body centre to show. snaps counts the restarts."""

    def __init__(self, a_lat=A_LAT, a_lat_hard=A_LAT_HARD, lag_s=LAG_S, k_rate=K_RATE, hard_m=HARD_M,
                 snap_m=SNAP_M):
        self.a_lat, self.a_lat_hard, self.lag_s, self.k_rate = a_lat, a_lat_hard, lag_s, k_rate
        self.hard_m, self.snap_m = hard_m, snap_m
        self.x = self.y = self.h = None      # rear axle position, heading (rad)
        self.k = 0.0
        self.snaps = 0
        self.max_off = 0.0

    @classmethod
    def participant(cls):
        return cls(**PARTICIPANT)

    def reset(self, rx, ry, h):
        self.x, self.y, self.h, self.k = rx, ry, h, 0.0

    def step(self, target, speed, length, dt, aim=None):
        """aim: a point on SUMO's path ahead of its front bumper (path_ahead()), to steer at; default the front
        bumper itself (a shorter look-ahead: kinks are smoothed over the body's length only)."""
        ro = rear_offset(length)
        ht = math.radians(target[2])
        ct, st = math.cos(ht), math.sin(ht)
        tx, ty = target[0] - ro * ct, target[1] - ro * st            # SUMO's rear axle
        fx, fy = aim if aim is not None else (target[0] + 0.5 * length * ct, target[1] + 0.5 * length * st)
        if self.x is None:
            self.reset(tx, ty, ht)
        v = max(0.0, speed)
        lag = self.lag_s * v
        ex, ey = tx - self.x, ty - self.y
        off = abs(-ex * st + ey * ct)          # off SUMO's path, across SUMO's heading
        behind = ex * ct + ey * st - lag       # along it, beyond the lag
        if max(off, behind, -behind - lag - 1.0) > self.snap_m or \
                (speed < 0.3 and abs(wrap_deg(math.degrees(ht - self.h))) > 60.0 and math.hypot(ex, ey) > 0.5):
            self.snaps += 1
            self.why = "off" if off > self.snap_m else "behind" if behind > self.snap_m else \
                "ahead" if -behind - lag - 1.0 > self.snap_m else "turned"
            self.reset(tx, ty, ht)
            ex = ey = off = 0.0
        self.max_off = max(self.max_off, off)
        ch, sh = math.cos(self.h), math.sin(self.h)
        e_long = ex * ch + ey * sh
        vs = v + max(-3.0, min(4.0, behind / LONG_S))      # keep `lag` behind SUMO's rear axle (along its path)
        if v < 0.05 and abs(e_long) < 0.05:
            vs = 0.0
        vs = max(0.0, vs)
        hard = max(0.0, min(1.0, (off - 0.4) / max(1e-6, self.hard_m - 0.4)))
        a_lat = self.a_lat + (self.a_lat_hard - self.a_lat) * hard
        k_lim = min(K_MAX, a_lat / max(vs, 0.5) ** 2)
        px, py = fx - self.x, fy - self.y      # pure pursuit at SUMO's front bumper (look-ahead: lag + wheelbase-ish)
        d2 = px * px + py * py
        p_lat, p_long = -px * sh + py * ch, px * ch + py * sh
        if d2 > 1.0 and p_long > 0.0:
            k_cmd = 2.0 * p_lat / d2
        else:
            k_cmd = math.radians(wrap_deg(math.degrees(ht - self.h))) / max(1.0, lag + 1.0)
        k_cmd = max(-k_lim, min(k_lim, k_cmd))
        rate = self.k_rate * (1.0 + 2.0 * hard) * dt
        self.k += max(-rate, min(rate, k_cmd - self.k))
        self.k = max(-k_lim, min(k_lim, self.k))
        wz = vs * self.k                       # rad/s
        self.h += wz * dt
        self.x += vs * math.cos(self.h) * dt
        self.y += vs * math.sin(self.h) * dt
        cx, cy = self.x + ro * math.cos(self.h), self.y + ro * math.sin(self.h)
        return cx, cy, wrap_deg(math.degrees(self.h)), math.degrees(wz), vs


LOOK_S = 0.6          # path_ahead: aim this much travel time ahead of the front bumper ...
LOOK_MIN = 3.0        # ... and at least this far (m)


def look_ahead(speed):
    return max(LOOK_MIN, LOOK_S * speed)


def path_point(shapes, pos, dist, lat=0.0):
    """The point `dist` metres beyond lane position `pos` of shapes[0], going on along shapes[1:] (the lanes the
    vehicle drives next), shifted `lat` metres to the left (SUMO's lateral lane position); SUMO coordinates.
    None without shapes. Beyond the last shape: straight on along its last segment."""
    if not shapes or not shapes[0] or len(shapes[0]) < 2:
        return None
    want = pos + dist
    acc = 0.0
    last = None
    for k, shape in enumerate(shapes):
        for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            last = (x0, y0, x1, y1, seg)
            if acc + seg >= want:
                u = (want - acc) / seg
                return _shift(x0 + u * (x1 - x0), y0 + u * (y1 - y0), (x1 - x0) / seg, (y1 - y0) / seg, lat)
            acc += seg
        if k == 0 and pos > acc:      # SUMO's lengths and the shape's differ a little
            want = acc + dist
    if last is None:
        return None
    x0, y0, x1, y1, seg = last
    ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
    return _shift(x1 + ux * (want - acc), y1 + uy * (want - acc), ux, uy, lat)


def _shift(x, y, ux, uy, lat):
    return x - uy * lat, y + ux * lat


class Trailer:
    """A vehicle body dragged by its front bumper along SUMO's path: SUMO's front position is exact (it lies on the
    lane's centre line), the rear axle follows it like a trailer's hitch (it moves toward the front, keeping its
    distance), and the heading is the line from the rear axle to the front. SUMO's own heading (the chord of its
    front and back positions) jumps where its back position passes onto another lane (a bus's centre, half a length
    behind the front, moved 0.9 m sideways in one tick); this body turns only as fast as the front's path bends, and
    a long vehicle's rear cuts inside a turn as a real one does. step(front (x, y), sumo_yaw, length) -> (centre x,
    y, yaw)."""

    def __init__(self):
        self.rx = self.ry = None

    def step(self, front, sumo_yaw, length):
        d = 0.5 * length + rear_offset(length)       # front bumper to rear axle
        fx, fy = front
        if self.rx is not None:
            dx, dy = fx - self.rx, fy - self.ry
            n = math.hypot(dx, dy)
            if abs(n - d) > TRAILER_JUMP_M or n < 1e-6:     # a teleport or a reset: start again
                self.rx = None
        if self.rx is None:
            h = math.radians(sumo_yaw)
            self.rx, self.ry = fx - d * math.cos(h), fy - d * math.sin(h)
        dx, dy = fx - self.rx, fy - self.ry
        n = math.hypot(dx, dy)
        ux, uy = dx / n, dy / n
        self.rx, self.ry = fx - d * ux, fy - d * uy
        yaw = math.degrees(math.atan2(uy, ux))
        return fx - 0.5 * length * ux, fy - 0.5 * length * uy, yaw


TRAILER_JUMP_M = 2.0   # the front moved this much farther than the body's length allows: start again
