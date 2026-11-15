"""Film cameras on the host: demo-video frames at an even rate in simulation time (tools/make_videos.py).

An experiment's `film` section (config.DEFAULTS) puts one or more cameras into a trial and keeps their images on a
grid of simulation time: image k of a camera is the one rendered at trial time start_s + k / fps, whatever the
real-time factor. Each camera renders every tick (so auto exposure and motion blur behave) and an image is kept
when its own timestamp falls on the grid, so frames stay evenly spaced even when the run is slower than real time.

Camera kinds (`camera.kind`):
  attach   rigidly on an actor (`target`): mount driver | passenger | hood | chase | roof | eye, or [x, y, z(, pitch,
           yaw)] in the actor's frame (metres forward, right, up); `spring: true` uses CARLA's spring arm (smooth chase)
  follow   not attached: `offset` [forward, right, up] in the target's frame, its heading smoothed over `smooth_s`,
           looking at the target (`look_ahead_m` in front, `look_up_m` up); the pose is predicted one tick ahead. Its
           boom collides like a game's spring arm (`collide: true`): a ray from above the target to the camera
           (World.cast_ray) pulls the camera in front of the first vehicle, building or wall on the way, at once,
           and lets it out again over `boom_release_s`; it never passes through a bus beside the target
  tripod   fixed at `at` {lat, lon, height_m}, looking at `look` ({lat, lon, height_m}, or an actor name, then
           panning after it with `smooth_s`)
  orbit    round `centre` ({lat, lon} or an actor name) at `radius_m`, `height_m`, from `start_deg` (compass) at
           `deg_per_s`, looking at the centre `look_up_m` up
  path     through keyframes [{t, lat, lon, height_m, look: {lat, lon, height_m}}] (t: seconds into the clip),
           smooth (Catmull-Rom); a crane or fly-over
  drone    a pilot's camera (`target`: the pilot's role; `view` fpv | los | chase), moved with the drone (drones.py)
Targets: a participant's role, an agent's id, or `bike`, `bus`, `car`, `walker`, `type:<part of a blueprint id>`: the
nearest such moving actor to `near` ({lat, lon}) or the first participant when the camera starts (picked again if it
leaves the world). A follow camera can keep its eyes on another actor or point (`look_at`, blended with straight
ahead by `look_weight`), as a pedestrian watching an approaching car.

Free cameras (tripod, orbit, path, follow) join the runner's focus actors, so SUMO traffic, scenery and parked cars are
spawned around them as around a participant. `hold: {role: until_s}` keeps a host-local driver standing with the hand brake on until that trial time (traffic
fills in around it while the clip's start waits). Frames go to <trial>/film/<camera>/NNNNNN.jpg with frames.csv
(index, CARLA frame, trial time) and camera.csv (the camera's pose every tick, for an offline soundscape).
"""
import csv
import math
import queue
import threading
from pathlib import Path

import numpy as np

DEFAULTS = {
    "start_s": 30.0,          # trial time of the first kept frame
    "duration_s": 20.0,       # seconds kept
    "fps": 25.0,              # frames per second of simulation time (a whole number of ticks apart is best)
    "size": [1280, 720],
    "fov": 90.0,
    "settle_s": 2.0,          # cameras start this long before start_s (exposure adapts, meshes stream in)
    "jpeg_quality": 93,
    "attributes": {},         # extra sensor.camera.rgb attributes (motion_blur_intensity, exposure_compensation, ...)
    "capture": {},            # image quality (look.py, as simulation.capture): {} = 2x supersampled, clean, exposure
                              # from the sun; false: CARLA's camera defaults
    "camera": None,           # one camera, or `cameras`: [{name, kind, ...}, ...]
    "cameras": [],
    "hold": {},               # {role: until_s}: a host-local driver waits with the hand brake on
}

MOUNTS = {   # (x forward, y right, z up, pitch, yaw) in the actor's frame
    "driver": (-0.05, -0.38, 1.22, 0.0, 0.0),
    "passenger": (-0.15, 0.36, 1.15, 0.0, 0.0),
    "hood": (1.6, 0.0, 1.6, 0.0, 0.0),
    "chase": (-6.5, 0.0, 2.8, -10.0, 0.0),
    "roof": (0.0, 0.0, 2.2, -5.0, 0.0),
    "eye": (0.22, 0.0, 0.72, -3.0, 0.0),        # a walker's eyes (station.EYE)
    "bike": (-3.2, 0.0, 1.9, -8.0, 0.0),
}
PICK = {"bike": ("gazelle", "bh.", "diamondback", "crossbike", "omafiets", "century", "bicycle", "bike"),
        "bus": ("bus", "fusorosa", "fuso.mitsubishi"), "car": (), "walker": ()}


# -- pure helpers (tests) ---------------------------------------------------------------------------------------

def spec(fc):
    """The film section merged over DEFAULTS, with `camera` folded into `cameras` (each named)."""
    out = {**DEFAULTS, **(fc or {})}
    cams = list(out.get("cameras") or [])
    if out.get("camera"):
        cams.insert(0, out["camera"])
    named = []
    for i, c in enumerate(cams):
        c = dict(c)
        c.setdefault("name", "main" if i == 0 else f"cam{i}")
        c.setdefault("kind", "attach")
        named.append(c)
    out["cameras"] = named
    out["frames"] = int(round(float(out["duration_s"]) * float(out["fps"])))
    return out


def grid_index(ts, ts0, fps, step):
    """The grid index of an image rendered at simulation time ts (grid starts at ts0, fps per second), or None when
    the image is not on the grid (more than half a tick off)."""
    k = int(round((ts - ts0) * fps))
    return k if abs(ts - (ts0 + k / fps)) < 0.5 * step else None


def look_rotation(frm, to):
    """(pitch, yaw) in degrees, CARLA's convention, for a camera at frm looking at to (x, y, z)."""
    dx, dy, dz = to[0] - frm[0], to[1] - frm[1], to[2] - frm[2]
    return math.degrees(math.atan2(dz, math.hypot(dx, dy))), math.degrees(math.atan2(dy, dx))


def ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def angle_lerp(a, b, w):
    """a moved toward b by the share w, the short way round (degrees)."""
    return a + ((b - a + 180.0) % 360.0 - 180.0) * w


def smooth_weight(dt, tau):
    return 1.0 if tau <= 0 else 1.0 - math.exp(-dt / tau)


def orbit_point(centre, radius, height, start_deg, deg_per_s, t):
    """Camera position on a circle round centre (x, y, z): compass angle start_deg + deg_per_s * t (0 = north of the
    centre, 90 = east; CARLA y points south)."""
    a = math.radians(start_deg + deg_per_s * t)
    return (centre[0] + radius * math.sin(a), centre[1] - radius * math.cos(a), centre[2] + height)


def _catmull(p0, p1, p2, p3, u):
    u2, u3 = u * u, u * u * u
    return tuple(0.5 * ((2 * b) + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u2 + (-a + 3 * b - 3 * c + d) * u3)
                 for a, b, c, d in zip(p0, p1, p2, p3))


def path_point(keys, t):
    """keys: [(t, (x, y, z)), ...] sorted by t. A Catmull-Rom curve through them, eased in and out over the whole
    path, clamped at the ends."""
    if not keys:
        raise ValueError("a path needs keyframes")
    if len(keys) == 1 or t <= keys[0][0]:
        return tuple(keys[0][1])
    if t >= keys[-1][0]:
        return tuple(keys[-1][1])
    t0, t1 = keys[0][0], keys[-1][0]
    te = t0 + (t1 - t0) * ease((t - t0) / (t1 - t0))     # slow start and stop
    i = max(j for j in range(len(keys) - 1) if keys[j][0] <= te)
    pts = [k[1] for k in keys]
    p0, p1, p2 = pts[max(0, i - 1)], pts[i], pts[i + 1]
    p3 = pts[min(len(pts) - 1, i + 2)]
    u = (te - keys[i][0]) / max(1e-9, keys[i + 1][0] - keys[i][0])
    return _catmull(p0, p1, p2, p3, u)


BOOM_PIVOT_UP = 2.0     # the follow camera's boom starts this far above the target's origin (above its roof)
BOOM_MARGIN = 0.5       # the camera stays this far in front of what blocks the boom
BOOM_MIN = 1.5          # ... and at least this far from the pivot
# what a boom collides with (CARLA 0.9.16 CityObjectLabel names): not trees (a canopy over the street would pull it
# in all the time), poles or signs (thin, gone in a frame)
BOOM_BLOCKERS = {"Buildings", "Walls", "Car", "Bus", "Truck", "Train", "Motorcycle", "Vehicles", "Bridge",
                 "GuardRail"}


def boom_share(prev, hit_dists, full, dt, release_s):
    """A spring arm's length as a share of the full boom: in at once to BOOM_MARGIN before the nearest hit (hit
    distances from the pivot, m), out again smoothly (time constant release_s) when nothing blocks it."""
    want = 1.0
    if hit_dists:
        want = max(BOOM_MIN, min(hit_dists) - BOOM_MARGIN) / full
        want = max(0.0, min(1.0, want))
    if prev is None or want < prev:
        return want
    return prev + (want - prev) * smooth_weight(dt, release_s)


def offset_point(origin, yaw_deg, fwd, right, up):
    """origin (x, y, z) moved by (forward, right, up) metres in a frame turned by yaw (CARLA: y points south, so
    right of a heading is +90 degrees of yaw)."""
    y = math.radians(yaw_deg)
    return (origin[0] + fwd * math.cos(y) - right * math.sin(y),
            origin[1] + fwd * math.sin(y) + right * math.cos(y), origin[2] + up)


def check_frames(rows, fps, step, n=None):
    """Evenness of kept frames: rows of (index, carla_frame, trial_t). Returns a dict with the count, missing and
    duplicate indices, and the spread of the time steps and frame steps (all equal when the frames are even)."""
    rows = sorted(rows)
    idx = [r[0] for r in rows]
    want = range(n if n is not None else (max(idx) + 1 if idx else 0))
    seen = set(idx)
    dts = [round(b[2] - a[2], 4) for a, b in zip(rows, rows[1:]) if b[0] == a[0] + 1]
    dfs = [b[1] - a[1] for a, b in zip(rows, rows[1:]) if b[0] == a[0] + 1]
    return {"frames": len(rows), "missing": [i for i in want if i not in seen],
            "duplicates": sorted({i for i in idx if idx.count(i) > 1}),
            "dt_min": min(dts) if dts else None, "dt_max": max(dts) if dts else None,
            "frame_steps": sorted(set(dfs)), "even": bool(dts) and max(dts) - min(dts) < 0.5 * step and
            abs(sum(dts) / len(dts) - 1.0 / fps) < 0.5 * step}


# -- the cameras on a running trial -----------------------------------------------------------------------------

def realism(rig, n):
    """The realism guard (look.py) on a film camera: an empty world in most of its semantic samples, or a dark,
    bright, clipped or glaring frame at 10, 50 and 90 % of the clip. [] when it may be shown."""
    from PIL import Image
    from . import look
    out = []
    # a semantic sample that is almost all "unlabeled" (tag 0) saw no world at all (its camera was elsewhere, as with
    # a drone camera moved by DroneWorld on 0.9.16): it says nothing about the frames and is not counted
    tags = [t for t in rig.sem_tags if float((np.asarray(t) == 0).mean()) < 0.95]
    # a drone camera's semantic samples that are all the same stayed where the drone took off (DroneWorld moves only
    # the RGB camera on 0.9.16): in a level with ground under the take-off spot (a CARLA town) they see that spot
    if getattr(rig, "kind", None) == "drone" and len(tags) > 1 and \
            all(np.array_equal(np.asarray(t), np.asarray(tags[0])) for t in tags[1:]):
        tags = []
    if tags:
        empty = [p for p in (look.semantic_problems(t) for t in tags) if p]
        if len(empty) > len(tags) / 2:
            out.append(f"{len(empty)} of {len(tags)} semantic samples: {empty[0][0]}")
    for k in sorted({int(n * q) for q in (0.1, 0.5, 0.9)}):
        f = rig.dir / f"{k:06d}.jpg"
        if f.exists():
            out += [f"frame {k}: {p}" for p in look.image_problems(np.asarray(Image.open(f).convert("RGB")))]
    return out


class _DroneView:
    """Stands in for drones.DroneView so DroneWorld moves this camera with the pilot every tick."""

    def __init__(self, pilot, camera):
        self.pilot, self.camera = pilot, camera


class Rig:
    def __init__(self, film, c):
        self.film, self.c, self.name = film, c, c["name"]
        self.kind = c.get("kind", "attach")
        self.camera = None
        self.target = None
        self.yaw = None
        self.look = None
        self.look_target = None
        self.boom = None             # follow: the boom's length now, as a share of the full offset (spring arm)
        self.dir = film.dir / self.name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.kept = {}               # index -> (carla frame, trial time)
        self.images = None           # False: the server cannot render (-nullrhi); poses only
        self.poses = []
        self.sem, self.sem_tags = None, []      # realism guard (look.py): semantic tags along the clip

    # positions from the config
    def point(self, p, default_h=1.7):
        import carla
        f = self.film
        if isinstance(p, str):
            a = f.actor(p)
            loc = a.get_location() if a is not None else carla.Location()
            return (loc.x, loc.y, loc.z)
        x, y = f.geo.to_carla(p["lat"], p["lon"])
        from .geo import ground
        g = ground(f.cmap, x, y)
        return (x, y, g.z + float(p.get("height_m", default_h)))

    def spawn(self, bp):
        import carla
        f, c = self.film, self.c
        world = f.world
        if self.kind in ("attach", "follow") or (self.kind == "tripod" and isinstance(c.get("look"), str)):
            self.target = f.actor(c.get("target") or c.get("look"), near=c.get("near"))
            if self.target is None:
                raise RuntimeError(f"film camera {self.name}: no actor {c.get('target') or c.get('look')!r}")
        if self.kind == "attach":
            m = c.get("mount", "driver")
            x, y, z, pitch, yaw = (list(m) + [0.0, 0.0])[:5] if isinstance(m, (list, tuple)) else MOUNTS[m]
            if m in ("driver", "passenger") and self.target.type_id.startswith("vehicle."):
                from . import cockpit       # the car's measured seat (data/driver_eyes.json)
                x, y, z, pitch = cockpit.actor_eye(self.target, m, bool(c.get("lefthand", False)))
                yaw = 0.0
            kind = carla.AttachmentType.SpringArmGhost if c.get("spring") else carla.AttachmentType.Rigid
            self.camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z),
                                                                carla.Rotation(pitch=pitch, yaw=yaw)),
                                            attach_to=self.target, attachment_type=kind)
        elif self.kind == "drone":
            dw = f.drone_world
            pilot = next((p for p in (dw.pilots if dw else []) if p.role == c.get("target")), None)
            if pilot is None:
                raise RuntimeError(f"film camera {self.name}: no drone pilot {c.get('target')!r}")
            if c.get("view"):
                pilot.view = c["view"]
            self.camera = world.spawn_actor(bp, dw.camera_transform(pilot, f.step))
            if dw.view is None:
                dw.view = _DroneView(pilot, self.camera)
        else:
            self.camera = world.spawn_actor(bp, self.transform(f.t + f.step, None) or carla.Transform())
        from .compat import listen
        self.images = listen(self.camera, self._on_image, f.log, f"film {self.name}")
        try:        # a small semantic camera riding on this one, once a second: is the world around it empty?
            if not self.images:
                raise RuntimeError("no rendering")
            sbp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
            for k, v in (("image_size_x", 320), ("image_size_y", 180), ("fov", bp.get_attribute("fov").as_float()),
                         ("sensor_tick", 1.0)):
                sbp.set_attribute(k, str(v))
            self.sem = world.spawn_actor(sbp, carla.Transform(), attach_to=self.camera)
            self.sem.listen(lambda img: self.sem_tags.append(
                np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, 2].copy())
                if len(self.sem_tags) < 120 else None)
        except RuntimeError:
            self.sem = None
        if f.focus_list is not None and self.kind in ("tripod", "orbit", "path", "follow") and self.c.get("focus", True):
            f.focus_list.append(self.camera)

    def _on_image(self, image):
        f = self.film
        if f.ts0 is None:
            return
        k = grid_index(image.timestamp, f.ts0, f.fps, f.step)
        if k is None or not 0 <= k < f.n or k in self.kept:
            return
        self.kept[k] = (image.frame, image.timestamp - f.sim0)
        f.jobs.put((self.dir / f"{k:06d}.jpg", bytes(image.raw_data), image.width, image.height))

    def _target_state(self, snap):
        a = snap.find(self.target.id) if self.target is not None else None
        if a is None:
            return None
        tf, v = a.get_transform(), a.get_velocity()
        return (tf.location.x, tf.location.y, tf.location.z), tf.rotation.yaw, (v.x, v.y, v.z)

    def transform(self, t_next, snap):
        """The camera's transform for the frame rendered at trial time t_next (None: leave it)."""
        import carla
        f, c = self.film, self.c
        u = t_next - f.start
        pos = look = None
        if self.kind == "tripod":
            pos = self.point(c["at"])
            if isinstance(c.get("look"), str):
                st = self._target_state(snap) if snap is not None else None
                if st is not None:
                    want = tuple(p + v * f.step for p, v in zip(st[0], st[2]))
                    want = (want[0], want[1], want[2] + float(c.get("look_up_m", 0.5)))
                    w = smooth_weight(f.step, float(c.get("smooth_s", 0.6)))
                    self.look = want if self.look is None else tuple(a + (b - a) * w for a, b in zip(self.look, want))
                look = self.look or pos
            else:
                look = self.point(c["look"], 0.0)
        elif self.kind == "orbit":
            centre = self.point(c["centre"], 0.0)
            pos = orbit_point(centre, float(c.get("radius_m", 40)), float(c.get("height_m", 15)),
                              float(c.get("start_deg", 0)), float(c.get("deg_per_s", 3)), u)
            look = (centre[0], centre[1], centre[2] + float(c.get("look_up_m", 1.0)))
        elif self.kind == "path":
            keys = c.get("_keys")
            if keys is None:
                keys = c["_keys"] = [(float(k["t"]), self.point(k)) for k in c["keys"]]
                c["_looks"] = [(float(k["t"]), self.point(k["look"], 0.0)) for k in c["keys"] if k.get("look")]
            pos = path_point(keys, u)
            look = path_point(c["_looks"], u) if c["_looks"] else None
            if look is None:      # along the path
                ahead = path_point(keys, u + 0.5)
                look = (2 * ahead[0] - pos[0], 2 * ahead[1] - pos[1], pos[2] - 2.0)
        elif self.kind == "follow":
            st = self._target_state(snap) if snap is not None else None
            if st is None:
                return None
            loc, yaw, vel = st
            loc = tuple(p + v * f.step for p, v in zip(loc, vel))     # where it will be when this frame renders
            w = smooth_weight(f.step, float(c.get("smooth_s", 0.8)))
            self.yaw = yaw if self.yaw is None else angle_lerp(self.yaw, yaw, w)
            fwd, right, up = (list(c.get("offset", [-7.0, 0.0, 2.6])) + [0, 0])[:3]
            pos = offset_point(loc, self.yaw, fwd, right, up)
            if c.get("collide", True):
                pos = self._boom(loc, pos, f.step, float(c.get("boom_release_s", 0.6)))
            look = offset_point(loc, self.yaw, float(c.get("look_ahead_m", 4.0)), 0.0, float(c.get("look_up_m", 1.0)))
            if c.get("look_at") is not None:      # eyes on another actor (or a point), blended with straight ahead
                if self.look_target is None:
                    self.look_target = f.actor(c["look_at"]) if isinstance(c["look_at"], str) else None
                want = None
                if self.look_target is not None:
                    a = snap.find(self.look_target.id)
                    if a is not None:
                        lt = a.get_transform().location
                        want = (lt.x, lt.y, lt.z + float(c.get("look_at_up_m", 0.6)))
                elif isinstance(c["look_at"], dict):
                    want = self.point(c["look_at"], 1.0)
                if want is not None:
                    w = smooth_weight(f.step, float(c.get("look_smooth_s", 0.5)))
                    self.look = want if self.look is None else tuple(a + (b - a) * w for a, b in zip(self.look, want))
                if self.look is not None and math.dist(self.look[:2], pos[:2]) < float(c.get("look_max_m", 60.0)):
                    p_a, y_a = look_rotation(pos, look)
                    p_b, y_b = look_rotation(pos, self.look)
                    lw = float(c.get("look_weight", 1.0))
                    yaw, pitch = angle_lerp(y_a, y_b, lw), p_a + (p_b - p_a) * lw
                    d = 10.0
                    look = (pos[0] + d * math.cos(math.radians(yaw)) * math.cos(math.radians(pitch)),
                            pos[1] + d * math.sin(math.radians(yaw)) * math.cos(math.radians(pitch)),
                            pos[2] + d * math.sin(math.radians(pitch)))
        else:
            return None
        pitch, yaw = look_rotation(pos, look)
        pitch += float(c.get("pitch_offset", 0.0))
        return carla.Transform(carla.Location(*pos), carla.Rotation(pitch=pitch, yaw=yaw))

    def _boom(self, loc, pos, dt, release_s):
        """The follow camera's position with its boom shortened in front of what lies between it and the target."""
        import carla
        if getattr(self, "_pivot_up", None) is None:    # above the target's own roof (a bus is 3-4 m tall)
            try:
                bb = self.target.bounding_box
                self._pivot_up = max(BOOM_PIVOT_UP, bb.location.z + bb.extent.z + 0.4)
            except (RuntimeError, AttributeError):
                self._pivot_up = BOOM_PIVOT_UP
        pivot = (loc[0], loc[1], loc[2] + self._pivot_up)
        full = math.dist(pivot, pos)
        if full < 0.5:
            return pos
        try:
            hits = self.film.world.cast_ray(carla.Location(*pivot), carla.Location(*pos))
        except (RuntimeError, AttributeError):      # an older server without cast_ray: as before
            return pos
        names = {str(h.label).split(".")[-1] for h in hits}
        dists = [math.dist(pivot, (h.location.x, h.location.y, h.location.z)) for h in hits
                 if str(h.label).split(".")[-1] in BOOM_BLOCKERS]
        self.boom = boom_share(self.boom, dists, full, dt, release_s)
        self.boom_hits = names
        k = self.boom
        return tuple(a + (b - a) * k for a, b in zip(pivot, pos))

    def update(self, t, snap):
        import carla
        f = self.film
        if self.kind in ("follow",) and self.target is not None and snap.find(self.target.id) is None:
            alt = f.actor(self.c.get("target"), near=self.c.get("near"))     # it left the world: pick another
            if alt is not None and alt.id != self.target.id:
                f.log(f"film {self.name}: target {self.target.id} gone, now {alt.id} {alt.type_id}")
                self.target, self.yaw, self._pivot_up = alt, None, None
        if self.kind in ("tripod", "orbit", "path", "follow"):
            tf = self.transform(t + f.step, snap)
            if tf is not None:
                f.queue(carla.command.ApplyTransform(self.camera.id, tf))
        if f.start - 1e-6 <= t <= f.start + f.duration + f.step:
            a = snap.find(self.camera.id)
            if a is not None:
                tf = a.get_transform()
                self.poses.append((round(t, 4), tf.location.x, tf.location.y, tf.location.z, tf.rotation.yaw,
                                   tf.rotation.pitch))


class Film:
    """The film cameras of one trial. tick(t, snap) once per tick after the world tick; close() at the end."""

    def __init__(self, fc, world, geo, out, log, actors, step, queue_cmd, drone_world=None, focus=None):
        self.fc = spec(fc)
        self.world, self.geo, self.log, self.step = world, geo, log, float(step)
        self.cmap = world.get_map()
        self.actors = actors                # name -> actor (participant roles, agent ids)
        self.queue = queue_cmd
        self.drone_world = drone_world
        # the runner's focus actors (participants): SUMO traffic, scenery, parked cars and traffic lights appear near
        # them; free cameras join the list, so what they film is there even far from the participants
        self.focus_list = focus if isinstance(focus, list) else None
        self.focus = (lambda: [a.get_location() for a in self.focus_list[:1]]) if self.focus_list is not None else \
            (focus or (lambda: None))
        self.start, self.duration = float(self.fc["start_s"]), float(self.fc["duration_s"])
        self.fps, self.n = float(self.fc["fps"]), self.fc["frames"]
        self.dir = Path(out) / "film"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.rigs = [Rig(self, c) for c in self.fc["cameras"]]
        self.sim0 = self.ts0 = None
        self.t = 0.0
        self.started = False
        self.jobs = queue.Queue(maxsize=256)
        self.quality = int(self.fc["jpeg_quality"])
        self.writers = [threading.Thread(target=self._write, daemon=True) for _ in range(3)]
        for w in self.writers:
            w.start()
        log(f"film: {len(self.rigs)} camera(s), {self.n} frames at {self.fps:g} fps from t = {self.start:g} s")

    def hold(self, parts):
        """Keep host-local drivers standing until their hold time (wraps their input)."""
        import carla
        for p in parts:
            until = (self.fc.get("hold") or {}).get(p.role)
            if until is None or p.input is None:
                continue
            if hasattr(p.input, "hold"):     # the SUMO driver: enters SUMO when the hold ends (sumodriver.py)
                p.input.hold(until)
                self.log(f"film: {p.role} waits until t = {float(until):g} s")
                continue
            orig = p.input.control

            def control(actor, events=(), orig=orig, until=float(until)):
                if self.t < until:
                    return carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                return orig(actor, events)
            p.input.control = control
            self.log(f"film: {p.role} waits until t = {until:g} s")

    def actor(self, name, near=None):
        if name is None:
            return None
        a = self.actors(name) if not isinstance(name, dict) else None
        sub = name[5:] if isinstance(name, str) and name.startswith("type:") else None
        if a is not None or (name not in PICK and sub is None):
            return a
        if near:
            x, y = self.geo.to_carla(near["lat"], near["lon"])
        else:
            locs = self.focus() or []
            if not locs:
                return None
            x, y = locs[0].x, locs[0].y
        pat = "walker.pedestrian.*" if name == "walker" else "vehicle.*"
        best, bd = None, 1e18
        for a in self.world.get_actors().filter(pat):
            tid = a.type_id
            if sub is not None:
                if sub not in tid:
                    continue
            elif name != "walker" and PICK[name] and not any(k in tid for k in PICK[name]):
                continue
            if name == "car" and any(k in tid for k in PICK["bike"] + PICK["bus"]):
                continue
            if a.attributes.get("role_name") not in (None, "", "autopilot", "sumo_driver", "crowd"):
                continue      # not a participant or agent
            loc = a.get_location()
            v = a.get_velocity()
            d = (loc.x - x) ** 2 + (loc.y - y) ** 2 + (0.0 if v.x * v.x + v.y * v.y > 1.0 else 400.0)
            if d < bd:
                best, bd = a, d
        return best

    def tick(self, t, snap):
        self.t = t
        if self.sim0 is None:
            self.sim0 = snap.timestamp.elapsed_seconds - t
            self.ts0 = self.sim0 + self.start
        if not self.started and t >= self.start - float(self.fc["settle_s"]) - 1e-6:
            self.started = True
            for r in self.rigs:
                r.spawn(self._blueprint(r.c))
                self.log(f"film: camera {r.name} ({r.kind}) at t = {t:.2f} s")
        if self.started:
            for r in self.rigs:
                r.update(t, snap)

    def _blueprint(self, c=None):
        """The camera blueprint: the film's size, fov and attributes, with a camera's own fov and attributes."""
        from . import look
        from .compat import setup_camera
        c = c or {}
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        # film.capture (look.py; default: simulation.capture's defaults): rendered at 2x and downscaled by the
        # writers, no motion blur or flare, bloom 0.3, a manual exposure from the sun and clouds (0.9.x)
        cap = look.settings(self.fc.get("capture", {}))
        w, h = look.capture_size(self.fc["size"], cap)
        bp.set_attribute("image_size_x", str(int(w)))
        bp.set_attribute("image_size_y", str(int(h)))
        bp.set_attribute("fov", str(c.get("fov", self.fc["fov"])))
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", "0.0")       # every tick: images are picked by their timestamp
        setup_camera(bp, 0.0, self.fc.get("camera_profile", "Dashcam"), world=self.world, capture=cap)
        for k, v in {**(self.fc.get("attributes") or {}), **(c.get("attributes") or {})}.items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        return bp

    def _write(self):
        from PIL import Image
        while True:
            job = self.jobs.get()
            if job is None:
                return
            path, raw, w, h = job
            try:
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)[:, :, 2::-1]
                from . import look
                arr = look.downscale(np.ascontiguousarray(arr), tuple(int(v) for v in self.fc["size"]))
                Image.fromarray(np.ascontiguousarray(arr)).save(path, quality=self.quality)
            except Exception as e:  # noqa: BLE001 - a lost frame shows in the check, never stops the trial
                self.log(f"film: could not write {path.name} ({e})")
            finally:
                self.jobs.task_done()

    def close(self):
        import time
        t_end = time.time() + 10.0        # images of the last ticks may still be on their way
        waiting = [r for r in self.rigs if r.camera is not None and r.images]
        while time.time() < t_end and any(len(r.kept) < self.n for r in waiting):
            time.sleep(0.2)
        for r in self.rigs:
            for cam in (r.sem, r.camera):
                if cam is not None:
                    try:
                        cam.stop()
                        cam.destroy()
                    except RuntimeError:
                        pass
        self.jobs.join()
        for _ in self.writers:
            self.jobs.put(None)
        summary = {}
        for r in self.rigs:
            with open(r.dir / "frames.csv", "w", newline="") as fh:
                w = csv.writer(fh, delimiter=";")
                w.writerow(["index", "frame", "t"])
                for k in sorted(r.kept):
                    w.writerow([k, r.kept[k][0], f"{r.kept[k][1]:.4f}"])
            with open(r.dir / "camera.csv", "w", newline="") as fh:
                w = csv.writer(fh, delimiter=";")
                w.writerow(["t", "x", "y", "z", "yaw", "pitch"])
                w.writerows(r.poses)
            chk = check_frames([(k, v[0], v[1]) for k, v in r.kept.items()], self.fps, self.step, self.n)
            summary[r.name] = {k: (v[:20] if isinstance(v, list) else v) for k, v in chk.items()}
            summary[r.name]["images"] = r.images is not False
            summary[r.name]["poses"] = len(r.poses)
            self.log(f"film: {r.name} kept {chk['frames']} of {self.n} frames, even {chk['even']}, "
                     f"missing {len(chk['missing'])}")
            summary[r.name]["realism"] = realism(r, self.n)
            if summary[r.name]["realism"]:
                self.log(f"film: {r.name} REJECTED by the realism guard: {'; '.join(summary[r.name]['realism'])}")
        import json
        (self.dir / "film.json").write_text(json.dumps({"fps": self.fps, "start_s": self.start, "frames": self.n,
                                                        "step": self.step, "cameras": summary}, indent=2))
        return summary
