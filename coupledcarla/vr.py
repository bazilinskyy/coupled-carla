"""A VR station: the participant's view in a headset, rendered by CARLA as two cameras at the eyes.

CARLA's server does no VR, so the station does it: two RGB cameras at the participant's eyes (the IPD apart,
default 63 mm), moved every tick from the headset's head pose (OpenXR) on top of the seat's eye point (the car's
measured seat from data/driver_eyes.json for drivers and AV passengers, the walker's eyes for pedestrians, the
gimbal camera for drone pilots), rendered at the headset's per-eye field of view and resolution and handed to the
headset as OpenXR swapchain images (vrxr.py, pyopenxr). In-car displays and the HUD are drawn into each eye on the
car's own panels (perspective-correct per eye, so they sit at their depth in stereo); the station's text, prompts
and the visual-sync bar are a head-locked panel ~1.5 m ahead.

    python -m coupledcarla.station --host <host> --role driver --input wheel --display vr
    participants: [{role: driver, station: true, view: vr, vr: {ipd_mm: 65}}]      # or in the experiment

Latency: CARLA renders a camera and sends it back 4-6 ticks after the pose was set (100-200 ms at 35-50 Hz, see the
README), far above VR's ~20 ms. Two things hide it: the pose given to the cameras is predicted ahead by the measured
pose-to-image latency (up to vr.max_predict_ms), and every image is submitted with the head pose it was rendered at,
so the runtime's reprojection (timewarp) turns it to where the head is at display time. Head rotation is then
corrected to the runtime's own ~1-frame latency; head translation and the world itself keep CARLA's latency, and
the runtime repeats the last image (reprojected) at the headset's rate when CARLA delivers fewer frames.

Per frame the station logs logs/stations/<role>_<time>_vr.csv: the head pose sampled, the pose predicted and used,
the prediction horizon, when both eye images arrived, when they were first shown (the runtime's predicted display
time) and the motion-to-photon estimate (display time minus head-pose sample time), plus the angle the runtime's
reprojection had to correct.

This module is the maths (no CARLA, no OpenXR: tested without either) and the CARLA side (VRView); vrxr.py is the
OpenXR side. Frames: OpenXR is right-handed (x right, y up, -z forward, metres); CARLA/UE is left-handed (x
forward, y right, z up, metres; rotators in degrees: pitch + up, yaw + right, roll + right side down).
"""
import csv
import math
import threading
import time
from collections import deque
from pathlib import Path

DEFAULTS = {
    "ipd_mm": 63.0,            # eye distance; None: the runtime's own eye positions (the headset's IPD setting)
    "scale": 0.5,              # render size per eye as a share of the headset's recommended size (each axis)
    "max_size": [1600, 1600],  # cap on the per-eye render size (w, h) in pixels
    "size": None,              # [w, h]: a fixed per-eye render size instead of scale (also for runtime: sim)
    "overscan": 0.10,          # extra field of view around each eye (share of the tangent) for the reprojection
    "predict_ms": "auto",      # head-pose prediction horizon: auto = the measured pose-to-image latency, or ms
    "max_predict_ms": 80.0,    # cap on it (longer predictions overshoot; the runtime's reprojection does the rest)
    "smooth": 0.5,             # angular-velocity smoothing for the prediction (0 none .. 0.9 heavy)
    "reference": "local",      # OpenXR reference space: local (seated, origin where the head starts) | stage
    "recenter": "trial",       # trial: the head's position and heading at the trial start are the seat's eye
    "runtime": "openxr",       # openxr: a headset through its OpenXR runtime | sim: a scripted head, no headset
    "sim_motion": "look",      # runtime sim: look (yaw +-35 deg at 0.2 Hz, pitch +-10 deg) | still
    "fov_deg": [100, 100],     # runtime sim: the per-eye field of view (h, v), symmetric
    "sim_hz": 90.0,            # runtime sim: the display rate
    "mirror": True,            # show the left eye in the station's window (the experimenter's view)
    "panel_distance_m": 1.5,   # the head-locked panel (station text, prompts, visual-sync bar): distance, width
    "panel_width_deg": 50.0,
    "displays_in_eyes": True,  # draw the in-car displays on the car's panels in each eye (dashboard style)
}
PEDESTRIAN_EYE_M = 0.72        # station.EYE: walker origin (its centre) to its eyes
PEDESTRIAN_FORWARD_M = 0.12    # the eyes a little in front of the walker's centre (its head stays out of view)


def settings(*layers):
    """DEFAULTS updated by each dict in turn (the experiment's `vr`, the participant's `vr`); unknown keys kept."""
    out = dict(DEFAULTS)
    for layer in layers:
        for k, v in (layer or {}).items():
            out[k] = v
    return out


def wanted(display_arg, welcome):
    """Whether a station runs in VR: --display vr, or the participant's view: vr."""
    return str(display_arg).lower() == "vr" or str((welcome or {}).get("view") or "").lower() == "vr"


def seat_for(kind, view=None):
    """The car seat whose eye point a VR station starts from: driver | passenger | None (not in a car)."""
    if kind == "driver":
        return "passenger" if view == "passenger" else "driver"
    if kind == "av_passenger":
        return "passenger"
    return None


# --- quaternions (x, y, z, w), Hamilton product -------------------------------------------------------------------

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def qrot(q, v):
    """v rotated by the unit quaternion q."""
    x, y, z = v
    p = qmul(qmul(q, (x, y, z, 0.0)), qconj(q))
    return (p[0], p[1], p[2])


def qaxis(axis, angle):
    """Rotation by angle (radians) about axis (right-hand rule in the coordinates given)."""
    n = math.sqrt(sum(c * c for c in axis))
    if n < 1e-12:
        return IDENTITY
    s = math.sin(angle / 2.0) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def qangle(a, b):
    """The angle (radians) between two orientations."""
    d = abs(sum(x * y for x, y in zip(qnorm(a), qnorm(b))))
    return 2.0 * math.acos(min(1.0, d))


def qmatrix(q):
    """3x3 rotation matrix (rows) of a unit quaternion."""
    x, y, z, w = qnorm(q)
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def matrix_quat(m):
    """Unit quaternion of a 3x3 rotation matrix (rows)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    q = qnorm(q)
    return q if q[3] >= 0 else tuple(-c for c in q)


# --- OpenXR <-> CARLA (UE) ------------------------------------------------------------------------------------------

def xr_to_ue_vec(v):
    """An OpenXR vector (x right, y up, z back) in CARLA's axes (x forward, y right, z up)."""
    return (-v[2], v[0], v[1])


def ue_to_xr_vec(v):
    return (v[1], v[2], -v[0])


def xr_to_ue_quat(q):
    """An OpenXR orientation in CARLA's axes. The axes change C (xr -> ue) has determinant -1 (right- to left-handed),
    so a rotation about axis a by angle t becomes a rotation about -C a by t: (x, y, z, w) -> (z, -x, -y, w)."""
    return (q[2], -q[0], -q[1], q[3])


def ue_to_xr_quat(q):
    return (-q[1], -q[2], q[0], q[3])


def rotator_matrix(pitch, yaw, roll):
    """UE's FRotationMatrix as rows of a 3x3 matrix whose columns are the forward, right and up axes (degrees)."""
    p, y, r = (math.radians(a) for a in (pitch, yaw, roll))
    cp, sp, cy, sy, cr, sr = math.cos(p), math.sin(p), math.cos(y), math.sin(y), math.cos(r), math.sin(r)
    fwd = (cp * cy, cp * sy, sp)
    right = (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp)
    up = (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp)
    return tuple((fwd[i], right[i], up[i]) for i in range(3))


def matrix_rotator(m):
    """(pitch, yaw, roll) in degrees of a rotation matrix (rows) in CARLA's axes."""
    fwd = (m[0][0], m[1][0], m[2][0])
    right_z, up_z = m[2][1], m[2][2]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, fwd[2]))))
    yaw = math.degrees(math.atan2(fwd[1], fwd[0]))
    roll = math.degrees(math.atan2(-right_z, up_z))
    return pitch, yaw, roll


def rotator_quat(pitch, yaw, roll):
    return matrix_quat(rotator_matrix(pitch, yaw, roll))


def quat_rotator(q):
    return matrix_rotator(qmatrix(q))


def compose(parent, child):
    """Pose (position, quaternion) of child given in parent's frame, in parent's parent frame."""
    (pp, pq), (cp, cq) = parent, child
    r = qrot(pq, cp)
    return ((pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), qnorm(qmul(pq, cq)))


def inverse(pose):
    p, q = pose
    qi = qconj(q)
    r = qrot(qi, p)
    return ((-r[0], -r[1], -r[2]), qi)


def yaw_only_xr(q):
    """The heading part of an OpenXR orientation (rotation about +y only): for recentering a seated user."""
    f = qrot(q, (0.0, 0.0, -1.0))
    return qaxis((0.0, 1.0, 0.0), math.atan2(-f[0], -f[2]))


def recenter_origin(head_xr):
    """The origin pose (position, heading-only rotation) that makes this OpenXR head pose the seat's eye point."""
    p, q = head_xr
    return (tuple(p), yaw_only_xr(q))


def head_in_seat(head_xr, origin_xr):
    """A head pose in OpenXR space relative to the recentering origin, in CARLA's axes (the seat's frame)."""
    p, q = compose(inverse(origin_xr), head_xr)
    return (xr_to_ue_vec(p), xr_to_ue_quat(q))


def eye_offsets_ue(ipd_m):
    """Left and right eye positions in the head's frame (CARLA's axes: y to the right)."""
    h = float(ipd_m) / 2.0
    return (0.0, -h, 0.0), (0.0, h, 0.0)


def eye_offsets_xr(ipd_m):
    h = float(ipd_m) / 2.0
    return (-h, 0.0, 0.0), (h, 0.0, 0.0)


def eye_poses(seat_xyz, head_seat, ipd_m, seat_yaw=0.0):
    """The two eyes' poses (position, quaternion) in the seat's parent frame (the vehicle, or the world for a
    walker): the seat's eye point (x, y, z), turned by seat_yaw (deg; a walker's heading), then the head pose
    relative to its recentering origin (head_in_seat), then half the IPD to each side along the head's right."""
    seat = (tuple(float(v) for v in seat_xyz), rotator_quat(0.0, seat_yaw, 0.0))
    head = compose(seat, head_seat)
    return tuple(compose(head, (off, IDENTITY)) for off in eye_offsets_ue(ipd_m))


def eye_poses_xr(head_xr, ipd_m):
    """The two eyes' poses in OpenXR space for a head pose (what the projection layer says the images show)."""
    return tuple(compose(head_xr, (off, IDENTITY)) for off in eye_offsets_xr(ipd_m))


def to_transform(pose):
    """(x, y, z, pitch, yaw, roll) of a pose in CARLA's axes, for carla.Transform."""
    p, q = pose
    return (p[0], p[1], p[2]) + tuple(quat_rotator(q))


def from_transform(x, y, z, pitch, yaw, roll):
    return ((x, y, z), rotator_quat(pitch, yaw, roll))


# --- fields of view ---------------------------------------------------------------------------------------------

def eye_camera(fov, recommended_w, scale=1.0, overscan=0.0, max_size=None, size=None):
    """The CARLA camera that covers one eye. fov: the eye's OpenXR angles (left < 0, right > 0, up > 0, down < 0;
    radians). CARLA's cameras are symmetric (one horizontal fov, square pixels), so the camera covers the larger
    half-angle each way, widened by overscan (a share of the tangent) for the runtime's reprojection; the image is
    then submitted with that symmetric fov, and the runtime samples the eye's own frustum from it.
    Returns {width, height, hfov_deg, layer_fov: (left, right, up, down)}."""
    l, r, u, d = fov
    th = max(math.tan(-l), math.tan(r)) * (1.0 + overscan)
    tv = max(math.tan(u), math.tan(-d)) * (1.0 + overscan)
    if size:
        w = int(size[0])
        h = int(size[1]) if len(size) > 1 and size[1] else int(round(w * tv / th))
        ppt = w / (2 * th)
        tv = h / (2 * ppt)     # square pixels: the vertical fov follows the size given
    else:
        ppt = float(recommended_w) / (math.tan(-l) + math.tan(r)) * float(scale)    # pixels per tangent unit
        w, h = int(round(2 * th * ppt)), int(round(2 * tv * ppt))
        if max_size:
            k = min(1.0, float(max_size[0]) / w, float(max_size[1]) / h)
            w, h = int(round(w * k)), int(round(h * k))
    w, h = max(16, w + (w % 2)), max(16, h + (h % 2))
    tv = th * h / w            # square pixels after rounding
    return {"width": w, "height": h, "hfov_deg": math.degrees(2 * math.atan(th)),
            "layer_fov": (-math.atan(th), math.atan(th), math.atan(tv), -math.atan(tv))}


def symmetric_fov(h_deg, v_deg):
    h, v = math.radians(h_deg) / 2, math.radians(v_deg) / 2
    return (-h, h, v, -v)


def project_point(point, eye_pose, size, layer_fov):
    """(u, v, depth) of a point (same frame as eye_pose, CARLA axes) in an eye image of this size whose frustum is
    layer_fov (OpenXR angles); depth along the view axis, <= 0 behind the eye."""
    p, q = eye_pose
    d = qrot(qconj(q), (point[0] - p[0], point[1] - p[1], point[2] - p[2]))      # forward, right, up
    if d[0] <= 1e-6:
        return (float("nan"), float("nan"), d[0])
    l, r, u, dn = (math.tan(a) for a in layer_fov)
    x, y = d[1] / d[0], d[2] / d[0]                  # tangents right and up
    return ((x - l) / (r - l) * size[0], (u - y) / (u - dn) * size[1], d[0])


def head_locked_quad(distance, width_deg, aspect):
    """Corners (top-left, top-right, bottom-right, bottom-left) of a panel `distance` m straight ahead of the head,
    `width_deg` wide, height/width = aspect, in the head's frame (CARLA axes)."""
    hw = distance * math.tan(math.radians(width_deg) / 2)
    hh = hw * aspect
    return [(distance, -hw, hh), (distance, hw, hh), (distance, hw, -hh), (distance, -hw, -hh)]


# --- prediction and latency -----------------------------------------------------------------------------------

def predict(pose, velocity, angular, dt):
    """A pose (position, quaternion) dt seconds ahead at constant linear and angular velocity (both in the pose's
    parent frame, angular in rad/s about the axis it points along, as OpenXR's XrSpaceVelocity)."""
    p, q = pose
    w = math.sqrt(sum(c * c for c in angular))
    q2 = qnorm(qmul(qaxis(angular, w * dt), q)) if w > 1e-9 else q
    return ((p[0] + velocity[0] * dt, p[1] + velocity[1] * dt, p[2] + velocity[2] * dt), q2)


class PoseTracker:
    """Head poses as they come (time in s, position, quaternion; OpenXR space) and their velocities: the runtime's
    when it gives them, else from the last two samples, smoothed (exponential, `smooth` the old share)."""

    def __init__(self, smooth=0.5):
        self.smooth = float(smooth)
        self.last = None
        self.velocity = (0.0, 0.0, 0.0)
        self.angular = (0.0, 0.0, 0.0)

    def add(self, t, pos, quat, velocity=None, angular=None):
        quat = qnorm(quat)
        if velocity is None or angular is None:
            if self.last is not None and t > self.last[0] + 1e-6:
                t0, p0, q0 = self.last
                dt = t - t0
                dq = qmul(quat, qconj(q0))
                if dq[3] < 0:
                    dq = tuple(-c for c in dq)
                ang = 2.0 * math.acos(min(1.0, dq[3]))
                s = math.sqrt(max(0.0, 1.0 - dq[3] * dq[3]))
                axis = (dq[0] / s, dq[1] / s, dq[2] / s) if s > 1e-9 else (0.0, 0.0, 0.0)
                w_new = tuple(a * ang / dt for a in axis)
                v_new = tuple((a - b) / dt for a, b in zip(pos, p0))
                k = self.smooth
                angular = tuple(k * a + (1 - k) * b for a, b in zip(self.angular, w_new)) if angular is None \
                    else angular
                velocity = tuple(k * a + (1 - k) * b for a, b in zip(self.velocity, v_new)) if velocity is None \
                    else velocity
        if velocity is not None:
            self.velocity = tuple(velocity)
        if angular is not None:
            self.angular = tuple(angular)
        self.last = (t, tuple(pos), quat)

    def at(self, t):
        """The pose predicted for time t (None before the first sample)."""
        if self.last is None:
            return None
        t0, p, q = self.last
        return predict((p, q), self.velocity, self.angular, max(0.0, t - t0))


class Latency:
    """Pose-to-image latency samples (ms) and the prediction horizon they give."""

    def __init__(self, fixed=None, cap_ms=80.0, keep=120):
        self.fixed = None if fixed in (None, "auto") else float(fixed)
        self.cap = float(cap_ms)
        self.samples = deque(maxlen=keep)

    def add(self, ms):
        if ms is not None and 0 <= ms < 5000:
            self.samples.append(float(ms))

    def median(self):
        if not self.samples:
            return None
        xs = sorted(self.samples)
        return xs[len(xs) // 2]

    def horizon_ms(self):
        if self.fixed is not None:
            return max(0.0, min(self.cap, self.fixed))
        m = self.median()
        return 0.0 if m is None else max(0.0, min(self.cap, m))


class PoseHistory:
    """What the station sent for each frame: the pose a camera image of frame F was rendered at is the last one put
    for a frame <= F (the camera moves with the batch sent after snapshot F-1 and renders at frame F)."""

    def __init__(self, keep=64):
        self.items = {}
        self.order = deque(maxlen=keep)

    def put(self, frame, item):
        if frame not in self.items:
            self.order.append(frame)
        self.items[frame] = item
        while len(self.items) > self.order.maxlen:
            self.items.pop(min(self.items), None)

    def get(self, frame):
        keys = [f for f in self.items if f <= frame]
        return self.items[max(keys)] if keys else None


class EyePair:
    """Pairs the two eyes' images by frame: done(frame) is true once both arrived."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pending = {}

    def put(self, eye, frame, data):
        with self.lock:
            slot = self.pending.setdefault(frame, [None, None])
            slot[eye] = data
            if slot[0] is not None and slot[1] is not None:
                del self.pending[frame]
                for f in [f for f in self.pending if f < frame]:      # an eye that never came: dropped
                    del self.pending[f]
                return slot
        return None


# --- the simulated headset (runtime: sim) -------------------------------------------------------------------------

class SimHeadset:
    """A scripted head for tests without a headset or OpenXR runtime: the same interface as vrxr.XrHeadset. The
    head looks left and right (yaw +-35 deg at 0.2 Hz) and a little up and down; submit() only counts frames."""

    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.log = log
        h, v = (cfg.get("fov_deg") or [100, 100])[:2]
        self.fov = symmetric_fov(h, v)
        self.recommended = (int((cfg.get("size") or [1440])[0] / max(1e-6, float(cfg.get("scale", 1.0)))),
                            None)
        self.hz = float(cfg.get("sim_hz", 90.0))
        self.t0 = time.perf_counter()
        self.frames = self.submitted = 0
        self.latest = None
        self.ok = True
        self.lock = threading.Lock()
        self.shown = {}             # image id -> (first display time, head pose then)
        self.stopped = threading.Event()
        self.thread = None

    def head(self, t):
        if self.cfg.get("sim_motion", "look") == "still":
            return ((0.0, 0.0, 0.0), IDENTITY)
        s = t - self.t0
        yaw = math.radians(35.0) * math.sin(2 * math.pi * 0.2 * s)
        pitch = math.radians(10.0) * math.sin(2 * math.pi * 0.13 * s)
        q = qmul(qaxis((0, 1, 0), yaw), qaxis((1, 0, 0), pitch))
        return ((0.02 * math.sin(2 * math.pi * 0.1 * s), 0.0, 0.0), q)

    def start(self):
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _loop(self):
        period = 1.0 / self.hz
        nxt = time.perf_counter()
        while not self.stopped.is_set():
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))
            now = time.perf_counter()
            with self.lock:
                self.frames += 1
                if self.latest is not None and self.latest[0] not in self.shown:
                    self.shown[self.latest[0]] = (now + period, self.head(now + period))
                    self.submitted += 1

    def pose(self):
        """(sample time, predicted display time, head pose in OpenXR space, velocity, angular velocity)."""
        now = time.perf_counter()
        disp = now + 1.0 / self.hz
        p, q = self.head(disp)
        p2, q2 = self.head(disp + 0.01)
        dq = qmul(q2, qconj(q))
        ang = 2.0 * math.acos(max(-1.0, min(1.0, dq[3])))
        s = math.sqrt(max(0.0, 1.0 - dq[3] ** 2))
        w = tuple(c / s * ang / 0.01 for c in dq[:3]) if s > 1e-9 else (0.0, 0.0, 0.0)
        return now, disp, (p, q), tuple((a - b) / 0.01 for a, b in zip(p2, p)), w

    def submit(self, image_id, eyes, poses_xr, fovs):
        with self.lock:
            self.latest = (image_id, poses_xr, fovs)

    def first_shown(self, image_id):
        with self.lock:
            return self.shown.pop(image_id, None)

    def stop(self):
        self.stopped.set()


def make_headset(cfg, log=print):
    """The headset for cfg['runtime']: vrxr.XrHeadset (openxr) or SimHeadset (sim). Raises RuntimeError with what to
    install when OpenXR is asked for and missing."""
    if str(cfg.get("runtime", "openxr")).lower() == "sim":
        return SimHeadset(cfg, log)
    from . import vrxr
    return vrxr.XrHeadset(cfg, log)


# --- drawing into the eyes ----------------------------------------------------------------------------------------

def warp_into(img, tex_rgba, corners, additive=False, gain=1.0):
    """Draws a texture (h x w x 4 RGBA uint8) into img (H x W x C uint8, C 3 = RGB or 4 = BGRA, changed in place)
    at the four pixel corners (top-left, top-right, bottom-right, bottom-left): alpha-blended, or added (light, the
    HUD). False when the quad is off the image or degenerate."""
    import numpy as np
    from PIL import Image
    from .cockpit import warp_coeffs
    if any(c is None or any(math.isnan(x) for x in c) for c in corners):
        return False
    hgt, wid = img.shape[:2]
    x0 = max(0, int(math.floor(min(u for u, _v in corners))))
    x1 = min(wid, int(math.ceil(max(u for u, _v in corners))) + 1)
    y0 = max(0, int(math.floor(min(v for _u, v in corners))))
    y1 = min(hgt, int(math.ceil(max(v for _u, v in corners))) + 1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return False
    th, tw = tex_rgba.shape[:2]
    c = warp_coeffs((tw, th), corners, (x0, y0))
    if c is None:
        return False
    pad = np.zeros((th + 4, tw + 4, 4), np.uint8)
    pad[2:-2, 2:-2] = tex_rgba
    h = [c[0] + 2 * c[6], c[1] + 2 * c[7], c[2] + 2 * c[8], c[3] + 2 * c[6], c[4] + 2 * c[7],
         c[5] + 2 * c[8], c[6], c[7], c[8]]
    h = [v / h[8] for v in h]
    warped = np.asarray(Image.fromarray(pad, "RGBA").transform((x1 - x0, y1 - y0), Image.PERSPECTIVE, h[:8],
                                                               Image.BILINEAR))
    on = warped[..., 3] > 0                  # only the drawn pixels (a panel of text is mostly transparent)
    if not on.any():
        return True
    w = warped[on].astype(np.float32)
    rgb = w[:, :3] if img.shape[2] == 3 else w[:, 2::-1]       # BGRA eye images
    a = w[:, 3:4] / 255.0
    roi = img[y0:y1, x0:x1, :3]
    px = roi[on].astype(np.float32)
    out = px + rgb * a * gain if additive else px * (1 - a) + rgb * gain * a
    roi[on] = np.clip(out, 0, 255).astype(np.uint8)
    return True


def crop_panel(tex, quad):
    """The visible part of a panel texture (alpha > 0) and its quad: (sub texture, sub quad), or (None, None) when
    nothing is drawn. quad: 3D corners (top-left, top-right, bottom-right, bottom-left) of a flat rectangle, so the
    sub quad is interpolated exactly."""
    import numpy as np
    on = tex[..., 3] > 0
    rows, cols = np.flatnonzero(on.any(axis=1)), np.flatnonzero(on.any(axis=0))
    if not len(rows):
        return None, None
    h, w = on.shape
    r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
    tl, tr, _br, bl = quad

    def at(u, v):
        return tuple(tl[i] + u * (tr[i] - tl[i]) + v * (bl[i] - tl[i]) for i in range(3))
    u0, u1, v0, v1 = c0 / w, c1 / w, r0 / h, r1 / h
    return tex[r0:r1, c0:c1], [at(u0, v0), at(u1, v0), at(u1, v1), at(u0, v1)]


def quad_in_eye(corners3d, eye_pose, size, layer_fov):
    """Pixel corners of a 3D quad (same frame as eye_pose) in an eye image; None if any corner is behind the eye."""
    out = []
    for c in corners3d:
        u, v, d = project_point(c, eye_pose, size, layer_fov)
        if d <= 0.02:
            return None
        out.append((u, v))
    return out


# --- the CARLA side ------------------------------------------------------------------------------------------------

class VRView:
    """A station's two eye cameras, moved every tick from the headset, their images drawn on and submitted.

    station: the coupledcarla.station.Station (its world, actor, welcome, control, pending batch, ux); cfg: settings();
    headset: make_headset(cfg). For a driver or AV passenger the cameras ride on the car (attached, rigid) at the seat's
    eye point; for a pedestrian they are moved in the world at the walker's eyes, turned with its heading; for a drone
    pilot they follow the camera the host moves (the gimbal)."""

    def __init__(self, station, cfg, headset, log=print):
        self.st, self.cfg, self.hs, self.log = station, cfg, headset, log
        self.cams = [None, None]
        self.anchor = None
        self.origin = None
        self.track = PoseTracker(cfg.get("smooth", 0.5))
        self.lat = Latency(cfg.get("predict_ms", "auto"), cfg.get("max_predict_ms", 80.0))
        self.history = PoseHistory()
        self.parents = {}           # frame -> the car's (or gimbal's) pose in that frame's snapshot
        self.relative = True        # ApplyTransform on an attached camera is relative to the car (CARLA 0.9.16)
        self.errors = []            # the first rendered-pose errors (cm), to check that
        self.draw_eye = None        # the station's latest drawer (compose_and_submit), used by the worker
        self._cv = threading.Condition()
        self._pending = None        # the newest complete pair, for the worker (_work)
        self._stop = False
        self.worker = None
        self.last_left = None       # the newest left eye drawn, for the station's mirror window
        self.pair = EyePair()
        self.ipd = None if cfg.get("ipd_mm") is None else float(cfg["ipd_mm"]) / 1000.0
        self.cam = None             # eye_camera() result
        self.shown_frames = 0
        self.lock = threading.Lock()
        self.records = {}           # image id -> log row (waiting for its display time)
        self.panel = None           # the head-locked panel's RGBA texture (pygame surface), drawn each tick
        self.log_f = self.log_w = None
        self.fps_n, self.fps_t0, self.fps = 0, time.perf_counter(), 0.0

    # -- setup --------------------------------------------------------------------------------------------------
    def camera_spec(self):
        fov = self.hs.fov
        rec_w = self.hs.recommended[0] if self.hs.recommended else 1440
        return eye_camera(fov, rec_w, float(self.cfg.get("scale", 0.5)), float(self.cfg.get("overscan", 0.1)),
                          self.cfg.get("max_size"), self.cfg.get("size"))

    def start(self, world, actor, kind, seat_eye, blueprint_setup, log_path):
        """Spawns the two eye cameras. seat_eye: (x, y, z, pitch) of the seat in the vehicle frame (None for a
        walker or a drone); blueprint_setup(bp): the station's camera attributes (exposure, profile)."""
        import carla
        self.world, self.actor, self.kind = world, actor, kind
        self.seat = seat_eye
        self.cam = self.camera_spec()
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.cam["width"]))
        bp.set_attribute("image_size_y", str(self.cam["height"]))
        bp.set_attribute("fov", f"{self.cam['hfov_deg']:.3f}")
        blueprint_setup(bp)
        self.bp = bp
        if kind == "drone":
            self.anchor = self.st.camera          # the host moves it (drones.py); the eyes follow it
        for i in range(2):
            if self.seat is not None:
                x, y, z, _pitch = self.seat
                tf = carla.Transform(carla.Location(x=x, y=y + (-1 if i == 0 else 1) * (self.ipd or 0.063) / 2, z=z))
                self.cams[i] = world.spawn_actor(bp, tf, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)
            else:
                self.cams[i] = world.spawn_actor(bp, (self.anchor or actor).get_transform())
            self.cams[i].listen(lambda image, i=i: self._on_image(i, image))
        self.worker = threading.Thread(target=self._work, name="vr-eyes", daemon=True)
        self.worker.start()
        self.log(f"vr: two eye cameras {self.cam['width']}x{self.cam['height']} at {self.cam['hfov_deg']:.1f} deg "
                 f"(IPD {'runtime' if self.ipd is None else f'{1000 * self.ipd:.1f} mm'}, overscan "
                 f"{100 * float(self.cfg.get('overscan', 0.1)):.0f}%, prediction "
                 f"{self.cfg.get('predict_ms', 'auto')} ms, runtime {self.cfg.get('runtime')})")
        self.log_f = open(log_path, "w", newline="", buffering=1)
        self.log_w = csv.writer(self.log_f, delimiter=";")
        self.log_w.writerow(["frame", "sample_time", "head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz",
                             "head_qw", "horizon_ms", "pred_qx", "pred_qy", "pred_qz", "pred_qw", "images_time",
                             "pose_to_image_ms", "display_time", "motion_to_photon_ms", "reprojection_deg",
                             "carla_check_deg", "carla_check_cm"])
        self.recentered = False
        return self.cams

    # -- per tick (the station's loop) ---------------------------------------------------------------------------
    def tick(self, snap):
        """Samples the head, predicts it, moves both eyes (into the station's batch) and remembers the pose for the
        next frame's images."""
        import carla
        sample_t, disp_t, head, vel, ang = self.hs.pose()
        if head is None:
            return
        self.track.add(disp_t, head[0], head[1], vel, ang)
        if self.origin is None or not self.recentered:
            self.origin = recenter_origin(head)
            self.recentered = True
        horizon = self.lat.horizon_ms() / 1000.0
        pred = self.track.at(disp_t + horizon) or head
        rel = head_in_seat(pred, self.origin)
        self.rel = rel
        ipd = self.ipd if self.ipd is not None else getattr(self.hs, "ipd_m", None) or 0.063
        if self.seat is not None:            # in a car: relative to the vehicle; set as world poses (see below)
            eyes = eye_poses(self.seat[:3], rel, ipd)
            parent = self._parent_pose(snap, self.actor.id)
        elif self.kind == "drone" and self.anchor is not None:
            eyes = eye_poses((0.0, 0.0, 0.0), rel, ipd)
            parent = self._parent_pose(snap, self.anchor.id)
        else:                                # a walker: its eye point, turned with the station's heading
            loc = self._parent_pose(snap, self.actor.id)[0]
            yaw = float(getattr(self.st, "heading", 0.0))
            fwd = (PEDESTRIAN_FORWARD_M * math.cos(math.radians(yaw)),
                   PEDESTRIAN_FORWARD_M * math.sin(math.radians(yaw)))
            eyes = eye_poses((loc[0] + fwd[0], loc[1] + fwd[1], loc[2] + PEDESTRIAN_EYE_M), rel, ipd, yaw)
            parent = None
        world_eyes = [compose(parent, e) if parent is not None else e for e in eyes]
        if parent is not None:
            self.parents[snap.frame] = parent
            for f in [f for f in self.parents if f < snap.frame - 30]:
                del self.parents[f]
        # ApplyTransform on an attached camera sets its pose relative to the car on CARLA 0.9.16 (measured: the
        # world pose given as relative put it ~60 m off), so a camera in a car gets its pose in the car and moves with
        # it, exactly, whatever the car does until the next tick. If the rendered poses say otherwise (another CARLA
        # version), pose_error flips to world poses (self.relative False) and logs it once.
        sent = eyes if (self.seat is not None and self.relative) else world_eyes
        for cam, e in zip(self.cams, sent):
            x, y, z, p, yw, r = to_transform(e)
            self.st.pending.append(carla.command.ApplyTransform(
                cam.id, carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=p, yaw=yw, roll=r))))
        xr_eyes = eye_poses_xr(pred, ipd)
        self.history.put(snap.frame + 1, {"t": sample_t, "disp": disp_t, "head": head, "pred": pred,
                                          "horizon_ms": 1000 * horizon, "xr_eyes": xr_eyes, "eyes": eyes,
                                          "world_eyes": world_eyes})
        self._flush_log()

    def _parent_pose(self, snap, actor_id):
        find = getattr(snap, "find", None)
        s = find(actor_id) if find is not None else None
        tf = s.get_transform() if s is not None else self.world.get_actor(actor_id).get_transform()
        return from_transform(tf.location.x, tf.location.y, tf.location.z, tf.rotation.pitch, tf.rotation.yaw,
                              tf.rotation.roll)

    # -- images (CARLA's callback threads) -----------------------------------------------------------------------
    def _on_image(self, eye, image):
        import numpy as np
        now = time.perf_counter()
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        tf = image.transform
        both = self.pair.put(eye, image.frame, (arr, (tf.location.x, tf.location.y, tf.location.z,
                                                      tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll), now))
        if both is None:
            return
        rec = self.history.get(image.frame)
        if rec is None:
            return
        self.lat.add(1000 * (now - rec["t"]))
        hook = getattr(self.st, "_vr_images", None)
        if hook is not None:
            hook(image.frame)
        # the callback only hands the pair over: CARLA's stream thread must get back to reading at once (on 0.9.16 a
        # stream the client does not read stalls the server, and with it every tick)
        with self._cv:
            self._pending = (image.frame, [both[0][0], both[1][0]], rec, now, both[0][1])
            self._cv.notify()

    def _work(self):
        """Draws and submits the newest pair of eye images (an older one waiting is dropped)."""
        while True:
            with self._cv:
                while self._pending is None and not self._stop:
                    self._cv.wait(0.5)
                if self._stop:
                    return
                job, self._pending = self._pending, None
            self._submit(*job)

    def compose_and_submit(self, draw_eye=None):
        """In the station's loop: sets the drawer for the next eye images (draw_eye(i, img, rec, size, fov), rec the
        poses they were rendered at: "eyes" in the seat's parent frame, "world_eyes") and returns the newest left eye
        drawn since the last call (BGRA), or None. The images themselves are drawn and handed to the headset by a
        worker thread as soon as both eyes of a frame are in (_submit), a tick earlier than this loop would."""
        self.draw_eye = draw_eye
        with self.lock:
            left, self.last_left = self.last_left, None
        return left

    def _submit(self, frame, eyes, rec, t_img, img_tf):
        check = self.pose_error(frame, img_tf, rec["eyes"][0])
        self._check_attachment(check)
        size = (self.cam["width"], self.cam["height"])
        draw_eye = self.draw_eye
        if draw_eye is not None:
            eyes = [e.copy() for e in eyes]
            for i in range(2):
                try:
                    draw_eye(i, eyes[i], rec, size, self.cam["layer_fov"])
                except Exception as e:  # noqa: BLE001 - a drawing bug must not stop the headset
                    self.log(f"vr: drawing into the eye failed: {e}")
                    self.draw_eye = None
        self.hs.submit(frame, eyes, rec["xr_eyes"], (self.cam["layer_fov"], self.cam["layer_fov"]))
        with self.lock:
            self.records[frame] = [frame, rec["t"], rec["head"], rec["horizon_ms"], rec["pred"], t_img,
                                   1000 * (t_img - rec["t"]), check, rec["disp"]]
            self.last_left = eyes[0]
            self.fps_n += 1
            if time.perf_counter() - self.fps_t0 >= 5.0:
                self.fps = self.fps_n / (time.perf_counter() - self.fps_t0)
                self.fps_n, self.fps_t0 = 0, time.perf_counter()

    def _check_attachment(self, check):
        """The first 20 rendered poses of a camera in a car: off by more than 50 cm (median) means ApplyTransform is
        not relative to the car on this server: world poses from then on."""
        if check is None or self.seat is None or self.errors is None:
            return
        self.errors.append(check[1])
        if len(self.errors) >= 20:
            med = sorted(self.errors)[len(self.errors) // 2]
            if med > 50.0:
                self.relative = not self.relative
                self.log(f"vr: eye cameras rendered {med:.0f} cm from where they were put: ApplyTransform on attached "
                         f"cameras is {'relative' if self.relative else 'world'} here; switched")
                self.errors = []
            else:
                self.errors = None

    def pose_error(self, frame, img_tf, wanted):
        """(degrees, cm) between the left eye's pose as CARLA rendered it (the image's transform, put into the car's
        frame with the car's pose in that frame's snapshot) and the pose vr.py set: the check that ApplyTransform on
        an attached camera does what VRView assumes. None when that snapshot is not known (or not in a car)."""
        parent = self.parents.get(frame)
        if parent is None:
            if self.seat is not None or self.anchor is not None:
                return None
            parent = ((0.0, 0.0, 0.0), IDENTITY)
        got = compose(inverse(parent), from_transform(*img_tf))
        return (math.degrees(qangle(got[1], wanted[1])), 100 * math.dist(got[0], wanted[0]))

    def _flush_log(self):
        """Writes the rows of images the headset has shown (their display time is known then)."""
        if self.log_w is None:
            return
        with self.lock:
            pending = dict(self.records)
        for frame in sorted(pending):
            r = pending[frame]
            shown = self.hs.first_shown(frame)
            if shown is None:
                if len(pending) < 50:
                    continue
                shown = (None, None)
            with self.lock:
                self.records.pop(frame, None)
            pending.pop(frame, None)
            disp, actual = shown
            f, t, head, hz, pred, t_img, p2i, check, _disp = r
            reproj = math.degrees(qangle(pred[1], actual[1])) if actual is not None else None
            self.log_w.writerow([f, f"{t:.4f}", *(f"{v:.4f}" for v in head[0]), *(f"{v:.5f}" for v in head[1]),
                                 f"{hz:.1f}", *(f"{v:.5f}" for v in pred[1]), f"{t_img:.4f}", f"{p2i:.1f}",
                                 f"{disp:.4f}" if disp else "", f"{1000 * (disp - t):.1f}" if disp else "",
                                 f"{reproj:.2f}" if reproj is not None else "",
                                 f"{check[0]:.2f}" if check is not None else "",
                                 f"{check[1]:.1f}" if check is not None else ""])
            if disp:
                self.m2p = 1000 * (disp - t)

    def stats(self):
        """Numbers for the station's stats message."""
        return {"vr_fps": round(self.fps, 1), "vr_horizon_ms": round(self.lat.horizon_ms(), 1),
                "vr_pose_to_image_ms": self.lat.median(), "vr_size": f"{self.cam['width']}x{self.cam['height']}"
                if self.cam else None, "vr_motion_to_photon_ms": round(getattr(self, "m2p", 0.0) or 0.0, 1),
                "vr_runtime": self.cfg.get("runtime")}

    def recenter(self):
        self.recentered = False

    def head_yaw(self):
        """The head's yaw on the body (deg, + right), for a pedestrian's gaze."""
        rel = getattr(self, "rel", None)
        return quat_rotator(rel[1])[1] if rel is not None else 0.0

    def drawer(self, ux, panel=None):
        """draw_eye for compose_and_submit: the in-car displays on the car's panels (dashboard style), the station-drawn
        eHMIs, and the head-locked panel (a pygame surface with alpha) ~panel_distance_m ahead."""
        import numpy as np
        tex = None
        if panel is not None:
            import pygame
            tex = np.dstack([pygame.surfarray.array3d(panel), pygame.surfarray.array_alpha(panel)])
            tex = np.ascontiguousarray(tex.swapaxes(0, 1))
            if not tex[..., 3].any():
                tex = None
        incar = getattr(ux, "incar", None) if ux is not None else None
        cabin = getattr(ux, "cabin", None) if ux is not None else None
        displays = []
        if incar is not None and cabin is not None and incar.t0 is not None and self.cfg.get("displays_in_eyes", True) \
                and self.seat is not None:
            displays = cabin_textures(incar, cabin)
        segs = None
        if ux is not None and (getattr(ux, "ehmi_draw", None) or getattr(ux, "ehmi_cabin", None)):
            try:
                segs = ux._ehmi_segments()
            except RuntimeError:
                segs = None
        ipd = self.ipd if self.ipd is not None else getattr(self.hs, "ipd_m", None) or 0.063
        quad = head_locked_quad(float(self.cfg.get("panel_distance_m", 1.5)), float(self.cfg.get("panel_width_deg", 50)),
                                tex.shape[0] / tex.shape[1]) if tex is not None else None
        if tex is not None:
            tex, quad = crop_panel(tex, quad)
        hfov = self.cam["hfov_deg"]

        def draw(i, img, rec, size, fov):
            for corners, rgba, additive, gain in displays:
                px = quad_in_eye(corners, rec["eyes"][i], size, fov)
                if px is not None:
                    warp_into(img, rgba, px, additive, gain)
            if segs is not None and (segs[0] or segs[1]):
                draw_segments(img, segs, to_transform(rec["world_eyes"][i]), size, hfov)
            if quad is not None:
                eye = (eye_offsets_ue(ipd)[i], IDENTITY)
                px = quad_in_eye(quad, eye, size, fov)
                if px is not None:
                    warp_into(img, tex, px)
        return draw

    def stop(self):
        with self._cv:
            self._stop = True
            self._cv.notify()
        for cam in self.cams:
            if cam is not None:
                try:
                    cam.stop()
                    cam.destroy()
                except RuntimeError:
                    pass
        self.cams = [None, None]
        self._flush_log()
        if self.log_f is not None:
            self.log_f.close()
            self.log_f = None


def cabin_textures(incar, cabin):
    """[(corners in the vehicle frame, RGBA texture, additive, gain)] of the in-car displays drawn on the car's panels
    (incar.InCar, cockpit.CabinView), as InCar._draw_dashboard draws them for a monitor."""
    import numpy as np
    import pygame
    from .cockpit import quad
    if incar.painter is None:
        from .incar import Painter
        incar.painter = Painter(pygame)
    out = []
    for name, disp in incar.displays.items():
        if disp.get("own_rect"):
            continue
        d = disp["design"]
        aspect = d["height"] / max(1e-6, d["width"])
        size = cabin.tex_size(name, aspect)
        if size is None:
            continue
        surf = incar._panel_for(name, disp, size)
        rgba = np.dstack([pygame.surfarray.array3d(surf), pygame.surfarray.array_alpha(surf)])
        centre, normal, (w, h) = cabin.panel(name, aspect)
        out.append((quad(centre, normal, w, h), np.ascontiguousarray(rgba.swapaxes(0, 1)), name == "hud",
                    incar.hud_gain if name == "hud" else incar.screen_gain))
    return out


def draw_segments(img, segs, cam, size, hfov):
    """Station-drawn eHMIs (stationux: segments seen from outside, and the own car's panels seen from behind as added
    light) projected into an eye image (BGRA) whose camera is cam (x, y, z, pitch, yaw, roll) with this fov."""
    import numpy as np
    from PIL import Image, ImageDraw
    from .ehmi_design import project_segments
    outside, behind = segs
    pil = Image.fromarray(np.ascontiguousarray(img[..., 2::-1]))
    d = ImageDraw.Draw(pil)
    for u0, v0, u1, v1, wpx, rgb, b in project_segments(outside, cam, size, hfov):
        d.line((u0, v0, u1, v1), fill=tuple(int(min(255, c * b)) for c in rgb), width=wpx)
    out = np.asarray(pil)
    if behind:
        glow = Image.new("RGB", pil.size)
        g = ImageDraw.Draw(glow)
        for u0, v0, u1, v1, wpx, rgb, b in project_segments(behind, cam, size, hfov):
            g.line((u0, v0, u1, v1), fill=tuple(int(min(255, c * b)) for c in rgb), width=wpx)
        out = np.clip(out.astype(np.int16) + np.asarray(glow, np.int16), 0, 255).astype(np.uint8)
    img[..., :3] = out[..., ::-1]


def log_path(station_log):
    """logs/stations/<role>_<time>_vr.csv next to the station's frame log."""
    p = Path(station_log)
    return p.with_name(p.stem + "_vr.csv")
