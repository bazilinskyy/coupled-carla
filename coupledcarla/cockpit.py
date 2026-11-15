"""The in-car views of drivers and AV passengers: where the eyes sit in each car, and the car's own displays.

Eye points per vehicle (data/driver_eyes.json, measured by tools/cabin_measure.py on a running server): the
driver's eye from the steering wheel and the seat (SAE J941/J1100 in spirit: ~0.635 m above the H-point, ~0.65 m
behind the wheel centre), the front passenger's mirrored. CARLA's cars are all left-hand drive; on a left-hand
traffic map (`map.lefthand`) the seats swap sides: the eye's y is mirrored (the wheel in the mesh stays on the left,
so a right-hand-drive driver sees no wheel in front: a limit of the meshes). Unknown cars get a default scaled by
their bounding box.

Displays on the car's own panels (displays.style: dashboard): the cluster's and centre screen's designs are drawn
into the car's instrument-panel and centre-screen quads (measured or hand-placed per car, else derived from the
wheel), warped into the camera image perspective-correct as lit screens, and hidden where the steering wheel's rim
and spokes or the dashboard stand in front (one depth image of the rigid cabin view, taken at the start). The HUD
is a virtual image ~2.5 m ahead (a windscreen projection: its light is added to the image, so no box shows).
`displays.style: overlay` keeps the flat panels at the screen's edges. CARLA 0.9.16's cars have no steering-wheel
bone, so the mesh's wheel never turns: WheelSpin turns its pixels with the car's steering.
"""
import json
import math
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data" / "driver_eyes.json"
DEFAULT_PITCH = -4.0          # deg: eyes rest a little below the horizon (the road ahead), as a driver's do
CABIN_EV_BIAS = -1.0          # EV: a cabin view's fixed exposure, one stop brighter than the street's (look.py)
HAND_TUNED = ("cluster", "centre", "pitch", "note", "eye_override", "glass_ev")    # kept when measured again
_cache = {}


# --- eye points --------------------------------------------------------------------------------------------------

def load(path=DATA):
    p = str(path)
    if p not in _cache:
        try:
            _cache[p] = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            _cache[p] = {}
    return _cache[p]


def dump_eyes(data):
    """The data file's text: one car per line, sorted, so diffs stay readable."""
    out = ["{"]
    keys = [k for k in data if not isinstance(data[k], dict) or k.startswith("_")] + \
        sorted(k for k in data if isinstance(data[k], dict) and not k.startswith("_"))
    for i, k in enumerate(keys):
        v = data[k]
        end = "," if i < len(keys) - 1 else ""
        if isinstance(v, dict) and not k.startswith("_"):
            out.append(f'"{k}": {{')
            cars = sorted(v)
            for j, c in enumerate(cars):
                out.append(f'  "{c}": {json.dumps(v[c], sort_keys=True)}' + ("," if j < len(cars) - 1 else ""))
            out.append("}" + end)
        else:
            out.append(f'"{k}": {json.dumps(v)}{end}')
    out.append("}")
    return "\n".join(out) + "\n"


def default_eye(extent, bb_x=0.0, base_type="car"):
    """A size-scaled guess for a car nobody measured: (x, y, z) of the driver's eye in the vehicle's frame
    (left-hand drive). extent: the bounding box's half sizes (m); bb_x its centre's x. Cars: a little behind the
    middle at 80 % of the height (the Lincoln MKZ's measured eye scales from these); vans, trucks and buses sit
    over the front axle, at 78 % of the height."""
    ex, ey, ez = (float(v) for v in extent)
    if str(base_type).lower() in ("van", "truck", "bus"):
        x = bb_x + ex - min(1.45, 0.42 * ex + 0.35)
        return [round(x, 3), round(-0.48 * ey, 3), round(1.56 * ez, 3)]
    return [round(bb_x - 0.06 * ex, 3), round(-0.42 * ey, 3), round(1.6 * ez, 3)]


def major_key(version):
    """'0.9' for CARLA 0.9.x/0.10.x, '1.0' for CARLA 1.0 and later (a server version string or an int major)."""
    if isinstance(version, int):
        return "1.0" if version >= 1 else "0.9"
    head = str(version or "0").split(".")[0].split("-")[0]
    try:
        return "1.0" if int(head) >= 1 else "0.9"
    except ValueError:
        return "0.9"


def entry_for(type_id, version="0.9", data=None):
    """The data file's entry for a blueprint (this version's table first, then the other's), or None."""
    data = load() if data is None else data
    first = major_key(version)
    for key in (first, "1.0" if first == "0.9" else "0.9"):
        e = (data.get(key) or {}).get(type_id)
        if e:
            return e
    return None


def mirror(p):
    """A point in the vehicle frame mirrored to the other side (y -> -y)."""
    return [p[0], -p[1], p[2]] + list(p[3:])


def eye_for(type_id, seat="driver", lefthand=False, extent=None, bb_x=0.0, base_type="car", version="0.9",
            data=None):
    """(x, y, z, pitch) of the eye in the vehicle's frame for `seat` (driver | passenger) of blueprint type_id.

    Measured cars come from the data file; others get default_eye from their bounding box (extent, bb_x). On a
    left-hand-traffic map the driver sits on the right: both seats are mirrored in y."""
    e = entry_for(type_id, version, data)
    if e:
        drv = list(e.get("eye_override") or e["eye"])
        pas = list(e.get("passenger") or mirror(drv))
        pitch = float(e.get("pitch", DEFAULT_PITCH))
    else:
        drv = default_eye(extent or (2.45, 0.92, 0.75), bb_x, base_type)
        pas = mirror(drv)
        pitch = DEFAULT_PITCH
    p = drv if seat == "driver" else pas
    if lefthand:
        p = mirror(p)
    return (float(p[0]), float(p[1]), float(p[2]), pitch)


def actor_eye(actor, seat="driver", lefthand=False, version="0.9", data=None):
    """eye_for() for a spawned vehicle actor (its blueprint id, bounding box and base_type)."""
    bb = getattr(actor, "bounding_box", None)
    attrs = getattr(actor, "attributes", None) or {}
    ext = (bb.extent.x, bb.extent.y, bb.extent.z) if bb is not None else None
    return eye_for(getattr(actor, "type_id", ""), seat, lefthand, ext, bb.location.x if bb is not None else 0.0,
                   attrs.get("base_type", "car") or "car", version, data)


def cabin_ev_bias(entry, base=CABIN_EV_BIAS):
    """A cabin view's exposure bias (EV, negative = brighter than the street's exposure): `base`, less the car's
    measured glass tint (entry "glass_ev": how many EV darker the street looks through its windscreen; the Lincoln
    MKZ 2020's glass, for one, is dark), kept within -2.5..0."""
    return max(-2.5, min(0.0, float(base) - float((entry or {}).get("glass_ev") or 0.0)))


def hfov_for(screens=1, screen_deg=None, default=90.0):
    """Horizontal field of view of a station's view: an explicit per-screen angle times the screens, else 90 deg for
    one monitor (a 27" screen at ~60 cm subtends ~50 deg, but 90 shows the mirrors' surroundings and the A-pillars
    as a driver sees them in the corners of the eye) and up to 180 for three."""
    n = max(1, int(screens or 1))
    if screen_deg:
        return float(min(179.0, float(screen_deg) * n))
    return float(min(179.0, default if n == 1 else 60.0 * n))


# --- projection ----------------------------------------------------------------------------------------------------
# Camera frame as CARLA's (UE): x forward, y right, z up. A camera at the eye E with pitch p (deg, + looks up), yaw 0
# and roll 0 relative to the vehicle; image u to the right, v down; f = W / (2 tan(hfov / 2)).

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(_dot(a, a)) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


def focal(size, fov):
    return size[0] / (2.0 * math.tan(math.radians(fov) / 2.0))


def cam_axes(pitch):
    p = math.radians(pitch)
    return (math.cos(p), 0.0, math.sin(p)), (0.0, 1.0, 0.0), (-math.sin(p), 0.0, math.cos(p))


def to_cam(point, eye, pitch):
    """A vehicle-frame point in the camera's frame (forward, right, up)."""
    fwd, right, up = cam_axes(pitch)
    d = _sub(point, eye)
    return (_dot(d, fwd), _dot(d, right), _dot(d, up))


def project(point, eye, pitch, size, fov):
    """(u, v, depth) of a vehicle-frame point in the image of a camera at eye (x, y, z) with this pitch; depth is
    the distance along the view axis (what CARLA's depth camera gives), <= 0 behind the camera."""
    x, y, z = to_cam(point, eye, pitch)
    f = focal(size, fov)
    if x <= 1e-6:
        return (float("nan"), float("nan"), x)
    return (size[0] / 2.0 + f * y / x, size[1] / 2.0 - f * z / x, x)


def panel_axes(normal):
    """In-plane (right, up) unit vectors of a display facing `normal` (pointing at the viewer): right is the
    viewer's right (+y for a normal along -x), up leans with the panel's tilt."""
    n = _unit(normal)
    right = _cross(n, (0.0, 0.0, 1.0))
    if math.sqrt(_dot(right, right)) < 1e-6:
        right = (0.0, 1.0, 0.0)
    right = _unit(right)
    return right, _unit(_cross(right, n))


def quad(centre, normal, width, height):
    """Corners (top-left, top-right, bottom-right, bottom-left) of a display rectangle in the vehicle frame."""
    r, u = panel_axes(normal)
    c, hw, hh = centre, width / 2.0, height / 2.0
    return [tuple(c[i] + sr * r[i] * hw + su * u[i] * hh for i in range(3))
            for sr, su in ((-1, 1), (1, 1), (1, -1), (-1, -1))]


def homography(src, dst):
    """3x3 H (row-major list of 9, H[8] = 1) with dst ~ H src for four point pairs [(x, y)], or None if
    degenerate."""
    a, b = [], []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        b.append(u)
        a.append([0, 0, 0, x, y, 1, -v * x, -v * y])
        b.append(v)
    sol = _solve(a, b)
    return None if sol is None else sol + [1.0]


def _solve(a, b):
    """Gaussian elimination with partial pivoting (8 x 8): no numpy needed for the maths."""
    n = len(b)
    m = [list(map(float, row)) + [float(bb)] for row, bb in zip(a, b)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r != col:
                k = m[r][col] / m[col][col]
                if k:
                    m[r] = [x - k * y for x, y in zip(m[r], m[col])]
    return [m[i][n] / m[i][i] for i in range(n)]


def apply_h(h, x, y):
    w = h[6] * x + h[7] * y + h[8]
    return ((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w)


def plane_depth(u, v, centre, normal, eye, pitch, size, fov):
    """Depth (along the view axis) at which pixel (u, v)'s ray meets the plane through centre with normal, or None
    when the ray runs parallel to it or meets it behind the camera."""
    f = focal(size, fov)
    ray = (1.0, (u - size[0] / 2.0) / f, -(v - size[1] / 2.0) / f)       # camera frame, depth 1
    c = to_cam(centre, eye, pitch)
    fwd, right, up = cam_axes(pitch)
    n = (_dot(normal, fwd), _dot(normal, right), _dot(normal, up))
    den = _dot(n, ray)
    if abs(den) < 1e-9:
        return None
    t = _dot(n, c) / den
    return t if t > 0 else None


# --- the car's own displays -----------------------------------------------------------------------------------------
WHEEL_TILT_DEG = 25.0          # a wheel without measurements: its plane leans 25 deg back from vertical
CLUSTER_SIZE = (0.28, 0.11)    # m: a 12" digital cluster (Lincoln, Mercedes and VW are 0.28-0.31 m wide)
CENTRE_SIZE = (0.22, 0.14)     # m: a 10" centre screen
HUD_DISTANCE = 2.5             # m: the HUD's virtual image (2-3 m in production cars; coupled-sim's HUD sits there)
# the HUD's width as seen from the eye (deg): AR HUDs are 10-15 deg, but a 90-deg view on a monitor shows it ~1.6x
# smaller, and 20 keeps its digits readable
HUD_FOV = 20.0
HUD_DOWN = 5.0                 # deg: its centre below the straight-ahead line of sight
STEER_RATIO = 7.0              # steering wheel degrees per front-wheel degree (inputs/steering_wheel.py's 7:1)


def wheel_of(entry, eye=None):
    """(centre, axis, radius) of the steering wheel: measured, else placed in front of a default eye."""
    w = (entry or {}).get("wheel")
    if w:
        return tuple(w["centre"]), _unit(tuple(w["axis"])), float(w["radius"])
    e = eye or (entry or {}).get("eye") or default_eye((2.45, 0.92, 0.75))
    t = math.radians(WHEEL_TILT_DEG)
    return (e[0] + 0.62, e[1], e[2] - 0.40), (-math.cos(t), 0.0, math.sin(t)), 0.19


def _panel_spec(spec):
    return tuple(spec["centre"]), _unit(tuple(spec["normal"])), tuple(spec["size"])


def cluster_panel(entry, aspect=None, eye=None):
    """(centre, normal, (w, h)) of the instrument cluster: the entry's own (hand-placed on the mesh's panel), else
    behind the wheel, 0.16 m beyond its plane and seen through the upper half of the rim from the driver's eye."""
    if (entry or {}).get("cluster"):
        return _panel_spec(entry["cluster"])
    c, n, r = wheel_of(entry, eye)
    e = list((entry or {}).get("eye") or eye or (c[0] - 0.65, c[1], c[2] + 0.42))
    right, up = panel_axes(n)
    through = tuple(c[i] + up[i] * 0.55 * r for i in range(3))     # between the hub and the rim's top
    plane_c = tuple(c[i] - n[i] * 0.16 for i in range(3))
    ray = _sub(through, e)
    den = _dot(ray, n)
    s = _dot(_sub(plane_c, e), n) / den if abs(den) > 1e-9 else 1.0
    centre = tuple(e[i] + ray[i] * s for i in range(3))
    w = CLUSTER_SIZE[0]
    h = w * aspect if aspect else CLUSTER_SIZE[1]
    return centre, n, (w, h)


def centre_panel(entry, aspect=None, eye=None):
    """(centre, normal, (w, h)) of the centre screen: the entry's own, else on the centre line level with the
    wheel's hub, 0.10 m ahead of it, leaning like the wheel and turned 12 deg towards the driver."""
    if (entry or {}).get("centre"):
        return _panel_spec(entry["centre"])
    c, n, r = wheel_of(entry, eye)
    yaw = math.radians(12.0) * (1 if c[1] < 0 else -1)
    nn = (n[0] * math.cos(yaw) - n[1] * math.sin(yaw), n[0] * math.sin(yaw) + n[1] * math.cos(yaw), n[2])
    w = CENTRE_SIZE[0]
    h = w * aspect if aspect else CENTRE_SIZE[1]
    return (c[0] + 0.10, 0.0, c[2] + 0.02), _unit(nn), (w, h)


def hud_panel(eye, pitch=0.0, aspect=0.27, distance=HUD_DISTANCE, fov=HUD_FOV, down=HUD_DOWN):
    """(centre, normal, (w, h)) of the HUD's virtual image: `distance` m ahead of the eye, `down` deg below the
    straight-ahead line, facing the eye, `fov` deg wide."""
    a = math.radians(down)
    centre = (eye[0] + distance * math.cos(a), eye[1], eye[2] - distance * math.sin(a))
    w = 2 * distance * math.tan(math.radians(fov) / 2)
    return centre, (-math.cos(a), 0.0, math.sin(a)), (w, w * aspect)


def mirror_panel(p):
    c, n, s = p
    return (c[0], -c[1], c[2]), (n[0], -n[1], n[2]), s


def wheel_degrees(steer, max_steer_deg=70.0, ratio=STEER_RATIO, lock=450.0):
    """The steering wheel's angle (deg, + clockwise as the driver sees it: turning right) for CARLA's steer in
    [-1, 1]: front wheels at steer x max_steer_angle, times the steering ratio, within the wheel's lock."""
    return max(-lock, min(lock, float(steer) * float(max_steer_deg) * float(ratio)))


def panel_corners_px(panel, eye, pitch, size, fov):
    """The panel's four corners projected [(u, v)] (top-left, top-right, bottom-right, bottom-left), or None when
    any lies behind the camera."""
    c, n, (w, h) = panel
    out = []
    for p in quad(c, n, w, h):
        u, v, d = project(p, eye, pitch, size, fov)
        if d <= 0.05:
            return None
        out.append((u, v))
    return out


def warp_coeffs(tex_size, corners, origin=(0, 0)):
    """PIL's PERSPECTIVE coefficients (output pixel -> texture pixel) that put a texture of tex_size (w, h) onto
    the image quad `corners` (TL, TR, BR, BL), for an output image whose top-left sits at `origin`."""
    tw, th = tex_size
    src = [(0.0, 0.0), (tw, 0.0), (tw, th), (0.0, th)]
    dst = [(u - origin[0], v - origin[1]) for u, v in corners]
    return homography(dst, src)


# --- compositing (numpy, Pillow) -------------------------------------------------------------------------------------

def _rays(size, fov, x0, y0, x1, y1):
    """Camera-frame rays (depth 1) of the pixel centres in [x0, x1) x [y0, y1): (forward, right, up) arrays."""
    import numpy as np
    f = focal(size, fov)
    u, v = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
    return np.ones_like(u), (u - size[0] / 2.0) / f, -(v - size[1] / 2.0) / f


def _plane_depths(rays, centre, normal, eye, pitch):
    """Depth at which each ray meets the plane (inf where it does not, in front of the camera)."""
    import numpy as np
    fwd, right, up = cam_axes(pitch)
    n = (_dot(normal, fwd), _dot(normal, right), _dot(normal, up))
    c = to_cam(centre, eye, pitch)
    den = n[0] * rays[0] + n[1] * rays[1] + n[2] * rays[2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = _dot(n, c) / den
    return np.where((np.abs(den) > 1e-9) & (t > 0), t, np.inf)


def _in_plane(rays, t, centre, normal, eye, pitch):
    """Where the rays meet the plane at depth t: offsets from centre as (along right, along up) of the panel."""
    fwd, right, up = cam_axes(pitch)
    pr, pu = panel_axes(normal)
    d = _sub(eye, centre)
    out = []
    for ax in (pr, pu):
        a = (_dot(fwd, ax), _dot(right, ax), _dot(up, ax))
        out.append(_dot(d, ax) + t * (a[0] * rays[0] + a[1] * rays[1] + a[2] * rays[2]))
    return out


class WheelSpin:
    """Turns the steering wheel in the image: CARLA 0.9.16's cars have no steering-wheel bone, so the mesh's wheel
    never moves. Its pixels (found once in the rigid cabin view's depth image: within 5 cm of the wheel's plane and
    its radius) are rotated about the wheel's axis by a homography of the wheel plane; the pixels it uncovers blend
    the nearest pixels outside the wheel to their left and right, above and below."""

    def __init__(self, depth, wheel, eye, pitch, size, fov):
        import numpy as np
        c, n, r = wheel
        right, up = panel_axes(n)
        pts = [tuple(c[i] + sa * (r + 0.04) * right[i] + sb * (r + 0.04) * up[i] for i in range(3))
               for sa, sb in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
        uv = [project(p, eye, pitch, size, fov) for p in pts]
        self.ok = all(d > 0.05 for _u, _v, d in uv)
        if not self.ok:
            return
        x0 = max(0, int(min(p[0] for p in uv)) - 2)
        x1 = min(size[0], int(max(p[0] for p in uv)) + 3)
        y0 = max(0, int(min(p[1] for p in uv)) - 2)
        y1 = min(size[1], int(max(p[1] for p in uv)) + 3)
        self.roi = (x0, y0, x1, y1)
        if x1 - x0 < 4 or y1 - y0 < 4:
            self.ok = False
            return
        rays = _rays(size, fov, x0, y0, x1, y1)
        dep = np.asarray(depth, dtype=np.float64)[y0:y1, x0:x1]
        fwd, rgt, upc = cam_axes(pitch)
        p = [eye[i] + dep * (fwd[i] * rays[0] + rgt[i] * rays[1] + upc[i] * rays[2]) for i in range(3)]
        d = [p[i] - c[i] for i in range(3)]
        h = d[0] * n[0] + d[1] * n[1] + d[2] * n[2]
        rad = np.sqrt(sum((d[i] - h * n[i]) ** 2 for i in range(3)))
        self.mask = (np.abs(h) < 0.05) & (rad < r + 0.025) & (dep < 2.0)
        self.ok = bool(self.mask.sum() > 50)
        if not self.ok:
            return
        # plane coordinates (a, b) in metres along (right, up) <-> image pixels of the ROI
        corners = [(-1, 1), (1, 1), (1, -1), (-1, -1)]
        img = [(u - x0, v - y0) for u, v, _d in uv]
        self.G = np.array(homography([(a * (r + 0.04), b * (r + 0.04)) for a, b in corners], img)).reshape(3, 3)
        self.Ginv = np.linalg.inv(self.G)
        rows = self._row_fill(self.mask)
        li, ri, w = self._row_fill(self.mask.T)          # the same along columns, back in row-major indices
        hh, ww = self.mask.shape
        to_rm = lambda k: (k % hh) * ww + k // hh                      # noqa: E731 - index in the transposed ROI
        cols = (to_rm(li).reshape(ww, hh).T.ravel(), to_rm(ri).reshape(ww, hh).T.ravel(), w.reshape(ww, hh).T.ravel())
        self.fill = (rows, cols)
        hh, ww = self.mask.shape
        yy, xx = np.mgrid[0:hh, 0:ww]
        self.grid = np.stack([xx.ravel() + 0.5, yy.ravel() + 0.5, np.ones(xx.size)])
        self.layer = None

    @staticmethod
    def _row_fill(mask):
        """For the pixels under the wheel: the flat indices of the nearest pixels outside it to the left and right
        in the same row and the right one's weight (a linear blend between them: what lies behind the wheel)."""
        import numpy as np
        hh, ww = mask.shape
        cols = np.arange(ww)
        left = np.where(~mask, cols[None, :], -1)
        left = np.maximum.accumulate(left, axis=1)
        right = np.where(~mask, cols[None, :], ww)
        right = np.minimum.accumulate(right[:, ::-1], axis=1)[:, ::-1]
        nearest = WheelSpin._nearest_outside(mask)
        rows = np.arange(hh)[:, None] * ww
        has_l, has_r = left >= 0, right < ww
        li = np.where(has_l, rows + np.clip(left, 0, ww - 1), nearest.reshape(hh, ww))
        ri = np.where(has_r, rows + np.clip(right, 0, ww - 1), li)
        li = np.where(has_l, li, ri)
        span = np.maximum(1, right - left)
        w = np.where(has_l & has_r, (cols[None, :] - left) / span, 0.0)
        return li.ravel(), ri.ravel(), w.ravel()

    @staticmethod
    def _nearest_outside(mask):
        """For every pixel the flat index of the nearest pixel outside the mask (itself when outside)."""
        import numpy as np
        hh, ww = mask.shape
        idx = np.where(mask, -1, np.arange(hh * ww).reshape(hh, ww))
        for _ in range(hh + ww):
            if (idx >= 0).all():
                break
            for sh in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nb = np.roll(idx, sh, axis=(0, 1))
                if sh[0] == 1:
                    nb[0, :] = -1
                elif sh[0] == -1:
                    nb[-1, :] = -1
                elif sh[1] == 1:
                    nb[:, 0] = -1
                else:
                    nb[:, -1] = -1
                idx = np.where((idx < 0) & (nb >= 0), nb, idx)
        return np.where(idx < 0, 0, idx).ravel()

    def lift(self, img, degrees):
        """Takes the wheel out of img (H x W x 3, changed in place) and keeps it turned by `degrees` (+ clockwise as
        the driver sees it) for put_back(). Returns False (and leaves img alone) when there is nothing to turn."""
        import numpy as np
        self.layer = None
        if not self.ok or abs(degrees) < 0.3:
            return False
        x0, y0, x1, y1 = self.roi
        roi = np.array(img[y0:y1, x0:x1])
        flat = roi.reshape(-1, roi.shape[2])
        th = math.radians(degrees)
        rot = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1.0]])
        # destination pixel -> plane -> turned back -> source pixel (clockwise for the driver: top moves right)
        m = self.G @ rot @ self.Ginv
        s = m @ self.grid
        sx = np.floor(s[0] / s[2]).astype(np.int64)
        sy = np.floor(s[1] / s[2]).astype(np.int64)
        hh, ww = self.mask.shape
        inside = (sx >= 0) & (sx < ww) & (sy >= 0) & (sy < hh)
        src = np.clip(sy, 0, hh - 1) * ww + np.clip(sx, 0, ww - 1)
        on = inside & self.mask.ravel()[src]
        behind = 0.0
        for li, ri, w in self.fill:      # what lies behind the wheel: the row's and the column's blends, averaged
            behind = behind + 0.5 * (flat[li] * (1 - w[:, None]) + flat[ri] * w[:, None])
        behind = behind.astype(roi.dtype).reshape(roi.shape)
        img[y0:y1, x0:x1] = np.where(self.mask[..., None], behind, roi)
        self.layer = (on.reshape(hh, ww), flat[src].reshape(roi.shape))
        return True

    def put_back(self, img):
        if self.layer is None:
            return
        import numpy as np
        x0, y0, x1, y1 = self.roi
        on, pix = self.layer
        img[y0:y1, x0:x1] = np.where(on[..., None], pix, img[y0:y1, x0:x1])
        self.layer = None

    def full_mask(self, size):
        """The wheel's pixels as a full-image boolean mask (H x W)."""
        import numpy as np
        m = np.zeros((size[1], size[0]), bool)
        if self.ok:
            x0, y0, x1, y1 = self.roi
            m[y0:y1, x0:x1] = self.mask
        return m


class CabinView:
    """One station's view from a seat: the car's display panels in its image and the steering wheel turning.

    entry: the car's data file entry (cockpit.entry_for) or None; eye (x, y, z) and pitch: this camera in the vehicle
    frame; size (w, h) and fov: the image the displays are drawn into; lefthand: the driver sits on the right;
    depth: the rigid cabin view's depth image (H x W, metres; set later with set_depth) for hiding panels behind the
    wheel and the dashboard and for finding the wheel's pixels."""

    def __init__(self, entry, eye, pitch, size, fov, lefthand=False, depth=None, hud=None):
        self.entry, self.eye, self.pitch = entry or {}, tuple(eye), float(pitch)
        self.size, self.fov, self.lefthand = tuple(int(v) for v in size), float(fov), bool(lefthand)
        self.hud = dict(hud or {})
        drv = list(self.entry.get("eye_override") or self.entry.get("eye") or
                   default_eye((2.45, 0.92, 0.75)))
        self.driver_eye = tuple(mirror(drv) if lefthand else drv)
        w = wheel_of(self.entry, drv)
        self.wheel = ((w[0][0], -w[0][1], w[0][2]), (w[1][0], -w[1][1], w[1][2]), w[2]) if lefthand else w
        self.panels, self._prep = {}, {}
        self.depth = None
        self.spin = None
        self._wheel_px = None
        if depth is not None:
            self.set_depth(depth)

    def set_depth(self, depth):
        import numpy as np
        d = np.asarray(depth, dtype=np.float32)
        if d.shape != (self.size[1], self.size[0]):
            from PIL import Image
            d = np.asarray(Image.fromarray(d).resize(self.size, Image.NEAREST))
        self.depth = d
        self._prep = {}
        sp = WheelSpin(d, self.wheel, self.eye, self.pitch, self.size, self.fov)
        self.spin = sp if sp.ok else None
        self._wheel_px = sp.full_mask(self.size) if sp.ok else None

    def _settle(self, pnl):
        """A derived panel moved along its up axis (at most 15 cm down or 10 cm up) and towards the viewer (at most
        10 cm: a cluster in a shallow binnacle), the smallest move first, until it lies on the dashboard: under 5 % of
        it floating in front of what is far behind it (the windscreen, the street) and under 20 % hidden inside the
        dashboard (the wheel's pixels do not count)."""
        import numpy as np
        c, n, (w, h) = pnl
        right, up = panel_axes(n)
        moves = sorted(((k / 100.0, j / 100.0) for k in range(-15, 11) for j in range(0, 11, 2)),
                       key=lambda m: (abs(m[0]) + m[1], m[1], m[0] > 0))
        for off, fwd in moves:
            cc = tuple(c[i] + up[i] * off + n[i] * fwd for i in range(3))
            corners = panel_corners_px((cc, n, (w, h)), self.eye, self.pitch, self.size, self.fov)
            if corners is None:
                continue
            x0 = max(0, int(min(u for u, _v in corners)))
            x1 = min(self.size[0], int(max(u for u, _v in corners)) + 1)
            y0 = max(0, int(min(v for _u, v in corners)))
            y1 = min(self.size[1], int(max(v for _u, v in corners)) + 1)
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            rays = _rays(self.size, self.fov, x0, y0, x1, y1)
            t = _plane_depths(rays, cc, n, self.eye, self.pitch)
            a, b = _in_plane(rays, t, cc, n, self.eye, self.pitch)
            inside = (np.abs(a) <= w / 2) & (np.abs(b) <= h / 2)
            if self._wheel_px is not None:
                inside &= ~self._wheel_px[y0:y1, x0:x1]
            if inside.sum() < 10:
                continue
            d = self.depth[y0:y1, x0:x1]
            floating = float((d[inside] > t[inside] + 0.15).mean())
            hidden = float((d[inside] < t[inside] - 0.04).mean())
            if floating < 0.05 and hidden < 0.2:
                return cc, n, (w, h)
        return pnl

    def panel(self, name, aspect):
        """The panel's geometry (centre, normal, (w, h)) for a design of this height/width aspect: the entry's own,
        else derived from the wheel and, once the depth image is in, settled onto the dashboard (_settle)."""
        key = (name, round(aspect, 4), self.depth is not None)
        if key not in self.panels:
            src = dict(self.entry)
            src["eye"] = list(mirror(self.driver_eye) if self.lefthand else self.driver_eye)
            if name == "cluster":
                p = cluster_panel(src, aspect)
            elif name == "centre":
                p = centre_panel(src, aspect)
            else:
                p = hud_panel(self.eye, self.pitch, aspect, float(self.hud.get("distance_m", HUD_DISTANCE)),
                              float(self.hud.get("fov_deg", HUD_FOV)), float(self.hud.get("down_deg", HUD_DOWN)))
            if self.lefthand and name != "hud":
                p = mirror_panel(p)
            if name != "hud" and self.depth is not None and not self.entry.get(name):
                p = self._settle(p)
            self.panels[key] = p
        return self.panels[key]

    def tex_size(self, name, aspect, scale=2.0, cap=900):
        """A texture size (w, h) for a panel: about `scale` x its projected width in pixels."""
        corners = panel_corners_px(self.panel(name, aspect), self.eye, self.pitch, self.size, self.fov)
        if corners is None:
            return None
        wpx = max(math.dist(corners[0], corners[1]), math.dist(corners[3], corners[2]))
        w = int(max(64, min(cap, wpx * scale)))
        return w, max(16, int(round(w * aspect)))

    def _prepare(self, name, aspect, tex):
        import numpy as np
        key = (name, round(aspect, 4), tex)
        if key in self._prep:
            return self._prep[key]
        pnl = self.panel(name, aspect)
        corners = panel_corners_px(pnl, self.eye, self.pitch, self.size, self.fov)
        prep = None
        if corners is not None:
            x0 = max(0, int(math.floor(min(u for u, _v in corners))))
            x1 = min(self.size[0], int(math.ceil(max(u for u, _v in corners))) + 1)
            y0 = max(0, int(math.floor(min(v for _u, v in corners))))
            y1 = min(self.size[1], int(math.ceil(max(v for _u, v in corners))) + 1)
            if x1 - x0 >= 2 and y1 - y0 >= 2:
                coeffs = warp_coeffs(tex, corners, (x0, y0))
                vis = np.ones((y1 - y0, x1 - x0), np.float32)
                rays = _rays(self.size, self.fov, x0, y0, x1, y1)
                if name == "hud":
                    pass        # light on the windscreen: no mask (the 2020 cars' glass is in the depth image)
                else:
                    t = _plane_depths(rays, pnl[0], pnl[1], self.eye, self.pitch)
                    if self.depth is not None:      # hidden where the wheel, the hood or the dashboard stand in front
                        vis = (self.depth[y0:y1, x0:x1] > t - 0.04).astype(np.float32)
                    else:                           # no depth image: hidden behind the wheel's rim (its geometry)
                        c, n, r = self.wheel
                        tw = _plane_depths(rays, c, n, self.eye, self.pitch)
                        a, b = _in_plane(rays, tw, c, n, self.eye, self.pitch)
                        rr = np.sqrt(a * a + b * b)
                        vis = (~((tw < t) & (rr > r - 0.035) & (rr < r + 0.01))).astype(np.float32)
                wheel = self._wheel_px[y0:y1, x0:x1] if self._wheel_px is not None else None
                prep = {"roi": (x0, y0, x1, y1), "coeffs": coeffs, "vis": vis, "wheel": wheel}
        self._prep[key] = prep
        return prep

    def draw(self, img, name, tex_rgba, additive=False, gain=1.0, spinning=False):
        """Warps a display's texture (h x w x 4 uint8, RGBA) into its panel in img (H x W x 3 uint8, changed in
        place): alpha-blended (a lit screen) or added (the HUD's light on the windscreen), times gain. spinning:
        the wheel is lifted out (WheelSpin) and put back after, so the panel is drawn behind it in full."""
        import numpy as np
        from PIL import Image
        th, tw = tex_rgba.shape[:2]
        aspect = th / max(1, tw)
        prep = self._prepare(name, aspect, (tw, th))
        if prep is None or prep["coeffs"] is None:
            return False
        x0, y0, x1, y1 = prep["roi"]
        pad = np.zeros((th + 4, tw + 4, 4), np.uint8)      # a transparent border: smooth edges
        pad[2:-2, 2:-2] = tex_rgba
        c = prep["coeffs"]
        # the coefficients map onto the unpadded texture: shift by the border
        h = [c[0] + 2 * c[6], c[1] + 2 * c[7], c[2] + 2 * c[8], c[3] + 2 * c[6], c[4] + 2 * c[7],
             c[5] + 2 * c[8], c[6], c[7], c[8]]
        h = [v / h[8] for v in h]
        im = Image.fromarray(pad, "RGBA").transform((x1 - x0, y1 - y0), Image.PERSPECTIVE, h[:8], Image.BILINEAR)
        warped = np.asarray(im, dtype=np.float32)
        vis = prep["vis"]
        if spinning and prep["wheel"] is not None:
            vis = np.maximum(vis, prep["wheel"].astype(np.float32))
        a = warped[..., 3:4] / 255.0 * vis[..., None]
        roi = img[y0:y1, x0:x1].astype(np.float32)
        if additive:
            out = roi + warped[..., :3] * a * gain
        else:
            out = roi * (1 - a) + warped[..., :3] * gain * a
        img[y0:y1, x0:x1] = np.clip(out, 0, 255).astype(np.uint8)
        return True
