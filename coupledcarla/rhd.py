"""Right-hand-drive cabins on left-hand-traffic maps (`displays.rhd: mirror`), made from CARLA's left-hand-drive cars.

Every CARLA car (0.9.16 and 1.0) has its steering wheel on the left (data/driver_eyes.json: all 34 measured wheels at
y < 0; CARLA 1.0's catalogue holds the same models). On a left-hand-traffic map (`map.lefthand`) the driver sits on
the right, so the seat's camera sees the passenger's side of a left-hand-drive dashboard: no wheel in front.

Mirroring the whole image gives a right-hand-drive cabin but a mirrored street. So the station renders two cameras on
the same rigid mount: A at the right-hand seat (the street as it is) and B at the mirror image of that seat, the
left-hand-drive driver's own seat. B mirrored left to right looks exactly like a right-hand-drive car seen from A,
because a car's shell (body, glass, pillars, roof, hood) is symmetric and only the interior is not. Each pixel then
takes the mirrored cabin from B, except where B looks out through the glass (or at the body's outer skin through it,
like the hood): there it takes the street from A. Which pixels those are is fixed for a rigid mount, so it is found
once, at the start: a hit point of B's depth image is "skin" when a depth camera outside the car sees that very point
(25 views round the car); CARLA's depth and semantic images both contain the windows, so neither can tell glass from
the dashboard on its own. Where B looks out but A sees its own cabin (the left-hand wheel's rim, seen from the right
seat), the street behind is unknown: those few pixels blend the nearest street pixels of the same row.

The steering sense and the in-car displays need nothing else: the street is A's (not mirrored), and cockpit.CabinView
already mirrors the wheel and the panels for left-hand traffic; it is given B's depth image, mirrored, so the wheel's
pixels that WheelSpin turns and the panels' occlusion match the mirrored cabin.

Costs and limits: a second full camera render per frame (the station's frame rate roughly halves on one GPU), ~3 ms
per 1280x720 frame for the composite; at the start 27 depth images. The cabin's light and shadows are mirrored too
(the sun through the left window lights the cabin's right side), mirrored lettering on the mesh's own dashboard
textures reads backwards where no display covers it, and a hand or a wiper crossing a window edge is not cut out.

Pure numpy here (and cockpit's maths); RhdCabin at the end wires it to a station's cameras.
"""
import math

SKIN_TOL = 0.03          # m: a cabin hit point this close to what an outside camera sees is on the car's outer skin
BOX_MARGIN = 0.08        # m: an outside camera's hit within the bounding box plus this is the car itself
EXT_RADIUS = 4.0         # m from the bounding box's centre to the outside cameras (beyond it a wall would block them)
EXT_FOV = 100.0
EXT_SIZE = (640, 360)
FEATHER = 3              # px: the seam between street and cabin is blended over this width
PLAN_STRIDE = 2          # the plan judges every 2nd pixel in each direction (4x faster; the seam is feathered anyway)


def basis(pitch, yaw=0.0):
    """(forward, right, up) of a camera with this pitch and yaw (deg) in the vehicle frame (UE: x forward, y right,
    z up; + pitch looks up, + yaw turns right)."""
    p, y = math.radians(pitch), math.radians(yaw)
    fwd = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p))
    right = (-math.sin(y), math.cos(y), 0.0)
    up = (-math.sin(p) * math.cos(y), -math.sin(p) * math.sin(y), math.cos(p))
    return fwd, right, up


def focal(size, fov):
    return size[0] / (2.0 * math.tan(math.radians(fov) / 2.0))


def hit_points(depth, cam, fov):
    """Vehicle-frame points (H x W x 3) of a depth image (metres along the view axis) from cam = (x, y, z, pitch,
    yaw)."""
    import numpy as np
    d = np.asarray(depth, dtype=np.float64)
    h, w = d.shape
    f = focal((w, h), fov)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    r, q = (u - w / 2.0) / f, -(v - h / 2.0) / f
    fwd, right, up = basis(cam[3], cam[4] if len(cam) > 4 else 0.0)
    return np.stack([cam[i] + d * (fwd[i] + r * right[i] + q * up[i]) for i in range(3)], axis=-1)


def project(points, cam, size, fov):
    """Pixel (u, v) and depth along the view axis of vehicle-frame points (... x 3) seen from cam."""
    import numpy as np
    fwd, right, up = basis(cam[3], cam[4] if len(cam) > 4 else 0.0)
    rel = points - np.asarray(cam[:3], dtype=np.float64)
    x = rel @ np.asarray(fwd)
    y = rel @ np.asarray(right)
    z = rel @ np.asarray(up)
    f = focal(size, fov)
    with np.errstate(divide="ignore", invalid="ignore"):
        return size[0] / 2.0 + f * y / x, size[1] / 2.0 - f * z / x, x


def exterior_cams(extent, centre=(0.0, 0.0, 0.0), radius=EXT_RADIUS):
    """Camera poses (x, y, z, pitch, yaw) round a car's bounding box, all looking at its centre: 12 at 15 deg up,
    12 at 50 deg up and one from above."""
    out = []
    for elev, n in ((15.0, 12), (50.0, 12), (89.0, 1)):
        for k in range(n):
            az = 360.0 * k / n
            x = centre[0] + radius * math.cos(math.radians(elev)) * math.cos(math.radians(az))
            y = centre[1] + radius * math.cos(math.radians(elev)) * math.sin(math.radians(az))
            z = centre[2] + radius * math.sin(math.radians(elev))
            out.append((x, y, z, -elev, math.degrees(math.atan2(centre[1] - y, centre[0] - x))))
    return out


def skin_mask(points, views, extent, centre=(0.0, 0.0, 0.0), tol=SKIN_TOL, margin=BOX_MARGIN):
    """Which cabin hit points (H x W x 3) lie on the car's outer skin (glass, or the body seen through it): an outside
    view sees the point itself, within tol. views: [(cam, depth image, fov)]. A view whose own hit in front of the
    point lies outside the car's box (plus margin: a parked car, a wall) says nothing; a point no view can judge
    counts as skin (the street from A: the view without this feature)."""
    import numpy as np
    seen = np.zeros(points.shape[:2], bool)
    hidden = np.zeros(points.shape[:2], bool)
    ext = np.asarray(extent, dtype=np.float64) + margin
    c = np.asarray(centre, dtype=np.float64)
    for cam, depth, fov in views:
        d = np.asarray(depth, dtype=np.float64)
        h, w = d.shape
        u, v, x = project(points, cam, (w, h), fov)
        ok = (x > 0.1) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        ui = np.clip(np.nan_to_num(u, nan=0.0).astype(np.int64), 0, w - 1)
        vi = np.clip(np.nan_to_num(v, nan=0.0).astype(np.int64), 0, h - 1)
        dv = d[vi, ui]
        seen |= ok & (dv >= x - tol)
        front = ok & (dv < x - tol)
        if front.any():      # what stands in front: the car itself (then the point is inside) or something else
            fu, fv = ui[front], vi[front]
            f = focal((w, h), fov)
            fwd, right, up = basis(cam[3], cam[4] if len(cam) > 4 else 0.0)
            dd = d[fv, fu]
            r, s = (fu + 0.5 - w / 2.0) / f, -(fv + 0.5 - h / 2.0) / f
            q = np.stack([cam[i] + dd * (fwd[i] + r * right[i] + s * up[i]) for i in range(3)], axis=-1)
            inside = np.all(np.abs(q - c) <= ext, axis=-1)
            hid = np.zeros_like(front)
            hid[front] = inside
            hidden |= hid
    return seen | ~hidden


def tidy(mask, k=5):
    """A k x k majority vote over a boolean mask: removes specks (single texels of the dashboard judged skin)."""
    import numpy as np
    from .cockpit import _box_blur
    return _box_blur(np.asarray(mask, np.float32)[..., None], k)[..., 0] > 0.5


def mirror(img):
    """An image (H x W [x C]) mirrored left to right."""
    return img[:, ::-1]


class Plan:
    """The fixed per-pixel recipe of one mount: weight of A's street (wa, feathered), and the pixels whose street is
    unknown (B looks out, A sees its own cabin) with the row neighbours that fill them."""

    def __init__(self, skin_a, skin_b_mirrored, feather=FEATHER):
        import numpy as np
        from .cockpit import WheelSpin, _box_blur
        street = tidy(skin_b_mirrored)
        ghost = street & ~tidy(skin_a)
        wa = street.astype(np.float32)[..., None]
        if feather > 1:
            wa = _box_blur(wa, feather if feather % 2 else feather + 1)
        wa = np.where(street[..., None], wa, 0.0)[..., 0]           # never A on the cabin's own pixels
        self.shape = street.shape
        self.pure_a = np.repeat((wa >= 0.999)[..., None], 3, axis=2)
        self.band = np.flatnonzero((wa > 0.001) & (wa < 0.999))
        self.band_w = wa.ravel()[self.band][:, None].astype(np.float32)
        self.shares = {"cabin": float((~street).mean()), "street": float((street & ~ghost).mean()),
                       "filled": float(ghost.mean())}
        self.fill = None
        if ghost.any():
            ys, xs = np.nonzero(ghost)
            y0, y1 = max(0, ys.min() - 2), min(ghost.shape[0], ys.max() + 3)
            x0, x1 = max(0, xs.min() - 8), min(ghost.shape[1], xs.max() + 9)
            m = ghost[y0:y1, x0:x1]
            li, ri, w = WheelSpin._row_fill(m)
            on = np.flatnonzero(m.ravel())             # only the unknown pixels are worked out per frame
            self.fill = ((y0, y1, x0, x1), on, li[on], ri[on], w[on].astype(np.float32)[:, None])

    def apply(self, a, b_mirrored):
        """The composite (H x W x 3 uint8) of A's frame and B's frame mirrored."""
        import numpy as np
        out = np.array(b_mirrored, dtype=np.uint8, copy=True, order="C")
        np.copyto(out, a, where=self.pure_a)
        flat, fa = out.reshape(-1, out.shape[-1]), np.ascontiguousarray(a).reshape(-1, out.shape[-1])
        if self.band.size:
            mix = fa[self.band] * self.band_w + flat[self.band] * (1.0 - self.band_w)
            flat[self.band] = np.clip(mix + 0.5, 0, 255).astype(np.uint8)
        if self.fill is not None:
            (y0, y1, x0, x1), on, li, ri, w = self.fill
            roi = np.ascontiguousarray(out[y0:y1, x0:x1])
            fl = roi.reshape(-1, roi.shape[2])
            fl[on] = np.clip(fl[li] * (1.0 - w) + fl[ri] * w + 0.5, 0, 255).astype(np.uint8)
            out[y0:y1, x0:x1] = roi
        return out


def plan_for(depth_a, depth_b, eye, pitch, fov, views, extent, centre=(0.0, 0.0, 0.0)):
    """The Plan of a mount: eye (x, y, z) is A's (the right-hand seat); B sits at (x, -y, z). depth_a, depth_b: the
    two seats' depth images (H x W, metres); views: the outside depth images [(cam, depth, fov)]."""
    import numpy as np
    full = np.asarray(depth_a).shape
    k = PLAN_STRIDE

    def skin(depth, y):     # judged on every k-th pixel (a k x k block shares its centre's answer), then upsampled
        d = np.asarray(depth, dtype=np.float32)
        h, w = full[0] // k, full[1] // k
        small = d[k // 2:h * k:k, k // 2:w * k:k]
        # the sampled pixels' own rays: the small image is the same view at 1/k of the size (same field of view)
        m = skin_mask(hit_points(small, (eye[0], y, eye[2], pitch, 0.0), fov), views, extent, centre)
        big = np.repeat(np.repeat(m, k, 0), k, 1)
        out = np.ones(full, bool)
        out[:big.shape[0], :big.shape[1]] = big
        return out
    return Plan(skin(depth_a, eye[1]), mirror(skin(depth_b, -eye[1])))


# -- on a station ------------------------------------------------------------------------------------------------

class RhdCabin:
    """The second camera (B) of a station's cabin view and the composite. start() spawns B with the station
    camera's blueprint at the mirrored mount, plus the depth cameras for the plan (destroyed once they have their
    image; ready() makes the plan). put() hands in A's frame (at `size`) with a payload; on_ready(payload, rgb) is
    called with the composite once B's image of that frame is in (at once, with A's frame unchanged, while there is
    no plan yet). The two cameras' callbacks run on different threads."""

    KEEP = 20            # frames of B (or A waiting for B) kept for pairing

    def __init__(self, size, fov, eye, pitch, log=print, on_ready=None):
        import threading
        self.size, self.fov, self.eye, self.pitch, self.log = tuple(size), float(fov), tuple(eye), float(pitch), log
        self.on_ready = on_ready
        self.plan = None
        self.cabin_depth = None        # B's depth mirrored: the right-hand-drive cabin's (cockpit.CabinView)
        self._b, self._pending = {}, {}
        self._lock = threading.Lock()
        self._depth = {}
        self._cams = []
        self.camera_b = None

    def start(self, world, actor, rgb_bp):
        import carla
        x, y, z = self.eye
        mount_b = carla.Transform(carla.Location(x=x, y=-y, z=z), carla.Rotation(pitch=self.pitch))
        self.camera_b = world.spawn_actor(rgb_bp, mount_b, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)
        self.camera_b.listen(self._on_b)
        bb = actor.bounding_box
        self.extent = (bb.extent.x, bb.extent.y, bb.extent.z)
        self.centre = (bb.location.x, bb.location.y, bb.location.z)
        lib = world.get_blueprint_library()

        def depth_cam(key, cam, size, fov):
            bp = lib.find("sensor.camera.depth")
            bp.set_attribute("image_size_x", str(size[0]))
            bp.set_attribute("image_size_y", str(size[1]))
            bp.set_attribute("fov", str(fov))
            tf = carla.Transform(carla.Location(x=cam[0], y=cam[1], z=cam[2]),
                                 carla.Rotation(pitch=cam[3], yaw=cam[4] if len(cam) > 4 else 0.0))
            c = world.spawn_actor(bp, tf, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)
            c.listen(lambda img, k=key: self._on_depth(k, img))
            self._cams.append(c)

        depth_cam("A", (x, y, z, self.pitch), self.size, self.fov)
        depth_cam("B", (x, -y, z, self.pitch), self.size, self.fov)
        self.ext_cams = exterior_cams(self.extent, self.centre)
        for i, cam in enumerate(self.ext_cams):
            depth_cam(i, cam, EXT_SIZE, EXT_FOV)
        self.log(f"rhd: right-hand-drive cabin from the mirrored seat ({len(self._cams)} depth images for the plan)")

    def _on_depth(self, key, image):
        if key in self._depth:
            return
        import numpy as np
        a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4).astype(np.float32)
        self._depth[key] = (a[..., 2] + a[..., 1] * 256.0 + a[..., 0] * 65536.0) / (256 ** 3 - 1) * 1000.0

    def ready(self):
        """Makes the plan once every depth image is in (call from the station's tick); True when there is one."""
        if self.plan is not None:
            return True
        if not self._cams or len(self._depth) < len(self._cams):
            return False
        for c in self._cams:
            try:
                c.stop()
                c.destroy()
            except RuntimeError:
                pass
        self._cams = []
        views = [(cam, self._depth[i], EXT_FOV) for i, cam in enumerate(self.ext_cams)]
        plan = plan_for(self._depth["A"], self._depth["B"], self.eye, self.pitch, self.fov, views,
                        self.extent, self.centre)
        self.cabin_depth = mirror(self._depth["B"]).copy()
        self.plan = plan
        s = plan.shares
        self.log(f"rhd: plan ready: cabin {100 * s['cabin']:.0f}% of the view, street {100 * s['street']:.0f}%, "
                 f"filled {100 * s['filled']:.1f}%")
        self._depth = {}
        return True

    def _emit(self, rgb_a, payload, b_image):
        from . import look
        b = look.downscale(look.to_rgb(b_image), self.size)
        if self.on_ready is not None:
            self.on_ready(payload, self.plan.apply(rgb_a, mirror(b)))

    def _on_b(self, image):
        with self._lock:
            self._b[image.frame] = image
            for f in [f for f in self._b if f < image.frame - self.KEEP]:
                del self._b[f]
            pend = self._pending.pop(image.frame, None)
        if pend is not None and self.plan is not None:
            self._emit(pend[0], pend[1], image)

    def put(self, frame, rgb_a, payload=None):
        """A's frame (H x W x 3 at self.size): its composite goes to on_ready now or when B's image arrives."""
        if self.plan is None:
            if self.on_ready is not None:
                self.on_ready(payload, rgb_a)
            return
        with self._lock:
            b = self._b.pop(frame, None)
            late = []
            if b is None:
                self._pending[frame] = (rgb_a, payload)
                late = [self._pending.pop(f) for f in [f for f in self._pending if f < frame - self.KEEP]]
        for rgb, pay in late:          # B never came for these: shown as they are
            if self.on_ready is not None:
                self.on_ready(pay, rgb)
        if b is not None:
            self._emit(rgb_a, payload, b)

    def stop(self):
        for c in self._cams + ([self.camera_b] if self.camera_b is not None else []):
            try:
                c.stop()
                c.destroy()
            except RuntimeError:
                pass
        self._cams, self.camera_b = [], None
