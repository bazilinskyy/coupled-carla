"""Measure where the driver's and front passenger's eyes sit in each CARLA car, on a running server, into
coupledcarla/data/driver_eyes.json (read by coupledcarla/cockpit.py for the driver and AV-passenger views).

    python tools/cabin_measure.py --port 2500                       # every 4-wheeled blueprint of the server
    python tools/cabin_measure.py --port 2500 --only vehicle.lincoln.mkz_2020 vehicle.tesla.model3
    python tools/cabin_measure.py --port 2500 --sheet scratch/eyes  # also a test frame per car from the eye found
    python tools/cabin_measure.py --port 2500 --check scratch/eyes  # only the test frames, from the file's eyes

Method (per car, physics off, at a spawn point of the loaded map):
1. Point clouds of the driver's side: from a row of seat positions along the cabin (every 0.3 m, on the driver's
   side at 80 % of the car's height; vans and buses sit further forward than cars) two depth cameras look forward
   and down; the points within 1.2 m are unprojected into the car's frame, cut to the driver's side (y < -0.15 m)
   and thinned to one point per 8 mm cube.
2. The steering wheel: per seat position, the disc standing out towards the seat (find_disc: per 2 cm cell across
   y and z the nearest surface ahead; cells 6 cm nearer than the dashboard behind them form blobs; a wheel-sized
   blob, 0.22-0.5 m wide, gives the plane, centre and radius) and the best 3D circle (find_wheel: RANSAC on three
   points and a least-squares fit, radius 0.125-0.24 m, axis pointing back and 3-64 deg up, rim seen in at least
   10 of 12 sectors); the wheel is where most of these fits from all seat positions agree within 6 cm (medians).
   Every fit's centre lies at least 0.2 m left of the centre line, above 40 % of the car's height and, in a car,
   behind the front axle.
3. The seat cushion is the surface a narrow depth camera sees straight down 0.55 m behind the wheel centre (at
   least 0.25 m below the wheel's centre, else unknown), the roof liner the surface it sees straight up 0.65 m
   behind it.
4. The eye (SAE J941/J1100 in spirit: the eyellipse centroid ~0.635 m above the seat's H-point, which sits ~0.10 m
   above the cushion; ~0.65 m behind the wheel centre): x = wheel x - 0.65, y = wheel y, z = H-point + 0.635,
   kept at least 0.12 m above the wheel's top and 0.20 m under the roof liner (the head's top and headroom). The
   front passenger's eye is the driver's mirrored in y (same seat height). Right-hand traffic maps keep these;
   left-hand ones mirror y (cockpit.eye_for).
A car whose wheel is not found keeps the size-scaled default of cockpit.default_eye and is listed as such.
5. --glass: the windscreen's tint, glass_ev: how many EV darker the street looks from the eye than from above the
   roof (median linear luminance of the same view's upper middle); cockpit.cabin_ev_bias brightens the view by it.
"""
import argparse
import json
import math
import queue
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from coupledcarla import cockpit  # noqa: E402

OUT = ROOT / "coupledcarla" / "data" / "driver_eyes.json"


# --- pure geometry (no CARLA) ---------------------------------------------------------------------------------------

def unproject(depth, fov, cam):
    """depth (H x W, metres along the view axis) of a camera at cam = (x, y, z, pitch, yaw) in the car's frame
    (UE axes: x forward, y right, z up; roll 0) -> H x W x 3 points in the car's frame."""
    h, w = depth.shape
    f = w / (2 * math.tan(math.radians(fov) / 2))
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    fx, fy, fz = depth, (u - w / 2) / f * depth, -(v - h / 2) / f * depth
    x, y, z, p, yw = cam
    cp, sp = math.cos(math.radians(p)), math.sin(math.radians(p))
    cy, sy = math.cos(math.radians(yw)), math.sin(math.radians(yw))
    x1, z1 = fx * cp - fz * sp, fx * sp + fz * cp
    return np.stack([x1 * cy - fy * sy + x, x1 * sy + fy * cy + y, z1 + z], -1)


def voxel_thin(p, cell=0.01):
    """One point per cell (the first), N x 3 -> M x 3."""
    if len(p) == 0:
        return p
    key = np.floor(p / cell).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return p[np.sort(idx)]


def circle3(p1, p2, p3):
    """The circle through three 3D points: (centre, unit normal, radius), or None when they are collinear."""
    a, b = p1 - p3, p2 - p3
    axb = np.cross(a, b)
    n2 = float(axb @ axb)
    if n2 < 1e-12:
        return None
    c = p3 + np.cross((a @ a) * b - (b @ b) * a, axb) / (2 * n2)
    return c, axb / math.sqrt(n2), float(np.linalg.norm(c - p3))


def ring_distance(pts, c, n, r):
    d = pts - c
    h = d @ n
    rad = np.linalg.norm(d - np.outer(h, n), axis=1)
    return np.sqrt(h * h + (rad - r) ** 2)


def coverage(pts, c, n, bins=12):
    """How many of `bins` equal sectors round the axis n hold at least 3 of the points."""
    u = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(n, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    d = pts - c
    ang = np.arctan2(d @ v, d @ u)
    hist = np.histogram(ang, bins=bins, range=(-math.pi, math.pi))[0]
    return int((hist >= 3).sum())


def fit_ring(pts, c, n):
    """Least squares: the plane through the points (SVD) and the circle in it (Kasa fit)."""
    m = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - m, full_matrices=False)
    nn = vt[2] if vt[2] @ n >= 0 else -vt[2]
    u = vt[0]
    v = np.cross(nn, u)
    q = np.stack([(pts - m) @ u, (pts - m) @ v], 1)
    A = np.column_stack([2 * q, np.ones(len(q))])
    sol = np.linalg.lstsq(A, (q ** 2).sum(1), rcond=None)[0]
    cc = m + sol[0] * u + sol[1] * v
    r = math.sqrt(max(1e-6, sol[2] + sol[0] ** 2 + sol[1] ** 2))
    return cc, nn, r


def find_wheel(pts, rmin=0.125, rmax=0.24, tol=0.012, iters=8000, seed=1, ymax=-0.2):
    """The steering wheel in a point cloud of the driver's half of the cabin: (centre, axis pointing back at the
    driver, radius, inliers, sectors) or None."""
    rng = np.random.default_rng(seed)
    if len(pts) < 50:
        return None
    best = None
    for _ in range(iters):
        i = rng.integers(len(pts))
        near = np.flatnonzero(np.abs(pts - pts[i]).max(1) < 2 * rmax)
        if len(near) < 30:
            continue
        j, k = rng.choice(near, 2, replace=False)
        cir = circle3(pts[i], pts[j], pts[k])
        if cir is None:
            continue
        c, n, r = cir
        if not rmin <= r <= rmax or c[1] > ymax:      # on the driver's (left) side, off the centre console
            continue
        if n[2] < 0:
            n = -n
        if n[0] > -0.3 or not 0.05 < n[2] < 0.9 or abs(n[1]) > 0.3:   # faces the driver: back and up, 3-64 deg
            continue
        sub = pts[near]
        inl = sub[ring_distance(sub, c, n, r) < tol]
        if best is not None and len(inl) <= best[0]:
            continue
        if coverage(inl, c, n) < 10:
            continue
        best = (len(inl), c, n, r)
    if best is None:
        return None
    _, c, n, r = best
    for _ in range(3):
        inl = pts[ring_distance(pts, c, n, r) < tol * 1.5]
        c, n, r = fit_ring(inl, c, n)
        if n[2] < 0:
            n = -n
    return c, n, r, len(inl), coverage(inl, c, n)


def find_disc(pts, cam, cell=0.02):
    """The steering wheel as the surface standing out towards a driver at cam (x, y, z): the older meshes' wheels
    are near-solid discs that a ring fit misses. Per 2 cm cell across (y, z) the nearest point ahead; cells more than
    6 cm nearer than the dashboard behind them in their row form blobs; the largest wheel-sized blob (0.22-0.5 m wide)
    gives the plane (SVD), the centre (middle of its extents) and the radius (half its width). Same result tuple as
    find_wheel, or None."""
    x0, _y0, z0 = cam
    box = pts[(pts[:, 0] > x0 + 0.3) & (pts[:, 0] < x0 + 1.1) & (pts[:, 1] > -0.8) & (pts[:, 1] < -0.15) &
              (pts[:, 2] > z0 - 0.8) & (pts[:, 2] < z0 - 0.02)]
    if len(box) < 100:
        return None
    iy = ((box[:, 1] + 0.8) / cell).astype(int)
    iz = ((box[:, 2] - (z0 - 0.8)) / cell).astype(int)
    ny, nz = iy.max() + 1, iz.max() + 1
    near = np.full((nz, ny), np.inf)
    np.minimum.at(near, (iz, iy), box[:, 0])
    ref = np.array([np.percentile(r[np.isfinite(r)], 80) if np.isfinite(r).sum() > 3 else np.inf for r in near])
    blob = near < ref[:, None] - 0.06
    seen = np.zeros_like(blob)
    best = None
    for start in zip(*np.nonzero(blob)):
        if seen[start]:
            continue
        stack, comp = [start], []
        seen[start] = True
        while stack:
            a, b = stack.pop()
            comp.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (a + da, b + db)
                if 0 <= q[0] < nz and 0 <= q[1] < ny and blob[q] and not seen[q]:
                    seen[q] = True
                    stack.append(q)
        comp = np.array(comp)
        wy = (comp[:, 1].max() - comp[:, 1].min() + 1) * cell
        wz = (comp[:, 0].max() - comp[:, 0].min() + 1) * cell
        if 0.22 <= wy <= 0.5 and 0.12 <= wz <= 0.55 and (best is None or len(comp) > len(best)):
            best = comp
    if best is None:
        return None
    sel = np.zeros_like(blob)
    sel[best[:, 0], best[:, 1]] = True
    wp = box[sel[iz, iy] & (box[:, 0] < near[iz, iy] + 0.03)]
    m = wp.mean(0)
    _, _, vt = np.linalg.svd(wp - m, full_matrices=False)
    n = vt[2] if vt[2][0] < 0 else -vt[2]
    if n[2] < 0:
        n = -n
    right = np.cross(n, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, n)
    a, b = (wp - m) @ right, (wp - m) @ up
    r = (a.max() - a.min()) / 2
    c = m + right * (a.max() + a.min()) / 2 + up * (b.max() + b.min()) / 2
    if not 0.11 <= r <= 0.25 or n[0] > -0.3 or abs(n[1]) > 0.35:
        return None
    return c, n, float(r), len(wp), coverage(wp[np.abs(np.hypot(a, b)) > 0], c, n)


def eye_from(wheel_c, wheel_r, axis, cushion_z, roof_z):
    """The driver's eye (x, y, z) from the wheel, the cushion's height and the roof liner's (None: unknown)."""
    top = wheel_c[2] + wheel_r * math.sqrt(max(0.0, 1 - axis[2] ** 2))     # the rim's highest point
    x = wheel_c[0] - 0.65
    z = (cushion_z + 0.10 + 0.635) if cushion_z is not None else top + 0.22
    z = max(z, top + 0.12)
    if roof_z is not None:
        z = min(z, roof_z - 0.20)       # the top of the head ~0.12 m above the eyes, and some headroom
    return [round(float(x), 3), round(float(wheel_c[1]), 3), round(float(z), 3)]


# --- on a running server ------------------------------------------------------------------------------------------

class Rig:
    def __init__(self, host, port):
        import carla
        self.carla = carla
        self.client = carla.Client(host, port)
        self.client.set_timeout(120)
        self.world = self.client.get_world()
        self.orig = self.world.get_settings()
        s = self.world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
        self.world.apply_settings(s)
        self.lib = self.world.get_blueprint_library()
        self.car = None

    def close(self):
        if self.car is not None:
            self.car.destroy()
        self.world.apply_settings(self.orig)

    def spawn(self, bpid):
        if self.car is not None:
            self.car.destroy()
        bp = self.lib.find(bpid)
        if bp.has_attribute("color"):
            bp.set_attribute("color", "120,120,130")
        for sp in self.world.get_map().get_spawn_points():      # the first free one
            self.car = self.world.try_spawn_actor(bp, sp)
            if self.car is not None:
                break
        else:
            raise RuntimeError(f"no free spawn point for {bpid}")
        self.car.set_simulate_physics(False)
        for _ in range(3):
            self.world.tick()
        return self.car

    def shoot(self, kind, cam, size=(384, 384), fov=100.0, attrs=None, ticks=2):
        c = self.carla
        bp = self.lib.find("sensor.camera." + kind)
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("fov", str(fov))
        for k, v in (attrs or {}).items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        x, y, z, pitch, yaw = cam
        s = self.world.spawn_actor(bp, c.Transform(c.Location(x=x, y=y, z=z), c.Rotation(pitch=pitch, yaw=yaw)),
                                   attach_to=self.car)
        q = queue.Queue()
        s.listen(q.put)
        img = None
        for _ in range(ticks):
            self.world.tick()
            img = q.get(timeout=30)
        s.stop()
        s.destroy()
        a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
        if kind == "depth":
            a = a.astype(np.float64)
            return (a[..., 2] + a[..., 1] * 256 + a[..., 0] * 65536) / (256 ** 3 - 1) * 1000.0
        return a[..., 2::-1].copy()


def near_cloud(rig, cam, top, ey, reach=1.2):
    """Points within `reach` m of a forward-looking camera (two pitches), ahead of it, in the cabin's left half."""
    pts = []
    for pitch in (-42.0, -22.0):
        c = (cam[0], cam[1], cam[2], pitch, 0.0)
        dep = rig.shoot("depth", c)
        p = unproject(dep, 100.0, c).reshape(-1, 3)
        d = np.linalg.norm(p - np.array(cam), axis=1)
        pts.append(p[(d < reach) & (dep.reshape(-1) < 5)])
    p = np.concatenate(pts)
    keep = (p[:, 0] > cam[0] + 0.15) & (p[:, 2] > 0.3 * top) & (p[:, 2] < cam[2] + 0.1) & (p[:, 1] < -0.15) & \
        (p[:, 1] > -0.85 * ey)
    return voxel_thin(p[keep], 0.008)


def probe(rig, cam, pitch, fov=40.0):
    """Median depth of the middle of a narrow depth camera's view (None when nothing within 2 m)."""
    dep = rig.shoot("depth", (cam[0], cam[1], cam[2], pitch, 0.0), size=(64, 64), fov=fov)
    m = float(np.median(dep[24:40, 24:40]))
    return m if m < 2.0 else None


def measure(rig, bpid, seed=None):
    """seed: (x, y, z) of a hand-placed eye: the wheel is looked for from there only (cars whose own search went
    wrong: a seat back or the dashboard of a van taken for the wheel)."""
    car = rig.spawn(bpid)
    bb = car.bounding_box
    ext = (bb.extent.x, bb.extent.y, bb.extent.z)
    bx, (ex, ey, ez) = bb.location.x, ext
    top = 2 * ez
    t0 = time.time()
    out = {"extent": [round(e, 3) for e in ext], "bb_x": round(bx, 3)}
    # look for the wheel from a row of seats along the cabin (vans and buses sit further forward than cars): the
    # disc standing out towards the seat, seen alike from at least two seats; else the best ring
    discs, rings = [], []
    seats = [(float(x0), -0.4 * ey, 0.8 * top) for x0 in np.arange(bx - 0.35 * ex, bx + ex - 0.4, 0.3)]
    if seed is not None:
        seats = [(seed[0] + dx, seed[1], seed[2]) for dx in (-0.15, 0.0, 0.15)]
    for cam in seats:
        x0 = cam[0]
        pts = near_cloud(rig, cam, top, ey)
        for found, into in ((find_disc(pts, cam), discs), (find_wheel(pts, iters=2500), rings)):
            if found is None or not 0.3 < found[0][0] - x0 < 1.1 or found[0][2] < 0.4 * top or found[0][1] > -0.2:
                continue
            if car.attributes.get("base_type", "car") in ("car", "") and found[0][0] > bx + 0.45 * ex:
                continue                  # a car's wheel is behind the front axle (not a tyre seen outside)
            into.append(found)
    # the wheel where most of the fits (discs and rings, from all seat positions) agree within 6 cm; one lone fit
    # only when nothing agrees (a disc before a ring)
    cands = [(d, "disc") for d in discs] + [(w, "ring") for w in rings]
    best = None
    if cands:
        groups = [[c for c in cands if np.linalg.norm(c[0][0] - e[0][0]) < 0.06] for e in cands]
        g = max(groups, key=lambda g: (len(g), sum(c[1] == "disc" for c in g), sum(c[0][3] for c in g)))
        fits = [c[0] for c in g]
        n = np.mean([f[1] for f in fits], axis=0)
        best = (len(g), (np.median([f[0] for f in fits], axis=0), n / np.linalg.norm(n),
                         float(np.median([f[2] for f in fits])), sum(f[3] for f in fits), min(f[4] for f in fits)),
                "+".join(sorted(set(c[1] for c in g))) + f" x{len(g)}")
    if best is None:
        d = cockpit.default_eye(ext, bx, car.attributes.get("base_type", "car"))
        out.update({"eye": d, "passenger": [d[0], -d[1], d[2]], "method": "default (no wheel found)"})
        print(f"{bpid}: no steering wheel found; default eye {d}", flush=True)
        return out
    c, n, r, ninl, sectors = best[1]
    out["wheel_fit"] = best[2]
    dn = probe(rig, (c[0] - 0.55, c[1], c[2] + 0.15), -90.0)
    cushion = c[2] + 0.15 - dn if dn is not None and dn < 0.9 else None
    if cushion is not None and cushion > c[2] - 0.25:       # the seat back or a bolster, not the cushion
        cushion = None
    up = probe(rig, (c[0] - 0.65, c[1], c[2] + 0.2), 90.0)
    roof = c[2] + 0.2 + up if up is not None else None
    eye = eye_from(c, r, n, cushion, roof)
    if seed is not None:
        out["seed"] = [round(float(v), 3) for v in seed]
    out.update({"eye": eye, "passenger": [eye[0], -eye[1], eye[2]],
                "wheel": {"centre": [round(float(v), 3) for v in c], "axis": [round(float(v), 3) for v in n],
                          "radius": round(float(r), 3)},
                "cushion_z": round(cushion, 3) if cushion is not None else None,
                "roof_z": round(roof, 3) if roof is not None else None, "method": "measured"})
    print(f"{bpid}: wheel at {out['wheel']['centre']} r {r:.3f} axis {out['wheel']['axis']} ({ninl} points, "
          f"{sectors}/12 sectors); cushion {out['cushion_z']}, roof {out['roof_z']} -> eye {eye} "
          f"({time.time() - t0:.0f} s)", flush=True)
    return out


def glass_ev(rig, bpid, entry):
    """How many EV darker the street looks through the car's windscreen: the median linear luminance of the upper
    middle of a view from the driver's eye against the same view from 0.3 m above the roof (fixed exposure)."""
    rig.spawn(bpid)
    x, y, z = entry.get("eye_override") or entry["eye"]
    attrs = {"exposure_mode": "manual", "shutter_speed": "60", "fstop": "1.4", "iso": "100",
             "motion_blur_intensity": "0", "lens_flare_intensity": "0", "bloom_intensity": "0"}
    lum = []
    for zz in (z, 2 * entry["extent"][2] + 0.3):
        a = rig.shoot("rgb", (x, y, zz, 0.0, 0.0), size=(320, 180), fov=60.0, attrs=attrs, ticks=6)
        roi = a[55:88, 100:160].astype(np.float64) / 255.0      # just above the horizon, left of the mirror
        lin = (0.2126 * roi[..., 0] + 0.7152 * roi[..., 1] + 0.0722 * roi[..., 2]) ** 2.2
        lum.append(float(np.median(lin)))
    if min(lum) <= 1e-6:
        return None
    return round(math.log2(lum[1] / lum[0]), 2)


def sheet(rig, bpid, entry, out_dir, fov=90.0, size=(960, 540)):
    """A test frame from the driver's eye (and the passenger's), for checking by eye."""
    from PIL import Image, ImageDraw
    rig.spawn(bpid)
    ims = []
    for key in ("eye", "passenger"):
        x, y, z = entry.get("eye_override") if key == "eye" and entry.get("eye_override") else entry[key]
        pitch = entry.get("pitch", cockpit.DEFAULT_PITCH)
        ims.append(Image.fromarray(rig.shoot("rgb", (x, y, z, pitch, 0.0), size=size, fov=fov, ticks=6)))
    im = Image.new("RGB", (size[0] * 2, size[1]))
    for i, a in enumerate(ims):
        im.paste(a, (i * size[0], 0))
    ImageDraw.Draw(im).text((8, 6), f"{bpid}  eye {entry['eye']}  {entry.get('method')}", fill=(255, 255, 0))
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    im.save(Path(out_dir) / f"{bpid}.jpg", quality=85)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", nargs="*", help="these blueprints only")
    ap.add_argument("--sheet", help="also save a test frame per car into this folder")
    ap.add_argument("--check", help="only the test frames (from the file's eyes) into this folder")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--resume", action="store_true", help="skip the cars the file already has")
    ap.add_argument("--seed", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="look for the wheel only from this hand-placed eye (with --only one car)")
    ap.add_argument("--recompute", action="store_true", help="only the eyes again from the file's measurements")
    ap.add_argument("--glass", action="store_true", help="only the windscreens' tint (glass_ev) of the file's cars")
    args = ap.parse_args()
    if args.recompute:            # no server: the eye rule applied again to the stored wheel, cushion and roof
        data = json.loads(Path(args.out).read_text())
        for table in (v for k, v in data.items() if isinstance(v, dict) and not k.startswith("_")):
            for e in table.values():
                if e.get("wheel"):
                    w = e["wheel"]
                    e["eye"] = eye_from(np.array(w["centre"]), w["radius"], np.array(w["axis"]), e.get("cushion_z"),
                                        e.get("roof_z"))
                    e["passenger"] = [e["eye"][0], -e["eye"][1], e["eye"][2]]
        Path(args.out).write_text(cockpit.dump_eyes(data))
        return
    rig = Rig(args.host, args.port)
    major = "1.0" if rig.client.get_server_version().split(".")[0] not in ("0",) else "0.9"
    data = json.loads(Path(args.out).read_text()) if Path(args.out).exists() else {}
    table = data.setdefault(major, {})
    try:
        ids = args.only or sorted(b.id for b in rig.lib.filter("vehicle.*")
                                  if not b.has_attribute("number_of_wheels")
                                  or b.get_attribute("number_of_wheels").as_int() >= 4)
        for bpid in ids:
            if args.glass:
                if bpid in table:
                    table[bpid]["glass_ev"] = glass_ev(rig, bpid, table[bpid])
                    print(f"{bpid}: glass {table[bpid]['glass_ev']} EV", flush=True)
                continue
            if args.check:
                if bpid in table:
                    sheet(rig, bpid, table[bpid], args.check)
                continue
            if args.resume and table.get(bpid, {}).get("eye") and not args.seed:
                continue
            entry = measure(rig, bpid, args.seed and tuple(args.seed))
            keep = {k: table.get(bpid, {}).get(k) for k in cockpit.HAND_TUNED if table.get(bpid, {}).get(k)}
            table[bpid] = {**entry, **keep}
            Path(args.out).write_text(cockpit.dump_eyes(data))      # after every car: a crash keeps the rest
            if args.sheet:
                sheet(rig, bpid, table[bpid], args.sheet)
    finally:
        rig.close()
    if not args.check:      # (--glass included)
        Path(args.out).write_text(cockpit.dump_eyes(data))
        print(f"wrote {args.out}: {len(table)} cars for CARLA {major}")


if __name__ == "__main__":
    main()
