"""In-car views (cockpit.py, tools/cabin_measure.py): eye points per car and their mirroring, the projection and
warp maths, displays on the car's own panels, the turning wheel and the wheel finder (no CARLA)."""
import importlib.util
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

from coupledcarla import cockpit, incar

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("cabin_measure", ROOT / "tools" / "cabin_measure.py")
cm = importlib.util.module_from_spec(spec)
sys.modules["cabin_measure"] = cm
spec.loader.exec_module(cm)

WHEEL = {"centre": [0.78, -0.40, 0.81], "axis": [-0.96, 0.0, 0.27], "radius": 0.19}
DATA = {"0.9": {"vehicle.tesla.model3": {"eye": [0.13, -0.40, 1.27], "passenger": [0.13, 0.40, 1.27],
                                         "wheel": WHEEL, "method": "measured"}},
        "1.0": {"vehicle.lincoln.mkz": {"eye": [0.0, -0.37, 1.2], "pitch": -6.0}}}


def test_eye_lookup_mirroring_and_defaults():
    assert cockpit.eye_for("vehicle.tesla.model3", data=DATA) == (0.13, -0.40, 1.27, cockpit.DEFAULT_PITCH)
    assert cockpit.eye_for("vehicle.tesla.model3", "passenger", data=DATA)[:3] == (0.13, 0.40, 1.27)
    # left-hand traffic: the driver sits on the right, the passenger on the left
    assert cockpit.eye_for("vehicle.tesla.model3", lefthand=True, data=DATA)[:3] == (0.13, 0.40, 1.27)
    assert cockpit.eye_for("vehicle.tesla.model3", "passenger", lefthand=True, data=DATA)[1] == -0.40
    # a 1.0 blueprint found from a 0.9 server's lookup (the other table), with its own pitch
    assert cockpit.eye_for("vehicle.lincoln.mkz", version="0.9.16", data=DATA) == (0.0, -0.37, 1.2, -6.0)
    assert cockpit.major_key("1.0.0") == "1.0" and cockpit.major_key("0.9.16") == "0.9"
    assert cockpit.major_key(1) == "1.0"
    # unknown: scaled by the box; a van sits forward and higher than a car of the same length
    car = cockpit.eye_for("vehicle.x.car", extent=(2.4, 0.92, 0.75), data={})
    van = cockpit.eye_for("vehicle.x.van", extent=(2.4, 0.92, 1.25), base_type="van", data={})
    assert -0.3 < car[0] < 0.1 and car[1] < 0 and 1.1 < car[2] < 1.3
    assert van[0] > car[0] + 0.5 and van[2] > car[2] + 0.5
    assert cockpit.mirror([1, 2, 3]) == [1, -2, 3]


def test_eye_file_is_complete_and_sane():
    data = cockpit.load()
    cars = data.get("0.9") or {}
    assert len(cars) >= 25
    for bp, e in cars.items():
        x, y, z = e["eye"]
        ex, ey, ez = e["extent"]
        assert abs(x - e.get("bb_x", 0)) < ex and -ey < y < 0 and 0.8 < z < 2 * ez, bp
        assert e["passenger"][1] == -y, bp
        if e.get("wheel"):
            c, r = e["wheel"]["centre"], e["wheel"]["radius"]
            assert 0.11 <= r <= 0.25 and c[0] > x + 0.4 and c[2] < z, bp
    assert cockpit.load() is data       # cached
    text = cockpit.dump_eyes(data)
    import json
    assert json.loads(text) == data


def test_projection_and_plane_depth():
    size, fov = (1280, 720), 90.0
    eye = (0.0, 0.0, 1.2)
    u, v, d = cockpit.project((2.0, 0.0, 1.2), eye, 0.0, size, fov)
    assert (u, v, d) == (640.0, 360.0, 2.0)
    u, v, d = cockpit.project((1.0, 1.0, 1.2), eye, 0.0, size, fov)      # 45 deg right: the image's edge
    assert u == pytest.approx(1280.0) and v == pytest.approx(360.0)
    u, v, d = cockpit.project((1.0, 0.0, 1.2 - math.tan(math.radians(10))), eye, -10.0, size, fov)
    assert v == pytest.approx(360.0)          # a camera pitched 10 deg down sees that point in the middle
    assert math.isnan(cockpit.project((-1.0, 0.0, 1.2), eye, 0.0, size, fov)[0])
    # plane depth agrees with the projection of a point on the plane
    c, n = (0.9, -0.1, 0.9), cockpit._unit((-0.9, 0.0, 0.4))
    p = (0.9, 0.0, 0.9 + 0.1 * 0.9 / 0.4 * 0)       # on the plane: same x and z as the centre, y moved
    u, v, d = cockpit.project(p, eye, -4.0, size, fov)
    assert cockpit.plane_depth(u, v, c, n, eye, -4.0, size, fov) == pytest.approx(d, rel=1e-6)


def test_quad_homography_and_warp():
    q = cockpit.quad((1.0, 0.0, 1.0), (-1.0, 0.0, 0.0), 0.4, 0.2)
    assert q[0] == pytest.approx((1.0, -0.2, 1.1)) and q[2] == pytest.approx((1.0, 0.2, 0.9))
    r, u = cockpit.panel_axes(cockpit._unit((-math.cos(0.4), 0.0, math.sin(0.4))))
    assert r == pytest.approx((0.0, 1.0, 0.0)) and u[0] > 0 and u[2] > 0     # tilted back: up leans forward
    dst = [(10, 10), (30, 12), (28, 40), (8, 35)]
    h = cockpit.homography([(0, 0), (1, 0), (1, 1), (0, 1)], dst)
    for s, d in zip([(0, 0), (1, 0), (1, 1), (0, 1)], dst):
        assert cockpit.apply_h(h, *s) == pytest.approx(d)
    assert cockpit.homography([(0, 0), (0, 0), (1, 1), (2, 2)], dst) is None
    c = cockpit.warp_coeffs((200, 100), dst, origin=(5, 5))
    assert cockpit.apply_h(c, 30 - 5, 12 - 5) == pytest.approx((200, 0))      # output pixel -> texture pixel


def test_panels_sit_where_a_driver_sees_them():
    e = DATA["0.9"]["vehicle.tesla.model3"]
    eye, size, fov = e["eye"], (1280, 720), 90.0
    c, n, (w, h) = cockpit.cluster_panel(e, 0.5)
    wc, wn, wr = cockpit.wheel_of(e)
    assert cockpit._dot(cockpit._sub(c, wc), wn) == pytest.approx(-0.16, abs=1e-6)    # 16 cm beyond the wheel
    assert h == pytest.approx(0.5 * w)
    # its centre shows inside the rim's upper half, below the horizon
    uc, vc, _ = cockpit.project(c, eye, cockpit.DEFAULT_PITCH, size, fov)
    top = tuple(wc[i] + cockpit.panel_axes(wn)[1][i] * wr for i in range(3))
    _, vtop, _ = cockpit.project(top, eye, cockpit.DEFAULT_PITCH, size, fov)
    uw, vw, _ = cockpit.project(wc, eye, cockpit.DEFAULT_PITCH, size, fov)
    assert vtop < vc < vw and abs(uc - uw) < 5 and vc > 360
    cc, cn, _ = cockpit.centre_panel(e)
    assert cc[1] == 0.0 and cn[1] < 0          # on the centre line, facing the (left) driver
    hc, hn, (hw, hh) = cockpit.hud_panel(eye, 0.0, 0.25)
    assert hc[0] - eye[0] == pytest.approx(2.5 * math.cos(math.radians(5))) and hc[2] < eye[2]
    assert 2 * math.degrees(math.atan(hw / 2 / 2.5)) == pytest.approx(cockpit.HUD_FOV)
    assert cockpit.mirror_panel(((1, 2, 3), (-1, 0.5, 0), (1, 1)))[0][1] == -2
    assert cockpit.wheel_degrees(0.5, 70) == pytest.approx(245) and cockpit.wheel_degrees(-1, 70) == -450


def _tex(w, h, rgb=(255, 255, 255), a=255):
    t = np.zeros((h, w, 4), np.uint8)
    t[..., :3], t[..., 3] = rgb, a
    return t


def test_cabin_view_draws_into_the_panel_and_hides_behind_the_wheel():
    e = DATA["0.9"]["vehicle.tesla.model3"]
    size, fov = (640, 360), 90.0
    cab = cockpit.CabinView(e, e["eye"], cockpit.DEFAULT_PITCH, size, fov)
    img = np.full((360, 640, 3), 50, np.uint8)
    tex = _tex(*cab.tex_size("cluster", 0.4))
    assert cab.draw(img, "cluster", tex)
    corners = cockpit.panel_corners_px(cab.panel("cluster", 0.4), e["eye"], cockpit.DEFAULT_PITCH, size, fov)
    uc = int(sum(p[0] for p in corners) / 4)
    vc = int(sum(p[1] for p in corners) / 4)
    assert img[vc, uc].min() > 200 and img[5, 5].max() == 50
    lit = (img > 100).any(2)
    # no depth image: the rim (geometry) still hides a band of the panel
    full = np.zeros_like(lit)
    from PIL import Image, ImageDraw
    m = Image.new("L", size, 0)
    ImageDraw.Draw(m).polygon([tuple(p) for p in corners], fill=1)
    full = np.asarray(m, bool)
    assert lit.sum() < 0.99 * full.sum()
    # with a depth image: something 10 cm in front of the eye everywhere hides the panel completely
    cab.set_depth(np.full((360, 640), 0.1, np.float32))
    img2 = np.full((360, 640, 3), 50, np.uint8)
    cab.draw(img2, "cluster", tex)
    assert (img2 == 50).all()
    # the HUD adds light (wherever the depth image says: CARLA's 2020 cars' glass is in it), a black box nothing
    cab.set_depth(np.full((360, 640), 0.5, np.float32))
    img3 = np.full((360, 640, 3), 100, np.uint8)
    t = _tex(200, 50)
    t[:, :100, :3] = 0
    assert cab.draw(img3, "hud", t, additive=True)
    assert img3.max() == 255 and img3.min() == 100


def test_wheel_spin_turns_the_wheels_pixels():
    e = {"eye": [0.0, 0.0, 1.2], "wheel": {"centre": [0.65, 0.0, 0.9], "axis": [-1.0, 0.0, 0.0], "radius": 0.19}}
    size, fov, pitch = (320, 240), 90.0, 0.0
    # depth: the wheel's plane (x = 0.65) inside the rim, far dashboard elsewhere
    cab = cockpit.CabinView(e, e["eye"], pitch, size, fov)
    rays = cockpit._rays(size, fov, 0, 0, size[0], size[1])
    t = cockpit._plane_depths(rays, (0.65, 0, 0.9), (-1, 0, 0), e["eye"], pitch)
    a, b = cockpit._in_plane(rays, t, (0.65, 0, 0.9), (-1, 0, 0), e["eye"], pitch)
    rr = np.hypot(a, b)
    depth = np.where(rr < 0.19, t, 1.5).astype(np.float32)
    cab.set_depth(depth)
    assert cab.spin is not None
    img = np.full((240, 320, 3), 40, np.uint8)
    marker = (rr < 0.19) & (b > 0.12) & (np.abs(a) < 0.03)        # a spoke pointing up
    img[marker] = (250, 0, 0)
    assert cab.spin.lift(img, 90.0)
    assert (img[marker] != (250, 0, 0)).any(1).mean() > 0.9        # lifted out
    cab.spin.put_back(img)
    turned = (rr < 0.19) & (a > 0.12) & (np.abs(b) < 0.03)        # clockwise for the driver: now pointing right
    assert (img[turned] == (250, 0, 0)).all(1).mean() > 0.7
    assert (img[marker] == (250, 0, 0)).all(1).mean() < 0.3
    assert not cab.spin.lift(img, 0.1)                              # nothing to turn


def test_incar_dashboard_style_and_overlay_fallback():
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    pygame.init()
    e = DATA["0.9"]["vehicle.tesla.model3"]
    cfg = {"cluster": True, "hud": "hud_speed", "centre": {"design": "centre_screen", "rect": [0.7, 0.7, 0.25]}}
    ic = incar.InCar(cfg, "driver", (640, 360), log=lambda s: None)
    ic.set_cabin(cockpit.CabinView(e, e["eye"], cockpit.DEFAULT_PITCH, (640, 360), 90.0))
    ic.update(0.0, 47.0, limit=50)
    screen = pygame.Surface((640, 360))
    screen.fill((60, 60, 60))
    ic.draw(screen)
    px = pygame.surfarray.array3d(screen).swapaxes(0, 1)
    changed = (px != 60).any(2)
    x, y, w, h = ic.displays["centre"]["rect"]       # a display with its own rect stays flat there
    assert changed[y:y + h, x:x + w].mean() > 0.5
    assert changed[:y, :].mean() < 0.2              # the rest: panels, not rectangles over the screen
    over = incar.InCar(dict(cfg, style="overlay"), "driver", (640, 360), log=lambda s: None)
    over.set_cabin(cockpit.CabinView(e, e["eye"], 0.0, (640, 360), 90.0))
    assert over.cabin is None


def _ring(c, n, r, k=400, noise=0.003, rng=None):
    rng = rng or np.random.default_rng(0)
    n = np.asarray(n, float) / np.linalg.norm(n)
    u = np.cross(n, [0, 0, 1.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    ang = rng.uniform(0, 2 * np.pi, k)
    return np.asarray(c) + r * (np.outer(np.cos(ang), u) + np.outer(np.sin(ang), v)) + rng.normal(0, noise, (k, 3))


def test_wheel_finder_and_eye_rule():
    rng = np.random.default_rng(3)
    axis = np.array([-0.9, 0.0, 0.42])
    wheel = _ring([0.8, -0.38, 0.85], axis, 0.185, rng=rng)
    clutter = np.column_stack([rng.uniform(0.3, 1.3, 3000), rng.uniform(-0.8, 0.0, 3000), rng.uniform(0.5, 0.75, 3000)])
    tyre = _ring([1.5, -0.8, 0.35], [0, 1, 0], 0.33, rng=rng)       # sideways: never a steering wheel
    found = cm.find_wheel(np.vstack([wheel, clutter, tyre]), iters=3000)
    assert found is not None
    c, n, r, ninl, sectors = found
    assert np.allclose(c, [0.8, -0.38, 0.85], atol=0.01) and abs(r - 0.185) < 0.01 and sectors >= 11
    assert np.dot(n, axis / np.linalg.norm(axis)) > 0.99
    eye = cm.eye_from(c, r, n, 0.42, 1.40)
    assert eye[0] == pytest.approx(c[0] - 0.65, abs=1e-3) and eye[1] == pytest.approx(c[1], abs=1e-3)
    assert eye[2] == pytest.approx(min(0.42 + 0.735, 1.20), abs=1e-3)
    assert cm.eye_from(c, r, n, 0.42, 1.25)[2] == pytest.approx(1.05)      # the roof wins
    assert cm.find_wheel(clutter, iters=500) is None
    pts = cm.unproject(np.full((4, 4), 2.0), 90.0, (0.0, 0.0, 1.0, 0.0, 0.0))
    assert np.allclose(pts[..., 0], 2.0) and np.allclose(pts.mean((0, 1)), [2.0, 0.0, 1.0])
    assert len(cm.voxel_thin(np.zeros((10, 3)))) == 1


def test_disc_finder_on_a_solid_wheel():
    rng = np.random.default_rng(5)
    # a dashboard wall at x 1.0 and a solid disc (an old mesh's wheel: no hole) at x ~0.68 in front of it
    dash = np.column_stack([np.full(20000, 1.0) + rng.normal(0, 0.003, 20000), rng.uniform(-0.8, -0.15, 20000),
                            rng.uniform(0.6, 1.2, 20000)])
    a, b = rng.uniform(-1, 1, (2, 20000))
    k = a * a + b * b < 1
    disc = np.column_stack([0.68 + 0.15 * b[k] * 0.14, -0.40 + 0.15 * a[k], 0.94 + 0.15 * b[k]])
    found = cm.find_disc(np.vstack([dash, disc]), (0.0, -0.36, 1.25))
    assert found is not None
    c, n, r, _npts, _sec = found
    assert np.allclose(c, [0.68, -0.40, 0.94], atol=0.02) and abs(r - 0.15) < 0.02 and n[0] < -0.9 and n[2] > 0
    assert cm.find_disc(dash, (0.0, -0.36, 1.25)) is None


def test_glass_tint_brightens_the_cabin_exposure():
    assert cockpit.cabin_ev_bias({}) == cockpit.CABIN_EV_BIAS == -1.0
    assert cockpit.cabin_ev_bias({"glass_ev": 0.8}) == pytest.approx(-1.8)
    assert cockpit.cabin_ev_bias({"glass_ev": 5.6}) == -2.5           # within -2.5..0
    assert cockpit.cabin_ev_bias({"glass_ev": -0.1}, base=-0.5) == pytest.approx(-0.4)
    assert cockpit.cabin_ev_bias({"glass_ev": 0.8}, sun_altitude=40) == pytest.approx(-1.8)
    assert cockpit.cabin_ev_bias({"glass_ev": 0.8}, sun_altitude=5) == pytest.approx(-0.9)
    assert cockpit.cabin_ev_bias({"glass_ev": 0.8}, sun_altitude=-12) == pytest.approx(-0.45)
    blurred = cockpit._box_blur(np.dstack([np.eye(9) * 9.0]), 3)
    assert blurred.shape == (9, 9, 1) and blurred[4, 4, 0] == pytest.approx(3.0)


def test_derived_panels_settle_onto_the_dashboard():
    e = DATA["0.9"]["vehicle.tesla.model3"]
    size, fov, pitch = (640, 360), 90.0, cockpit.DEFAULT_PITCH
    cab = cockpit.CabinView(e, e["eye"], pitch, size, fov)
    pnl = cab.panel("cluster", 0.4)
    corners = cockpit.panel_corners_px(pnl, e["eye"], pitch, size, fov)
    vline = int(max(v for _u, v in corners)) - 2        # the dashboard's edge: nearly all of the panel above it
    rays = cockpit._rays(size, fov, 0, 0, size[0], size[1])
    t = cockpit._plane_depths(rays, pnl[0], pnl[1], e["eye"], pitch)
    depth = np.where(np.arange(360)[:, None] >= vline, np.minimum(t + 0.01, 50), 50.0).astype(np.float32)
    cab.set_depth(depth)
    moved = cab.panel("cluster", 0.4)
    assert moved[0][2] < pnl[0][2] - 0.02                 # lowered onto the dashboard
    top = max(v for _u, v in cockpit.panel_corners_px(moved, e["eye"], pitch, size, fov)[:2])
    assert top >= vline - 3
    hand = dict(e, cluster={"centre": [1.0, -0.4, 0.95], "normal": [-1, 0, 0.3], "size": [0.3, 0.12]})
    cab2 = cockpit.CabinView(hand, e["eye"], pitch, size, fov, depth=depth)
    assert cab2.panel("cluster", 0.4)[0] == (1.0, -0.4, 0.95)   # a hand-placed panel stays
