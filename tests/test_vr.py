"""vr.py: OpenXR <-> CARLA frames, eye poses and IPD offsets, the eye cameras' fields of view, head-pose prediction,
latency, frame bookkeeping, drawing into the eyes, and the simulated headset (no CARLA, no OpenXR)."""
import math
import time

import numpy as np
import pytest

from coupledcarla import config, vr


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


# --- quaternions and rotators -------------------------------------------------------------------------------------

def test_quaternion_basics():
    q = vr.qaxis((0, 0, 1), math.pi / 2)
    assert close(vr.qrot(q, (1, 0, 0)), (0, 1, 0))
    assert close(vr.qmul(q, vr.qconj(q)), vr.IDENTITY)
    m = vr.qmatrix(q)
    assert close(apply(m, (1, 0, 0)), (0, 1, 0))
    assert close(vr.matrix_quat(m), q)
    assert vr.qangle(q, vr.IDENTITY) == pytest.approx(math.pi / 2)


def test_rotator_matrix_is_ues():
    """UE's rotation matrix: yaw about z (x towards y), pitch up, roll right side down = Rz(y) Ry(-p) Rx(-r)."""
    for p, y, r in ((10, 30, 5), (-20, 170, -40), (0, -90, 0), (45, 0, 0)):
        m = vr.rotator_matrix(p, y, r)
        want = matmul(rz(math.radians(y)), matmul(ry(-math.radians(p)), rx(-math.radians(r))))
        assert all(close(a, b, 1e-9) for a, b in zip(m, want))
        assert close(vr.matrix_rotator(m), (p, y, r), 1e-9)
        assert close(vr.quat_rotator(vr.rotator_quat(p, y, r)), (p, y, r), 1e-7)
    # forward of pitch 0 yaw 90 is +y (CARLA's right); pitch +30 looks up
    assert close(apply(vr.rotator_matrix(0, 90, 0), (1, 0, 0)), (0, 1, 0))
    assert apply(vr.rotator_matrix(30, 0, 0), (1, 0, 0))[2] == pytest.approx(0.5)


def test_xr_to_ue_quaternion_is_the_change_of_axes():
    """C R C^T for C: (x, y, z)_xr -> (-z, x, y)_ue, an improper change of axes (det -1)."""
    c = ((0, 0, -1), (1, 0, 0), (0, 1, 0))
    ct = tuple(tuple(c[j][i] for j in range(3)) for i in range(3))
    for axis, ang in (((0, 1, 0), 0.4), ((1, 0, 0), -0.7), ((0.3, 0.5, 0.8), 1.1)):
        q = vr.qnorm(vr.qaxis(axis, ang))
        want = matmul(c, matmul(vr.qmatrix(q), ct))
        got = vr.qmatrix(vr.xr_to_ue_quat(q))
        assert all(close(a, b, 1e-9) for a, b in zip(got, want))
        assert close(vr.ue_to_xr_quat(vr.xr_to_ue_quat(q)), q)
    v = (0.1, 0.2, 0.3)
    assert close(vr.ue_to_xr_vec(vr.xr_to_ue_vec(v)), v)
    assert vr.xr_to_ue_vec((0, 0, -1)) == (1, 0, -0.0)       # OpenXR forward is CARLA's +x


def test_head_turns_map_to_carla_rotators():
    left = vr.qaxis((0, 1, 0), math.radians(30))         # OpenXR: +30 deg about up = turning left
    p, y, r = vr.quat_rotator(vr.xr_to_ue_quat(left))
    assert y == pytest.approx(-30) and p == pytest.approx(0, abs=1e-9) and r == pytest.approx(0, abs=1e-9)
    up = vr.qaxis((1, 0, 0), math.radians(20))            # +20 deg about right = looking up
    assert vr.quat_rotator(vr.xr_to_ue_quat(up))[0] == pytest.approx(20)
    tilt = vr.qaxis((0, 0, -1), math.radians(15))         # about forward (-z): right ear down
    assert vr.quat_rotator(vr.xr_to_ue_quat(tilt))[2] == pytest.approx(15)


# --- eye poses ----------------------------------------------------------------------------------------------------

def test_ipd_offsets_and_eye_poses():
    l, r = vr.eye_offsets_ue(0.063)
    assert l == (0.0, -0.0315, 0.0) and r == (0.0, 0.0315, 0.0)
    seat = (0.2, -0.38, 1.2)
    head = ((0.0, 0.0, 0.0), vr.IDENTITY)
    le, re = vr.eye_poses(seat, head, 0.063)
    assert math.dist(le[0], re[0]) == pytest.approx(0.063)
    assert close(le[0], (0.2, -0.38 - 0.0315, 1.2)) and close(re[0], (0.2, -0.38 + 0.0315, 1.2))
    # head turned 90 deg right: the eyes now lie along the car's x (left eye in front)
    turned = ((0.0, 0.0, 0.0), vr.rotator_quat(0, 90, 0))
    le, re = vr.eye_poses(seat, turned, 0.063)
    assert close(le[0], (0.2 + 0.0315, -0.38, 1.2), 1e-9) and close(re[0], (0.2 - 0.0315, -0.38, 1.2), 1e-9)
    assert vr.to_transform(le)[4] == pytest.approx(90)
    # a head moved 10 cm forward and up from where it was recentred
    moved = ((0.1, 0.0, 0.05), vr.IDENTITY)
    assert close(vr.eye_poses(seat, moved, 0.0)[0][0], (0.3, -0.38, 1.25))
    # a walker's eyes turn with its heading (seat_yaw)
    le, _ = vr.eye_poses((10, 20, 1.6), head, 0.063, seat_yaw=90)
    assert close(le[0], (10 + 0.0315, 20, 1.6), 1e-9)


def test_xr_eye_poses_and_recentering():
    head = ((0.3, 1.2, -0.5), vr.qaxis((0, 1, 0), math.radians(40)))
    le, re = vr.eye_poses_xr(head, 0.064)
    assert math.dist(le[0], re[0]) == pytest.approx(0.064)
    origin = vr.recenter_origin(head)
    p, q = vr.head_in_seat(head, origin)
    assert close(p, (0, 0, 0), 1e-9) and vr.qangle(q, vr.IDENTITY) < 1e-6
    # pitch is kept (only the heading recentres); moving forward in the recentred frame is CARLA +x
    tilted = (head[0], vr.qmul(head[1], vr.qaxis((1, 0, 0), math.radians(10))))
    o2 = vr.recenter_origin(tilted)
    assert vr.qangle(o2[1], origin[1]) < 1e-6
    assert vr.quat_rotator(vr.head_in_seat(tilted, o2)[1])[0] == pytest.approx(10)
    fwd = vr.qrot(head[1], (0, 0, -0.2))
    ahead = ((head[0][0] + fwd[0], head[0][1] + fwd[1], head[0][2] + fwd[2]), head[1])
    assert close(vr.head_in_seat(ahead, origin)[0], (0.2, 0, 0), 1e-9)


def test_compose_and_inverse():
    a = ((1.0, 2.0, 3.0), vr.rotator_quat(10, 20, 30))
    b = ((0.5, -0.2, 0.1), vr.rotator_quat(-5, 40, 0))
    ab = vr.compose(a, b)
    back = vr.compose(vr.inverse(a), ab)
    assert close(back[0], b[0], 1e-9) and vr.qangle(back[1], b[1]) < 1e-7
    assert vr.qangle(vr.from_transform(*vr.to_transform(ab))[1], ab[1]) < 1e-6


# --- fields of view -----------------------------------------------------------------------------------------------

def test_eye_camera_covers_an_asymmetric_eye():
    fov = (math.radians(-52), math.radians(44), math.radians(48), math.radians(-52))     # Quest 2 like, left eye
    cam = vr.eye_camera(fov, 1832, scale=1.0, overscan=0.0)
    assert cam["hfov_deg"] == pytest.approx(104.0, abs=0.01)                  # 2 x the wider side
    l, r, u, d = cam["layer_fov"]
    assert l == pytest.approx(-r) and u == pytest.approx(-d)                    # symmetric: CARLA's cameras are
    assert math.tan(u) >= math.tan(math.radians(52)) - 0.01
    assert cam["width"] / cam["height"] == pytest.approx(math.tan(r) / math.tan(u), rel=1e-3)   # square pixels
    ppt = 1832 / (math.tan(math.radians(52)) + math.tan(math.radians(44)))
    assert cam["width"] == pytest.approx(2 * math.tan(math.radians(52)) * ppt, abs=2)
    wider = vr.eye_camera(fov, 1832, 1.0, 0.1)
    assert math.tan(wider["layer_fov"][1]) == pytest.approx(1.1 * math.tan(math.radians(52)), rel=1e-3)
    half = vr.eye_camera(fov, 1832, 0.5, 0.0)
    assert half["width"] == pytest.approx(cam["width"] / 2, abs=2)
    capped = vr.eye_camera(fov, 1832, 1.0, 0.0, max_size=[1000, 1000])
    assert capped["width"] <= 1000 and capped["height"] <= 1000
    fixed = vr.eye_camera(vr.symmetric_fov(100, 100), 0, size=[960, 1056])
    assert (fixed["width"], fixed["height"]) == (960, 1056) and fixed["hfov_deg"] == pytest.approx(100, abs=0.01)
    assert fixed["width"] % 2 == 0 and fixed["height"] % 2 == 0


def test_project_point_and_head_locked_quad():
    size, fov = (1000, 800), vr.symmetric_fov(90, 2 * math.degrees(math.atan(0.8)))
    eye = ((0.0, 0.0, 0.0), vr.IDENTITY)
    u, v, d = vr.project_point((5, 0, 0), eye, size, fov)
    assert (u, v, d) == pytest.approx((500, 400, 5))
    u, v, _ = vr.project_point((1, 1, 0), eye, size, fov)          # 45 deg right: the right edge
    assert u == pytest.approx(1000) and v == pytest.approx(400)
    assert vr.project_point((-1, 0, 0), eye, size, fov)[2] < 0
    quad = vr.head_locked_quad(1.5, 50, 0.5)
    le, re = (o for o in vr.eye_offsets_ue(0.063))
    pl = vr.quad_in_eye(quad, (le, vr.IDENTITY), size, fov)
    pr = vr.quad_in_eye(quad, (re, vr.IDENTITY), size, fov)
    # stereo: the left eye sees the panel further right than the right eye does (crossed disparity at 1.5 m)
    assert pl[0][0] > pr[0][0]
    disparity = pl[0][0] - pr[0][0]
    assert disparity == pytest.approx(0.063 / 1.5 * 500, rel=0.01)
    assert vr.quad_in_eye([(-1, 0, 0)] * 4, eye, size, fov) is None


# --- prediction, latency and bookkeeping ------------------------------------------------------------------------

def test_prediction_at_constant_angular_velocity():
    q0 = vr.qaxis((0, 1, 0), 0.2)
    w = (0.0, 1.5, 0.0)                     # rad/s about up
    p, q = vr.predict(((0, 1.2, 0), q0), (0.1, 0, 0), w, 0.05)
    assert vr.qangle(q, vr.qaxis((0, 1, 0), 0.2 + 0.075)) < 1e-9
    assert close(p, (0.005, 1.2, 0))
    tr = vr.PoseTracker(smooth=0.0)
    for k in range(5):
        t = 0.011 * k
        tr.add(t, (0.0, 0.0, 0.0), vr.qaxis((0, 1, 0), 1.5 * t))
    assert close(tr.angular, (0, 1.5, 0), 1e-6)
    pred = tr.at(0.044 + 0.1)
    assert vr.qangle(pred[1], vr.qaxis((0, 1, 0), 1.5 * 0.144)) < 1e-6
    given = vr.PoseTracker()
    given.add(0.0, (0, 0, 0), vr.IDENTITY, (0, 0, 0), (0, 0, 2.0))      # the runtime's velocities win
    assert given.angular == (0, 0, 2.0)
    assert vr.PoseTracker().at(1.0) is None


def test_latency_horizon():
    lat = vr.Latency("auto", cap_ms=80)
    assert lat.horizon_ms() == 0.0
    for ms in (40, 50, 60, 1e6, -3):
        lat.add(ms)
    assert lat.median() == 50 and lat.horizon_ms() == 50
    for _ in range(10):
        lat.add(200)
    assert lat.horizon_ms() == 80              # capped
    assert vr.Latency(30).horizon_ms() == 30 and vr.Latency(500, cap_ms=80).horizon_ms() == 80


def test_pose_history_and_eye_pairs():
    h = vr.PoseHistory(keep=4)
    for f in (101, 102, 103):
        h.put(f, f * 10)
    assert h.get(102) == 1020 and h.get(105) == 1030 and h.get(100) is None
    for f in range(104, 110):
        h.put(f, f)
    assert len(h.items) <= 4 and h.get(103) is None
    pair = vr.EyePair()
    assert pair.put(0, 5, "L5") is None
    assert pair.put(0, 6, "L6") is None
    assert pair.put(1, 6, "R6") == ["L6", "R6"]
    assert 5 not in pair.pending                 # frame 5's right eye never came: dropped
    assert pair.put(1, 7, "R7") is None and pair.put(0, 7, "L7") == ["L7", "R7"]


# --- settings, the station's switches, the simulated headset -------------------------------------------------------

def test_settings_and_switches():
    assert vr.DEFAULTS == config.DEFAULTS["vr"]          # one set of defaults: config documents them
    s = vr.settings({"ipd_mm": 60}, {"scale": 0.7})
    assert s["ipd_mm"] == 60 and s["scale"] == 0.7 and s["overscan"] == vr.DEFAULTS["overscan"]
    assert vr.wanted("vr", {}) and vr.wanted(0, {"view": "vr"}) and not vr.wanted(1, {"view": "driver"})
    assert vr.seat_for("driver") == "driver" and vr.seat_for("av_passenger") == "passenger"
    assert vr.seat_for("driver", "passenger") == "passenger" and vr.seat_for("pedestrian") is None
    assert str(vr.log_path("logs/stations/driver_20261115-1200.csv")).endswith("driver_20261115-1200_vr.csv")


def test_sim_headset_runs_and_reports_shown_frames():
    hs = vr.make_headset(vr.settings({"runtime": "sim", "sim_hz": 200, "size": [320, 360]}))
    assert hs.start()
    try:
        t, disp, head, vel, ang = hs.pose()
        assert disp > t and len(head[1]) == 4 and len(vel) == 3 and len(ang) == 3
        assert hs.fov[1] == pytest.approx(math.radians(50))
        hs.submit(42, [None, None], (head, head), (hs.fov, hs.fov))
        deadline = time.time() + 2
        shown = None
        while shown is None and time.time() < deadline:
            time.sleep(0.01)
            shown = hs.first_shown(42)
        assert shown is not None and shown[0] > t and len(shown[1][1]) == 4
        assert hs.first_shown(42) is None
    finally:
        hs.stop()
    still = vr.SimHeadset(vr.settings({"runtime": "sim", "sim_motion": "still"}))
    assert still.head(5.0) == ((0.0, 0.0, 0.0), vr.IDENTITY)


def test_openxr_missing_says_what_to_install(monkeypatch):
    import builtins
    real = builtins.__import__

    def no_xr(name, *a, **kw):
        if name in ("xr", "glfw"):
            raise ImportError(f"No module named {name!r}")
        return real(name, *a, **kw)
    monkeypatch.setattr(builtins, "__import__", no_xr)
    with pytest.raises(RuntimeError, match=r"\[vr\]"):
        vr.make_headset(vr.settings({"runtime": "openxr"}))


# --- drawing into the eyes ----------------------------------------------------------------------------------------

def test_warp_into_draws_a_panel_where_it_projects():
    img = np.zeros((200, 300, 4), np.uint8)            # BGRA, as CARLA's images
    tex = np.zeros((10, 20, 4), np.uint8)
    tex[..., 0] = 255                                  # red (RGBA)
    tex[..., 3] = 255
    corners = [(100, 50), (200, 50), (200, 100), (100, 100)]
    assert vr.warp_into(img, tex, corners)
    assert img[75, 150, 2] == 255 and img[75, 150, 0] == 0          # red lands in BGRA's R channel
    assert img[20, 20, :3].sum() == 0 and img[150, 250, :3].sum() == 0
    rgb = np.full((200, 300, 3), 100, np.uint8)
    half = tex.copy()
    half[..., 3] = 128
    assert vr.warp_into(rgb, half, corners, additive=True, gain=0.5)
    assert 160 <= rgb[75, 150, 0] <= 170 and rgb[75, 150, 1] == 100      # light added, not painted
    assert not vr.warp_into(img, tex, [(500, 500), (600, 500), (600, 600), (500, 600)])      # off the image
    assert not vr.warp_into(img, tex, [(float("nan"), 0)] * 4)


def test_crop_panel_keeps_the_visible_part_in_place():
    tex = np.zeros((100, 200, 4), np.uint8)
    tex[60:80, 20:120, :] = 255
    quad = vr.head_locked_quad(1.5, 50, 0.5)
    sub, q = vr.crop_panel(tex, quad)
    assert sub.shape == (20, 100, 4)
    tl, tr, br, bl = quad
    assert close(q[0], tuple(tl[i] + 0.1 * (tr[i] - tl[i]) + 0.6 * (bl[i] - tl[i]) for i in range(3)))
    assert close(q[2], tuple(tl[i] + 0.6 * (tr[i] - tl[i]) + 0.8 * (bl[i] - tl[i]) for i in range(3)))
    assert vr.crop_panel(np.zeros((10, 10, 4), np.uint8), quad) == (None, None)
