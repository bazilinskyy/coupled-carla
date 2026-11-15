"""Right-hand-drive cabins (rhd.py): the camera maths, the skin test that tells glass from the cabin, the per-pixel
plan and the composite. No CARLA, no pygame."""
import math

import numpy as np
import pytest

from coupledcarla import rhd


def test_basis_and_projection_round_trip():
    for pitch, yaw in ((0.0, 0.0), (-12.0, 150.0), (-89.0, 0.0)):
        f, r, u = (np.array(v) for v in rhd.basis(pitch, yaw))
        assert np.allclose([f @ f, r @ r, u @ u], 1.0) and abs(f @ r) < 1e-9 and abs(f @ u) < 1e-9
    cam = (0.1, -0.4, 1.2, -4.0, 0.0)
    depth = np.full((54, 96), 2.0)
    pts = rhd.hit_points(depth, cam, 90.0)
    u, v, x = rhd.project(pts, cam, (96, 54), 90.0)
    gu, gv = np.meshgrid(np.arange(96) + 0.5, np.arange(54) + 0.5)
    assert np.allclose(u, gu) and np.allclose(v, gv) and np.allclose(x, 2.0)


def test_exterior_cams_look_at_the_car():
    cams = rhd.exterior_cams((2.4, 1.0, 0.75), (0.1, 0.0, 0.7))
    assert len(cams) == 25
    for cam in cams:
        u, v, x = rhd.project(np.array([[0.1, 0.0, 0.7]]), cam, (640, 360), rhd.EXT_FOV)
        assert u[0] == pytest.approx(320) and v[0] == pytest.approx(180) and x[0] == pytest.approx(rhd.EXT_RADIUS)


def test_skin_mask_tells_glass_from_cabin():
    # three cabin hit points: on the windscreen (an outside camera sees it), on the dashboard (behind the glass:
    # the outside camera's hit in front lies inside the car's box) and one behind a parked car (unknown: skin)
    ext, centre = (2.4, 1.0, 0.75), (0.0, 0.0, 0.75)
    cam = (6.0, 0.0, 1.0, 0.0, 180.0)                  # in front of the car, looking back at it
    pts = np.array([[[1.0, 0.0, 1.0], [0.6, 0.3, 1.0], [0.8, -0.3, 1.0]]])
    us, vs, xs = rhd.project(pts, cam, (64, 36), 60.0)
    depth = np.full((36, 64), 50.0)
    def at(i, d):
        depth[int(vs[0, i]), int(us[0, i])] = d
    at(0, xs[0, 0] - 0.005)                            # the glass's outer face, 5 mm in front: seen
    at(1, xs[0, 1] - 0.4)                              # the windscreen, 40 cm in front, inside the box: hidden
    at(2, 1.0)                                         # something 1 m from the camera, far outside the car's box
    skin = rhd.skin_mask(pts, [(cam, depth, 60.0)], ext, centre)
    assert skin.tolist() == [[True, False, True]]


def test_plan_composite_and_fill():
    h, w = 20, 40
    skin_a = np.ones((h, w), bool)
    skin_b_m = np.zeros((h, w), bool)
    skin_b_m[:10] = True                              # top half: street through the windscreen
    skin_a[:10, 30:36] = False                         # where A sees its own (left-hand) wheel: unknown street
    plan = rhd.Plan(skin_a, skin_b_m, feather=1)
    assert plan.shares["cabin"] == pytest.approx(0.5) and plan.shares["filled"] > 0
    a = np.zeros((h, w, 3), np.uint8)
    a[:, :, 0] = 200                                   # A: red street
    a[:10, 30:36] = 7                                  # ... with A's own wheel in it
    b_m = np.zeros((h, w, 3), np.uint8)
    b_m[:, :, 2] = 100                                 # mirrored B: blue cabin
    out = plan.apply(a, b_m)
    assert (out[15, 5] == [0, 0, 100]).all()           # cabin from B
    assert (out[3, 5] == [200, 0, 0]).all()            # street from A
    assert (out[3, 32] == [200, 0, 0]).all()           # the wheel's pixels filled with the street around them


def test_mirror():
    img = np.arange(12).reshape(2, 6)
    assert rhd.mirror(img).tolist() == [[5, 4, 3, 2, 1, 0], [11, 10, 9, 8, 7, 6]]


def _box_depth(cam, size, fov, ext, centre, inside):
    """Depth image of a box (the car's shell) from cam: the exit distance from inside, the entry from outside."""
    h, w = size[1], size[0]
    f = rhd.focal(size, fov)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    fw, rt, up = (np.array(a) for a in rhd.basis(cam[3], cam[4] if len(cam) > 4 else 0.0))
    ray = fw[None, None] + ((u - w / 2) / f)[..., None] * rt + (-(v - h / 2) / f)[..., None] * up
    o = np.array(cam[:3]) - np.array(centre)
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-np.array(ext) - o) / ray
        t2 = (np.array(ext) - o) / ray
    tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = tmax >= np.maximum(tmin, 0)
    return np.where(hit, tmax if inside else tmin, 1000.0)      # depth along the view axis: ray's forward part is 1


def test_plan_for_a_box_car():
    # the shell is a box (glass and body alike, as CARLA's depth images show them); the dashboard fills the lower
    # half of both seats' views 0.8 m ahead: the plan takes the cabin there and the street through the shell above
    size, fov = (48, 27), 90.0
    eye, pitch = (0.0, 0.4, 1.2), 0.0
    ext, centre = (2.0, 0.9, 0.75), (0.0, 0.0, 0.75)
    ce = (eye[0], eye[1], eye[2], pitch, 0.0)
    d = _box_depth(ce, size, fov, ext, centre, inside=True)
    d[size[1] // 2 + 2:, :] = 0.8
    views = [(cam, _box_depth(cam, (64, 36), rhd.EXT_FOV, ext, centre, inside=False), rhd.EXT_FOV)
             for cam in rhd.exterior_cams(ext, centre)]
    plan = rhd.plan_for(d, d, eye, pitch, fov, views, ext, centre)
    assert plan.shares["cabin"] == pytest.approx(11 / 27, abs=0.08)
    assert plan.shares["filled"] == pytest.approx(0.0, abs=0.01)
