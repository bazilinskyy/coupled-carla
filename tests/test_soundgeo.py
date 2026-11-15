"""coupledcarla.soundgeo: occlusion by buildings, facade reflections, the street canyon; and their effect in the
soundscape. No CARLA."""
import json
import math

import numpy as np
import pytest

from coupledcarla import soundscape as ss
from coupledcarla.soundgeo import Buildings, canyon_reverb


def box(x0, y0, x1, y1, h=15.0, z=0.0):
    return {"p": [[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]], "h": h, "z": z}


def test_occlusion_by_a_building():
    b = Buildings([box(-10, -10, 10, 10)])
    att, n = b.occlusion((-30.0, 0.0, 0.05), (30.0, 0.0, 1.6))
    assert n == 1
    assert att[4] > 12.0 and att[7] > att[0]                 # more at high frequencies (Maekawa)
    assert att.max() <= 25.0
    clear, n0 = b.occlusion((-30.0, 20.0, 0.05), (30.0, 20.0, 1.6))     # beside the building
    assert n0 == 0 and np.all(clear == 0)
    low = Buildings([box(-10, -10, 10, 10, h=1.0)])          # a 1 m wall
    att_low, _ = low.occlusion((-30.0, 0.0, 0.05), (30.0, 0.0, 1.6))
    assert att_low[4] < att[4]                               # a lower obstacle screens less
    # a long block screens more than a short one (the way round the sides is longer)
    long_b = Buildings([box(-10, -100, 10, 100)])
    att_long, _ = long_b.occlusion((-30.0, 0.0, 0.05), (30.0, 0.0, 1.6))
    assert att_long[4] >= att[4]


def test_reflections_off_a_facade():
    wall = Buildings([box(-100, 10, 100, 30)])               # its facade at y = 10 faces -y
    e, dist = wall.facades_near(0.0, 0.0, radius=40)
    assert len(e) == 1 and dist[0] == pytest.approx(10.0)
    refl = wall.reflections((-20.0, 0.0, 0.5), (20.0, 0.0, 1.6), e)
    assert len(refl) == 1
    L, (rx, ry), _edge = refl[0]
    assert L == pytest.approx(math.hypot(math.hypot(40.0, 20.0), 1.1), rel=1e-6)
    assert rx == pytest.approx(0.0, abs=1e-6) and ry == pytest.approx(10.0)
    # a building in the way of a leg: no reflection
    blocked = Buildings([box(-100, 10, 100, 30), box(-12, 2, -8, 6)])
    e2, _ = blocked.facades_near(0.0, 0.0, radius=40)
    assert blocked.reflections((-20.0, 0.0, 0.5), (20.0, 0.0, 1.6), e2) == []


def test_street_canyon_and_its_reverb():
    street = Buildings([box(-200, 10, 200, 30, h=18), box(-200, -30, 200, -10, h=12)])
    w, h, sides = street.canyon(0.0, 0.0, 0.0)
    assert sides == 2 and w == pytest.approx(20.0) and h == pytest.approx(15.0)
    rt, lev = canyon_reverb(w, h, sides)
    assert 0.6 < rt[4] < 2.5 and rt[7] < rt[4]               # air absorption shortens the highs
    assert -45.0 < lev < -25.0
    rt_open, lev_open = canyon_reverb(None, 0.0, 0)
    assert lev_open < lev and rt_open[4] < rt[4]
    open_space = Buildings([])
    assert open_space.canyon(0.0, 0.0, 0.0)[2] == 0


def test_buildings_from_a_map_folder(tmp_path):
    (tmp_path / "buildings.json").write_text(json.dumps([box(0, 0, 10, 10), {"p": [[0, 0], [1, 0]], "h": 3}]))
    b = Buildings.from_map(tmp_path, log=lambda s: None)
    assert b.n == 1
    assert Buildings.from_map(tmp_path / "nowhere", log=lambda s: None) is None


def _car(i, x, y, kmh=50.0, yaw=0.0):
    v = kmh / 3.6
    return ss.Body(i, "vehicle.audi.a2", "car", x, y, 0.0, v * math.cos(math.radians(yaw)),
                   v * math.sin(math.radians(yaw)), 0.0, yaw, True, False)


def test_soundscape_hears_round_corners_through_reflections():
    cfg = {"enabled": True, "environment": False}
    blk = Buildings([box(10, -15, 40, 15, h=20)])           # a block between listener and car
    open_sc = ss.Soundscape(cfg, log=lambda s: None)
    hid_sc = ss.Soundscape(cfg, log=lambda s: None, buildings=blk)
    car = _car(1, 60.0, 0.0)
    for k in range(3):
        open_sc.update(0.05 * k, (0, 0, 1.6, 0), [car])
        hid_sc.update(0.05 * k, (0, 0, 1.6, 0), [car])
    a, b = open_sc.voices_info()[1]["tyre_db"], hid_sc.voices_info()[1]["tyre_db"]
    assert a - b > 10.0                                      # the block screens the direct path
    # a street wall behind the listener reflects a car passing in front: a delayed second path
    wall = Buildings([box(-200, -40, 200, -10)])
    sc = ss.Soundscape(cfg, log=lambda s: None, buildings=wall)
    sc.update(0.0, (0, 0, 1.6, 0), [_car(2, 15.0, 5.0)])
    v = sc.mixer.get(("veh", 2, "tyre"))
    refl = [p for p in v.paths if p.kind == "refl" and p.w is not None and np.any(p.w)]
    assert refl and refl[0].delay > 0.01
