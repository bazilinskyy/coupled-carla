"""Stacked roads (levels.py): lane picks by height, the bridge putting a car back onto its level or upright, the
check that SUMO's heights match CARLA's roads. No CARLA, no SUMO."""
import math
import types

import pytest

from coupledcarla import levels
from coupledcarla.bridge import animate


class _Edge:
    def __init__(self, eid):
        self.eid = eid

    def getID(self):
        return self.eid

    def getFunction(self):
        return ""


class _Lane3:
    """A straight lane along +x at height z (SUMO x, y)."""
    def __init__(self, eid, y, z, length=100.0):
        self.edge, self.y, self.z, self.length = _Edge(eid), y, z, length

    def getShape(self):
        return [(0.0, self.y), (self.length, self.y)]

    def getShape3D(self):
        return [(0.0, self.y, self.z), (self.length, self.y, self.z + 10.0)]     # climbs 10 m over its length

    def getEdge(self):
        return self.edge

    def allows(self, vclass):
        return True

    def getLength(self):
        return self.length

    def getClosestLanePosAndDist(self, p):
        return max(0.0, min(self.length, p[0])), abs(p[1] - self.y)


def test_lane_z_follows_the_shape_and_nets_without_heights_give_zero():
    ln = _Lane3("e", 0.0, 20.0)
    assert levels.lane_z(ln, 0.0) == pytest.approx(20.0)
    assert levels.lane_z(ln, 50.0) == pytest.approx(25.0)
    assert levels.lane_z(ln, 500.0) == pytest.approx(30.0)
    flat = types.SimpleNamespace(getShape3D=lambda: [(0.0, 0.0), (10.0, 0.0)])
    assert levels.lane_z(flat, 5.0) == 0.0
    assert levels.lane_z(object(), 5.0) == 0.0


def test_level_keys_are_relative_to_the_best_height():
    assert levels.level_keys([(1.0, 0.0), (2.0, 20.0)], None) == [1.0, 2.0]
    assert levels.level_keys([(1.0, 0.0), (2.0, 20.0)], 19.0) == [pytest.approx(19.0), pytest.approx(2.0)]
    # a net without heights on a hilly map: every lane 7 m off, so the ranking is the 2D one
    assert levels.level_keys([(1.0, 0.0), (2.0, 0.0)], 7.0) == [1.0, 2.0]


def test_pick_lane_keeps_the_cars_level_on_a_stacked_road():
    sd = pytest.importorskip("coupledcarla.sumodriver")
    street = _Lane3("street", 1.0, 10.0)          # 1 m from the point in 2D
    flyover = _Lane3("flyover", 2.5, 30.0)        # 2.5 m: right above the street
    net = types.SimpleNamespace(getNeighboringLanes=lambda x, y, r: [(street, 1.0), (flyover, 2.5)])
    assert sd.pick_lane(net, 10.0, 0.0, yaw=None)[0] is street                   # no height: the nearest in 2D
    assert sd.pick_lane(net, 10.0, 0.0, yaw=None, z=31.0)[0] is flyover          # a car up on the flyover
    assert sd.pick_lane(net, 10.0, 0.0, yaw=None, z=11.0)[0] is street


def test_tipped_and_off_level():
    assert not levels.tipped(5.0, -3.0)
    assert levels.tipped(45.0, 0.0) and levels.tipped(0.0, -55.0) and levels.tipped(0.0, 300.0)
    assert not levels.tipped(0.0, 359.0)
    assert levels.off_level(24.0, 18.4) and not levels.off_level(19.0, 18.4)
    assert not levels.off_level(24.0, None)


def test_heights_agree():
    assert levels.heights_agree([0.0] * 150 + [0.4] * 40 + [2.9] * 10)
    assert not levels.heights_agree([0.1] * 10)                       # too few points to tell
    assert not levels.heights_agree([12.0] * 100)                     # a flat net on a hilly map
    assert not levels.heights_agree([0.1] * 60 + [3.0] * 40)          # every 2.5th point a level off


def test_road_height_only_under_the_point():
    wp = types.SimpleNamespace(transform=types.SimpleNamespace(location=types.SimpleNamespace(x=1.0, y=1.0, z=7.0)))
    here = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    assert levels.road_height(wp, here) == 7.0
    assert levels.road_height(wp, types.SimpleNamespace(x=9.0, y=0.0, z=0.0)) is None
    assert levels.road_height(None, here) is None


def test_follow_puts_a_car_back_onto_its_level_or_upright():
    base = dict(target=(10.0, 0.0, 0.0), current=(10.0, 0.0, 0.0), speed=10.0, yaw_rate_deg=0.0, wheelbase=2.8)
    f = animate.follow(**base)
    assert not f["teleport"] and f["why"] is None
    f = animate.follow(**base, dz=-5.6)                  # knocked 5.6 m up over the street (Shibuya, CARLA 1.0)
    assert f["teleport"] and f["why"] == "level"
    assert not animate.follow(**base, dz=-1.0)["teleport"]
    f = animate.follow(**base, tilt=(23.0, 55.0))
    assert f["teleport"] and f["why"] == "tipped"
    assert animate.follow(**dict(base, current=(0.0, 0.0, 0.0)))["why"] == "position"


def test_sample_lanes_mid_points():
    lanes = [_Lane3("a", 0.0, 10.0), _Lane3("b", 5.0, 0.0)]
    edges = [types.SimpleNamespace(getFunction=lambda: "", getLanes=lambda ln=ln: [ln]) for ln in lanes]
    net = types.SimpleNamespace(getEdges=lambda: edges)
    pts = levels.sample_lanes(net, 10)
    assert pts == [(50.0, 0.0, pytest.approx(15.0)), (50.0, 5.0, pytest.approx(5.0))]
    assert math.isclose(pts[0][2], 15.0)
