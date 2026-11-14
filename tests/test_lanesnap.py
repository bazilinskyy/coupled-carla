"""coupledcarla.lanesnap: SUMO's convertRoad + convert2D in Python (checked against SUMO 1.28 on 104,000 points of
eleven maps: the same point to 1e-13 m everywhere; here the rules on a hand-made net)."""
import math

from pytest import approx

from coupledcarla.lanesnap import ASK, LaneIndex

NET = """<net>
  <edge id="main">
    <lane id="main_0" index="0" speed="13.9" length="100.00" width="3.20" shape="0.00,0.00 100.00,0.00"/>
    <lane id="main_1" index="1" speed="13.9" length="100.00" width="3.20" shape="0.00,3.20 100.00,3.20"/>
  </edge>
  <edge id="bike">
    <lane id="bike_0" index="0" allow="bicycle" speed="5" length="100.00" width="2.00" shape="0.00,-2.60 100.00,-2.60"/>
  </edge>
  <edge id=":j_0" function="internal">
    <lane id=":j_0_0" index="0" speed="8" length="20.00" width="3.20" shape="40.00,-10.00 40.00,10.00"/>
  </edge>
  <edge id="hill">
    <lane id="hill_0" index="0" speed="13.9" length="60.00" width="3.20" shape="200.00,0.00,0.00 230.00,0.00,30.00"/>
  </edge>
</net>
"""


def index(tmp_path):
    f = tmp_path / "map.net.xml"
    f.write_text(NET)
    return LaneIndex(f)


def test_point_beside_a_lane_snaps_to_its_centre_line(tmp_path):
    idx = index(tmp_path)
    assert len(idx.lanes) == 4                      # the bike lane is not for cars
    assert idx.nearest_centre(10.0, 0.4) == approx((10.0, 0.0))
    assert idx.nearest_centre(10.0, 3.0) == approx((10.0, 3.2))
    # on the bike lane: the nearest car lane (main_0, 1.0 m beyond its edge) rather than nothing
    assert idx.nearest_centre(10.0, -2.6) == approx((10.0, 0.0))


def test_on_several_lanes_the_smallest_id_wins(tmp_path):
    idx = index(tmp_path)
    # (40, 1.0) lies on main_0 (1.0 m from its centre) and on the junction lane :j_0_0 (on its centre): both
    # count 0 once less half the width, and ":j_0_0" < "main_0"
    assert idx.lanes[idx.lane_at(40.0, 1.0)][0] == ":j_0_0"
    assert idx.nearest_centre(40.0, 1.0) == approx((40.0, 1.0))


def test_lane_end_is_left_to_sumo_and_the_offset_walks_in_3d(tmp_path):
    idx = index(tmp_path)
    assert idx.nearest_centre(105.0, 0.5) is ASK       # beyond main's end: convert2D's rounding decides
    # hill: 2D offset 15 m along a shape 42.43 m long in 3D, of a lane 60 m long: SUMO takes the offset as
    # geometry offset (via the lane position and back) and walks it along the 3D shape
    x, y = idx.nearest_centre(215.0, 0.5)
    assert y == 0.0 and math.isclose(x, 200.0 + 15.0 / math.sqrt(2.0), rel_tol=1e-12)
