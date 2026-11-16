"""geo.nearest_waypoint: the lane lookup at the road's own height, so maps far above sea level pick the nearest lane
in plan (a stand-in carla module and map that measure the distance in 3D, as CARLA does)."""
import math
import sys
import types

import pytest


@pytest.fixture
def fake_carla(monkeypatch):
    m = types.ModuleType("carla")

    class Location:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    m.Location = Location
    m.LaneType = types.SimpleNamespace(Driving=1, Any=-1)
    monkeypatch.setitem(sys.modules, "carla", m)
    try:
        import sumolib  # noqa: F401
    except ImportError:                    # geo imports sumolib for Geo (CI installs no SUMO); not used here
        monkeypatch.setitem(sys.modules, "sumolib", types.ModuleType("sumolib"))
    return m


class Map:
    """Two lanes: A 3 m from the point in plan at 402 m, B 12 m away at 396 m (a hillside, about 400 m up)."""

    def __init__(self, carla):
        self.lanes = [("A", 3.0, 402.0), ("B", 12.0, 396.0)]
        self.carla, self.asked = carla, []

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        self.asked.append(loc.z)
        name, d, z = min(self.lanes, key=lambda l: math.hypot(l[1], l[2] - loc.z))
        return types.SimpleNamespace(name=name, transform=types.SimpleNamespace(
            location=self.carla.Location(loc.x + d, loc.y, z)))


def test_lookup_at_road_height_picks_nearest_lane(fake_carla):
    from coupledcarla.geo import ground, nearest_waypoint
    cmap = Map(fake_carla)
    assert cmap.get_waypoint(fake_carla.Location(0, 0, 0)).name == "B"     # at z = 0 the lower lane wins
    cmap.asked.clear()
    wp = nearest_waypoint(cmap, 0.0, 0.0)
    assert wp.name == "A" and cmap.asked[0] == 0.0 and cmap.asked[1] == pytest.approx(396.0)
    assert ground(cmap, 0.0, 0.0).z == pytest.approx(402.0)
    assert nearest_waypoint(cmap, 0.0, 0.0, z=401.0).name == "A"              # a known height: one lookup


def test_sea_level_map_needs_one_extra_lookup(fake_carla):
    from coupledcarla.geo import nearest_waypoint
    cmap = Map(fake_carla)
    cmap.lanes = [("A", 3.0, 0.2), ("B", 12.0, 0.0)]
    assert nearest_waypoint(cmap, 0.0, 0.0).name == "A" and len(cmap.asked) == 2
