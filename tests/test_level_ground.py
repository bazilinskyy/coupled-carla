"""geo.level_ground / pick_ground: the ground under a point is the level's own surface found by a vertical ray (one
surface: that one; a bridge: the one nearest the lane height), on a map far above sea level too; and no code asks
CARLA for the nearest lane at z = 0 any more (realism pilot round 41b: 21 of SalamancaPlaza's 39 street cameras
stood 2-9 m under the level's ground, placed on a lower street that a lookup at z = 0 returned)."""
import math
import re
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def fake_carla(monkeypatch):
    m = types.ModuleType("carla")

    class Location:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    m.Location = Location
    m.LaneType = types.SimpleNamespace(Driving=1, Any=-1, Sidewalk=2)
    monkeypatch.setitem(sys.modules, "carla", m)
    return m


class Hillside:
    """Salamanca-like: the street at the point is at 793 m (lane 1 m away in plan); a lower street at 788 m runs 60 m
    away. CARLA measures the lookup distance in 3D, so asked at z = 0 it returns the lower street."""

    def __init__(self, carla):
        self.carla = carla
        self.lanes = [("near", 1.0, 793.0), ("far", 60.0, 788.0)]

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        name, d, z = min(self.lanes, key=lambda l: math.hypot(l[1], l[2] - loc.z))
        return types.SimpleNamespace(name=name, transform=types.SimpleNamespace(
            location=self.carla.Location(loc.x + d, loc.y, z)))


def hit(carla, z, label):
    return types.SimpleNamespace(location=carla.Location(0.0, 0.0, z), label=f"CityObjectLabel.{label}")


def test_pick_ground_layers():
    from coupledcarla.geo import pick_ground
    assert pick_ground([], 10.0) is None
    assert pick_ground([(20.0, "buildings"), (12.0, "vegetation"), (5.0, "car")], 10.0) is None
    # road and its kerb (15 cm) are one surface, at its top; one surface wins however far from the lane height
    assert pick_ground([(793.0, "roads"), (793.15, "sidewalks")], 788.0) == pytest.approx(793.15)
    # a roof and a tree over the street are not ground
    assert pick_ground([(812.0, "buildings"), (800.0, "vegetation"), (793.2, "terrain")], 788.0) == 793.2
    # an expressway deck over a street: the surface nearest the lane height
    deck = [(39.6, "roads"), (25.4, "roads"), (25.55, "sidewalks")]
    assert pick_ground(deck, 26.0) == pytest.approx(25.55)
    assert pick_ground(deck, 39.0) == pytest.approx(39.6)


def test_hit_label():
    from coupledcarla.geo import hit_label
    assert hit_label(types.SimpleNamespace(label="CityObjectLabel.Sidewalks")) == "sidewalks"
    assert hit_label(types.SimpleNamespace(label="Roads")) == "roads"


def test_level_ground_on_a_hillside(fake_carla):
    from coupledcarla.geo import level_ground
    cmap = Hillside(fake_carla)
    assert cmap.get_waypoint(fake_carla.Location(0, 0, 0)).name == "far"      # the old lookup's answer

    class World:
        def __init__(self):
            self.rays = []

        def cast_ray(self, a, b):
            self.rays.append((a.z, b.z))
            return [hit(fake_carla, 812.0, "Buildings"), hit(fake_carla, 793.3, "Ground"),
                    hit(fake_carla, 793.15, "Sidewalks")]
    w = World()
    z, src, lane = level_ground(w, cmap, 0.0, 0.0)
    assert (src, lane) == ("ray", 793.0) and z == pytest.approx(793.3)
    top, bottom = w.rays[0]
    assert top > 793.0 + 20 and bottom < 793.0 - 20        # the ray spans the lane height both ways

    class OldWorld:        # no cast_ray: ground_projection under the (right) lane height
        def ground_projection(self, loc, dist):
            assert loc.z == pytest.approx(794.5)
            return types.SimpleNamespace(location=fake_carla.Location(loc.x, loc.y, 793.1))
    assert level_ground(OldWorld(), cmap, 0.0, 0.0) == (pytest.approx(793.1), "projection", 793.0)

    class Streaming:       # nothing streamed in yet: the lane height, never the lower street's
        def cast_ray(self, a, b):
            return []

        def ground_projection(self, loc, dist):
            return None
    assert level_ground(Streaming(), cmap, 0.0, 0.0) == (793.0, "lane", 793.0)


ZLESS = re.compile(r"get_waypoint\(\s*(?:carla|c)\.Location\(\s*(?:x=)?[^,()]+,\s*(?:y=)?[^,()]+?"
                   r"(?:,\s*0(?:\.0)?)?\s*\)\s*,\s*project_to_road=True")


def test_no_lane_lookup_at_sea_level():
    """Every nearest-lane lookup from a plan position goes through geo.nearest_waypoint (at the road's height)."""
    bad = []
    for folder in ("coupledcarla", "tools"):
        for f in sorted((ROOT / folder).rglob("*.py")):
            text = f.read_text(encoding="utf-8", errors="replace")
            for m in ZLESS.finditer(text):
                bad.append(f"{f.relative_to(ROOT)}:{text[:m.start()].count(chr(10)) + 1}")
    assert not bad, "lane lookups at z = 0 (use geo.nearest_waypoint): " + ", ".join(bad)


def test_pattern_catches_the_old_forms():
    for src in ("cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True)",
                "self.cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Any)",
                "cmap.get_waypoint(carla.Location(x=x, y=-y), project_to_road=True,\n lane_type=LT.Sidewalk)",
                "cmap.get_waypoint(carla.Location(x, y, 0.0), project_to_road=True)",
                "self.cmap.get_waypoint(c.Location(x=x, y=y), project_to_road=True, lane_type=c.LaneType.Driving)"):
        assert ZLESS.search(src), src
    for src in ("cmap.get_waypoint(carla.Location(x=x, y=y, z=z0), project_to_road=True)",
                "cmap.get_waypoint(loc, project_to_road=True)",
                "cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=False, lane_type=t)"):
        assert not ZLESS.search(src), src
