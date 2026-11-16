"""streetlife.py: bike variants and Swapfiets share, stone strips, context, bikes against facades and shop signs
facing the street (no network, no CARLA)."""
import collections
import math

import pytest

from coupledcarla import scenery, streetlife


@pytest.fixture(scope="module")
def cat10():
    return scenery.Catalog("1.0")


def test_bike_mix(cat10):
    names = [streetlife.bike_mesh(cat10, i, j) for i in range(60) for j in range(50)]
    assert all(n and n in cat10 for n in names)
    swap = sum("Swapfiets" in n for n in names) / len(names)
    assert 0.09 < swap < 0.15                                  # about 1 in 8
    men = sum("BikeMen_" in n for n in names) / len(names)
    assert 0.33 < men < 0.47
    black = sum(n.endswith("_Black") for n in names) / len(names)
    assert black > 0.33 and names == [streetlife.bike_mesh(cat10, i, j) for i in range(60) for j in range(50)]
    assert streetlife.bike_mesh(scenery.Catalog("0.9.16"), 1) is None


def test_stone_strip_sizes(cat10):
    assert streetlife.strip_mesh("clinker", 3.2, cat10) == ("SM_CC_NL_Clinker_W3", pytest.approx(1.067, abs=0.01))
    assert streetlife.strip_mesh("sett", 6.3, cat10)[0] == "SM_CC_Any_Sett_W6"
    assert streetlife.strip_mesh("clinker", 20.0, cat10)[1] == 1.25          # capped: the stones barely grow
    assert streetlife.strip_mesh("clinker", 3.0, scenery.Catalog("0.9.16")) is None


def test_context():
    near = streetlife.Near([(0.0, 0.0, "station", {}), (500.0, 0.0, "cafe", {}), (900.0, 0.0, "shop", {})])
    assert near.context(100.0, 0.0) == "station"
    assert near.context(520.0, 0.0) == "cafe" and near.context(905.0, 10.0) == "shop"
    assert near.context(700.0, 0.0) == "home"


class Placer:
    def __init__(self, cat):
        self.cat, self.items, self.bases, self.mesh_index = cat, [], [], {}
        self.counts = collections.defaultdict(int)
        self.ground = lambda x, y: 0.0

    def add(self, kind, mesh, cx, cy, yaw, scale, sink=0.0, tilt=None, z=None, **kw):
        idx = self.mesh_index.setdefault(mesh, len(self.mesh_index))
        it = [idx, cx, cy, z or 0.0, yaw, scale, kind]
        self.items.append(it)
        self.bases.append((cx, cy, 0.0))
        return it

    def on_lane(self, x, y, z=0.0, masks=None):
        return y < -6.0          # a road south of the building


def test_bikes_against_a_facade_lean_on_it(cat10):
    pytest.importorskip("shapely")
    from shapely.geometry import Polygon
    pl = Placer(cat10)
    shop = Polygon([(0, 0), (20, 0), (20, 10), (0, 10)])
    near = streetlife.Near([(10.0, -2.0, "station", {})])
    taken = streetlife.Taken(pl)

    def free(x, y, r=0.6, masks=None):
        return not shop.contains(__import__("shapely.geometry", fromlist=["Point"]).Point(x, y)) and y > -6.0
    got = streetlife.parked_bikes(pl, cat10, [{"poly": shop}], near, taken, free)
    assert got.get("facades", 0) >= 5
    for idx, x, y, z, yaw, s, kind in pl.items:
        assert not shop.contains(__import__("shapely.geometry", fromlist=["Point"]).Point(x, y))
        assert min(abs(y - 0) , abs(y - 10), abs(x - 0), abs(x - 20)) < 0.6        # along a wall
        # the wall is on the bike's left (Unreal: left = yaw - 90): stepping that way gets closer to the building
        lx, ly = x + math.cos(math.radians(yaw - 90)) * 0.5, y + math.sin(math.radians(yaw - 90)) * 0.5
        assert shop.exterior.distance(__import__("shapely.geometry", fromlist=["Point"]).Point(lx, ly)) < 0.45 or \
            shop.contains(__import__("shapely.geometry", fromlist=["Point"]).Point(lx, ly))


def test_shop_sign_faces_the_street(cat10):
    pytest.importorskip("shapely")
    from shapely.geometry import Polygon
    pl = Placer(cat10)
    shop = Polygon([(0, 0), (20, 0), (20, 10), (0, 10)])
    n = streetlife.shop_signs(pl, cat10, [{"poly": shop}], [(8.0, -3.0, "cafe", {"amenity": "cafe"}),
                                                            (9.0, -3.0, "shop", {"shop": "bakery"})])
    assert n == 1            # one sign per wall
    idx, x, y, z, yaw, s, kind = pl.items[0]
    assert abs(y) < 0.1 and 1.5 < x < 18.5 and z == pytest.approx(2.9)
    # the face is the mesh's Unreal -y, i.e. direction yaw - 90: south, out of the wall, towards the road
    fx, fy = math.cos(math.radians(yaw - 90)), math.sin(math.radians(yaw - 90))
    assert fy == pytest.approx(-1.0, abs=1e-6)
    assert [m for m, i in pl.mesh_index.items()] == ["SM_CC_NL_ShopSign_cafe"]
