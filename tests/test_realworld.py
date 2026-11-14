"""Sun position, weather mapping, terrain sampling and building fitting (no CARLA, no network)."""
import math
from datetime import datetime, timezone

import numpy as np
import pytest

from coupledcarla import realworld as rw


def test_sun_at_solar_noon_midsummer():
    # Eindhoven, 21 June 2026, solar noon is about 11:38 UTC: the sun due south at 90 - 51.44 + 23.44 deg
    alt, az = rw.sun_position(51.44, 5.48, datetime(2026, 6, 21, 11, 38, tzinfo=timezone.utc))
    assert alt == pytest.approx(62.0, abs=0.4)
    assert az == pytest.approx(180.0, abs=2.0)


def test_sun_morning_east_and_night():
    alt, az = rw.sun_position(51.44, 5.48, datetime(2026, 3, 20, 7, 0, tzinfo=timezone.utc))
    assert 5 < alt < 15 and 95 < az < 115          # equinox morning: low in the east
    alt, _ = rw.sun_position(51.44, 5.48, datetime(2026, 12, 21, 23, 0, tzinfo=timezone.utc))
    assert alt < -50
    alt, az = rw.sun_position(-33.87, 151.21, datetime(2026, 6, 21, 2, 0, tzinfo=timezone.utc))
    assert az == pytest.approx(0.0, abs=15) or az > 345   # Sydney's winter noon sun stands in the north


def test_weather_mapping():
    f = rw.carla_weather_fields({"cloud_cover": 80, "rain": 2.5, "precipitation_3h": 4.0, "wind_speed_10m": 20,
                                 "visibility": 24000}, 30.0, 180.0)
    assert f["cloudiness"] == pytest.approx(48) and f["precipitation"] == 50 and f["precipitation_deposits"] == 60
    assert f["wind_intensity"] == 40 and f["fog_density"] == 0
    assert f["sun_azimuth_angle"] == 270.0          # sun in the south: light travels north (-y), yaw 270
    fog = rw.carla_weather_fields({"visibility": 500}, 10, 90)
    assert fog["fog_density"] > 80 and fog["fog_distance"] == 25


def test_dem_merc_bilinear():
    # a ramp east-west over one zoom-15 tile: elevation follows longitude linearly
    z = np.tile(np.arange(256, dtype=np.float64), (256, 1))
    n = 2 ** 15
    tx, ty = 16882, 10928
    d = rw.Dem(z, "merc", {"zoom": 15, "x0": tx * 256, "y0": ty * 256})
    lon_mid = (tx + 0.5) / n * 360 - 180
    lat_mid = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (ty + 0.5) / n))))
    assert d.sample(lat_mid, lon_mid) == pytest.approx(127.5, abs=0.6)
    assert d.sample(np.array([lat_mid]), np.array([lon_mid]))[0] == pytest.approx(127.5, abs=0.6)


def test_fill_holes_and_blur():
    z = np.ones((20, 20)) * 5.0
    z[5:9, 5:9] = np.nan
    f = rw._fill_holes(z)
    assert not np.isnan(f).any() and np.allclose(f, 5.0)
    assert np.allclose(rw._box_blur(np.ones((10, 10)), 2), 1.0)


def test_netherlands_test():
    assert rw.in_netherlands([51.436, 5.47, 51.45, 5.493])        # Eindhoven
    assert not rw.in_netherlands([50.872, 4.69, 50.885, 4.71])     # Leuven
    assert not rw.in_netherlands([45.75, 4.82, 45.76, 4.84])       # Lyon


shapely = pytest.importorskip("shapely")


class FakeCatalog:
    sizes = {"SM_House03": (17.2, 15.9, 8.8), "SM_TerracedHouse_1": (11.8, 10.5, 6.9),
             "SM_Block13NY_v04_Merged": (31.5, 17.0, 24.1), "SM_Skycraper04_2": (15.0, 15.0, 45.0),
             "SM_Factory_v02_Merged": (51.8, 28.4, 15.0)}

    def __contains__(self, name):
        return name in self.sizes

    def size(self, name):
        return self.sizes[name]

    def optimised(self, name):
        return name


def test_split_l_shape():
    from shapely.geometry import Polygon
    from coupledcarla.scenery import split_footprint
    l_shape = Polygon([(0, 0), (40, 0), (40, 10), (10, 10), (10, 40), (0, 40)])
    rects = split_footprint(l_shape)
    assert len(rects) >= 2
    assert sum(r[2] * r[3] for r in rects) == pytest.approx(l_shape.area, rel=0.3)
    square = Polygon([(0, 0), (12, 0), (12, 10), (0, 10)])
    (cx, cy, length, width, _), = split_footprint(square)
    assert (cx, cy, length, width) == pytest.approx((6, 5, 12, 10))


def test_fit_prefers_kind_and_height():
    from coupledcarla.scenery import fit_building
    cat = FakeCatalog()
    house = fit_building(cat, (0, 0, 12, 10.5, 0), 7.0, ["house"], 1)
    assert house[0] == "SM_TerracedHouse_1" and house[1] == 1
    row = fit_building(cat, (0, 0, 48, 10.5, 0), 7.0, ["house"], 1)
    assert row[0] == "SM_TerracedHouse_1" and row[1] == 4          # a terrace of four
    tower = fit_building(cat, (0, 0, 15, 15, 0), 50.0, ["commercial"], 1)
    assert tower[0] == "SM_Skycraper04_2"


def test_place_top_anchor_with_tilt():
    from coupledcarla.scenery import Catalog
    cat = Catalog.__new__(Catalog)
    cat.by_name = {"Cube": ("/x.Cube", {"ext": [0.5, 0.5, 0.5], "off": [0.0, 0.0, 0.0]})}
    x, y, z, _ = cat.place("Cube", 10, 20, 5.0, 0.0, 40.0, anchor="top")
    assert (x, y, z) == pytest.approx((10, 20, -15.0))          # a 40 m cube sunk with its top at 5 m
    x, y, z, _ = cat.place("Cube", 10, 20, 5.0, 0.0, 40.0, pitch=10.0, anchor="top")
    # the top centre is 20 m from the pivot along the tilted up axis
    assert x == pytest.approx(10 + 20 * math.sin(math.radians(10)), abs=1e-6)
    assert z == pytest.approx(5 - 20 * math.cos(math.radians(10)), abs=1e-6)
