"""Trees nobody maps: crowns of the canopy map get a tree where none stands near (canopy.crowns, scenery helpers)."""
import math

import numpy as np
import pytest

from coupledcarla import canopy, scenery


def _grid(z):
    x, y = canopy.mercator(0.0005, -0.0005)  # top left near the equator, 1 m pixels
    return canopy.Canopy(z, x, y, 1.0)


def test_latlon_inverts_mercator():
    for lat, lon in ((51.44, 5.48), (-36.85, 174.76), (0.0, 0.0)):
        assert canopy.latlon(*canopy.mercator(lat, lon)) == pytest.approx((lat, lon), abs=1e-9)


def test_crowns_one_per_block_tallest_first():
    z = np.zeros((100, 100), np.uint8)
    z[20:24, 20:24] = 8
    z[21, 22] = 14                          # the crown's top
    z[70:73, 60:63] = 5
    z[10, 90] = 2                           # under SEEN_M: no tree
    out = canopy.crowns(_grid(z), [-0.0005, -0.0005, 0.0005, 0.0005], block_m=5.0)
    assert all(c >= canopy.SEEN_M for _, _, c in out)
    assert out[0][2] == 14 and out[-1][2] == 5
    tops = [(lat, lon) for lat, lon, c in out if c == 14]
    lat, lon = tops[0]                       # at pixel (21, 22): 0.0005 - 21.5 m, -0.0005 + 22.5 m
    m_per_deg = 6378137.0 * math.pi / 180
    assert lat == pytest.approx(0.0005 - 21.5 / m_per_deg, abs=2e-7)
    assert lon == pytest.approx(-0.0005 + 22.5 / m_per_deg, abs=2e-7)
    assert len(out) <= 1 + 1 + 4 + 1         # a 4 x 4 crown spans at most 4 blocks, the 3 x 3 one up to 4


def test_crowns_clipped_to_the_box_and_empty_grid():
    z = np.full((100, 100), 10, np.uint8)
    g = _grid(z)
    small = [-0.0001, -0.0001, 0.0001, 0.0001]
    out = canopy.crowns(g, small, block_m=5.0)
    assert out and all(small[0] <= la <= small[2] and small[1] <= lo <= small[3] for la, lo, _ in out)
    assert canopy.crowns(canopy.Canopy(np.zeros((0, 0), np.uint8), 0, 0, 1.0), small) == []
    assert canopy.crowns(None, small) == []
    assert canopy.crowns(g, [10, 10, 11, 11]) == []          # the box is off the grid


def test_point_hash_and_family_by_height():
    h = scenery._PointHash(6.0, [(0.0, 0.0), (20.0, 20.0)])
    assert h.any_within(5.0, 0.0, 6.0) and not h.any_within(7.0, 0.0, 6.0)
    h.add(7.0, 0.0)
    assert h.any_within(12.5, 0.0, 6.0)
    assert scenery.canopy_family(22.0, 51.4) == ["oak", "beech", "ash"]
    assert "sassafras" in scenery.canopy_family(7.0, 51.4)
    assert scenery.canopy_family(14.0, 51.4) == scenery._tree_family({}, 51.4)
    assert scenery.canopy_family(22.0, 10.0) == scenery._tree_family({}, 10.0)     # tropics: the usual mix


def test_canopy_cover_counts_crowns_with_a_tree_near():
    z = np.zeros((100, 100), np.uint8)
    z[10:20, 10:20] = 10                     # two 10 x 10 m crowns
    z[60:70, 60:70] = 10
    g = _grid(z)
    bbox = [-0.0005, -0.0005, 0.0005, 0.0005]

    class Geo:                               # CARLA x, y = metres from the grid's top left
        m = 6378137.0 * math.pi / 180

        def to_carla(self, lat, lon):
            return (lon + 0.0005) * self.m, (0.0005 - lat) * self.m

    near_first = scenery._PointHash(6.0, [(15.0, 15.0)])
    pct = scenery.canopy_cover(g, bbox, Geo(), near_first, lambda x, y: False, radius=5.0)
    assert pct == pytest.approx(39, abs=4)  # the first crown's disc of 5 m (78.5 of its 100 m2) of 200 m2
    inside = lambda x, y: x > 50            # noqa: E731 - the second crown stands "in a building"
    assert scenery.canopy_cover(g, bbox, Geo(), near_first, inside) == pytest.approx(78.5, abs=4)
    assert scenery.canopy_cover(canopy.Canopy(np.zeros((0, 0), np.uint8), 0, 0, 1.0), bbox, Geo(),
                                near_first, lambda x, y: False) is None


def test_regional_lamps_have_lit_blueprints():
    from coupledcarla import season, streetlook
    lamps = [m for m, spec in streetlook.props().items() if spec.get("light")]
    assert lamps and all(season.LAMP_LIGHTS.get(m) == "blueprint.cc_" + m[6:].lower() for m in lamps)
    import json
    from pathlib import Path
    pack = json.loads((Path(season.PACKS) / "CoupledCarla" / "Content" / "Config" /
                       "BlueprintParameters.json").read_text())
    names = {b["Name"]: b["Path"] for b in pack["Blueprints"]}
    for m in lamps:
        bp = season.LAMP_LIGHTS[m].split(".", 1)[1]
        assert names[bp].endswith("BP_" + m[3:] + ".BP_" + m[3:])
