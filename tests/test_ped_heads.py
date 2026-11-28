"""Pedestrian heads at the signalised crossings: both ends, on the kerb, facing across (pedsignals.crossing_heads);
the bar and pedestrian pictograms and head meshes (no CARLA, no SUMO)."""
import json
import math
from pathlib import Path

import pytest

from coupledcarla import pedsignals, signalfaces as sf

ROOT = Path(__file__).resolve().parent.parent


def _road(x, y):
    return abs(y) <= 3.5 and -100 <= x <= 100          # a two-lane road along x


CROSSING = [(48.0, -4.0), (52.0, -4.0), (52.0, 4.0), (48.0, 4.0)]


def test_heads_at_both_ends_facing_across():
    links = [("tl1", 0, (47.0, -1.75), (1.0, 0.0)), ("tl1", 1, (53.0, 1.75), (-1.0, 0.0))]
    heads = pedsignals.crossing_heads([CROSSING], links, _road)
    assert len(heads) == 2
    for x, y, yaw, ctl in heads:
        assert x == pytest.approx(50.0) and 3.5 < abs(y) <= 3.5 + 0.25 + pedsignals.HEAD_BEYOND_M + 1e-6
        assert not _road(x, y)                                         # on the kerb, not the road
        # lenses face away from the travel yaw: back across the road, towards the people at the other end
        face = (-math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)))
        assert face[1] * y < 0 and abs(face[0]) < 1e-9
        assert ctl == [("tl1", 0), ("tl1", 1)]


def test_unsignalised_crossings_get_none():
    assert pedsignals.crossing_heads([CROSSING], [], _road) == []
    far = [("tl1", 0, (10.0, -1.75), (1.0, 0.0))]                        # its stop line 40 m away
    assert pedsignals.crossing_heads([CROSSING], far, _road) == []
    side = [("tl1", 0, (50.0, -10.0), (0.0, 1.0))]                       # a stream along the crossing's line
    heads = pedsignals.crossing_heads([CROSSING], side, _road)
    assert heads == []                      # its own lane runs along the crossing, so it does not control it


def test_overlapping_crosswalks_share_heads():
    links = [("tl1", 0, (47.0, -1.75), (1.0, 0.0))]
    twice = [CROSSING, [(x + 0.3, y) for x, y in CROSSING]]
    assert len(pedsignals.crossing_heads(twice, links, _road)) == 2


def test_bar_signal_pictograms():
    assert sf.faces_for("bus", 3) == ["bar_stop", "bar_dot", "bar_go"]
    st = sf.covered(sf.pictogram_lines((0, 0, 0), 0.0, 0.2, "bar_stop"), (0, 0, 0), 0.0, 0.002)
    go = sf.covered(sf.pictogram_lines((0, 0, 0), 0.0, 0.2, "bar_go"), (0, 0, 0), 0.0, 0.002)
    width = lambda cells, k: max(c[k] for c in cells) - min(c[k] for c in cells)   # noqa: E731
    assert width(st, 0) > 2 * width(st, 1) and width(go, 1) > 2 * width(go, 0)      # horizontal, vertical bar


def test_pedestrian_and_bus_head_meshes():
    meshes = json.loads((ROOT / "coupledcarla" / "data" / "street_props.json").read_text())["meshes"]
    by = {(v["region"], v["role"]): v for v in meshes.values()}
    for region in ("NL", "UK", "JP", "Any"):
        ped = by[(region, "signal_ped")]
        assert ped["faces"] == ["ped_stand", "ped_walk"] and len(ped["lenses"]) == 2, region
        assert (region, "signal_pole") in by, region
    bus = by[("NL", "signal_bus")]
    assert bus["faces"] == ["bar_stop", "bar_dot", "bar_go"] and set(bus["slots"]) >= {"LensWhite", "SignalBlack"}
