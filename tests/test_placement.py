"""placement.py: the OpenDRIVE CARLA gets, junction areas, sign countries and the audit's bookkeeping (no CARLA)."""
import math
from types import SimpleNamespace

import pytest

from coupledcarla import placement

XODR = """<OpenDRIVE>
    <road id="1">
        <signals>
            <signal id="sig0" name="sig0" s="37.4" t="-1.0" orientation="+" dynamic="yes" zOffset="5" type="1000001">
                <validity fromLane="-1" toLane="-1"/>
            </signal>
            <signal id="sign1" name="B6" s="10" t="-3" orientation="+" dynamic="no" type="205"/>
            <signalReference id="sig0" s="1" t="0" orientation="+"/>
        </signals>
    </road>
    <road id="2">
        <signals>
            <signal id="sig2" name="sig2" s="1" t="-1" dynamic="yes" zOffset="5" type="1000011"/>
        </signals>
    </road>
    <controller id="GS_1">
        <control signalId="sig0"/>
        <control signalId="sig2"/>
    </controller>
    <junction name="j" id="7">
        <connection id="0" incomingRoad="1" connectingRoad="3" contactPoint="start"/>
        <controller id="GS_1" type="0"/>
    </junction>
</OpenDRIVE>
"""


def test_carla_xodr_leaves_out_the_traffic_lights_only():
    text, n = placement.carla_xodr(XODR)
    assert n == 2
    assert 'id="sig0"' not in text and 'id="sig2"' not in text and "signalReference" not in text
    assert "<controller" not in text and "control signalId" not in text
    assert 'id="sign1"' in text and 'type="205"' in text           # a static sign stays
    assert text.count("<signals>") == 1                              # road 2's emptied block is gone
    assert '<connection id="0"' in text and "</junction>" in text
    assert placement.carla_xodr(XODR, keep_lights=True) == (XODR, 0)


def test_carla_xodr_on_a_real_netconvert_layout():
    sig = ('            <signal id="GS_9_{i}" name="GS_9_{i}" s="1" t="-1" orientation="+" dynamic="yes" zOffset="5" '
           'country="OpenDRIVE" type="1000001" subtype="-1" height="0.78" width="0.26">\n'
           '                <validity fromLane="-1" toLane="-1"/>\n            </signal>\n')
    text = ("<OpenDRIVE>\n    <road id=\"5\">\n        <signals>\n" + "".join(sig.format(i=i) for i in range(3))
            + "        </signals>\n    </road>\n    <controller id=\"GS_9\">\n"
            + "".join(f'        <control signalId="GS_9_{i}"/>\n' for i in range(3)) + "    </controller>\n</OpenDRIVE>\n")
    out, n = placement.carla_xodr(text)
    assert n == 3 and out == '<OpenDRIVE>\n    <road id="5">\n    </road>\n</OpenDRIVE>\n'


def test_junction_area_fills_the_island_between_its_lanes():
    pytest.importorskip("shapely")
    # four lanes round a 10 x 10 m block (a junction's turning lanes with an island in the middle)
    sq = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
    strips = [[(sq[i][0], sq[i][1], 1.5), (sq[i + 1][0], sq[i + 1][1], 1.5)] for i in range(4)]
    area = placement.junction_area(strips)
    assert area.contains(placement_point(5, 5))                 # the island is paved
    assert area.contains(placement_point(0, 5)) and not area.contains(placement_point(-4, 5))
    j = placement.JunctionAreas({7: area, 8: None})
    assert j.which(5, 5) == 7 and j.contains(10, 1) and not j.contains(30, 30)
    assert placement.JunctionAreas({}).which(0, 0) is None


def placement_point(x, y):
    from shapely.geometry import Point
    return Point(x, y)


class Wp:
    """A lane along +x; x < 0 is the approach, 0 <= x < 20 the junction."""
    def __init__(self, x):
        self.x = x
        self.is_junction = 0 <= x < 20

    def previous(self, d):
        return [Wp(self.x - d)] if self.x - d > -100 else []


def test_approach_steps_back_out_of_the_junction():
    assert placement.approach(Wp(12.0)).x < 0 and not placement.approach(Wp(12.0)).is_junction
    w = Wp(-5.0)
    assert placement.approach(w) is w
    assert placement.approach(None) is None
    far = Wp(15.0)
    assert placement.approach(far, reach=4.0) is far           # no lane leads in within reach: unchanged


def test_signs_only_in_their_own_country():
    assert placement.sign_fits("give_way", "USA") and not placement.sign_fits("give_way", "NLD")
    assert not placement.sign_fits("give_way", None)            # unknown: none rather than a US YIELD plate
    assert placement.sign_fits("stop", "NLD") and placement.sign_fits("stop", None)
    assert not placement.sign_fits("stop", "JPN")
    assert placement.sign_fits("speed", "FRA")


def test_classify_tilt_and_facing():
    assert placement.classify(0.05) == [] and placement.classify(0.4) == ["floating"]
    assert placement.classify(-0.5) == ["sunk"] and placement.classify(-0.2) == []
    assert placement.classify(0.0, 12.0) == ["tilted"] and placement.classify(0.0, 12.0, upright=False) == []
    assert placement.tilt_of(0, 0) == pytest.approx(0.0) and placement.tilt_of(10, 0) == pytest.approx(10.0)
    assert placement.tilt_of(0, -20) == pytest.approx(20.0)
    # traffic travels along +x (yaw 0): a face turned to yaw 180 looks at it
    assert placement.facing_error(180.0, 0.0) == pytest.approx(0.0)
    assert placement.facing_error(0.0, 0.0) == pytest.approx(180.0)
    assert placement.facing_error(-90.0, 0.0) == pytest.approx(90.0)
    assert placement.facing_error(170.0, 350.0) == pytest.approx(0.0)


def rec(kind, x, y, z=0.0, foot=None, mesh=None):
    return {"kind": kind, "x": x, "y": y, "z": z, "mesh": mesh, "foot": foot, "problems": [], "detail": {}}


def box(cx, cy, l, w):
    return [(cx - l / 2, cy - w / 2), (cx + l / 2, cy - w / 2), (cx + l / 2, cy + w / 2), (cx - l / 2, cy + w / 2)]


def test_near_pairs_and_duplicates():
    pts = [(0, 0), (1.0, 0), (5, 5), (5, 5.2, 0.0), (5, 5.1, 3.0)]
    pairs = {(i, j) for i, j, _ in placement.near_pairs(pts, 1.2)}
    assert (0, 1) in pairs and (2, 3) in pairs and (2, 4) in pairs      # 2-D points know no height
    assert (3, 4) not in pairs                                          # a second post on top of the first
    recs = [rec("lamp", 0, 0), rec("lamp", 1.0, 0), rec("lamp", 10, 0), rec("tree", 10.2, 0),
            rec("sign_post", 3, 3, 0.0), rec("sign_post", 3, 3, 2.3)]
    assert placement.duplicates(recs) == 1
    assert recs[1]["problems"] == ["duplicate"] and recs[1]["detail"]["duplicate_of"][1] == pytest.approx(1.0)
    assert not recs[3]["problems"] and not recs[5]["problems"]        # other kinds; the tall post's second piece


def test_overlaps_and_summary():
    pytest.importorskip("shapely")
    recs = [rec("parked", 0, 0, foot=box(0, 0, 4.4, 1.8)), rec("parked", 3.0, 0, foot=box(3.0, 0, 4.4, 1.8)),
            rec("parked", 20, 0, foot=box(20, 0, 4.4, 1.8)), rec("lamp", 20.5, 0, foot=box(20.5, 0, 0.8, 0.8)),
            rec("parked", 40, 0, foot=box(40, 0, 4.4, 1.8))]
    n = placement.overlaps(recs, {"parked"}, {"parked", "lamp"})
    assert n == 2
    assert recs[1]["problems"] == ["overlap"] and not recs[0]["problems"]      # a pair is marked once
    assert recs[2]["detail"]["overlap"][0] == "lamp" and not recs[4]["problems"]
    per, tot = placement.summary(recs)
    assert per["parked"] == {"n": 4, "overlap": 2} and per["lamp"] == {"n": 1} and tot == {"overlap": 2}
    recs[0]["problems"].append("floating")
    recs[0]["gap"] = 0.2
    recs[2]["problems"].append("floating")
    recs[2]["gap"] = 1.5
    assert [r["x"] for r in placement.worst(recs, "floating")] == [20, 0]


def test_ground_surface_and_settle():
    gs = placement.GroundSurface([(20.0, 20.0, 40.0, 5.0, 0.1, 0.0), (60.0, 20.0, 40.0, 3.0, 0.0, 0.0)])
    assert gs.z(20.0, 20.0) == pytest.approx(5.0) and gs.z(30.0, 25.0) == pytest.approx(6.0)
    assert gs.z(40.0, 10.0) == pytest.approx(7.0)            # on the edge between two tiles: the higher
    assert gs.z(70.0, 0.0) == pytest.approx(3.0) and gs.z(-5.0, 0.0) is None
    assert placement.GroundSurface(gs.to_list()).z(30.0, 25.0) == pytest.approx(6.0) and len(gs) == 2
    items = [[0, 20.0, 20.0, 5.5, 0, 1, "lamp"], [1, 21.0, 20.0, 7.2, 0, 1, "sign"], [2, 60.0, 20.0, 4.0, 0, 1, "tree"],
             [3, 60.0, 21.0, 4.0, 0, 1, "water"], [4, 61.0, 21.0, 4.0, 0, 1, "lamp"], [5, 0.0, -30, 1.0, 0, 1, "tree"]]
    # the lamp and the sign plate (on its post's base) were placed 0.5 m over the tile; the tree 1 m over; the
    # water tile is left alone; the second lamp stands on a lane; the last tree is off the tiles
    bases = [(20.0, 20.0, 5.5), (20.0, 20.0, 5.5), (60.0, 20.0, 4.0), (60.0, 21.0, 4.0), (61.0, 21.0, 4.0),
             (0.0, -30.0, 1.0)]
    moved, total = placement.settle(items, bases, gs, lambda x, y, z: x == 61.0)
    assert moved == 3 and total == pytest.approx(2.0)
    assert [it[3] for it in items] == [5.0, 6.7, 3.0, 4.0, 4.0, 1.0]


def test_ground_surface_from_scenery(tmp_path):
    import json
    f = tmp_path / "scenery.json"
    f.write_text(json.dumps({"items": [], "ground_planes": [[0, 0, 10, 2.0, 0, 0]]}))
    assert placement.ground_surface(f).z(1, 1) == pytest.approx(2.0)
    f.write_text(json.dumps({"items": []}))
    assert placement.ground_surface(f) is None and placement.ground_surface(tmp_path / "none.json") is None


def test_lane_points_catch_a_lane_between_sample_points():
    pytest.importorskip("shapely")
    # a 3.2 m lane running north-south through the middle of a 40 x 20 m building: the nine sample points of
    # the old check (corners, edge middles, centre at x = -20, 0, 20) miss it at x = 9
    lanes = placement.LanePoints([(9.0, y, 1.6) for y in range(-30, 31, 2)])
    assert lanes.hits(placement.rect(0.0, 0.0, 40.0, 20.0, 0.0))
    assert not lanes.hits(placement.rect(0.0, 0.0, 14.0, 20.0, 0.0))        # it ends at x = 7, 2 m from the lane
    assert lanes.hits(placement.rect(0.0, 0.0, 20.0, 14.0, 90.0)) is False      # turned: 14 m along x
    assert not placement.LanePoints([]).hits(placement.rect(0, 0, 5, 5, 0))
    r = placement.rect(1.0, 2.0, 4.0, 2.0, 90.0)
    assert r.area == pytest.approx(8.0) and r.bounds == pytest.approx((0.0, 0.0, 2.0, 4.0))


def test_buildings_go_down_to_their_lowest_corner():
    gs = placement.GroundSurface([(20.0, 20.0, 40.0, 5.0, 0.1, 0.0)])     # 3 m at x = 0, 7 m at x = 40
    items = [[0, 20.0, 20.0, 6.0, 0, 1, "building"], [1, 20.0, 20.0, 1.0, 0, 1, "building"]]
    bases = [(20.0, 20.0, 6.0, 20.0, 10.0, 0.0), (20.0, 20.0, 1.0, 20.0, 10.0, 0.0)]
    moved, _ = placement.settle(items, bases, gs, lambda x, y, z: False)
    # the first one's west corners stand on 4 m of ground: it goes down 2 m; the second is under the ground already
    assert moved == 1 and items[0][3] == pytest.approx(4.0) and items[1][3] == 1.0
    deep = [[0, 20.0, 20.0, 9.0, 0, 1, "landmark"]]
    placement.settle(deep, [(20.0, 20.0, 9.0, 20.0, 10.0, 0.0)], gs, lambda x, y, z: False)
    assert deep[0][3] == pytest.approx(9.0 - placement.BUILDING_DROP_MAX)
