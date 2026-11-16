"""compat.road_lanes: every lane across a road, both directions."""


from coupledcarla import compat


class _Wp:
    def __init__(self, lane_id, kind="Driving"):
        self.road_id, self.section_id, self.lane_id, self.lane_type = 7, 0, lane_id, kind
        self.left = self.right = None

    def get_left_lane(self):
        return self.left

    def get_right_lane(self):
        return self.right


def _road(ids, kinds):
    wps = [_Wp(i, k) for i, k in zip(ids, kinds)]
    for a, b in zip(wps, wps[1:]):
        a.right, b.left = b, a
    return wps


def test_road_lanes_both_sides_and_kind():
    # left edge to right edge: sidewalk, two lanes one way, two the other, sidewalk
    road = _road([3, 2, 1, -1, -2, -3], ["Sidewalk", "Driving", "Driving", "Driving", "Driving", "Sidewalk"])
    got = compat.road_lanes(road[3])
    assert [w.lane_id for w in got] == [3, 2, 1, -1, -2, -3]
    assert [w.lane_id for w in compat.road_lanes(road[3], "Driving")] == [2, 1, -1, -2]
    loop = _road([1, -1], ["Driving", "Driving"])
    loop[1].right = loop[0]            # a lane that leads back to one already seen stops the walk
    assert [w.lane_id for w in compat.road_lanes(loop[0])] == [1, -1]
