"""tools/snap_scenery.py: the level's ground under each object, with a fake level (no server) and from an export."""
import json
import struct

import numpy as np

from tools import snap_scenery as ss


class _Cat:
    """scenery.Catalog's lookups: a 1 m cube pivoted at its bottom, and a 12 m tall tree."""
    by_name = {"Cube": (0, {"off": [0, 0, 0.5], "ext": [0.5, 0.5, 0.5]}),
               "Tree": (1, {"off": [0, 0, 6.0], "ext": [3.0, 3.0, 6.0]})}

    def __contains__(self, name):
        return name in self.by_name


def _data(items):
    return {"meshes": ["/Game/X/Cube.Cube", "/Game/X/Tree.Tree"], "items": items}


def _level(surfaces):
    """surfaces: [(x0, x1, z, label)] spanning every y: a ray returns those above x, top first."""
    def ray(x, y, z0, z1):
        return sorted(((z, lb) for a, b, z, lb in surfaces if a <= x < b and min(z0, z1) <= z <= max(z0, z1)),
                      reverse=True)
    return ray


def test_one_ground_surface_is_where_it_goes_however_far():
    # the terrain model put these 5 m too high, and their neighbours are on the ground already: the area offset is 0,
    # so before they were left floating; with one surface under them there is nothing to be unsure about
    items = [[0, x, 0.0, 0.0, 0, 1.0, "furniture"] for x in range(0, 40, 2)] + \
            [[0, 50.0 + x, 0.0, 5.0, 0, 1.0, "furniture"] for x in range(3)]
    d = _data(items)
    res = ss.snap_items(d, _Cat(), _level([(-10, 100, 0.0, "sidewalks")]))
    assert [it[3] for it in d["items"][-3:]] == [0.0, 0.0, 0.0]
    assert res["far_on_the_only_ground"] == 3 and res["left_alone"] == 0


def test_under_a_bridge_stays_under_it_and_unsure_ones_are_left_alone():
    # a road at 0 and a deck at 8 m over x 10-20: an object planned at 0.3 goes to the road under the deck
    lvl = _level([(-10, 100, 0.0, "roads"), (10, 20, 8.0, "roads")])
    d = _data([[0, 15.0, 0.0, 0.3, 0, 1.0, "lamp"], [0, 16.0, 0.0, 7.6, 0, 1.0, "lamp"]])
    ss.snap_items(d, _Cat(), lvl)
    assert [it[3] for it in d["items"]] == [0.0, 8.0]
    # planned 4.5 m between the two with no neighbours to say which: neither is within max_shift of the area's offset
    d = _data([[0, 15.0, 0.0, 4.5, 0, 1.0, "lamp"]] + [[0, 30.0 + i, 0.0, 9.0, 0, 1.0, "lamp"] for i in range(5)])
    res = ss.snap_items(d, _Cat(), lvl)
    assert d["items"][0][3] == 4.5 and res["left_alone"] == 1


def test_neighbours_with_one_surface_give_a_deck_object_its_offset():
    # the whole area is planned 6 m too high; the object over the deck takes its neighbours' -6 m: the road, not
    # the deck 2 m under its planned height
    lvl = _level([(-10, 100, 0.0, "roads"), (10, 20, 4.0, "roads")])
    d = _data([[0, 15.0, 0.0, 6.0, 0, 1.0, "furniture"]] + [[0, 22.0 + i, 0.0, 6.0, 0, 1.0, "tree"]
                                                               for i in range(5)])
    ss.snap_items(d, _Cat(), lvl)
    assert d["items"][0][3] == 0.0


def test_a_tree_under_a_building_is_left_out_a_bench_stays():
    lvl = _level([(-10, 100, 0.0, "sidewalks"), (5, 10, 4.0, "buildings")])      # an arcade 4 m high
    d = _data([[1, 7.0, 0.0, 0.0, 0, 1.0, "tree"], [0, 8.0, 0.0, 0.0, 0, 1.0, "furniture"],
               [1, 30.0, 0.0, 0.0, 0, 1.0, "tree"]])
    res = ss.snap_items(d, _Cat(), lvl)
    assert res["under_a_building_left_out"] == 1
    assert [(it[1], it[6]) for it in d["items"]] == [(8.0, "furniture"), (30.0, "tree")]


def test_flat_things_keep_their_kinds_lift_and_signs_stay():
    lvl = _level([(-10, 100, 0.0, "ground")])
    d = _data([[0, float(x), 0.0, 0.1, 0, 1.0, "leaves"] for x in range(5)] +       # on the ground, 0.1 over it
              [[0, 10.0, 0.0, 0.6, 0, 1.0, "leaves"], [0, 11.0, 0.0, 0.4, 0, 1.0, "wear"],
               [0, 12.0, 0.0, 0.05, 0, 1.0, "wear"], [0, 13.0, 0.0, 1.5, 0, 1.0, "sign"]])
    res = ss.snap_items(d, _Cat(), lvl)
    z = {(it[1], it[6]): it[3] for it in d["items"]}
    assert z[(10.0, "leaves")] == 0.1 and z[(11.0, "wear")] == 0.05    # floating: down, at the kind's lift
    assert z[(12.0, "wear")] == 0.05 and z[(0.0, "leaves")] == 0.1     # already on the ground: untouched
    assert z[(13.0, "sign")] == 1.5 and res["flat_items_moved"] == 2


def test_past_the_level_and_rooftop_units():
    lvl = _level([(-10, 50, 0.0, "roads"), (20, 30, 12.0, "buildings")])
    d = _data([[0, 25.0, 0.0, 20.0, 0, 1.0, "roof"], [0, 40.0, 0.0, 20.0, 0, 1.0, "roof"],
               [0, 60.0, 0.0, 0.0, 0, 1.0, "tree"], [0, 61.0, 0.0, 0.0, 0, 1.0, "sign"]])
    res = ss.snap_items(d, _Cat(), lvl)
    assert [(it[1], it[3]) for it in d["items"]] == [(25.0, 12.0)]
    assert res["roof_units_left_out"] == 1 and res["past_the_level_left_out"] == 2


def _glb(path, tris):
    """A GLB of triangles given in CARLA's frame, as the twin export writes them (glTF = x, z, y)."""
    p = np.asarray(tris, np.float32).reshape(-1, 3)[:, [0, 2, 1]].copy()
    idx = np.arange(len(p), dtype=np.uint32)
    blob = p.tobytes() + idx.tobytes()
    g = {"asset": {"version": "2.0"}, "buffers": [{"byteLength": len(blob)}],
         "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": p.nbytes},
                         {"buffer": 0, "byteOffset": p.nbytes, "byteLength": idx.nbytes}],
         "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(p), "type": "VEC3"},
                       {"bufferView": 1, "componentType": 5125, "count": len(idx), "type": "SCALAR"}],
         "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}]}
    j = json.dumps(g).encode()
    j += b" " * (-len(j) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(j) + 8 + len(blob))
    out += struct.pack("<II", len(j), 0x4E4F534A) + j + struct.pack("<II", len(blob), 0x004E4942) + blob
    path.write_bytes(out)


def test_export_ground_answers_like_the_level(tmp_path):
    (tmp_path / "meshes").mkdir()
    sq = lambda x0, x1, y0, y1, z: [[(x0, y0, z), (x1, y0, z), (x1, y1, z)], [(x0, y0, z), (x1, y1, z), (x0, y1, z)]]
    _glb(tmp_path / "meshes/road.glb", sq(0, 10, -10, 10, 2.0))
    _glb(tmp_path / "meshes/plane.glb", sq(-50, 50, -50, 50, 1.5))
    _glb(tmp_path / "meshes/house.glb", sq(4, 6, 4, 6, 9.0))
    (tmp_path / "manifest.json").write_text(json.dumps({
        "kinds": {"drivable": {"semantic": "Road"}},
        "assets": [{"file": "meshes/road.glb", "kind": "drivable"},
                   {"file": "meshes/plane.glb", "kind": "groundplane", "semantic": "Static"},
                   {"file": "meshes/house.glb", "kind": "building", "semantic": "Building"}]}))
    g = ss.ExportGround(tmp_path)
    assert g.ray(5.0, 5.0, 100.0, -100.0) == [(9.0, "buildings"), (2.0, "roads"), (1.5, "ground")]
    assert g.ray(5.0, -5.0, 1.8, -100.0) == [(1.5, "ground")]      # only within the ray's span
    assert g.ray(80.0, 0.0, 100.0, -100.0) == []                   # past the level's edge
