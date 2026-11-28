"""coupledcarla.farring: what carla-digitaltwins' far_ring tool needs from a generated map, and the run-time spawn
that falls back to the paving slab when the server has no ring cooked in."""
import json
import types

from coupledcarla import farring

XODR = """<?xml version='1.0' encoding='UTF-8'?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="" north="2111.62" south="0.00" east="1722.02" west="0.00">
        <geoReference>
 +proj=utm +zone=31 +ellps=WGS84 +datum=WGS84 +units=m +no_defs
        </geoReference>
        <offset x="641679.85" y="5067593.55" z="-0.00" hdg="0" />
    </header>
</OpenDRIVE>
"""


def make_map(tmp_path, planes=None):
    m = tmp_path / "lyon"
    m.mkdir()
    (m / "map.xodr").write_text(XODR)
    planes = planes if planes is not None else [[10.0, -10.0, 20.0, 5.0, 0.0, 0.0], [30.0, -10.0, 20.0, 6.0, 0.1, 0.0],
                                                [10.0, -30.0, 20.0, 7.0, 0.0, 0.0], [30.0, -30.0, 20.0, 8.0]]
    (m / "scenery.json").write_text(json.dumps({"ground_planes": planes}))
    (m / "meta.json").write_text(json.dumps({"elevation": {"base_m": 158.0}, "speed_defaults": {"country": "FR"}}))
    return m


def test_ring_args_from_a_generated_map(tmp_path):
    a = farring.ring_args(make_map(tmp_path))
    assert a["name"] == "Lyon" and a["country"] == "FR" and a["z_offset"] == -158.0
    assert a["georef"].startswith("+proj=utm +zone=31") and a["offset"] == (641679.85, 5067593.55)
    # CARLA y points south: the tiles at y = -10 and -30 lie north of the origin (model y 0..40)
    assert a["inner"] == (0.0, 0.0, 40.0, 40.0)
    assert any(p[:2] == [40.0, 20.0] and abs(p[2] - 7.0) < 1e-6 for p in a["seam_points"])   # slope zx on tile 2
    (tmp_path / "x").mkdir()
    assert farring.ring_args(make_map(tmp_path / "x", planes=[])) is None


def test_ring_name():
    assert farring.ring_name("maps/eindhoven_centre") == "EindhovenCentre"
    assert farring.ring_name("new-york midtown") == "NewYorkMidtown"


class FakeBP:
    def __init__(self):
        self.attrs = {}

    def set_attribute(self, k, v):
        self.attrs[k] = v


def fake_world(cooked):
    bp = FakeBP()
    spawned = []

    def try_spawn_actor(b, tf):
        if cooked and "/FarRing/Lyon/" in b.attrs["mesh_path"]:
            spawned.append(b.attrs["mesh_path"])
            return object()
        return None
    lib = types.SimpleNamespace(find=lambda name: bp)
    return types.SimpleNamespace(get_blueprint_library=lambda: lib, try_spawn_actor=try_spawn_actor), spawned


CARLA = types.SimpleNamespace(Transform=lambda loc: loc, Location=lambda x, y, z: (x, y, z))


def test_spawn_uses_the_cooked_ring_else_nothing(tmp_path):
    m = make_map(tmp_path)
    w, spawned = fake_world(True)
    assert farring.spawn(w, m, CARLA, log=lambda s: None) == 0           # no far_ring.json: the slab
    (m / "far_ring").mkdir()
    (m / "far_ring" / "far_ring.json").write_text(json.dumps(
        {"name": "Lyon", "assets": [{"asset": "Lyon_FR_ground_mid", "semantic": "Terrain"},
                                    {"asset": "Lyon_FR_walls_n0_0_v1", "semantic": "Building"}]}))
    assert farring.spawn(w, m, CARLA, log=lambda s: None) == 2
    assert spawned[0] == "/Game/Carla/Static/Terrain/FarRing/Lyon/Lyon_FR_ground_mid.Lyon_FR_ground_mid"
    assert spawned[1].startswith("/Game/Carla/Static/Building/FarRing/Lyon/")
    w2, _ = fake_world(False)
    said = []
    assert farring.spawn(w2, m, CARLA, log=said.append) == 0             # not cooked: the caller lays its slab
    assert "not on this server" in said[0]
