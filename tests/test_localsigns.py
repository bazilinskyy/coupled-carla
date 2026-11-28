"""The place's own road signs instead of CARLA's stock (US) sign actors (localsigns.py, tools/sign_check.py)."""
import json
import sys
import types
from pathlib import Path

from coupledcarla import localsigns as ls

ROOT = Path(__file__).resolve().parents[1]

XODR = """<?xml version="1.0"?>
<OpenDRIVE><road id="1"><signals>
  <signal id="s1" s="0" t="-4" name="Speed_30" country="{c}" type="274" subtype="30" value="30"/>
  <signal id="s2" s="5" t="-4" name="Speed_25" country="{c}" type="274" subtype="25" value="25"/>
  <signal id="s3" s="5" t="-4" name="Sign_Yield" country="{c}" type="205" subtype="-1"/>
  <signal id="s4" s="7" t="-4" name="Stencil_STOP" country="{c}" type="206" subtype="-1"/>
  <signal id="s5" s="8" t="-4" name="Sign_Stop" country="{c}" type="206" subtype="-1"/>
  <signal id="s6" s="9" t="-4" name="Sign_PriorityRoad" country="{c}" type="306" subtype="-1"/>
  <signal id="t1" s="9" t="-4" name="Signal_3Light_Post01" country="{c}" type="1000001" subtype="-1"/>
</signals></road></OpenDRIVE>
"""


def test_kinds_and_designs():
    assert ls.kind_of("traffic.speed_limit.30") == ("speed", 30) and ls.kind_of("traffic.yield") == ("give_way", None)
    assert ls.kind_of("traffic.stop") == ("stop", None)
    assert ls.kind_of("traffic.traffic_light") is None and ls.kind_of("traffic.unknown") is None
    # the stock speed plate is the US SPEED LIMIT rectangle: only the US family and Canada
    assert ls.stock_fits("speed", "USA") and ls.stock_fits("speed", "CAN") and not ls.stock_fits("speed", "ESP")
    assert not ls.stock_fits("give_way", "ESP") and ls.stock_fits("give_way", "USA")
    assert ls.stock_fits("stop", "ESP") and not ls.stock_fits("stop", "JPN") and ls.stock_fits("stop", None)
    assert not ls.stock_fits("speed", None)              # unknown country: no US plate


def test_face_keys():
    assert ls.face_key("speed", 30, "ESP") == "A1-30" and ls.face_key("speed", 50, "JPN") == "A1-50"
    assert ls.face_key("give_way", None, "CHE") == "B6" and ls.face_key("give_way", None, "JPN") is None
    assert ls.face_key("stop", None, "ESP") is None            # the octagon fits: kept
    assert ls.face_key("stop", None, "JPN") is None            # no 止まれ face: removed
    assert ls.face_key("speed", 30, "USA") is None and ls.face_key("speed", 30, None) is None
    assert ls.face_key("no_entry", None, "NLD") == "C2"


def test_plan():
    faces = {"A1-30": {"mesh": "SM_NL_A1_30"}, "B6": {"mesh": "SM_NL_B6"}}
    signs = [(1, "traffic.speed_limit.30"), (2, "traffic.speed_limit.90"), (3, "traffic.yield"),
             (4, "traffic.stop"), (5, "traffic.traffic_light")]
    es = {a: (act, m) for a, _, _, act, m in ls.plan(signs, "ESP", faces)}
    assert es == {1: ("replace", "SM_NL_A1_30"), 2: ("remove", None), 3: ("replace", "SM_NL_B6"), 4: ("keep", None)}
    jp = {a: act for a, _, _, act, _ in ls.plan(signs, "JPN", faces)}
    assert jp == {1: "replace", 2: "remove", 3: "remove", 4: "remove"}
    assert {act for *_, act, _ in ls.plan(signs, "USA", faces)} == {"keep"}
    assert {act for *_, act, _ in ls.plan(signs, "ESP", faces, "keep")} == {"keep"}
    assert {a: act for a, _, _, act, _ in ls.plan(signs, "ESP", faces, "remove")}[1] == "remove"


def test_xodr_signs(tmp_path):
    p = tmp_path / "x.xodr"
    p.write_text(XODR.format(c="ES"))
    # 25 km/h has no stock blueprint, the painted STOP and the priority road spawn nothing
    assert ls.xodr_signs(p) == {"274": 1, "205": 1, "206": 1, "country": "ES"}
    p.write_text(XODR.format(c="US"))
    assert ls.xodr_signs(p)["274"] == 2          # US signals: every value gets a MUTCD plate


class _Loc:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Rot:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class _Tf:
    def __init__(self, location=None, rotation=None):
        self.location, self.rotation = location or _Loc(), rotation or _Rot()


class _Actor:
    def __init__(self, i, t, x):
        self.id, self.type_id, self._tf = i, t, _Tf(_Loc(x, 0.0, 790.0), _Rot(yaw=0.0))

    def get_transform(self):
        return self._tf


class _BP:
    def __init__(self):
        self.attrs = {}

    def set_attribute(self, k, v):
        self.attrs[k] = v


def _fake_carla(monkeypatch):
    cmd = types.SimpleNamespace(DestroyActor=lambda i: ("destroy", i),
                                SpawnActor=lambda bp, tf: ("spawn", dict(bp.attrs), tf))
    mod = types.SimpleNamespace(command=cmd, Transform=_Tf, Location=_Loc, Rotation=_Rot,
                                LaneType=types.SimpleNamespace(Driving=1))
    monkeypatch.setitem(sys.modules, "carla", mod)
    return mod


def test_replace_on_a_server(monkeypatch):
    _fake_carla(monkeypatch)
    from coupledcarla.scenery import Catalog
    cat = Catalog("1.0")
    actors = [_Actor(1, "traffic.speed_limit.30", 10.0), _Actor(2, "traffic.yield", 20.0),
              _Actor(3, "traffic.stop", 30.0), _Actor(9, "traffic.traffic_light", 40.0)]
    wp = types.SimpleNamespace(transform=_Tf(_Loc(0, 0, 790.0), _Rot(yaw=90.0)))

    class World:
        def get_actors(self):
            return actors[:2]

        def get_static_actors(self):          # CARLA 1.0 since carla#10381: signs are static actors
            return actors[1:]

        def get_blueprint_library(self):
            return types.SimpleNamespace(find=lambda name: _BP())

        def get_map(self):
            return types.SimpleNamespace(get_waypoint=lambda loc, project_to_road=True, lane_type=None: wp)

    sent = []

    class Client:
        def apply_batch_sync(self, cmds, do_tick=False):
            sent.extend(cmds)
            return [types.SimpleNamespace(error="") for _ in cmds]

    out = ls.replace(Client(), World(), "ESP", log=lambda m: None, catalog=cat)
    assert out["replaced"] == 2 and out["kept"] == 1 and out["removed"] == 0
    assert ("destroy", 1) in sent and ("destroy", 2) in sent and ("destroy", 3) not in sent
    spawned = [c for c in sent if c[0] == "spawn"]
    meshes = [c[1]["mesh_path"].rsplit(".", 1)[-1] for c in spawned]
    assert meshes == ["SM_Pole01", "SM_NL_A1_30", "SM_Pole01", "SM_NL_B6"]
    assert spawned[1][2].rotation.yaw == 180.0           # the face towards the traffic of the nearest lane
    assert spawned[1][2].location.z > spawned[0][2].location.z > 789.0     # on the post, at the sign's ground
    sent.clear()
    assert ls.replace(Client(), World(), "USA", log=lambda m: None, catalog=cat)["kept"] == 3 and not sent


def _level(content, name, files):
    d = content / "Carla" / "Maps" / "Twins" / name / "OpenDrive"
    d.mkdir(parents=True)
    for f, c in files.items():
        (d / f).write_text(XODR.format(c=c) if c else XODR.replace('type="274"', 'type="x"').replace(
            'type="205"', 'type="x"').replace('type="206"', 'type="x"').format(c="ES"))


def test_sign_check_content(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    import sign_check
    c = tmp_path / "Content"
    # the 24-28 Nov 2026 state: the full copy beside the dropped one, read first by the editor server
    _level(c, "SalamancaPlaza", {"SalamancaPlaza.prebake_signs.xodr": "ES", "SalamancaPlaza.xodr": None})
    _level(c, "NycMidtown", {"NycMidtown.xodr": "US"})
    _level(c, "ZurichHB", {"ZurichHB.xodr": None})
    od = c / "Carla" / "Maps" / "OpenDrive"
    od.mkdir(parents=True)
    (od / "SalamancaPlaza.xodr").write_text(XODR.format(c="ES"))       # a package copy with the stock signs
    ext = c / "Carla" / "__ExternalActors__" / "Carla" / "Maps" / "Twins" / "ZurichHB" / "0"
    ext.mkdir(parents=True)
    (ext / "A.uasset").write_bytes(b"\x00/Game/Carla/Static/TrafficSign/BP_SpeedLimit30\x00")
    rows = {r["level"]: r for r in sign_check.check_content(c)}
    s = rows["SalamancaPlaza"]
    assert s["problem"] and s["ambiguous_xodr"] and s["editor_reads"] == "SalamancaPlaza.prebake_signs.xodr"
    assert s["editor_reads_fixed"] == "SalamancaPlaza.xodr" and s["wrong_editor"] == {"speed": 1, "give_way": 1}
    assert s["wrong_editor_fixed"] == {} and s["wrong_package"] == {"speed": 1, "give_way": 1}
    assert not rows["NycMidtown"]["problem"] and rows["NycMidtown"]["country"] == "USA"
    z = rows["ZurichHB"]
    assert z["problem"] and z["wrong_assets"] == {"BP_SpeedLimit": 1} and not z["wrong_editor"]
    assert sign_check.main(["content", str(c), "--json", str(tmp_path / "o.json")]) == 1
    assert json.loads((tmp_path / "o.json").read_text())


def test_sign_check_level_country(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    import sign_check
    assert sign_check.level_country("Town10HD_Opt")[0] == "USA"
    (tmp_path / "place.json").write_text(json.dumps({"country": "PT"}))
    assert sign_check.level_country("Anywhere", tmp_path)[0] == "PRT"
    assert sign_check.level_country("Elsewhere", None, "ES")[0] == "ESP"
    assert sign_check.level_country("Elsewhere", None, "OpenDRIVE")[0] is None
