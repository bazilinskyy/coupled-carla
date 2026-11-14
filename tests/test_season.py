"""Leaf state by date and place (season.py) and lamps that carry a light on CARLA 1.0 (no network, no CARLA)."""
import json
import sys
import types
from datetime import date
from pathlib import Path
from types import SimpleNamespace

import pytest

from coupledcarla import config, scenery, season

EHV = {"source": "modis", "greenup": 107.0, "senescence": 270.0, "greendown": 305.0}      # Eindhoven-like


def test_circular_median_wraps_the_new_year():
    assert season._circ_median([360, 2, 5]) == pytest.approx(2.0)
    assert season._circ_median([100, 110, 120]) == pytest.approx(110.0)
    assert season._circ_median([]) is None


def test_latitude_model():
    nh = season.model_phenology(52.0)
    a, b = season.MODEL["greenup"]
    assert nh["greenup"] == pytest.approx(a + b * 2)
    assert season.model_phenology(50.0)["greenup"] == 113.0          # 23 April (Rotzer and Chmielewski 2001)
    assert nh["greenup"] < nh["senescence"] < nh["greendown"]
    north = season.model_phenology(55.0)
    assert north["greenup"] > nh["greenup"] and north["greendown"] < nh["greendown"]     # Hopkins
    sh = season.model_phenology(-52.0)
    assert (sh["greenup"] - nh["greenup"]) % 365.25 == pytest.approx(182.6, abs=0.2)     # half a year apart
    assert season.model_phenology(10.0) is None and season.model_phenology(-20.0) is None


def test_plausible_cycle():
    assert season.plausible(EHV, 51.4)
    assert not season.plausible({**EHV, "greendown": None}, 51.4)
    assert not season.plausible(EHV, -33.9)              # spring in the wrong half of the year for the south
    assert not season.plausible({"greenup": 200.0, "senescence": 230.0, "greendown": 250.0}, 40.0)   # too short
    sydney = {"greenup": 280.0, "senescence": 100.0, "greendown": 140.0}
    assert season.plausible(sydney, -33.9)
    assert not season.plausible(EHV, 15.0)               # tropics
    ehv_modis = {"greenup": 109.0, "senescence": 210.0, "greendown": 284.0}   # measured: lawns brown in July
    assert season._cycle(ehv_modis, 51.44) and not season.plausible(ehv_modis, 51.44)
    taipei = {"greenup": 113.5, "senescence": 203.0, "greendown": 255.0}     # MODIS there: lawns and paddies
    assert season._cycle(taipei, 25.05) and not season.plausible(taipei, 25.05)


def test_leaf_state_by_date():
    assert season.leaf_state(date(2026, 7, 1), EHV) == ("leaf", 0.0)
    state, p = season.leaf_state(date(2026, 10, 15), EHV)      # day 288: between colouring and leaf fall
    assert state == "autumn" and 0.4 < p < 0.6
    assert season.leaf_state(date(2026, 11, 14), EHV)[0] == "bare"
    assert season.leaf_state(date(2027, 1, 20), EHV)[0] == "bare"
    assert season.leaf_state(date(2027, 4, 30), EHV)[0] == "leaf"
    sydney = {"greenup": 280.0, "senescence": 100.0, "greendown": 140.0}
    assert season.leaf_state(date(2026, 7, 15), sydney)[0] == "bare"
    assert season.leaf_state(date(2026, 1, 15), sydney)[0] == "leaf"
    assert season.leaf_state(date(2026, 4, 30), sydney)[0] == "autumn"
    assert season.leaf_state(date(2026, 1, 15), None) == ("leaf", 0.0)       # tropics


def test_deciduous_or_evergreen():
    d = season.deciduous
    assert d({"genus": "Platanus"}, "plane") and d({"species": "Quercus robur"}, "oak")
    assert not d({"species": "Quercus ilex"}, "oak")             # holm oak keeps its leaves
    assert not d({"genus": "Pinus"}, "pine") and not d({"genus": "Eucalyptus"}, "ash")
    assert d({"genus": "Larix"}, "pine") and d({"genus": "Ginkgo"}, "sassafras")      # deciduous conifer, ginkgo
    assert not d({"leaf_type": "needleleaved"}, "maple")
    assert d({"genus": "Pinus", "leaf_cycle": "deciduous"}, "pine")                   # the mapped tag wins
    assert not d({"genus": "Acer", "leaf_cycle": "evergreen"}, "maple")
    assert not d({}, "palm") and not d({}, "cypress") and d({}, "maple")
    assert not d({"species": "Magnolia grandiflora"}, "ornamental") and d({"genus": "Magnolia"}, "ornamental")


@pytest.mark.parametrize("version", ["0.9.16", "1.0"])
def test_variants_autumn_and_bare(version):
    cat = scenery.Catalog(version)
    mesh, s = scenery._tree(cat, "maple", 12.0, "k1")
    alt = season.variants(cat, cat.m, {"genus": "Acer"}, "maple", mesh, s, "k1")
    assert alt["autumn"] in season.AUTUMN[scenery.major(version)]
    assert alt["bare"] in cat.m.TREES["maple"]
    h = cat.size(mesh)[2] * s
    assert cat.size(alt["bare"])[2] * alt["bare_scale"] == pytest.approx(h * season.BARE_SCALE, rel=0.35)
    assert 0.5 <= alt["autumn_scale"] <= 1.6
    pine, ps = scenery._tree(cat, "pine", 10.0, "k2")
    assert season.variants(cat, cat.m, {"genus": "Pinus"}, "pine", pine, ps, "k2") is None
    assert all(m in cat for m in season.AUTUMN[scenery.major(version)])      # the autumn meshes exist


def test_record_and_finish():
    cat = scenery.Catalog("0.9.16")
    pl = SimpleNamespace(items=[], mesh_index={}, counts={})
    for key, fam, tags in (("a", "maple", {"genus": "Acer"}), ("b", "pine", {"genus": "Pinus"})):
        mesh, s = scenery._tree(cat, fam, 11.0, key)
        idx = pl.mesh_index.setdefault(mesh, len(pl.mesh_index))
        x, y, z, yaw = cat.place(mesh, 5.0, 6.0, 1.0, 30.0, s, 0.1)
        pl.items.append([idx, x, y, z, yaw, s, "tree"])
        season.record(pl, cat, tags, fam, mesh, s, key, 5.0, 6.0)
    placed = dict(pl.mesh_index)
    leaf = season.finish(pl)
    assert len(leaf) == 1 and leaf[0][0] == 0 and len(leaf[0]) == 11       # only the maple
    assert all(pl.mesh_index[m] == i for m, i in placed.items())             # placed meshes keep their index
    names = sorted(pl.mesh_index, key=pl.mesh_index.get)
    assert names[leaf[0][1]] in season.AUTUMN[0] and names[leaf[0][6]] in scenery.TREES["maple"]
    assert pl.counts["tree_deciduous"] == 1


def test_apply_to_items():
    items = [[0, 1.0, 2.0, 3.0, 40.0, 1.0, "tree"], [1, 5.0, 5.0, 0.0, 0.0, 1.0, "lamp"]]
    leaf = [[0, 2, 0.8, 0.1, 0.0, 0.2, 3, 0.6, 0.0, 0.1, -0.1]]
    bare = season.apply(items, leaf, {"state": "bare", "bare": "small"}, lambda m: None)
    assert bare[0] == [3, 1.0, 2.1, 2.9, 40.0, 0.6, "tree"] and bare[1] is items[1]
    assert season.apply(items, leaf, {"state": "bare", "bare": "drop"}, lambda m: None)[0] is None
    assert season.apply(items, leaf, {"state": "bare", "bare": "keep"}, lambda m: None)[0] is items[0]
    autumn = season.apply(items, leaf, {"state": "autumn", "progress": 1.0}, lambda m: None)
    assert autumn[0] in (items[0], [2, 1.1, 2.0, 3.2, 40.0, 0.8, "tree"])
    msgs = []
    assert season.apply(items, None, {"state": "bare"}, msgs.append) is items and "rebuild" in msgs[0]


def test_autumn_share_grows():
    alt = {"autumn": True}
    picks = [[season.choose((i, 0), alt, "autumn", p, "small") for i in range(1000)] for p in (0.0, 0.5, 1.0)]
    shares = [sum(x == "autumn" for x in row) / 1000 for row in picks]
    assert shares[0] == 0 and 0.25 < shares[1] < 0.45 and 0.6 < shares[2] < 0.8


def _modis(values_by_band):
    def fake(lat, lon, band, years=None, km=None, timeout=None):
        return {"subset": [{"calendar_date": "2021-01-01", "data": values_by_band[band] + [season.FILL]}]}
    return fake


def test_modis_parse(monkeypatch):
    d0 = (date(2021, 1, 1) - date(1970, 1, 1)).days
    monkeypatch.setattr(season, "_band", _modis({
        "MidGreenup.Num_Modes_01": [d0 + 100, d0 + 106, d0 + 110],
        "Senescence.Num_Modes_01": [d0 + 268, d0 + 270],
        "MidGreendown.Num_Modes_01": [d0 + 300, d0 + 306, d0 + 304]}))
    ph = season.modis_phenology(51.44, 5.48)
    assert (ph["greenup"], ph["greendown"]) == (107.0, 305.0) and ph["greenup_n"] == 3


def test_phenology_cache_and_fallback(monkeypatch, tmp_path):
    bbox = [51.43, 5.46, 51.45, 5.50]
    msgs, calls = [], []
    monkeypatch.setattr(season, "log", msgs.append)

    def down(*a, **k):
        calls.append(1)
        raise RuntimeError("offline")
    monkeypatch.setattr(season, "modis_phenology", down)
    ph = season.phenology(bbox, tmp_path, modis=True)
    assert ph["source"] == "latitude" and not (tmp_path / "phenology.json").exists()
    assert (tmp_path / "phenology_failed.json").exists() and any("failed" in m for m in msgs)
    assert season.phenology(bbox, tmp_path, quick=True, modis=True)["source"] == "latitude" and len(calls) == 1  # not today
    (tmp_path / "phenology_failed.json").unlink()                                       # another day
    monkeypatch.setattr(season, "modis_phenology", lambda lat, lon, **k: dict(EHV, greenup_n=50))
    assert season.phenology(bbox, tmp_path, modis=True)["greenup"] == 107.0
    monkeypatch.setattr(season, "modis_phenology", down)
    assert season.phenology(bbox, tmp_path, modis=True)["source"] == "modis" and len(calls) == 1     # from the cache
    assert season.phenology(bbox, tmp_path)["source"] == "latitude"                    # the default: no MODIS
    (tmp_path / "phenology.json").write_text(json.dumps({"source": "modis", "greenup": None}))
    assert season.phenology(bbox, tmp_path, modis=True)["source"] == "latitude"                     # MODIS saw no cycle


def test_trial_leaves(monkeypatch, tmp_path):
    from datetime import datetime
    from coupledcarla import transit
    monkeypatch.setattr(transit, "place_now", lambda *a, **k: datetime(2026, 11, 14, 12, 0))     # no network
    (tmp_path / "real").mkdir()
    (tmp_path / "real" / "phenology.json").write_text(json.dumps(EHV))
    cfg = config._merge(config.DEFAULTS, {"map": {"bbox": [51.43, 5.46, 51.45, 5.50]}})
    quiet = lambda m: None      # noqa: E731
    assert season.trial_leaves(cfg, tmp_path, quiet)[0] == "leaf"           # no simulation.time: no season
    cfg["simulation"]["time"] = "2026-01-20 12:00"
    state, _, info = season.trial_leaves(cfg, tmp_path, quiet)
    assert state == "bare" and info["date"] == "2026-01-20" and info["phenology"]["source"] == "latitude"
    cfg["simulation"]["time"] = "2026-10-20 12:00"
    assert season.trial_leaves(cfg, tmp_path, quiet)[0] == "autumn"
    cfg["simulation"]["scenery"]["leaves"] = "modis"            # the cached MODIS dates (plausible here)
    assert season.trial_leaves(cfg, tmp_path, quiet)[2]["phenology"]["source"] == "modis"
    cfg["simulation"]["scenery"]["leaves"] = "auto"
    cfg["simulation"]["scenery"]["leaves"] = False
    assert season.trial_leaves(cfg, tmp_path, quiet)[0] == "leaf"
    cfg["simulation"]["scenery"]["leaves"] = "autumn"
    assert season.trial_leaves(cfg, tmp_path, quiet)[0] == "autumn"


def test_config_defaults():
    sc = config.DEFAULTS["simulation"]["scenery"]
    assert sc["leaves"] == "auto" and sc["bare"] == "small" and sc["lamp_lights"] == "auto"
    assert sc["lights_radius_m"] > 0


def test_content_pack():
    pack = season.PACKS / "CoupledCarla"
    manifest = json.loads((pack / "carla-pack.json").read_text())
    plugin = json.loads((pack / "CoupledCarla.uplugin").read_text())
    bps = json.loads((pack / "Content" / "Config" / "BlueprintParameters.json").read_text())["Blueprints"]
    assert manifest["name"] == "CoupledCarla" and "platform" not in manifest     # a platform would bar Windows
    assert plugin["ExplicitlyLoaded"] and plugin["CanContainContent"]
    ids = {"blueprint." + b["Name"] for b in bps}
    assert set(season.LAMP_LIGHTS.values()) <= ids
    assert all(b["Path"].startswith("/Game/Carla/Static/StreetLight/BP_") for b in bps)
    assert season.pack_args("CarlaUnreal.sh") == [f"-carla-packs={season.PACKS}"]
    assert season.pack_args("CarlaUE4.sh") == []
    from coupledcarla import meshes10
    assert {meshes10.LAMP, meshes10.PARK_LAMP} <= set(season.LAMP_LIGHTS)


# --- the run-time Scenery with a fake carla module ---------------------------------------------------------

class _Cmd(SimpleNamespace):
    pass


def _fake_carla(monkeypatch):
    m = types.ModuleType("carla")
    m.Location = lambda x=0.0, y=0.0, z=0.0: SimpleNamespace(x=x, y=y, z=z)
    m.Rotation = lambda pitch=0.0, yaw=0.0, roll=0.0: SimpleNamespace(pitch=pitch, yaw=yaw, roll=roll)
    m.Transform = lambda loc, rot: SimpleNamespace(location=loc, rotation=rot)
    m.command = SimpleNamespace(SpawnActor=lambda bp, tf: _Cmd(kind="spawn", bp=bp.id, attrs=dict(bp.attrs), tf=tf),
                                DestroyActor=lambda i: _Cmd(kind="destroy", id=i))
    monkeypatch.setitem(sys.modules, "carla", m)


class _BP:
    def __init__(self, id_):
        self.id, self.attrs = id_, {}

    def set_attribute(self, k, v):
        self.attrs[k] = v


class _World:
    def __init__(self):
        self.lib = {"static.prop.mesh": _BP("static.prop.mesh"),
                    "blueprint.cc_streetlight06_a": _BP("blueprint.cc_streetlight06_a"),
                    "blueprint.cc_streetlight03": _BP("blueprint.cc_streetlight03")}

    def get_blueprint_library(self):
        lib = self.lib
        return SimpleNamespace(find=lambda k: lib[k], filter=lambda pat: [b for k, b in lib.items()
                                                                           if k.startswith(pat.rstrip("*"))])

    def get_weather(self):
        return SimpleNamespace(sun_altitude_angle=-10.0)


class _Client:
    def __init__(self):
        self.next, self.alive, self.spawned = 1, {}, []

    def apply_batch_sync(self, cmds, _due=False):
        out = []
        for c in cmds:
            if c.kind == "spawn":
                self.alive[self.next] = c
                self.spawned.append(c)
                out.append(SimpleNamespace(actor_id=self.next, error=""))
                self.next += 1
            else:
                self.alive.pop(c.id, None)
                out.append(SimpleNamespace(actor_id=c.id, error=""))
        return out

    apply_batch = apply_batch_sync


def _scenery_json(tmp_path):
    meshes = ["/Game/Carla/Static/Static/Streetlights/SM_StreetLight06_A.SM_StreetLight06_A",
              "/Game/Carla/Static/Vegetation/00_LegacyAssets/Trees/SM_Maple01_L_A.SM_Maple01_L_A",
              "/Game/Carla/Static/Vegetation/00_LegacyAssets/Trees/SM_Maple01_M_A.SM_Maple01_M_A"]
    items = [[0, 10.0, 10.0, 0.0, 0.0, 1.0, "lamp"],            # cell (0, 0)
             [1, 20.0, 20.0, 0.0, 0.0, 1.0, "tree"],
             [0, 450.0, 10.0, 0.0, 0.0, 1.0, "lamp"]]           # cell (4, 0): beyond the lights radius at first
    leaf = [[1, -1, 0.0, 0.0, 0.0, 0.0, 2, 0.9, 0.0, 0.0, 0.0]]
    (tmp_path / "scenery.json").write_text(json.dumps({"meshes": meshes, "items": items, "cell_m": 100.0,
                                                       "leaf": leaf}))


def test_scenery_lamps_lit_and_bare_trees(monkeypatch, tmp_path):
    _fake_carla(monkeypatch)
    _scenery_json(tmp_path)
    client, world = _Client(), _World()
    seas = {"state": "bare", "bare": "small", "lights": dict(season.LAMP_LIGHTS), "lights_radius_m": 150.0}
    sc = scenery.Scenery(client, world, tmp_path, {"radius_m": 600.0}, lambda m: None, season=seas)
    sc.update([SimpleNamespace(x=0.0, y=0.0)])
    kinds = [(c.bp, c.attrs.get("mesh_path", "").rsplit(".", 1)[-1], round(c.tf.location.x)) for c in client.spawned]
    assert ("blueprint.cc_streetlight06_a", "", 10) in kinds                       # lit lamp near the participant
    assert ("static.prop.mesh", "SM_StreetLight06_A", 450) in kinds                # far lamp: a plain mesh
    assert ("static.prop.mesh", "SM_Maple01_M_A", 20) in kinds                     # bare: the small stand-in
    assert sc.n_lights == 1 and sc.n == 3
    client.spawned.clear()
    sc.update([SimpleNamespace(x=400.0, y=0.0)])                                     # the far lamp comes near
    assert [c.bp for c in client.spawned] == ["blueprint.cc_streetlight06_a"]
    assert sc.n_lights == 2 and sc.n == 3 and len(client.alive) == 3                # the mesh lamp was replaced
    sc.destroy()
    assert not client.alive and sc.n_lights == 0


def test_scenery_daylight_unchanged(monkeypatch, tmp_path):
    _fake_carla(monkeypatch)
    _scenery_json(tmp_path)
    client = _Client()
    sc = scenery.Scenery(client, _World(), tmp_path, {"radius_m": 600.0}, lambda m: None)
    sc.update([SimpleNamespace(x=0.0, y=0.0)])
    assert all(c.bp == "static.prop.mesh" for c in client.spawned) and sc.n == 3 and not sc.light_bp
    assert "SM_Maple01_L_A" in [c.attrs["mesh_path"].rsplit(".", 1)[-1] for c in client.spawned]


def test_for_trial_lights_only_at_night(monkeypatch, tmp_path):
    cfg = config._merge(config.DEFAULTS, {"map": {"bbox": [51.43, 5.46, 51.45, 5.50]}})
    world = _World()
    seas, info = season.for_trial(cfg, tmp_path, world, lambda m: None)
    assert seas["lights"] == season.LAMP_LIGHTS and info["lamp_lights"]
    world.get_weather = lambda: SimpleNamespace(sun_altitude_angle=20.0)
    seas, info = season.for_trial(cfg, tmp_path, world, lambda m: None)
    assert "lights" not in seas and not info["lamp_lights"]
    world.get_weather = lambda: SimpleNamespace(sun_altitude_angle=-10.0)
    world.lib = {"static.prop.mesh": _BP("static.prop.mesh")}                         # 0.9.16: no pack
    msgs = []
    seas, _ = season.for_trial(cfg, tmp_path, world, msgs.append)
    assert seas["lights"] == {} and any("-carla-packs" in m for m in msgs)
