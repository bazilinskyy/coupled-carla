"""Scenery on CARLA 0.9.16 and CARLA 1.0: the mesh catalogue per version, and scenery on baked levels."""
import json

import pytest

from coupledcarla import mapbuild, scenery


def test_catalog_version_by_server():
    assert scenery.catalog_version("auto", 0) == "0.9.16"
    assert scenery.catalog_version("auto", 1) == "1.0"
    assert scenery.catalog_version("0.9.16", 1) == "0.9.16"       # pinned
    assert scenery.catalog_version("auto", 7) is None
    assert scenery.major("1.0-38b72cc52") == 1 and scenery.major("0.9.16") == 0


def test_catalog_0916_keeps_its_tables():
    cat = scenery.Catalog("0.9.16")
    assert cat.m is scenery.MESHES_09
    assert cat.m.BUILDINGS is scenery.BUILDINGS and cat.m.TREES is scenery.TREES
    assert cat.m.LAMP == scenery.LAMP and cat.m.SPEED.format(50) == "SM_SpeedSign50"
    assert not any("/" in n for n in cat.by_name)      # no folder/name keys on 0.9.16


def test_catalog_10_names_exist():
    """Every mesh the 1.0 tables name is in meshes_1.0.json (so nothing silently drops out)."""
    from coupledcarla import meshes10
    cat = scenery.Catalog("1.0")
    assert cat.m is meshes10.MESHES
    m = cat.m
    names = [n for k in m.BUILDINGS.values() for n in k] + [n for k in m.TREES.values() for n in k] + m.BUSHES
    names += [n for v in m.FURNITURE.values() for n in v] + m.SHELTER + [m.LAMP, m.PARK_LAMP, m.POST, m.STOP,
                                                                         m.GIVE_WAY, m.PAVING]
    names += [m.SPEED.format(v) for v in m.SPEEDS] + list(m.GROUND_MESH.values())
    missing = sorted(n for n in names if n not in cat)
    assert not missing, missing
    assert set(m.GENUS.values()) <= set(m.TREES)       # every genus maps to a family that has meshes
    assert all(fam in m.TREES for fam in ("maple", "oak", "pine", "ash", "beech", "palm"))


def test_catalog_10_path_keys():
    """A name CARLA 1.0 has twice is told apart by its path under Content/Carla/Static."""
    cat = scenery.Catalog("1.0")
    assert cat.path("Static/SM_Bench02") == "/Game/Carla/Static/Static/SM_Bench02.SM_Bench02"
    assert cat.path("Static/00_LegacyAssets/SM_Bench02").endswith("00_LegacyAssets/SM_Bench02.SM_Bench02")
    assert cat.size("Static/SM_Bench02") != cat.size("Static/00_LegacyAssets/SM_Bench02")
    assert cat.optimised("SM_House07_A") == "SM_House07_A_Opt"


def test_ground_pivot_anchor():
    """1.0 buildings with a foundation below their pivot stand on the pivot (as CARLA's towns place them)."""
    cat = scenery.Catalog.__new__(scenery.Catalog)
    cat.by_name = {"Deep": ("/x.Deep", {"ext": [3.0, 8.0, 8.8], "off": [0.0, 0.0, 7.2]}),     # 1.6 m below pivot
                   "Flat": ("/x.Flat", {"ext": [3.0, 8.0, 8.8], "off": [0.0, 0.0, 8.8]}),     # bottom at pivot
                   "Podium": ("/x.P", {"ext": [3.0, 8.0, 8.8], "off": [0.0, 0.0, 5.0]})}      # 3.8 m below
    assert cat.place("Deep", 0, 0, 10.0, 0, 1.0, anchor="ground")[2] == pytest.approx(10.0)
    assert cat.place("Deep", 0, 0, 10.0, 0, 1.0)[2] == pytest.approx(11.6)                    # bottom anchor
    assert cat.place("Flat", 0, 0, 10.0, 0, 1.0, anchor="ground")[2] == pytest.approx(10.0)
    assert cat.place("Podium", 0, 0, 10.0, 0, 1.0, anchor="ground")[2] == pytest.approx(13.8)
    assert scenery.MESHES_09.GROUND_PIVOT is False


def _map(tmp_path, catalog):
    (tmp_path / "scenery.json").write_text(json.dumps({"catalog": catalog, "items": [], "meshes": []}))
    (tmp_path / "meta.json").write_text(json.dumps({"bbox": [51.4, 5.4, 51.5, 5.5]}))
    return tmp_path


def test_scenery_for_server(tmp_path, monkeypatch):
    msgs, built = [], []
    cfg = {"map": {"scenery": {"catalog": "auto"}}}
    out = _map(tmp_path, "0.9.16")
    assert mapbuild.scenery_for_server(cfg, out, 0, msgs.append) and not msgs
    monkeypatch.setattr(mapbuild, "build_scenery", lambda c, o, meta, server_major=None, log=None:
                        built.append(server_major) or meta)
    assert mapbuild.scenery_for_server(cfg, out, 1, msgs.append)     # rebuilt for the 1.0 server
    assert built == [1] and "rebuilding" in msgs[-1]
    pinned = {"map": {"scenery": {"catalog": "0.9.16"}}}
    assert not mapbuild.scenery_for_server(pinned, out, 1, msgs.append) and "set catalog: auto" in msgs[-1]
    assert built == [1]

    def fail(*a, **k):
        raise RuntimeError("offline")
    monkeypatch.setattr(mapbuild, "build_scenery", fail)
    assert not mapbuild.scenery_for_server(cfg, out, 1, msgs.append) and "rebuild failed" in msgs[-1]
    assert not mapbuild.scenery_for_server(cfg, tmp_path / "nomap", 1, msgs.append)


def test_baked_level_scenery_has_no_buildings_or_ground(tmp_path, monkeypatch):
    pytest.importorskip("sumolib")    # geo.Geo reads a net (CI installs no SUMO)
    seen = {}

    def fake_build(out, cfg, geo, dem, base, cmap, log, server_major):
        seen.update(sc=cfg["map"]["scenery"], bbox=cfg["map"]["bbox"], base=base, dem=dem)
        (out / "buildings.json").write_text("[]")
        return {"counts": {"tree": 3}, "sources": ["osm"]}

    monkeypatch.setattr(scenery, "build", fake_build)
    import coupledcarla.geo
    monkeypatch.setattr(coupledcarla.geo, "Geo", lambda p: None)

    class FakeDem:
        def sample(self, lat, lon):
            return lat * 0 + 15.85

    monkeypatch.setattr(mapbuild.realworld.Dem, "load", staticmethod(lambda p: FakeDem()))
    cfg = {"map": {"carla_level": "EhvCentre", "bbox": None, "scenery": {"trees": True}}}
    meta = {"bbox": [51.43, 5.46, 51.45, 5.49], "elevation": {"source": "ahn", "base_m": 0.0, "absolute": True}}
    mapbuild.build_scenery(cfg, tmp_path, meta)
    assert seen["sc"] == {"trees": True, "buildings": False, "ground": False}
    assert seen["bbox"] == meta["bbox"] and seen["base"] == 0.0 and seen["dem"] is not None
    assert meta["scenery"] == {"tree": 3}


def test_absolute_elevation(tmp_path, monkeypatch):
    import numpy as np

    class FakeDem:
        def sample(self, lat, lon):
            return 15.0 + np.asarray(lat) * 0 + (np.asarray(lon) - 5.46) * 100

    monkeypatch.setattr(mapbuild.realworld, "dem", lambda bbox, path: (FakeDem(), "ahn"))
    e = mapbuild.absolute_elevation([51.43, 5.46, 51.45, 5.49], tmp_path)
    assert e["base_m"] == 0.0 and e["absolute"] and e["min_m"] == pytest.approx(15.0) and e["max_m"] == 18.0
