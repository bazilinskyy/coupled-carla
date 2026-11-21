"""tools/cook_catalog_dirs.py (the package cooks every catalogue mesh) and tools/package_check.py's verdicts."""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import cook_catalog_dirs as ccd  # noqa: E402
import package_check as pc  # noqa: E402


def _tree(tmp_path):
    c = tmp_path / "Content"
    for rel, data in {"Carla/Static/SideWalk/SideWalkCube.uasset": b"mesh",
                      "Carla/Static/Building/Houses/H1/SM_House01.uasset": b"mesh",
                      "Carla/Static/Building/Houses/H2/SM_House02.uasset": b"mesh",
                      "Carla/Static/Static/SM_PlantPot01_B.uasset": b"..ObjectRedirector..",
                      "Carla/Static/Vegetation/SM_Tree.uasset": b"mesh"}.items():
        f = c / rel
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_bytes(data)
    return c


PATHS = ["/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube",
         "/Game/Carla/Static/Building/Houses/H1/SM_House01.SM_House01",
         "/Game/Carla/Static/Building/Houses/H2/SM_House02.SM_House02",
         "/Game/Carla/Static/Static/SM_PlantPot01_B.SM_PlantPot01_B",
         "/Game/Carla/Static/Vegetation/SM_Tree.SM_Tree",
         "/Game/Carla/Static/Water/SM_Lake01.SM_Lake01"]


def test_plan_dirs_redirectors_missing(tmp_path):
    c = _tree(tmp_path)
    dirs, red, missing = ccd.plan(PATHS, c, ["/Game/Carla/Static/Vegetation"])
    assert dirs == ["/Game/Carla/Static/Building/Houses/H1", "/Game/Carla/Static/Building/Houses/H2",
                    "/Game/Carla/Static/SideWalk"]
    assert red == ["/Game/Carla/Static/Static/SM_PlantPot01_B.SM_PlantPot01_B"]
    assert missing == ["/Game/Carla/Static/Water/SM_Lake01.SM_Lake01"]


def test_collapse_keeps_topmost():
    assert ccd.collapse(["/Game/A/B", "/Game/A", "/Game/AB", "/Game/A/B/C"]) == ["/Game/A", "/Game/AB"]


def test_block_after_last_cook_line_and_replaced(tmp_path):
    lines = ["[/Script/UnrealEd.ProjectPackagingSettings]", '+DirectoriesToAlwaysCook=(Path="/Game/X")',
             '+DirectoriesToAlwaysStageAsUFS=(Path="Carla/Maps/OpenDrive")']
    once = ccd.with_block(lines, ["/Game/Y"])
    assert once[2] == ccd.BEGIN and once[3] == '+DirectoriesToAlwaysCook=(Path="/Game/Y")' and once[4] == ccd.END
    assert once[-1].startswith("+DirectoriesToAlwaysStageAsUFS")
    twice = ccd.with_block(once, ["/Game/Z"])
    assert twice.count(ccd.BEGIN) == 1 and '+DirectoriesToAlwaysCook=(Path="/Game/Y")' not in twice
    # the block's own lines do not count as existing ones on a re-run
    assert ccd.existing_dirs(ccd.without_block(twice)) == ["/Game/X"]


def test_main_writes_crlf_ini(tmp_path):
    c = _tree(tmp_path)
    cat = tmp_path / "meshes.json"
    cat.write_text(json.dumps({"meshes": {p: {} for p in PATHS[:4]}}))
    ini = tmp_path / "DefaultGame.ini"
    ini.write_bytes(b'[/Script/UnrealEd.ProjectPackagingSettings]\r\n+DirectoriesToAlwaysCook=(Path="/Game/X")\r\n')
    red = tmp_path / "red.json"
    assert ccd.main(["--content", str(c), "--ini", str(ini), "--catalog", str(cat), "--redirectors-out", str(red)]) == 0
    t = ini.read_bytes().decode()
    assert "\r\n" in t and "\n" not in t.replace("\r\n", "")
    assert '+DirectoriesToAlwaysCook=(Path="/Game/Carla/Static/SideWalk")' in t
    assert json.loads(red.read_text()) == ["/Game/Carla/Static/Static/SM_PlantPot01_B.SM_PlantPot01_B"]
    before = ini.read_bytes()
    ccd.main(["--content", str(c), "--ini", str(ini), "--catalog", str(cat)])
    assert ini.read_bytes() == before        # idempotent


def _img(rgb):
    return np.tile(np.array(rgb, dtype=np.uint8), (64, 64, 1))


def test_verdict_grey_pavers_pass_lawn_fails():
    lawn, pavers = _img((70, 110, 50)), _img((120, 118, 115))
    assert pc.green_excess(lawn) > pc.GREEN_MAX
    ok, why = pc.verdict(lawn, pavers)
    assert ok, why
    ok, why = pc.verdict(_img((90, 90, 90)), lawn)
    assert not ok and "green" in why


def test_verdict_unchanged_view_fails():
    ok, why = pc.verdict(_img((120, 118, 115)), _img((121, 118, 115)))
    assert not ok and "did not change" in why


def test_verdict_dark_fails():
    ok, why = pc.verdict(_img((120, 118, 115)), _img((5, 5, 5)))
    assert not ok and "dark" in why
