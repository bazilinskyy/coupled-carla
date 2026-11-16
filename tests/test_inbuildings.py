"""coupledcarla.inbuildings / tools/trial_buildings.py: actors inside building footprints, from a trial's logs."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from coupledcarla import inbuildings  # noqa: E402

HEAD = ("timestamp;frame;actor_id;kind;role;type_id;pos_x;pos_y;pos_z;rot_pitch;rot_yaw;rot_roll;lat;lon;vel_x;vel_y;"
        "vel_z;speed_kmh")


def row(t, frame, aid, kind, ty, x, y, yaw=0.0, z=0.5):
    return f"{t:.3f};{frame};{aid};{kind};;{ty};{x};{y};{z};0;{yaw};0;52.0;5.0;0;0;0;0"


def make_trial(tmp_path, rows, footprints, name="buildings.json"):
    trial = tmp_path / "logs" / "x" / "trial0"
    trial.mkdir(parents=True)
    (trial / "frames.csv").write_text("\n".join([HEAD] + rows) + "\n")
    (trial / "config.yaml").write_text("map:\n  name: m\n")
    maps = tmp_path / "maps" / "m"
    maps.mkdir(parents=True)
    (maps / name).write_text(json.dumps(footprints))
    return trial, maps


SQUARE = {"id": "B", "p": [[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]]}


def test_vehicle_and_walker_inside_and_outside(tmp_path):
    rows = []
    for k in range(10):
        t = k * 0.1
        rows.append(row(t, 100 + k, 1, "ai", "vehicle.tesla.model3", -5 + k * 1.0, 5))     # drives into the block
        rows.append(row(t, 100 + k, 2, "ai", "walker.pedestrian.0001", 5, 5))              # stands inside
        rows.append(row(t, 100 + k, 3, "ai", "walker.pedestrian.0002", 10.05, 5))          # at the wall: fine
        rows.append(row(t, 100 + k, 4, "ai", "vehicle.tesla.model3", 30, 30, z=-30))       # held under the road
    trial, maps = make_trial(tmp_path, rows, [SQUARE])
    rep = inbuildings.scan_trial(trial, sizes={"vehicle.tesla.model3": {"length": 4.8, "width": 2.1}})
    assert rep["buildings"] == 1 and rep["footprints"].endswith("buildings.json")
    who = {e["actor"]: e for e in rep["episodes"]}
    assert set(who) == {"1", "2"}
    assert who["2"]["t"] == 0.0 and abs(who["2"]["t_end"] - 0.9) < 1e-9
    assert who["1"]["t"] >= 0.3          # two corners in once the nose is ~2 m deep
    lines = inbuildings.summary_lines("x", rep)
    assert "2 stays of 2 actors" in lines[0]


def test_twin_footprints_win_and_passages_are_open(tmp_path):
    """The twin's cut footprints (a passage through the block) are used when they exist."""
    rows = [row(k * 0.1, k, 1, "ai", "vehicle.mitsubishi.fusorosa", -10 + k * 2.0, 5) for k in range(15)]
    trial, maps = make_trial(tmp_path, rows, [SQUARE])
    assert inbuildings.scan_trial(trial)["episodes"]
    cut = [{"id": "B", "p": [[0, 0], [10, 0], [10, 3], [0, 3]]}, {"id": "B", "p": [[0, 7], [10, 7], [10, 10], [0, 10]]}]
    (maps / "twin_buildings.json").write_text(json.dumps(cut))
    rep = inbuildings.scan_trial(trial)
    assert rep["footprints"].endswith("twin_buildings.json") and rep["episodes"] == []


def test_window_and_tool_exit_code(tmp_path, capsys):
    import trial_buildings
    rows = [row(k * 1.0, k, 2, "ai", "walker.pedestrian.0001", 5 if k >= 5 else -5, 5) for k in range(10)]
    trial, maps = make_trial(tmp_path, rows, [SQUARE])
    assert inbuildings.scan_trial(trial, t0=0, t1=4)["episodes"] == []
    assert trial_buildings.main([str(trial), "--start", "0", "--duration", "4"]) == 0
    out = tmp_path / "r.json"
    assert trial_buildings.main([str(trial), "--json", str(out)]) == 1
    assert json.loads(out.read_text())[str(trial)]["episodes"][0]["t"] == 5.0
    assert "inside buildings" in capsys.readouterr().out


def test_mapbuild_copies_the_twins_footprints(tmp_path, monkeypatch):
    """build_from_opendrive puts <Name>.buildings.json next to the twin's OpenDRIVE into twin_buildings.json."""
    from coupledcarla import mapbuild
    src = tmp_path / "twins" / "T.xodr"
    src.parent.mkdir()
    src.write_text("<OpenDRIVE/>")
    (src.parent / "T.buildings.json").write_text(json.dumps([SQUARE]))
    for fn in ("name_signals", "netconvert", "sumo_net", "build_scenery", "build_transit"):
        monkeypatch.setattr(mapbuild, fn, lambda *a, **k: None)
    monkeypatch.setattr(mapbuild, "check_xodr", lambda x: {})
    out = tmp_path / "maps" / "t"
    out.mkdir(parents=True)
    cfg = {"_root": str(tmp_path), "map": {"name": "t", "opendrive": str(src), "bbox": [0, 0, 1, 1],
                                          "scenery": {"enabled": False}, "transit": False}}
    mapbuild.build_from_opendrive(cfg, out)
    assert json.loads((out / "twin_buildings.json").read_text()) == [SQUARE]
    assert json.loads((out / "meta.json").read_text())["twin_buildings"] == 1
