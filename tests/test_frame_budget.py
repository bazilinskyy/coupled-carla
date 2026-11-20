"""tools/frame_budget.py: the verdict arithmetic (no server)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import frame_budget as fb  # noqa: E402


def test_parse_variants():
    assert fb.parse_variants("") == {"current": ""}
    v = fb.parse_variants('epic=;nomega=-ExecCmds="r.MegaLights.Enable 0"')
    assert v == {"epic": "", "nomega": '-ExecCmds="r.MegaLights.Enable 0"'}


def test_percentile():
    assert fb.percentile([], 0.5) is None
    assert fb.percentile([1, 2, 3, 4], 0.5) == pytest.approx(2.5)
    assert fb.percentile([0.01] * 99 + [0.1], 0.99) == pytest.approx(0.0109, abs=1e-4)


def test_summarise_pass_and_fail():
    dts = [1 / 70.0] * 700
    r = fb.summarise(dts, [690], 10.0, 60.0, vr=False)
    assert r["server_fps"] == pytest.approx(70.0) and r["camera_fps"] == [69.0] and r["pass"]
    r = fb.summarise(dts, [500], 10.0, 60.0, vr=False)           # the camera falls behind
    assert not r["pass"]
    r = fb.summarise(dts, [690, 690], 10.0, 60.0, vr=True)       # a headset needs 90
    assert r["need_fps"] == 90.0 and not r["pass"]
    slow = [1 / 70.0] * 690 + [0.1] * 10                          # hitches show in the 1 % low
    assert fb.summarise(slow, [], 10.0, 60.0, False)["low_1pct_fps"] < 20


def test_cases_cover_day_and_night():
    assert fb.CASES["day"][0] > 0 and fb.CASES["night"][0] < 0
    assert fb.CASES["day"][1] == 0.0            # no lamp lights by day


def test_report_tables(tmp_path):
    import json
    import frame_budget_report as rep
    a = tmp_path / "A_epic"
    a.mkdir()
    rows = []
    for lvl, s, c in (("TUeCampus", 50.0, 17.0), ("TUeCampusWin", 45.0, 15.0)):
        for cam, k in (("1920x1080", 1.0), ("1920x1080+dlss", 1.6)):
            rows.append({"variant": "epic", "level": lvl, "case": "day", "cam": cam, "server_fps": s,
                         "low_1pct_fps": 30.0, "camera_fps": [round(c * k, 1)], "lamps_lit": 0})
    rows.append({"variant": "epic", "level": "TUeCampus", "case": "night", "cam": "1280x720", "error": "boom"})
    (a / "frame_budget.json").write_text(json.dumps(rows))
    c = tmp_path / "C_lamps"
    c.mkdir()
    (c / "megalights.json").write_text(json.dumps([{"radius_m": 600.0, "lamps_placed": 120, "lamps_lit": 118,
                                                    "street_lights": 118, "street_lights_on": 118,
                                                    "register_specs_applied": 110, "brightness_off": 4.0,
                                                    "brightness_on": 9.0, "lit_gain": 2.25,
                                                    "fps": {"camera_fps": [14.0]}}]))
    text = rep.report(*rep.load(tmp_path))
    assert "| day | 1920x1080 | 50.0 -> 45.0 (-10 %) | 17.0 -> 15.0 |" in text
    assert "Errors: epic/TUeCampus/night/1280x720: boom" in text
    assert "| 600 | 120 | 118 | 118/118 | 110 | 4.0 -> 9.0 (x2.25) | 14.0 |" in text
    assert "| TUeCampus | day | 1920x1080+dlss | 27.2 | 32.8 |" in text
