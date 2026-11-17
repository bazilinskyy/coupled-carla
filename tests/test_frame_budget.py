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
