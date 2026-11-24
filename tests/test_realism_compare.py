"""A/B rounds of the realism pilot (tools/realism_score.py compare, score --align-from) and the reference photos that
are not street views (realism_check.not_street, experiments/pilot/excluded_views.yaml). No GPU, no server."""
import csv
import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from coupledcarla import realism as R  # noqa: E402
from test_realism import scene  # noqa: E402  (the synthetic street of the alignment tests)


def test_align_keeps_a_given_alignment():
    w, h, hfov = 320, 180, 90.0
    ww, wh, wf = R.wide_size(w, h, hfov, 15.0, 8.0)
    wide = scene(ww, wh, wf)
    real = scene(w, h, hfov, yaw=6.0)
    found = R.align(real, wide, hfov, w, h, wf, yaw_range=12, pitch_range=6, step=3)[:2]
    # an earlier round's (-3, 3): kept as it is, though another offset would match better
    kept = R.align(real, wide, hfov, w, h, wf, yaw_range=0, pitch_range=0, center=(-3.0, 3.0))[:2]
    assert found == (6.0, 0.0) and kept == (-3.0, 3.0)


def test_align_pano_keeps_a_given_cut():
    w, h, hfov = 160, 90, 90.0
    W, H = 720, 360
    pano = np.full((H, W), R.GROUPS.index("building"), np.uint8)
    pano[: H // 3] = R.GROUPS.index("sky")
    render = np.full((h, w), R.GROUPS.index("building"), np.uint8)
    err, perr, _, _, cut = R.align_pano(pano, render, hfov, w, h, src_yaw=0.0, yaw_range=0, pitch_range=0,
                                        center=(9.0, -3.0))
    assert cut == (9.0, -3.0) and (err, perr) == (-9.0, 3.0)      # the errors are the cut negated


def test_not_street_listed_and_by_the_photos_labels():
    import realism_check as RC
    listed = {"com_1": "indoors (a station platform)"}
    street = {"id": "map_2", "nom_real_road": 0.30, "nom_real_sidewalk": 0.05, "nom_real_sky": 0.20}
    wall = {"id": "map_3", "nom_real_road": 0.01, "nom_real_sidewalk": 0.0, "nom_real_sky": 0.0}
    facade = {"id": "map_4", "nom_real_road": 0.01, "nom_real_sidewalk": 0.0, "nom_real_sky": 0.15}   # sky: kept
    assert RC.not_street({**street, "id": "com_1"}, listed) == "indoors (a station platform)"
    assert RC.not_street(street, listed) == ""
    assert "no ground and no sky" in RC.not_street(wall, listed)
    assert RC.not_street(facade, listed) == ""
    assert RC.not_street({"id": "x"}, listed) == ""          # no labels (an old compare.csv): never guessed


def test_the_exclusion_list_reads_and_says_why():
    import realism_check as RC
    ex = RC.load_exclusions(ROOT / "experiments" / "pilot" / "excluded_views.yaml")
    assert "station" in ex["com_95642078"] and len(ex) >= 20
    assert all(r.split(" (")[0].split(";")[0] in ("indoors", "close-up of one object", "people filling the frame",
                                                  "from high up", "sky only") for r in ex.values())
    assert RC.load_exclusions(None) == {} and RC.load_exclusions("/nonexistent.yaml") == {}


def _row(i, spot, he=0.0, pe=0.0, road=0.3, sky=0.2, layout=0.5, side=0.05):
    return {"id": i, "spot_score": spot, "heading_error": he, "pitch_error": pe, "nom_real_road": road,
            "nom_real_sidewalk": side, "nom_real_sky": sky, "al_layout_tv": layout}


def test_compare_rounds_on_the_same_views():
    import realism_score as S
    a = {f"v{i}": _row(f"v{i}", 0.30 + 0.01 * (i % 5)) for i in range(40)}
    b = {f"v{i}": _row(f"v{i}", 0.30 + 0.01 * (i % 5) + 0.02, layout=0.52) for i in range(40)}
    b["v0"] = _row("v0", 0.10, he=6.0)                  # re-aligned and much worse
    b["v1"] = _row("v1", 0.20, he=3.0)
    a["v2"] = _row("v2", 0.9, road=0.0, sky=0.0, side=0.0)        # a wall photo: left out
    b["v2"] = _row("v2", 0.1, road=0.0, sky=0.0, side=0.0)
    a["only_a"] = _row("only_a", 0.5)                   # not in B: not compared
    r = S.compare_rounds(a, b, {"v3": "indoors"}, scale=0.2)
    assert r["views"]["n"] == 38 and set(r["excluded"]) == {"v2", "v3"} and r["realigned"] == 2
    s = r["same_alignment"]
    assert s["n"] == 36 and s["mean"] == pytest.approx(0.02) and s["ci95"][0] == pytest.approx(0.02)
    assert r["views"]["mean"] < s["mean"] and r["views"]["ci95"][0] < r["views"]["mean"] < r["views"]["ci95"][1]
    assert r["views"]["index_points"]["mean"] == pytest.approx(r["views"]["mean"] / 0.2)
    assert r["components"]["al_layout_tv"] == pytest.approx(0.02 * 36 / 38)
    assert r["biggest_drops"][0][0] == "v0"
    again = S.compare_rounds(a, b, {"v3": "indoors"}, scale=0.2)      # fixed seed: the same interval every time
    assert again["views"]["ci95"] == r["views"]["ci95"]


def _write(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(next(iter(rows)).keys()))
        w.writeheader()
        w.writerows(rows)


def test_compare_command_writes_a_table_and_json(tmp_path, capsys):
    import realism_score as S
    pilot = tmp_path / "pilot.yaml"
    pilot.write_text("places:\n- {name: alpha, role: train, experiment: experiments/a.yaml}\n"
                     "- {name: beta, role: held_out, experiment: experiments/b.yaml}\n")
    root = tmp_path / "logs"
    _write(root / "rounds" / "ra" / "alpha" / "compare.csv", [_row(f"v{i}", 0.3) for i in range(10)])
    _write(root / "rounds" / "rb" / "alpha" / "compare.csv", [_row(f"v{i}", 0.31) for i in range(10)])
    (root / "rounds" / "rb" / "alpha" / "pairs_summary.json").write_text(
        json.dumps({"neighbour": {"spot_score": 0.5}, "random": {"spot_score": 0.3}}))
    args = types.SimpleNamespace(pilot=str(pilot), root=str(root), a="ra", b="rb", places=None,
                                 exclude=str(tmp_path / "none.yaml"), out=None)
    S.cmd_compare(args)
    out = capsys.readouterr().out
    assert "| alpha | 10 | +0.0100 [+0.0100, +0.0100] | +0.050 | 0 |" in out and "beta" not in out
    js = json.loads((root / "rounds" / "rb" / "compare_ra.json").read_text())
    assert js["places"]["alpha"]["views"]["index_points"]["mean"] == pytest.approx(0.05)
