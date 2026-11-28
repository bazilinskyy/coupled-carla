"""Lit signal lenses are round (and pictogram lenses light their pictogram), and the head meshes carry the lens sizes
and pictograms signals.py draws (signalfaces.py, tools/street_props.py; no CARLA, no SUMO)."""
import json
import math
import sys
from pathlib import Path

import pytest

from coupledcarla import signalfaces as sf

ROOT = Path(__file__).resolve().parent.parent
STEP = 0.002


def _cells(lines, yaw=0.0, centre=(0.0, 0.0, 0.0)):
    return sf.covered(lines, centre, yaw, STEP)


def _r(cell):
    i, j = cell
    return math.hypot((i + 0.5) * STEP, (j + 0.5) * STEP)


@pytest.mark.parametrize("d", [0.20, 0.30])
def test_lit_plain_lens_is_a_disc_inside_the_lens(d):
    lines = sf.lit_lines((0.0, 0.0, 0.0), 0.0, d)
    assert len(lines) == len(sf.DISC_ANGLES) > 1
    cells = _cells(lines)
    R = d * sf.LIT_FILL / 2
    assert max(_r(c) for c in cells) <= d / 2                       # never past the lens (and its visor)
    assert len(cells) * STEP ** 2 / (math.pi * R * R) > 0.88         # as big as the lens
    # round: a square's corners are not lit (the old 0.24 m square stuck out past the 0.30 m lens's circle)
    for sx in (-1, 1):
        for sz in (-1, 1):
            corner = (int(sx * 0.42 * d / STEP), int(sz * 0.42 * d / STEP))
            assert corner not in cells
    centre = (0, 0)
    assert centre in cells and (int(0.4 * d / STEP), 0) in cells and (0, int(0.4 * d / STEP)) in cells


def test_disc_follows_the_face():
    # a face turned to yaw 90 (its local +x along world +y): the disc spans y and z, not x
    lines = sf.disc_lines((10.0, 5.0, 3.0), 90.0, 0.2)
    for a, b, t in lines:
        assert a[0] == pytest.approx(10.0) and b[0] == pytest.approx(10.0)
        assert a[2] == b[2] == pytest.approx(3.0) and (a[1] + b[1]) / 2 == pytest.approx(5.0)
        assert 0 < t <= 0.2


@pytest.mark.parametrize("key", sorted(sf.PICTOGRAMS))
def test_pictograms_light_inside_the_lens_only(key):
    d = 0.20
    lit = _cells(sf.lit_lines((0.0, 0.0, 0.0), 0.0, d, key))
    assert lit and max(_r(c) for c in lit) < d / 2 * 0.97           # inside the mask's rim
    disc = _cells(sf.lit_lines((0.0, 0.0, 0.0), 0.0, d))
    assert len(lit) < 0.6 * len(disc)                                # a symbol, not a filled lens


def test_pictogram_shapes():
    bike = _cells(sf.pictogram_lines((0, 0, 0), 0.0, 0.2, "bike"))
    s = 0.1 * sf.PICTO_FILL
    left = {c for c in bike if (c[0] + 0.5) * STEP < -0.5 * s}       # two wheels, left and right, low down
    right = {c for c in bike if (c[0] + 0.5) * STEP > 0.5 * s}
    assert left and right and all((c[1] + 0.5) * STEP < 0.1 * s for c in left)
    stand = _cells(sf.pictogram_lines((0, 0, 0), 0.0, 0.2, "ped_stand"))
    walk = _cells(sf.pictogram_lines((0, 0, 0), 0.0, 0.2, "ped_walk"))
    feet = lambda cells: [(c[0] + 0.5) * STEP for c in cells if (c[1] + 0.5) * STEP < -0.7 * s]  # noqa: E731
    assert max(feet(walk)) - min(feet(walk)) > 2 * (max(feet(stand)) - min(feet(stand)))     # striding legs


def test_faces_for():
    assert sf.faces_for("bike", 3) == ["bike"] * 3
    assert sf.faces_for("ped", 2) == ["ped_stand", "ped_walk"]
    assert sf.faces_for("car", 3) == [None] * 3 and sf.faces_for("ped", 3) == [None] * 3


def test_head_meshes_carry_lens_sizes_and_pictograms():
    meshes = json.loads((ROOT / "coupledcarla" / "data" / "street_props.json").read_text())["meshes"]
    heads = {(v["region"], v["role"]): (k, v) for k, v in meshes.items() if v["role"].startswith("signal_")
             and v["role"] != "signal_pole"}
    assert len(heads) == sum(1 for v in meshes.values() if v["role"] in ("signal_car", "signal_bike", "signal_ped"))
    for (region, role), (name, h) in heads.items():
        assert len(h["faces"]) == len(h["lenses"]), name
        pitch = abs(h["lenses"][0][2] - h["lenses"][1][2]) or abs(h["lenses"][0][0] - h["lenses"][1][0])
        assert pitch == pytest.approx(h["lens_d"] + 0.06), name      # lens centres match the lens size
    nl = {r: heads[("NL", r)][1] for r in ("signal_car", "signal_bike", "signal_ped")}
    assert nl["signal_car"]["lens_d"] == 0.2 and nl["signal_car"]["faces"] == [None] * 3        # 200 mm, Dutch urban
    assert nl["signal_bike"]["lens_d"] == 0.2 and nl["signal_bike"]["faces"] == ["bike"] * 3
    assert nl["signal_ped"]["faces"] == ["ped_stand", "ped_walk"]
    assert heads[("UK", "signal_car")][1]["lens_d"] == 0.2 and heads[("JP", "signal_car")][1]["lens_d"] == 0.3
    # the renamed 200 mm car heads say which older mesh a server with older content has, with its lens positions
    for region in ("NL", "UK"):
        name, h = heads[(region, "signal_car")]
        old = h["replaces"]
        assert name.endswith("SignalCar200") and old["path"].endswith(f"SM_CC_{region}_SignalCar")
        assert old["lens_d"] == 0.3 and abs(old["lenses"][0][2]) == pytest.approx(0.36)


def test_head_mesh_pictograms_sit_between_lens_and_lit_strokes():
    sys.path.insert(0, str(ROOT / "tools"))
    import street_props as sp
    m, extra = sp.signal_head(3, 0.20, faces=sf.faces_for("bike", 3))
    parts = m.merged()
    lens_y = -extra["lenses"][0][1]                               # model y of the lens (faces model -y)
    mask = parts["SignalBlack"][0]
    green = parts["LensGreen"][0]
    ys = sorted({round(float(v[1]), 4) for v in green})
    assert ys[-1] == pytest.approx(lens_y, abs=1e-4)                  # the lens itself
    assert ys[0] == pytest.approx(lens_y - sf.MASK_FRONT - 0.001, abs=1e-4)      # its bicycle, on the mask
    assert any(abs(float(v[1]) - (lens_y - sf.MASK_FRONT)) < 1e-4 for v in mask)
    assert sf.MASK_FRONT + 0.001 < sf.LIT_FRONT < 0.16               # the lit strokes in front, inside the visor
    plain, ex2 = sp.signal_head(3, 0.20)
    assert "LensGreen" in plain.merged() and ex2["faces"] == [None] * 3
