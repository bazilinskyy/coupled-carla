"""tools/nl_sign_faces.py: Commons file names to face keys, derived speed signs, plate outlines, and the texture
mapping onto a plate's front (no network, no rsvg: synthetic drawings)."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
nf = pytest.importorskip("nl_sign_faces")


def test_titles_to_face_keys():
    p = nf.parse_title
    assert p("File:Nederlands verkeersbord B6.svg") == ("B6", "B6", 0)
    assert p("File:Nederlands verkeersbord A1 30 Zone.svg")[0] == "A1-30-ZB"
    assert p("File:Nederlands verkeersbord A1 30 Eindzone.svg")[0] == "A1-30-ZE"
    assert p("File:Netherlands traffic sign A2-60-ZE.svg")[0] == "A2-60-ZE"
    assert p("File:Netherlands traffic sign A1-50.svg")[:2] == ("A1-50", "A1")
    assert p("File:Nederlands verkeersbord A1 130.svg")[0] == "A1-130"
    assert p("File:Nederlands verkeersbord C4 (rechts).svg")[0] == "C4-R"
    assert p("File:Nederlands verkeersbord E1 Zone.svg")[0] == "E1-ZB"
    assert p("File:NL verkeersbord G13.svg") == ("G13", "G13", 1)
    assert p("File:Nederlands verkeersbord G12a.svg")[:2] == ("G12a", "G12a")
    for left_out in ("B6 2023", "C1 mb", "E4 zb", "B1 (old)", "G5 (1988-1990)", "J21 (school)",
                     "G13 tweetalig Nederlands-Fries"):
        assert p(f"File:Nederlands verkeersbord {left_out}.svg") is None, left_out
    assert p("File:Nederlands verkeersbord B6.png") is None and p("File:NL-A10.svg") is None
    picked = nf.pick(["File:NL verkeersbord G13.svg", "File:Nederlands verkeersbord G13.svg",
                      "File:Nederlands verkeersbord J37.svg", "File:Nederlands verkeersbord J37 zonder onderbord.svg"])
    assert picked["G13"][0] == "File:Nederlands verkeersbord G13.svg"
    assert picked["J37"][0].endswith("zonder onderbord.svg")      # the sign without its sub-plate
    assert nf.licence_ok("Public domain") and nf.licence_ok("CC0") and nf.licence_ok("CC BY-SA 4.0")
    assert not nf.licence_ok("GFDL") and not nf.licence_ok("")


def test_derived_speed_number():
    src = ('<svg><text><tspan font-weight="bold" font-size="160" x="50%">50</tspan></text>'
           '<text><tspan font-size="40">ZONE</tspan></text></svg>')
    assert '>30</tspan>' in nf.derive_svg(src, "50", "30") and 'font-size="160"' in nf.derive_svg(src, "50", "30")
    big = nf.derive_svg(src, "50", "120")
    assert '>120</tspan>' in big and 'font-size="123"' in big and "ZONE" in big
    assert nf.derive_svg(src, "60", "30") is None


def test_outlines():
    assert nf.outline(1.0, 0.785, 0.3, 0.3) == "round"
    assert nf.outline(1.0, 0.83, 0.4, 0.4) == "octagon"
    assert nf.outline(1.17, 0.54, 0.05, 1.0) == "triangle"
    assert nf.outline(1.15, 0.54, 1.0, 0.05) == "inv_triangle"
    assert nf.outline(1.0, 0.54, 0.05, 0.05, 0.05, 0.05) == "diamond"
    assert nf.outline(1.05, 0.58, 0.05, 0.05, 1.0, 0.1) == "rect"      # a finger post with its pole
    assert nf.outline(0.8, 1.0, 1.0, 1.0) == "rect"
    table = nf.plates()
    assert nf.choose_plate(0.8, 1.0, 1, 1, 1, 1, table)[0] == "SM_VertRect_10"     # zone board 240 x 300
    assert nf.choose_plate(1.0, 0.785, 0.3, 0.3, 0.3, 0.3, table)[0] == "SM_CircleShape"
    assert nf.width_m("rect", 1.0) == 0.6 and nf.width_m("round", 1.0) == 0.6


def _plate(uv_of):
    """A 100 x 50 cm plate front (x -50..50, z -25..25) of two triangles whose UVs are uv_of(x, z)."""
    corners = [(-50, -25), (50, -25), (50, 25), (-50, 25)]
    tris = [[corners[i] for i in t] for t in ((0, 1, 2), (0, 2, 3))]
    return {"front": [-50, 50, -25, 25], "tris": [[[x, z, *uv_of(x, z)] for x, z in t] for t in tris]}


def test_texture_lands_on_the_front(tmp_path):
    from PIL import Image
    img = np.zeros((50, 100, 4), dtype=np.uint8)
    img[..., 3] = 255
    img[:25, :50, 0] = 255          # drawing: red top-left, green top-right, blue bottom
    img[:25, 50:, 1] = 255
    img[25:, :, 2] = 255
    Image.fromarray(img, "RGBA").save(tmp_path / "d.png")
    # plain map: u with x, v down with z, on the left half of the texture
    nf.compose(tmp_path / "d.png", tmp_path / "a.png", _plate(lambda x, z: ((x + 50) / 200, (25 - z) / 50)), 64)
    a = np.asarray(Image.open(tmp_path / "a.png"))
    assert tuple(a[8, 8]) == (255, 0, 0) and tuple(a[8, 24]) == (0, 255, 0) and tuple(a[48, 16]) == (0, 0, 255)
    assert tuple(a[8, 33]) == (0, 255, 0) and tuple(a[40, 40]) == (0, 0, 255)  # off the front: the nearest colour
    # rotated a quarter turn in UV (as some CARLA plates are): top of the drawing along u = 1
    nf.compose(tmp_path / "d.png", tmp_path / "b.png", _plate(lambda x, z: ((z + 25) / 50, (x + 50) / 100)), 64)
    b = np.asarray(Image.open(tmp_path / "b.png"))
    assert tuple(b[8, 60]) == (255, 0, 0) and tuple(b[56, 60]) == (0, 255, 0) and tuple(b[32, 4]) == (0, 0, 255)


def test_plates_from_a_probe():
    # one plate: a front quad facing +y (Unreal winds it clockwise seen from +y) in group 0, a back in group 1
    f = [[-10, 1, -10, 0, 1], [10, 1, 10, 1, 0], [-10, 1, 10, 0, 0]]
    f2 = [[-10, 1, -10, 0, 1], [10, 1, -10, 1, 1], [10, 1, 10, 1, 0]]
    back = [[-10, 0, -10, 0, 0], [10, 0, 10, 0.1, 0.1], [-10, 0, 10, 0, 0.1]]
    out = nf.plates_from_sections({"SM_CircleShape": {"tris": [[0, f], [0, f2], [1, back]]},
                                   "SM_Unknown": {"tris": [[0, f]]}})
    assert list(out) == ["SM_CircleShape"]
    p = out["SM_CircleShape"]
    assert p["shape"] == "round" and p["front"] == [-10, 10, -10, 10] and p["flat"] and len(p["tris"]) == 2
    assert p["ext"] == [0.1, 0.005, 0.1] and p["path"].endswith("/Signs/SM_CircleShape.SM_CircleShape")


def test_manifest_is_complete():
    man = json.loads(nf.MANIFEST.read_text())
    plates = nf.plates()
    assert man["folder"] == "/Game/CoupledCarla/Signs/NL" and len(man["faces"]) > 200
    for key, f in man["faces"].items():
        assert f["plate"] in plates and nf.licence_ok(f["licence"]) and f["page"].startswith("https://commons")
        assert f["mesh"] == "SM_NL_" + key.replace("-", "_")
    for code in ("E1", "E6", "G11", "C3", "B1", "G7", "E4", "D2", "C2", "B6", "J24", "L8", "A1-30-ZB", "A2-30-ZE"):
        assert code in man["faces"], code
