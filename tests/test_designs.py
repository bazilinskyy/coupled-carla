"""coupledcarla.designs and coupledcarla.ehmi_design: design files (coupled-sim's format), animations, state aliases,
message schedules, strokes, placement on a car, drawing with a fake debug helper, projection. No CARLA."""
import json
import math
import sys
import types

import pytest

from coupledcarla import designs, ehmi_design
from coupledcarla.strokefont import GLYPHS

SHIPPED_EHMI = ["light_bar_cyan", "text_walk", "directional_chevrons", "road_projection", "stopping_bar"]
SHIPPED_INCAR = ["cluster", "hud_speed", "centre_screen"]


def test_colours_and_animations():
    assert designs.parse_color("#FF8000") == (255, 128, 0, 255)
    assert designs.parse_color("#00E5FFAA") == (0, 229, 255, 170)
    assert designs.parse_color("#fff") == (255, 255, 255, 255)
    assert designs.parse_color([0, 190, 255]) == (0, 190, 255, 255)
    assert designs.parse_color("nonsense") == (0, 255, 255, 255)
    b = designs.brightness
    assert b("steady", 1, 0.3) == 1.0
    assert b("blink", 1, 0.2) == 1.0 and b("blink", 1, 0.7) == 0.0
    assert b("pulse", 1, 0.0) == pytest.approx(0.25) and b("pulse", 1, 0.5) == pytest.approx(1.0)
    assert b("sweep", 1, 0.5, pos=0.3) == 1.0 and b("sweep", 1, 0.5, pos=0.8) == 0.12
    assert b("sweepleft", 1, 0.5, pos=0.8) == 1.0
    assert b("sweepout", 1, 0.3, pos=0.5) == 1.0 and b("sweepout", 1, 0.3, pos=0.0) == 0.12
    assert b("sweepin", 1, 0.3, pos=0.0) == 1.0


def test_shipped_designs_load_and_alias_states():
    for name in SHIPPED_EHMI:
        d = designs.load(name, "ehmi")
        assert d["width"] > 0 and d["states"]
        json.dumps(d)                                       # sent to stations in the welcome
        assert designs.elements_for(d, "off") == []         # off draws nothing
        assert designs.elements_for(d, "yielding")          # coupled-carla's names
        assert designs.elements_for(d, "WALK") or designs.elements_for(d, "go")
    for name in SHIPPED_INCAR:
        d = designs.load(name, "incar")
        assert set(designs.state_names(d)) >= {"TAKEOVER"} or "ANY" in d["states"]
    cyan = designs.load("light_bar_cyan")
    assert designs.elements_for(cyan, "driving")[0]["animation"] == "sweep"      # STOP
    assert designs.elements_for(cyan, "go")[0]["animation"] == "pulse"           # WALK
    own = designs.load("stopping_bar")
    assert designs.elements_for(own, "WALK")[1]["text"] == "I'M STOPPING"        # WALK -> yielding
    with pytest.raises(FileNotFoundError):
        designs.load("no_such_design")


def test_inline_design_and_yaml_shorthand(tmp_path):
    d = designs.load({"width": 1.0, "height": 0.2, "states": {"go": [{"type": "text", "text": "GO"}]}})
    assert d["states"]["go"][0]["text"] == "GO" and d["background"] == (0, 0, 0, 255)
    p = tmp_path / "mine.yaml"
    p.write_text("width: 2\nheight: 0.5\nplacement: road\nstates:\n  WALK:\n    - {type: rect, color: '#FFFFFF'}\n")
    d2 = designs.load(str(p))
    assert d2["placement"] == "road" and designs.elements_for(d2, "yielding")
    assert designs.normalise(d2) == d2 or designs.normalise(d2)["states"] == d2["states"]   # idempotent


def test_fill_holds_and_schedule():
    assert designs.fill("{speed} km/h, {gear}", {"speed": 42, "gear": "D3"}) == "42 km/h, D3"
    assert designs.fill("{unknown}", {}) == "{unknown}"
    assert designs.holds("", {}) and designs.holds("left", {"left": True})
    assert not designs.holds("!left", {"left": True}) and designs.holds("whatever", {})
    csv = ("at;seconds;display;text;color;atDistance;onState\n"
           "2;4;hud;Please drive to the end of the street;#FFFFFF;;\n"
           "# a comment\n"
           ";5;centre;Turn right at the next junction;#FFFFFF;150;\n"
           ";4;all;Automation engaged;#00E5FF;;AUTOMATED\n")
    ms = designs.parse_schedule(csv)
    assert [m["text"] for m in ms] == ["Please drive to the end of the street", "Turn right at the next junction",
                                       "Automation engaged"]
    assert ms[0]["at"] == 2 and ms[0]["atDistance"] == -1 and ms[1]["atDistance"] == 150
    assert ms[2]["onState"] == "AUTOMATED" and ms[2]["at"] == -1
    assert designs.norm_message({"at_m": 30, "on_state": "TAKEOVER", "text": "x"})["atDistance"] == 30
    with pytest.raises(ValueError):
        designs.parse_schedule("at;seconds\n1;2\n")


def test_font_has_digits_and_letters():
    for ch in "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ'!-.:/?":
        assert ch in GLYPHS


def test_strokes_of_elements():
    d = designs.load("directional_chevrons")
    st = ehmi_design.strokes(d, "go", 0.0)
    assert st and all(len(s) == 7 for s in st)
    assert st[0][5] == (10, 10, 10)                         # the opaque background first
    for u0, v0, u1, v1, th, rgb, f in st:
        assert -1e-6 <= u0 <= d["width"] + 1e-6 and -1e-6 <= v1 <= d["height"] + 1e-6 and th > 0
    assert ehmi_design.strokes(d, "off", 0.0) == []
    blink = ehmi_design.strokes(d, "driving", 0.7)          # STOP: a blinking red bar, in its off phase
    red = [s for s in blink if s[5] == (255, 59, 48)]
    assert red and all(s[6] == 0.0 for s in red)
    road = designs.load("road_projection")
    assert not any(s[5] == (0, 0, 0) for s in ehmi_design.strokes(road, "yielding", 0.0))   # transparent bg
    text = []
    ehmi_design.text_strokes(text, "30", 0, 0, 1.0, 0.3, (255, 255, 255), 1.0)
    assert len(text) == (3 + 1) + (4 + 1)                   # "3": a polyline of 3 and a bar; "0": 4 and a diagonal
    big = []
    ehmi_design._fill(big, 0, 0, 1.0, 0.3, (1, 2, 3), 1.0)
    assert len(big) == 4 and all(abs(s[4] - 0.075) < 1e-9 for s in big)


def test_frames_place_the_panel_on_the_car():
    W, H = 1.2, 0.2
    ext = (2.3, 0.9, 0.75)
    (c, r, u), = ehmi_design.frames("front", (0, 0, 0), 0.0, ext, W, H)
    assert c[0] == pytest.approx(2.36) and c[2] == pytest.approx(0.75 * 1.05)
    assert r == (-0.0, -1.0, 0.0) or r[1] == pytest.approx(-1.0)     # the reader's right is the car's left
    (c, r, u), = ehmi_design.frames("rear", (0, 0, 0), 0.0, ext, W, H)
    assert c[0] < 0 and r[1] == pytest.approx(1.0)
    (c, r, u), = ehmi_design.frames("road", (10, 5, 0.2), 90.0, ext, 3.0, 2.4)
    assert c[1] == pytest.approx(5 + 2.3 + 1.8 + 1.2) and c[2] == pytest.approx(0.26)
    assert u[1] == pytest.approx(-1.0)                       # the top of the projection towards the car
    (c, r, u), = ehmi_design.frames("roof", (0, 0, 0), 0.0, ext, W, H)
    assert c[2] == pytest.approx(1.5 + 0.1 + 0.1) and r[1] == pytest.approx(-1.0)
    assert ehmi_design.frames("roof_rear", (0, 0, 0), 0.0, ext, W, H)[0][1][1] == pytest.approx(1.0)
    assert ehmi_design.frames("front", (0, 0, 0), 0.0, ext, W, H, offset=[0.5, 0, 0.1])[0][0][0] == \
        pytest.approx(2.86)
    with pytest.raises(ValueError):
        ehmi_design.frames("bonnet_ornament", (0, 0, 0), 0.0, ext, W, H)
    st = [(0.0, 0.0, W, 0.0, 0.05, (1, 1, 1), 1.0)]
    (p0, p1, th, rgb, f), = ehmi_design.to_world(st, ((0, 0, 1), (0, -1, 0), (0, 0, 1)), W, H)
    assert p0 == pytest.approx((0, 0.6, 0.9)) and p1 == pytest.approx((0, -0.6, 0.9))


def test_designed_ehmi_draws_with_debug_lines(monkeypatch):
    fake = types.ModuleType("carla")
    fake.Location = lambda x, y, z: (x, y, z)
    fake.Color = lambda r, g, b: (r, g, b)
    monkeypatch.setitem(sys.modules, "carla", fake)
    lines = []
    debug = types.SimpleNamespace(draw_line=lambda a, b, thickness, color, life_time: lines.append((a, b, color)))
    ext = types.SimpleNamespace(x=2.3, y=0.9, z=0.75)
    loc = types.SimpleNamespace(x=100.0, y=50.0, z=0.0)
    vehicle = types.SimpleNamespace(get_transform=lambda: types.SimpleNamespace(
        location=loc, rotation=types.SimpleNamespace(yaw=90.0)), bounding_box=types.SimpleNamespace(extent=ext))
    de = ehmi_design.DesignedEhmi(designs.load("stopping_bar"))
    n = ehmi_design.draw_world(debug, vehicle, "yielding", de, 10.0, 0.06)
    assert n > 0 and len(lines) <= n
    assert all(abs(a[0] - 100.0) < 1.0 for a, b, c in lines)          # on the car's front and roof (it faces +y)
    assert max(a[1] for a, b, c in lines) > 50.0 + 2.3
    lines.clear()
    assert ehmi_design.draw_world(debug, vehicle, "off", de, 10.5, 0.06) == 0 and not lines
    de.segments("go", 11.0, (0, 0, 0), 0, (2, 1, 1))
    assert de.since == 11.0                                   # animations restart with the state


def test_project_and_for_agents():
    cam = (0, 0, 1.2, 0, 0, 0)
    u, v, depth = ehmi_design.project((10, 0, 1.2), cam, (1280, 720), 90)
    assert (u, v, depth) == pytest.approx((640, 360, 10))
    u, v, _ = ehmi_design.project((10, 10, 1.2), cam, (1280, 720), 90)
    assert u == pytest.approx(1280)                            # 45 degrees right: the edge at fov 90
    assert ehmi_design.project((-5, 0, 1), cam, (1280, 720), 90) is None
    u, v, _ = ehmi_design.project((0, 10, 1.2), (0, 0, 1.2, 0, 90, 0), (1280, 720), 90)
    assert u == pytest.approx(640)                              # yawed to +y
    segs = [((10, -1, 1.2), (10, 1, 1.2), 0.1, (255, 0, 0), 1.0), ((-3, 0, 0), (-3, 1, 0), 0.1, (1, 1, 1), 1.0)]
    out = ehmi_design.project_segments(segs, cam, (1280, 720), 90)
    assert len(out) == 1 and out[0][4] == round(640 * 0.1 / 10)
    logs = []
    tc = {"ehmi": {"design": "light_bar_cyan", "placement": "bumper"},
          "agents": [{"id": "av1", "kind": "car"}, {"id": "car2", "ehmi_design": "text_walk"},
                     {"id": "w", "kind": "walker"}, {"id": "car3", "ehmi_design": "missing_one"}]}
    specs = ehmi_design.for_agents(tc, log=logs.append)
    assert set(specs) == {"av1", "car2"} and specs["av1"]["placement"] == "bumper"
    assert specs["car2"]["design"]["name"].startswith("Text") and any("car3" in m for m in logs)
    assert set(ehmi_design.drawers(specs)) == {"av1", "car2"}
    assert ehmi_design.for_agents({"agents": [{"id": "a"}]}) == {}
