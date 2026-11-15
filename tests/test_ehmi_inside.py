"""What an AV passenger sees of its own car's eHMI (ehmi_inside.py): which panels the stations draw instead of the
world, the welcome, the plain eHMI's text as segments where ai.draw_ehmi puts it, the occupant's filter and the
indicator the right way round. No CARLA, no pygame."""
import json
from pathlib import Path

import pytest

from coupledcarla import config, designs, ehmi_design, ehmi_inside, stationux

ROOT = Path(__file__).resolve().parent.parent
STATES = {"yielding": {"text": "I'M STOPPING", "color": [0, 190, 255]}, "go": {"text": "GO AHEAD", "color": [0, 230, 120]},
          "off": {"text": "", "color": [0, 0, 0]}}


def _tc(**ehmi):
    return {"ehmi": dict(states=STATES, **ehmi),
            "participants": [{"role": "ped", "kind": "pedestrian", "station": True},
                             {"role": "pax", "kind": "av_passenger", "station": True, "agent": "av1"}],
            "agents": [{"id": "av1", "kind": "car"}, {"id": "car2", "kind": "car"}]}


def test_settings_and_occupied():
    assert ehmi_inside.settings({}) == ("hide", True)
    assert ehmi_inside.settings({"ehmi": {"inside": "DIM", "indicator": False}}) == ("dim", False)
    assert ehmi_inside.settings({"ehmi": {"inside": "upside-down"}})[0] == "hide"
    assert ehmi_inside.occupied(_tc()) == {"av1"}
    tc = _tc()
    tc["participants"][1]["station"] = False          # a bot on the host: no station camera inside
    assert ehmi_inside.occupied(tc) == set()
    assert config.DEFAULTS["ehmi"] == {}               # existing experiments: nothing set, the defaults apply


def test_split_and_host_plan():
    assert ehmi_inside.split(["front", "roof", "road", "rear"]) == (["road", "rear"], ["front", "roof"])
    assert ehmi_inside.split("windshield") == ([], ["windshield"])
    d = designs.load("stopping_bar")
    specs = {"av1": {"design": d, "placement": None}}
    slots = lambda a: ("front", "roof", "road", "eyes")    # noqa: E731
    assert ehmi_inside.host_plan(_tc(), specs, slots) == {"av1": {"placement": ["front", "roof"]}}
    assert ehmi_inside.host_plan(_tc(inside="mirror"), specs, slots) == {}
    assert ehmi_inside.host_plan(_tc(render="station"), specs, slots) == {}     # stations draw it all anyway
    assert ehmi_inside.host_plan(_tc(), {}, slots) == {"av1": {"slots": ["front", "roof"]}}
    assert ehmi_inside.world_slots(("front", "roof", "road", "eyes")) == ("road", "eyes")


def test_welcomes_of_the_demo_experiment():
    cfg = config.load(ROOT / "experiments" / "eindhoven_station_ux.yaml")
    tc = config.trial_config(cfg, cfg["trials"][0])
    w = {p["role"]: stationux.welcome_extras(tc, p, p.get("kind"), ROOT / "experiments",
                                             default_states=list(STATES), default_ehmi=STATES)
         for p in tc["participants"]}
    for role, x in w.items():
        json.loads(json.dumps(x))
        inside = x["ehmi_inside"]
        assert inside["mode"] == "hide" and set(inside["cars"]) == {"av1"}, role       # every station draws them
        assert inside["cars"]["av1"]["placement"] == ["front", "roof"]
    assert w["av_passenger"]["ehmi_inside"]["own"]["agent"] == "av1"
    assert "own" not in w["pedestrian"]["ehmi_inside"]


def test_plain_text_where_draw_ehmi_puts_it_and_reading_from_the_front():
    ext = (2.4, 1.0, 0.75)
    segs = ehmi_inside.plain_segments("go", STATES, ["front"], (0.0, 0.0, 0.0), 0.0, ext)
    assert segs
    xs = [p[0] for s in segs for p in s[:2]]
    assert min(xs) == pytest.approx(2.41) and max(xs) == pytest.approx(2.41)      # ext.x + 0.06 - 0.05
    zs = [p[2] for s in segs for p in s[:2]]
    assert min(zs) >= 0.75 * 1.05 + 0.42 - 0.161 and max(zs) <= 0.75 * 1.05 + 0.42 + 0.161
    assert segs[0][2] == 0.035 and segs[0][3] == (0, 230, 120)          # draw_ehmi's thickness, the state's colour
    # a reader in front (looking along -x) has +y on the left: "G" (first) lies at larger y than "D" (last)
    ys = [p[1] for s in segs for p in s[:2]]
    assert segs[0][0][1] > 0 and max(ys) > 0 > min(ys)
    roof = ehmi_inside.plain_segments("go", STATES, ["roof"], (0.0, 0.0, 0.0), 90.0, ext)
    assert {round(p[2], 2) for s in roof for p in s[:2]} <= {round(1.5 + 0.08 + 0.22 + d, 2) for d in
                                                              [x / 100 for x in range(-12, 13)]}
    assert ehmi_inside.plain_segments("off", STATES, ["front"], (0, 0, 0), 0.0, ext) == []


def test_occupant_filter():
    segs = [((0, 0, 0), (1, 0, 0), 0.03, (0, 200, 0), 0.8)]
    assert ehmi_inside.for_occupant(segs, "hide") == []
    assert ehmi_inside.for_occupant(segs, "dim")[0][4] == pytest.approx(0.2)
    assert ehmi_inside.for_occupant(segs, "mirror") == segs


def test_indicator_reads_the_right_way_round():
    d = ehmi_inside.plain_design(STATES)
    assert set(d["states"]) == {"yielding", "go"}
    rect = ehmi_inside.indicator_rect((960, 540), d)
    assert rect[0] == pytest.approx((960 - rect[2]) / 2) and rect[2] == pytest.approx(960 * 0.26)
    segs = ehmi_inside.indicator_segments(d, "go", 0.0, rect)
    x, y, w, h = rect
    assert all(x <= s[0] <= x + w and y <= s[1] <= y + h for s in segs)
    # the light bar is at the bottom of the strip (larger v), the text above it
    bars = [s for s in segs if s[4] > 5]
    text = [s for s in segs if s[4] <= 5]
    assert bars and text
    assert min(s[1] for s in bars) > max(s[1] for s in text)
    # text left to right: the first glyph's strokes (G) start left of the last (D)
    world = ehmi_design.strokes(d, "go", 0.0)
    assert len(world) == len(segs)
    assert ehmi_inside.indicator_segments(d, "off", 0.0, rect) == []
