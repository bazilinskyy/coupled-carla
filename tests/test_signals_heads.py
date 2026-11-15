"""signals.py head style: lane grouping, head geometry and lamp rows (no SUMO, no CARLA server)."""
import math

import pytest

signals = pytest.importorskip("coupledcarla.signals")


def test_lanes_with_the_same_states_share_a_head():
    phases = ["GGgrrr", "yyyrrr", "rrrGGg", "rrryyy"]
    # lanes 0 and 1 (links 0, 1) always show the same; lane 2 (link 2, a protected left turn 'g') too in this
    # program, so one head for the approach
    assert signals.group_lanes([(0, [0]), (1, [1]), (2, [2])], phases) == [[0, 1, 2]]
    phases = ["GGrrrr", "yyrrrr", "rrGrrr", "rryrrr"]      # lane 2 has its own phase: its own head
    assert signals.group_lanes([(1, [1]), (0, [0]), (2, [2])], phases) == [[0, 1], [2]]
    assert signals.group_lanes([(0, [0]), (1, [1])], []) == [[0], [1]]       # no program: one per lane


def test_head_faces_oncoming_traffic():
    p = signals.head_points(10.0, 20.0, 1.0, 0.0)          # traffic travels along +x
    assert p["yaw"] == pytest.approx(90.0)
    xs = [lx for lx, _, _ in p["lamps"]]
    assert all(x < 10.0 for x in xs)                        # lamps on the face towards the oncoming cars (-x)
    zs = [lz for _, _, lz in p["lamps"]]
    assert zs[0] > zs[1] > zs[2] > 1.0 + 2.0                 # red on top, about 2.3-2.8 m above the road
    assert p["pole"][2] < 1.0 and p["head"][2] == pytest.approx(1.0 + signals.HEAD_Z)
    q = signals.head_points(0.0, 0.0, 0.0, 90.0)             # along +y: the face towards -y
    assert all(ly < 0 for _, ly, _ in q["lamps"]) and all(abs(lx) < 1e-9 for lx, _, _ in q["lamps"])
    assert math.hypot(*q["lamps"][0][:2]) == pytest.approx(signals.LAMP_FRONT)


def test_lamp_rows():
    assert signals.lamp_row("r") == 0 and signals.lamp_row("y") == 1 and signals.lamp_row("G") == 2
    assert signals.lamp_row("u") == 1 and signals.lamp_row("o") is None
