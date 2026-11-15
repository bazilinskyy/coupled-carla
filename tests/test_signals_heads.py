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


def test_lights_of_one_approach_share_heads():
    # an approach with a light per turn (maps from OpenDRIVE): lanes 0 and 1 go straight by light "s" with the same
    # states, so they share a head even though light "r" (lane 0's right turn) is another programme on that edge;
    # grouped per light, the approach had a head per light on one spot
    phases = {"r": ["G", "y", "r", "r"], "s": ["GGr", "yyr", "rrG", "rry"]}
    lanes = [(0, [("s", 0)]), (1, [("s", 1)]), (2, [("s", 2)]), (3, [("r", 0)])]
    assert signals.group_links(lanes, phases) == [[0, 1], [2], [3]]
    # lane 0 shows light "r" and "s": not the same as lane 1, which shows only "s"
    assert signals.group_links([(0, [("r", 0), ("s", 0)]), (1, [("s", 1)])], phases) == [[0], [1]]
    assert signals.group_links(lanes, {"s": phases["s"]}) == [[0, 1], [2], [3]]   # a light without a programme
    assert signals.group_lanes([(1, [1]), (0, [0])], ["GG", "yy"]) == [[0, 1]]


def test_lamp_is_a_square_in_the_world():
    a, b = signals.lamp_segment((10.0, 5.0, 3.0), 90.0)
    assert a[2] == b[2] == 3.0 and math.dist(a, b) == pytest.approx(signals.LAMP_SIZE)
    assert a[0] == pytest.approx(10.0) and b[0] == pytest.approx(10.0)       # across the face (local x)
    assert (a[1] + b[1]) / 2 == pytest.approx(5.0)


def test_pole_reaches_down_to_the_ground():
    p = signals.head_points(0.0, 0.0, 5.0, 0.0)
    q = signals.head_points(0.0, 0.0, 5.0, 0.0, ground=4.6)
    foot = lambda pt: pt["pole"][2] + pt["pole_scale"] * signals.POLE_BASE
    top = lambda pt: foot(pt) + pt["pole_scale"] * signals.POLE_LEN
    assert foot(p) == pytest.approx(5.0, abs=0.01) and foot(q) == pytest.approx(4.6)
    assert top(q) == pytest.approx(top(p)) and q["pole_scale"] > p["pole_scale"]
    assert q["head"] == p["head"] and q["lamps"] == p["lamps"]


def test_head_spots_stay_off_the_lanes():
    groups = [[0], [1], [2]]
    lanes = {0: ([], "L0"), 1: ([], "L1"), 2: ([], "L2")}
    calls = []

    def place(lane, kerb, out=0.0):
        calls.append((lane, kerb, out))
        return ((lane, kerb, out), 0.0)
    # a median beside the innermost lane: the second and third heads stand there, side by side
    spots = [signals.head_spot(k, groups, lanes, place, lambda loc: False)[0] for k in range(3)]
    assert spots == [("L0", True, 0.0), ("L2", False, 0.0), ("L2", False, signals.SIDE_BY_SIDE)]
    # no median (the other direction's lane is there): beside the kerb head, further out
    spots = [signals.head_spot(k, groups, lanes, place, lambda loc: not loc[1])[0] for k in range(3)]
    assert spots == [("L0", True, 0.0), ("L0", True, signals.SIDE_BY_SIDE), ("L0", True, 2 * signals.SIDE_BY_SIDE)]


def test_head_candidates_go_out_then_back():
    c = signals.head_candidates(True)
    assert c[0] == (0.0, 0.0) and c[1] == (0.0, 0.5) and (5.0, 2.0) in c
    assert [b for b, _ in c] == sorted(b for b, _ in c)                 # nearer the stop line first
    assert signals.head_candidates(False) == [(b, 0.0) for b in (0.0, 1.0, 2.0, 3.5, 5.0)]
    assert signals.head_candidates(True, 1.5)[-1] == (1.0, 2.0)          # a short lane: not further back than it
