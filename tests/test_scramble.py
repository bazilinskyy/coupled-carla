"""Busy and scramble crossings (coupledcarla/scramble.py, pedsignals.py): geometry, signal, programmes and the walkers'
model run kinematically without CARLA."""
import json
import math
import random
import xml.etree.ElementTree as ET

import pytest

from coupledcarla import pedsignals as P
from coupledcarla import scramble as S
from coupledcarla.crowdpeds import Segment

TIMING = {"lead_s": 2.0, "walk_s": 35.0, "flash_s": 7.0, "clear_s": 3.0, "cycle_s": 120.0}


def _road(sid, heading_deg, half=6.0, start=8.0, end=60.0, step=2.5):
    """A road leaving the junction at (0, 0) along heading: its two kerbs (half m either side), sampled every step."""
    hx, hy = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    nx, ny = -hy, hx
    n = int((end - start) / step) + 1
    a = [(hx * (start + i * step) + nx * half, hy * (start + i * step) + ny * half) for i in range(n)]
    b = [(hx * (start + i * step) - nx * half, hy * (start + i * step) - ny * half) for i in range(n)]
    return Segment(sid, a, b, step, junction=(True, True))


def four_way():
    return [_road("E", 0), _road("N", 90), _road("W", 180), _road("S", 270)]


def scramble_signal(t, cycle=120.0, red_from=73.0):
    """Vehicles have green or amber until red_from in each cycle, then all red to its end."""
    c = t % cycle
    return (c >= red_from), (cycle - c if c >= red_from else 0.0)


# --- the signal -------------------------------------------------------------------------------------------------------

def test_stage_walk_flash_clear():
    assert S.stage(False, 0.0, 0.0, TIMING) == "dont"
    assert S.stage(True, 47.0, 0.5, TIMING) == "dont"          # cars just stopped: the lead
    assert S.stage(True, 45.0, 2.0, TIMING) == "walk"
    assert S.stage(True, 10.0 + 0.1, 40.0, TIMING) == "walk"   # 7.1 s of flashing and clearance still to come
    assert S.stage(True, 9.0, 40.0, TIMING) == "flash"
    assert S.stage(True, 2.0, 40.0, TIMING) == "clear"


def test_red_ahead_reads_the_programme():
    phases = [(30.0, "GGrr"), (3.0, "yyrr"), (2.0, "rrrr"), (30.0, "rrGG"), (3.0, "rryy"), (47.0, "rrrr", "scramble")]
    assert P.red_ahead(phases, 0, 10.0, [0]) == 0.0                      # green now
    assert P.red_ahead(phases, 2, 1.5, [0, 1]) == pytest.approx(1.5 + 30 + 3 + 47)
    assert P.red_ahead(phases, 5, 20.0, [0, 1, 2, 3]) == pytest.approx(20.0)   # the scramble: all four
    assert P.red_ahead([(10.0, "r")], 0, 4.0, [0]) == P.LOOKAHEAD_S       # never green


def test_pedsignals_red_remaining():
    phases = [(30.0, "Gr"), (3.0, "yr"), (20.0, "rG"), (3.0, "ry"), (40.0, "rr", "scramble")]
    now = {"i": 4, "left": 25.0}
    sig = P.PedSignals([("J", 0, (0.0, 0.0), (1.0, 0.0)), ("J", 1, (0.0, 5.0), (0.0, 1.0))],
                       lambda t: phases[now["i"]][1], lambda t: (phases, now["i"], now["left"]))
    red, rem, name = sig.red_remaining([("J", 0), ("J", 1)])
    assert red and rem == pytest.approx(25.0) and name == "scramble"
    now["i"] = 0
    assert sig.red_remaining([("J", 0)])[0] is False
    red, rem, _ = sig.red_remaining([("J", 1)])                         # link 1: red until its own green
    assert red and rem == pytest.approx(25.0 + 3.0)
    assert sig.tls_links(["J"]) == [("J", 0), ("J", 1)]


# --- programmes -----------------------------------------------------------------------------------------------------

def test_scramble_programme_in_step():
    a = {"id": "s0", "nodes": ["1"], "links": [], "type": "actuated",
         "phases": [{"duration": 20.0, "state": "GGrr", "minDur": 6, "maxDur": 40},
                    {"duration": 3.0, "state": "yyrr"}, {"duration": 2.0, "state": "rrrr"},
                    {"duration": 25.0, "state": "rrGG"}, {"duration": 3.0, "state": "rryy"}]}
    b = {"id": "s1", "nodes": ["2"], "links": [], "type": "static",
         "phases": [{"duration": 43.0, "state": "Gg"}]}
    cyc = P.site_cycle([a, b], TIMING)
    assert cyc == pytest.approx(120.0)
    pa, pb = P.scramble_programme(a, TIMING, cyc), P.scramble_programme(b, TIMING, cyc)
    for prog in (pa, pb):
        assert prog["type"] == "static" and prog["offset"] == 0.0
        assert sum(p["duration"] for p in prog["phases"]) == pytest.approx(120.0)
        last = prog["phases"][-1]
        assert last["name"] == "scramble" and set(last["state"]) == {"r"} and last["duration"] == 47.0
    assert pb["phases"][-2]["name"] == "amber" and pb["phases"][-2]["state"] == "yy"   # green before: amber first
    # both all-red phases start at 73 s: in step
    assert sum(p["duration"] for p in pa["phases"][:-1]) == pytest.approx(73.0)
    assert sum(p["duration"] for p in pb["phases"][:-1]) == pytest.approx(73.0)
    # a controller that cannot fit lengthens the junction's cycle for all of them
    long = {"id": "x", "phases": [{"duration": 100.0, "state": "Gr"}, {"duration": 50.0, "state": "rG"}]}
    assert P.site_cycle([a, long], TIMING) > 120.0


def test_scramble_programs_cycle_sync(tmp_path):
    programs = {"J": {"0": [(30.0, "GGrr"), (3.0, "yyrr"), (30.0, "rrGg")]}, "K": {"0": [(42.0, "G"), (3.0, "y")]}}
    assert P.scramble_programs(programs, ["J", "K"], tmp_path / "s.add.xml", seconds=40, cycle=120) == 2
    for tl in ET.parse(tmp_path / "s.add.xml").getroot().iter("tlLogic"):
        ph = [(float(p.get("duration")), p.get("state"), p.get("name")) for p in tl.iter("phase")]
        assert sum(d for d, _, _ in ph) == pytest.approx(120.0)
        assert ph[-1][2] == "scramble" and set(ph[-1][1]) == {"r"}


def test_scale_greens_keeps_ambers():
    out = P.scale_greens([(30.0, "Gr"), (3.0, "yr"), (30.0, "rG"), (3.0, "ry")], 100.0)
    assert sum(d for d, _ in out) == pytest.approx(100.0) and out[1][0] == 3.0 and out[3][0] == 3.0
    out = P.scale_greens([(30.0, "GrG"), (3.0, "yrG"), (30.0, "rGG"), (3.0, "ryG")], 80.0)   # a stream green throughout
    assert out[1][0] == 3.0 and out[3][0] == 3.0 and sum(d for d, _ in out) == pytest.approx(80.0)


def test_uncoordinated_lights():
    class N:
        pass
    n = N()
    n.edges = {"a": {"to": "J"}, "b": {"to": "J"}}
    n.conns = [{"from": "a", "tl": "a:x"}, {"from": "b", "tl": "b:y"}]
    n.tls = {"a:x": ["g", "y", "r"], "b:y": ["g", "y", "r"]}
    n.tls_durations = {"a:x": [42, 3, 3], "b:y": [42, 3, 3]}
    assert P.uncoordinated(n)                      # two lights at one junction (OpenDRIVE's per-signal lights)
    n.conns = [{"from": "a", "tl": "J"}, {"from": "b", "tl": "J"}]
    n.tls = {"J": ["Gr", "yr", "rG", "ry"]}
    n.tls_durations = {"J": [30, 3, 30, 3]}
    assert not P.uncoordinated(n)                  # one light with stages
    n.tls = {"J": ["GG", "yy", "rr"]}
    n.tls_durations = {"J": [42, 3, 3]}
    assert P.uncoordinated(n)                      # one light, all green together, 3 s red: no time to cross


# --- where ----------------------------------------------------------------------------------------------------------

def test_osm_crossings_json_and_clusters(tmp_path):
    data = {"elements": [
        {"type": "node", "id": 1, "lat": 35.6595, "lon": 139.7005,
         "tags": {"highway": "crossing", "crossing": "traffic_signals", "crossing:scramble": "yes"}},
        {"type": "way", "id": 2, "tags": {"footway": "crossing", "crossing:scramble": "yes"},
         "geometry": [{"lat": 35.6596, "lon": 139.7004}, {"lat": 35.6594, "lon": 139.7007}]},
        {"type": "node", "id": 3, "lat": 35.70, "lon": 139.80, "tags": {"crossing": "traffic_signals"}}]}
    (tmp_path / "real").mkdir()
    (tmp_path / "real" / "osm_crossings.json").write_text(json.dumps(data))
    got = P.osm_crossings(tmp_path, fetch=False)
    assert len(got["scramble"]) == 3 and got["signals"] == [(35.6595, 139.7005), (35.70, 139.80)]
    cs = P.clusters([(0, 0), (30, 0), (55, 0), (500, 500)], reach=40)
    assert cs[0][2] == 3 and cs[1][:2] == (500, 500)
    centres = P.scramble_centres(tmp_path, None, lambda la, lo: (lo * 1000, la * 1000), fetch=False)
    assert len(centres) == 1 and centres[0][4] == 3
    assert P.osm_crossings(tmp_path / "nothing", fetch=False) == {"scramble": [], "signals": []}


def test_published_counts_near_shibuya():
    pub = S.published(35.6595, 139.7006)
    assert pub and pub["per_cycle_peak"] == 3000
    assert S.published(51.44, 5.48) is None


def test_movement_from_crowd():
    jp = S.movement("JPN")
    assert jp["stopped_pct"] == pytest.approx(3.78) and jp["from"] == "Japan"
    assert S.movement("ZZZ")["from"] == "16 countries"


def test_scramble_site_four_way():
    site = S.make_site("scramble", four_way(), (0.0, 0.0), carla=False)
    assert len(site.corners) == 4
    kinds = sorted(lg.kind for lg in site.legs)
    assert kinds == ["diagonal", "diagonal", "side", "side", "side", "side"]
    for lg in site.legs:
        assert lg.a.corner != lg.b.corner
        if lg.kind == "side":
            assert lg.length == pytest.approx(12.0)           # across a road, kerb to kerb
        else:
            assert lg.length > 15.0                           # corner to opposite corner
    # waiting spots: rows behind each kerb end, on the pavement side (away from the road)
    assert len(site.slots) > 4 * 2 * 4
    for x, y, k, row, along in site.slots:
        e = site.ends[k]
        assert (x - e.p[0]) * e.back[0] + (y - e.p[1]) * e.back[1] >= -1e-6
    # the best spots for a walker crossing from an end are next to it, front row first
    e = site.ends[0]
    first = site.slots[site.order[e.k][0]]
    assert first[3] == 0 and math.dist(first[:2], e.p) < 1.0


def test_busy_site_crosses_each_road():
    site = S.make_site("busy", four_way(), (0.0, 0.0), carla=False)
    assert len(site.legs) == 4 and all(lg.kind == "busy" and lg.a.seg == lg.b.seg for lg in site.legs)
    assert S.make_site("busy", four_way(), (500.0, 500.0), carla=False) is None


def test_kerb_ends_carla_coordinates():
    ends = S.kerb_ends(four_way(), (0.0, 0.0), carla=True)
    assert len(ends) == 8
    assert any(e.p == pytest.approx((8.0, -6.0)) for e in ends)       # geo (8, 6) -> CARLA (8, -6)


# --- who ------------------------------------------------------------------------------------------------------------

def test_draw_units_kinds_and_groups():
    sc = S.settings({})
    units = S.draw_units(400, random.Random(3), sc, "scramble", {"speed_median": 1.3, "speed_sd": 0.16},
                         S.movement("JPN"), rain_mm_h=0.0)
    people = [m for u in units for m in u]
    assert len(people) == 400
    kinds = {m["kind"] for m in people}
    assert {"commuter", "group", "tourist", "older"} <= kinds
    assert any(m.get("child") for m in people)
    assert sum(len(u) > 1 for u in units) > 30                          # groups at moderate density
    assert any(m["photo"] for m in people) and all(m["kind"] == "tourist" for m in people if m["photo"])
    assert not any(m["umbrella"] for m in people)
    older = [m["speed"] for m in people if m["kind"] == "older"]
    com = [m["speed"] for m in people if m["kind"] == "commuter"]
    assert sum(older) / len(older) < 0.8 * sum(com) / len(com)
    wet = S.draw_units(200, random.Random(3), sc, "busy", {}, S.movement("JPN"), rain_mm_h=3.0)
    share = sum(m["umbrella"] for u in wet for m in u) / 200
    assert 0.6 < share < 0.95


def test_per_cycle_and_population():
    site = S.make_site("scramble", four_way(), (0.0, 0.0), carla=False)
    n, why = S.per_cycle(site, {"density_factor": 1.4}, 2.0)
    assert n == pytest.approx(6 * 6 * 1.4 * 2.0) and "per leg" in why
    n, why = S.per_cycle(site, {}, 1.0, pub={"name": "X", "per_cycle": 1500})
    assert n == 1500 and "published" in why
    assert S.population(1500, 120, 360) == 360 and S.population(20, 120, 360) == 18


# --- steering -------------------------------------------------------------------------------------------------------

def test_steer_spacing_and_keep_side():
    (dx, dy), v = S.steer(0, 0, 10, 0, 1.3, [])
    assert (dx, dy) == pytest.approx((1.0, 0.0)) and v == pytest.approx(1.3)
    (dx, dy), v = S.steer(0, 0, 10, 0, 1.3, [(0.6, 0.0, 0.5, 0.0)])     # a slower walker right in front
    assert v <= 0.55
    (dx, dy), v = S.steer(0, 0, 10, 0, 1.3, [(1.0, 0.0, -1.2, 0.0)], keep_left=True)   # someone coming
    assert dy < -0.05                      # steps to its left: in CARLA's x, y the left of +x is -y
    (dx2, dy2), _ = S.steer(0, 0, 10, 0, 1.3, [(1.0, 0.0, -1.2, 0.0)], keep_left=False)
    assert dy2 > 0.05


# --- the model, kinematically -----------------------------------------------------------------------------------------

def run_model(seconds=250.0, n=110, dt=0.12, signal=scramble_signal, kind="scramble", jay=0.0, photo=False,
              hidden=None, seed=2):
    site = S.make_site(kind, four_way(), (0.0, 0.0), carla=False)
    sc = S.settings({})
    units = S.draw_units(n, random.Random(seed), sc, kind, {"speed_median": 1.3, "speed_sd": 0.16},
                         {"stopped_pct": 60.0 if photo else 3.78, "with_others_pct": 80.0})
    model = S.CrossingModel([site], [units], {**sc, **TIMING}, {"jaywalk_share": jay}, seed=seed, keep_left=True)
    pos = {p.i: p.pos for p in model.peds}
    t, log = 0.0, []
    while t < seconds:
        cmds = model.step(t, dict(pos), lambda s, lg: signal(t), vehicles=lambda: [], hidden=hidden,
                          viewers=[(-40.0, 0.0, 0.0)], dt=dt)
        for i, c in cmds.items():
            if c[0] == "walk":
                (dx, dy), v = c[1], c[2]
                pos[i] = (pos[i][0] + dx * v * dt, pos[i][1] + dy * v * dt)
            elif c[0] == "teleport":
                pos[i] = (c[1], c[2])
        log.append((t, sum(1 for p in model.peds if p.state in ("queue", "toslot")),
                    len(model.on_road()), [p.i for p in model.peds if p.state in ("cross", "photo")]))
        t += dt
    return model, log


def test_model_waves_on_the_scramble_phase_only():
    model, log = run_model()
    assert len(model.phases) >= 2
    for rec in model.phases:
        assert rec["crossers"] > 30
        assert rec["diagonal"] > 0                                       # both diagonals are used
        assert rec["queued"] > 20
    # nobody starts on red: every crossing begins inside a walk phase (or within its flashing for late runners)
    for c in model.crossings:
        cyc = c["t_start"] % 120.0
        assert 73.0 + 2.0 - 0.2 <= cyc <= 120.0 - 3.0 + 0.5 or c["t_start"] < 1.0, c
    # and ends before the cars' green, but for a few stragglers held up by the crowd (cars then wait for them)
    over = [c["t_end"] - (math.floor(c["t_start"] / 120.0) + 1) * 120.0 for c in model.crossings]
    assert sum(o > 0 for o in over) <= 0.05 * len(over) and max(over) < 4.0
    # the waiting crowd builds during the vehicle phase and empties on walk
    def queue_at(when):
        return min(log, key=lambda row: abs(row[0] - when))[1]
    assert queue_at(72.0) > queue_at(100.0)          # emptied by the wave
    assert queue_at(192.0) > queue_at(150.0)         # built up again during the vehicle phase
    # crossing times: the 12 m side legs at about walking speed, diagonals longer
    side = [c["s"] for c in model.crossings if c["leg"] == "side" and not c["ran"]]
    diag = [c["s"] for c in model.crossings if c["leg"] == "diagonal" and not c["ran"]]
    assert side and diag and sorted(side)[len(side) // 2] < sorted(diag)[len(diag) // 2]
    assert 5.0 < sorted(side)[len(side) // 2] < 20.0
    assert any(rec["in_view_max"] > 0 for rec in model.phases)


def test_model_stragglers_run_at_the_end():
    model, _ = run_model(seconds=250.0, n=150)
    late = [c for c in model.crossings if c["ran"]]
    assert late, "someone should hurry when the light runs out"


def test_model_wave_ripples_back():
    site = S.make_site("scramble", four_way(), (0.0, 0.0), carla=False)
    sc = S.settings({})
    units = [[{"kind": "commuter", "speed": 1.3, "run": 2.6, "react": 0.7, "leader": True}] for _ in range(60)]
    model = S.CrossingModel([site], [units], {**sc, **TIMING}, {}, seed=1)
    for p in model.peds:                      # everyone waiting
        if p.state != "queue":
            model._choose_leg(p, 0)
            if model._take_slot(p):
                p.pos, p.state = model._slot_xy(p), "queue"
    pos = {p.i: p.pos for p in model.peds}
    model.step(80.0, pos, lambda s, lg: (True, 40.0), dt=0.08)        # walk begins (red for 7 s already)
    model.red_since = {(0,): 73.0}
    model.phase.clear()
    model.step(80.08, pos, lambda s, lg: (True, 40.0), dt=0.08)
    rows = {}
    for p in model.peds:
        if p.state == "queue" and p.start_at is not None and p.slot is not None:
            rows.setdefault(site.slots[p.slot][3], []).append(p.start_at)
    assert len(rows) >= 2
    means = [sum(v) / len(v) for k, v in sorted(rows.items())]
    assert means == sorted(means)            # the back rows start later


def test_model_red_light_crossers_only_with_crowd_share():
    model, _ = run_model(seconds=250.0, jay=0.0)
    assert not any(c["red"] for c in model.crossings)
    model, _ = run_model(seconds=250.0, jay=0.3, seed=4)
    reds = [c for c in model.crossings if c["red"]]
    assert reds and all(c["leg"] != "diagonal" for c in reds)


def test_model_photo_stops():
    model, _ = run_model(seconds=250.0, photo=True)
    assert any(c["photo"] for c in model.crossings)
    assert any(name == "crossing_photo" for name, _ in model.events)


def test_model_busy_site_follows_each_leg():
    site = S.make_site("busy", four_way(), (0.0, 0.0), carla=False)
    sc = S.settings({})
    units = S.draw_units(40, random.Random(1), sc, "busy", {"speed_median": 1.3, "speed_sd": 0.1},
                         S.movement("NLD"))
    model = S.CrossingModel([site], [units], {**sc, **TIMING}, {}, seed=1)
    pos = {p.i: p.pos for p in model.peds}

    def signal(t):
        def f(s, lg):
            c = t % 90.0
            # a leg across road E or W (its kerbs lie along x) conflicts with E-W traffic (green 0-40)
            along_x = abs(lg.a.p[0] - lg.b.p[0]) < abs(lg.a.p[1] - lg.b.p[1])
            if along_x:
                return (c >= 43.0), (90.0 - c if c >= 43.0 else 0.0)
            return (c < 40.0 or c >= 88.0), ((40.0 - c) if c < 40.0 else 0.0)
        return f
    t = 0.0
    while t < 200.0:
        cmds = model.step(t, dict(pos), signal(t), dt=0.12)
        for i, c in cmds.items():
            if c[0] == "walk":
                (dx, dy), v = c[1], c[2]
                pos[i] = (pos[i][0] + dx * v * 0.12, pos[i][1] + dy * v * 0.12)
            elif c[0] == "teleport":
                pos[i] = (c[1], c[2])
        t += 0.12
    assert model.crossings and model.phases
    assert all(rec["name"].startswith(" leg") or "leg" in rec["name"] for rec in model.phases)


def test_phase_row_fields():
    rec = {"site": 0, "kind": "scramble", "name": "scramble", "t_start": 75.0, "t_end": 117.0, "queued": 80,
           "queue_by_corner": {0: 20, 1: 60}, "crossers": 140, "diagonal": 40, "in_view_max": 55, "in_view_sum": 300,
           "in_view_n": 10, "late_runners": 2, "red": 1, "crossing_s": [8.0, 9.0, 10.0, 22.0]}
    row = S.phase_row(rec)
    assert row["crossers"] == 140 and row["in_view_mean"] == 30.0 and row["crossing_s_median"] == 10.0
    assert row["red_crossers"] == 1 and row["arrived"] == 4


def test_settings_from_config():
    from coupledcarla import config
    sc = S.settings({"traffic": {"walkers": config.DEFAULTS["traffic"]["walkers"]}})
    assert sc["walk_s"] == 35.0 and sc["enabled"] is True
    assert set(config.DEFAULTS["traffic"]["walkers"]["crossings"]) == set(S.DEFAULTS)
    assert S.settings({"traffic": {"walkers": {"crossings": {"enabled": False}}}})["enabled"] is False


def test_yield_grid_finds_near_walkers():
    pytest.importorskip("traci")
    from coupledcarla import yielding as Y
    wl = [(0.0, 0.0, 0, 0), (45.0, 0.0, 0, 0), (300.0, 0.0, 0, 0)]
    grid = {}
    for w in wl:
        grid.setdefault((int(w[0] // Y.GRID_M), int(w[1] // Y.GRID_M)), []).append(w)
    assert sorted(w[0] for w in Y.near_walkers(grid, 10.0, 0.0, 70.0)) == [0.0, 45.0]
    assert Y.near_walkers(grid, 1000.0, 0.0, 70.0) == []
