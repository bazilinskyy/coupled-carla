"""Dutch traffic-light programmes per junction (coupledcarla/signalplan.py) and real phase data (signaldata.py).
No SUMO binary, no network: tests/data/signals_cross.net.xml is a netconvert crossroads (four arms of a bike lane, a
through-and-right lane and a left-turn lane, 50 km/h)."""
import json
import shutil
from datetime import datetime, timedelta
from pathlib import Path

import pytest

from coupledcarla import junctions, signaldata, signalplan

CROSS = Path(__file__).parent / "data" / "signals_cross.net.xml"


@pytest.fixture(scope="module")
def net():
    return junctions.read_net(CROSS)


@pytest.fixture(scope="module")
def prog(net):
    return signalplan.design(net, ["C"], "C")


def test_crow_amber_and_rounding():
    assert signalplan.amber("car", "s", 50 / 3.6) == 3.5
    assert signalplan.amber("car", "s", 70 / 3.6) == 4.5
    assert signalplan.amber("car", "s", 80 / 3.6) == 5.0
    assert signalplan.amber("car", "l", 80 / 3.6) == 3.0
    assert signalplan.amber("bike", "s", 5.0) == 3.0
    assert signalplan.round_half_up(1.01) == 1.5 and signalplan.round_half_up(2.0) == 2.0


def test_groups_are_dutch(prog):
    by = {(g["edge"], g["mode"], g["kind"]) for g in prog["groups"]}
    for arm in "NESW":
        assert (f"{arm}in", "car", "l") in by          # a protected left from its own lane
        assert (f"{arm}in", "car", "x") in by          # through and right
        assert (f"{arm}in", "bike", "x") in by         # the cyclists
    assert len(prog["groups"]) == 12


def test_stages_conflict_free_and_complete(net, prog, tmp_path):
    names = {g["name"] for g in prog["groups"]}
    assert names == {n for st in prog["stages"] for n in st}       # every group gets a green
    assert len(prog["stages"]) == 4
    # opposite through streams share a stage, the lefts get their own
    stage_of = {n: i for i, st in enumerate(prog["stages"]) for n in st}
    g = {(x["edge"], x["mode"], x["kind"]): x["name"] for x in prog["groups"]}
    assert stage_of[g[("Nin", "car", "x")]] == stage_of[g[("Sin", "car", "x")]]
    assert stage_of[g[("Nin", "car", "l")]] == stage_of[g[("Sin", "car", "l")]]
    assert stage_of[g[("Nin", "car", "l")]] != stage_of[g[("Nin", "car", "x")]]
    # the written net: no two crossing links green at once with neither yielding
    out = tmp_path / "t.net.xml"
    signalplan.write_net(CROSS, out, [prog])
    n2 = junctions.read_net(out)
    assert junctions.green_conflicts(n2, "C") == []
    # a right turn shares its green with the straight-on cyclists beside it, and yields to them
    states = [p["state"] for p in prog["phases"] if p["name"] not in ("amber", "all-red")]
    right = next(lk["k"] for lk in prog["links"] if lk["from"] == "Nin_1" and lk["dir"] == "r")
    assert any(s[right] == "g" for s in states)


def test_timings(prog):
    ph = prog["phases"]
    ambers = [p for p in ph if p["name"] == "amber"]
    assert ambers and all(p["duration"] in (0.5, 3.0) for p in ambers)     # 3.0 s, then 0.5 s more straight on
    reds = [p for p in ph if p["name"] == "all-red"]
    assert all(0.5 <= p["duration"] <= signalplan.ALL_RED_MAX and p["duration"] * 2 == int(p["duration"] * 2)
               for p in reds)
    assert prog["type"] == "actuated"
    greens = [p for p in ph if p["minDur"]]
    assert all(p["minDur"] <= p["duration"] <= p["maxDur"] for p in greens)
    assert prog["max_cycle"] <= signalplan.CYCLE_MAX_INNER
    assert max(signalplan.max_red(prog).values()) <= 80.0        # Nota: cyclists wait at most 80 s inside the Ring
    # every phase state has a character per link
    assert all(len(p["state"]) == len(prog["links"]) for p in ph)


def test_write_net_keeps_geometry(net, prog, tmp_path):
    out = tmp_path / "t.net.xml"
    prog2 = dict(prog, id="vri9999")
    signalplan.write_net(CROSS, out, [prog2])
    n2 = junctions.read_net(out)
    assert {k: v["shape"] for k, v in n2.lanes.items()} == {k: v["shape"] for k, v in net.lanes.items()}
    assert list(n2.tls) == ["vri9999"] and len(n2.tls["vri9999"]) == len(prog["phases"])
    tl = [c for c in n2.conns if c.get("tl")]
    assert len(tl) == len(prog["links"]) and {c["tl"] for c in tl} == {"vri9999"}
    assert sorted(int(c["linkIndex"]) for c in tl) == list(range(len(prog["links"])))
    assert n2.junctions["C"]["type"] == "traffic_light"
    text = out.read_text()
    assert 'type="actuated"' in text and 'minDur=' in text and 'key="max-gap"' in text


def test_controllers_match_listed_junctions(net):
    xy = lambda lat, lon: (lon, lat)      # noqa: E731 - test coordinates: (x, y) = (lon, lat)
    cx, cy = net.junctions["C"]["x"], net.junctions["C"]["y"]
    near = {"id": "ehv1", "name": "Testlaan - Proefstraat", "lat": cy + 3.0, "lon": cx + 4.0, "vri": "1234",
            "ivri": "EIN1234", "tlc_id": "abc"}
    far = {"id": "ehv2", "name": "Elders", "lat": 500.0, "lon": 500.0, "vri": None, "ivri": None, "tlc_id": None}
    ctl = signalplan.controllers(net, [near, far], xy)
    assert len(ctl) == 1 and ctl[0]["id"] == "vri1234" and ctl[0]["nodes"] == ["C"]
    assert ctl[0]["source"] == "matched" and ctl[0]["ivri"] == "EIN1234"
    ctl = signalplan.controllers(net, [], xy)
    assert ctl[0]["id"] == "tlsC" and ctl[0]["source"] == "osm"


def test_locations_bundled_and_elsewhere(tmp_path):
    centre = [51.4360, 5.4700, 51.4500, 5.4930]
    locs = signalplan.locations(centre, tmp_path, fetch=False)
    names = {loc["name"]: loc for loc in locs}
    assert names["Vestdijk - 18 Septemberplein"]["vri"] == "1020"
    assert all(loc["lat"] and loc["lon"] for loc in locs)
    ring = signalplan.locations([51.418, 5.437, 51.462, 5.508], tmp_path, fetch=False)
    assert len({loc["ivri"] for loc in ring if loc["ivri"]}) >= 10          # the Ring's iVRIs (UDAP)
    assert signalplan.locations([48.85, 2.33, 48.87, 2.36], tmp_path, fetch=False) == []


def test_locations_fetch_is_cached_and_fails_soft(tmp_path, monkeypatch):
    import requests
    calls = []

    class R:
        def json(self):
            return [{"objectid": 7, "locatie": "Vestdijk - 18 Septemberplein",
                     "geo_point_2d": {"lat": 51.4386, "lon": 5.4786}}]

    def get(url, params=None, headers=None, timeout=None):
        calls.append(url)
        return R()
    monkeypatch.setattr(requests, "get", get)
    box = [51.4360, 5.4700, 51.4500, 5.4930]
    locs = signalplan.locations(box, tmp_path)
    assert [loc["id"] for loc in locs] == ["ehv7"] and locs[0]["vri"] == "1020"
    signalplan.locations(box, tmp_path)
    assert len(calls) == 1 and (tmp_path / "signal_locations.json").exists()

    def down(*a, **k):
        raise OSError("offline")
    monkeypatch.setattr(requests, "get", down)
    other = tmp_path / "other"
    assert len(signalplan.locations(box, other)) > 5        # the bundled list


def test_mode_for(monkeypatch):
    from coupledcarla import speeds
    monkeypatch.setattr(speeds, "country_of", lambda cfg, bbox: "NL")
    assert signalplan.mode_for({"traffic": {"signals": "auto"}}, [0, 0, 1, 1], False) == "dutch"
    assert signalplan.mode_for({"traffic": {"signals": "auto"}}, [0, 0, 1, 1], True) == "real"
    assert signalplan.mode_for({"traffic": {"signals": "sumo"}}, [0, 0, 1, 1], True) == "sumo"
    monkeypatch.setattr(speeds, "country_of", lambda cfg, bbox: "JP")
    assert signalplan.mode_for({"traffic": {}}, [0, 0, 1, 1], False) == "sumo"


def _map(tmp_path):
    m = tmp_path / "maps" / "cross"
    m.mkdir(parents=True)
    shutil.copy(CROSS, m / "map.net.xml")
    (m / "meta.json").write_text(json.dumps({"bbox": [10.0, 10.0, 10.1, 10.1]}))
    return m


def test_prepare_writes_and_caches(tmp_path):
    m = _map(tmp_path)
    cfg = {"traffic": {"signals": "dutch"}, "_root": str(tmp_path)}
    msgs = []
    net, add, info = signalplan.prepare(cfg, m, tmp_path / "w", None, msgs.append)
    assert net and Path(net).exists() and add == [] and info["controllers"] == 1
    assert json.loads((tmp_path / "w" / "signals.json").read_text())["mode"] == "dutch"
    net2, _, _ = signalplan.prepare(cfg, m, tmp_path / "w2", None, msgs.append)
    assert net2 == net and "(cached)" in msgs[-1]
    assert signalplan.prepare({"traffic": {"signals": "sumo"}}, m, tmp_path / "w3", None, msgs.append)[0] is None


# --- real phase data -----------------------------------------------------------------------------------------------

def test_norms():
    assert signaldata.norm_state("protected-Movement-Allowed") == "G"
    assert signaldata.norm_state("permissive_movement_allowed") == "g"
    assert signaldata.norm_state("stop-And-Remain") == "r"
    assert signaldata.norm_state("Y") == "y" and signaldata.norm_state("dark") == "o"
    assert signaldata.norm_controller("EIN2001") == "vri2001" == signaldata.norm_controller("2001")
    assert signaldata.norm_group("fc02") == "2" == signaldata.norm_group("02")


def _groups_cross():
    """Signal groups of the test crossroads as a controller's MAP would give them (Dutch numbering)."""
    return {"vri1": {"02": {"from": 0, "turns": "sr", "mode": "car"}, "03": {"from": 0, "turns": "l", "mode": "car"},
                     "08": {"from": 180, "turns": "sr", "mode": "car"}, "09": {"from": 180, "turns": "l", "mode": "car"},
                     "05": {"from": 90, "turns": "lsr", "mode": "car"}, "11": {"from": 270, "turns": "lsr", "mode": "car"},
                     "22": {"from": 0, "turns": "lsr", "mode": "bike"}, "28": {"from": 180, "turns": "lsr",
                                                                              "mode": "bike"}}}


def _log_rows(start, cycles, cycle_s, jitter=0.0):
    """A recorded log: N-S through (02, 08, 22, 28) green 30 s, N-S left (03, 09) 10 s, E-W (05, 11) 20 s."""
    rows = []
    t = start - timedelta(seconds=cycle_s)
    import random
    rng = random.Random(3)
    for _ in range(cycles + 2):
        c = cycle_s + (rng.uniform(-jitter, jitter) if jitter else 0.0)
        ns = 30.0 * c / cycle_s
        plan = [(("02", "08", "22", "28"), 0.0, ns), (("03", "09"), ns + 5, ns + 15), (("05", "11"), ns + 20, c - 5)]
        for groups, a, b in plan:
            for g in groups:
                rows.append((t + timedelta(seconds=a), "vri1", signaldata.norm_group(g), "G"))
                rows.append((t + timedelta(seconds=b), "vri1", signaldata.norm_group(g), "y"))
                rows.append((t + timedelta(seconds=b + 3), "vri1", signaldata.norm_group(g), "r"))
        t += timedelta(seconds=c)
    return sorted(rows)


def test_read_rows_csv_and_ndjson(tmp_path):
    csv_text = "time,controller,signal_group,state\n2026-11-02T08:00:00,EIN2001,fc02,G\n2026-11-02T08:00:30,2001,02,Y\n"
    rows = signaldata.read_rows(csv_text.splitlines())
    assert [(r[1], r[2], r[3]) for r in rows] == [("vri2001", "2", "G"), ("vri2001", "2", "y")]
    nd = ['{"time": "2026-11-02T07:00:00Z", "controller": "vri2001", "signal_group": "5", '
          '"state": "stop-And-Remain"}']
    r = signaldata.read_rows(nd)[0]
    assert r[0] == datetime(2026, 11, 2, 8, 0) and r[3] == "r"        # UTC -> Amsterdam (CET)


def test_match_groups(prog):
    g = _groups_cross()["vri1"]
    lg = signaldata.match_groups(prog, g)
    for lk in prog["links"]:
        want = {("Nin", "car", "s"): "2", ("Nin", "car", "r"): "2", ("Nin", "car", "l"): "3",
                ("Sin", "car", "s"): "8", ("Sin", "car", "l"): "9", ("Ein", "car", "s"): "5",
                ("Win", "car", "l"): "11", ("Nin", "bike", "l"): "22", ("Sin", "bike", "r"): "28"}
        key = (lk["edge"], lk["mode"], lk["turn"])
        if key in want:
            assert lg[lk["k"]] == want[key], key
    full = signaldata.fill_unmatched(prog, lg)
    assert set(full) == {lk["k"] for lk in prog["links"]}


def test_program_from_fixed_time_log(prog):
    start = datetime(2026, 11, 2, 8, 0)
    rows = _log_rows(start - timedelta(seconds=20), 60, 90.0)
    tls = signaldata.timelines(rows, "vri1")
    lg = signaldata.fill_unmatched(prog, signaldata.match_groups(prog, _groups_cross()["vri1"]))
    p = signaldata.program_from_log(prog, tls, lg, start)
    assert p["type"] == "static" and abs(p["cycle"] - 90.0) < 1.5
    assert abs(sum(ph["duration"] for ph in p["phases"]) - p["cycle"]) < 1.5
    # the north approach's through lane: green about 30 s of the cycle
    k = next(lk["k"] for lk in prog["links"] if lk["from"] == "Nin_1" and lk["dir"] == "s")
    green = sum(ph["duration"] for ph in p["phases"] if ph["state"][k] in "Gg")
    assert 28 <= green <= 32
    # 20 s into the cycle (N-S through green since 20 s) at the start: SUMO's programme time t - offset at t = 0
    def state_at(t):
        pos, at = (t - p["offset"]) % p["cycle"], 0.0
        for ph in p["phases"]:
            if at <= pos < at + ph["duration"]:
                return ph["state"][k]
            at += ph["duration"]
    assert state_at(0.0) == "G" and state_at(9.0) == "G" and state_at(11.5) == "y" and state_at(15.0) == "r"


def test_program_from_actuated_log(prog):
    start = datetime(2026, 11, 2, 8, 0)
    rows = _log_rows(start, 50, 80.0, jitter=20.0)
    tls = signaldata.timelines(rows, "vri1")
    lg = signaldata.fill_unmatched(prog, signaldata.match_groups(prog, _groups_cross()["vri1"]))
    p = signaldata.program_from_log(prog, tls, lg, start)
    assert p["type"] == "actuated" and p["stats"]["green_cv_max"] > signaldata.GREEN_CV_ACTUATED
    assert any(ph["maxDur"] and ph["maxDur"] > ph["minDur"] for ph in p["phases"])


class FakeTraci:
    class trafficlight:
        calls = []

        @classmethod
        def setRedYellowGreenState(cls, tls, state):
            cls.calls.append(("state", tls, state))

        @classmethod
        def setProgram(cls, tls, pid):
            cls.calls.append(("program", tls, pid))


def _plan(prog):
    return [dict(prog, id="vri1", vri="1")]


def test_exact_driver(prog):
    FakeTraci.trafficlight.calls = []
    start = datetime(2026, 11, 2, 8, 0)
    rows = _log_rows(start, 3, 90.0)
    d = signaldata.ExactDriver(_plan(prog), rows, _groups_cross(), start, log=lambda m: None)
    d.step(0.0, FakeTraci)
    d.step(40.0, FakeTraci)
    calls = FakeTraci.trafficlight.calls
    assert calls and all(c[1] == "vri1" and len(c[2]) == len(prog["links"]) for c in calls)
    k = next(lk["k"] for lk in prog["links"] if lk["from"] == "Nin_1" and lk["dir"] == "s")
    assert calls[-1][2][k] == "r"            # 40 s in: N-S through has had its 30 s and its amber


def test_live_driver_and_fallback(prog):
    import time
    FakeTraci.trafficlight.calls = []
    lines = ['{"time": 1, "controller": "EIN1", "signal_group": "02", "state": "G"}',
             '{"time": 2, "controller": "1", "signal_group": "08", "state": "protected-Movement-Allowed"}']
    d = signaldata.LiveDriver(_plan(prog), "test", _groups_cross(), log=lambda m: None, opener=lambda s: iter(lines))
    d.thread.join(2)
    d.step(0.0, FakeTraci)
    states = [c for c in FakeTraci.trafficlight.calls if c[0] == "state"]
    k = next(lk["k"] for lk in prog["links"] if lk["from"] == "Sin_1" and lk["dir"] == "s")
    assert states and states[-1][2][k] == "G"
    assert ("program", "vri1", "dutch") in FakeTraci.trafficlight.calls     # the feed ended: programmes back
    time.sleep(0)


def test_real_programs_written(prog, tmp_path):
    start = datetime(2026, 11, 2, 8, 0)
    rows = _log_rows(start, 60, 90.0)
    log = tmp_path / "phases.csv"
    log.write_text("time,controller,signal_group,state\n" + "".join(
        f"{t.isoformat()},EIN1,fc{g},{s}\n" for t, _c, g, s in rows))
    groups = tmp_path / "groups.json"
    groups.write_text(json.dumps(_groups_cross()))
    src = signaldata.Source({"log": str(log), "groups": str(groups), "date": "2026-11-02", "hour": 8}, tmp_path)
    assert src.available()
    name, info = src.programs(_plan(prog), tmp_path, None, log=lambda m: None)
    assert name == "signals_real.add.xml" and info["replayed"] == 1
    text = (tmp_path / name).read_text()
    assert 'programID="real"' in text and 'id="vri1"' in text
    assert signaldata.Source({}, tmp_path).available() is False
