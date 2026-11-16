"""SUMO junction analysis (coupledcarla/junctions.py) and the SUMO net without internal junctions (mapbuild.sumo_net).
No SUMO binary: a tiny net written by hand."""
from coupledcarla import config, junctions, mapbuild

# Junction J: a_0 -> c_0 runs west to east, b_0 -> d_0 south to north; their internal lanes cross at (5, 0).
NET = """<?xml version="1.0" encoding="UTF-8"?>
<net version="1.20">
    <edge id=":J_0" function="internal">
        <lane id=":J_0_0" index="0" speed="13.9" length="10.00" width="3.20" shape="0.00,0.00 10.00,0.00"/>
    </edge>
    <edge id=":J_1" function="internal">
        <lane id=":J_1_0" index="0" allow="bicycle" speed="5.0" length="10.00" width="1.00" shape="5.00,-5.00 5.00,5.00"/>
    </edge>
    <edge id="a" from="W" to="J" priority="1">
        <lane id="a_0" index="0" speed="13.9" length="50.00" shape="-50.00,0.00 0.00,0.00"/>
    </edge>
    <edge id="b" from="S" to="J" priority="1">
        <lane id="b_0" index="0" allow="bicycle" speed="5.0" length="50.00" width="1.00" shape="5.00,-55.00 5.00,-5.00"/>
    </edge>
    <edge id="c" from="J" to="E" priority="1">
        <lane id="c_0" index="0" speed="13.9" length="50.00" shape="10.00,0.00 60.00,0.00"/>
    </edge>
    <edge id="d" from="J" to="N" priority="1">
        <lane id="d_0" index="0" allow="bicycle" speed="5.0" length="50.00" width="1.00" shape="5.00,5.00 5.00,55.00"/>
    </edge>
    <tlLogic id="tl" type="static" programID="0" offset="0">
        <phase duration="30" state="Gr"/>
        <phase duration="30" state="GG"/>
    </tlLogic>
    <junction id="J" type="priority" x="5.00" y="0.00" incLanes="a_0 b_0" intLanes=":J_0_0 :J_1_0" shape="">
        <request index="0" response="00" foes="10" cont="0"/>
        <request index="1" response="RESP1" foes="01" cont="0"/>
    </junction>
    <junction id="W" type="dead_end" x="-50.00" y="0.00" incLanes="" intLanes="" shape=""/>
    <connection from="a" to="c" fromLane="0" toLane="0" via=":J_0_0" tl="tl" linkIndex="0" dir="s" state="O"/>
    <connection from="b" to="d" fromLane="0" toLane="0" via=":J_1_0" tl="tl" linkIndex="1" dir="s" state="O"/>
    <connection from=":J_0" to="c" fromLane="0" toLane="0" dir="s" state="M"/>
    <connection from=":J_1" to="d" fromLane="0" toLane="0" dir="s" state="M"/>
</net>
"""


def net(tmp_path, resp1="00"):
    p = tmp_path / "t.net.xml"
    p.write_text(NET.replace("RESP1", resp1))
    return junctions.read_net(p)


def test_read_net_and_links(tmp_path):
    n = net(tmp_path)
    assert n.junctions["J"]["int"] == [":J_0_0", ":J_1_0"]
    assert n.junctions["J"]["requests"][0][1] == {1}          # foes "10": link 1
    ls = junctions.links(n, "J")
    assert [(lk["from"], lk["to"], lk["linkIndex"]) for lk in ls] == [("a_0", "c_0", 0), ("b_0", "d_0", 1)]
    assert n.junction_of_lane(":J_1_0") == "J" and n.junction_of_lane("a_0") == "J"
    assert n.tls["tl"] == ["Gr", "GG"]


def test_crossing():
    assert junctions.crossing([(0, 0), (10, 0)], [(5, -5), (5, 5)]) == (5.0, 5.0)
    assert junctions.crossing([(0, 0), (10, 0)], [(0, 1), (10, 1)]) is None            # parallel
    assert junctions.crossing([(0, 0), (10, 0)], [(0, 0), (0, 10)]) is None            # a shared start is no crossing


def test_conflicts_need_someone_to_yield(tmp_path):
    assert junctions.conflicts(net(tmp_path), "J") == [(0, 1, "cross")]
    assert junctions.conflicts(net(tmp_path, resp1="01"), "J") == []      # the bike yields to the car


def test_green_conflicts(tmp_path):
    # phase 1 shows both green with priority ('G'): neither yields
    assert junctions.green_conflicts(net(tmp_path, resp1="01"), "J") == [(0, 1, 1)]


def test_events_per_junction(tmp_path):
    n = net(tmp_path)
    col = tmp_path / "collisions.xml"
    col.write_text('<collisions><collision time="12.30" type="junction" lane=":J_1_0" pos="1.0" collider="bike1" '
                   'victim="car2"/></collisions>')
    log = tmp_path / "sumo.log"
    log.write_text("Warning: Vehicle 'car2' performs emergency braking on lane 'a_0' with decel=8.00, wished=3.40, "
                   "severity=1.00, time=11.98.\nWarning: Teleporting vehicle 'car9'; waited too long (yield), "
                   "lane='b_0', time=250.00.\n")
    ev = junctions.events(col, log, n)
    assert len(ev["J"]["collision"]) == 1 and ev["J"]["emergency"][0][0] == 11.98
    assert ev["J"]["teleport"][0][3] == "waited too long (yield)"
    row = junctions.rank(ev, n)[0]
    assert (row["junction"], row["collisions"], row["emergency"], row["teleports"]) == ("J", 1, 1, 1)
    assert row["conflicts"] == [(0, 1, "cross")] and row["wait_inside"] == 0


def test_sumo_net_without_internal_junctions(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(mapbuild.subprocess, "check_call", lambda cmd: calls.append(cmd))
    (tmp_path / "geo.net.xml").write_text('<net version="1.20">')
    mapbuild.sumo_net(tmp_path, smooth=False)
    mapbuild.sumo_net(tmp_path, internal_junctions=True, smooth=False)
    assert "--netconvert-args=--default.connection.cont-pos 0" in calls[0]
    assert not any(c.startswith("--netconvert-args") for c in calls[1])
    assert config.DEFAULTS["map"]["internal_junctions"] is False


def test_lefthand_sumo_net(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(mapbuild.subprocess, "check_call", lambda cmd: calls.append(cmd))
    (tmp_path / "geo.net.xml").write_text('<net version="1.20" lefthand="true">')
    mapbuild.sumo_net(tmp_path, smooth=False)          # geo.net.xml re-written without internal junctions
    assert calls[0][0] == "netconvert" and "--sumo-net-file" in calls[0] and "--default.connection.cont-pos" in calls[0]
    calls.clear()
    mapbuild.sumo_net(tmp_path, internal_junctions=True, smooth=False)     # a plain copy, as before
    assert calls == [] and (tmp_path / "map.net.xml").read_text() == '<net version="1.20" lefthand="true">'


def test_mapbuild_sumo_net_only(tmp_path, monkeypatch):
    import yaml
    p = tmp_path / "t.yaml"
    p.write_text(yaml.safe_dump({"name": "t", "map": {"name": "t", "bbox": [0, 0, 1, 1]}}))
    cfg = config.load(p)
    cfg["_root"] = str(tmp_path)
    out = config.map_dir(cfg)
    out.mkdir(parents=True)
    (out / "meta.json").write_text("{}")
    seen = []
    monkeypatch.setattr(mapbuild, "sumo_net", lambda o, guess_tls=True, internal_junctions=False, smooth=True, drop_uturns=False:
                        seen.append((o, guess_tls, internal_junctions)))
    mapbuild.build(cfg, sumo_only=True)
    assert seen == [(out, True, False)]


def test_sumo_net_smoothing_passes_the_turn_speed_and_smooths(tmp_path, monkeypatch):
    from coupledcarla import netsmooth
    calls, smoothed = [], []
    monkeypatch.setattr(mapbuild.subprocess, "check_call", lambda cmd: calls.append(cmd))
    monkeypatch.setattr(netsmooth, "smooth", lambda path, extra, log, **kw: smoothed.append((path, list(extra))))
    (tmp_path / "geo.net.xml").write_text('<net version="1.20">')
    mapbuild.sumo_net(tmp_path)
    assert any("--junctions.limit-turn-speed" in c for c in calls[0])
    assert smoothed and smoothed[0][0] == tmp_path / "map.net.xml" and "--default.connection.cont-pos" in smoothed[0][1]
