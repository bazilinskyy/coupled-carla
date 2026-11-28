"""Vehicles wait before a crosswalk at a junction, not on it (stoplines.py; no SUMO, no CARLA)."""
import json

import pytest

from coupledcarla import stoplines as sl

XODR = """<?xml version="1.0"?>
<OpenDRIVE><header revMajor="1" revMinor="4"/>
 <road id="1" length="100" junction="-1">
  <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
  <lanes><laneSection s="0"><right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
  </right></laneSection></lanes>
  <objects>
   <object id="c1" name="Crosswalk" type="crosswalk" s="97.5" t="-1.75" hdg="1.5707963" length="8" width="4"/>
   <object id="c2" type="crosswalk" s="50" t="0" hdg="0">
    <outline><cornerLocal u="-1" v="-2"/><cornerLocal u="1" v="-2"/><cornerLocal u="1" v="2"/><cornerLocal u="-1" v="2"/>
    </outline></object>
   <object id="tree" type="tree" s="10" t="5"/>
  </objects>
 </road>
 <road id="2" length="50" junction="-1">
  <planView><geometry s="0" x="0" y="100" hdg="1.5707963" length="50"><line/></geometry></planView>
  <lanes><laneSection s="0"/></lanes>
  <objects><object id="c3" type="crosswalk" s="10" t="0" hdg="0">
   <outline><cornerRoad s="9" t="-2"/><cornerRoad s="11" t="-2"/><cornerRoad s="11" t="2"/><cornerRoad s="9" t="2"/></outline>
  </object></objects>
 </road>
</OpenDRIVE>
"""


def _bounds(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def test_xodr_crosswalks():
    polys = sl.xodr_crosswalks(XODR)
    assert len(polys) == 3                                           # the tree is not one
    x0, y0, x1, y1 = _bounds(polys[0])     # turned 90 deg: 4 m along the road (x), 8 m across it, centred at t
    assert (x0, x1) == pytest.approx((95.5, 99.5), abs=1e-3) and (y0, y1) == pytest.approx((-5.75, 2.25), abs=1e-3)
    assert _bounds(polys[1]) == pytest.approx((49, -2, 51, 2), abs=1e-3)              # cornerLocal outline
    x0, y0, x1, y1 = _bounds(polys[2])                               # cornerRoad on a road heading north
    assert (x0, x1) == pytest.approx((-2, 2), abs=1e-3) and (y0, y1) == pytest.approx((109, 111), abs=1e-3)


def test_osm_crossings(tmp_path):
    osm = tmp_path / "map.osm"
    osm.write_text("""<osm>
 <node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="10"/><node id="3" lat="0" lon="20"/>
 <node id="4" lat="0" lon="5"><tag k="highway" v="crossing"/><tag k="crossing" v="zebra"/></node>
 <node id="5" lat="0" lon="15"><tag k="highway" v="crossing"/><tag k="crossing" v="no"/></node>
 <node id="6" lat="5" lon="5"><tag k="crossing" v="traffic_signals"/></node>
 <way id="10"><nd ref="1"/><nd ref="4"/><nd ref="2"/><nd ref="5"/><nd ref="3"/><tag k="highway" v="residential"/></way>
 <way id="11"><nd ref="6"/><nd ref="4"/><tag k="highway" v="footway"/></way>
</osm>""")
    polys = sl.osm_crossings(osm, lambda lat, lon: (lon, lat))       # x = lon, y = lat (metres)
    assert len(polys) == 1               # crossing=no is none; node 6 is on no road
    x0, y0, x1, y1 = _bounds(polys[0])
    assert (x0, x1) == pytest.approx((5 - sl.CROSSING_W_M / 2, 5 + sl.CROSSING_W_M / 2))
    assert (y0, y1) == pytest.approx((-sl.CROSSING_ACROSS_M, sl.CROSSING_ACROSS_M))


NET = """<?xml version="1.0"?>
<net version="1.16">
 <location netOffset="{ox},{oy}" convBoundary="0,0,200,100" origBoundary="0,0,200,100" projParameter="!"/>
 <edge id="E" from="e" to="f"><lane id="E_0" index="0" speed="13.9" length="100.00" shape="{e0} {e1}"/></edge>
 <edge id="F" from="f" to="g"><lane id="F_0" index="0" speed="13.9" length="100.00" shape="{f0} {f1}"/></edge>
 <edge id="A" from="a" to="b"><lane id="A_0" index="0" speed="13.9" length="100.00" shape="{a0} {a1}"/></edge>
 <edge id="B" from="b" to="c"><lane id="B_0" index="0" speed="13.9" length="1.00" shape="{b0} {b1}"/></edge>
 <edge id="C" from="c" to="d"><lane id="C_0" index="0" speed="13.9" length="99.00" shape="{c0} {c1}"/></edge>
 <junction id="f" type="traffic_light"/><junction id="g" type="dead_end"/><junction id="b" type="priority"/>
 <junction id="c" type="traffic_light"/><junction id="d" type="dead_end"/><junction id="a" type="dead_end"/>
 <junction id="e" type="dead_end"/>
 <connection from="E" to="F" fromLane="0" toLane="0" tl="f" linkIndex="0"/>
 <connection from="A" to="B" fromLane="0" toLane="0"/>
 <connection from="B" to="C" fromLane="0" toLane="0" tl="c" linkIndex="0"/>
</net>
"""


def _net(path, ox=0.0, oy=0.0):
    def p(x, y):
        return f"{x + ox:.2f},{y + oy:.2f}"
    path.write_text(NET.format(ox=ox, oy=oy, e0=p(0, 50), e1=p(100, 50), f0=p(100, 50), f1=p(200, 50),
                               a0=p(0, 0), a1=p(100, 0), b0=p(100, 0), b1=p(101, 0), c0=p(101, 0), c1=p(200, 0)))
    return path


def _rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def test_stop_line_before_the_crosswalk(tmp_path):
    net = _net(tmp_path / "map.net.xml")
    lanes, nodes, junctions = sl.net_lanes(net)
    assert lanes["E_0"]["tls"] and not lanes["A_0"]["tls"] and lanes["C_0"]["preds"] == ["B_0"]
    # E: a crosswalk on its last 4 m (96-99.5) and one at its start; B (1 m) behind a plain join: across A's end too
    polys = [_rect(96, 99.5, 45, 55), _rect(2, 6, 45, 55), _rect(97, 100.6, -5, 5)]
    spans = sl.lane_spans(lanes, polys)
    assert spans["E_0"] == [(2.0, 6.0), (96.0, 99.5)] and spans["B_0"] == [(0.0, 0.6)]
    aps = sl.approaches(lanes, nodes, spans)
    assert set(aps) == {"E_0", "B_0"}            # A only runs on into B (a plain join), C and F end at dead ends
    assert aps["B_0"]["spans"] == [(-3.0, 0.0), (0.0, 0.6)]           # A's end, in B's positions
    offs = sl.offsets(aps)
    assert offs["E_0"] == pytest.approx(100 - 96 + sl.MARGIN_M)       # the front 1 m before the crosswalk
    assert offs["B_0"] == pytest.approx(1 + 3 + sl.MARGIN_M)          # longer than B: the car waits on A
    assert not sl.violations(aps, offs) and len(sl.violations(aps)) == 2
    assert sl.violations(aps)[0]["front_in"] is False                 # B's end is just past the crosswalk


def test_stop_line_rules():
    ap = {"length": 30.0, "spans": [(18.0, 22.0)], "start": None}
    assert sl.stop_offset(ap) == 0.0                                  # 8 m clear in front: the car fits there
    ap = {"length": 30.0, "spans": [(21.0, 24.5), (25.0, 29.5)], "start": None}
    assert sl.stop_offset(ap) == pytest.approx(30 - 21 + sl.MARGIN_M)       # before both (the car would cover one)
    # a short link between two signals with crosswalks at both ends: no room before the far one; the car's front
    # stays out of the crosswalk of the junction it waits at, its rear over the other
    ap = {"length": 11.8, "spans": [(0.0, 2.9), (5.1, 9.1)], "start": 0.0}
    assert sl.stop_offset(ap) == pytest.approx(11.8 - 4.1)
    ap = {"length": 3.0, "spans": [(0.0, 2.8)], "start": 0.0}           # nowhere to stop but on it
    assert sl.stop_offset(ap) == 0.0
    ap = {"length": 60.0, "spans": [(10.0, 59.0)], "start": None}       # more than MAX_OFFSET_M back
    assert sl.stop_offset(ap) == 0.0


def test_one_junction_drawn_as_two_signal_nodes():
    # P (tls, 50 m) -> X (0.2 m, tls): the crosswalk over both; X is too short to wait in, so the stop line goes
    # before P's end (where vehicles wait for P's light too), not at X's end over the crosswalk
    lanes = {"P": {"length": 50.0, "preds": [], "tls": True, "to_type": "traffic_light", "from": "p"},
             "X": {"length": 0.2, "preds": ["P"], "tls": True, "to_type": "traffic_light", "from": "x"}}
    aps = sl.approaches(lanes, {}, {"P": [(47.9, 50.0)], "X": [(0.0, 0.2)]})
    assert aps["X"]["inner"] and not aps["P"]["inner"] and aps["X"]["start"] is None
    assert sl.stop_offset(aps["X"]) == pytest.approx(0.2 + 2.1 + sl.MARGIN_M)
    assert sl.stop_offset(aps["P"]) == pytest.approx(2.1 + sl.MARGIN_M)       # the same place
    assert aps["X"]["no_in"] is False and aps["P"]["no_in"] is True


def test_write_and_read_net(tmp_path):
    net = _net(tmp_path / "map.net.xml")
    out = tmp_path / "stops.net.xml"
    assert sl.write_net(net, out, {"E_0": 5.0, "B_0": 4.6}) == 2
    assert sl.read_offsets(out) == {"E_0": 5.0, "B_0": 4.6}
    assert sl.read_offsets(net) == {}
    assert sl.write_net(out, tmp_path / "again.net.xml", {"E_0": 3.0}) == 0      # a larger one is kept
    text = out.read_text()
    assert '<stopOffset value="5.00" vClasses="all"' in text and text.count("<lane ") == 5


def test_prepare_and_check_a_map(tmp_path):
    mdir = tmp_path / "m"
    mdir.mkdir()
    # an OpenDRIVE map: SUMO's net shifted by its netOffset (geo.net.xml none: the OpenDRIVE's own x, y)
    _net(mdir / "map.net.xml", ox=1000.0, oy=-500.0)
    (mdir / "map.xodr").write_text(XODR.replace('s="97.5" t="-1.75"', 's="98" t="50"'))     # E: 96-100
    assert sl.net_shift(mdir) == (1000.0, -500.0)
    r = sl.check(mdir, log=lambda m: None)
    assert r["crosswalks"] == 3 and r["source"] == "opendrive"
    assert [v["lane"] for v in r["before"]] == ["E_0"] and r["after"] == [] and r["offsets"] == {"E_0": 5.0}
    assert json.loads((mdir / "crossings.json").read_text())["key"]           # cached
    work = tmp_path / "w"
    work.mkdir()
    out = sl.prepare(mdir, mdir / "map.net.xml", work, log=lambda m: None)
    assert out == work / "stops.net.xml" and sl.read_offsets(out) == {"E_0": 5.0}
    assert sl.prepare(mdir, out, work, log=lambda m: None) is None             # already there
    empty = tmp_path / "e"
    empty.mkdir()
    _net(empty / "map.net.xml")
    assert sl.prepare(empty, empty / "map.net.xml", work, log=lambda m: None) is None     # no crosswalks


def test_point_back():
    a = [(0.0, 0.0), (10.0, 0.0)]
    b = [(-5.0, 5.0), (0.0, 0.0)]          # the lane before, ending where a begins
    assert sl.point_back([a], 0.0) == pytest.approx((10.0, 0.0, 1.0, 0.0))
    assert sl.point_back([a], 4.0) == pytest.approx((6.0, 0.0, 1.0, 0.0))
    x, y, ux, uy = sl.point_back([a, b], 10.0 + 2 ** 0.5)
    assert (x, y) == pytest.approx((-1.0, 1.0)) and (ux, uy) == pytest.approx((2 ** -0.5, -(2 ** -0.5)))
    assert sl.point_back([a], 12.0)[:2] == pytest.approx((-2.0, 0.0))        # straight on past the last
