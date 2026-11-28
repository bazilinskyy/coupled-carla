"""SUMO's net must be built from the map's own OpenDRIVE (mapbuild.net_status / ensure_net), nets without geoid grids
(mapbuild.strip_geoid_file), transit.json lines on edges the net lacks left out (transit.routes). No SUMO needed."""
import json
import os
import time
from datetime import datetime

import pytest

from coupledcarla import mapbuild, transit

XODR = """<?xml version="1.0"?>
<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=51.4 +lon_0=5.4 +datum=WGS84 +units=m
 +geoidgrids=egm96_15.gtx +vunits=m +no_defs]]></geoReference></header>
<road id="1" length="{L}" junction="-1"><planView><geometry s="0" x="0" y="0" hdg="0" length="{L}"><line/></geometry>
</planView><lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes></road>
</OpenDRIVE>
"""
NET = '<net><location netOffset="0,0" projParameter="+proj=tmerc +units=m +geoidgrids=egm96_15.gtx +no_defs"/></net>'


def _map(tmp_path, length=100):
    (tmp_path / "map.xodr").write_text(XODR.format(L=length))
    (tmp_path / "map.net.xml").write_text(NET)
    return tmp_path


def test_strip_geoid_from_xodr_and_net(tmp_path):
    m = _map(tmp_path)
    assert mapbuild.strip_geoid_file(m / "map.xodr") and mapbuild.strip_geoid_file(m / "map.net.xml")
    assert "geoidgrids" not in (m / "map.xodr").read_text() and "vunits" not in (m / "map.xodr").read_text()
    assert 'projParameter="+proj=tmerc +units=m +no_defs"' in (m / "map.net.xml").read_text()
    assert not mapbuild.strip_geoid_file(m / "map.net.xml")          # nothing left: unchanged
    assert not mapbuild.strip_geoid_file(m / "missing.xml")


def test_net_status_by_stamp(tmp_path):
    m = _map(tmp_path)
    mapbuild.stamp_net(m)
    ok, why = mapbuild.net_status(m)
    assert ok, why
    (m / "map.xodr").write_text(XODR.format(L=121))                  # another OpenDRIVE (a road 21 m longer)
    ok, why = mapbuild.net_status(m)
    assert not ok and "another map.xodr" in why
    mapbuild.stamp_net(m)
    assert mapbuild.net_status(m)[0]
    (m / "map.net.xml").write_text(NET + " ")                        # a net replaced by hand
    assert not mapbuild.net_status(m)[0]


def test_net_status_by_file_times_without_a_stamp(tmp_path):
    m = _map(tmp_path)
    old = time.time() - 7 * 86400
    os.utime(m / "map.net.xml", (old, old))                          # a 17 Nov net, a 24 Nov OpenDRIVE
    ok, why = mapbuild.net_status(m)
    assert not ok and "older than map.xodr" in why
    os.utime(m / "map.net.xml", None)
    assert mapbuild.net_status(m)[0]
    (m / "map.net.xml").unlink()
    assert not mapbuild.net_status(m)[0]


def test_ensure_net_modes(tmp_path, monkeypatch):
    m = _map(tmp_path)
    old = time.time() - 86400
    os.utime(m / "map.net.xml", (old, old))
    msgs, calls = [], []
    monkeypatch.setattr(mapbuild, "rebuild_net", lambda cfg, out: calls.append(out) or mapbuild.stamp_net(out))
    assert mapbuild.ensure_net({"map": {"stale_net": "warn"}}, m, msgs.append) == "stale" and not calls
    assert any("WARNING" in s for s in msgs)
    assert "geoidgrids" not in (m / "map.net.xml").read_text()       # stripped on the way
    os.utime(m / "map.net.xml", (old, old))
    with pytest.raises(SystemExit, match="--sumo-net"):
        mapbuild.ensure_net({"map": {"stale_net": "stop"}, "_path": "experiments/x.yaml"}, m, msgs.append)
    os.utime(m / "map.net.xml", (old, old))
    assert mapbuild.ensure_net({"map": {}}, m, msgs.append) == "rebuilt" and calls == [m]
    assert mapbuild.ensure_net({"map": {}}, m, msgs.append) == "ok"   # stamped by the rebuild


class _Lane:
    def __init__(self, i):
        self.i = i

    def getID(self):
        return self.i


class _Edge:
    def __init__(self, e):
        self.e = e

    def getID(self):
        return self.e

    def getLength(self):
        return 200.0

    def getLanes(self):
        return [_Lane(f"{self.e}_0")]


class _Net:
    def __init__(self, ids):
        self.edges = {e: _Edge(e) for e in ids}

    def getEdges(self):
        return list(self.edges.values())

    def getEdge(self, e):
        return self.edges[e]


def test_transit_lines_on_unknown_edges_are_left_out(tmp_path, monkeypatch):
    data = {"lines": [{"ref": "1", "name": "1", "edges": ["a", "b"], "stops": ["s1"], "interval_min": 10},
                      {"ref": "2", "name": "2", "edges": ["a", "-85"], "stops": ["s2"], "interval_min": 10}],
            "stops": [{"id": "s1", "lane": "b_0", "start": 10, "end": 30, "lines": ["1"]},
                      {"id": "s2", "lane": "-85_0", "start": 10, "end": 30, "lines": ["2"]}],
            "gtfs": None}
    (tmp_path / "transit.json").write_text(json.dumps(data))
    seen = []
    monkeypatch.setattr(transit, "headway_s", lambda line, gtfs, day, hour, d: (seen.append(line["ref"]), 600)[1])
    out = transit.routes(tmp_path, tmp_path, 120, "bus", '<vType id="bus"/>', datetime(2026, 9, 22, 12), {}, 1,
                         _Net(["a", "b"]))
    assert seen == ["1"]                     # line 2 runs on edge -85 of an old net: left out, no KeyError
    assert out is not None and "-85" not in (tmp_path / out[1]).read_text()


def test_a_stamped_net_with_a_geoid_stays_fresh(tmp_path):
    m = _map(tmp_path)
    mapbuild.stamp_net(m)                     # stamped while it still named the geoid grid
    assert mapbuild.ensure_net({"map": {}}, m, lambda s: None) == "ok"
    assert "geoidgrids" not in (m / "map.net.xml").read_text() and mapbuild.net_status(m)[0]
