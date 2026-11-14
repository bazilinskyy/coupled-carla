"""Driving style from measured speeds: the fetchers' parsing, live snapshots, free-flow speeds, the fitted speed
factor, urban default limits and the fallbacks (no network, no SUMO)."""
import gzip
import io
import json
from datetime import datetime

import pytest

from coupledcarla import speeds as S


BOX = [45.512, -122.688, 45.525, -122.670]


def _pbot_feature(lat, lon, loc, bound, start, v50, v85, posted):
    return {"geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": {"LocationID": loc, "Bound": bound, "StartDate": start, "Spd50th": v50, "Spd85th": v85,
                           "PostedSpeed": posted, "LocationDesc": "SW 5TH AVE S of SW OAK ST"}}


def test_parse_pbot_keeps_the_latest_count_in_the_box():
    data = {"features": [
        _pbot_feature(45.52, -122.68, "L1", "N", "2015-03-01T00:00:00Z", 20, 25, 25),
        _pbot_feature(45.52, -122.68, "L1", "N", "2022-03-01T00:00:00Z", 18, 22, 20),   # newer: replaces it
        _pbot_feature(45.52, -122.68, "L1", "S", "2022-03-01T00:00:00Z", 19, 24, 20),
        _pbot_feature(45.60, -122.60, "L2", "N", "2022-03-01T00:00:00Z", 30, 35, 30),   # outside the box
        _pbot_feature(45.52, -122.68, "L3", "E", "2005-03-01T00:00:00Z", 30, 35, 30),   # too old
        _pbot_feature(45.52, -122.68, "L4", "E", "2022-03-01T00:00:00Z", 30, 35, 2),    # no real posted speed
    ]}
    sites = S.parse_pbot(data, BOX, newest_year=2026)
    assert sorted(s["id"] for s in sites) == ["pbotL1N", "pbotL1S"]
    n = next(s for s in sites if s["id"] == "pbotL1N")
    assert n["v50"] == pytest.approx(18 * 1.609344, abs=0.1) and n["limit_kmh"] == pytest.approx(32.2, abs=0.1)
    assert n["heading"] == 0.0 and n["road"] == "SW 5TH AVE" and n["kind"] == "spot"


def test_parse_montevideo_drops_fill_values_and_takes_hourly_medians():
    rows = []
    for i, v in enumerate([30, 32, 34, 45, 45, 45, 90, 31, 33, 35, "", 0, 150]):
        rows.append({"cod_detector": "100", "fecha": "2026-11-10", "hora": f"08:{5 * i:02d}:00",
                     "dsc_avenida": "18 de Julio", "dsc_int_anterior": "Andes", "dsc_int_siguiente": "Convencion",
                     "latitud": "-34.906", "longitud": "-56.199", "velocidad": str(v)})
    rows.append(dict(rows[0], latitud="-34.80"))      # outside the box
    sites = S.parse_montevideo(rows, [-34.913, -56.207, -34.900, -56.191])
    assert len(sites) == 1
    s = sites[0]
    assert s["road"] == "18 de Julio" and s["from"] == "Andes" and s["to"] == "Convencion"
    assert s["hourly"]["weekday"][8] == pytest.approx(32.5)      # median of 30..35, without 45, 90, blanks, 0, 150
    assert s["hourly"]["weekday"][9] is None


def test_parse_taipei_sections():
    xml = ("<?xml version='1.0' encoding='UTF-8'?><vd:ExchangeData xmlns:vd=\"http://www.iii.org.tw/dax/vd\">"
           "<vd:ExchangeTime>2026/11/15T03:15:02</vd:ExchangeTime><vd:SectionDataSet>"
           "<vd:SectionData><vd:SectionId>A1</vd:SectionId><vd:SectionName>承德路    南京西路-民生西路</vd:SectionName>"
           "<vd:AvgSpd>49.5</vd:AvgSpd><vd:TotalVol>12.0</vd:TotalVol><vd:StartWgsX>121.5170</vd:StartWgsX>"
           "<vd:StartWgsY>25.0450</vd:StartWgsY><vd:EndWgsX>121.5170</vd:EndWgsX><vd:EndWgsY>25.0480</vd:EndWgsY>"
           "</vd:SectionData>"
           "<vd:SectionData><vd:SectionId>A2</vd:SectionId><vd:SectionName>濟南路</vd:SectionName><vd:AvgSpd>0.0</vd:AvgSpd>"
           "<vd:TotalVol>0.0</vd:TotalVol><vd:StartWgsX>121.5200</vd:StartWgsX><vd:StartWgsY>25.0450</vd:StartWgsY>"
           "<vd:EndWgsX>121.5190</vd:EndWgsX><vd:EndWgsY>25.0450</vd:EndWgsY></vd:SectionData>"
           "<vd:SectionData><vd:SectionId>A3</vd:SectionId><vd:SectionName>far</vd:SectionName><vd:AvgSpd>40</vd:AvgSpd>"
           "<vd:TotalVol>3</vd:TotalVol><vd:StartWgsX>121.60</vd:StartWgsX><vd:StartWgsY>25.10</vd:StartWgsY>"
           "<vd:EndWgsX>121.61</vd:EndWgsX><vd:EndWgsY>25.10</vd:EndWgsY></vd:SectionData>"
           "</vd:SectionDataSet></vd:ExchangeData>").encode()
    sites, when = S.parse_taipei(xml, [25.0415, 121.5100, 25.0541, 121.5239])
    assert when == datetime(2026, 11, 15, 3, 15, 2)
    assert [s["id"] for s in sites] == ["tpeA1", "tpeA2"]
    a1, a2 = sites
    assert a1["now"] == 49.5 and a1["road"] == "承德路" and a1["heading"] == pytest.approx(0.0, abs=0.5)
    assert a2["now"] is None and a2["heading"] == pytest.approx(270.0, abs=0.5)     # no vehicles: no speed


NDW_SITES = """<?xml version="1.0"?>
<d2:d2LogicalModel xmlns:d2="http://datex2.eu/schema/2/2_0"><d2:payloadPublication>
<d2:measurementSiteRecord id="GEH01_EIN_Kanaalstraat_W2" version="1">
 <d2:measurementEquipmentTypeUsed><d2:values><d2:value lang="nl">fcd</d2:value></d2:values></d2:measurementEquipmentTypeUsed>
 <d2:measurementSiteName><d2:values><d2:value lang="nl">Kanaalstraat_Kanaaldijk-Zuid_Vestdijk</d2:value></d2:values></d2:measurementSiteName>
 <d2:measurementSiteLocation>
  <d2:locationContainedInItinerary index="0"><d2:location>
   <d2:supplementaryPositionalDescription><d2:affectedCarriagewayAndLanes><d2:lengthAffected>124.2</d2:lengthAffected></d2:affectedCarriagewayAndLanes></d2:supplementaryPositionalDescription>
   <d2:linearExtension><d2:linearByCoordinatesExtension>
    <d2:linearCoordinatesStartPoint><d2:pointCoordinates><d2:latitude>51.437904</d2:latitude><d2:longitude>5.4839973</d2:longitude></d2:pointCoordinates></d2:linearCoordinatesStartPoint>
    <d2:linearCoordinatesEndPoint><d2:pointCoordinates><d2:latitude>51.438133</d2:latitude><d2:longitude>5.48236</d2:longitude></d2:pointCoordinates></d2:linearCoordinatesEndPoint>
   </d2:linearByCoordinatesExtension></d2:linearExtension></d2:location></d2:locationContainedInItinerary>
  <d2:locationContainedInItinerary index="1"><d2:location>
   <d2:supplementaryPositionalDescription><d2:affectedCarriagewayAndLanes><d2:lengthAffected>6.1</d2:lengthAffected></d2:affectedCarriagewayAndLanes></d2:supplementaryPositionalDescription>
   <d2:linearExtension><d2:linearByCoordinatesExtension>
    <d2:linearCoordinatesStartPoint><d2:pointCoordinates><d2:latitude>51.438133</d2:latitude><d2:longitude>5.48236</d2:longitude></d2:pointCoordinates></d2:linearCoordinatesStartPoint>
    <d2:linearCoordinatesEndPoint><d2:pointCoordinates><d2:latitude>51.438175</d2:latitude><d2:longitude>5.482305</d2:longitude></d2:pointCoordinates></d2:linearCoordinatesEndPoint>
   </d2:linearByCoordinatesExtension></d2:linearExtension></d2:location></d2:locationContainedInItinerary>
 </d2:measurementSiteLocation></d2:measurementSiteRecord>
<d2:measurementSiteRecord id="RWS01_MONIBASE_0021hrl0010ra" version="1">
 <d2:measurementEquipmentTypeUsed><d2:values><d2:value lang="nl">lus</d2:value></d2:values></d2:measurementEquipmentTypeUsed>
</d2:measurementSiteRecord>
</d2:payloadPublication></d2:d2LogicalModel>"""

NDW_TIMES = """<?xml version="1.0"?>
<d2:d2LogicalModel xmlns:d2="http://datex2.eu/schema/2/2_0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
<d2:siteMeasurements><d2:measurementSiteReference id="GEH01_EIN_Kanaalstraat_W2"/>
 <d2:measuredValue index="1"><d2:measuredValue><d2:basicData xsi:type="TravelTimeData"><d2:travelTime><d2:duration>26.0</d2:duration></d2:travelTime></d2:basicData></d2:measuredValue></d2:measuredValue>
</d2:siteMeasurements>
<d2:siteMeasurements><d2:measurementSiteReference id="other"/>
 <d2:measuredValue index="1"><d2:measuredValue><d2:basicData><d2:travelTime><d2:duration>10</d2:duration></d2:travelTime></d2:basicData></d2:measuredValue></d2:measuredValue>
</d2:siteMeasurements></d2:d2LogicalModel>"""


def test_parse_ndw_segments_and_travel_times():
    sites = S.parse_ndw_sites(io.BytesIO(NDW_SITES.encode()), [51.436, 5.47, 51.45, 5.493])
    assert len(sites) == 1
    s = sites[0]
    assert s["kind"] == "segment" and s["road"] == "Kanaalstraat" and s["to"] == "Vestdijk"
    assert s["length_m"] == pytest.approx(130.3) and len(s["points"]) == 3
    assert 270 < s["heading"] < 290          # westwards
    times = S.parse_ndw_times(io.BytesIO(NDW_TIMES.encode()), {s["id"]})
    assert times == {s["id"]: 26.0}


def test_merge_live_keeps_a_running_mean_per_hour():
    when = datetime(2026, 11, 14, 20, 15)                 # a Saturday
    a = S.merge_live([], [{"id": "x", "now": 30.0, "lat": 1, "lon": 2}, {"id": "y", "now": None}], when)
    b = S.merge_live(a, [{"id": "x", "now": 40.0, "lat": 1, "lon": 2}], when)
    x = next(s for s in b if s["id"] == "x")
    assert x["hourly"]["saturday"][20] == 35.0 and x["n"]["saturday"][20] == 2
    assert "now" not in x
    y = next(s for s in b if s["id"] == "y")              # kept although the second snapshot lacked it
    assert y["hourly"] == {}


def test_free_flow_and_measured():
    assert S.free_flow({"v50": 30.0, "v85": 40.36}) == (30.0, pytest.approx(10.0))
    prof = {"weekday": [50, 50, 48, 48, 45, 40, 30, 25] + [20] * 16}
    v, spread = S.free_flow({"hourly": prof})
    assert v == pytest.approx(45.0, abs=3.0) and spread is None        # the quiet (night) hours
    assert S.free_flow({}) == (None, None)
    site = {"hourly": {"saturday": [None] * 20 + [33.0] + [None] * 3}, "v50": None}
    assert S.measured(site, "saturday", 20) == 33.0
    assert S.measured(site, "weekday", 20) == 33.0                     # another day type when the day lacks it
    assert S.measured(site, "weekday", 8) is None
    assert S.measured({"v50": 29.0}, "sunday", 3) == 29.0


def test_fit_median_spread_and_bounds():
    matched = [({"v50": v, "v85": v + 10.36, "kind": "spot"}, f"e{i}", 40.0, 2.0)
               for i, v in enumerate([28.0, 30.0, 32.0, 36.0])]
    f = S.fit(matched, "pbot")
    assert f["mean"] == pytest.approx(0.775) and f["sd"] == pytest.approx(0.25) and f["sites"] == 4
    assert f["min"] == 0.4 and f["max"] == pytest.approx(1.4)
    assert S.distribution(f) == "normc(0.775,0.25,0.4,1.4)"
    assert S.fit(matched[:2], "pbot") is None                     # too few sites
    assert S.fit(matched, "ndw") is None                          # segment speeds set no factor
    detectors = [({"hourly": {"weekday": [45.0] * 24}}, f"e{i}", 50.0, 1.0) for i in range(3)]
    f = S.fit(detectors, "taipei")
    assert f["mean"] == pytest.approx(0.9) and f["sd"] == S.DEFAULT["sd"]


def test_urban_limits_by_country():
    assert S.urban_limit("US", "highway.secondary") == pytest.approx(40.2)
    assert S.urban_limit("BR", "primary") == 60 and S.urban_limit("BR", "residential") == 30
    assert S.urban_limit("BR", "tertiary_link") == 40
    assert S.urban_limit(None, "primary") == S.GENERIC_URBAN_KMH
    assert S.country_of({"city": {"country": "Taiwan"}}, [0, 0, 0, 0]) == "TW"
    assert S.country_of({}, [51.436, 5.47, 51.45, 5.493]) == "NL"
    assert S.country_of({}, [0, 0, 0.01, 0.01]) is None


class _Lane:
    def __init__(self, lid):
        self.lid = lid

    def getID(self):
        return self.lid


class _Edge:
    def __init__(self, eid, shape, speed, typ="", lanes=1):
        self.eid, self.shape, self.speed, self.typ = eid, shape, speed, typ
        self.lanes = [_Lane(f"{eid}_{i}") for i in range(lanes)]

    def getID(self):
        return self.eid

    def getShape(self):
        return self.shape

    def getSpeed(self):
        return self.speed

    def getType(self):
        return self.typ

    def allows(self, vclass):
        return True

    def getLanes(self):
        return self.lanes


class _Net:
    def __init__(self, edges, offset=(0.0, 0.0)):
        self.edges, self.offset = {e.getID(): e for e in edges}, offset

    def getEdges(self):
        return list(self.edges.values())

    def getEdge(self, eid):
        return self.edges[eid]

    def getLocationOffset(self):
        return self.offset


def test_implicit_limits_only_on_untagged_roads(tmp_path):
    geo = _Net([_Edge("100#0", [(0, 0), (100, 0)], 27.78, "highway.secondary"),        # untagged: 100 km/h
                _Edge("200#0", [(0, 50), (100, 50)], 13.89, "highway.secondary"),       # maxspeed=50
                _Edge("300#0", [(0, 100), (100, 100)], 22.22, "highway.trunk"),         # maxspeed=80 on a trunk
                _Edge("400#0", [(0, 150), (100, 150)], 13.89, "highway.residential")])  # untagged, 50 km/h
    net = _Net([_Edge("-5", [(0, 0.5), (100, 0.5)], 27.78, lanes=2), _Edge("-6", [(0, 50.5), (100, 50.5)], 13.89),
                _Edge("-7", [(0, 100.5), (100, 100.5)], 22.22), _Edge("-8", [(0, 150.5), (100, 150.5)], 13.89)])
    lim = S.implicit_limits(net, geo, "US")
    assert lim == {"-5": pytest.approx(40.2 / 3.6, abs=1e-3), "-8": pytest.approx(40.2 / 3.6, abs=1e-3)}
    assert S.implicit_limits(net, geo, "NL") == {"-5": pytest.approx(50 / 3.6, abs=1e-3)}
    S.write_vss(tmp_path / "vss.add.xml", net, lim)
    text = (tmp_path / "vss.add.xml").read_text()
    assert 'lanes="-5_0 -5_1 -8_0"' in text and 'step time="0" speed="11.167"' in text


def test_plan_falls_back_to_the_default(tmp_path, monkeypatch):
    (tmp_path / "meta.json").write_text(json.dumps({"bbox": [0, 0, 0.01, 0.01]}))
    cfg = {"traffic": {"speed_factor": "auto", "speed_limits": "auto"}}
    monkeypatch.setattr(S, "load", lambda *a, **k: None)
    assert S.plan(cfg, tmp_path, tmp_path / "w", [0, 0, 0.01, 0.01], log=lambda m: None) == \
        ("normc(1,0.1,0.8,1.2)", None)
    cfg["traffic"]["speed_factor"] = "normc(0.9,0.2,0.5,1.3)"
    assert S.plan(cfg, tmp_path, tmp_path / "w", [0, 0, 0.01, 0.01], log=lambda m: None)[0] == "normc(0.9,0.2,0.5,1.3)"

    def broken(*a, **k):
        raise RuntimeError("no net")
    monkeypatch.setattr(S, "load", lambda *a, **k: {"source": "pbot", "sites": [{"id": "a"}], "what": "x"})
    import coupledcarla.counts as C
    monkeypatch.setattr(C, "net_edges", broken)
    cfg["traffic"]["speed_factor"] = "auto"
    assert S.plan(cfg, tmp_path, tmp_path / "w", [0, 0, 0.01, 0.01], log=lambda m: None)[0] == "normc(1,0.1,0.8,1.2)"


def test_load_caches_and_fails_soft(tmp_path, monkeypatch):
    box = [45.512, -122.688, 45.525, -122.670]
    calls = []

    def fetch(bbox, real):
        calls.append(1)
        return [{"id": "pbotA", "lat": 45.52, "lon": -122.68, "v50": 30.0, "kind": "spot"}]
    monkeypatch.setattr(S, "FETCHERS", [("pbot", lambda b: True, fetch)])
    d = S.load(tmp_path, box)
    assert d["source"] == "pbot" and len(d["sites"]) == 1 and (tmp_path / "real" / "speeds.json").exists()
    S.load(tmp_path, box)
    assert len(calls) == 1                                       # cached

    def offline(bbox, real):
        raise OSError("offline")
    monkeypatch.setattr(S, "FETCHERS", [("pbot", lambda b: True, offline)])
    assert S.load(tmp_path, box, refresh=True)["sites"][0]["id"] == "pbotA"   # the cache, not a crash
    assert S.load(tmp_path / "other", box) is None


def test_live_source_adds_snapshots(tmp_path, monkeypatch):
    box = [25.0415, 121.5100, 25.0541, 121.5239]
    snaps = iter([([{"id": "t1", "now": 40.0, "lat": 25.05, "lon": 121.52, "kind": "spot"}], datetime(2026, 11, 15, 3)),
                  ([{"id": "t1", "now": 30.0, "lat": 25.05, "lon": 121.52, "kind": "spot"}], datetime(2026, 11, 16, 8))])
    monkeypatch.setattr(S, "FETCHERS", [("taipei", lambda b: True, lambda b, r: next(snaps))])
    S.load(tmp_path, box)
    d = S.load(tmp_path, box, refresh=True)
    h = d["sites"][0]["hourly"]
    assert h["sunday"][3] == 40.0 and h["weekday"][8] == 30.0


def test_calibration_lookup(tmp_path, monkeypatch):
    book = tmp_path / "cal.json"
    book.write_text(json.dumps({"portland_or": {"mean": 0.914, "sd": 0.297, "min": 0.4, "max": 1.66,
                                                "source": "pbot", "when": "2026-11-17T12:00:00",
                                                "mape": {"limits": 38.0, "calibrated": 32.7}}}))
    monkeypatch.setattr(S, "CALIBRATION", book)
    cal = S.calibration("portland_or", "pbot")
    assert S.distribution(cal) == "normc(0.914,0.297,0.4,1.66)"
    assert S.calibration("portland_or", "ndw") is None and S.calibration("lyon", "pbot") is None
    monkeypatch.setattr(S, "CALIBRATION", tmp_path / "missing.json")
    assert S.calibration("portland_or", "pbot") is None


def test_shipped_calibrations_are_sane():
    book = json.loads(S.CALIBRATION.read_text())
    for name, cal in book.items():
        assert cal["source"] in S.SOURCES and 0.5 <= cal["mean"] <= 1.5 and cal["min"] < cal["mean"] < cal["max"]
        assert cal["mape"]["calibrated"] <= cal["mape"]["before"]
