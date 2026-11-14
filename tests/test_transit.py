"""Public transport: OSM line parsing, GTFS frequencies and the trial's bus routes (no SUMO, no network)."""
import io
import xml.etree.ElementTree as ET
import zipfile
from datetime import date, datetime

import pytest

from coupledcarla import transit as T


def test_interval_tag():
    assert T._interval("10") == 10
    assert T._interval("00:15") == 15
    assert T._interval("PT20M") == 20
    assert T._interval("PT1H") == 60
    assert T._interval("") is None and T._interval("often") is None


def test_chain_turns_ways_and_splits_at_gaps():
    a = [(51.0, 5.0), (51.0, 5.001)]
    b = [(51.0, 5.002), (51.0, 5.001)]          # drawn the other way round: turned
    c = [(51.01, 5.01), (51.01, 5.011)]         # 1 km away: a new piece
    first_backwards = [(51.0, 5.001), (51.0, 5.0)]
    pieces = T._chain([first_backwards, b, [], c])
    assert pieces[0] == [(51.0, 5.0), (51.0, 5.001), (51.0, 5.002)]
    assert pieces[1] == c
    assert T._chain([a, c]) == [a, c]


def test_parse_relation_with_clipped_geometry():
    geom = [{"lat": 51.0, "lon": 5.0}, {"lat": 51.0, "lon": 5.002}, None, {"lat": 51.0, "lon": 5.006},
            {"lat": 51.0, "lon": 5.009}]
    data = {"elements": [{"type": "relation", "id": 7, "tags": {"ref": "401", "route": "bus", "interval": "15"},
                          "members": [{"type": "way", "role": "", "geometry": geom},
                                      {"type": "node", "role": "stop", "lat": 51.0, "lon": 5.001},
                                      {"type": "node", "role": "platform", "lat": 51.0001, "lon": 5.001}]}]}
    (line,) = T.parse_relations(data)
    assert line["ref"] == "401" and line["interval_min"] == 15
    assert len(line["pieces"]) == 1                     # 140 m and 210 m pieces: only the second one is kept
    assert line["stops"] == [(51.0, 5.001)]             # stop positions win over platforms


def test_headway_from_timetable_osm_and_default():
    gtfs = {"5": {"weekday": [0] * 7 + [4, 6] + [4] * 15, "saturday": [2] * 24, "sunday": [0] * 24}}
    assert T.headway_s({"ref": "5"}, gtfs, "weekday", 8, 10) == 600
    assert T.headway_s({"ref": "5"}, gtfs, "sunday", 8, 10) is None       # the timetable runs none then
    assert T.headway_s({"ref": "9", "interval_min": 12}, gtfs, "weekday", 8, 10) == 720
    assert T.headway_s({"ref": ""}, gtfs, "weekday", 8, 10) == 600


def test_trial_time():
    assert T.trial_time("2026-11-14 17:30") == datetime(2026, 11, 14, 17, 30)
    d = T.trial_time(None)
    assert d.hour == 8 and d.weekday() < 5
    assert T.day_type(datetime(2026, 11, 14)) == "saturday" and T.day_type(datetime(2026, 11, 16)) == "weekday"


def _feed():
    """A tiny GTFS: line 5 runs every 10 min 07:00-09:00 on weekdays (both ways), line 6 is outside the box."""
    files = {
        "stops.txt": "stop_id,stop_lat,stop_lon\nA,51.44,5.48\nB,51.445,5.485\nX,52.0,5.0\n",
        "routes.txt": "route_id,route_short_name,route_type\nr5,5,3\nr6,6,3\nt1,5,0\n",
        "calendar.txt": "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date\n"
                        "wk,1,1,1,1,1,0,0,20260101,20271231\n",
        "calendar_dates.txt": "service_id,date,exception_type\n",
    }
    trips, times = ["route_id,service_id,trip_id,direction_id"], ["trip_id,arrival_time,departure_time,stop_id,stop_sequence"]
    n = 0
    for direction in ("0", "1"):
        for minute in range(7 * 60, 9 * 60, 10):
            n += 1
            trips.append(f"r5,wk,t{n},{direction}")
            hh, mm = divmod(minute, 60)
            first, second = ("A", "B") if direction == "0" else ("B", "A")
            times.append(f"t{n},{hh:02d}:{mm:02d}:00,{hh:02d}:{mm:02d}:00,{first},1")
            times.append(f"t{n},{hh:02d}:{mm + 3:02d}:00,{hh:02d}:{mm + 3:02d}:00,{second},2")
    trips.append("r6,wk,u1,0")
    times.append("u1,08:00:00,08:00:00,X,1")
    files["trips.txt"] = "\n".join(trips) + "\n"
    files["stop_times.txt"] = "\n".join(times) + "\n"
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for k, v in files.items():
            z.writestr(k, "﻿" + v)       # with a BOM, as many agencies write them
    return buf.getvalue()


def test_gtfs_frequencies(tmp_path, monkeypatch):
    feed = tmp_path / "f.zip"
    feed.write_bytes(_feed())
    monkeypatch.setattr(T, "feeds_for", lambda bbox, cache: [{"id": "f", "provider": "test"}])
    monkeypatch.setattr(T, "_fetch_feed", lambda row, cache: feed)
    monkeypatch.setattr(T, "date", type("D", (), {"today": staticmethod(lambda: date(2026, 11, 14))}))
    freq, used = T.gtfs_frequencies([51.43, 5.47, 51.45, 5.49], tmp_path, {"5", "6"})
    assert set(freq) == {"5"}
    assert freq["5"]["weekday"][7] == 6 and freq["5"]["weekday"][8] == 6 and freq["5"]["weekday"][9] == 0
    assert sum(freq["5"]["saturday"]) == 0
    assert used[0]["days"]["weekday"] == "2026-11-17"


class _Edge:
    def __init__(self, n):
        self.n = n

    def getLength(self):
        return self.n


class _Net:
    def getEdge(self, e):
        return _Edge(500.0)


def test_trial_routes_spread_buses_and_keep_stops(tmp_path):
    import json
    data = {"lines": [{"ref": "5", "name": "", "interval_min": None, "edges": ["a", "b", "c", "d"],
                       "stops": ["stop0", "stop1"]}],
            "stops": [{"id": "stop0", "lane": "b_0", "start": 100.0, "end": 120.0, "lines": ["5"]},
                      {"id": "stop1", "lane": "d_0", "start": 300.0, "end": 320.0, "lines": ["5"]}],
            "gtfs": {"5": {"weekday": [12] * 24, "saturday": [0] * 24, "sunday": [0] * 24}}, "feeds": []}
    (tmp_path / "transit.json").write_text(json.dumps(data))
    res = T.routes(tmp_path, tmp_path, 600, "bus", '<vType id="bus" vClass="bus"/>', datetime(2026, 11, 16, 8),
                   {"dwell_s": 20}, 1, _Net())
    assert res and res[2]["lines_running"] == 1
    root = ET.parse(tmp_path / "pt.rou.xml").getroot()
    vehs = root.findall("vehicle")
    at_start = [v for v in vehs if float(v.get("depart")) == 0]
    # every 300 s at 6 m/s: buses 1800 m apart on a 2 km line -> one on the line at t = 0 besides new departures
    assert len(at_start) >= 1
    later = [v for v in vehs if float(v.get("depart")) > 0]
    assert all(len(v.findall("stop")) == 2 for v in later)
    assert len(later) == 2                  # 600 s + 60 s of departures every 300 s
    assert ET.parse(tmp_path / "stops.add.xml").getroot().find("busStop").get("lines") == "5"
    assert T.routes(tmp_path, tmp_path, 600, "bus", "", datetime(2026, 11, 15, 8), {}, 1, _Net()) is None


def test_line_name_matching():
    assert T._key("都０１（Ｔ０１）") == T._key("都01(T01)") == "都01(T01)"
    assert T._base("都01(T01)") == "都01"
    assert T._key(" m 7 ") == "M7"
