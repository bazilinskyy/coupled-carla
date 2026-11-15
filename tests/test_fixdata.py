"""Regression tests for bugs in the data and map-building code (no network, no SUMO binary, no CARLA)."""
import json
from datetime import datetime, timedelta, timezone

import pytest

from coupledcarla import realworld as rw


def test_sources_keep_nyc_buildings_and_name_trees_and_gbfs():
    # "nyc" was in SOURCES twice: the tree census replaced the building footprints, so a New York map's meta.json
    # credited its building heights to the tree census; GBFS (shared bikes) had no entry and was dropped
    assert "Building Footprints" in rw.SOURCES["nyc"][0]
    assert "Tree Census" in rw.SOURCES["nyc_trees"][0]
    assert "gbfs" in rw.SOURCES
    assert [m[0] for m in rw.MUNICIPAL] == ["eindhoven", "nyc_trees"]


def test_municipal_cache_with_the_old_nyc_name(tmp_path):
    f = tmp_path / "municipal.json"
    f.write_text(json.dumps({"elements": [], "sources": ["nyc"]}))
    assert rw.municipal_scenery([40.75, -73.99, 40.76, -73.97], f)["sources"] == ["nyc_trees"]


def test_weather_now_east_of_utc_after_local_midnight(monkeypatch):
    # Tokyo at 20:30 UTC is 05:30 the next day: the hour must be found, not the last hour of the UTC day
    class Clock(datetime):
        @classmethod
        def now(cls, tz=None):
            t = datetime(2026, 11, 14, 20, 30, tzinfo=timezone.utc)
            return t if tz else t.replace(tzinfo=None)

    asked = {}

    class Answer:
        def __init__(self, params):
            d0 = datetime.fromisoformat(params["start_date"])
            d1 = datetime.fromisoformat(params["end_date"])
            hours = int((d1 - d0).total_seconds() // 3600) + 24
            times = [(d0 + timedelta(hours=h)).strftime("%Y-%m-%dT%H:%M") for h in range(hours)]
            self.data = {"utc_offset_seconds": 9 * 3600, "timezone": "Asia/Tokyo",
                         "hourly": {"time": times, "precipitation": [0.0] * hours,
                                    "temperature_2m": list(range(hours))}}

        def raise_for_status(self):
            pass

        def json(self):
            return self.data

    def get(url, params=None, **kw):
        asked.update(params)
        return Answer(params)

    monkeypatch.setattr(rw, "datetime", Clock)
    monkeypatch.setattr(rw.requests, "get", get)
    rec, local, off = rw.weather_at(35.66, 139.70, "now")
    assert local == datetime(2026, 11, 15, 5, 30)
    assert rec["time"] == "2026-11-15T05:00"


def test_parking_median_side_follows_the_traffic_side():
    # left-hand maps skipped every kerb of a two-way street as a "median" (London 926 kerb stalls -> 1479 fixed)
    from coupledcarla import parking
    assert parking.median_side(False) == "left"
    assert parking.median_side(True) == "right"

