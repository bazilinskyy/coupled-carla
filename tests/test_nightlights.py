"""Night lights and rain (coupledcarla/nightlights.py): register lamps, vehicle lights, cyclists in rain."""
import json
import math
import types

import pytest

from coupledcarla import nightlights as nl
from coupledcarla import realworld as rw

from test_bridge_speed import mods  # noqa: F401 - the fixture: bridge modules with a stand-in carla


def reg(mast="ST 9 STANDAARD VZ", kind="led", model="", lumen="", watt="", colour=""):
    return {"highway": "street_lamp", "support:register": mast, "lamp_type": kind, "lamp_model": model,
            "light:lumen": lumen, "light:wattage": watt, "light:colour": colour}


# --- lamps -------------------------------------------------------------------------------------------------------

def test_mount_from_register_and_osm():
    assert nl.mount(reg("BB IN/OP/AAN WAND/GEVEL")) == "wall"
    assert nl.mount(reg("BB IN/ONDER PLAFOND/LUIFEL")) == "canopy"
    assert nl.mount(reg("Luifel Neckerspoel")) == "canopy"
    assert nl.mount(reg("BB AAN SPANDRAAD")) == "wire"
    assert nl.mount(reg("BB IN BESTRATING")) == "ground"
    assert nl.mount(reg("Graze Inground Enclosure")) == "ground"
    assert nl.mount(reg("AGMI KUNSTWERK STADHUISPLEIN")) == "special"
    for pole in ("ST 9 STANDAARD VZ", "AL 3,5 CONISCH NIEUW", "Mast met trap", "GY 3,5 (met deur)"):
        assert nl.mount(reg(pole)) == "pole"
    assert nl.mount({"highway": "street_lamp", "support": "wall_mounted"}) == "wall"
    assert nl.mount({"highway": "street_lamp", "support": "suspended"}) == "wire"
    assert nl.mount({"highway": "street_lamp"}) == "pole"


@pytest.mark.parametrize("model,kind,colour,k", [
    ("CorePro LED Urban PLL Mains 13W 830", "led", "wit", 3000),
    ("Lightronics A2 LED 51.2W 4000K 8000Lm", "led", "", 4000),
    ("LED119/740 DM50", "led", "", 4000),
    ("LED50-4S/730 MDS", "led", "", 3000),
    ("LED25W WW TB-IO", "led", "", 3000),
    ("Xeon 2 SC1 KNON 3K PFM", "led", "", 3000),
    ("LLN-SubtleLight RGBWw RF", "led", "", 3000),       # no code: LED default
    ("SON-T PIA PLUS 150W", "sont", "geel", 2000),
    ("SON PIA PLUS 70W I", "son", "geel", 2000),
    ("CDM-T  35W", "cdm-t", "wit", 3000),
    ("CDO-TT Plus 70W", "cdo-tt", "", 2800),
    ("PLL 36 W xtra polar", "pl", "", 4000),
    ("TL-D SUPER 80 18W/830", "tl", "", 3000),          # the model's code wins over the kind
    ("MHN-LA 2000W", "", "", 4000),                     # kind from the model's first word
    ("", "", "oranje", 1800),
    ("", "", "", None),
])
def test_cct(model, kind, colour, k):
    assert nl.cct(reg(model=model, kind=kind, colour=colour)) == k


def test_cct_osm_lamp_type():
    assert nl.cct({"lamp_type": "sodium"}) == 2000
    assert nl.cct({"lamp_type": "LED"}) == 3000
    assert nl.cct({"lamp_type": "mercury"}) == 4000
    assert nl.cct({"lamp_type": "electric"}) is None


def test_lumen_listed_estimated_capped():
    assert nl.lumen(reg(lumen="3500")) == 3500
    assert nl.lumen(reg(kind="son", lumen="", watt="70")) == pytest.approx(7000)
    assert nl.lumen(reg(kind="led", lumen="0", watt="20")) == pytest.approx(2200)
    assert nl.lumen(reg(kind="", lumen="", watt="")) is None
    assert nl.lumen(reg(kind="", model="MHN-LA 2000W", lumen="220000")) == nl.MAX_LUMEN


def test_lamp_spec():
    s = nl.lamp_spec(reg("BB IN/OP/AAN WAND/GEVEL", "led", "LED24/830", "2500"))
    assert s == {"mount": "wall", "cct": 3000, "lumen": 2500.0}


def test_kelvin_rgb_warm_to_cool():
    sodium, led3, led4, day = (nl.kelvin_rgb(k) for k in (2000, 3000, 4000, 6500))
    assert max(sodium) == 1.0 and sodium[0] == 1.0
    assert sodium[2] < led3[2] < led4[2] < day[2]          # bluer as it gets hotter
    assert day == pytest.approx((1.0, 1.0, 1.0), abs=0.02)
    assert sodium[1] == pytest.approx(0.54, abs=0.03) and sodium[2] < 0.2


def test_carla_lumen_never_read_as_kilolumens():
    assert nl.carla_lumen(655) == nl.CARLA_MIN_LUMEN
    assert nl.carla_lumen(3500, 2.0) == 7000


def test_spec_rows_and_index():
    items = [[0, 10.0, 20.0, 1.0, 0, 1.0, "lamp"], [0, 50.0, 50.0, 1.0, 0, 1.0, "lamp"],
             [0, 90.0, 90.0, 1.0, 0, 1.0, "lamp"]]
    specs = [{"cct": 2000, "lumen": 7000.0}, {"cct": None, "lumen": None}, {"cct": None, "lumen": 1550.4}]
    rows = nl.spec_rows(items, specs)
    assert rows == [[10.0, 20.0, 2000, 7000], [90.0, 90.0, 0, 1550]]     # the lamp with nothing known left out
    idx = nl.SpecIndex(rows)
    assert len(idx) == 2
    assert idx.at(10.3, 19.8)[2:] == (2000, 7000)
    assert idx.at(11.5, 20.0) is None
    assert idx.at(90.0, 90.0)[2:] == (None, 1550)
    L = types.SimpleNamespace
    lights = [L(location=L(x=10.2, y=20.1)), L(location=L(x=90.0, y=90.0)), L(location=L(x=300.0, y=0.0))]
    sets = nl.lamp_settings(lights, idx, scale=0.5)
    assert len(sets) == 2
    assert sets[0][1] == nl.kelvin_rgb(2000) and sets[0][2] == 3500.0
    # 1,550 lm x 0.5 = 775: sent as 1,000 (not read as kilolumens)
    assert sets[1][1] is None and sets[1][2] == nl.CARLA_MIN_LUMEN
    assert nl.lamp_settings(lights, idx, colours=False)[0][1] is None


# --- the register fetch and the scenery ----------------------------------------------------------------------------

def test_ehv_register_fields(monkeypatch):
    rows = [{"geo_point_2d": {"lat": 51.44, "lon": 5.48}, "lph_armatuur": 800.0, "soort_lamp": "son",
             "kleur_lamp": "geel", "type_lamp": "SON-T PIA PLUS 150W", "lumen_lamp": "17500", "wattage_lamp": "150",
             "type_drager_mast": "ST 9 STANDAARD VZ", "type_drager_uithouder": "ST ENKEL 1 M LOS VZ"}]
    asked = []

    def get(url, params=None, headers=None, timeout=None):
        asked.append((url, params))
        return types.SimpleNamespace(json=lambda: rows if "verlichting" in url else [])
    monkeypatch.setattr(rw.requests, "get", get)
    out = rw._ehv((51.43, 5.47, 51.45, 5.49))
    assert "type_drager_mast" in asked[1][1]["select"] and "lumen_lamp" in asked[1][1]["select"]
    t = out[0]["tags"]
    assert t["height"] == "8.0" and t["support:register"] == "ST 9 STANDAARD VZ"
    assert nl.lamp_spec(t) == {"mount": "pole", "cct": 2000, "lumen": 17500.0}


def test_municipal_cache_refetched_for_lamp_fields(tmp_path, monkeypatch):
    path = tmp_path / "municipal.json"
    path.write_text(json.dumps({"elements": [{"type": "node", "id": 1, "lat": 51.44, "lon": 5.48,
                                              "tags": {"highway": "street_lamp"}}], "sources": ["eindhoven"]}))
    fresh = [{"type": "node", "id": 2, "lat": 51.44, "lon": 5.48, "tags": {"highway": "street_lamp",
                                                                          "support:register": "BB IN BESTRATING"}}]
    calls = []
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57),
                                           lambda b: calls.append(b) or fresh, "test")])
    data = rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)
    assert calls and data["elements"] == fresh and data["version"] == rw.MUNICIPAL_VERSION
    assert json.loads(path.read_text())["version"] == rw.MUNICIPAL_VERSION
    rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)          # now cached
    assert len(calls) == 1


def test_municipal_old_cache_kept_when_register_down(tmp_path, monkeypatch):
    path = tmp_path / "municipal.json"
    old = {"elements": [{"type": "node", "id": 1, "lat": 51.44, "lon": 5.48, "tags": {"highway": "street_lamp"}}],
           "sources": ["eindhoven"]}
    path.write_text(json.dumps(old))

    def down(b):
        raise OSError("503")
    monkeypatch.setattr(rw, "MUNICIPAL", [("eindhoven", (51.38, 5.38, 51.50, 5.57), down, "test")])
    assert rw.municipal_scenery((51.43, 5.47, 51.45, 5.49), path)["elements"] == old["elements"]
    assert "version" not in json.loads(path.read_text())             # asked again next build


# --- vehicle lights ----------------------------------------------------------------------------------------------

def test_vehicle_lights_rules():
    day = nl.vehicle_lights(40.0)
    assert not day["low_beam"] and not day["position"] and not day["fog_rear"]
    night = nl.vehicle_lights(-10.0)
    assert night["low_beam"] and night["position"] and not night["fog_rear"] and night["why"] == "night"
    assert nl.vehicle_lights(2.0)["low_beam"] and not nl.vehicle_lights(4.0)["low_beam"]
    assert not nl.vehicle_lights(30.0, rain_mmh=1.0)["low_beam"]            # light rain: not impaired
    rain = nl.vehicle_lights(30.0, rain_mmh=4.0)
    assert rain["low_beam"] and rain["position"] and not rain["fog_rear"]    # never the rear fog light for rain
    fog = nl.vehicle_lights(30.0, visibility_m=400.0)
    assert fog["low_beam"] and not fog["fog_rear"]
    thick = nl.vehicle_lights(30.0, visibility_m=40.0)
    assert thick["fog_rear"]                                                 # RVV art. 34(2): under 50 m
    assert not nl.vehicle_lights(30.0, rain_mmh=10.0, visibility_m=40.0)["fog_rear"]
    assert nl.vehicle_lights(30.0, mode="on")["low_beam"]
    assert not nl.vehicle_lights(-30.0, mode="off")["low_beam"]


def test_visibility_from_fog_inverts_the_real_mapping():
    for vis in (40.0, 400.0, 2000.0, 8000.0):
        fog = rw.carla_weather_fields({"visibility": vis}, 10.0, 180.0)["fog_density"]
        assert nl.visibility_from_fog(fog) == pytest.approx(vis, rel=0.01)
    assert nl.visibility_from_fog(0) is None


def test_weather_lights_from_preset_and_real():
    W = types.SimpleNamespace
    assert nl.weather_lights(W(sun_altitude_angle=45.0, precipitation=100.0, fog_density=0.0))["low_beam"]
    assert not nl.weather_lights(W(sun_altitude_angle=45.0, precipitation=30.0, fog_density=0.0))["low_beam"]
    # the real hour's own rain and visibility, not the CARLA fields derived from them
    real = nl.weather_lights(W(sun_altitude_angle=45.0, precipitation=100.0, fog_density=0.0),
                             {"rain": 0.5, "visibility": 30.0})
    assert real["low_beam"] and real["fog_rear"]


def test_light_bits_and_brake_light():
    import enum
    LS = enum.IntFlag("LS", {"NONE": 0, "Position": 1, "LowBeam": 2, "Brake": 8, "Fog": 64, "LeftBlinker": 32})
    sent = []
    fake = types.SimpleNamespace(VehicleLightState=LS, command=types.SimpleNamespace(
        SetVehicleLightState=lambda i, ls: sent.append((i, int(ls)))))
    assert nl.light_bits({"low_beam": True, "position": True, "fog_rear": True}, fake) == 2 | 1 | 64
    assert nl.light_bits({"low_beam": False, "position": False, "fog_rear": False}, fake) == 0
    car = types.SimpleNamespace(id=7, state=LS.LowBeam | LS.Position | LS.LeftBlinker)
    car.get_light_state = lambda: car.state
    bl = nl.BrakeLight(car, lambda cmd: None)      # the stand-in command records what it was given
    ctl = types.SimpleNamespace
    bl.update(ctl(brake=0.0), fake)
    assert sent == []                                  # not braking, nothing sent
    bl.update(ctl(brake=0.4), fake)
    assert sent == [(7, 1 | 2 | 8 | 32)]               # headlights and blinker kept, brake added
    bl.update(ctl(brake=0.6), fake)
    assert len(sent) == 1                              # only changes are sent
    bl.update(ctl(brake=0.0), fake)
    assert sent[-1] == (7, 1 | 2 | 32)


def test_bridge_rear_and_fog_lights_follow_the_rules(mods, monkeypatch):  # noqa: F811 - the fixture
    bh = mods.sync.BridgeHelper
    LS = mods.carla.VehicleLightState
    monkeypatch.setattr(bh, "night", True)
    monkeypatch.setattr(bh, "fog", False)
    ls = bh.get_carla_lights_state(LS(0), 0)
    assert int(ls) & int(LS.LowBeam) and int(ls) & int(LS.Position)    # RVV art. 32(3): rear lights too
    assert not int(ls) & int(LS.Fog)
    monkeypatch.setattr(bh, "fog", True)
    assert int(bh.get_carla_lights_state(ls, 0)) & int(LS.Fog)
    monkeypatch.setattr(bh, "night", False)
    monkeypatch.setattr(bh, "fog", False)
    day = bh.get_carla_lights_state(LS(int(LS.LowBeam) | int(LS.Position) | int(LS.Fog)), 0)
    assert int(day) & (int(LS.LowBeam) | int(LS.Position) | int(LS.Fog)) == 0


# --- cyclists in rain --------------------------------------------------------------------------------------------

def test_bike_rain_factor_wessel():
    assert nl.bike_rain_factor(0.0) == 1.0
    assert nl.bike_rain_factor(0.05) == 1.0                       # under Open-Meteo's 0.1 mm: dry
    assert nl.bike_rain_factor(0.3) == pytest.approx(math.exp(-0.158), abs=1e-4)
    assert nl.bike_rain_factor(1.5) == pytest.approx(0.768, abs=1e-3)
    assert nl.bike_rain_factor(3.0) == pytest.approx(0.739, abs=1e-3)
    assert nl.bike_rain_factor(25.0) == pytest.approx(math.exp(-0.285), abs=1e-4)   # the 5-10 class above 10
    assert nl.bike_rain_factor(0.0, 1.2) == pytest.approx(math.exp(-0.241), abs=1e-4)
    assert nl.bike_rain_factor(3.0, 4.0) == pytest.approx(math.exp(-0.303 - 0.241), abs=1e-4)


def test_trial_rain_and_factor(tmp_path, monkeypatch):
    def cfg(weather, time=None, cycling=None):
        return {"simulation": {"weather": weather, "time": time}, "map": {"bbox": [51.43, 5.47, 51.45, 5.49]},
                "traffic": {"cycling": cycling or {}}}
    assert nl.trial_rain(cfg("HardRainNoon"), tmp_path)[:2] == (5.0, 0.0)
    assert nl.trial_rain(cfg("MidRainyNoon"), tmp_path)[0] == 3.0
    assert nl.trial_rain(cfg("ClearNoon"), tmp_path)[0] == 0.0
    assert nl.trial_rain(cfg({"precipitation": 40}), tmp_path)[0] == 2.0
    seen = {}

    def weather_at(lat, lon, when, cache_dir=None):
        seen["when"] = when
        return {"rain": 1.2, "snowfall": 0.0, "precipitation_before_3h": 0.4}, None, 0
    monkeypatch.setattr(rw, "weather_at", weather_at)
    rain, before, src = nl.trial_rain(cfg("real", "2026-11-16 17:00"), tmp_path)
    assert (rain, before, src) == (1.2, 0.4, "open-meteo") and seen["when"].hour == 17
    assert nl.bike_weather_factor(cfg("real", "2026-11-16 17:00"), tmp_path, log=lambda m: None) == \
        pytest.approx(math.exp(-0.264 - 0.241), abs=1e-4)
    assert nl.bike_weather_factor(cfg("real", "2026-11-16 17:00", {"weather": False}), tmp_path) == 1.0

    def offline(*a, **k):
        raise OSError("offline")
    monkeypatch.setattr(rw, "weather_at", offline)
    assert nl.bike_weather_factor(cfg("real", "now"), tmp_path, log=lambda m: None) == 1.0


def test_weather_record_has_rain_before(tmp_path, monkeypatch):
    from datetime import datetime
    hours = [f"2026-11-16T{h:02d}:00" for h in range(24)]
    precip = [0.0] * 24
    precip[14], precip[15], precip[16], precip[17] = 0.2, 0.3, 0.5, 2.0
    data = {"hourly": {"time": hours, "precipitation": precip, "rain": precip}, "utc_offset_seconds": 3600}
    (tmp_path / "weather_51.440_5.480_2026-11-16.json").write_text(json.dumps(data))
    rec, _, _ = rw.weather_at(51.44, 5.48, datetime(2026, 11, 16, 17, 20), tmp_path)
    assert rec["precipitation_before_3h"] == pytest.approx(1.0)
    assert rec["precipitation_3h"] == pytest.approx(2.8)
