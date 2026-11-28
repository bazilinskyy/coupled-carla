"""The place's traffic from CROWD (coupledcarla/crowdtraffic.py, data/crowd_traffic.json, data/place_vehicles.json)
and the warm-up of SUMO's demand (coupledcarla/warmup.py). No SUMO, no CARLA."""
import math
from datetime import datetime
from pathlib import Path

import pytest

from coupledcarla import crowdtraffic as ct
from coupledcarla import drivermix, drivers, warmup
from coupledcarla.drones import look_ahead

NOON = datetime(2026, 9, 22, 12, 30)
NIGHT = datetime(2026, 9, 22, 23, 0)


def test_table_has_every_level_and_class():
    t = ct.table()
    assert set(t["classes"]) == set(ct.CLASSES)
    assert t["world"]["minutes"] > 100000 and t["world"]["day"] and t["world"]["night"]
    for k in ct.CLASSES:
        assert t["world"]["per_min"][k] >= 0
    assert {"NLD", "JPN", "USA", "RWA", "VNM"} <= set(t["countries"])
    assert t["median_locality"]["localities"] > 100
    names = {l["name"] for l in t["localities"]}
    assert {"Eindhoven", "Tokyo", "New York", "Amsterdam"} <= names
    assert Path(ct.DATA / "crowd_traffic.json").stat().st_size < 20e6


def test_places_differ_as_the_streets_do():
    ehv = ct.plan_for("NLD", 51.4393, 5.4786, NOON)
    tyo = ct.plan_for("JPN", 35.658, 139.7016, NOON)
    hcm = ct.plan_for("VNM", 10.776, 106.700, NOON)
    nyc = ct.plan_for("USA", 40.7549, -73.984, NOON)
    assert ehv["seen"]["bicycles"] > 3 * tyo["seen"]["bicycles"]           # Dutch cyclists
    assert hcm["seen"]["motorcycles"] > 0.4 > ehv["seen"]["motorcycles"]    # Saigon's scooters
    assert tyo["seen"]["trucks"] > ehv["seen"]["trucks"]                    # Tokyo's vans and kei trucks
    assert nyc["density"] > ehv["density"]
    for p in (ehv, tyo, hcm, nyc):
        assert math.isclose(sum(p["seen"].values()), 1.0, rel_tol=1e-9)
        assert math.isclose(sum(p["stream"].values()), 1.0, rel_tol=1e-9)
        lo, hi = ct.DEFAULTS["density_range"]
        assert lo <= p["density"] <= hi
    assert ehv["levels"][0][0].startswith("locality Eindhoven")


def test_small_localities_lean_on_their_country():
    t = ct.table()
    sal = next(l for l in t["localities"] if l["name"] == "Salamanca" and l["iso3"] == "ESP")
    assert sal["minutes"] < 120
    p = ct.plan_for("ESP", sal["lat"], sal["lon"], NOON)
    esp = t["countries"]["ESP"]["day"]["per_min"]
    raw = ct._part(sal, False, t["world"])[1]      # no hour of day footage: its rates x the world's day ratio
    # shrunk: between the locality's own rate and Spain's, nearer Spain's (an hour against a 2-hour prior)
    for k in ("cars", "persons"):
        lo, hi = sorted((raw[k], esp[k]))
        assert lo - 1e-6 <= p["rates"][k] <= hi + 1e-6
    # a locality with a lot of footage keeps (nearly) its own rates
    ehv = next(l for l in t["localities"] if l["name"] == "Eindhoven")
    pe = ct.plan_for("NLD", ehv["lat"], ehv["lon"], None)
    assert abs(pe["rates"]["bicycles"] - ehv["per_min"]["bicycles"]) / ehv["per_min"]["bicycles"] < 0.05


def test_night_has_less_traffic_and_unknown_places_get_the_world():
    day = ct.plan_for("NLD", 51.4393, 5.4786, NOON)
    night = ct.plan_for("NLD", 51.4393, 5.4786, NIGHT)
    assert night["night"] and not day["night"]
    assert ct.motor(night["rates"]) < ct.motor(day["rates"])
    w = ct.plan_for(None, 0.0, -30.0, NOON)            # the Atlantic: no country, no locality
    assert w["levels"][0][0] == "world"
    assert ct.is_night(NOON, True) and not ct.is_night(NIGHT, False)


def test_mix_types_weights_and_classes():
    p = ct.plan_for("JPN", 35.658, 139.7016, NOON)
    pv = ct.place_vehicles("JPN")
    assert pv["region"] == "japan"
    cars = {"vehicle.toyota.prius": 3, "vehicle.mini.cooper_s_2021": 2}
    m = ct.mix_types(p, pv, cars, major=0)
    assert math.isclose(sum(w for w, _ in m.values()), 100.0, rel_tol=1e-3)
    classes = {c for _, c in m.values()}
    assert {"passenger", "taxi", "motorcycle", "delivery", "truck", "emergency"} <= classes
    taxi = sum(w for w, c in m.values() if c == "taxi")
    assert taxi > 5          # Tokyo's taxis
    # a model that is also a private car gets its own taxi type; the bridge strips the suffix
    m_nl = ct.mix_types(ct.plan_for("NLD", 51.44, 5.48, NOON), ct.place_vehicles("NLD"),
                        {"vehicle.toyota.prius": 1}, major=0)
    assert "vehicle.toyota.prius#taxi" in m_nl and m_nl["vehicle.toyota.prius"][1] == "passenger"
    liv = ct.colors(ct.place_vehicles("NLD"))
    assert liv["vehicle.toyota.prius#taxi"] == "18,18,20"
    # a register that has its own taxis (New York's yellow cabs) keeps them
    m_ny = ct.mix_types(ct.plan_for("USA", 40.75, -73.98, NOON), ct.place_vehicles("USA"),
                        {"vehicle.lincoln.mkz_2020": 9, "vehicle.ford.crown": 1}, major=0)
    assert m_ny["vehicle.ford.crown"][1] == "taxi"
    # ... except where the street has many more than the register (Manhattan's core: a listed place)
    pv_m = ct.place_vehicles("USA", 40.7549, -73.984)
    assert "Manhattan" in pv_m["region"] and pv_m["taxi"]["replace_register"]
    m_m = ct.mix_types(ct.plan_for("USA", 40.75, -73.98, NOON), pv_m,
                       {"vehicle.lincoln.mkz_2020": 9, "vehicle.ford.crown": 1}, major=0)
    assert m_m["vehicle.ford.crown"][0] > 15
    assert ct.place_vehicles("USA", 45.52, -122.68)["region"] == "usa"     # Portland: the region's


def test_server_names_on_carla_10():
    # CARLA 1.0's models that drive stand in for 0.9.16 names
    assert ct.for_server("vehicle.mercedes.sprinter", 1) == "vehicle.sprinter.mercedes"
    assert ct.for_server("vehicle.carlamotors.european_hgv", 1) == "vehicle.carlacola.actors"
    assert ct.for_server("vehicle.toyota.prius", 1) == "vehicle.kia.niro"
    assert ct.for_server("vehicle.vespa.zx125", 1) == "vehicle.vespa.zx125"
    assert ct.for_server("vehicle.toyota.prius", 0) == "vehicle.toyota.prius"
    for region in ct.vehicles_table()["regions"]:
        pv = ct.place_vehicles(ct.vehicles_table()["regions"][region]["countries"][0]
                               if ct.vehicles_table()["regions"][region]["countries"] else None)
        for key in ("motorcycles", "vans", "lorries", "service"):
            for bp in pv[key]:
                bp10 = ct.for_server(bp, 1)
                assert drivers.sizes("1.0").get(bp10, {}).get("drives", False), (region, key, bp, bp10)


def test_every_country_in_one_region_only():
    seen = {}
    for name, reg in ct.vehicles_table()["regions"].items():
        for iso3 in reg.get("countries", []):
            assert iso3 not in seen, (iso3, name, seen.get(iso3))
            seen[iso3] = name
    assert ct.region_of("RWA") == "africa" and ct.region_of("XYZ") == "world"


def test_vtypes_by_class():
    m = drivers.vtype("vehicle.vespa.zx125", "motorcycle", "EIDM", "normc(1,0.1,0.8,1.2)", 2)
    assert 'vClass="motorcycle"' in m and 'latAlignment="arbitrary"' in m and "length=" in m
    lorry = drivers.vtype("vehicle.carlamotors.european_hgv", "truck", "EIDM", "1", 1)
    assert 'vClass="truck"' in lorry and 'accel="1.2"' in lorry
    fire = drivers.vtype("vehicle.carlamotors.firetruck", "emergency", "EIDM", "1", 1)
    assert 'accel="1.2"' in fire
    taxi = drivers.vtype("vehicle.toyota.prius#taxi", "taxi", "EIDM", "1", 1)
    assert 'id="vehicle.toyota.prius#taxi"' in taxi and "length=" in taxi
    assert drivermix.kind_of("motorcycle") == "car" and drivermix.kind_of("truck") == "bus"


ROUTES = """<routes>
    <vTypeDistribution id="cars">
        <vType id="vehicle.toyota.prius" vClass="passenger" probability="5"/>
        <vType id="vehicle.carlamotors.european_hgv" vClass="truck" probability="5"/>
    </vTypeDistribution>
    <vehicle id="car0" type="cars" depart="0.00"><route edges="a b c"/></vehicle>
    <vehicle id="car1" type="cars" depart="100.00"><route edges="a b c"/></vehicle>
    <vehicle id="car2" type="cars" depart="550.00"><route edges="a b c"/></vehicle>
    <vehicle id="car3" type="cars" depart="700.00"><route edges="a b c"/></vehicle>
    <vehicle id="pt1_0" type="bus" depart="5.00"><route edges="a b"/><stop busStop="s" duration="20"/></vehicle>
</routes>
"""


class Edge:
    def __init__(self, length, speed, trucks=True):
        self.length, self.speed, self.trucks = length, speed, trucks

    def getLength(self):
        return self.length

    def getSpeed(self):
        return self.speed

    def allows(self, vclass):
        return self.trucks or vclass != "truck"


class Net:
    def __init__(self, edges):
        self.edges = edges

    def getEdge(self, e):
        return self.edges[e]


def test_prefill_places_running_vehicles(tmp_path):
    import xml.etree.ElementTree as ET
    p = tmp_path / "car.rou.xml"
    p.write_text(ROUTES)
    # each edge 600 m at 10 m/s: 100 s per edge at 0.6 of the limit
    net = Net({e: Edge(600.0, 10.0) for e in "abc"})
    placed, dropped, shifted = warmup.prefill(p, net, 600, seed=1)
    root = ET.parse(p).getroot()
    v = {e.get("id"): e for e in root.iter("vehicle")}
    assert "car0" not in v                       # left at -600 s: 300 s of route, arrived long ago
    assert "car1" not in v                       # left at -500 s: arrived at -200 s
    assert v["car2"].get("departEdge") is None   # left at -50 s: still on its first edge
    assert float(v["car2"].get("depart")) <= 1.0 and v["car2"].get("departPos") == "random_free"
    assert float(v["car3"].get("depart")) == pytest.approx(100.0)
    assert float(v["pt1_0"].get("depart")) == pytest.approx(5.0)     # buses keep their timetable
    assert (placed, dropped, shifted) == (1, 2, 1)
    deps = [float(e.get("depart")) for e in root if e.tag == "vehicle"]
    assert deps == sorted(deps)
    assert root[0].tag == "vTypeDistribution"


def test_prefill_departedge_and_warmup_setting(tmp_path):
    import xml.etree.ElementTree as ET
    p = tmp_path / "r.rou.xml"
    p.write_text(ROUTES.replace('depart="550.00"', 'depart="420.00"'))
    warmup.prefill(p, Net({e: Edge(600.0, 10.0) for e in "abc"}), 600)
    v = {e.get("id"): e for e in ET.parse(p).getroot().iter("vehicle")}
    assert v["car2"].get("departEdge") == "1"    # 180 s on the way: on its second edge
    assert warmup.warmup_for({"traffic": {}}, 60) == 600
    assert warmup.warmup_for({"traffic": {"warmup_s": 0}}, 60) == 0
    assert warmup.warmup_for({"traffic": {"warmup_s": 120}}, 60) == 120


def test_assign_draws_models_and_keeps_lorries_off_closed_streets(tmp_path):
    import xml.etree.ElementTree as ET
    p = tmp_path / "car.rou.xml"
    rows = "".join(f'<vehicle id="car{i}" type="cars" depart="{i}"><route edges="a b"/></vehicle>' for i in range(400))
    p.write_text(f'<routes><vTypeDistribution id="cars"/>{rows}</routes>')
    w = {"vehicle.toyota.prius": (50.0, "passenger"), "vehicle.toyota.prius#taxi": (10.0, "taxi"),
         "vehicle.carlamotors.european_hgv": (40.0, "truck")}
    n = ct.assign(p, w, Net({"a": Edge(10, 10, trucks=False), "b": Edge(10, 10)}), seed=3,
                  livery={"vehicle.toyota.prius#taxi": "18,18,20"})
    assert "truck" not in n and n["taxi"] > 20 and sum(n.values()) == 400
    taxis = [e for e in ET.parse(p).getroot().iter("vehicle") if e.get("type").endswith("#taxi")]
    assert taxis and all(e.get("color") == "18,18,20" for e in taxis)
    n2 = ct.assign(p, w, None, seed=3)           # already drawn (as duarouter does): kept, counted
    assert n2 == n


def test_look_ahead_of_a_drone_camera():
    x, y = look_ahead((0.0, 0.0, 45.0), 0.0, -15.0, 45.0)
    assert x == pytest.approx(45.0 / math.tan(math.radians(15.0))) and y == pytest.approx(0.0)
    x, y = look_ahead((0.0, 0.0, 45.0), 90.0, -60.0, 45.0)       # steep: clipped to the near limit
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(30.0)
    x, y = look_ahead((0.0, 0.0, 45.0), 0.0, 0.0, 45.0)          # level: the far limit
    assert x == pytest.approx(220.0)
