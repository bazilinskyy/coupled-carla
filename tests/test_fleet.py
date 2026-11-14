"""Fleet from vehicle registers: body types to CARLA blueprints, colours on SUMO cars (no network)."""
import re

from coupledcarla import fleet


def test_car_types_follow_body_shares():
    reg = {"body": {"hatchback": 60, "van": 40}, "colors": {"grey": 1}}
    w = fleet.car_types(reg)
    hatch = sum(v for k, v in w.items() if k in fleet.BODY_BLUEPRINTS["hatchback"])
    van = sum(v for k, v in w.items() if k in fleet.BODY_BLUEPRINTS["van"])
    assert abs(hatch - 60) < 0.1 and abs(van - 40) < 0.1


def test_painter_follows_register_colours_and_paints_taxis():
    p = fleet.Painter({"colors": {"black": 90, "white": 10}}, seed=3)
    cols = [p("vehicle.seat.leon") for _ in range(500)]
    dark = sum(1 for c in cols if int(c.split(",")[0]) < 60)
    assert 400 < dark < 490
    p.taxis = True
    assert p("vehicle.ford.crown") == "250,195,0"


def test_paint_routes(tmp_path):
    f = tmp_path / "car.rou.xml"
    f.write_text('<routes>\n  <vehicle id="car0" type="vehicle.seat.leon" depart="0.00">\n    <route edges="a"/>\n'
                 '  </vehicle>\n  <vehicle id="car1" type="x" depart="1" color="1,2,3"/>\n</routes>\n')
    fleet.paint_routes(f, lambda bp: "9,9,9")
    t = f.read_text()
    assert re.search(r'<vehicle id="car0"[^>]*color="9,9,9">', t) and 'color="1,2,3"' in t
