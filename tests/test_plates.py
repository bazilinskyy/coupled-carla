"""Country number plates: the map's country picks the plates, a server without the attribute changes nothing."""
from coupledcarla import plates


class BP:
    def __init__(self, attrs):
        self.attrs = dict.fromkeys(attrs, "")

    def has_attribute(self, k):
        return k in self.attrs

    def set_attribute(self, k, v):
        self.attrs[k] = v


def test_country_for():
    assert plates.country_for("NLD") == "NL" and plates.country_for("jpn") == "JP"
    assert plates.country_for("BRA") is None and plates.country_for(None) is None


def test_setup_override_and_tag(tmp_path):
    assert plates.setup({"vehicles": {"license_plates": "es"}, "map": {}}, tmp_path, log=lambda m: None) == "ES"
    car, walker = BP(["license_plate", "color"]), BP(["speed"])
    plates.tag(car)
    plates.tag(walker)
    assert car.attrs["license_plate"] == "ES" and "license_plate" not in walker.attrs
    assert plates.setup({"vehicles": {"license_plates": False}, "map": {}}, tmp_path, log=lambda m: None) is None
    old = BP(["license_plate"])
    plates.tag(old)
    assert old.attrs["license_plate"] == ""      # off: CARLA's plates


def test_setup_auto_from_the_map(tmp_path, monkeypatch):
    from coupledcarla import crowdpeds
    monkeypatch.setattr(crowdpeds, "locate", lambda cfg, bbox, cache=None: ("NLD", "test"))
    assert plates.setup({"map": {"bbox": [51.44, 5.48, 51.45, 5.49]}}, tmp_path, log=lambda m: None) == "NL"
    monkeypatch.setattr(crowdpeds, "locate", lambda cfg, bbox, cache=None: ("BRA", "test"))
    assert plates.setup({"map": {"bbox": [-23.55, -46.64, -23.54, -46.63]}}, tmp_path, log=lambda m: None) is None
    assert plates.iso3() == "BRA"           # the country stays known for the signs


def test_towns_are_north_american_and_more_countries(tmp_path):
    assert plates.setup({"map": {"carla_town": "Town10HD_Opt"}}, tmp_path, log=lambda m: None) == "US"
    assert plates.iso3() == "USA"
    assert plates.country_for("UKR") == "UA" and plates.country_for("RWA") == "RW"


def test_spots_travel_with_the_country(tmp_path, monkeypatch):
    f = tmp_path / "plate_spots.json"
    f.write_text('{"spots": {"vehicle.lincoln.mkz": {"rear": [-231.4, 0.0, 52.0, -1.0, 0.0, 0.06]}}}')
    monkeypatch.setattr(plates, "SPOTS_FILE", f)
    plates.spots.cache_clear()
    try:
        assert plates.value_for("vehicle.lincoln.mkz", "ES") == "ES;rear=-231.4,0.0,52.0,-1.0,0.0,0.06"
        assert plates.value_for("vehicle.bmw.grandtourer", "ES") == "ES"      # has plate material: the code alone
        assert plates.setup({"vehicles": {"license_plates": "es"}, "map": {}}, tmp_path, log=lambda m: None) == "ES"
        car = BP(["license_plate"])
        car.id = "vehicle.lincoln.mkz"
        plates.tag(car)
        assert car.attrs["license_plate"].startswith("ES;rear=")
    finally:
        plates.spots.cache_clear()


def test_plate_spots_harvest(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import plate_spots
    log = tmp_path / "server.log"
    log.write_text("[x]LogCarla: Licence plates: body spots vehicle.mini.cooper measured;"
                   "rear=-190.0,0.0,60.0,-0.990,0.000,0.141;front=180.5,0.0,45.0,1.000,0.000,0.000\n"
                   "[x]LogCarla: Licence plates: body spots vehicle.kia.niro sent;rear=1,2,3,4,5,6\n")
    out = tmp_path / "spots.json"
    assert plate_spots.main([str(log), "--out", str(out)]) == 0
    import json
    d = json.loads(out.read_text())["spots"]
    assert list(d) == ["vehicle.mini.cooper"] and d["vehicle.mini.cooper"]["front"][0] == 180.5


def test_ua_and_rw_plates_draw(tmp_path):
    import random
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import plate_faces
    fp = plate_faces.font_path()
    for cc in ("UA", "RW"):
        for side in ("band", "front", "rear"):
            img, reg = plate_faces.render(cc, side, random.Random(f"1-{cc}-0"), fp, None)
            assert img.width in (512, 1024) and reg
    _, reg = plate_faces.render("UA", "rear", random.Random(3), fp, None)
    assert len(reg.split()) == 3 and reg.split()[0] in plate_faces.UA_REGIONS
    rear, _ = plate_faces.render("RW", "rear", random.Random(3), fp, None)
    front, _ = plate_faces.render("RW", "front", random.Random(3), fp, None)
    assert rear.convert("RGB").getpixel((40, rear.height // 2)) != front.convert("RGB").getpixel((40, front.height // 2))
