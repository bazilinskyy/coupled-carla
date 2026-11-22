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
    assert plates.country_for("RWA") is None and plates.country_for(None) is None


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
    monkeypatch.setattr(crowdpeds, "locate", lambda cfg, bbox, cache=None: ("RWA", "test"))
    assert plates.setup({"map": {"bbox": [-1.95, 30.05, -1.94, 30.06]}}, tmp_path, log=lambda m: None) is None
