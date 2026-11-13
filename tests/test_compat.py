"""coupledcarla.compat: CARLA 0.9.16 blueprint names on a CARLA 1.0 blueprint library."""
import fnmatch

from coupledcarla import compat


class Attr:
    def __init__(self, v):
        self.v = v

    def as_str(self):
        return str(self.v)

    def as_int(self):
        return int(self.v)


class BP:
    def __init__(self, id_, **attrs):
        self.id, self.attrs = id_, attrs

    def has_attribute(self, k):
        return k in self.attrs

    def get_attribute(self, k):
        return Attr(self.attrs[k])

    def set_attribute(self, k, v):
        self.attrs[k] = v


class Lib(list):
    def filter(self, pattern):
        return [b for b in self if fnmatch.fnmatch(b.id, pattern)]


LIB10 = Lib([BP("vehicle.lincoln.mkz", base_type="car", number_of_wheels=4),
             BP("vehicle.mini.cooper", base_type="car", number_of_wheels=4),
             BP("vehicle.fuso.mitsubishi", base_type="bus", number_of_wheels=4),
             BP("vehicle.yamaha.yzf", base_type="motorcycle", number_of_wheels=2),
             BP("walker.pedestrian.0015"), BP("walker.pedestrian.0016"), BP("walker.pedestrian.0017")])


def test_exact_and_alias():
    assert compat.blueprint(LIB10, "vehicle.lincoln.mkz").id == "vehicle.lincoln.mkz"
    assert compat.blueprint(LIB10, "vehicle.lincoln.mkz_2020").id == "vehicle.lincoln.mkz"
    assert compat.blueprint(LIB10, "vehicle.audi.a2").id == "vehicle.mini.cooper"
    assert compat.blueprint(LIB10, "vehicle.mitsubishi.fusorosa").id == "vehicle.fuso.mitsubishi"


def test_class_fallback():
    assert compat.blueprint(LIB10, "vehicle.nonexistent.coach", "bus").id == "vehicle.fuso.mitsubishi"
    assert compat.blueprint(LIB10, "vehicle.nonexistent.moped", "moped").id == "vehicle.yamaha.yzf"
    car = compat.blueprint(LIB10, "vehicle.nonexistent.car")
    assert car.attrs["base_type"] == "car"
    assert compat.blueprint(LIB10, "vehicle.nonexistent.car").id == car.id     # the same stand-in every time


def test_walkers_are_stable():
    a = compat.blueprint(LIB10, "walker.pedestrian.0001")
    assert a.id.startswith("walker.pedestrian.00")
    assert compat.blueprint(LIB10, "walker.pedestrian.0001").id == a.id
    assert compat.blueprint(LIB10, "walker.pedestrian.*").id.startswith("walker.pedestrian.")


def test_nothing_fits():
    assert compat.blueprint(Lib([]), "vehicle.audi.a2") is None
    assert compat.blueprint(LIB10, "static.prop.bench") is None


def test_exposure():
    bp = BP("sensor.camera.rgb", exposure_mode="histogram", shutter_speed="200")
    compat.set_manual_exposure(bp, 400)
    assert bp.attrs == {"exposure_mode": "manual", "shutter_speed": "400"}
    compat.set_manual_exposure(BP("sensor.camera.rgb"), 400)      # attributes missing: no error


def test_lanes():
    class WP09:
        def get_left_lane(self):
            return "left09"

        def get_right_lane(self):
            return "right09"

    class WP10:
        def get_left_waypoint(self):
            return "left10"

        def get_right_waypoint(self):
            return "right10"

    assert (compat.left_lane(WP09()), compat.right_lane(WP09())) == ("left09", "right09")
    assert (compat.left_lane(WP10()), compat.right_lane(WP10())) == ("left10", "right10")
