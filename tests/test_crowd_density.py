"""Crowd density and coverage (coupledcarla/crowdpeds.py, tools/crowd_seen.py): no CARLA, SUMO or network."""
import importlib.util
import math
from pathlib import Path

from coupledcarla import config, crowdpeds

ROOT = Path(__file__).resolve().parent.parent


def _tool():
    spec = importlib.util.spec_from_file_location("crowd_seen", ROOT / "tools" / "crowd_seen.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_base_density_is_14_walkers_per_kerb_km_in_both_defaults():
    # the run's config overrides crowdpeds.DEFAULTS: both must say the same (8 left the twins at half CROWD's rate)
    assert crowdpeds.DEFAULTS["walkers_per_km"] == 14.0
    assert config.DEFAULTS["traffic"]["walkers"]["walkers_per_km"] == 14


def test_route_centres_every_60_m_with_both_ends():
    pts = crowdpeds.route_centres([[(0, 0), (100, 0), (100, 50)]])
    assert pts[0] == (0.0, 0.0) and pts[-1] == (100.0, 50.0)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert all(s <= 60.0 + 1e-9 for s in steps) and len(pts) == 4      # 0, 60 m, 120 m (corner past), end
    assert crowdpeds.route_centres([[], None]) == []
    assert crowdpeds.route_reach(45) == 13.9 * 45 and crowdpeds.route_reach(3600) == 1500.0


class _Edge:
    def __init__(self, shape):
        self.shape = shape

    def getShape(self):
        return self.shape


class _Net:
    def __init__(self, edges):
        self.edges = edges

    def getEdge(self, e):
        return self.edges[e]


def test_edges_xy_follows_the_route_in_carla_coordinates_up_to_max_m():
    net = _Net({"a": _Edge([(100, 200), (200, 200)]), "b": _Edge([(200, 200), (200, 400)])})
    off = (100.0, 300.0)                          # CARLA = (SUMO x - off x, off y - SUMO y)
    xy = crowdpeds.edges_xy(net, ["a", "missing", "b"], off, 150.0, start_pos=20.0)
    assert xy[0] == (20.0, 100.0)                  # 20 m into edge a
    assert xy[-1] == (100.0, 30.0)                 # 80 m along a, then 70 m up b (SUMO y 270)
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:]))
    assert abs(total - 150.0) < 1e-6


def test_seen_per_minute_counts_unique_walkers_ahead_while_moving():
    tool = _tool()
    rows = []
    for k in range(121):                           # 12 s at 10 Hz, 10 m/s along +x
        t = k / 10.0
        rows.append({"timestamp": str(t), "kind": "participant", "role": "driver", "actor_id": "1",
                     "pos_x": str(10 * t), "pos_y": "0", "rot_yaw": "0", "speed_kmh": "36"})
        rows.append({"timestamp": str(t), "kind": "walker", "actor_id": "w1", "pos_x": "60", "pos_y": "5",
                     "rot_yaw": "0", "speed_kmh": "5"})   # passed on the way: seen while ahead
        rows.append({"timestamp": str(t), "kind": "walker", "actor_id": "w2", "pos_x": "-50", "pos_y": "0",
                     "rot_yaw": "0", "speed_kmh": "5"})   # always behind: never seen
        rows.append({"timestamp": str(t), "kind": "bike", "actor_id": "b1", "pos_x": "300", "pos_y": "0",
                     "rot_yaw": "0", "speed_kmh": "15"})  # out of range
    m, w, b = tool.seen_per_minute(rows, "driver", 40.0)
    assert abs(m - 0.2) < 1e-6 and w == 1 and b == 0


def test_crowd_rate_locality_then_country_then_world():
    tool = _tool()
    d = {"world": {"persons_per_min": 18.0},
         "countries": {"ESP": {"name": "Spain", "persons_per_min": 21.6}},
         "localities": [{"name": "Madrid", "iso3": "ESP", "lat": 40.42, "lon": -3.70, "hours": 37, "persons_per_min": 24.7}]}
    assert tool.crowd_rate(40.43, -3.69, d)[0] == 24.7                  # within 30 km
    assert tool.crowd_rate(40.96, -5.66, d) == (21.6, "country Spain")   # Salamanca: 180 km from Madrid
    assert tool.crowd_rate(-33.9, 151.2, d) == (18.0, "world")
