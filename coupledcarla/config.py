"""Experiment configuration: a YAML file merged over the defaults below."""
import copy
from pathlib import Path

import yaml

DEFAULTS = {
    "name": "experiment",
    "map": {
        "name": None,                # folder under maps/; defaults to the experiment name
        "bbox": None,                # [south, west, north, east] in degrees
        "road_types": ["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
                       "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
                       "residential", "living_street"],
        "cycleways": True,           # keep highway=cycleway (SUMO cyclists ride them)
        "sidewalks": False,          # guessed sidewalks make SUMO (1.24, 1.28) crash when the bridge moves the
                                     # participant's car with moveToXY, so they stay off for now
        "trim_dead_ends": True,      # drop dead-end chains and keep the largest strongly connected road graph
        "lefthand": False,           # left-hand traffic (UK, Japan, Australia, ...)
        "buildings": True,           # also fetch OSM building footprints (trial viewer, figures)
        "lane_width": 3.2,
        "carla": {"vertex_distance": 2.0, "max_road_length": 500.0, "wall_height": 0.0,
                  "additional_width": 0.6, "smooth_junctions": True, "pedestrian_navigation": True,
                  "ground": True},   # a large slab under the roads, so nothing drives off into the void
    },
    "server": {
        "host": "127.0.0.1",         # not "localhost": Windows tries ::1 and CARLA listens on IPv4 only
        "port": 2000,
        "tm_port": 8100,             # 8000 is often taken
        "launch": True,              # start CarlaUE4 ourselves (only when host is this machine)
        "quality": "Epic",
        "offscreen": True,
        "extra_args": [],
    },
    "simulation": {
        "step": 0.02,                # 50 Hz, as coupled-sim's fixed-time log
        "real_time": True,           # sleep so that simulated time follows the wall clock
        "weather": "CloudyNoon",     # a carla.WeatherParameters preset name, or a dict of its fields
        "sun_altitude": None,
        "tls_manager": "sumo",       # who runs the traffic lights: sumo | carla | none
        "lod_radius_m": 250.0,       # SUMO actors exist in CARLA only within this distance of a participant
        "traffic_manager": False,    # start CARLA's Traffic Manager (only for CARLA-driven autopilot cars)
    },
    "traffic": {
        "seed": 42,
        "cars_per_hour": 2400,
        "bikes_per_hour": 1200,
        "buses_per_hour": 12,
        "car_following": "EIDM",
        "car_types": {               # CARLA blueprint -> share of the car fleet
            "vehicle.toyota.prius": 3, "vehicle.volkswagen.t2_2021": 1, "vehicle.audi.a2": 2,
            "vehicle.citroen.c3": 3, "vehicle.mini.cooper_s_2021": 2, "vehicle.nissan.micra": 2,
            "vehicle.seat.leon": 3, "vehicle.bmw.grandtourer": 1, "vehicle.mercedes.coupe_2020": 1,
            "vehicle.tesla.model3": 2, "vehicle.ford.crown": 1, "vehicle.mercedes.sprinter": 1,
        },
        "bike_types": {"vehicle.gazelle.omafiets": 4, "vehicle.bh.crossbike": 1,
                       "vehicle.diamondback.century": 1},
        "bus_type": "vehicle.mitsubishi.fusorosa",
    },
    "participants": [],
    "agents": [],                    # scripted AI cars and walkers (coupledcarla/ai.py)
    "coupling": {"port": 2100, "wait_for_stations_s": 600, "visual_sync_at_s": []},
    "ehmi": {},                      # {states: {name: {text, color}}}; default in coupledcarla/ai.py
    "pois": [],
    "logging": {
        "dir": "logs",
        "radius_m": 150.0,           # AI actors within this distance of a participant are logged every frame
        "udp": {"host": "127.0.0.1", "port": 40131, "interval": 0.2},
        "carla_recorder": True,      # CARLA's own .log for replay (written on the server machine)
        "frames_every_s": 0.0,       # >0: save the driver camera every so many seconds (headless checks)
    },
    "trials": [{"duration_s": 300}],
}


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k not in ("car_types", "bike_types"):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load(path):
    path = Path(path)
    cfg = _merge(DEFAULTS, yaml.safe_load(path.read_text()) or {})
    cfg["map"]["name"] = cfg["map"]["name"] or cfg["name"]
    cfg["_path"] = str(path.resolve())
    cfg["_root"] = str(Path(__file__).resolve().parent.parent)   # the project folder (maps/, logs/)
    return cfg


def trial_config(cfg, trial):
    """The experiment config with one trial's overrides merged in."""
    over = {k: v for k, v in trial.items() if k in ("simulation", "traffic", "participants", "agents", "logging",
                                                      "coupling", "ehmi")}
    return _merge(cfg, over)


def map_dir(cfg):
    return Path(cfg["_root"]) / "maps" / cfg["map"]["name"]
