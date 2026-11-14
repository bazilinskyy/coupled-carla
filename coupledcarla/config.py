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
                       "residential", "living_street", "busway"],   # busway: bus-only roads (restricted lanes)
        "cycleways": True,           # keep highway=cycleway (SUMO cyclists ride them)
        "sidewalks": False,          # guessed sidewalks in the OpenDRIVE (CARLA walkers, traffic.walkers); the
                                     # SUMO net never has them (SUMO crashes on moveToXY with sidewalks)
        "trim_dead_ends": True,      # drop dead-end chains and keep the largest strongly connected road graph
        "lefthand": False,           # left-hand traffic (UK, Japan, Australia, ...)
        "buildings": True,           # also fetch building footprints (trial viewer, figures)
        "elevation": True,           # real terrain heights on the roads (AHN in NL, AWS Terrain Tiles elsewhere)
        "overture": True,            # Overture Maps buildings (needs duckdb), else OSM
        "national_buildings": True,  # measured heights: 3DBAG (NL), IGN BD TOPO (FR), NYC Open Data, PLATEAU (JP)
        "transit": True,             # the real bus lines and stops of OSM, with GTFS frequencies (transit.py)
        "scenery": {                 # CARLA meshes placed from real-world data (coupledcarla/scenery.py)
            "enabled": True, "buildings": True, "trees": True, "furniture": True, "guess_lamps": True, "signs": True,
            "ground": True,
            "catalog": "auto",       # mesh catalogue (data/meshes_<v>.json): "auto" = the CARLA version in use
                                     # (0.9.16 or 1.0; at run time a scenery built for the other one is rebuilt)
            "bgt": "auto",           # the Dutch BGT (bgt.py): ground surfaces (grass, bricks/tiles, asphalt,
                                     # gravel) and municipal street furniture, for maps in the Netherlands; false: off
            "canopy": True,          # trees without a mapped height: sized by Meta/WRI's 1 m canopy height map
            "ndw_signs": "auto",     # the Dutch national traffic sign register (NDW, ndwsigns.py): every official
                                     # sign at its position, for maps in the Netherlands; false: OSM's signs only
            "parked": True,          # parking stalls at the kerbs (OSM parking tags, BGT parkeervlak, a default by
                                     # road class) and on car parks, with a car each (parking.py -> parked.json)
            "shared": True,          # shared bikes and scooters at their GBFS positions (gbfs.py); a trial at
                                     # simulation.time "now" puts them where they are at its start
            "shared_per_station": 10,    # at most this many bikes shown per dock
        },
        "opendrive": None,           # build the map from this OpenDRIVE file instead of OSM (a baked level's own,
                                     # e.g. a carla-digitaltwins twin); SUMO's net is made from it too
        "carla_level": None,         # load this CARLA level (by name) instead of generating an OpenDRIVE-only world
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
        "weather": "CloudyNoon",     # a carla.WeatherParameters preset name, a dict of its fields, or "real"
        "sun_altitude": None,
        "tls_manager": "sumo",       # who runs the traffic lights: sumo | carla | none
        "lod_radius_m": 250.0,       # SUMO actors exist in CARLA only within this distance of a participant
        "traffic_manager": False,    # start CARLA's Traffic Manager (only for CARLA-driven autopilot cars)
        "time": None,                # local time at the place, "YYYY-MM-DD HH:MM", "HH:MM" (today) or "now":
                                     # the sun stands where it did; with weather: real, Open-Meteo's weather
        "scenery": {"radius_m": 400.0,   # scenery cells within this distance of participants (0: all at once)
                    "kinds": ["building", "tree", "bush", "lamp", "furniture", "sign", "ground", "water",
                              "shared"],
                    "leaves": "auto",    # deciduous trees by the date of simulation.time (season.py): "auto"
                                         # (leaf dates from latitude; no time: in leaf), "modis" (MODIS MCD12Q2
                                         # dates where plausible), "leaf", "autumn", "bare", or false (in leaf)
                    "bare": "small",     # deciduous trees out of leaf (CARLA has no bare tree): "small" (half
                                         # size), "drop" (left out) or "keep" (in leaf)
                    "lamp_lights": "auto",   # CARLA 1.0: lamps spawned as street lights, which the server lights
                                             # when the sun is at or below the horizon; true: always; false: never
                    "lights_radius_m": 200.0},   # ... only within this distance of a participant (0: all)
        "parked": {"enabled": True,  # parked cars (map.scenery.parked) in the stalls taken at the trial's hour
                   "radius_m": 250.0,     # streamed within this distance of the participants
                   "occupancy": "auto"},  # "auto": by zone and local hour (parking.PRESENCE), or a share 0-1
    },
    "traffic": {
        "seed": 42,
        "cars_per_hour": "auto",     # cars entering per hour, or "auto": from the map's lane-km by road class and
                                     # the trial's hour (coupledcarla/counts.py)
        "counts": "auto",            # "auto": fit the cars to open traffic counts where they cover the map
                                     # (NYC DOT, DfT, Lyon, Montevideo, Auckland, Portland, VTrans) with SUMO's
                                     # routeSampler, topped up to cars_per_hour; false: random trips only
        "bikes_per_hour": 1200,
        "buses_per_hour": 12,        # random bus trips, only on maps without OSM bus lines
        "public_transport": {        # the map's real bus lines (map.transit): at their GTFS frequency for the
            "enabled": True,         # trial's local hour (simulation.time, else 08:00 on a weekday), else the OSM
            "dwell_s": 20,           # interval tag, else default_headway_min
            "default_headway_min": 10,
            "live": "auto",          # start with the real buses where they are now (GTFS-Realtime): "auto" when
        },                           # simulation.time is "now", true, or false
        "cycling": {                 # where and how many cyclists ride (coupledcarla/cycling.py)
            "routes": "auto",        # "auto": bike trips follow the OSM cycle routes (LCN/RCN/NCN, knooppunten) and
                                     # cycle tracks and lanes of the map; false: uniform random trips only
            "counts": "auto",        # "auto": bikes_per_hour from open bike counters in or near the box (Lyon, NYC,
                                     # London, Flanders, Sydney) at the trial's hour and day type; false: as set
            "route_share": 0.7,      # share of bike trips that start and end on the cycle network (the rest: random)
            "od_weight": 10,         # how much more a cycle-route edge draws those trips' ends than a street
            "prefer_route": 0.5,     # cyclists' perceived travel time on a cycle route, relative to the real one
            "prefer_infra": 0.7,     # the same on other cycle tracks and lanes
            "max_bikes_per_hour": 3000,   # cap on the counted demand (a cyclist in CARLA costs ~3 ms per tick)
            "dock_weight": 2.0,      # bike edges within 30 m of a shared-bike dock (GBFS, gbfs.py) draw 1 + this
                                     # times the trip starts and ends; 0: off
        },
        "fleet": "auto",             # "auto": car body types and colours from the open vehicle register where
                                     # there is one (fleet.py: RDW for NL, NY DMV for New York, Waka Kotahi
                                     # for New Zealand, Senatran for Brazilian cities); else car_types
        "car_following": "EIDM",
        "speed_limits": "auto",      # "auto": roads without an OSM maxspeed (netconvert gives them 80-100 km/h)
                                     # get the country's urban default limit (speeds.py); false: as built
        "speed_factor": "auto",      # cars' desired speed / limit: "auto" = from open speed measurements where they
                                     # cover the map (speeds.py: PBOT Portland, Montevideo, Taipei), else
                                     # normc(1.0,0.1,0.8,1.2); a SUMO distribution string; false: that default
        "car_types": {               # CARLA blueprint -> share of the car fleet
            "vehicle.toyota.prius": 3, "vehicle.volkswagen.t2_2021": 1, "vehicle.audi.a2": 2,
            "vehicle.citroen.c3": 3, "vehicle.mini.cooper_s_2021": 2, "vehicle.nissan.micra": 2,
            "vehicle.seat.leon": 3, "vehicle.bmw.grandtourer": 1, "vehicle.mercedes.coupe_2020": 1,
            "vehicle.tesla.model3": 2, "vehicle.ford.crown": 1, "vehicle.mercedes.sprinter": 1,
        },
        "bike_types": {"vehicle.gazelle.omafiets": 4, "vehicle.bh.crossbike": 1,
                       "vehicle.diamondback.century": 1},
        "bus_type": "vehicle.mitsubishi.fusorosa",
        "walkers": {                 # the pedestrian crowd near the participants (coupledcarla/crowdpeds.py)
            "count": 0,              # walkers, or "auto": walkers_per_km x the kerb length within radius_m, scaled by
                                     # CROWD's pedestrians per minute of the place against the world's (and at night)
            "radius_m": 120, "seed": 1,
            "behaviour": "auto",     # "auto": speeds, crossings (how often, where: marked crossing, junction or
                                     # mid-block) and gap acceptance of the map's country from the CROWD dashcam
                                     # dataset; "off": walk up and down beside the roads, never crossing (ai.Crowd)
            "places": "auto",        # walkers start where Overture/OSM places, transit stops and open pedestrian
                                     # counts (NYC DOT, City of Sydney) are; false: evenly along the kerbs
            "walkers_per_km": 8,     # count "auto": walkers per km of kerb at CROWD's world pedestrian rate
            "max_count": 80,         # cap on count "auto" (30 walkers cost the server ~12 ms per tick)
            "crossings_per_km": 2.0, # crossings per km walked where CROWD's crossing share is the median city's
            "speed_mps": "auto",     # median walking speed, or "auto": the place's CROWD crossing speed
            "jaywalk_share": "auto", # share of crossings mid-block, or "auto": the country's in CROWD
        },
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
        "analyse": True,             # encounter measures in <trial>/analysis/ after each trial (coupledcarla/analysis.py)
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
                                                      "coupling", "ehmi", "eye_contact")}
    return _merge(cfg, over)


def map_dir(cfg):
    return Path(cfg["_root"]) / "maps" / cfg["map"]["name"]
