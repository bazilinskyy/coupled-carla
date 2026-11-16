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
        "internal_junctions": False,  # SUMO's net with netconvert's internal junctions (turning vehicles wait inside
                                     # the junction); off: they wait at the stop line, as the waiting points of the
                                     # re-made junctions lay in other lanes (mapbuild.NO_INTERNAL_JUNCTIONS)
        "buildings": True,           # also fetch building footprints (trial viewer, figures)
        "elevation": True,           # real terrain heights on the roads (AHN in NL, AWS Terrain Tiles elsewhere)
        "overture": True,            # Overture Maps buildings (needs duckdb), else OSM
        "national_buildings": True,  # measured heights: 3DBAG (NL), IGN BD TOPO (FR), NYC Open Data, PLATEAU (JP)
        "transit": True,             # the real bus lines and stops of OSM, with GTFS frequencies (transit.py)
        "scenery": {                 # CARLA meshes placed from real-world data (coupledcarla/scenery.py)
            "enabled": True, "buildings": True, "trees": True, "furniture": True, "guess_lamps": True, "signs": True,
            "municipal": True,       # city tree and street-light registers (realworld.MUNICIPAL) where they exist
            "ground": True,
            "catalog": "auto",       # mesh catalogue (data/meshes_<v>.json): "auto" = the CARLA version in use
                                     # (0.9.16 or 1.0; at run time a scenery built for the other one is rebuilt)
            "bgt": "auto",           # the Dutch BGT (bgt.py): ground surfaces (grass, bricks/tiles, asphalt,
                                     # gravel) and municipal street furniture, for maps in the Netherlands; false: off
            "canopy": True,          # trees without a mapped height: sized by Meta/WRI's 1 m canopy height map
            "ndw_signs": "auto",     # the Dutch national traffic sign register (NDW, ndwsigns.py): every official
                                     # sign at its position, for maps in the Netherlands; false: OSM's signs only
            "ndw_faces": True,       # CARLA 1.0: register signs show their Dutch design (data/nl_sign_faces.json,
                                     # meshes in /Game/CoupledCarla/Signs/NL, cooked into the server); a server
                                     # without them spawns the blank plate; false: blank plates as before
            "parked": True,          # parking stalls at the kerbs (OSM parking tags, BGT parkeervlak, a default by
                                     # road class) and on car parks, with a car each (parking.py -> parked.json)
            "shared": True,          # shared bikes and scooters at their GBFS positions (gbfs.py); a trial at
                                     # simulation.time "now" puts them where they are at its start
            "shared_per_station": 10,    # at most this many bikes shown per dock
            "park_shoulders_m": None,    # [min, max]: OpenDRIVE shoulders this wide count as parking strips (the
                                         # CARLA towns: [2.0, 3.0], their kerb shoulders; highways' are 3.5 m)
        },
        "opendrive": None,           # build the map from this OpenDRIVE file instead of OSM (a baked level's own,
                                     # e.g. a carla-digitaltwins twin); SUMO's net is made from it too
        "carla_level": None,         # load this CARLA level (by name) instead of generating an OpenDRIVE-only world
        "carla_town": None,          # a CARLA standard map (Town01..Town07, Town10HD, their _Opt variants; on 1.0 the
                                     # _Opt towns, Mine_01): its level, its OpenDRIVE for SUMO (maps/<town>/) and
                                     # per-feature defaults for a town (coupledcarla/towns.py)
        "place": None,               # [lat, lon] whose real weather and sun a map without a real place takes (a
                                     # CARLA town; default: the map's centre, for the towns their geoReference 0, 0)
        "photoreal": False,          # "google": Google's Photorealistic 3D Tiles over a baked level, under its own roads
                                     # (photoreal.py; needs the server's own key and billing, the Cesium plugins and
                                     # `photoreal prepare`); or a mapping of photoreal.DEFAULTS; false: off
        "urban_speeds": "auto",      # "auto": roads without an OSM maxspeed get the country's urban default limit
                                     # (speeds.URBAN_KMH) in the net and OpenDRIVE instead of netconvert's rural
                                     # defaults (100 km/h on primary roads): urban.typ.xml (urbanspeeds.py); false: off
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
        "multigpu": {                # CARLA 1.0 multi-GPU (multigpu.py): secondaries render the cameras
            "secondaries": [],       # {name, host, rpc_port (3000, 4000, ...), gpu, launch, roles: [station roles
                                     # whose camera it renders], root, packs, os, extra_args}; []: one server
            "primary_renders": False,    # False: the primary runs with -nullrhi (no GPU, no cameras of its own)
            "secondary_port": None,  # where the primary listens for secondaries (default port + 2)
            "primary_host": None,    # the primary's LAN address, for secondaries on other machines
            "relay": "auto",         # stations relay streams of secondaries on other machines (off: never)
            "placement": True,       # a station retries until its camera is on its role's own secondary
            "wait_s": 180,           # how long the host waits for the secondaries' ports at start-up
        },
    },
    "simulation": {
        "step": 0.02,                # 50 Hz, as coupled-sim's fixed-time log
        "real_time": True,           # sleep so that simulated time follows the wall clock
        "profile": False,            # print ms per tick by part of the loop every 10 s (the trial's in summary.json)
        "weather": "CloudyNoon",     # a carla.WeatherParameters preset name, a dict of its fields, or "real"
        "cloud_scale": 0.6,          # weather "real": CARLA cloudiness per % of cloud cover (0.6: overcast = 60;
                                     # above ~60 CARLA 0.9.16 draws an OpenDRIVE world's sky dark grey to black)
        "sun_altitude": None,
        "tls_manager": "sumo",       # who runs the traffic lights: sumo | carla | none
        "traffic_lights": {"style": "head"},   # how SUMO's lights show in CARLA (signals.py): "head" (a signal
                                     # head on a kerb pole per approach, CARLA's meshes, the lit lamp drawn) or
                                     # "marker" (a line pole and a big glowing point per lane, as before)
        "lod_radius_m": 250.0,       # SUMO actors exist in CARLA only within this distance of a participant
        "animate_radius_m": 60.0,    # ... and within this distance move with physics on (bridge/animate.py): wheels
                                     # roll and steer, cyclists pedal; 0: all placed with physics off as before
        "autopilot": "sumo",         # input autopilot: "sumo" (SUMO drives the car with the traffic's rules,
                                     # sumodriver.py; route: [[lat, lon], ...] or destination: {lat, lon} on the
                                     # participant) or "carla" (CARLA's BasicAgent on random lanes); a participant's
                                     # own autopilot: wins
        "traffic_manager": False,    # start CARLA's Traffic Manager (only for CARLA-driven autopilot cars)
        "camera_profile": "Dashcam", # CARLA 1.0: the post-process profile of every camera (exposure; CARLA's
                                     # Content/Carla/Config/PostProcess: Dashcam, AutomotiveHDR, Cinematic, ...;
                                     # "" for the server's default). CARLA 0.9.x ignores it
        "capture": {                 # cameras whose frames are kept (--frames-every, stations' --frames-every,
                                     # drone observers, tools/scenery_shots.py): coupledcarla/look.py
            "enabled": True,         # 0.9.x: no motion blur, bloom 0.3, no lens flare, a manual exposure from
                                     # the sun's altitude and the cloudiness; false: the cameras as before
            "supersample": 2,        # render at this multiple of the frame size and downscale (Lanczos); 1: off
            "render_hz": 10.0,       # 0.9.x: a camera that keeps a frame every few s still renders this often
                                     # (temporal anti-aliasing needs recent frames; at 1 Hz leaves and paint dither)
            "ev_bias": 0.0,          # added to the exposure (EV; +1 = one stop darker)
            "profile": None,         # CARLA 1.0: post-process profile of these cameras (e.g. "CoupledClean", or
                                     # "CoupledReal" fitted to Eindhoven photos by tools/look_fit.py; both in
                                     # coupledcarla/data/postprocess); None: camera_profile
        },
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
                   "occupancy": "auto",   # "auto": by zone and local hour (parking.PRESENCE), or a share 0-1
                   "town_layer": "auto"},  # CARLA towns' own parked cars: "auto" places Coupled CARLA's where the
                                          # map has stalls (unloading an _Opt town's ParkedVehicles layer; a full
                                          # level with cars of its own keeps them); true: the level's only; false: ours
        "town_lights": "auto",       # CARLA towns: street and building lights on at night (LightManager); "on",
                                     # "off", or false (left to the server)
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
        "bus_min_distance_m": 1000,  # ... each at least this long (CARLA towns: 300, they are small)
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
            "perception": {          # how walkers judge gaps at the kerb (coupledcarla/embodiedped.py)
                "model": "off",      # "off": every car's true position and speed (HCM critical gap); "embodied":
                                     # only what each walker has seen, with embodied-human's eyes (saccades),
                                     # peripheral vision and reaction times (needs embodied-human installed)
                "seed": 1,
                "distance_weber": 0.10, "tta_weber": 0.15,   # foveal SD of a seen distance and time to arrival
                "looming_threshold": 0.003,  # rad/s: a car's time to arrival is seen only above this expansion
                "prior_speed_kmh": 30.0,     # speed assumed for a car whose approach is not seen yet
                "prior_sd": 0.2, "max_rel_sd": 0.35, "recheck_s": 1.5, "fixation_s": 0.3, "side_gaze_deg": 85.0, "reach_m": 150.0,
                "log_gaze": True,    # <trial>/crowd_gaze.csv: gaze, believed and true gap of walkers at the kerb
            },
            "ai": {"count": 0,       # CARLA's AI walkers (WalkerAIController) on the level's navigation mesh: CARLA
                   "radius_m": 150,  # towns and baked levels only (towns.NavWalkers); spawned within radius_m of the
                   "speed_mps": 1.35, "run_share": 0.05,   # participants, walking to random destinations,
                   "cross_share": 0.05, "seed": 3},        # crossing at crosswalks (and cross_share elsewhere)
        },
    },
    "participants": [],
    "agents": [],                    # scripted AI cars and walkers (coupledcarla/ai.py)
    "drones": {                      # AI drones and the look of all drones (coupledcarla/drones.py); a participant
                                     # of kind `drone` flies one (input keyboard | gamepad | rc | bot)
        "missions": [],              # {kind: delivery | patrol | swarm | hover, id, count, airframe (consumer |
                                     # industrial), start_s, speed, clearance_m, ...}; see drones.py and the README
        "visual": "auto",            # auto | blueprint (CARLA 1.0 content pack) | mesh | shapes (0.9.16) | none
        "wireframe": False,          # also draw arms, rotor discs and lights with debug lines (on with shapes)
        "wireframe_radius_m": 120.0, # ... for drones within this distance of a pilot or observer camera
        "wind_mps": "auto",          # mean wind at 10 m: "auto" = Open-Meteo's with weather real, else CARLA's
                                     # wind_intensity / 10; Dryden gusts (MIL-F-8785C) on top
        "wind_from_deg": 270,        # compass direction the wind comes from (Open-Meteo's with weather real)
        "separation_m": 10.0,        # AI drones keep this far from each other and from the pilots' drones
        "log_every_s": 0.1,          # AI drones in drones.csv (pilots every frame; frames.csv has all every frame)
        "cameras": [],               # observer cameras saving frames: {name, lat, lon, height_m, look: drone name |
                                     # {lat, lon, height_m} | nearest, every_s, fov, size}
        "centre": None,              # {lat, lon} missions plan round (default: the first pilot, participant, box)
        "seed": 1,
        "routing": {                 # delivery and patrol routes in 3D (droneplan.Router); a mission's `routing`
                                     # overrides these keys for that mission
            "mode": "band",          # band: cruise in a fixed height band and fly round what reaches into it |
                                     # over: the old way, cruise clearance_m above everything under the route
            "band_m": [40.0, 60.0],  # cruise band above the ground (m): Wing cruises at about 45 m, Manna 50-80 m,
                                     # Amazon's MK30 55-85 m out and 85-115 m back; all under the EU's 120 m
            "layer_m": 4.0,          # layers within the band, by the route's heading (Doole et al. 2021,
            "layers": "heading",     # Aerospace 8(2) 38) | drone (one per drone, as before)
            "max_altitude_m": 120.0, # never cruise higher (EU 2019/947 open category, U-space airspace)
            "lateral_clearance_m": "auto",   # from buildings and trees reaching into the band: auto = SORA's
            "vertical_clearance_m": "auto",  # operational volume (flight geography half-width + contingency volume:
                                     # GNSS, position hold, map error, reaction distance, stopping distance), from
                                     # the airframe and speed (FOCA GM SORA 13.3.2); about 34 m and 17 m for the
                                     # industrial drone at 12 m/s
            "reaction_s": 1.0,       # ... its reaction time
            "gnss_m": 3.0,           # ... GNSS error (SORA's default; RTK: less)
            "position_hold_m": 3.0,  # ... position-holding error
            "map_error_m": 1.0,      # ... map and path-definition error
            "altimeter_m": 1.0,      # ... height error (barometric; 4 m for GNSS heights)
            "climb_over": True,      # climb above the band over a tall building when that costs less energy (the
                                     # battery model) than flying round it
            "over_penalty": 1.0,     # ... only when its energy times this is still less (> 1: stay in the band)
            "over_step_m": 10.0,     # ... heights tried above the band, this far apart
            "turn_accel_mps2": 2.0,  # sideways acceleration in turns: corners become arcs of v^2 / a, slower when
                                     # tighter (the industrial drone's tilt limit allows 5.6 m/s^2)
            "cell_m": 4.0,           # planning grid (A* with line-of-sight smoothing)
            "no_fly": [],            # areas never flown over: [[[lat, lon], ...], ...]
            "no_fly_osm": False,     # also OSM's aerodromes, heliports, helipads and military land (Overpass,
                                     # cached in real/osm_nofly.json)
            "no_fly_buffer_m": "auto",   # around them: auto = the lateral clearance
        },
    },
    "coupling": {"port": 2100, "wait_for_stations_s": 600, "visual_sync_at_s": []},
    "ehmi": {},                      # {states: {name: {text, color}}, slots}; default in coupledcarla/ai.py; design:
                                     # a designed eHMI (data/ehmi/<name>, a path or a dict; ehmi_design.py) with
                                     # placement and render: world | station | both; inside: what an AV passenger
                                     # sees of its own car's eHMI from behind: hide (default) | dim | mirror;
                                     # indicator: true (default) shows the passenger the eHMI the right way round
                                     # (ehmi_inside.py)
    # What the stations add (stationux.py), all off unless set; a participant's own `sound`, `displays` or
    # `haptics` is merged over these. Sound files and speech are made on the station.
    "sound": {                       # soundscape.py, calibrated in dB SPL (README: Sound)
        "enabled": False,
        "reference_db": 94.0,        # dB SPL at the ears of the full-scale 1 kHz tone (F3 at a station plays it)
        "vehicles": True,            # the nearest max_vehicles within max_distance_m one by one, the rest as a hum
        "max_vehicles": 16, "max_distance_m": 150.0,
        "model": "cnossos",          # vehicle sound power: cnossos (CNOSSOS-EU as amended 2021) | cnossos2015
        "road_surface": "DEF",       # CNOSSOS surface: DEF (dense asphalt/SMA), NL01 (porous), NL05 (SMA 8),
                                     # NL10 / NL11 (paving blocks, herring-bone / not)
        "diesel_cars": 0.2,          # share of combustion cars with a diesel (by actor id)
        "engine_sound": "recorded",  # recorded: loops of real engines resampled to the rpm (data/sounds/engines:
                                     # petrol, diesel for vans, buses and trucks, motorbike); synth: the additive model
        "engine_part_load_db": 6.0,  # engines up to this much quieter than CNOSSOS's average when cruising or
                                     # coasting (load under 0.4), so steady urban traffic is mostly tyres; 0: off
        "own_diesel": False,         # the participant's own combustion car has a diesel (else petrol)
        "ev_motor_db": 38.0,         # an electric own car's motor whine at the driver's ears at 50 km/h, dB(A)
                                     # (+20 lg v, louder with torque); 0: none
        "ev_motor_rpm_per_kmh": 72.0,    # its motor speed per km/h (the single fixed gear; a Tesla Model 3)
        "electric": "auto",          # electric cars (no engine): "auto" (CARLA's EVs and the scripted agents),
                                     # true, false, a share 0-1, or blueprint patterns
        "doppler": 1.0,              # 0: none, 1: physical
        "buildings": True,           # occlusion and facade reflections from maps/<name>/buildings.json
        "reflections": 2,            # first-order reflections (image sources) per source, 0-2
        "reverb": True,              # the street canyon's reverberant tail (street: {width_m, height_m} without
                                     # buildings)
        "ground": "auto",            # ISO 9613-2 ground factor G: auto (0: hard, in town) or 0..1
        "spatial": "stereo",         # stereo (panning) | binaural (HRTFs, for headphones)
        "hrtf": "kemar",             # binaural: kemar (MIT KEMAR, fetched once) | sphere (Brown & Duda model)
        "environment": True,         # birds, wind, rain and distant traffic from the trial's time and weather
        "weather": {},               # overrides of what the sound takes from the weather: rain (mm/h), wind (m/s),
                                     # temperature (C), humidity (%), wet (0..1)
        "recordings": True,          # fetch open recordings (birds, rain, bicycle bells) from Wikimedia Commons,
                                     # cached in ~/.cache/coupled-carla/sounds (else synthesised)
        "distant_traffic_db": 50.0,  # the city beyond the map at the peak hour, dB(A) (follows the hour)
        "birds_dawn_db": 58.0, "birds_day_db": 47.0,   # in spring among trees; scaled by season, trees, rain, wind
        "walkers": True, "footstep_db": 61.0,          # footsteps: dB(A) at 1 m of a step's loudest 100 ms with hard
                                     # soles on paving (~60 dB(A) at the walker's own ears; trainers 9 dB, grass 6 dB less)
        "footsteps": "shoes",        # recorded steps (data/sounds/footsteps): shoes | trainers | grass; synth: modelled
        "aps": True, "aps_db": 65.0, # accessible pedestrian signals near a pedestrian (dB(A) at 1 m per tick)
        "bicycles": {"bells": 0.03}, # bell rate (per second, when a pedestrian is ahead)
        "sirens": "lights",          # emergency vehicles: lights (with their special lights) | always | off
        "horn": False,               # a vehicle braking hard with a pedestrian ahead sounds its horn
        "own_vehicle": True,         # a driver's or passenger's car from inside: tyres, engine, wind, blinker
        "wind_db_at_100": 60.0, "blinker_db": 58.0,    # interior wind noise dB(A) at 100 km/h (+60 lg v)
        "interior_offset_db": 0.0,   # added to the own car's tyres and engine inside (a quieter or noisier car)
        "ambience": {"kind": "city", "db": 0.0, "file": None},   # a fixed bed: city | rural | a sound file; 0: none
        "avas": {"enabled": True, "db": 56.0, "until_kmh": 20.0, "vehicles": "electric"},   # db at 2 m (UN R138)
        "auditory_ehmi": {"enabled": False, "db": 65.0, "cues": []},   # cues: {ehmi: state | on: braking,
                                     # stopped, driving_off, pedestrian_ahead, at; sound | say | file; db; repeat_s}
        "takeover_db": 72.0,         # the take-over alarm in the car
        "record": False,             # host: write the first participant's soundscape to <trial>/audio.wav in
                                     # simulation time (as --record-audio); stations: also what they play
        "record_own_drone": True,    # a drone pilot's recording hears its own rotors at the camera
        "record_reference_db": 100.0,    # the host's audio.wav: dB SPL of a full-scale 1 kHz sine (headroom)
    },
    "displays": {                    # in-car displays over a driver's or AV passenger's view (incar.py)
        "cluster": None,             # a design (data/incar/<name>, a path or a dict), true for the default, or
        "hud": None,                 # {design, rect: [x, y, w(, h)]} (screen fractions)
        "centre": None,
        "messages": [],              # {at | at_m | on_state, seconds, display, text, color, takeover, sound, say,
                                     # haptic}
        "messages_file": None,       # the same as coupled-sim's CSV (at;seconds;display;text;color;atDistance;...)
        "style": "dashboard",        # dashboard: cluster and centre screen on the car's own panels, perspective-correct
                                     # and behind the wheel, the HUD as light on the windscreen (cockpit.py); overlay:
                                     # flat panels at the screen's edges (as before). A display with a rect is flat.
        "wheel": "auto",             # turn the steering wheel in driver and AV-passenger views (the meshes' does not):
                                     # auto when the station draws anything else (displays, sound, haptics), true
                                     # always (a per-frame image step), false never
        "hud_distance_m": 2.5,       # the HUD's virtual image: distance, width (deg) and centre below the line of sight
        "hud_fov_deg": 20.0,
        "hud_down_deg": 5.0,
        "cabin_ev_bias": -1.0,       # CARLA 0.9.x cabin views: exposure this many EV off the street's (look.py)
        "rhd": False,                # left-hand-traffic maps: "mirror" makes CARLA's left-hand-drive cabins right-hand
                                     # drive (rhd.py: a second camera at the mirrored seat, its cabin mirrored and
                                     # the street from the real seat; about halves the station's frame rate)
                                     # The seat comes from data/driver_eyes.json; a participant's `fov` (deg) or
                                     # `screens` (1: 90 deg, 3: 180) sets the view's width unless --fov is given.
    },
    "vr": {                          # VR stations (vr.py, vrxr.py): a participant with view: vr, or a station started
                                     # with --display vr, sees the world in a headset (OpenXR); a participant's own
                                     # `vr` block overrides these
        "ipd_mm": 63.0,              # eye distance; null: the runtime's eye positions (the headset's IPD setting)
        "scale": 0.5,                # per-eye render size as a share of the headset's recommended size (each axis)
        "max_size": [1600, 1600],    # cap on the per-eye render size (w, h)
        "size": None,                # [w, h]: a fixed per-eye render size instead of scale
        "overscan": 0.10,            # extra field of view around each eye for the runtime's reprojection
        "predict_ms": "auto",        # head-pose prediction: auto = the measured pose-to-image latency, or ms
        "max_predict_ms": 80.0,      # cap on the prediction (the runtime's reprojection corrects the rest)
        "smooth": 0.5,               # angular-velocity smoothing for the prediction (0 none .. 0.9 heavy)
        "reference": "local",        # OpenXR space: local (seated; origin where the head starts) | stage
        "recenter": "trial",         # the head at the trial start is the seat's eye point (Home recentres again)
        "runtime": "openxr",         # openxr: a headset | sim: a scripted head without headset (tests)
        "sim_motion": "look",        # runtime sim: look (yaw +-35 deg, pitch +-10 deg) | still
        "fov_deg": [100, 100],       # runtime sim: per-eye field of view (h, v)
        "sim_hz": 90.0,              # runtime sim: display rate
        "mirror": True,              # the station's window mirrors the left eye
        "panel_distance_m": 1.5,     # head-locked panel (station text, prompts, visual-sync bar): distance, width
        "panel_width_deg": 50.0,
        "displays_in_eyes": True,    # in-car displays on the car's panels in each eye (dashboard style)
    },
    "haptics": {                     # haptics.py: take-over requests and cues on the wheel, gamepads, UDP devices
        "enabled": False, "wheel": True, "gamepad": True,
        "udp": [],                   # ["host:port"]: a JSON datagram per cue
        "cues": [],                  # {at | on: over_limit, takeover, message, "ehmi:<state>"; pattern, intensity,
                                     # duration}
    },
    "film": {                        # demo-video cameras (film.py; tools/make_videos.py writes this section): frames
                                     # kept on a grid of simulation time from start_s for duration_s at fps
        "start_s": 30.0, "duration_s": 20.0, "fps": 25.0, "size": [1280, 720], "fov": 90.0, "settle_s": 2.0,
        "jpeg_quality": 93, "attributes": {},
        "camera": None,              # {kind: attach | follow | tripod | orbit | path | drone, target, ...} (film.py)
        "cameras": [],               # more cameras, each {name, kind, ...}
        "hold": {},                  # {role: until_s}: a host-local driver waits with the hand brake on until then
    },
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
    # study tools (coupledcarla/study.py, studytools.py, dashboard.py; check a file with python -m coupledcarla.validate)
    "conditions": {},                # name -> overrides of a trial, used by trials (condition: / conditions: [..])
    "design": {                      # trials generated from conditions, ordered per participant (run --participant N)
        "factors": {},               # factor -> [condition names]: every combination is a condition (full factorial)
        "order": "latin",            # blocks per participant: latin (balanced Latin square) | full (every
                                     # permutation) | random | none
        "repeat": 1,                 # each combination this many times, within its block
        "block_by": None,            # a factor whose levels form the blocks (default: each combination is a block)
        "trial": {},                 # merged into every generated trial (e.g. duration_s)
        "practice": [],              # trials run first, in this order
        "after": [],                 # trials run last, in this order
    },
    "dashboard": {                   # experimenter dashboard (coupledcarla/dashboard.py), also run --dashboard
        "enabled": False,
        "host": "127.0.0.1",         # where it listens; anything but localhost needs the token it prints
        "port": 8780,
        "gate_trials": True,         # each trial waits for "Start trial" on the dashboard once it is loaded
    },
}


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k not in ("car_types", "bike_types"):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _scenery_switches(cfg):
    """`map.scenery: false` (or `simulation.scenery: false`) means no scenery: the merge keeps the bare false, and
    code reading `.get("scenery") or {}` took it as an empty dict, i.e. everything on."""
    for sect, off in (("map", {"enabled": False}), ("simulation", {"kinds": []})):
        if cfg.get(sect, {}).get("scenery") in (False, None):
            cfg[sect]["scenery"] = dict(DEFAULTS[sect]["scenery"], **off)
    return cfg


def load(path):
    from . import towns
    path = Path(path)
    raw = yaml.safe_load(path.read_text()) or {}
    # a CARLA town (map.carla_town): the town's defaults under the file's own settings (towns.overrides)
    cfg = _scenery_switches(_merge(_merge(DEFAULTS, towns.overrides(raw)), raw))
    cfg["map"]["name"] = cfg["map"]["name"] or cfg["name"]
    cfg["_path"] = str(path.resolve())
    cfg["_root"] = str(Path(__file__).resolve().parent.parent)   # the project folder (maps/, logs/)
    return cfg


def trial_config(cfg, trial):
    """The experiment config with one trial's overrides merged in."""
    over = {k: v for k, v in trial.items() if k in ("simulation", "traffic", "participants", "agents", "logging",
                                                      "coupling", "ehmi", "eye_contact", "sound", "displays",
                                                      "haptics", "drones")}
    out = _merge(cfg, over)
    # patch: {agents: {id: {...}}, participants: {role: {...}}} changes one agent or participant (a condition can
    # switch av1's eHMI without repeating the whole agents list)
    for section, key in (("agents", "id"), ("participants", "role")):
        for name, fields in ((trial.get("patch") or {}).get(section) or {}).items():
            out[section] = [_merge(item, fields) if item.get(key) == name else item for item in out.get(section) or []]
    return _scenery_switches(out)


def map_dir(cfg):
    return Path(cfg["_root"]) / "maps" / cfg["map"]["name"]


def place(cfg, bbox=None):
    """(lat, lon) of the map's place for weather and sun: map.place, else the centre of the box."""
    p = (cfg.get("map") or {}).get("place")
    if p:
        return float(p[0]), float(p[1])
    bb = bbox or cfg["map"]["bbox"]
    return (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
