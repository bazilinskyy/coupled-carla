"""How SUMO's drivers drive: car-following, junction and size parameters of the vehicle types (traffic.py), chosen
from published measurements and checked with tools/traffic_check.py (README: Traffic, driving behaviour).

Car following is SUMO's Extended Intelligent Driver Model (EIDM: Salles, Kaufmann & Reuss 2020, "Extending the
Intelligent Driver Model in SUMO and Verifying the Drive Off Trajectories with Aerial Measurements", SUMO User
Conference; calibrated on drone recordings of two signalised junctions) for cars, buses and bikes. Its own driver
errors (reaction time, estimation errors, drive-off curve) stay at SUMO's defaults, which come from that calibration.
SUMO's Krauss model, which bikes used before, dawdles by a random part of its acceleration in every step: at the
bridge's 0.02 s step that is a new random acceleration 50 times a second (bike jerk p95 6.8 m/s^3 vs 1.3-2.6 for the
EIDM vehicles in the Eindhoven rush hour).

- decel is IDM's comfortable deceleration b, the rate a driver plans to brake at. SUMO's default (4.5 m/s^2, a
  Krauss value) made EIDM drivers brake late and hard. Cars: 3.4 m/s^2, the deceleration AASHTO's Green Book (2018,
  section 3.2.2) takes as comfortable for most drivers (90 % brake at least this hard when they must stop); buses:
  2.0 m/s^2, as standing passengers need (about 0.15 g in Hoberock 1977, "A survey of longitudinal acceleration
  comfort studies in ground transportation vehicles"); bikes: 2.5 m/s^2.
- emergencyDecel: what the vehicle can do on dry asphalt when it must (cars 8, buses 6, bikes 5 m/s^2; SUMO's
  defaults 9 / 7 / 7 are above what a bus with standing passengers or a city bike does).
- accel: cars 2.6 m/s^2 (SUMO's default; EIDM shapes the drive-off below it), buses 1.0, bikes 1.0 m/s^2.
- tau (time gap) 1.2 s, minGap 2 m for cars and buses (IDM's urban values, Treiber & Kesting 2013, "Traffic Flow
  Dynamics", table 11.2: T 1.0-1.5 s, s0 2 m); bikes tau 1.0 s, minGap 1.0 m.
- impatience 0 and jmIgnoreFoeProb 0: nobody forces a way through a junction against a vehicle with priority (the
  former 0.1 made every driver a little pushy from the start, so vehicles with priority braked hard for them).
  Tried and left out: --time-to-impatience 60 (SUMO: 180 s, after the 120 s time-to-teleport), so that a driver
  who finds no gap pushes in rather than being teleported (critical gaps shrink with waiting: Polus, Lazar & Livneh
  2003, J. Transp. Eng. 129(5)); New York's 3 yield teleports went, London's 25 stayed and Eindhoven's collisions
  rose from 4 to 11 (seed 42). SUMO's jmTimegapMinor (1 s) stays: 2 s more than tripled the vehicles teleported after waiting 120 s at a minor road
  (Eindhoven rush hour, seed 42: 2 -> 7).
- length, width, height of each vehicle type from its CARLA blueprint's bounding box (data/blueprint_sizes_*.json),
  so SUMO's gaps are the gaps seen in CARLA; SUMO's defaults (car 5 m, bus 12 m, bike 1.6 m) overlapped the bodies of
  long CARLA cars in queues and left gaps behind short ones. A bus is at most 2.55 m wide (EU Directive 96/53/EC;
  the CARLA bus's box includes its mirrors).
"""
import json
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"
# SUMO options of every trial (the bridge, sumo_simulation.py) and of tools/traffic_check.py
SUMO_ARGS = ["--lateral-resolution", "0.25",        # the sublane model: cyclists at the right, cars overtaking them
             "--collision.check-junctions",
             # a collision is logged, not "resolved" by teleporting the collider to its next edge (SUMO's default),
             # which in CARLA made a car vanish or jump; mingap 0: only touching bodies count
             "--collision.action", "warn", "--collision.mingap-factor", "0"]
MAX_WIDTH = {"bus": 2.55, "passenger": 2.1, "delivery": 2.3, "bicycle": 0.8}
# CARLA bodies wider than their class's MAX_WIDTH, without mirrors: the collision box measured with World.cast_ray
# across the middle of the vehicle on CARLA 0.9.16 (lin4080, 2026-11-16). CARLA's bus is a scaled-up minibus: 3.18 m
# where a real city bus has 2.55. SUMO gets this width, so two of them never drive or turn side by side closer than
# SUMO's minGapLat (bodies touched at a 0.45 m SUMO gap: Eindhoven rush hour, buses turning onto -6110).
BODY_WIDTH = {"vehicle.mitsubishi.fusorosa": 3.18}

CAR = {"accel": 2.6, "decel": 3.4, "emergencyDecel": 8.0, "apparentDecel": 3.4, "tau": 1.2, "minGap": 2.0,
       "jmIgnoreFoeProb": 0, "impatience": 0}
BUS = {"accel": 1.0, "decel": 2.0, "emergencyDecel": 6.0, "apparentDecel": 2.0, "tau": 1.2, "minGap": 2.0,
       "jmIgnoreFoeProb": 0, "impatience": 0}
BIKE = {"accel": 1.0, "decel": 2.5, "emergencyDecel": 5.0, "apparentDecel": 2.5, "tau": 1.0, "minGap": 1.0,
        "jmIgnoreFoeProb": 0, "impatience": 0}


@lru_cache(maxsize=None)
def sizes(version="0.9.16"):
    """{blueprint id: {length, width, height}} measured on a CARLA server (bounding boxes), {} when missing."""
    path = DATA / f"blueprint_sizes_{version}.json"
    try:
        return json.loads(path.read_text())["blueprints"]
    except (OSError, ValueError, KeyError):
        return {}


def size_attrs(bp, vclass):
    """length/width/height attributes of a SUMO vType for a CARLA blueprint ("" when the size is unknown)."""
    s = sizes().get(bp) or sizes("1.0").get(bp)     # a CARLA 1.0 blueprint (fleet.py names them on 1.0 levels)
    if not s:
        return ""
    width = BODY_WIDTH.get(bp) or min(s["width"], MAX_WIDTH.get(vclass, s["width"]))
    return f'length="{s["length"]:.2f}" width="{width:.2f}" height="{s["height"]:.2f}"'


def attrs(params):
    return " ".join(f'{k}="{v}"' for k, v in params.items())


def car(bp, vclass, cf, speed_factor, probability):
    return (f'<vType id="{bp}" vClass="{vclass}" probability="{probability}" carFollowModel="{cf}" '
            f'speedFactor="{speed_factor}" '
            f'{attrs(CAR)} {size_attrs(bp, vclass)}').rstrip() + "/>"


def bike(bp, cf, probability):
    return (f'<vType id="{bp}" vClass="bicycle" probability="{probability}" carFollowModel="{cf}" maxSpeed="6.5" '
            f'speedFactor="normc(0.8,0.15,0.5,1.1)" latAlignment="right" minGapLat="0.4" {attrs(BIKE)} '
            f'{size_attrs(bp, "bicycle")}').rstrip() + "/>"


def bus(bp, cf):
    return (f'<vType id="{bp}" vClass="bus" carFollowModel="{cf}" {attrs(BUS)} '
            f'{size_attrs(bp, "bus")}').rstrip() + "/>"


RED_RUNNER = "#red"     # suffix of a bike type whose riders ride on through red (drivermix.py); the bridge strips it


def bike_red_runner(bp, cf):
    """The red-light running variant of a bike type: SUMO's junction model lets it go on through a red light at
    walking pace (jmDriveAfterRedTime, jmDriveRedSpeed), still giving way to vehicles on conflicting links."""
    from .drivermix import RED_RUNNER as RR
    xml = bike(bp, cf, 1.0).replace(f'id="{bp}"', f'id="{bp}{RED_RUNNER}"').replace(' probability="1.0"', "")
    return xml.replace("/>", " " + " ".join(f'{k}="{v}"' for k, v in RR.items()) + "/>")
