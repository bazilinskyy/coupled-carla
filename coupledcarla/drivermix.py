"""Drivers and riders who differ from each other: per-vehicle parameters drawn when SUMO inserts a vehicle.

drivers.py gives every vehicle type one set of EIDM and lane-change parameters, so all drivers of a class kept the
same time gap, accelerated and changed lanes alike; only the desired speed varied (speedFactor, speeds.py). Real
drivers differ: car-following parameters fitted to single drivers' trajectories spread widely (Ossen, Hoogendoorn
& Gorte 2006, "Interdriver differences in car-following: a vehicle trajectory-based study", Transportation Research
Record 1965, Dutch helicopter trajectories; Kesting & Treiber 2008, "Calibrating car-following models by using
trajectory data", TRR 2088: IDM time gaps of single drivers 0.6-2.0 s, minimum gaps 1-3 m, accelerations 1-3 m/s^2).
No open data set gives per-vehicle gaps in a Dutch city centre (NDW's loops are on motorways and report minute
aggregates), so the spreads below are those published ranges around drivers.py's means, as distributions cut at
their ends (DRAWS).

Cyclists in the Netherlands: a share of the bikes are pedelecs (e-bikes), which ride faster. KiM, Cycling Facts
2023 (Kennisinstituut voor Mobiliteitsbeleid): e-bikes made 18 % of cycling trips in 2019, and their trips are about
70 % longer, so roughly 27 % of the cycled kilometres (EBIKE_SHARE, the share of bikes on the road). Pedelec riders
average 18.7 km/h (Fietsberaad, cited by KiM) and ride about 2 km/h faster than riders of ordinary bikes (Schleinitz
et al. 2017, "The German Naturalistic Cycling Study", Safety Science 92): BIKE_KMH and EBIKE_KMH. Red lights: video
observations at five junctions in The Hague (SWOV) found a quarter of the cyclists crossing on red (35 % under 20,
24 % aged 21-64, 22 % over 65; 27.4 % in SWOV's overall observation report): RED_RUNNER_SHARE of the bikes may ride
on through a red light at walking pace when nothing is coming (their type's red-running twin, drivers.RED_RUNNER:
SUMO's jmDriveAfterRedTime / jmDriveRedSpeed, which TraCI cannot set per vehicle; they still give way to vehicles
on the conflicting links). Red-light running is on in the Netherlands only (traffic.cyclist_red_running: auto),
where SWOV's numbers apply: SUMO's red runners do not look as carefully as real ones (Lyon, 300 s: 4 of 6 SUMO
collisions involved one). The other draws use the same values everywhere for lack of comparable data
(traffic.driver_mix: false switches all of this off).

apply(vid, kind, seed) sets one vehicle's parameters through TraCI (the bridge on each new SUMO vehicle in a trial,
tools/traffic_check.py in a SUMO-only run). Pure draws (draw()) are unit-tested.
"""
import math
import random
import zlib

EBIKE_SHARE = 0.27
BIKE_KMH = (16.7, 3.0, 10.0, 24.0)      # mean, sd, min, max (km/h): ordinary bikes
EBIKE_KMH = (18.7, 3.0, 12.0, 25.0)     # pedelecs (assisted up to 25 km/h)
RED_RUNNER_SHARE = 0.24
RED_RUNNER = {"jmDriveAfterRedTime": "300", "jmDriveRedSpeed": "2.0"}
# (distribution, a, b, lo, hi): normal(mean a, sd b) or lognormal(median a, sigma b), cut to [lo, hi]
DRAWS = {
    # cut tighter than the published ranges at the short end: SUMO's junction model has no human anticipation, and
    # the shortest time gaps and minimum gaps (0.7 s, 1 m) with the softest braking made cars run into vehicles
    # stuck in junctions (19 maps, 300 s: SUMO collisions 29 -> 46, Sao Paulo 0 -> 11)
    "car": {"tau": ("lognormal", 1.2, 0.2, 0.9, 2.0), "minGap": ("normal", 2.0, 0.4, 1.5, 3.0),
            "accel": ("normal", 2.6, 0.4, 1.6, 3.6), "decel": ("normal", 3.5, 0.4, 3.0, 4.5),
            "lcAssertive": ("lognormal", 1.0, 0.25, 0.7, 1.8), "lcSpeedGain": ("lognormal", 1.0, 0.4, 0.3, 3.0),
            "lcCooperative": ("normal", 0.9, 0.08, 0.6, 1.0)},
    "bus": {"tau": ("normal", 1.2, 0.1, 1.0, 1.5), "accel": ("normal", 1.0, 0.1, 0.8, 1.2)},
    "bike": {"tau": ("lognormal", 1.0, 0.25, 0.6, 2.0), "minGap": ("normal", 1.0, 0.25, 0.5, 1.8),
             "accel": ("normal", 1.0, 0.2, 0.6, 1.6)},
}
SETTERS = {"tau": "setTau", "minGap": "setMinGap", "accel": "setAccel", "decel": "setDecel"}
LANE_CHANGE = ("lcAssertive", "lcSpeedGain", "lcCooperative")


def rng_for(vid, seed=42):
    return random.Random(zlib.crc32(f"{seed}:{vid}".encode()))


def sample(rng, spec):
    kind, a, b, lo, hi = spec
    v = rng.gauss(a, b) if kind == "normal" else a * math.exp(rng.gauss(0.0, b))
    return min(hi, max(lo, v))


def draw(kind, rng):
    """{parameter: value} of one vehicle of kind car | bus | bike (bike also: ebike, max_speed m/s, red_runner)."""
    out = {k: round(sample(rng, s), 3) for k, s in DRAWS.get(kind, {}).items()}
    if kind == "bike":
        ebike = rng.random() < EBIKE_SHARE
        m, sd, lo, hi = EBIKE_KMH if ebike else BIKE_KMH
        out["ebike"] = ebike
        out["max_speed"] = round(min(hi, max(lo, rng.gauss(m, sd))) / 3.6, 3)
        out["red_runner"] = rng.random() < RED_RUNNER_SHARE
    return out


def kind_of(vclass):
    return {"bicycle": "bike", "bus": "bus"}.get(vclass, "car" if vclass in ("passenger", "taxi", "delivery",
                                                                               "private", "evehicle") else None)


def apply(vid, vclass, seed=42, traci_mod=None, red_running=True):
    """Draws and sets the parameters of SUMO vehicle vid (SUMO gives it its own copy of its type, "<type>@<vid>");
    returns them, or None for a class left as it is."""
    kind = kind_of(vclass)
    if kind is None:
        return None
    if traci_mod is None:
        import traci as traci_mod
    p = draw(kind, rng_for(vid, seed))
    v = traci_mod.vehicle
    if kind == "bike" and not red_running:
        p["red_runner"] = False
    if kind == "bike" and p["red_runner"]:      # its type's red-running twin (traffic.py: variants.add.xml), first:
        from .drivers import RED_RUNNER as SUFFIX           # a new type replaces the values set below
        base = v.getTypeID(vid).split("@")[0]
        if not base.endswith(SUFFIX) and base + SUFFIX in traci_mod.vehicletype.getIDList():
            v.setType(vid, base + SUFFIX)
        else:
            p["red_runner"] = False
    for k, setter in SETTERS.items():
        if k in p:
            getattr(v, setter)(vid, p[k])
    for k in LANE_CHANGE:
        if k in p:
            v.setParameter(vid, f"laneChangeModel.{k}", str(p[k]))
    if kind == "bike":
        v.setMaxSpeed(vid, p["max_speed"])
        v.setSpeedFactor(vid, 1.5)          # the bike's own top speed, not the road's limit, sets its pace
        try:                                # inserted faster than its own top speed: start at that, not brake
            if v.getSpeed(vid) > p["max_speed"]:
                v.setPreviousSpeed(vid, p["max_speed"])
        except Exception:  # noqa: BLE001 - older SUMO without setPreviousSpeed
            pass
    return p


def red_running(cfg):
    """traffic.cyclist_red_running: auto (in the Netherlands) | true | false."""
    v = (cfg.get("traffic") or {}).get("cyclist_red_running", "auto")
    if isinstance(v, bool):
        return v
    try:
        from . import realworld
        return bool(realworld.in_netherlands((cfg.get("map") or {}).get("bbox") or [0, 0, 0, 0]))
    except Exception:  # noqa: BLE001
        return False
