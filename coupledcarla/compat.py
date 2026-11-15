"""Run the same experiments on CARLA 0.9.x (Unreal Engine 4) and CARLA 1.0 (Unreal Engine 5.8).

CARLA 1.0 renamed or dropped many blueprints (vehicle.lincoln.mkz_2020 is vehicle.lincoln.mkz, the Audi A2 is
gone, walkers start at 0015) and renamed a few calls. Experiments name the 0.9 blueprints; `blueprint()` turns
such a name into the closest blueprint the connected server has, and says so once in the log.
"""
import logging
import random
import zlib

# 0.9.16 id -> 1.0 candidates, closest first (body type and size)
ALIASES = {
    "vehicle.lincoln.mkz_2020": ["vehicle.lincoln.mkz"],
    "vehicle.lincoln.mkz_2017": ["vehicle.lincoln.mkz"],
    "vehicle.audi.a2": ["vehicle.mini.cooper", "vehicle.kia.niro"],
    "vehicle.audi.tt": ["vehicle.ue4.audi.tt"],
    "vehicle.audi.etron": ["vehicle.kia.niro", "vehicle.tesla.model3"],
    "vehicle.bmw.grandtourer": ["vehicle.kia.niro"],
    "vehicle.chevrolet.impala": ["vehicle.ue4.chevrolet.impala"],
    "vehicle.citroen.c3": ["vehicle.mini.cooper"],
    "vehicle.dodge.charger_2020": ["vehicle.dodge.charger"],
    "vehicle.dodge.charger_police": ["vehicle.dodgecop.charger"],
    "vehicle.dodge.charger_police_2020": ["vehicle.dodgecop.charger"],
    "vehicle.ford.crown": ["vehicle.taxi.ford", "vehicle.ue4.ford.crown"],
    "vehicle.ford.mustang": ["vehicle.ue4.ford.mustang"],
    "vehicle.ford.ambulance": ["vehicle.ambulance.ford"],
    "vehicle.jeep.wrangler_rubicon": ["vehicle.nissan.patrol"],
    "vehicle.mercedes.coupe": ["vehicle.ue4.mercedes.ccc"],
    "vehicle.mercedes.coupe_2020": ["vehicle.ue4.mercedes.ccc"],
    "vehicle.mercedes.sprinter": ["vehicle.sprinter.mercedes"],
    "vehicle.micro.microlino": ["vehicle.mini.cooper"],
    "vehicle.mini.cooper_s": ["vehicle.mini.cooper"],
    "vehicle.mini.cooper_s_2021": ["vehicle.mini.cooper"],
    "vehicle.nissan.micra": ["vehicle.mini.cooper"],
    "vehicle.nissan.patrol_2021": ["vehicle.nissan.patrol"],
    "vehicle.seat.leon": ["vehicle.kia.niro"],
    "vehicle.tesla.cybertruck": ["vehicle.nissan.patrol"],
    "vehicle.toyota.prius": ["vehicle.kia.niro"],
    "vehicle.volkswagen.t2": ["vehicle.sprinter.mercedes"],
    "vehicle.volkswagen.t2_2021": ["vehicle.sprinter.mercedes"],
    "vehicle.carlamotors.carlacola": ["vehicle.carlacola.actors"],
    "vehicle.carlamotors.european_hgv": ["vehicle.carlacola.actors"],
    "vehicle.carlamotors.firetruck": ["vehicle.firetruck.actors"],
    "vehicle.mitsubishi.fusorosa": ["vehicle.fuso.mitsubishi"],
}

# SUMO vehicle class -> blueprint `base_type`s that may stand in for it (when no alias fits)
CLASSES = {
    "passenger": ["car"], "taxi": ["car"], "evehicle": ["car"], "private": ["car"],
    "delivery": ["van", "car"], "emergency": ["van", "car"], "authority": ["car"],
    "bus": ["bus", "truck"], "coach": ["bus", "truck"], "truck": ["truck"], "trailer": ["truck"],
    "motorcycle": ["motorcycle"], "moped": ["motorcycle"], "bicycle": ["bicycle"],
}
WHEELS = {"motorcycle": 2, "moped": 2, "bicycle": 2}

_said = set()


def _ids(lib):
    return {bp.id: bp for bp in lib}


def _base_type(bp):
    if bp.has_attribute("base_type"):
        return bp.get_attribute("base_type").as_str().lower()
    return None


def blueprint(lib, wanted, vclass=None, rng=None):
    """The blueprint named `wanted` on this server, or its closest stand-in, or None.

    `wanted` may be a pattern (walker.pedestrian.*). Stand-ins come from ALIASES, then from blueprints of the
    same `base_type` as the SUMO class, then from the same number of wheels. Walkers missing on 1.0 map to a
    fixed (per name) walker that exists, so a participant keeps the same look across trials.
    """
    ids = _ids(lib)
    if wanted in ids:
        return ids[wanted]
    if "*" in wanted:
        found = list(lib.filter(wanted))
        if found:
            return (rng or random).choice(found)
    pick = next((ids[c] for c in ALIASES.get(wanted, []) if c in ids), None)
    if pick is None and wanted.startswith("walker."):
        walkers = sorted((bp for bp in lib.filter("walker.pedestrian.*")), key=lambda b: b.id)
        if walkers:
            pick = walkers[zlib.crc32(wanted.encode()) % len(walkers)]
    if pick is None and wanted.startswith("vehicle."):
        vclass = vclass or "passenger"
        vehicles = sorted(lib.filter("vehicle.*"), key=lambda b: b.id)
        kinds = CLASSES.get(vclass, ["car"])
        same = [bp for bp in vehicles if _base_type(bp) in kinds]
        if not same:
            wheels = WHEELS.get(vclass, 4)
            same = [bp for bp in vehicles if bp.has_attribute("number_of_wheels")
                    and bp.get_attribute("number_of_wheels").as_int() == wheels]
        if same:
            pick = same[zlib.crc32(wanted.encode()) % len(same)]
    if pick is not None and wanted not in _said:
        _said.add(wanted)
        logging.getLogger("coupledcarla").warning("blueprint %s is not on this server; using %s", wanted, pick.id)
    return pick


def find(world, wanted, vclass=None):
    """world.get_blueprint_library().find() that falls back to a stand-in (raises if none exists)."""
    bp = blueprint(world.get_blueprint_library(), wanted, vclass)
    if bp is None:
        raise RuntimeError(f"no blueprint {wanted} (nor a stand-in) on this CARLA server")
    return bp


def left_lane(wp):
    return wp.get_left_waypoint() if hasattr(wp, "get_left_waypoint") else wp.get_left_lane()


def right_lane(wp):
    return wp.get_right_waypoint() if hasattr(wp, "get_right_waypoint") else wp.get_right_lane()


def server_major(client):
    """0 for CARLA 0.9.x / 0.10.x, 1 for CARLA 1.0 (UE 5.8) and later."""
    try:
        return int(client.get_server_version().split(".")[0].split("-")[0])
    except (ValueError, IndexError):
        return 0


def setup_camera(bp, every_s=0.0, profile="Dashcam", shutter=400, world=None, capture=None):
    """Exposure of an RGB camera; returns the interval at which the caller should keep frames itself (0: all).

    CARLA 0.9.x: a camera that keeps a frame only every few seconds renders only then (sensor_tick) with a fixed
    manual exposure, as auto exposure never adapts between such far-apart frames.
    CARLA 1.0 has no exposure attributes on the RGB camera; exposure comes from a post-process profile
    (Content/Carla/Config/PostProcess), all with histogram auto exposure. Rendered every few seconds it never adapts
    (a cloudy day clipped 25 % of the frame to white, night was 99.7 % black), so it renders every tick and the
    caller keeps every every_s seconds; "Dashcam" was the best of CARLA's profiles at the Eindhoven twin (day mean
    brightness 105-116 of 255 with nothing clipped, night 30 with 1 % crushed; the default profile: 101 and 5).

    capture (simulation.capture, look.py), for a camera whose frames are kept: on 0.9.x no motion blur, bloom 0.3,
    a manual exposure from the sun and clouds of world's weather, and it renders at least capture.render_hz (the
    caller keeps every every_s); on 1.0 capture.profile instead of profile when set. None: as before."""
    from . import look
    cap = look.settings(capture) if capture is not None else None
    if cap is not None and not cap["enabled"]:
        cap = None
    if bp.has_attribute("post_process_profile"):
        prof = (cap or {}).get("profile") or profile
        if prof:
            bp.set_attribute("post_process_profile", str(prof))
        return float(every_s or 0.0)
    if cap is not None:
        look.apply_09(bp, cap, world.get_weather() if world is not None else None, shutter)
        tick = look.render_interval(every_s, cap)
        if tick > 0 and bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(tick))
        return float(every_s) if 0 < tick < float(every_s or 0) else 0.0
    if every_s and every_s > 0:
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(every_s))
        set_manual_exposure(bp, shutter)
    return 0.0


def set_manual_exposure(bp, shutter):
    """Fixed exposure for cameras that save few frames (auto exposure would never adapt between them)."""
    if bp.has_attribute("exposure_mode"):
        bp.set_attribute("exposure_mode", "manual")
    if bp.has_attribute("shutter_speed"):
        bp.set_attribute("shutter_speed", str(shutter))
