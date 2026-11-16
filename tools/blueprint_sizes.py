"""Measure every vehicle blueprint of a running CARLA server: bounding box, wheels, base type, colour attribute.

Writes coupledcarla/data/blueprint_sizes_<version>.json (drivers.sizes() reads it: SUMO vehicle types get the CARLA
cars' real lengths and widths; fleet.py maps the register's body types onto these blueprints). One spawn per
blueprint on one of the map's spawn points, in synchronous mode: settled for 1 s, then full throttle for 3 s; "drives"
records whether it moved (CARLA 1.0's older UE4 cars registered from the CoupledCarla pack spawn but some do not
drive: fine for SUMO traffic, which the bridge moves, not for a participant's car). A server without rendering
(-nullrhi) is enough.

    python tools/blueprint_sizes.py --port 2000            # writes data/blueprint_sizes_<server version>.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

DATA = Path(__file__).resolve().parent.parent / "coupledcarla" / "data"


def attrs(bp):
    out = {}
    for a in bp:
        rec = {"modifiable": bool(a.is_modifiable)}
        try:
            rec["value"] = a.as_str()
        except RuntimeError:
            rec["value"] = ""
        out[a.id] = rec
    return out


def main(argv=None):
    import carla
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--out", help="default data/blueprint_sizes_<server version>.json")
    a = ap.parse_args(argv)
    client = carla.Client(a.host, a.port)
    client.set_timeout(60.0)
    world = client.get_world()
    version = client.get_server_version().split("-")[0]
    sp = world.get_map().get_spawn_points()
    settings = world.get_settings()
    was = (settings.synchronous_mode, settings.fixed_delta_seconds)
    settings.synchronous_mode, settings.fixed_delta_seconds = True, 0.05
    world.apply_settings(settings)
    out = {}
    try:
        for i, bp in enumerate(sorted(world.get_blueprint_library().filter("vehicle.*"), key=lambda b: b.id)):
            at = attrs(bp)
            actor = world.try_spawn_actor(bp, sp[(i * 7) % len(sp)]) if sp else None
            if actor is None:
                print(f"{bp.id}: not spawned", flush=True)
                continue
            try:
                for _ in range(20):
                    world.tick()
                p0 = actor.get_location()
                actor.apply_control(carla.VehicleControl(throttle=0.6))
                for _ in range(60):
                    world.tick()
                moved = actor.get_location().distance(p0)
                bb = actor.bounding_box
                wheels = at.get("number_of_wheels", {}).get("value") or "0"
                rec = {"length": round(2 * bb.extent.x, 3), "width": round(2 * bb.extent.y, 3),
                       "height": round(2 * bb.extent.z, 3), "centre_x": round(bb.location.x, 3),
                       "wheels": int(float(wheels)), "base_type": (at.get("base_type", {}).get("value") or "").lower(),
                       "color": bool(at.get("color", {}).get("modifiable")), "drives": moved > 1.0,
                       "attributes": sorted(at)}
                out[bp.id] = rec
                print(f"{bp.id}: {rec['length']} x {rec['width']} x {rec['height']} m, {rec['base_type']}, "
                      f"{rec['wheels']} wheels, colour {'yes' if rec['color'] else 'no'}, moved {moved:.1f} m", flush=True)
            finally:
                actor.destroy()
                world.tick()
    finally:
        settings.synchronous_mode, settings.fixed_delta_seconds = was
        world.apply_settings(settings)
    path = Path(a.out) if a.out else DATA / f"blueprint_sizes_{version}.json"
    path.write_text(json.dumps({"carla": version, "note": "vehicle bounding boxes (m), tools/blueprint_sizes.py: one "
                                "spawn per blueprint", "measured": time.strftime("%Y-%m-%d"), "blueprints": out},
                               indent=1))
    print(f"{len(out)} blueprints -> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
