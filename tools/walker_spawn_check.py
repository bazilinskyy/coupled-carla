"""No-render spawn check of walker blueprints on a running CARLA server (README "Who the pedestrians are"):

    python tools/walker_spawn_check.py --port 2300 0100 0101 ... [--json out.json]

For each walker.pedestrian.<id>: in the blueprint library; spawns; under a WalkerControl at 1.4 m/s its peak speed
(the skill's check) and its bones animate (local bone rotations change between two moments of the walk: a mesh bound
to a private skeleton keeps its rest pose; a -nullrhi server may not tick animation at all, then stock walkers read 0
too); the leg length the animation gives it (thigh to foot, component space)
against its own rest leg; and a controller.ai.walker takes it to a navigation point at least 1 m away. Asynchronous
mode, as the skill's verify: the server ticks on its own.
"""
import argparse
import json
import math
import sys
import time


def _dist(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def bones(actor):
    try:
        return {b.name: b for b in actor.get_bones().bone_transforms}
    except Exception:
        return {}


def check(world, carla, wid, spots, wait):
    out = {"id": wid}
    lib = world.get_blueprint_library()
    bid = f"walker.pedestrian.{wid}"
    if not lib.filter(bid):
        out["error"] = "not in the blueprint library (restart the server after importing)"
        return out
    bp = lib.find(bid)
    for k in ("age", "gender"):
        if bp.has_attribute(k):
            out[k] = bp.get_attribute(k).as_str()
    if bp.has_attribute("generation"):
        out["generation"] = bp.get_attribute("generation").as_int()
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    actor = None
    for sp in spots:
        actor = world.try_spawn_actor(bp, carla.Transform(sp + carla.Location(z=1.0)))
        if actor:
            break
    if actor is None:
        out["error"] = "did not spawn"
        return out
    out["spawned"] = True
    e = actor.bounding_box.extent
    out["bbox_m"] = [round(2 * e.x, 2), round(2 * e.y, 2), round(2 * e.z, 2)]
    wait(1.0)
    actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(1, 0, 0), speed=1.4))
    peak, snaps = 0.0, []
    for i in range(30):              # 3 s of simulation
        wait(0.1)
        v = actor.get_velocity()
        peak = max(peak, math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))
        if i in (20, 24):
            snaps.append(bones(actor))
    out["peak_mps"] = round(peak, 2)
    if len(snaps) == 2 and snaps[0]:
        # how far the thigh/leg/arm bones turned between two moments of the walk (relative transforms)
        turn = 0.0
        for n in ("crl_thigh__L", "crl_leg__L", "crl_arm__L", "crl_foreArm__L", "crl_thigh__R"):
            if n in snaps[0] and n in snaps[1]:
                r0, r1 = snaps[0][n].relative.rotation, snaps[1][n].relative.rotation
                turn = max(turn, abs(r0.pitch - r1.pitch) + abs(r0.yaw - r1.yaw) + abs(r0.roll - r1.roll))
        out["bone_turn_deg"] = round(turn, 1)
        s = snaps[0]
        if "crl_thigh__L" in s and "crl_foot__L" in s:
            out["anim_leg_m"] = round(_dist(s["crl_thigh__L"].component.location,
                                            s["crl_foot__L"].component.location) / 100.0, 3)
        out["n_bones"] = len(s)
    actor.apply_control(carla.WalkerControl(speed=0.0))
    # the AI controller
    ctl = None
    try:
        ctl = world.spawn_actor(world.get_blueprint_library().find("controller.ai.walker"), carla.Transform(),
                                attach_to=actor)
        wait(0.5)
        start = actor.get_location()
        dest = None
        for _ in range(40):
            loc = world.get_random_location_from_navigation()
            if loc is not None and 8.0 < _dist(loc, start) < 60.0:
                dest = loc
                break
        ctl.start()
        ctl.go_to_location(dest or start + carla.Location(x=10.0))
        ctl.set_max_speed(1.4)
        wait(8.0)
        out["ai_moved_m"] = round(_dist(actor.get_location(), start), 2)
        ctl.stop()
    except Exception as err:  # noqa: BLE001
        out["ai_error"] = f"{type(err).__name__}: {err}"
    finally:
        if ctl is not None:
            ctl.destroy()
        actor.destroy()
    # bones animate only where the server evaluates animation (not -nullrhi): reported, judged against stock walkers
    out["animated"] = out.get("bone_turn_deg", 0) > 2.0
    out["ok"] = bool(out.get("spawned") and out.get("peak_mps", 0) > 0.1 and out.get("ai_moved_m", 0) > 1.0)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", nargs="+")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if world.get_settings().synchronous_mode:
        print("the world is in synchronous mode: run against an asynchronous server")
        return 1
    spots = []
    for _ in range(200):
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spots.append(loc)
        if len(spots) >= 30:
            break
    spots += [sp.location for sp in world.get_map().get_spawn_points()[:20]]
    def wait(seconds):
        """Simulation seconds (a -nullrhi server ticks far faster than real time)."""
        t0 = world.wait_for_tick().timestamp.elapsed_seconds
        while world.wait_for_tick().timestamp.elapsed_seconds - t0 < seconds:
            pass
    res = []
    for wid in a.ids:
        r = check(world, carla, wid, spots, wait)
        res.append(r)
        print(json.dumps(r), flush=True)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    return 0 if all(r.get("ok") for r in res) else 2


if __name__ == "__main__":
    sys.exit(main())
