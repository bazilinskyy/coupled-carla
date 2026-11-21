"""What one synchronous tick costs the server with a participant's camera and the map's scenery (ms per tick).

The host's tick is mostly world.tick() waiting for the server's frame (23-25 ms on TUeCampus with scenery and a
1920x1080 station camera, 2026-11-21). This drives a car along the road in synchronous mode at a fixed step, the
scenery (coupled-carla's Scenery, --scenery MAPDIR, 250 m) spawned around it, and per case times world.tick():

  none        no camera (the world and scenery alone)
  WxH         one RGB camera of that size on the car, listening (frames dropped), the post-process profile
  WxH+dlss    the same with DLSS Super Resolution (enable_dlss), if the server's camera has it

The car is moved with apply_batch_sync (a fire-and-forget set_transform before the tick waits for a delayed ACK on
CARLA 1.0). Run once per server quality level (restart the server with -quality-level=High etc.) and compare.

    python tools/tick_cost.py --level TUeCampus --scenery maps/tue_campus_twin --cases none,1920x1080,1920x1080+dlss,1280x720 \\
        --label epic --out tick_epic.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def stats(ms):
    s = sorted(ms)
    if not s:
        return {}
    return {"median": round(s[len(s) // 2], 2), "p95": round(s[min(len(s) - 1, int(0.95 * len(s)))], 2),
            "mean": round(sum(s) / len(s), 2), "n": len(s)}


def route(cmap, start, n, step):
    import carla
    wp = cmap.get_waypoint(start, project_to_road=True, lane_type=carla.LaneType.Driving)
    out = []
    while wp is not None and len(out) < n:
        out.append(wp.transform)
        nxt = wp.next(step)
        wp = nxt[0] if nxt else None
    return out


def measure(client, world, spec, args, path):
    import carla
    lib = world.get_blueprint_library()
    car = world.spawn_actor(lib.find(args.vehicle) if lib.filter(args.vehicle) else lib.filter("vehicle.*")[0],
                            carla.Transform(path[0].location + carla.Location(z=0.5), path[0].rotation))
    car.set_simulate_physics(False)
    cam = None
    try:
        if spec != "none":
            size, _, opt = spec.partition("+")
            w, h = (int(v) for v in size.split("x"))
            bp = lib.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(w))
            bp.set_attribute("image_size_y", str(h))
            if opt == "dlss":
                if not bp.has_attribute("enable_dlss"):
                    return {"error": "no enable_dlss on this server's camera"}
                bp.set_attribute("enable_dlss", "true")
            cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.3, y=-0.35, z=1.25)), attach_to=car)
            cam.listen(lambda img: None)
        ms, i = [], 0
        total = int((args.warm_s + args.seconds) / args.step)
        warm = int(args.warm_s / args.step)
        for k in range(total):
            i = min(len(path) - 1, int(k * args.step * args.speed))
            client.apply_batch_sync([carla.command.ApplyTransform(car.id, path[i])])
            t = time.perf_counter()
            world.tick()
            if k >= warm:
                ms.append(1000 * (time.perf_counter() - t))
        return stats(ms)
    finally:
        if cam is not None:
            cam.stop()
            cam.destroy()
        car.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--level", default="")
    ap.add_argument("--scenery", default=None)
    ap.add_argument("--cases", default="none,1920x1080,1920x1080+dlss,1280x720")
    ap.add_argument("--step", type=float, default=1 / 60)
    ap.add_argument("--speed", type=float, default=8.3)
    ap.add_argument("--warm-s", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--vehicle", default="vehicle.lincoln.mkz")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    import os
    os.environ.setdefault("COUPLED_CARLA_MUTE", "1")
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(300.0)
    world = client.load_world(a.level) if a.level else client.get_world()
    cmap = world.get_map()
    start = cmap.get_spawn_points()[0].location
    path = route(cmap, start, 2000, 1.0)
    st = world.get_settings()
    st.synchronous_mode, st.fixed_delta_seconds = True, a.step
    world.apply_settings(st)
    scen = None
    try:
        if a.scenery:
            from coupledcarla import season, scenery
            scen = scenery.Scenery(client, world, a.scenery, {"radius_m": 250.0}, log=print,
                                   season={"state": "leaf", "lights": {}, "lights_radius_m": 0.0})
            scen.update([path[0].location])     # the drive stays within ~200 m of the start
            for _ in range(5):
                world.tick()
        res = {"label": a.label, "server": client.get_server_version(), "step_s": a.step, "cases": {}}
        for spec in [c for c in a.cases.split(",") if c]:
            res["cases"][spec] = measure(client, world, spec, a, path)
            print(spec, json.dumps(res["cases"][spec]), flush=True)
    finally:
        if scen is not None:
            scen.destroy()
        st.synchronous_mode, st.fixed_delta_seconds = False, None
        world.apply_settings(st)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
