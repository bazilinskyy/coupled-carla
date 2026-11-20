"""How many frames per second does a participant's camera deliver to the client? (no coupled-carla needed)

Only the carla package and numpy: runs the same on a native Windows client and on the WSL client, so the two can
be compared against one server (is it WSL's network bridge that caps the stream?). Asynchronous server, one RGB
camera on a car driven along the road by set_transform, per size: frames delivered per second (callback only, and
with the frame copied into a numpy array, as a display client does), and the server's own frame rate.

    python camera_stream_test.py --host 127.0.0.1 --port 2000 --level TUeCampus --sizes 1920x1080,1280x720 --out res.json
"""
import argparse
import json
import sys
import time


def measure(world, lib, start_tf, size, copy, seconds, warm):
    import numpy as np
    import carla
    w, h = (int(v) for v in size.split("x"))
    cmap = world.get_map()
    wp = cmap.get_waypoint(start_tf.location, project_to_road=True)
    path = []
    while wp is not None and len(path) < 1200:
        path.append(wp.transform)
        nxt = wp.next(1.0)
        wp = nxt[0] if nxt else None
    car = world.spawn_actor(lib.filter("vehicle.*")[0], carla.Transform(path[0].location + carla.Location(z=0.5),
                                                                         path[0].rotation))
    car.set_simulate_physics(False)
    bp = lib.find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.3, y=-0.35, z=1.25)), attach_to=car)
    n = [0]

    def cb(img):
        if copy:
            np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, :3].copy()
        n[0] += 1
    cam.listen(cb)
    try:
        t0 = time.time()
        frames = 0
        tm = None
        i = 0
        while time.time() - t0 < warm + seconds:
            el = time.time() - t0
            j = min(len(path) - 1, int(el * 8.3))
            if j != i:
                i = j
                car.set_transform(path[i])
            world.wait_for_tick(5.0)
            if el >= warm:
                if tm is None:
                    tm, n[0], frames = time.time(), 0, 0
                frames += 1
        wall = time.time() - tm
        return {"size": size, "copy": copy, "camera_fps": round(n[0] / wall, 1), "server_fps": round(frames / wall, 1),
                "mbytes_per_s": round(n[0] / wall * w * h * 4 / 1e6, 1)}
    finally:
        cam.stop()
        cam.destroy()
        car.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--level", default="")
    ap.add_argument("--sizes", default="1920x1080,1280x720")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--warm", type=float, default=10.0)
    ap.add_argument("--label", default=sys.platform)
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(180.0)
    world = client.load_world(a.level) if a.level else client.get_world()
    st = world.get_settings()
    st.synchronous_mode = False
    st.fixed_delta_seconds = None
    world.apply_settings(st)
    lib = world.get_blueprint_library()
    start = world.get_map().get_spawn_points()[0]
    rows = []
    for size in [s for s in a.sizes.split(",") if s]:
        for copy in (False, True):
            r = measure(world, lib, start, size, copy, a.seconds, a.warm)
            r.update(client=a.label, python=sys.version.split()[0], server=client.get_server_version())
            print(json.dumps(r), flush=True)
            rows.append(r)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(rows, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
