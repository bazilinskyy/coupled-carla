"""Check that the facades of a baked twin level light their windows at night on a CARLA 1.0 server.

    python tools/window_lights_check.py [--host 127.0.0.1] [--port 2000] [--map EhvCentre] [--out logs/windows]
        [--poses 0,1,2] [--at X,Y,Z,YAW] [--sun-day 40] [--sun-dusk 0] [--sun-night -20]

The twin's facade materials (carla-digitaltwins bake_level.py, MI_TwinFacadeNight) light a share of their windows
when CARLA's weather sets NightFactor (sun below the horizon), by building use: homes, offices, shops. For each
camera pose this saves four frames and prints, for each, the share of warm bright pixels in the upper two thirds
of the frame (the facades) and the mean brightness:

* day     sun --sun-day, the stations' fixed daylight exposure: no lit windows (warm share near 0: sunlit brick
          is darker and redder than a lit window);
* dusk    sun --sun-dusk (NightFactor 0.5 at 0 degrees): about half the night's windows lit;
* night   sun --sun-night, the same daylight exposure: what a station shows at night;
* night_ev sun --sun-night, a night exposure (f/1.4, 1/30 s, ISO 3200, EV100 about 1): the lit windows plainly.

Verdict per pose: the night_ev frame has at least three times the warm share of the day frame. The poses are
EhvCentre road points facing the most windowed facade (from the twin's manifest: homes, shop streets, offices);
--at gives another one. Needs a rendering server (a -nullrhi server has no camera frames).
Leaves the weather as it found it and destroys its camera.
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import carla  # noqa: E402

# EhvCentre: (dominant use in view, x, y, z eye height, yaw) in CARLA coordinates, from the twin's manifest
# (spawn points on its roads; the 30-degree heading with the most window facade area 8-60 m away).
POSES = [
    ("home", 185.3, 648.0, 18.1, 90),
    ("shop", -589.5, 221.9, 19.0, 120),
    ("office", -207.1, -233.4, 18.5, 180),
    ("home", -790.7, 211.9, 19.2, 300),
    ("shop", -258.7, 690.3, 19.0, 300),
    ("shop", -355.6, -312.4, 20.5, 150),
]
STATION_SHUTTER = 220      # coupled-carla station.py: the driver camera's fixed daylight exposure


def tick(world, n=1):
    for _ in range(n):
        if world.get_settings().synchronous_mode:
            world.tick()
        else:
            world.wait_for_tick(10.0)


def camera_bp(world, night_ev):
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    attrs = [("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "90"), ("motion_blur_intensity", "0"),
             ("exposure_mode", "manual"),
             # CARLA 1.0 has no exposure attributes: a post-process profile with auto exposure (compat.setup_camera)
             ("post_process_profile", "Dashcam")]
    if night_ev:     # EV100 about 1 (a night street): f/1.4, 1/30 s, ISO 3200
        attrs += [("fstop", "1.4"), ("shutter_speed", "30"), ("iso", "3200")]
    else:
        attrs += [("shutter_speed", str(STATION_SHUTTER))]
    for k, v in attrs:
        if bp.has_attribute(k):
            bp.set_attribute(k, v)
    return bp


def measure(img):
    """(share of warm bright pixels in the upper two thirds, mean brightness) of a BGRA frame."""
    raw = bytes(img.raw_data)
    w, h = img.width, img.height
    warm = n = 0
    lum = 0.0
    for y in range(0, h * 2 // 3, 3):
        row = y * w * 4
        for x in range(0, w, 3):
            i = row + x * 4
            b, g, r = raw[i], raw[i + 1], raw[i + 2]
            lum += 0.114 * b + 0.587 * g + 0.299 * r
            n += 1
            warm += r > 200 and g > 110 and r > b + 40   # lamp-lit window; brick is darker and less yellow
    return warm / max(n, 1), lum / max(n, 1)


def shoot(world, tf, night_ev, path):
    # a baked World Partition level streams its cells around the spectator: put it over the view first (without
    # it the frames showed sky over a black void)
    world.get_spectator().set_transform(carla.Transform(tf.location + carla.Location(z=30.0), carla.Rotation(pitch=-90)))
    tick(world, 30)
    cam = world.spawn_actor(camera_bp(world, night_ev), tf)
    frames = []
    try:
        try:
            cam.listen(frames.append)
        except RuntimeError as err:     # a -nullrhi server has no camera
            sys.exit(f"no camera: {err}")
        tick(world, 60)                 # streaming and the camera's auto exposure (CARLA 1.0 profiles adapt)
        frames.clear()
        t0 = time.time()
        while not frames and time.time() - t0 < 15:
            tick(world)
        if not frames:
            print("  no camera frame")
            return None
        img = frames[-1]
        img.save_to_disk(str(path))
        return measure(img)
    finally:
        try:
            cam.stop()
        except RuntimeError:
            pass
        cam.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="EhvCentre", help="load this map unless it is the one loaded")
    ap.add_argument("--out", default="logs/window_lights")
    ap.add_argument("--poses", default=None, help="comma-separated indices into POSES (default: all)")
    ap.add_argument("--at", default=None, help="X,Y,Z,YAW in CARLA coordinates instead of the POSES")
    ap.add_argument("--sun-day", type=float, default=40.0)
    ap.add_argument("--sun-dusk", type=float, default=0.0)
    ap.add_argument("--sun-night", type=float, default=-20.0)
    a = ap.parse_args()

    client = carla.Client(a.host, a.port)
    client.set_timeout(180.0)
    world = client.get_world()
    if a.map and not world.get_map().name.endswith(a.map):
        world = client.load_world(a.map)
    print("server", client.get_server_version(), "map", world.get_map().name)
    if a.at:
        x, y, z, yaw = (float(v) for v in a.at.split(","))
        poses = [("at", x, y, z, yaw)]
    else:
        idx = [int(i) for i in a.poses.split(",")] if a.poses else range(len(POSES))
        poses = [POSES[i] for i in idx]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    old = world.get_weather()
    shots = [("day", a.sun_day, False), ("dusk", a.sun_dusk, False), ("night", a.sun_night, False),
             ("night_ev", a.sun_night, True)]
    ok = 0
    try:
        for k, (use, x, y, z, yaw) in enumerate(poses):
            tf = carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch=6.0, yaw=yaw))
            print(f"pose {k} ({use}): ({x:.1f}, {y:.1f}, {z:.1f}) yaw {yaw:.0f}")
            res = {}
            for label, sun, night_ev in shots:
                w = world.get_weather()
                w.sun_altitude_angle = sun
                w.cloudiness = min(w.cloudiness, 30.0)
                w.precipitation = 0.0
                world.set_weather(w)
                tick(world, 3)
                path = out / f"pose{k}_{use}_{label}.png"
                m = shoot(world, tf, night_ev, path)
                if m is None:
                    continue
                res[label] = m
                print(f"  {label:8s} sun {sun:+5.1f}: warm bright pixels {100 * m[0]:5.2f} %, "
                      f"mean brightness {m[1]:5.1f} -> {path}")
            if "day" in res and "night_ev" in res:
                good = res["night_ev"][0] >= 3 * max(res["day"][0], 0.001)
                ok += good
                print(f"  lit windows at night: {'yes' if good else 'NO'} "
                      f"(night_ev/day warm share x{res['night_ev'][0] / max(res['day'][0], 1e-4):.1f})")
    finally:
        world.set_weather(old)
        tick(world, 2)
    print(f"{ok} of {len(poses)} poses show lit windows at night")
    return 0 if ok == len(poses) else 1


if __name__ == "__main__":
    sys.exit(main())
