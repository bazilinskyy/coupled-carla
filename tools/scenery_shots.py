"""Render views of a map with all its real-world scenery, for the README.

    python tools/scenery_shots.py experiments/cities/lyon.yaml --time "2026-11-13 10:00" \
        --view bellecour:45.7578,4.8320,80,270,-15 --out docs/img/

Each --view is name:lat,lon,height above the road,compass heading,pitch. Starts its own CARLA server.
"""
import argparse
import queue
import sys
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config  # noqa: E402
from coupledcarla.geo import Geo, ground, heading_to_yaw  # noqa: E402
from coupledcarla.run import load_world, set_weather  # noqa: E402
from coupledcarla.scenery import Scenery  # noqa: E402
from coupledcarla.server import Server  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--time", default="12:00")
    ap.add_argument("--weather", default="real")
    ap.add_argument("--view", action="append", required=True)
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--size", default="1600x900")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    mdir = config.map_dir(cfg)
    geo = Geo(mdir / "geo.net.xml")
    server = Server(cfg["server"])
    client = server.start()
    try:
        world = load_world(client, mdir / "map.xodr", cfg["map"], slab=True)
        sc = dict(cfg["simulation"], weather=args.weather, time=args.time)
        bb = cfg["map"]["bbox"]
        set_weather(world, sc, ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2), mdir / "real")
        scen = Scenery(client, world, mdir, {"radius_m": 0})
        scen.update([])
        print(f"{scen.n} scenery meshes", flush=True)
        w, h = args.size.split("x")
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", w)
        bp.set_attribute("image_size_y", h)
        bp.set_attribute("fov", "80")
        cmap = world.get_map()
        for v in args.view:
            name, spec = v.split(":")
            lat, lon, up, heading, pitch = (float(x) for x in spec.split(","))
            x, y = geo.to_carla(lat, lon)
            loc = ground(cmap, x, y) + carla.Location(z=up)
            q = queue.Queue()
            cam = world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=heading_to_yaw(heading), pitch=pitch)))
            cam.listen(q.put)
            for _ in range(120):          # let the meshes stream in and the exposure settle
                world.wait_for_tick()
            img = None
            while not q.empty():
                img = q.get()
            cam.destroy()
            path = Path(args.out) / f"scenery_{cfg['name']}_{name}.jpg"
            img.save_to_disk(str(path.with_suffix(".png")))
            from PIL import Image
            Image.open(path.with_suffix(".png")).convert("RGB").save(path, quality=85)
            path.with_suffix(".png").unlink()
            print(path, flush=True)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
