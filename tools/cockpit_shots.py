"""Before/after frames of the in-car views (coupledcarla/cockpit.py) on a running CARLA server.

    python tools/cockpit_shots.py --port 2500 --out scratch/cockpit                    # four cars, day/dusk/night
    python tools/cockpit_shots.py --port 2500 --cars vehicle.tesla.model3 --seats passenger --times day

For each car, seat and time of day: "before" is the old fixed mount (x -0.05, y -0.38, z 1.22 for every car; the
passenger x -0.15, y 0.36, z 1.15), the street's exposure (look.py) and the displays as flat panels (displays.style
overlay); "after" is the car's measured seat (data/driver_eyes.json), the cabin exposure (cockpit.cabin_ev_bias) and
the displays on the car's own panels with the wheel turned by `--steer`. Frames are saved side by side as JPEG.
"""
import argparse
import os
import queue
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
from coupledcarla import cockpit, incar, look  # noqa: E402

TIMES = {"day": 40.0, "dusk": 2.0, "night": -12.0}
OLD = {"driver": (-0.05, -0.38, 1.22, 0.0), "passenger": (-0.15, 0.36, 1.15, 0.0)}
CARS = ["vehicle.lincoln.mkz_2020", "vehicle.tesla.model3", "vehicle.dodge.charger_2020", "vehicle.mini.cooper_s_2021"]


class _Ctl:
    def __init__(self, steer=0.0, throttle=0.3):
        self.steer, self.throttle, self.brake, self.reverse = steer, throttle, 0.0, False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--cars", nargs="*", default=CARS)
    ap.add_argument("--seats", nargs="*", default=["driver", "passenger"])
    ap.add_argument("--times", nargs="*", default=list(TIMES))
    ap.add_argument("--size", default="960x540")
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--steer", type=float, default=0.12, help="CARLA steer for the turned wheel")
    ap.add_argument("--spawn", type=int, default=0, help="the map's spawn point")
    ap.add_argument("--out", default="scratch/cockpit")
    args = ap.parse_args()
    import carla
    import pygame
    from PIL import Image, ImageDraw
    pygame.init()
    size = tuple(int(v) for v in args.size.split("x"))
    client = carla.Client(args.host, args.port)
    client.set_timeout(120)
    world = client.get_world()
    orig = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    world.apply_settings(s)
    lib = world.get_blueprint_library()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    disp_cfg = {"cluster": True, "hud": "hud_speed", "centre": "centre_screen",
                "messages": [{"at": 0, "seconds": 60, "display": "centre", "text": "Pedestrian crossing ahead",
                              "color": "#FFD60A"}]}
    car = None

    def shoot(kind, mount, ev=None, ticks=8):
        bp = lib.find("sensor.camera." + kind)
        bp.set_attribute("image_size_x", str(size[0] * (2 if kind == "rgb" else 1)))
        bp.set_attribute("image_size_y", str(size[1] * (2 if kind == "rgb" else 1)))
        bp.set_attribute("fov", str(args.fov))
        if kind == "rgb":
            look.apply_09(bp, {"enabled": True, "ev_bias": ev or 0.0}, world.get_weather())
        x, y, z, pitch = mount
        cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch)),
                                attach_to=car)
        q = queue.Queue()
        cam.listen(q.put)
        img = None
        for _ in range(ticks):
            world.tick()
            img = q.get(timeout=30)
        cam.stop()
        cam.destroy()
        a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
        if kind == "depth":
            a = a.astype(np.float32)
            return (a[..., 2] + a[..., 1] * 256.0 + a[..., 0] * 65536.0) / (256 ** 3 - 1) * 1000.0
        return look.downscale(a[..., 2::-1].copy(), size)

    def compose(rgb, seat, style, cab, steer):
        kind = "driver" if seat == "driver" else "av_passenger"
        ic = incar.InCar(dict(disp_cfg, style=style), kind, size, log=lambda m: None, sim_time="14:30")
        ic.set_cabin(cab)
        ic.update(0.0, 38.0, lights=0, control=_Ctl(steer), limit=50)
        surf = pygame.surfarray.make_surface(np.ascontiguousarray(rgb).swapaxes(0, 1))
        ic.draw(surf)
        return pygame.surfarray.array3d(surf).swapaxes(0, 1)

    try:
        for bpid in args.cars:
            if car is not None:
                car.destroy()
            bp = lib.find(bpid)
            if bp.has_attribute("color"):
                bp.set_attribute("color", "40,60,110")
            pts = world.get_map().get_spawn_points()
            car = next(a for a in (world.try_spawn_actor(bp, p) for p in pts[args.spawn:] + pts[:args.spawn]) if a)
            car.set_simulate_physics(False)
            if bp.has_attribute("number_of_wheels"):
                car.set_light_state(carla.VehicleLightState.NONE)
            entry = cockpit.entry_for(bpid, client.get_server_version())
            for tname in args.times:
                w = world.get_weather()
                w.sun_altitude_angle, w.cloudiness, w.precipitation, w.fog_density = TIMES[tname], 20.0, 0.0, 0.0
                world.set_weather(w)
                if TIMES[tname] < 3:
                    car.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LowBeam |
                                                                carla.VehicleLightState.Position))
                for seat in args.seats:
                    old = compose(shoot("rgb", OLD[seat]), seat, "overlay", None, 0.0)
                    eye = cockpit.actor_eye(car, seat)
                    cab = cockpit.CabinView(entry, eye[:3], eye[3], size, args.fov)
                    cab.set_depth(shoot("depth", eye, ticks=3))
                    bias = cockpit.cabin_ev_bias(entry, sun_altitude=TIMES[tname])
                    new = compose(shoot("rgb", eye, bias), seat, "dashboard", cab, args.steer)
                    im = Image.new("RGB", (size[0] * 2 + 8, size[1]), (255, 255, 255))
                    im.paste(Image.fromarray(old), (0, 0))
                    im.paste(Image.fromarray(new), (size[0] + 8, 0))
                    d = ImageDraw.Draw(im)
                    d.text((10, 8), f"before: {bpid} {seat} {tname}", fill=(255, 255, 0))
                    d.text((size[0] + 18, 8), f"after: eye {tuple(round(v, 2) for v in eye[:3])}, "
                                              f"wheel {'turned' if cab.spin else 'not found'}", fill=(255, 255, 0))
                    name = f"{bpid.split('.', 1)[1].replace('.', '_')}_{seat}_{tname}.jpg"
                    im.save(out / name, quality=86)
                    print(out / name, flush=True)
    finally:
        if car is not None:
            car.destroy()
        world.apply_settings(orig)


if __name__ == "__main__":
    main()
