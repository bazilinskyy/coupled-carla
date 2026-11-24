"""Render views of a map with all its real-world scenery and parked cars, for the README.

    python tools/scenery_shots.py experiments/cities/lyon.yaml --time "2026-11-13 10:00" \
        --view bellecour:45.7578,4.8320,80,270,-15 --out docs/img/
    python tools/scenery_shots.py experiments/eindhoven_centre.yaml --weather doc --size 1280x720 \
        --chase parked_eindhoven:51.4417,5.4754,200 --name "{view}" --out docs/img/

Each --view is name:lat,lon,height above the road,compass heading,pitch[,fov]; each --chase is name:lat,lon,compass
heading: a car on the nearest driving lane there, seen by the chase camera of display.py. Images are taken with
simulation.capture (coupledcarla/look.py): rendered at 2x and downscaled, no motion blur, bloom 0.3, and a manual
exposure chosen per view among EVs around look.exposure_ev (the one whose median ground luminance is nearest
look.target_luminance). --weather doc is look.DOC_WEATHER with the sun's azimuth of --time. --before DIR also saves
the view with the cameras as they were (1x, CARLA's default post-processing, auto exposure) for comparison.
Night and rain (nightlights.py): --ahead N puts N cars in the lane ahead of a --chase car (every 10 m), and every car
there drives with the lights the rules ask for (--lights rules: dipped beam and rear lights at night, in fog and in
rain from 2.5 mm/h) or as before nightlights.py (--lights old: the participant's car dipped beam and rear lights at
night only, SUMO's cars dipped beam only, with dark rear lights). --brake lights the first car's brake lights.
--scenery-json FILE spawns that scenery.json instead of the map's (e.g. one built before a change).
A shot the realism guard rejects (look.realism_problems: an empty world by the semantic camera, too dark or bright,
clipped, glaring) is not saved; the reason is printed (--force saves it anyway).
Starts its own CARLA server (or uses the one at the experiment's server port).
"""
import argparse
import json
import sys
from pathlib import Path

import carla
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, look, nightlights, parking  # noqa: E402
from coupledcarla.display import VIEWS  # noqa: E402
from coupledcarla.geo import Geo, ground, heading_to_yaw  # noqa: E402
from coupledcarla.run import load_world, set_weather  # noqa: E402
from coupledcarla.scenery import Scenery  # noqa: E402
from coupledcarla.server import Server  # noqa: E402
from coupledcarla.compat import left_lane, right_lane  # noqa: E402

SKY = 11      # CARLA 0.9.14+ semantic tag of the sky
AHEAD = ["vehicle.audi.a2", "vehicle.toyota.prius", "vehicle.volkswagen.t2_2021", "vehicle.mini.cooper_s_2021",
         "vehicle.nissan.patrol_2021", "vehicle.citroen.c3"]
AHEAD_COLOURS = ["120,124,128", "30,40,70", "180,180,176", "110,20,24", "40,42,44", "200,200,196"]


def keep(q, img):
    q[img.frame] = img
    for k in sorted(q)[:-8]:
        q.pop(k, None)


def median_ground(rgb, sem):
    lum = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
    m = sem != SKY
    return float(np.median(lum[m])) if m.any() else float(np.median(lum))


def shoot(world, tf, size, fov, cap, weather, parent=None, before=False, settle=100):
    """Render tf with a few exposures (and the old camera when before): {name: rgb}, the picked image's name, the
    best EV and the median ground luminance of each exposure."""
    lib = world.get_blueprint_library()
    rsize = look.capture_size(size, cap)
    base_ev = look.exposure_ev(weather.sun_altitude_angle, weather.cloudiness) + float(cap.get("ev_bias") or 0)
    cams = {}

    def spawn(name, bp):
        cam = world.spawn_actor(bp, tf, attach_to=parent) if parent else world.spawn_actor(bp, tf)
        q = {}
        cam.listen(lambda img, q=q: keep(q, img))
        cams[name] = (cam, q)
    for dev in (-1.0, -0.5, 0.0, 0.5, 1.0, 1.5):    # aerial views want up to +1.2 EV
        bp = lib.find("sensor.camera.rgb")
        for k, v in (("image_size_x", rsize[0]), ("image_size_y", rsize[1]), ("fov", fov)):
            bp.set_attribute(k, str(v))
        look.apply_09(bp, {**cap, "ev_bias": float(cap.get("ev_bias") or 0) + dev}, weather)
        spawn(f"ev{base_ev + dev:.2f}", bp)
    if before:
        bp = lib.find("sensor.camera.rgb")
        for k, v in (("image_size_x", size[0]), ("image_size_y", size[1]), ("fov", fov)):
            bp.set_attribute(k, str(v))
        spawn("before", bp)
    sbp = lib.find("sensor.camera.semantic_segmentation")
    for k, v in (("image_size_x", size[0]), ("image_size_y", size[1]), ("fov", fov)):
        sbp.set_attribute(k, str(v))
    spawn("_sem", sbp)
    frame = None
    for _ in range(settle):          # meshes stream in, temporal anti-aliasing and shadows settle
        frame = world.tick()
    for _ in range(50):
        if all(frame in q for _, q in cams.values()):
            break
        frame = world.tick()
    import time
    time.sleep(0.5)
    common = set.intersection(*(set(q) for _, q in cams.values()))
    if frame not in common and common:      # a camera was late with the last tick's image: the newest all have
        frame = max(common)
    sem = np.frombuffer(cams["_sem"][1][frame].raw_data, dtype=np.uint8).reshape(size[1], size[0], 4)[:, :, 2].copy()
    out, meds = {}, {}
    for name, (cam, q) in cams.items():
        if name == "_sem":
            continue
        out[name] = np.array(look.downscale(look.to_rgb(q[frame]), size))     # a copy: the image buffer goes
        if name.startswith("ev"):
            meds[name] = round(median_ground(out[name], sem), 1)
    for cam, q in cams.values():
        cam.stop()
        cam.destroy()
    ev = look.best_ev([(float(n[2:]), m, weather.sun_altitude_angle) for n, m in meds.items()])
    pick = min(meds, key=lambda n: abs(float(n[2:]) - ev))
    return out, pick, ev, meds, sem


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--time", default="12:00")
    ap.add_argument("--weather", default="real", help="real | doc | a carla.WeatherParameters preset")
    ap.add_argument("--sun", type=float, default=None, help="sun altitude (deg), overrides the weather's")
    ap.add_argument("--view", action="append", default=[])
    ap.add_argument("--chase", action="append", default=[])
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--name", default="scenery_{exp}_{view}", help="file name pattern ({exp}, {view})")
    ap.add_argument("--size", default="1600x900")
    ap.add_argument("--before", default=None, help="also save the old camera's image here")
    ap.add_argument("--no-parked", action="store_true")
    ap.add_argument("--quality", type=int, default=88)
    ap.add_argument("--force", action="store_true", help="save shots the realism guard rejects (look.py)")
    ap.add_argument("--ahead", type=int, default=0, help="cars in the lane ahead of a --chase car")
    ap.add_argument("--lights", choices=["rules", "old"], default="rules", help="the cars' lights (see above)")
    ap.add_argument("--brake", action="store_true", help="the first car ahead brakes (brake lights)")
    ap.add_argument("--scenery-json", help="spawn this scenery.json instead of the map's")
    args = ap.parse_args()
    cfg = config.load(args.experiment)
    mdir = config.map_dir(cfg)
    geo = Geo(mdir / "geo.net.xml")
    cap = look.settings(cfg["simulation"])
    cap["enabled"] = True
    server = Server(cfg["server"])
    client = server.start()
    try:
        world = load_world(client, mdir / "map.xodr", cfg["map"], slab=True)
        sc = dict(cfg["simulation"], time=args.time)
        if args.weather == "doc":       # its own clouds and haze, the sun's azimuth at that time, 40 deg up
            sc.update(weather=dict(look.DOC_WEATHER), sun_altitude=look.DOC_WEATHER["sun_altitude_angle"])
        else:
            sc["weather"] = args.weather
        if args.sun is not None:
            sc["sun_altitude"] = args.sun
        # a twin experiment names no bbox (its OpenDRIVE sets the box): the map build wrote it to meta.json
        bb = cfg["map"].get("bbox") or json.loads((mdir / "meta.json").read_text())["bbox"]
        wx = set_weather(world, sc, ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2), mdir / "real")
        world.wait_for_tick()       # get_weather() returns the new weather only after a tick
        weather = world.get_weather()
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds = True, 0.05
        world.apply_settings(settings)
        world.get_spectator().set_transform(carla.Transform(carla.Location(z=3000), carla.Rotation(pitch=90)))
        cmap = world.get_map()
        shots = []
        for v in args.view:
            name, spec = v.split(":")
            vals = [float(x) for x in spec.split(",")]
            lat, lon, up, heading, pitch = vals[:5]
            x, y = geo.to_carla(lat, lon)
            loc = ground(cmap, x, y) + carla.Location(z=up)
            shots.append((name, carla.Transform(loc, carla.Rotation(yaw=heading_to_yaw(heading), pitch=pitch)),
                          vals[5] if len(vals) > 5 else 80, None, loc))
        for v in args.chase:
            name, spec = v.split(":")
            lat, lon, heading = (float(x) for x in spec.split(","))
            x, y = geo.to_carla(lat, lon)
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
            want = heading_to_yaw(heading)
            for cand in (wp, left_lane(wp), right_lane(wp)):
                if cand is not None and cand.lane_type == carla.LaneType.Driving and \
                        abs((cand.transform.rotation.yaw - want + 180) % 360 - 180) < 90:
                    wp = cand
                    break
            shots.append((name, VIEWS["chase"], 90, wp, wp.transform.location))
        data = json.loads(Path(args.scenery_json).read_text()) if args.scenery_json else None
        scen = Scenery(client, world, mdir, {"radius_m": 0.0}, data=data)   # all of it: aerial views see the whole map
        LS = carla.VehicleLightState
        if args.lights == "rules":      # every car as nightlights.vehicle_lights says
            own = other = nightlights.light_bits(wx.get("lights") or {}, carla)
        else:                           # before nightlights.py: night only; SUMO cars without rear lights
            own = int(LS.LowBeam) | int(LS.Position) if wx["night"] else 0
            other = int(LS.LowBeam) if wx["night"] else 0
        print(f"lights ({args.lights}): own car {own}, other cars {other} ({(wx.get('lights') or {}).get('why')})")
        parked = None if args.no_parked else parking.scenery(client, world, mdir, cfg)
        lib = world.get_blueprint_library()
        for name, tf, fov, wp, at in shots:
            scen.update([at])
            if parked is not None:
                parked.update([at])
            car = None
            if wp is not None:
                vbp = lib.find("vehicle.lincoln.mkz_2020")
                if vbp.has_attribute("color"):
                    vbp.set_attribute("color", "52,58,66")
                t = wp.transform
                t.location.z += 0.3
                car = world.spawn_actor(vbp, t)
                cars = [car]
                for k in range(args.ahead):     # cars in the lane ahead, with their lights
                    nxt = wp.next(10.0 * (k + 1))
                    if not nxt:
                        break
                    bp2 = lib.find(AHEAD[k % len(AHEAD)])
                    if bp2.has_attribute("color"):
                        bp2.set_attribute("color", AHEAD_COLOURS[k % len(AHEAD_COLOURS)])
                    t2 = nxt[0].transform
                    t2.location.z += 0.3
                    other_car = world.try_spawn_actor(bp2, t2)
                    if other_car is not None:
                        bits = other | (int(LS.Brake) if args.brake and k == 0 else 0)
                        other_car.set_light_state(LS(bits))
                        cars.append(other_car)
                car.set_light_state(LS(own))
                for _ in range(40):         # let them settle on their wheels
                    world.tick()
                for c in cars:
                    c.set_simulate_physics(False)
            w, h = (int(v) for v in args.size.split("x"))
            imgs, pick, ev, meds, sem = shoot(world, tf, (w, h), fov, cap, weather, parent=car, before=bool(args.before))
            from PIL import Image
            stem = args.name.format(exp=cfg["name"], view=name)
            path = Path(args.out) / f"{stem}.jpg"
            path.parent.mkdir(parents=True, exist_ok=True)
            probs = look.realism_problems(imgs[pick], sem)      # never an empty world, a black or a glaring frame
            if probs and not args.force:
                print(f"REJECTED {path}: {'; '.join(probs)} (--force saves it anyway)", flush=True)
                if car is not None:
                    for c in cars:
                        c.destroy()
                continue
            Image.fromarray(np.ascontiguousarray(imgs[pick])).save(path, quality=args.quality)
            if args.before:
                Path(args.before).mkdir(parents=True, exist_ok=True)
                Image.fromarray(np.ascontiguousarray(imgs["before"])).save(Path(args.before) / f"{stem}.jpg",
                                                                            quality=args.quality)
            print(f"{path} ({path.stat().st_size // 1024} KB): EV {pick[2:]} (best {ev:.2f}; median ground by EV "
                  f"{meds}; sun {weather.sun_altitude_angle:.1f} deg, clouds {weather.cloudiness:.0f})", flush=True)
            if car is not None:
                for c in cars:
                    c.destroy()
        scen.destroy()
        if parked is not None:
            parked.destroy()
        settings.synchronous_mode, settings.fixed_delta_seconds = False, None
        world.apply_settings(settings)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
