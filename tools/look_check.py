"""Render one view with several camera variants at the same frame and measure them (coupledcarla/look.py's numbers).

    python tools/look_check.py experiments/eindhoven_centre.yaml --set rate --out /tmp/look --weather ClearNoon
    python tools/look_check.py experiments/eindhoven_twin.yaml --variants profiles.json --out /tmp/look

A car drives along the lane at --lat/--lon (default: the first participant's spawn) at --speed with every camera
on the chase mount of display.py (or --fixed x,y,z,yaw,pitch: a fixed camera). Variants are {name: {attribute:
value, "_ss": supersample factor, "_tick": sensor_tick s}} (--variants FILE) or a built-in --set: blur, rate, tone
(CARLA 0.9.x) or profiles (CARLA 1.0 post_process_profile). For each variant it prints, on the frame all cameras
share: sharpness (variance of the Laplacian; also without the sky), clipped white/black %, mean and contrast
(standard deviation) of the luminance, mean luminance of roads/sidewalks/buildings/vegetation/sky (semantic camera),
PSNR against --ref, and saves NAME.jpg. Uses the CARLA server at the experiment's server.host/port.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import carla
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import config, look, parking, run  # noqa: E402
from coupledcarla.display import VIEWS  # noqa: E402
from coupledcarla.geo import Geo, heading_to_yaw  # noqa: E402
from coupledcarla.scenery import Scenery  # noqa: E402

TAGS = {"road": 1, "sidewalk": 2, "building": 3, "vegetation": 9, "sky": 11}
EV11 = {"exposure_mode": "manual", "shutter_speed": f"{look.shutter_for_ev(11):.1f}"}
SETS = {
    "blur": {"default": {}, "default_1hz": {"_tick": 1.0, **EV11}, "noblur": {"motion_blur_intensity": 0},
             "noblur_1hz": {"_tick": 1.0, "motion_blur_intensity": 0, **EV11}},
    "rate": {"ref_ss2": {"_ss": 2, "motion_blur_intensity": 0, **EV11},
             "every": {"motion_blur_intensity": 0, **EV11},
             "hz10": {"_tick": 0.1, "motion_blur_intensity": 0, **EV11},
             "hz1": {"_tick": 1.0, "motion_blur_intensity": 0, **EV11},
             "ss2_hz10": {"_ss": 2, "_tick": 0.1, "motion_blur_intensity": 0, **EV11}},
    "tone": {"bloom0675": {**EV11}, "bloom03": {"bloom_intensity": 0.3, "lens_flare_intensity": 0, **EV11},
             "bloom0": {"bloom_intensity": 0, "lens_flare_intensity": 0, **EV11},
             "toe04": {"bloom_intensity": 0.3, "toe": 0.4, **EV11},
             "slope1": {"bloom_intensity": 0.3, "slope": 1.0, **EV11}},
    "profiles": {p: {"post_process_profile": p} for p in ("Dashcam", "CoupledClean", "Smartphone",
                                                          "AutomotiveHDRClean", "Cinematic")},
}
REFS = {"rate": "ref_ss2"}


def keep(q, img, n):
    q[img.frame] = img
    for k in sorted(q)[:-n]:
        q.pop(k, None)


def lum(rgb):
    a = rgb.astype(np.float32)
    return 0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]


def laplacian(g):
    k = np.zeros_like(g)
    k[1:-1, 1:-1] = g[1:-1, :-2] + g[1:-1, 2:] + g[:-2, 1:-1] + g[2:, 1:-1] - 4 * g[1:-1, 1:-1]
    return k


def metrics(rgb, sem=None):
    g = lum(rgb)
    k = laplacian(g)
    mx = rgb.max(-1)
    out = {"sharp": round(float(k[1:-1, 1:-1].var()), 1), "white%": round(100 * float((mx >= 250).mean()), 2),
           "black%": round(100 * float((mx <= 5).mean()), 2), "mean": round(float(g.mean()), 1),
           "contrast": round(float(g.std()), 1)}
    if sem is not None:
        ns = sem != TAGS["sky"]
        out["sharp_ground"] = round(float(k[ns].var()), 1)
        out["median_ground"] = round(float(np.median(g[ns])), 1)
        for name, tag in TAGS.items():
            m = sem == tag
            if m.mean() > 0.005:
                out[name] = round(float(g[m].mean()), 1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--set", choices=sorted(SETS), help="a built-in set of variants")
    ap.add_argument("--variants", help="json file: {name: {attribute: value, _ss, _tick}}")
    ap.add_argument("--ref", help="variant to compute PSNR against")
    ap.add_argument("--out", required=True)
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--heading", type=float, help="compass heading of the lane to use")
    ap.add_argument("--fixed", help="x,y,z,yaw,pitch (CARLA): a fixed camera instead of the car")
    ap.add_argument("--weather", default="CloudyNoon", help="a carla.WeatherParameters preset or a json dict")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--speed", type=float, default=8.0)
    ap.add_argument("--settle", type=int, default=150)
    ap.add_argument("--move", type=int, default=160)
    ap.add_argument("--load", action="store_true", help="load the experiment's world first (else: as loaded)")
    a = ap.parse_args()
    variants = json.loads(Path(a.variants).read_text()) if a.variants else SETS[a.set or "blur"]
    ref = a.ref or REFS.get(a.set)
    cfg = config.load(a.experiment)
    mdir = config.map_dir(cfg)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    W, H = (int(v) for v in a.size.split("x"))
    client = carla.Client(cfg["server"]["host"], cfg["server"]["port"])
    client.set_timeout(120.0)
    world = run.load_world(client, mdir / "map.xodr", cfg["map"], slab=True) if a.load else client.get_world()
    if a.weather.startswith("{"):
        w = world.get_weather()
        for k, v in json.loads(a.weather).items():
            setattr(w, k, v)
    else:
        w = getattr(carla.WeatherParameters, a.weather)
    world.set_weather(w)
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, 0.02
    world.apply_settings(settings)
    actors = []
    try:
        cmap = world.get_map()
        lib = world.get_blueprint_library()
        geo = Geo(mdir / "geo.net.xml")
        sp = (cfg["participants"][0].get("spawn") or {}) if cfg.get("participants") else {}
        lat, lon = a.lat or sp.get("lat"), a.lon or sp.get("lon")
        rig, wp, mount = None, None, VIEWS["chase"]
        if a.fixed:
            fx, fy, fz, fyaw, fp = (float(v) for v in a.fixed.split(","))
            mount = carla.Transform(carla.Location(x=fx, y=fy, z=fz), carla.Rotation(yaw=fyaw, pitch=fp))
            focus = mount.location
        else:
            x, y = geo.to_carla(lat, lon)
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
            if a.heading is not None:
                want = heading_to_yaw(a.heading)
                for cand in (wp, wp.get_left_lane(), wp.get_right_lane()):
                    if cand is not None and abs((cand.transform.rotation.yaw - want + 180) % 360 - 180) < 90:
                        wp = cand
                        break
            focus = wp.transform.location
        scen = Scenery(client, world, mdir, {"radius_m": 250.0}) if (mdir / "scenery.json").exists() else None
        parked = parking.scenery(client, world, mdir, cfg)
        for s in (scen, parked):
            if s is not None:
                s.update([focus])
        if wp is not None:
            back = wp.previous(a.speed * 0.02 * a.move + 0.1)
            start = back[0] if back else wp
            tf = start.transform
            tf.location.z += 0.3
            rig = world.spawn_actor(lib.find("vehicle.lincoln.mkz_2020"), tf)
            rig.set_simulate_physics(False)
            actors.append(rig)
        history = a.move if any(v.get("_tick") for v in variants.values()) else 2
        cams = {}
        for name, v in variants.items():
            bp = lib.find("sensor.camera.rgb")
            ss = float(v.get("_ss", 1))
            bp.set_attribute("image_size_x", str(int(W * ss)))
            bp.set_attribute("image_size_y", str(int(H * ss)))
            for k, val in v.items():
                if k == "_tick":
                    bp.set_attribute("sensor_tick", str(val))
                elif not k.startswith("_"):
                    if bp.has_attribute(k):
                        bp.set_attribute(k, str(val))
                    else:
                        print(f"{name}: this server's camera has no attribute {k}")
            cam = world.spawn_actor(bp, mount, attach_to=rig) if rig else world.spawn_actor(bp, mount)
            q = {}
            cam.listen(lambda img, q=q: keep(q, img, history))
            cams[name] = (cam, q)
            actors.append(cam)
        sbp = lib.find("sensor.camera.semantic_segmentation")
        sbp.set_attribute("image_size_x", str(W))
        sbp.set_attribute("image_size_y", str(H))
        sem = world.spawn_actor(sbp, mount, attach_to=rig) if rig else world.spawn_actor(sbp, mount)
        sq = {}
        sem.listen(lambda img: keep(sq, img, history))
        actors.append(sem)
        for _ in range(a.settle):
            world.tick()
        frames, cur = [], (start if rig else None)
        for _ in range(a.move):
            if rig:
                nxt = cur.next(a.speed * 0.02)
                cur = nxt[0] if nxt else cur
                tf = cur.transform
                tf.location.z += 0.05
                rig.set_transform(tf)
            frames.append(world.tick())
        time.sleep(1.0)
        common = set(frames) & set(sq)
        for _, q in cams.values():
            common &= set(q)
        if not common:
            print("no frame shared by every camera (raise --move)")
            return
        f = max(common)
        semimg = np.frombuffer(sq[f].raw_data, dtype=np.uint8).reshape(H, W, 4)[:, :, 2].copy()
        rows, imgs = {}, {}
        for name, (cam, q) in cams.items():
            rgb = look.downscale(np.ascontiguousarray(look.to_rgb(q[f])), (W, H))
            imgs[name] = rgb
            rows[name] = metrics(rgb, semimg)
            look.save_frame(rgb, out / f"{name}.jpg")
        if ref in imgs:
            r = imgs[ref].astype(np.float32)
            for name, rgb in imgs.items():
                mse = float(((rgb.astype(np.float32) - r) ** 2).mean())
                rows[name]["psnr_vs_" + ref] = round(10 * math.log10(255 ** 2 / mse), 2) if mse > 0 else None
        keys = list(dict.fromkeys(k for r in rows.values() for k in r))
        print("variant".ljust(16) + "".join(k[:13].rjust(14) for k in keys))
        for name, r in rows.items():
            print(name[:15].ljust(16) + "".join(str(r.get(k, "")).rjust(14) for k in keys))
        (out / "metrics.json").write_text(json.dumps({"frame": f, "weather": a.weather, "variants": variants,
                                                      "metrics": rows}, indent=1))
        for s in (scen, parked):
            if s is not None:
                s.destroy()
    finally:
        for x in reversed(actors):
            try:
                if hasattr(x, "stop"):
                    x.stop()
                x.destroy()
            except RuntimeError:
                pass
        settings.synchronous_mode, settings.fixed_delta_seconds = False, None
        world.apply_settings(settings)


if __name__ == "__main__":
    main()
