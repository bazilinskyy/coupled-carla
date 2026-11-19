"""Do all the register lamps light the street at night, and what does it cost? (MegaLights, CARLA 1.0)

CARLA 1.0 enables MegaLights for the project (Config/DefaultEngine.ini r.MegaLights.EnableForProject=True): many
shadowed lights at a cost that grows with the screen, not with the light count. coupled-carla spawns the lamps of a
map (scenery.json: the city's lamp register in Eindhoven) as CARLA's lit street-light blueprints within
``lights_radius_m`` of the participant at night (season.py, default 200 m), each with its register flux and colour
(nightlights.py; needs the CarlaLight SetLightState patch). This check, per lamp radius:

  1. census: lamps placed, lit lamps (light blueprints), the light manager's street lights and how many are on,
     how many of them got their register flux (intensity set) and colour;
  2. evidence: two camera frames down the road at the same spot, street lights off and on (fixed exposure), and
     the mean brightness of the road in each: lamps that really light the street make "on" clearly brighter;
  3. cost: frame_budget.measure_case at night with that radius (server and camera fps, PASS/FAIL against 60).

    python tools/megalights_check.py --scenery maps/tue_campus_twin --level TUeCampus --radii 200,600,1000 \\
        --out ~/cc58/megalights/2026-11-17

Run it on win5090 in a GPU window only. For MegaLights off as a baseline, start the server with
-ExecCmds="r.MegaLights.Enable 0" (frame_budget.py --variants does this per variant).
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))


def log(msg):
    print(f"[megalights] {msg}", flush=True)


def brightness(img):
    import numpy as np
    a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, :3]
    lower = a[a.shape[0] // 2:]          # the road and the pavement, not the sky
    return float(lower.mean())


def census_and_evidence(client, world, args, radius, out_dir):
    import carla
    from coupledcarla import season, scenery
    w = world.get_weather()
    w.sun_altitude_angle = args.sun
    world.set_weather(w)
    cmap = world.get_map()
    sp = cmap.get_spawn_points()[args.spawn_index]
    seas = {"state": "leaf", "lights": season.light_blueprints(world, log=log), "lights_radius_m": radius}
    # scenery as in a trial (250 m around the camera); only the lamp LIGHT radius changes
    scen = scenery.Scenery(client, world, args.scenery, {"radius_m": max(250.0, min(radius, 1000.0) + 50.0)}, log=log,
                           season=seas)
    cam = None
    try:
        scen.update([sp.location])
        for _ in range(5):
            world.wait_for_tick(10.0)
        lm = world.get_lightmanager()
        street = lm.get_all_lights(carla.LightGroup.Street)
        res = {"radius_m": radius, "lamps_placed": sum(len(v) for v in scen.lamps.values()),
               "lamps_lit": sum(len(v) for k, v in scen.lamps.items() if k in scen.lit),
               "street_lights": len(street), "street_lights_on": sum(1 for li in street if li.is_on),
               "register_specs_applied": getattr(scen, "n_lamp_specs", None)}
        lib = world.get_blueprint_library()
        bp = lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", "1280")
        bp.set_attribute("image_size_y", "720")
        if bp.has_attribute("exposure_mode"):
            bp.set_attribute("exposure_mode", "manual")
        cam = world.spawn_actor(bp, carla.Transform(sp.location + carla.Location(z=2.0), sp.rotation))
        got = {}
        cam.listen(lambda im: got.__setitem__("img", im))

        def frame():
            got.pop("img", None)
            t0 = time.time()
            while "img" not in got and time.time() - t0 < 10:
                world.wait_for_tick(5.0)
            for _ in range(3):           # let the frame after the change settle
                world.wait_for_tick(5.0)
            got.pop("img", None)
            while "img" not in got and time.time() - t0 < 20:
                world.wait_for_tick(5.0)
            return got.get("img")
        lm.turn_off(street)
        off = frame()
        lm.turn_on(street)
        on = frame()
        if off is not None and on is not None:
            res["brightness_off"] = round(brightness(off), 1)
            res["brightness_on"] = round(brightness(on), 1)
            res["lit_gain"] = round(res["brightness_on"] / max(res["brightness_off"], 0.5), 2)
            off.save_to_disk(str(out_dir / f"lamps_r{int(radius)}_off.png"))
            on.save_to_disk(str(out_dir / f"lamps_r{int(radius)}_on.png"))
        return res
    finally:
        if cam is not None:
            cam.stop()
            cam.destroy()
        scen.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--level", default=None)
    ap.add_argument("--scenery", required=True, help="coupled-carla map folder with scenery.json")
    ap.add_argument("--radii", default="200,600,1000", help="lamp light radii to try (m; the scenery radius follows, "
                                                         "up to 1 km)")
    ap.add_argument("--sun", type=float, default=-30.0)
    ap.add_argument("--spawn-index", type=int, default=0)
    ap.add_argument("--no-fps", action="store_true", help="census and evidence only")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    import os
    os.environ.setdefault("COUPLED_CARLA_MUTE", "1")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    import carla
    import frame_budget as fb
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(180.0)
    world = client.load_world(args.level) if args.level else client.get_world()
    st = world.get_settings()
    st.synchronous_mode = False
    st.fixed_delta_seconds = None
    world.apply_settings(st)
    rows = []
    for r in [float(x) for x in args.radii.split(",") if x]:
        res = census_and_evidence(client, world, args, r, out_dir)
        if not args.no_fps:
            fb.CASES[f"night_r{int(r)}"] = (args.sun, r)
            fargs = argparse.Namespace(start=None, spawn_index=args.spawn_index, speed=8.3, warm_s=15.0,
                                       measure_s=30.0, vehicle="vehicle.lincoln.mkz", cam="1920x1080", cams=1,
                                       fov=90.0, profile="", target=60.0, vr=False, stills=True,
                                       scenery=args.scenery, scenery_radius=250.0, cam_specs="", warmup_s=0.0)
            res["fps"] = fb.measure_case(client, world, fargs, f"night_r{int(r)}", out_dir, f"night_r{int(r)}")
        log(json.dumps(res))
        rows.append(res)
        (out_dir / "megalights.json").write_text(json.dumps(rows, indent=1))
    lines = ["| radius m | lamps placed | lit | street lights on | register flux set | brightness off -> on | "
             "server fps | camera fps | verdict |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        f = r.get("fps") or {}
        lines.append(f"| {int(r['radius_m'])} | {r['lamps_placed']} | {r['lamps_lit']} | "
                     f"{r['street_lights_on']}/{r['street_lights']} | {r.get('register_specs_applied')} | "
                     f"{r.get('brightness_off')} -> {r.get('brightness_on')} (x{r.get('lit_gain')}) | "
                     f"{f.get('server_fps', '')} | {', '.join(map(str, f.get('camera_fps', [])))} | "
                     f"{'' if not f else ('PASS' if f.get('pass') else 'FAIL')} |")
    (out_dir / "megalights.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
