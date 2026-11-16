"""Profile Google's photo tiles on a CARLA 1.0 server: A/B phases in ONE level load (so one billed root request).

Spawns blueprint.cc_photoreal_google with prof and control on, a car with a chase camera (1280 x 720, like a trial's
view: chase) driven along the same stretch of road in every phase (set_transform, synchronous mode, 0.02 s steps),
and switches the plugin's settings between phases through photoreal_control.txt beside the server's usage ledger
(the plugin acknowledges each change in photoreal_control.ack). Per phase: the real-time factor seen here (wall time
per tick), and the server's own frame timings and tile counts from google_tiles_profile.csv (game, render, RHI
thread and GPU ms per frame); a stamped camera frame at the same spot of the route for the visual comparison.

    # wsl5090, in a GPU window, against the packaged server on win5090 (EhvCentre loaded or --load):
    python tools/photoreal_profile.py --host 127.0.0.1 --port 2000 --level EhvCentre \
        --config-dir /mnt/c/Users/20255250/.config/coupled-carla --out ~/cc58/tiles_prof

Phases (--phases, default all of PHASES): the twin alone (tiles hidden), tiles without a cut, the mask cut (2.8 m and
full height), Cesium's polygon overlay at its default and a coarse detail, the old setup (spectator following the
camera, the server window rendering), screen-space error 24 and 32, the game-thread load budget, fog culling off.
"""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import photoreal  # noqa: E402

# name -> control settings applied at the phase's start (on top of the phase before: each lists what it needs)
BASE = {"tiles": 1, "cut": "mask", "cut_top": 2.8, "sse": 16, "cams": 1, "player": "auto", "viewport": 0, "fog": 1,
        "load_ms": 5, "clip_sse": 2, "clip_tex": 2048}
PHASES = [
    ("twin_only", {"tiles": 0}),
    ("tiles_nocut", {"cut": "none"}),
    ("mask_2.8m", {}),
    ("mask_full", {"cut_top": 0}),
    ("polygons", {"cut": "polygons"}),
    ("polygons_coarse", {"cut": "polygons", "clip_sse": 16, "clip_tex": 512}),
    ("old_setup", {"cut": "polygons", "cams": 0, "player": 1, "viewport": 1, "_follow": True}),
    ("mask_viewport_on", {"viewport": 1}),
    ("mask_sse24", {"sse": 24}),
    ("mask_sse32", {"sse": 32}),
    ("mask_load2ms", {"load_ms": 2}),
    ("mask_nofog", {"fog": 0}),
]


def log(msg):
    print(f"[profile] {msg}", flush=True)


def route(cmap, start, metres, step):
    """Transforms every `step` m along the lanes from the waypoint nearest `start` (first branch at forks)."""
    import carla
    wp = cmap.get_waypoint(start, project_to_road=True, lane_type=carla.LaneType.Driving)
    out = []
    while wp is not None and len(out) * step < metres:
        out.append(wp.transform)
        nxt = wp.next(step)
        wp = nxt[0] if nxt else None
    return out


def control(cfgdir, settings, world, timeout_s=20.0):
    """Write the control file and tick until the plugin acknowledges it."""
    text = ";".join(f"{k}={v}" for k, v in settings.items()) + "\n"
    (cfgdir / "photoreal_control.txt").write_text(text)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        world.tick()
        try:
            if (cfgdir / "photoreal_control.ack").read_text() == text:
                return True
        except OSError:
            pass
    log(f"no acknowledgement for {text.strip()} (is control=1 on the actor?)")
    return False


def main(argv=None):
    import carla
    import numpy as np
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--level", default="EhvCentre")
    ap.add_argument("--load", action="store_true", help="load_world(--level) first (a level load)")
    ap.add_argument("--config-dir", required=True, help="the SERVER's coupled-carla config folder (ledger, profile)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--start", default="-188.3,-171.5,17.0", help="x,y,z of the route's start (CARLA coordinates)")
    ap.add_argument("--speed", type=float, default=13.9, help="m/s along the route")
    ap.add_argument("--warm-s", type=float, default=60.0, help="simulated seconds of the first, loading drive")
    ap.add_argument("--phase-s", type=float, default=20.0, help="simulated seconds measured per phase")
    ap.add_argument("--settle-s", type=float, default=4.0, help="simulated seconds after a change before measuring")
    ap.add_argument("--phases", default="", help="comma-separated phase names (default all)")
    ap.add_argument("--options", default="", help="extra actor options k=v;k=v (e.g. mat=cesium)")
    ap.add_argument("--yes", action="store_true", help="really spawn the tiles (ONE billed root request)")
    a = ap.parse_args(argv)
    if not a.yes:
        print("spawning the tiles sends one billed Google root request; add --yes")
        return 1
    cfgdir, out = Path(a.config_dir).expanduser(), Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(a.host, a.port)
    client.set_timeout(120.0)
    world = client.load_world(a.level) if a.load else client.get_world()
    step = 0.02
    settings = world.get_settings()
    was = (settings.synchronous_mode, settings.fixed_delta_seconds)
    settings.synchronous_mode, settings.fixed_delta_seconds = True, step
    world.apply_settings(settings)
    actors = []
    try:
        x, y, z = (float(v) for v in a.start.split(","))
        path = route(world.get_map(), carla.Location(x, y, z), a.speed * (a.warm_s + 5), a.speed * step)
        log(f"route of {len(path)} steps ({len(path) * a.speed * step:.0f} m)")
        lib = world.get_blueprint_library()
        car_bp = next(iter(lib.filter("vehicle.lincoln.mkz*")), None) or lib.filter("vehicle.*")[0]
        car = world.spawn_actor(car_bp, path[0])
        car.set_simulate_physics(False)
        actors.append(car)
        cam_bp = lib.find("sensor.camera.rgb")
        for k, v in (("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "90")):
            cam_bp.set_attribute(k, v)
        cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=-6.5, z=2.6), carla.Rotation(pitch=-8)),
                                attach_to=car)
        actors.append(cam)
        last = {}
        cam.listen(lambda img: last.__setitem__("img", img))
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z + 60), carla.Rotation(pitch=90)))

        s = photoreal.settings({"photoreal": {"provider": "google", "profile_s": 2, "control": True}})
        opts = photoreal.role_options(s, night=False, offscreen=True)
        if a.options:
            opts += ";" + a.options
        bp = lib.find(photoreal.BLUEPRINT)
        bp.set_attribute("role_name", opts)
        (cfgdir / "photoreal_control.txt").write_text("")
        tiles = world.spawn_actor(bp, carla.Transform())
        actors.append(tiles)
        log(f"tiles spawned (ONE root request): {opts}")
        tag = time.strftime("%H%M%S")

        def drive(i0, n, follow=False):
            t0 = time.time()
            for i in range(n):
                tf = path[min(len(path) - 1, i0 + i)]
                car.set_transform(tf)
                if follow and i % 5 == 0:
                    fwd = tf.get_forward_vector()
                    spectator.set_transform(carla.Transform(tf.location - fwd * 8.0 + carla.Location(z=3.0),
                                                            tf.rotation))
                world.tick()
            return time.time() - t0

        cur = dict(BASE)
        control(cfgdir, dict(cur, phase=f"{tag}:warm"), world)
        wall = drive(0, int(a.warm_s / step))
        log(f"warm drive: RTF {a.warm_s / wall:.2f}")
        want = [p for p in PHASES if not a.phases or p[0] in a.phases.split(",")]
        n = int(a.phase_s / step)
        results = []
        for name, change in want:
            follow = bool(change.get("_follow"))
            cur = dict(BASE, **{k: v for k, v in change.items() if not k.startswith("_")})
            control(cfgdir, dict(cur, phase=f"{tag}:{name}:settle"), world)
            if not follow:
                spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z + 60), carla.Rotation(pitch=90)))
            drive(0, int(a.settle_s / step), follow)
            control(cfgdir, dict(cur, phase=f"{tag}:{name}"), world)
            wall = drive(0, n, follow)
            img = last.get("img")
            if img is not None:
                rgb = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, 2::-1]
                from PIL import Image
                Image.fromarray(photoreal.stamp(rgb, photoreal.attribution_text(s))).save(out / f"{tag}_{name}.jpg",
                                                                                          quality=90)
            results.append({"phase": name, "rtf": round(a.phase_s / wall, 3), "ms_per_tick": round(1000 * wall / n, 1),
                            "settings": cur})
            log(f"{name}: RTF {a.phase_s / wall:.2f} ({1000 * wall / n:.1f} ms/tick)")
        control(cfgdir, dict(BASE, phase=f"{tag}:end"), world)
        world.tick()
        # the server's rows of this session
        rows = []
        try:
            with open(cfgdir / "google_tiles_profile.csv", newline="") as f:
                rows = [r for r in csv.DictReader(f) if r["phase"].startswith(tag + ":")]
        except OSError:
            log("no google_tiles_profile.csv from the server")
        for r in results:
            mine = [row for row in rows if row["phase"] == f"{tag}:{r['phase']}"]
            if mine:
                for k in ("frame_ms", "game_ms", "render_ms", "rhi_ms", "gpu_ms", "tiles_visible", "tiles_loaded",
                          "tile_mb", "max_frame_ms"):
                    vals = [float(m[k]) for m in mine]
                    r[k] = round(sum(vals) / len(vals), 2)
        (out / f"{tag}_profile.json").write_text(json.dumps(results, indent=1))
        cols = ["phase", "rtf", "ms_per_tick", "frame_ms", "game_ms", "render_ms", "rhi_ms", "gpu_ms", "tiles_visible",
                "tile_mb"]
        lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
        lines += ["| " + " | ".join(str(r.get(c, "")) for c in cols) + " |" for r in results]
        (out / f"{tag}_profile.md").write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0
    finally:
        for act in reversed(actors):
            try:
                if act.type_id.startswith("sensor."):
                    act.stop()
                act.destroy()
            except RuntimeError:
                pass
        settings.synchronous_mode, settings.fixed_delta_seconds = was
        world.apply_settings(settings)


if __name__ == "__main__":
    sys.exit(main())
