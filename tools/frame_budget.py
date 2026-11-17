"""Frame budget of a CARLA 1.0 twin: does the participant's view hold 60 fps (90 for VR)?

Per server variant (launch options: quality level, MegaLights on/off, hardware ray tracing...), per level (a twin
and, for a before/after, the same twin baked with a change, e.g. TUeCampus and TUeCampusWin) and per case (day,
night, night with lamp lights within R metres), it drives a car along the same stretch of road, with the
participant's camera(s) on it (default one 1920 x 1080 RGB camera, Epic; --cams 2 --cam 2064x2208 for a VR
headset's two eyes), the map's scenery (trees, lamps, furniture: coupled-carla's scenery.py, from --scenery
MAPDIR) spawned around it as in a trial, and measures in asynchronous mode (the server as fast as it can):

  server fps   frames per wall second (world snapshots), and the 1 % low (99th percentile frame time)
  camera fps   frames each camera delivered per wall second

and writes a JSON and a markdown table with PASS/FAIL against --target (60) and, with --vr, 90 per eye. One still
per case is saved (a realistic frame of what was measured). Run it only in a GPU window (win5090 evenings): other
GPU work on the machine makes the numbers meaningless.

    # wsl5090 against the packaged server on win5090, the server restarted per variant:
    python tools/frame_budget.py --host 127.0.0.1 --port 2000 --levels TUeCampus,TUeCampusWin \\
        --scenery maps/tue_campus_twin --cases day,night,night_r600 --out ~/cc58/frame_budget/2026-11-17 \\
        --server-cmd "powershell.exe -File C:\\\\cc58\\\\frame_budget_server.ps1 {extra}" \\
        --variants "epic=;epic_nomegalights=-ExecCmds=\\"r.MegaLights.Enable 0\\""

Without --server-cmd the running server is measured as it is (one variant, "current").
"""
import argparse
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

CASES = {   # name -> (sun altitude, lamp lights radius m or None for the season default, or 0 = none)
    "day": (45.0, 0.0),
    "dusk": (2.0, None),
    "night": (-30.0, None),
    "night_r600": (-30.0, 600.0),
    "night_all": (-30.0, 1e6),
}
DEFAULT_VARIANTS = {"current": ""}


def log(msg):
    print(f"[frame_budget] {msg}", flush=True)


def parse_variants(text):
    out = {}
    for part in [p for p in (text or "").split(";") if p.strip()]:
        k, _, v = part.partition("=")
        out[k.strip()] = v.strip()
    return out or dict(DEFAULT_VARIANTS)


def wait_port(host, port, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            with socket.create_connection((host, port), timeout=3):
                return True
        except OSError:
            time.sleep(3)
    return False


def percentile(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def summarise(frame_dts, cam_counts, wall_s, target, vr):
    """Numbers and the verdict of one measurement (frame_dts: server frame times in s, from the snapshots)."""
    fps = len(frame_dts) / wall_s if wall_s > 0 else 0.0
    p99 = percentile(frame_dts, 0.99)
    cams = [n / wall_s for n in cam_counts] if wall_s > 0 else []
    need = 90.0 if vr else target
    worst = min([fps] + cams) if cams else fps
    return {"server_fps": round(fps, 1), "low_1pct_fps": round(1.0 / p99, 1) if p99 else None,
            "frame_ms_mean": round(1000 * sum(frame_dts) / len(frame_dts), 2) if frame_dts else None,
            "camera_fps": [round(c, 1) for c in cams], "need_fps": need, "pass": worst >= need}


def route(cmap, start, metres, step=1.0):
    import carla
    wp = cmap.get_waypoint(start, project_to_road=True, lane_type=carla.LaneType.Driving)
    out = []
    while wp is not None and len(out) * step < metres:
        out.append(wp.transform)
        nxt = wp.next(step)
        wp = nxt[0] if nxt else None
    return out


def measure_case(client, world, args, case, out_dir, tag):
    """Drive the route with the participant's camera(s) on and measure (asynchronous mode). Returns a dict."""
    import carla
    from coupledcarla import season, scenery
    sun, radius = CASES[case]
    st = world.get_settings()
    st.synchronous_mode = False
    st.fixed_delta_seconds = None
    world.apply_settings(st)
    w = world.get_weather()
    w.sun_altitude_angle = sun
    world.set_weather(w)
    cmap = world.get_map()
    if args.start:
        x, y, z = (float(v) for v in args.start.split(","))
        start = carla.Location(x, y, z)
    else:
        start = cmap.get_spawn_points()[args.spawn_index].location
    path = route(cmap, start, args.speed * (args.warm_s + args.measure_s) + 50.0)
    if len(path) < 10:
        raise RuntimeError("no drivable route from %s" % start)
    lib = world.get_blueprint_library()
    car_bp = lib.find(args.vehicle) if lib.filter(args.vehicle) else lib.filter("vehicle.*")[0]
    car_bp.set_attribute("role_name", "hero")
    actors, scen, counts, frames = [], None, [], []
    still = {}
    measuring = [False]     # set before the cameras: their callbacks read it from the first frame
    try:
        car = world.spawn_actor(car_bp, carla.Transform(path[0].location + carla.Location(z=0.5), path[0].rotation))
        actors.append(car)
        car.set_simulate_physics(False)
        if args.scenery:
            seas = {"state": "leaf", "lights": season.light_blueprints(world) if sun < 3 and radius != 0 else {},
                    "lights_radius_m": radius if radius is not None else 200.0}
            scen = scenery.Scenery(client, world, args.scenery, {"radius_m": args.scenery_radius}, log=log,
                                   season=seas)
        cw, ch = (int(v) for v in args.cam.split("x"))
        cam_bp = lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(cw))
        cam_bp.set_attribute("image_size_y", str(ch))
        cam_bp.set_attribute("fov", str(args.fov))
        if cam_bp.has_attribute("post_process_profile") and args.profile:
            cam_bp.set_attribute("post_process_profile", args.profile)
        for i in range(args.cams):
            off = (i - (args.cams - 1) / 2) * 0.064     # eyes 64 mm apart for --cams 2
            cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=0.3, y=-0.35 + off, z=1.25)),
                                    attach_to=car)
            actors.append(cam)
            counts.append(0)

            def cb(img, i=i):
                counts[i] += 1
                if i == 0 and args.stills and "img" not in still and measuring[0]:
                    still["img"] = img
            cam.listen(cb)
        dt_route = 1.0 / args.speed
        t0 = time.time()
        k = 0
        t_measure = None
        while True:
            now = time.time()
            el = now - t0
            if el > args.warm_s + args.measure_s:
                break
            i = min(len(path) - 1, int(el / dt_route))
            if i != k:
                k = i
                car.set_transform(path[i])
                if scen is not None and i % 20 == 0:
                    scen.update([path[i].location])
            snap = world.wait_for_tick(5.0)
            if el >= args.warm_s:
                if t_measure is None:
                    t_measure = time.time()
                    counts[:] = [0] * len(counts)
                    measuring[0] = True
                    continue
                frames.append(snap.timestamp.delta_seconds)
        wall = time.time() - (t_measure or time.time())
        res = summarise(frames, counts, wall, args.target, args.vr)
        if still.get("img") is not None and args.stills:
            p = out_dir / f"{tag}.png"
            still["img"].save_to_disk(str(p))
            res["still"] = p.name
        if scen is not None:
            res["lamps_lit"] = sum(len(v) for k, v in scen.lamps.items() if k in scen.lit)
        lm = world.get_lightmanager()
        street = lm.get_all_lights(carla.LightGroup.Street)
        res["street_lights"] = len(street)
        res["street_lights_on"] = sum(1 for li in street if li.is_on)
        return res
    finally:
        for a in actors[::-1]:
            try:
                if a.type_id.startswith("sensor."):
                    a.stop()
                a.destroy()
            except RuntimeError:
                pass
        if scen is not None:
            scen.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--levels", default="", help="comma-separated levels to load (default: the loaded one)")
    ap.add_argument("--cases", default="day,night", help=f"of {sorted(CASES)}")
    ap.add_argument("--scenery", default=None, help="coupled-carla map folder with scenery.json (trees, lamps)")
    ap.add_argument("--scenery-radius", type=float, default=250.0)
    ap.add_argument("--start", default=None, help="x,y,z of the route's start (default: a spawn point)")
    ap.add_argument("--spawn-index", type=int, default=0)
    ap.add_argument("--speed", type=float, default=8.3, help="m/s")
    ap.add_argument("--warm-s", type=float, default=15.0)
    ap.add_argument("--measure-s", type=float, default=30.0)
    ap.add_argument("--vehicle", default="vehicle.lincoln.mkz")
    ap.add_argument("--cam", default="1920x1080")
    ap.add_argument("--cams", type=int, default=1)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--profile", default="", help="post_process_profile of the cameras (default: CARLA's)")
    ap.add_argument("--target", type=float, default=60.0)
    ap.add_argument("--vr", action="store_true", help="need 90 fps (a VR headset)")
    ap.add_argument("--no-stills", dest="stills", action="store_false")
    ap.add_argument("--server-cmd", default="", help="shell command that (re)starts the server; {extra} = the "
                                                       "variant's launch options")
    ap.add_argument("--server-stop-cmd", default="")
    ap.add_argument("--variants", default="", help="name=launch options;name=... (default: current)")
    ap.add_argument("--server-wait-s", type=float, default=240.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    import os
    os.environ.setdefault("COUPLED_CARLA_MUTE", "1")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    import carla
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = [c.strip() for c in args.cases.split(",") if c.strip()]
    bad = [c for c in cases if c not in CASES]
    if bad:
        sys.exit(f"unknown cases {bad}; known: {sorted(CASES)}")
    variants = parse_variants(args.variants) if args.server_cmd else dict(DEFAULT_VARIANTS)
    results = []
    for vname, extra in variants.items():
        if args.server_cmd:
            if args.server_stop_cmd:
                subprocess.run(args.server_stop_cmd, shell=True, check=False)
                time.sleep(5)
            cmd = args.server_cmd.format(extra=extra)
            log(f"variant {vname}: {cmd}")
            subprocess.Popen(cmd, shell=True)
            time.sleep(10)
            if not wait_port(args.host, args.port, args.server_wait_s):
                results.append({"variant": vname, "error": "server did not come up"})
                continue
        client = carla.Client(args.host, args.port)
        client.set_timeout(180.0)
        levels = [lv for lv in args.levels.split(",") if lv] or [None]
        for level in levels:
            world = client.load_world(level) if level else client.get_world()
            lname = world.get_map().name.split("/")[-1]
            for case in cases:
                tag = f"{vname}_{lname}_{case}"
                log(f"measuring {tag}")
                try:
                    r = measure_case(client, world, args, case, out_dir, tag)
                except Exception as exc:  # noqa: BLE001 - one failing case must not lose the others
                    r = {"error": f"{type(exc).__name__}: {exc}"}
                r.update(variant=vname, level=lname, case=case, server=client.get_server_version(),
                         cam=args.cam, cams=args.cams)
                log(json.dumps(r))
                results.append(r)
                (out_dir / "frame_budget.json").write_text(json.dumps(results, indent=1))
    if args.server_cmd and args.server_stop_cmd:
        subprocess.run(args.server_stop_cmd, shell=True, check=False)
    lines = ["| variant | level | case | server fps | 1 % low | camera fps | lamps lit | need | verdict |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        if "error" in r:
            lines.append(f"| {r.get('variant')} | {r.get('level', '')} | {r.get('case', '')} | error: "
                         f"{r['error']} | | | | | |")
            continue
        lines.append(f"| {r['variant']} | {r['level']} | {r['case']} | {r['server_fps']} | {r['low_1pct_fps']} | "
                     f"{', '.join(map(str, r['camera_fps']))} | {r.get('lamps_lit', '')} | {r['need_fps']} | "
                     f"{'PASS' if r['pass'] else 'FAIL'} |")
    (out_dir / "frame_budget.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if all(r.get("pass") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
