"""What a VR station costs CARLA: two eye cameras at the driver's seat, moved from a head pose every tick.

    python tools/vr_bench.py --port 2500 --sizes 0 1280x720:1 960x1056 1440x1584 1832x1920 --ticks 300
    python tools/vr_bench.py --port 2500 --runtime openxr --sizes 1440x1584     # with a headset (or Monado)

Needs a running CARLA server (0.9.16 or 1.0) on --port. In synchronous mode it ticks as fast as the server can with
a car on autopilot and, per size, two eye cameras (vr.VRView, the station's own code: head pose, prediction,
ApplyTransform to the attached cameras every tick, images paired by frame). "WxH:1" is one monitor camera instead
(an ordinary station), "0" no camera. Per size it prints the server's tick time and frame rate, the pose-to-image
latency (head pose sampled -> both eye images at the client), which frame the images belong to and how far the
image's own camera pose is from the pose that was set (the check that ApplyTransform moves attached cameras where
vr.py thinks), and with a headset the motion-to-photon estimate and the reprojection the runtime has to do. The
rows also go to --out as JSON.
"""
import argparse
import json
import math
import statistics
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import carla  # noqa: E402

from coupledcarla import cockpit, vr  # noqa: E402


class _Station:
    """What VRView needs of a station."""

    def __init__(self, camera=None):
        self.pending = []
        self.heading = 0.0
        self.camera = camera


def q(xs, p):
    xs = sorted(x for x in xs if x is not None)
    return round(xs[min(len(xs) - 1, int(p * (len(xs) - 1)))], 1) if xs else None


def run_size(world, client, vehicle, spec, args, headset):
    """One configuration: returns its row."""
    eye = cockpit.actor_eye(vehicle, "driver", False, client.get_server_version())
    st = _Station()
    view = None
    mono = None
    images = []
    if spec != "0":
        size, _, n = spec.partition(":")
        w, h = (int(v) for v in size.split("x"))
        if n == "1":                        # one monitor camera (an ordinary station's view)
            bp = world.get_blueprint_library().find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(w))
            bp.set_attribute("image_size_y", str(h))
            bp.set_attribute("fov", "90")
            mono = world.spawn_actor(bp, carla.Transform(carla.Location(*eye[:3]), carla.Rotation(pitch=eye[3])),
                                     attach_to=vehicle)
            mono.listen(lambda im: images.append((im.frame, time.perf_counter())))
        else:
            cfg = vr.settings({"runtime": args.runtime, "size": [w, h], "predict_ms": args.predict_ms,
                               "fov_deg": [args.fov, args.fov * h / w]})
            view = vr.VRView(st, cfg, headset, log=lambda m: print(m, flush=True))
            view.start(world, vehicle, "driver", eye, lambda bp: None,
                       Path(args.out).with_name(f"vr_bench_{spec}.csv"))
    tick_ms = []
    t_start = time.perf_counter()
    for i in range(args.warmup + args.ticks):
        snap = world.get_snapshot()
        if view is not None:
            view.tick(snap)
            view.compose_and_submit()
        if st.pending:
            client.apply_batch_sync(list(st.pending), False)
            st.pending.clear()
        if i == args.warmup:
            t_start = time.perf_counter()
        t0 = time.perf_counter()
        world.tick(10.0)
        if i >= args.warmup:
            tick_ms.append(1000 * (time.perf_counter() - t0))
    t_end = time.perf_counter()
    time.sleep(0.5)
    row = {"config": spec, "ticks": args.ticks, "tick_ms_median": q(tick_ms, 0.5), "tick_ms_p95": q(tick_ms, 0.95),
           "fps": round(args.ticks / (t_end - t_start), 1)}
    if view is not None:
        row["eye_size"] = f"{view.cam['width']}x{view.cam['height']}"
        row["hfov_deg"] = round(view.cam["hfov_deg"], 1)
        row["pose_to_image_ms_median"] = q(view.lat.samples, 0.5)
        row["pose_to_image_ms_p95"] = q(view.lat.samples, 0.95)
        row["attached_relative"] = view.relative
        view._flush_log()
        view.records.clear()
        view.stop()
        rows = list(_read_vr_csv(Path(args.out).with_name(f"vr_bench_{spec}.csv")))
        row["image_pose_error_deg_median"] = q([r["carla_check_deg"] for r in rows], 0.5)
        row["image_pose_error_cm_median"] = q([r["carla_check_cm"] for r in rows], 0.5)
        row["image_pose_error_cm_p95"] = q([r["carla_check_cm"] for r in rows], 0.95)
        row["horizon_ms_median"] = q([r["horizon_ms"] for r in rows], 0.5)
        m2p = [r["motion_to_photon_ms"] for r in rows if r["motion_to_photon_ms"] is not None]
        row["motion_to_photon_ms_median"] = q(m2p, 0.5)
        row["motion_to_photon_ms_p95"] = q(m2p, 0.95)
        row["reprojection_deg_median"] = q([r["reprojection_deg"] for r in rows], 0.5)
        row["reprojection_deg_p95"] = q([r["reprojection_deg"] for r in rows], 0.95)
        row["images_paired"] = len(rows)
    if mono is not None:
        mono.stop()
        mono.destroy()
        row["images"] = len(images)
    return row


def _read_vr_csv(path):
    import csv
    with open(path) as f:
        for r in csv.DictReader(f, delimiter=";"):
            yield {k: (float(v) if v not in ("", None) else None) for k, v in r.items()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2500)
    ap.add_argument("--tm-port", type=int, default=8502)
    ap.add_argument("--sizes", nargs="+", default=["0", "1280x720:1", "960x1056", "1440x1584", "1832x1920"])
    ap.add_argument("--ticks", type=int, default=300)
    ap.add_argument("--warmup", type=int, default=60)
    ap.add_argument("--step", type=float, default=0.02, help="fixed_delta_seconds (the host's step)")
    ap.add_argument("--fov", type=float, default=100.0, help="runtime sim: per-eye horizontal fov (deg)")
    ap.add_argument("--runtime", default="sim", choices=["sim", "openxr"])
    ap.add_argument("--predict-ms", default="auto")
    ap.add_argument("--vehicle", default="vehicle.lincoln.mkz_2020")
    ap.add_argument("--out", default="logs/vr_bench.json")
    args = ap.parse_args(argv)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    print(f"server {client.get_server_version()}, map {world.get_map().name}", flush=True)
    original = world.get_settings()
    st = world.get_settings()
    st.synchronous_mode, st.fixed_delta_seconds = True, args.step
    world.apply_settings(st)
    tm = client.get_trafficmanager(args.tm_port)
    tm.set_synchronous_mode(True)
    bp = world.get_blueprint_library().find(args.vehicle)
    vehicle = None
    for sp in world.get_map().get_spawn_points():
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle:
            break
    vehicle.set_autopilot(True, args.tm_port)
    world.tick()
    headset = vr.make_headset(vr.settings({"runtime": args.runtime, "fov_deg": [args.fov, args.fov]}))
    headset.start()
    rows = []
    try:
        for spec in args.sizes:
            row = run_size(world, client, vehicle, spec, args, headset)
            if args.runtime == "openxr":
                row["runtime"] = getattr(headset, "info", {})
            print(json.dumps(row), flush=True)
            rows.append(row)
    finally:
        headset.stop()
        vehicle.destroy()
        tm.set_synchronous_mode(False)
        world.apply_settings(original)
        Path(args.out).write_text(json.dumps(rows, indent=1))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
