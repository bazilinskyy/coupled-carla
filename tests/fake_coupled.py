"""A fake host and fake stations over the real control channel (coupling.py), for tests without CARLA.

    python tests/fake_coupled.py host EXPERIMENT DURATION OUT_DIR          # writes a trial folder like run.py
    python tests/fake_coupled.py station ROLE CONTROL_PORT [--rtt-ms N] [--fps F] [--images-share S] [--bad ROLE]

The host waits for the experiment's station roles, runs a "trial" of DURATION s at 50 Hz (tick messages, no world),
logs station_joined / station_stats / trial_start / trial_end in events.csv and the last stats per station in
summary.json, then ends the trial and closes. A station answers like coupledcarla.station: it prints the welcome
line, counts frames and images with labcheck.Counter, and sends stats (every 0.5 s here).
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import labcheck  # noqa: E402
from coupledcarla.coupling import ControlClient, ControlServer  # noqa: E402


def host(a):
    from coupledcarla import config
    cfg = config.load(a.experiment)
    roles = [p["role"] for p in cfg["participants"] if p.get("station")]
    out = Path(a.out) / f"{time.strftime('%Y%m%d-%H%M%S')}_trial0"
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.yaml").write_text("simulation: {step: 0.02}\n")
    print(f"[    0.0s] trial 0 -> {out}", flush=True)
    srv = ControlServer(int(cfg["coupling"]["port"]), log=lambda m: print(m, flush=True))
    srv.on_hello = lambda role: {"kind": role, "experiment": cfg["name"], "step": 0.02,
                                 "sound": {"enabled": role == "pedestrian"}}
    ev = open(out / "events.csv", "w")
    ev.write("timestamp;frame;event;actor_id;detail\n")

    def event(t, frame, name, detail):
        ev.write(f"{t:.3f};{frame};{name};;{json.dumps(detail) if not isinstance(detail, str) else detail}\n")
    missing = srv.wait_for(roles, 6)
    if missing:
        event(0, 0, "stations_missing", ",".join(sorted(missing)))
    for r in roles:
        srv.send(r, {"type": "state", "state": "running", "actor_id": 1, "trial": 0})
    event(0, 0, "trial_start", json.dumps({"trial": 0, "stations": roles}))
    stats, frame, t0 = {}, 100, time.time()
    while time.time() - t0 < a.duration:
        frame += 1
        srv.broadcast({"type": "tick", "frame": frame, "t": time.time()})
        for msg in srv.poll():
            if msg.get("type") == "stats":
                st = {k: v for k, v in msg.items() if k not in ("type", "recv")}
                stats[msg["role"]] = st
                event(time.time() - t0, frame, "station_stats", json.dumps(st))
            elif msg.get("type") == "hello":
                event(time.time() - t0, frame, "station_joined", msg["role"])
        time.sleep(0.02)
    wall = time.time() - t0
    summary = {"trial": 0, "ticks": frame - 100, "sim_s": round(0.02 * (frame - 100), 2), "wall_s": round(wall, 2),
               "rtf": round(0.02 * (frame - 100) / wall, 3), "stations": roles, "station_stats": stats,
               "collisions": 0}
    event(wall, frame, "trial_end", json.dumps(summary))
    ev.close()
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    srv.broadcast({"type": "state", "state": "ended"})
    time.sleep(0.3)
    srv.close()
    time.sleep(0.3)


def station(a):
    if a.bad == a.role:          # a station on a poor link: slow round trips, half the frames and images
        a.rtt_ms, a.fps, a.images_share, a.image_p95 = 80.0, 20.0, 0.5, 400.0
    welcome = {}
    client = ControlClient("127.0.0.1", a.port, a.role, retry_s=20)
    count = labcheck.Counter()
    running, t_last, t_frame, n = False, time.time(), time.time(), 0
    while True:
        for msg in client.poll():
            if msg.get("type") == "welcome":
                welcome = msg
                print(f"[station {a.role}] {msg.get('kind')} in {msg.get('experiment')}", flush=True)
            elif msg.get("type") == "state" and msg.get("state") == "running":
                running = True
                count.running()
            elif msg.get("type") == "state" and msg.get("state") in ("ended", "closed"):
                return 0
        if running and time.time() - t_frame >= 1.0 / a.fps:
            t_frame = time.time()
            n += 1
            count.frame(1000 + n * (2 if a.skip else 1))
            if (n * a.images_share) % 1 < a.images_share:
                count.image()
        if running and time.time() - t_last >= 0.5:
            t_last = time.time()
            count.window(0.0)
            client.send({"type": "stats", "rtt_ms": a.rtt_ms, "fps": count.fps, "frames": count.frames,
                         "host": "fakehost", "ticks_seen": count.frames, "camera_hz": 0, **{
                             k: v for k, v in count.stats().items() if k not in ("fps", "frames")},
                         "latency": {"clock_offset_ms": 0.0, "tick_to_station_ms": {"median": 0.1, "p95": 0.2, "n": 9},
                                     "tick_to_image_ms": {"median": 40.0, "p95": a.image_p95, "n": 9}},
                         "ux": labcheck.ux_flags(welcome, None)})
        time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="what", required=True)
    h = sub.add_parser("host")
    h.add_argument("experiment")
    h.add_argument("duration", type=float)
    h.add_argument("out")
    s = sub.add_parser("station")
    s.add_argument("role")
    s.add_argument("port", type=int)
    s.add_argument("--rtt-ms", type=float, default=1.0)
    s.add_argument("--fps", type=float, default=50.0)
    s.add_argument("--images-share", type=float, default=1.0)
    s.add_argument("--image-p95", type=float, default=60.0)
    s.add_argument("--skip", action="store_true", help="see every second frame only (dropped ticks)")
    s.add_argument("--bad", help="this role behaves like a station on a poor link")
    a = ap.parse_args()
    raise SystemExit(host(a) if a.what == "host" else station(a))


if __name__ == "__main__":
    main()
