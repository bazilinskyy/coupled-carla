"""Test coupled-carla on CARLA 1.0 (Unreal Engine 5.8): the feature matrix against one server, a Markdown table out.

    python tools/test_carla10.py --port 2080                        # every headless test against a running server
    python tools/test_carla10.py --port 2080 --only twin,sound_driver --duration 40
    python tools/test_carla10.py --port 2080 --launch scratch/srv58fg.sh   # start (and restart) the server itself
    python tools/test_carla10.py --render-checks --port 2000        # the camera checks, on a rendering server
    python tools/test_carla10.py --list

Each test runs a short trial (coupledcarla.run, headless, `simulation.profile: true`, no real-time sleep unless
stations need it) of an experiment derived from one in experiments/, or one of the tools (make_videos.py,
station_dryrun.py), and reads the trial's summary.json, events.csv, audio.json, season.json, weather.json and the
host's log. The table has: feature, result (ok / FAILED / partial / skipped), numbers, notes; results.json and
the logs go to logs/test10/<time>/.

Headless servers (`-nullrhi`, no GPU): CARLA 1.0's -nullrhi server dies (SIGSEGV in ImageUtil::ReadImageDataBegin)
as soon as a camera sensor renders, so the headless matrix spawns no camera: frames_every_s 0, no drone observer
cameras, film and stations run in their no-camera modes. `--render-checks` runs what needs the camera on a rendering
server (the win5090 package): film frames, station images, the drone FPV and observer cameras, frames_every views,
night lighting in frames.

--launch CMD starts the server with CMD (it must run the server in the foreground, e.g. `exec UnrealEditor ...
-game -nullrhi -carla-rpc-port=2080`), restarts it before every test that loads another map or after its memory
passed --restart-above-gb, waits for --need-free-gb of available memory before each start (shared machines), and
kills it (its whole process group) when its RSS passes --max-rss-gb or available memory falls under --min-free-gb
(Linux). Without --launch the tests attach to the server on --host:--port and never stop it.
"""
import argparse
import csv
import json
import os
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import wave
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config as ccconfig  # noqa: E402  (yaml only, no CARLA)

TWIN_MAP = {"name": "eindhoven_twin", "opendrive": "twins/EhvCentre.xodr", "carla_level": "EhvCentre"}
WEEKDAY = "2026-11-17 14:00"          # a Tuesday afternoon: parked occupancy, bus timetables
LOG_LINE = re.compile(r"^\[\s*([0-9.]+)s\] (.*)$")
ERROR_WORDS = ("Traceback", "Error:", "ERROR", "Segmentation fault", "core dumped", "failed", "TIMEOUT")
BENIGN = ("Retrying in", "analysis failed")       # rpclib's reconnect notice


# -- the server -------------------------------------------------------------------------------------------------

def port_open(host, port):
    try:
        socket.create_connection((host, port), timeout=2).close()
        return True
    except OSError:
        return False


def available_gb():
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024 ** 2
    except OSError:
        pass
    return 1e9


def group_pids(pgid):
    """The processes of process group pgid (Linux /proc)."""
    out = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            stat = (d / "stat").read_text()
            fields = stat[stat.rindex(")") + 2:].split()
            if int(fields[2]) == pgid:
                out.append(int(d.name))
        except (OSError, ValueError, IndexError):
            continue
    return out


def rss_gb(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024 ** 2
    except OSError:
        pass
    return 0.0


class Server:
    """Attaches to host:port, or (launch) starts CMD in its own process group with a memory watchdog."""

    def __init__(self, a, log):
        self.a, self.log = a, log
        self.proc = self.logf = None
        self.map = None
        self.max_rss = 0.0
        self.killed = None
        self.starts = []          # seconds from start to the RPC port, per start
        self._stop = threading.Event()
        self._dog = None

    @property
    def launch(self):
        return bool(self.a.launch)

    def rss(self):
        if not self.proc:
            return 0.0
        return max((rss_gb(p) for p in group_pids(self.proc.pid)), default=0.0)

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def ensure(self, map_key):
        """A server for a test on map_key: restarted when it loaded another map or grew past --restart-above-gb."""
        if not self.launch:
            if not port_open(self.a.host, self.a.port):
                raise RuntimeError(f"no CARLA server on {self.a.host}:{self.a.port}")
            return
        if self.running() and (map_key != self.map or self.rss() > self.a.restart_above_gb):
            self.log(f"server: restarting ({'map ' + str(map_key) if map_key != self.map else f'{self.rss():.1f} GB'})")
            self.stop()
        if not self.running():
            self.start()
        self.map = map_key

    def start(self):
        t_wait = time.time()
        while available_gb() < self.a.need_free_gb:
            if time.time() - t_wait > self.a.wait_free_s:
                raise RuntimeError(f"under {self.a.need_free_gb:g} GB available for {self.a.wait_free_s} s")
            if int(time.time() - t_wait) % 120 < 10:
                self.log(f"server: {available_gb():.1f} GB available, waiting for {self.a.need_free_gb:g} GB")
            time.sleep(10)
        if port_open(self.a.host, self.a.port):
            raise RuntimeError(f"port {self.a.port} is taken by a server this tool did not start")
        self.logf = open(self.a.out_dir / f"server_{len(self.starts)}.log", "w")
        t0 = time.time()
        self.proc = subprocess.Popen(self.a.launch, shell=True, cwd=ROOT, stdout=self.logf, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        self.log(f"server: started (process group {self.proc.pid})")
        self._stop.clear()
        self._dog = threading.Thread(target=self._watch, daemon=True)
        self._dog.start()
        while not port_open(self.a.host, self.a.port):
            if self.proc.poll() is not None:
                raise RuntimeError(f"the server exited during start-up ({self.killed or 'rc %s' % self.proc.returncode})")
            if time.time() - t0 > 900:
                self.stop()
                raise RuntimeError("the server did not open its port in 900 s")
            time.sleep(2)
        time.sleep(5)
        self.starts.append(round(time.time() - t0, 1))
        self.log(f"server: port {self.a.port} open after {self.starts[-1]} s, {self.rss():.1f} GB")

    def _watch(self):
        if not sys.platform.startswith("linux"):
            return
        while not self._stop.wait(2.0):
            if not self.running():
                return
            r, avail = self.rss(), available_gb()
            self.max_rss = max(self.max_rss, r)
            if r > self.a.max_rss_gb or avail < self.a.min_free_gb:
                self.killed = f"watchdog: RSS {r:.1f} GB, {avail:.1f} GB available"
                self.log(f"server: {self.killed}; killing it")
                self._kill()
                return

    def _kill(self):
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except OSError:
            pass

    def stop(self):
        if not self.launch or self.proc is None:
            return
        self._stop.set()
        if self.proc.poll() is None:
            self._kill()
            try:
                self.proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                pass
        for _ in range(30):
            if not port_open(self.a.host, self.a.port):
                break
            time.sleep(1)
        if self.logf:
            self.logf.close()
        self.proc = self.map = None
        self.log("server: stopped")

    def died(self):
        """Why the server went away during a test, or None."""
        if self.launch:
            if self.proc is not None and self.proc.poll() is not None:
                return self.killed or f"the server exited (rc {self.proc.returncode})"
            return None
        return None if port_open(self.a.host, self.a.port) else "the server's port closed"


# -- running trials ---------------------------------------------------------------------------------------------

def derive(base, name, ctx, over=None, duration=None):
    """The experiment for a test: base (a file under experiments/) with `over` merged in, this server, one trial."""
    raw = yaml.safe_load((ROOT / base).read_text()) or {}
    exp = ccconfig._merge(raw, over or {})
    exp["map"] = dict(exp.get("map") or {})
    exp["map"].setdefault("name", raw.get("name"))
    od = exp["map"].get("opendrive")
    if od and not Path(od).is_absolute():
        exp["map"]["opendrive"] = str(ROOT / od)
    exp["name"] = f"t10_{name}"
    exp["server"] = ccconfig._merge(exp.get("server") or {}, {"host": ctx.a.host, "port": ctx.a.port, "launch": False,
                                                             "tm_port": ctx.a.port + 40})
    sim = exp.setdefault("simulation", {})
    sim["profile"] = True
    sim.setdefault("real_time", False)
    lg = exp.setdefault("logging", {})
    lg.update(carla_recorder=False, analyse=False)
    if not ctx.a.render_checks:
        lg["frames_every_s"] = 0.0
    for k in ("conditions", "design", "dashboard"):
        exp.pop(k, None)
    exp["trials"] = [{"duration_s": float(duration or ctx.a.duration)}]
    return exp


def parse_log(text):
    """Timed lines of a run.py log: [(t, message)], plus the error lines."""
    lines, errors = [], []
    for raw in text.splitlines():
        m = LOG_LINE.match(raw)
        lines.append((float(m.group(1)), m.group(2)) if m else (None, raw))
        if any(w in raw for w in ERROR_WORDS) and not any(b in raw for b in BENIGN):
            errors.append(raw.strip()[:240])
    return lines, errors


def first(lines, pattern, group=0, cast=str):
    rx = re.compile(pattern)
    for t, msg in lines:
        m = rx.search(msg)
        if m:
            return (t, cast(m.group(group)))
    return (None, None)


def read_events(trial):
    p = Path(trial) / "events.csv"
    if not p.exists():
        return []
    with open(p, newline="") as fh:
        return list(csv.DictReader(fh, delimiter=";"))


class Ctx:
    def __init__(self, a):
        self.a = a
        self.python = a.python or sys.executable
        self.server = Server(a, self.log)
        self.results = []

    def log(self, msg):
        print(f"[test10 {time.strftime('%H:%M:%S')}] {msg}", file=sys.stderr, flush=True)

    def run_host(self, name, exp, extra=(), timeout=None):
        """Runs coupledcarla.run on a derived experiment; returns a record with the trial folder and what it logged."""
        wd = self.a.out_dir / name
        wd.mkdir(parents=True, exist_ok=True)
        path = wd / "experiment.yaml"
        path.write_text(yaml.safe_dump(exp, sort_keys=False, allow_unicode=True))
        argv = [self.python, "-m", "coupledcarla.run", str(path), "--trial", "0", "--headless", *extra]
        duration = exp["trials"][0]["duration_s"]
        timeout = timeout or duration * 8 + 900
        self.log(f"{name}: {' '.join(argv[2:])}")
        t0 = time.time()
        with open(wd / "host.log", "w") as lf:
            p = subprocess.Popen(argv, cwd=ROOT, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True,
                                 env={**os.environ, "PYTHONUNBUFFERED": "1"})
            try:
                rc = p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                rc = p.wait()
                lf.write("\nTIMEOUT\n")
        return self.record(wd, rc, time.time() - t0)

    def record(self, wd, rc, wall):
        text = (wd / "host.log").read_text(errors="replace")
        lines, errors = parse_log(text)
        m = re.search(r"trial \d+ -> (\S+)", text)
        trial = Path(m.group(1)) if m else None
        summary = {}
        if trial and (trial / "summary.json").exists():
            summary = json.loads((trial / "summary.json").read_text())
        rec = {"rc": rc, "wall_s": round(wall, 1), "trial": str(trial) if trial else None, "summary": summary,
               "errors": errors, "lines": lines, "server_died": self.server.died(),
               "server_rss_gb": round(self.server.rss(), 1) if self.server.launch else None}
        rec["events"] = read_events(trial) if trial else []
        return rec


# -- what a trial measured --------------------------------------------------------------------------------------

def trial_numbers(rec):
    """The common numbers of a trial: RTF, ms per tick (total, server tick, carla_to_sumo), SUMO vehicles, buses,
    collisions, world load time, scenery, parked cars, signals, crowd."""
    s, lines, ev = rec["summary"], rec["lines"], rec["events"]
    ms = s.get("ms_per_tick") or {}
    t_conn, _ = first(lines, r"connected to CARLA")
    t_ready, sp = first(lines, r"world ready: (\d+) spawn points", 1, int)
    n = {
        "sim_s": s.get("sim_s"), "rtf": s.get("rtf"), "ticks": s.get("ticks"),
        "ms_tick": round(sum(ms.values()), 1) if ms else None,
        "carla_tick_ms": ms.get("carla_tick"), "carla_to_sumo_ms": ms.get("carla_to_sumo"),
        "sumo_ms": round(ms.get("sumo_step", 0) + ms.get("sumo_to_carla", 0) + ms.get("lod", 0), 2) if ms else None,
        "sumo_vehicles": s.get("sumo_vehicles_spawned"),
        "buses": sum(1 for e in ev if e.get("event") == "spawn" and str(e.get("detail", "")).startswith("bus ")),
        "collisions": s.get("collisions"), "parked": s.get("parked_cars"),
        "load_s": round(t_ready - t_conn, 1) if t_conn is not None and t_ready is not None else None,
        "spawn_points": sp,
        "scenery": first(lines, r"scenery: (\d+) meshes", 1, int)[1],
        "lamps_lit": first(lines, r"; (\d+) lamps with light", 1, int)[1],
        "signals": first(lines, r"traffic lights: (\d+) (?:signalled lanes|signal heads)", 1, int)[1],
        "crowd": first(lines, r"crowd: (\d+) of \d+ walkers spawned", 1, int)[1] or
        first(lines, r"crowd: (\d+) walkers beside", 1, int)[1],
        "left_road": sum(1 for e in ev if e.get("event") == "left_road"),
        "reset": sum(1 for e in ev if e.get("event") in ("reset_to_lane", "respawn")),
    }
    return n


def fmt(n, keys):
    labels = {"rtf": "RTF", "ms_tick": "ms/tick", "carla_tick_ms": "server tick ms", "carla_to_sumo_ms":
              "carla_to_sumo ms", "sumo_ms": "SUMO ms", "sumo_vehicles": "SUMO vehicles", "buses": "buses",
              "collisions": "collisions", "parked": "parked", "load_s": "load s", "scenery": "scenery",
              "lamps_lit": "lamps lit", "signals": "signals", "crowd": "crowd", "left_road": "left road",
              "reset": "reset to lane"}
    return ", ".join(f"{labels.get(k, k)} {n[k]}" for k in keys if n.get(k) is not None)


def verdict(rec, expect_s, extra_ok=True):
    s = rec["summary"]
    if rec["server_died"]:
        return "FAILED"
    if rec["rc"] != 0 or not s or (s.get("sim_s") or 0) < expect_s - 0.5:
        return "FAILED"
    return "ok" if extra_ok else "partial"


def notes_of(rec, more=()):
    out = list(more)
    if rec["server_died"]:
        out.append(f"server: {rec['server_died']}")
    errs = [e for e in rec["errors"] if "blueprint" not in e]
    if errs:
        out.append(f"{len(errs)} error lines, first: `{errs[0][:160]}`")
    return "; ".join(out)


# -- the tests ------------------------------------------------------------------------------------------------------

def crowd_scene_on_twin(ctx, name, over=None, duration=None):
    """eindhoven_crowd.yaml's scene (pedestrian, AV passenger, driver, two AI cars, two AI walkers) on the twin."""
    o = {"map": dict(TWIN_MAP), "simulation": {"time": WEEKDAY, "weather": "CloudyNoon"},
         "traffic": {"walkers": {"count": "auto", "radius_m": 150}, "cars_per_hour": 2400, "bikes_per_hour": 1200}}
    return derive("experiments/eindhoven_crowd.yaml", name, ctx, ccconfig._merge(o, over or {}), duration)


def t_twin(ctx):
    """1. The Eindhoven twin (baked EhvCentre) with --local bots, SUMO traffic, buses, parked cars, scenery, signals,
    crowd."""
    rec = ctx.run_host("twin", crowd_scene_on_twin(ctx, "twin"), ["--local"])
    n = trial_numbers(rec)
    ok = bool(n["sumo_vehicles"]) and bool(n["scenery"]) and n["parked"] and n["signals"] and n["crowd"]
    missing = [k for k in ("sumo_vehicles", "scenery", "parked", "signals", "crowd", "buses") if not n.get(k)]
    return rec, verdict(rec, ctx.a.duration, ok), fmt(n, ["rtf", "ms_tick", "carla_tick_ms", "carla_to_sumo_ms",
                                                       "sumo_vehicles", "buses", "collisions", "scenery", "parked",
                                                       "signals", "crowd", "load_s"]), \
        notes_of(rec, [f"none of: {', '.join(missing)}"] if missing else [])


def t_odr(ctx, name, base, over=None):
    exp = derive(base, name, ctx, ccconfig._merge({"simulation": {"time": WEEKDAY}}, over or {}))
    if not any(p.get("kind", "driver") == "driver" for p in exp.get("participants") or []):
        exp["participants"] = [{"role": "driver", "vehicle": "vehicle.lincoln.mkz_2020"}] + \
            list(exp.get("participants") or [])
    for p in exp["participants"]:
        if p.get("kind", "driver") == "driver":
            p["input"] = "autopilot"
    rec = ctx.run_host(name, exp, ["--local"] if any(p.get("station") for p in exp["participants"]) else [])
    n = trial_numbers(rec)
    ok = bool(n["sumo_vehicles"]) and bool(n["scenery"]) and n["signals"] is not None
    return rec, verdict(rec, ctx.a.duration, ok), fmt(n, ["load_s", "spawn_points", "rtf", "ms_tick", "carla_tick_ms",
                                                       "carla_to_sumo_ms", "sumo_vehicles", "buses", "collisions",
                                                       "scenery", "parked", "signals", "left_road", "reset"]), \
        notes_of(rec)


def t_odr_centre(ctx):
    """2a. eindhoven_centre: an OpenDRIVE world generated from the OSM map (generate_opendrive_world)."""
    return t_odr(ctx, "odr_centre", "experiments/eindhoven_centre.yaml")


def t_odr_mapmaker(ctx):
    """2b. A map made by the map maker (--mapmaker-exp), generated as an OpenDRIVE world."""
    if not ctx.a.mapmaker_exp:
        return None, "skipped", "", "no --mapmaker-exp (build one: python -m coupledcarla.mapmaker \"<place>\")"
    exp_rel = os.path.relpath(Path(ctx.a.mapmaker_exp).resolve(), ROOT)
    return t_odr(ctx, "odr_mapmaker", exp_rel)


def audio_check(rec):
    """audio.wav against audio.json and the trial: samples = ticks x step x 44100, length = simulated time."""
    trial = Path(rec["trial"]) if rec["trial"] else None
    if not trial or not (trial / "audio.json").exists():
        return False, "no audio.json", {}
    aj = json.loads((trial / "audio.json").read_text())
    with wave.open(str(trial / "audio.wav")) as w:
        frames, rate, ch = w.getnframes(), w.getframerate(), w.getnchannels()
    s = rec["summary"]
    ticks, step = s.get("ticks", 0), float(aj.get("step_s", 0.02))
    want = int(round(ticks * step * rate))
    ok = frames == aj.get("samples") and abs(frames - want) <= rate * step + 1 and rate == 44100 and ch == 2
    info = {"wav_s": round(frames / rate, 3), "sim_s": s.get("sim_s"), "ticks": ticks, "samples": frames,
            "expected": want, "start_frame": aj.get("start_frame"), "audio_ms_tick": aj.get("ms_per_tick"),
            "kind": aj.get("kind") or aj.get("listener"), "limiter": aj.get("limiter_samples")}
    return ok, (f"wav {info['wav_s']} s for {ticks} ticks ({want} samples expected, {frames} written), "
                f"start_frame {info['start_frame']}, {info['audio_ms_tick']} ms/tick"), info


def t_sound(ctx, name, first_role):
    over = {}
    exp = crowd_scene_on_twin(ctx, name, over)
    parts = exp["participants"]
    exp["participants"] = sorted(parts, key=lambda p: p["role"] != first_role)
    rec = ctx.run_host(name, exp, ["--local", "--record-audio"])
    ok, text, info = audio_check(rec)
    n = trial_numbers(rec)
    return rec, verdict(rec, ctx.a.duration, ok), f"{text}; " + fmt(n, ["rtf", "ms_tick"]), notes_of(rec)


def t_sound_driver(ctx):
    """3a. --record-audio with a driver first (inside the car)."""
    return t_sound(ctx, "sound_driver", "driver")


def t_sound_pedestrian(ctx):
    """3b. --record-audio with a pedestrian first (at the walker's head)."""
    return t_sound(ctx, "sound_pedestrian", "pedestrian")


def t_drones(ctx):
    """4. experiments/eindhoven_drones.yaml: the bot pilot and the AI missions (no cameras on -nullrhi)."""
    over = {"simulation": {"time": WEEKDAY}}
    if not ctx.a.render_checks:
        over["drones"] = {"cameras": []}
    exp = derive("experiments/eindhoven_drones.yaml", "drones", ctx, over, max(ctx.a.duration, 60))
    if not ctx.a.render_checks:
        exp["drones"]["cameras"] = []
    exp["participants"][0]["input"] = "bot"
    rec = ctx.run_host("drones", exp)
    n = trial_numbers(rec)
    d = rec["summary"].get("drones") or {}
    ev = rec["events"]
    phases = sum(1 for e in ev if e.get("event") == "drone_phase")
    crashes = sum(1 for e in ev if e.get("event") == "drone_crash")
    deliveries = sum(1 for e in ev if e.get("event") == "drone_delivery")
    ms = (rec["summary"].get("ms_per_tick") or {}).get("drones")
    txt = (f"{d.get('ai', d.get('n_ai', '?'))} AI drones, {phases} phase changes, {deliveries} deliveries, "
           f"{crashes} crashes, drones {ms} ms/tick; " + fmt(n, ["rtf", "ms_tick", "sumo_vehicles"]))
    ok = phases > 0
    if ctx.a.render_checks:     # the pilot's FPV (drone_pilot_*.jpg) and the observer camera on the square
        trial = Path(rec["trial"]) if rec["trial"] else None
        k1, m1, _ = frames_info(trial, "drone_*.jpg")
        k2, m2, _ = frames_info(trial, "*_frames/*.jpg")
        txt += f"; FPV frames {k1} (brightness {m1}), observer frames {k2} (brightness {m2})"
        ok = ok and k1 > 0 and k2 > 0
    return rec, verdict(rec, max(ctx.a.duration, 60), ok), txt, notes_of(rec, [f"summary {json.dumps(d)[:200]}"])


def t_film(ctx):
    """5. tools/make_videos.py against the running server (--no-launch): setup, the film section, timing grid, sound,
    clean exit. On -nullrhi the camera is not spawned (film.no_camera) and only camera.csv is written."""
    clip = ctx.a.clip
    work = ctx.a.out_dir / "film"
    argv = [ctx.python, "tools/make_videos.py", "render", clip, "--no-launch", "--server-host", ctx.a.host,
            "--port", str(ctx.a.port), "--work", str(work / "work"), "--out", str(work / "out"),
            "--start", "8", "--duration", "8", "--fps", "5"]
    if not ctx.a.render_checks:
        argv += ["--no-encode"]
    work.mkdir(parents=True, exist_ok=True)
    ctx.log(f"film: {' '.join(argv[1:])}")
    t0 = time.time()
    with open(work / "make_videos.log", "w") as lf:
        rc = subprocess.call(argv, cwd=ROOT, stdout=lf, stderr=subprocess.STDOUT, timeout=1800)
    log = (work / "make_videos.log").read_text(errors="replace")
    run = work / "work" / clip / "run.json"
    rj = json.loads(run.read_text()) if run.exists() else {}
    trial = Path(rj["trial_dir"]) if rj.get("trial_dir") else None
    film = json.loads((trial / "film" / "film.json").read_text()) if trial and (trial / "film" / "film.json").exists() \
        else {}
    cams = film.get("cameras") or {}
    rows = 0
    for cname in cams:
        p = trial / "film" / cname / "camera.csv"
        if p.exists():
            rows = sum(1 for _ in open(p)) - 1
    want = int(round(8 * 5))
    audio = trial / "audio.wav" if trial else None
    wav_s = None
    if audio and audio.exists():
        with wave.open(str(audio)) as w:
            wav_s = round(w.getnframes() / w.getframerate(), 2)
    imgs = sum(1 for c in cams for _ in (trial / "film" / c).glob("*.jpg")) if trial else 0
    no_images = [c for c, v in cams.items() if v.get("images") is False]
    if ctx.a.render_checks:
        ok = rc == 0 and rj.get("host_rc") == 0 and imgs >= want - 1
    else:       # headless: the trial, the camera's path and the sound, without images
        ok = rj.get("host_rc") == 0 and rows >= want - 1
    rec = {"rc": rc, "trial": str(trial) if trial else None, "summary": {}, "errors": parse_log(log)[1],
           "server_died": ctx.server.died(), "run": rj, "film": film}
    txt = (f"clip {clip}: host rc {rj.get('host_rc')}, RTF {rj.get('rtf')}, {rows} camera poses of {want} on the "
           f"5 fps grid, {imgs} images{' (server cannot render)' if no_images else ''}, audio {'recorded ' + str(wav_s) + ' s' if wav_s else 'none'}, "
           f"{time.time() - t0:.0f} s")
    return rec, ("ok" if ok else "FAILED"), txt, notes_of(rec, [] if ctx.a.render_checks else
                                                          ["images need a rendering server (--render-checks)"])


def t_stations(ctx):
    """6. tools/station_dryrun.py with bot stations (eindhoven_crossing_study.yaml). Headless: --no-camera stations."""
    exp = derive("experiments/eindhoven_crossing_study.yaml", "stations", ctx,
                 {"simulation": {"real_time": True}, "coupling": {"port": ctx.a.port + 30, "wait_for_stations_s": 120}},
                 30)
    wd = ctx.a.out_dir / "stations"
    wd.mkdir(parents=True, exist_ok=True)
    path = wd / "experiment.yaml"
    path.write_text(yaml.safe_dump(exp, sort_keys=False, allow_unicode=True))
    argv = [ctx.python, "tools/station_dryrun.py", str(path), "--duration", "30", "--out", str(wd / "dryrun"),
            "--server-host", ctx.a.host]
    ctx.log(f"stations: {' '.join(argv[1:])}")
    with open(wd / "dryrun.log", "w") as lf:
        rc = subprocess.call(argv, cwd=ROOT, stdout=lf, stderr=subprocess.STDOUT, timeout=1800)
    log = (wd / "dryrun.log").read_text(errors="replace")
    joined = len(re.findall(r"^\| \S+ \| yes", log, re.M))
    res = re.search(r"result: (.*)$", log, re.M)
    rec = {"rc": rc, "trial": None, "summary": {}, "errors": parse_log(log)[1], "server_died": ctx.server.died()}
    tline = re.search(r"^trial .*RTF.*$", log, re.M)
    txt = f"exit {rc} ({res.group(1) if res else 'no result'}); {tline.group(0)[-120:] if tline else ''}"
    # headless: images are absent by design, so 'thresholds missed' (exit 1) for images is the expected outcome
    ok = rc == 0 or (not ctx.a.render_checks and rc == 1 and "never joined" not in log)
    return rec, ("ok" if ok and not rec["server_died"] else "FAILED"), txt, \
        notes_of(rec, [] if ctx.a.render_checks else ["no camera images on -nullrhi: image rate/latency need "
                                                      "--render-checks"])


def frames_info(trial, pattern):
    """(number of saved images matching pattern, their mean brightness 0-255 and share of near-black pixels, from
    up to 8 of them), or (0, None, None)."""
    files = sorted(Path(trial).glob(pattern)) if trial else []
    if not files:
        return 0, None, None
    try:
        import numpy as np
        from PIL import Image
    except ImportError:
        return len(files), None, None
    picks = files[:: max(1, len(files) // 8)][:8]
    lum = [np.asarray(Image.open(f).convert("L"), dtype=float) for f in picks]
    return len(files), round(float(np.mean([x.mean() for x in lum])), 1), \
        round(float(np.mean([(x < 8).mean() for x in lum])), 3)


def t_weather(ctx, name, sim, check):
    exp = crowd_scene_on_twin(ctx, name, {"simulation": sim}, ctx.a.short)
    extra = ["--local"]
    if ctx.a.render_checks:     # the driver's view, saved every 2 s (the host saves the first participant's view)
        exp["participants"] = sorted(exp["participants"], key=lambda p: p["role"] != "driver")
        extra += ["--frames-every", "2"]
    rec = ctx.run_host(name, exp, extra)
    n = trial_numbers(rec)
    trial = Path(rec["trial"]) if rec["trial"] else None
    wx = json.loads((trial / "weather.json").read_text()) if trial and (trial / "weather.json").exists() else {}
    se = json.loads((trial / "season.json").read_text()) if trial and (trial / "season.json").exists() else {}
    ok, text = check(wx, se, n, rec)
    if ctx.a.render_checks:
        k, mean, black = frames_info(trial, "view_*.jpg")
        text += f"; {k} driver frames, mean brightness {mean}, near-black share {black}"
        ok = ok and k > 0
    return rec, verdict(rec, ctx.a.short, ok), text + "; " + fmt(n, ["rtf", "ms_tick", "lamps_lit"]), notes_of(rec)


def _wx_text(wx):
    keys = ("source", "preset", "sun_altitude", "sun_altitude_angle", "cloudiness", "precipitation", "night",
            "temperature_c")
    return ", ".join(f"{k} {wx[k]}" for k in keys if k in wx)


def t_weather_real(ctx):
    """7a. weather: real at time now (Open-Meteo, cached)."""
    def check(wx, se, n, rec):
        om = wx.get("open_meteo") or {}
        cw = wx.get("carla") or {}
        return bool(om) and "cloudiness" in cw, (f"Open-Meteo {om.get('time')}: {om.get('cloud_cover')}% cloud, rain "
                                                 f"{om.get('rain')} mm/h, wind {om.get('wind_speed_10m')} km/h, "
                                                 f"{om.get('temperature_2m')} C -> CARLA cloudiness "
                                                 f"{cw.get('cloudiness')}, wind {cw.get('wind_intensity')}; "
                                                 f"sun {wx.get('sun_altitude')} deg, night {wx.get('night')}")
    return t_weather(ctx, "weather_real", {"weather": "real", "time": "now"}, check)


def t_night(ctx):
    """7b. night: street lights lit (lamp blueprints of the content pack), headlights, crowd at night."""
    def check(wx, se, n, rec):
        lit = n.get("lamps_lit") or 0
        heads = any("headlights" in m or "LowBeam" in m for _, m in rec["lines"])
        return bool(wx.get("night")) and lit > 0, f"night {wx.get('night')}, {lit} lamps lit, " + _wx_text(wx) + \
            (", headlights logged" if heads else "")
    return t_weather(ctx, "night", {"weather": "ClearNight", "time": "2026-11-17 22:30"}, check)


def t_season(ctx):
    """7c. season: a winter day (deciduous trees bare or small, season.json)."""
    def check(wx, se, n, rec):
        state = se.get("state") or se.get("leaves")
        return bool(se) and state not in (None, "leaf"), f"season {state}, " + \
            ", ".join(f"{k} {v}" for k, v in se.items() if k in ("counts", "n", "deciduous", "bare"))[:200]
    return t_weather(ctx, "season", {"time": "2026-01-20 12:00", "weather": "CloudyNoon"}, check)


PROBE = r'''
import json, sys, time
import carla
host, port = sys.argv[1], int(sys.argv[2])
c = carla.Client(host, port); c.set_timeout(60)
out = {"server": c.get_server_version(), "client": c.get_client_version()}
w = c.get_world()
if sys.argv[3] not in w.get_map().name:
    c.set_timeout(600); t0 = time.time(); w = c.load_world(sys.argv[3]); c.set_timeout(60)
    out["load_world_s"] = round(time.time() - t0, 1)
out["map"] = w.get_map().name
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.02; w.apply_settings(s)
sp = w.get_map().get_spawn_points()
far = max(sp, key=lambda t: t.location.x ** 2 + t.location.y ** 2).location
spec = w.get_spectator()
def ground(at):
    hit = w.ground_projection(carla.Location(at.x, at.y, at.z + 50), 200.0)
    return None if hit is None else round(hit.location.z, 2)
spec.set_transform(carla.Transform(carla.Location(z=3000), carla.Rotation(pitch=90)))
for _ in range(100): w.tick()
out["ground_far_spectator_away"] = ground(far)
spec.set_transform(carla.Transform(carla.Location(far.x, far.y, far.z + 60), carla.Rotation(pitch=90)))
t0 = time.time(); found = None
for k in range(500):
    w.tick()
    if k % 10 == 9 and ground(far) is not None:
        found = round(time.time() - t0, 2); break
out["ground_far_spectator_over"] = ground(far)
out["streamed_in_s"] = found
out["far_point"] = [round(far.x), round(far.y)]
# rpclib stall: tick alone vs. tick after two fire-and-forget calls (apply_control on one vehicle)
bp = w.get_blueprint_library().filter("vehicle.lincoln.mkz")[0]
v = w.try_spawn_actor(bp, sp[0]); w.tick()
def timed(n, pre):
    ts = []
    for _ in range(n):
        pre(); t = time.perf_counter(); w.tick(); ts.append(1000 * (time.perf_counter() - t))
    ts.sort(); return round(ts[len(ts) // 2], 2)
out["tick_ms"] = timed(100, lambda: None)
if v is not None:
    ctl = carla.VehicleControl(throttle=0.2)
    out["tick_after_2_async_ms"] = timed(100, lambda: (v.apply_control(ctl), v.apply_control(ctl)))
    out["tick_after_batch_sync_ms"] = timed(100, lambda: c.apply_batch_sync([carla.command.ApplyVehicleControl(v.id, ctl)]))
    v.destroy()
s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
out["vehicles"] = sorted((b.id, b.get_attribute("number_of_wheels").as_int() if b.has_attribute("number_of_wheels")
                          else 4) for b in w.get_blueprint_library().filter("vehicle.*"))
print("PROBE " + json.dumps(out))
'''


def t_probe(ctx):
    """8. CARLA 1.0 specifics on the loaded twin: World Partition streaming round the spectator, rpclib stall."""
    wd = ctx.a.out_dir / "probe"
    wd.mkdir(parents=True, exist_ok=True)
    (wd / "probe_script.py").write_text(PROBE)
    try:
        out = subprocess.run([ctx.python, str(wd / "probe_script.py"), ctx.a.host, str(ctx.a.port),
                              TWIN_MAP["carla_level"]], cwd=wd,
                             capture_output=True, text=True, timeout=900)
        text = out.stdout + out.stderr
    except subprocess.TimeoutExpired:
        text = "TIMEOUT"
    (wd / "probe.log").write_text(text)
    m = re.search(r"PROBE (\{.*\})", text)
    res = json.loads(m.group(1)) if m else {}
    rec = {"rc": 0 if m else 1, "trial": None, "summary": res, "errors": parse_log(text)[1],
           "server_died": ctx.server.died()}
    if not res:
        return rec, "FAILED", "", notes_of(rec, [text.strip()[-200:]])
    txt = (f"map {res['map'].rsplit('/', 1)[-1]}" + (f" (load_world {res['load_world_s']} s)" if res.get("load_world_s")
                                                     else "") + f"; ground at far point {res['far_point']}: "
           f"{res['ground_far_spectator_away']} with the spectator away, {res['ground_far_spectator_over']} over it "
           f"(streamed in {res['streamed_in_s']} s); tick {res['tick_ms']} ms, after 2 async calls "
           f"{res.get('tick_after_2_async_ms')} ms, after apply_batch_sync {res.get('tick_after_batch_sync_ms')} ms")
    from collections import Counter
    from coupledcarla import soundscape
    kinds = Counter(soundscape.classify(v, w) for v, w in res.get("vehicles", []))
    txt += f"; {len(res.get('vehicles', []))} vehicle blueprints, as the soundscape hears them: " + \
        ", ".join(f"{k} {n}" for k, n in sorted(kinds.items()))
    return rec, "ok", txt, notes_of(rec)


CAMERA_PROBE = r'''
import sys, time
import carla
c = carla.Client(sys.argv[1], int(sys.argv[2])); c.set_timeout(30)
w = c.get_world()
bp = w.get_blueprint_library().find("sensor.camera.rgb")
bp.set_attribute("image_size_x", "320"); bp.set_attribute("image_size_y", "180")
cam = w.spawn_actor(bp, carla.Transform(carla.Location(z=30)))
print("spawned", cam.id, flush=True)
got = []
try:
    cam.listen(lambda im: got.append(im.frame))
except RuntimeError as e:
    print("LISTEN refused:", e, flush=True)
    cam.destroy()
    sys.exit(0)
for _ in range(40):
    w.wait_for_tick(5.0)
print("CAMERA images", len(got), flush=True)
cam.stop(); cam.destroy()
'''


def t_camera_nullrhi(ctx):
    """8b. A camera sensor on the -nullrhi server (expected: no images, or the server dies). Runs last."""
    wd = ctx.a.out_dir / "camera"
    wd.mkdir(parents=True, exist_ok=True)
    (wd / "camera_script.py").write_text(CAMERA_PROBE)
    try:
        out = subprocess.run([ctx.python, str(wd / "camera_script.py"), ctx.a.host, str(ctx.a.port)], cwd=wd,
                             capture_output=True, text=True, timeout=300)
        text = out.stdout + out.stderr
    except subprocess.TimeoutExpired:
        text = "TIMEOUT"
    (wd / "camera.log").write_text(text)
    time.sleep(5)
    died = ctx.server.died()
    m = re.search(r"CAMERA images (\d+)", text)
    rec = {"rc": 0, "trial": None, "summary": {}, "errors": [], "server_died": died}
    refused = re.search(r"LISTEN refused: (.*)", text)
    txt = (f"spawned: {'yes' if 'spawned' in text else 'no'}; listen: "
           f"{'refused (' + refused.group(1)[:90] + ')' if refused else 'ok'}; images {m.group(1) if m else '-'}; "
           f"server {'died: ' + died if died else 'still up'}")
    ok = not died and (refused or (m and int(m.group(1)) > 0))
    return rec, "ok" if ok else "FAILED", txt, "coupled-carla's cameras go on without images then (compat.listen)"


TESTS = {   # name: (function, map key for server restarts, headless, render check)
    "twin": (t_twin, "twin", True, False),
    "probe": (t_probe, "twin", True, False),
    "sound_driver": (t_sound_driver, "twin", True, False),
    "sound_pedestrian": (t_sound_pedestrian, "twin", True, False),
    "weather_real": (t_weather_real, "twin", True, False),
    "night": (t_night, "twin", True, True),
    "season": (t_season, "twin", True, True),
    "drones": (t_drones, "twin", True, True),
    "film": (t_film, "centre", True, True),
    "stations": (t_stations, "centre", True, True),
    "odr_centre": (t_odr_centre, "centre", True, False),
    "odr_mapmaker": (t_odr_mapmaker, "mapmaker", True, False),
    "camera_nullrhi": (t_camera_nullrhi, "camera", True, False),
}
TITLES = {
    "twin": "Eindhoven twin (EhvCentre), --local bots, SUMO, buses, parked, scenery, signals, crowd",
    "probe": "1.0: World Partition streaming, rpclib stall",
    "sound_driver": "Sound --record-audio, driver first",
    "sound_pedestrian": "Sound --record-audio, pedestrian first",
    "weather_real": "Weather: real, time now",
    "night": "Night: street lights, headlights",
    "season": "Season: winter (bare trees)",
    "drones": "Drones: bot pilot + AI missions",
    "film": "Film cameras / make_videos.py",
    "stations": "Stations: station_dryrun.py with bot stations",
    "odr_centre": "OpenDRIVE world: eindhoven_centre",
    "odr_mapmaker": "OpenDRIVE world: map-maker map",
    "camera_nullrhi": "1.0: camera on a -nullrhi server",
}
RENDER = ["film", "stations", "drones", "night", "season"]


def table(results):
    out = ["| Feature | Result | Numbers | Notes |", "|---|---|---|---|"]
    for r in results:
        out.append(f"| {TITLES.get(r['test'], r['test'])} | {r['result']} | {r['numbers']} | {r['notes']} |")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", help="comma-separated tests (default: all headless tests)")
    ap.add_argument("--skip", default="", help="comma-separated tests to leave out")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--duration", type=float, default=45.0, help="seconds per trial")
    ap.add_argument("--short", type=float, default=25.0, help="seconds per weather/season trial")
    ap.add_argument("--render-checks", action="store_true",
                    help="the camera checks (film frames, station images, drone cameras, night frames) for a "
                         "rendering server")
    ap.add_argument("--clip", default="eindhoven_pedestrian_ehmi", help="film: the clip of experiments/videos/clips.yaml")
    ap.add_argument("--mapmaker-exp", help="odr_mapmaker: an experiment written and built by coupledcarla.mapmaker")
    ap.add_argument("--python", help="interpreter with the carla 1.0 wheel (default: this one)")
    ap.add_argument("--out", default=str(ROOT / "logs" / "test10"))
    ap.add_argument("--launch", help="command that runs the server in the foreground (this tool starts, restarts "
                                     "and kills it)")
    ap.add_argument("--max-rss-gb", type=float, default=22.0)
    ap.add_argument("--min-free-gb", type=float, default=10.0)
    ap.add_argument("--need-free-gb", type=float, default=28.0, help="wait for this much available memory "
                                                                     "before each server start")
    ap.add_argument("--wait-free-s", type=int, default=3 * 3600)
    ap.add_argument("--restart-above-gb", type=float, default=17.0)
    a = ap.parse_args(argv)
    if a.list:
        for k, (fn, mk, _, rc) in TESTS.items():
            print(f"{k:18s} {'render ' if rc else '       '} {TITLES[k]}")
        return 0
    names = a.only.split(",") if a.only else (RENDER if a.render_checks else list(TESTS))
    names = [n for n in names if n and n not in a.skip.split(",")]
    for n in names:
        if n not in TESTS:
            ap.error(f"no test {n}; --list shows them")
    a.out_dir = Path(a.out) / time.strftime("%Y%m%d-%H%M%S")
    a.out_dir.mkdir(parents=True, exist_ok=True)
    ctx = Ctx(a)
    started = time.time()
    try:
        for n in names:
            fn, mk, _, _ = TESTS[n]
            t0 = time.time()
            try:
                ctx.server.ensure(mk)
                rec, result, numbers, notes = fn(ctx)
            except Exception as e:  # noqa: BLE001 - one broken test does not stop the matrix
                rec, result, numbers, notes = None, "FAILED", "", f"{type(e).__name__}: {e}"
            row = {"test": n, "result": result, "numbers": numbers, "notes": notes, "seconds": round(time.time() - t0),
                   "trial": (rec or {}).get("trial"), "server_rss_gb": round(ctx.server.rss(), 1) if a.launch else None}
            ctx.results.append(row)
            ctx.log(f"{n}: {result} {numbers} {notes}")
            (a.out_dir / "results.json").write_text(json.dumps(ctx.results, indent=1, default=str))
            if a.launch and ctx.server.died():
                ctx.server.stop()
    finally:
        ctx.server.stop()
    print(f"CARLA 1.0 test matrix, {time.strftime('%Y-%m-%d')}: {len(names)} tests against {a.host}:{a.port}"
          f"{' (render checks)' if a.render_checks else ''}, {a.duration:.0f} s trials, "
          f"{(time.time() - started) / 60:.0f} min; logs in {a.out_dir}\n")
    print(table(ctx.results))
    return 0 if all(r["result"] in ("ok", "skipped") for r in ctx.results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
