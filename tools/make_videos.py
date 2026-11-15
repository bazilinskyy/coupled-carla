"""Short demo videos of coupled-carla: run a clip's trial with film cameras (or bot stations), then encode.

    python tools/make_videos.py list                          # the clips in experiments/videos/clips.yaml
    python tools/make_videos.py render tokyo_crowd            # run the trial, check the frames, sound, encode
    python tools/make_videos.py render --all                  # every clip, one after the other
    python tools/make_videos.py encode tokyo_crowd [--crf 27] # encode again from the kept frames (no CARLA)
    python tools/make_videos.py remux tokyo_crowd --audio new.wav   # new sound on the encoded video (no re-encode)
    python tools/make_videos.py crf tokyo_crowd --values 24,26,28,30  # sizes and SSIM per CRF
    python tools/make_videos.py table                         # Markdown table of the encoded clips
    python tools/make_videos.py watchdog --port 2500          # only the memory watchdog (other tools' servers)

A clip (experiments/videos/clips.yaml) names an experiment and trial, overrides (time, weather, traffic, participants via
`set` and `patch`), a camera (coupledcarla/film.py: attach, follow, tripod, orbit, path, drone) or `kind: stations`
(the host plus one bot station per view, composed into a grid), the start in trial time and the length. The trial
runs headless with a film section: frames at `fps` on a grid of simulation time (film.py), so they are evenly spaced
whatever the real-time factor. Sound: the runner's own `--record-audio` (audio.wav from trial time 0) when it has the
flag, else the station soundscape rendered offline from frames.csv at the camera (filmaudio.py). ffmpeg makes the
H.264/AAC mp4 with a small caption, a poster JPEG and an animated WebP preview.

The server: by default the runner starts CARLA itself on --port (2500) and a watchdog kills that server when its
memory passes --max-rss-gb or free memory falls under --min-free-gb (Linux). --no-launch attaches to a server that is
already running on --server-host:--port (e.g. a packaged CARLA 1.0 on Windows with the client in WSL). Run it with the
repo's venv and its env (CARLA_ROOT, SUMO_HOME), e.g. `source env.sh; .venv/bin/python tools/make_videos.py ...`.
"""
import argparse
import copy
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import config as ccconfig  # noqa: E402  (yaml only, no CARLA)

CLIPS = ROOT / "experiments" / "videos" / "clips.yaml"
OVERRIDE_KEYS = ("simulation", "traffic", "participants", "agents", "logging", "coupling", "ehmi", "eye_contact",
                 "sound", "displays", "haptics", "drones")
CLIP_DEFAULTS = {
    "trial": 0, "start_s": 30.0, "duration_s": 20.0, "tail_s": 1.5, "fps": 25, "size": [1280, 720], "fov": 90,
    "kind": "film",            # film: host cameras (film.py) | stations: host + bot stations, views in a grid
    "crf": 28, "preset": "slow", "scale": None,      # scale: [w, h] of the encoded video (default: size)
    "audio": "auto",           # auto: the runner's audio.wav if it records one, else offline | offline | none
    "audio_kbps": 96, "loudness_db": -20.0,
    "caption": True,           # true: title + place / role / time; false; or {title, line}
    "caption_s": 7.0,          # the caption fades out after this many seconds (0: stays)
    "poster_s": None,          # seconds into the clip for the poster (default: the middle)
    "preview": {"start_s": None, "seconds": 3.5, "width": 480, "fps": 12, "quality": 55},
    "local": False,            # run station participants on the host as bots (run --local)
    "real_time": False,        # the runner's real-time sleep (stations: true, so bots keep up)
    "set": {}, "patch": {}, "camera": None, "cameras": [], "hold": {}, "attributes": {},
    "stations": {"views": [], "layout": [2, 2], "control_port": 2510},
}


# -- clip specs -----------------------------------------------------------------------------------------------------

def load_clips(path=CLIPS):
    data = yaml.safe_load(Path(path).read_text()) or {}
    defaults = ccconfig._merge(CLIP_DEFAULTS, data.get("defaults") or {})
    clips = []
    for raw in data.get("clips") or []:
        c = ccconfig._merge(defaults, raw)
        if not c.get("name") or not c.get("experiment"):
            raise ValueError(f"a clip needs a name and an experiment: {raw}")
        if c["kind"] not in ("film", "stations"):
            raise ValueError(f"clip {c['name']}: kind {c['kind']!r} is not film or stations")
        if c["kind"] == "film" and not (c.get("camera") or c.get("cameras")):
            raise ValueError(f"clip {c['name']}: a film clip needs a camera")
        if c["kind"] == "stations" and not c["stations"].get("views"):
            raise ValueError(f"clip {c['name']}: a stations clip needs stations.views")
        c["frames"] = int(round(float(c["duration_s"]) * float(c["fps"])))
        clips.append(c)
    names = [c["name"] for c in clips]
    dup = {n for n in names if names.count(n) > 1}
    if dup:
        raise ValueError(f"clip names used twice: {sorted(dup)}")
    return clips


def find_clip(clips, name):
    for c in clips:
        if c["name"] == name:
            return c
    raise SystemExit(f"no clip {name!r}; clips: {', '.join(c['name'] for c in clips)}")


def cell_size(clip):
    cols, rows = clip["stations"].get("layout") or [2, 2]
    w, h = clip["size"]
    return int(w) // int(cols), int(h) // int(rows)


def film_section(clip):
    out = {"start_s": float(clip["start_s"]), "duration_s": float(clip["duration_s"]), "fps": float(clip["fps"]),
           "size": list(clip["size"]), "fov": float(clip["fov"]), "hold": dict(clip.get("hold") or {}),
           "attributes": dict(clip.get("attributes") or {})}
    if clip.get("camera"):
        out["camera"] = copy.deepcopy(clip["camera"])
    if clip.get("cameras"):
        out["cameras"] = copy.deepcopy(clip["cameras"])
    return out


def derive_experiment(clip, server, root=ROOT):
    """The experiment the clip runs: the base experiment with its chosen trial's overrides folded in, the clip's
    `set` merged over it, then the film section, server, logging and one trial."""
    base = root / clip["experiment"]
    raw = yaml.safe_load(base.read_text()) or {}
    trials = raw.pop("trials", None) or [{"duration_s": 300}]
    trial = dict(trials[int(clip["trial"])])
    for k in ("conditions", "design", "dashboard"):
        raw.pop(k, None)
    exp = raw
    exp["map"] = dict(exp.get("map") or {})
    exp["map"].setdefault("name", raw.get("name"))
    for k in OVERRIDE_KEYS:
        if k in trial:
            exp[k] = ccconfig._merge(exp.get(k) or {}, trial[k]) if isinstance(trial[k], dict) else \
                copy.deepcopy(trial[k])
    exp = ccconfig._merge(exp, clip.get("set") or {})
    exp["name"] = f"video_{clip['name']}"
    sim = exp.setdefault("simulation", {})
    if clip.get("time") is not None:
        sim["time"] = clip["time"]
    if clip.get("weather") is not None:
        sim["weather"] = clip["weather"]
    sim["real_time"] = bool(clip["real_time"] if clip["kind"] == "film" else True)
    exp["server"] = ccconfig._merge(exp.get("server") or {}, server)
    lg = exp.setdefault("logging", {})
    lg.update(carla_recorder=False, frames_every_s=0.0, analyse=False)
    lg["radius_m"] = max(float(lg.get("radius_m", 150)), 220.0)
    if clip["kind"] == "film":
        exp["film"] = film_section(clip)
    else:      # the stations' cameras take the film attributes (motion blur, exposure) from their welcome
        exp["film"] = {"attributes": dict(clip.get("attributes") or {})}
        exp.setdefault("coupling", {})["port"] = int(clip["stations"]["control_port"])
        exp["coupling"]["wait_for_stations_s"] = 300
        exp["coupling"]["visual_sync_at_s"] = []
    t = {"duration_s": round(float(clip["start_s"]) + float(clip["duration_s"]) + float(clip["tail_s"]), 2)}
    if clip.get("patch"):
        t["patch"] = copy.deepcopy(clip["patch"])
    exp["trials"] = [t]
    return exp


def caption_lines(clip):
    cap = clip.get("caption")
    if not cap:
        return None
    if isinstance(cap, dict):
        return cap.get("title", clip.get("title", "")), cap.get("line", "")
    bits = [clip.get("place"), clip.get("role"), clip.get("when")]
    return clip.get("title", clip["name"]), "  ·  ".join(b for b in bits if b)


# -- ffmpeg commands (pure: tests) --------------------------------------------------------------------------------

def find_font(bold=False):
    names = (["DejaVuSans-Bold.ttf", "LiberationSans-Bold.ttf", "Arial Bold.ttf", "arialbd.ttf"] if bold else
             ["DejaVuSans.ttf", "LiberationSans-Regular.ttf", "Arial.ttf", "arial.ttf"])
    dirs = ["/usr/share/fonts", "/usr/local/share/fonts", "/Library/Fonts", "/System/Library/Fonts",
            "C:/Windows/Fonts", str(Path.home() / ".fonts")]
    for d in dirs:
        for n in names:
            hits = list(Path(d).rglob(n)) if Path(d).is_dir() else []
            if hits:
                return str(hits[0])
    return None


def _esc_path(p):
    return str(p).replace("\\", "/").replace(":", "\\:").replace("'", "\\'")


def caption_filter(title_file, line_file, out_h, font, font_bold, fade_s=7.0, position="bottom-left"):
    """drawtext filters for a two-line lower third (text read from files: no escaping of the captions)."""
    s = out_h / 720.0
    big, small = max(12, round(26 * s)), max(10, round(18 * s))
    pad = round(10 * s)
    x = f"{round(28 * s)}" if position == "bottom-left" else "(w-text_w)/2"
    y1, y2 = f"h-{round(84 * s)}", f"h-{round(46 * s)}"
    alpha = "1" if not fade_s else \
        f"'if(lt(t,0.4),t/0.4,if(lt(t,{fade_s:g}),1,if(lt(t,{fade_s + 0.8:g}),({fade_s + 0.8:g}-t)/0.8,0)))'"
    common = f"expansion=none:fontcolor=white:box=1:boxcolor=black@0.45:boxborderw={pad}:alpha={alpha}"
    out = []
    if title_file:
        out.append(f"drawtext=fontfile='{_esc_path(font_bold or font)}':textfile='{_esc_path(title_file)}':"
                   f"fontsize={big}:x={x}:y={y1}:{common}")
    if line_file:
        out.append(f"drawtext=fontfile='{_esc_path(font)}':textfile='{_esc_path(line_file)}':"
                   f"fontsize={small}:x={x}:y={y2}:{common}")
    return out


def encode_cmd(frames_pattern, out, fps, crf, preset="slow", audio=None, audio_ss=0.0, duration=None,
               scale=None, filters=(), audio_kbps=96, start_number=0, ffmpeg="ffmpeg"):
    """The ffmpeg command for one clip: an image sequence (+ a WAV) -> H.264 + AAC mp4 for the web."""
    cmd = [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-framerate", f"{fps:g}",
           "-start_number", str(start_number), "-i", str(frames_pattern)]
    if audio:
        if audio_ss:
            cmd += ["-ss", f"{audio_ss:.3f}"]
        if duration:
            cmd += ["-t", f"{duration:.3f}"]
        cmd += ["-i", str(audio)]
    vf = []
    if scale:
        vf.append(f"scale={int(scale[0])}:{int(scale[1])}:flags=lanczos")
    vf += list(filters)
    vf.append("format=yuv420p")
    cmd += ["-vf", ",".join(vf), "-c:v", "libx264", "-preset", preset, "-crf", str(crf), "-pix_fmt", "yuv420p",
            "-profile:v", "high", "-movflags", "+faststart"]
    if duration:
        cmd += ["-t", f"{duration:.3f}"]
    if audio:
        cmd += ["-map", "0:v:0", "-map", "1:a:0", "-c:a", "aac", "-b:a", f"{int(audio_kbps)}k", "-ac", "2",
                "-ar", "44100", "-shortest"]
    else:
        cmd += ["-an"]
    return cmd + [str(out)]


def poster_cmd(frame, out, width=640, q=4, ffmpeg="ffmpeg"):
    return [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-i", str(frame), "-vf",
            f"scale={width}:-2:flags=lanczos", "-q:v", str(q), str(out)]


def preview_cmd(frames_pattern, out, fps_in, start_number, seconds, width=480, fps=12, quality=55,
                ffmpeg="ffmpeg"):
    return [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-framerate", f"{fps_in:g}",
            "-start_number", str(start_number), "-i", str(frames_pattern), "-t", f"{seconds:g}",
            "-vf", f"fps={fps},scale={width}:-2:flags=lanczos", "-c:v", "libwebp", "-lossless", "0",
            "-q:v", str(quality), "-compression_level", "6", "-loop", "0", "-an", str(out)]


def remux_cmd(video, audio, out, audio_ss=0.0, duration=None, audio_kbps=96, ffmpeg="ffmpeg"):
    cmd = [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-i", str(video)]
    if audio_ss:
        cmd += ["-ss", f"{audio_ss:.3f}"]
    cmd += ["-i", str(audio), "-map", "0:v:0", "-map", "1:a:0", "-c:v", "copy", "-c:a", "aac",
            "-b:a", f"{int(audio_kbps)}k", "-ac", "2", "-ar", "44100", "-shortest", "-movflags", "+faststart"]
    if duration:
        cmd += ["-t", f"{duration:.3f}"]
    return cmd + [str(out)]


# -- frames ---------------------------------------------------------------------------------------------------------

def fill_gaps(folder, n, pattern="{:06d}.jpg"):
    """Makes NNNNNN.jpg exist for 0..n-1 (ffmpeg's image2 stops at a gap): a missing frame repeats the one before
    (a hard link, else a copy). Returns the indices filled."""
    folder = Path(folder)
    filled = []
    last = None
    for k in range(n):
        p = folder / pattern.format(k)
        if p.exists():
            last = p
            continue
        src = last or next((folder / pattern.format(j) for j in range(k + 1, n) if (folder / pattern.format(j)).exists()),
                           None)
        if src is None:
            raise RuntimeError(f"no frames at all in {folder}")
        try:
            os.link(src, p)
        except OSError:
            shutil.copyfile(src, p)
        filled.append(k)
        last = p
    return filled


def frame_numbers(folder):
    return sorted(int(p.stem) for p in Path(folder).glob("*.jpg") if p.stem.isdigit())


def trial_frame0(frames_csv, step):
    """The CARLA frame of trial time 0, from frames.csv's first row."""
    with open(frames_csv, newline="") as fh:
        r = csv.DictReader(fh, delimiter=";")
        row = next(r)
    return int(row["frame"]) - int(round(float(row["timestamp"]) / step))


def station_plan(views_frames, frame0, start_s, n, fps, step):
    """For each output frame, the frame file of each view: {role: [frame number or None] * n}. The grid starts at the
    trial frame of start_s, moved by one tick if the stations' common grid has the other parity."""
    per = int(round(1.0 / (fps * step)))
    f_start = frame0 + int(round(start_s / step))
    first = next((fr for fr in views_frames.values() if fr), [])
    if per > 1 and first:
        options = [f_start + d for d in range(per)]
        f_start = max(options, key=lambda f: sum(1 for x in first if (x - f) % per == 0))
    plan = {}
    for role, frames in views_frames.items():
        have = set(frames)
        plan[role] = [(f_start + k * per) if (f_start + k * per) in have else None for k in range(n)]
    return plan, f_start


def compose_grid(plan, folders, labels, out_dir, size, layout, font=None, quality=92):
    """Writes the grid frames NNNNNN.jpg: each view in its cell (a missing image repeats that view's last one),
    labelled at the top left. Returns {role: frames repeated}."""
    from PIL import Image, ImageDraw, ImageFont
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cols, rows = layout
    w, h = size
    cw, ch = w // cols, h // rows
    roles = list(plan)
    n = len(plan[roles[0]])
    fnt = ImageFont.truetype(font, max(12, round(20 * h / 720))) if font else ImageFont.load_default()
    last, repeats = {}, {r: 0 for r in roles}
    for k in range(n):
        canvas = Image.new("RGB", (w, h), (12, 12, 14))
        draw = ImageDraw.Draw(canvas)
        for i, role in enumerate(roles):
            fr = plan[role][k]
            img = None
            if fr is not None:
                p = Path(folders[role]) / f"{fr:08d}.jpg"
                if p.exists():
                    try:
                        img = Image.open(p).convert("RGB")
                    except OSError:
                        img = None
            if img is None:
                img = last.get(role)
                repeats[role] += 1
            if img is None:
                continue
            last[role] = img
            if img.size != (cw, ch):
                img = img.resize((cw, ch), Image.LANCZOS)
            x, y = (i % cols) * cw, (i // cols) * ch
            canvas.paste(img, (x, y))
            text = labels.get(role, role)
            tb = draw.textbbox((0, 0), text, font=fnt)
            pad = max(4, round(7 * h / 720))
            bx, by = x + pad * 2, y + pad * 2
            draw.rectangle([bx - pad, by - pad, bx + tb[2] + pad, by + tb[3] + pad], fill=(0, 0, 0))
            draw.text((bx, by), text, font=fnt, fill=(255, 255, 255))
        for c in range(1, cols):
            draw.line([(c * cw, 0), (c * cw, h)], fill=(12, 12, 14), width=2)
        for r in range(1, rows):
            draw.line([(0, r * ch), (w, r * ch)], fill=(12, 12, 14), width=2)
        canvas.save(out_dir / f"{k:06d}.jpg", quality=quality)
    return repeats


# -- running -------------------------------------------------------------------------------------------------------

class Watchdog(threading.Thread):
    """Kills this user's CARLA server on `port` when its resident memory passes rss_gb or the machine's available
    memory falls under free_gb (Linux /proc); remembers the largest RSS seen."""

    def __init__(self, port, rss_gb=12.0, free_gb=4.0, log=print):
        super().__init__(daemon=True)
        self.port, self.rss_gb, self.free_gb, self.log = port, rss_gb, free_gb, log
        self.stop_ev = threading.Event()
        self.max_rss_gb = 0.0
        self.killed = None

    def pids(self):
        out = []
        for d in Path("/proc").iterdir():
            if not d.name.isdigit():
                continue
            try:
                if d.stat().st_uid != os.getuid():
                    continue
                cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if f"carla-rpc-port={self.port}" in cmd and ("Shipping" in cmd or "CarlaUnreal" in cmd) \
                    and "CarlaUE4.sh" not in cmd and "CarlaUnreal.sh" not in cmd:
                out.append(int(d.name))
        return out

    @staticmethod
    def rss_gb_of(pid):
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024 ** 2
        return 0.0

    @staticmethod
    def available_gb():
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024 ** 2
        return 1e9

    def run(self):
        while not self.stop_ev.wait(2.0):
            try:
                avail = self.available_gb()
                for pid in self.pids():
                    rss = self.rss_gb_of(pid)
                    self.max_rss_gb = max(self.max_rss_gb, rss)
                    if rss > self.rss_gb or avail < self.free_gb:
                        os.kill(pid, signal.SIGKILL)
                        self.killed = f"pid {pid}: RSS {rss:.1f} GB, available {avail:.1f} GB"
                        self.log(f"watchdog: killed the CARLA server ({self.killed})")
            except OSError:
                pass

    def stop(self):
        self.stop_ev.set()


def runner_has_flag(python, flag, cwd=ROOT):
    try:
        out = subprocess.run([python, "-m", "coupledcarla.run", "--help"], cwd=cwd, capture_output=True, text=True,
                             timeout=120).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return flag in out


def kill_server(port):
    subprocess.call(["pkill", "-9", "-f", f"(CarlaUE4|CarlaUnreal)-Linux-Shipping.*carla-rpc-port={port}"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Ctx:
    def __init__(self, args):
        self.args = args
        self.python = args.python or sys.executable
        self.work = Path(args.work).resolve()
        self.out = Path(args.out).resolve()
        self.ffmpeg = args.ffmpeg
        self.font = args.font or find_font()
        self.font_bold = args.font_bold or find_font(bold=True) or self.font
        self.server = {"host": args.server_host, "port": args.port, "launch": not args.no_launch,
                       "quality": args.quality, "tm_port": args.port + 50}
        self._audio_flag = None

    def log(self, msg):
        print(f"[videos {time.strftime('%H:%M:%S')}] {msg}", flush=True)

    def record_audio(self):
        if self._audio_flag is None:
            self._audio_flag = runner_has_flag(self.python, "--record-audio")
            self.log("the runner records audio (--record-audio)" if self._audio_flag else
                     "the runner has no --record-audio: offline soundscape from the logs (filmaudio.py)")
        return self._audio_flag


START_FAILURES = ("waiting for the simulator", "CARLA server exited during start-up", "no CARLA server on")


def run_clip(ctx, clip, attempts=3):
    """Runs the clip's trial (and stations), again when the CARLA server failed to start (CARLA 0.9.16 now and then
    crashes while starting); returns the run record (also written to work/<clip>/run.json)."""
    for k in range(attempts):
        rec = _run_clip(ctx, clip)
        log = (ctx.work / clip["name"] / "host.log").read_text(errors="replace")
        started = "trial done:" in log or "film: camera" in log
        if rec["host_rc"] == 0 or started or not any(m in log for m in START_FAILURES) or k == attempts - 1:
            return rec
        ctx.log(f"{clip['name']}: the CARLA server did not come up; trying again")
        if ctx.server["launch"]:
            kill_server(ctx.server["port"])
            time.sleep(5)
    return rec


def hears_first_participant(clip):
    """Whether the clip's sound is what the first participant hears (the runner's --record-audio): first-person
    cameras (a drone's FPV, an attached camera, a follow camera within 1.5 m of its target: a pedestrian's eyes) and
    station grids (the first station's ears). Chase, orbit, tripod and crane cameras hear the street where the
    camera is instead (filmaudio.py offline from frames.csv): a recording from inside the car would sound wrong under
    an outside view."""
    if clip.get("kind") == "stations":
        return True
    cam = clip.get("camera") or {}
    kind = cam.get("kind")
    if kind in ("attach", "drone"):
        return kind != "drone" or cam.get("view", "fpv") == "fpv"
    if kind == "follow":
        off = cam.get("offset") or [0, 0, 0]
        return sum(float(v) ** 2 for v in off) ** 0.5 <= 1.5
    return False


def _run_clip(ctx, clip):
    wd = ctx.work / clip["name"]
    if wd.exists():
        shutil.rmtree(wd)
    wd.mkdir(parents=True)
    exp = derive_experiment(clip, ctx.server)
    exp_path = wd / "experiment.yaml"
    exp_path.write_text(yaml.safe_dump(exp, sort_keys=False, allow_unicode=True))
    argv = [ctx.python, "-m", "coupledcarla.run", str(exp_path), "--trial", "0", "--headless"]
    if clip["local"]:
        argv.append("--local")
    want_audio = clip["audio"] == "auto" and hears_first_participant(clip) and ctx.record_audio()
    if want_audio:
        argv.append("--record-audio")
    dog = None
    if ctx.server["launch"] and sys.platform.startswith("linux") and ctx.args.need_free_gb > 0:
        t_wait = time.time()       # a shared machine: start a server only with room for it
        while Watchdog.available_gb() < ctx.args.need_free_gb:
            if time.time() - t_wait > 3 * 3600:
                raise RuntimeError(f"under {ctx.args.need_free_gb:g} GB free for 3 hours")
            if int(time.time() - t_wait) % 300 < 10:
                ctx.log(f"{clip['name']}: {Watchdog.available_gb():.1f} GB free, waiting for "
                        f"{ctx.args.need_free_gb:g} GB before starting CARLA")
            time.sleep(10)
    if ctx.server["launch"] and sys.platform.startswith("linux") and not ctx.args.no_watchdog:
        dog = Watchdog(ctx.server["port"], ctx.args.max_rss_gb, ctx.args.min_free_gb, ctx.log)
        dog.start()
    ctx.log(f"{clip['name']}: running {' '.join(argv[1:])}")
    t0 = time.time()
    host_log = open(wd / "host.log", "w")
    host = subprocess.Popen(argv, cwd=ROOT, stdout=host_log, stderr=subprocess.STDOUT, start_new_session=True)
    stations = []
    if clip["kind"] == "stations":
        cw, ch = cell_size(clip)
        st = clip["stations"]
        for v in st["views"]:
            sargv = [ctx.python, "-m", "coupledcarla.station", "--host", ctx.server["host"], "--port",
                     str(ctx.server["port"]), "--control-host", "127.0.0.1", "--control-port", str(st["control_port"]),
                     "--role", v["role"], "--input", "bot", "--headless", "--frames-every", f"{1.0 / clip['fps']:g}",
                     "--camera-hz", "0", "--size", f"{cw}x{ch}", "--fov", str(v.get("fov", clip["fov"])),
                     "--out", str(wd / "st")]
            lf = open(wd / f"station_{v['role']}.log", "w")
            stations.append((subprocess.Popen(sargv, cwd=ROOT, stdout=lf, stderr=subprocess.STDOUT,
                                              start_new_session=True), lf))
    timeout = (float(clip["start_s"]) + float(clip["duration_s"])) * 12 + 900
    try:
        host.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        ctx.log(f"{clip['name']}: timed out after {timeout:.0f} s")
        os.killpg(host.pid, signal.SIGKILL)
        host.wait()
        if ctx.server["launch"]:
            kill_server(ctx.server["port"])
    finally:
        host_log.close()
        for p, lf in stations:
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
            lf.close()
        if dog is not None:
            dog.stop()
    text = (wd / "host.log").read_text(errors="replace")
    m = re.search(r"trial \d+ -> (\S+)", text)
    if not m:
        raise RuntimeError(f"{clip['name']}: the trial did not start; see {wd / 'host.log'}")
    trial = Path(m.group(1))
    summary = json.loads((trial / "summary.json").read_text()) if (trial / "summary.json").exists() else {}
    rec = {"clip": clip["name"], "trial_dir": str(trial), "kind": clip["kind"], "host_rc": host.returncode,
           "wall_s": round(time.time() - t0, 1), "rtf": summary.get("rtf"), "sim_s": summary.get("sim_s"),
           "audio_recorded": bool(want_audio), "max_server_rss_gb": round(dog.max_rss_gb, 2) if dog else None,
           "watchdog_killed": dog.killed if dog else None}
    (wd / "run.json").write_text(json.dumps(rec, indent=2))
    ctx.log(f"{clip['name']}: trial done in {rec['wall_s']} s, RTF {rec['rtf']}, rc {host.returncode}")
    return rec


def collect_frames(ctx, clip, rec):
    """The folder of NNNNNN.jpg frames for the encoder and the frame check."""
    trial = Path(rec["trial_dir"])
    wd = ctx.work / clip["name"]
    step = float(yaml.safe_load((trial / "config.yaml").read_text())["simulation"]["step"])
    n = clip["frames"]
    if clip["kind"] == "film":
        d = trial / "film" / ((clip.get("camera") or clip["cameras"][0]).get("name", "main"))
        film = json.loads((trial / "film" / "film.json").read_text())
        chk = film["cameras"][d.name]
        filled = fill_gaps(d, n)
        return d, {"step": step, "check": chk, "filled": len(filled)}
    st = clip["stations"]
    frame0 = trial_frame0(trial / "frames.csv", step)
    folders = {v["role"]: wd / "st" / "stations" / f"{v['role']}_trial0_frames" for v in st["views"]}
    have = {r: frame_numbers(f) if f.is_dir() else [] for r, f in folders.items()}
    plan, f_start = station_plan(have, frame0, float(clip["start_s"]), n, float(clip["fps"]), step)
    per_view = {}
    for r, fr in plan.items():
        window = [x for x in have[r] if f_start <= x < f_start + n * round(1 / (clip["fps"] * step))]
        steps = sorted({b - a for a, b in zip(window, window[1:])})
        per_view[r] = {"frames_in_clip": sum(x is not None for x in fr), "of": n, "frame_steps": steps,
                       "frames_saved_total": len(have[r])}
    labels = {v["role"]: v.get("label", v["role"]) for v in st["views"]}
    repeats = compose_grid(plan, folders, labels, wd / "composite", tuple(clip["size"]), tuple(st["layout"]),
                           ctx.font_bold)
    for r in per_view:
        per_view[r]["repeated"] = repeats[r]
    ctx.log(f"{clip['name']}: views {json.dumps(per_view)}")
    return wd / "composite", {"step": step, "views": per_view, "frame_start": f_start}


def make_audio(ctx, clip, rec, frames_info):
    """The clip's sound as (wav path, seconds to skip in it), or (None, 0)."""
    if clip["audio"] == "none":
        return None, 0.0
    trial = Path(rec["trial_dir"])
    if rec.get("audio_recorded") and (trial / "audio.wav").exists():
        return trial / "audio.wav", float(clip["start_s"])
    from coupledcarla import filmaudio
    cfgd = yaml.safe_load((trial / "config.yaml").read_text())
    sound = {k: v for k, v in (cfgd.get("sound") or {}).items() if k not in ("record", "enabled")}
    sound = ccconfig._merge(sound, clip.get("sound") or {})
    t0, t1 = float(clip["start_s"]), float(clip["start_s"]) + float(clip["duration_s"])
    listener, listener_role, own = None, None, clip.get("own")
    if clip["kind"] == "film":
        cam = clip.get("camera") or clip["cameras"][0]
        listener = filmaudio.read_listener(trial / "film" / cam.get("name", "main") / "camera.csv")
        if own is None and cam.get("kind") == "attach" and cam.get("mount") in ("driver", "passenger"):
            own = cam.get("target")
    else:
        v0 = clip["stations"]["views"][0]
        own = own or v0.get("own")
        listener_role = own or v0["role"]       # an AV passenger has no rows of its own: its car's
    x = filmaudio.render(trial / "frames.csv", t0, t1, listener=listener, listener_role=listener_role,
                         own_role=own, sound=sound, log=lambda m: None)
    x = filmaudio.normalise(x, float(clip["loudness_db"]))
    path = ctx.work / clip["name"] / "audio.wav"
    filmaudio.write(path, x)
    return path, 0.0


def encode_clip(ctx, clip, rec=None, crf=None, out_name=None, frames=None):
    """Frames + sound -> out/<clip>.mp4, .jpg (poster), .webp (preview), .json (what went in)."""
    wd = ctx.work / clip["name"]
    rec = rec or json.loads((wd / "run.json").read_text())
    if frames is None:
        frames = collect_frames(ctx, clip, rec)
    fdir, info = frames
    ctx.out.mkdir(parents=True, exist_ok=True)
    name = out_name or clip["name"]
    crf = crf if crf is not None else clip["crf"]
    audio, ss = make_audio(ctx, clip, rec, info)
    filters = []
    cap = caption_lines(clip)
    if cap and ctx.font:
        (wd / "title.txt").write_text(cap[0])
        (wd / "line.txt").write_text(cap[1])
        out_h = (clip.get("scale") or clip["size"])[1]
        filters = caption_filter(wd / "title.txt" if cap[0] else None, wd / "line.txt" if cap[1] else None, out_h,
                                 ctx.font, ctx.font_bold, float(clip["caption_s"]),
                                 "bottom-centre" if clip["kind"] == "stations" and
                                 cell_size(clip) != tuple(clip["size"]) else "bottom-left")
    mp4 = ctx.out / f"{name}.mp4"
    cmd = encode_cmd(fdir / "%06d.jpg", mp4, float(clip["fps"]), crf, clip["preset"], audio, ss,
                     float(clip["duration_s"]), clip.get("scale"), filters, clip["audio_kbps"], ffmpeg=ctx.ffmpeg)
    subprocess.run(cmd, check=True)
    res = {"clip": clip["name"], "file": mp4.name, "bytes": mp4.stat().st_size, "crf": crf,
           "duration_s": float(clip["duration_s"]), "fps": clip["fps"],
           "size": list(clip.get("scale") or clip["size"]), "audio": ("recorded" if rec.get("audio_recorded") else
                                                                        "offline soundscape" if audio else "none"),
           "frames": info, "run": rec, **{k: clip.get(k) for k in ("title", "place", "role", "agents", "notice",
                                                                  "when")}}
    res["camera"] = (clip.get("camera") or {}).get("kind") if clip["kind"] == "film" else \
        f"{len(clip['stations']['views'])} station views"
    if out_name is None:
        n = clip["frames"]
        k = int(round(float(clip["poster_s"]) * clip["fps"])) if clip.get("poster_s") is not None else n // 2
        jpg = ctx.out / f"{name}.jpg"
        for q in (3, 4, 5, 6, 8, 10, 13):
            subprocess.run(poster_cmd(fdir / f"{min(k, n - 1):06d}.jpg", jpg, 640, q, ctx.ffmpeg), check=True)
            if jpg.stat().st_size < 80_000:
                break
        pv = clip["preview"]
        ps = pv.get("start_s")
        start = int(round((float(ps) if ps is not None else max(0.0, float(clip["duration_s"]) / 2 - 1.5))
                          * clip["fps"]))
        webp = ctx.out / f"{name}.webp"
        for q, wpx in ((pv["quality"], pv["width"]), (45, pv["width"]), (40, 400), (35, 360)):
            subprocess.run(preview_cmd(fdir / "%06d.jpg", webp, float(clip["fps"]), start, float(pv["seconds"]), wpx,
                                       pv["fps"], q, ctx.ffmpeg), check=True)
            if webp.stat().st_size < 1_500_000:
                break
        res.update(poster_bytes=jpg.stat().st_size, preview_bytes=webp.stat().st_size)
        (ctx.out / f"{name}.json").write_text(json.dumps(res, indent=2, default=str))
    ctx.log(f"{name}: {mp4.stat().st_size / 1e6:.2f} MB at CRF {crf}")
    return res


def ssim_of(ctx, mp4, fdir, fps):
    cmd = [ctx.ffmpeg, "-hide_banner", "-i", str(mp4), "-framerate", f"{fps:g}", "-i", str(fdir / "%06d.jpg"),
           "-lavfi", "[0:v]format=yuv420p[a];[1:v]format=yuv420p[b];[a][b]ssim", "-f", "null", "-"]
    err = subprocess.run(cmd, capture_output=True, text=True).stderr
    m = re.search(r"All:([0-9.]+)", err)
    return float(m.group(1)) if m else None


def table(results):
    lines = ["| | Clip | Place | Role | What to notice | Length | Size |", "|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| ![{r['clip']}]({r['clip']}.webp) | [{r.get('title') or r['clip']}]({r['file']}) | "
                     f"{r.get('place') or ''} | {r.get('role') or ''} | {r.get('notice') or ''} | "
                     f"{r['duration_s']:.0f} s | {r['bytes'] / 1e6:.1f} MB |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["list", "render", "encode", "remux", "crf", "table", "experiment",
                                        "watchdog"])
    ap.add_argument("clips", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--file", default=str(CLIPS), help="clip list (default experiments/videos/clips.yaml)")
    ap.add_argument("--work", default=str(ROOT / "videos" / "work"), help="trial logs links, composites, sound")
    ap.add_argument("--out", default=str(ROOT / "videos" / "out"), help="mp4, poster jpg, preview webp, json")
    ap.add_argument("--python", help="the interpreter with the carla package (default: this one)")
    ap.add_argument("--ffmpeg", default="ffmpeg")
    ap.add_argument("--font"), ap.add_argument("--font-bold")
    ap.add_argument("--server-host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2500, help="CARLA RPC port")
    ap.add_argument("--no-launch", action="store_true", help="attach to a running server (server.launch: false)")
    ap.add_argument("--quality", default="Epic", choices=["Epic", "Low"])
    ap.add_argument("--max-rss-gb", type=float, default=12.0)
    ap.add_argument("--min-free-gb", type=float, default=4.0)
    ap.add_argument("--no-watchdog", action="store_true")
    ap.add_argument("--need-free-gb", type=float, default=12.0,
                    help="wait for this much free memory before the runner starts CARLA (Linux; 0: do not wait)")
    ap.add_argument("--crf", type=int, help="encode/remux: override the clip's CRF")
    ap.add_argument("--values", default="24,26,28,30", help="crf: the CRF values to try")
    ap.add_argument("--audio", help="remux: the WAV to put on the video (default: the trial's audio.wav)")
    ap.add_argument("--clean", action="store_true", help="render: delete the frames after encoding")
    ap.add_argument("--fps", type=float, help="render: override the clip's fps (e.g. 2 for a framing check)")
    ap.add_argument("--start", type=float, help="render: override start_s")
    ap.add_argument("--duration", type=float, help="render: override duration_s")
    ap.add_argument("--no-encode", action="store_true", help="render: run and keep the frames only")
    args = ap.parse_args(argv)
    ctx = Ctx(args)
    if args.command == "watchdog":         # on its own, for servers started by other tools (scenery_shots.py)
        dog = Watchdog(args.port, args.max_rss_gb, args.min_free_gb, ctx.log)
        ctx.log(f"watchdog on port {args.port}: RSS > {args.max_rss_gb:g} GB or free < {args.min_free_gb:g} GB")
        dog.run()
        return 0
    clips = load_clips(args.file)
    chosen = clips if args.all else [find_clip(clips, n) for n in args.clips]
    for c in chosen:
        for key, v in (("fps", args.fps), ("start_s", args.start), ("duration_s", args.duration)):
            if v is not None:
                c[key] = v
        c["frames"] = int(round(float(c["duration_s"]) * float(c["fps"])))
    if args.command == "list":
        for c in clips:
            print(f"{c['name']:28s} {c['kind']:8s} {c['experiment']:42s} t {c['start_s']:g}+{c['duration_s']:g} s  "
                  f"{(c.get('camera') or {}).get('kind', '') if c['kind'] == 'film' else 'stations'}")
        return 0
    if args.command == "experiment":
        for c in chosen:
            print(yaml.safe_dump(derive_experiment(c, ctx.server), sort_keys=False, allow_unicode=True))
        return 0
    if args.command == "table":
        res = [json.loads(p.read_text()) for p in sorted(ctx.out.glob("*.json"))]
        order = [c["name"] for c in clips]
        res.sort(key=lambda r: order.index(r["clip"]) if r["clip"] in order else 99)
        print(table(res))
        return 0
    failed = []
    for c in chosen:
        try:
            if args.command == "render":
                rec = run_clip(ctx, c)
                if args.no_encode:
                    continue
                encode_clip(ctx, c, rec, crf=args.crf)
                if args.clean:
                    shutil.rmtree(Path(rec["trial_dir"]) / "film", ignore_errors=True)
                    for sub in ("st", "composite"):
                        shutil.rmtree(ctx.work / c["name"] / sub, ignore_errors=True)
            elif args.command == "encode":
                encode_clip(ctx, c, crf=args.crf)
            elif args.command == "remux":
                rec = json.loads((ctx.work / c["name"] / "run.json").read_text())
                wav = Path(args.audio) if args.audio else Path(rec["trial_dir"]) / "audio.wav"
                ss = 0.0 if args.audio else float(c["start_s"])
                src = ctx.out / f"{c['name']}.mp4"
                tmp = ctx.out / f"{c['name']}.remux.mp4"
                subprocess.run(remux_cmd(src, wav, tmp, ss, float(c["duration_s"]), c["audio_kbps"], ctx.ffmpeg),
                               check=True)
                tmp.replace(src)
                ctx.log(f"{c['name']}: new sound from {wav}")
            elif args.command == "crf":
                rec = json.loads((ctx.work / c["name"] / "run.json").read_text())
                frames = collect_frames(ctx, c, rec)
                print(f"{c['name']}: CRF, size, SSIM against the frames")
                for v in [int(x) for x in args.values.split(",")]:
                    r = encode_clip(ctx, c, rec, crf=v, out_name=f"{c['name']}_crf{v}", frames=frames)
                    mp4 = ctx.out / r["file"]
                    print(f"  {v}  {r['bytes'] / 1e6:6.2f} MB  SSIM {ssim_of(ctx, mp4, frames[0], c['fps'])}", flush=True)
        except Exception as e:  # noqa: BLE001 - one bad clip does not stop the others
            ctx.log(f"{c['name']}: FAILED ({e})")
            failed.append(c["name"])
    if failed:
        ctx.log(f"failed: {', '.join(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
