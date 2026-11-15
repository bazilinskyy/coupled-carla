"""The station soundscape rendered offline from a trial's logs, for demo videos (tools/make_videos.py).

frames.csv holds every participant, agent and AI actor near a participant at 50 Hz; a film camera's camera.csv (or a
participant's own rows) gives the listener. Each tick builds soundscape.Body records and renders the mixer for exactly
that tick's share of samples, so the sound is aligned with simulation time whatever the run's real-time factor. Used
when the runner cannot record its own audio (no `--record-audio`).
"""
import csv
import math
from bisect import bisect_left
from types import SimpleNamespace

import numpy as np

from . import audio
from .soundscape import Body, LEFT_BLINKER, RIGHT_BLINKER, Soundscape, classify, is_electric

BLINK_BITS = {0: 0, 1: LEFT_BLINKER, 2: RIGHT_BLINKER, 3: LEFT_BLINKER | RIGHT_BLINKER}
DEFAULT_SOUND = {"enabled": True, "reference_db": 94.0, "ambience": {"kind": "city", "db": 50.0},
                 "avas": {"enabled": True, "db": 56.0, "until_kmh": 20.0, "vehicles": "electric"}}


def _f(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def read_frames(path, t0, t1):
    """{frame: (t, [row dict, ...])} for frames.csv rows with t0 <= timestamp <= t1."""
    out = {}
    with open(path, newline="") as fh:
        r = csv.DictReader(fh, delimiter=";")
        for row in r:
            t = _f(row["timestamp"], None)
            if t is None or t < t0 or t > t1:
                continue
            fr = int(row["frame"])
            out.setdefault(fr, (t, []))[1].append(row)
    return out


def body_of_row(row, electric="auto"):
    tid = row.get("type_id", "")
    kind = classify(tid)
    auto = row.get("kind") == "agent"
    ehmi = row.get("ehmi") or None
    return Body(int(row["actor_id"]) if str(row["actor_id"]).isdigit() else hash(row["actor_id"]) % 10 ** 8, tid, kind,
                _f(row["pos_x"]), _f(row["pos_y"]), _f(row["pos_z"]), _f(row["vel_x"]), _f(row["vel_y"]),
                _f(row["vel_z"]), _f(row["rot_yaw"]), kind != "walker" and is_electric(electric, tid, row["actor_id"], auto),
                auto, BLINK_BITS.get(int(_f(row.get("blinkers")) or 0), 0), ehmi)


def read_listener(path):
    """[(t, x, y, z, yaw)] from a film camera.csv."""
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh, delimiter=";"):
            out.append((_f(row["t"]), _f(row["x"]), _f(row["y"]), _f(row["z"]), _f(row["yaw"])))
    return out


def listener_at(track, t):
    """The listener (x, y, z, yaw) at time t, linearly between samples of track [(t, x, y, z, yaw)]."""
    if not track:
        return None
    ts = [p[0] for p in track]
    i = bisect_left(ts, t)
    if i <= 0:
        return tuple(track[0][1:])
    if i >= len(track):
        return tuple(track[-1][1:])
    a, b = track[i - 1], track[i]
    w = (t - a[0]) / max(1e-9, b[0] - a[0])
    yaw = a[4] + ((b[4] - a[4] + 180.0) % 360.0 - 180.0) * w
    return (a[1] + (b[1] - a[1]) * w, a[2] + (b[2] - a[2]) * w, a[3] + (b[3] - a[3]) * w, yaw)


def samples_per_tick(times, t0, rate):
    """Samples to render for each tick time in `times` so that the total after tick i is round((t_i+1 - t0) * rate):
    no drift between the audio and simulation time."""
    out, done = [], 0
    for i, t in enumerate(times):
        t_next = times[i + 1] if i + 1 < len(times) else t + (t - times[i - 1] if i else 0.02)
        want = int(round((t_next - t0) * rate))
        out.append(max(0, want - done))
        done += out[-1]
    return out


def render(frames_csv, t0, t1, listener=None, listener_role=None, own_role=None, sound=None, kind=None,
           rate=audio.RATE, log=print):
    """Stereo float32 (n, 2) of the soundscape from t0 to t1 (trial seconds). listener: a track [(t, x, y, z, yaw)]
    (a film camera); else the rows of `listener_role` (a participant's role or an agent's id; eyes 0.8 m above
    its centre). own_role: whose car is heard from
    inside (a driver's or passenger's camera)."""
    cfg = {**DEFAULT_SOUND, **(sound or {})}
    cfg["enabled"] = True
    frames = read_frames(frames_csv, t0 - 0.1, t1 + 0.1)
    if not frames:
        raise ValueError(f"no rows between {t0} and {t1} s in {frames_csv}")
    order = sorted(frames)
    times = [frames[f][0] for f in order]
    scape = Soundscape(cfg, kind=kind or ("driver" if own_role else "pedestrian"), log=log, rate=rate)
    counts = samples_per_tick(times, t0, rate)
    out = []
    started = False
    for fr, t, n in zip(order, times, counts):
        rows = frames[fr][1]
        bodies, own, ctl, lst = [], None, None, None
        for row in rows:
            if listener_role and row.get("role") == listener_role:     # a participant or an agent
                lst = (_f(row["pos_x"]), _f(row["pos_y"]), _f(row["pos_z"]) + 0.8, _f(row["rot_yaw"]))
            b = body_of_row(row, cfg.get("electric", "auto"))
            if own_role and row.get("role") == own_role:
                own = b
                ctl = SimpleNamespace(throttle=_f(row.get("throttle")), brake=_f(row.get("brake")),
                                      steer=_f(row.get("steer")))
            bodies.append(b)
        if listener is not None:
            lst = listener_at(listener, t)
        if lst is None:
            continue
        scape.update(t, lst, bodies, own=own, own_control=ctl)
        block = scape.mixer.render(n) if n else np.zeros((0, 2), np.float32)
        if t >= t0 - 1e-6:
            started = True
        if started:
            out.append(block)
    x = np.concatenate(out) if out else np.zeros((0, 2), np.float32)
    want = int(round((t1 - t0) * rate))
    if len(x) < want:
        x = np.concatenate([x, np.zeros((want - len(x), 2), np.float32)])
    return x[:want]


def normalise(x, target_rms_db=-20.0, peak=0.89):
    """Scales to an RMS level (dBFS), then softly limits peaks (tanh above `peak`)."""
    r = float(np.sqrt(np.mean(x.astype(np.float64) ** 2))) if len(x) else 0.0
    if r < 1e-7:
        return x
    y = x * (10 ** (target_rms_db / 20.0) / r)
    over = np.abs(y) > peak
    if over.any():
        y = np.where(over, np.sign(y) * (peak + (1 - peak) * np.tanh((np.abs(y) - peak) / (1 - peak))), y)
    return y.astype(np.float32)


def write(path, x, rate=audio.RATE):
    audio.write_wav(path, x, rate)
    return math.ceil(len(x) / rate)
