"""Audio recording on the host, for videos: `python -m coupledcarla.run <exp> --headless --record-audio` (or
`sound.record: true`) renders the soundscape heard by the FIRST participant, the one whose view --frames-every saves
(a driver inside the car, an AV passenger in the passenger seat, a pedestrian at the walker's head, a drone pilot at
the drone's camera), in simulation time, and writes it to the trial's log folder:

- audio.wav: 44.1 kHz, stereo, 16-bit PCM. Sample 0 is simulation time t = 0 of the trial (the first tick of the
  trial loop, t in frames.csv and events.csv). Each tick of `step` seconds renders exactly the samples of
  [t, t + step) from the scene at t, so the file stays in step with the saved frames however slowly the run goes.
- audio.json: the alignment. A camera frame number f (view_<f>.jpg, drone_<role>_<f>.jpg) is at
  t = (f - start_frame) * step_s and at sample round(t * 44100); frames before start_frame precede the audio.
  summary.json gets `audio_start_sim_s` (0.0) and the same record under `audio`.
- audio_credits.txt: the attribution of every recording (and HRTF set) the soundscape used, if any.

Nothing here needs a sound device. The per-tick cost is measured and written to audio.json.
"""
import json
import math
import time
import wave
from pathlib import Path

import numpy as np

from . import acoustics as ac
from . import audio

RATE = audio.RATE


# -- level measurement (listen-proxy checks, tests) -----------------------------------------------------------------

def a_weighting_db(f):
    """IEC 61672-1 A-weighting (dB) at frequencies f."""
    f = np.maximum(np.asarray(f, float), 1e-3)
    f2 = f * f
    ra = (12194.0 ** 2 * f2 * f2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) *
                                     (f2 + 12194.0 ** 2))
    return 20.0 * np.log10(ra) + 2.0


def level_dba(x, reference_db=94.0, rate=RATE):
    """A-weighted level (dB) of a mono signal, or of a stereo one as the power sum of its two channels (the
    calibration tone, a full-scale sine panned to the centre, reads reference_db this way, as at the ears)."""
    x = np.asarray(x, float)
    chans = x.T if x.ndim == 2 else x[None]
    p = []
    for ch in chans:
        spec = np.abs(np.fft.rfft(ch)) ** 2
        f = np.fft.rfftfreq(len(ch), 1.0 / rate)
        p.append(np.sum(spec * 10 ** (a_weighting_db(f) / 10)) * 2.0 / len(ch) ** 2)
    pm = float(np.mean(p)) if x.ndim == 1 else float(np.sum(p))
    return reference_db + 10 * math.log10(pm / audio.FS_RMS ** 2 + 1e-30)


def dba_timeline(x, reference_db=94.0, window_s=0.125, rate=RATE):
    """A-weighted level (dB, 'fast' 125 ms windows) over time of a stereo or mono signal: (times, levels)."""
    x = np.asarray(x, float)
    if x.ndim == 1:
        x = x[:, None]
    n = int(window_s * rate)
    m = len(x) // n
    if m == 0:
        return np.zeros(0), np.zeros(0)
    f = np.fft.rfftfreq(n, 1.0 / rate)
    w = 10 ** (a_weighting_db(f) / 10)
    seg = x[:m * n].reshape(m, n, x.shape[1])
    spec = np.abs(np.fft.rfft(seg, axis=1)) ** 2
    p = np.sum(spec * w[None, :, None], axis=1) * 2.0 / n ** 2
    p = p.sum(axis=1)
    return (np.arange(m) + 0.5) * window_s, reference_db + 10 * np.log10(p / audio.FS_RMS ** 2 + 1e-30)


def read_wav(path):
    with wave.open(str(path)) as w:
        ch, n, fr = w.getnchannels(), w.getnframes(), w.getframerate()
        x = np.frombuffer(w.readframes(n), "<i2").astype(np.float32).reshape(-1, ch) / 32768.0
    return x, fr


# -- accessible pedestrian signals from SUMO ------------------------------------------------------------------------

class PedSignals:
    """Where a pedestrian hears accessible pedestrian signals: the signal poles (Signals.heads: the lamp beside each
    signalled lane's stop line) within `radius` m, one per spot, and whether crossing that lane is allowed: its light
    and those of the lanes parallel to it within 50 m (the rest of the road being crossed, both directions) are red
    (SUMO's programmes rarely have pedestrian phases of their own; on maps from OpenDRIVE each signal is a programme
    of its own, hence the search by distance rather than by junction)."""

    def __init__(self, signals, net, radius=30.0):
        import traci
        self.traci = traci
        self.radius = radius
        self.poles = []          # (tls, link idxs, (x, y, z) CARLA, unit direction of the lane (SUMO x, y))
        for tls, idxs, loc, *_ in signals.heads:      # (Signals.heads also carry the travel yaw)
            try:
                links = traci.trafficlight.getControlledLinks(tls)
                lane = net.getLane(links[idxs[0]][0][0])
                (x0, y0), (x1, y1) = lane.getShape()[-2:]
                dn = math.hypot(x1 - x0, y1 - y0) or 1.0
                self.poles.append((tls, idxs, (loc.x, loc.y, loc.z), ((x1 - x0) / dn, (y1 - y0) / dn)))
            except Exception:  # noqa: BLE001
                continue
        self._cache = (-1e9, [])

    def near(self, x, y, t):
        if t - self._cache[0] < 0.25:
            return self._cache[1]
        near = sorted(((math.hypot(p[2][0] - x, p[2][1] - y), p) for p in self.poles), key=lambda q: q[0])
        states = {}

        def state(tls):
            if tls not in states:
                try:
                    states[tls] = self.traci.trafficlight.getRedYellowGreenState(tls)
                except Exception:  # noqa: BLE001
                    states[tls] = ""
            return states[tls]

        out = []
        for d, (tls, idxs, loc, u) in near:
            if d > self.radius or len(out) >= 2:
                break
            if any(math.hypot(loc[0] - o[0], loc[1] - o[1]) < 4.0 for o in out):
                continue                       # one signal box per spot
            st = state(tls)
            if not st:
                continue
            red = all(st[i] in "rRs" for i in idxs if i < len(st))
            # the lanes this crossing crosses: this one and those parallel to it (both directions) within 50 m
            par = [q for q in self.poles if math.hypot(q[2][0] - loc[0], q[2][1] - loc[1]) < 50.0
                   and abs(u[0] * q[3][0] + u[1] * q[3][1]) > 0.85]
            par_stopped = all(all(state(q[0])[i] in "rRs" for i in q[1] if i < len(state(q[0]))) for q in par)
            out.append((loc[0], loc[1], loc[2], bool(red and par_stopped)))
        self._cache = (t, out)
        return out


# -- the recorder ---------------------------------------------------------------------------------------------------

def trees_of(mdir):
    """Tree positions (x, y CARLA) from the map's scenery.json (for the birds and the wind in the leaves)."""
    try:
        s = json.loads((Path(mdir) / "scenery.json").read_text())
        return np.array([[it[1], it[2]] for it in s["items"] if it[6] == "tree"], float)
    except Exception:  # noqa: BLE001
        return None


def sound_env(wx, mdir, carla_weather=None, override=None):
    """The soundscape's Env (soundenv.py) of a trial: its weather.json record (or CARLA's weather parameters), the
    map's meta.json and the experiment's sound.weather overrides."""
    from .soundenv import Env
    try:
        meta = json.loads((Path(mdir) / "meta.json").read_text())
    except Exception:  # noqa: BLE001
        meta = {}
    return Env.from_weather(wx, meta, carla_weather, override)


class HostAudio:
    """Renders the first participant's soundscape on the host, tick by tick in simulation time, into audio.wav."""

    def __init__(self, tc, out, part, mdir, wx, step, world, log=print, agents=(), drone_world=None,
                 signals=None, net=None):
        from . import config
        from .soundscape import Soundscape, WorldBodies
        self.log, self.step, self.part, self.drones = log, float(step), part, drone_world
        snd = config._merge(tc.get("sound") or {}, (part.cfg or {}).get("sound") or {})
        snd["enabled"] = True
        # a recording has no amplifier to calibrate: more headroom than a lab's 94 dB (a car's cabin boom and a bus
        # passing at the kerb reach 90-95 dB unweighted)
        snd["reference_db"] = float(snd.get("record_reference_db") or snd.get("reference_db", 94.0))
        self.cfg = snd
        self.kind = part.kind
        self.out = Path(out)
        try:
            cw = world.get_weather()
        except Exception:  # noqa: BLE001
            cw = None
        self.env = sound_env(wx, mdir, cw, snd.get("weather"))
        buildings = None
        if snd.get("buildings", True):
            from .soundgeo import Buildings
            buildings = Buildings.from_map(mdir, log)
        self.cues = []
        self.scape = Soundscape(snd, self.kind, log=log, on_cue=lambda *a: self.cues.append((self.t,) + a),
                                env=self.env, buildings=buildings, base_dir=Path(tc.get("_root", ".")))
        self.scape.trees = trees_of(mdir)
        self.bodies = WorldBodies(world, snd.get("electric", "auto"), float(snd.get("max_distance_m", 150.0)) + 450.0,
                                  every=10)
        self.agents = list(agents)
        self.bodies.automated = {a.vehicle.id for a in self.agents if hasattr(a, "vehicle")}
        self.peds = None
        if self.kind == "pedestrian" and signals is not None and net is not None and snd.get("aps", True):
            try:
                self.peds = PedSignals(signals, net)
                log(f"sound: {len(self.peds.poles)} signal poles for accessible pedestrian signals")
            except Exception as e:  # noqa: BLE001
                log(f"sound: no pedestrian signals ({e})")
        self.path = self.out / "audio.wav"
        self.wav = wave.open(str(self.path), "wb")
        self.wav.setnchannels(2)
        self.wav.setsampwidth(2)
        self.wav.setframerate(RATE)
        self.written = 0
        self.frame0 = self.elapsed0 = None
        self.t = 0.0
        self.ticks = 0
        self.cost = 0.0
        self.peak = 0.0
        self.levels = []          # (t, dB(A)) per tick block (a quick check; dba_timeline on the file is finer)
        log(f"sound: recording {self.kind} {part.role}'s soundscape to {self.path} in simulation time")

    def _listener(self, snap):
        """(x, y, z, yaw), height above ground, own Body (vehicles), own drone id."""
        from .soundscape import Body, classify, is_electric
        p = self.part
        if self.kind == "drone" and self.drones is not None:
            pil = next((q for q in self.drones.pilots if q.part is p), None)
            if pil is not None:
                tf = self.drones.camera_transform(pil, 0.0)
                agl = float(self.drones.fleet.pos[pil.idx, 2] - pil.ground)
                did = self.drones._aid(pil.idx) or -1 - pil.idx
                return (tf.location.x, tf.location.y, tf.location.z, tf.rotation.yaw), max(0.3, agl), None, did
        a = snap.find(p.actor.id) if p.actor is not None else None
        if a is None:
            tf = p.actor.get_transform()
            v = p.actor.get_velocity()
        else:
            tf, v = a.get_transform(), a.get_velocity()
        loc, yaw = tf.location, tf.rotation.yaw
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        if self.kind == "pedestrian":
            x, y, z = loc.x + 0.22 * cy, loc.y + 0.22 * sy, loc.z + 0.72
            return (x, y, z, yaw), 1.6, None, None
        seat = (-0.05, -0.38, 1.22) if self.kind == "driver" else (-0.15, 0.36, 1.15)
        x = loc.x + seat[0] * cy - seat[1] * sy
        y = loc.y + seat[0] * sy + seat[1] * cy
        tid = p.actor.type_id
        lights = self._lights.get(p.actor.id, 0) if self._lights else 0
        own = Body(p.actor.id, tid, classify(tid, p.actor.attributes.get("number_of_wheels", 4)), loc.x, loc.y, loc.z,
                   v.x, v.y, v.z, yaw, is_electric(self.cfg.get("electric", "auto"), tid, p.actor.id,
                                                   self.kind == "av_passenger"),
                   self.kind == "av_passenger", int(lights or 0))
        return (x, y, loc.z + seat[2], yaw), 1.2, own, None

    def tick(self, t, snap, lights=None):
        """After the tick at simulation time t (s since the trial's start): update the scene, render [t, t+step)."""
        c0 = time.perf_counter()
        self.t = t
        if self.frame0 is None:
            self.frame0 = int(snap.frame)
            self.elapsed0 = float(snap.timestamp.elapsed_seconds)
        self._lights = lights or {}
        try:
            listener, h, own, own_drone = self._listener(snap)
            self.bodies.ehmi = {a.vehicle.id: a.ehmi for a in self.agents if hasattr(a, "vehicle")}
            bodies = self.bodies(snap, listener[:2])
            if self._lights:
                bodies = [b._replace(lights=int(self._lights.get(b.id) or 0)) if b.kind != "walker" else b
                          for b in bodies]
            drones = self.drones.sound_sources() if self.drones is not None else None
            aps = self.peds.near(listener[0], listener[1], t) if self.peds is not None else None
            ctl = getattr(self.part, "ctl", None)
            self.scape.update(t, listener, bodies, own=own, own_control=ctl, drones=drones, aps=aps, listener_h=h,
                              own_drone=own_drone)
        except Exception as e:  # noqa: BLE001 - the recording must not stop the trial; keep the clock going
            if not getattr(self, "_warned", False):
                self.log(f"sound: scene update failed ({type(e).__name__}: {e}); the audio continues")
                self._warned = True
        target = int(round((t + self.step) * RATE))
        n = target - self.written
        if n > 0:
            x = self.scape.mixer.render(n)
            self.peak = max(self.peak, float(np.max(np.abs(x))) if len(x) else 0.0)
            self.wav.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())
            self.written += n
            if self.ticks % 10 == 0:
                self.levels.append((round(t, 2), round(level_dba(x, self.scape.ref), 1)))
        self.ticks += 1
        self.cost += time.perf_counter() - c0

    def sound_event(self, se):
        """A sound event of the living city (horn, bell, from livingcity.py and reactions.py): the soundscape renders
        it from that actor (Soundscape.event)."""
        if se.get("name") in ("horn", "bell", "shout"):
            self.scape.event(se["name"], se.get("actor_id"), se.get("kind"))

    def close(self, summary=None):
        try:
            self.wav.close()
        except Exception:  # noqa: BLE001
            pass
        credits = self.scape.used_credits()
        lines = []
        from . import soundfetch
        for role, info in sorted(credits.items()):
            lines.append(f"{role}: " + (info["line"] if "line" in info else soundfetch.credit_line(info)))
        if lines:
            (self.out / "audio_credits.txt").write_text(
                "Recordings and data used in audio.wav (Wikimedia Commons, Freesound; licences as given):\n"
                + "\n".join(lines) + "\nEverything else in audio.wav is synthesised by coupled-carla (MIT licence).\n")
        rec = {"file": self.path.name, "rate": RATE, "channels": 2, "sample_format": "s16le",
               "start_sim_s": 0.0, "start_frame": self.frame0, "start_elapsed_s": self.elapsed0,
               "step_s": self.step, "samples": self.written, "duration_s": round(self.written / RATE, 3),
               "frame_rule": "t = (frame - start_frame) * step_s; sample = round(t * rate)",
               "listener": {"role": self.part.role, "kind": self.kind}, "reference_db": self.scape.ref,
               "spatial": self.cfg.get("spatial", "stereo"), "peak": round(self.peak, 3),
               "limiter_samples": self.scape.mixer.limiter.active,
               "ms_per_tick": round(1000 * self.cost / max(1, self.ticks), 2), "ticks": self.ticks,
               "env": self.env.as_dict(), "credits": lines, "levels_dba": self.levels[:2000],
               "cues": [list(c) for c in self.cues[:500]]}
        (self.out / "audio.json").write_text(json.dumps(rec, indent=1, default=str))
        if summary is not None:
            summary["audio_start_sim_s"] = 0.0
            summary["audio"] = {k: rec[k] for k in ("file", "rate", "start_sim_s", "start_frame", "step_s", "samples",
                                                    "duration_s", "ms_per_tick", "frame_rule")}
        self.log(f"sound: audio.wav {self.written / RATE:.1f} s, {1000 * self.cost / max(1, self.ticks):.1f} ms per "
                 f"tick, peak {self.peak:.2f}{', credits in audio_credits.txt' if lines else ''}")
        return rec


# -- offline: a soundscape again from a trial's logs ------------------------------------------------------------------

def replay(trial, out=None, cfg_over=None, log=print, probe=None):
    """Renders a finished trial's soundscape again from its frames.csv (the first participant as the listener, the
    vehicles logged within logging.radius_m around it), weather.json and map: new sound settings without running
    CARLA again (and a way to inspect a recording: probe(t, scape) is called every tick). Writes `out` (default
    <trial>/audio_replay.wav) in the same simulation-time alignment as audio.wav. Drones and pedestrian signals are
    not in frames.csv, so they are missing; walkers are heard when the crowd was logged."""
    import csv
    import yaml
    from . import config
    from .soundscape import Body, Soundscape, classify, is_electric
    from .soundgeo import Buildings
    trial = Path(trial)
    tc = yaml.safe_load((trial / "config.yaml").read_text())
    tc = config._merge(config.DEFAULTS, tc)
    snd = config._merge(tc.get("sound") or {}, cfg_over or {})
    p0 = (tc.get("participants") or [{}])[0]
    snd = config._merge(snd, p0.get("sound") or {})
    snd["enabled"] = True
    snd["reference_db"] = float(snd.get("record_reference_db") or snd.get("reference_db", 94.0))
    kind, role = p0.get("kind", "driver"), p0.get("role")
    mdir = Path(config.__file__).resolve().parent.parent / "maps" / tc["map"]["name"]
    wx = json.loads((trial / "weather.json").read_text()) if (trial / "weather.json").exists() else {}
    env = sound_env(wx, mdir, None, snd.get("weather"))
    b = Buildings.from_map(mdir, log) if snd.get("buildings", True) else None
    sc = Soundscape(snd, kind, log=log, env=env, buildings=b)
    sc.trees = trees_of(mdir)
    step = float(tc["simulation"]["step"])
    frames = {}
    with open(trial / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            frames.setdefault(int(r["frame"]), []).append(r)
    out = Path(out) if out else trial / "audio_replay.wav"
    w = wave.open(str(out), "wb")
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(RATE)
    written = 0
    for fr in sorted(frames):
        rows = frames[fr]
        t = float(rows[0]["timestamp"])
        me = next((r for r in rows if r["kind"] == "participant" and r["role"] == role), None)
        if me is None:
            continue
        bodies = []
        own = None
        for r in rows:
            tid = r["type_id"]
            if not (tid.startswith("vehicle.") or tid.startswith("walker.")):
                continue
            i = int(r["actor_id"])
            bl = int(r.get("blinkers") or 0)            # logger: 1 left, 2 right, 3 both
            lights = (32 if bl in (1, 3) else 0) | (16 if bl in (2, 3) else 0)
            body = Body(i, tid, classify(tid), float(r["pos_x"]), float(r["pos_y"]), float(r["pos_z"]), float(r["vel_x"]),
                        float(r["vel_y"]), float(r["vel_z"]), float(r["rot_yaw"]),
                        is_electric(snd.get("electric", "auto"), tid, i, r["kind"] == "agent"), r["kind"] == "agent",
                        lights)
            if r is me:
                own = body if kind in ("driver", "av_passenger") else None
                continue
            bodies.append(body)
        x, y, z, yaw = float(me["pos_x"]), float(me["pos_y"]), float(me["pos_z"]), float(me["rot_yaw"])
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        if kind == "pedestrian":
            lis, h = (x + 0.22 * cy, y + 0.22 * sy, z + 0.72, yaw), 1.6
        else:
            lis, h = (x - 0.38 * -sy, y - 0.38 * cy, z + 1.22, yaw), 1.2
        ctl = None
        if own is not None and me.get("throttle"):
            ctl = types_ns(throttle=float(me["throttle"] or 0.0))
        sc.update(t, lis, bodies, own=own, own_control=ctl, listener_h=h)
        if probe is not None:
            probe(t, sc)
        n = int(round((t + step) * RATE)) - written
        if n > 0:
            xb = sc.mixer.render(n)
            w.writeframes((np.clip(xb, -1, 1) * 32767).astype("<i2").tobytes())
            written += n
    w.close()
    log(f"sound: replayed {written / RATE:.1f} s into {out}")
    return out


def types_ns(**kw):
    import types
    return types.SimpleNamespace(**kw)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Render a trial's soundscape again from its logs (frames.csv)")
    ap.add_argument("trial")
    ap.add_argument("--out")
    ap.add_argument("--spatial", choices=["stereo", "binaural"])
    a = ap.parse_args()
    replay(a.trial, a.out, {"spatial": a.spatial} if a.spatial else None)
