"""The soundscape a station plays (coupled-sim #135 and the auditory eHMIs of #95), in calibrated dB SPL.

- Vehicles around the listener: tyre-road noise rising 30 log10(v) (68 dB at 50 km/h and 7.5 m for a car, as in
  road-traffic noise models), an engine for combustion vehicles (pitch stepping like gears), louder buses and
  trucks, near-silent bicycles; inverse-distance attenuation (-6 dB per doubling), air absorption, constant-power
  panning by direction and Doppler. The nearest `max_vehicles` within `max_distance_m` are heard.
- The participant's own car (driver, AV passenger), heard inside: tyres and engine quieter and muffled, wind
  rising 60 log10(v), and the ticking of the turn signal.
- Ambience: distant city traffic or wind in trees (synthesised), or a sound file, at a level at the listener.
- AVAS on electric (or automated) cars: a hum whose pitch rises with speed, at `db` at 2 m up to 15 km/h, faded
  out by `until_kmh` (20 km/h, as UN R138 / EU 540/2014 require).
- Auditory eHMIs: a sound or a spoken message from the front of a car when its eHMI changes to a state, when it
  starts braking, stops, drives off, has a pedestrian ahead, or at a time; optionally repeated.
- Calibration: F3 at a station plays a 1 kHz tone at full scale (everything else muted); the sound level meter's
  reading at the participant's ears goes in `sound.reference_db`.

The scene is pure data (Body records), so it runs and is tested without CARLA; bodies_from_world() makes the
records from a station's CARLA world.
"""
import fnmatch
import math
import zlib
from collections import namedtuple
from pathlib import Path

import numpy as np

from . import audio

C_SOUND = 343.0
EV_PATTERNS = ["vehicle.tesla.*", "vehicle.micro.microlino", "vehicle.nissan.leaf*", "vehicle.*.e_*", "*electric*"]
BICYCLES = ("gazelle", "bh.", "diamondback", "crossbike", "omafiets", "century", "bicycle", "bike")
MOTORBIKES = ("yamaha", "harley", "kawasaki", "vespa", "motorbike", "scooter")
BUSES = ("bus", "fusorosa")
TRUCKS = ("truck", "carlacola", "european_hgv", "firetruck", "hgv")
VANS = ("sprinter", "t2", "ambulance", "van", "transit")
# tyre and engine offsets (dB) against a car; None: no engine
CLASS_DB = {"car": (0.0, 0.0), "van": (2.0, 3.0), "bus": (5.0, 8.0), "truck": (6.0, 9.0),
            "bike": (-22.0, None), "motorbike": (-4.0, 10.0)}

Body = namedtuple("Body", "id type_id kind x y z vx vy vz yaw electric automated lights ehmi")
Body.__new__.__defaults__ = (False, False, 0, None)

LEFT_BLINKER, RIGHT_BLINKER = 32, 16      # carla.VehicleLightState bits


def classify(type_id, wheels=4):
    t = (type_id or "").lower()
    if t.startswith("walker."):
        return "walker"
    if int(wheels or 4) == 2 or any(k in t for k in BICYCLES + MOTORBIKES):
        return "bike" if any(k in t for k in BICYCLES) else "motorbike"
    if any(k in t for k in BUSES):
        return "bus"
    if any(k in t for k in TRUCKS):
        return "truck"
    if any(k in t for k in VANS):
        return "van"
    return "car"


def is_electric(setting, type_id, actor_id, automated=False):
    """Whether a vehicle is electric under `sound.electric`: "auto" (CARLA's electric models and the scripted
    agents), true, false, a share 0-1 (chosen by actor id, the same at every station) or a list of patterns."""
    if setting is True or setting is False:
        return setting
    if setting in (None, "auto"):
        return automated or any(fnmatch.fnmatch(type_id or "", p) for p in EV_PATTERNS)
    if isinstance(setting, (int, float)):
        return (zlib.crc32(str(actor_id).encode()) % 1000) / 1000.0 < float(setting)
    if isinstance(setting, (list, tuple)):
        return any(fnmatch.fnmatch(type_id or "", p) for p in setting) or ("automated" in setting and automated)
    return False


def engine_state(kmh, throttle=0.0):
    """(gear, rpm) of a simple automatic gearbox: gears change at 20, 40, 65, 95 and 130 km/h, the revs rise from
    1100 to 3000 in each gear; idle 800. The engine sound's firing frequency is rpm / 30 (four cylinders)."""
    if kmh < 2.0:
        return 0, 800.0 + 600.0 * max(0.0, min(1.0, throttle))
    bands = [0, 20, 40, 65, 95, 130, 1e9]
    for g in range(1, len(bands)):
        if kmh < bands[g]:
            lo, hi = bands[g - 1], min(bands[g], 200.0)
            return g, 1100.0 + 1900.0 * min(1.0, (kmh - lo) / max(1.0, hi - lo))
    return 6, 3000.0


def tyre_db_at_7_5(kmh, at50=68.0):
    """Tyre-road noise of a car, dB at 7.5 m: about 30 log10 v (CNOSSOS/Harmonoise rolling noise), none below 3 km/h."""
    if kmh < 3.0:
        return None
    return at50 + 30.0 * math.log10(kmh / 50.0)


def at_distance(db_ref, d, ref_m=7.5):
    """A level given at ref_m, at distance d (inverse-distance law, -6 dB per doubling; 1 m nearest)."""
    if db_ref is None:
        return None
    return db_ref - 20.0 * math.log10(max(d, 1.0) / ref_m)


class _Track:
    """What the soundscape remembers of one vehicle: smoothed speed, braking and stop state, eHMI and cue times."""

    def __init__(self, body, t):
        self.kmh = 3.6 * math.hypot(body.vx, body.vy)
        self.prev = self.kmh
        self.accel = 0.0
        self.braking = False
        self.stopped = self.kmh < 0.7
        self.stopped_since = t if self.stopped else None
        self.ehmi = body.ehmi           # the first state seen is the baseline: no cue for it
        self.ehmi_known = body.ehmi is not None
        self.ped_ahead = False
        self.last = {}                  # cue index -> last time played

    def step(self, body, t, dt):
        raw = 3.6 * math.hypot(body.vx, body.vy)
        a = 1.0 - math.exp(-max(dt, 1e-3) / 0.25)
        self.prev = self.kmh
        self.kmh += (raw - self.kmh) * a
        acc = (self.kmh - self.prev) / 3.6 / max(dt, 1e-3)
        self.accel += (acc - self.accel) * a
        events = []
        braking = self.kmh > 1.8 and self.accel < -0.5
        if braking and not self.braking and self.kmh > 10.0 and self.accel < -1.0:
            events.append("braking")
        self.braking = braking
        stopped = self.kmh < 0.7
        if stopped and not self.stopped:
            events.append("stopped")
            self.stopped_since = t
        elif not stopped and self.stopped and self.kmh > 3.0:
            if self.stopped_since is None or t - self.stopped_since > 1.0:
                events.append("driving_off")
        if not stopped and self.kmh > 3.0:
            self.stopped = False
        elif stopped:
            self.stopped = True
        return events


class Soundscape:
    """The station's sound scene. update() once per tick with the listener and the bodies around; the Output
    thread renders the mixer meanwhile."""

    def __init__(self, cfg, kind="pedestrian", log=print, on_cue=None, rate=audio.RATE, base_dir=None):
        self.cfg = cfg or {}
        self.kind = kind
        self.log = log
        self.on_cue = on_cue or (lambda *a: None)
        self.ref = float(self.cfg.get("reference_db", 94.0))
        self.mixer = audio.Mixer(rate)
        self.base_dir = Path(base_dir) if base_dir else Path(__file__).resolve().parent.parent
        self.tracks = {}
        self.t_last = None
        self.calibrating = False
        self.muted = False
        self._ncue = 0
        self.cue_voices = []            # (key, body id or None, db at 2 m or at the listener)
        self.blink_next = None
        self.blink_was = False
        self.info = {}                  # per body: levels at the listener (tests, smoke checks)
        self._buffers()
        self.cues = self._prepare_cues((self.cfg.get("auditory_ehmi") or {}))
        amb = self.cfg.get("ambience") or {}
        if float(amb.get("db") or 0) > 0:
            buf = self._ambience_buffer(amb)
            if buf is not None:
                self.mixer.add("ambience", audio.Loop(buf, gain=self._g(float(amb["db"]))))
        self.reported = False

    # -- set-up ---------------------------------------------------------------------------------------------------
    def _g(self, db):
        return 0.0 if self.muted or self.calibrating else audio.db_gain(db, self.ref)

    def _buffers(self):
        self.buf_tyre = audio.pink_noise(3.0, seed=7)
        self.buf_wind = audio.band_noise(80.0, 3000.0, 3.0, seed=13)
        self.buf_tone = audio.tone(1000.0, 1.0)
        self.sounds = {n: audio.cue_sound(n) for n in ("chime", "beeps", "ding", "tick", "takeover")}

    def _path(self, p):
        p = Path(p).expanduser()
        return p if p.is_absolute() else self.base_dir / p

    def _ambience_buffer(self, amb):
        kind = amb.get("kind", "city")
        if amb.get("file"):
            try:
                return audio.load_sound(self._path(amb["file"]))
            except Exception as e:
                self.log(f"sound: cannot read ambience {amb['file']} ({e}); using {kind}")
        if kind == "rural":
            return audio.rural_ambience()
        if kind in ("city", None, True):
            return audio.city_ambience()
        return None

    def sound_for(self, spec, default="chime"):
        """The buffer a cue plays: `say` (speech, from the machine's synthesiser), `file`, or `sound` (a built-in:
        chime, beeps, ding, tick, takeover). Falls back to the built-in with one log line."""
        if spec.get("say"):
            wav = audio.speak_to_wav(str(spec["say"]), spec.get("voice"))
            if wav is not None:
                try:
                    return audio.load_sound(wav)
                except Exception as e:
                    self.log(f"sound: speech for {spec['say']!r} unreadable ({e})")
            else:
                self.log(f"sound: no speech synthesiser here for {spec['say']!r} (espeak-ng, say or SAPI); "
                         f"playing {spec.get('sound', default)}")
        if spec.get("file"):
            try:
                return audio.load_sound(self._path(spec["file"]))
            except Exception as e:
                self.log(f"sound: cannot read {spec['file']} ({e}); playing {spec.get('sound', default)}")
        name = spec.get("sound", default)
        return self.sounds.get(name, self.sounds[default])

    def _prepare_cues(self, ae):
        if not ae.get("enabled"):
            return []
        cues = ae.get("cues") or [{"ehmi": "yielding", "sound": "chime"}, {"ehmi": "go", "sound": "chime"},
                                  {"ehmi": "driving", "sound": "beeps"}]
        out = []
        for c in cues:
            c = dict(c)
            c.setdefault("db", ae.get("db", 65.0))
            c["_buf"] = self.sound_for(c)
            out.append(c)
        self.log(f"sound: auditory eHMI with {len(out)} cues")
        return out

    # -- per tick -------------------------------------------------------------------------------------------------
    def toggle_calibration(self):
        self.calibrating = not self.calibrating
        if self.calibrating:
            self.mixer.add("calibration", audio.Loop(self.buf_tone, gain=1.0))
            self.log(f"sound: calibration tone on (1 kHz, full scale): measure it at the ears and set "
                     f"sound.reference_db (now {self.ref:.1f})")
        else:
            self.mixer.remove("calibration")
            self.log("sound: calibration tone off")
        self.on_cue("calibration", "", "on" if self.calibrating else "off")
        amb = self.mixer.get("ambience")
        if amb is not None:
            amb.gain = self._g(float((self.cfg.get("ambience") or {}).get("db") or 0))

    def play(self, name_or_buf, db, body=None, name="cue", detail=""):
        """Plays a cue: in the car (body None: at db at the listener) or from the front of a body (db at 2 m)."""
        buf = self.sounds.get(name_or_buf) if isinstance(name_or_buf, str) else name_or_buf
        if buf is None:
            return
        self._ncue += 1
        key = ("cue", self._ncue)
        v = self.mixer.add(key, audio.OneShot(buf, gain=self._g(db) if body is None else 0.0))
        self.cue_voices.append((key, body.id if body is not None else None, db, v))
        self.on_cue("sound_cue", name, detail)
        return v

    def update(self, t, listener, bodies, own=None, own_control=None):
        """listener: (x, y, z, yaw) of the ears; bodies: Body records around (own vehicle among them or not);
        own: the Body of the participant's vehicle (None for a pedestrian)."""
        dt = 0.05 if self.t_last is None else max(1e-3, t - self.t_last)
        self.t_last = t
        lx, ly, lz, lyaw = listener
        yaw = math.radians(lyaw)
        fx, fy = math.cos(yaw), math.sin(yaw)
        lvx = own.vx if own is not None else 0.0
        lvy = own.vy if own is not None else 0.0
        cfg = self.cfg
        maxd = float(cfg.get("max_distance_m", 150.0))
        others = []
        for b in bodies:
            if own is not None and b.id == own.id:
                continue
            d = math.dist((b.x, b.y, b.z), (lx, ly, lz))
            others.append((d, b))
        others.sort(key=lambda x: x[0])
        # tracks for every vehicle in range (cues need them), voices for the nearest
        seen = set()
        heard = [] if not cfg.get("vehicles", True) else \
            [(d, b) for d, b in others if b.kind != "walker" and d <= maxd][:int(cfg.get("max_vehicles", 16))]
        walkers = [b for d, b in others if b.kind == "walker"]
        for d, b in others:
            if b.kind == "walker" or d > maxd:
                continue
            tr = self.tracks.get(b.id)
            if tr is None:
                tr = self.tracks[b.id] = _Track(b, t)
                events = []
            else:
                events = tr.step(b, t, dt)
            seen.add(b.id)
            if self.cues:
                self._cues_for(b, tr, events, t, walkers)
        for bid in [k for k in self.tracks if k not in seen]:
            del self.tracks[bid]
        keep = set()
        self.info = {}
        for d, b in heard:
            keep.add(b.id)
            self._vehicle_voice(b, d, lx, ly, lz, fx, fy, lvx, lvy)
        for key in self.mixer.keys():
            if isinstance(key, tuple) and key[0] == "veh" and key[1] not in keep:
                self.mixer.remove(key)
        if own is not None and cfg.get("own_vehicle", True):
            self._own(own, own_control, t)
        # positional cues follow their car
        live = []
        pos = {b.id: (d, b) for d, b in others}
        for key, bid, db, v in self.cue_voices:
            if v.done:
                continue
            live.append((key, bid, db, v))
            if bid is None:
                v.gain = self._g(db)
            elif bid in pos:
                d, b = pos[bid]
                fb = 2.3          # roughly the front of the car
                px, py = b.x + fb * math.cos(math.radians(b.yaw)), b.y + fb * math.sin(math.radians(b.yaw))
                d2 = math.dist((px, py, b.z), (lx, ly, lz))
                v.gain = self._g(at_distance(db, d2, 2.0))
                v.pan, v.cutoff = self._pan(px - lx, py - ly, fx, fy), self._air(d2)
            else:
                v.gain = 0.0
        self.cue_voices = live
        if not self.reported and heard:
            self.reported = True
            self.log(f"sound: {len(heard)} vehicles heard within {maxd:.0f} m, reference {self.ref:.1f} dB")

    @staticmethod
    def _pan(dx, dy, fx, fy):
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return 0.0
        right = (-dx * fy + dy * fx) / d          # CARLA: right of heading (fx, fy) is (-fy, fx)
        return max(-1.0, min(1.0, right))

    @staticmethod
    def _air(d, behind=False):
        return 14000.0 / (1.0 + d / 60.0) * (0.6 if behind else 1.0)

    def _doppler(self, b, dx, dy, d, lvx, lvy):
        k = float(self.cfg.get("doppler", 0.5))
        if k <= 0 or d < 1e-3:
            return 1.0
        ux, uy = dx / d, dy / d               # listener -> source
        vs = b.vx * ux + b.vy * uy            # source moving away > 0
        vl = lvx * ux + lvy * uy              # listener moving towards the source > 0
        ratio = (C_SOUND + vl) / max(1.0, C_SOUND + vs)
        return 1.0 + k * (ratio - 1.0)

    def _vehicle_voice(self, b, d, lx, ly, lz, fx, fy, lvx, lvy):
        cfg = self.cfg
        tr = self.tracks.get(b.id)
        kmh = tr.kmh if tr else 3.6 * math.hypot(b.vx, b.vy)
        tyre_off, eng_off = CLASS_DB.get(b.kind, (0.0, 0.0))
        dx, dy = b.x - lx, b.y - ly
        pan = self._pan(dx, dy, fx, fy)
        behind = (dx * fx + dy * fy) < 0
        cut = self._air(d, behind)
        dop = self._doppler(b, dx, dy, d, lvx, lvy)
        t75 = tyre_db_at_7_5(kmh, float(cfg.get("tyre_db_at_50", 68.0)))
        tyre = at_distance(t75 + tyre_off, d) if t75 is not None else None
        eng = None
        if eng_off is not None and not b.electric:
            eng = at_distance(float(cfg.get("engine_db", 64.0)) + 6.0 * min(1.0, kmh / 60.0) + eng_off, d)
        avas = None
        av = cfg.get("avas") or {}
        if av.get("enabled") and b.kind in ("car", "van", "bus", "truck"):
            who = av.get("vehicles", "electric")
            if who == "all" or (who == "automated" and b.automated) or (who == "electric" and b.electric):
                until = float(av.get("until_kmh", 20.0))
                fade = 1.0 if kmh <= until - 5.0 else max(0.0, (until - kmh) / 5.0)
                if kmh < 0.5 and not av.get("stopped", False):
                    fade = 0.0
                if fade > 0:
                    avas = at_distance(float(av.get("db", 56.0)) + 20 * math.log10(max(fade, 1e-3)), d, 2.0)
        self.info[b.id] = {"kind": b.kind, "kmh": kmh, "d": d, "tyre_db": tyre, "engine_db": eng, "avas_db": avas,
                           "pan": pan, "electric": b.electric}
        for part, db in (("tyre", tyre), ("engine", eng), ("avas", avas)):
            key = ("veh", b.id, part)
            v = self.mixer.get(key)
            if db is None:
                if v is not None:
                    v.gain = 0.0
                continue
            if v is None:
                if part == "tyre":
                    v = audio.Loop(self.buf_tyre, start=(b.id * 7919) % len(self.buf_tyre))
                elif part == "engine":
                    v = audio.Engine(seed=b.id)
                else:
                    v = audio.Avas()
                self.mixer.add(key, v)
            v.gain, v.pan = self._g(db), pan
            if part == "tyre":
                v.rate, v.cutoff = (0.6 + 0.8 * min(1.0, kmh / 100.0)) * dop, min(cut, 8000.0)
            elif part == "engine":
                _g, rpm = engine_state(kmh)
                v.freq, v.rate, v.cutoff = rpm / 30.0, dop, cut
            else:
                v.rate, v.cutoff = (1.0 + 0.01 * kmh) * dop, cut

    def _own(self, own, ctl, t):
        cfg = self.cfg
        kmh = 3.6 * math.hypot(own.vx, own.vy)
        red = float(cfg.get("interior_reduction_db", 6.0))
        tyre_off, eng_off = CLASS_DB.get(own.kind, (0.0, 0.0))
        t75 = tyre_db_at_7_5(kmh, float(cfg.get("tyre_db_at_50", 68.0)))
        tyre = t75 + tyre_off - red if t75 is not None else None
        throttle = float(getattr(ctl, "throttle", 0.0) or 0.0) if ctl is not None else 0.0
        eng = None
        if eng_off is not None and not own.electric:
            eng = float(cfg.get("engine_db", 64.0)) + 6.0 * min(1.0, kmh / 60.0) + 4.0 * throttle + eng_off - red
        wind = float(cfg.get("wind_db_at_100", 64.0)) + 60.0 * math.log10(kmh / 100.0) if kmh > 10.0 else None
        self.info["own"] = {"kmh": kmh, "tyre_db": tyre, "engine_db": eng, "wind_db": wind}
        for part, db in (("tyre", tyre), ("engine", eng), ("wind", wind)):
            key = ("own", part)
            v = self.mixer.get(key)
            if db is None:
                if v is not None:
                    v.gain = 0.0
                continue
            if v is None:
                v = self.mixer.add(key, audio.Engine(seed=1) if part == "engine" else
                                   audio.Loop(self.buf_tyre if part == "tyre" else self.buf_wind))
            v.gain, v.pan = self._g(db), 0.0
            if part == "tyre":
                v.rate, v.cutoff = 0.6 + 0.8 * min(1.0, kmh / 100.0), 700.0
            elif part == "engine":
                _g, rpm = engine_state(kmh, throttle)
                v.freq, v.cutoff = rpm / 30.0, 1200.0
            else:
                v.rate, v.cutoff = 0.8 + 0.4 * min(1.0, kmh / 130.0), 1800.0
        # turn signal: a relay click every 1/3 s while a blinker is on (the lamp's on and off edges at 1.5 Hz)
        on = bool(int(own.lights or 0) & (LEFT_BLINKER | RIGHT_BLINKER))
        if on and (not self.blink_was or (self.blink_next is not None and t >= self.blink_next)):
            self.play("tick", float(cfg.get("blinker_db", 58.0)), name="blinker_tick")
            self.blink_next = t + 1.0 / 3.0
        self.blink_was = on

    def _cues_for(self, b, tr, events, t, walkers):
        """Auditory eHMI cues of one vehicle."""
        new_state = None
        if b.ehmi != tr.ehmi:
            if tr.ehmi_known:                 # the first state learnt for a car is the baseline
                new_state = b.ehmi
            tr.ehmi = b.ehmi
        tr.ehmi_known = tr.ehmi_known or b.ehmi is not None
        ped = False
        for i, c in enumerate(self.cues):
            who = c.get("vehicles", "automated" if "ehmi" not in c else "ehmi")
            if who == "automated" and not b.automated:
                continue
            if who == "electric" and not b.electric:
                continue
            if who == "ehmi" and b.ehmi is None:
                continue
            on = c.get("on")
            if "ehmi" in c:
                cond = b.ehmi == c["ehmi"]
                edge = cond and new_state == c["ehmi"]
            elif on in ("braking", "stopped", "driving_off"):
                cond = (on == "braking" and tr.braking) or (on == "stopped" and tr.stopped)
                edge = on in events
                cond = cond or edge
            elif on == "pedestrian_ahead":
                reach = float(c.get("value", 15.0))
                yaw = math.radians(b.yaw)
                cond = False
                for w in walkers:
                    dx, dy = w.x - b.x, w.y - b.y
                    fwd, side = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
                    if 0 < fwd < reach and abs(side) < 3.0:
                        cond = True
                        break
                edge = cond and not tr.ped_ahead
                ped = ped or cond
            elif on == "at":
                cond = edge = t >= float(c.get("value", 0.0)) and i not in tr.last
            else:
                continue
            rep = float(c.get("repeat_s") or 0.0)
            if edge or (cond and rep > 0 and t - tr.last.get(i, -1e9) >= rep):
                tr.last[i] = t
                self.play(c["_buf"], float(c["db"]), body=b, name=c.get("say") or c.get("file") or c.get("sound", "chime"),
                          detail=f"actor {b.id} {('ehmi ' + str(c['ehmi'])) if 'ehmi' in c else on}")
        tr.ped_ahead = ped

    def voices_info(self):
        return self.info

    def close(self):
        for k in self.mixer.keys():
            self.mixer.remove(k)


# -- from a station's CARLA world -------------------------------------------------------------------------------

class WorldBodies:
    """Body records of the vehicles and walkers near the listener, from the CARLA world on the station. The
    snapshot (client side) gives poses every tick; the types of new actors in range are fetched in one call every
    `every` ticks and cached (the thousands of scenery props are looked up once and then skipped)."""

    def __init__(self, world, electric="auto", radius=150.0, every=25):
        self.world, self.electric, self.radius, self.every = world, electric, radius, every
        self.types = {}            # id -> (type_id, kind, wheels, role) or None for anything else
        self.ids = set()           # vehicles and walkers
        self.n = 0
        self.automated = set()     # actor ids of the host's scripted agents
        self.ehmi = {}             # actor id -> eHMI state (from the host)

    def __call__(self, snapshot, centre):
        self.n += 1
        cx, cy = centre[0], centre[1]
        r2 = (self.radius + 30.0) ** 2
        if self.n % self.every == 1:
            new = []
            for a in snapshot:
                if a.id in self.types:
                    continue
                loc = a.get_transform().location
                if (loc.x - cx) ** 2 + (loc.y - cy) ** 2 <= r2:
                    new.append(a.id)
            if new:
                try:
                    for actor in self.world.get_actors(new):
                        tid = actor.type_id
                        if tid.startswith("vehicle.") or tid.startswith("walker.pedestrian"):
                            wheels = actor.attributes.get("number_of_wheels", 4)
                            self.types[actor.id] = (tid, classify(tid, wheels), wheels,
                                                    actor.attributes.get("role_name", ""))
                            self.ids.add(actor.id)
                        else:
                            self.types[actor.id] = None
                except RuntimeError:
                    pass
                for i in new:
                    self.types.setdefault(i, None)
        out = []
        for i in list(self.ids):
            a = snapshot.find(i)
            if a is None:
                self.ids.discard(i)
                self.types.pop(i, None)
                continue
            tf = a.get_transform()
            loc = tf.location
            if (loc.x - cx) ** 2 + (loc.y - cy) ** 2 > r2:
                continue
            v = a.get_velocity()
            tid, kind, _w, _role = self.types[i]
            auto = i in self.automated
            out.append(Body(i, tid, kind, loc.x, loc.y, loc.z, v.x, v.y, v.z, tf.rotation.yaw,
                            kind != "walker" and is_electric(self.electric, tid, i, auto), auto, 0,
                            self.ehmi.get(i)))
        return out


def body_of(actor, electric="auto", automated=False, ehmi=None):
    """A Body for one CARLA actor (the participant's own vehicle)."""
    tf = actor.get_transform()
    v = actor.get_velocity()
    tid = actor.type_id
    try:
        lights = int(actor.get_light_state())
    except (RuntimeError, AttributeError, TypeError):
        lights = 0
    kind = classify(tid, actor.attributes.get("number_of_wheels", 4))
    return Body(actor.id, tid, kind, tf.location.x, tf.location.y, tf.location.z, v.x, v.y, v.z,
                tf.rotation.yaw, is_electric(electric, tid, actor.id, automated), automated, lights, ehmi)


def render_offline(scape, seconds, rate=audio.RATE):
    """Renders the mixer for `seconds` (tests, previews) as (n, 2) float32."""
    n = int(seconds * rate)
    blocks = [scape.mixer.render(min(audio.BLOCK, n - i)) for i in range(0, n, audio.BLOCK)]
    return np.concatenate(blocks) if blocks else np.zeros((0, 2), np.float32)
