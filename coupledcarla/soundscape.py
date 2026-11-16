"""The soundscape a participant hears (coupled-sim #135 and the auditory eHMIs of #95), in calibrated dB SPL, from
published acoustic models (acoustics.py) rather than tuned guesses:

- Vehicles: CNOSSOS-EU sound power per octave band (63 Hz-8 kHz) for cars, vans, buses and trucks, and
  two-wheelers: rolling noise A_R + B_R lg(v/70) (with the road surface, the air temperature and, on wet roads,
  IMAGINE's wet-road increase), propulsion noise A_P + B_P (v-70)/70 with the acceleration correction and the engine
  speed from a gearbox model (idling at 600-1200 rpm when stopped). Rolling noise is band-shaped noise; the engine
  is synthesised from its firing frequency and orders with load-dependent harmonics, intake and exhaust formants and
  cycle-to-cycle jitter (synth.EngineVoice), carrying CNOSSOS's band levels. Electric vehicles have rolling noise
  and an AVAS (UN R138: up to 20 km/h). Bicycles: tyre noise, a freewheel ratchet when coasting and now and then a
  bell near pedestrians. Emergency vehicles: a siren with their special lights. Buses: air hiss when they stop.
- Propagation per band and per path: spherical spreading (20 lg d + 11), ISO 9613-1 air absorption at the trial's
  temperature and humidity, ISO 9613-2 ground effect, occlusion by the map's buildings (ISO 9613-2 barrier over the
  roofs and round the sides), two first-order reflections off the nearest facades (image sources, delayed), a
  reverberant tail sized by the street canyon (soundgeo.py), Doppler, and panning or binaural (HRTF) output.
- Inside a car (driver, AV passenger): the own car's tyres and engine through a cabin insertion loss rising with
  frequency, wind noise rising 60 lg v, the indicator relay, rain on the roof; outside sounds through the cabin.
- Pedestrians' footsteps (Cook 2002) by walking speed, the participant's own; the ticking of accessible pedestrian
  signals at signalised crossings (slow while red, fast while green, as in the Netherlands and Germany).
- Drones (drones.py): blade-passing tones and harmonics over broadband noise, at the EU 2019/945 sound power.
- Ambience (soundenv.py): birds by dawn, season and trees, wind gusts, rain from the real weather (recorded where
  Wikimedia Commons has a good open recording, else synthesised), and distant traffic following the hour.
- AVAS on electric (or automated) cars, auditory eHMIs (a sound or a spoken message from the front of a car when its
  eHMI changes, when it brakes, stops, drives off, has a pedestrian ahead, or at a time).
- Calibration: F3 at a station plays a 1 kHz tone at full scale (everything else muted); the sound level meter's
  reading at the participant's ears goes in `sound.reference_db`. A limiter keeps the sum from clipping.

The scene is pure data (Body records), so it runs and is tested without CARLA; WorldBodies makes the records from a
CARLA world (a station's or the host's).
"""
import fnmatch
import math
import zlib
from collections import namedtuple
from pathlib import Path

import numpy as np

from . import acoustics as ac
from . import audio, soundsamples, synth
from .soundenv import Ambience, Env

C_SOUND = 343.0
EV_PATTERNS = ["vehicle.tesla.*", "vehicle.micro.microlino", "vehicle.nissan.leaf*", "vehicle.*.e_*", "*electric*",
               "vehicle.audi.etron"]
BICYCLES = ("gazelle", "bh.", "diamondback", "crossbike", "omafiets", "century", "bicycle", "bike")
MOTORBIKES = ("yamaha", "harley", "kawasaki", "vespa", "motorbike", "scooter")
MOPEDS = ("vespa", "scooter", "moped")
BUSES = ("bus", "fusorosa", "fuso.mitsubishi")     # CARLA 1.0: vehicle.fuso.mitsubishi
TRUCKS = ("truck", "carlacola", "european_hgv", "firetruck", "hgv")
VANS = ("sprinter", "t2", "ambulance", "van", "transit")
EMERGENCY = ("ambulance", "firetruck", "police", "dodgecop")     # 1.0: vehicle.dodgecop.charger
# offsets of the old model (coupled-sim): kept for the cue levels and tests of the legacy functions below
CLASS_DB = {"car": (0.0, 0.0), "van": (2.0, 3.0), "bus": (5.0, 8.0), "truck": (6.0, 9.0),
            "bike": (-22.0, None), "motorbike": (-4.0, 10.0)}

Body = namedtuple("Body", "id type_id kind x y z vx vy vz yaw electric automated lights ehmi")
Body.__new__.__defaults__ = (False, False, 0, None)

LEFT_BLINKER, RIGHT_BLINKER, SPECIAL1 = 32, 16, 512      # carla.VehicleLightState bits

# gearbox per kind: idle rpm, rpm per km/h in each gear, upshift rpm at no load and its rise at full load,
# downshift rpm, cylinders, diesel, full-load acceleration (m/s^2)
GEARS = {"car": (800.0, [118, 68, 46, 35, 28, 23], 2000.0, 2000.0, 1150.0, 4, False, 3.0),
         "van": (750.0, [100, 58, 38, 28, 22, 18], 1800.0, 1300.0, 1050.0, 4, True, 2.0),
         "bus": (600.0, [60, 38, 26, 19, 14.5, 11.5], 1450.0, 700.0, 850.0, 6, True, 1.2),
         "truck": (600.0, [60, 38, 26, 19, 14.5, 11.5], 1450.0, 700.0, 850.0, 6, True, 1.2),
         "motorbike": (1200.0, [260, 180, 140, 115, 98, 88], 5000.0, 4000.0, 3000.0, 2, False, 4.0)}


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


def category(kind, type_id=""):
    """CNOSSOS-EU category of a vehicle kind (None for bicycles and walkers)."""
    if kind == "motorbike" and any(k in (type_id or "").lower() for k in MOPEDS):
        return "4a"
    return ac.CATEGORY.get(kind)


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


def gear_for(kmh, load=0.3, kind="car"):
    """(gear, rpm) a driver would use at a steady `kmh`: the highest gear whose rpm stays above the downshift
    point and the lowest whose rpm is under the upshift point for this load; idle (gear 0) when stopped."""
    idle, ratios, up0, upk, down, *_ = GEARS.get(kind, GEARS["car"])
    if kmh < 1.5:
        return 0, idle + 600.0 * max(0.0, min(1.0, load))
    up = up0 + upk * max(0.0, min(1.0, load))
    g = 1
    while g < len(ratios) and kmh * ratios[g - 1] > up and kmh * ratios[g] >= down:
        g += 1
    rpm = kmh * ratios[g - 1]
    if g == 1:     # pulling away: the clutch slips, the engine stays above idle
        rpm = max(rpm, idle + 400.0 + 600.0 * load)
    return g, rpm


def engine_state(kmh, throttle=0.0):
    """(gear, rpm) of a car's automatic gearbox (the cluster's rev counter and the engine sound share it)."""
    return gear_for(kmh, 0.3 + 0.7 * max(0.0, min(1.0, throttle)), "car")


class Gearbox:
    """A vehicle's gear and engine speed over time (hysteresis between up- and downshifts, rpm easing over 0.3 s,
    as a gear change takes)."""

    def __init__(self, kind):
        self.kind = kind if kind in GEARS else "car"
        self.gear, self.rpm = 0, GEARS[self.kind][0]

    def step(self, kmh, load, dt):
        idle, ratios, up0, upk, down, *_ = GEARS[self.kind]
        if kmh < 1.5:
            self.gear = 0
            target = idle + 500.0 * load
        else:
            g = max(1, self.gear)
            up = up0 + upk * load
            if kmh * ratios[g - 1] > up and g < len(ratios) and kmh * ratios[g] >= down:
                g += 1
            elif kmh * ratios[g - 1] < down and g > 1:
                g -= 1
            self.gear = g
            target = kmh * ratios[g - 1]
            if g == 1:
                target = max(target, idle + 400.0 + 600.0 * load)
        a = 1.0 - math.exp(-max(dt, 1e-3) / 0.3)
        self.rpm += (target - self.rpm) * a
        return self.gear, self.rpm


def tyre_db_at_7_5(kmh, at50=68.0):
    """The old (coupled-sim) tyre model: a car's tyre-road noise in dB at 7.5 m, 30 log10 v; none below 3 km/h."""
    if kmh < 3.0:
        return None
    return at50 + 30.0 * math.log10(kmh / 50.0)


def at_distance(db_ref, d, ref_m=7.5):
    """A level given at ref_m, at distance d (inverse-distance law, -6 dB per doubling; 1 m nearest)."""
    if db_ref is None:
        return None
    return db_ref - 20.0 * math.log10(max(d, 1.0) / ref_m)


def _lw_from_lp(lp, d=2.0):
    """Sound power of a point source near the ground that gives `lp` at d m (hemispherical: Lw = Lp + 20 lg d + 8)."""
    return lp + 20.0 * math.log10(d) + 8.0


def _shape(shape, total_dba):
    """Band levels with spectrum `shape` whose A-weighted total is total_dba."""
    s = np.asarray(shape, float)
    return s - ac.a_weighted(s) + total_dba


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


class _Paths:
    """What a source looks like from the listener this tick: per path (direct, reflection 1, reflection 2, reverb)
    the attenuation per band (dB, from sound power to the ears), the extra delay (s) and the direction."""
    __slots__ = ("att", "delay", "az", "el", "d")

    def __init__(self):
        self.att = [None, None, None, None]
        self.delay = [0.0, 0.0, 0.0, 0.0]
        self.az = [0.0, 0.0, 0.0, 0.0]
        self.el = [0.0, 0.0, 0.0, 0.0]
        self.d = 1.0


class Soundscape:
    """The participant's sound scene. update() once per tick with the listener and the bodies around; the mixer is
    rendered meanwhile (the station's Output thread in real time, or the host's recorder in simulation time)."""

    def __init__(self, cfg, kind="pedestrian", log=print, on_cue=None, rate=audio.RATE, base_dir=None, env=None,
                 buildings=None, map_dir=None):
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
        self.credits = {}               # recordings used: role -> attribution (soundfetch)
        cfg = self.cfg
        self.env = env if isinstance(env, Env) else Env(env if env is not None else cfg.get("env"))
        self.alpha = ac.air_absorption(self.env.temp, self.env.rh)
        self.surface = cfg.get("road_surface", "DEF")
        self.table = ac.CNOSSOS_2015 if cfg.get("model") == "cnossos2015" else ac.CNOSSOS
        g = cfg.get("ground", "auto")
        self.G = 0.0 if g in ("auto", None, True) else float(g)
        self.buildings = buildings
        if self.buildings is None and cfg.get("buildings", True) and map_dir is not None:
            from .soundgeo import Buildings
            self.buildings = Buildings.from_map(map_dir, log)
        self.n_refl = int(cfg.get("reflections", 2)) if self.buildings is not None else 0
        self.facades = (np.zeros(0, int), None, -1e9)
        self.rev_level = -40.0
        self.canyon = None
        self._canyon_t = -1e9
        if cfg.get("reverb", True):
            self.mixer.reverb = synth.Reverb(0.8)
        self.spatial = cfg.get("spatial", "stereo")
        if self.spatial == "binaural":
            from . import hrtf
            hset, credit = hrtf.load(cfg.get("hrtf", "kemar"), log)
            self.mixer.spatializer = hrtf.Binaural(hset)
            if credit:
                self.credits["hrtf"] = {"line": credit}
            log(f"sound: binaural output with {hset.name}")
        self.listener_prev = None
        self.lv = (0.0, 0.0)
        self.rng = np.random.default_rng(17)
        self._buffers()
        self.cues = self._prepare_cues((cfg.get("auditory_ehmi") or {}))
        amb = cfg.get("ambience") or {}
        if float(amb.get("db") or 0) > 0:
            buf = self._ambience_buffer(amb)
            if buf is not None:
                self.mixer.add("ambience", audio.Loop(buf, gain=self._g(float(amb["db"]))))
        self.ambience = None
        if cfg.get("environment", True):
            try:
                self.ambience = Ambience(self, cfg, self.env, log)
                self.credits.update(self.ambience.credits)
            except Exception as e:  # noqa: BLE001 - the ambience is extra: never take the station down for it
                log(f"sound: no ambience ({type(e).__name__}: {e})")
        self.trees = None                # tree positions (x, y) near the listener (set by the caller, optional)
        self.reported = False
        self.levels = {}

    # -- set-up ---------------------------------------------------------------------------------------------------
    def _g(self, db):
        return 0.0 if self.muted or self.calibrating else audio.db_gain(db, self.ref)

    @property
    def _on(self):
        return 0.0 if self.muted or self.calibrating else 1.0

    def _buffers(self):
        self.buf_tone = audio.tone(1000.0, 1.0)
        self.sounds = {n: audio.cue_sound(n) for n in ("chime", "beeps", "ding", "tick", "takeover")}
        wet = self.env.wet > 0.3
        # footsteps: recorded single steps (soundsamples, data/sounds/footsteps) of `footsteps` (shoes, trainers,
        # grass), else synthesised (Cook 2002)
        fw = self.cfg.get("footsteps", "shoes")
        self.steps = soundsamples.footstep_buffers(fw, self.log, wet=wet) if fw != "synth" else None
        self.steps_recorded = self.steps is not None
        if self.steps is None:
            self.steps = synth.footstep_buffers("soft" if fw == "grass" else "hard", 6, wet=wet)
        self.aps_ticks = synth.tick_buffers("aps", 3)
        self.ratchet = synth.tick_buffers("ratchet", 3)
        self.bell_buf = None
        if self.cfg.get("recordings", True) and (self.cfg.get("bicycles") or {}).get("bells", 0.03):
            try:
                from . import soundfetch
                from .soundenv import load_recording
                path, info = soundfetch.recording("bicycle_bell", log=self.log)
                if path is not None:
                    self.bell_buf = audio._peak_norm(load_recording(path, 3.0, highpass=300.0))
                    self.credits["bicycle_bell"] = info
            except Exception as e:  # noqa: BLE001
                self.log(f"sound: no bicycle bell recording ({e}); synthesised")
        if self.bell_buf is None:
            self.bell_buf = synth.bell()
        country = self.env.country
        self.siren_buf = synth.siren("wail" if country in ("US", "CA", "MX", "AU", "NZ") else "two_tone")
        self.horn_buf = synth.horn(1.0)
        self.hiss_buf = synth.air_hiss()

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
            if "on" not in c:       # YAML reads an unquoted `on:` key as true (and the welcome's JSON as "true")
                for k in (True, "true"):
                    if k in c:
                        c["on"] = c.pop(k)
                        break
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
        if self.calibrating:      # silence every spatial voice at once (update() keeps them silent)
            for k in self.mixer.keys():
                v = self.mixer.get(k)
                for p in getattr(v, "paths", ()):
                    p.silence()

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

    def update(self, t, listener, bodies, own=None, own_control=None, drones=None, aps=None, listener_h=None,
               own_drone=None):
        """listener: (x, y, z, yaw[, pitch]) of the ears; bodies: Body records around (own vehicle among them or
        not); own: the Body of the participant's vehicle (None for a pedestrian or a drone); drones: the host's
        drone list ([id, x, y, z, vx, vy, vz, blade-passing Hz, thrust ratio, sound power dB(A), blades],
        drones.py); aps: accessible pedestrian signals [(x, y, z, walk)]; listener_h: the ears' height above the
        ground (default by kind); own_drone: the id of the drone whose camera is the listener (its rotors heard at
        the camera, sound.record_own_drone)."""
        dt = 0.05 if self.t_last is None else max(1e-3, t - self.t_last)
        self.t_last = t
        lx, ly, lz, lyaw = listener[:4]
        yaw = math.radians(lyaw)
        fx, fy = math.cos(yaw), math.sin(yaw)
        if self.listener_prev is not None:
            px, py = self.listener_prev
            self.lv = ((lx - px) / dt, (ly - py) / dt)
        self.listener_prev = (lx, ly)
        lvx = own.vx if own is not None else self.lv[0]
        lvy = own.vy if own is not None else self.lv[1]
        cfg = self.cfg
        hr = float(listener_h if listener_h is not None else
                   {"pedestrian": 1.6, "driver": 1.2, "av_passenger": 1.2}.get(self.kind, 1.5))
        self.lis = (lx, ly, lz, fx, fy, hr, lz - hr)
        self.inside = own is not None and self.kind in ("driver", "av_passenger") and cfg.get("own_vehicle", True)
        self.il = (ac.CABIN_IL_BUS if own is not None and own.kind == "bus" else ac.CABIN_IL) if self.inside \
            else np.zeros(ac.NB)
        maxd = float(cfg.get("max_distance_m", 150.0))
        bodies = self._velocities(bodies, t)
        others = []
        for b in bodies:
            if own is not None and b.id == own.id:
                continue
            d = math.dist((b.x, b.y, b.z), (lx, ly, lz))
            others.append((d, b))
        others.sort(key=lambda x: x[0])
        self._geometry_listener(t, lx, ly, yaw)
        # tracks for every vehicle in range (cues need them), voices for the nearest
        seen = set()
        vehicles = [(d, b) for d, b in others if b.kind != "walker"]
        heard = [] if not cfg.get("vehicles", True) else \
            [(d, b) for d, b in vehicles if d <= maxd][:int(cfg.get("max_vehicles", 16))]
        walkers = [b for d, b in others if b.kind == "walker"]
        for d, b in vehicles:
            if d > maxd:
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
            tr.events_now = events
        if own is not None:
            seen.add(own.id)
        for bid in [k for k in self.tracks if k not in seen]:
            del self.tracks[bid]
        keep = set()
        self.info = {}
        for d, b in heard:
            keep.add(b.id)
            self._vehicle(b, d, t, dt, lvx, lvy, walkers)
        for key in self.mixer.keys():
            if isinstance(key, tuple) and key[0] == "veh" and key[1] not in keep:
                self.mixer.remove(key)
        # the vehicles too far to hear one by one: their power into the distant hum
        far = np.zeros(ac.NB)
        for d, b in vehicles[len(heard):]:
            far += self._far_power(b, d)
        if own is not None and cfg.get("own_vehicle", True):
            self._own(own, own_control, t, dt)
        self._drones(drones or [], lvx, lvy, own_drone)
        self._walkers(walkers, t)
        self._aps(aps or [])
        if self.ambience is not None:
            self.levels.update(self.ambience.update(t, self.inside, self._trees_near(lx, ly), far, self.il))
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
                v.gain = self._g(at_distance(db, d2, 2.0) - (6.0 if self.inside else 0.0))
                v.pan, v.cutoff = self._pan(px - lx, py - ly, fx, fy), self._air(d2)
            else:
                v.gain = 0.0
        self.cue_voices = live
        if not self.reported and heard:
            self.reported = True
            self.log(f"sound: {len(heard)} vehicles heard within {maxd:.0f} m, reference {self.ref:.1f} dB"
                     + (f", {self.buildings.n} buildings" if self.buildings is not None else ""))

    def _velocities(self, bodies, t):
        """Velocities from the positions where the reported one is zero: the SUMO bridge moves its vehicles by
        transform, so CARLA (and frames.csv) report them standing still. Smoothed over ~0.15 s; jumps (a reset to
        the lane) are ignored."""
        if not hasattr(self, "_vel"):
            self._vel = {}
        out = []
        seen = {}
        for b in bodies:
            prev = self._vel.get(b.id)
            vx, vy = b.vx, b.vy
            ex, ey = (prev[3], prev[4]) if prev else (0.0, 0.0)
            if prev is not None and t - prev[0] > 1e-4:
                dt = t - prev[0]
                rx, ry = (b.x - prev[1]) / dt, (b.y - prev[2]) / dt
                if math.hypot(rx, ry) < 70.0:
                    a = 1.0 - math.exp(-dt / 0.15)
                    ex, ey = ex + (rx - ex) * a, ey + (ry - ey) * a
            seen[b.id] = (t, b.x, b.y, ex, ey)
            if math.hypot(vx, vy) < 0.05 and math.hypot(ex, ey) > 0.2:
                b = b._replace(vx=ex, vy=ey)
            out.append(b)
        self._vel = seen
        return out

    # -- geometry -------------------------------------------------------------------------------------------------
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

    def _dir(self, x, y, z):
        lx, ly, lz, fx, fy = self.lis[:5]
        dx, dy, dz = x - lx, y - ly, z - lz
        az = math.atan2(-dx * fy + dy * fx, dx * fx + dy * fy)
        el = math.atan2(dz, math.hypot(dx, dy))
        return az, el

    def _geometry_listener(self, t, lx, ly, yaw):
        if self.buildings is None:
            if self.mixer.reverb is not None and self.canyon is None:
                from .soundgeo import canyon_reverb
                st = self.cfg.get("street") or {}
                w, h = st.get("width_m", 20.0), st.get("height_m", 12.0)
                rt, self.rev_level = canyon_reverb(w, h, 2 if w else 0)
                self.mixer.reverb.set_rt(rt)
                self.canyon = (w, h, 2)
            return
        e, pos, t0 = self.facades
        if pos is None or math.dist(pos, (lx, ly)) > 2.0 or t - t0 > 0.5:
            idx, _ = self.buildings.facades_near(lx, ly, radius=float(self.cfg.get("facade_radius_m", 50.0)), k=8)
            self.facades = (idx, (lx, ly), t)
        if self.mixer.reverb is not None and t - self._canyon_t > 1.0:
            from .soundgeo import canyon_reverb
            self._canyon_t = t
            self.canyon = self.buildings.canyon(lx, ly, yaw)
            rt, self.rev_level = canyon_reverb(*self.canyon)
            self.mixer.reverb.set_rt(rt)

    def _propagation(self, key, x, y, z, hs, t, refl=None):
        """_Paths of a source at (x, y, z), hs m above the ground, cached per source (occlusion and reflections
        are recomputed every 0.15 s or 1.5 m)."""
        lx, ly, lz, fx, fy, hr, gz = self.lis
        p = _Paths()
        d3 = max(0.5, math.dist((x, y, z), (lx, ly, lz)))
        p.d = d3
        dp = math.hypot(x - lx, y - ly)
        base = 20.0 * math.log10(d3) + 11.0
        gr = ac.ground_attenuation(max(hs, 0.0), hr, dp, self.G, self.G, self.G)
        att = base + self.alpha * d3 + gr + self.il
        n_refl = self.n_refl if refl is None else min(refl, self.n_refl)
        occl = 0.0
        refls = []
        if self.buildings is not None and dp > 2.0:
            tr = self.tracks.get(key) if not isinstance(key, tuple) else None
            cache = getattr(tr, "geo", None) if tr is not None else self._geo_cache.get(key) \
                if hasattr(self, "_geo_cache") else None
            if cache is None or t - cache[0] > 0.15 or math.dist(cache[1], (x, y, lx, ly)) > 1.5:
                s, r = (x, y, gz + hs), (lx, ly, lz)
                occl, _nb = self.buildings.occlusion(s, r)
                refls = self.buildings.reflections(s, r, self.facades[0], n=n_refl) if n_refl else []
                cache = (t, (x, y, lx, ly), occl, refls)
                if tr is not None:
                    tr.geo = cache
                else:
                    if not hasattr(self, "_geo_cache"):
                        self._geo_cache = {}
                    self._geo_cache[key] = cache
            occl, refls = cache[2], cache[3]
        p.att[0] = att + occl
        p.az[0], p.el[0] = self._dir(x, y, z)
        for i, (L, (rx, ry), _e) in enumerate(refls[:2]):
            L = max(L, d3)
            p.att[1 + i] = 20.0 * math.log10(L) + 11.0 + self.alpha * L + gr + self.il + 1.0
            p.delay[1 + i] = (L - d3) / C_SOUND
            p.az[1 + i], p.el[1 + i] = self._dir(rx, ry, (z + lz) / 2.0)
        if self.mixer.reverb is not None:
            # the tail is the listener's street answering: sources further along it (or in other streets, behind
            # buildings) feed it less (the reverberant level falls along a street, Kang 2000)
            w = (self.canyon[0] if self.canyon and self.canyon[0] else 20.0) / 2.0
            occ = float(np.mean(occl)) if np.ndim(occl) else float(occl)
            p.att[3] = np.full(ac.NB, -self.rev_level + 20.0 * math.log10(max(1.0, d3 / w)) + 0.5 * occ) + self.il
        return p

    def _apply(self, voice, lw, paths, mute=1.0):
        """Sets every path of `voice` for a source of sound power `lw` (dB per band) seen through `paths`."""
        for i, pth in enumerate(voice.paths):
            slot = {"direct": 0, "reverb": 3}.get(pth.kind, None)
            if pth.kind == "refl":
                slot = 1 + sum(1 for q in voice.paths[:i] if q.kind == "refl")
            a = paths.att[slot] if slot is not None and slot < 4 else None
            if a is None or mute == 0.0:
                if pth.w is not None:
                    pth.w = np.zeros_like(pth.w)
                else:
                    pth.w = np.zeros(voice.K, np.float32)
                continue
            pth.set(voice.weights(lw - a, self.ref) * mute, paths.delay[slot], paths.az[slot], paths.el[slot])

    def _doppler(self, b, dx, dy, d, lvx, lvy):
        k = float(self.cfg.get("doppler", 1.0))
        if k <= 0 or d < 1e-3:
            return 1.0
        ux, uy = dx / d, dy / d               # listener -> source
        vs = b.vx * ux + b.vy * uy            # source moving away > 0
        vl = lvx * ux + lvy * uy              # listener moving towards the source > 0
        ratio = (C_SOUND + vl) / max(1.0, C_SOUND + vs)
        return 1.0 + k * (ratio - 1.0)

    def _trees_near(self, x, y):
        if self.trees is None or not len(self.trees):
            return float(self.cfg.get("trees", 0.4))
        d2 = (self.trees[:, 0] - x) ** 2 + (self.trees[:, 1] - y) ** 2
        return min(1.0, float(np.count_nonzero(d2 < 40.0 ** 2)) / 25.0)

    # -- vehicles ------------------------------------------------------------------------------------------------
    def _source_power(self, b, tr, dt):
        """(rolling L_W, propulsion L_W or None, rpm, load) per band for a vehicle."""
        kmh = tr.kmh if tr else 3.6 * math.hypot(b.vx, b.vy)
        acc = tr.accel if tr else 0.0
        wet = self.env.wet
        if b.kind == "bike":
            if kmh < 2.0:
                return None, None, 0.0, 0.0
            r, _p = ac.vehicle_power("1", max(kmh, 3.0), temp_c=self.env.temp, surface=self.surface, wet=wet,
                                     table=self.table)
            return r - 22.0, None, 0.0, 0.0
        cat = category(b.kind, b.type_id) or "1"
        roll, _ = ac.vehicle_power(cat, max(kmh, 3.0), temp_c=self.env.temp, surface=self.surface, wet=wet,
                                   table=self.table)
        if kmh < 2.0 or cat in ("4a", "4b"):
            roll = roll if cat not in ("4a", "4b") and kmh >= 2.0 else None
        if b.electric:
            return roll, None, 0.0, 0.0
        gk = b.kind if b.kind in GEARS else "car"
        *_r, a_full = GEARS[gk]
        v = kmh / 3.6
        load = max(0.0, min(1.0, (acc + 0.12 + 0.0004 * v * v) / a_full)) if acc > -0.3 else 0.0
        if tr is not None:
            if getattr(tr, "gearbox", None) is None:
                tr.gearbox = Gearbox(gk)
            tr.load = getattr(tr, "load", load) + (load - getattr(tr, "load", load)) * min(1.0, dt / 0.2)
            _g, rpm = tr.gearbox.step(kmh, tr.load, dt)
            load = tr.load
        else:
            _g, rpm = gear_for(kmh, load, gk)
        _r2, prop = ac.vehicle_power(cat, max(kmh, 20.0), accel=acc, temp_c=self.env.temp, surface=self.surface,
                                     table=self.table)
        # CNOSSOS assumes the engine speed typical of the speed (and its acceleration term covers the load); the
        # actual engine speed moves the level with ~35 lg(rpm) (engine noise rises 30-50 lg n): idling below
        # 20 km/h, and the gearbox's lag behind the gear a steady driver would pick at this load
        _g0, rpm_typ = gear_for(max(kmh, 20.0), load, gk)
        prop = prop + max(-18.0, min(5.0, 35.0 * math.log10(max(rpm, 300.0) / rpm_typ)))
        # CNOSSOS's propulsion is a fleet average over all driving; the engine's orders rise with its load (torque;
        # Pieren et al. 2016), so a car cruising or coasting (part load) is quieter than that average and one pulling
        # away louder (the acceleration term above): up to `engine_part_load_db` less below a load of 0.4
        if kmh >= 2.0:
            prop = prop - float(self.cfg.get("engine_part_load_db", 6.0)) * max(0.0, min(1.0, (0.4 - load) / 0.4))
        return roll, prop, rpm, load

    def _vehicle(self, b, d, t, dt, lvx, lvy, walkers):
        cfg = self.cfg
        tr = self.tracks.get(b.id)
        kmh = tr.kmh if tr else 3.6 * math.hypot(b.vx, b.vy)
        lx, ly, lz, fx, fy, hr, gz = self.lis
        dx, dy = b.x - lx, b.y - ly
        on = self._on
        roll, prop, rpm, load = self._source_power(b, tr, dt)
        # a vehicle that appears close by (SUMO puts one on an edge near the listener) fades in over 0.6 s
        # instead of popping up
        if tr is not None:
            if not hasattr(tr, "born"):
                tr.born = t if d < 60.0 and self.t_last is not None and t > 0.5 else -1e9
            on *= min(1.0, max(0.05, (t - tr.born) / 0.6))
        paths = self._propagation(b.id, b.x, b.y, b.z + 0.3, 0.05 if b.kind != "bike" else 0.3, t)
        dop = self._doppler(b, dx, dy, max(paths.d, 1e-3), lvx, lvy)
        voices = {}

        def voice(part, make):
            key = ("veh", b.id, part)
            v = self.mixer.get(key)
            if v is None:
                v = self.mixer.add(key, make())
            voices[part] = v
            return v

        info = {"kind": b.kind, "kmh": kmh, "d": d, "pan": self._pan(dx, dy, fx, fy), "electric": b.electric,
                "tyre_db": None, "engine_db": None, "avas_db": None, "rpm": rpm, "load": load}
        direct = paths.att[0]
        if roll is not None:
            v = voice("tyre", lambda: synth.BandNoise("pink", seed=b.id % 9973, flutter_db=0.6))
            v.rate = dop
            self._apply(v, roll, paths, on)
            info["tyre_db"] = ac.a_weighted(roll - direct)
        elif self.mixer.get(("veh", b.id, "tyre")) is not None:
            self._apply(self.mixer.get(("veh", b.id, "tyre")), np.full(ac.NB, -200.0), paths, 0.0)
        if prop is not None:
            gk = b.kind if b.kind in GEARS else "car"
            cyl, diesel = GEARS[gk][5], GEARS[gk][6]
            if gk == "car":
                diesel = (zlib.crc32(f"d{b.id}".encode()) % 1000) / 1000.0 < float(cfg.get("diesel_cars", 0.2))
            v = voice("engine", lambda: soundsamples.engine_voice(gk, cyl, diesel, seed=b.id % 9973, log=self.log,
                                                                   mode=cfg.get("engine_sound", "recorded")))
            v.set_rpm(rpm, load)
            v.rate = dop
            v.nlive = v.nh if d < 30.0 else v.nh // 2       # far synthesised engines: the lower orders carry it
            self._apply(v, prop, paths, on)
            info["engine_db"] = ac.a_weighted(prop - direct)
        # AVAS (UN R138 / EU 540/2014): electric cars up to 20 km/h
        av = cfg.get("avas") or {}
        if av.get("enabled", True) and b.kind in ("car", "van", "bus", "truck"):
            who = av.get("vehicles", "electric")
            if who == "all" or (who == "automated" and b.automated) or (who == "electric" and b.electric):
                until = float(av.get("until_kmh", 20.0))
                fade = 1.0 if kmh <= until - 5.0 else max(0.0, (until - kmh) / 5.0)
                if kmh < 0.5 and not av.get("stopped", False):
                    fade = 0.0
                v = voice("avas", lambda: synth.AvasVoice(seed=b.id % 997))
                v.base = v.base0 * (1.0 + 0.01 * kmh)
                v.rate = dop
                lw = _shape(synth.AvasVoice.SHAPE, _lw_from_lp(float(av.get("db", 56.0))) +
                            20 * math.log10(max(fade, 1e-4)))
                self._apply(v, lw, paths, on if fade > 0 else 0.0)
                if fade > 0:
                    info["avas_db"] = ac.a_weighted(lw - direct)
        # emergency vehicles: the siren with the special lights (or always while moving)
        sir = cfg.get("sirens", "lights")
        if sir and sir != "off" and any(k in (b.type_id or "") for k in EMERGENCY):
            on_s = (int(b.lights or 0) & SPECIAL1) if sir == "lights" else (sir == "always" and kmh > 5)
            v = voice("siren", lambda: synth.ShapedMono(self.siren_buf, loop=True, start=b.id * 997))
            v.rate = dop
            lw = v.band_db(float(cfg.get("siren_lw_dba", 125.0)))
            self._apply(v, lw, paths, on if on_s else 0.0)
            info["siren"] = bool(on_s)
        # bicycles: the freewheel ratchet while coasting, a bell near pedestrians
        if b.kind == "bike" and tr is not None and kmh > 6.0:
            coast = tr.accel < -0.05 or (zlib.crc32(f"{b.id}{int(t / 4)}".encode()) % 3 == 0)
            v = voice("ratchet", lambda: synth.EventVoice(self.ratchet, seed=b.id, jitter=0.05, gain_db_sd=2.0))
            wheel_hz = kmh / 3.6 / (2 * math.pi * 0.34)
            v.period = 1.0 / max(1.0, 24.0 * wheel_hz) if coast else None
            self._apply(v, v.band_db(_lw_from_lp(48.0, 1.0)), paths, on)
            bells = float((cfg.get("bicycles") or {}).get("bells", 0.03))
            if bells > 0 and self._ped_ahead(b, walkers, 15.0) and self.rng.random() < bells * dt * 10 \
                    and t - getattr(tr, "bell_t", -1e9) > 6.0:
                tr.bell_t = t
                self._oneshot(("veh", b.id, "bell", int(t * 10)), self.bell_buf, _lw_from_lp(72.0, 2.0), paths)
                self.bell_played = True
                self.on_cue("sound_cue", "bicycle_bell", f"actor {b.id}")
        # horn: a car braking hard with a pedestrian ahead (optional)
        hc = cfg.get("horn")
        if hc and b.kind in ("car", "van", "bus", "truck") and tr is not None and tr.accel < -4.0 and \
                t - getattr(tr, "horn_t", -1e9) > 5.0 and self._ped_ahead(b, walkers, 20.0):
            tr.horn_t = t
            self._oneshot(("veh", b.id, "horn", int(t * 10)), self.horn_buf, _lw_from_lp(100.0, 7.0), paths)
            self.on_cue("sound_cue", "horn", f"actor {b.id}")
        # buses: an air hiss when they stop (doors, kneeling) and drive off (brakes released)
        if b.kind == "bus" and tr is not None:
            for ev in getattr(tr, "events_now", ()):
                if ev in ("stopped", "driving_off"):
                    self._oneshot(("veh", b.id, "hiss", int(t * 10)), self.hiss_buf, _lw_from_lp(80.0, 2.0), paths)
        self.info[b.id] = info

    def _ped_ahead(self, b, walkers, reach):
        yaw = math.radians(b.yaw)
        cands = [(w.x, w.y) for w in walkers]
        if self.kind == "pedestrian":
            cands.append(self.lis[:2])
        for wx, wy in cands:
            dx, dy = wx - b.x, wy - b.y
            fwd, side = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
            if 0 < fwd < reach and abs(side) < 3.0:
                return True
        return False

    def _oneshot(self, key, buf, lw_dba, paths):
        v = self.mixer.add(key, synth.ShapedMono(buf, kinds=("direct", "refl", "refl", "reverb")))
        self._apply(v, v.band_db(lw_dba), paths, self._on)
        return v

    def _far_power(self, b, d):
        """Power per band (full-scale units) of a far vehicle at the listener: spreading, air, ground and, in
        town, a typical 10 dB of screening by the buildings."""
        tr = self.tracks.get(b.id)
        if b.kind == "bike":
            return np.zeros(ac.NB)
        roll, prop, _rpm, _load = self._source_power(b, tr, 0.05)
        parts = [x for x in (roll, prop) if x is not None]
        if not parts:
            return np.zeros(ac.NB)
        lw = ac.db_sum(np.vstack(parts), axis=0)
        att = 20 * math.log10(max(d, 1.0)) + 11.0 + self.alpha * d - 3.0 + \
            (10.0 if self.buildings is not None else 0.0) + self.il
        return synth.band_db_to_power(lw - att, self.ref)

    # -- drones ---------------------------------------------------------------------------------------------------
    def _drones(self, drones, lvx, lvy, own_drone=None):
        """The nearest `max_drones` drones within `drone_distance_m` (300 m) as rotor voices: the drone's sound
        power (EU 2019/945 class, dronemodel.py) rising with the thrust, through the same propagation as vehicles."""
        cfg = self.cfg
        lx, ly, lz = self.lis[:3]
        maxd = float(cfg.get("drone_distance_m", 300.0))
        near = sorted(((math.dist((d[1], d[2], d[3]), (lx, ly, lz)), d) for d in drones), key=lambda x: x[0])
        near = [(dist, d) for dist, d in near if dist <= maxd][:int(cfg.get("max_drones", 12))]
        keep = set()
        for dist, d in near:
            did, x, y, z, vx, vy, vz, bpf, thrust, lwa = d[:10]
            blades = d[10] if len(d) > 10 else 2
            own = own_drone is not None and did == own_drone
            if own and not cfg.get("record_own_drone", True):
                continue
            key = ("drone", did)
            keep.add(key)
            v = self.mixer.get(key)
            if v is None:
                v = self.mixer.add(key, synth.RotorVoice(blades, seed=abs(int(did)) % 997))
            v.set_bpf(max(20.0, bpf))
            body = Body(did, "drone", "drone", x, y, z, vx, vy, vz, 0.0)
            on = self._on if bpf > 5.0 else 0.0
            t_ratio = max(0.05, float(thrust))
            lw = _shape(synth.RotorVoice.SHAPE, float(lwa) + max(-15.0, min(10.0, 25.0 * math.log10(t_ratio))))
            if own:          # the camera under the drone's nose: the rotors ~0.4 m away, free field
                p = _Paths()
                p.d = 0.4
                p.att[0] = np.full(ac.NB, 20.0 * math.log10(0.4) + 11.0)
                p.el[0] = 0.8
                v.rate = 1.0
            else:
                p = self._propagation(("drone", did), x, y, z, max(0.5, z - self.lis[6]), self.t_last or 0.0,
                                      refl=1)
                v.rate = self._doppler(body, x - lx, y - ly, max(p.d, 1e-3), lvx, lvy)
            self._apply(v, lw, p, on)
            self.info[key] = {"kind": "drone", "d": dist, "db": ac.a_weighted(lw - p.att[0]) if on else None,
                              "bpf": bpf}
        for key in self.mixer.keys():
            if isinstance(key, tuple) and key[0] == "drone" and key not in keep:
                self.mixer.remove(key)

    # -- pedestrians and signals ---------------------------------------------------------------------------------
    def _walkers(self, walkers, t):
        """Footsteps of the walkers within 20 m (nearest 8) and the participant's own."""
        cfg = self.cfg
        keep = set()
        if cfg.get("walkers", True):
            lx, ly, lz = self.lis[:3]
            near = sorted(((math.dist((w.x, w.y, w.z), (lx, ly, lz)), w) for w in walkers), key=lambda x: x[0])
            for dist, w in near[:8]:
                if dist > 20.0:
                    break
                sp = math.hypot(w.vx, w.vy)
                key = ("steps", w.id)
                keep.add(key)
                v = self.mixer.get(key)
                if v is None:
                    v = self.mixer.add(key, synth.EventVoice(self.steps, seed=w.id % 997, jitter=0.04,
                                                             rate_sd=0.04 if self.steps_recorded else 0.0,
                                                             kinds=("direct", "reverb")))
                    v.next = self.rng.uniform(0, 0.5) * audio.RATE
                v.period = 1.0 / (0.85 + 0.75 * sp) if sp > 0.25 else None
                p = self._propagation(key, w.x, w.y, w.z, 0.0, t, refl=0)
                self._apply(v, v.band_dba(self._step_lw()), p, self._on)
            if self.kind == "pedestrian":
                key = ("steps", "own")
                keep.add(key)
                v = self.mixer.get(key)
                if v is None:
                    v = self.mixer.add(key, synth.EventVoice(self.steps, seed=5, jitter=0.04, gain_db_sd=1.5,
                                                             rate_sd=0.04 if self.steps_recorded else 0.0,
                                                             kinds=("direct",)))
                sp = math.hypot(*self.lv)
                v.period = 1.0 / (0.85 + 0.75 * sp) if 0.25 < sp < 4.0 else None
                p = _Paths()
                p.att[0] = np.full(ac.NB, 20.0 * math.log10(self.lis[5]) + 8.0 - 3.0)
                p.el[0] = -1.3
                self._apply(v, v.band_dba(self._step_lw()), p, self._on)
                self.own_steps = {"dba": self._step_lw() - float(p.att[0][0]), "period": v.period}
        for key in self.mixer.keys():
            if isinstance(key, tuple) and key[0] == "steps" and key not in keep:
                self.mixer.remove(key)

    def _step_lw(self):
        """A-weighted sound power of one footstep (its loudest 100 ms): `footstep_db` dB(A) at 1 m for hard soles on
        paving, less for trainers or grass."""
        fw = self.cfg.get("footsteps", "shoes")
        return _lw_from_lp(float(self.cfg.get("footstep_db", 61.0)), 1.0) + {"trainers": -9.0, "grass": -6.0}.get(fw, 0.0)

    def _aps(self, aps):
        """Accessible pedestrian signals: a tick every second while the crossing is red (the locator) and about 8
        a second while it is green, from the push-button box on the signal pole (NL 'rateltikker', German
        Akustischer Signalgeber); `aps_db` at 1 m."""
        cfg = self.cfg
        keep = set()
        if cfg.get("aps", True):
            for i, (x, y, z, walk) in enumerate(aps[:2]):
                key = ("aps", i)
                keep.add(key)
                v = self.mixer.get(key)
                if v is None:
                    v = self.mixer.add(key, synth.EventVoice(self.aps_ticks, seed=40 + i, jitter=0.0, gain_db_sd=0.3))
                if v.period != (0.125 if walk else 1.0):
                    self.on_cue("aps", "walk" if walk else "wait", f"pole {i} at {x:.0f},{y:.0f}")
                v.period = 0.125 if walk else 1.0
                p = self._propagation(key, x, y, z + 1.0, 1.0, self.t_last or 0.0, refl=1)
                self._apply(v, v.band_db(_lw_from_lp(float(cfg.get("aps_db", 65.0)), 1.0)), p, self._on)
                self.info[key] = {"kind": "aps", "walk": bool(walk), "d": p.d}
        for key in self.mixer.keys():
            if isinstance(key, tuple) and key[0] == "aps" and key not in keep:
                self.mixer.remove(key)

    # -- the own car ----------------------------------------------------------------------------------------------
    def _own(self, own, ctl, t, dt):
        """The own car from inside: its tyres and engine (CNOSSOS power at ~1.5 m, through the cabin's insertion
        loss), wind noise (60 lg v), the indicator relay."""
        cfg = self.cfg
        tr = self.tracks.get(own.id)
        if tr is None:
            tr = self.tracks[own.id] = _Track(own, t)
        else:
            tr.step(own, t, dt)
        kmh = tr.kmh
        throttle = float(getattr(ctl, "throttle", 0.0) or 0.0) if ctl is not None else 0.0
        if ctl is not None and throttle > 0.05:      # the driver's pedal is a better load signal than the speed
            tr.accel = max(tr.accel, 2.5 * throttle - 0.5)
        roll, prop, rpm, load = self._source_power(own, tr, dt)
        cab = ac.CABIN_IL_BUS if own.kind == "bus" else ac.CABIN_IL
        near = 20.0 * math.log10(float(cfg.get("cabin_distance_m", 1.5))) + 11.0 - 3.0
        p = _Paths()
        p.d = 1.5
        on = self._on
        info = {"kmh": kmh, "rpm": rpm, "load": load, "tyre_db": None, "engine_db": None, "wind_db": None}
        red = float(cfg.get("interior_offset_db", 0.0))
        p.att[0] = near + cab - red
        p.el[0] = -0.6
        pe = _Paths()
        pe.att[0] = near + (ac.CABIN_IL_BUS if own.kind == "bus" else ac.CABIN_IL_ENGINE) - red
        pe.el[0] = -0.3
        gk = own.kind if own.kind in GEARS else "car"
        cyl, diesel = GEARS[gk][5], GEARS[gk][6]
        if gk == "car":
            diesel = bool(cfg.get("own_diesel", False))
        for part, lw, make in (("tyre", roll, lambda: synth.BandNoise("pink", seed=3, flutter_db=0.5,
                                                                       kinds=("direct",))),
                               ("engine", prop, lambda: soundsamples.engine_voice(
                                   gk, cyl, diesel, seed=1, log=self.log, mode=cfg.get("engine_sound", "recorded"),
                                   kinds=("direct",)))):
            key = ("own", part)
            v = self.mixer.get(key)
            if lw is None:
                if v is not None:
                    self._apply(v, np.full(ac.NB, -200.0), p, 0.0)
                continue
            if v is None:
                v = self.mixer.add(key, make())
            q = pe if part == "engine" else p
            if part == "engine":
                v.set_rpm(rpm, load)
            self._apply(v, lw, q, on)
            info[f"{part}_db"] = ac.a_weighted(lw - q.att[0])
        wind = ac.wind_interior(kmh, float(cfg.get("wind_db_at_100", 60.0)))
        key = ("own", "wind")
        v = self.mixer.get(key)
        if wind is not None or v is not None:
            if v is None:
                v = self.mixer.add(key, synth.BandNoise("pink", seed=13, flutter_db=1.2, flutter_hz=0.4,
                                                        kinds=("direct",)))
            q = _Paths()
            q.att[0] = np.zeros(ac.NB)
            self._apply(v, wind if wind is not None else np.full(ac.NB, -200.0), q, on if wind is not None else 0.0)
            info["wind_db"] = ac.a_weighted(wind) if wind is not None else None
        self._own_electric(own, tr, kmh, ctl, cab, red, on, info)
        tot = [x for x in (info["tyre_db"], info["engine_db"], info["wind_db"], info.get("motor_db"),
                           info.get("avas_db")) if x is not None]
        info["total_dba"] = float(ac.db_sum(tot)) if tot else None
        self.info["own"] = info
        # turn signal: a relay click every 1/3 s while a blinker is on (the lamp's on and off edges at 1.5 Hz)
        lit = bool(int(own.lights or 0) & (LEFT_BLINKER | RIGHT_BLINKER))
        if lit and (not self.blink_was or (self.blink_next is not None and t >= self.blink_next)):
            self.play("tick", float(cfg.get("blinker_db", 58.0)), name="blinker_tick")
            self.blink_next = t + 1.0 / 3.0
        self.blink_was = lit

    def _own_electric(self, own, tr, kmh, ctl, cab, red, on, info):
        """An electric own car from inside: no engine, the motor and reduction gear's whine rising with speed and
        torque (soundsamples.MotorVoice, `ev_motor_db` at 50 km/h), and its AVAS heard faintly through the cabin
        (the speaker sits behind the front bumper, ~2.5 m from the driver's ears)."""
        cfg = self.cfg
        info["motor_db"] = info["avas_db"] = None
        lv = None
        if own.electric and own.kind in ("car", "van", "bus", "truck"):
            throttle = float(getattr(ctl, "throttle", 0.0) or 0.0) if ctl is not None else 0.0
            load = max(throttle, max(0.0, min(1.0, (tr.accel + 0.1) / 3.0)))
            tr.motor_load = getattr(tr, "motor_load", load) + (load - getattr(tr, "motor_load", load)) * 0.3
            at50 = float(cfg.get("ev_motor_db", 38.0))
            lv = soundsamples.motor_dba(kmh, tr.motor_load, at50) if at50 > 0 else None
        key = ("own", "motor")
        v = self.mixer.get(key)
        if lv is not None or v is not None:
            if v is None:
                v = self.mixer.add(key, soundsamples.MotorVoice(float(cfg.get("ev_motor_rpm_per_kmh", 72.0)),
                                                                 seed=2, kinds=("direct",)))
            v.set_speed(kmh, getattr(tr, "motor_load", 0.0))
            q = _Paths()
            q.att[0] = np.zeros(ac.NB)
            q.el[0] = -0.4
            self._apply(v, _shape(soundsamples.MotorVoice.SHAPE, lv) if lv is not None else np.full(ac.NB, -200.0),
                        q, on if lv is not None else 0.0)
            info["motor_db"] = lv
        # the own AVAS (UN R138: up to 20 km/h), through the cabin
        av = cfg.get("avas") or {}
        fade = 0.0
        if own.electric and av.get("enabled", True) and av.get("vehicles", "electric") in ("electric", "all") \
                and own.kind in ("car", "van", "bus", "truck"):
            until = float(av.get("until_kmh", 20.0))
            fade = 1.0 if kmh <= until - 5.0 else max(0.0, (until - kmh) / 5.0)
            if kmh < 0.5 and not av.get("stopped", False):
                fade = 0.0
        key = ("own", "avas")
        v = self.mixer.get(key)
        if fade > 0 or v is not None:
            if v is None:
                v = self.mixer.add(key, synth.AvasVoice(seed=7, kinds=("direct",)))
            v.base = v.base0 * (1.0 + 0.01 * kmh)
            lw = _shape(synth.AvasVoice.SHAPE, _lw_from_lp(float(av.get("db", 56.0))) + 20 * math.log10(max(fade, 1e-4)))
            q = _Paths()
            q.att[0] = 20.0 * math.log10(2.5) + 11.0 - 3.0 + cab - red
            q.el[0] = -0.5
            self._apply(v, lw, q, on if fade > 0 else 0.0)
            info["avas_db"] = ac.a_weighted(lw - q.att[0]) if fade > 0 else None

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

    def used_credits(self):
        """The attributions of the recordings (and HRTF set) actually heard so far."""
        used = set(self.ambience.used) if self.ambience is not None else set()
        if getattr(self, "bell_played", False):
            used.add("bicycle_bell")
        used.add("hrtf")
        return {k: v for k, v in self.credits.items() if k in used}

    def close(self):
        for k in self.mixer.keys():
            self.mixer.remove(k)


# -- from a CARLA world -----------------------------------------------------------------------------------------

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
