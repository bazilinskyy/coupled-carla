"""Sounds from recordings shipped with the code (coupledcarla/data/sounds/): engines from recordings of real engines,
the game-audio way, and footsteps; and an electric car's motor whine (a model).

coupledcarla/data/sounds/engines/ holds a few seamless loops per engine type (petrol, diesel, motorbike), each a
steady stretch of one real engine's recording at a known firing frequency (cylinders/2 x rpm/60), cut by
tools/sound_samples.py (sources and licences in data/sounds/SOURCES.md). SampleEngineVoice plays the two loops
around the wanted firing frequency, each resampled to it (pitch follows rpm continuously; the loop's speed of play is
the ratio of the frequencies), and cross-fades them by log frequency with equal power.

The recordings give the timbre only: the level stays CNOSSOS-EU's, as for the synthesised engine. Each loop is split
into the 8 octave bands (63 Hz-8 kHz) at load time, every band row at the same RMS, so a band row played at weight w
sounds at reference + 20 lg w dB SPL like every other basis row (synth.py); weights() gives each row the power the
band levels at the ears ask for. A row played `ratio` times faster lies `log2(ratio)` octaves higher, so its weight
reads the band levels there (interpolated), and the whole is scaled to keep the A-weighted level of the band levels.
Basis: 2 x 8 rows, the lower and the upper loop of the current pair (a loop keeps its slot, index parity, so a pair
change swaps only the silent one); the cross-fade gains are in the weights, which the mixer ramps over each block.

Electric cars have no engine; MotorVoice is the whine of the traction motor and its reduction gear (tonal orders of
the motor shaft speed, which is proportional to the road speed through a single fixed gear).

If the loops cannot be read or decoded, engine_voice() falls back to the synthesised synth.EngineVoice (one log line).

Footsteps: single recorded steps (heel and toe) of a walker per surface or footwear (shoes: leather soles on
concrete paving; trainers; grass), one 0.5 s slot each in one file; footstep_buffers() returns them normalised like
synth.footstep_buffers (loudest 100 ms at full-scale RMS), or None (the caller synthesises them).
"""
import json
import math
from pathlib import Path

import numpy as np

from . import acoustics as ac
from . import audio, synth

RATE = audio.RATE
NB = ac.NB
DATA = Path(__file__).resolve().parent / "data" / "sounds"
MANIFEST = DATA / "sounds.json"
# engine kind (soundscape.GEARS) -> loop set; cars are petrol unless chosen diesel (sound.diesel_cars)
KIND_SET = {"car": "petrol", "van": "diesel", "bus": "diesel", "truck": "diesel", "motorbike": "motorbike"}
_SETS = {}
_said = set()


def manifest(path=None):
    try:
        return json.loads(Path(path or MANIFEST).read_text())
    except (OSError, ValueError):
        return {}


def decode(path, rate=RATE):
    """A sound file as mono float64 at `rate`: WAV with the standard library, anything else decoded once with
    soundfetch's decoder (ffmpeg, else pygame) into the sound cache."""
    path = Path(path)
    if path.suffix.lower() != ".wav":
        from . import soundfetch
        import hashlib
        h = hashlib.sha1(path.read_bytes()).hexdigest()[:12]
        dst = soundfetch.cache_dir() / "decoded" / f"{path.parent.name}_{path.stem}_{h}_{rate}.wav"
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            tmp = dst.with_name(dst.stem + ".part.wav")
            if not soundfetch._decode(path, tmp, rate) or not tmp.exists():
                raise RuntimeError(f"cannot decode {path.name} (no ffmpeg, pygame failed)")
            tmp.replace(dst)
        path = dst
    import wave
    with wave.open(str(path)) as w:
        ch, width, fr = w.getnchannels(), w.getsampwidth(), w.getframerate()
        raw = w.readframes(w.getnframes())
    if width != 2:
        raise RuntimeError(f"{path.name}: {8 * width}-bit WAV (16-bit expected)")
    x = np.frombuffer(raw, "<i2").astype(np.float64).reshape(-1, ch).mean(axis=1) / 32768.0
    if fr != rate:
        x = np.interp(np.arange(0, len(x) - 1, fr / rate), np.arange(len(x)), x)
    return x


SHIFTS = np.linspace(-2.0, 2.0, 81)       # octaves a loop may be played off its pitch (A-weighting table)


def row_profile(rows, rate=RATE):
    """Per band row of a loop: where its power lies (power-weighted mean of log2(f / 63 Hz): band index units) and
    its A-weighting (power ratio) when played SHIFTS octaves higher, from the row's own spectrum in 1/24-octave bins
    (a row's power is rarely in the middle of its octave: an engine's 63 Hz row may sit at its upper edge, 5 dB less
    A-attenuated than the band's centre)."""
    from .soundrec import a_weighting_db
    n = rows.shape[1]
    f = np.fft.rfftfreq(n, 1.0 / rate)
    oc = np.log2(np.maximum(f, 1.0) / ac.EXACT[0])               # octaves above 63 Hz
    edges = np.arange(-2.0, 9.0 + 1e-9, 1.0 / 24.0)
    mid = 0.5 * (edges[1:] + edges[:-1])
    fmid = ac.EXACT[0] * 2.0 ** mid
    A = 10.0 ** (a_weighting_db(fmid[None, :] * 2.0 ** SHIFTS[:, None]) / 10.0)     # (shifts, bins)
    cent = np.zeros(NB)
    agrid = np.zeros((NB, len(SHIFTS)))
    for b in range(NB):
        p = np.abs(np.fft.rfft(rows[b].astype(np.float64))) ** 2
        h = np.histogram(oc, edges, weights=p)[0]
        tot = max(float(h.sum()), 1e-30)
        cent[b] = float(np.sum(h * mid)) / tot
        agrid[b] = A @ h / tot
    return cent, agrid


class EngineSet:
    """The loops of one engine type: firing frequencies (ascending) and each loop's band rows (NB, n + 1), the last
    column a copy of the first (for interpolation across the loop's end), with row_profile() of each."""

    def __init__(self, name, firing_hz, buffers, credit=""):
        order = np.argsort(firing_hz)
        self.name = name
        self.firing = np.asarray(firing_hz, float)[order]
        self.rows, self.share, self.cent, self.agrid = [], [], [], []
        for i in order:
            rows, share = synth.band_split(buffers[i])
            self.rows.append(np.concatenate([rows, rows[:, :1]], axis=1))
            self.share.append(share)
            c, a = row_profile(rows)
            self.cent.append(c)
            self.agrid.append(a)
        self.lf = np.log(self.firing)
        self.credit = credit

    def __len__(self):
        return len(self.firing)

    def pair(self, f):
        """(lower index, upper index, weight of the upper 0..1) of the loops around firing frequency f."""
        if len(self.firing) == 1 or f <= self.firing[0]:
            return 0, 0, 0.0
        if f >= self.firing[-1]:
            k = len(self.firing) - 1
            return k, k, 0.0
        k = int(np.searchsorted(self.firing, f) - 1)
        a = (math.log(f) - self.lf[k]) / (self.lf[k + 1] - self.lf[k])
        return k, k + 1, float(min(1.0, max(0.0, a)))


def load_set(name, log=print, path=None):
    """The EngineSet `name` from data/sounds (cached), or None (logged once) when its files are missing or cannot be
    decoded."""
    key = (name, str(path or MANIFEST))
    if key in _SETS:
        return _SETS[key]
    es = None
    try:
        man = manifest(path)
        spec = (man.get("engines") or {})[name]
        base = Path(path or MANIFEST).parent
        bufs = [decode(base / lp["file"]) for lp in spec["loops"]]
        es = EngineSet(name, [lp["firing_hz"] for lp in spec["loops"]], bufs, spec.get("source_engine", ""))
    except Exception as e:  # noqa: BLE001 - a missing or undecodable file: the synthesised engine instead
        if key not in _said:
            _said.add(key)
            log(f"sound: no recorded {name} engine ({type(e).__name__}: {e}); synthesised engine instead")
    _SETS[key] = es
    return es


class SampleEngineVoice(synth.SpatialVoice):
    """An engine from looped recordings (see the module doc). set_rpm() as synth.EngineVoice; `rate` is Doppler."""

    def __init__(self, eset, cylinders=4, seed=0, **kw):
        super().__init__(**kw)
        self.es = eset
        self.K = 2 * NB
        self.cyl = int(cylinders)
        self.rpm, self.load = 800.0, 0.2
        self.base = self.cyl / 2.0 * self.rpm / 60.0
        self.nh = 0                       # (the synthesised voice's partial count: the soundscape thins far ones)
        self.nlive = 0
        rng = np.random.default_rng(seed)
        self.pos = [float(rng.uniform(0, r.shape[1] - 1)) for r in eset.rows]
        self.rho = [None] * len(eset)     # last speed of play of each loop (ramped across a block)
        self._pair()

    def set_rpm(self, rpm, load):
        self.rpm, self.load = float(rpm), max(0.0, min(1.0, float(load)))
        self.base = max(1.0, self.cyl / 2.0 * self.rpm / 60.0)
        self._pair()

    def _pair(self):
        lo, hi, a = self.es.pair(self.base)
        g_lo, g_hi = math.cos(0.5 * math.pi * a), math.sin(0.5 * math.pi * a)
        slots = [None, None]
        gains = [0.0, 0.0]
        slots[lo % 2], gains[lo % 2] = lo, g_lo
        if hi != lo:
            slots[hi % 2], gains[hi % 2] = hi, g_hi
        self.slots, self.gains = slots, gains

    def ratio(self, i):
        return self.base / self.es.firing[i]

    def basis(self, n):
        X = np.zeros((self.K, n), np.float32)
        for s, i in enumerate(self.slots):
            if i is None:
                continue
            rows = self.es.rows[i]
            m = rows.shape[1] - 1
            r1 = self.ratio(i) * self.rate
            r0 = self.rho[i] if self.rho[i] is not None else r1
            self.rho[i] = r1
            steps = r0 + (r1 - r0) * (np.arange(n) + 0.5) / n
            idx = self.pos[i] + np.cumsum(steps) - steps[0]
            self.pos[i] = float((idx[-1] + steps[-1]) % m)
            idx %= m
            i0 = idx.astype(np.int64)
            fr = (idx - i0).astype(np.float32)
            X[s * NB:(s + 1) * NB] = rows[:, i0] * (1.0 - fr) + rows[:, i0 + 1] * fr
        for j in range(len(self.rho)):     # a loop out of the pair starts afresh at its own speed next time
            if j not in self.slots:
                self.rho[j] = None
        return X

    def advance(self, n):
        self.basis(n)

    def weights(self, band_db, ref):
        P = synth.band_db_to_power(band_db, ref)
        w = np.zeros(self.K, np.float32)
        if not np.any(P > 0):
            return w
        lp = np.log10(np.maximum(P, 1e-30))
        aw = 10.0 ** (ac.A_WEIGHT / 10.0)
        want = float(np.sum(P * aw))
        bands = np.arange(NB, dtype=float)
        have = 0.0
        parts = []
        for s, i in enumerate(self.slots):
            if i is None or self.gains[s] <= 0.0:
                continue
            shift = math.log2(max(1e-3, self.ratio(i)))
            at = np.clip(self.es.cent[i] + shift, 0.0, NB - 1.0)      # where each row's power now lies
            p = 10.0 ** np.interp(at, bands, lp)
            k_s = min(max((shift - SHIFTS[0]) / (SHIFTS[1] - SHIFTS[0]), 0.0), len(SHIFTS) - 1.001)
            j = int(k_s)
            a = self.es.agrid[i][:, j] * (1 - (k_s - j)) + self.es.agrid[i][:, j + 1] * (k_s - j)
            g2 = self.gains[s] ** 2
            have += g2 * float(np.sum(p * a))
            parts.append((s, g2 * p))
        k = want / have if have > 0 else 0.0
        for s, p in parts:
            w[s * NB:(s + 1) * NB] = np.sqrt(p * k)
        return w


def engine_voice(kind, cylinders, diesel, seed=0, log=print, mode="recorded", **kw):
    """The voice of a combustion engine: recorded loops (`mode` "recorded", the default) of the engine type of this
    kind (cars: petrol, or diesel when `diesel`), or the synthesised synth.EngineVoice ("synth", or when the loops
    are missing)."""
    if mode != "synth":
        name = "diesel" if diesel and kind in ("car", "van", "bus", "truck") else KIND_SET.get(kind, "petrol")
        es = load_set(name, log)
        if es is not None:
            return SampleEngineVoice(es, cylinders, seed=seed, **kw)
    return synth.EngineVoice(cylinders, diesel, seed=seed, **kw)


class MotorVoice(synth._Partials):
    """The whine of an electric car's traction motor and reduction gear: tones at orders of the motor shaft
    frequency (motor rpm = `rpm_per_kmh` x road speed through the single fixed gear) over a little broadband noise.
    Orders of a permanent-magnet synchronous motor with 3 pole pairs (6 poles): the electrical frequency (order 6) and
    its 6th and 12th harmonics (orders 36, 72; torque ripple) and the second harmonic (12); the pinion's gear mesh
    (order 25, a 25-tooth input gear). The orders grow louder with torque (`load`): electromagnetic forces scale with
    the current. 72 rpm per km/h is a Tesla Model 3's rear motor (9:1 reduction, 0.70 m tyres)."""
    ORDERS = [6.0, 12.0, 25.0, 36.0, 50.0, 72.0]
    REL = np.array([0.5, 0.25, 1.0, 0.8, 0.3, 0.35])
    SHAPE = np.array([-40.0, -30.0, -14.0, -4.0, 0.0, -1.0, -5.0, -14.0])

    def __init__(self, rpm_per_kmh=72.0, seed=0, **kw):
        super().__init__(self.ORDERS, base=10.0, seed=seed, jitter_f=0.0005, jitter_a=0.06, noise_kind="white", **kw)
        self.rpm_per_kmh = float(rpm_per_kmh)
        self.share = np.full(NB, 0.9)
        self.kmh, self.load = 0.0, 0.0

    def set_speed(self, kmh, load=0.0):
        self.kmh, self.load = max(0.0, float(kmh)), max(0.0, min(1.0, float(load)))
        self.base = max(0.5, self.kmh * self.rpm_per_kmh / 60.0)

    def weights(self, band_db, ref):
        P = synth.band_db_to_power(band_db, ref)
        w = np.zeros(self.K, np.float32)
        w[:self.nh] = self.partial_weights(P, self.share, self.REL)
        w[self.nh:] = np.sqrt(P * (1.0 - self.share))
        return w


def motor_dba(kmh, load, at50=38.0):
    """A-weighted level of an electric car's motor whine at the driver's ears: `at50` at 50 km/h at part load,
    rising 20 lg v with speed and up to 6 dB with torque (load 0..1); none below 3 km/h. Interior measurements of
    electric cars put the motor's tonal orders a few dB under the tyre and wind noise at urban speeds, most
    noticeable when accelerating."""
    if kmh < 3.0:
        return None
    return at50 + 20.0 * math.log10(kmh / 50.0) + 8.0 * (max(0.0, min(1.0, load)) - 0.25)


_STEPS = {}


def footstep_buffers(surface="shoes", log=print, path=None, wet=False):
    """The recorded single steps of `surface` (shoes, trainers, grass) as a list of mono buffers normalised like a cue,
    or None when missing (logged once). Wet ground adds a little splash (synthesised) to each."""
    key = (surface, str(path or MANIFEST), bool(wet))
    if key in _STEPS:
        return _STEPS[key]
    out = None
    try:
        man = manifest(path)
        spec = (man.get("footsteps") or {})[surface]
        base = Path(path or MANIFEST).parent
        x = decode(base / spec["file"])
        slot = int(round(float(spec["slot_s"]) * RATE))
        out = []
        rng = np.random.default_rng(31)
        for k in range(int(spec["steps"])):
            st = x[k * slot:(k + 1) * slot]
            nz = np.flatnonzero(np.abs(st) > 1e-4)
            if len(nz) < RATE // 20:
                continue
            st = st[:nz[-1] + 1].copy()
            if wet:
                n = int(0.12 * RATE)
                sp = np.fft.irfft(np.fft.rfft(rng.standard_normal(n)) * (np.fft.rfftfreq(n, 1 / RATE) > 2000), n)
                sp *= np.exp(-np.arange(n) / RATE / 0.04) * 0.15 * float(np.max(np.abs(st))) / max(1e-9, np.max(np.abs(sp)))
                st[:min(n, len(st))] += sp[:len(st)]
            out.append(audio._peak_norm(st, RATE))
        if not out:
            raise ValueError("no steps in the file")
    except Exception as e:  # noqa: BLE001 - the synthesised steps instead
        if key not in _said:
            _said.add(key)
            log(f"sound: no recorded footsteps for {surface} ({type(e).__name__}: {e}); synthesised steps instead")
        out = None
    _STEPS[key] = out
    return out
