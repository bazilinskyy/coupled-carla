"""Sound for the stations: synthesised sources, a block mixer with panning, pitch and low-pass, and its output.

Everything is generated with numpy (no audio files in the repository, see data/sound_credits.md). Levels are
calibrated as in coupled-sim (#135): every source buffer has the RMS of a full-scale sine, so a source played at
gain g sounds at reference_db + 20 log10 g dB SPL once reference_db (the reading of a sound level meter at the
participant's ears while the full-scale 1 kHz calibration tone plays) is set.

Mixer.render(n) returns n stereo samples and needs no audio device, so the whole soundscape can be rendered to a
WAV file (headless stations, tests). Output plays the mixer through pygame.mixer (a channel fed block by block
from a thread) and can record what it plays.
"""
import hashlib
import math
import os
import shutil
import subprocess
import sys
import threading
import time
import wave
from pathlib import Path

import numpy as np

RATE = 44100
BLOCK = 2048                       # samples per block (46 ms at 44.1 kHz)
FS_RMS = 1.0 / math.sqrt(2.0)      # RMS of a full-scale sine: the 0 dB of every source


def db_gain(db, reference_db=94.0):
    """Linear gain of a source that should sound at `db` dB SPL, given the calibration reference."""
    if db is None or db <= -90:
        return 0.0
    return float(min(1.0, 10.0 ** ((db - reference_db) / 20.0)))


def gain_db(gain, reference_db=94.0):
    return reference_db + 20.0 * math.log10(max(gain, 1e-9))


def rms(x):
    x = np.asarray(x, dtype=np.float64)
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


# -- synthesised buffers (mono float32, normalised to FS_RMS) ---------------------------------------------------

def _norm(x, target=FS_RMS):
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    r = rms(x)
    return (x * (target / r if r > 0 else 1.0)).astype(np.float32)


def _seamless(x, rate=RATE, fade_s=0.1):
    """A loop without a click: the last fade_s cross-faded into the start, then cut off."""
    n = int(fade_s * rate)
    if len(x) <= 2 * n:
        return x
    a = np.linspace(0.0, 1.0, n, dtype=np.float32)
    head = x[:n] * a + x[-n:] * (1.0 - a)
    return np.concatenate([head, x[n:-n]]).astype(np.float32)


def _fir_lowpass(x, cutoff, rate=RATE, taps=127):
    h = lowpass_kernel(cutoff, rate, taps)
    return np.convolve(x, h, mode="same")


def pink_noise(seconds=4.0, seed=7, rate=RATE):
    """Pink noise (-3 dB per octave, shaped in the frequency domain): tyre roar."""
    rng = np.random.default_rng(seed)
    w = rng.uniform(-1.0, 1.0, int(seconds * rate))
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(len(w), 1.0 / rate)
    f[0] = f[1]
    spec /= np.sqrt(f)
    return _norm(_seamless(np.fft.irfft(spec, len(w)), rate))


def brown_noise(seconds=4.0, seed=11, rate=RATE, low_hz=20.0):
    """Brown (red) noise with its lowest rumble removed: distant traffic."""
    rng = np.random.default_rng(seed)
    w = rng.uniform(-1.0, 1.0, int(seconds * rate))
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(len(w), 1.0 / rate)
    shape = 1.0 / np.maximum(f, low_hz)
    shape[f < low_hz * 0.5] = 0.0
    return _norm(_seamless(np.fft.irfft(spec * shape, len(w)), rate))


def band_noise(lo, hi, seconds=4.0, seed=3, rate=RATE):
    """White noise limited to lo..hi Hz (wind, rustle)."""
    rng = np.random.default_rng(seed)
    w = rng.uniform(-1.0, 1.0, int(seconds * rate))
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(len(w), 1.0 / rate)
    spec[(f < lo) | (f > hi)] = 0.0
    spec /= np.sqrt(np.maximum(f, lo))          # a little darker inside the band
    return _norm(_seamless(np.fft.irfft(spec, len(w)), rate))


def tone(freq=1000.0, seconds=1.0, rate=RATE):
    """A sine at full scale (the calibration tone: its RMS is FS_RMS)."""
    t = np.arange(int(seconds * rate)) / rate
    return np.sin(2 * np.pi * freq * t).astype(np.float32)


def city_ambience(seconds=12.0, seed=5, rate=RATE):
    """Distant city traffic: a brown-noise hum with a pink bed and a slow swell, as a loop."""
    hum = brown_noise(seconds, seed, rate)
    bed = _fir_lowpass(pink_noise(seconds, seed + 1, rate), 2500.0, rate)
    t = np.arange(len(hum)) / rate
    swell = 1.0 + 0.25 * np.sin(2 * np.pi * t / seconds) + 0.1 * np.sin(2 * np.pi * 3 * t / seconds + 1.0)
    return _norm(_seamless((0.75 * hum + 0.35 * bed / max(rms(bed), 1e-9) * rms(hum)) * swell, rate))


def rural_ambience(seconds=12.0, seed=9, rate=RATE):
    """Wind in trees: band noise with gusts."""
    x = band_noise(150.0, 4000.0, seconds, seed, rate)
    t = np.arange(len(x)) / rate
    gust = 0.6 + 0.4 * np.sin(2 * np.pi * t / seconds) ** 2
    return _norm(_seamless(x * gust, rate))


def _bell(t, f):
    return np.exp(-t * 4.0) * (np.sin(2 * np.pi * f * t) + 0.4 * np.sin(2 * np.pi * 2.76 * f * t) * np.exp(-t * 6.0))


def cue_sound(name, rate=RATE):
    """The cue sounds of coupled-sim's auditory eHMIs (#95), plus a blinker tick and a take-over warning.
    Normalised so their loudest 100 ms has the RMS of a full-scale sine."""
    if name == "chime":            # two descending bell tones (E5, C5)
        t = np.arange(int(1.6 * rate)) / rate
        x = _bell(t, 659.3) + np.where(t > 0.35, _bell(np.maximum(t - 0.35, 0), 523.3), 0.0)
    elif name == "beeps":          # three short 1.5 kHz beeps
        t = np.arange(int(0.9 * rate)) / rate
        x = np.where((t % 0.3) < 0.15, np.sin(2 * np.pi * 1500 * t), 0.0)
    elif name == "ding":
        t = np.arange(int(1.2 * rate)) / rate
        x = _bell(t, 880.0)
    elif name == "tick":           # a relay click of the turn signal
        t = np.arange(int(0.03 * rate)) / rate
        x = np.sin(2 * np.pi * 2400 * t) * np.exp(-t * 300) + 0.5 * np.sin(2 * np.pi * 900 * t) * np.exp(-t * 200)
    elif name == "takeover":       # an urgent two-tone alarm, 1.2 s
        t = np.arange(int(1.2 * rate)) / rate
        f = np.where((t % 0.2) < 0.1, 2000.0, 1600.0)
        x = np.sin(2 * np.pi * np.cumsum(f) / rate) * ((t % 0.4) < 0.32)
    else:
        raise ValueError(f"unknown cue sound {name!r}")
    return _peak_norm(x, rate)


def _peak_norm(x, rate=RATE, window_s=0.1):
    """Scales a short sound so its loudest window has the RMS of a full-scale sine (its level is that window's)."""
    x = np.asarray(x, dtype=np.float64)
    n = max(1, min(len(x), int(window_s * rate)))
    c = np.concatenate([[0.0], np.cumsum(x * x)])
    loud = math.sqrt(max(1e-18, max((c[i + n] - c[i]) / n for i in range(0, len(x) - n + 1, max(1, n // 4)))))
    return (x * FS_RMS / loud).astype(np.float32)


# -- sound files and speech ------------------------------------------------------------------------------------

def load_sound(path, rate=RATE):
    """A sound file as a mono float32 array at `rate`, scaled like a cue (loudest 100 ms = full-scale RMS).
    WAV (8/16/24/32-bit PCM) with the standard library; other formats through pygame.mixer if it is there."""
    path = Path(path)
    if path.suffix.lower() == ".wav":
        with wave.open(str(path), "rb") as w:
            ch, width, fr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
            raw = w.readframes(n)
        if width == 1:
            x = (np.frombuffer(raw, np.uint8).astype(np.float64) - 128) / 128
        elif width == 2:
            x = np.frombuffer(raw, "<i2").astype(np.float64) / 32768
        elif width == 3:
            b = np.frombuffer(raw, np.uint8).reshape(-1, 3).astype(np.int32)
            v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
            x = np.where(v >= 1 << 23, v - (1 << 24), v).astype(np.float64) / (1 << 23)
        else:
            x = np.frombuffer(raw, "<i4").astype(np.float64) / 2 ** 31
        x = x.reshape(-1, ch).mean(axis=1)
    else:
        import pygame
        if not pygame.mixer.get_init():
            pygame.mixer.init(rate, -16, 2)
        fr = pygame.mixer.get_init()[0]
        a = pygame.sndarray.array(pygame.mixer.Sound(str(path))).astype(np.float64) / 32768
        x = a.mean(axis=1) if a.ndim == 2 else a
    if fr != rate and len(x) > 1:
        x = np.interp(np.arange(0, len(x) - 1, fr / rate), np.arange(len(x)), x)
    return _peak_norm(x, rate)


def _speech_cache():
    d = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "coupled-carla" / "speech"
    d.mkdir(parents=True, exist_ok=True)
    return d


def speech_engines():
    """The text-to-speech programs found on this machine, best first."""
    found = []
    for exe in ("espeak-ng", "espeak", "pico2wave", "say"):
        if shutil.which(exe):
            found.append(exe)
    if sys.platform == "win32":
        found.append("sapi")
    return found


def speak_to_wav(text, voice=None, cache=None):
    """A WAV file of `text` spoken by the machine's own speech synthesiser (espeak-ng, espeak, pico2wave, macOS
    say, Windows SAPI), cached by text and voice; None when there is none (the caller falls back to a sound)."""
    cache = Path(cache) if cache else _speech_cache()
    key = hashlib.sha1(f"{voice}|{text}".encode()).hexdigest()[:16]
    out = cache / f"{key}.wav"
    if out.exists() and out.stat().st_size > 1000:
        return out
    for exe in speech_engines():
        try:
            if exe in ("espeak-ng", "espeak"):
                cmd = [exe, "-w", str(out)] + (["-v", voice] if voice else []) + [text]
            elif exe == "pico2wave":
                cmd = [exe, "-w", str(out)] + (["-l", voice] if voice else []) + [text]
            elif exe == "say":
                cmd = ["say", "-o", str(out), "--file-format=WAVE", "--data-format=LEI16@22050"] + \
                      (["-v", voice] if voice else []) + [text]
            else:
                ps = ("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis."
                      "SpeechSynthesizer; $s.SetOutputToWaveFile('%s'); $s.Speak('%s'); $s.Dispose()"
                      % (str(out).replace("'", "''"), text.replace("'", "''")))
                cmd = ["powershell", "-NoProfile", "-Command", ps]
            subprocess.run(cmd, check=True, capture_output=True, timeout=30)
            if out.exists() and out.stat().st_size > 1000:
                return out
        except (OSError, subprocess.SubprocessError):
            continue
    return None


# -- voices -----------------------------------------------------------------------------------------------------

class Voice:
    """A sound source in the mixer. The scene sets gain (linear), pan (-1 left .. 1 right), rate (pitch factor)
    and cutoff (low-pass, Hz, None: none); the mixer ramps gain and pan over a block, so changes never click."""

    def __init__(self, gain=0.0, pan=0.0, rate=1.0, cutoff=None):
        self.gain, self.pan, self.rate, self.cutoff = gain, pan, rate, cutoff
        self.done = False
        self._g = self._l = self._r = None   # the mixer's state: last gains
        self._tail = None                    # the low-pass filter's state

    def render(self, n):
        raise NotImplementedError


class Loop(Voice):
    """A looped buffer, played at a variable rate (pitch and Doppler)."""

    def __init__(self, buf, start=0.0, **kw):
        super().__init__(**kw)
        self.buf = np.concatenate([buf, buf[:1]]).astype(np.float32)   # one sample more, for interpolation
        self.n = len(buf)
        self.pos = float(start) % self.n

    def render(self, n):
        idx = self.pos + self.rate * np.arange(n)
        self.pos = (self.pos + self.rate * n) % self.n
        idx %= self.n
        i = idx.astype(np.int64)
        frac = (idx - i).astype(np.float32)
        return self.buf[i] * (1.0 - frac) + self.buf[i + 1] * frac


class OneShot(Voice):
    """A buffer played once (a cue, a spoken message); done at its end."""

    def __init__(self, buf, **kw):
        super().__init__(**kw)
        self.buf, self.pos = buf, 0

    def render(self, n):
        out = np.zeros(n, np.float32)
        k = min(n, len(self.buf) - self.pos)
        if k > 0:
            out[:k] = self.buf[self.pos:self.pos + k]
        self.pos += n
        self.done = self.pos >= len(self.buf)
        return out


class Engine(Voice):
    """A four-cylinder engine: the firing frequency (rpm / 30) with harmonics and a little roughness, with a
    continuous phase as the frequency changes. `freq` is the firing frequency; rate scales it (Doppler)."""

    def __init__(self, freq=27.0, seed=0, **kw):
        super().__init__(**kw)
        self.freq, self.phase = freq, 0.0
        self.rough_phase = (seed * 0.37) % 1.0
        self._f_last = freq
        amps = np.array([0.35, 0.25, 0.15, 0.08])
        self.norm = FS_RMS / math.sqrt(0.5 * float(np.sum(amps ** 2)) * (1 + 0.5 * 0.15 ** 2))

    def render(self, n):
        f_new = self.freq * self.rate
        f = np.linspace(self._f_last, f_new, n, endpoint=False)
        self._f_last = f_new
        ph = self.phase + np.cumsum(f) / RATE
        self.phase = float(ph[-1] % 1.0)
        w = 2 * np.pi * ph
        x = (0.35 * np.sin(w) + 0.25 * np.sin(2 * w + 0.3) + 0.15 * np.sin(3 * w + 1.1) + 0.08 * np.sin(4 * w))
        rp = self.rough_phase + np.cumsum(f / 4.0) / RATE      # roughness at a quarter of the firing rate
        self.rough_phase = float(rp[-1] % 1.0)
        x *= 1.0 + 0.15 * np.sin(2 * np.pi * rp)
        return (x * self.norm).astype(np.float32)


class Rotor(Voice):
    """A multirotor's noise: the tone at the blade-passing frequency (blades x rotor revolutions per second) with its
    harmonics, the main audible feature of drone noise, from two slightly detuned rotor pairs (the beating of rotors
    at different speeds), with a little amplitude modulation at the rotor frequency, over broadband rotor noise
    (`broadband` of the power). `freq` is the blade-passing frequency; rate scales it (Doppler)."""

    def __init__(self, freq=200.0, blades=2, seed=0, broadband=0.4, **kw):
        super().__init__(**kw)
        self.freq, self.blades = freq, max(1, int(blades))
        self.phase = np.array([0.0, (seed * 0.31) % 1.0])
        self.am = (seed * 0.17) % 1.0
        self._f_last = freq
        self.harm = np.array([1.0, 0.55, 0.35, 0.22, 0.14, 0.09])
        self.ph0 = np.array([0.0, 1.3, 2.6, 0.7, 2.0, 0.4])
        self.noise = band_noise(150.0, 6000.0, 2.0, seed=17 + seed % 5)
        self.npos = (seed * 7919) % len(self.noise)
        self.bb = float(broadband)
        tone_rms = math.sqrt(0.5 * float(np.sum(self.harm ** 2)) * 2)     # two pairs
        self.k_tone = FS_RMS * math.sqrt(1.0 - self.bb) / tone_rms
        self.k_noise = math.sqrt(self.bb)

    def render(self, n):
        f_new = max(1.0, self.freq * self.rate)
        f = np.linspace(self._f_last, f_new, n, endpoint=False)
        self._f_last = f_new
        x = np.zeros(n)
        for k, det in enumerate((0.992, 1.008)):
            ph = self.phase[k] + np.cumsum(f * det) / RATE
            self.phase[k] = float(ph[-1] % 1.0)
            w = 2 * np.pi * ph
            for h, (a, p0) in enumerate(zip(self.harm, self.ph0), start=1):
                x += a * np.sin(h * w + p0)
        am = self.am + np.cumsum(f / self.blades) / RATE             # the rotor frequency
        self.am = float(am[-1] % 1.0)
        x *= 1.0 + 0.06 * np.sin(2 * np.pi * am)
        idx = (self.npos + np.arange(n)) % len(self.noise)
        self.npos = int((self.npos + n) % len(self.noise))
        return (x * self.k_tone + self.noise[idx] * self.k_noise).astype(np.float32)


class Avas(Voice):
    """An AVAS hum (acoustic vehicle alerting system): two tones and an octave with a slow modulation; `rate`
    shifts its pitch with the speed, as UN R138 asks."""

    def __init__(self, base=220.0, **kw):
        super().__init__(**kw)
        self.base, self.phase, self.mod = base, 0.0, 0.0
        self._r_last = 1.0
        self.norm = FS_RMS / math.sqrt(0.5 * (0.25 ** 2 * (0.8 ** 2 + 0.5 * 0.2 ** 2) + 0.12 ** 2 + 0.05 ** 2))

    def render(self, n):
        r = np.linspace(self._r_last, self.rate, n, endpoint=False)
        self._r_last = self.rate
        ph = self.phase + np.cumsum(self.base * r) / RATE
        self.phase = float(ph[-1] % 1.0)
        m = self.mod + np.arange(1, n + 1) * 2.0 / RATE
        self.mod = float(m[-1] % 1.0)
        w = 2 * np.pi * ph
        x = 0.25 * np.sin(w) * (0.8 + 0.2 * np.sin(2 * np.pi * m)) + 0.12 * np.sin(1.5 * w) + 0.05 * np.sin(3 * w)
        return (x * self.norm).astype(np.float32)


# -- mixer ------------------------------------------------------------------------------------------------------

_KERNELS = {}


def lowpass_kernel(cutoff, rate=RATE, taps=63):
    """A windowed-sinc FIR low-pass (unity gain at DC), cached by cutoff rounded to a third of an octave."""
    key = (int(round(math.log2(max(cutoff, 20.0)) * 3)), rate, taps)
    h = _KERNELS.get(key)
    if h is None:
        fc = min(0.49, 2.0 ** (key[0] / 3.0) / rate)
        m = np.arange(taps) - (taps - 1) / 2
        h = np.sinc(2 * fc * m) * np.blackman(taps)
        h = (h / h.sum()).astype(np.float32)
        _KERNELS[key] = h
    return h


class Limiter:
    """A peak limiter on the master bus so that nothing clips: a gain that falls at once (within the block) to
    keep the block's peak under `threshold` and recovers with `release_s`, then a soft clipper (tanh above the knee)
    for what the block gain misses. `active` counts the samples it reduced."""

    def __init__(self, threshold=0.89, release_s=0.3, rate=RATE):
        self.thr, self.rate, self.release = threshold, rate, release_s
        self.g = 1.0
        self.active = 0

    def process(self, out):
        n = len(out)
        peak = float(np.max(np.abs(out))) if n else 0.0
        want = min(1.0, self.thr / peak) if peak > 0 else 1.0
        if want < self.g:
            g1 = want
        else:
            g1 = self.g + (want - self.g) * (1.0 - math.exp(-n / (self.release * self.rate)))
        if g1 < 1.0 or self.g < 1.0:
            env = np.linspace(self.g, g1, n, endpoint=False, dtype=np.float32)
            if want < self.g:       # attack within ~1 ms (no step, so no click); the soft clipper takes the rest
                k = min(n, 48)
                env[:k] = np.linspace(self.g, g1, k, endpoint=False, dtype=np.float32)
                env[k:] = g1
            out *= env[:, None]
            self.active += int(np.count_nonzero(env < 0.999))
        self.g = g1
        knee = 0.9
        over = np.abs(out) > knee
        if over.any():
            x = out[over]
            out[over] = np.sign(x) * (knee + (1.0 - knee) * np.tanh((np.abs(x) - knee) / (1.0 - knee)))
        return out


class Mixer:
    """Sums its voices into stereo blocks. Thread-safe: the scene changes voices while the output thread renders.

    Plain voices have a gain, pan and low-pass; spatial voices (synth.SpatialVoice) a basis and paths: each path is
    weighted, delayed (reflections), spatialised (stereo panning, or `spatializer`: binaural, hrtf.py) or sent to
    `reverb` (synth.Reverb). A limiter keeps the sum under full scale."""

    def __init__(self, rate=RATE, master=1.0):
        self.rate, self.master = rate, master
        self.voices = {}
        self.lock = threading.Lock()
        self.clipped = 0
        self.rendered = 0
        self.reverb = None
        self.spatializer = None
        self.limiter = Limiter(rate=rate)
        self.cost_s = 0.0

    def _spatial(self, v, n, out, rev, ramp):
        if not v.audible():
            v.advance(n)
            for p in v.paths:
                p._w0 = p.w
            return
        X = v.basis(n)
        act, W0, DW = [], [], []
        for p in v.paths:
            w1 = p.w
            if w1 is None:
                continue
            w0 = p._w0 if p._w0 is not None and len(p._w0) == len(w1) else w1
            p._w0 = w1
            if not (np.any(w1) or np.any(w0)):
                if p._line is not None and p.kind == "refl":     # keep the delay line's clock
                    p.delayed(np.zeros(n, np.float32))
                continue
            act.append(p)
            W0.append(w0)
            DW.append(w1 - w0)
        if not act:
            return
        k = len(act)
        Y = np.vstack(W0 + DW).astype(np.float32) @ X           # one product for every path: (2k, n)
        for j, p in enumerate(act):
            y = Y[j] + Y[k + j] * ramp
            if p.kind == "reverb":
                rev += y
                continue
            if p.kind == "refl":
                y = p.delayed(y)
            if self.spatializer is not None:
                out += self.spatializer.render(y, p)
                continue
            s = math.sin(p.az) * math.cos(p.el)
            th = (max(-1.0, min(1.0, s)) + 1.0) * math.pi / 4.0
            l, r = math.cos(th), math.sin(th)
            l0, r0 = p._lr if p._lr is not None else (l, r)
            p._lr = (l, r)
            out[:, 0] += y * (l0 + (l - l0) * ramp)
            out[:, 1] += y * (r0 + (r - r0) * ramp)

    def add(self, key, voice):
        with self.lock:
            self.voices[key] = voice
        return voice

    def get(self, key):
        return self.voices.get(key)

    def remove(self, key):
        with self.lock:
            self.voices.pop(key, None)

    def keys(self):
        with self.lock:
            return list(self.voices)

    def render(self, n=BLOCK):
        t0 = time.perf_counter()
        out = np.zeros((n, 2), np.float32)
        rev = np.zeros(n, np.float32)
        ramp = np.linspace(0.0, 1.0, n, endpoint=False, dtype=np.float32)
        with self.lock:
            items = list(self.voices.items())
        for key, v in items:
            if hasattr(v, "paths"):
                self._spatial(v, n, out, rev, ramp)
                if v.done:
                    self.remove(key)
                continue
            g = float(v.gain)
            th = (max(-1.0, min(1.0, float(v.pan))) + 1.0) * math.pi / 4.0
            l, r = math.cos(th) * g, math.sin(th) * g
            if v._l is None:
                v._l, v._r = l, r
            if max(l, r, v._l, v._r) < 1e-6:       # silent: keep its time running, skip the work
                v.render(n)
                v._l, v._r = l, r
            else:
                x = v.render(n)
                if v.cutoff:
                    h = lowpass_kernel(v.cutoff, self.rate)
                    tail = v._tail if v._tail is not None and len(v._tail) == len(h) - 1 else \
                        np.zeros(len(h) - 1, np.float32)
                    xx = np.concatenate([tail, x])
                    v._tail = xx[-(len(h) - 1):]
                    x = np.convolve(xx, h, mode="valid").astype(np.float32)
                else:
                    v._tail = None
                out[:, 0] += x * (v._l + (l - v._l) * ramp)
                out[:, 1] += x * (v._r + (r - v._r) * ramp)
                v._l, v._r = l, r
            if v.done:
                self.remove(key)
        if self.reverb is not None:
            out += self.reverb.process(rev)
        out *= self.master
        if self.limiter is not None:
            self.limiter.process(out)
        over = np.abs(out) > 1.0
        if over.any():
            self.clipped += int(over.sum())
            np.clip(out, -1.0, 1.0, out=out)
        self.rendered += n
        self.cost_s += time.perf_counter() - t0
        return out


def write_wav(path, stereo, rate=RATE):
    """Writes float stereo (n, 2) in [-1, 1] as 16-bit PCM."""
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((np.clip(stereo, -1, 1) * 32767).astype("<i2").tobytes())


class Output:
    """Plays a Mixer. With a device (pygame.mixer; True: the default output, a string: that output's name) a thread
    keeps one block queued on a reserved channel, so the sound runs on while the station waits for CARLA; without
    one (headless) a thread renders in real time to the recording only. `record` writes every block played to a
    WAV file."""

    def __init__(self, mixer, device=True, record=None, block=BLOCK, log=print):
        self.mixer, self.block, self.log = mixer, block, log
        self.channel = None
        self.rec = None
        self.running = False
        self.thread = None
        self.blocks = 0
        if device:
            try:
                import pygame
                if not pygame.mixer.get_init():
                    if isinstance(device, str):      # a named output (two stations on one PC, two headsets)
                        pygame.mixer.pre_init(mixer.rate, -16, 2, 1024, devicename=device)
                    else:
                        pygame.mixer.pre_init(mixer.rate, -16, 2, 1024)
                    pygame.mixer.init()
                freq, _size, chans = pygame.mixer.get_init()
                if chans != 2:
                    raise RuntimeError(f"mixer has {chans} channels")
                mixer.rate = freq
                pygame.mixer.set_reserved(1)
                self.channel = pygame.mixer.Channel(0)
                self._pg = pygame
            except Exception as e:      # no audio device (a server without sound card): record only
                log(f"sound: no audio output ({e}); {'recording only' if record else 'silent'}")
                self.channel = None
        if record:
            self.rec = wave.open(str(record), "wb")
            self.rec.setnchannels(2)
            self.rec.setsampwidth(2)
            self.rec.setframerate(mixer.rate)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="sound", daemon=True)
        self.thread.start()

    def _pcm(self):
        x = self.mixer.render(self.block)
        pcm = (x * 32767).astype("<i2")
        if self.rec is not None:
            self.rec.writeframes(pcm.tobytes())
        self.blocks += 1
        return pcm

    def _loop(self):
        dur = self.block / self.mixer.rate
        t_next = time.time()
        while self.running:
            try:
                if self.channel is not None:
                    if self.channel.get_queue() is None:
                        snd = self._pg.sndarray.make_sound(np.ascontiguousarray(self._pcm()))
                        if self.channel.get_busy():
                            self.channel.queue(snd)
                        else:
                            self.channel.play(snd)
                    time.sleep(dur / 4)
                else:
                    self._pcm()
                    t_next += dur
                    time.sleep(max(0.0, t_next - time.time()))
            except Exception as e:        # never take the station down for its sound
                self.log(f"sound: output stopped ({e})")
                self.running = False

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.channel is not None:
            try:
                self.channel.stop()
            except Exception:
                pass
        if self.rec is not None:
            self.rec.close()
            self.rec = None
