"""Sound sources shaped in octave bands, so that every source carries a published spectrum (acoustics.py) and every
propagation path (direct, reflections off facades, the reverberant tail) its own per-band attenuation.

A spatial voice renders a small *basis* each block, K rows of signal: octave bands of noise (63 Hz - 8 kHz, brick-
wall bands cut from one looped noise, so they add up exactly and are uncorrelated), the partials of an engine or a
rotor, or the low and high halves of a recorded or synthesised sound. Each path of the voice is a weight vector over
the rows (from the band levels at the listener), a delay, a direction and a kind (direct, reflection, reverb send);
the mixer (audio.Mixer) forms one signal per path, delays, spatialises and sums them. A row with RMS FS_RMS played at
weight w is reference_db + 20 lg w dB SPL, as everywhere in audio.py.

Engine synthesis follows the spectral-modelling approach of Pieren, Buetler & Heutschi (2016, Auralization of
accelerating passenger cars using spectral modeling synthesis, Applied Sciences 6(1):5: engine orders from the
instantaneous rpm and load plus a broadband residual, tyre noise as broadband noise) and the physically informed
synthesis of Baldan, Lachambre, Delle Monache & Boussard (2015, Physically informed car engine sound synthesis for
virtual and augmented environments, IEEE SIVE: firing pulses and the resonances of intake and exhaust), with
cycle-to-cycle variation of speed and amplitude (the main cue that tells a real engine from a tone generator, Jagla,
Maillard & Martin 2012, JASA 132(5)). The levels per band are CNOSSOS-EU's; the synthesis only decides how each
band's energy is spread over partials and noise.
"""
import math

import numpy as np

from . import acoustics as ac
from . import audio

RATE = audio.RATE
FS = audio.FS_RMS
NB = ac.NB
_LOOPS = {}


def band_loops(kind="pink", seconds=4.0, seed=7, rate=RATE):
    """(8, N) float32: a looped noise split into the octave bands 63..8000 Hz (brick-wall in the frequency domain of
    the whole loop, so each band loops seamlessly), each band normalised to RMS FS_RMS. Cached."""
    key = (kind, seconds, seed, rate)
    out = _LOOPS.get(key)
    if out is not None:
        return out
    n = int(seconds * rate)
    rng = np.random.default_rng(seed)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / rate)
    if kind == "pink":
        spec /= np.sqrt(np.maximum(f, 20.0))
    elif kind == "brown":
        spec /= np.maximum(f, 20.0)
    out = np.zeros((NB, n), np.float32)
    edges = ac.EDGES.copy()
    edges[0] = 30.0                     # nothing below 30 Hz (rumble no headphone or speaker plays)
    edges[-1] = min(16000.0, rate / 2)
    for b in range(NB):
        x = np.fft.irfft(spec * ((f >= edges[b]) & (f < edges[b + 1])), n)
        r = float(np.sqrt(np.mean(x * x)))
        out[b] = (x * (FS / r if r > 0 else 0.0)).astype(np.float32)
    _LOOPS[key] = out
    return out


def split_low_high(x, rate=RATE):
    """A mono buffer as two rows (below / above ~1.4 kHz, the edge between the 1 and 2 kHz octaves), and its power
    share per octave band (sums to 1)."""
    x = np.asarray(x, np.float64)
    n = len(x)
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / rate)
    edge = ac.EDGES[5]
    low = np.fft.irfft(spec * (f < edge), n).astype(np.float32)
    high = np.fft.irfft(spec * (f >= edge), n).astype(np.float32)
    p = np.abs(spec) ** 2
    share = np.array([p[(f >= ac.EDGES[b]) & (f < ac.EDGES[b + 1])].sum() for b in range(NB)])
    share = share / max(share.sum(), 1e-30)
    return low, high, share


def psd_interp(freqs, band_power):
    """Power per Hz at `freqs` from octave-band powers, interpolated log-log between the band centres (flat
    beyond the outer bands): a smooth spectrum so a partial does not jump in level when it crosses a band edge."""
    bw = ac.EXACT * (2 ** 0.5 - 2 ** -0.5)
    dens = np.maximum(np.asarray(band_power, float), 1e-30) / bw
    return 10.0 ** np.interp(np.log10(np.maximum(freqs, 1.0)), np.log10(ac.EXACT), np.log10(dens))


def band_db_to_power(band_db, ref):
    """dB SPL per band -> power in full-scale units (a row at weight w has power w^2)."""
    a = np.asarray(band_db, float)
    with np.errstate(over="ignore"):
        return np.where(np.isfinite(a), 10.0 ** ((a - ref) / 10.0), 0.0)


class Path:
    """One way a voice reaches the ears: weights over the basis rows, a delay (s, reflections), a direction
    (azimuth rad, positive to the right; elevation rad) and a kind: direct, refl or reverb (sent to the reverb)."""
    __slots__ = ("w", "delay", "az", "el", "kind", "_w0", "_lr", "_line", "_wpos", "_d0", "_conv")

    def __init__(self, kind="direct"):
        self.kind = kind
        self.w = None
        self.delay, self.az, self.el = 0.0, 0.0, 0.0
        self._w0 = self._lr = self._line = self._d0 = self._conv = None
        self._wpos = 0

    def delayed(self, y):
        """y delayed by self.delay (ramped from the last block's delay, fractional, linear interpolation)."""
        n = len(y)
        size = 1 << 16
        if self._line is None:
            self._line = np.zeros(size, np.float32)
        line = self._line
        idx = (self._wpos + np.arange(n)) & (size - 1)
        line[idx] = y
        d1 = min(self.delay, 1.3) * RATE
        d0 = d1 if self._d0 is None else self._d0
        self._d0 = d1
        pos = self._wpos + np.arange(n) - (d0 + (d1 - d0) * np.arange(n) / n)
        self._wpos = (self._wpos + n) & (size - 1)
        i = np.floor(pos).astype(np.int64)
        fr = (pos - i).astype(np.float32)
        return line[i & (size - 1)] * (1.0 - fr) + line[(i + 1) & (size - 1)] * fr

    def set(self, w, delay=0.0, az=0.0, el=0.0):
        self.w, self.delay, self.az, self.el = w, delay, az, el

    def silence(self):
        if self.w is not None:
            self.w = np.zeros_like(self.w)


class SpatialVoice(audio.Voice):
    """A voice with a K-row basis and paths (see the module doc). `rate` scales its pitch (Doppler)."""
    K = 1

    def __init__(self, kinds=("direct", "refl", "refl", "reverb"), **kw):
        super().__init__(**kw)
        self.paths = [Path(k) for k in kinds]

    def basis(self, n):
        raise NotImplementedError

    def advance(self, n):
        """Moves the voice's clock on without rendering (all paths silent)."""
        self.basis(n)

    def weights(self, band_db, ref):
        """The row weights that make this voice sound at `band_db` (dB SPL per octave band at the ears)."""
        raise NotImplementedError

    def audible(self):
        return any(p.w is not None and (np.any(p.w) or (p._w0 is not None and np.any(p._w0))) for p in self.paths)


class BandNoise(SpatialVoice):
    """Noise shaped by band weights (tyre-road noise, wind, rain hiss, distant traffic). A slow random level
    fluctuation (`flutter_db`, a few tenths of a dB at ~1 Hz) keeps it from sounding like a static hiss."""
    K = NB

    def __init__(self, kind="pink", seed=7, flutter_db=0.0, flutter_hz=1.0, **kw):
        super().__init__(**kw)
        self.loops = band_loops(kind, 4.0, seed % 3)       # three loops shared by all voices, offset by seed
        self.n = self.loops.shape[1]
        self.pos = float((seed * 7919) % self.n)
        self.rng = np.random.default_rng(seed + 1000)
        self.flutter = 10 ** (flutter_db / 20.0) - 1.0 if flutter_db else 0.0
        self.fl_hz, self._fl = flutter_hz, 0.0

    def advance(self, n):
        self.pos = (self.pos + self.rate * n) % self.n

    def basis(self, n):
        if abs(self.rate - 1.0) < 1e-4:
            i0 = int(self.pos)
            idx = (i0 + np.arange(n)) % self.n if i0 + n > self.n else None
            x = self.loops[:, idx] if idx is not None else self.loops[:, i0:i0 + n].copy()
            self.pos = (self.pos + n) % self.n
        else:      # Doppler: nearest sample (interpolating noise buys nothing audible)
            idx = ((self.pos + self.rate * np.arange(n)) % self.n).astype(np.int64)
            self.pos = (self.pos + self.rate * n) % self.n
            x = self.loops[:, idx]
        if self.flutter:
            a = math.exp(-n / RATE * self.fl_hz * 2 * math.pi)
            new = a * self._fl + (1 - a) * float(self.rng.standard_normal()) * 1.5
            env = 1.0 + self.flutter * np.linspace(self._fl, new, n, dtype=np.float32)
            self._fl = new
            x *= env
        return x

    def weights(self, band_db, ref):
        return np.sqrt(band_db_to_power(band_db, ref)).astype(np.float32)


class _Partials(SpatialVoice):
    """Partials (multiples `mults` of a base frequency, continuous phase, Doppler through `rate`) with a slow
    amplitude jitter per partial, plus NB noise bands. Subclasses set the base frequency and the spectrum."""

    def __init__(self, mults, base=30.0, seed=0, jitter_f=0.004, jitter_a=0.15, noise_kind="white", **kw):
        super().__init__(**kw)
        self.mults = np.asarray(mults, float)
        self.nh = len(self.mults)
        self.K = self.nh + NB
        self.base = base
        self._b_last = base
        self.phase = 0.0
        self.rng = np.random.default_rng(seed)
        self.ph0 = self.rng.uniform(0, 2 * np.pi, self.nh)
        self._rot = np.exp(1j * self.ph0).astype(np.complex64)
        self.nlive = self.nh          # partials rendered (fewer for far sources)
        self._plan = self._groups(self.mults)
        self.jf, self.ja = jitter_f, jitter_a
        self._jf = 0.0
        self._ja = np.zeros(self.nh)
        self.noise = BandNoise(noise_kind, seed=seed + 31)
        self.noise.rate = 1.0
        self.mod_depth = 0.0          # pulse modulation of the noise at the base frequency (engine firing)
        self.mod_kappa = 3.0
        self._mp = 0.0

    @staticmethod
    def _groups(mults):
        """Rows as integer multiples k of a few units (0.5 for engine orders, the detuned rotor pairs), sorted by k."""
        plan = []
        left = list(range(len(mults)))
        while left:
            unit = min(mults[i] for i in left)
            for u in (unit, unit / 2, unit / 3, unit / 4):
                ks = [mults[i] / u for i in left]
                grp = [(int(round(k)), i) for k, i in zip(ks, left) if abs(k - round(k)) < 1e-6 and round(k) <= 64]
                if grp:
                    break
            grp.sort()
            plan.append((u, [k for k, _ in grp], [i for _, i in grp]))
            done = {i for _, i in grp}
            left = [i for i in left if i not in done]
        return plan

    def _freqs(self):
        return self.mults * self.base * self.rate

    def advance(self, n):
        self.basis(n)

    def basis(self, n):
        b_new = max(1.0, self.base * self.rate)
        # speed jitter: a smoothed random walk (cycle-to-cycle variation)
        a = math.exp(-n / RATE * 2 * math.pi * 4.0)
        jf_new = a * self._jf + (1 - a) * float(self.rng.standard_normal()) * 2.0
        f = np.linspace(self._b_last * (1 + self.jf * self._jf), b_new * (1 + self.jf * jf_new), n, endpoint=False)
        self._jf, self._b_last = jf_new, b_new
        ph = self.phase + np.cumsum(f) / RATE           # base phase in cycles
        self.phase = float(ph[-1] % 1.0)
        w = 2 * np.pi * ph
        rows = np.empty((self.K, n), np.float32)
        live = self.mults * b_new < 0.45 * RATE
        ja_new = np.clip(self._ja * 0.6 + self.rng.standard_normal(self.nh) * 0.8, -2.5, 2.5)
        ramp = np.linspace(0.0, 1.0, n, endpoint=False, dtype=np.float32)
        a0 = (self.ja * self._ja).astype(np.float32)[:, None]
        da = (self.ja * (ja_new - self._ja)).astype(np.float32)[:, None]
        self._ja = ja_new
        # partials by complex recurrence: exp(i k u w) = exp(i u w)^k for each group of multiples k of a unit u
        nlive = self.nlive
        for unit, ks, idx in self._plan:
            wu = (unit * w).astype(np.float32)
            e = np.empty(n, np.complex64)
            e.real, e.imag = np.cos(wu), np.sin(wu)
            z = e.copy()
            k_have = 1
            for k, r in zip(ks, idx):
                if r >= nlive or not live[r]:
                    rows[r] = 0.0
                    continue
                while k_have < k:
                    z *= e
                    k_have += 1
                rows[r] = (z * self._rot[r]).imag
        h = rows[:self.nh]
        h *= 1.0 + a0 + da * ramp[None, :]
        nz = self.noise.basis(n)
        if self.mod_depth > 0:
            # firing pulses: a von Mises pulse train at the base frequency, mean 1, scaled to keep the power
            p = np.exp(self.mod_kappa * (np.cos(w) - 1.0)) * self._pnorm
            m = (1.0 + self.mod_depth * (p - 1.0)) * self._mscale
            nz = nz * m.astype(np.float32)
        rows[self.nh:] = nz
        return rows

    @property
    def _pnorm(self):
        k = self.mod_kappa
        if getattr(self, "_pn_k", None) != (k, self.mod_depth):
            self._pn_k = (k, self.mod_depth)
            self._pn = 1.0 / (math.exp(-k) * float(np.i0(k)))
            var_p = math.exp(-2 * k) * float(np.i0(2 * k)) * self._pn ** 2 - 1.0
            self._ms = 1.0 / math.sqrt(1.0 + self.mod_depth ** 2 * var_p)
        return self._pn

    @property
    def _mscale(self):
        self._pnorm
        return self._ms

    def partial_weights(self, band_power, share, rel):
        """Weights of the partials from the band powers (full-scale units): tonal power share(f) * PSD(f) * spacing
        * rel, normalised so the partials carry the tonal share of the A-weighted power."""
        f = self._freqs()
        ok = f < 0.45 * RATE
        spacing = max(1.0, float(np.median(np.diff(np.sort(f))))) if self.nh > 1 else max(1.0, f[0])
        sh = np.interp(np.log10(np.maximum(f, 1.0)), np.log10(ac.EXACT), share)
        p = psd_interp(f, band_power) * spacing * sh * rel * ok
        aw = 10 ** (np.interp(np.log10(np.maximum(f, 1.0)), np.log10(ac.EXACT), ac.A_WEIGHT) / 10)
        want = float(np.sum(np.asarray(band_power) * share * 10 ** (ac.A_WEIGHT / 10)))
        have = float(np.sum(p * aw))
        if have > 0:
            p *= want / have
        return np.sqrt(p)


class EngineVoice(_Partials):
    """A combustion engine. Partials are the firing frequency (cylinders/2 x rpm/60) and its harmonics plus the
    half orders between them (cylinder-to-cylinder variation, stronger on diesels); the residual noise is pulsed at
    the firing frequency (combustion and valve impacts; deeper on diesels: their clatter). Spectral shape inside the
    CNOSSOS band levels: harmonics fall with order, less so under load (load 0..1), with an exhaust and an intake
    resonance (formants) and the silencer's roll-off."""

    def __init__(self, cylinders=4, diesel=False, seed=0, harmonics=12, exhaust_hz=None, intake_hz=None, **kw):
        mults = np.arange(1, 2 * harmonics + 1) * 0.5
        super().__init__(mults, seed=seed, jitter_f=0.008 if diesel else 0.004,
                         jitter_a=0.30 if diesel else 0.15, **kw)
        self.cyl, self.diesel = int(cylinders), bool(diesel)
        self.rpm, self.load = 800.0, 0.2
        self.exhaust = exhaust_hz or (70.0 if diesel else 110.0)
        self.intake = intake_hz or (350.0 if diesel else 450.0)
        self.half = 0.45 if diesel else 0.22
        self.mod_depth = 0.85 if diesel else 0.45
        self.share = np.array([0.8, 0.7, 0.55, 0.4, 0.25, 0.15, 0.08, 0.04]) if diesel else \
            np.array([0.85, 0.8, 0.7, 0.55, 0.35, 0.2, 0.1, 0.05])

    def set_rpm(self, rpm, load):
        self.rpm, self.load = float(rpm), max(0.0, min(1.0, float(load)))
        self.base = self.cyl / 2.0 * self.rpm / 60.0

    def rel(self):
        f = self.mults * self.base
        order = self.mults
        tilt = 1.1 - 0.5 * self.load
        r = order ** (-2.0 * tilt)
        r = np.where(np.abs(order - np.round(order)) > 0.25, r * self.half ** 2, r)

        def formant(f0, gain_db, q):
            g = 10 ** (gain_db / 20.0)
            return (1.0 + (g - 1.0) / (1.0 + q * q * (f / f0 - f0 / np.maximum(f, 1.0)) ** 2)) ** 2

        r = r * formant(self.exhaust, 8.0, 1.5) * formant(self.intake, 4.0 + 3.0 * self.load, 2.0)
        r = r / (1.0 + (f / 1500.0) ** 4)          # the silencer
        return r

    def weights(self, band_db, ref):
        P = band_db_to_power(band_db, ref)
        w = np.zeros(self.K, np.float32)
        w[:self.nh] = self.partial_weights(P, self.share, self.rel())
        w[self.nh:] = np.sqrt(P * (1.0 - self.share))
        return w


class RotorVoice(_Partials):
    """A multirotor: tones at the blade-passing frequency and its harmonics, from two rotor pairs turning slightly
    apart (beating), over broadband rotor noise, the shape published for small drones (e.g. Cabell et al. 2016,
    NASA; Schaeffer et al. 2021, IJERPH 18:5940: BPF harmonics dominate below ~2 kHz, broadband above)."""
    SHAPE = np.array([-14.0, -8.0, -4.0, -2.0, 0.0, 0.0, -2.0, -7.0])     # band levels of a hovering drone, dB

    def __init__(self, blades=2, seed=0, harmonics=8, **kw):
        mults = np.concatenate([np.arange(1, harmonics + 1) * 0.992, np.arange(1, harmonics + 1) * 1.008])
        super().__init__(mults, seed=seed, jitter_f=0.003, jitter_a=0.1, **kw)
        self.blades = blades
        self.share = np.array([0.9, 0.85, 0.8, 0.7, 0.55, 0.35, 0.2, 0.1])
        self.mod_depth = 0.15
        self.mod_kappa = 1.0
        h = np.concatenate([np.arange(1, harmonics + 1)] * 2)
        self._rel = h ** -1.5

    def set_bpf(self, bpf):
        self.base = max(5.0, float(bpf))

    def weights(self, band_db, ref):
        P = band_db_to_power(band_db, ref)
        w = np.zeros(self.K, np.float32)
        w[:self.nh] = self.partial_weights(P, self.share, self._rel)
        w[self.nh:] = np.sqrt(P * (1.0 - self.share))
        return w


class AvasVoice(_Partials):
    """An acoustic vehicle alerting system: a tonal hum whose pitch rises with speed (UN R138 asks for a frequency
    shift with speed), partials at 1, 1.5, 2, 3 and 4 times the base, with a 2 Hz modulation, little noise."""
    SHAPE = np.array([-30.0, -10.0, -2.0, 0.0, -3.0, -8.0, -16.0, -24.0])

    def __init__(self, base=220.0, seed=0, **kw):
        super().__init__([1.0, 1.5, 2.0, 3.0, 4.0], base=base, seed=seed, jitter_f=0.0, jitter_a=0.05, **kw)
        self.base0 = base
        self.share = np.full(NB, 0.97)
        self._rel = np.array([1.0, 0.5, 0.3, 0.12, 0.05])

    def weights(self, band_db, ref):
        P = band_db_to_power(band_db, ref)
        w = np.zeros(self.K, np.float32)
        w[:self.nh] = self.partial_weights(P, self.share, self._rel)
        w[self.nh:] = np.sqrt(P * (1.0 - self.share))
        return w


class ShapedMono(SpatialVoice):
    """A mono buffer (a recording, a cue, a synthesised one-shot) as two rows, low and high, so that occlusion and
    distance darken it. `share` is its power per octave band; weights() gets the band levels at the ears."""
    K = 2

    def __init__(self, buf, loop=False, start=0, **kw):
        super().__init__(**kw)
        low, high, self.share = split_low_high(buf)
        self.rows = np.stack([low, high]).astype(np.float32)
        self.n = len(buf)
        self.loop = loop
        self.pos = float(start % max(1, self.n)) if loop else 0.0
        self.p_low = max(float(self.share[:5].sum()), 1e-9)
        self.p_high = max(float(self.share[5:].sum()), 1e-9)

    def advance(self, n):
        self.basis(n)

    def basis(self, n):
        if self.loop:
            idx = (self.pos + self.rate * np.arange(n)) % self.n
            self.pos = (self.pos + self.rate * n) % self.n
            i = idx.astype(np.int64)
            return self.rows[:, i]
        out = np.zeros((2, n), np.float32)
        i0 = int(self.pos)
        k = min(n, self.n - i0)
        if k > 0:
            out[:, :k] = self.rows[:, i0:i0 + k]
        self.pos += n
        self.done = self.pos >= self.n
        return out

    def band_db(self, level_db, atten_db=0.0):
        """Band levels at the ears for a total level `level_db` (dB at the ears before `atten_db` per band)."""
        with np.errstate(divide="ignore"):
            return level_db + 10.0 * np.log10(np.maximum(self.share, 1e-12)) - np.asarray(atten_db, float)

    def weights(self, band_db, ref):
        P = band_db_to_power(band_db, ref)
        return np.array([math.sqrt(P[:5].sum() / self.p_low), math.sqrt(P[5:].sum() / self.p_high)], np.float32)


class EventVoice(SpatialVoice):
    """Repeated short sounds (footsteps, the ticks of a pedestrian signal, a freewheel's ratchet, raindrops): every
    `period` s (with `jitter`, a share of the period, or Poisson arrivals when poisson is set) one of `buffers`
    starts, its level varied by `gain_db_sd`. Two rows, low and high, as ShapedMono. period None: silent."""
    K = 2

    def __init__(self, buffers, seed=0, jitter=0.03, gain_db_sd=1.5, poisson=False, **kw):
        super().__init__(**kw)
        self.bufs = []
        shares = []
        for b in buffers:
            low, high, sh = split_low_high(b)
            self.bufs.append(np.stack([low, high]).astype(np.float32))
            shares.append(sh)
        self.share = np.mean(shares, axis=0)
        self.p_low = max(float(self.share[:5].sum()), 1e-9)
        self.p_high = max(float(self.share[5:].sum()), 1e-9)
        self.rng = np.random.default_rng(seed)
        self.jitter, self.gsd, self.poisson = jitter, gain_db_sd, poisson
        self.period = None
        self.next = 0.0           # samples until the next event
        self.active = []          # [buffer index, position, gain]
        self.events = 0

    def advance(self, n):
        self.basis(n)

    def basis(self, n):
        out = np.zeros((2, n), np.float32)
        if self.period:
            per = self.period * RATE
            while self.next < n:
                start = int(max(0.0, self.next))
                g = 10 ** (self.rng.normal(0.0, self.gsd) / 20.0)
                self.active.append([int(self.rng.integers(len(self.bufs))), -start, g])
                self.events += 1
                if self.poisson:
                    self.next += self.rng.exponential(per)
                else:
                    self.next += per * (1.0 + self.rng.uniform(-self.jitter, self.jitter))
            self.next -= n
        else:
            self.next = max(self.next - n, 0.0)
        keep = []
        for ev in self.active:
            b = self.bufs[ev[0]]
            pos = ev[1]
            s0 = max(0, -pos)
            i0 = max(0, pos)
            k = min(n - s0, b.shape[1] - i0)
            if k > 0:
                out[:, s0:s0 + k] += b[:, i0:i0 + k] * ev[2]
            ev[1] = pos + n
            if ev[1] < b.shape[1]:
                keep.append(ev)
        self.active = keep
        return out

    def weights(self, band_db, ref):
        """band_db: band levels of one event (its loudest 100 ms) at the ears."""
        P = band_db_to_power(band_db, ref)
        return np.array([math.sqrt(P[:5].sum() / self.p_low), math.sqrt(P[5:].sum() / self.p_high)], np.float32)

    band_db = ShapedMono.band_db


# -- reverberation ---------------------------------------------------------------------------------------------------

def reverb_ir(rt60, seconds=None, rate=RATE, seed=3):
    """A stereo impulse response of a diffuse tail: two decorrelated noises, each octave band decaying with its own
    reverberation time (rt60 a number or 8 per-band values), unit energy. A 5 ms pre-delay."""
    rt = np.broadcast_to(np.asarray(rt60, float), (NB,)).copy()
    dur = float(seconds or min(3.0, 1.2 * rt.max() + 0.05))
    n = int(dur * rate)
    t = np.arange(n) / rate
    rng = np.random.default_rng(seed)
    out = np.zeros((2, n))
    loops_f = np.fft.rfftfreq(n, 1.0 / rate)
    for ch in range(2):
        spec = np.fft.rfft(rng.standard_normal(n))
        x = np.zeros(n)
        for b in range(NB):
            band = np.fft.irfft(spec * ((loops_f >= ac.EDGES[b]) & (loops_f < ac.EDGES[b + 1])), n)
            x += band * np.exp(-6.91 * t / max(rt[b], 0.05))
        x[: int(0.005 * rate)] = 0.0
        out[ch] = x
    out /= math.sqrt(np.sum(out ** 2) / 2.0)
    return out.astype(np.float32)


class Reverb:
    """Uniformly partitioned FFT convolution of a mono send with a stereo IR (blocks of `part` samples; the send is
    buffered, so any block length works). set_rt() swaps the IR with a 0.5 s cross-fade."""

    def __init__(self, rt60=1.0, part=1024, rate=RATE):
        self.part, self.rate = part, rate
        self.fifo_in = np.zeros(0, np.float32)
        self.fifo_out = np.zeros((0, 2), np.float32)
        self.engines = []
        self.rt = None
        self.set_rt(rt60)
        self.fade = None

    def _engine(self, rt60):
        ir = reverb_ir(rt60, rate=self.rate)
        P = self.part
        nparts = int(math.ceil(ir.shape[1] / P))
        irp = np.zeros((2, nparts * P), np.float32)
        irp[:, :ir.shape[1]] = ir
        H = np.fft.rfft(irp.reshape(2, nparts, P), n=2 * P, axis=2)          # (2, nparts, P+1)
        return {"H": H, "X": np.zeros((nparts, P + 1), np.complex64), "prev": np.zeros(P, np.float32), "gain": 1.0}

    def set_rt(self, rt60):
        rt = np.broadcast_to(np.asarray(rt60, float), (NB,))
        if self.rt is not None and np.all(np.abs(np.log(rt / self.rt)) < 0.2):
            return
        self.rt = rt.copy()
        e = self._engine(rt)
        if self.engines:
            old = self.engines[-1]
            self.engines = [old, e]
            e["gain"] = 0.0
            self.fade = 0
        else:
            self.engines = [e]

    def _process_part(self, x):
        P = self.part
        y = np.zeros((P, 2), np.float32)
        nfade = int(0.5 * self.rate / P)
        for k, e in enumerate(self.engines):
            seg = np.concatenate([e["prev"], x])
            e["prev"] = x.copy()
            e["X"] = np.roll(e["X"], 1, axis=0)
            e["X"][0] = np.fft.rfft(seg)
            Y = np.einsum("pf,cpf->cf", e["X"], e["H"])
            out = np.fft.irfft(Y, n=2 * P, axis=1)[:, P:]
            if len(self.engines) == 2:
                g = (self.fade + 1) / nfade
                e["gain"] = g if k == 1 else 1.0 - g
            y += (out * e["gain"]).T
        if len(self.engines) == 2:
            self.fade += 1
            if self.fade >= nfade:
                self.engines = [self.engines[1]]
                self.engines[0]["gain"] = 1.0
        return y

    def process(self, x):
        """x: mono (n,) send -> (n, 2), delayed by up to one partition."""
        n = len(x)
        self.fifo_in = np.concatenate([self.fifo_in, x.astype(np.float32)])
        outs = [self.fifo_out]
        while len(self.fifo_in) >= self.part:
            outs.append(self._process_part(self.fifo_in[:self.part]))
            self.fifo_in = self.fifo_in[self.part:]
        buf = np.concatenate(outs) if len(outs) > 1 else self.fifo_out
        if len(buf) < n:
            buf = np.concatenate([np.zeros((n - len(buf), 2), np.float32), buf])
        self.fifo_out = buf[n:]
        return buf[:n]


# -- synthesised one-shots -------------------------------------------------------------------------------------------

def _decay_noise(rng, n, tau, lo, hi, rate=RATE):
    t = np.arange(n) / rate
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / rate)
    spec[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(spec, n) * np.exp(-t / tau)


def footstep_buffers(surface="hard", n=6, seed=21, wet=False, rate=RATE):
    """Footsteps after Cook (2002, Modeling Bill's gait: analysis and parametric synthesis of walking sounds, AES 22nd
    Int. Conf.): a heel strike and, ~90 ms later, a softer toe, each a decaying noise burst filtered by the surface
    (hard paving: bright and short; soft ground: dull and longer), with a wet splash on wet ground. Each buffer is
    normalised like a cue (loudest 100 ms at full-scale RMS)."""
    rng = np.random.default_rng(seed)
    out = []
    lo, hi, tau = (250.0, 6000.0, 0.012) if surface == "hard" else (120.0, 2500.0, 0.025)
    for _ in range(n):
        m = int(0.28 * rate)
        x = np.zeros(m)
        heel = _decay_noise(rng, int(0.08 * rate), tau * rng.uniform(0.8, 1.25), lo, hi * rng.uniform(0.8, 1.1))
        x[:len(heel)] += heel
        d = int(rng.uniform(0.07, 0.11) * rate)
        toe = 0.45 * _decay_noise(rng, int(0.06 * rate), tau * 0.8, lo * 1.5, hi, rate)
        x[d:d + len(toe)] += toe[: m - d]
        if wet:
            sp = 0.3 * _decay_noise(rng, int(0.12 * rate), 0.04, 2000.0, 9000.0, rate)
            x[:len(sp)] += sp
        out.append(audio._peak_norm(x, rate))
    return out


def tick_buffers(kind="aps", n=3, seed=5, rate=RATE):
    """Short mechanical ticks: `aps` the 'tok' of a pedestrian signal's push-button box (a click exciting a
    ~1.6 kHz plastic resonance, as the Dutch/German ticking signals), `ratchet` a freewheel pawl, `drop` a raindrop
    on a hard surface (a click and a small bubble resonance)."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        if kind == "aps":
            m = int(0.04 * rate)
            t = np.arange(m) / rate
            f0 = 1600.0 * rng.uniform(0.97, 1.03)
            x = np.sin(2 * np.pi * f0 * t) * np.exp(-t / 0.006) + 0.4 * np.sin(2 * np.pi * 2.7 * f0 * t) * np.exp(-t / 0.003)
            x += 0.3 * _decay_noise(rng, m, 0.002, 500.0, 8000.0)
        elif kind == "ratchet":
            m = int(0.012 * rate)
            t = np.arange(m) / rate
            x = _decay_noise(rng, m, 0.0015, 2000.0, 12000.0) + 0.5 * np.sin(2 * np.pi * 4200 * t) * np.exp(-t / 0.002)
        elif kind == "drop":
            m = int(0.03 * rate)
            t = np.arange(m) / rate
            f0 = rng.uniform(1500.0, 5000.0)
            x = _decay_noise(rng, m, 0.0008, 1000.0, 12000.0) + 0.6 * np.sin(2 * np.pi * f0 * t * (1 + 8 * t)) * np.exp(-t / 0.006)
        elif kind == "roof":            # a raindrop on a car roof: a dull thud of the panel
            m = int(0.05 * rate)
            t = np.arange(m) / rate
            x = np.sin(2 * np.pi * rng.uniform(250, 450) * t) * np.exp(-t / 0.01) + 0.4 * _decay_noise(rng, m, 0.002, 300, 4000)
        else:
            raise ValueError(kind)
        out.append(audio._peak_norm(x, rate))
    return out


def bell(f0=2600.0, seconds=1.2, rings=2, rate=RATE):
    """A bicycle bell: inharmonic partials of a struck dome (ratios after a small bell: 1, 1.52, 2.0, 2.65, 3.4),
    struck `rings` times."""
    n = int(seconds * rate)
    t = np.arange(n) / rate
    x = np.zeros(n)
    for k in range(rings):
        tk = t - 0.16 * k
        on = tk >= 0
        for r, a, d in ((1.0, 1.0, 0.5), (1.52, 0.5, 0.3), (2.0, 0.35, 0.25), (2.65, 0.25, 0.15), (3.4, 0.15, 0.1)):
            x += on * a * np.sin(2 * np.pi * f0 * r * np.maximum(tk, 0)) * np.exp(-np.maximum(tk, 0) / d)
    return audio._peak_norm(x, rate)


def siren(kind="two_tone", seconds=4.0, rate=RATE):
    """An emergency siren: `two_tone` (the European hi-lo, 440/587 Hz alternating every 0.75 s, after DIN 14610's
    fourth) or `wail` (a 600-1400 Hz sweep over 4 s, North America), with odd harmonics of a square-ish horn."""
    t = np.arange(int(seconds * rate)) / rate
    if kind == "wail":
        f = 1000.0 + 400.0 * np.sin(2 * np.pi * t / seconds - np.pi / 2)
    else:
        f = np.where((t % 1.5) < 0.75, 440.0, 587.0)
    ph = 2 * np.pi * np.cumsum(f) / rate
    x = np.sin(ph) + 0.33 * np.sin(3 * ph) + 0.2 * np.sin(5 * ph) + 0.1 * np.sin(7 * ph)
    return audio._seamless(audio._peak_norm(x, rate), rate, 0.02)


def horn(seconds=1.0, rate=RATE):
    """A car's two-tone electric horn (~420 and 500 Hz, rich in harmonics: a diaphragm buzzing)."""
    t = np.arange(int(seconds * rate)) / rate
    x = np.zeros_like(t)
    for f in (420.0, 500.0):
        ph = 2 * np.pi * f * t
        x += np.tanh(3.0 * np.sin(ph))
    env = np.minimum(1.0, t / 0.01) * np.minimum(1.0, (seconds - t) / 0.03)
    return audio._peak_norm(x * env, rate)


def air_hiss(seconds=1.2, seed=9, rate=RATE):
    """A bus's air release (kneeling, doors, brakes): a burst of high-passed noise with a fast attack."""
    rng = np.random.default_rng(seed)
    n = int(seconds * rate)
    t = np.arange(n) / rate
    x = _decay_noise(rng, n, 0.35, 1500.0, 12000.0) * np.minimum(1.0, t / 0.02)
    return audio._peak_norm(x, rate)


def birds_synth(seconds=12.0, seed=4, rate=RATE, density=1.0):
    """A fallback bird chorus (used only when no recording can be fetched): FM chirps and trills of a few
    'species' at random times."""
    rng = np.random.default_rng(seed)
    n = int(seconds * rate)
    x = np.zeros(n)
    for _ in range(int(25 * density * seconds / 12.0)):
        t0 = rng.uniform(0, seconds - 1.0)
        kind = rng.integers(3)
        dur = rng.uniform(0.08, 0.35) if kind < 2 else rng.uniform(0.5, 1.2)
        m = int(dur * rate)
        t = np.arange(m) / rate
        f0 = rng.uniform(2500, 5500)
        if kind == 0:
            f = f0 + rng.uniform(-1500, 1500) * t / dur
        elif kind == 1:
            f = f0 * (1 + 0.3 * np.sin(2 * np.pi * rng.uniform(15, 30) * t))
        else:
            f = f0 * (1 + 0.15 * np.sign(np.sin(2 * np.pi * rng.uniform(8, 14) * t)))
        env = np.sin(np.pi * t / dur) ** 2
        s = np.sin(2 * np.pi * np.cumsum(f) / rate) * env * rng.uniform(0.2, 1.0)
        i = int(t0 * rate)
        x[i:i + m] += s[: n - i]
    return audio._norm(audio._seamless(x, rate, 0.05))
