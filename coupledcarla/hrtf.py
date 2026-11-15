"""Binaural output for headphones (`sound.spatial: binaural`): each path of each source is convolved with the
head-related impulse responses (HRIRs) of its direction.

HRIRs: the MIT KEMAR dummy-head measurements (Gardner & Martin 1994, MIT Media Lab Perceptual Computing TR 280;
"provided free with no restrictions on use, provided the authors are cited"), the 128-tap compact set, fetched once
at run time from sound.media.mit.edu and cached in ~/.cache/coupled-carla/sounds/ (never in the repository).
Offline, or with `sound.hrtf: sphere`, the spherical-head model of Brown & Duda (1998, A structural model for binaural
sound synthesis, IEEE Trans. Speech Audio Process. 6(5)) gives the interaural time and level differences instead:
Woodworth's delay and a one-pole/one-zero head-shadow filter per ear.
"""
import io
import math
import re
import zipfile

import numpy as np

from . import audio

KEMAR_URL = "https://sound.media.mit.edu/resources/KEMAR/compact.zip"
KEMAR_CREDIT = ("HRTF: MIT KEMAR compact set, Bill Gardner and Keith Martin, MIT Media Lab (1994), "
                "https://sound.media.mit.edu/resources/KEMAR.html, free use with citation")
TAPS = 128


def _sphere_hrir(az, el, rate=audio.RATE, a=0.0875, c=343.0, n=TAPS):
    """Left and right HRIR of a rigid sphere (Brown & Duda 1998) for azimuth az (rad, + right) and elevation el."""
    # direction cosine towards each ear (ears on the +-y axis of the head)
    lateral = math.sin(az) * math.cos(el)
    out = np.zeros((2, n), np.float32)
    w0 = c / a
    f = np.fft.rfftfreq(n, 1.0 / rate)
    w = 2 * np.pi * f
    for ch, ear in enumerate((-1.0, 1.0)):
        cos_t = max(-1.0, min(1.0, ear * lateral))
        theta = math.degrees(math.acos(cos_t))           # 0: facing the source
        alpha = 1.05 + 0.95 * math.cos(math.radians(theta / 150.0 * 180.0))
        H = (1 + 1j * alpha * w / (2 * w0)) / (1 + 1j * w / (2 * w0))
        th = math.radians(theta)
        delay = (-a / c * math.cos(th)) if th < math.pi / 2 else (a / c * (th - math.pi / 2))
        delay += a / c + 0.0005                            # keep it causal
        H = H * np.exp(-1j * w * delay)
        out[ch] = np.fft.irfft(H, n).astype(np.float32) * np.hanning(2 * n)[n:] ** 0.25
    return out


class HRTFSet:
    """A set of HRIR pairs on directions; ir(az, el) gives the nearest one (and its index)."""

    def __init__(self, dirs, irs, name, equalise=True):
        self.dirs = np.asarray(dirs, float)        # (m, 2): az, el in rad
        irs = np.asarray(irs, np.float64)          # (m, 2, taps)
        if equalise:
            irs = self._diffuse_eq(irs)
        self.irs = irs.astype(np.float32)
        self.taps = self.irs.shape[2]
        self.name = name
        el, az = self.dirs[:, 1], self.dirs[:, 0]
        self.vec = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])

    @staticmethod
    def _diffuse_eq(irs, n=256):
        """Diffuse-field equalisation: every HRIR divided by the direction-averaged response (smoothed over a third
        of an octave), scaled so that the two ears together carry the power of the source on average, as constant-
        power panning does: the levels of the stereo and binaural outputs agree, and the measurement's own
        loudspeaker and low-frequency roll-off (128-tap compact set) drop out."""
        H = np.fft.rfft(irs, n=n, axis=2)
        P = np.mean(np.abs(H) ** 2, axis=(0, 1))                       # per frequency
        f = np.arange(len(P))
        Ps = np.array([P[max(0, int(k / 1.12)):int(k * 1.12) + 2].mean() for k in f])
        Ps[0] = Ps[1]
        eq = 1.0 / np.sqrt(np.maximum(Ps, Ps.max() * 1e-4))
        out = np.fft.irfft(H * eq, n=n, axis=2)
        out *= np.hanning(2 * n)[n:] ** 0.5                            # taper the circular tail
        e = np.mean(np.sum(out ** 2, axis=(1, 2)))
        return out / math.sqrt(e)

    def index(self, az, el):
        v = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        return int(np.argmax(self.vec @ v))

    @classmethod
    def sphere(cls, step_deg=10):
        dirs, irs = [], []
        for el in range(-40, 91, step_deg):
            for az in range(-180, 180, step_deg):
                dirs.append((math.radians(az), math.radians(el)))
                irs.append(_sphere_hrir(math.radians(az), math.radians(el)))
        return cls(dirs, irs, "sphere (Brown & Duda 1998)")

    @classmethod
    def kemar(cls, data):
        """From the bytes of MIT's compact.zip (WAV files elev<E>/H<E>e<AAA>a.wav, left/right ear, right side)."""
        import wave
        dirs, irs = [], []
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            for name in z.namelist():
                m = re.search(r"H(-?\d+)e(\d+)a\.wav$", name)
                if not m:
                    continue
                el, az = int(m.group(1)), int(m.group(2))
                with wave.open(io.BytesIO(z.read(name))) as w:
                    x = np.frombuffer(w.readframes(w.getnframes()), "<i2").reshape(-1, 2).T.astype(np.float32) / 32768
                for a, pair in ((az, x), (-az, x[::-1])):
                    if a == -az and az in (0, 180):
                        continue
                    dirs.append((math.radians(a), math.radians(el)))
                    irs.append(pair)
        if len(irs) < 100:
            raise ValueError(f"only {len(irs)} HRIRs in the archive")
        irs = np.asarray(irs)
        irs /= np.sqrt(np.mean(np.sum(irs ** 2, axis=2)))          # unit mean energy per ear
        return cls(dirs, irs, "MIT KEMAR compact")


def load(which="kemar", log=print):
    """The HRTF set: `kemar` (fetched and cached; the sphere model if that fails) or `sphere`. Returns (set, credit)."""
    if which == "kemar":
        from . import soundfetch
        data = soundfetch.fetch_bytes(KEMAR_URL, "kemar_compact.zip", log=log)
        if data is not None:
            try:
                return HRTFSet.kemar(data), KEMAR_CREDIT
            except Exception as e:  # noqa: BLE001
                log(f"sound: the KEMAR archive is unreadable ({e}); spherical-head model instead")
        else:
            log("sound: KEMAR HRTFs not available offline; spherical-head model (Brown & Duda 1998) instead")
    return HRTFSet.sphere(), None


class Binaural:
    """The mixer's spatializer: render(y, path) -> (n, 2), the path's signal through the HRIR of its direction,
    with overlap-add state per path and a cross-fade when the direction moves to another HRIR."""

    def __init__(self, hset):
        self.h = hset

    def render(self, y, p):
        idx = self.h.index(p.az, p.el)
        st = p._conv
        if st is None:
            st = p._conv = {"idx": idx, "tail": np.zeros((2, self.h.taps - 1), np.float32)}
        n = len(y)
        ir = self.h.irs[idx]
        yy = np.concatenate([np.zeros(0, np.float32), y])
        new = np.stack([np.convolve(yy, ir[0]), np.convolve(yy, ir[1])])           # (2, n + taps - 1)
        if st["idx"] != idx:
            old_ir = self.h.irs[st["idx"]]
            old = np.stack([np.convolve(yy, old_ir[0]), np.convolve(yy, old_ir[1])])
            fade = np.ones(new.shape[1], np.float32)
            fade[:n] = np.linspace(0.0, 1.0, n, endpoint=False)
            new = old * (1 - fade) + new * fade
            st["idx"] = idx
        new[:, :self.h.taps - 1] += st["tail"]
        st["tail"] = new[:, n:].copy()
        return new[:, :n].T
