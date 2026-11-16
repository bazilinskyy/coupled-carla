"""Builds the recorded engine loops and footsteps in coupledcarla/data/sounds/ from openly licensed recordings.

    python tools/sound_samples.py fetch            # the source recordings into ~/.cache/coupled-carla/sound-sources
    python tools/sound_samples.py build            # cut, flatten, loop and encode them (needs ffmpeg)
    python tools/sound_samples.py build --plots    # also a spectrogram per loop (needs matplotlib)

Engines, the game-audio way: a few steady-rpm stretches of one engine's recording each become a seamless loop. The
firing frequency (the strongest line of a held rpm; cylinders/2 x rpm/60) is tracked every 10 ms by a harmonic sum,
the stretch is time-warped to a constant firing frequency (so a slow ramp also gives a loop), cut to a whole number
of firing cycles and its ends cross-faded. engines.json records each loop's firing frequency; the soundscape
resamples the two loops around the wanted engine speed and cross-fades them (coupledcarla/enginesamples.py).

Footsteps: single steps (heel and toe) cut at their onsets from a recording of a walker, each in a 0.5 s slot of one
file per surface, the noise floor between steps faded out.

Sources: Freesound (the HQ preview of each sound, which needs no account or key) and Wikimedia Commons; only CC0 and
CC BY. The cut table below is what coupledcarla/data/sounds/SOURCES.md documents.
"""
import argparse
import html
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "coupledcarla" / "data" / "sounds"
CACHE = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "coupled-carla" / "sound-sources"
RATE = 44100
UA = {"User-Agent": "coupled-carla/1.0 (research driving simulator; https://github.com/bazilinskyy/coupled-carla)"}

# Freesound id -> (author, licence); each is re-checked against the sound's page by `fetch`
SOURCES = {
    779214: ("YannSauvin", "CC0 1.0", "Ford Ranger (2000) RPM variations outside 4 mics"),
    425837: ("Soundholder", "CC BY 3.0", "Renault Master 28dTi Engine Start Idle Gas Pressing Off Mono.wav"),
    425838: ("Soundholder", "CC BY 3.0", "Renault Master 28dTi Engine 1_3 Accelerations Wind Mono.wav"),
    641223: ("AlexanderChe", "CC0 1.0", "bike, ignition, start, idle, working, rev, bmw f800gs"),
    546827: ("Kinoton", "CC0 1.0", "Footsteps Leather Concrete"),
    545543: ("rsellick", "CC0 1.0", "light footsteps on concrete walking.wav"),
    764531: ("ChloePieterse", "CC0 1.0", "Footsteps on grass"),
}

# engine set -> cylinders of the recorded engine (only for the notes), loops: (name, source, t0, t1, f0 range Hz)
ENGINES = {
    "petrol": {"source_engine": "Ford Ranger (2000), petrol, exterior at the front of the engine, free revving",
               "loops": [("idle", 779214, 40.0, 46.0, (24, 34)),
                         ("1950", 779214, 100.0, 106.0, (60, 75)),
                         ("2900", 779214, 160.0, 166.0, (90, 105))]},
    "diesel": {"source_engine": "Renault Master 2.8 dTi, turbo diesel, exterior, idle and revs",
               "loops": [("idle", 425837, 5.0, 10.0, (26, 32)),
                         ("1600", 425838, 4.4, 6.4, (48, 58)),
                         ("2050", 425838, 58.0, 61.0, (60, 72)),
                         ("2500", 425838, 53.0, 56.0, (75, 92)),
                         ("3050", 425838, 11.0, 13.8, (95, 108))]},
    "motorbike": {"source_engine": "BMW F 800 GS, parallel twin, exterior, idle and revs",
                  "loops": [("idle", 641223, 60.0, 66.0, (20, 25)),
                            ("2700", 641223, 202.5, 206.0, (38, 52)),
                            ("3600", 641223, 38.3, 41.6, (54, 66)),
                            ("4600", 641223, 190.5, 196.0, (70, 82))]},
}

# surface -> (source, [(t0, t1)] stretches to take steps from, steps wanted)
FOOTSTEPS = {
    "shoes": (546827, [(0.5, 40.0)], 8),           # leather soles on concrete paving
    "trainers": (545543, [(0.3, 32.0)], 8),        # light (soft-soled) footsteps on concrete
    "grass": (764531, [(0.0, 15.0)], 8),
}
SLOT = 0.5


# -- fetching ---------------------------------------------------------------------------------------------------------

def _get(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def fetch():
    CACHE.mkdir(parents=True, exist_ok=True)
    for sid, (author, lic, title) in SOURCES.items():
        dst = CACHE / f"fs{sid}.ogg"
        if dst.exists():
            continue
        page = _get(f"https://freesound.org/s/{sid}/").decode("utf8", "replace")
        ogg = re.search(r'data-ogg="([^"]+)"', page)[1].replace("-lq.ogg", "-hq.ogg")
        have = sorted(set(re.findall(r"creativecommons\.org/(?:licenses|publicdomain)/[a-z-]+/[0-9.]+", page)))
        want = "publicdomain/zero/1.0" if lic.startswith("CC0") else "licenses/by/" + lic.split()[-1]
        if not any(want in h for h in have):
            sys.exit(f"{sid}: licence on the page is {have}, expected {lic}")
        dst.write_bytes(_get(ogg))
        (CACHE / f"fs{sid}.json").write_text(json.dumps({"id": sid, "title": html.unescape(title), "author": author,
                                                         "licence": lic, "preview": ogg}, indent=1))
        print(f"fetched {sid} ({author}, {lic})")
        time.sleep(1.5)


# -- signal helpers ---------------------------------------------------------------------------------------------------

def load(path, rate=RATE):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-ac", "1", "-ar", str(rate), "-f", "f32le", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.float32).astype(np.float64)


def write_ogg(path, x, rate=RATE, quality=7):
    path.parent.mkdir(parents=True, exist_ok=True)
    pcm = (np.clip(x, -1, 1) * 32767).astype("<i2").tobytes()
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "s16le", "-ar", str(rate), "-ac", "1", "-i", "-",
                    "-c:a", "libvorbis", "-q:a", str(quality), str(path)], input=pcm, check=True)


def track_f0(x, lo, hi, rate=RATE, win=0.25, hop=0.01, nh=6):
    """Firing frequency every `hop` s in [lo, hi] Hz: the frequency whose first `nh` harmonics carry the most log
    magnitude (a fine grid, then a parabola through the best three), median-filtered over 70 ms."""
    n, h = int(win * rate), int(hop * rate)
    nfft = 1 << 16
    w = np.hanning(n)
    f = np.fft.rfftfreq(nfft, 1.0 / rate)
    cand = np.linspace(lo, hi, 600)
    out = []
    for i in range(0, len(x) - n, h):
        ls = np.log(np.abs(np.fft.rfft(x[i:i + n] * w, nfft)) + 1e-9)
        sc = sum(np.interp(cand * k, f, ls) for k in range(1, nh + 1))
        j = int(np.clip(np.argmax(sc), 1, len(cand) - 2))
        a, b, c = sc[j - 1], sc[j], sc[j + 1]
        den = a - 2 * b + c
        d = float(np.clip(0.5 * (a - c) / den, -1.0, 1.0)) if den < 0 else 0.0
        out.append(cand[j] + d * (cand[1] - cand[0]))
    f0 = np.array(out)
    k = 3
    pad = np.pad(f0, k, mode="edge")
    f0 = np.array([np.median(pad[i:i + 2 * k + 1]) for i in range(len(f0))])
    t = np.arange(len(f0)) * hop + win / 2
    return t, f0


def flatten(x, t, f0, rate=RATE):
    """x time-warped so that its firing frequency f0(t) becomes its median: (y, F)."""
    F = float(np.median(f0))
    ts = np.arange(len(x)) / rate
    fi = np.interp(ts, t, f0)
    phase = np.cumsum(fi) / rate                         # cycles at each input sample
    n_out = int(phase[-1] / F * rate)
    want = np.arange(n_out) * F / rate                   # cycles at each output sample
    src = np.interp(want, phase, np.arange(len(x)))
    up = 4                                               # band-limited x4 oversampling, then linear interpolation
    X = np.fft.rfft(x)
    xu = np.fft.irfft(np.concatenate([X, np.zeros(len(X) * (up - 1))]), len(x) * up) * up
    return np.interp(src * up, np.arange(len(xu)), xu), F


def make_loop(y, F, rate=RATE, xfade=0.12):
    """A seamless loop of y: a whole number of firing cycles, the tail cross-faded (linearly: the tonal parts are in
    phase at a cycle boundary) into the head."""
    X = int(xfade * rate)
    cycles = math.floor((len(y) - X) / rate * F)
    L = int(round(cycles / F * rate))
    out = y[:L].copy()
    r = np.linspace(0.0, 1.0, X)
    out[:X] = out[:X] * r + y[L:L + X] * (1.0 - r)
    return out, L / rate * F


def onsets(x, rate=RATE, min_gap=0.3, rise_db=10.0):
    """Sample indices where a step starts: the 2 ms high-passed energy rises `rise_db` over the median floor."""
    hp = x - np.convolve(x, np.ones(48) / 48, "same")
    m = int(0.002 * rate)
    e = 10 * np.log10(np.convolve(hp * hp, np.ones(m) / m, "same") + 1e-14)
    floor = np.median(e)
    out, i = [], 0
    above = np.flatnonzero(e > floor + rise_db)
    for j in above:
        if not out or j - out[-1] > min_gap * rate:
            out.append(int(j))
    return out, floor


# -- building ---------------------------------------------------------------------------------------------------------

def build(plots=False):
    man = {"_about": "Engine loops at a known firing frequency (Hz) and footsteps, cut by tools/sound_samples.py; "
                     "sources and licences in SOURCES.md.", "engines": {}, "footsteps": {}}
    for name, spec in ENGINES.items():
        loops = []
        for label, sid, t0, t1, (lo, hi) in spec["loops"]:
            x = load(CACHE / f"fs{sid}.ogg")[int(t0 * RATE):int(t1 * RATE)]
            t, f0 = track_f0(x, lo, hi)
            spread = float(np.ptp(np.log(f0)) * 100)
            y, F = flatten(x, t, f0)
            loop, cyc = make_loop(y, F)
            loop *= 10 ** (-12 / 20) / max(1e-9, float(np.sqrt(np.mean(loop ** 2))))   # -12 dBFS RMS
            fn = f"engines/{name}_{label}.ogg"
            write_ogg(OUT / fn, loop)
            loops.append({"file": fn, "firing_hz": round(F, 2), "seconds": round(len(loop) / RATE, 3),
                          "source": sid, "cut_s": [t0, t1]})
            print(f"{name:9} {label:5} {F:6.2f} Hz  {len(loop) / RATE:4.2f} s  {cyc:5.0f} cycles  "
                  f"f0 spread {spread:4.1f} %  (fs{sid} {t0}-{t1} s)")
            if plots:
                _plot(loop, OUT.parent.parent.parent / "scratch" / f"loop_{name}_{label}.png", F)
        man["engines"][name] = {"source_engine": spec["source_engine"], "loops": loops}
    for surf, (sid, spans, want) in FOOTSTEPS.items():
        x = load(CACHE / f"fs{sid}.ogg")
        steps = []
        for a, b in spans:
            seg = x[int(a * RATE):int(b * RATE)]
            on, floor = onsets(seg)
            for k, o in enumerate(on):
                s0 = max(0, o - int(0.012 * RATE))
                s1 = min(len(seg), s0 + int(SLOT * RATE), on[k + 1] - int(0.01 * RATE) if k + 1 < len(on) else len(seg))
                st = seg[s0:s1]
                if len(st) < int(0.18 * RATE):
                    continue
                pk = float(np.max(np.convolve(st * st, np.ones(4410) / 4410, "same")))
                steps.append((pk, st))
        pks = np.array([p for p, _ in steps])
        med = np.median(pks)
        typical = sorted(steps, key=lambda s: abs(math.log(s[0] / med)))[:want]       # the most typical steps
        buf = np.zeros(int(SLOT * RATE) * len(typical))
        for k, (_, st) in enumerate(typical):
            st = st[:int(SLOT * RATE)].copy()
            fade = int(0.06 * RATE)
            st[-fade:] *= np.linspace(1.0, 0.0, fade)
            buf[k * int(SLOT * RATE):k * int(SLOT * RATE) + len(st)] = st
        buf *= 10 ** (-3 / 20) / max(1e-9, float(np.max(np.abs(buf))))
        fn = f"footsteps/{surf}.ogg"
        write_ogg(OUT / fn, buf)
        man["footsteps"][surf] = {"file": fn, "slot_s": SLOT, "steps": len(typical), "source": sid,
                                  "cut_s": spans}
        print(f"footsteps {surf:8} {len(typical)} steps of {len(steps)} found (fs{sid})")
    (OUT / "sounds.json").write_text(json.dumps(man, indent=1) + "\n")
    total = sum(p.stat().st_size for p in OUT.rglob("*.ogg"))
    print(f"wrote {OUT / 'sounds.json'}; {total / 1e6:.2f} MB of audio")


def _plot(x, path, F):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = 8192
    fr = range(0, len(x) - n, 512)
    S = np.array([np.log(np.abs(np.fft.rfft(x[i:i + n] * np.hanning(n))) + 1e-9) for i in fr]).T
    f = np.fft.rfftfreq(n, 1.0 / RATE)
    m = f < 1200
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4))
    plt.pcolormesh(np.arange(S.shape[1]), f[m], S[m], shading="auto", cmap="magma")
    plt.title(f"{path.stem} {F:.1f} Hz")
    plt.savefig(path, dpi=60)
    plt.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["fetch", "build"])
    ap.add_argument("--plots", action="store_true")
    a = ap.parse_args(argv)
    if not shutil.which("ffmpeg"):
        sys.exit("needs ffmpeg")
    if a.what == "fetch":
        fetch()
    else:
        build(a.plots)


if __name__ == "__main__":
    main()
