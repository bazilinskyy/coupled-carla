"""coupledcarla.synth and the mixer's spatial paths: band-shaped noise, engine and rotor partials carrying given band
levels, delays, reverb, the limiter, binaural output, recordings (fetch monkeypatched). No network, no device."""
import io
import json
import math
import wave
import zipfile

import numpy as np
import pytest

from coupledcarla import acoustics as ac
from coupledcarla import audio, hrtf, soundfetch, synth
from coupledcarla.soundrec import level_dba

REF = 94.0


def band_levels(x, ref=REF, rate=audio.RATE):
    spec = np.abs(np.fft.rfft(x)) ** 2 * 2.0 / len(x) ** 2
    f = np.fft.rfftfreq(len(x), 1.0 / rate)
    return np.array([ref + 10 * math.log10(spec[(f >= ac.EDGES[b]) & (f < ac.EDGES[b + 1])].sum() / audio.FS_RMS ** 2
                                           + 1e-30) for b in range(ac.NB)])


def render(v, seconds=2.0, warm=2):
    m = audio.Mixer()
    m.limiter = None
    m.add("v", v)
    for _ in range(warm):
        m.render(2048)
    return m.render(int(seconds * audio.RATE))


def test_band_loops_are_calibrated_and_disjoint():
    L = synth.band_loops("pink", 2.0, 1)
    assert L.shape == (8, 88200)
    assert np.allclose(np.sqrt(np.mean(L.astype(float) ** 2, axis=1)), audio.FS_RMS, rtol=1e-3)
    c = np.corrcoef(L)
    assert np.all(np.abs(c - np.eye(8)) < 0.02)              # the bands are uncorrelated: their powers add


def test_band_noise_plays_the_given_band_levels():
    want = np.array([60.0, 65, 68, 70, 72, 69, 63, 55])
    v = synth.BandNoise("pink", seed=2, kinds=("direct",))
    v.paths[0].w = v.weights(want, REF)
    x = render(v)
    got = band_levels(x[:, 0] * math.sqrt(2))                # centre pan: -3 dB per ear
    assert np.allclose(got, want, atol=0.7)                   # 2 s of noise: the 63 Hz band varies most
    assert level_dba(x) == pytest.approx(ac.a_weighted(want), abs=0.3)


@pytest.mark.parametrize("diesel", [False, True])
def test_engine_carries_the_band_levels_a_weighted(diesel):
    want = np.array([75.0, 72, 70, 67, 65, 62, 58, 52])
    e = synth.EngineVoice(4, diesel, seed=3, kinds=("direct",))
    e.set_rpm(1800.0, 0.4)
    e.paths[0].w = e.weights(want, REF)
    x = render(e, 3.0)
    assert level_dba(x) == pytest.approx(ac.a_weighted(want), abs=1.0)
    # the firing frequency (4 cylinders: rpm / 30 = 60 Hz) stands out of the spectrum
    s = np.abs(np.fft.rfft(x[:, 0]))
    f = np.fft.rfftfreq(len(x), 1.0 / audio.RATE)
    peak = s[(f > 55) & (f < 65)].max()
    floor = np.median(s[(f > 40) & (f < 80)])
    assert peak > 10 * floor
    # cycle-to-cycle jitter: the firing tone is not a pure line (its energy spreads over a few Hz)
    band = (f > 58) & (f < 62)
    assert np.count_nonzero(s[band] > 0.3 * peak) >= 2


def test_engine_partials_follow_rpm_and_load():
    e = synth.EngineVoice(4, False)
    e.set_rpm(3000.0, 0.0)
    assert e.base == pytest.approx(100.0)                     # firing frequency
    soft = e.rel()
    e.set_rpm(3000.0, 1.0)
    hard = e.rel()
    assert hard[-1] / hard[1] > soft[-1] / soft[1]            # full load: stronger high orders
    six = synth.EngineVoice(6, True)
    six.set_rpm(1200.0, 0.5)
    assert six.base == pytest.approx(60.0)


def test_rotor_tones_at_the_blade_passing_frequency():
    r = synth.RotorVoice(2, seed=1, kinds=("direct",))
    r.set_bpf(180.0)
    shape = synth.RotorVoice.SHAPE - ac.a_weighted(synth.RotorVoice.SHAPE) + 60.0
    r.paths[0].w = r.weights(shape, REF)
    x = render(r, 2.0)
    assert level_dba(x) == pytest.approx(60.0, abs=1.0)
    s = np.abs(np.fft.rfft(x[:, 0]))
    f = np.fft.rfftfreq(len(x), 1.0 / audio.RATE)
    for h in (1, 2, 3):        # tonal peaks at BPF harmonics over the broadband noise
        assert s[(f > 180 * h * 0.98) & (f < 180 * h * 1.02)].max() > 5 * np.median(s[(f > 150 * h) & (f < 210 * h)])


def test_reflection_paths_are_delayed():
    v = synth.ShapedMono(audio.cue_sound("tick"), kinds=("direct", "refl"))
    w = v.weights(v.band_db(80.0), REF)
    v.paths[0].set(w, 0.0, -1.5, 0.0)        # direct from the left
    v.paths[1].set(w * 0.5, 0.050, 1.5, 0.0)   # a reflection 50 ms later from the right
    m = audio.Mixer()
    m.add("v", v)
    x = m.render(8192)
    on_l = int(np.argmax(np.abs(x[:, 0]) > 0.01))
    on_r = int(np.argmax(np.abs(x[:, 1]) > 0.005))
    assert on_l < 50 and abs((on_r - on_l) - 0.050 * audio.RATE) < 30


def test_reverb_tail_and_limiter():
    r = synth.Reverb(1.0, part=1024)
    imp = np.zeros(44100, np.float32)
    imp[0] = 1.0
    y = np.concatenate([r.process(imp[i:i + 2205]) for i in range(0, len(imp), 2205)])
    e = (y ** 2).sum(axis=1)
    early, late = e[2000:6000].sum(), e[30000:34000].sum()
    assert early > 0 and 1e-4 < late / early < 0.5            # decays, not instantly
    r.set_rt(2.0)
    assert len(r.engines) == 2                                 # cross-fading to the new tail
    # limiter: a sum that would clip stays under full scale
    m = audio.Mixer()
    m.add("a", audio.Loop(audio.tone(500, 1.0), gain=1.0, pan=1.0))
    m.add("b", audio.Loop(audio.tone(500, 1.0), gain=1.0, pan=1.0))
    x = np.concatenate([m.render(2048) for _ in range(10)])
    assert np.abs(x).max() <= 1.0 and m.clipped == 0 and m.limiter.active > 0


def test_event_voice_rate_and_footsteps():
    steps = synth.footstep_buffers("hard", 4)
    v = synth.EventVoice(steps, seed=1, jitter=0.0, kinds=("direct",))
    v.period = 0.5
    v.paths[0].w = v.weights(v.band_db(70.0), REF)
    m = audio.Mixer()
    m.add("s", v)
    for _ in range(int(4.0 * audio.RATE / 2048)):
        m.render(2048)
    assert 7 <= v.events <= 9                                  # 2 steps a second for 4 s
    v.period = None
    n = v.events
    m.render(44100)
    assert v.events == n


def test_hrtf_sphere_and_kemar_archive():
    s = hrtf.HRTFSet.sphere(30)
    right = s.irs[s.index(math.radians(90), 0.0)]
    assert np.sum(right[1] ** 2) > np.sum(right[0] ** 2)       # a source on the right: louder in the right ear
    assert np.argmax(np.abs(right[1])) < np.argmax(np.abs(right[0]))   # and earlier
    # a KEMAR-style archive (two directions) parses, mirrored to the left
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for el in range(-40, 91, 10):
            for az in range(0, 181, 15):
                w = io.BytesIO()
                with wave.open(w, "wb") as wf:
                    wf.setnchannels(2)
                    wf.setsampwidth(2)
                    wf.setframerate(44100)
                    ir = np.zeros((128, 2), np.int16)
                    ir[5, 0], ir[3, 1] = 1000, 2000 + 10 * az
                    wf.writeframes(ir.tobytes())
                z.writestr(f"elev{el}/H{el}e{az:03d}a.wav", w.getvalue())
    k = hrtf.HRTFSet.kemar(buf.getvalue())
    i_r, i_l = k.index(math.radians(90), 0), k.index(math.radians(-90), 0)
    assert k.irs[i_r][1, 3] > 0 and k.irs[i_l][0, 3] > 0       # mirrored: the left source's near ear is the left
    # the binaural spatializer renders a path
    b = hrtf.Binaural(s)
    p = synth.Path("direct")
    p.az = math.radians(90)
    y = b.render(np.ones(1000, np.float32), p)
    assert y.shape == (1000, 2) and np.abs(y[:, 1]).sum() > np.abs(y[:, 0]).sum()


def test_recordings_licence_check_and_cache(monkeypatch, tmp_path):
    assert soundfetch.licence_ok("CC BY-SA 4.0") and soundfetch.licence_ok("Public domain")
    assert not soundfetch.licence_ok("CC BY-NC 4.0") and not soundfetch.licence_ok("")
    audio.write_wav(tmp_path / "x.wav", np.zeros((4410, 2)))
    wav_bytes = (tmp_path / "x.wav").read_bytes()
    calls = []

    def fake_get(url, timeout=60):
        calls.append(url)
        if "api.php" in url:
            lic = "CC BY-NC 4.0" if "Bad" in url else "CC0"
            return json.dumps({"query": {"pages": {"1": {"title": "File:X.wav", "imageinfo": [{
                "url": "https://upload.example/X.wav", "descriptionurl": "https://commons.example/X",
                "size": 1000, "mime": "audio/wav", "extmetadata": {"LicenseShortName": {"value": lic},
                                                                   "Artist": {"value": "<a>Someone</a>"}}}]}}}}).encode()
        return wav_bytes

    monkeypatch.setattr(soundfetch, "_get", fake_get)
    monkeypatch.setattr(soundfetch, "RECORDINGS", {"r": ["File:Bad.wav", "File:X.wav"]})
    monkeypatch.setattr(soundfetch, "_last_download", [0.0])
    monkeypatch.setattr(soundfetch.time, "sleep", lambda s: None)
    monkeypatch.setattr(soundfetch.shutil, "which", lambda exe: None)       # no ffmpeg: WAV copied as is
    path, info = soundfetch.recording("r", log=lambda s: None)
    assert path is not None and path.exists() and info["licence"] == "CC0" and info["artist"] == "Someone"
    assert "Someone" in soundfetch.credit_line(info)
    n = len(calls)
    assert soundfetch.recording("r", log=lambda s: None)[0] == path and len(calls) == n    # cached
