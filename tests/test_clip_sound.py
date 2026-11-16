"""Demo-video sound: drones only in clips about drones, the outside cameras' high-pass on the engines' hum."""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def mv():
    spec = importlib.util.spec_from_file_location("make_videos_t", ROOT / "tools" / "make_videos.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["make_videos_t"] = m
    spec.loader.exec_module(m)
    return m


def test_drone_sound_only_in_drone_clips(mv):
    exp = {"participants": [{"role": "pedestrian", "kind": "pedestrian"}, {"role": "pilot", "kind": "drone"}]}
    assert mv.wants_drone_sound({"camera": {"kind": "drone", "target": "pilot"}})
    assert not mv.wants_drone_sound({"camera": {"kind": "follow", "target": "driver"}})
    four = {"kind": "stations", "stations": {"views": [{"role": "pedestrian"}, {"role": "pilot"}]}}
    assert mv.wants_drone_sound(four, exp)
    assert not mv.wants_drone_sound({"kind": "stations", "stations": {"views": [{"role": "pedestrian"}]}}, exp)
    assert mv.wants_drone_sound({"camera": {"kind": "follow"}, "drone_sound": True})


def test_every_clip_in_the_list(mv):
    clips = mv.load_clips() if hasattr(mv, "load_clips") else None
    if clips is None:
        pytest.skip("no load_clips")
    for c in clips:
        exp = mv.derive_experiment(c, {"launch": False, "host": "127.0.0.1", "port": 2000})
        drones = (exp.get("sound") or {}).get("drone_sound", True)
        about = "drone" in c["name"] or "four_views" in c["name"]
        assert drones == about, c["name"]


def test_highpass_takes_out_the_hum():
    from coupledcarla import audio, filmaudio
    r = audio.RATE
    t = np.arange(r * 2) / r
    hum, tyre = np.sin(2 * np.pi * 50 * t), 0.2 * np.sin(2 * np.pi * 800 * t)
    x = np.stack([hum + tyre, hum + tyre], axis=1).astype(np.float32)
    y = filmaudio.highpass(x, 120.0, rate=r)
    spec = lambda s, f: np.abs(np.fft.rfft(s[:, 0]))[int(f * 2)]      # 2 s: bin = 2 f
    assert spec(y, 50) < 0.05 * spec(x, 50) and spec(y, 800) > 0.95 * spec(x, 800)
    assert filmaudio.highpass(x, 0) is x
