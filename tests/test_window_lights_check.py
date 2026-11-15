"""tools/window_lights_check.py: the lit-window pixel measure and the EhvCentre camera poses (no CARLA server)."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
wl = pytest.importorskip("window_lights_check")


def _image(w, h, pixel_at):
    raw = bytearray()
    for y in range(h):
        for x in range(w):
            r, g, b = pixel_at(x, y)
            raw += bytes((b, g, r, 255))     # BGRA, as carla.Image.raw_data
    return SimpleNamespace(width=w, height=h, raw_data=bytes(raw))


def test_measure_counts_lit_windows_not_brick_or_sky():
    brick = _image(60, 60, lambda x, y: (122, 52, 38))
    sky = _image(60, 60, lambda x, y: (230, 235, 240))
    lit = _image(60, 60, lambda x, y: (255, 190, 110) if (x // 10) % 2 else (20, 15, 12))
    assert wl.measure(brick)[0] == 0.0
    assert wl.measure(sky)[0] == 0.0
    share, mean = wl.measure(lit)
    assert 0.4 < share < 0.6 and mean > 50
    # only the upper two thirds count (the facades, not the road)
    low = _image(60, 60, lambda x, y: (255, 190, 110) if y >= 40 else (0, 0, 0))
    assert wl.measure(low)[0] == 0.0


def test_poses_are_eindhoven_centre_points():
    assert len(wl.POSES) >= 3
    assert {p[0] for p in wl.POSES} >= {"home", "shop", "office"}
    for use, x, y, z, yaw in wl.POSES:
        assert -900 < x < 900 and -900 < y < 900       # inside the twin (bbox_ue +-860 m)
        assert 10 < z < 40 and 0 <= yaw < 360            # NAP ground (~16-19 m) plus eye height
