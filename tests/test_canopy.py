"""Canopy heights: quadkeys, strip decoding, sampling, calibration, caching and fail-soft (no network)."""
import io
import zlib

import numpy as np
import pytest

from coupledcarla import canopy


def test_quadkeys_of_known_tiles():
    assert canopy.quadkey(-36.8485, 174.7633) == "311333000"      # Auckland CBD
    assert canopy.quadkey(51.4416, 5.4697) == "120202131"         # Eindhoven centre


def test_decode_rows_undoes_the_horizontal_predictor():
    rows = np.array([[0, 5, 9, 9, 2], [1, 1, 1, 1, 1]], np.uint8)
    diff = np.diff(rows.astype(np.int16), axis=1, prepend=0).astype(np.uint8)   # what a predictor-2 TIFF stores
    out = canopy._decode_rows(zlib.compress(diff.tobytes()), 5, 2, predictor=2)
    assert (out == rows).all()


def _grid():
    z = np.zeros((100, 100), np.uint8)
    z[40:60, 40:60] = 10                     # a crown of 10 m in the middle
    x, y = canopy.mercator(0.0005, -0.0005)  # top left near the equator, 1 m pixels
    return canopy.Canopy(z, x, y, 1.0)


def test_sampling_and_calibration():
    c = _grid()
    lat, lon = 0.0, 0.0                      # the centre pixel (50, 50)
    assert c.at(lat, lon, radius_m=0) == 10
    assert c.tree_height(lat, lon) == pytest.approx(canopy.CALIBRATION[0] + canopy.CALIBRATION[1] * 10)
    assert c.at(0.00045, -0.00045, radius_m=0) == 0 and c.tree_height(0.00045, -0.00045) is None
    assert c.at(0.0003, -0.0003, radius_m=0) == 0 and c.at(0.0003, -0.0003, radius_m=30) == 10   # crown nearby
    assert c.at(1.0, 1.0) is None                                                                  # off the grid


def test_canopy_caches_and_fails_soft(monkeypatch, tmp_path):
    pytest.importorskip("tifffile")
    bbox = [-0.0002, -0.0002, 0.0002, 0.0002]
    monkeypatch.setattr(canopy, "_read_window", lambda *a: (_ for _ in ()).throw(OSError("offline")))
    assert canopy.canopy(bbox, tmp_path / "c.npz") is None and not (tmp_path / "c.npz").exists()

    def missing(*a):
        raise FileNotFoundError("no tile")
    monkeypatch.setattr(canopy, "_read_window", missing)
    c = canopy.canopy(bbox, tmp_path / "c.npz")          # no tile here: an empty grid, cached
    assert c.z.size == 0 and c.tree_height(0, 0) is None and (tmp_path / "c.npz").exists()


def test_read_window_from_a_striped_geotiff(monkeypatch):
    tifffile = pytest.importorskip("tifffile")
    pytest.importorskip("imagecodecs")
    z = (np.arange(64 * 64) % 23).astype(np.uint8).reshape(64, 64)
    left, top, res = 1000.0, 2000.0, 1.0
    buf = io.BytesIO()
    tifffile.imwrite(buf, z, compression="zlib", predictor=True, rowsperstrip=1,
                     extratags=[(33550, "d", 3, (res, res, 0.0)), (33922, "d", 6, (0, 0, 0, left, top, 0))])
    data = buf.getvalue()

    class Resp:
        def __init__(self, content, status=206):
            self.content, self.status_code = content, status
            self.headers = {"Content-Range": f"bytes 0-0/{len(data)}"}

        def raise_for_status(self):
            pass

    class Session:
        def get(self, url, headers=None, timeout=None):
            a, b = headers["Range"].split("=")[1].split("-")
            return Resp(data[int(a):int(b) + 1])
    out, l, t, r = canopy._read_window("https://x/0.tif", Session(), left + 10, top - 30, left + 20, top - 5)
    assert (l, t, r) == (left + 10, top - 5, 1.0)
    assert (out == z[5:30, 10:20]).all()
