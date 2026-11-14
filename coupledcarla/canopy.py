"""Tree heights from Meta and WRI's global 1 m canopy height map (Tolan et al. 2024, CC BY 4.0).

The map is a set of cloud-optimised GeoTIFFs on AWS open data (s3://dataforgood-fb-data, anonymous), one per
zoom-9 quadkey: 65,536 x 65,536 pixels of about 1.2 m (Web Mercator), canopy height in whole metres, stored in
deflate-compressed rows. Only the rows of the map's box are read (HTTP range requests); the crop is cached in
maps/<name>/real/canopy.npz and the scenery samples it where OSM gives a tree no height (scenery._tree).
"""
import io
import math
import time
import zlib
from pathlib import Path

import numpy as np
import requests

CHM = "https://dataforgood-fb-data.s3.amazonaws.com/forests/v1/alsgedi_global_v6_float/chm/{qk}.tif"
ZOOM = 9                     # the quadkey level of the files
# The map sees urban trees too low: against Eindhoven's tree register (1,132 trees it sees, height classes
# 4.5-26 m) its value is 6.5 m under the register's (median) and street trees under ~8 m are mostly missed.
# Its ranks still carry the size, so a tree's height is a line fitted by quantile matching to that register.
SEEN_M = 3.0                 # canopy under this: the map sees no tree here (the guessed size stays)
CALIBRATION = (6.0, 1.1)     # tree height = 6.0 m + 1.1 x canopy height
MAX_M = 30.0
R = 6378137.0
HALF = math.pi * R           # Web Mercator half extent (m)


def log(msg):
    print(f"[canopy] {msg}", flush=True)


def mercator(lat, lon):
    lat = max(-85.05, min(85.05, lat))
    return R * math.radians(lon), R * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))


def quadkey(lat, lon, z=ZOOM):
    x, y = mercator(lat, lon)
    n = 2 ** z
    tx = min(n - 1, int((x + HALF) / (2 * HALF) * n))
    ty = min(n - 1, int((HALF - y) / (2 * HALF) * n))
    return "".join(str(((tx >> i) & 1) + 2 * ((ty >> i) & 1)) for i in range(z - 1, -1, -1))


class RangeFile(io.RawIOBase):
    """A read-only file over HTTP range requests, in cached blocks (for tifffile to parse the header)."""

    def __init__(self, url, session, block=1 << 20):
        super().__init__()
        self.url, self.s, self.block, self.cache, self.pos = url, session, block, {}, 0
        r = self.s.get(url, headers={"Range": "bytes=0-0", **_ua()}, timeout=60)
        if r.status_code in (403, 404):
            raise FileNotFoundError(f"{url}: HTTP {r.status_code}")
        r.raise_for_status()
        self.size = int(r.headers.get("Content-Range", "/0").split("/")[-1] or 0)

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos

    def get(self, a, b):
        """Bytes a..b-1 in one request."""
        r = self.s.get(self.url, headers={"Range": f"bytes={a}-{b - 1}", **_ua()}, timeout=120)
        r.raise_for_status()
        return r.content

    def read(self, n=-1):
        end = self.size if n is None or n < 0 else min(self.size, self.pos + n)
        out = bytearray()
        while self.pos < end:
            i, o = divmod(self.pos, self.block)
            if i not in self.cache:
                self.cache[i] = self.get(i * self.block, min(self.size, (i + 1) * self.block))
            chunk = self.cache[i][o:o + end - self.pos]
            if not chunk:
                break
            out += chunk
            self.pos += len(chunk)
        return bytes(out)

    def readinto(self, b):
        d = self.read(len(b))
        b[:len(d)] = d
        return len(d)


def _ua():
    from . import realworld
    return realworld.UA


def _decode_rows(data, width, rows, predictor):
    """One deflate strip of uint8 rows (TIFF predictor 2: horizontal differences, summed back mod 256)."""
    a = np.frombuffer(zlib.decompress(data), dtype=np.uint8)[:width * rows].reshape(rows, width)
    return np.cumsum(a, axis=1, dtype=np.uint8) if predictor == 2 else a


def _read_window(url, session, x0, y0, x1, y1):
    """The canopy heights of one file inside the Mercator box (x0, y0)-(x1, y1): (array, left, top, res)."""
    import tifffile
    f = RangeFile(url, session)
    with tifffile.TiffFile(f) as tif:
        p = tif.pages[0]
        res = float(p.tags[33550].value[0])
        left, top = float(p.tags[33922].value[3]), float(p.tags[33922].value[4])
        width, height = p.imagewidth, p.imagelength
        if p.is_tiled or p.dtype != np.uint8 or p.compression not in (8, 32946) or p.samplesperpixel != 1:
            raise ValueError(f"unexpected layout ({p.compression}, {p.dtype}, tiled {p.is_tiled})")
        rps = int(p.tags[278].value) if 278 in p.tags else height
        offsets, counts = p.dataoffsets, p.databytecounts
        predictor = int(p.predictor or 1)
    c0, c1 = max(0, int((x0 - left) / res)), min(width, int(math.ceil((x1 - left) / res)))
    r0, r1 = max(0, int((top - y1) / res)), min(height, int(math.ceil((top - y0) / res)))
    if c1 <= c0 or r1 <= r0:
        return None
    s0, s1 = r0 // rps, (r1 - 1) // rps + 1
    out = np.zeros(((s1 - s0) * rps, c1 - c0), dtype=np.uint8)
    i = s0
    while i < s1:     # contiguous runs of strips in one request each (at most ~8 MB)
        j, a = i, offsets[i]
        while j + 1 < s1 and offsets[j + 1] == offsets[j] + counts[j] and offsets[j + 1] - a < 8 << 20:
            j += 1
        blob = f.get(a, offsets[j] + counts[j])
        for k in range(i, j + 1):
            o = offsets[k] - a
            rows = min(rps, height - k * rps)
            out[(k - s0) * rps:(k - s0) * rps + rows] = _decode_rows(blob[o:o + counts[k]], width, rows,
                                                                     predictor)[:, c0:c1]
        i = j + 1
    out = out[r0 - s0 * rps:r1 - s0 * rps]
    return out, left + c0 * res, top - r0 * res, res


class Canopy:
    """Canopy heights (m) on a Web Mercator grid, sampled at lat/lon."""

    def __init__(self, z, left, top, res):
        self.z, self.left, self.top, self.res = z, float(left), float(top), float(res)

    def at(self, lat, lon, radius_m=3.0):
        """The tallest canopy within radius_m of the point (a mapped trunk is not always under the crown's top),
        or None outside the grid."""
        if self.z.size == 0:
            return None
        x, y = mercator(lat, lon)
        k = 1 / math.cos(math.radians(lat))       # Mercator metres per ground metre
        c, r = (x - self.left) / self.res, (self.top - y) / self.res
        d = max(0, int(round(radius_m * k / self.res)))
        ci, ri = int(c), int(r)
        if not (0 <= ci < self.z.shape[1] and 0 <= ri < self.z.shape[0]):
            return None
        win = self.z[max(0, ri - d):ri + d + 1, max(0, ci - d):ci + d + 1]
        return float(win.max())

    def tree_height(self, lat, lon):
        """The height (m) of a tree standing here by the calibrated canopy map, or None where it sees none."""
        c = self.at(lat, lon)
        if c is None or c < SEEN_M:
            return None
        return min(MAX_M, CALIBRATION[0] + CALIBRATION[1] * c)

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, z=self.z, geo=np.array([self.left, self.top, self.res]))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d["z"], *d["geo"])


def canopy(bbox, path, margin_m=30.0):
    """The canopy grid of the box (cached in path), or None when it cannot be read (offline: no cache, so the
    next build tries again). Outside the map's coverage: an empty grid."""
    path = Path(path)
    if path.exists():
        return Canopy.load(path)
    try:
        import tifffile  # noqa: F401
    except ImportError:
        log("skipped: pip install tifffile")
        return None
    s, w, n, e = bbox
    k = 1 / math.cos(math.radians((s + n) / 2))
    x0, y0 = mercator(s, w)
    x1, y1 = mercator(n, e)
    x0, y0, x1, y1 = x0 - margin_m * k, y0 - margin_m * k, x1 + margin_m * k, y1 + margin_m * k
    keys = sorted({quadkey(lat, lon) for lat in (s, n) for lon in (w, e)})
    session = requests.Session()
    t0 = time.time()
    parts = []
    try:
        for qk in keys:
            try:
                win = _read_window(CHM.format(qk=qk), session, x0, y0, x1, y1)
            except FileNotFoundError:
                continue          # no file: open sea, or outside the map
            if win is not None:
                parts.append(win)
    except Exception as err:  # noqa: BLE001 - a broken service: today's guessed tree sizes
        log(f"not read ({err}); trees keep their guessed sizes")
        return None
    if not parts:
        grid = Canopy(np.zeros((0, 0), np.uint8), x0, y1, 1.0)
    else:
        res = parts[0][3]
        left, top = min(p[1] for p in parts), max(p[2] for p in parts)
        right = max(p[1] + p[0].shape[1] * res for p in parts)
        bottom = min(p[2] - p[0].shape[0] * res for p in parts)
        z = np.zeros((int(round((top - bottom) / res)), int(round((right - left) / res))), np.uint8)
        for a, l, t, _ in parts:
            r, c = int(round((top - t) / res)), int(round((l - left) / res))
            z[r:r + a.shape[0], c:c + a.shape[1]] = a
        grid = Canopy(z, left, top, res)
    grid.save(path)
    log(f"{grid.z.shape[1]} x {grid.z.shape[0]} px from {len(parts)} of {len(keys)} tiles ({', '.join(keys)}) "
        f"in {time.time() - t0:.0f} s")
    return grid
