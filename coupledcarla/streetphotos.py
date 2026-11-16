"""Street photos for the realism check (realism.py, tools/realism_check.py): Mapillary, the extra comparison metrics,
and the street objects the photos show against the ones the map places.

- Mapillary (CC BY-SA 4.0; needs the client token `mapillary_token`, read with settings.get_secret and never
  printed): images and map features come from its vector tiles (the graph API's bbox search answers empty lists
  since 2026), the chosen images' poses and URLs from the graph API by id. Everything is cached under the map's
  real/streetphotos/ (cache.py-style files; tiles kept 30 days).
- Metrics beside realism.compare_labels: boundary F-score (do the outlines of road, buildings, trees and sky line
  up?), DINOv2 similarity (a perceptual distance: the whole image and patch by patch), and the main differences of a
  spot in words ("missing building on the left", "road too wide").
- Objects: Mapillary map features (street lights, signs, benches, bins, hydrants...) and poles found in Panoramax
  panoramas by the Cityscapes model, matched with the lamps, signs and furniture of scenery.json; a correction pass
  (move / add / remove) that is off unless asked for and leaves items from official registers alone.
"""
import hashlib
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

MAPILLARY = ("Mapillary street-level imagery (tiles.mapillary.com, graph.mapillary.com; token)", "CC BY-SA 4.0")
TOKEN_HELP = ("Mapillary needs a client token: log in at mapillary.com, open Dashboard > Developers, register an "
              "application (any name; read access is enough) and copy its client token (it starts with MLY|). Put it "
              "in the project's `secret` file as \"mapillary_token\" (python -m coupledcarla.settings --set "
              "mapillary_token asks for it without echo), or in $MAPILLARY_TOKEN. Never paste it into a chat, a "
              "commit or a log.")
TILE_Z = 14
TILES = "https://tiles.mapillary.com/maps/vtp/{layer}/2/{z}/{x}/{y}"
GRAPH = "https://graph.mapillary.com"
IMAGE_FIELDS = ("id,captured_at,compass_angle,computed_compass_angle,geometry,computed_geometry,camera_type,"
                "camera_parameters,width,height,make,model,creator,sequence,thumb_2048_url,is_pano")


class FetchError(RuntimeError):
    pass


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- polite HTTP and the disk cache

class Http:
    """GET with a minimum interval per host, retries with backoff on 429/5xx, and error messages that never carry
    the URL (Mapillary's tiles take the token in the query)."""

    def __init__(self, interval=None, tries=4, session=None, sleep=time.sleep):
        self.interval = {"tiles.mapillary.com": 0.2, "graph.mapillary.com": 0.1, "api.panoramax.xyz": 0.5}
        self.interval.update(interval or {})
        self.tries, self.sleep, self.last = tries, sleep, {}
        self.session = session
        self.n = 0

    def get(self, url, params=None, headers=None, what=None):
        import requests
        from .realworld import UA
        host = urlsplit(url).hostname or ""
        what = what or host
        s = self.session or requests
        for k in range(self.tries):
            wait = self.interval.get(host, 0.2) - (time.time() - self.last.get(host, 0.0))
            if wait > 0:
                self.sleep(wait)
            self.last[host] = time.time()
            try:
                r = s.get(url, params=params, headers={**UA, **(headers or {})}, timeout=120)
            except Exception as err:  # noqa: BLE001 - network error: retried, then reported without the URL
                code, retry = type(err).__name__, True
            else:
                self.n += 1
                if r.status_code == 200:
                    return r
                code, retry = f"HTTP {r.status_code}", r.status_code == 429 or r.status_code >= 500
            if not retry or k == self.tries - 1:
                raise FetchError(f"{what}: {code}")
            self.sleep(min(60.0, 2.0 * 3 ** k))
        raise FetchError(f"{what}: no answer")


class Cache:
    """Files under one folder: cache.get(name, make, max_age_days) returns the cached bytes or makes, writes and
    returns them. Names are relative paths; they never hold secrets."""

    def __init__(self, root):
        self.root = Path(root)

    def path(self, name):
        p = (self.root / name).resolve()
        if self.root.resolve() not in p.parents:
            raise ValueError(f"cache name outside the cache: {name}")
        return p

    def fresh(self, name, max_age_days=None):
        p = self.path(name)
        return p.exists() and (max_age_days is None or time.time() - p.stat().st_mtime < max_age_days * 86400)

    def get(self, name, make, max_age_days=None):
        p = self.path(name)
        if self.fresh(name, max_age_days):
            return p.read_bytes()
        data = make()
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix(p.suffix + ".part")
        tmp.write_bytes(data)
        tmp.replace(p)
        return data

    def json(self, name, make, max_age_days=None):
        return json.loads(self.get(name, lambda: json.dumps(make()).encode(), max_age_days))


def key(*parts):
    return hashlib.sha1("|".join(map(str, parts)).encode()).hexdigest()[:16]


# ---------------------------------------------------------------- Mapbox vector tiles (no extra package)

def _varint(b, i):
    out = shift = 0
    while True:
        c = b[i]
        i += 1
        out |= (c & 0x7F) << shift
        if c < 0x80:
            return out, i
        shift += 7


def _fields(b):
    """(field number, wire type, value) of a protobuf message; length-delimited values as bytes."""
    i, n = 0, len(b)
    while i < n:
        tag, i = _varint(b, i)
        f, wt = tag >> 3, tag & 7
        if wt == 0:
            v, i = _varint(b, i)
        elif wt == 1:
            v, i = b[i:i + 8], i + 8
        elif wt == 2:
            ln, i = _varint(b, i)
            v, i = b[i:i + ln], i + ln
        elif wt == 5:
            v, i = b[i:i + 4], i + 4
        else:
            raise ValueError(f"protobuf wire type {wt}")
        yield f, wt, v


def _packed(b):
    i, out = 0, []
    while i < len(b):
        v, i = _varint(b, i)
        out.append(v)
    return out


def _zigzag(v):
    return (v >> 1) ^ -(v & 1)


def _value(b):
    import struct
    for f, wt, v in _fields(b):
        if f == 1:
            return bytes(v).decode("utf-8", "replace")
        if f == 2:
            return struct.unpack("<f", v)[0]
        if f == 3:
            return struct.unpack("<d", v)[0]
        if f in (4, 5):
            return v if v < 2 ** 63 else v - 2 ** 64
        if f == 6:
            return _zigzag(v)
        if f == 7:
            return bool(v)
    return None


def _geometry(cmds):
    """MVT geometry commands -> list of parts, each a list of (x, y) tile pixel coordinates."""
    parts, cur, x, y, i = [], None, 0, 0, 0
    while i < len(cmds):
        cmd, count = cmds[i] & 7, cmds[i] >> 3
        i += 1
        if cmd == 7:          # close path
            continue
        for _ in range(count):
            x += _zigzag(cmds[i])
            y += _zigzag(cmds[i + 1])
            i += 2
            if cmd == 1:      # move to: a new part
                cur = [(x, y)]
                parts.append(cur)
            else:
                cur.append((x, y))
    return parts


def decode_mvt(data):
    """A Mapbox vector tile -> {layer name: (extent, [{'id', 'type', 'parts', 'props'}])}."""
    out = {}
    for f, _, layer in _fields(memoryview(data)):
        if f != 3:
            continue
        name, extent, keys, vals, feats = "", 4096, [], [], []
        for lf, _, v in _fields(layer):
            if lf == 1:
                name = bytes(v).decode()
            elif lf == 5:
                extent = v
            elif lf == 3:
                keys.append(bytes(v).decode())
            elif lf == 4:
                vals.append(_value(v))
            elif lf == 2:
                feats.append(v)
        rows = []
        for fb in feats:
            fid, ftype, tags, geom = None, 0, [], []
            for ff, _, v in _fields(fb):
                if ff == 1:
                    fid = v
                elif ff == 2:
                    tags = _packed(v)
                elif ff == 3:
                    ftype = v
                elif ff == 4:
                    geom = _packed(v)
            props = {keys[tags[k]]: vals[tags[k + 1]] for k in range(0, len(tags) - 1, 2)}
            rows.append({"id": fid, "type": ftype, "parts": _geometry(geom), "props": props})
        out[name] = (extent, rows)
    return out


def tile_of(lat, lon, z):
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return x, y


def tile_lonlat(z, x, y, px, py, extent=4096):
    """(lat, lon) of pixel (px, py) of tile (z, x, y)."""
    n = 2 ** z
    lon = (x + px / extent) / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (y + py / extent) / n))))
    return lat, lon


def tiles_for_bbox(bbox, z=TILE_Z):
    s, w, n, e = bbox
    x0, y0 = tile_of(n, w, z)
    x1, y1 = tile_of(s, e, z)
    return [(z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]


# ---------------------------------------------------------------- Mapillary

def mapillary_token():
    from .settings import get_secret
    return get_secret("mapillary_token")[0]


class Mapillary:
    """Images and map features of a box. Tiles (the token in the query) are cached for 30 days, image details for
    good (their thumbnail URLs expire: url_for() asks again by id)."""

    def __init__(self, cache_dir, token=None, http=None, log=log):
        self.token = token or mapillary_token()
        if not self.token:
            raise FetchError("Mapillary: no token. " + TOKEN_HELP)
        self.cache = Cache(cache_dir)
        self.http = http or Http()
        self.log = log

    def tile(self, layer, z, x, y):
        name = f"mapillary/tiles/{layer}/{z}/{x}/{y}.pbf"
        return decode_mvt(self.cache.get(name, lambda: self.http.get(
            TILES.format(layer=layer, z=z, x=x, y=y), {"access_token": self.token},
            what=f"Mapillary tile {layer} {z}/{x}/{y}").content, max_age_days=30))

    def points(self, layer, bbox, z=TILE_Z):
        """Point features of a tile layer inside bbox: dicts of their properties with lat, lon (and 'layer')."""
        s, w, n, e = bbox
        out = {}
        for (zz, x, y) in tiles_for_bbox(bbox, z):
            for lname, (extent, rows) in self.tile(layer, zz, x, y).items():
                for r in rows:
                    if r["type"] != 1 or not r["parts"]:
                        continue
                    px, py = r["parts"][0][0]
                    lat, lon = tile_lonlat(zz, x, y, px, py, extent)
                    if s <= lat <= n and w <= lon <= e:
                        p = dict(r["props"], lat=lat, lon=lon, layer=lname)
                        out[str(p.get("id", r["id"]))] = p
        return list(out.values())

    def details(self, ids, fields=IMAGE_FIELDS, max_age_days=None):
        """Graph API records of image ids, cached one file per id (50 ids per request)."""
        out, todo = {}, []
        for i in ids:
            name = f"mapillary/images/{i}.json"
            if self.cache.fresh(name, max_age_days):
                out[str(i)] = json.loads(self.cache.path(name).read_bytes())
            else:
                todo.append(str(i))
        for k in range(0, len(todo), 50):
            chunk = todo[k:k + 50]
            res = self.http.get(GRAPH, {"ids": ",".join(chunk), "fields": fields},
                                {"Authorization": f"OAuth {self.token}"}, what="Mapillary images").json()
            for i, rec in res.items():
                if not isinstance(rec, dict) or "id" not in rec:
                    continue
                p = self.cache.path(f"mapillary/images/{i}.json")
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(json.dumps(rec))
                out[str(i)] = rec
        return out

    def url_for(self, image_id):
        """A fresh 2048 px thumbnail URL (the stored ones are signed and expire)."""
        res = self.http.get(f"{GRAPH}/{image_id}", {"fields": "thumb_2048_url"},
                            {"Authorization": f"OAuth {self.token}"}, what="Mapillary image").json()
        return res.get("thumb_2048_url")

    def images(self, bbox, spacing_m=8.0, max_images=3000):
        """Image records (realism.py's record format) in bbox: the tile points thinned to one per spacing_m per
        sequence (a sequence shot every metre does not cost a graph call per image), then their details."""
        pts = self.points("mly1_public", bbox)
        pts = [p for p in pts if p.get("layer") == "image"]
        kept, last = [], {}
        for p in sorted(pts, key=lambda p: (str(p.get("sequence_id")), p.get("captured_at") or 0)):
            seq = str(p.get("sequence_id"))
            q = last.get(seq)
            if q is not None and _metres(q["lat"], q["lon"], p["lat"], p["lon"]) < spacing_m:
                continue
            last[seq] = p
            kept.append(p)
        if len(kept) > max_images:      # the most recent first, spread over the box
            kept.sort(key=lambda p: -(p.get("captured_at") or 0))
            kept = kept[:max_images]
        self.log(f"mapillary: {len(pts)} images in the box, {len(kept)} after thinning to one per {spacing_m:.0f} m "
                 f"of a sequence")
        try:
            det = self.details([p["id"] for p in kept])
        except FetchError as err:
            det = {}
            self.log(f"mapillary: image details refused ({err})")
        if kept and not det:      # raised, so that realism.fetch does not cache an empty list
            raise FetchError(f"the graph API gave no details for any of {len(kept)} images (it answers 'does not "
                             f"exist, cannot be loaded due to missing permissions' when the token's application "
                             f"lacks read access): no Mapillary photos to compare; the tiles (coverage, map "
                             f"features) still work")
        recs = [mapillary_record(det[str(p["id"])]) for p in kept if str(p["id"]) in det]
        return [r for r in recs if r]

    def camera_points(self, bbox):
        """(lat, lon, day) of every image in the box (from the tiles; no graph calls): where photos were taken."""
        out = []
        for p in self.points("mly1_public", bbox):
            if p.get("layer") == "image":
                t = p.get("captured_at") or 0
                out.append((p["lat"], p["lon"], int(t // 86400000)))
        return out

    def features(self, bbox):
        """Map features (points and traffic signs) in bbox: dicts with id, value, lat, lon, first/last seen (ms)."""
        out = []
        for layer in ("mly_map_feature_point", "mly_map_feature_traffic_sign"):
            for p in self.points(layer, bbox):
                out.append({"id": str(p.get("id")), "value": p.get("value") or "", "lat": p["lat"], "lon": p["lon"],
                            "first_seen": p.get("first_seen_at"), "last_seen": p.get("last_seen_at"),
                            "layer": "traffic_sign" if "traffic_sign" in layer else "point"})
        return out


def _metres(lat1, lon1, lat2, lon2):
    k = 111320.0
    return math.hypot((lat2 - lat1) * k, (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2)))


def mapillary_fov(camera_type, params, width, height):
    """Horizontal field of view (deg) of a Mapillary perspective camera: camera_parameters[0] is the focal length
    as a fraction of the larger image side. None when unknown; 360 for spherical images."""
    if camera_type in ("spherical", "equirectangular"):
        return 360.0
    if not params or not width or not height or not params[0]:
        return None
    f = float(params[0]) * max(width, height)
    return math.degrees(2.0 * math.atan(width / (2.0 * f)))


def mapillary_record(x):
    """A graph API image -> realism.py record, or None (fisheye cameras and images without a heading)."""
    ctype = x.get("camera_type") or ("spherical" if x.get("is_pano") else "perspective")
    if ctype not in ("perspective", "spherical", "equirectangular"):
        return None        # fisheye (action cams on helmets and handlebars): no pinhole model
    head = x.get("computed_compass_angle")
    if head is None:
        head = x.get("compass_angle")
    geom = x.get("computed_geometry") or x.get("geometry")
    if head is None or not geom:
        return None
    lon, lat = geom["coordinates"][:2]
    w, h = x.get("width"), x.get("height")
    pano = ctype != "perspective"
    fov = mapillary_fov(ctype, x.get("camera_parameters"), w, h)
    t = x.get("captured_at")
    when = datetime.fromtimestamp(t / 1000.0, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ") if t else None
    creator = (x.get("creator") or {}).get("username")
    return {
        "source": "mapillary", "src_id": str(x["id"]), "lat": float(lat), "lon": float(lon),
        "heading": float(head) % 360.0, "pitch": 0.0, "roll": 0.0, "fov": fov,
        "projection": "equirect" if pano else "pinhole", "kind": "mly_pano" if pano else "mapillary",
        "width": w, "height": h, "time": when,
        "camera": " ".join(v for v in (x.get("make"), x.get("model")) if v) or None,
        "accuracy_m": None, "url": x.get("thumb_2048_url"), "url_sd": None,
        "page": f"https://www.mapillary.com/app/?pKey={x['id']}", "licence": "CC BY-SA 4.0",
        "author": creator, "sequence": str(x.get("sequence") or ""), "next": None,
    }


def fetch_mapillary(bbox, cache_dir, log=log):
    return Mapillary(cache_dir, log=log).images(bbox)


# ---------------------------------------------------------------- metrics beside realism.compare_labels

def dilate(mask, r):
    """Binary dilation by a (2r+1)^2 square (separable, numpy only)."""
    if r <= 0:
        return mask.copy()
    out = mask.copy()
    for s in range(1, r + 1):
        out[:, s:] |= mask[:, :-s]
        out[:, :-s] |= mask[:, s:]
    m2 = out.copy()
    for s in range(1, r + 1):
        out[s:] |= m2[:-s]
        out[:-s] |= m2[s:]
    return out


def boundaries(g, ignore=255):
    """Pixels where the label changes to the right or below (both sides labelled)."""
    b = np.zeros(g.shape, bool)
    ok = g != ignore
    dx = (g[:, 1:] != g[:, :-1]) & ok[:, 1:] & ok[:, :-1]
    dy = (g[1:] != g[:-1]) & ok[1:] & ok[:-1]
    b[:, 1:] |= dx
    b[1:] |= dy
    return b


def boundary_f(real, render, tol=None, ignore=255):
    """Boundary F-score (0..1) of two label maps: the share of each one's outlines within tol px (default 1 % of
    the width) of the other's, combined. It says whether the edges of road, buildings, trees and sky line up, which
    area overlaps (IoU) cannot see for thin strips. NaN when neither has outlines."""
    tol = max(1, int(round(0.01 * real.shape[1]))) if tol is None else int(tol)
    valid = (real != ignore) & (render != ignore)
    br, bd = boundaries(real, ignore) & valid, boundaries(render, ignore) & valid
    nr, nd = int(br.sum()), int(bd.sum())
    if nr == 0 and nd == 0:
        return float("nan")
    if nr == 0 or nd == 0:
        return 0.0
    precision = float((bd & dilate(br, tol)).sum()) / nd
    recall = float((br & dilate(bd, tol)).sum()) / nr
    return 0.0 if precision + recall == 0 else 2 * precision * recall / (precision + recall)


def _share(g, i, valid, rows=slice(None), cols=slice(None)):
    m = valid[rows, cols]
    n = int(m.sum())
    return float(((g[rows, cols] == i) & m).sum()) / n if n else 0.0


def differences(real, render, hfov, groups, top=4, ignore=255):
    """The main differences between a photo's and a render's group maps (realism.GROUPS), in words, largest first:
    [(size, text)]. Shares are of the labelled pixels of a third of the image (left, middle, right) or of its lower
    half (the street); the skyline is the elevation of the highest non-sky pixel per column."""
    from .realism import skyline
    gi = {name: i for i, name in enumerate(groups)}
    h, w = real.shape
    valid = (real != ignore) & (render != ignore)
    out = []
    thirds = (("on the left", slice(0, w // 3)), ("in the middle", slice(w // 3, 2 * w // 3)),
              ("on the right", slice(2 * w // 3, w)))
    for where, cols in thirds:
        for name, more, less, lim in (("building", "a building {w} where the photo has none",
                                       "missing building {w}", 0.12),
                                      ("vegetation", "trees or greenery {w} where the photo has none",
                                       "missing trees {w}", 0.10)):
            a, b = _share(real, gi[name], valid, cols=cols), _share(render, gi[name], valid, cols=cols)
            if b - a > lim:
                out.append((b - a, more.format(w=where) + f" (render {b:.0%}, photo {a:.0%})"))
            elif a - b > lim:
                out.append((a - b, less.format(w=where) + f" (photo {a:.0%}, render {b:.0%})"))
    low = slice(h // 2, h)
    a, b = _share(real, gi["road"], valid, rows=low), _share(render, gi["road"], valid, rows=low)
    if b - a > 0.12:
        out.append((b - a, f"road too wide (render {b:.0%} of the lower half, photo {a:.0%})"))
    elif a - b > 0.12:
        out.append((a - b, f"road too narrow (render {b:.0%} of the lower half, photo {a:.0%})"))
    a, b = _share(real, gi["sidewalk"], valid, rows=low), _share(render, gi["sidewalk"], valid, rows=low)
    if a - b > 0.08:
        out.append((a - b, f"pavement missing or too narrow (photo {a:.0%} of the lower half, render {b:.0%})"))
    elif b - a > 0.08:
        out.append((b - a, f"more pavement than the photo (render {b:.0%} of the lower half, photo {a:.0%})"))
    a, b = _share(real, gi["vehicle"], valid), _share(render, gi["vehicle"], valid)
    if a > 0.03 and b < a / 3:
        out.append((a - b, f"cars in the photo, few or none in the render (photo {a:.0%}, render {b:.0%})"))
    a, b = _share(real, gi["pole_sign"], valid), _share(render, gi["pole_sign"], valid)
    if a - b > 0.01 and a > 2 * b:
        out.append((5 * (a - b), f"poles, lamps or signs missing (photo {a:.1%}, render {b:.1%})"))
    sa, sb = skyline(real, hfov), skyline(render, hfov)
    both = ~np.isnan(sa) & ~np.isnan(sb)
    if both.sum() > 0.2 * w:
        d = float(np.mean(sa[both] - sb[both]))
        if d > 3.0:
            out.append((d / 25.0, f"skyline {d:.0f} deg too low: buildings or trees lower than in the photo, "
                                  f"or missing"))
        elif d < -3.0:
            out.append((-d / 25.0, f"skyline {-d:.0f} deg too high: buildings or trees taller than in the photo"))
    a, b = _share(real, gi["sky"], valid), _share(render, gi["sky"], valid)
    if b - a > 0.10 and not any("skyline" in t for _, t in out):
        out.append((b - a, f"more open sky than the photo (render {b:.0%}, photo {a:.0%})"))
    out.sort(key=lambda x: -x[0])
    return [(round(s, 3), t) for s, t in out[:top]]


class Perceptual:
    """DINOv2 features of two images of the same view (transformers, CPU or GPU): cosine similarity of the image
    tokens (dino_global: is it the same kind of scene?) and the mean cosine similarity of the patch tokens at the
    same places (dino_patch: do the same things appear in the same parts of the view?). Apache-2.0 weights."""

    def __init__(self, model="facebook/dinov2-base", device=None, width=448):
        import torch
        from transformers import AutoModel
        self.torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = AutoModel.from_pretrained(model).to(self.device).eval()
        self.width = width
        self.mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)

    def _tokens(self, rgbs):
        from PIL import Image
        torch = self.torch
        h0, w0 = rgbs[0].shape[:2]
        w = self.width // 14 * 14
        h = max(14, int(round(h0 * w / w0 / 14)) * 14)
        x = np.stack([np.asarray(Image.fromarray(a).resize((w, h), Image.BICUBIC)) for a in rgbs]).astype(np.float32)
        t = (torch.from_numpy(x).permute(0, 3, 1, 2) / 255.0 - self.mean) / self.std
        with torch.no_grad():
            hs = self.model(pixel_values=t.to(self.device)).last_hidden_state
        hs = torch.nn.functional.normalize(hs, dim=-1)
        return hs[:, 0], hs[:, 1:]

    def __call__(self, a, b):
        cls, patches = self._tokens([a, b])
        g = float((cls[0] * cls[1]).sum())
        p = float((patches[0] * patches[1]).sum(-1).mean())
        return {"dino_global": g, "dino_patch": p}


SPOT_PARTS = ("al_miou_coarse", "boundary_f", "dino_patch")


def spot_score(m):
    """One number per spot (0..1): the mean of the aligned coarse IoU (where things are), the boundary F-score
    (whether their edges line up) and the DINOv2 patch similarity (whether they look alike), the parts it has."""
    vals = []
    for k in SPOT_PARTS:
        try:
            v = float(m.get(k))
        except (TypeError, ValueError):
            continue
        if not math.isnan(v):
            vals.append(v)
    return float(np.mean(vals)) if vals else float("nan")


def map_score(rows):
    """Per-map summary: the median spot score x 100 and how many spots have each kind of difference."""
    scores = [r["spot_score"] for r in rows
              if isinstance(r.get("spot_score"), float) and not math.isnan(r["spot_score"])]
    kinds = {}
    for r in rows:
        for t in (r.get("differences") or "").split("; "):
            if t:
                k = t.split(" (")[0].replace(" on the left", "").replace(" on the right", "").replace(
                    " in the middle", "")
                if k.startswith("skyline"):
                    k = "skyline too low" if " too low" in k else "skyline too high"
                kinds[k] = kinds.get(k, 0) + 1
    return {"score": round(100 * float(np.median(scores)), 1) if scores else None, "spots": len(scores),
            "differences": dict(sorted(kinds.items(), key=lambda kv: -kv[1]))}


# ---------------------------------------------------------------- objects: photos' evidence against scenery.json

# Mapillary map feature values (prefix) -> object class
MLY_CLASSES = [("object--street-light", "lamp"), ("object--bench", "bench"), ("object--trash-can", "bin"),
               ("object--fire-hydrant", "hydrant"), ("object--bike-rack", "bike_rack"),
               ("object--mailbox", "post_box"), ("object--traffic-light--", "traffic_light"),
               ("object--support--utility-pole", "pole"), ("object--support--pole", "pole"),
               ("object--billboard", "billboard"), ("object--sign--advertisement", "billboard"),
               ("object--traffic-sign--front", "sign"), ("object--traffic-sign--back", "sign"),
               ("object--traffic-sign--direction-front", "sign"), ("object--traffic-sign--direction-back", "sign"),
               ("object--traffic-sign--information-parking", "sign")]
# OSM tags of scenery.py / meshes10.py furniture -> object class (their meshes are classed through these)
OSM_CLASS = {("amenity", "bench"): "bench", ("amenity", "waste_basket"): "bin",
             ("emergency", "fire_hydrant"): "hydrant",
             ("amenity", "bicycle_parking"): "bike_rack", ("amenity", "post_box"): "post_box",
             ("advertising", "column"): "billboard", ("advertising", "billboard"): "billboard",
             ("advertising", "board"): "billboard", ("advertising", "poster_box"): "billboard",
             ("barrier", "bollard"): "bollard"}
MATCH_M = {"lamp": 6.0, "sign": 5.0, "traffic_light": 6.0, "pole": 5.0}      # others: 4 m
ADDABLE = ("lamp", "bench", "bin", "hydrant", "bike_rack", "post_box")
REMOVABLE = ("lamp", "bench", "bin", "hydrant", "post_box")


def mly_class(value, layer="point"):
    if layer == "traffic_sign":
        return "sign"
    for prefix, cls in MLY_CLASSES:
        if value.startswith(prefix):
            return cls
    return None


def mesh_classes():
    """CARLA mesh name -> object class for both mesh catalogues' furniture."""
    from . import meshes10, scenery
    out = {}
    for table in (scenery.FURNITURE, meshes10.FURNITURE):
        for tag, meshes in table.items():
            cls = OSM_CLASS.get(tag)
            if cls:
                for m in meshes:
                    out[m] = cls
    return out


def scenery_objects(data, classes=None):
    """scenery.json -> [{'cls', 'x', 'y', 'items': [indices]}]: lamps, sign posts with their plates (one object), and
    furniture by its mesh. Removed items (None) are skipped."""
    classes = classes or mesh_classes()
    names = [m.rsplit(".", 1)[-1] for m in data["meshes"]]
    out, signs = [], []
    for i, it in enumerate(data["items"]):
        if it is None:
            continue
        kind = it[6]
        if kind == "lamp":
            out.append({"cls": "lamp", "x": it[1], "y": it[2], "items": [i]})
        elif kind == "sign":
            signs.append((it[1], it[2], i))
        elif kind == "furniture" and classes.get(names[it[0]]):
            out.append({"cls": classes[names[it[0]]], "x": it[1], "y": it[2], "items": [i]})
    for x, y, i in signs:      # a post and its plates stand within a metre of each other
        near = next((o for o in out[::-1] if o["cls"] == "sign" and math.hypot(o["x"] - x, o["y"] - y) < 1.0), None)
        if near:
            near["items"].append(i)
        else:
            out.append({"cls": "sign", "x": x, "y": y, "items": [i]})
    return out


def merge_points(points, radius):
    """Greedy merge of [{'x', 'y', ...}] within radius (first come first served); merged points get 'n'."""
    out = []
    grid = {}
    for p in points:
        cx, cy = int(p["x"] // radius), int(p["y"] // radius)
        hit = None
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for q in grid.get((cx + i, cy + j), ()):
                    if math.hypot(q["x"] - p["x"], q["y"] - p["y"]) < radius:
                        hit = q
                        break
        if hit is None:
            q = dict(p, n=1)
            out.append(q)
            grid.setdefault((cx, cy), []).append(q)
        else:
            hit["n"] += 1
            for k in ("first_seen", "last_seen"):
                if p.get(k) is not None:
                    hit[k] = (min if k == "first_seen" else max)(hit.get(k) or p[k], p[k])
    return out


def match(a, b, radius):
    """One-to-one nearest pairs between point lists a and b within radius: [(i, j, distance)], shortest first."""
    pairs = []
    bx = np.array([[q["x"], q["y"]] for q in b]) if b else np.zeros((0, 2))
    for i, p in enumerate(a):
        if not len(bx):
            break
        d = np.hypot(bx[:, 0] - p["x"], bx[:, 1] - p["y"])
        for j in np.where(d < radius)[0]:
            pairs.append((float(d[j]), i, int(j)))
    pairs.sort()
    ua, ub, out = set(), set(), []
    for d, i, j in pairs:
        if i in ua or j in ub:
            continue
        ua.add(i)
        ub.add(j)
        out.append((i, j, d))
    return out


class Observations:
    """Where the photos were taken: camera points (x, y, day) on a 10 m grid, to tell an item nobody photographed
    from one the photos show is absent."""

    def __init__(self, pts, cell=10.0):
        self.cell = cell
        self.grid = {}
        for x, y, day in pts:
            self.grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y, day))

    def near(self, x, y, r):
        k = int(math.ceil(r / self.cell))
        cx, cy = int(x // self.cell), int(y // self.cell)
        n, days = 0, set()
        for i in range(-k, k + 1):
            for j in range(-k, k + 1):
                for px, py, day in self.grid.get((cx + i, cy + j), ()):
                    if math.hypot(px - x, py - y) <= r:
                        n += 1
                        days.add(day)
        return n, len(days)


def compare_objects(evidence, objects, obs, protect=(), seen_r=20.0, min_images=20, min_days=3, move_min=1.5,
                    remove_r=10.0, min_span_days=1.0, add_clear=2.0, move_max=None):
    """Evidence objects [{'cls', 'x', 'y', 'n', 'first_seen', 'last_seen', 'id'}] against scenery objects.
    Returns (per-class stats, proposed corrections). A correction is strong enough when:
    - move: a matched pair is more than move_min apart and the evidence was seen on two dates or more;
    - add: evidence with no scenery object of its class within twice the match radius, seen on two dates or more;
    - remove: a scenery object with no evidence of its class within remove_r although at least min_images photos on
      min_days days were taken within seen_r of it.
    Classes in `protect` (official registers: surveyed positions) are reported but never corrected."""
    stats, corr = {}, []
    classes = sorted({o["cls"] for o in objects} | {e["cls"] for e in evidence})
    for cls in classes:
        ev = [e for e in evidence if e["cls"] == cls]
        sc = [o for o in objects if o["cls"] == cls]
        r = MATCH_M.get(cls, 4.0)
        observed = [k for k, o in enumerate(sc) if obs.near(o["x"], o["y"], seen_r)[0] >= 1]
        pairs = match(ev, sc, r)
        offs = [d for _, _, d in pairs]
        st = {"evidence": len(ev), "scenery": len(sc), "scenery_photographed": len(observed),
              "matched": len(pairs), "median_offset_m": round(float(np.median(offs)), 2) if offs else None,
              "protected": cls in protect}
        st["recall"] = round(len(pairs) / len(observed), 3) if observed else None
        st["precision"] = round(len(pairs) / len(ev), 3) if ev else None
        stats[cls] = st
        if cls in protect:
            continue
        two_dates = (lambda e: e.get("first_seen") is not None and e.get("last_seen") is not None and
                     abs(e["last_seen"] - e["first_seen"]) > min_span_days * 86400000)
        for i, j, d in pairs:
            if d > move_min and (move_max is None or d <= move_max) and two_dates(ev[i]):
                corr.append({"action": "move", "cls": cls, "items": sc[j]["items"],
                             "from": [round(sc[j]["x"], 2), round(sc[j]["y"], 2)],
                             "to": [round(ev[i]["x"], 2), round(ev[i]["y"], 2)], "m": round(d, 2),
                             "evidence": ev[i].get("id")})
        matched_ev = {i for i, _, _ in pairs}
        if cls in ADDABLE:
            sx = np.array([[o["x"], o["y"]] for o in sc]) if sc else np.zeros((0, 2))
            for i, e in enumerate(ev):
                if i in matched_ev or not two_dates(e):
                    continue
                if len(sx) and np.min(np.hypot(sx[:, 0] - e["x"], sx[:, 1] - e["y"])) < add_clear * r:
                    continue
                corr.append({"action": "add", "cls": cls, "to": [round(e["x"], 2), round(e["y"], 2)],
                             "evidence": e.get("id")})
        if cls in REMOVABLE:
            ex = np.array([[e["x"], e["y"]] for e in ev]) if ev else np.zeros((0, 2))
            matched_sc = {j for _, j, _ in pairs}
            for j, o in enumerate(sc):
                if j in matched_sc:
                    continue
                if len(ex) and np.min(np.hypot(ex[:, 0] - o["x"], ex[:, 1] - o["y"])) < remove_r:
                    continue
                n, days = obs.near(o["x"], o["y"], seen_r)
                if n >= min_images and days >= min_days:
                    corr.append({"action": "remove", "cls": cls, "items": o["items"],
                                 "from": [round(o["x"], 2), round(o["y"], 2)], "photos": n, "days": days})
    return stats, corr


def summarise_corrections(corr):
    out = {}
    for c in corr:
        out.setdefault(c["cls"], {}).setdefault(c["action"], 0)
        out[c["cls"]][c["action"]] += 1
    return out


def apply_corrections(data, corr, actions=("move", "add"), ground=None, on_lane=None, log=log):
    """Apply corrections to scenery.json data (in place; returns counts). Removed items become None (the season's
    leaf list keeps its indices). An added object copies the mesh, yaw and height above ground of the nearest
    scenery object of its class; ground(x, y) gives the terrain height (else that object's), on_lane(x, y) True
    refuses a spot on a driving lane."""
    objs = None
    counts = {"move": 0, "add": 0, "remove": 0, "refused_on_lane": 0}
    for c in corr:
        a = c["action"]
        if a not in actions:
            continue
        if a == "move":
            dx, dy = c["to"][0] - c["from"][0], c["to"][1] - c["from"][1]
            if on_lane is not None and on_lane(*c["to"]):
                counts["refused_on_lane"] += 1
                continue
            for i in c["items"]:
                it = data["items"][i]
                if it is None:
                    continue
                it[1], it[2] = round(it[1] + dx, 2), round(it[2] + dy, 2)
                if ground is not None:
                    it[3] = round(it[3] + ground(it[1], it[2]) - ground(it[1] - dx, it[2] - dy), 2)
                if len(it) > 8:
                    it[8] = [round(it[8][0] + dx, 1), round(it[8][1] + dy, 1)]
            counts["move"] += 1
        elif a == "remove":
            for i in c["items"]:
                data["items"][i] = None
            counts["remove"] += 1
        elif a == "add":
            if on_lane is not None and on_lane(*c["to"]):
                counts["refused_on_lane"] += 1
                continue
            if objs is None:
                objs = scenery_objects(data)
            same = [o for o in objs if o["cls"] == c["cls"]]
            if not same:
                continue
            x, y = c["to"]
            o = min(same, key=lambda o: math.hypot(o["x"] - x, o["y"] - y))
            t = data["items"][o["items"][0]]
            z = t[3] if ground is None else round(t[3] - ground(t[1], t[2]) + ground(x, y), 2)
            data["items"].append([t[0], round(x, 2), round(y, 2), z, t[4], t[5], t[6]])
            counts["add"] += 1
    log(f"photo corrections applied: {counts}")
    return counts


# ---------------------------------------------------------------- poles in panoramas (the open evidence)

CS_POLE, CS_LIGHT, CS_SIGN = 5, 6, 7
CS_GROUND = (0, 1, 9)        # road, sidewalk, terrain: a pole whose foot stands on these is not hidden


def pano_poles(lab, cam_h, min_px=None):
    """Poles in an equirectangular Cityscapes label map (H x 2H): [{'az', 'dist', 'height', 'sign', 'col'}] with
    az the angle from the panorama's centre column (deg, right positive), dist the ground distance from the foot's
    angle below the horizon at camera height cam_h, height from the top's angle (None when the pole leaves the image
    at the top) and sign True when a traffic sign touches it. Only poles whose foot is seen on the ground."""
    h, w = lab.shape
    min_px = min_px or max(8, h // 40)
    pole = (lab == CS_POLE) | (lab == CS_LIGHT)
    cols = []
    for c in range(w):
        ys = np.where(pole[:, c])[0]
        if len(ys) < min_px:
            cols.append(None)
            continue
        # the longest vertical run in this column
        br = np.where(np.diff(ys) > 2)[0]
        starts = np.concatenate([[0], br + 1])
        ends = np.concatenate([br, [len(ys) - 1]])
        k = int(np.argmax(ys[ends] - ys[starts]))
        top, bot = int(ys[starts[k]]), int(ys[ends[k]])
        cols.append((top, bot) if bot - top + 1 >= min_px else None)
    out, c = [], 0
    while c < w:
        if cols[c] is None:
            c += 1
            continue
        c0 = c
        while c < w and cols[c] is not None:
            c += 1
        seg = cols[c0:c]
        if c - c0 > w // 40:          # wider than 9 deg: a wall of poles or a mislabel, not one pole
            continue
        top = min(t for t, _ in seg)
        bot = max(b for _, b in seg)
        mid = (c0 + c - 1) / 2.0
        below = lab[min(h - 1, bot + 2):min(h, bot + 6), c0:c]
        if below.size == 0 or not np.isin(below, CS_GROUND).mean() > 0.5:
            continue      # its foot is hidden (a car, a person, the image edge)
        el_bot = 90.0 - (bot + 1.0) / h * 180.0
        if el_bot > -2.0:
            continue
        dist = cam_h / math.tan(math.radians(-el_bot))
        el_top = 90.0 - top / h * 180.0
        height = None if top <= 1 else cam_h + dist * math.tan(math.radians(el_top))
        win = lab[max(0, top - 10):bot, max(0, c0 - 6):min(w, c + 6)]
        out.append({"az": (mid + 0.5) / w * 360.0 - 180.0, "dist": dist,
                    "height": None if height is None else float(height), "sign": bool((win == CS_SIGN).any()),
                    "col": mid})
    return out


def pole_class(p):
    if p["sign"] and (p["height"] is None or p["height"] < 5.0):
        return "sign"
    if p["height"] is None or p["height"] >= 5.0:
        return "lamp"
    return "pole"


def offset_xy(x, y, heading_deg, az_deg, dist):
    """CARLA (x, y) of a point dist metres away from (x, y) at compass bearing heading+az (CARLA: x east, y south)."""
    b = math.radians(heading_deg + az_deg)
    return x + dist * math.sin(b), y - dist * math.cos(b)


def time_ms(text):
    """ISO time -> ms since 1970 (UTC; naive times taken as UTC), None if unreadable."""
    if not text:
        return None
    try:
        t = datetime.fromisoformat(str(text).replace("Z", "+00:00"))
    except ValueError:
        return None
    if t.tzinfo is None:
        t = t.replace(tzinfo=timezone.utc)
    return int(t.timestamp() * 1000)


def pano_evidence(panos, geo, labels, cam_h=2.1, max_dist=20.0, merge_m=2.5, min_views=2, log=log):
    """Poles seen in panoramas -> evidence objects. panos: realism records (equirect) with lat, lon, heading, time,
    src_id; labels(record) gives its 2048 x 1024 Cityscapes labels (or None). A pole counts when at least min_views
    panoramas put it within merge_m of the same spot. Returns (evidence, cameras)."""
    dets, cams = [], []
    for k, r in enumerate(panos):
        lab = labels(r)
        if lab is None:
            continue
        x, y = geo.to_carla(r["lat"], r["lon"])
        t = time_ms(r.get("time"))
        cams.append((x, y, int((t or 0) // 86400000)))
        for p in pano_poles(lab, cam_h):
            if p["dist"] > max_dist:
                continue
            px, py = offset_xy(x, y, r["heading"], p["az"], p["dist"])
            dets.append({"cls": pole_class(p), "x": px, "y": py, "first_seen": t, "last_seen": t,
                         "id": f"{r['src_id']}@{p['az']:.0f}", "pano": r["src_id"], "dist": round(p["dist"], 1)})
        if (k + 1) % 100 == 0:
            log(f"{k + 1}/{len(panos)} panoramas searched for poles ({len(dets)} sightings)")
    ev = []
    for cls in sorted({d["cls"] for d in dets}):
        # nearer sightings first: their positions are better (the foot's angle shrinks with distance)
        cl = merge_points(sorted([d for d in dets if d["cls"] == cls], key=lambda d: d["dist"]), merge_m)
        ev += [dict(c, n_views=c["n"]) for c in cl if c["n"] >= min_views]
    by = {}
    for e in ev:
        by[e["cls"]] = by.get(e["cls"], 0) + 1
    log(f"panoramas: {len(dets)} pole sightings in {len(cams)} panoramas -> {len(ev)} objects seen in "
        f"{min_views}+ panoramas (" + ", ".join(f"{c} {n}" for c, n in sorted(by.items())) + ")")
    return ev, cams


def mapillary_evidence(mly, bbox, geo, log=log):
    """Mapillary map features in bbox -> evidence objects in CARLA coordinates (signs seen from the front and the back
    merged within 1.5 m), and the camera points of all its images."""
    ev = []
    feats = mly.features(bbox)
    for f in feats:
        cls = mly_class(f["value"], f["layer"])
        if not cls:
            continue
        x, y = geo.to_carla(f["lat"], f["lon"])
        ev.append({"cls": cls, "x": x, "y": y, "id": f["id"], "value": f["value"], "first_seen": f["first_seen"],
                   "last_seen": f["last_seen"]})
    merged = []
    for cls in sorted({e["cls"] for e in ev}):
        merged += merge_points([e for e in ev if e["cls"] == cls], 1.5)
    cams = []
    for lat, lon, day in mly.camera_points(bbox):
        x, y = geo.to_carla(lat, lon)
        cams.append((x, y, day))
    log(f"mapillary: {len(feats)} map features, {len(merged)} street objects of the compared classes; "
        f"{len(cams)} camera points")
    return merged, cams


def protected_classes(mdir):
    """Classes whose scenery positions come from an official register (surveyed, better than any photo estimate):
    lamps from a city's lighting register (real/municipal.json), signs from the Dutch NDW register."""
    mdir = Path(mdir)
    out = set()
    try:
        mun = json.loads((mdir / "real" / "municipal.json").read_text())
        if any((e.get("tags") or {}).get("highway") == "street_lamp" for e in mun.get("elements", [])):
            out.add("lamp")
    except (OSError, ValueError):
        pass
    try:
        meta = json.loads((mdir / "meta.json").read_text())
        if (meta.get("scenery") or {}).get("sign_ndw_real") or (meta.get("scenery") or {}).get("sign_ndw"):
            out.add("sign")
    except (OSError, ValueError):
        pass
    return out


def evidence_path(mdir, source):
    return Path(mdir) / "real" / "streetphotos" / f"evidence_{source}.json"


def save_evidence(mdir, source, evidence, cams, extra=None):
    p = evidence_path(mdir, source)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"source": source, "made": datetime.now().strftime("%Y-%m-%d %H:%M"),
                             "evidence": evidence, "cameras": [[round(x, 1), round(y, 1), d] for x, y, d in cams],
                             **(extra or {})}))
    return p


def correct_map(mdir, sources=("mapillary",), actions=("add", "remove"), protect=None, apply=False, on_lane=None,
                ground=None, meta=None, log=log):
    """The correction pass: the saved evidence of each source (evidence_<source>.json) against scenery.json.
    Returns {source: {'stats', 'corrections', 'summary'}}; with apply=True the corrections of `actions` are written
    into scenery.json (the old one kept as scenery.before_photos.json) and the scenery counts of `meta` (a dict the
    caller writes) or else of meta.json."""
    mdir = Path(mdir)
    path = mdir / "scenery.json"
    data = json.loads(path.read_text())
    protect = protected_classes(mdir) if protect is None else set(protect)
    res = {}
    for src in sources:
        p = evidence_path(mdir, src)
        if not p.exists():
            log(f"{src}: no evidence saved ({p})")
            continue
        ev = json.loads(p.read_text())
        objs = scenery_objects(data)
        stats, corr = compare_objects(ev["evidence"], objs, Observations([tuple(c) for c in ev["cameras"]]), protect)
        res[src] = {"stats": stats, "corrections": corr, "summary": summarise_corrections(corr),
                    "protected": sorted(protect)}
        if apply:
            backup = mdir / "scenery.before_photos.json"
            if not backup.exists():
                backup.write_text(path.read_text())
            counts = apply_corrections(data, corr, actions, ground=ground, on_lane=on_lane, log=log)
            res[src]["applied"] = counts
            tmp = path.with_suffix(".json.part")
            tmp.write_text(json.dumps(data, separators=(",", ":")))
            tmp.replace(path)
            if meta is not None:
                meta.setdefault("scenery", {})[f"photo_corrections_{src}"] = counts
                continue
            try:
                m = json.loads((mdir / "meta.json").read_text())
                m.setdefault("scenery", {})[f"photo_corrections_{src}"] = counts
                (mdir / "meta.json").write_text(json.dumps(m, indent=2))
            except (OSError, ValueError):
                pass
    return res


def validate_corrections(truth, evidence, obs, cls="lamp", shift_m=5.0, drop=0.3, extra=0.3, seed=1, **kw):
    """How well the correction rules recover a register's positions (truth: scenery objects of one class from an
    official register, e.g. Eindhoven's lamps) from the evidence: the register is spoiled the way guessed or OSM
    positions are (each object shifted by up to shift_m, a share `drop` left out, a share `extra` of made-up objects
    added 10-30 m from any real one), the corrections are applied, and the result is compared with the register.
    Returns error and count figures before and after."""
    rng = np.random.default_rng(seed)
    tr = [o for o in truth if o["cls"] == cls]
    txy = np.array([[o["x"], o["y"]] for o in tr])
    keep = rng.random(len(tr)) >= drop
    spoilt = []
    for k in np.where(keep)[0]:
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0, shift_m)
        spoilt.append({"cls": cls, "x": txy[k, 0] + d * math.cos(a), "y": txy[k, 1] + d * math.sin(a),
                       "items": [len(spoilt)], "truth": int(k)})
    n_fake = int(extra * len(tr))
    tries = 0
    while n_fake and tries < 50 * n_fake:
        tries += 1
        k = rng.integers(len(tr))
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(10, 30)
        x, y = txy[k, 0] + d * math.cos(a), txy[k, 1] + d * math.sin(a)
        if np.min(np.hypot(txy[:, 0] - x, txy[:, 1] - y)) > 10:
            spoilt.append({"cls": cls, "x": x, "y": y, "items": [len(spoilt)], "truth": None})
            n_fake -= 1
    _, corr = compare_objects([e for e in evidence if e["cls"] == cls], spoilt, obs, (), **kw)

    def score(objs):
        """mean distance of matched objects to the register, objects with no register object within 4 m (wrong),
        register objects with no object within 4 m (missing)"""
        oxy = np.array([[o["x"], o["y"]] for o in objs]) if objs else np.zeros((0, 2))
        pairs = match([{"x": x, "y": y} for x, y in oxy], [{"x": x, "y": y} for x, y in txy], 4.0)
        return {"objects": len(objs), "within_4m": len(pairs),
                "mean_error_m": round(float(np.mean([d for _, _, d in pairs])), 2) if pairs else None,
                "wrong": len(objs) - len(pairs), "missing": len(tr) - len(pairs)}
    before = score(spoilt)
    after_objs = [dict(o) for o in spoilt]
    gone = set()
    for c in corr:
        if c["action"] == "move":
            o = after_objs[c["items"][0]]
            o["x"], o["y"] = c["to"]
        elif c["action"] == "remove":
            gone.add(c["items"][0])
    added = [{"cls": cls, "x": c["to"][0], "y": c["to"][1]} for c in corr if c["action"] == "add"]
    moved_only = score([o for o in after_objs])
    after = score([o for i, o in enumerate(after_objs) if i not in gone] + added)
    # each action on its own: did it help?
    moves = [c for c in corr if c["action"] == "move"]
    better = 0
    for c in moves:
        t = spoilt[c["items"][0]]["truth"]
        if t is not None and math.hypot(c["to"][0] - txy[t, 0], c["to"][1] - txy[t, 1]) < \
                math.hypot(c["from"][0] - txy[t, 0], c["from"][1] - txy[t, 1]):
            better += 1
    rem = [c for c in corr if c["action"] == "remove"]
    rem_fake = sum(1 for c in rem if spoilt[c["items"][0]]["truth"] is None)
    add_real = sum(1 for a in added if np.min(np.hypot(txy[:, 0] - a["x"], txy[:, 1] - a["y"])) < 4.0)
    return {"class": cls, "register": len(tr), "shift_m": shift_m, "drop": drop, "extra": extra,
            "before": before, "after_moves": moved_only, "after": after,
            "moves": len(moves), "moves_closer": better, "removes": len(rem), "removes_of_made_up": rem_fake,
            "adds": len(added), "adds_at_a_register_object": add_real}
