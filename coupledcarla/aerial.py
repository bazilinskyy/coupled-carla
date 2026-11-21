"""Open aerial photos (orthophotos) of a small box, for the realism pilot's top-down check (tools/realism_score.py).

Only open services, each with its licence; the photos are comparison targets and are not shipped:

| country | service | licence |
|---|---|---|
| NL | PDOK Luchtfoto RGB (Beeldmateriaal Nederland), 8 cm; also earlier years | CC BY 4.0 |
| ES | IGN PNOA máxima actualidad | CC BY 4.0 (scne.es) |
| FR | IGN Géoplateforme ORTHOIMAGERY.ORTHOPHOTOS | Licence Ouverte 2.0 (Etalab) |
| DE-NW | Geobasis NRW DOP (wms_nw_dop) | dl-de/zero-2-0 |
| JP | GSI seamless photo tiles (seamlessphoto) | GSI terms of use (credit: 国土地理院) |

Google's Photorealistic 3D Tiles and other commercial imagery are never used here.
"""
import io
import math
from pathlib import Path

import numpy as np

WMS = {
    "NL": ("https://service.pdok.nl/hwh/luchtfotorgb/wms/v1_0", "Actueel_orthoHR"),
    "NL-2023": ("https://service.pdok.nl/hwh/luchtfotorgb/wms/v1_0", "2023_orthoHR"),
    "ES": ("https://www.ign.es/wms-inspire/pnoa-ma", "OI.OrthoimageCoverage"),
    "FR": ("https://data.geopf.fr/wms-r/wms", "ORTHOIMAGERY.ORTHOPHOTOS"),
    "DE-NW": ("https://www.wms.nrw.de/geobasis/wms_nw_dop", "nw_dop_rgb"),
}
GSI = "https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg"
CREDIT = {"NL": "Beeldmateriaal Nederland / PDOK (CC BY 4.0)", "NL-2023": "Beeldmateriaal Nederland / PDOK (CC BY 4.0)",
          "ES": "PNOA cedido por © Instituto Geográfico Nacional (CC BY 4.0)",
          "FR": "IGN Géoplateforme (Licence Ouverte 2.0)", "DE-NW": "Geobasis NRW (dl-de/zero-2-0)",
          "JP": "国土地理院 (GSI seamless photo)"}
# an earlier photo of the same place: "same place" baseline of the top-down check (NL only for now)
PREVIOUS = {"NL": "NL-2023"}


def available(country):
    return country in WMS or country == "JP"


def _get(url, params=None, timeout=60, tries=3):
    import time

    import requests
    err = None
    for k in range(tries):
        try:
            r = requests.get(url, params=params, timeout=timeout, headers={"User-Agent": "coupled-carla realism pilot"})
            if r.status_code == 200 and r.headers.get("content-type", "").startswith("image"):
                return r.content
            err = RuntimeError(f"HTTP {r.status_code} {r.headers.get('content-type')}")
        except Exception as e:  # noqa: BLE001
            err = e
        time.sleep(2 * (k + 1))
    raise RuntimeError(f"aerial photo not fetched: {err}")


def _decode(data):
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(data)).convert("RGB"))


def _wms(country, s, w, n, e, size):
    url, layer = WMS[country]
    p = {"SERVICE": "WMS", "VERSION": "1.3.0", "REQUEST": "GetMap", "LAYERS": layer, "STYLES": "",
         "CRS": "EPSG:4326", "BBOX": f"{s},{w},{n},{e}", "WIDTH": size, "HEIGHT": size, "FORMAT": "image/jpeg"}
    return _decode(_get(url, p))


def _tile_xy(lat, lon, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def _xyz(s, w, n, e, size, z=18):
    from PIL import Image
    x0, y0 = _tile_xy(n, w, z)
    x1, y1 = _tile_xy(s, e, z)
    tx0, ty0, tx1, ty1 = int(x0), int(y0), int(x1), int(y1)
    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            img = Image.open(io.BytesIO(_get(GSI.format(z=z, x=tx, y=ty)))).convert("RGB")
            mosaic.paste(img, ((tx - tx0) * 256, (ty - ty0) * 256))
    box = ((x0 - tx0) * 256, (y0 - ty0) * 256, (x1 - tx0) * 256, (y1 - ty0) * 256)
    return np.asarray(mosaic.crop(tuple(int(round(v)) for v in box)).resize((size, size), Image.BICUBIC))


def ortho(country, s, w, n, e, size=512, cache=None):
    """RGB array (size x size, north up) of the open aerial photo of the box; cached as JPEG when cache is a path."""
    from PIL import Image
    if cache is not None and Path(cache).exists():
        return np.asarray(Image.open(cache).convert("RGB"))
    if country == "JP":
        img = _xyz(s, w, n, e, size)
    elif country in WMS:
        img = _wms(country, s, w, n, e, size)
    else:
        raise ValueError(f"no open aerial photo for {country}")
    if cache is not None:
        Path(cache).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(img).save(cache, quality=92)
    return img


# ---------------------------------------------------------------- top-down metrics

def vegetation(rgb, thresh=0.04):
    """Vegetation mask from the excess-green index on chromaticity (2g - r - b), dark pixels (shadows) left out."""
    a = rgb.astype(np.float32) + 1.0
    s = a.sum(-1)
    r, g, b = a[..., 0] / s, a[..., 1] / s, a[..., 2] / s
    return ((2 * g - r - b) > thresh) & (s > 3 * 40)


def _cells(mask, cell):
    h, w = mask.shape
    h2, w2 = h // cell * cell, w // cell * cell
    return mask[:h2, :w2].reshape(h2 // cell, cell, w2 // cell, cell).mean((1, 3)) > 0.3


def veg_iou(a, b, cell=5):
    """IoU of the vegetation cover of two top-down images, on cells of `cell` pixels (5 px ~ 2 m at 512 px/200 m)."""
    va, vb = _cells(vegetation(a), cell), _cells(vegetation(b), cell)
    union = (va | vb).sum()
    return float((va & vb).sum() / union) if union else 1.0


def veg_share(a):
    return float(vegetation(a).mean())


def topdown_metrics(real, render, perc=None):
    from .realism import colour_hist_sim, image_stats
    from PIL import Image
    if render.shape != real.shape:
        render = np.asarray(Image.fromarray(render).resize((real.shape[1], real.shape[0]), Image.BICUBIC))
    m = {"veg_iou": round(veg_iou(real, render), 4), "veg_share_real": round(veg_share(real), 4),
         "veg_share_render": round(veg_share(render), 4),
         "colour_hist_sim": round(float(colour_hist_sim(real, render)), 4)}
    sa, sb = image_stats(real), image_stats(render)
    for k in ("lum_mean", "sat_mean", "edge_density"):
        if k in sa and k in sb:
            m[k + "_diff"] = round(float(sb[k] - sa[k]), 4)
    va = vegetation(real)
    if va.mean() > 0.01:        # where the photo is green: is the render green there too (lawns, crowns)?
        m["veg_recall"] = round(float(vegetation(render)[va].mean()), 4)
        rr = render[va].astype(np.float32).mean(0)
        pr = real[va].astype(np.float32).mean(0)
        m["green_excess_render"] = round(float(2 * rr[1] - rr[0] - rr[2]), 2)   # 2G - R - B of the mean colour
        m["green_excess_photo"] = round(float(2 * pr[1] - pr[0] - pr[2]), 2)
    if perc is not None:
        m.update({k: round(v, 4) for k, v in perc(real, render).items()})
    return m
