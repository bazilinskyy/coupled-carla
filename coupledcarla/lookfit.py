"""A camera look for CARLA 1.0 fitted to real street photos (tools/look_fit.py; README "How close is the twin").

CARLA 1.0's cameras get their look from a post-process profile (Content/Carla/Config/PostProcess/*.json, Unreal's
FPostProcessSettings): exposure bias, local exposure, white balance, colour grading and the filmic tone curve. The
realism check (tools/realism_check.py) found the twin's renders brighter, flatter, bluer and paler than the real
photos of the same views (Dashcam profile: luminance spread 35 against 62, mean 120 against 98). This module models
the part of Unreal's tone mapping those settings control, per pixel, so that a profile can be fitted to paired
real and rendered views without a GPU:

  scene colour -> local exposure -> white balance -> colour correction (saturation, contrast, gamma, gain)
               -> filmic tone curve (slope, toe, shoulder, black and white clip) -> sRGB

after Unreal 5's PostProcessCombineLUTs.usf / TonemapCommon.ush (FilmToneMap with its pre- and post-desaturation,
ColorCorrect around mid grey 0.18, a Bradford white balance from the daylight locus) and the local exposure of
PostProcessLocalExposure (log luminance of a blurred base scaled by the highlight or shadow contrast scale, detail
kept). Simplifications: sRGB primaries stand in for ACEScg (AP1), the bilateral base is a blend of two Gaussian
blurs, and bloom, vignette, film grain, colour fringes and sharpening are left out (the fitted profile turns the last
four off). A render is first taken back to scene colour through the profile it was rendered with (invert()), then
forward() renders it with candidate settings; fit() searches the settings that bring the renders' per-class colour
statistics (road, sidewalk, building, vegetation, sky: the segmentation of both images) to the photos'.
"""
import json
import math
from pathlib import Path

import numpy as np

DATA = Path(__file__).parent / "data" / "postprocess"
LUMA = np.array([0.2126, 0.7152, 0.0722])

# The settings fitted (name, Unreal FPostProcessSettings key, lower bound, upper bound)
PARAMS = [
    ("ev", "autoExposureBias", -2.5, 1.5),
    ("temp", "whiteTemp", 5000.0, 9000.0),
    ("sat", "colorSaturation", 0.6, 1.6),
    ("contrast", "colorContrast", 0.7, 1.5),
    ("gamma", "colorGamma", 0.6, 1.5),
    ("slope", "filmSlope", 0.6, 1.3),
    ("toe", "filmToe", 0.1, 0.8),
    ("local_hi", "localExposureHighlightContrastScale", 0.4, 1.0),
    ("local_sh", "localExposureShadowContrastScale", 0.4, 1.0),
]
# Unreal's defaults for settings a profile does not override
UE_DEFAULTS = {"ev": 0.0, "temp": 6500.0, "tint": 0.0, "sat": 1.0, "contrast": 1.0, "gamma": 1.0, "gain": 1.0,
               "slope": 0.88, "toe": 0.55, "shoulder": 0.26, "black": 0.0, "white": 0.04, "local_hi": 1.0,
               "local_sh": 1.0, "sat_shadows": 1.0, "shadows_max": 0.09}
_KEYS = {"ev": "AutoExposureBias", "temp": "WhiteTemp", "tint": "WhiteTint", "sat": "ColorSaturation",
         "contrast": "ColorContrast", "gamma": "ColorGamma", "gain": "ColorGain", "slope": "FilmSlope",
         "toe": "FilmToe", "shoulder": "FilmShoulder", "black": "FilmBlackClip", "white": "FilmWhiteClip",
         "local_hi": "LocalExposureHighlightContrastScale", "local_sh": "LocalExposureShadowContrastScale",
         "sat_shadows": "ColorSaturationShadows", "shadows_max": "ColorCorrectionShadowsMax"}


def _lower(k):
    return k[0].lower() + k[1:]


def profile_params(settings):
    """FPostProcessSettings (a profile's "settings") -> the model's parameters (Unreal's defaults where a value is
    not overridden). Vector settings (saturation, contrast, ...) use their mean of x, y, z times w."""
    out = dict(UE_DEFAULTS)
    for name, key in _KEYS.items():
        if not settings.get("bOverride_" + key, False):
            continue
        v = settings.get(_lower(key))
        if isinstance(v, dict):
            v = float(np.mean([v.get("x", 1.0), v.get("y", 1.0), v.get("z", 1.0)])) * float(v.get("w", 1.0))
        if v is not None:
            out[name] = float(v)
    return out


def load_profile(name_or_path, search=(DATA,)):
    p = Path(name_or_path)
    if not p.suffix:
        for d in search:
            if (Path(d) / f"{name_or_path}.json").exists():
                p = Path(d) / f"{name_or_path}.json"
                break
    return json.loads(p.read_text())


# ---------------------------------------------------------------- the pieces of the tone mapping

def srgb_encode(x):
    x = np.clip(x, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def srgb_decode(v):
    v = np.clip(v, 0.0, 1.0)
    return np.where(v <= 0.04045, v / 12.92, np.power((v + 0.055) / 1.055, 2.4))


def film_curve(x, slope=0.88, toe=0.55, shoulder=0.26, black=0.0, white=0.04):
    """Unreal's filmic tone curve (FilmToneMap) on linear values (any shape)."""
    toe_scale = 1.0 + black - toe
    sh_scale = 1.0 + white - shoulder
    in_m, out_m = 0.18, 0.18
    if toe > 0.8:
        toe_match = (1.0 - toe - out_m) / slope + math.log10(in_m)
    else:
        bt = (out_m + black) / toe_scale - 1.0
        toe_match = math.log10(in_m) - 0.5 * math.log((1.0 + bt) / (1.0 - bt)) * (toe_scale / slope)
    straight_match = (1.0 - toe) / slope - toe_match
    shoulder_match = shoulder / slope - straight_match
    lc = np.log10(np.maximum(x, 1e-10))
    straight = slope * (lc + straight_match)
    with np.errstate(over="ignore"):
        toe_c = -black + (2.0 * toe_scale) / (1.0 + np.exp((-2.0 * slope / toe_scale) * (lc - toe_match)))
        sh_c = (1.0 + white) - (2.0 * sh_scale) / (1.0 + np.exp((2.0 * slope / sh_scale) * (lc - shoulder_match)))
    toe_c = np.where(lc < toe_match, toe_c, straight)
    sh_c = np.where(lc > shoulder_match, sh_c, straight)
    t = np.clip((lc - toe_match) / (shoulder_match - toe_match), 0.0, 1.0)
    if shoulder_match < toe_match:
        t = 1.0 - t
    t = (3.0 - 2.0 * t) * t * t
    return np.maximum(0.0, toe_c + (sh_c - toe_c) * t)


def _desat(c, amount):
    y = (c * LUMA).sum(-1, keepdims=True)
    return y + (c - y) * amount


def tone(c, p):
    """Linear colour (..., 3) -> display-linear 0..1 through the filmic curve with its desaturations."""
    c = _desat(np.maximum(c, 0.0), 0.96)
    out = film_curve(c, p["slope"], p["toe"], p["shoulder"], p["black"], p["white"])
    return np.maximum(0.0, _desat(out, 0.93))


_GRID = np.logspace(-6, 3, 4000)


def untone(d, p):
    """Inverse of tone() (the desaturations undone approximately, the curve per channel by lookup)."""
    d = _desat(np.clip(d, 0.0, 1.0), 1.0 / 0.93)
    curve = film_curve(_GRID, p["slope"], p["toe"], p["shoulder"], p["black"], p["white"])
    curve = np.maximum.accumulate(curve)
    lin = np.interp(np.clip(d, curve[0], curve[-1]), curve, _GRID)
    return np.maximum(_desat(lin, 1.0 / 0.96), 0.0)


def daylight_xy(t):
    """CIE daylight locus chromaticity at correlated colour temperature t (K); Planckian below 4000 K."""
    t = float(min(max(t, 1667.0), 25000.0))
    if t < 4000.0:      # Kim et al.'s Planckian locus fit
        x = -0.2661239e9 / t ** 3 - 0.2343589e6 / t ** 2 + 0.8776956e3 / t + 0.179910
        y = -1.1063814 * x ** 3 - 1.34811020 * x ** 2 + 2.18555832 * x - 0.20219683
        return x, y
    if t <= 7000.0:
        x = -4.6070e9 / t ** 3 + 2.9678e6 / t ** 2 + 0.09911e3 / t + 0.244063
    else:
        x = -2.0064e9 / t ** 3 + 1.9018e6 / t ** 2 + 0.24748e3 / t + 0.237040
    return x, -3.0 * x * x + 2.87 * x - 0.275


_SRGB_TO_XYZ = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]])
_BRADFORD = np.array([[0.8951, 0.2664, -0.1614], [-0.7502, 1.7135, 0.0367], [0.0389, -0.0685, 1.0296]])


def white_balance_matrix(temp):
    """Linear sRGB -> linear sRGB matrix that maps the white of a light of temp (K) to D65 (Unreal's WhiteTemp:
    lower = the light is taken as warmer, so the picture turns bluer)."""
    def xyz(xy):
        x, y = xy
        return np.array([x / y, 1.0, (1.0 - x - y) / y])
    src, dst = _BRADFORD @ xyz(daylight_xy(temp)), _BRADFORD @ xyz(daylight_xy(6504.0))
    cat = np.linalg.inv(_BRADFORD) @ np.diag(dst / src) @ _BRADFORD
    return np.linalg.inv(_SRGB_TO_XYZ) @ cat @ _SRGB_TO_XYZ


def colour_correct(c, p):
    """Unreal's ColorCorrect: saturation around the luma, contrast around mid grey 0.18, gamma, gain; and the
    shadows' saturation below ColorCorrectionShadowsMax."""
    y = (c * LUMA).sum(-1, keepdims=True)
    sat = p["sat"]
    if abs(p.get("sat_shadows", 1.0) - 1.0) > 1e-6:
        w = 1.0 - np.clip(y / max(p.get("shadows_max", 0.09), 1e-6), 0.0, 1.0)
        w = w * w * (3 - 2 * w)
        sat = sat * (1.0 + w * (p["sat_shadows"] - 1.0))
    c = np.maximum(0.0, y + (c - y) * sat)
    c = np.power(np.maximum(c, 0.0) / 0.18, p["contrast"]) * 0.18
    c = np.power(c, 1.0 / p["gamma"])
    return c * p.get("gain", 1.0)


def colour_uncorrect(c, p):
    c = np.maximum(c / p.get("gain", 1.0), 0.0)
    c = np.power(c, p["gamma"])
    c = np.power(np.maximum(c, 0.0) / 0.18, 1.0 / p["contrast"]) * 0.18
    y = (c * LUMA).sum(-1, keepdims=True)
    return np.maximum(0.0, y + (c - y) / p["sat"])       # (the shadows' saturation is left as it is)


def _blur(a, sigma):
    """Gaussian blur of a 2-D array (separable, reflected edges)."""
    r = max(1, int(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    p = np.pad(a, r, mode="reflect")
    p = np.apply_along_axis(lambda v: np.convolve(v, k, mode="valid"), 0, p)
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode="valid"), 1, p)


def base_log_lum(c):
    """The local exposure's base: log2 luminance blurred at two scales (a stand-in for its bilateral grid blended
    with its blurred luminance, 60/40)."""
    h = c.shape[0]
    ll = np.log2(np.maximum((c * LUMA).sum(-1), 1e-6))
    return 0.6 * _blur(ll, 0.25 * h) + 0.4 * _blur(ll, 0.03 * h)


MID = math.log2(0.18)


def local_exposure(c, hi, sh):
    if abs(hi - 1.0) < 1e-6 and abs(sh - 1.0) < 1e-6:
        return c
    b = base_log_lum(c) - MID
    s = np.where(b > 0, hi, sh)
    return c * np.exp2((s - 1.0) * b)[..., None]


def local_unexposure(c, hi, sh):
    """Inverse of local_exposure (the base of the result scales by the contrast scale, so it is divided back)."""
    if abs(hi - 1.0) < 1e-6 and abs(sh - 1.0) < 1e-6:
        return c
    b1 = base_log_lum(c) - MID
    s = np.where(b1 > 0, hi, sh)
    b0 = b1 / s
    return c * np.exp2(-(s - 1.0) * b0)[..., None]


def forward(scene, p):
    """Scene colour (linear, at the profile-independent exposure of invert()) -> sRGB uint8 with settings p."""
    c = scene * (2.0 ** (p["ev"] - UE_DEFAULTS["ev"]))
    c = local_exposure(c, p["local_hi"], p["local_sh"])
    c = c @ white_balance_matrix(p["temp"]).T
    c = colour_correct(c, p)
    return (srgb_encode(tone(c, p)) * 255.0 + 0.5).astype(np.uint8)


def invert(rgb, p):
    """sRGB uint8 rendered with settings p -> scene colour (forward(invert(x, p), p) ~ x)."""
    c = untone(srgb_decode(rgb.astype(np.float64) / 255.0), p)
    c = colour_uncorrect(c, p)
    c = c @ np.linalg.inv(white_balance_matrix(p["temp"])).T
    c = local_unexposure(c, p["local_hi"], p["local_sh"])
    return c / (2.0 ** (p["ev"] - UE_DEFAULTS["ev"]))


# ---------------------------------------------------------------- fitting

CLASSES = {0: 1.0, 1: 0.5, 2: 1.0, 4: 1.0, 5: 0.5}      # realism.GROUPS index -> weight (road, sidewalk, ...)


def class_stats(rgb, groups, classes=CLASSES, min_px=50):
    """Per class: mean R, G, B, the luminance spread inside the class and its 10th and 90th percentiles (0-255);
    {} for classes too small."""
    a = rgb.astype(np.float64)
    lum = (a * LUMA).sum(-1)
    out = {}
    for k in classes:
        m = groups == k
        if m.sum() >= min_px:
            out[k] = np.concatenate([a[m].mean(0), [lum[m].std()], np.percentile(lum[m], [10, 90])])
    return out


def loss(pairs, p, classes=CLASSES):
    """Mean over the views of the weighted squared differences of the class statistics (render with p against the
    photo), scaled so that 20 levels of mean colour or of a percentile, or 10 of spread, count as 1, plus the global
    luminance spread."""
    tot, n = 0.0, 0
    for pr in pairs:
        out = forward(pr["scene"], p)
        rs = class_stats(out, pr["render_groups"], classes)
        for k, w in classes.items():
            if k in rs and k in pr["real_stats"]:
                d = rs[k] - pr["real_stats"][k]
                tot += w * (float((d[:3] ** 2).sum()) / 3 / 400.0 + float(d[3] ** 2) / 100.0
                            + float((d[4:] ** 2).sum()) / 2 / 400.0)
        lum = (out.astype(np.float64) * LUMA).sum(-1)
        tot += (lum.std() - pr["real_std"]) ** 2 / 100.0
        n += 1
    return tot / max(1, n)


def nelder_mead(f, x0, step, iters=300, tol=1e-5):
    """A plain Nelder-Mead minimiser (numpy only: the project's environment has no scipy)."""
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(x) for x in pts]
    for _ in range(iters):
        order = np.argsort(vals)
        pts, vals = [pts[i] for i in order], [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        c = np.mean(pts[:-1], 0)
        xr = c + (c - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = c + 2 * (c - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (x - pts[0]) for x in pts[1:]]
                vals = [vals[0]] + [f(x) for x in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def fit(pairs, start, names=None, reg=0.02, classes=CLASSES, log=print):
    """Settings (a dict like profile_params) that minimise loss() on pairs, starting from start; names: the PARAMS
    fitted (default all). reg pulls each towards its start (per unit of its range) so that settings the data do
    not constrain stay put."""
    names = names or [n for n, *_ in PARAMS]
    bounds = {n: (lo, hi) for n, _, lo, hi in PARAMS}
    span = {n: bounds[n][1] - bounds[n][0] for n in names}

    def unpack(x):
        p = dict(start)
        for n, v in zip(names, x):
            p[n] = float(min(max(v * span[n], bounds[n][0]), bounds[n][1]))
        return p

    def f(x):
        p = unpack(x)
        return loss(pairs, p, classes) + reg * sum(((p[n] - start[n]) / span[n]) ** 2 for n in names)
    x0 = [start[n] / span[n] for n in names]
    x, v = nelder_mead(f, x0, [0.08] * len(names))
    best = unpack(x)
    log(f"fit: loss {loss(pairs, start, classes):.3f} -> {loss(pairs, best, classes):.3f} on {len(pairs)} views")
    return best


def to_profile(base, p, extra=None):
    """A profile (FPostProcessSettings JSON) from base with the fitted settings written and overridden."""
    out = json.loads(json.dumps(base))
    s = out["settings"]
    for name, key in _KEYS.items():
        if name not in p or name in ("tint", "gain", "sat_shadows", "shadows_max", "shoulder", "black", "white"):
            continue
        v = round(float(p[name]), 4)
        cur = s.get(_lower(key))
        s[_lower(key)] = {"x": v, "y": v, "z": v, "w": 1.0} if isinstance(cur, dict) else v
        s["bOverride_" + key] = True
    for k, v in (extra or {}).items():
        s[k] = v
    return out
