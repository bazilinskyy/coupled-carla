"""Fit a CARLA 1.0 post-process profile to real street photos (coupledcarla/lookfit.py), from a realism run.

    # renders made with the Dashcam profile (tools/realism_check.py render/compare), CPU only
    python tools/look_fit.py --out logs/realism --rendered Dashcam --base CoupledClean \\
        --write coupledcarla/data/postprocess/CoupledReal.json

Each compared view gives a pair: the photo and the render cut to the photo's best alignment, with both images'
segmentation (realism_check's compare cache; the pairs are kept in <out>/look_pairs.npz). The views are split by
place into two halves (150 m cells, alternating): the profile is fitted on one and scored on the other, so the
numbers it prints are for views it was not fitted to. --rendered names the profile the renders were made with
(a name in coupledcarla/data/postprocess or CARLA's Config/PostProcess folder given with --profiles, or a JSON
path; "ue4" for a CARLA 0.9.x camera); --base the profile whose other settings the written one keeps ("ue4" with
--params ev,temp,slope,toe fits a 0.9.x camera: the camera_09 line gives its ev_bias and attributes).
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from coupledcarla import lookfit as L  # noqa: E402
from coupledcarla import realism as R  # noqa: E402

FIT_W, EVAL_W = 192, 384
# what the written profile turns off besides the fitted settings: effects the photos lack or the model leaves out
CLEAN = {"vignetteIntensity": 0.0, "bOverride_VignetteIntensity": True, "sceneFringeIntensity": 0.0,
         "bOverride_SceneFringeIntensity": True, "filmGrainIntensity": 0.0, "bOverride_FilmGrainIntensity": True,
         "motionBlurAmount": 0.0, "bOverride_MotionBlurAmount": True, "lensFlareIntensity": 0.0,
         "bOverride_LensFlareIntensity": True}


def _resize(a, w, nearest=False):
    from PIL import Image
    h = int(round(a.shape[0] * w / a.shape[1]))
    return np.asarray(Image.fromarray(a).resize((w, h), Image.NEAREST if nearest else Image.LANCZOS))


def load_pairs(out, cache=None):
    """{view id: dict(real, ren, rg, ag at EVAL_W, source, lat, lon)} from a compared realism run (cached)."""
    import realism_check as RC
    out = Path(out)
    cache = Path(cache or out / "look_pairs.npz")
    if cache.exists():
        z = np.load(cache, allow_pickle=False)
        meta = json.loads(str(z["meta"]))
        return {i: dict(meta[i], **{f: z[f"{i}__{f}"] for f in ("real", "ren", "rg", "ag")}) for i in meta}
    views = {v["id"]: v for v in R.read_views(out / "views.csv")}
    renders = {r["id"]: r for r in csv.DictReader((out / "renders.csv").open())}
    labels = RC.Labels(out, "nvidia/segformer-b2-finetuned-cityscapes-1024-1024", "cpu")
    pairs = {}
    for vid, rr in renders.items():
        v = views.get(vid)
        rp = out / "renders" / f"{vid}.png"
        if v is None or not rp.exists() or not (labels.dir / f"render_{vid}.png").exists():
            continue
        try:
            real = RC.real_spec(v, out, labels)
        except Exception as err:  # noqa: BLE001 - a missing photo
            print(f"{vid}: {err}", flush=True)
            continue
        wide = RC._load(rp)
        _, rg, ag, arg, real_rgb = RC.view_metrics(v, real, wide, labels(f"render_{vid}", wide), float(rr["wide_hfov"]))
        pairs[vid] = {"real": _resize(real_rgb, EVAL_W), "ren": _resize(arg, EVAL_W), "rg": _resize(rg, EVAL_W, True),
                      "ag": _resize(ag, EVAL_W, True), "source": v["source"], "lat": v["lat"], "lon": v["lon"]}
    meta = {i: {k: p[k] for k in ("source", "lat", "lon")} for i, p in pairs.items()}
    np.savez_compressed(cache, meta=json.dumps(meta),
                        **{f"{i}__{f}": p[f] for i, p in pairs.items() for f in ("real", "ren", "rg", "ag")})
    return pairs


def split(pairs, cell_m=150.0):
    """(fit ids, test ids): alternating 150 m cells, so neighbouring photos of one street stay on one side."""
    a, b = [], []
    for i, p in sorted(pairs.items()):
        cx = math.floor(p["lon"] * 111320.0 * math.cos(math.radians(p["lat"])) / cell_m)
        cy = math.floor(p["lat"] * 111320.0 / cell_m)
        (a if (cx + cy) % 2 == 0 else b).append(i)
    return a, b


def scores(pairs, ids, fn=None):
    """Median image statistics of photo and render (fn: a transform of the render) over ids, as the realism check
    computes them (over pixels both segmentations label)."""
    rows = []
    for i in ids:
        p = pairs[i]
        m = (p["rg"] != R.IGNORE) & (p["ag"] != R.IGNORE)
        ren = fn(i) if fn else p["ren"]
        sr, sd = R.image_stats(p["real"], m), R.image_stats(ren, m)
        rows.append([R.colour_hist_sim(p["real"], ren, m)] + [sr[k] for k in KEYS] + [sd[k] for k in KEYS])
    med = np.median(np.array(rows), 0)
    n = len(KEYS)
    return {"colour_hist_sim": round(float(med[0]), 3),
            **{k: [round(float(med[1 + j]), 3), round(float(med[1 + n + j]), 3)] for j, k in enumerate(KEYS)}}


KEYS = ("lum_mean", "lum_std", "sat_mean", "colourfulness", "edge_density")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="logs/realism", help="the realism run (views.csv, renders/, seg/)")
    ap.add_argument("--rendered", default="Dashcam", help="profile the renders were made with")
    ap.add_argument("--base", default="CoupledClean", help="profile the written one starts from")
    ap.add_argument("--profiles", default=None, help="folder with CARLA's profiles (Content/Carla/Config/PostProcess)")
    ap.add_argument("--params", default=None, help="comma-separated settings to fit (default all of lookfit.PARAMS)")
    ap.add_argument("--no-sky", action="store_true", help="leave the sky out of the fit (e.g. for an overcast sky)")
    ap.add_argument("--write", default=None, help="write the fitted profile here (JSON)")
    ap.add_argument("--name", default="CoupledReal")
    ap.add_argument("--summary", default=None, help="write the numbers here (JSON)")
    a = ap.parse_args()
    search = [L.DATA] + ([Path(a.profiles)] if a.profiles else [])
    # "ue4": a CARLA 0.9.x camera (Unreal 4's defaults: no local exposure, no colour grading attributes)
    rendered = dict(L.UE_DEFAULTS) if a.rendered == "ue4" else \
        L.profile_params(L.load_profile(a.rendered, search)["settings"])
    base_prof = None if a.base == "ue4" else L.load_profile(a.base, search)
    base = dict(L.UE_DEFAULTS) if base_prof is None else L.profile_params(base_prof["settings"])
    pairs = load_pairs(a.out)
    fit_ids, test_ids = split(pairs)
    print(f"{len(pairs)} pairs: {len(fit_ids)} to fit, {len(test_ids)} to test", flush=True)
    classes = {k: w for k, w in L.CLASSES.items() if not (a.no_sky and k == 5)}
    fit_pairs = []
    for i in fit_ids:
        p = pairs[i]
        ren, ag, rg, real = (_resize(p[k], FIT_W, k in ("ag", "rg")) for k in ("ren", "ag", "rg", "real"))
        lum = (real.astype(np.float64) * L.LUMA).sum(-1)
        fit_pairs.append({"scene": L.invert(ren, rendered), "render_groups": ag,
                          "real_stats": L.class_stats(real, rg, classes), "real_std": float(lum.std())})
    # the start: the base profile's settings at the rendered profile's exposure (a profile change alone)
    start = dict(base)
    names = a.params.split(",") if a.params else None
    best = L.fit(fit_pairs, start, names, classes=classes)
    scene = {}

    def render_with(p):
        def fn(i):
            if i not in scene:
                scene[i] = L.invert(pairs[i]["ren"], rendered)
            return L.forward(scene[i], p)
        return fn
    res = {"rendered": a.rendered, "base": a.base, "fit_views": len(fit_ids), "test_views": len(test_ids),
           "fitted": {k: round(v, 4) for k, v in best.items() if k in dict((n, 1) for n, *_ in L.PARAMS)},
           "test": {"as_rendered": scores(pairs, test_ids), "base_profile": scores(pairs, test_ids, render_with(base)),
                    "fitted": scores(pairs, test_ids, render_with(best))},
           "fit": {"as_rendered": scores(pairs, fit_ids), "fitted": scores(pairs, fit_ids, render_with(best))}}
    res["camera_09"] = {"ev_bias": round(base["ev"] - best["ev"], 2),      # look.py's ev_bias: +1 = a stop darker
                        "camera_attr": f"temp={best['temp']:.0f},slope={best['slope']:.3f},toe={best['toe']:.3f}"}
    print(json.dumps(res, indent=1))
    if a.write and base_prof is not None:
        prof = L.to_profile(base_prof, best, CLEAN)
        Path(a.write).write_text(json.dumps(prof, indent="\t"))
        print(f"wrote {a.write} ({a.name})")
    if a.summary:
        Path(a.summary).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
