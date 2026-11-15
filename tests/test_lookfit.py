"""coupledcarla/lookfit.py: the model of Unreal's tone mapping and the fit of a post-process profile."""
import json

import numpy as np
import pytest

from coupledcarla import lookfit as L


def _scene(seed=0, h=36, w=64):
    rng = np.random.default_rng(seed)
    base = np.exp(rng.normal(np.log(0.15), 1.0, (h, w, 1)))        # a log-normal spread of scene luminance
    tint = rng.uniform(0.7, 1.3, (h, w, 3))
    return base * tint


def test_curve_and_white_balance():
    assert L.film_curve(np.array([0.18]), 0.88, 0.4, 0.26, 0.0, 0.04)[0] == pytest.approx(0.18, abs=1e-6)
    x = np.logspace(-4, 2, 200)
    assert np.all(np.diff(L.film_curve(x)) >= 0)                   # monotonic
    np.testing.assert_allclose(L.white_balance_matrix(6504.0) @ np.ones(3), np.ones(3), atol=1e-3)
    cool, warm = L.white_balance_matrix(6000.0) @ np.ones(3), L.white_balance_matrix(7500.0) @ np.ones(3)
    assert cool[2] > cool[0] and warm[0] > warm[2]                 # Unreal: lower WhiteTemp = bluer picture


def test_invert_is_the_inverse_of_forward():
    p = dict(L.UE_DEFAULTS, ev=0.3, temp=6000.0, sat=1.1, contrast=0.9, toe=0.4, local_hi=0.5, sat_shadows=1.0)
    rgb = L.forward(_scene(), p)
    back = L.forward(L.invert(rgb, p), p)
    assert np.abs(back.astype(int) - rgb.astype(int)).mean() < 2.0


def test_profile_params_and_to_profile():
    prof = {"settings": {"bOverride_AutoExposureBias": True, "autoExposureBias": -0.5,
                         "bOverride_ColorSaturation": True, "colorSaturation": {"x": 1.2, "y": 1.2, "z": 1.2, "w": 1},
                         "bOverride_WhiteTemp": False, "whiteTemp": 5000, "colorContrast": {"x": 1, "y": 1, "z": 1,
                                                                                            "w": 1}}}
    p = L.profile_params(prof["settings"])
    assert p["ev"] == -0.5 and p["sat"] == pytest.approx(1.2) and p["temp"] == 6500.0      # not overridden: default
    out = L.to_profile(prof, dict(p, contrast=1.3, temp=7000.0), {"vignetteIntensity": 0.0})
    s = out["settings"]
    assert s["bOverride_ColorContrast"] and s["colorContrast"] == {"x": 1.3, "y": 1.3, "z": 1.3, "w": 1.0}
    assert s["bOverride_WhiteTemp"] and s["whiteTemp"] == 7000.0 and s["vignetteIntensity"] == 0.0
    assert prof["settings"]["whiteTemp"] == 5000                     # the base is not changed
    real = L.load_profile("CoupledReal")      # the shipped fitted profile: darker, warmer, more saturated than Dashcam
    rp = L.profile_params(real["settings"])
    assert rp["ev"] < 0.3 and rp["temp"] > 6500 and rp["sat"] > 1.1
    assert real["settings"]["bOverride_VignetteIntensity"] and real["settings"]["vignetteIntensity"] == 0.0


def test_fit_recovers_settings_from_their_renders():
    rendered = dict(L.UE_DEFAULTS, ev=0.3, temp=6000.0, contrast=0.9, local_hi=1.0)
    target = dict(rendered, ev=-0.4, temp=7000.0, contrast=1.2)
    pairs = []
    for k in range(4):
        scene = _scene(k)
        groups = np.zeros(scene.shape[:2], np.uint8)
        groups[:, 32:] = 2
        groups[:10] = 5
        real = L.forward(scene, target)
        lum = (real.astype(float) * L.LUMA).sum(-1)
        pairs.append({"scene": L.invert(L.forward(scene, rendered), rendered), "render_groups": groups,
                      "real_stats": L.class_stats(real, groups, min_px=10), "real_std": float(lum.std())})
    before = L.loss(pairs, rendered)
    best = L.fit(pairs, rendered, ["ev", "temp", "contrast"], reg=0.0, log=lambda m: None)
    assert L.loss(pairs, best) < 0.1 * before
    assert best["ev"] == pytest.approx(-0.4, abs=0.25) and best["temp"] == pytest.approx(7000, abs=500)


def test_look_fit_split_keeps_places_together():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    import look_fit
    pairs = {f"v{i}": {"lat": 51.44 + 0.0001 * i, "lon": 5.48} for i in range(5)}         # within 60 m
    pairs.update({f"w{i}": {"lat": 51.44, "lon": 5.48 + 0.01 * i} for i in range(1, 9)})   # 700 m apart
    a, b = look_fit.split(pairs)
    assert a and b and len(a) + len(b) == len(pairs)
    near = [i for i in pairs if i.startswith("v")]
    assert sum(i in a for i in near) in (0, 1, 4, 5)                # neighbours mostly on one side
    json.dumps(look_fit.CLEAN)
