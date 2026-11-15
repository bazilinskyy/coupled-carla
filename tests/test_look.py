"""coupledcarla.look: exposure from the sun, render rates, supersampling and capture cameras (no CARLA)."""
import math

import numpy as np
import pytest

from coupledcarla import compat, look


class BP:
    """A blueprint with the attributes of a CARLA 0.9.16 RGB camera (or 1.0 with post_process_profile)."""

    def __init__(self, v10=False):
        self.attrs = {"image_size_x": "800", "image_size_y": "600", "sensor_tick": "0.0"}
        if v10:
            self.attrs["post_process_profile"] = ""
        else:
            self.attrs.update({"exposure_mode": "histogram", "shutter_speed": "200", "fstop": "1.4", "iso": "100",
                               "motion_blur_intensity": "0.45", "bloom_intensity": "0.675",
                               "lens_flare_intensity": "0.1", "chromatic_aberration_intensity": "0"})

    def has_attribute(self, k):
        return k in self.attrs

    def set_attribute(self, k, v):
        self.attrs[k] = str(v)


class Weather:
    def __init__(self, alt, clouds):
        self.sun_altitude_angle, self.cloudiness = alt, clouds


class World:
    def __init__(self, alt=45.0, clouds=5.0):
        self.w = Weather(alt, clouds)

    def get_weather(self):
        return self.w


def test_exposure_table_points_and_interpolation():
    assert look.exposure_ev(45, 5) == pytest.approx(11.0)
    assert look.exposure_ev(45, 60) == pytest.approx(10.45)
    assert look.exposure_ev(12, 5) == pytest.approx(9.35)
    assert look.exposure_ev(18.5, 5) == pytest.approx((9.35 + 10.3) / 2)
    assert look.exposure_ev(45, 32.5) == pytest.approx((11.0 + 10.45) / 2)     # half way between clear and overcast
    assert look.exposure_ev(45, 100) == look.exposure_ev(45, 60)               # beyond the table: overcast
    assert look.exposure_ev(45, 0) == look.exposure_ev(45, 5)


def test_exposure_falls_with_the_sun_and_is_flat_at_night():
    evs = [look.exposure_ev(a, 5) for a in (90, 45, 25, 12, 5, 3, 1, 0)]
    assert all(a >= b for a, b in zip(evs, evs[1:]))
    assert look.exposure_ev(-5, 5) == look.exposure_ev(-60, 5) == pytest.approx(6.2)
    assert look.exposure_ev(-100, 60) == pytest.approx(6.8)      # clamped to the table
    assert look.exposure_ev(120, 5) == pytest.approx(11.0)


def test_shutter_and_ev_are_inverse():
    assert look.shutter_for_ev(11) == pytest.approx(2048 / 1.96)
    for ev in (6.2, 9.6, 11.0):
        assert look.ev_of_shutter(look.shutter_for_ev(ev)) == pytest.approx(ev)
    assert look.ev_of_shutter(400) == pytest.approx(math.log2(1.96 * 400))      # the old daylight shutter: 9.6


def test_target_luminance():
    assert look.target_luminance(30) == 118 and look.target_luminance(3) == 118
    assert look.target_luminance(-10) == 55 and look.target_luminance(0) == 55
    assert 55 < look.target_luminance(1.5) < 118


def test_best_ev_interpolates_between_bracketing_exposures():
    cands = [(10.0, 160.0, 40), (10.5, 139.0, 40), (11.0, 118.5, 40), (11.5, 98.0, 40)]
    assert look.best_ev(cands) == pytest.approx(11.0 + 0.5 * 0.5 / 20.5, abs=1e-6)
    assert look.best_ev([(9.0, 60.0, 40), (9.5, 50.0, 40)]) == 9.0            # all too dark: the brightest
    assert look.best_ev([(6.0, 70.0, -8), (6.5, 50.0, -8)]) == pytest.approx(6.0 + 0.5 * 15 / 20)   # night: 55
    assert look.best_ev([]) is None


def test_settings_merge_and_switches():
    assert look.settings(None) == look.DEFAULTS
    assert look.settings({"capture": {"supersample": 1}})["supersample"] == 1
    assert look.settings({"step": 0.02, "capture": False})["enabled"] is False
    assert look.settings(False)["enabled"] is False
    assert look.settings({"render_hz": 5})["render_hz"] == 5


def test_render_interval_and_capture_size():
    cap = look.settings(None)
    assert look.render_interval(1.0, cap) == pytest.approx(0.1)
    assert look.render_interval(0.04, cap) == pytest.approx(0.04)      # already faster than render_hz
    assert look.render_interval(0.0, cap) == 0.0
    assert look.render_interval(2.0, {**cap, "enabled": False}) == 2.0
    assert look.capture_size((1280, 720), cap) == (2560, 1440)
    assert look.capture_size((1280, 720), cap, saving=False) == (1280, 720)
    assert look.capture_size((2560, 1440), cap) == (3840, 2160)        # capped at MAX_WIDTH
    assert look.capture_size((1280, 720), {**cap, "enabled": False}) == (1280, 720)


def test_setup_camera_capture_on_09():
    bp = BP()
    keep = compat.setup_camera(bp, 1.0, "Dashcam", 400, world=World(45, 5), capture={"supersample": 2})
    assert keep == 1.0                                   # renders at 10 Hz, the caller keeps one frame a second
    assert float(bp.attrs["sensor_tick"]) == pytest.approx(0.1)
    assert bp.attrs["motion_blur_intensity"] == "0" and bp.attrs["bloom_intensity"] == "0.3"
    assert bp.attrs["exposure_mode"] == "manual"
    assert look.ev_of_shutter(float(bp.attrs["shutter_speed"])) == pytest.approx(11.0, abs=0.01)
    dark = BP()
    compat.setup_camera(dark, 1.0, world=World(-10, 5), capture={})
    assert look.ev_of_shutter(float(dark.attrs["shutter_speed"])) == pytest.approx(6.2, abs=0.01)
    biased = BP()
    compat.setup_camera(biased, 1.0, world=World(45, 5), capture={"ev_bias": 1.0})
    assert look.ev_of_shutter(float(biased.attrs["shutter_speed"])) == pytest.approx(12.0, abs=0.01)


def test_setup_camera_without_capture_is_unchanged():
    bp = BP()
    assert compat.setup_camera(bp, 2.0, "Dashcam", 400) == 0.0
    assert bp.attrs["sensor_tick"] == "2.0" and bp.attrs["shutter_speed"] == "400"
    assert bp.attrs["motion_blur_intensity"] == "0.45"
    off = BP()
    assert compat.setup_camera(off, 2.0, "Dashcam", 400, world=World(), capture=False) == 0.0
    assert off.attrs["shutter_speed"] == "400"
    nw = BP()          # capture without a world: the caller's fixed shutter, but clean and at 10 Hz
    assert compat.setup_camera(nw, 2.0, "Dashcam", 400, capture={}) == 2.0
    assert look.ev_of_shutter(float(nw.attrs["shutter_speed"])) == pytest.approx(look.ev_of_shutter(400), abs=0.01)


def test_setup_camera_on_10_uses_profiles():
    bp = BP(v10=True)
    assert compat.setup_camera(bp, 2.0, "Dashcam", capture={}) == 2.0
    assert bp.attrs["post_process_profile"] == "Dashcam"
    bp = BP(v10=True)
    compat.setup_camera(bp, 2.0, "Dashcam", capture={"profile": "CoupledClean"})
    assert bp.attrs["post_process_profile"] == "CoupledClean"


def test_downscale_and_save(tmp_path):
    rgb = np.zeros((40, 80, 3), np.uint8)
    rgb[:, ::2] = 255                      # one-pixel stripes average to grey
    small = look.downscale(rgb, (40, 20))
    assert small.shape == (20, 40, 3) and 100 < small.mean() < 155
    assert look.downscale(rgb, (80, 40)) is rgb

    class Img:          # a carla.Image: BGRA bytes
        width, height = 80, 40
        raw_data = np.dstack([rgb[..., 2], rgb[..., 1], rgb[..., 0], np.full((40, 80), 255, np.uint8)]).tobytes()
    look.save_frame(Img(), tmp_path / "f.jpg", (40, 20))
    look.save_frame(rgb, tmp_path / "f.png")
    from PIL import Image
    assert Image.open(tmp_path / "f.jpg").size == (40, 20)
    assert Image.open(tmp_path / "f.png").size == (80, 40)


def test_clean_profile_for_carla_10():
    import json
    from pathlib import Path
    p = Path(look.__file__).parent / "data" / "postprocess" / "CoupledClean.json"
    s = json.loads(p.read_text())["settings"]
    assert s["motionBlurAmount"] == 0 and s["bOverride_MotionBlurAmount"]
    assert s["sceneFringeIntensity"] == 0 and s["vignetteIntensity"] == 0 and s["filmGrainIntensity"] == 0
    assert s["lensFlareIntensity"] == 0 and s["bOverride_LensFlareIntensity"]
    assert s["autoExposureMethod"] == "AEM_Histogram" and not s["weightedBlendables"]["array"]


def test_debug_colour_steps():
    look.DEBUG_COMPRESS = True
    assert look.debug_colour((0, 255, 0)) == (0, 2, 0)
    assert look.debug_colour((230, 20, 0)) == (2, 0, 0)
    assert look.debug_colour((255, 160, 0)) == (2, 1, 0)
    assert look.debug_colour((20, 20, 20)) == (0, 0, 0)
    look.DEBUG_COMPRESS = False
    try:
        assert look.debug_colour((0, 255, 0)) == (0, 255, 0)
    finally:
        look.DEBUG_COMPRESS = True


def test_realism_guard_rejects_an_empty_world_and_bad_exposure():
    h, w = 90, 160
    tags = np.full((h, w), 10, np.uint8)            # terrain (an OpenDRIVE road) ...
    tags[:40] = 11                                  # ... under the sky
    rgb = np.full((h, w, 3), 120, np.uint8)
    probs = look.realism_problems(rgb, tags)
    assert any("empty world" in p for p in probs)
    tags[30:70, 10:60] = 3                          # a building block and trees
    tags[40:80, 100:140] = 9
    assert look.realism_problems(rgb, tags) == []
    car = np.full((h, w), 10, np.uint8)
    car[40:90, 40:120] = 14                         # only the ego car in an empty world
    assert any("vehicles" in p for p in look.realism_problems(rgb, car))
    assert any("too dark" in p for p in look.realism_problems(np.full((h, w, 3), 10, np.uint8)))
    assert any("overexposed" in p for p in look.realism_problems(np.full((h, w, 3), 255, np.uint8)))
    glare = rgb.copy()
    glare[:40, :40] = (0, 255, 120)                 # a saturated green blob: 11 % of the frame
    assert any("glare" in p for p in look.realism_problems(glare))


def test_semantic_shares_and_cli(tmp_path):
    s = look.semantic_shares(np.array([[3, 9], [11, 14]]))
    assert s == {"scene": 0.75, "fixed": 0.5, "empty": 0.25}
    from PIL import Image
    Image.fromarray(np.full((20, 20, 3), 120, np.uint8)).save(tmp_path / "ok.png")
    Image.fromarray(np.zeros((20, 20, 3), np.uint8)).save(tmp_path / "dark.png")
    assert look.main([str(tmp_path / "ok.png")]) == 0
    assert look.main([str(tmp_path / "ok.png"), str(tmp_path / "dark.png")]) == 1
