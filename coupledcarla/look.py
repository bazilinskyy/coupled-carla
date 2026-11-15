"""Image quality of the cameras whose frames are kept: screenshots, --frames-every, station frames and videos.

One switch, simulation.capture (config.py), used by compat.setup_camera and the callers that save frames
(display.View, station.py, drones.py, tools/scenery_shots.py). Measured on CARLA 0.9.16 at Eindhoven (lin4080,
RTX 4080 SUPER, Epic; README "Camera image quality"):

- Motion blur off. A camera that renders only every few ticks (sensor_tick) blurs the motion since its last render:
  sharpness (variance of the Laplacian) 78 against 312 without blur at 1 Hz from a car at 8 m/s.
- Render often enough for temporal anti-aliasing: at 1 Hz every frame is a first frame (dithered leaves and car
  paint, PSNR 26.8 dB against a 2x supersampled every-tick reference); at 10 Hz rendered at 2x and downscaled:
  32.6 dB, by eye the same as the reference. So a camera that keeps a frame every few seconds still renders at
  render_hz (10) and the caller keeps every every_s.
- Supersampling: render at 2x and downscale with Lanczos (44 ms per kept 1280x720 frame on the client). A 2560x1440
  camera costs 1.5 ms per tick at 10 Hz (10 ms when rendered every tick; 720p: 6 ms), and about 300 MB of VRAM.
- Exposure: manual, from the sun's altitude and the cloudiness (exposure_ev). CARLA's histogram auto exposure
  always sits at its lower bound (exposure_min_bright 10 = EV100 10) in daylight, one EV too bright at noon (median
  ground 160 of 255, road 186); with a wide range it overexposes further (237). The table keeps the median of the
  ground (everything but the sky) near 118 of 255 by day and about 55 at night.
- Bloom 0.3 instead of 0.675: sharpness +12 %; no lens flare, no colour fringes. CARLA's default filmic tonemapper
  stays: toe 0.4 lifted the shadows (contrast -19 %), slope 1.0 crushed them (2-5 % black).
"""
import math

# Sun altitude (deg) -> EV100 of a street-level view (CARLA 0.9.16, Eindhoven, measured with tools/look_check.py):
# clear (cloudiness 5) and overcast (cloudiness 60, the most coupled-carla's real weather sets). Below the horizon
# CARLA 0.9.16 lights the world with a constant night light; at 1 deg the sky is already at dusk.
EV_TABLE = [
    # alt, clear, overcast
    (-90.0, 6.2, 6.8),
    (0.0, 6.2, 6.8),
    (1.0, 7.5, 8.0),
    (3.0, 8.1, 8.7),
    (5.0, 8.65, 9.2),
    (12.0, 9.35, 9.7),
    (25.0, 10.3, 10.1),
    (45.0, 11.0, 10.45),
    (90.0, 11.0, 10.6),
]
CLOUD_CLEAR, CLOUD_OVERCAST = 5.0, 60.0

# 0.9.x RGB camera attributes of a capture camera (exposure comes from exposure_ev)
CLEAN_09 = {
    "motion_blur_intensity": "0",
    "lens_flare_intensity": "0",
    "chromatic_aberration_intensity": "0",
    "bloom_intensity": "0.3",
}

DEFAULTS = {"enabled": True, "supersample": 2, "render_hz": 10.0, "ev_bias": 0.0, "profile": None}
MAX_WIDTH = 3840     # a supersampled camera never renders wider than this


def settings(sim_or_capture):
    """The capture settings from a config's simulation section (or a capture dict, or None: the defaults)."""
    c = sim_or_capture
    if isinstance(c, dict) and ("capture" in c or "step" in c):
        c = c.get("capture")
    if c is False:
        c = {"enabled": False}
    elif not isinstance(c, dict):
        c = {}
    return {**DEFAULTS, **c}


def exposure_ev(sun_altitude, cloudiness=0.0):
    """EV100 that keeps a CARLA 0.9.16 street view mid-grey at this sun altitude (deg) and cloudiness (0-100)."""
    alt = max(-90.0, min(90.0, float(sun_altitude)))
    w = (float(cloudiness) - CLOUD_CLEAR) / (CLOUD_OVERCAST - CLOUD_CLEAR)
    w = max(0.0, min(1.0, w))
    for (a0, c0, o0), (a1, c1, o1) in zip(EV_TABLE, EV_TABLE[1:]):
        if a0 <= alt <= a1:
            f = (alt - a0) / (a1 - a0) if a1 > a0 else 0.0
            clear = c0 + f * (c1 - c0)
            overcast = o0 + f * (o1 - o0)
            return clear + w * (overcast - clear)
    return EV_TABLE[-1][1]


def target_luminance(sun_altitude):
    """Median luminance (0-255) of the ground the exposure aims at: 118 by day, 55 at night, linear in between
    (sun from 3 deg down to 0, where CARLA 0.9.16 switches to its night light)."""
    alt = float(sun_altitude)
    if alt >= 3.0:
        return 118.0
    if alt <= 0.0:
        return 55.0
    return 55.0 + (118.0 - 55.0) * alt / 3.0


def best_ev(candidates):
    """[(ev, median ground luminance, sun altitude)] -> the EV whose image is nearest target_luminance, refined by
    interpolating between the two candidates that bracket the target (a tool's auto exposure)."""
    pts = sorted((float(e), float(m)) for e, m, _ in candidates)
    if not pts:
        return None
    target = target_luminance(candidates[0][2])
    for (e0, m0), (e1, m1) in zip(pts, pts[1:]):
        if m0 >= target >= m1 and m0 > m1:      # higher EV = darker
            return e0 + (m0 - target) / (m0 - m1) * (e1 - e0)
    return min(pts, key=lambda p: abs(p[1] - target))[0]


# A natural daylight for documentation shots (tools/scenery_shots.py --weather doc): the sun 40 deg up, some clouds,
# CARLA's light haze; ClearNoon's 45 deg under a cloudless sky looks harsh, a real November afternoon in the
# Netherlands has the sun 15 deg up with long shadows.
DOC_WEATHER = {"cloudiness": 30.0, "precipitation": 0.0, "precipitation_deposits": 0.0, "wetness": 0.0,
               "wind_intensity": 10.0, "sun_altitude_angle": 40.0, "fog_density": 2.0, "fog_distance": 0.75,
               "fog_falloff": 0.1, "scattering_intensity": 1.0, "mie_scattering_scale": 0.03,
               "rayleigh_scattering_scale": 0.0331}


# CARLA 0.9.16 draws debug shapes (traffic lights, eHMIs) after tone mapping with a huge gain: a colour channel
# shows as 0 at 0, about 200 at 1, 235 at 2 and saturates from 3 (measured, the same by day and at night), so
# (0, 255, 0) is a blinding square with a bloom halo. debug_colour() maps a wanted 0-255 colour onto those steps.
DEBUG_COMPRESS = True          # run.py sets it False on CARLA 1.0, whose debug colours were not measured


def debug_colour(rgb):
    """A wanted display colour (0-255 per channel) -> the debug colour that shows close to it on CARLA 0.9.16
    (0 below 90, 1 below 200, else 2), without bloom; unchanged when DEBUG_COMPRESS is off."""
    if not DEBUG_COMPRESS:
        return tuple(int(c) for c in rgb)
    return tuple(0 if c < 90 else (1 if c < 200 else 2) for c in rgb)


def shutter_for_ev(ev, fstop=1.4, iso=100.0):
    """CARLA's manual exposure: EV100 = log2(fstop^2 * shutter_speed) - log2(iso / 100); shutter_speed is 1/s."""
    return 2.0 ** float(ev) * (float(iso) / 100.0) / float(fstop) ** 2


def ev_of_shutter(shutter, fstop=1.4, iso=100.0):
    return math.log2(float(fstop) ** 2 * float(shutter)) - math.log2(float(iso) / 100.0)


def render_interval(every_s, capture):
    """sensor_tick (s) of a 0.9.x camera that keeps a frame every every_s: no slower than 1/render_hz."""
    if not every_s or every_s <= 0:
        return 0.0
    hz = float(capture.get("render_hz") or 0)
    if capture.get("enabled") and hz > 0:
        return min(float(every_s), 1.0 / hz)
    return float(every_s)


def capture_size(size, capture, saving=True):
    """The camera's render size: the output size times capture.supersample (only for cameras that save frames)."""
    w, h = int(size[0]), int(size[1])
    ss = float(capture.get("supersample") or 1) if (saving and capture.get("enabled")) else 1.0
    ss = max(1.0, min(ss, MAX_WIDTH / max(1, w)))
    return int(round(w * ss)), int(round(h * ss))


def apply_09(bp, capture, weather=None, shutter=None):
    """A capture camera's attributes on CARLA 0.9.x: CLEAN_09 and a manual exposure from the weather (else the
    caller's fixed shutter). Returns the EV100 set (None when the blueprint has no exposure attributes)."""
    for k, v in CLEAN_09.items():
        if bp.has_attribute(k):
            bp.set_attribute(k, v)
    if not bp.has_attribute("exposure_mode"):
        return None
    if weather is not None:
        ev = exposure_ev(weather.sun_altitude_angle, weather.cloudiness) + float(capture.get("ev_bias") or 0.0)
    elif shutter:
        ev = ev_of_shutter(shutter)
    else:
        return None
    bp.set_attribute("exposure_mode", "manual")
    for k, v in (("fstop", "1.4"), ("iso", "100"), ("shutter_speed", f"{shutter_for_ev(ev):.2f}")):
        if bp.has_attribute(k):
            bp.set_attribute(k, v)
    return ev


def to_rgb(image):
    """A carla.Image (BGRA) as an H x W x 3 RGB numpy array."""
    import numpy as np
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    return arr[:, :, 2::-1]


def downscale(rgb, size):
    """H x W x 3 uint8 -> size (w, h) with a Lanczos filter (unchanged when it already has that size)."""
    import numpy as np
    if rgb.shape[1] == size[0] and rgb.shape[0] == size[1]:
        return rgb
    from PIL import Image
    return np.asarray(Image.fromarray(np.ascontiguousarray(rgb)).resize(tuple(size), Image.LANCZOS))


def save_frame(image, path, size=None, quality=90):
    """Save a carla.Image (or an RGB array), downscaled to size (w, h) when it was rendered larger. JPEG or PNG
    by the file's suffix. Falls back to CARLA's own save_to_disk when Pillow is missing."""
    try:
        from PIL import Image
    except ImportError:          # pragma: no cover - pillow is in requirements.txt
        image.save_to_disk(str(path))
        return
    rgb = to_rgb(image) if hasattr(image, "raw_data") else image
    if size is not None:
        rgb = downscale(rgb, size)
    import numpy as np
    im = Image.fromarray(np.ascontiguousarray(rgb))
    if str(path).lower().endswith((".jpg", ".jpeg")):
        im.save(str(path), quality=quality)
    else:
        im.save(str(path))


# --- realism guard for screenshots and videos (tools/scenery_shots.py, python -m coupledcarla.look FILES) ------
# A shot of an empty world (white ground to the horizon, no buildings) or a black / blown-out / glaring frame is
# never shown (Pavlo, 2026-11-15). CARLA 0.9.14+ semantic tags:
SCENE_TAGS = {3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 26, 28}   # buildings ... vehicles, static
FIXED_TAGS = {3, 4, 5, 6, 7, 8, 9, 20, 26, 28}       # the same without people and vehicles (an ego car is not a city)
EMPTY_TAGS = {0, 1, 2, 10, 11, 22, 24, 25}           # unlabelled, roads, sidewalks, terrain, sky, other, ground
LIMITS = {"scene": 0.10, "fixed": 0.05, "empty": 0.80, "dark": 35.0, "bright": 205.0, "white": 0.05, "black": 0.30,
          "glow": 0.06}       # glow: the README's glaring passenger view had 16 %, eHMI pedestrian views 0.1-1.7 %,
                              # colourful street scenes and a sunset sky up to 1.5 %


def semantic_shares(tags):
    """{scene, fixed, empty}: shares of the frame (an H x W array of semantic tags) covered by each group."""
    import numpy as np
    t = np.asarray(tags)
    n = float(t.size) or 1.0
    return {"scene": float(np.isin(t, list(SCENE_TAGS)).sum()) / n,
            "fixed": float(np.isin(t, list(FIXED_TAGS)).sum()) / n,
            "empty": float(np.isin(t, list(EMPTY_TAGS)).sum()) / n}


def image_stats(rgb, tags=None):
    """{mean, white, black, glow} of an H x W x 3 uint8 image: mean luminance, clipped-white and crushed-black
    shares, and the share of bright, strongly coloured pixels (max channel >= 180, max - min >= 80: an eHMI or a
    debug drawing glaring with its bloom), outside the sky when the semantic tags are given."""
    import numpy as np
    a = np.asarray(rgb).astype(np.int16)
    lum = 0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]
    mx, mn = a.max(-1), a.min(-1)
    glow = (mx >= 180) & (mx - mn >= 80)
    if tags is not None:
        glow &= np.asarray(tags) != 11
    return {"mean": float(lum.mean()), "white": float((mn >= 250).mean()), "black": float((mx <= 8).mean()),
            "glow": float(glow.mean())}


def semantic_problems(tags, limits=None):
    """Why the frame shows an empty world, from its semantic tags (H x W, any size): [] when it does not."""
    lim = {**LIMITS, **(limits or {})}
    out = []
    s = semantic_shares(tags)
    if s["scene"] < lim["scene"]:
        out.append(f"empty world: buildings, trees, poles, signs and vehicles cover {100 * s['scene']:.0f}% "
                   f"of the frame (< {100 * lim['scene']:.0f}%)")
    elif s["fixed"] < lim["fixed"]:
        out.append(f"empty world: buildings, trees, poles and signs cover {100 * s['fixed']:.1f}% "
                   f"(< {100 * lim['fixed']:.0f}%; the rest is vehicles)")
    if s["empty"] > lim["empty"]:
        out.append(f"empty world: road, ground and sky fill {100 * s['empty']:.0f}% (> {100 * lim['empty']:.0f}%)")
    return out


def image_problems(rgb, tags=None, limits=None):
    """Why the frame is too dark, too bright, clipped, crushed or glaring: [] when it is not. tags (the same
    size as rgb) keep the sky out of the glare count."""
    import numpy as np
    lim = {**LIMITS, **(limits or {})}
    if tags is not None and np.asarray(tags).shape != np.asarray(rgb).shape[:2]:
        tags = None
    st = image_stats(rgb, tags)
    out = []
    if st["mean"] < lim["dark"]:
        out.append(f"too dark: mean luminance {st['mean']:.0f} (< {lim['dark']:.0f})")
    if st["mean"] > lim["bright"]:
        out.append(f"too bright: mean luminance {st['mean']:.0f} (> {lim['bright']:.0f})")
    if st["white"] > lim["white"]:
        out.append(f"overexposed: {100 * st['white']:.1f}% clipped white (> {100 * lim['white']:.0f}%)")
    if st["black"] > lim["black"]:
        out.append(f"underexposed: {100 * st['black']:.0f}% crushed black (> {100 * lim['black']:.0f}%)")
    if st["glow"] > lim["glow"]:
        out.append(f"glare: {100 * st['glow']:.1f}% bright saturated pixels (> {100 * lim['glow']:.0f}%; "
                   f"an over-bright eHMI or debug drawing)")
    return out


def realism_problems(rgb, tags=None, limits=None):
    """Why a frame should not be shown ([] when it may): an empty world (semantic tags given), too dark, too bright,
    clipped, crushed or glaring."""
    return (semantic_problems(tags, limits) if tags is not None else []) + image_problems(rgb, tags, limits)


def main(argv=None):
    """python -m coupledcarla.look FRAME.jpg ...: the realism guard's verdict on saved frames (no semantics)."""
    import sys
    from PIL import Image
    import numpy as np
    bad = 0
    for f in (argv if argv is not None else sys.argv[1:]):
        probs = realism_problems(np.asarray(Image.open(f).convert("RGB")))
        bad += bool(probs)
        print(f"{f}: {'; '.join(probs) if probs else 'ok'}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
