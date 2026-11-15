"""Demo-video tool: clip specs, derived experiments, ffmpeg commands, sim-time frame grids, offline sound (no CARLA,
no ffmpeg)."""
import csv
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("make_videos", ROOT / "tools" / "make_videos.py")
mv = importlib.util.module_from_spec(spec)
sys.modules["make_videos"] = mv
spec.loader.exec_module(mv)

from coupledcarla import film, filmaudio  # noqa: E402

SERVER = {"host": "127.0.0.1", "port": 2500, "launch": True, "quality": "Epic", "tm_port": 2550}


def test_clip_list_parses():
    clips = mv.load_clips()
    assert 8 <= len(clips) <= 14
    names = [c["name"] for c in clips]
    assert len(names) == len(set(names))
    for c in clips:
        assert (ROOT / c["experiment"]).exists(), c["experiment"]
        assert 15 <= c["duration_s"] <= 25, c["name"]
        assert c["frames"] == round(c["duration_s"] * c["fps"])
        assert c["kind"] in ("film", "stations")


def test_bad_clips_are_refused(tmp_path):
    p = tmp_path / "v.yaml"
    p.write_text("clips:\n  - {name: a, experiment: experiments/eindhoven_centre.yaml}\n")
    with pytest.raises(ValueError, match="needs a camera"):
        mv.load_clips(p)
    p.write_text("clips:\n  - {name: a, experiment: x.yaml, camera: {kind: tripod}}\n"
                 "  - {name: a, experiment: x.yaml, camera: {kind: tripod}}\n")
    with pytest.raises(ValueError, match="twice"):
        mv.load_clips(p)


def test_derived_experiment():
    clips = {c["name"]: c for c in mv.load_clips()}
    c = clips["eindhoven_rush_hour"]
    exp = mv.derive_experiment(c, SERVER)
    assert exp["name"] == "video_eindhoven_rush_hour"
    assert exp["map"]["name"] == "eindhoven_centre"            # the base's map, not the clip's name
    assert exp["server"]["port"] == 2500 and exp["server"]["launch"] is True
    assert exp["simulation"]["time"] == c["time"] and exp["simulation"]["weather"] == c["weather"]
    assert exp["simulation"]["real_time"] is False
    assert exp["traffic"]["cars_per_hour"] == 3600 and exp["traffic"]["bikes_per_hour"] == 2400
    assert exp["logging"]["carla_recorder"] is False and exp["logging"]["frames_every_s"] == 0
    f = exp["film"]
    assert f["start_s"] == c["start_s"] and f["duration_s"] == c["duration_s"] and f["fps"] == 25
    assert f["hold"] == c["hold"] and f["camera"] == c["camera"]
    assert exp["trials"] == [{"duration_s": c["start_s"] + c["duration_s"] + c["tail_s"]}]


def test_derived_experiment_folds_trial_and_stations():
    c = mv.load_clips()[0]
    c = dict(c, trial=1, set={}, time=None, weather=None)      # eindhoven_centre's dusk trial
    exp = mv.derive_experiment(c, SERVER)
    assert exp["simulation"]["weather"] == "WetCloudySunset" and exp["simulation"]["sun_altitude"] == 4
    assert exp["traffic"]["cars_per_hour"] == 3600
    st = {x["name"]: x for x in mv.load_clips()}["crossing_four_views"]
    exp = mv.derive_experiment(st, dict(SERVER, launch=False))
    assert "camera" not in exp["film"] and exp["film"]["attributes"]["motion_blur_intensity"] == 0
    assert exp["coupling"]["port"] == 2510 and exp["server"]["launch"] is False
    assert exp["simulation"]["real_time"] is True
    assert {p["role"] for p in exp["participants"] if p.get("station")} == {v["role"] for v in st["stations"]["views"]}
    rt = {x["name"]: x for x in mv.load_clips()}["robotaxi_passenger"]
    assert mv.derive_experiment(rt, SERVER)["trials"][0]["patch"]["participants"]["driver"]["station"] is False


def test_derived_experiments_have_known_keys():
    from coupledcarla import config, validate
    for c in mv.load_clips():
        raw = mv.derive_experiment(c, SERVER)
        rep = validate.Report() if hasattr(validate, "Report") else None
        if rep is None:
            pytest.skip("validator without Report")
        validate._keys(rep, raw, config.DEFAULTS)
        bad = [i for i in rep.items if "unknown key" in i[2]]
        assert not bad, (c["name"], bad)


def test_encode_command():
    cmd = mv.encode_cmd("f/%06d.jpg", "o.mp4", 25, 27, audio="a.wav", audio_ss=40.0, duration=20, scale=(960, 540),
                        filters=["drawtext=x"])
    s = " ".join(cmd)
    assert cmd[:2] == ["ffmpeg", "-y"]
    assert "-framerate 25 -start_number 0 -i f/%06d.jpg" in s
    assert "-ss 40.000 -t 20.000 -i a.wav" in s           # the sound trimmed to the clip's start
    assert "scale=960:540:flags=lanczos,drawtext=x,format=yuv420p" in s
    for part in ("-c:v libx264", "-preset slow", "-crf 27", "-pix_fmt yuv420p", "-movflags +faststart",
                 "-c:a aac", "-b:a 96k", "-ac 2", "-shortest"):
        assert part in s
    silent = mv.encode_cmd("f/%06d.jpg", "o.mp4", 25, 28)
    assert "-an" in silent and "aac" not in " ".join(silent)


def test_poster_preview_remux_commands():
    assert "scale=640:-2:flags=lanczos" in " ".join(mv.poster_cmd("a.jpg", "p.jpg"))
    pv = " ".join(mv.preview_cmd("f/%06d.jpg", "p.webp", 25, 200, 3.5))
    assert "-start_number 200" in pv and "-t 3.5" in pv and "libwebp" in pv and "-loop 0" in pv
    rm = " ".join(mv.remux_cmd("v.mp4", "a.wav", "o.mp4", 30.0, 20))
    assert "-c:v copy" in rm and "-ss 30.000 -i a.wav" in rm


def test_caption_filter():
    f = mv.caption_filter("/t/title.txt", "/t/line.txt", 720, "/f/Sans.ttf", "/f/Bold.ttf", 7.0)
    assert len(f) == 2
    assert f[0].startswith("drawtext=fontfile='/f/Bold.ttf':textfile='/t/title.txt'")
    assert "expansion=none" in f[0] and "fontsize=26" in f[0] and "fontsize=18" in f[1]
    assert "if(lt(t,7)" in f[0]                                  # fades out after 7 s
    assert "fontsize=13" in mv.caption_filter("a", None, 360, "x", None)[0]
    assert mv.caption_lines({"name": "n", "caption": False}) is None
    assert mv.caption_lines({"name": "n", "title": "T", "place": "P", "role": "R", "caption": True}) == ("T", "P  ·  R")


def test_grid_index_and_frame_check():
    step, fps, ts0 = 0.02, 25.0, 1234.56
    kept = {}
    for i in range(400):                 # every tick renders; floating-point timestamps
        ts = ts0 - 1.0 + i * step + 1e-9 * (i % 7)
        k = film.grid_index(ts, ts0, fps, step)
        if k is not None and 0 <= k < 100 and k not in kept:
            kept[k] = (1000 + i, ts - ts0)
    assert sorted(kept) == list(range(100))
    chk = film.check_frames([(k, v[0], v[1]) for k, v in kept.items()], fps, step, 100)
    assert chk["even"] and chk["missing"] == [] and chk["frame_steps"] == [2]
    del kept[50]
    chk = film.check_frames([(k, v[0], v[1]) for k, v in kept.items()], fps, step, 100)
    assert chk["missing"] == [50]
    assert film.grid_index(ts0 + 0.02, ts0, fps, step) is None     # half-way between grid points: not kept


def test_camera_paths():
    c = (10.0, 20.0, 5.0)
    p0 = film.orbit_point(c, 30, 12, 0, 10, 0)
    assert p0 == pytest.approx((10.0, -10.0, 17.0))           # compass 0: north = CARLA -y
    p9 = film.orbit_point(c, 30, 12, 0, 10, 9)                 # 90 degrees later: east = +x
    assert p9[0] == pytest.approx(40.0) and p9[1] == pytest.approx(20.0)
    keys = [(0.0, (0.0, 0.0, 50.0)), (10.0, (100.0, 0.0, 20.0)), (20.0, (100.0, 100.0, 10.0))]
    assert film.path_point(keys, -1) == (0.0, 0.0, 50.0) and film.path_point(keys, 25) == (100.0, 100.0, 10.0)
    xs = [film.path_point(keys, t)[0] for t in np.linspace(0, 10, 30)]
    assert all(b >= a - 1e-6 for a, b in zip(xs, xs[1:]))      # goes one way
    assert film.offset_point((0, 0, 0), 0, -6, 0, 2) == pytest.approx((-6, 0, 2))
    assert film.offset_point((0, 0, 0), 90, 0, 1, 0) == pytest.approx((-1, 0, 0))   # right of yaw 90 (south) is west
    pitch, yaw = film.look_rotation((0, 0, 10), (10, 10, 0))
    assert yaw == pytest.approx(45) and pitch == pytest.approx(-math.degrees(math.atan2(10, math.hypot(10, 10))))
    assert film.angle_lerp(350, 10, 0.5) == pytest.approx(360)
    assert film.ease(0) == 0 and film.ease(1) == 1 and film.ease(0.5) == 0.5


def test_film_spec_names_cameras():
    s = film.spec({"camera": {"kind": "tripod"}, "cameras": [{"kind": "orbit"}], "duration_s": 2, "fps": 25})
    assert [c["name"] for c in s["cameras"]] == ["main", "cam1"] and s["frames"] == 50


def test_station_plan_aligns_parity():
    frame0, step, fps = 1000, 0.02, 25
    odd = list(range(1301, 1900, 2))                      # the stations' common grid fell on odd frames
    views = {"a": odd, "b": [f for f in odd if f != 1311]}
    plan, start = mv.station_plan(views, frame0, 6.0, 20, fps, step)
    assert start == 1301 and plan["a"][:3] == [1301, 1303, 1305]
    assert plan["b"][5] is None and plan["b"][6] == 1313


def test_fill_gaps(tmp_path):
    for k in (0, 1, 4):
        (tmp_path / f"{k:06d}.jpg").write_bytes(bytes([k]))
    assert mv.fill_gaps(tmp_path, 6) == [2, 3, 5]
    assert (tmp_path / "000003.jpg").read_bytes() == bytes([1]) and (tmp_path / "000005.jpg").read_bytes() == bytes([4])


def test_compose_grid(tmp_path):
    Image = pytest.importorskip("PIL.Image")
    folders = {}
    for i, role in enumerate(("a", "b")):
        d = tmp_path / role
        d.mkdir()
        Image.new("RGB", (64, 36), (200 * i, 50, 50)).save(d / "00000010.jpg")
        folders[role] = d
    plan = {"a": [10, None], "b": [10, 10]}
    rep = mv.compose_grid(plan, folders, {"a": "A"}, tmp_path / "out", (128, 36), (2, 1))
    assert rep == {"a": 1, "b": 0}
    im = Image.open(tmp_path / "out" / "000001.jpg")
    assert im.size == (128, 36)


def test_trial_frame0(tmp_path):
    p = tmp_path / "frames.csv"
    p.write_text("timestamp;frame;actor_id\n0.100;205;1\n")
    assert mv.trial_frame0(p, 0.02) == 200


# -- offline sound -----------------------------------------------------------------------------------------------

def test_samples_per_tick_no_drift():
    times = [i * 0.02 for i in range(100)]
    n = filmaudio.samples_per_tick(times, 0.0, 44100)
    assert sum(n) == 2 * 44100 and set(n) == {882}
    n = filmaudio.samples_per_tick([0.0, 0.0333, 0.0667, 0.1], 0.0, 44100)
    assert sum(n) == round((0.1 + 0.0333) * 44100)


def test_listener_interpolation():
    track = [(0.0, 0, 0, 0, 350.0), (1.0, 10, 0, 2, 10.0)]
    x, y, z, yaw = filmaudio.listener_at(track, 0.5)
    assert (x, z) == (5, 1) and yaw == pytest.approx(360.0)
    assert filmaudio.listener_at(track, 5)[0] == 10


def _frames_csv(path, seconds=3.0, step=0.02):
    cols = ["timestamp", "frame", "actor_id", "kind", "role", "type_id", "pos_x", "pos_y", "pos_z", "rot_pitch",
            "rot_yaw", "rot_roll", "lat", "lon", "vel_x", "vel_y", "vel_z", "speed_kmh", "acc_x", "acc_y", "acc_z",
            "blinkers", "front_lights", "stop_lights", "throttle", "steer", "brake", "reverse", "hand_brake",
            "next_tl", "next_tl_dist", "next_tl_state", "ehmi"]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh, delimiter=";")
        w.writerow(cols)
        for i in range(int(seconds / step)):
            t = i * step
            row = dict.fromkeys(cols, "")
            # a car passing 4 m to the listener's right (+y), at 50 km/h along x
            row.update(timestamp=f"{t:.3f}", frame=100 + i, actor_id=7, kind="car", type_id="vehicle.audi.a2",
                       pos_x=f"{-20 + 13.9 * t:.3f}", pos_y="4", pos_z="0", rot_yaw="0", vel_x="13.9", vel_y="0",
                       vel_z="0", blinkers="0")
            w.writerow([row[c] for c in cols])


def test_offline_render(tmp_path):
    p = tmp_path / "frames.csv"
    _frames_csv(p)
    listener = [(0.0, 0.0, 0.0, 1.6, 0.0), (3.0, 0.0, 0.0, 1.6, 0.0)]     # facing +x: the car passes on the right
    x = filmaudio.render(p, 0.5, 2.5, listener=listener, sound={"ambience": {"db": 0}})
    assert x.shape == (2 * 44100, 2)
    rms = np.sqrt((x ** 2).mean(axis=0))
    assert rms.max() > 1e-4                 # the car is heard
    assert rms[1] > rms[0]                  # on the right
    y = filmaudio.normalise(x, -20.0)
    assert 20 * math.log10(float(np.sqrt((y.astype(float) ** 2).mean()))) == pytest.approx(-20.0, abs=1.0)
    assert np.abs(y).max() <= 1.0


def test_hears_first_participant():
    h = mv.hears_first_participant
    assert h({"kind": "stations"})
    assert h({"kind": "film", "camera": {"kind": "drone", "view": "fpv"}})
    assert not h({"kind": "film", "camera": {"kind": "drone", "view": "chase"}})
    assert h({"kind": "film", "camera": {"kind": "follow", "offset": [0.22, 0, 0.72]}})
    assert not h({"kind": "film", "camera": {"kind": "follow", "offset": [-8.5, 0, 3.4]}})
    assert not h({"kind": "film", "camera": {"kind": "orbit"}})
