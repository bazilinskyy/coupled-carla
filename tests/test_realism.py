"""Realism check (coupledcarla/realism.py): parsing the street-level photo sources, the cache and its fail-soft,
choosing the views, the camera geometry (panorama crops, the alignment search) and the metrics. No network, no
CARLA, no segmentation model."""
import json
import math
from datetime import datetime, timezone

import numpy as np
import pytest

from coupledcarla import realism as R

BOX = [51.4360, 5.4700, 51.4500, 5.4930]


def pmx_item(pid="a", lat=51.443, lon=5.48, az=90, fov=360, model="Max", when="2024-10-08T12:35:56+00:00", seq="s1"):
    return {"id": pid, "collection": seq, "geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": {"view:azimuth": az, "datetime": when, "license": "CC-BY-SA-4.0",
                           "pers:interior_orientation": {"camera_manufacturer": "GoPro", "camera_model": model,
                                                         "field_of_view": fov},
                           "exif": {"Exif.Photo.PixelXDimension": "4000", "Exif.Photo.PixelYDimension": "3000"},
                           "quality:horizontal_accuracy": 4.0},
            "assets": {"hd": {"href": "https://x/hd.jpg"}, "sd": {"href": "https://x/sd.jpg"}},
            "providers": [{"name": "Someone"}], "links": []}


def test_panoramax_kartaview_commons_records():
    r = R.panoramax_record(pmx_item())
    assert (r["kind"], r["projection"], r["heading"], r["licence"]) == ("pano", "equirect", 90.0, "CC BY-SA 4.0")
    assert R.panoramax_record(pmx_item(fov=92, model="HERO7 Black"))["kind"] == "action"
    phone = pmx_item(fov=None, model="iPhone")
    phone["properties"]["pers:interior_orientation"]["camera_manufacturer"] = "Apple"
    phone["properties"]["exif"]["Exif.Photo.FocalLengthIn35mmFilm"] = "26"
    p = R.panoramax_record(phone)
    assert p["kind"] == "phone" and 60 < p["fov"] < 72
    nohead = pmx_item()
    del nohead["properties"]["view:azimuth"]
    assert R.panoramax_record(nohead) is None
    k = R.kartaview_record({"id": 7, "lat": "51.44", "lng": "5.48", "heading": "327.7", "fieldOfView": "0",
                            "projection": "PLANE", "width": "4000", "height": "3000", "shotDate": "2019-03-13 15:33:29.000",
                            "fileurlProc": "https://k/p.jpg", "sequenceId": "1", "sequenceIndex": "2"})
    assert (k["fov"], k["heading"], k["time"], k["projection"]) == (None, 327.7, "2019-03-13T15:33:29", "pinhole")
    page = {"pageid": 5, "title": "File:x.jpg", "coordinates": [{"lat": 51.44, "lon": 5.48}],
            "imageinfo": [{"mime": "image/jpeg", "width": 4000, "height": 3000, "thumburl": "https://c/t.jpg",
                           "commonmetadata": [{"name": "FocalLengthIn35mmFilm", "value": 28},
                                              {"name": "DateTimeOriginal", "value": "2013:05:04 14:03:00"}],
                           "extmetadata": {"LicenseShortName": {"value": "CC BY-SA 3.0"},
                                           "Artist": {"value": "<a href='u'>A</a>"}}}],
            "revisions": [{"slots": {"main": {"*": "{{Location|51|26|0|N|5|28|0|E|heading:NE}}"}}}]}
    c = R.commons_record(page)
    assert (c["heading"], c["licence"], c["author"], c["time"]) == (45.0, "CC BY-SA 3.0", "A", "2013-05-04T14:03:00")
    assert 60 < c["fov"] < 70
    assert R.commons_record(page, (51.441, 5.481), 200.0)["heading"] == 200.0     # structured data wins
    page["revisions"][0]["slots"]["main"]["*"] = "no template"
    assert R.commons_record(page) is None


def test_fetch_caches_and_fails_soft(tmp_path):
    calls = []

    def ok(bbox):
        calls.append(bbox)
        return [R.panoramax_record(pmx_item())]

    def broken(bbox):
        raise OSError("offline")
    lines = []
    recs = R.fetch(tmp_path, BOX, ("panoramax",), fetchers={"panoramax": ok}, log=lines.append)
    assert len(recs) == 1 and (tmp_path / "real" / "streetview_panoramax.json").exists()
    assert R.fetch(tmp_path, BOX, ("panoramax",), fetchers={"panoramax": broken}, log=lines.append) == recs
    assert len(calls) == 1                         # second time from the cache
    again = R.fetch(tmp_path, BOX, ("panoramax",), refresh=True, fetchers={"panoramax": broken}, log=lines.append)
    assert again == recs and "fetch failed" in lines[-1]
    assert R.fetch(tmp_path, BOX, ("kartaview",), fetchers={"kartaview": broken}, log=lines.append) == []


def test_times_and_select():
    assert R.utc_time("2024-10-08T14:35:56+00:00", 5.48).hour == 14
    assert R.utc_time("2019-07-01T12:00:00", 5.48).hour == 10          # CEST
    assert R.utc_time("2019-01-10T12:00:00", 5.48).hour == 11          # CET
    assert R.local_time(datetime(2024, 10, 8, 12, 0, tzinfo=timezone.utc), 5.48).hour == 14
    recs = [R.panoramax_record(pmx_item("a", lon=5.4800)),
            R.panoramax_record(pmx_item("b", lon=5.48005)),               # 3.5 m on: thinned
            R.panoramax_record(pmx_item("c", lon=5.4850)),
            R.panoramax_record(pmx_item("n", lon=5.4900, when="2024-09-29T20:24:29+00:00")),   # night
            R.panoramax_record(pmx_item("h", lon=5.4860, fov=92, model="HERO7 Black")),
            R.panoramax_record(pmx_item("o", lat=51.46))]                  # outside
    views = R.select(recs, BOX, spacing_m=20, log=lambda m: None)
    assert sorted(v["src_id"] for v in views) == ["a", "c"]
    v = views[0]
    assert (v["hfov"], v["width"], v["height"], v["height_m"]) == (90.0, 1024, 576, R.HEIGHTS["pano"])
    assert v["local_time"] == "2024-10-08 14:35"
    two = R.select(recs[:1], BOX, pano_offsets=(0, 90), log=lambda m: None)
    assert [v["heading"] for v in two] == [90.0, 180.0] and two[0]["id"] != two[1]["id"]


def test_views_csv_roundtrip(tmp_path):
    views = R.select([R.panoramax_record(pmx_item())], BOX, log=lambda m: None)
    R.write_views(tmp_path / "v.csv", views)
    back = R.read_views(tmp_path / "v.csv")
    assert back[0]["id"] == views[0]["id"] and back[0]["heading"] == 90.0 and back[0]["width"] == 1024


def test_equirect_crop_looks_where_asked():
    W, H = 720, 360
    az = np.tile(np.arange(W, dtype=np.int32), (H, 1))         # column index = (azimuth + 180) * 2
    # panorama centred on heading 90; a view at heading 135 sees azimuth +45 from the centre
    out = R.reproject(az, "equirect", 101, 61, 60.0, src_yaw=90.0 - 135.0, nearest=True)
    centre_az = out[30, 50] / 2.0 - 180.0
    assert centre_az == pytest.approx(45.0, abs=0.6)
    left = out[30, 0] / 2.0 - 180.0                              # left edge: 30 deg less
    assert left == pytest.approx(45.0 - 30.0, abs=1.0)


def test_pinhole_identity_and_wide_size():
    rng = np.random.default_rng(0)
    img = rng.integers(0, 7, (60, 80)).astype(np.uint8)
    same = R.reproject(img, "pinhole", 80, 60, 70.0, src_hfov=70.0, nearest=True)
    assert (same == img).mean() > 0.97
    w, h, f = R.wide_size(1024, 576, 90.0, 15.0, 8.0)
    assert 120.0 <= f < 130 and w > 1024 and h > 576
    # the view cut from the wide render at no offset covers it completely
    wide = np.ones((h, w), np.uint8)
    cut = R.reproject(wide, "pinhole", 1024, 576, 90.0, yaw=12, pitch=6, src_hfov=f, nearest=True, fill=R.IGNORE)
    assert (cut == R.IGNORE).mean() == 0


def scene(w=320, h=180, hfov=90.0, yaw=0.0):
    """A synthetic street as group labels: sky above a skyline that varies with azimuth, road below."""
    d = R.pinhole_rays(w, h, hfov, yaw)
    az = np.degrees(np.arctan2(d[..., 0], d[..., 2]))
    el = np.degrees(np.arcsin(d[..., 1]))
    top = 10 + 8 * np.sin(np.radians(az * 4)) + 6 * (az > 20)
    g = np.full((h, w), R.GROUPS.index("building"), np.uint8)
    g[el > top] = R.GROUPS.index("sky")
    g[el < -8] = R.GROUPS.index("road")
    g[(el < -8) & (az < -25)] = R.GROUPS.index("sidewalk")
    g[(el >= -8) & (el < 2) & (np.abs(az - 5) < 4)] = R.GROUPS.index("vegetation")
    return g


def test_metrics_identical_and_different():
    g = scene()
    m = R.compare_labels(g, g, 90.0)
    assert m["pix_acc"] == 1.0 and m["miou"] == 1.0 and m["layout_tv"] == 0.0 and m["skyline_mae_deg"] == 0.0
    sky = np.full_like(g, R.GROUPS.index("sky"))
    d = R.compare_labels(g, sky, 90.0)
    assert d["miou"] < 0.5 and d["layout_tv"] > 0.4 and math.isnan(d["skyline_mae_deg"])
    ign = g.copy()
    ign[:, :100] = R.IGNORE                                    # ignored pixels count for neither
    assert R.compare_labels(ign, g, 90.0)["pix_acc"] == 1.0
    sl = R.skyline(g, 90.0)
    assert np.nanmin(sl) > 0 and np.nanmax(sl) < 30


def test_align_recovers_a_heading_error():
    w, h, hfov = 320, 180, 90.0
    ww, wh, wf = R.wide_size(w, h, hfov, 15.0, 8.0)
    wide = scene(ww, wh, wf)
    real = scene(w, h, hfov, yaw=6.0)                          # the photo looks 6 deg right of its heading
    dy, dp, g, m = R.align(real, wide, hfov, w, h, wf, yaw_range=12, pitch_range=6, step=3)
    assert (dy, dp) == (6.0, 0.0) and m["miou"] > 0.95
    nominal = R.compare_labels(real, R.reproject(wide, "pinhole", w, h, hfov, src_hfov=wf, nearest=True,
                                                 fill=R.IGNORE), hfov)
    assert nominal["miou"] < m["miou"]


def test_group_tables_and_image_stats():
    cs = np.array([[0, 2, 10, 11, 13, 9]], np.uint8)           # road building sky person car terrain
    assert list(R.groups_of(cs)[0]) == [0, 2, 5, R.IGNORE, 6, 4]
    carla = np.array([[1, 24, 3, 11, 12, 14]], np.uint8)       # roads roadline building sky pedestrian car
    assert list(R.groups_of(carla, R.CARLA_TO_GROUP)[0]) == [0, 0, 2, 5, R.IGNORE, 6]
    grey = np.full((20, 20, 3), 128, np.uint8)
    s = R.image_stats(grey)
    assert s["sat_mean"] == 0 and s["edge_density"] == 0 and s["lum_mean"] == pytest.approx(128, abs=0.5)
    assert R.colour_hist_sim(grey, grey) == pytest.approx(1.0)
    red = np.zeros_like(grey)
    red[..., 0] = 255
    assert R.colour_hist_sim(grey, red) == 0.0 and R.image_stats(red)["colourfulness"] > 50
    assert R.colourise(np.array([[0, R.IGNORE]], np.uint8)).tolist() == [[[128, 64, 128], [0, 0, 0]]]


def test_fov_and_compass():
    assert R.fov_from_f35(26, 4032, 3024) == pytest.approx(67.3, abs=0.5)
    assert R.compass("NE") == 45.0 and R.compass("212.5") == 212.5 and R.compass(None) is None
    assert R.metres(51.44, 5.48, 51.441, 5.48) == pytest.approx(111.3, abs=0.2)
    assert json.dumps(R.SOURCES)                                 # licences recorded
