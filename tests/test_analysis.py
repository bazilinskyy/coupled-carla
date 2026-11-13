"""coupledcarla.analysis on a synthetic trial: a car stops for a crossing pedestrian, then drives on."""
import csv
import math

import numpy as np
import pytest
import yaml

from coupledcarla import analysis

# coupledcarla.logger.COLUMNS (the logger imports carla, so the list is repeated here)
COLUMNS = ["timestamp", "frame", "actor_id", "kind", "role", "type_id", "pos_x", "pos_y", "pos_z",
           "rot_pitch", "rot_yaw", "rot_roll", "lat", "lon", "vel_x", "vel_y", "vel_z", "speed_kmh",
           "acc_x", "acc_y", "acc_z", "blinkers", "front_lights", "stop_lights", "throttle", "steer", "brake",
           "reverse", "hand_brake", "next_tl", "next_tl_dist", "next_tl_state", "ehmi"]
LAT0, LON0 = 51.44, 5.48


def latlon(x, y):        # CARLA x east, y south
    return LAT0 - y / 111194.9, LON0 + x / (111194.9 * math.cos(math.radians(LAT0)))


def car_x(t):
    """-40 m at 10 m/s, brake from t=2 to a stop at x=-6 (t=4.8), wait until t=9, then 2 m/s^2."""
    a = 100 / 28
    if t <= 2:
        return -40 + 10 * t, 10.0
    if t <= 4.8:
        u = t - 2
        return -20 + 10 * u - a * u * u / 2, 10 - a * u
    if t <= 9:
        return -6.0, 0.0
    return -6 + (t - 9) ** 2, 2 * (t - 9)


def ped_y(t):
    """Waits at y=-6 (kerb), crosses at x=0 from t=3 to t=11 at 1.5 m/s."""
    if t < 3:
        return -6.0, 0.0
    if t <= 11:
        return -6 + 1.5 * (t - 3), 1.5
    return 6.0, 0.0


def row(t, frame, aid, kind, role, type_id, x, y, yaw, vx, vy, ehmi=""):
    lat, lon = latlon(x, y)
    r = dict.fromkeys(COLUMNS, "")
    r.update(timestamp=f"{t:.3f}", frame=frame, actor_id=aid, kind=kind, role=role, type_id=type_id, pos_x=x,
             pos_y=y, pos_z=0, rot_pitch=0, rot_yaw=yaw, rot_roll=0, lat=f"{lat:.8f}", lon=f"{lon:.8f}",
             vel_x=vx, vel_y=vy, vel_z=0, speed_kmh=3.6 * math.hypot(vx, vy), ehmi=ehmi)
    return [r[c] for c in COLUMNS]


@pytest.fixture()
def trial(tmp_path):
    d = tmp_path / "20260101-000000_trial3"
    d.mkdir()
    with open(d / "frames.csv", "w", newline="") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(COLUMNS)
        for k in range(0, 1001):                      # 20 s at 50 Hz
            t, frame = k * 0.02, 100 + k
            cx, cv = car_x(t)
            py, pv = ped_y(t)
            ehmi = "yielding" if 2 <= t < 9 else "driving"
            w.writerow(row(t + 7.5, frame, 11, "agent", "av1", "vehicle.tesla.model3", cx, 0.0, 0.0, cv, 0.0, ehmi))
            w.writerow(row(t + 7.5, frame, 12, "participant", "pedestrian", "walker.pedestrian.0001", 0.0, py,
                           90.0, 0.0, pv))
    # events in trial time (frames.csv above uses server time, as logs before 2026-11-14 did)
    with open(d / "events.csv", "w", newline="") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["timestamp", "frame", "event", "actor_id", "detail"])
        w.writerow(["2.000", 200, "ehmi", 11, "av1 yielding (by auto)"])
        w.writerow(["4.000", 300, "eye_contact_start", "", "pedestrian<->av1 20 m, pedestrian 0 deg, vehicle 3 deg"])
        w.writerow(["6.000", 400, "eye_contact_end", "", "pedestrian<->av1 6 m, pedestrian 0 deg, vehicle 3 deg"])
        w.writerow(["9.000", 550, "ehmi", 11, "av1 driving (by auto)"])
    lat, lon = latlon(0.0, -6.0)
    (d / "config.yaml").write_text(yaml.safe_dump({"pois": [
        {"name": "kerb", "lat": lat, "lon": lon, "radius_m": 1.0},
        {"name": "east", "lat": LAT0, "lon": LON0, "heading": 90}]}))
    return d


def test_ttc_box():
    t = analysis.ttc_box(np.array([10.0, 10.0, 0.5, 5.0]), np.array([0.0, 5.0, 0.0, 0.0]),
                         np.array([-2.0, -2.0, 0.0, 0.0]), np.zeros(4), 1.0, 1.0)
    assert t[0] == pytest.approx(4.5)        # head on: (10 - 1) / 2
    assert np.isinf(t[1])                    # passes 5 m aside
    assert t[2] == 0                         # already inside
    assert np.isinf(t[3])                    # not moving


def test_dims():
    assert analysis.dims("vehicle.mitsubishi.fusorosa") == (5.5, 1.3)
    assert analysis.dims("vehicle.gazelle.omafiets", "bike") == (1.0, 0.4)
    assert analysis.dims("vehicle.tesla.model3") == analysis.CAR


def test_encounter(trial):
    rows = analysis.analyse_trial(trial, plots=False, poi_frames=True)
    assert len(rows) == 1
    e = rows[0]
    assert (e["pedestrian"], e["vehicle"], e["trial"]) == ("pedestrian", "av1", 3)
    assert e["t_first"] == 0                                          # put on trial time
    # car front at -3.65 while the pedestrian walks past x=0: 3.65 - 0.3 m
    assert e["min_gap_m"] == pytest.approx(3.35, abs=0.02)
    assert e["stopped"] == 1 and 4.5 < e["t_stop"] < 4.8
    assert e["stop_gap_m"] == pytest.approx(4.2, abs=0.3)
    # the pedestrian leaves the car's box (half width 0.95 + 0.3) at t = 3 + 7.25/1.5 = 7.83 s; the car box
    # reaches x = 0 (front at -0.3) at t = 9 + sqrt(3.35) = 10.83 s
    assert e["first"] == "pedestrian" and e["pet_s"] == pytest.approx(3.0, abs=0.15)
    assert e["t_in_path"] == pytest.approx(3 + 4.25 / 1.5, abs=0.05)   # |y| <= 0.95 + 0.3 + 0.5 = 1.75
    assert e["ehmi_in_path"] == "yielding"
    assert e["eye_contact_s"] == pytest.approx(2.0) and e["t_eye_contact"] == pytest.approx(4.0)
    assert e["ehmi_states"] == "2.00:yielding|9.00:driving"
    assert e["collision"] == 0
    assert e["min_ttc_s"] is not None and e["min_ttc_s"] > 0
    out = trial / "analysis"
    enc = list(csv.DictReader(open(out / "encounters.csv"), delimiter=";"))
    assert len(enc) == 1 and enc[0]["vehicle"] == "av1"
    poi = {(r["poi"], r["actor"]): r for r in csv.DictReader(open(out / "poi.csv"), delimiter=";")}
    assert float(poi[("kerb", "pedestrian")]["min_dist_m"]) < 0.1
    assert float(poi[("kerb", "pedestrian")]["t_within"]) == 0
    assert float(poi[("kerb", "pedestrian")]["t_min_dist"]) < 3.01
    frames = list(csv.DictReader(open(out / "frames_poi.csv"), delimiter=";"))
    east = [r for r in frames if r["poi"] == "east" and r["actor"] == "pedestrian"]
    # heading 90 (east): forward is CARLA x, left is north (CARLA -y)
    assert float(east[0]["x_forward"]) == pytest.approx(0.0, abs=0.05)
    assert float(east[0]["y_left"]) == pytest.approx(6.0, abs=0.05)


def test_far_pair_is_skipped(trial):
    rows = analysis.analyse_trial(trial, radius=1.0, plots=False)
    assert rows == []


def test_experiment_folder(trial, capsys):
    other = trial.parent / "20260101-000100_trial4"
    other.mkdir()
    for name in ("frames.csv", "events.csv"):
        (other / name).write_text((trial / name).read_text())
    analysis.main([str(trial.parent), "--no-plots"])
    table = list(csv.DictReader(open(trial.parent / "analysis.csv"), delimiter=";"))
    assert [r["trial"] for r in table] == ["3", "4"]
    assert "pedestrian / av1" in capsys.readouterr().out
