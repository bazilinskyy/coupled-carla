"""Smoothed SUMO paths (bridge/animate.py), smoother nets (netsmooth.py), the clip QA (clipcheck.py) and the steadied
follow camera (film.steady_step). No CARLA, no SUMO binary."""
import csv
import math

from coupledcarla import clipcheck, film, netsmooth
from coupledcarla.bridge import animate


def kinked_path(v=10.0, dt=0.02, shift=6.0, at=40.0, over=6.0, length=4.9):
    """Body-centre poses of a car at v m/s along x whose lane jumps `shift` m sideways over `over` m at x = at."""
    out, x = [], 0.0
    while x < 120.0:
        u = min(1.0, max(0.0, (x - at) / over))
        y = shift * u
        slope = shift / over if 0.0 < u < 1.0 else 0.0
        out.append((x, y, math.degrees(math.atan(slope))))
        x += v * dt
    return out


def lateral_peak(poses, dt=0.02):
    series = [(i * dt, p[0], p[1], p[2]) for i, p in enumerate(poses)]
    return max((d[1] for d in clipcheck.dynamics(series) if d[3] > 2.0), default=0.0)


def test_participant_smoother_turns_a_lane_jump_into_a_curve():
    raw = kinked_path()
    sm = animate.PathSmoother.participant()
    out = [sm.step(p, 10.0, 4.9, 0.02)[:3] for p in raw]
    assert lateral_peak(raw) > 15.0
    assert lateral_peak(out) < 5.0
    assert sm.snaps == 0
    assert abs(out[-1][1] - raw[-1][1]) < 0.3          # back on SUMO's path afterwards


def test_traffic_smoother_stays_close_and_aims_ahead():
    raw = kinked_path(shift=1.5, over=3.0)
    shapes = [[(0.0, 0.0), (40.0, 0.0), (43.0, -1.5), (200.0, -1.5)]]     # SUMO coordinates (y up)
    sm = animate.PathSmoother()
    out = []
    for p in raw:
        q = animate.path_point(shapes, p[0] + 2.45, animate.look_ahead(10.0))
        out.append(sm.step(p, 10.0, 4.9, 0.02, aim=(q[0], -q[1]))[:3])
    dev = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(raw, out))
    assert dev < 1.2
    assert lateral_peak(out) < lateral_peak(raw)


def test_slow_traffic_shows_sumo_pose():
    sm = animate.PathSmoother()
    assert sm.step((1.0, 2.0, 30.0), 0.5, 4.5, 0.02)[:3] == (1.0, 2.0, 30.0) or \
        math.isclose(sm.step((1.0, 2.0, 30.0), 0.5, 4.5, 0.02)[2], 30.0, abs_tol=1e-6)


def test_path_point_follows_the_next_lanes_and_shifts_left():
    shapes = [[(0, 0), (10, 0)], [(10, 0), (10, 10)]]
    assert animate.path_point(shapes, 8.0, 4.0) == (10.0, 2.0)
    x, y = animate.path_point(shapes, 2.0, 1.0, lat=0.5)
    assert math.isclose(x, 3.0) and math.isclose(y, 0.5)
    assert animate.path_point([], 0, 1) is None


def test_trailer_cuts_inside_a_turn():
    tr = animate.Trailer()
    # the front drives a quarter circle of radius 10 m; a 10 m bus's rear axle must run inside it
    pts = [(10 * math.sin(a / 100.0 * math.pi / 2), 10 - 10 * math.cos(a / 100.0 * math.pi / 2)) for a in range(101)]
    tr.step((pts[0][0] - 0.1, 0.0), 0.0, 10.0)
    for p in pts:
        tr.step(p, 0.0, 10.0)
    r = math.hypot(tr.rx - 0.0, tr.ry - 10.0)
    assert r < 9.5


def test_fillet_and_bezier_have_no_kinks():
    sh = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0)]
    f = netsmooth.fillet(sh)
    turns = [abs(math.degrees(netsmooth._heading(b, c) - netsmooth._heading(a, b))) for a, b, c in zip(f, f[1:], f[2:])]
    assert max(turns) < 20.0
    corner = min(math.dist(p, (20.0, 0.0)) for p in f)
    assert corner <= netsmooth.FILLET_MAX_M + 0.05
    assert min(math.dist(a, b) for a, b in zip(f, f[1:])) >= 0.05
    bz = netsmooth.bezier((0.0, 0.0), (1.0, 0.0), (10.0, 10.0), (0.0, 1.0))
    assert bz[0] == (0.0, 0.0) and math.dist(bz[-1], (10.0, 10.0)) < 1e-9
    assert netsmooth._angle(netsmooth._heading(bz[0], bz[1]), 0.0) < 10.0


def test_plan_drops_a_shift_with_a_straight_alternative_and_slows_one_without():
    base = {"from": "a", "to": "b", "fromLane": 1, "angle": 5.0, "speed": 13.9}
    conns = [dict(base, toLane=1, side=0.2, along=8.8), dict(base, toLane=3, side=6.3, along=7.8),
             dict(base, fromLane=0, toLane=0, side=3.0, along=8.0)]
    deletes, slowed = netsmooth.plan(conns)
    assert [c["toLane"] for c in deletes] == [3]
    assert len(slowed) == 1 and slowed[0]["new_speed"] < 13.9
    assert netsmooth.min_radius([(0, 0), (10, 0)]) is None
    r = netsmooth.min_radius([(10 * math.cos(a / 50), 10 * math.sin(a / 50)) for a in range(80)])
    assert 9.0 < r < 11.0
    assert netsmooth.curve_speed(10.0, 13.9) == round(math.sqrt(netsmooth.CURVE_ACC * 10.0), 2)


def test_steady_step_follows_a_steady_target_and_damps_jitter():
    p, v = (0.0, 0.0, 0.0), (1.4, 0.0, 0.0)
    worst = 0.0
    for i in range(1, 300):
        jitter = 0.1 if i % 7 == 0 else 0.0
        target = (1.4 * i * 0.02, jitter, 0.0)
        p, v = film.steady_step(p, v, target, (1.4, 0.0, 0.0), 0.02, 0.25)
        worst = max(worst, abs(p[1]))
    assert abs(p[0] - 1.4 * 299 * 0.02) < 0.05
    assert worst < 0.03


COLS = ["timestamp", "frame", "actor_id", "kind", "role", "type_id", "pos_x", "pos_y", "pos_z", "rot_pitch",
        "rot_yaw", "rot_roll", "lat", "lon", "vel_x", "vel_y", "vel_z", "speed_kmh"]


def write_trial(tmp_path, rows, cam=None, log=""):
    (tmp_path / "sumo").mkdir()
    (tmp_path / "sumo" / "sumo.log").write_text(log)
    (tmp_path / "events.csv").write_text("timestamp;frame;event;actor_id;detail\n0.0;0;spawn;7;car x sumo:car7\n")
    with open(tmp_path / "frames.csv", "w", newline="") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(COLS)
        for r in rows:
            w.writerow(r)
    if cam:
        (tmp_path / "film" / "main").mkdir(parents=True)
        with open(tmp_path / "film" / "main" / "camera.csv", "w") as f:
            f.write("t;x;y;z;yaw;pitch\n")
            for c in cam:
                f.write(";".join(str(v) for v in c) + "\n")


def row(t, aid, kind, typ, x, y, yaw, vx=0.0, vy=0.0, role=""):
    return [f"{t:.3f}", int(round(t / 0.02)), aid, kind, role, typ, x, y, 1.0, 0, yaw, 0, 0, 0, vx, vy, 0,
            3.6 * math.hypot(vx, vy)]


def test_clip_check_finds_an_overlap_a_swerve_a_jump_a_teleport_and_jitter(tmp_path):
    rows, cam = [], []
    for i in range(0, 300):
        t = i * 0.02
        # the ego swerves at t 2-2.5 s (2 m sideways in 0.5 s at 10 m/s)
        y = 0.0 if t < 2.0 else (2.0 if t > 2.5 else 4.0 * (t - 2.0))
        rows.append(row(t, "1", "participant", "vehicle.lincoln.mkz_2020", 10 * t, y, 0.0, 10.0, 0.0, "driver"))
        # two cars overlapping in view, one jumping 2 m at t = 4 s
        rows.append(row(t, "7", "car", "vehicle.audi.a2", 20.0, 10.0, 90.0))
        rows.append(row(t, "8", "car", "vehicle.audi.a2", 20.5 + (2.0 if t >= 4.0 else 0.0), 10.0, 90.0))
        cam.append((round(t, 2), 10 * t - 8.0, (0.15 if i % 10 == 0 else 0.0), 3.0, 0.0, -5.0))
    write_trial(tmp_path, rows, cam, "Warning: Teleporting vehicle 'car7'; waited too long (yield), lane='-1_0', "
                                     "time=3.00.\n")
    rep = clipcheck.check_trial(tmp_path, 1.0, 5.0, camera="main", fov=90.0, every=1)
    checks = {p["check"] for p in rep["problems"] if p["in_window"]}
    assert {"overlap", "swerve", "jump", "teleport", "jitter"} <= checks
    assert not rep["ok"]
    assert rep["metrics"]["participants"]["driver"]["max_lat_acc"] > 4.0
    assert rep["metrics"]["sumo"]["teleports_total"] == 1


def test_clip_check_is_quiet_on_smooth_traffic(tmp_path):
    rows, cam = [], []
    for i in range(0, 300):
        t = i * 0.02
        rows.append(row(t, "1", "participant", "vehicle.lincoln.mkz_2020", 10 * t, 0.0, 0.0, 10.0, 0.0, "driver"))
        rows.append(row(t, "7", "car", "vehicle.audi.a2", 10 * t + 15.0, 3.5, 0.0, 10.0, 0.0))
        cam.append((round(t, 2), 10 * t - 8.0, 0.0, 3.0, 0.0, -5.0))
    write_trial(tmp_path, rows, cam)
    rep = clipcheck.check_trial(tmp_path, 1.0, 5.0, camera="main", fov=90.0, every=1)
    assert rep["ok"], rep["problems"]


def test_geometry_helpers():
    a = (0.0, 0.0, 0.0, 2.0, 1.0)
    assert clipcheck.box_clearance(a, (0.0, 3.0, 0.0, 2.0, 1.0)) == 1.0
    assert clipcheck.box_clearance(a, (0.5, 0.5, 0.0, 2.0, 1.0)) == 0.0
    assert clipcheck.side_by_side(a, (0.0, 3.0, 0.0, 2.0, 1.0))
    assert not clipcheck.side_by_side(a, (10.0, 0.0, 0.0, 2.0, 1.0))
    assert clipcheck.in_view((0, 0, 0, 2, 0, 0), 20.0, 1.0, 90.0)
    assert not clipcheck.in_view((0, 0, 0, 2, 0, 0), -20.0, 1.0, 90.0)
    assert clipcheck.point_in_polygon(1, 1, [(0, 0), (2, 0), (2, 2), (0, 2)])
    assert clipcheck.sumo_teleports("Teleporting vehicle 'b'; waited too long (jam), lane='x_0', time=5.50.")[0][0] == 5.5
    r = clipcheck.savgol_residual([0.0, 1.0, 4.0, 9.0, 16.0, 25.0, 36.0, 49.0])
    assert all(abs(v) < 1e-9 for v in r)


def test_driver_mix_draws_within_bounds_and_is_repeatable():
    from coupledcarla import drivermix
    a = drivermix.draw("car", drivermix.rng_for("car7", 42))
    assert a == drivermix.draw("car", drivermix.rng_for("car7", 42))
    for k, spec in drivermix.DRAWS["car"].items():
        assert spec[3] <= a[k] <= spec[4]
    bikes = [drivermix.draw("bike", drivermix.rng_for(f"b{i}", 1)) for i in range(2000)]
    share = sum(b["ebike"] for b in bikes) / len(bikes)
    reds = sum(b["red_runner"] for b in bikes) / len(bikes)
    assert abs(share - drivermix.EBIKE_SHARE) < 0.04 and abs(reds - drivermix.RED_RUNNER_SHARE) < 0.04
    e = [b["max_speed"] * 3.6 for b in bikes if b["ebike"]]
    n = [b["max_speed"] * 3.6 for b in bikes if not b["ebike"]]
    assert sum(e) / len(e) > sum(n) / len(n) + 1.0
    assert drivermix.kind_of("bus") == "bus" and drivermix.kind_of("pedestrian") is None


def test_red_runner_bike_type():
    from coupledcarla import drivers
    xml = drivers.bike_red_runner("vehicle.bh.crossbike", "EIDM")
    assert 'id="vehicle.bh.crossbike#red"' in xml and 'jmDriveAfterRedTime="300"' in xml and "probability" not in xml


def test_tomtom_answer_to_site():
    from coupledcarla import tomtomspeeds
    ans = {"flowSegmentData": {"frc": "FRC3", "currentSpeed": 22, "freeFlowSpeed": 31, "confidence": 0.9}}
    s = tomtomspeeds.parse(ans, 45.76, 4.83, 90.0, "Rue X")
    assert s["v50"] == 31.0 and s["heading"] == 90.0 and s["kind"] == "spot"
    assert tomtomspeeds.parse({"flowSegmentData": {"freeFlowSpeed": 30, "roadClosure": True}}, 0, 0, 0, "") is None
