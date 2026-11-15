"""The quadrotor dynamics and flight controller, Dryden turbulence, flocking and mission guidance: coupled-sim's
DroneModelTests (Assets/Tests/EditMode/DroneModelTests.cs, PR #190) with the same expected numbers, plus the
vectorised fleet and the battery model."""
import math

import numpy as np
import pytest

from coupledcarla import dronemodel as dm


def hovering(p, at):
    q = dm.Quadrotor(p, (0.0, 0.0, 0.0), 0.0)
    q.command(mode="position", pos=at)
    for _ in range(1000):      # 20 s
        q.step(0.02)
    return q


def test_presets_match_their_class():
    c = dm.CONSUMER
    assert 550 <= c.hover_w <= 650, "hover rotor speed (rad/s)"
    assert 2.8 <= c.thrust_to_weight <= 3.6
    ind = dm.INDUSTRIAL
    assert 320 <= ind.hover_w <= 380
    assert 3.0 <= ind.thrust_to_weight <= 3.8
    assert dm.preset("Delivery") is dm.INDUSTRIAL and dm.preset("whatever") is dm.CONSUMER


@pytest.mark.parametrize("name", ["Consumer", "Industrial"])
def test_takes_off_and_holds_a_position(name):
    q = hovering(dm.preset(name), (0, 0, 10))
    assert np.linalg.norm(q.pos - (0, 0, 10)) < 0.3
    assert np.linalg.norm(q.vel) < 0.2
    assert not q.on_ground


def test_flies_to_a_target_within_its_envelope():
    p = dm.CONSUMER
    q = hovering(p, (0, 0, 20))
    q.command(mode="position", pos=(80, 0, 20))
    max_speed = max_tilt = 0.0
    for _ in range(1250):      # 25 s
        q.step(0.02)
        max_speed = max(max_speed, float(np.hypot(*q.vel[:2])))
        max_tilt = max(max_tilt, q.tilt)
    assert np.linalg.norm(q.pos - (80, 0, 20)) < 0.5, "there"
    assert max_speed <= p.max_speed + 0.5, "speed limit"
    assert max_tilt <= p.max_tilt + 0.08, "tilt limit"
    assert max_speed > 10, "and not dawdling"


def test_follows_a_velocity_and_a_yaw_rate():
    q = hovering(dm.CONSUMER, (0, 0, 15))
    q.command(mode="velocity", vel=(0, 8, 0), yaw_rate=0.5)
    for _ in range(300):       # 6 s
        q.step(0.02)
    assert q.vel[1] == pytest.approx(8.0, abs=0.3)
    assert q.vel[0] == pytest.approx(0.0, abs=0.3)
    assert abs(float(dm.wrap(q.yaw - 3.0))) < 0.3       # 6 s at 0.5 rad/s: 3 rad, minus the start-up


def test_holds_its_position_in_wind():
    q = hovering(dm.CONSUMER, (0, 0, 30))
    q.wind = (8, 0, 0)
    for _ in range(1500):      # 30 s
        q.step(0.02)
    assert np.linalg.norm(q.pos - (0, 0, 30)) < 0.6
    assert q.tilt > 0.05, "leaning into the wind"


def test_falls_with_the_rotors_off():
    q = hovering(dm.CONSUMER, (0, 0, 50))
    q.command(mode="off")
    for _ in range(50):        # 1 s
        q.step(0.02)
    assert q.vel[2] < -6, "about g, less the drag and the spinning down"
    for _ in range(500):
        q.step(0.02)
    assert q.on_ground
    assert q.pos[2] == pytest.approx(0.0, abs=1e-3)


def test_dryden_gusts_have_their_standard_deviation():
    sh, sv, lh, lv = dm.Dryden.intensities(50.0, 10.0)
    assert sv == pytest.approx(1.0, abs=1e-4), "0.1 W20"
    assert sh > sv, "stronger horizontally near the ground"
    assert lh > lv
    t = dm.Dryden(1, seed=1)
    z = np.array([t.step(0.05, 50.0, 10.0, 10.0)[0, 2] for _ in range(60000)])
    assert math.sqrt(float(np.mean(z * z))) == pytest.approx(1.0, abs=0.1)


def test_braking_curve_is_continuous():
    a, p = 4.0, 3.0
    r0 = a / (p * p)           # where the linear part ends
    assert float(dm.braking(r0 - 1e-4, a, p)) == pytest.approx(float(dm.braking(r0 + 1e-4, a, p)), abs=1e-2)
    assert float(dm.braking(-1.0, a, p)) == 0.0
    assert float(dm.braking(20.0, a, p)) == pytest.approx(math.sqrt(2 * a * 20 - a * a / (p * p)), abs=1e-4)


def test_flock_members_repel_and_align():
    k = dm.FLOCK_COUPLED_SIM
    v0 = dm.flock_velocity(0, [(0, 0, 0), (5, 0, 0)], [(0, 0, 0), (0, 0, 0)], None, k)
    assert v0[0] < 0, "pushed away from a drone 5 m off"
    # far apart, one fast: the other is pulled towards its velocity (friction above the braking curve)
    assert dm.flock_velocity(0, [(0, 0, 0), (0, 20, 0)], [(0, 0, 0), (9, 0, 0)], None, k)[0] > 0
    # towards the migration point at the flock speed
    assert dm.flock_velocity(0, [(0, 0, 0)], [(0, 0, 0)], (0, 100, 0), k)[1] == pytest.approx(k.flock_speed, abs=1e-3)


def test_flies_a_mission_and_stops_at_its_end():
    q = dm.Quadrotor(dm.INDUSTRIAL, (0, 0, 0), 0.0)
    g = dm.Waypoints([(0, 0, 30), (150, 0, 30), (150, 100, 30), (150, 100, 0.5)], cruise=12.0)
    for _ in range(4500):      # up to 90 s
        if g.done:
            break
        target, vel = g.step(q.pos, 0.02)
        q.command(mode="position", pos=target, vel=vel, yaw=0.0)
        q.step(0.02)
    assert g.done, f"mission done (at {q.pos})"
    assert np.linalg.norm(q.pos - (150, 100, 0.5)) < 2.0


# -- beyond coupled-sim ------------------------------------------------------------------------------------------

def test_fleet_steps_drones_independently():
    """Three drones in one fleet fly as each would alone."""
    f = dm.Fleet()
    for i, p in enumerate((dm.CONSUMER, dm.INDUSTRIAL, dm.CONSUMER)):
        f.add(p, (10.0 * i, 0, 0), 0.0)
    f.set_command(0, "position", pos=(0, 0, 10))
    f.set_command(1, "position", pos=(10, 0, 25))
    f.set_command(2, "off")
    for _ in range(1000):
        f.step(0.02)
    assert np.linalg.norm(f.pos[0] - (0, 0, 10)) < 0.3
    assert np.linalg.norm(f.pos[1] - (10, 0, 25)) < 0.3
    assert f.on_ground[2] and f.pos[2, 2] == 0.0
    alone = hovering(dm.CONSUMER, (0, 0, 10))
    assert np.allclose(alone.pos, f.pos[0], atol=1e-6)


def test_attitude_mode_drifts_with_the_wind():
    q = hovering(dm.CONSUMER, (0, 0, 20))
    q.wind = (5, 0, 0)
    q.command(mode="attitude", roll=0.0, pitch=0.0, climb=0.0)
    for _ in range(500):
        q.step(0.02)
    assert q.vel[0] > 3.0, "carried downwind"
    assert abs(q.pos[2] - 20) < 1.0, "the climb rate is held"


def test_hover_power_and_endurance_match_their_class():
    for p, lo, hi in ((dm.CONSUMER, 25, 40), (dm.INDUSTRIAL, 40, 60)):
        q = hovering(p, (0, 0, 10))
        minutes = p.battery_wh / float(q.fleet.power_w[0]) * 60
        assert lo <= minutes <= hi, (p.name, minutes)
        assert 0.9 < q.fleet.battery()[0] < 1.0


def test_blade_passing_frequency_in_hover():
    q = hovering(dm.CONSUMER, (0, 0, 10))
    bpf = float(q.fleet.blade_passing_hz()[0])
    assert bpf == pytest.approx(2 * dm.CONSUMER.hover_w / (2 * math.pi), rel=0.05)
    assert float(q.fleet.thrust_ratio()[0]) == pytest.approx(1.0, abs=0.05)


def test_vasarhelyi_preset_keeps_its_distance():
    """With the article's own parameters drones settle about r0 apart (repulsion vs nothing pulling them together)."""
    k = dm.FLOCK_VASARHELYI_2018
    v = dm.flock_velocity(0, [(0, 0, 0), (10, 0, 0)], [(0, 0, 0), (0, 0, 0)], None, k)
    assert v[0] < 0 and np.linalg.norm(v) <= k.rep_max + 1e-9
    v = dm.flock_velocity(0, [(0, 0, 0), (30, 0, 0)], [(0, 0, 0), (0, 0, 0)], None, k)
    assert np.allclose(v, 0)
    assert dm.flock_params({"preset": "vasarhelyi2018", "flock_speed": 5}).flock_speed == 5
