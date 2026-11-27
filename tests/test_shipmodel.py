"""The MMG ship model (shipmodel.py) against Yasukawa & Yoshimura (2015): the KVLCC2 7 m model's turning indices
(Table 4, calculated) and zig-zag overshoot angles (Table 5, calculated), the IMO manoeuvring criteria in full scale,
the actuators, autopilot and frames; then current, wind (Blendermann 1994) and wave drift (IMO MEPC.1/Circ.850/Rev.3)."""
import math
from dataclasses import replace

import numpy as np
import pytest

from coupledcarla import shipmodel as sm


def test_presets_and_scaling():
    h = sm.KVLCC2
    assert (h.lpp, h.beam, h.draught, h.volume) == (320.0, 58.0, 20.8, 312600.0)
    assert h.eta == pytest.approx(9.86 / 15.80)
    l7 = sm.preset("KVLCC2_L7")
    assert l7.lpp == 7.00 and l7.a_rudder == 0.0539 and l7.w_p0 == 0.40
    assert l7.rudder_rate_deg == pytest.approx(1.76 * math.sqrt(45.7))        # Froude-scaled time
    assert l7.service_speed_kn * math.sqrt(45.7) == pytest.approx(15.5)
    assert l7.yv == h.yv and l7.f_alpha == h.f_alpha                          # same coefficients
    with pytest.raises(ValueError):
        sm.preset("titanic")


@pytest.mark.parametrize("hull", [sm.KVLCC2, sm.KVLCC2_L7])
def test_holds_its_approach_speed_on_a_straight_course(hull):
    ship = sm.Ship.at_speed(hull)
    u0 = ship.u
    dt = 0.02 * hull.lpp / u0
    for _ in range(500):
        ship.step(dt)
    assert ship.u == pytest.approx(u0, rel=1e-3)
    assert abs(ship.vm) < 1e-6 and abs(ship.r) < 1e-9 and abs(ship.y) < 1e-6
    assert ship.x == pytest.approx(u0 * ship.t, rel=1e-3)


def test_full_scale_revolutions_are_plausible():
    n = sm.KVLCC2.revolutions_for(15.5 * sm.KNOT)
    assert 1.3 < n < 2.2            # rev/s: about 80-130 rpm for a VLCC's slow-speed engine


@pytest.mark.parametrize("rudder, advance, tactical", [(35, 3.31, 3.36), (-35, 3.26, 3.26)])
def test_turning_matches_the_paper_l7_model(rudder, advance, tactical):
    """Table 4, calculated (the same model and coefficients): within 10 % (the propeller position x_P' and the
    model-scale resistance are not printed in the paper)."""
    got = sm.turning_circle(sm.KVLCC2_L7, rudder)
    assert got["advance"] == pytest.approx(advance, rel=0.10)
    assert got["tactical_diameter"] == pytest.approx(tactical, rel=0.10)


def test_turning_is_asymmetric_like_the_paper():
    stb, port = sm.turning_circle(sm.KVLCC2_L7, 35), sm.turning_circle(sm.KVLCC2_L7, -35)
    assert stb["tactical_diameter"] != pytest.approx(port["tactical_diameter"], rel=1e-3)


@pytest.mark.parametrize("rudder, osa1, osa2", [(10, 5.2, 15.8), (20, 10.9, None), (-10, 7.6, 10.2),
                                                (-20, 14.5, None)])
def test_zigzag_overshoots_match_the_paper_l7_model(rudder, osa1, osa2):
    """Table 5, calculated: overshoot angles within 2 deg."""
    got = sm.zigzag(sm.KVLCC2_L7, rudder)
    assert got["osa1"] == pytest.approx(osa1, abs=2.0)
    if osa2 is not None:
        assert got["osa2"] == pytest.approx(osa2, abs=2.0)


def test_full_scale_meets_the_imo_criteria():
    """IMO MSC.137(76): advance < 4.5 L, tactical diameter < 5 L; 10/10 zig-zag overshoots below 20 and 40 deg for
    L/V of 30 s or more (KVLCC2: 320 m at 7.97 m/s, 40 s); 20/20 first overshoot below 25 deg."""
    for rudder in (35, -35):
        t = sm.turning_circle(sm.KVLCC2, rudder)
        assert t["advance"] < 4.5 and t["tactical_diameter"] < 5.0
    z = sm.zigzag(sm.KVLCC2, 10)
    assert z["osa1"] < 20.0 and z["osa2"] < 40.0
    assert sm.zigzag(sm.KVLCC2, 20)["osa1"] < 25.0


def test_rudder_turns_at_its_rate_and_stops_at_its_limit():
    ship = sm.Ship.at_speed(sm.KVLCC2)
    ship.rudder_cmd = math.radians(90)          # more than the limit
    ship.step(1.0)
    assert math.degrees(ship.delta) == pytest.approx(1.76)
    for _ in range(100):
        ship.step(1.0)
    assert math.degrees(ship.delta) == pytest.approx(35.0)
    assert ship.r > 0 and ship.psi > 0           # positive rudder: turning to starboard (heading increases)


def test_engine_orders_change_speed_through_the_lag():
    ship = sm.Ship.at_speed(sm.KVLCC2, 15.5)
    n0 = ship.n
    ship.order_speed(8.0)
    ship.step(1.0)
    assert ship.n_cmd < ship.n < n0              # still on its way down
    for _ in range(3000):
        ship.step(1.0)
    assert ship.speed / sm.KNOT == pytest.approx(8.0, abs=0.2)
    ship.order_speed(0.0)
    for _ in range(4000):
        ship.step(1.0)
    assert ship.n == pytest.approx(0.0, abs=1e-6) and ship.speed / sm.KNOT < 3.0    # coasting down, no crash


def test_a_stopped_ship_stays_put():
    ship = sm.Ship(sm.KVLCC2)
    ship.rudder_cmd = 0.5
    for _ in range(10):
        ship.step(1.0)
    assert (ship.x, ship.y, ship.u, ship.vm, ship.r) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_autopilot_takes_a_new_heading_without_much_overshoot():
    ship = sm.Ship.at_speed(sm.KVLCC2)
    ship.autopilot = sm.Autopilot(heading=math.radians(60))
    worst = 0.0
    for _ in range(1500):
        ship.step(1.0)
        worst = max(worst, math.degrees(ship.psi))
    assert math.degrees(ship.psi) == pytest.approx(60.0, abs=1.0)
    assert worst < 70.0


def test_autopilot_follows_waypoints():
    wps = [(3000.0, 0.0), (3000.0, 3000.0), (6000.0, 3000.0)]       # north, east, north (m)
    ship = sm.Ship.at_speed(sm.KVLCC2)
    ship.autopilot = sm.Autopilot(waypoints=wps)
    t_max = 6 * 3600
    while not ship.autopilot.done and ship.t < t_max:
        ship.step(1.0)
    assert ship.autopilot.done
    assert math.hypot(ship.x - 6000.0, ship.y - 3000.0) < 2 * ship.hull.lpp


@pytest.mark.parametrize("x, y, psi", [(0.0, 0.0, 0.0), (100.0, -50.0, 1.0), (-20.0, 7.0, -2.5)])
def test_carla_frames_round_trip(x, y, psi):
    cx, cy, yaw = sm.to_carla(x, y, psi)
    back = sm.from_carla(cx, cy, yaw)
    assert back == pytest.approx((x, y, psi))


def test_carla_directions():
    # heading north (psi 0): CARLA yaw -90 (towards -y, which is north in CARLA's x east, y south)
    assert sm.to_carla(10.0, 0.0, 0.0) == pytest.approx((0.0, -10.0, -90.0))
    # east (psi 90 deg): CARLA yaw 0, +x
    assert sm.to_carla(0.0, 10.0, math.pi / 2) == pytest.approx((10.0, 0.0, 0.0))


# --- current and wind --------------------------------------------------------------------------------------------

def test_environment_conventions():
    env = sm.Environment(current_mps=2.0, current_to_deg=90.0, wind_mps=10.0, wind_from_deg=0.0)
    assert env.current_ne() == pytest.approx((0.0, 2.0))         # sets towards the east
    assert env.wind_ne() == pytest.approx((-10.0, 0.0))          # a north wind blows towards the south
    assert sm.to_body(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, -1.0))   # heading east, north is to port


def test_a_uniform_current_only_moves_the_track():
    """Through-water motion is the same as in still water; the track over the ground is shifted by current x time."""
    hull = replace(sm.KVLCC2, windage=None)                       # no air drag, so the comparison is exact
    still = sm.Ship.at_speed(hull)
    moving = sm.Ship.at_speed(hull, env=sm.Environment(current_mps=1.5, current_to_deg=60.0))
    for ship in (still, moving):
        ship.rudder_cmd = math.radians(35)
        for _ in range(600):
            ship.step(1.0)
    cn, ce = moving.env.current_ne()
    assert (moving.psi, moving.u, moving.vm, moving.r) == pytest.approx((still.psi, still.u, still.vm, still.r))
    assert (moving.x, moving.y) == pytest.approx((still.x + cn * 600, still.y + ce * 600))


def test_a_stopped_ship_drifts_with_the_current():
    ship = sm.Ship(replace(sm.KVLCC2, windage=None), env=sm.Environment(current_mps=1.0, current_to_deg=180.0))
    for _ in range(100):
        ship.step(1.0)
    assert (ship.x, ship.y) == pytest.approx((-100.0, 0.0))
    assert ship.speed == 0.0 and ship.speed_over_ground == pytest.approx(1.0)


def test_blendermann_coefficients():
    w = sm.KVLCC2.windage                       # loaded tanker: CD_t 0.70, CD_l_AF 0.90 (bow) / 0.55 (stern)
    assert sm.wind_coefficients(w, 0.0)[:2] == pytest.approx((-0.90, 0.0))
    assert sm.wind_coefficients(w, math.pi)[:2] == pytest.approx((0.55, 0.0))
    cx, cy, cn = sm.wind_coefficients(w, math.pi / 2)
    assert cx == pytest.approx(0.0, abs=1e-12) and cy == pytest.approx(0.70)
    assert cn == pytest.approx(w.s_l / w.loa * 0.70)


def test_a_wind_from_starboard_pushes_to_port():
    # a ship at rest, the air moving to port (wind from starboard): the ship moves to starboard relative to the air
    x, y, n = sm.wind_forces(sm.KVLCC2, 0.0, 20.0)
    assert y < 0 and abs(x) < 1e-6 * abs(y)
    assert n > 0              # superstructure aft: the stern is pushed to port, the bow turns to starboard
    assert sm.wind_forces(replace(sm.KVLCC2, windage=None), 0.0, 20.0) == (0.0, 0.0, 0.0)


def _sail(env, seconds=3600):
    ship = sm.Ship.at_speed(sm.KVLCC2, env=env)
    ship.autopilot = sm.Autopilot(heading=0.0)
    for _ in range(seconds):
        ship.step(1.0)
    return ship


def test_head_wind_slows_and_tail_wind_speeds_up():
    """Revolutions held; Beaufort 9 (22.7 m/s, as in Yasukawa et al. 2018, Table 8)."""
    calm = _sail(sm.Environment()).speed
    head = _sail(sm.Environment(wind_mps=22.7, wind_from_deg=0.0)).speed
    tail = _sail(sm.Environment(wind_mps=22.7, wind_from_deg=180.0)).speed
    assert head < calm < tail
    assert 0.2 < (calm - head) / sm.KNOT < 1.5


def test_beam_wind_drift_and_check_helm():
    ship = _sail(sm.Environment(wind_mps=22.7, wind_from_deg=90.0))          # from starboard
    assert ship.psi == pytest.approx(0.0, abs=math.radians(0.1))             # the autopilot's integral trims it out
    assert ship.vm < 0 and ship.course < 0                                    # drifting to port (leeward)
    assert -5.0 < math.degrees(ship.delta) < 0.0                              # a little port helm
    assert 0.1 < math.degrees(math.atan2(-ship.vm, ship.u)) < 3.0             # drift angle


def test_waypoints_in_cross_current_and_wind():
    wps = [(3000.0, 0.0), (3000.0, 3000.0), (6000.0, 3000.0)]
    env = sm.Environment(current_mps=1.0, current_to_deg=270.0, wind_mps=15.0, wind_from_deg=45.0)
    ship = sm.Ship.at_speed(sm.KVLCC2, env=env)
    ship.autopilot = sm.Autopilot(waypoints=wps)
    while not ship.autopilot.done and ship.t < 6 * 3600:
        ship.step(1.0)
    assert ship.autopilot.done


def test_windage_scales_with_the_model():
    w, w7 = sm.KVLCC2.windage, sm.KVLCC2_L7.windage
    assert w7.a_lateral == pytest.approx(w.a_lateral / 45.7 ** 2) and w7.loa == pytest.approx(w.loa / 45.7)


# --- waves -------------------------------------------------------------------------------------------------------

SEAS = [(4.0, 7.7), (5.5, 9.1), (7.0, 10.2)]        # Beaufort 7, 8, 9 (Yasukawa et al. 2018, Table 8)
U_SERVICE = 15.5 * sm.KNOT


def test_jonswap_has_the_right_area_and_peak():
    w = np.linspace(0.05, 6.0, 20000)
    s = sm.jonswap(w, 4.0, 8.0)
    assert np.trapezoid(s, w) == pytest.approx(4.0 ** 2 / 16, rel=1e-3)
    assert w[np.argmax(s)] == pytest.approx(2 * math.pi / 8.0, rel=0.01)


def _sigma(beta_deg, lam_over_l):
    h = sm.KVLCC2
    w = np.sqrt(2 * math.pi * sm.G / (np.asarray(lam_over_l) * h.lpp))
    return sm.added_resistance_rao(h, w, math.radians(beta_deg), U_SERVICE) / (h.rho * sm.G * h.beam ** 2 / h.lpp)


def test_head_sea_added_resistance_has_its_resonance_near_one_ship_length():
    lam = np.linspace(0.3, 2.4, 43)
    sig = _sigma(180, lam)
    assert 0.8 < lam[np.argmax(sig)] < 1.3
    assert 1.0 < sig[0] < 4.0                     # short waves: reflection, about 2 for KVLCC2 at Fn 0.14
    assert _sigma(180, [3.0])[0] < 0.1 * sig.max()    # long waves: the ship rides them


@pytest.mark.parametrize("hs, tp", SEAS)
def test_spectral_added_resistance_agrees_with_imos_simple_formula(hs, tp):
    """IMO 15.2 in long-crested head seas x 1.3 (its para 18) against IMO 15.1, 1336 (5.3 + U) (B d / L)^0.75 hs^2:
    two routes in the same guideline, within 20 %."""
    h = sm.KVLCC2
    x, y = sm.wave_drift(h, sm.Environment(wave_hs=hs, wave_tp=tp, wave_from_deg=0.0), 0.0, U_SERVICE)
    simple = 1336 * (5.3 + U_SERVICE) * (h.beam * h.draught / h.lpp) ** 0.75 * hs ** 2
    assert -1.3 * x == pytest.approx(simple, rel=0.20)
    assert y == pytest.approx(0.0, abs=1e-6 * abs(x))


def test_side_drift_reaches_full_reflection_in_short_beam_seas():
    h, hs = sm.KVLCC2, 2.0
    _, y = sm.wave_drift(h, sm.Environment(wave_hs=hs, wave_tp=5.0, wave_from_deg=90.0), 0.0, U_SERVICE)
    assert -y == pytest.approx(0.5 * h.rho * sm.G * h.lpp * hs ** 2 / 8, rel=0.05)   # 1/2 rho g (2 m0) L


def test_wave_directions():
    h = sm.KVLCC2
    def drift(frm):
        return sm.wave_drift(h, sm.Environment(wave_hs=4.0, wave_tp=8.0, wave_from_deg=frm), 0.0, U_SERVICE)
    head, stbd, port, follow = drift(0), drift(90), drift(-90), drift(180)
    assert head[0] < 0 and stbd[1] < 0 and port[1] > 0                  # pushed back, and away from the waves
    assert stbd[0] == pytest.approx(port[0]) and stbd[1] == pytest.approx(-port[1])
    assert abs(follow[0]) < 0.2 * abs(head[0])                          # following seas: small either way
    assert sm.wave_drift(h, sm.Environment(), 0.0, U_SERVICE) == (0.0, 0.0)


def test_head_seas_slow_the_ship_and_beam_seas_push_it_sideways():
    calm = _sail(sm.Environment()).speed
    head = _sail(sm.Environment(wave_hs=5.5, wave_tp=9.1, wave_from_deg=0.0))
    beam = _sail(sm.Environment(wave_hs=5.5, wave_tp=9.1, wave_from_deg=90.0))
    assert 0.5 < (calm - head.speed) / sm.KNOT < 3.0
    assert beam.psi == pytest.approx(0.0, abs=math.radians(0.2))       # the autopilot holds the heading
    assert beam.course < 0 and beam.vm < 0                               # drifting to port, away from the waves
    assert beam.speed < calm
