"""coupledcarla.acoustics: CNOSSOS-EU source levels, ISO 9613-1 absorption against the standard's table, ISO 9613-2
ground and barrier attenuation, IMAGINE's wet-road correction, the cabin model. Numbers only, no audio."""
import math

import numpy as np
import pytest

from coupledcarla import acoustics as ac


def test_cnossos_car_at_50_kmh_and_7_5_m():
    # CNOSSOS-EU 2021: a passenger car at 50 km/h, 7.5 m, hard ground: ~73 dB(A) (the 2015 table: 70.5); measured
    # statistical pass-by levels on dense asphalt at 50 km/h are about 70-73 dB(A)
    assert ac.car_at_7_5(50.0) == pytest.approx(72.9, abs=0.3)
    assert ac.car_at_7_5(50.0, table=ac.CNOSSOS_2015) == pytest.approx(70.5, abs=0.3)
    assert 70.0 <= ac.car_at_7_5(50.0) <= 74.0
    # +9-10 dB per doubling of speed at urban speeds, heavy vehicles ~9 dB above cars
    assert 8.0 < ac.car_at_7_5(100.0) - ac.car_at_7_5(50.0) < 11.0
    assert 7.0 < ac.car_at_7_5(50.0, cat="3") - ac.car_at_7_5(50.0) < 11.0


def test_cnossos_formulas():
    r, p = ac.vehicle_power("1", 70.0)
    assert np.allclose(r, ac.CNOSSOS["1"][0]) and np.allclose(p, ac.CNOSSOS["1"][2])      # at v_ref
    r, p = ac.vehicle_power("1", 140.0)
    assert np.allclose(r, np.array(ac.CNOSSOS["1"][0]) + np.array(ac.CNOSSOS["1"][1]) * math.log10(2))
    assert np.allclose(p, np.array(ac.CNOSSOS["1"][2]) + np.array(ac.CNOSSOS["1"][3]) * 1.0)
    # below 20 km/h CNOSSOS uses 20 km/h
    assert np.allclose(ac.vehicle_power("1", 5.0)[1], ac.vehicle_power("1", 20.0)[1])
    # two-wheelers have no rolling term; electric: no propulsion
    r4, p4 = ac.vehicle_power("4b", 50.0)
    assert np.all(np.isneginf(r4)) and np.all(np.isfinite(p4))
    assert np.all(np.isneginf(ac.vehicle_power("1", 50.0, electric=True)[1]))
    # temperature: +0.08 dB per degree below 20 C on cars' rolling noise, half that for heavy vehicles
    assert np.allclose(ac.vehicle_power("1", 50.0, temp_c=10.0)[0] - ac.vehicle_power("1", 50.0)[0], 0.8)
    assert np.allclose(ac.vehicle_power("3", 50.0, temp_c=10.0)[0] - ac.vehicle_power("3", 50.0)[0], 0.4)
    # acceleration: IMAGINE's C_P a, clipped at a_max (cars 2 m/s^2) and -1
    p0 = ac.vehicle_power("1", 50.0)[1]
    assert np.allclose(ac.vehicle_power("1", 50.0, accel=1.0)[1] - p0, 2.75)
    assert np.allclose(ac.vehicle_power("1", 50.0, accel=9.0)[1] - p0, 5.5)            # = CNOSSOS's C_P at a crossing
    assert np.allclose(ac.vehicle_power("1", 50.0, accel=-5.0)[1] - p0, -2.75)
    # junction (Directive Table 2.3.b) at the crossing, fading out over 100 m
    pj = ac.vehicle_power("1", 50.0, junction=(1, 0.0))[1]
    assert np.allclose(pj - p0, 5.5)
    assert np.allclose(ac.vehicle_power("1", 50.0, junction=(1, 50.0))[1] - p0, 2.75)
    # paving blocks are louder than the reference surface; porous asphalt quieter at 1-4 kHz
    assert ac.car_at_7_5(50.0, surface="NL11") > ac.car_at_7_5(50.0) + 3.0
    assert ac.car_at_7_5(80.0, surface="NL01") < ac.car_at_7_5(80.0)


def test_wet_road_raises_the_high_frequencies():
    w = ac.wet_correction(50.0)
    assert np.all(w[:4] < 0.5) and 5.0 < w[6] < 11.0 and w[7] > w[6]            # mostly above 2 kHz
    # IMAGINE D11: 15 lg f - 12 lg(v/70) - 48 at 4 kHz, 50 km/h = 7.8 dB (third-octave), the octave a little more
    assert 15 * math.log10(4000) - 12 * math.log10(50 / 70) - 48 == pytest.approx(7.78, abs=0.01)
    dry = ac.car_at_7_5(50.0)
    wet = ac.car_at_7_5(50.0, wet=1.0)
    assert 1.0 < wet - dry < 6.0             # wet roads raise tyre noise by several dB(A)


ISO_TABLE = {   # ISO 9613-2 Table 2: alpha (dB/km) at 63..8000 Hz
    (10.0, 70.0): [0.1, 0.4, 1.0, 1.9, 3.7, 9.7, 32.8, 117.0],
    (20.0, 70.0): [0.1, 0.3, 1.1, 2.8, 5.0, 9.0, 22.9, 76.6],
    (30.0, 70.0): [0.1, 0.3, 1.0, 3.1, 7.4, 12.7, 23.1, 59.3],
    (15.0, 20.0): [0.3, 0.6, 1.2, 2.7, 8.2, 28.2, 88.8, 202.0],
    (15.0, 50.0): [0.1, 0.5, 1.2, 2.2, 4.2, 10.8, 36.2, 129.0],
    (15.0, 80.0): [0.1, 0.3, 1.1, 2.4, 4.1, 8.3, 23.7, 82.8],
}


@pytest.mark.parametrize("cond", list(ISO_TABLE))
def test_iso_9613_1_absorption_matches_the_table(cond):
    got = ac.air_absorption(*cond) * 1000.0
    want = np.array(ISO_TABLE[cond])
    assert np.all(np.abs(got - want) <= 0.06 + 0.02 * want), (got, want)


def test_iso_9613_2_ground_and_barrier():
    # hard ground near the source: +3 dB in every band (the reflected path)
    assert np.allclose(ac.ground_attenuation(0.05, 1.5, 7.5), -3.0)
    # porous ground far away: attenuation around 250-500 Hz (the ground dip), none at 63 Hz beyond A_m
    g = ac.ground_attenuation(0.05, 1.5, 200.0, 1.0, 1.0, 1.0)
    assert g[2] > 5.0 and g[3] > 5.0 and g[0] < 0.0
    # barrier: 10 lg(3 + 20 z / lambda), capped at 20 dB single / 25 double
    d = ac.barrier_attenuation(1.0)
    assert d[4] == pytest.approx(10 * math.log10(3 + 20 * 1000 / 343.0), abs=0.01)
    assert np.all(np.diff(d) >= 0) and d.max() <= 20.0
    assert ac.barrier_attenuation(0.0)[0] == pytest.approx(10 * math.log10(3), abs=0.01)
    assert ac.barrier_attenuation(30.0, double=True, e=20.0).max() == 25.0


def test_a_weighting_sums_and_cabin():
    assert ac.a_weighted([94.0] + [-200.0] * 7) == pytest.approx(94.0 - 26.2, abs=0.01)
    assert float(ac.db_sum([60.0, 60.0])) == pytest.approx(63.01, abs=0.01)
    assert ac.wind_interior(10.0) is None
    assert ac.a_weighted(ac.wind_interior(100.0)) == pytest.approx(60.0, abs=0.01)
    assert ac.a_weighted(ac.wind_interior(200.0)) - ac.a_weighted(ac.wind_interior(100.0)) == \
        pytest.approx(60 * math.log10(2), abs=0.01)
    assert np.all(np.diff(ac.CABIN_IL) > 0)
    assert ac.rain_level(0.0) is None and ac.rain_level(10.0) - ac.rain_level(1.0) == pytest.approx(10.0)
