"""Published acoustic models behind the soundscape, in octave bands 63 Hz - 8 kHz. Pure numpy, no audio, no CARLA.

- Road vehicle sound power: CNOSSOS-EU (Commission Directive (EU) 2015/996, Annex II, as amended by Commission
  Delegated Directive (EU) 2021/1226): rolling noise A_R + B_R lg(v/70) and propulsion noise A_P + B_P (v-70)/70 per
  octave band for categories 1 (cars), 2 (vans, medium), 3 (heavy: buses, trucks), 4a/4b (two-wheelers), with the
  road surface (alpha + beta lg(v/70)), air temperature (K (20 - T)) and junction corrections of the Directive.
  The 2021 coefficients describe a point source radiating into the full sphere (3 dB above the 2015 ones, which
  assumed the half space); `CNOSSOS_2015` keeps the original table.
- Acceleration: the Harmonoise/IMAGINE form Delta L_P = C_P a, a clipped to [-1, a_max] (IMAGINE Deliverable D11,
  2007, section 4.3.1: a_max 2, 1, 1 and 4 m/s^2 for categories 1, 2, 3 and 4). IMAGINE tabulates C_P only in its
  database, so C_P is set here so that a_max gives the CNOSSOS junction maximum C_P (5.5 dB cars, 9 dB heavy).
- Wet roads: IMAGINE D11 section 4.3.5, Delta L_wet = max(15 lg f - 12 lg(v/70) - 48, 0) (2 mm water film).
- Atmospheric absorption: ISO 9613-1:1993, the pure-tone formula at the exact mid-band frequencies (as ISO 9613-2
  uses it for octave bands), from temperature, relative humidity and pressure.
- Ground effect: ISO 9613-2:1996 general method (Table 3) with the ground factor G (0 hard, 1 porous).
- Barriers (buildings): ISO 9613-2 section 7.4, D_z = 10 lg(3 + C2/lambda C3 z) (a Maekawa-type law), capped at
  20 dB single / 25 dB double diffraction.
- Cabin: airborne insertion loss of a passenger car body rising with frequency (values in the range measured for
  production cars, e.g. Harrison, Vehicle Refinement, 2004, ch. 4) and wind noise rising with 60 lg v (dipole sound
  power ~ v^6, Lighthill/Curle; measured interior wind noise rises 50-60 lg v).
"""
import math

import numpy as np

BANDS = np.array([63.0, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0])
EXACT = 1000.0 * 10.0 ** (0.3 * np.arange(-4, 4))          # exact octave mid-band frequencies (IEC 61260)
EDGES = np.concatenate([[0.0], EXACT[:-1] * 2 ** 0.5, [np.inf]])   # brick-wall edges: everything belongs to a band
A_WEIGHT = np.array([-26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1])
NB = len(BANDS)

# -- CNOSSOS-EU vehicle coefficients (Directive 2015/996 Annex II Table 2.3.a, amended by 2021/1226) ---------------
# A_R, B_R, A_P, B_P per octave band 63..8000 Hz
CNOSSOS = {
    "1": ([83.1, 89.2, 87.7, 93.1, 100.1, 96.7, 86.8, 76.2], [30.0, 41.5, 38.9, 25.7, 32.5, 37.2, 39.0, 40.0],
          [97.9, 92.5, 90.7, 87.2, 84.7, 88.0, 84.4, 77.1], [-1.3, 7.2, 7.7, 8.0, 8.0, 8.0, 8.0, 8.0]),
    "2": ([88.7, 93.2, 95.7, 100.9, 101.7, 95.1, 87.8, 83.6], [30.0, 35.8, 32.6, 23.8, 30.1, 36.2, 38.3, 40.1],
          [105.5, 100.2, 100.5, 98.7, 101.0, 97.8, 91.2, 85.0], [-1.9, 4.7, 6.4, 6.5, 6.5, 6.5, 6.5, 6.5]),
    "3": ([91.7, 96.2, 98.2, 104.9, 105.1, 98.5, 91.1, 85.6], [30.0, 33.5, 31.3, 25.4, 31.8, 37.1, 38.6, 40.6],
          [108.8, 104.2, 103.5, 102.9, 102.6, 98.5, 93.8, 87.5], [0.0, 3.0, 4.6, 5.0, 5.0, 5.0, 5.0, 5.0]),
    "4a": ([0.0] * 8, [0.0] * 8,
           [93.0, 93.0, 93.5, 95.3, 97.2, 100.4, 95.8, 90.9], [4.2, 7.4, 9.8, 11.6, 15.7, 18.9, 20.3, 20.6]),
    "4b": ([0.0] * 8, [0.0] * 8,
           [99.9, 101.9, 96.7, 94.4, 95.2, 94.7, 92.1, 88.6], [3.2, 5.9, 11.9, 11.6, 11.5, 12.6, 11.1, 12.0]),
}
CNOSSOS_2015 = {
    "1": ([79.7, 85.7, 84.5, 90.2, 97.3, 93.9, 84.1, 74.3], CNOSSOS["1"][1],
          [94.5, 89.2, 88.0, 85.9, 84.2, 86.9, 83.3, 76.1], CNOSSOS["1"][3]),
    "2": ([84.0, 88.7, 91.5, 96.7, 97.4, 90.9, 83.8, 80.5], CNOSSOS["2"][1],
          [101.0, 96.5, 98.8, 96.8, 98.6, 95.2, 88.8, 82.7], CNOSSOS["2"][3]),
    "3": ([87.0, 91.7, 94.1, 100.7, 100.8, 94.3, 87.1, 82.5], CNOSSOS["3"][1],
          [104.4, 100.6, 101.7, 101.0, 100.1, 95.9, 91.3, 85.3], CNOSSOS["3"][3]),
    "4a": (CNOSSOS["4a"][0], CNOSSOS["4a"][1], [88.0, 87.5, 89.5, 93.7, 96.6, 98.8, 93.9, 88.7], CNOSSOS["4a"][3]),
    "4b": (CNOSSOS["4b"][0], CNOSSOS["4b"][1], [95.0, 97.2, 92.7, 92.9, 94.7, 93.2, 90.1, 86.5], CNOSSOS["4b"][3]),
}
V_REF = 70.0
# junction correction (Table 2.3.b): k=1 crossing with lights, 2 roundabout: (C_R, C_P)
JUNCTION = {"1": {1: (-4.5, 5.5), 2: (-4.4, 3.1)}, "2": {1: (-4.0, 9.0), 2: (-2.3, 6.7)},
            "3": {1: (-4.0, 9.0), 2: (-2.3, 6.7)}, "4a": {1: (0.0, 0.0), 2: (0.0, 0.0)},
            "4b": {1: (0.0, 0.0), 2: (0.0, 0.0)}}
TEMP_K = {"1": 0.08, "2": 0.04, "3": 0.04, "4a": 0.0, "4b": 0.0}
# acceleration (IMAGINE D11 4.3.1): a_max and C_P (dB per m/s^2, see the module doc)
ACCEL = {"1": (2.0, 2.75), "2": (1.0, 9.0), "3": (1.0, 9.0), "4a": (4.0, 2.75), "4b": (4.0, 2.75)}
# road surfaces (Directive Table 2.3.c / F-4 as amended): name -> {cat: (alpha per band, beta)}; cat 4: none
ROAD_SURFACES = {
    "DEF": {},     # the CNOSSOS reference: average of dense asphalt 0/11 and SMA 0/11
    "NL01": {"1": ([0.0, 5.4, 4.3, 4.2, -1.0, -3.2, -2.6, 0.8], -6.5),          # 1-layer porous asphalt (ZOAB)
             "2": ([7.9, 4.3, 5.3, -0.4, -5.2, -4.6, -3.0, -1.4], 0.2),
             "3": ([9.3, 5.0, 5.5, -0.4, -5.2, -4.6, -3.0, -1.4], 0.2)},
    "NL05": {"1": ([6.0, 0.3, 0.3, 0.0, -0.6, -1.2, -0.7, -0.7], -1.4)},          # SMA-NL8
    "NL10": {"1": ([27.0, 16.2, 14.7, 6.1, 3.0, -1.0, 1.2, 4.5], 2.5),           # paving blocks, herring-bone
             "2": ([29.5, 20.0, 17.6, 8.0, 6.2, -1.0, 3.1, 5.2], 2.5),
             "3": ([29.4, 21.2, 18.2, 8.4, 5.6, -1.0, 3.0, 5.8], 2.5)},
    "NL11": {"1": ([31.4, 19.7, 16.8, 8.4, 7.2, 3.3, 7.8, 9.1], 2.9),            # paving blocks, not herring-bone
             "2": ([34.0, 23.6, 19.8, 10.5, 11.7, 8.2, 12.2, 10.0], 2.9),
             "3": ([33.8, 24.7, 20.4, 10.9, 10.9, 6.8, 12.0, 10.8], 2.9)},
}
# OSM surface tags -> a CNOSSOS surface
OSM_SURFACE = {"asphalt": "DEF", "paved": "DEF", "concrete": "DEF", "paving_stones": "NL10", "sett": "NL11",
               "cobblestone": "NL11", "unhewn_cobblestone": "NL11", "bricks": "NL10"}

# kinds of the soundscape -> CNOSSOS category (bicycles have none: their own small model in soundscape.py)
CATEGORY = {"car": "1", "van": "2", "bus": "3", "truck": "3", "motorbike": "4b", "moped": "4a"}


def db_sum(levels, axis=None):
    """Energetic sum of levels in dB (None and -inf ignored)."""
    a = np.asarray(levels, dtype=float)
    with np.errstate(divide="ignore"):
        return 10.0 * np.log10(np.sum(10.0 ** (a / 10.0), axis=axis))


def a_weighted(band_db):
    """A-weighted total of octave-band levels (8 bands, 63..8000 Hz)."""
    return float(db_sum(np.asarray(band_db, float) + A_WEIGHT))


def wet_correction(kmh, bands=BANDS):
    """IMAGINE D11 4.3.5: extra rolling noise on a wet road (dB per octave band), averaged over each octave's three
    third-octave bands."""
    v = max(float(kmh), 20.0)
    out = []
    for fc in bands:
        thirds = fc * 2.0 ** (np.array([-1, 0, 1]) / 3.0)
        d = np.maximum(15.0 * np.log10(thirds) - 12.0 * math.log10(v / V_REF) - 48.0, 0.0)
        out.append(10.0 * math.log10(np.mean(10.0 ** (d / 10.0))))
    return np.array(out)


def vehicle_power(cat, kmh, accel=0.0, temp_c=20.0, surface="DEF", wet=0.0, junction=None, table=None,
                  electric=False):
    """(L_W rolling, L_W propulsion) per octave band, dB re 1 pW, of one vehicle of CNOSSOS category `cat` at
    `kmh` (km/h, 20 km/h at least as CNOSSOS asks), accelerating `accel` m/s^2. wet: 0..1 (1: a 2 mm water film);
    junction: (k, distance m) for the Directive's statistical junction correction (not needed when the real
    acceleration is known). electric: no propulsion noise (its own AVAS is added by the caller)."""
    table = table or CNOSSOS
    ar, br, ap, bp = (np.asarray(x, float) for x in table[cat])
    v = max(float(kmh), 20.0)
    rolling = ar + br * math.log10(v / V_REF) if ar.any() else np.full(NB, -np.inf)
    prop = ap + bp * (v - V_REF) / V_REF
    surf = (ROAD_SURFACES.get(surface) or {}).get(cat)
    if surf is not None and np.isfinite(rolling).all():
        alpha, beta = np.asarray(surf[0], float), surf[1]
        rolling = rolling + alpha + beta * math.log10(v / V_REF)
        prop = prop + np.minimum(alpha, 0.0)
    if np.isfinite(rolling).all():
        rolling = rolling + TEMP_K.get(cat, 0.0) * (20.0 - float(temp_c))
        if wet > 0:
            rolling = rolling + 10.0 * np.log10(1.0 + float(wet) * (10.0 ** (wet_correction(v) / 10.0) - 1.0))
    if junction is not None:
        k, x = junction
        cr, cp = JUNCTION[cat][k]
        f = max(1.0 - abs(float(x)) / 100.0, 0.0)
        if np.isfinite(rolling).all():
            rolling = rolling + cr * f
        prop = prop + cp * f
    amax, cp = ACCEL.get(cat, (2.0, 2.75))
    prop = prop + cp * max(-1.0, min(float(accel), amax))
    if electric:
        prop = np.full(NB, -np.inf)
    return rolling, prop


def car_at_7_5(kmh=50.0, cat="1", **kw):
    """A-weighted level (dB) at 7.5 m from a vehicle passing at kmh, 1.2 m up, over hard ground: L_W - (20 lg d +
    11) + 3 (the ground reflection of ISO 9613-2 for G = 0 at short range). The pass-by check of the source model."""
    r, p = vehicle_power(cat, kmh, **kw)
    lw = db_sum(np.vstack([r, p]), axis=0)
    return a_weighted(lw - (20.0 * math.log10(7.5) + 11.0) + 3.0)


# -- ISO 9613-1 atmospheric absorption ----------------------------------------------------------------------------

def air_absorption(temp_c=15.0, rh=70.0, pa_kpa=101.325, freqs=EXACT):
    """Attenuation coefficient alpha (dB/m) of ISO 9613-1 at the given frequencies."""
    f = np.asarray(freqs, float)
    T, T0, T01, pr = float(temp_c) + 273.15, 293.15, 273.16, 101.325
    pa = float(pa_kpa)
    C = -6.8346 * (T01 / T) ** 1.261 + 4.6151
    h = float(rh) * 10.0 ** C * pr / pa        # molar concentration of water vapour, %
    frO = pa / pr * (24.0 + 4.04e4 * h * (0.02 + h) / (0.391 + h))
    frN = pa / pr * (T / T0) ** -0.5 * (9.0 + 280.0 * h * math.exp(-4.170 * ((T / T0) ** (-1.0 / 3.0) - 1.0)))
    return 8.686 * f ** 2 * (1.84e-11 * (pr / pa) * (T / T0) ** 0.5 + (T / T0) ** -2.5 * (
        0.01275 * math.exp(-2239.1 / T) / (frO + f ** 2 / frO) + 0.1068 * math.exp(-3352.0 / T) / (frN + f ** 2 / frN)))


# -- ISO 9613-2 ground effect (general method, Table 3) ------------------------------------------------------------

def _ground_region(h, dp, G):
    """A_s or A_r (dB per octave band 63..8000) of ISO 9613-2 Table 3 for a source/receiver at height h."""
    h = max(float(h), 0.0)
    e50 = 1.0 - math.exp(-dp / 50.0)
    a = 1.5 + 3.0 * math.exp(-0.12 * (h - 5.0) ** 2) * e50 + 5.7 * math.exp(-0.09 * h * h) * (1.0 - math.exp(-2.8e-6 * dp * dp))
    b = 1.5 + 8.6 * math.exp(-0.09 * h * h) * e50
    c = 1.5 + 14.0 * math.exp(-0.46 * h * h) * e50
    d = 1.5 + 5.0 * math.exp(-0.9 * h * h) * e50
    return np.array([-1.5, -1.5 + G * a, -1.5 + G * b, -1.5 + G * c, -1.5 + G * d,
                     -1.5 * (1 - G), -1.5 * (1 - G), -1.5 * (1 - G)])


def ground_attenuation(hs, hr, dp, Gs=0.0, Gr=0.0, Gm=0.0):
    """A_gr = A_s + A_r + A_m (dB per band; negative: the ground adds sound) of ISO 9613-2. dp: horizontal distance."""
    dp = max(float(dp), 0.1)
    q = 0.0 if dp <= 30.0 * (hs + hr) else 1.0 - 30.0 * (hs + hr) / dp
    am = np.array([-3.0 * q] + [-3.0 * q * (1.0 - Gm)] * 7)
    return _ground_region(hs, dp, Gs) + _ground_region(hr, dp, Gr) + am


# -- ISO 9613-2 barrier (Maekawa-type) ---------------------------------------------------------------------------

def barrier_attenuation(z, double=False, c_sound=343.0, freqs=EXACT, e=0.0):
    """D_z (dB per band) for a path length difference z (m) over/around an obstacle: 10 lg(3 + 20 z C3 / lambda),
    C3 = 1 (single edge) or (1 + (5 lambda/e)^2) / (1/3 + (5 lambda/e)^2) (two edges e apart); capped 20 / 25 dB.
    Negative z (line of sight just clear) is allowed down to D_z = 0."""
    lam = c_sound / np.asarray(freqs, float)
    if double and e > 0:
        k = (5.0 * lam / e) ** 2
        c3 = (1.0 + k) / (1.0 / 3.0 + k)
    else:
        c3 = 1.0
    arg = 3.0 + 20.0 * c3 * float(z) / lam
    d = 10.0 * np.log10(np.maximum(arg, 1.0))
    return np.clip(d, 0.0, 25.0 if double else 20.0)


# -- inside a passenger car -----------------------------------------------------------------------------------------

# what reaches the cabin of a passenger car from its own tyres and engine and from outside (dB per band, from the
# level ~1.5 m from the source outside): the body's airborne insertion loss rises with frequency (~25-40 dB above
# 500 Hz); at low frequencies the structure-borne paths (engine mounts, suspension, the tyre cavity) and the cabin's
# resonances let more in. Tuned within published ranges to measured interior levels of mid-size cars: ~40 dB(A) at
# idle, ~60 at 50 km/h, ~67 at 100 km/h, ~71 at 130 km/h (with wind noise); this model gives 41, 60, 70
# and 74 dB(A) (CNOSSOS rolling noise rises faster with speed than measured interiors).
CABIN_IL = np.array([2.0, 6.0, 12.0, 22.0, 31.0, 37.0, 40.0, 42.0])          # outside sounds, own tyres
CABIN_IL_ENGINE = np.array([6.0, 10.0, 16.0, 25.0, 32.0, 36.0, 39.0, 41.0])   # own engine (mounts, firewall)
CABIN_IL_BUS = np.array([8.0, 12.0, 17.0, 22.0, 27.0, 30.0, 33.0, 35.0])
# spectrum shape of interior wind noise (dB relative, broad hump around 500 Hz - 1 kHz)
WIND_SHAPE = np.array([-4.0, -2.0, 0.0, 0.0, -2.0, -6.0, -11.0, -18.0])


def wind_interior(kmh, at100_dba=60.0):
    """Interior wind noise band levels (dB) whose A-weighted total is at100_dba at 100 km/h, rising 60 lg v."""
    if kmh < 15.0:
        return None
    total = at100_dba + 60.0 * math.log10(kmh / 100.0)
    shape = WIND_SHAPE - a_weighted(WIND_SHAPE)
    return shape + total


def rain_level(mm_h, inside=False):
    """A-weighted level of rain (dB) from the rain rate: the drops' sound power is proportional to the rain rate
    (number of drops x their energy), so +10 lg R; about 48 dB(A) at 1 mm/h outside on hard ground and 52 dB(A) on a
    car roof (estimates from field recordings of rain; heavy rain at 20 mm/h: 61 and 65 dB(A))."""
    if not mm_h or mm_h < 0.05:
        return None
    return (52.0 if inside else 48.0) + 10.0 * math.log10(min(float(mm_h), 60.0))
