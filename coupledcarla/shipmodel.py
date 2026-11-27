"""Ship manoeuvring model: the MMG standard method (3 degrees of freedom: surge, sway, yaw) with hull, propeller and
rudder forces, a rate-limited rudder, a lagged propeller and a heading autopilot, engine-free like dronemodel.py.

Model: Yasukawa, H. & Yoshimura, Y. (2015), "Introduction of MMG standard method for ship maneuvering predictions",
Journal of Marine Science and Technology 20(1), 37-52 (open access): equation numbers below are the paper's.

    (m + m_x) du/dt - (m + m_y) v_m r - x_G m r^2          = X_H + X_R + X_P
    (m + m_y) dv_m/dt + (m + m_x) u r + x_G m dr/dt        = Y_H + Y_R
    (I_zG + x_G^2 m + J_z) dr/dt + x_G m (dv_m/dt + u r)   = N_H + N_R                                         (4)

Frames (the paper's): earth x0 north, y0 east, z0 down; heading psi from north, clockwise; body x forward, y to
starboard, origin amidships; u, v_m the velocities at midship, r the yaw rate (positive turning to starboard).
A positive rudder angle turns the ship to starboard. to_carla() / from_carla() convert to CARLA's x east, y south.

Hulls are parameter sets (Hull, PRESETS). KVLCC2 is the paper's sample ship: principal particulars from its Table 1,
coefficients from its Table 3 (the hull derivatives also in Table 2), propeller open water and w_P0 from Sect. 5.1,
rudder rate 1.76 deg/s and k_zz = 0.25 Lpp from Sect. 5.1; x_P' (propeller position) is printed only in a figure,
so it is taken from shipmmg's KVLCC2 example (github.com/ShipMMG/shipmmg, MIT): -0.690. Like l_R' (-0.710), it
is an effective coordinate of the flow reaching the propeller in a turn, not where the propeller sits (about
-0.48 L). The full-scale preset keeps
the model's R_0' (the paper extrapolates the full-scale resistance with Schoenherr's formula and gives no value),
with w_P0 = 0.35 as the paper uses in full scale; the propeller revolutions are solved for the approach speed.

Environment (Environment, Ship.env): a uniform, steady current and a uniform wind.
- Current: the ship's u, v_m are through the water, so the hull, propeller and rudder see the same flow as in still
  water; the current only adds to the track over the ground (exact for a uniform, steady current).
- Wind: Blendermann (1994), "Parameter identification of wind loads on ships", J. Wind Eng. Ind. Aerodyn. 51,
  339-351, with his parameters per ship type as tabulated in Fossen's MSS toolbox (blendermann94.m,
  github.com/cybergalactic/MSS, MIT), on the hull's windage (Windage): frontal and lateral areas above the
  waterline. The relative wind uses the ship's velocity over the ground.

- Waves: the mean (second-order) wave drift in a long-crested irregular sea (JONSWAP, peak factor 3.3; the
  slow forces that change speed and course, not the ship's oscillations). Along the ship: the added resistance of
  IMO MEPC.1/Circ.850/Rev.3 (2021), appendix to appendix 2 (the SNNM method of Liu & Papanikolaou): a motion part
  and a reflection part per waterline segment, for any wave direction (its motion part is given from head to beam
  seas only, so following seas get the reflection part alone). Across the ship: an estimate, the short-wave limit
  of full reflection on a vertical wall (1/2 rho g A^2 per metre of side, Longuet-Higgins), times sin^2 of the
  incidence and IMO's draught factor (no reflection for waves longer than 2.5 L); no yaw moment from waves.

Not modelled (MMG's assumptions or v1 limits): wave-induced motions (heave, roll, pitch), gusts, shallow water, banks, other ships' suction, roll,
astern propulsion (revolutions are ahead only, >= 0).
"""
import math
from dataclasses import dataclass, field, replace

import numpy as np

RHO_SEA = 1025.0
RHO_AIR = 1.224                   # Blendermann / MSS, 15 deg C
G = 9.81
KNOT = 0.514444

# Blendermann (1994) per ship type: CD_t, CD_l_AF(bow wind), CD_l_AF(stern wind), delta, kappa
# (MSS blendermann94.m, vessel_no 1-17, same order; kappa is for roll and unused in 3 DOF)
BLENDERMANN = {
    "car_carrier": (0.95, 0.55, 0.60, 0.80, 1.2),
    "cargo_loaded": (0.85, 0.65, 0.55, 0.40, 1.7),
    "cargo_container_on_deck": (0.85, 0.55, 0.50, 0.40, 1.4),
    "container_loaded": (0.90, 0.55, 0.55, 0.40, 1.4),
    "destroyer": (0.85, 0.60, 0.65, 0.65, 1.1),
    "diving_support": (0.90, 0.60, 0.80, 0.55, 1.7),
    "drilling": (1.00, 0.85, 0.925, 0.10, 1.7),
    "ferry": (0.90, 0.45, 0.50, 0.80, 1.1),
    "fishing": (0.95, 0.70, 0.70, 0.40, 1.1),
    "lng_tanker": (0.70, 0.60, 0.65, 0.50, 1.1),
    "offshore_supply": (0.90, 0.55, 0.80, 0.55, 1.2),
    "passenger_liner": (0.90, 0.40, 0.40, 0.80, 1.2),
    "research": (0.85, 0.55, 0.65, 0.60, 1.4),
    "speed_boat": (0.90, 0.55, 0.60, 0.60, 1.1),
    "tanker_loaded": (0.70, 0.90, 0.55, 0.40, 3.1),
    "tanker_ballast": (0.70, 0.75, 0.55, 0.40, 2.2),
    "tender": (0.85, 0.55, 0.55, 0.65, 1.1),
}


@dataclass(frozen=True)
class Windage:
    """What the wind sees: frontal and lateral projected areas above the waterline (m^2), length overall (m), and
    the lateral area's centroid: s_l from midship (forward positive), s_h above the waterline (m); kind: a key of
    BLENDERMANN."""
    a_front: float
    a_lateral: float
    loa: float
    s_l: float
    s_h: float
    kind: str
    source: str = ""

    def scaled(self, s):
        return replace(self, a_front=self.a_front / s ** 2, a_lateral=self.a_lateral / s ** 2, loa=self.loa / s,
                       s_l=self.s_l / s, s_h=self.s_h / s)


def wind_coefficients(w, gamma):
    """Blendermann's C_X, C_Y, C_N for windage w at the relative wind angle gamma (rad; Fossen's convention:
    gamma = -atan2(v_rw, u_rw) with u_rw, v_rw the ship's velocity relative to the air, so 0 is a head wind and
    +90 deg a wind from port). C_X goes with the frontal area, C_Y with the lateral area, C_N with lateral area x
    Loa (moment about midship)."""
    cd_t, cd_bow, cd_stern, dlt, _ = BLENDERMANN[w.kind]
    cd_l_af = cd_bow if abs(gamma) <= math.pi / 2 else cd_stern
    cd_l = cd_l_af * w.a_front / w.a_lateral
    den = 1.0 - 0.5 * dlt * (1.0 - cd_l / cd_t) * math.sin(2.0 * gamma) ** 2
    cx = -cd_l_af * math.cos(gamma) / den
    cy = cd_t * math.sin(gamma) / den
    cn = (w.s_l / w.loa - 0.18 * (abs(gamma) - math.pi / 2)) * cy
    return cx, cy, cn


@dataclass(frozen=True)
class Hull:
    """One ship: dimensions (m, m^3), and the MMG coefficients (non-dimensional, primes dropped in the names)."""
    name: str
    lpp: float
    beam: float
    draught: float
    volume: float                 # displacement volume
    x_g: float                    # centre of gravity from midship, forward positive
    d_prop: float                 # propeller diameter
    h_rudder: float               # rudder span
    a_rudder: float               # rudder area (movable part)
    # hull: resistance and manoeuvring derivatives (eq. 7)
    r0: float
    xvv: float
    xvr: float
    xrr: float
    xvvvv: float
    yv: float
    yr: float
    yvvv: float
    yvvr: float
    yvrr: float
    yrrr: float
    nv: float
    nr: float
    nvvv: float
    nvvr: float
    nvrr: float
    nrrr: float
    # added masses
    mx: float
    my: float
    jz: float
    # propeller (eqs. 8-16)
    t_p: float
    w_p0: float
    k0: float
    k1: float
    k2: float
    x_p: float
    c1: float
    c2_pos: float                 # C2 for beta_P > 0
    c2_neg: float                 # C2 for beta_P < 0
    # rudder (eqs. 18-25)
    t_r: float
    a_h: float
    x_h: float
    x_r: float
    gamma_neg: float              # gamma_R for beta_R < 0
    gamma_pos: float              # gamma_R for beta_R > 0
    l_r: float
    epsilon: float
    kappa: float
    f_alpha: float
    # actuators and service
    rudder_rate_deg: float = 2.32     # deg/s (SOLAS: 35 deg one side to 30 deg the other in 28 s)
    rudder_max_deg: float = 35.0
    gyration: float = 0.25            # radius of yaw gyration / Lpp
    service_speed_kn: float = 15.0
    rpm_lag_s: float = 30.0           # time constant of the revolutions after a telegraph order
    rho: float = RHO_SEA
    windage: Windage = None           # None: no wind forces
    kijima: tuple = None              # hull sway/yaw in Kijima's form instead of eq. 7 (estimate_hull): Y'_b, Y'_r,
                                      # Y'_bb, Y'_rr, Y'_bbr, Y'_brr, N'_b, N'_r, N'_bb, N'_rr, N'_bbr, N'_brr
    depth: float = None               # moulded depth (m): the freeboard is depth - draught; None: 1.45 x draught
    kyy: float = 0.25                 # pitch radius of gyration / Lpp (wave added resistance)
    l_entrance: float = None          # waterline entrance and run lengths (m, IMO's L_E and L_R: from the
    l_run: float = None               # ends to 0.99 of the half beam); None: 0.2 and 0.25 Lpp (estimates)
    source: str = ""

    @property
    def mass(self):
        return self.rho * self.volume

    @property
    def freeboard(self):
        return (self.depth or 1.45 * self.draught) - self.draught

    @property
    def cb(self):
        return self.volume / (self.lpp * self.beam * self.draught)

    @property
    def eta(self):
        """Propeller diameter / rudder span."""
        return self.d_prop / self.h_rudder

    def scaled(self, scale, **over):
        """The same hull at 1/scale (Froude scaling): lengths / scale, areas / scale^2, volume / scale^3, the
        rudder rate x sqrt(scale) (time / sqrt(scale)), the service speed / sqrt(scale)."""
        s = float(scale)
        out = replace(self, lpp=self.lpp / s, beam=self.beam / s, draught=self.draught / s, volume=self.volume / s ** 3,
                      x_g=self.x_g / s, d_prop=self.d_prop / s, h_rudder=self.h_rudder / s,
                      a_rudder=self.a_rudder / s ** 2, rudder_rate_deg=self.rudder_rate_deg * math.sqrt(s),
                      service_speed_kn=self.service_speed_kn / math.sqrt(s), rpm_lag_s=self.rpm_lag_s / math.sqrt(s),
                      windage=self.windage.scaled(s) if self.windage else None,
                      l_entrance=self.l_entrance / s if self.l_entrance else None,
                      l_run=self.l_run / s if self.l_run else None, depth=self.depth / s if self.depth else None)
        return replace(out, **over) if over else out

    def k_t(self, j):
        return self.k2 * j * j + self.k1 * j + self.k0

    def revolutions_for(self, speed):
        """Propeller revolutions (1/s) that hold `speed` (m/s) in a straight line: X_H + X_P = 0 with v_m = r = 0."""
        if speed <= 0:
            return 0.0
        resistance = 0.5 * self.rho * self.lpp * self.draught * speed ** 2 * self.r0
        a = speed * (1.0 - self.w_p0) / self.d_prop
        c = self.k2 * a * a - resistance / ((1.0 - self.t_p) * self.rho * self.d_prop ** 4)
        disc = (self.k1 * a) ** 2 - 4.0 * self.k0 * c
        return (-self.k1 * a + math.sqrt(disc)) / (2.0 * self.k0)


# KVLCC2 (Yasukawa & Yoshimura 2015): full-scale particulars (Table 1) with the Table 3 coefficients
KVLCC2 = Hull(
    "KVLCC2", lpp=320.0, beam=58.0, draught=20.8, volume=312600.0, x_g=11.2, d_prop=9.86, h_rudder=15.80,
    a_rudder=112.5,
    r0=0.022, xvv=-0.040, xvr=0.002, xrr=0.011, xvvvv=0.771,
    yv=-0.315, yr=0.083, yvvv=-1.607, yvvr=0.379, yvrr=-0.391, yrrr=0.008,
    nv=-0.137, nr=-0.049, nvvv=-0.030, nvvr=-0.294, nvrr=0.055, nrrr=-0.013,
    mx=0.022, my=0.223, jz=0.011,
    t_p=0.220, w_p0=0.35, k0=0.2931, k1=-0.2753, k2=-0.1385, x_p=-0.690, c1=2.0, c2_pos=1.6, c2_neg=1.1,
    t_r=0.387, a_h=0.312, x_h=-0.464, x_r=-0.5, gamma_neg=0.395, gamma_pos=0.640, l_r=-0.710, epsilon=1.09,
    kappa=0.50, f_alpha=2.747,
    rudder_rate_deg=1.76, service_speed_kn=15.5, kyy=0.25, depth=30.0,
    windage=Windage(a_front=1161.0, a_lateral=4258.0, loa=330.0, s_l=-40.0, s_h=9.0, kind="tanker_loaded",
                    source="areas as reported for KVLCC2 with an estimated superstructure by Yasukawa et al. (2018), "
                           "J. Mar. Sci. Technol. 23, 'Effect of engine output on maneuverability of a VLCC in still "
                           "water and adverse weather conditions' (text not checked: paywalled); they agree with the "
                           "hull above the waterline (depth 30 m, their Table 1) plus an aft accommodation block. "
                           "Loa, s_l and s_h are estimated."),
    source="Yasukawa & Yoshimura (2015), J. Mar. Sci. Technol. 20(1) 37-52, Tables 1 and 3; x_P' from shipmmg")

# the 7 m free-running model of the same paper (Table 1, L7-model at 1/45.7; w_P0 0.40), for validation
KVLCC2_L7 = replace(KVLCC2.scaled(45.7, w_p0=0.40), name="KVLCC2_L7", lpp=7.00, beam=1.27, draught=0.46, volume=3.27,
                    x_g=0.25, d_prop=0.216, h_rudder=0.345, a_rudder=0.0539)

# --- hulls estimated from their particulars --------------------------------------------------------------------

NU_SEA = 1.19e-6          # kinematic viscosity of sea water, 15 deg C (ITTC)


def estimate_hull(name, lpp, beam, draught, cb, service_speed_kn, d_prop, a_rudder, h_rudder=None, x_g=0.0,
                  depth=None, windage=None, rudder_rate_deg=2.32, source=""):
    """A hull for which no captive or free-running tests are published: its MMG coefficients estimated from its
    particulars (metres, m^2, knots). An estimate: on KVLCC2, whose own coefficients are known, it gives the
    turning and zig-zag of tests/test_shipmodel.py::test_the_estimate_against_kvlcc2.

    Sourced:
    - hull sway force and yaw moment, in Kijima's form (Hull.kijima), and the hull-propeller-rudder interaction
      (1 - t_R, a_H, x'_H, 1 - w_P0, epsilon, gamma_R): Kijima, Katsuno, Nakiri & Furukawa (1990), "On the
      manoeuvring performance of a ship with the parameter of loading condition", J. Soc. Naval Arch. Japan 168,
      141-148, as reproduced in Kim, Lee & Lee (2021), J. Mar. Sci. Eng. 9(6) 569, eq. 7 (open access, CC BY);
      k = 2 d / L there. Kijima's rudder inflow angle takes beta - 2 x'_R r' with x'_R = -0.5: l'_R = -1.
    - added masses: m'_x after Brix, m'_y and J'_z after Clarke, Gedling & Hine ("The application of manoeuvring
      criteria in hull design using linear theory"), as implemented in VesselManoeuvringModels
      (github.com/martinlarsalbert/VesselManoeuvringModels, MIT, vessel_manoeuvring_models/parameters.py),
      converted from their 1/2 rho L^3 (and 1/2 rho L^5) to MMG's 1/2 rho L^2 d (and 1/2 rho L^4 d).
    - rudder lift slope: Fujii's 6.13 L / (L + 2.25) for the aspect ratio L = h_R^2 / A_R (as in the MMG standard).
    Assumed (no source for these hulls): the propeller's open-water curve, C_1, C_2, kappa as KVLCC2's (Yasukawa &
    Yoshimura 2015); t_P 0.22; x'_P -0.5; the surge terms beyond the resistance zero; the resistance R'_0 from the
    ITTC-1957 friction line at the service speed with a form factor of 1.15 and C_A 0.0004 on Mumford's wetted
    surface (1.7 L d + volume / d), no wave resistance; the rudder span 1.25 x the propeller diameter when not
    given; the yaw gyration 0.25 L."""
    L, B, d = float(lpp), float(beam), float(draught)
    vol = cb * L * B * d
    k = 2.0 * d / L
    cbbl = cb * B / L
    m = 2.0 * vol / (L * L * d)                                    # m' = m / (1/2 rho L^2 d)
    # added masses (Brix; Clarke): their primes over 1/2 rho L^3 (L^5 for N_rdot), MMG's over 1/2 rho L^2 d
    mx = m / (math.pi * math.sqrt(L ** 3 / vol) - 14.0)
    my = math.pi * (d / L) ** 2 * (1.0 + 0.16 * cb * B / d - 5.1 * (B / L) ** 2) * L / d
    jz = math.pi * (d / L) ** 2 * (0.0833 + 0.017 * cb * B / d - 0.0033 * B / L) * L / d
    kij = (
        0.5 * math.pi * k + 1.4 * cbbl,                         # Y'_b
        (m + mx) - 1.5 * cbbl,                                  # Y'_r: Y'_r - (m' + m'_x) = -1.5 C_b B / L
        2.5 * d * (1.0 - cb) / B + 0.5,                         # Y'_bb
        0.343 * d * cb / B - 0.07,                              # Y'_rr
        1.5 * d * cb / B - 0.65,                                # Y'_bbr
        5.95 * d * (1.0 - cb) / B,                              # Y'_brr
        k,                                                      # N'_b
        -0.54 * k + k * k,                                      # N'_r
        -0.96 * d * (1.0 - cb) / B + 0.066,                     # N'_bb
        0.5 * cb * B / L - 0.09,                                # N'_rr
        -(57.5 * cbbl ** 2 - 18.4 * cbbl + 1.6),                # N'_bbr
        -(0.5 * d * cb / B - 0.05),                             # N'_brr
    )
    speed = service_speed_kn * KNOT
    re = speed * L / NU_SEA
    cf = 0.075 / (math.log10(re) - 2.0) ** 2
    wetted = 1.7 * L * d + vol / d
    r0 = (1.15 * cf + 0.0004) * wetted / (L * d)
    h_r = h_rudder or 1.25 * d_prop
    aspect = h_r * h_r / a_rudder
    gamma = -22.2 * cbbl ** 2 + 0.02 * cbbl + 0.68
    return Hull(
        name, lpp=L, beam=B, draught=d, volume=vol, x_g=x_g, d_prop=d_prop, h_rudder=h_r, a_rudder=a_rudder,
        r0=r0, xvv=0.0, xvr=0.0, xrr=0.0, xvvvv=0.0,
        yv=-kij[0], yr=kij[1], yvvv=0.0, yvvr=0.0, yvrr=0.0, yrrr=0.0,      # linear parts, for reference
        nv=-kij[6], nr=kij[7], nvvv=0.0, nvvr=0.0, nvrr=0.0, nrrr=0.0,
        mx=mx, my=my, jz=jz,
        t_p=0.22, w_p0=1.0 - (1.05 - 0.5 * cb), k0=KVLCC2.k0, k1=KVLCC2.k1, k2=KVLCC2.k2, x_p=-0.5,
        c1=KVLCC2.c1, c2_pos=KVLCC2.c2_pos, c2_neg=KVLCC2.c2_neg,
        t_r=1.0 - (0.28 * cb + 0.55), a_h=2.2835 * cb ** 2 - 0.833 * cb,
        x_h=9.72289 * cb ** 2 - 8.243538 * cb - 0.00498539, x_r=-0.5,
        gamma_neg=gamma, gamma_pos=gamma, l_r=-1.0,
        epsilon=-156.2 * cbbl ** 2 + 41.6 * cbbl - 1.76, kappa=KVLCC2.kappa,
        f_alpha=6.13 * aspect / (aspect + 2.25),
        rudder_rate_deg=rudder_rate_deg, service_speed_kn=service_speed_kn, depth=depth, windage=windage,
        kijima=kij, source="estimated (estimate_hull: Kijima et al. 1990, Clarke et al., Brix; see its docstring)"
                           + (f"; particulars: {source}" if source else ""))


# KCS, the KRISO container ship of the SIMMAN workshops, estimated: particulars as published for the SIMMAN
# benchmark (Lpp 230 m, B 32.2 m, T 10.8 m, volume 52,030 m^3: C_b 0.6505; propeller 7.9 m; rudder area 115 m^2
# at full scale, from the 1/31.6 model's 0.1152 m^2); its design speed Fn 0.26 is 24 kn. Its own MMG coefficients
# (Okuda, Yasukawa & Matsuda 2023, Ocean Eng. 284, 115174) are not open; until they are copied in, the estimate.
# Estimated besides: the rudder span (1.25 D_P), x_G (0), the windage (a loaded container ship's: the hull above
# the water, about 10 m of containers over 85 % of the length, a 30 m wide bridge front).
KCS = estimate_hull(
    "KCS", lpp=230.0, beam=32.2, draught=10.8, cb=52030.0 / (230.0 * 32.2 * 10.8), service_speed_kn=24.0,
    d_prop=7.9, a_rudder=115.0, depth=19.0,
    windage=Windage(a_front=32.2 * (19.0 - 10.8) + 30.0 * 10.0, a_lateral=230.0 * (19.0 - 10.8) + 0.85 * 230.0 * 10.0,
                    loa=243.0, s_l=-10.0, s_h=9.0, kind="container_loaded",
                    source="estimated: freeboard x length plus 10 m of containers"),
    source="SIMMAN KCS (Lpp, B, T, volume, D_P, A_R); depth 19.0 m, Loa 243 m estimated")

PRESETS = {"kvlcc2": KVLCC2, "kvlcc2_l7": KVLCC2_L7, "kcs": KCS}


def preset(name):
    try:
        return PRESETS[str(name).lower()]
    except KeyError:
        raise ValueError(f"unknown hull {name!r}; known: {', '.join(sorted(PRESETS))}") from None


# --- forces ----------------------------------------------------------------------------------------------------------

def forces(h, u, vm, r, delta, n):
    """X, Y, N_m (N, N, N m) on the ship at midship: hull + propeller + rudder (eqs. 5-25). delta in rad, n in 1/s."""
    p = force_parts(h, u, vm, r, delta, n)
    return tuple(p["hull"][k] + p["propeller"][k] + p["rudder"][k] for k in range(3))


def force_parts(h, u, vm, r, delta, n):
    """The forces of forces() by where they act: {"hull", "propeller", "rudder"}: (X, Y, N_m) each."""
    big_u = math.hypot(u, vm)
    if big_u < 1e-6:
        return {"hull": (0.0, 0.0, 0.0), "propeller": (0.0, 0.0, 0.0), "rudder": (0.0, 0.0, 0.0)}
    L, d, rho = h.lpp, h.draught, h.rho
    vp, rp = vm / big_u, r * L / big_u            # v_m', r'
    beta = math.atan2(-vm, u)                     # drift angle at midship
    q = 0.5 * rho * L * d * big_u ** 2
    # hull (eqs. 6, 7)
    xh = q * (-h.r0 + h.xvv * vp ** 2 + h.xvr * vp * rp + h.xrr * rp ** 2 + h.xvvvv * vp ** 4)
    if h.kijima is not None:      # Kijima et al. (1990): in the drift angle, with |.| terms (estimate_hull)
        yb, yr_, ybb, yrr, ybbr, ybrr, nb, nr_, nbb, nrr, nbbr, nbrr = h.kijima
        b = beta
        yh = q * (yb * b + yr_ * rp + ybb * b * abs(b) + yrr * rp * abs(rp) + (ybbr * b + ybrr * rp) * b * rp)
        nh = q * L * (nb * b + nr_ * rp + nbb * b * abs(b) + nrr * rp * abs(rp) + (nbbr * b + nbrr * rp) * b * rp)
    else:
        yh = q * (h.yv * vp + h.yr * rp + h.yvvv * vp ** 3 + h.yvvr * vp ** 2 * rp + h.yvrr * vp * rp ** 2
                  + h.yrrr * rp ** 3)
        nh = q * L * (h.nv * vp + h.nr * rp + h.nvvv * vp ** 3 + h.nvvr * vp ** 2 * rp + h.nvrr * vp * rp ** 2
                      + h.nrrr * rp ** 3)
    # propeller (eqs. 8-16)
    beta_p = beta - h.x_p * rp
    c2 = h.c2_pos if beta_p > 0 else h.c2_neg
    one_wp = (1.0 - h.w_p0) * (1.0 + (1.0 - math.exp(-h.c1 * abs(beta_p))) * (c2 - 1.0))
    if n > 1e-6:
        j = u * one_wp / (n * h.d_prop)
        kt = h.k_t(j)
        xp = (1.0 - h.t_p) * rho * n * n * h.d_prop ** 4 * kt
    else:
        j, kt, xp = 0.0, 0.0, 0.0
    # rudder (eqs. 18-25)
    beta_r = beta - h.l_r * rp
    gamma = h.gamma_neg if beta_r < 0 else h.gamma_pos
    v_r = big_u * gamma * beta_r
    if n > 1e-6 and j > 1e-6:
        race = h.eta * (1.0 + h.kappa * (math.sqrt(1.0 + 8.0 * kt / (math.pi * j * j)) - 1.0)) ** 2
    else:      # no propeller race (stopped propeller), or a propeller turning with no advance: bounded
        race = h.eta * (1.0 + h.kappa) ** 2 if n > 1e-6 else h.eta
    u_r = h.epsilon * u * one_wp * math.sqrt(race + (1.0 - h.eta))
    alpha_r = delta - math.atan2(v_r, u_r)
    f_n = 0.5 * rho * h.a_rudder * (u_r ** 2 + v_r ** 2) * h.f_alpha * math.sin(alpha_r)
    xr = -(1.0 - h.t_r) * f_n * math.sin(delta)
    yr = -(1.0 + h.a_h) * f_n * math.cos(delta)
    nr = -(h.x_r + h.a_h * h.x_h) * L * f_n * math.cos(delta)
    return {"hull": (xh, yh, nh), "propeller": (xp, 0.0, 0.0), "rudder": (xr, yr, nr)}


def wind_forces(h, u_rw, v_rw):
    """X, Y, N (midship) from the wind; u_rw, v_rw: the ship's velocity relative to the air in the body frame (m/s).
    Blendermann (1994) as in MSS blendermann94.m."""
    w = h.windage
    vr2 = u_rw * u_rw + v_rw * v_rw
    if w is None or vr2 < 1e-12:
        return 0.0, 0.0, 0.0
    cx, cy, cn = wind_coefficients(w, -math.atan2(v_rw, u_rw))
    q = 0.5 * RHO_AIR * vr2
    return q * cx * w.a_front, q * cy * w.a_lateral, q * cn * w.a_lateral * w.loa


@dataclass
class Environment:
    """Uniform, steady current and wind. current_to_deg: where the current sets towards (from north, clockwise, the
    nautical convention); wind_from_deg: where the wind comes from (the meteorological convention, like Open-Meteo's
    wind_direction_10m); wind_mps at 10 m. Waves: significant height wave_hs (m), peak period wave_tp (s), and where they come from
    (wave_from_deg, like Open-Meteo marine's wave_direction)."""
    current_mps: float = 0.0
    current_to_deg: float = 0.0
    wind_mps: float = 0.0
    wind_from_deg: float = 0.0
    wave_hs: float = 0.0
    wave_tp: float = 0.0
    wave_from_deg: float = 0.0

    def current_ne(self):
        a = math.radians(self.current_to_deg)
        return self.current_mps * math.cos(a), self.current_mps * math.sin(a)

    def wind_ne(self):
        """The air's velocity (north, east), m/s: it blows towards wind_from + 180 deg."""
        a = math.radians(self.wind_from_deg)
        return -self.wind_mps * math.cos(a), -self.wind_mps * math.sin(a)


def to_body(n, e, psi):
    """An earth-frame vector (north, east) in the body frame (forward, starboard)."""
    c, s = math.cos(psi), math.sin(psi)
    return n * c + e * s, -n * s + e * c


# --- waves -----------------------------------------------------------------------------------------------------------

def jonswap(omega, hs, tp, gamma=3.3):
    """JONSWAP spectrum S(omega) (m^2 s/rad) with peak factor gamma: the Pierson-Moskowitz shape times the peak
    enhancement (sigma 0.07 / 0.09), scaled numerically so that its area m0 = hs^2 / 16."""
    wp = 2.0 * math.pi / tp

    def shape(w):
        sigma = np.where(w <= wp, 0.07, 0.09)
        return w ** -5.0 * np.exp(-1.25 * (wp / w) ** 4) * gamma ** np.exp(-0.5 * ((w - wp) / (sigma * wp)) ** 2)

    grid = np.linspace(0.2 * wp, 8.0 * wp, 4000)
    return hs * hs / 16.0 * shape(np.asarray(omega, dtype=float)) / np.trapezoid(shape(grid), grid)


def _alpha_d(d_star, lam, lpp):
    """IMO's draught coefficient: how much of a wave a hull of draught d_star reflects (0 for waves > 2.5 L)."""
    return np.where(lam / lpp > 2.5, 0.0, 1.0 - np.exp(-4.0 * math.pi * (d_star / lam - d_star / (2.5 * lpp))))


def added_resistance_rao(h, omega, beta, speed):
    """X'_d = X_d / A^2 (N/m^2), the added resistance in regular waves of frequency omega (rad/s, array) from
    direction beta (rad, IMO's: pi head seas, 0 following; port and starboard alike) at speed (m/s, through the
    water). IMO MEPC.1/Circ.850/Rev.3, appendix to appendix 2: X'_dM + X'_dR."""
    w = np.asarray(omega, dtype=float)
    L, B, d, cb, rho = h.lpp, h.beam, h.draught, h.cb, h.rho
    beta = min(abs(beta), math.pi)
    fr = max(speed, 0.0) / math.sqrt(G * L)
    lam = 2.0 * math.pi * G / w ** 2
    cosb = math.cos(beta)
    # motion part (given for head to beam seas)
    x_m = np.zeros_like(w)
    if beta >= math.pi / 2:
        base = (2.142 * h.kyy ** (1 / 3) * np.sqrt(L / lam) * (1.0 - 0.111 / cb * (math.log(B / d) - math.log(2.75)))
                * (2.0 - cosb) / 3.0)
        wbar = base * ((fr + 0.62) if fr < 0.1 else fr ** 0.143)
        a1 = (60.3 * cb ** 1.34 * (4.0 * h.kyy) ** 2 * (0.87 / cb) ** (-(1.0 + fr) * cosb)
              / math.log(B / d) * (1.0 - 2.0 * cosb) / 3.0)
        a2 = 0.0072 + 0.1676 * fr if fr < 0.12 else fr ** 1.5 * math.exp(-3.5 * fr)
        below = wbar < 1.0
        b1 = np.where(below, 11.0, -8.5)
        far = -566.0 * (L / B) ** -2.66 * 6.0
        d1 = np.where(below, 566.0 * (L / B) ** -2.66 if cb > 0.75 else 14.0, far)
        x_m = 4.0 * rho * G * B * B / L * a1 * a2 * wbar ** b1 * np.exp(b1 / d1 * (1.0 - wbar ** d1))
    # reflection part, per waterline segment (S1, S2 at the bow; S3, S4 at the stern)
    e1 = math.atan(0.495 * B / (h.l_entrance or 0.20 * L))
    e2 = math.atan(0.495 * B / (h.l_run or 0.25 * L))
    k = 2.25 / 4.0 * rho * G * B
    sp = 2.0 * w * max(speed, 0.0) / G
    f_beta = -cosb if beta >= math.pi - e1 else 0.0
    bow_factor = (0.87 / cb) ** ((1.0 + 4.0 * math.sqrt(fr)) * f_beta)
    a_bow = _alpha_d(d, lam, L)
    d_stern = d * (4.0 + math.sqrt(abs(cosb))) / 5.0 if cb <= 0.75 else d * (2.0 + math.sqrt(abs(cosb))) / 3.0
    a_stern = _alpha_d(d_stern, lam, L)
    x_r = np.zeros_like(w)
    if e1 <= beta:
        x_r += k * a_bow * (math.sin(e1 - beta) ** 2 + sp * (math.cos(e1) * math.cos(e1 - beta) - cosb)) * bow_factor
    if math.pi - e1 <= beta:
        x_r += k * a_bow * (math.sin(e1 + beta) ** 2 + sp * (math.cos(e1) * math.cos(e1 + beta) - cosb)) * bow_factor
    if beta <= math.pi - e2:
        x_r -= k * a_stern * (math.sin(e2 + beta) ** 2 + sp * (math.cos(e2) * math.cos(e2 + beta) - cosb))
    if beta <= e2:
        x_r -= k * a_stern * (math.sin(e2 - beta) ** 2 + sp * (math.cos(e2) * math.cos(e2 - beta) - cosb))
    return x_m + x_r


def side_drift_rao(h, omega, beta):
    """|Y_d| / A^2 (N/m^2): full reflection on the ship's side (1/2 rho g per metre, over Lpp) times sin^2 of the
    incidence and IMO's draught coefficient. An estimate (the short-wave limit), not part of the IMO method."""
    lam = 2.0 * math.pi * G / np.asarray(omega, dtype=float) ** 2
    return 0.5 * h.rho * G * h.lpp * math.sin(beta) ** 2 * _alpha_d(h.draught, lam, h.lpp)


WAVE_OMEGA = np.linspace(0.15, 3.0, 96)       # rad/s: periods of 2-42 s


def wave_drift(h, env, psi, speed):
    """Mean wave drift X, Y (N, body frame) in env's long-crested irregular sea: 2 x integral of rao x S."""
    if env is None or env.wave_hs <= 0 or env.wave_tp <= 0:
        return 0.0, 0.0
    rel = wrap(math.radians(env.wave_from_deg) - psi)        # where the waves come from, relative to the bow
    beta = math.pi - abs(rel)
    s = jonswap(WAVE_OMEGA, env.wave_hs, env.wave_tp)
    x = 2.0 * np.trapezoid(added_resistance_rao(h, WAVE_OMEGA, beta, speed) * s, WAVE_OMEGA)
    y = 2.0 * np.trapezoid(side_drift_rao(h, WAVE_OMEGA, beta) * s, WAVE_OMEGA)
    return -float(x), -math.copysign(float(y), rel)        # resistance acts aft; pushed away from the waves


def accelerations(h, u, vm, r, delta, n, ext=(0.0, 0.0, 0.0)):
    """du/dt, dv_m/dt, dr/dt from eq. 4 (a 2 x 2 solve for sway and yaw); ext: external X, Y, N (wind)."""
    m = h.mass
    scale = 0.5 * h.rho * h.lpp ** 2 * h.draught
    mx, my, jz = h.mx * scale, h.my * scale, h.jz * scale * h.lpp ** 2
    izg = m * (h.gyration * h.lpp) ** 2
    x, y, nm = forces(h, u, vm, r, delta, n)
    x, y, nm = x + ext[0], y + ext[1], nm + ext[2]
    du = (x + (m + my) * vm * r + h.x_g * m * r * r) / (m + mx)
    # [[m + my, xG m], [xG m, IzG + xG^2 m + Jz]] [dv, dr] = [Y - (m + mx) u r, N - xG m u r]
    a11, a12 = m + my, h.x_g * m
    a21, a22 = h.x_g * m, izg + h.x_g ** 2 * m + jz
    b1, b2 = y - (m + mx) * u * r, nm - h.x_g * m * u * r
    det = a11 * a22 - a12 * a21
    return du, (b1 * a22 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det


# --- a ship ----------------------------------------------------------------------------------------------------------

@dataclass
class Ship:
    """A ship's state and orders. x, y: midship in the earth frame (m, north and east); psi: heading (rad, from
    north, clockwise); u, vm (m/s, through the water), r (rad/s); delta: rudder (rad), n: revolutions (1/s). Orders:
    rudder_cmd (rad) and n_cmd (1/s); with an autopilot, rudder_cmd comes from it. env: current and wind (None:
    calm, still water)."""
    hull: Hull
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    u: float = 0.0
    vm: float = 0.0
    r: float = 0.0
    delta: float = 0.0
    n: float = 0.0
    rudder_cmd: float = 0.0
    n_cmd: float = 0.0
    lag_revolutions: bool = True
    autopilot: "Autopilot" = None
    env: Environment = None
    t: float = 0.0
    wave_force: tuple = (0.0, 0.0)        # the mean wave drift (X, Y) over the last step
    wave_every_s: float = 0.0             # recompute the wave drift at most this often (0: every step)
    _wave_t: float = field(default=-1e9, repr=False)
    log: list = field(default_factory=list, repr=False)

    @classmethod
    def at_speed(cls, hull, speed_kn=None, x=0.0, y=0.0, psi=0.0, **kw):
        """A ship sailing straight ahead at speed_kn (default the hull's service speed), revolutions to hold it."""
        speed = (hull.service_speed_kn if speed_kn is None else speed_kn) * KNOT
        n = hull.revolutions_for(speed)
        return cls(hull, x=x, y=y, psi=psi, u=speed, n=n, n_cmd=n, **kw)

    @property
    def speed(self):
        """Speed through the water (m/s)."""
        return math.hypot(self.u, self.vm)

    def velocity_ne(self):
        """Velocity over the ground (north, east), m/s."""
        c, s = math.cos(self.psi), math.sin(self.psi)
        cn, ce = self.env.current_ne() if self.env else (0.0, 0.0)
        return self.u * c - self.vm * s + cn, self.u * s + self.vm * c + ce

    @property
    def speed_over_ground(self):
        return math.hypot(*self.velocity_ne())

    @property
    def course(self):
        """Course over the ground (rad, from north, clockwise)."""
        return math.atan2(self.velocity_ne()[1], self.velocity_ne()[0])

    def order_speed(self, speed_kn):
        """Engine order: the revolutions that would hold speed_kn in a straight line (the telegraph's steps)."""
        self.n_cmd = self.hull.revolutions_for(max(0.0, speed_kn) * KNOT)

    def _wind(self, psi, u, vm):
        """Wind X, Y, N on the ship (body frame); the relative wind from the velocity over the ground."""
        if self.env is None or self.hull.windage is None:
            return 0.0, 0.0, 0.0
        cn, ce = self.env.current_ne()
        wn, we = self.env.wind_ne()
        uw, vw = to_body(cn - wn, ce - we, psi)          # current minus air, in the body frame
        return wind_forces(self.hull, u + uw, vm + vw)

    def force_breakdown(self):
        """Every force on the ship now (body frame, N and N m about midship): {"hull", "propeller", "rudder",
        "wind", "waves"}: (X, Y, N) each."""
        p = force_parts(self.hull, self.u, self.vm, self.r, self.delta, self.n)
        p["wind"] = self._wind(self.psi, self.u, self.vm)
        p["waves"] = (self.wave_force[0], self.wave_force[1], 0.0)
        return p

    def _deriv(self, s, delta, n):
        x, y, psi, u, vm, r = s
        cn = ce = 0.0
        ext = (self.wave_force[0], self.wave_force[1], 0.0)
        if self.env is not None:
            cn, ce = self.env.current_ne()
            xw, yw, nw = self._wind(psi, u, vm)
            ext = (ext[0] + xw, ext[1] + yw, nw)
        du, dv, dr = accelerations(self.hull, u, vm, r, delta, n, ext)
        c, si = math.cos(psi), math.sin(psi)
        return (u * c - vm * si + cn, u * si + vm * c + ce, r, du, dv, dr)

    def step(self, dt):
        """Advance dt seconds: autopilot, rudder towards its order at the rudder rate, revolutions towards theirs,
        then RK4 on the motion with the actuators held over the step."""
        h = self.hull
        if self.autopilot is not None:
            self.rudder_cmd = self.autopilot.rudder(self, dt)
        lim = math.radians(h.rudder_max_deg)
        cmd = max(-lim, min(lim, self.rudder_cmd))
        rate = math.radians(h.rudder_rate_deg) * dt
        self.delta += max(-rate, min(rate, cmd - self.delta))
        target = max(0.0, self.n_cmd)
        if self.lag_revolutions and h.rpm_lag_s > 0:
            self.n += (target - self.n) * (1.0 - math.exp(-dt / h.rpm_lag_s))
        else:
            self.n = target
        if self.t - self._wave_t >= self.wave_every_s:                  # slowly varying: once per step at most
            self.wave_force = wave_drift(h, self.env, self.psi, self.u)
            self._wave_t = self.t
        s = (self.x, self.y, self.psi, self.u, self.vm, self.r)
        k1 = self._deriv(s, self.delta, self.n)
        k2 = self._deriv(tuple(a + 0.5 * dt * b for a, b in zip(s, k1)), self.delta, self.n)
        k3 = self._deriv(tuple(a + 0.5 * dt * b for a, b in zip(s, k2)), self.delta, self.n)
        k4 = self._deriv(tuple(a + dt * b for a, b in zip(s, k3)), self.delta, self.n)
        self.x, self.y, self.psi, self.u, self.vm, self.r = (
            a + dt / 6.0 * (b1 + 2 * b2 + 2 * b3 + b4) for a, b1, b2, b3, b4 in zip(s, k1, k2, k3, k4))
        self.t += dt


def wrap(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Autopilot:
    """Heading autopilot (PD on the heading error, rudder limited), optionally following waypoints [(x, y)] in the
    earth frame: it steers for the next waypoint and takes the one after once within `reach_m` of it (default one
    ship length). gains: rudder (rad) per rad of heading error and per rad/s of yaw rate; the integral term (per
    rad s of error, only while the error is small) trims out a steady wind moment."""
    heading: float = 0.0
    kp: float = 2.0
    kd: float = 0.0          # set from the hull when 0: about the ship's yaw time constant
    ki: float = None         # set from the hull when None: kp / (10 L/U); 0 turns it off
    integral_band_deg: float = 5.0
    integral: float = 0.0
    max_rudder_deg: float = 20.0
    waypoints: list = field(default_factory=list)
    reach_m: float = 0.0
    index: int = 0

    @property
    def done(self):
        return bool(self.waypoints) and self.index >= len(self.waypoints)

    def rudder(self, ship, dt):
        if self.waypoints and self.index < len(self.waypoints):
            wx, wy = self.waypoints[self.index]
            reach = self.reach_m or ship.hull.lpp
            if math.hypot(wx - ship.x, wy - ship.y) < reach:
                self.index += 1
            if self.index < len(self.waypoints):
                wx, wy = self.waypoints[self.index]
                self.heading = math.atan2(wy - ship.y, wx - ship.x)
        tau = ship.hull.lpp / max(ship.speed, 0.5)
        kd = self.kd or 2.0 * tau
        ki = self.kp / (10.0 * tau) if self.ki is None else self.ki
        err = wrap(self.heading - ship.psi)
        lim = math.radians(self.max_rudder_deg)
        if abs(err) < math.radians(self.integral_band_deg):
            self.integral = max(-lim / max(ki, 1e-12), min(lim / max(ki, 1e-12), self.integral + err * dt))
        return max(-lim, min(lim, self.kp * err - kd * ship.r + ki * self.integral))


# --- frames ----------------------------------------------------------------------------------------------------------

def to_carla(x_north, y_east, psi):
    """Earth frame (x north, y east, heading from north clockwise, rad) -> CARLA (x east, y south, yaw deg clockwise
    from east)."""
    return y_east, -x_north, math.degrees(psi) - 90.0


def from_carla(x, y, yaw_deg):
    """CARLA (x east, y south, yaw deg) -> earth frame (x north, y east, psi rad)."""
    return -y, x, wrap(math.radians(yaw_deg + 90.0))


# --- standard manoeuvres (IMO MSC.137(76) and the paper's Sect. 5) ------------------------------------------------

def turning_circle(hull, rudder_deg=35.0, speed_kn=None, dt=None, max_s=None):
    """35 deg turning test from a straight approach at speed_kn with the approach revolutions held. Returns
    {advance, tactical_diameter} in ship lengths: the distance along the approach heading when the heading has
    changed 90 deg, and across it at 180 deg, both from where the rudder order is given."""
    ship = Ship.at_speed(hull, speed_kn, lag_revolutions=False)
    L = hull.lpp
    dt = dt or 0.002 * L / max(ship.speed, 1e-3) * 10      # ~200 steps per ship length sailed
    max_s = max_s or 60.0 * L / ship.speed
    ship.rudder_cmd = math.radians(rudder_deg)
    sign = 1.0 if rudder_deg > 0 else -1.0
    advance = tactical = None
    turned, prev = 0.0, ship.psi
    while ship.t < max_s and tactical is None:
        ship.step(dt)
        turned += wrap(ship.psi - prev)
        prev = ship.psi
        if advance is None and sign * turned >= math.pi / 2:
            advance = ship.x / L
        if sign * turned >= math.pi:
            tactical = abs(ship.y) / L
    return {"advance": advance, "tactical_diameter": tactical}


def zigzag(hull, rudder_deg=10.0, check_deg=None, speed_kn=None, dt=None, max_s=None):
    """rudder/check zig-zag (10/10, 20/20; negative angles start to port). Returns the first and second overshoot
    angles (deg): how far the heading goes past the check heading after the first and second rudder reversals."""
    check = math.radians(abs(check_deg if check_deg is not None else rudder_deg))
    sign = 1.0 if rudder_deg > 0 else -1.0
    ship = Ship.at_speed(hull, speed_kn, lag_revolutions=False)
    L = hull.lpp
    dt = dt or 0.002 * L / max(ship.speed, 1e-3) * 10
    max_s = max_s or 40.0 * L / ship.speed
    ship.rudder_cmd = sign * math.radians(abs(rudder_deg))
    side, last, extreme, overshoots = sign, None, 0.0, []     # side: where the heading is going now
    while ship.t < max_s and len(overshoots) < 2:
        ship.step(dt)
        h = ship.psi
        if last is not None:           # how far past the check on the side last reached
            extreme = max(extreme, last * h)
        if side * h >= check:          # the heading reached the check on this side: reverse the rudder
            if last is not None:
                overshoots.append(math.degrees(extreme - check))
            last, extreme = side, side * h
            side = -side
            ship.rudder_cmd = side * math.radians(abs(rudder_deg))
    return {"osa1": overshoots[0] if overshoots else None, "osa2": overshoots[1] if len(overshoots) > 1 else None}
