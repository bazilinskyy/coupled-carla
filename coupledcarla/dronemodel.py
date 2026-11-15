"""Quadrotor flight model, flight controller, turbulence, flocking, mission guidance and battery: the engine-free
drone model of coupled-sim (Assets/Scripts/Drones/Models/Quadrotor.cs and Flocking.cs, PR #190) ported to numpy,
vectorised across drones so that a whole fleet steps in one set of array operations.

Dynamics (per drone, X configuration). A rigid body with rotor thrust and reaction torque proportional to the square of
the rotor speed (T = kF w^2, Q = kM w^2), first-order motor lag, linear and quadratic airframe drag relative to the air
(wind and gusts), rotational damping and the gyroscopic term w x Iw in Euler's equations: the standard multirotor model
(Mahony, Kumar & Corke 2012, "Multirotor aerial vehicles: modeling, estimation, and control of quadrotor", IEEE
Robotics & Automation Magazine 19(3); Beard & McLain 2012, Small Unmanned Aircraft, Princeton UP). Integrated at
500 Hz (2 ms sub-steps) inside Fleet.step.

Flight controller, a cascade of the kind PX4 and ArduPilot run: position (P) -> velocity (PI with anti-windup while
the tilt limit saturates) -> thrust vector -> desired attitude by the geometric mapping of Lee, Leok & McClamroch
(2010, "Geometric tracking control of a quadrotor UAV on SE(3)", IEEE CDC) -> body rates (P on the attitude error
e_R = 1/2 vee(Rd'R - R'Rd)) -> torques (rate P with gyroscopic compensation) -> rotor speeds (the X mixer's inverse).
Modes: position (hold or fly to a point, with a velocity feed-forward), velocity (a pilot's sticks in "GPS" mode),
attitude (tilt and climb rate from the sticks, drifting with the wind: "ATTI" mode) and off.

Turbulence: the Dryden model's low-altitude form (MIL-F-8785C) as first-order Gauss-Markov gusts.
Flocking: Vasarhelyi et al. (2018), "Optimized flocking of autonomous drones in confined environments", Science
Robotics 3(20) eaat3536.
Battery (not in coupled-sim): electrical power from momentum theory (ideal induced power T^1.5 / sqrt(2 rho A) per
rotor over a figure of merit; Leishman 2006, Principles of Helicopter Aerodynamics, 2nd ed., ch. 2-3) plus the drag
power and the avionics, over the motor and ESC efficiency (the structure of Liu et al. 2017, "A power consumption
model for multi-rotor small unmanned aircraft systems", ICUAS).

Frames: world x east, y north, z up (ENU); body x forward, y left, z up (the thrust axis). drones.py converts to
CARLA's (x east, y south, z up, yaw clockwise from east).
"""
import math
from dataclasses import dataclass, field, replace

import numpy as np

G = 9.81
SUB_STEP = 0.002          # 500 Hz
RHO = 1.225               # air density (kg/m^3)
ANGLES = np.array([math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4])   # FL, RL, RR, FR
SIN4, COS4 = np.sin(ANGLES), np.cos(ANGLES)
SPIN = np.array([1.0, -1.0, 1.0, -1.0])        # reaction torque sign about +z per rotor
OFF, POSITION, VELOCITY, ATTITUDE = 0, 1, 2, 3
MODES = {"off": OFF, "position": POSITION, "velocity": VELOCITY, "attitude": ATTITUDE}
MODE_NAMES = {v: k for k, v in MODES.items()}


@dataclass(frozen=True)
class Airframe:
    """Physical parameters of a quadrotor (coupled-sim's QuadrotorParameters), plus its battery and noise."""
    name: str
    mass: float                 # take-off mass (kg)
    arm: float                  # centre to rotor axis (m)
    ixx: float
    iyy: float
    izz: float                  # moments of inertia about body x, y, z (kg m^2)
    kf: float                   # thrust coefficient (N per (rad/s)^2)
    km: float                   # reaction torque coefficient (N m per (rad/s)^2)
    motor_tau: float            # motor time constant (s)
    w_min: float
    w_max: float                # rotor speed range (rad/s)
    lin_drag: float             # N per m/s
    quad_drag: float            # N per (m/s)^2
    rot_drag: float             # N m per rad/s
    rotor_radius: float         # m
    blades: int
    max_tilt: float             # rad
    max_speed: float            # horizontal m/s
    max_climb: float
    max_descent: float          # m/s
    max_yaw_rate: float         # rad/s
    # battery and power (not in coupled-sim): capacity, figure of merit, motor+ESC efficiency, avionics power
    battery_wh: float = 59.0
    figure_of_merit: float = 0.55
    drive_efficiency: float = 0.75
    avionics_w: float = 8.0
    # A-weighted sound power level in hover (dB re 1 pW), see drones.py and soundscape.py
    sound_power_dba: float = 81.0

    @property
    def hover_w(self):
        return math.sqrt(self.mass * G / (4.0 * self.kf))

    @property
    def thrust_to_weight(self):
        return 4.0 * self.kf * self.w_max ** 2 / (self.mass * G)

    @property
    def size(self):
        """Scale of the body against the consumer drone (arm 0.19 m)."""
        return self.arm / 0.19


# A folding consumer camera drone (about 0.9 kg, 0.38 m diagonal, 9-inch props): hovers at about 600 rad/s,
# thrust-to-weight about 3, up to 15 m/s, 6 m/s climb, 5 m/s descent. Battery 59 Wh (Mavic 2 class: about 31 min
# hover). Sound: the EU limit for class C1 (< 900 g) from 2024, LWA 81 dB (Delegated Regulation (EU) 2019/945,
# Annex Part 15).
CONSUMER = Airframe("Consumer", mass=0.9, arm=0.19, ixx=0.005, iyy=0.005, izz=0.009, kf=6.13e-6, km=1.0e-7,
                    motor_tau=0.03, w_min=150.0, w_max=1100.0, lin_drag=0.12, quad_drag=0.014, rot_drag=0.002,
                    rotor_radius=0.12, blades=2, max_tilt=0.61, max_speed=15.0, max_climb=6.0, max_descent=5.0,
                    max_yaw_rate=2.0, battery_wh=59.0, figure_of_merit=0.55, drive_efficiency=0.75, avionics_w=8.0,
                    sound_power_dba=81.0)
# An industrial inspection or delivery quadrotor (about 6.3 kg, 0.9 m diagonal, 21-inch props): hovers at about
# 350 rad/s, thrust-to-weight about 3.4, up to 17 m/s. Battery 548 Wh (two 274 Wh packs, M300 class: about 55 min).
# Sound: the C2 limit's mass law, 81 + 18.5 lg(m / 0.9 kg) dB, extended to 6.3 kg: LWA 96.6 dB.
INDUSTRIAL = Airframe("Industrial", mass=6.3, arm=0.45, ixx=0.17, iyy=0.17, izz=0.30, kf=1.26e-4, km=2.5e-6,
                      motor_tau=0.06, w_min=90.0, w_max=650.0, lin_drag=0.6, quad_drag=0.11, rot_drag=0.03,
                      rotor_radius=0.27, blades=2, max_tilt=0.52, max_speed=17.0, max_climb=6.0, max_descent=5.0,
                      max_yaw_rate=1.5, battery_wh=548.0, figure_of_merit=0.65, drive_efficiency=0.8,
                      avionics_w=20.0, sound_power_dba=96.6)
PRESETS = {"consumer": CONSUMER, "industrial": INDUSTRIAL, "delivery": INDUSTRIAL}


def preset(name):
    """Consumer or Industrial (also "delivery"); anything else: Consumer, as coupled-sim's Preset()."""
    return PRESETS.get(str(name or "").lower(), CONSUMER)


def mix_matrix(p):
    """[T, tx, ty, tz] = M . [w1^2, w2^2, w3^2, w4^2] for the X configuration."""
    return np.array([np.full(4, p.kf), p.kf * p.arm * SIN4, -p.kf * p.arm * COS4, SPIN * p.km])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# -- quaternions (x, y, z, w), body to world, as System.Numerics ---------------------------------------------------

def q_from_yaw(yaw):
    yaw = np.asarray(yaw, float)
    out = np.zeros(yaw.shape + (4,))
    out[..., 2] = np.sin(yaw / 2)
    out[..., 3] = np.cos(yaw / 2)
    return out


def q_matrix(q):
    """(N, 4) -> (N, 3, 3) rotation matrices (columns: body x, y, z in the world)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z)
    r[:, 0, 1] = 2 * (x * y - z * w)
    r[:, 0, 2] = 2 * (x * z + y * w)
    r[:, 1, 0] = 2 * (x * y + z * w)
    r[:, 1, 1] = 1 - 2 * (x * x + z * z)
    r[:, 1, 2] = 2 * (y * z - x * w)
    r[:, 2, 0] = 2 * (x * z - y * w)
    r[:, 2, 1] = 2 * (y * z + x * w)
    r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def q_mul_vec(q, v):
    """Hamilton product q (x) (v, 0) for (N, 4) and (N, 3)."""
    x1, y1, z1, w1 = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    x2, y2, z2 = v[:, 0], v[:, 1], v[:, 2]
    return np.stack([w1 * x2 + y1 * z2 - z1 * y2,
                     w1 * y2 + z1 * x2 - x1 * z2,
                     w1 * z2 + x1 * y2 - y1 * x2,
                     -x1 * x2 - y1 * y2 - z1 * z2], axis=1)


def _norm(v, axis=-1):
    return np.sqrt(np.sum(v * v, axis=axis))


class Fleet:
    """Any number of quadrotors stepped together. Per drone i: pos, vel (world), q (body to world, x y z w), rates
    (body), rotors (FL, RL, RR, FR, rad/s), on_ground; the command arrays mode, cmd_pos, cmd_vel, cmd_yaw,
    cmd_yaw_rate, use_yaw_rate, cmd_roll, cmd_pitch (positive nose down: forward), cmd_climb; wind (world, gusts
    included) and ground (the height of the ground or roof under the drone). Set them, then step(dt).

    Inside, every vector is stored as rows (3, N) and the 500 Hz sub-step is written component by component: a sub-step
    is about 200 numpy operations on arrays of N, so its cost hardly depends on N (the overhead of a numpy call, not
    the arithmetic, is what costs at a few dozen drones). The (N, 3) attributes are views of those rows."""

    # controller gains (per second), coupled-sim's
    KP_XY, KP_Z, KV_XY, KV_Z, KI_XY, KI_Z = 1.0, 1.2, 2.2, 4.0, 1.0, 1.5
    K_ATT, K_YAW, K_RATE, K_YAW_RATE = 7.0, 3.0, 25.0, 10.0
    PARAMS = ("mass", "arm", "kf", "km", "motor_tau", "w_min", "w_max", "lin_drag", "quad_drag", "rot_drag",
              "max_tilt", "max_speed", "max_climb", "max_descent", "max_yaw_rate", "rotor_radius", "blades",
              "figure_of_merit", "drive_efficiency", "avionics_w", "battery_wh")
    VEC3 = ("pos", "vel", "rates", "cmd_pos", "cmd_vel", "wind", "vel_int", "last_err", "p_inertia")

    def __init__(self):
        self.airframes = []
        self.n = 0
        self.time = 0.0
        self._alloc(0)

    def _alloc(self, n):
        for k in self.VEC3:
            setattr(self, "_" + k, np.zeros((3, n)))
        self._q = np.zeros((4, n))
        self._q[3] = 1.0
        self._rotors = np.zeros((4, n))
        self.on_ground = np.zeros(n, bool)
        self.mode = np.zeros(n, int)
        self.cmd_yaw, self.cmd_yaw_rate = np.zeros(n), np.zeros(n)
        self.use_yaw_rate = np.zeros(n, bool)
        self.cmd_roll, self.cmd_pitch, self.cmd_climb = np.zeros(n), np.zeros(n), np.zeros(n)
        self.ground = np.zeros(n)
        self.yaw_target = np.zeros(n)
        self.energy_wh = np.zeros(n)        # used since the start
        self.power_w = np.zeros(n)
        for k in self.PARAMS:
            setattr(self, "p_" + k, np.zeros(n))
        self._minv = np.zeros((4, 4, n))   # mixer inverse per drone
        self._derived()

    def _derived(self):
        self._max_acc = G * np.tan(self.p_max_tilt)
        self._mg = self.p_mass * G
        self._w2h = self.p_mass * G / (4.0 * np.maximum(self.p_kf, 1e-12))
        self._lo2, self._hi2 = self.p_w_min ** 2, self.p_w_max ** 2
        self._kfl = self.p_kf * self.p_arm
        self._area_k = 1.0 / (np.sqrt(2.0 * RHO * np.pi * np.maximum(self.p_rotor_radius, 1e-6) ** 2) *
                              np.maximum(self.p_figure_of_merit, 1e-6))

    # (N, 3) and (N, 4) views of the rows
    pos = property(lambda s: s._pos.T)
    vel = property(lambda s: s._vel.T)
    rates = property(lambda s: s._rates.T)
    cmd_pos = property(lambda s: s._cmd_pos.T)
    cmd_vel = property(lambda s: s._cmd_vel.T)
    wind = property(lambda s: s._wind.T)
    vel_int = property(lambda s: s._vel_int.T)
    q = property(lambda s: s._q.T)
    rotors = property(lambda s: s._rotors.T)
    p_inertia = property(lambda s: s._p_inertia.T)

    def add(self, airframe, pos, yaw=0.0, on_ground=True):
        """Adds a drone at rest at pos (world, ENU) with heading yaw (rad from east towards north); returns its index."""
        old = {k: v.copy() for k, v in vars(self).items() if isinstance(v, np.ndarray)}
        n = self.n + 1
        self._alloc(n)
        for k, v in old.items():
            getattr(self, k)[..., :-1] = v
        i = n - 1
        self.n = n
        self.airframes.append(airframe)
        self._pos[:, i] = pos
        self._cmd_pos[:, i] = pos
        self._q[:, i] = q_from_yaw(yaw)
        self.on_ground[i] = on_ground
        self.cmd_yaw[i] = self.yaw_target[i] = yaw
        for k in self.PARAMS:
            getattr(self, "p_" + k)[i] = getattr(airframe, k)
        self._p_inertia[:, i] = (airframe.ixx, airframe.iyy, airframe.izz)
        self._minv[:, :, i] = np.linalg.inv(mix_matrix(airframe))
        self._derived()
        return i

    # -- derived quantities ---------------------------------------------------------------------------------------
    def matrices(self):
        return q_matrix(self.q)

    def yaw(self):
        x, y, z, w = self._q
        return np.arctan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z))

    def tilt(self):
        x, y = self._q[0], self._q[1]
        return np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))

    def blade_passing_hz(self):
        """Blade-passing frequency of the rotors at their mean speed: the fundamental of a multirotor's tonal noise."""
        return self.p_blades * self._rotors.mean(axis=0) / (2 * np.pi)

    def thrust_ratio(self):
        """Thrust against hover thrust (1 in hover)."""
        return np.sum(self._rotors ** 2, axis=0) / 4.0 / self._w2h

    def battery(self):
        """Remaining charge (0-1)."""
        return np.clip(1.0 - self.energy_wh / np.maximum(1e-6, self.p_battery_wh), 0.0, 1.0)

    def endurance_s(self):
        """Seconds of flight left at the present power."""
        left = self.battery() * self.p_battery_wh * 3600.0
        return left / np.maximum(1.0, self.power_w)

    def set_command(self, i, mode="position", pos=None, vel=None, yaw=None, yaw_rate=None, roll=0.0, pitch=0.0,
                    climb=0.0):
        """One drone's command (coupled-sim's DroneCommand); yaw_rate given means turn at that rate."""
        self.mode[i] = MODES[mode] if isinstance(mode, str) else int(mode)
        if pos is not None:
            self._cmd_pos[:, i] = pos
        self._cmd_vel[:, i] = (0.0, 0.0, 0.0) if vel is None else vel
        self.use_yaw_rate[i] = yaw_rate is not None
        self.cmd_yaw_rate[i] = 0.0 if yaw_rate is None else yaw_rate
        if yaw is not None:
            self.cmd_yaw[i] = yaw
        self.cmd_roll[i], self.cmd_pitch[i], self.cmd_climb[i] = roll, pitch, climb

    def place(self, i, pos, vel=(0.0, 0.0, 0.0), yaw=None, on_ground=None):
        """Puts drone i somewhere (a crash stop, a reset); rates zeroed, attitude level when yaw is given."""
        self._pos[:, i] = pos
        self._vel[:, i] = vel
        self._rates[:, i] = 0.0
        if yaw is not None:
            self._q[:, i] = q_from_yaw(yaw)
        if on_ground is not None:
            self.on_ground[i] = on_ground

    # -- stepping -------------------------------------------------------------------------------------------------
    def step(self, dt):
        """Advances every drone by dt seconds in sub-steps of at most 2 ms."""
        if self.n == 0:
            self.time += dt
            return
        k = max(1, int(math.ceil(dt / SUB_STEP - 1e-9)))
        h = dt / k
        for _ in range(k):
            self._step_once(h)
        self.time += dt

    def _level(self, m):
        """Drones in mask m rest level on the ground."""
        self._pos[2] = np.where(m, np.maximum(self._pos[2], self.ground), self._pos[2])
        lev = q_from_yaw(self.yaw()).T
        self._q[:] = np.where(m, lev, self._q)
        self._vel_int[:] = np.where(m, 0.0, self._vel_int)

    def _rot(self):
        """Columns of the rotation matrix: body x, y, z in the world, each a tuple of three (N,) arrays."""
        x, y, z, w = self._q
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
        return ((1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)),
                (2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)),
                (2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)))

    def _step_once(self, h):
        c0, c1, c2 = self._rot()
        cmd = self._control(h, c0, c1, c2)
        kk = 1.0 - np.exp(-h / np.maximum(1e-4, self.p_motor_tau))
        rot = self._rotors
        rot += (cmd - rot) * kk
        w2 = rot * rot
        w1s, w2s, w3s, w4s = w2
        thrust = self.p_kf * (w1s + w2s + w3s + w4s)
        vx, vy, vz = self._vel
        ax_, ay_, az_ = vx - self._wind[0], vy - self._wind[1], vz - self._wind[2]
        sa = np.sqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_)
        kd = self.p_lin_drag + self.p_quad_drag * sa
        dx, dy, dz = -kd * ax_, -kd * ay_, -kd * az_
        im = 1.0 / self.p_mass
        accx = (c2[0] * thrust + dx) * im
        accy = (c2[1] * thrust + dy) * im
        accz = (c2[2] * thrust + dz) * im - G
        # power: induced (momentum theory over the figure of merit), drag and avionics, over the drive efficiency
        induced = np.sum((self.p_kf * w2) ** 1.5, axis=0) * self._area_k
        parasite = np.abs(dx * ax_ + dy * ay_ + dz * az_)
        p = np.where(rot.max(axis=0) > 1.0, (induced + parasite) / self.p_drive_efficiency, 0.0) + self.p_avionics_w
        self.power_w = p
        self.energy_wh += p * (h / 3600.0)
        # resting on the ground until the thrust lifts the drone
        og = self.on_ground
        stay = og & (accz <= 0.0)
        any_stay = stay.any()
        if any_stay:
            self._vel[:] = np.where(stay, 0.0, self._vel)
            self._rates[:] = np.where(stay, 0.0, self._rates)
            self._level(stay)
        og &= stay
        act = ~stay
        if any_stay:
            accx, accy, accz = accx * act, accy * act, accz * act
        vx += accx * h
        vy += accy * h
        vz += accz * h
        px, py, pz = self._pos
        if any_stay:
            px += vx * h * act
            py += vy * h * act
            pz += vz * h * act
        else:
            px += vx * h
            py += vy * h
            pz += vz * h
        below = act & (pz <= self.ground)
        free = act
        if below.any():
            pz[below] = self.ground[below]
            down = below & (vz < 0.0)
            self._vel[:, down] = 0.0
            landed = below & (thrust < self._mg)
            if landed.any():
                og |= landed
                self._rates[:, landed] = 0.0
                self._level(landed)
                free = act & ~landed
        # rotation: Euler's equations with the rotor torques and the gyroscopic term
        wx, wy, wz = self._rates
        ix, iy, iz = self._p_inertia
        kfl = self._kfl
        tx = kfl * (SIN4[0] * w1s + SIN4[1] * w2s + SIN4[2] * w3s + SIN4[3] * w4s) - self.p_rot_drag * wx
        ty = -kfl * (COS4[0] * w1s + COS4[1] * w2s + COS4[2] * w3s + COS4[3] * w4s) - self.p_rot_drag * wy
        tz = self.p_km * (w1s - w2s + w3s - w4s) - self.p_rot_drag * wz
        hx, hy, hz = ix * wx, iy * wy, iz * wz
        nwx = wx + (tx - (wy * hz - wz * hy)) / ix * h
        nwy = wy + (ty - (wz * hx - wx * hz)) / iy * h
        nwz = wz + (tz - (wx * hy - wy * hx)) / iz * h
        x, y, z, w = self._q
        hh = 0.5 * h
        qx = x + hh * (w * nwx + y * nwz - z * nwy)
        qy = y + hh * (w * nwy + z * nwx - x * nwz)
        qz = z + hh * (w * nwz + x * nwy - y * nwx)
        qw = w + hh * (-x * nwx - y * nwy - z * nwz)
        inv = 1.0 / np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if free.all():
            self._rates[0], self._rates[1], self._rates[2] = nwx, nwy, nwz
            self._q[0], self._q[1], self._q[2], self._q[3] = qx * inv, qy * inv, qz * inv, qw * inv
        elif free.any():
            self._rates[:] = np.where(free, np.array([nwx, nwy, nwz]), self._rates)
            self._q[:] = np.where(free, np.array([qx * inv, qy * inv, qz * inv, qw * inv]), self._q)

    def _control(self, h, c0, c1, c2):
        """Rotor speed commands of the flight controller (coupled-sim's Quadrotor.Control), for every drone."""
        mode = self.mode
        on = mode != OFF
        if not on.any():
            return np.zeros((4, self.n))
        maxyaw = self.p_max_yaw_rate
        yt = np.where(self.use_yaw_rate,
                      wrap(self.yaw_target + np.minimum(np.maximum(self.cmd_yaw_rate, -maxyaw), maxyaw) * h),
                      self.cmd_yaw)
        self.yaw_target = yt = np.where(on, yt, self.yaw_target)
        px, py, pz = self._pos
        vx, vy, vz = self._vel
        att = mode == ATTITUDE
        any_att = att.any()
        # position -> velocity (limited to the envelope) -> acceleration (PI)
        posf = mode == POSITION
        cvx, cvy, cvz = self._cmd_vel
        cpx, cpy, cpz = self._cmd_pos
        wx_ = cvx + posf * ((cpx - px) * self.KP_XY)
        wy_ = cvy + posf * ((cpy - py) * self.KP_XY)
        wz_ = cvz + posf * ((cpz - pz) * self.KP_Z)
        s = np.sqrt(wx_ * wx_ + wy_ * wy_)
        sc = np.where(s > self.p_max_speed, self.p_max_speed / np.maximum(s, 1e-9), 1.0)
        wx_, wy_ = wx_ * sc, wy_ * sc
        wz_ = np.minimum(np.maximum(wz_, -self.p_max_descent), self.p_max_climb)
        ex, ey, ez = wx_ - vx, wy_ - vy, wz_ - vz
        pv = on & ~att if any_att else on
        le = self._last_err
        le[:] = np.where(pv, np.array([ex, ey, ez]), le)
        gi = self._vel_int
        grow = (pv & ~self.on_ground) * h
        gi[0] += ex * grow
        gi[1] += ey * grow
        gi[2] += ez * grow
        np.clip(gi[0], -8.0, 8.0, out=gi[0])
        np.clip(gi[1], -8.0, 8.0, out=gi[1])
        np.clip(gi[2], -4.0, 4.0, out=gi[2])
        ax = self.KV_XY * ex + self.KI_XY * gi[0]
        ay = self.KV_XY * ey + self.KI_XY * gi[1]
        az = self.KV_Z * ez + self.KI_Z * gi[2]
        if any_att:   # tilt from the sticks, climb rate held by the thrust
            yaw = np.arctan2(c0[1], c0[0])
            vzc = np.clip(self.cmd_climb, -self.p_max_descent, self.p_max_climb)
            aza = self.KV_Z * (vzc - vz)
            fx, fy = np.cos(yaw), np.sin(yaw)
            tp = np.tan(np.clip(self.cmd_pitch, -self.p_max_tilt, self.p_max_tilt))
            tr = np.tan(np.clip(self.cmd_roll, -self.p_max_tilt, self.p_max_tilt))
            g_az = G + aza
            ax = np.where(att, (fx * tp + fy * tr) * g_az, ax)      # fwd*tan(pitch) - left*tan(roll), left = (-fy, fx)
            ay = np.where(att, (fy * tp - fx * tr) * g_az, ay)
            az = np.where(att, aza, az)
        # tilt limit: the horizontal acceleration the thrust can give (and no integrating while it limits)
        axy = np.sqrt(ax * ax + ay * ay)
        lim = axy > self._max_acc
        if lim.any():
            sc = np.where(lim, self._max_acc / np.maximum(axy, 1e-9), 1.0)
            ax, ay = ax * sc, ay * sc
            aw = (lim & pv) * h
            gi[0] -= le[0] * aw
            gi[1] -= le[1] * aw
        # the thrust vector, gravity compensated (the drag is left to the integrator)
        m = self.p_mass
        fx, fy = m * ax, m * ay
        fz = np.maximum(m * (az + G), 0.1 * self._mg)
        fn = 1.0 / np.sqrt(fx * fx + fy * fy + fz * fz)
        b3x, b3y, b3z = fx * fn, fy * fn, fz * fn
        thrust = np.maximum(0.0, fx * c2[0] + fy * c2[1] + fz * c2[2])
        # desired attitude: thrust axis b3, heading from the yaw target (b1c = (cos, sin, 0))
        cy, sy = np.cos(yt), np.sin(yt)
        b2x, b2y, b2z = -b3z * sy, b3z * cy, b3x * sy - b3y * cy
        n2 = 1.0 / np.sqrt(b2x * b2x + b2y * b2y + b2z * b2z)
        b2x, b2y, b2z = b2x * n2, b2y * n2, b2z * n2
        b1x, b1y, b1z = b2y * b3z - b2z * b3y, b2z * b3x - b2x * b3z, b2x * b3y - b2y * b3x
        r1x, r1y, r1z = c0
        r2x, r2y, r2z = c1
        r3x, r3y, r3z = c2
        erx = 0.5 * ((b3x * r2x + b3y * r2y + b3z * r2z) - (b2x * r3x + b2y * r3y + b2z * r3z))
        ery = 0.5 * ((b1x * r3x + b1y * r3y + b1z * r3z) - (b3x * r1x + b3y * r1y + b3z * r1z))
        erz = 0.5 * ((b2x * r1x + b2y * r1y + b2z * r1z) - (b1x * r2x + b1y * r2y + b1z * r2z))
        wx, wy, wz = self._rates
        ix, iy, iz = self._p_inertia
        hx, hy, hz = ix * wx, iy * wy, iz * wz
        tx = ix * (self.K_RATE * (-self.K_ATT * erx - wx)) + (wy * hz - wz * hy)
        ty = iy * (self.K_RATE * (-self.K_ATT * ery - wy)) + (wz * hx - wx * hz)
        tz = iz * (self.K_YAW_RATE * (-self.K_YAW * erz - wz)) + (wx * hy - wy * hx)
        u = np.array([thrust, tx, ty, tz])
        s2 = np.sum(self._minv * u[None, :, :], axis=1)
        w = np.sqrt(np.minimum(np.maximum(s2, self._lo2), self._hi2))
        return w * on


class Quadrotor:
    """One drone, with coupled-sim's API (tests, simple uses): q = Quadrotor(CONSUMER, pos, yaw); q.command(...);
    q.step(0.02); q.pos, q.vel, ..."""

    def __init__(self, airframe, pos=(0.0, 0.0, 0.0), yaw=0.0):
        self.fleet = Fleet()
        self.fleet.add(airframe, pos, yaw)
        self.airframe = airframe

    def command(self, **kw):
        self.fleet.set_command(0, **kw)

    def step(self, dt):
        self.fleet.step(dt)

    pos = property(lambda s: s.fleet.pos[0].copy())
    vel = property(lambda s: s.fleet.vel[0].copy())
    on_ground = property(lambda s: bool(s.fleet.on_ground[0]))
    yaw = property(lambda s: float(s.fleet.yaw()[0]))
    tilt = property(lambda s: float(s.fleet.tilt()[0]))
    rotors = property(lambda s: s.fleet.rotors[0].copy())

    @property
    def wind(self):
        return self.fleet.wind[0]

    @wind.setter
    def wind(self, v):
        self.fleet.wind[0] = v


class Dryden:
    """Atmospheric turbulence by the Dryden model's low-altitude form (MIL-F-8785C): gust components as first-order
    (Gauss-Markov) processes with sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4 and length
    scales L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2 (h in feet), driven by the airspeed. W20 is the wind
    speed at 20 ft (6 m). Vectorised: one gust per drone."""

    def __init__(self, n=0, seed=0):
        self.rng = np.random.default_rng(seed)
        self.gust = np.zeros((n, 3))

    def resize(self, n):
        g = np.zeros((n, 3))
        g[:min(n, len(self.gust))] = self.gust[:n]
        self.gust = g

    @staticmethod
    def intensities(height_m, w20):
        """(sigma horizontal, sigma vertical, length horizontal, length vertical), m/s and m."""
        hft = np.maximum(10.0, np.asarray(height_m, float) / 0.3048)
        k = 0.177 + 0.000823 * hft
        sv = 0.1 * np.asarray(w20, float)
        sh = sv / k ** 0.4
        lv = hft * 0.3048
        lh = hft / k ** 1.2 * 0.3048
        return sh, sv, lh, lv

    def step(self, dt, height_m, w20, airspeed):
        """The gusts (N, 3) world (x, y horizontal, z up) after dt."""
        n = len(self.gust)
        sh, sv, lh, lv = self.intensities(height_m, w20)
        v = np.maximum(1.0, np.asarray(airspeed, float))
        sig = np.stack(np.broadcast_arrays(sh, sh, sv), axis=-1).reshape(-1, 3) * np.ones((n, 1))
        ln = np.stack(np.broadcast_arrays(lh, lh, lv), axis=-1).reshape(-1, 3) * np.ones((n, 1))
        a = np.exp(-(v * np.ones(n))[:, None] * dt / ln)
        self.gust = a * self.gust + sig * np.sqrt(1.0 - a * a) * self.rng.standard_normal((n, 3))
        return self.gust


# -- flocking ------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class FlockParams:
    """Vasarhelyi et al. (2018) flocking parameters. The repulsion is linear below rep_radius (gain rep_gain, at most
    rep_max in total), the alignment is a viscous friction above a speed difference allowed by the braking curve D
    beyond frict_radius, obstacles act through shill agents with the same braking curve."""
    flock_speed: float = 6.0
    rep_radius: float = 15.0
    rep_gain: float = 0.3
    frict_radius: float = 30.0
    frict_gain: float = 0.08
    frict_min_speed: float = 0.6
    frict_acc: float = 4.0
    frict_slope: float = 3.0
    shill_speed: float = 8.0
    shill_radius: float = 3.0
    shill_acc: float = 3.0
    shill_slope: float = 3.5
    max_speed: float = 10.0
    rep_max: float = math.inf


# coupled-sim's set: "of the paper's form, set for a 6 m/s flock with drones about 15 m apart" (a city sky)
FLOCK_COUPLED_SIM = FlockParams()
# The authors' own parameter file of the simulator used in the article (github.com/csviragh/robotsim,
# parameters/flockingparams.dat): v_flock 4 m/s, r0 25.7 m, p_rep 0.4 /s, v_rep 6.2 m/s, friction from
# r0 + 10 m, C_frict 0.0238, v_frict 0.5 m/s, p_frict 0.4 /s, a_frict 2.5 m/s^2, v_shill 6.5 m/s, r0_shill 0,
# p_shill 0.4 /s, a_shill 2.5 m/s^2, v_max 8 m/s
FLOCK_VASARHELYI_2018 = FlockParams(flock_speed=4.0, rep_radius=25.6668, rep_gain=0.4, rep_max=6.19664,
                                    frict_radius=35.6668, frict_gain=0.023802, frict_min_speed=0.5, frict_acc=2.5,
                                    frict_slope=0.4, shill_speed=6.52174, shill_radius=0.0, shill_acc=2.5,
                                    shill_slope=0.4, max_speed=8.0)
FLOCK_PRESETS = {"coupled_sim": FLOCK_COUPLED_SIM, "vasarhelyi2018": FLOCK_VASARHELYI_2018}


def flock_params(spec):
    """A preset name, or a dict of FlockParams fields over coupled-sim's set (or over a preset named in `preset`)."""
    if isinstance(spec, FlockParams):
        return spec
    if isinstance(spec, str):
        return FLOCK_PRESETS[spec]
    spec = dict(spec or {})
    base = FLOCK_PRESETS[spec.pop("preset", "coupled_sim")]
    return replace(base, **spec)


def braking(r, a, p):
    """The braking curve D(r, a, p): the highest speed from which one stops within r at deceleration a, with a linear
    part (slope p) near zero."""
    r = np.asarray(r, float)
    lin = r * p
    sq = np.sqrt(np.maximum(0.0, 2.0 * a * r - a * a / (p * p)))
    out = np.where((lin > 0) & (lin < a / p), lin, sq)
    return np.where(r <= 0.0, 0.0, out)


def flock_velocities(pos, vel, goals, k, shills=None, members=None):
    """Desired velocities of drones `members` (default all) of a flock given all positions and velocities (N, 3),
    their migration points `goals` ((N, 3), NaN rows: keep heading) and shill agents: per member a list of
    (position, velocity pointing out of the obstacle). Returns (len(members), 3)."""
    pos, vel = np.asarray(pos, float), np.asarray(vel, float)
    n = len(pos)
    members = np.arange(n) if members is None else np.asarray(members, int)
    goals = np.full((n, 3), np.nan) if goals is None else np.asarray(goals, float)
    out = np.zeros((len(members), 3))
    for row, i in enumerate(members):
        pi, vi = pos[i], vel[i]
        g = goals[i]
        has_goal = not np.isnan(g).any()
        d = g - pi if has_goal else vi
        dl = float(_norm(d))
        if has_goal and dl < 0.5:
            d, dl = np.zeros(3), 0.0
        v = d / dl * min(k.flock_speed, dl * 0.5 if has_goal else k.flock_speed) if dl > 1e-3 else np.zeros(3)
        others = np.arange(n) != i
        dd = pi - pos[others]
        r = _norm(dd)
        ok = r >= 1e-3
        dd, r, vj = dd[ok], r[ok], vel[others][ok]
        rep = r < k.rep_radius
        vrep = np.sum((k.rep_gain * (k.rep_radius - r[rep]) / r[rep])[:, None] * dd[rep], axis=0)
        rl = float(_norm(vrep))
        if rl > k.rep_max:
            vrep *= k.rep_max / rl
        v = v + vrep
        dv = vj - vi
        vij = _norm(dv)
        allowed = np.maximum(k.frict_min_speed, braking(r - k.frict_radius, k.frict_acc, k.frict_slope))
        fr = vij > allowed
        v = v + np.sum((k.frict_gain * (vij[fr] - allowed[fr]) / vij[fr])[:, None] * dv[fr], axis=0)
        for sp, sv in (shills[row] if shills is not None else ()):
            sp, sv = np.asarray(sp, float), np.asarray(sv, float)
            r_s = float(_norm(pi - sp))
            dvs = sv * k.shill_speed / max(1e-3, float(_norm(sv))) - vi
            vis = float(_norm(dvs))
            allowed = float(braking(r_s - k.shill_radius, k.shill_acc, k.shill_slope))
            if vis > allowed and vis > 1e-3:
                v = v + (vis - allowed) * dvs / vis
        vl = float(_norm(v))
        if vl > k.max_speed:
            v = v / vl * k.max_speed
        out[row] = v
    return out


def flock_velocity(i, pos, vel, goal, k, shills=None):
    """coupled-sim's Flocking.Velocity for one drone."""
    pos = np.asarray(pos, float)
    goals = np.full((len(pos), 3), np.nan)
    if goal is not None:
        goals[i] = goal
    return flock_velocities(pos, vel, goals, k, [shills or []], members=[i])[0]


# -- mission guidance ----------------------------------------------------------------------------------------------

@dataclass
class Waypoints:
    """A drone's mission as a path of waypoints flown with a smooth speed profile: constant acceleration up to the
    cruise speed and down to a stop at the last waypoint (and at corners sharper than 60 degrees), as multirotor
    autopilots do in mission mode (coupled-sim's WaypointGuidance). step() gives the controller a position target and
    a velocity feed-forward."""
    points: list
    cruise: float = 10.0
    climb: float = 4.0
    accel: float = 2.5
    accept: float = 1.5
    index: int = 0
    speed: float = 0.0
    loop_from: int = None        # with a loop: after the last point, go on from this index
    pts: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self.pts = np.asarray(self.points, float).reshape(-1, 3)

    @property
    def done(self):
        return self.index >= len(self.pts)

    def step(self, p, dt):
        p = np.asarray(p, float)
        pts = self.pts
        while self.index < len(pts) and _norm(pts[self.index] - p) < self.accept:
            self.index += 1
            if self.index >= len(pts) and self.loop_from is not None:
                self.index = self.loop_from
                break
        if self.done:
            self.speed = 0.0
            return (pts[-1].copy() if len(pts) else p.copy()), np.zeros(3)
        to = pts[self.index] - p
        dist = float(_norm(to))
        remaining = dist
        for k in range(self.index, len(pts) - 1):
            a = pts[k] - (p if k == self.index else pts[k - 1])
            b = pts[k + 1] - pts[k]
            la, lb = float(_norm(a)), float(_norm(b))
            if la > 0.1 and lb > 0.1 and float(np.dot(a / la, b / lb)) < 0.5:
                break
            remaining += lb
        if self.loop_from is not None:
            remaining = max(remaining, 1e6)       # a loop never stops
        d = to / max(dist, 1e-9)
        cruise = self.climb if abs(d[2]) > 0.8 else self.cruise
        v_stop = math.sqrt(2.0 * self.accel * max(0.0, remaining - self.accept * 0.5))
        want = min(cruise, v_stop)
        self.speed = min(want, self.speed + self.accel * dt) if want > self.speed else want
        vel = d * self.speed
        target = p + d * min(dist, max(1.0, self.speed * 0.5))
        return target, vel
