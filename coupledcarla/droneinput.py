"""A drone pilot's input: sticks from a gamepad, an RC transmitter seen as a joystick, the keyboard or a bot, and the
mapping of the sticks to the flight controller's command (coupled-sim's DroneController.PilotCommand).

Sticks are four values in -1..1 in the "mode 2" layout of consumer drones and RC transmitters: the left stick is climb
(up/down) and yaw (left/right), the right stick forward (up/down) and lateral (left/right).

- position mode (default, consumer drones' "GPS" mode): the sticks set the velocity in the heading's frame (forward and
  lateral up to max_speed, climb up to max_climb / max_descent, yaw rate up to max_yaw_rate); releasing them brakes and
  holds the position where the drone comes to rest. On the ground, the climb stick up takes off.
- attitude mode ("ATTI"): forward and lateral tilt the drone (up to max_tilt), the climb stick sets the climb rate;
  released, it stays level and drifts with the wind.

Keyboard (as coupled-sim's): W/S forward/back, A/D sideways, Up/Down or Space/Left Shift climb/descend, Left/Right or
Q/E turn; I/K tilt the gimbal up/down; C cycles the view (fpv, los, chase); M switches position/attitude mode; H flies
home and lands (return to home); Esc quits. Gamepad (Xbox layout): the two sticks (mode 2), triggers or the d-pad
tilt the gimbal, Y the view, X the mode, B return to home. An RC transmitter (USB joystick, AETR channel order: aileron,
elevator, throttle, rudder) with a self-centring throttle stick works as `input: rc`.
"""
import math
from dataclasses import dataclass, field

KEYS_HELP = ("W/S forward  A/D sideways  Up/Down climb  Left/Right turn  I/K gimbal  C view  M mode  H home  "
             "Esc quit")

# axis numbers (pygame joystick) and signs per device; a participant's `axes` overrides them
GAMEPAD_AXES = {"climb": (1, -1.0), "yaw": (0, 1.0), "forward": (3, -1.0), "lateral": (2, 1.0)}   # XInput, SDL2
RC_AXES = {"lateral": (0, 1.0), "forward": (1, 1.0), "climb": (2, 1.0), "yaw": (3, 1.0)}            # AETR
GAMEPAD_BUTTONS = {"view": 3, "mode": 2, "home": 1}
VIEWS = ("fpv", "los", "chase")


def shape(x, deadband=0.05, expo=0.3):
    """A stick value with a dead band around the centre and RC-style expo (softer near the centre):
    (1 - e) x + e x^3 on the part outside the dead band, rescaled to reach 1."""
    x = max(-1.0, min(1.0, float(x)))
    a = abs(x)
    if a <= deadband:
        return 0.0
    a = (a - deadband) / (1.0 - deadband)
    a = (1.0 - expo) * a + expo * a ** 3
    return math.copysign(a, x)


@dataclass
class Sticks:
    climb: float = 0.0
    yaw: float = 0.0
    forward: float = 0.0
    lateral: float = 0.0
    gimbal: float = 0.0          # gimbal pitch rate command, -1 (down) .. 1 (up)
    events: list = field(default_factory=list)      # "view", "mode", "home", "quit", "sync"

    def as_list(self):
        return [round(self.climb, 3), round(self.yaw, 3), round(self.forward, 3), round(self.lateral, 3)]

    @staticmethod
    def from_list(v, gimbal=0.0, events=()):
        v = list(v or [0, 0, 0, 0]) + [0, 0, 0, 0]
        return Sticks(*(max(-1.0, min(1.0, float(x))) for x in v[:4]), gimbal=float(gimbal or 0.0),
                      events=list(events or ()))

    @property
    def centred(self):
        return abs(self.climb) < 0.05 and abs(self.forward) < 0.05 and abs(self.lateral) < 0.05


def keyboard_sticks(pressed, keydowns, k):
    """Sticks from pygame's key state. pressed: callable(key) -> bool; keydowns: the keys pressed this tick;
    k: the pygame module (or anything with its K_* names)."""
    def axis(plus, minus):
        return float(any(pressed(x) for x in plus)) - float(any(pressed(x) for x in minus))
    s = Sticks(climb=axis((k.K_UP, k.K_SPACE), (k.K_DOWN, k.K_LSHIFT)),
               yaw=axis((k.K_RIGHT, k.K_e), (k.K_LEFT, k.K_q)),
               forward=axis((k.K_w,), (k.K_s,)), lateral=axis((k.K_d,), (k.K_a,)),
               gimbal=axis((k.K_i,), (k.K_k,)))
    for key in keydowns:
        name = {k.K_c: "view", k.K_m: "mode", k.K_h: "home", k.K_ESCAPE: "quit"}.get(key)
        if name:
            s.events.append(name)
    return s


def joystick_sticks(axis_value, n_axes, axes, deadband=0.05, expo=0.3):
    """Sticks from a joystick. axis_value: callable(i) -> -1..1; axes: {name: (axis, sign)}."""
    vals = {}
    for name, (i, sign) in axes.items():
        vals[name] = shape(sign * axis_value(i), deadband, expo) if 0 <= int(i) < n_axes else 0.0
    return Sticks(climb=vals.get("climb", 0.0), yaw=vals.get("yaw", 0.0), forward=vals.get("forward", 0.0),
                  lateral=vals.get("lateral", 0.0))


class PygameInput:
    """Keyboard, gamepad or RC transmitter at a window (host-local or at a station). poll(events) -> Sticks; the
    keyboard always works as well. cfg: the participant's entry (input: keyboard | gamepad | rc; axes, joystick,
    deadband, expo)."""

    def __init__(self, kind="keyboard", cfg=None):
        import pygame
        self.pg = pygame
        cfg = cfg or {}
        self.kind = kind
        self.joy = None
        self.axes = dict(RC_AXES if kind == "rc" else GAMEPAD_AXES)
        for name, v in (cfg.get("axes") or {}).items():
            self.axes[name] = (int(v[0]), float(v[1])) if isinstance(v, (list, tuple)) else (int(v), 1.0)
        self.deadband = float(cfg.get("deadband", 0.05))
        self.expo = float(cfg.get("expo", 0.3 if kind != "rc" else 0.0))
        self.buttons = dict(GAMEPAD_BUTTONS, **(cfg.get("buttons") or {}))
        if kind in ("gamepad", "rc"):
            pygame.joystick.init()
            idx = int(cfg.get("joystick", 0))
            if pygame.joystick.get_count() > idx:
                self.joy = pygame.joystick.Joystick(idx)
                self.joy.init()
            else:
                print(f"[drone input] no joystick {idx}: keyboard only", flush=True)

    def poll(self, events):
        pg = self.pg
        downs = [e.key for e in events if e.type == pg.KEYDOWN]
        keys = pg.key.get_pressed()
        s = keyboard_sticks(lambda key: keys[key], downs, pg)
        if self.joy is not None:
            j = joystick_sticks(self.joy.get_axis, self.joy.get_numaxes(), self.axes, self.deadband, self.expo)
            for name in ("climb", "yaw", "forward", "lateral"):
                if abs(getattr(j, name)) > abs(getattr(s, name)):
                    setattr(s, name, getattr(j, name))
            if self.kind == "gamepad":
                n = self.joy.get_numaxes()
                if n >= 6:        # triggers: right up, left down (rest at -1)
                    s.gimbal += 0.5 * (self.joy.get_axis(5) + 1) - 0.5 * (self.joy.get_axis(4) + 1)
                if self.joy.get_numhats():
                    s.gimbal += self.joy.get_hat(0)[1]
                s.gimbal = max(-1.0, min(1.0, s.gimbal))
            for e in events:
                if e.type == pg.JOYBUTTONDOWN:
                    for name, b in self.buttons.items():
                        if e.button == b:
                            s.events.append(name)
        return s


class BotPilot:
    """A pilot without a human (tests, demos): flies through `path` points (world ENU, metres; z is the height) at
    `speed`, turning to face its way, by setting the sticks as a person would (a P controller on the velocity in the
    heading's frame); or plays `sticks` [[t, climb, yaw, forward, lateral], ...] (held until the next entry). Takes
    off at start_s. After the last point it hovers (or lands with land: true)."""

    def __init__(self, path=(), speed=6.0, start_s=0.0, sticks=None, land=False, max_speed=15.0, max_climb=6.0,
                 accept=2.0):
        self.path = [tuple(map(float, p)) for p in path]
        self.speed, self.start_s, self.land = float(speed), float(start_s), bool(land)
        self.schedule = sorted([list(map(float, s)) for s in (sticks or [])])
        self.max_speed, self.max_climb, self.accept = max_speed, max_climb, accept
        self.i = 0

    @property
    def done(self):
        return self.i >= len(self.path)

    def sticks(self, t, pos, vel, yaw):
        if t < self.start_s:
            return Sticks()
        if self.schedule:
            cur = [s for s in self.schedule if s[0] <= t]
            return Sticks(*cur[-1][1:5]) if cur else Sticks()
        while self.i < len(self.path) and math.dist(self.path[self.i], pos) < self.accept:
            self.i += 1
        if self.done:
            return Sticks(climb=-1.0 if self.land else 0.0)
        tx, ty, tz = self.path[self.i]
        dx, dy, dz = tx - pos[0], ty - pos[1], tz - pos[2]
        dh = math.hypot(dx, dy)
        want = min(self.speed, 0.6 * dh)
        vx, vy = (dx / dh * want, dy / dh * want) if dh > 1e-6 else (0.0, 0.0)
        heading = math.atan2(dy, dx) if dh > 3.0 else yaw
        dyaw = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
        c, s = math.cos(yaw), math.sin(yaw)
        fwd, right = vx * c + vy * s, vx * s - vy * c
        return Sticks(climb=max(-1.0, min(1.0, 0.5 * dz / self.max_climb * 2.0)),
                      yaw=max(-1.0, min(1.0, -1.5 * dyaw)),      # right turns clockwise (negative yaw)
                      forward=max(-1.0, min(1.0, fwd / self.max_speed)),
                      lateral=max(-1.0, min(1.0, right / self.max_speed)))


@dataclass
class Hold:
    """The pilot command's memory: where to hold when the sticks are released."""
    holding: bool = False
    point: tuple = (0.0, 0.0, 0.0)


def pilot_command(st, pos, vel, yaw, on_ground, af, mode="position", hold=None, agl=None):
    """The participant's sticks to a flight-controller command (a dict for Fleet.set_command): velocities in the
    heading's frame (position mode, holding the position when released, braking first) or tilts (attitude mode).
    On the ground with the climb stick not up: rotors off. In position mode, with the height above the ground `agl`
    known, the descent slows below 5 m to 0.8 m/s per metre (at least 0.6 m/s), as consumer drones' downward
    sensors do ("landing protection"); attitude mode has none."""
    hold = hold if hold is not None else Hold()
    fwd = (math.cos(yaw), math.sin(yaw))
    right = (math.sin(yaw), -math.cos(yaw))
    climb = st.climb * af.max_climb if st.climb >= 0 else st.climb * af.max_descent
    yaw_rate = -st.yaw * af.max_yaw_rate          # right turns clockwise (negative yaw)
    if on_ground and st.climb <= 0.05:
        hold.holding = False
        return {"mode": "off", "yaw_rate": yaw_rate}
    if mode == "attitude":
        return {"mode": "attitude", "yaw_rate": yaw_rate, "pitch": st.forward * af.max_tilt,
                "roll": st.lateral * af.max_tilt, "climb": climb}
    if st.centred:
        if not hold.holding:     # sticks centred: hold where it comes to rest (braking first)
            hold.point = (pos[0] + vel[0] * 0.6, pos[1] + vel[1] * 0.6, pos[2] + vel[2] * 0.6)
            hold.holding = True
        return {"mode": "position", "pos": hold.point, "yaw_rate": yaw_rate}
    hold.holding = False
    if agl is not None and agl < 5.0 and climb < 0:
        climb = max(climb, -max(0.6, 0.8 * agl))
    v = (fwd[0] * st.forward * af.max_speed + right[0] * st.lateral * af.max_speed,
         fwd[1] * st.forward * af.max_speed + right[1] * st.lateral * af.max_speed, climb)
    return {"mode": "velocity", "vel": v, "yaw_rate": yaw_rate}
