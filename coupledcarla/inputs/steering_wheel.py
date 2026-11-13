"""Logitech G25 (or any similar wheel) as a CARLA driving input, with force feedback.

Reads steering, throttle, brake and clutch from the wheel through pygame and
drives its force feedback through SDL's haptic API, called with ctypes on the
SDL2 library pygame itself loaded (pygame does not expose haptics).

Steering goes through a steering ratio (STEER_RATIO, 7:1 by default): the
900-degree wheel turns the front wheels 64 degrees each way, instead of
CARLA's full 70 at a quarter turn. Real sedans are about 15:1, which feels
sluggish without a car's motion around you.

The force feedback is built like a real car's:

- aligning torque: the front tyres' slip angle, estimated from the car's
  motion, pulls the wheel back to straight. It builds through a corner and
  falls away past the grip limit, so understeer makes the wheel go light.
- power steering: light at parking speed, firmer as the speed rises.
- road: a rumble from the car's vertical acceleration, kicks from body roll
  (one wheel over a bump), and a jolt on a collision.
- spring, damper and friction effects that the wheel's firmware runs, for
  centring at a standstill, weight, and tyre scrub. On autopilot the spring's
  centre follows the car's steering, so the wheel turns itself.

Needs the Logitech driver (G HUB or Logitech Gaming Software) so the G25 runs
in native mode: 900 degrees, separate clutch axis, force feedback. In G HUB
keep the wheel at 900 degrees and the force-feedback strength at the top.

The H-shifter works as an automatic's selector (SHIFTER_GEARS): 1st is P,
3rd D, 4th R, and the middle or any other slot N.

python steering_wheel.py prints the live inputs, to check the mapping.
"""

import ctypes
import math
import os
import random
import sys

# Joystick input reaches pygame only while its window has the focus unless
# this is set; the dashboard window on screen 2 must not stop the pedals.
os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')

import pygame


# Axis numbers of the G25 in native mode under G HUB (separate pedals).
STEER_AXIS = 0
THROTTLE_AXIS = 1
BRAKE_AXIS = 2
CLUTCH_AXIS = 4

# H-shifter slots as buttons (G25: gears 1-6 are 8-13, reverse 14), laid out
# as an automatic's selector: 1st is P, 3rd D, 4th R. The stick in the
# middle, or in any other slot, is N.
SHIFTER_GEARS = {8: 'P', 10: 'D', 11: 'R'}
SHIFTER_BUTTONS = range(8, 15)

# Paddles and the wheel's top buttons, as measured on the G25.
PADDLE_LEFT = 5
PADDLE_RIGHT = 4
WHEEL_BUTTON_LEFT = 7
WHEEL_BUTTON_RIGHT = 6

# The shifter unit's d-pad is the joystick's hat 0; pressed() reports its
# up and down as these two buttons.
DPAD_UP = 'dpad-up'
DPAD_DOWN = 'dpad-down'

# The assists' buttons, as measured on the G25 (python steering_wheel.py).
# The shifter's red row, left to right, is 0-3: cruise control, adaptive
# cruise, the autopilot, and one unused. Each assist's button switches it on,
# resumes it from standby, and switches it off. The silver cluster's top and
# bottom (15, 17) step the set speed up and down; its left and right (16, 18)
# are unused. Gap - and + set the following distance, on the d-pad: up
# closes the gap to the car ahead, down opens it.
CC_BUTTON = 0
ACC_BUTTON = 1
AUTOPILOT_BUTTON = 2
CRUISE_UP = 15
CRUISE_DOWN = 17
GAP_DOWN = DPAD_UP
GAP_UP = DPAD_DOWN

# Pedals travel too little near the ends to reach the extremes reliably.
PEDAL_DEADZONE = 0.03

# Lock-to-lock rotation of the wheel, as set in G HUB.
WHEEL_RANGE = 900.0
# Degrees of wheel per degree of front-wheel angle; picked by driving it
# (a real sedan is about 15; 10 still felt a little slow, and after the
# pilot 8 did too).
STEER_RATIO = 7.0

# CARLA's default steering curve: the share of the steer that reaches the
# wheels at a speed (km/h). It works like a variable ratio on top of ours.
STEERING_CURVE = [(0.0, 1.0), (20.0, 0.9), (60.0, 0.8), (120.0, 0.7)]

# Front slip angle (radians) at which the tyres give the most aligning
# torque; past it the wheel goes light.
PEAK_SLIP = math.radians(7.0)

# Sign that makes a positive constant force push the wheel to the right
# (positive steer). Measured on a G25; DirectInput forces point to where
# they come from.
FFB_SIGN = -1.0


# ==============================================================================
# -- SDL haptic ----------------------------------------------------------------
# ==============================================================================

SDL_INIT_HAPTIC = 0x1000
SDL_HAPTIC_CONSTANT = 1 << 0
SDL_HAPTIC_SINE = 1 << 1
SDL_HAPTIC_SPRING = 1 << 7
SDL_HAPTIC_DAMPER = 1 << 8
SDL_HAPTIC_FRICTION = 1 << 10
SDL_HAPTIC_GAIN = 1 << 12
SDL_HAPTIC_CARTESIAN = 1
SDL_HAPTIC_INFINITY = 4294967295


class _HapticDirection(ctypes.Structure):
    _fields_ = [('type', ctypes.c_uint8), ('dir', ctypes.c_int32 * 3)]


_EFFECT_HEADER = [
    ('type', ctypes.c_uint16),
    ('direction', _HapticDirection),
    ('length', ctypes.c_uint32),
    ('delay', ctypes.c_uint16),
    ('button', ctypes.c_uint16),
    ('interval', ctypes.c_uint16),
]

_ENVELOPE = [
    ('attack_length', ctypes.c_uint16),
    ('attack_level', ctypes.c_uint16),
    ('fade_length', ctypes.c_uint16),
    ('fade_level', ctypes.c_uint16),
]


class _HapticConstant(ctypes.Structure):
    _fields_ = _EFFECT_HEADER + [('level', ctypes.c_int16)] + _ENVELOPE


class _HapticPeriodic(ctypes.Structure):
    _fields_ = _EFFECT_HEADER + [
        ('period', ctypes.c_uint16),
        ('magnitude', ctypes.c_int16),
        ('offset', ctypes.c_int16),
        ('phase', ctypes.c_uint16),
    ] + _ENVELOPE


class _HapticCondition(ctypes.Structure):
    _fields_ = _EFFECT_HEADER + [
        ('right_sat', ctypes.c_uint16 * 3),
        ('left_sat', ctypes.c_uint16 * 3),
        ('right_coeff', ctypes.c_int16 * 3),
        ('left_coeff', ctypes.c_int16 * 3),
        ('deadband', ctypes.c_uint16 * 3),
        ('center', ctypes.c_int16 * 3),
    ]


class _HapticEffect(ctypes.Union):
    # SDL copies the whole SDL_HapticEffect union, so the padding keeps this
    # at least as large as SDL's (which is under 80 bytes).
    _fields_ = [('type', ctypes.c_uint16),
                ('constant', _HapticConstant),
                ('periodic', _HapticPeriodic),
                ('condition', _HapticCondition),
                ('_size', ctypes.c_uint8 * 128)]


def _load_sdl():
    """The SDL2 library pygame uses, so joystick handles are shared with it."""
    path = os.path.join(os.path.dirname(pygame.__file__), 'SDL2.dll')
    sdl = ctypes.CDLL(path if os.path.exists(path) else 'libSDL2-2.0.so.0')
    sdl.SDL_JoystickFromInstanceID.restype = ctypes.c_void_p
    sdl.SDL_HapticOpenFromJoystick.argtypes = [ctypes.c_void_p]
    sdl.SDL_HapticOpenFromJoystick.restype = ctypes.c_void_p
    sdl.SDL_HapticQuery.argtypes = [ctypes.c_void_p]
    sdl.SDL_HapticQuery.restype = ctypes.c_uint
    sdl.SDL_HapticSetGain.argtypes = [ctypes.c_void_p, ctypes.c_int]
    sdl.SDL_HapticNewEffect.argtypes = [ctypes.c_void_p, ctypes.POINTER(_HapticEffect)]
    sdl.SDL_HapticUpdateEffect.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(_HapticEffect)]
    sdl.SDL_HapticRunEffect.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32]
    sdl.SDL_HapticStopAll.argtypes = [ctypes.c_void_p]
    sdl.SDL_HapticClose.argtypes = [ctypes.c_void_p]
    sdl.SDL_GetError.restype = ctypes.c_char_p
    return sdl


def _clamp(value, low=-1.0, high=1.0):
    return min(high, max(low, value))


class _Effect(object):
    """One running effect; set() updates it, sending only real changes."""

    def __init__(self, sdl, haptic, effect_type):
        self._sdl = sdl
        self._haptic = haptic
        self._effect = _HapticEffect()
        self._effect.type = effect_type
        e = self._effect.condition  # the header is the same in every member
        e.direction.type = SDL_HAPTIC_CARTESIAN
        e.direction.dir[0] = 1
        e.length = SDL_HAPTIC_INFINITY
        self._init()
        self._state = None
        self._id = sdl.SDL_HapticNewEffect(haptic, ctypes.byref(self._effect))
        if self._id < 0:
            raise RuntimeError(sdl.SDL_GetError().decode())
        sdl.SDL_HapticRunEffect(haptic, self._id, 1)

    def _init(self):
        pass

    def _send(self, state):
        if state == self._state:
            return False
        self._state = state
        return True

    def _update(self):
        self._sdl.SDL_HapticUpdateEffect(self._haptic, self._id, ctypes.byref(self._effect))


class _Condition(_Effect):
    """Spring, damper or friction: a strength in [0, 1], a centre in [-1, 1]."""

    def _init(self):
        c = self._effect.condition
        c.right_sat[0] = c.left_sat[0] = 0xFFFF

    def set(self, strength, center=0.0):
        coeff = int(round(32767 * _clamp(strength, 0.0)))
        centre = int(round(32767 * _clamp(center)))
        if self._send((coeff, centre)):
            c = self._effect.condition
            c.right_coeff[0] = c.left_coeff[0] = coeff
            c.center[0] = centre
            self._update()


class _Constant(_Effect):
    """A steady torque in [-1, 1]; positive turns the wheel right."""

    def set(self, torque):
        level = int(round(32767 * FFB_SIGN * _clamp(torque)))
        # Steps finer than the wheel can render are not worth a USB write.
        if self._state is not None and abs(level - self._state) < 128:
            return
        self._state = level
        self._effect.constant.level = level
        self._update()


class _Rumble(_Effect):
    """A sine vibration; strength in [0, 1]."""

    def _init(self):
        self._effect.periodic.period = 40  # ms, a coarse road texture

    def set(self, strength):
        magnitude = int(round(32767 * _clamp(strength, 0.0)))
        if self._state is not None and abs(magnitude - self._state) < 128:
            return
        self._state = magnitude
        self._effect.periodic.magnitude = magnitude
        self._update()


# ==============================================================================
# -- SteeringWheel -------------------------------------------------------------
# ==============================================================================


class _CarGeometry(object):
    """What the steering model needs of a car, read once per car."""

    def __init__(self, vehicle):
        self.id = vehicle.id
        try:
            angles = [w.max_steer_angle for w in vehicle.get_physics_control().wheels]
            self.max_steer = max(angles) or 70.0
        except RuntimeError:
            self.max_steer = 70.0
        # Front axle about 58% of the half-length ahead of the box centre.
        bb = vehicle.bounding_box
        self.front_axle = bb.location.x + 0.58 * bb.extent.x


class SteeringWheel(object):
    """A wheel with pedals: read() for the inputs, update_ffb() once a frame."""

    def __init__(self, joystick, ffb=True, steer_ratio=STEER_RATIO, ffb_gain=1.0):
        self.joystick = joystick
        self.name = joystick.get_name()
        self.steer_ratio = steer_ratio
        self.ffb_gain = ffb_gain
        # Until a pedal first moves, SDL reports 0, which is half pressed;
        # treat each pedal as released until it has reported something else.
        self._moved = set()
        # The shifter is optional; it takes over the gear once it is used.
        self._shifter_used = False
        self._buttons_down = set()
        self._car = None
        # Filters for the road effects.
        self._accel_z_mean = None
        self._roll_rate_mean = 0.0
        self._rumble = 0.0
        self._kick = 0.0
        self._sdl = None
        self._haptic = None
        if ffb:
            try:
                self._open_ffb()
            except Exception as error:
                print('Steering wheel: no force feedback (%s)' % error)
                self.close()

    @classmethod
    def find(cls, pattern='G25', **kwargs):
        """The first joystick whose name contains pattern, or None."""
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            if pattern.lower() in joystick.get_name().lower():
                joystick.init()
                return cls(joystick, **kwargs)
        return None

    @property
    def has_ffb(self):
        return self._haptic is not None

    def _open_ffb(self):
        sdl = _load_sdl()
        self._sdl = sdl
        if sdl.SDL_InitSubSystem(SDL_INIT_HAPTIC) != 0:
            raise RuntimeError(sdl.SDL_GetError().decode())
        handle = sdl.SDL_JoystickFromInstanceID(self.joystick.get_instance_id())
        haptic = sdl.SDL_HapticOpenFromJoystick(handle)
        if not haptic:
            raise RuntimeError(sdl.SDL_GetError().decode())
        self._haptic = haptic
        features = sdl.SDL_HapticQuery(haptic)
        needed = (SDL_HAPTIC_CONSTANT | SDL_HAPTIC_SINE | SDL_HAPTIC_SPRING |
                  SDL_HAPTIC_DAMPER | SDL_HAPTIC_FRICTION)
        if features & needed != needed:
            raise RuntimeError('wheel lacks constant/sine/spring/damper/friction effects')
        if features & SDL_HAPTIC_GAIN:
            sdl.SDL_HapticSetGain(haptic, 100)
        self._torque = _Constant(sdl, haptic, SDL_HAPTIC_CONSTANT)
        self._road = _Rumble(sdl, haptic, SDL_HAPTIC_SINE)
        self._spring = _Condition(sdl, haptic, SDL_HAPTIC_SPRING)
        self._damper = _Condition(sdl, haptic, SDL_HAPTIC_DAMPER)
        self._friction = _Condition(sdl, haptic, SDL_HAPTIC_FRICTION)

    def _geometry(self, vehicle):
        if self._car is None or self._car.id != vehicle.id:
            self._car = _CarGeometry(vehicle)
        return self._car

    def _pedal(self, axis):
        """0 released .. 1 fully pressed; the G25 reports +1 released, -1 pressed."""
        value = self.joystick.get_axis(axis)
        if axis not in self._moved:
            if value == 0.0:
                return 0.0
            self._moved.add(axis)
        pressed = (1.0 - value) / 2.0
        if pressed < PEDAL_DEADZONE:
            return 0.0
        return min(1.0, (pressed - PEDAL_DEADZONE) / (1.0 - 2 * PEDAL_DEADZONE))

    def _wheel_per_steer(self, vehicle):
        """Wheel axis travel (fraction of half range) for a CARLA steer of 1."""
        max_steer = self._geometry(vehicle).max_steer if vehicle is not None else 70.0
        return max_steer * self.steer_ratio / (WHEEL_RANGE / 2.0)

    def deviation(self, vehicle, steer):
        """Degrees the wheel is turned away from where CARLA steer would put it."""
        target = steer * self._wheel_per_steer(vehicle)
        return (self.joystick.get_axis(STEER_AXIS) - target) * WHEEL_RANGE / 2.0

    def read(self, vehicle=None):
        """(steer, throttle, brake, clutch): CARLA steer in [-1, 1], pedals in [0, 1]."""
        steer = self.joystick.get_axis(STEER_AXIS) / self._wheel_per_steer(vehicle)
        return (_clamp(steer),
                self._pedal(THROTTLE_AXIS),
                self._pedal(BRAKE_AXIS),
                self._pedal(CLUTCH_AXIS))

    def gear(self):
        """'P', 'N', 'D' or 'R' from the shifter; None until it is first used."""
        count = self.joystick.get_numbuttons()
        for button in SHIFTER_BUTTONS:
            if button < count and self.joystick.get_button(button):
                self._shifter_used = True
                return SHIFTER_GEARS.get(button, 'N')
        return 'N' if self._shifter_used else None

    def pressed(self):
        """Buttons pressed since the last call, the d-pad's DPAD_UP and DPAD_DOWN
        among them; call it once a frame."""
        down = set(b for b in range(self.joystick.get_numbuttons())
                   if self.joystick.get_button(b))
        if self.joystick.get_numhats():
            hat_y = self.joystick.get_hat(0)[1]
            if hat_y > 0:
                down.add(DPAD_UP)
            elif hat_y < 0:
                down.add(DPAD_DOWN)
        new = down - self._buttons_down
        self._buttons_down = down
        return new

    def is_down(self, button):
        """Whether the button is held now."""
        return button < self.joystick.get_numbuttons() and bool(self.joystick.get_button(button))

    def kick(self, impulse):
        """A jolt for a collision of this normal impulse (N s)."""
        self._kick = random.choice((-1.0, 1.0)) * min(1.0, impulse / 2000.0)

    def update_ffb(self, vehicle=None, imu=None, autopilot=False, dt=1.0 / 60.0):
        """Sets the forces from the car's motion; imu is the car's IMUSensor."""
        if self._haptic is None:
            return
        speed = aligning = 0.0
        steer = 0.0
        if vehicle is not None:
            speed, aligning, steer = self._aligning_torque(vehicle)
        kmh = 3.6 * speed
        rolling = min(1.0, kmh / 100.0)

        # Road: rumble from bumps, a kick from body roll, and collisions.
        roll_kick = 0.0
        if imu is not None:
            accel_z, roll_rate = imu.accelerometer[2], imu.gyroscope[0]
            if self._accel_z_mean is None:
                self._accel_z_mean = accel_z
            follow = min(1.0, dt / 0.25)
            self._accel_z_mean += (accel_z - self._accel_z_mean) * follow
            self._roll_rate_mean += (roll_rate - self._roll_rate_mean) * follow
            bump = abs(accel_z - self._accel_z_mean)
            self._rumble = max(bump * 0.05, self._rumble * 0.85)
            roll_kick = _clamp(0.006 * (roll_rate - self._roll_rate_mean), -0.35, 0.35)
        texture = 0.03 * min(1.0, kmh / 30.0)  # tyres on asphalt
        self._road.set(self.ffb_gain * min(0.6, texture + self._rumble))
        kick = self._kick
        self._kick *= math.exp(-dt / 0.08)

        if autopilot:
            # The spring turns the wheel to the car's steering; stiff enough
            # to turn it and to resist a hand at first, loose enough to grab
            # back (the driver script hands over past a set angle).
            self._spring.set(0.8, steer * self._wheel_per_steer(vehicle))
            self._friction.set(0.0)
            self._damper.set(0.2)
            self._torque.set(self.ffb_gain * (roll_kick + kick))
            return

        # Power steering: most of the tyres' torque is assisted away at
        # parking speed, less of it as the speed rises.
        feel = 0.45 + 0.55 * rolling
        self._torque.set(self.ffb_gain * (feel * aligning + roll_kick + kick))
        # Caster centring for when the tyres give no torque (standing, slow).
        self._spring.set(self.ffb_gain * (0.12 + 0.08 * rolling))
        # Scrub at a standstill, mostly gone by 15 km/h; power steering
        # keeps it light.
        self._friction.set(self.ffb_gain * 0.25 * max(0.0, 1.0 - kmh / 15.0))
        self._damper.set(self.ffb_gain * (0.20 + 0.15 * rolling))

    def _aligning_torque(self, vehicle):
        """(speed m/s, torque in [-1, 1] turning the wheel to the right, steer)."""
        car = self._geometry(vehicle)
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        steer = vehicle.get_control().steer
        forward = transform.get_forward_vector()
        right = transform.get_right_vector()
        vx = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
        vy = velocity.x * right.x + velocity.y * right.y + velocity.z * right.z
        speed = math.hypot(vx, vy)
        if vx < 0.5:
            return speed, 0.0, steer
        # Yaw rate, positive turning right (CARLA reports deg/s).
        yaw_rate = math.radians(vehicle.get_angular_velocity().z)
        kmh = 3.6 * speed
        share = _interpolate(STEERING_CURVE, kmh)
        wheel_angle = math.radians(steer * car.max_steer * share)
        travel = math.atan2(vy + yaw_rate * car.front_axle, vx)
        slip = (wheel_angle - travel) / PEAK_SLIP
        # Rises with slip, peaks at PEAK_SLIP, then falls away (understeer).
        torque = slip * math.exp(0.5 * (1.0 - slip * slip))
        # Slip is noise at a crawl; fade the torque in over the first 15 km/h.
        fade = min(1.0, (vx - 0.5) / 4.0)
        # The tyres turn the wheel back towards straight ahead (the travel).
        return speed, -_clamp(torque) * fade, steer

    def close(self):
        if self._haptic is not None:
            self._sdl.SDL_HapticStopAll(self._haptic)
            self._sdl.SDL_HapticClose(self._haptic)
            self._haptic = None


def _interpolate(curve, x):
    if x <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return curve[-1][1]


if __name__ == '__main__':
    # Prints the inputs as they change, with the standstill forces on.
    pygame.init()
    pygame.display.set_mode((360, 80))
    pygame.display.set_caption('Steering wheel test')
    wheel = SteeringWheel.find(sys.argv[1] if len(sys.argv) > 1 else 'G25')
    if wheel is None:
        sys.exit('No wheel found; joysticks: %s' % [
            pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())])
    print('%s, force feedback %s' % (wheel.name, 'on' if wheel.has_ffb else 'off'))
    clock = pygame.time.Clock()
    last = None
    try:
        while not any(e.type == pygame.QUIT for e in pygame.event.get()):
            wheel.update_ffb()
            now = tuple(round(v, 2) for v in wheel.read()) + (wheel.gear() or '-',)
            if now != last:
                print('steer %+.2f  throttle %.2f  brake %.2f  clutch %.2f  gear %s' % now)
                last = now
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        wheel.close()
