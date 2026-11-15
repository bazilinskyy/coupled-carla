"""coupled-carla: SUMO vehicles near a camera move in CARLA as real vehicles, so their wheels roll and steer, cyclists
pedal and motorbike riders ride.

The bridge used to place every SUMO vehicle with ApplyTransform and physics off. CARLA 0.9.16 then reports 0 m/s for
it, its wheels never turn, and a bike's pedals (and with them the rider's legs: the rider follows the pedals) only
move when the bike's own physics drives it with throttle (measured: physics off, the pedal bone stays at its angle;
physics on with a target velocity alone, it stays too; with throttle as well, it turns). So near a participant
(simulation.animate_radius_m) a SUMO vehicle has physics on and follows its SUMO pose by velocity: each tick a
target velocity (SUMO's speed along its heading plus the position error over FOLLOW_S), a target yaw rate (the
heading error over FOLLOW_S plus SUMO's own turn rate), and a control with a little throttle while it moves (the
bike's pedalling animation) and the steering angle of its turn (the front wheels). Off by more than TELEPORT_M or
TELEPORT_DEG (a bump, a collision, a SUMO jump) it is put back with a transform. Farther away it is placed as before.
On lin4080 the follow tracked within 0.1 m at 5 m/s (bike, car, bus).

Pure functions here; the bridge (synchronization.py) turns them into carla commands.
"""
import math

FOLLOW_S = 0.3        # time constant of the position and heading correction (s)
MAX_CORRECTION = 3.0  # m/s at most added to SUMO's velocity for the position error
TELEPORT_M = 1.2      # farther off than this: put back with a transform
TELEPORT_DEG = 25.0
THROTTLE = 0.3        # while moving: the bike's pedalling (and a car's drive wheels)
MAX_STEER_DEG = 60.0  # steer 1.0 as a front-wheel angle (CARLA cars: 60-70 deg)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def follow(target, current, speed, yaw_rate_deg, wheelbase, vz=0.0):
    """Commands to follow a SUMO pose with physics on.

    target, current: (x, y, yaw_deg) in CARLA coordinates (the SUMO pose as CARLA's, the actor's now); speed (m/s)
    and yaw_rate_deg (deg/s) from SUMO; wheelbase (m) for the steering angle; vz: the actor's vertical speed now
    (gravity keeps it on the road). Returns {"teleport": bool, "velocity": (vx, vy, vz), "angular_z": deg/s,
    "throttle", "steer", "brake"}.
    """
    ex, ey = target[0] - current[0], target[1] - current[1]
    err = math.hypot(ex, ey)
    eyaw = wrap_deg(target[2] - current[2])
    teleport = err > TELEPORT_M or abs(eyaw) > TELEPORT_DEG
    yaw = math.radians(target[2])
    k = min(1.0, MAX_CORRECTION / (err / FOLLOW_S)) if err > 1e-6 else 0.0
    vx = speed * math.cos(yaw) + ex / FOLLOW_S * k
    vy = speed * math.sin(yaw) + ey / FOLLOW_S * k
    if teleport:
        eyaw = 0.0
    wz = yaw_rate_deg + eyaw / FOLLOW_S
    moving = speed > 0.3
    if moving and abs(yaw_rate_deg) > 0.05:
        delta = math.degrees(math.atan(wheelbase * math.radians(yaw_rate_deg) / speed))
    else:
        delta = 0.0
    steer = max(-1.0, min(1.0, delta / MAX_STEER_DEG))
    return {"teleport": teleport, "velocity": (vx, vy, vz), "angular_z": wz,
            "throttle": THROTTLE if moving else 0.0, "steer": steer, "brake": 0.0 if moving else 1.0}


def wheelbase(length):
    """A vehicle's wheelbase from its length (cars about 0.6 of it; a bike 1.1 m of 1.8 m)."""
    return max(0.9, 0.6 * length)


def yaw_rate(prev_yaw, yaw, dt):
    """deg/s between two SUMO headings (CARLA degrees) dt seconds apart."""
    return wrap_deg(yaw - prev_yaw) / dt if dt > 0 else 0.0


def velocity(speed, yaw_deg):
    """(vx, vy) in CARLA coordinates of a vehicle driving at speed along yaw_deg."""
    r = math.radians(yaw_deg)
    return speed * math.cos(r), speed * math.sin(r)
