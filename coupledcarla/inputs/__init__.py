"""Driver inputs. Each returns a carla.VehicleControl (or None to leave the car alone) once per tick.

autopilot  SUMO drives the car (sumodriver.py; simulation.autopilot: sumo, the default), or with autopilot: carla
           CARLA's BasicAgent, following random lanes and stopping for cars and red lights.
keyboard   W/S or arrows to drive, A/D to steer, Q reverse, Space hand brake, Z/X blinkers, R back onto the
           nearest lane, Esc ends the trial.
wheel      a Logitech G25 (or similar) with force feedback, through steering_wheel.py.
myosuite   a musculoskeletal driver: MyoSuite muscles work the pedals and the wheel (myodriver.py, myosuite-carla).
"""
import os
import sys

import carla

from .. import compat


class Autopilot:
    """CARLA's BasicAgent with two changes for clips and studies: its junction choices come from its own random
    generator seeded by the car's role (the same lanes in every run of an experiment; the global one, unseeded, sent
    the Lyon clip's car left at one render and straight at the next), and it slows before curves and turns so its
    lateral acceleration stays under A_LAT (it took a junction turn at 39 km/h: 13 m/s^2)."""
    needs_display = False
    A_LAT = 2.0         # m/s^2: the lateral acceleration it plans its speed for in a curve or turn
    A_LAT_MAX = 2.6     # m/s^2: never more (steering is limited to it; measured from positions ~25 % more): CARLA's
                        # cars steer their wheels up to ~70 deg, at 10 km/h a 2 m radius and 4-5 m/s^2 (the clip
                        # QA blocks a participant at 4)
    B_PLAN = 2.5        # m/s^2: the deceleration it plans with to be slow enough at a curve ahead

    def __init__(self, vehicle, target_speed=40.0, seed=None):
        sys.path.append(os.path.join(os.environ["CARLA_ROOT"], "PythonAPI", "carla"))
        import random
        import zlib
        import agents.navigation.basic_agent as basic_agent
        import agents.navigation.local_planner as local_planner
        if seed is None:
            seed = zlib.crc32(str(vehicle.attributes.get("role_name", "autopilot")).encode())
        self.rng = random.Random(seed)
        local_planner.random = self.rng       # the local planner's junction choices (it calls random.choice)
        self.target_speed = float(target_speed)
        # CARLA's GlobalRoutePlanner can loop forever on OSM-derived maps; without it BasicAgent follows
        # random lanes, which is what a background test drive needs.
        basic_agent.GlobalRoutePlanner = lambda *a, **k: None
        self.agent = basic_agent.BasicAgent(vehicle, target_speed=target_speed)
        # CARLA's light actors do not work in OpenDRIVE-generated worlds (see signals.py); the runner sets
        # self.light from SUMO instead: (tls id, distance m, state) of the next light ahead, or None.
        self.agent.ignore_traffic_lights(True)
        self.light = None
        self.quit = False
        self.stuck = False

    def _curve_speed(self, vehicle):
        """km/h the car may go now so that, braking at B_PLAN, it takes every curve within the next waypoints of
        its plan at no more than A_LAT."""
        import math
        try:
            queue = list(self.agent.get_local_planner()._waypoints_queue)[:60]
            tfv = vehicle.get_transform()
        except (AttributeError, RuntimeError):
            return self.target_speed
        loc = tfv.location
        best = self.target_speed / 3.6
        # the turn the car still has to make towards its target waypoint: the local planner drops the waypoints
        # within a few metres and aims across a corner, so the curve it is in is no longer in its plan (pure
        # pursuit: curvature 2 sin(heading error) / distance)
        hy = math.radians(tfv.rotation.yaw)
        for wp, _opt in queue:
            dx, dy = wp.transform.location.x - loc.x, wp.transform.location.y - loc.y
            L = math.hypot(dx, dy)
            if L < 3.0:
                continue
            err = abs((math.degrees(math.atan2(dy, dx) - hy) + 180.0) % 360.0 - 180.0)
            k0 = 2.0 * math.sin(math.radians(min(err, 90.0))) / L
            if k0 > 1e-3:
                best = min(best, math.sqrt(self.A_LAT / k0))
            break
        s, prev = 0.0, (loc.x, loc.y, tfv.rotation.yaw)
        px, py = loc.x, loc.y
        for wp, _opt in queue:
            tf = wp.transform
            s += math.hypot(tf.location.x - px, tf.location.y - py)
            px, py = tf.location.x, tf.location.y
            yaw = tf.rotation.yaw
            if prev is not None:
                ds = max(0.5, math.hypot(tf.location.x - prev[0], tf.location.y - prev[1]))
                dyaw = abs((yaw - prev[2] + 180.0) % 360.0 - 180.0)
                k = math.radians(dyaw) / ds
                if k > 1e-3:
                    v_curve = math.sqrt(self.A_LAT / k)
                    best = min(best, math.sqrt(v_curve * v_curve + 2.0 * self.B_PLAN * max(0.0, s - ds)))
            prev = (tf.location.x, tf.location.y, yaw)
        return max(5.0, min(self.target_speed, best * 3.6))

    def _steer_limit(self, vehicle, speed):
        """The steering (0-1) that turns the car at A_LAT_MAX at this speed (1 when slow)."""
        import math
        if getattr(self, "_geom", None) is None:
            try:
                pc = vehicle.get_physics_control()
                w = pc.wheels
                front = [x for x in w[:2]]
                rear = [x for x in w[2:4]]
                fx = sum(x.position.x for x in front) / 2 / 100.0
                fy = sum(x.position.y for x in front) / 2 / 100.0
                rx = sum(x.position.x for x in rear) / 2 / 100.0
                ry = sum(x.position.y for x in rear) / 2 / 100.0
                self._geom = (max(1.5, math.hypot(fx - rx, fy - ry)), max(20.0, float(front[0].max_steer_angle)))
            except (AttributeError, RuntimeError, IndexError, ZeroDivisionError):
                self._geom = (2.8, 70.0)
        wheelbase, max_deg = self._geom
        if speed < 1.0:
            return 1.0
        k = self.A_LAT_MAX / (speed * speed)             # the tightest curvature allowed
        return min(1.0, math.degrees(math.atan(k * wheelbase)) / max_deg)

    def control(self, vehicle, events=()):
        try:
            self.agent.set_target_speed(self._curve_speed(vehicle))
        except AttributeError:
            pass
        try:
            ctl = self.agent.run_step()
            v = vehicle.get_velocity()
            lim = self._steer_limit(vehicle, (v.x ** 2 + v.y ** 2) ** 0.5)
            ctl.steer = max(-lim, min(lim, ctl.steer))
        except (IndexError, AttributeError):
            # CARLA's local planner throws where a lane simply ends (a lane drop); continue from a neighbouring
            # driving lane, or stand still and let the runner reset the car.
            ctl = self._replan(vehicle)
        if self.light and self.light[2] in "rRyYs":
            v = vehicle.get_velocity()
            speed = (v.x ** 2 + v.y ** 2) ** 0.5
            stop_dist = 4.0 + speed * 1.0 + speed ** 2 / (2 * 4.0)   # margin + 1 s + braking at 4 m/s^2
            if self.light[1] < stop_dist and not (self.light[2] in "yY" and self.light[1] < speed * 1.0):
                ctl = carla.VehicleControl(throttle=0.0, brake=min(1.0, 0.3 + speed / 15.0))
        return ctl

    def restart(self, wp):
        """Start following lanes again from waypoint wp (after the car was put back on the road)."""
        return self._plan_from(wp)

    def _plan_from(self, wp):
        from agents.navigation.local_planner import RoadOption
        for cand in (wp, compat.left_lane(wp), compat.right_lane(wp)):
            if cand is None or cand.lane_type != carla.LaneType.Driving or cand.lane_id * wp.lane_id < 0:
                continue
            plan, w = [], cand
            for _ in range(100):
                nxt = w.next(2.0)
                if not nxt:
                    break
                w = self.rng.choice(nxt)
                plan.append((w, RoadOption.LANEFOLLOW))
            if len(plan) > 10:
                self.agent.set_global_plan(plan, stop_waypoint_creation=False, clean_queue=True)
                return True
        return False

    def _replan(self, vehicle):
        wp = vehicle.get_world().get_map().get_waypoint(vehicle.get_location())
        if self._plan_from(wp):
            return self.agent.run_step()
        self.stuck = True
        return carla.VehicleControl(throttle=0.0, brake=1.0)


class Keyboard:
    needs_display = True

    def __init__(self, vehicle):
        self.ctl = carla.VehicleControl()
        self.quit = False
        self.reset = False
        self.lights = carla.VehicleLightState.NONE
        self.steer = 0.0
        self.send = None   # the runner's command queue: the light state goes with the tick's batch (run.py)
        self.keep = None   # the car's other lights (headlights after dark), read at the first control()

    def control(self, vehicle, events=()):
        self._keys(vehicle, events)
        self._send_lights(vehicle)
        return self.ctl

    def _send_lights(self, vehicle):
        """Blinkers and brake light from this input, the car's other lights as they were: the runner (and the
        host, for a station's car) switches the headlights on after dark, which every tick's light state from
        here switched off again."""
        LS = carla.VehicleLightState
        if self.keep is None:
            try:
                self.keep = int(vehicle.get_light_state()) & ~(int(LS.LeftBlinker) | int(LS.RightBlinker) |
                                                               int(LS.Brake))
            except (RuntimeError, AttributeError):
                self.keep = 0
        lights = carla.VehicleLightState(self.keep | int(self.lights) | (int(LS.Brake) if self.ctl.brake > 0 else 0))
        if self.send is not None:
            self.send(carla.command.SetVehicleLightState(vehicle.id, lights))
        else:
            vehicle.set_light_state(lights)

    def _keys(self, vehicle, events):
        import pygame
        for e in events:
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    self.quit = True
                elif e.key == pygame.K_q:
                    self.ctl.reverse = not self.ctl.reverse
                elif e.key == pygame.K_z:
                    self.lights ^= carla.VehicleLightState.LeftBlinker
                elif e.key == pygame.K_x:
                    self.lights ^= carla.VehicleLightState.RightBlinker
                elif e.key == pygame.K_r:
                    self.reset = True   # the runner puts the car back on the nearest lane
        k = pygame.key.get_pressed()
        self.ctl.throttle = 1.0 if (k[pygame.K_w] or k[pygame.K_UP]) else 0.0
        self.ctl.brake = 1.0 if (k[pygame.K_s] or k[pygame.K_DOWN]) else 0.0
        target = (-1 if (k[pygame.K_a] or k[pygame.K_LEFT]) else 0) + (1 if (k[pygame.K_d] or k[pygame.K_RIGHT]) else 0)
        self.steer += max(-0.05, min(0.05, 0.6 * target - self.steer)) if target else -self.steer * 0.3
        self.ctl.steer = max(-1.0, min(1.0, self.steer))
        self.ctl.hand_brake = bool(k[pygame.K_SPACE])


class Wheel(Keyboard):
    """G25 for steering and pedals; the keyboard still works for blinkers, reverse and Esc."""

    def __init__(self, vehicle, pattern="G25"):
        super().__init__(vehicle)
        import pygame
        from .steering_wheel import SteeringWheel
        pygame.joystick.init()
        self.wheel = SteeringWheel.find(pattern)
        if self.wheel is None:
            raise RuntimeError(f"no joystick named like {pattern!r}")

    def control(self, vehicle, events=()):
        self._keys(vehicle, events)
        steer, throttle, brake, _clutch = self.wheel.read(vehicle)
        self.ctl.steer, self.ctl.throttle, self.ctl.brake = steer, throttle, brake
        gear = self.wheel.gear() if hasattr(self.wheel, "gear") else None
        if gear in ("R", "D"):
            self.ctl.reverse = gear == "R"
        self._send_lights(vehicle)      # after the pedals: the brake light follows the brake pedal
        self.wheel.update_ffb(vehicle)
        return self.ctl


def make(kind, vehicle, **kw):
    if kind == "sumo":       # the SUMO driver (sumodriver.py): SUMO drives, the bridge moves the car
        from ..sumodriver import SumoDriver
        return SumoDriver(vehicle, **kw)
    if kind == "myosuite":   # the musculoskeletal driver (myodriver.py); its body is built in attach()
        from ..myodriver import MyoDriver
        return MyoDriver(vehicle, **kw)
    return {"autopilot": Autopilot, "keyboard": Keyboard, "wheel": Wheel}[kind](vehicle, **kw)
