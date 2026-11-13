"""Driver inputs. Each returns a carla.VehicleControl (or None to leave the car alone) once per tick.

autopilot  CARLA's BasicAgent, following random lanes and stopping for cars and red lights (headless tests).
keyboard   W/S or arrows to drive, A/D to steer, Q reverse, Space hand brake, Z/X blinkers, R back onto the
           nearest lane, Esc ends the trial.
wheel      a Logitech G25 (or similar) with force feedback, through steering_wheel.py.
"""
import os
import sys

import carla

from .. import compat


class Autopilot:
    needs_display = False

    def __init__(self, vehicle, target_speed=40.0):
        sys.path.append(os.path.join(os.environ["CARLA_ROOT"], "PythonAPI", "carla"))
        import agents.navigation.basic_agent as basic_agent
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

    def control(self, vehicle, events=()):
        try:
            ctl = self.agent.run_step()
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
        import random
        from agents.navigation.local_planner import RoadOption
        for cand in (wp, compat.left_lane(wp), compat.right_lane(wp)):
            if cand is None or cand.lane_type != carla.LaneType.Driving or cand.lane_id * wp.lane_id < 0:
                continue
            plan, w = [], cand
            for _ in range(100):
                nxt = w.next(2.0)
                if not nxt:
                    break
                w = random.choice(nxt)
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

    def control(self, vehicle, events=()):
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
        lights = self.lights | (carla.VehicleLightState.Brake if self.ctl.brake > 0 else 0)
        vehicle.set_light_state(carla.VehicleLightState(lights))
        return self.ctl


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
        super().control(vehicle, events)
        steer, throttle, brake, _clutch = self.wheel.read(vehicle)
        self.ctl.steer, self.ctl.throttle, self.ctl.brake = steer, throttle, brake
        gear = self.wheel.gear() if hasattr(self.wheel, "gear") else None
        if gear in ("R", "D"):
            self.ctl.reverse = gear == "R"
        self.wheel.update_ffb(vehicle)
        return self.ctl


def make(kind, vehicle, **kw):
    return {"autopilot": Autopilot, "keyboard": Keyboard, "wheel": Wheel}[kind](vehicle, **kw)
