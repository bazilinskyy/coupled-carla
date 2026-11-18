#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
SUMO <-> CARLA synchronisation, vendored from CARLA 0.9.16 Co-Simulation/Sumo/run_synchronization.py.

Changes for coupled-carla: imported as a module (no CLI, no SUMO_HOME path hack; traci and sumolib come from
pip), and the Traffic Manager port is configurable because port 8000 is often taken. Level of detail (SUMO actors
in CARLA only near the participants, far ones subscribed to their pose only), batched CARLA updates, light states
from the bridge's own cache and a cached traffic-light index keep a tick within a few ms of the server's own.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import os
import sys
import time


# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from .sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from .sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from .sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from .sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
from . import animate  # pylint: disable=wrong-import-position
from . import pipeline  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False,
                 tm_port=8000,
                 focus=None,
                 radius=None,
                 animate_radius=0.0):

        self.sumo = sumo_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

        # coupled-carla level of detail: SUMO simulates the whole network, but a SUMO actor exists in CARLA
        # only while it is within `radius` metres of one of the locations focus() returns (the participants);
        # it is removed again beyond radius + 30 m. Each CARLA vehicle costs server time every tick (a
        # cyclist about 3 ms), so this is what keeps a city-sized map real time.
        self.focus = focus
        self.radius = radius
        # coupled-carla: SUMO vehicles within animate_radius of a participant move with physics on, so wheels roll
        # and cyclists pedal (bridge/animate.py); velocity and poses give every SUMO vehicle's speed to the CARLA
        # side (CARLA reads 0 m/s for one placed with physics off): {carla id: (vx, vy)}, {carla id: (x, y, yaw,
        # speed, half length, half width)}, refilled in place every tick
        self.animate_radius = float(animate_radius or 0.0)
        self.velocity, self.poses, self._prev_yaw = {}, {}, {}
        # coupled-carla: CARLA actors whose SUMO vehicle drives them (a participant on the SUMO driver, inputs.py):
        # {sumo id: carla id}; mapped like SUMO's own vehicles once SUMO has inserted them, never spawned, despawned
        # or destroyed by the bridge. arrived_adopted: those SUMO took off the network this tick.
        self.adopted, self.arrived_adopted = {}, set()
        self._sumo_live = set()   # SUMO-controlled actors currently in SUMO (subscribed)
        self._lod_tick = 0
        self._common_landmarks = None   # coupled-carla: traffic lights both sides know (cached, see tick())
        # coupled-carla: laps(name) is called after each part of tick() (run.py times them for its profile line)
        self.laps = lambda name: None

        # coupled-carla: the pipelined tick (bridge/pipeline.py; configure_pipeline() turns it on). hooks run at each
        # SUMO-step boundary, before the step starts (where TraCI calls cost no wait); sumo_stepped says whether this
        # tick had a SUMO step (always on the plain tick).
        self.pipelined, self.ratio, self.threaded, self.near = False, 1, False, False
        self.hooks = []
        self.sumo_stepped = True

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        # coupled-carla: CARLA's Traffic Manager is not used (SUMO drives the traffic), and creating it builds
        # its own map of the whole network, which segfaults on some OSM-derived maps (Luang Prabang). It is
        # only started when a port is given.
        if tm_port:
            traffic_manager = self.carla.client.get_trafficmanager(tm_port)
            traffic_manager.set_synchronous_mode(True)

    def tick(self):
        """
        Tick to simulation synchronization
        """
        if getattr(self, "pipelined", False):
            return self._tick_pipelined()
        laps = self.laps
        lod = bool(self.focus and self.radius)
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()
        laps("sumo_step")

        # New sumo actors (not controlled by carla) are subscribed; whether they appear in carla is decided
        # by the level-of-detail pass below. coupled-carla: with level of detail, to their pose only until then.
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sumo_spawned_actors & self.adopted.keys():     # coupled-carla: a SUMO-driven participant
            self.sumo.subscribe(sumo_actor_id)
            self.sumo2carla_ids[sumo_actor_id] = self.adopted[sumo_actor_id]
        sumo_spawned_actors -= self.adopted.keys()
        for sumo_actor_id in sumo_spawned_actors:
            if lod:
                self.sumo.subscribe_pose(sumo_actor_id)
            else:
                self.sumo.subscribe(sumo_actor_id)
            self._sumo_live.add(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        self.arrived_adopted = self.sumo.destroyed_actors & self.adopted.keys()
        for sumo_actor_id in self.arrived_adopted:      # coupled-carla: the participant's car stays in CARLA
            self.sumo2carla_ids.pop(sumo_actor_id, None)
        for sumo_actor_id in self.sumo.destroyed_actors - self.adopted.keys():
            self._sumo_live.discard(sumo_actor_id)
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # coupled-carla: SUMO also removes the twin of a CARLA-driven car: one standing still for time-to-teleport
        # (120 s, e.g. a participant waiting with SUMO traffic queued behind it) is teleported along its one-edge
        # route and arrives. get_actor() on the gone twin raised KeyError and ended the trial; it gets a new twin
        # below instead, where new CARLA actors get theirs.
        lost_twins = {carla_actor_id for carla_actor_id, sumo_actor_id in self.carla2sumo_ids.items()
                      if sumo_actor_id in self.sumo.destroyed_actors}
        for carla_actor_id in lost_twins:
            logging.warning('SUMO removed the twin %s of CARLA actor %s; adding a new one',
                            self.carla2sumo_ids.pop(carla_actor_id), carla_actor_id)

        # Level of detail: every 5 ticks (and for actors that just appeared), spawn in carla what is near a
        # participant and remove what has gone far.
        self._lod_tick += 1
        centres = self.focus() if (self.focus and self.radius) else None
        check = self._sumo_live if self._lod_tick % 5 == 0 else sumo_spawned_actors & self._sumo_live
        for sumo_actor_id in list(check):
            pose = self.sumo.get_pose(sumo_actor_id)
            if pose is None:
                continue
            carla_transform = BridgeHelper.get_carla_transform(*pose)
            loc = carla_transform.location
            if centres is None:
                near = far = None
            else:
                d = min((((loc.x - c.x) ** 2 + (loc.y - c.y) ** 2) ** 0.5 for c in centres), default=1e9)
                near, far = d <= self.radius, d > self.radius + 30.0
            in_carla = sumo_actor_id in self.sumo2carla_ids
            if not in_carla and (centres is None or near):
                if lod:   # everything about it now (traci fills the results at once, without a step)
                    self.sumo.subscribe(sumo_actor_id)
                sumo_actor = self.sumo.get_actor(sumo_actor_id)
                carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
                if carla_blueprint is None:
                    self.sumo.unsubscribe(sumo_actor_id)
                    self._sumo_live.discard(sumo_actor_id)
                    continue
                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                elif lod:
                    self.sumo.unsubscribe(sumo_actor_id)   # SUMO adds to a subscription: drop it first
                    self.sumo.subscribe_pose(sumo_actor_id)
            elif in_carla and far:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))
                if lod:
                    self.sumo.unsubscribe(sumo_actor_id)
                    self.sumo.subscribe_pose(sumo_actor_id)
        laps("lod")

        self._place_sumo_actors(self.sumo.get_actor, centres)
        laps("sumo_to_carla")

        # Updates traffic lights in carla based on sumo information.
        if self.tls_manager == 'sumo':
            # coupled-carla: both id sets are fixed (SUMO's changes only with a light's program)
            sumo_ids = self.sumo.traffic_light_ids
            if self._common_landmarks is None or self._common_landmarks[0] != sumo_ids:
                self._common_landmarks = (sumo_ids, sumo_ids & self.carla.traffic_light_ids)
            for landmark_id in self._common_landmarks[1]:
                sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)
        laps("lights")

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()
        laps("carla_tick")

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = (self.carla.spawned_actors | (lost_twins - self.carla.destroyed_actors)) - \
            set(self.sumo2carla_ids.values()) - set(self.adopted.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)
            if carla_actor is None:     # coupled-carla: gone meanwhile (a lost twin's car)
                continue

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            for sid in [k for k, v in self.adopted.items() if v == carla_actor_id]:   # coupled-carla
                self.adopted.pop(sid)
                if self.sumo2carla_ids.pop(sid, None) is not None:
                    self.sumo.destroy_actor(sid)
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                self.carla.frame_lights[carla_actor_id] = carla_lights   # coupled-carla: the logger reuses it
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)
        laps("carla_to_sumo")

    def _place_sumo_actors(self, get_actor, centres):
        """Moves each SUMO actor's CARLA car to where get_actor(sumo id) says (a SumoActor; KeyError: none this
        tick). The pipelined tick passes an interpolating getter."""
        # Updating sumo actors in carla.
        # coupled-carla: vehicles whose SUMO bodies touch another's (a SUMO collision, logged, not resolved) are placed
        # with physics off: with physics on CARLA pushed them apart, and they spun round (bridge/animate.py)
        touching = set()
        if self.animate_radius > 0 and self.poses:
            from coupledcarla.drivecheck import frame_conflicts  # pylint: disable=import-outside-toplevel
            boxes = {cid: (p[0], p[1], p[2], p[4], p[5]) for cid, p in self.poses.items()}
            if len(boxes) > 1:
                pairs, _ = frame_conflicts(boxes, {}, shrink=0.05)
                touching = {c for pair in pairs for c in pair}
        self.velocity.clear()
        self.poses.clear()
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

            try:
                sumo_actor = get_actor(sumo_actor_id)
            except KeyError:     # coupled-carla: off the network for a moment (a SUMO teleport of an adopted car)
                continue

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            loc = carla_transform.location
            yaw = animate.steady_yaw(self._prev_yaw.get(carla_actor_id), carla_transform.rotation.yaw,
                                     sumo_actor.speed)
            if yaw != carla_transform.rotation.yaw:  # a SUMO heading flip is not shown (animate.steady_yaw): the
                # centre was put half a length behind SUMO's front bumper along the flipped heading; along this one
                (ox, oy), (nx, ny) = (animate.velocity(sumo_actor.extent.x, carla_transform.rotation.yaw),
                                      animate.velocity(sumo_actor.extent.x, yaw))
                loc.x, loc.y = loc.x + ox - nx, loc.y + oy - ny
                carla_transform.rotation.yaw = yaw
            yaw_rate = animate.yaw_rate(self._prev_yaw.get(carla_actor_id, yaw), yaw, self.carla.step_length)
            self._prev_yaw[carla_actor_id] = yaw
            self.velocity[carla_actor_id] = animate.velocity(sumo_actor.speed, yaw)
            self.poses[carla_actor_id] = (loc.x, loc.y, yaw, sumo_actor.speed, sumo_actor.extent.x,
                                          sumo_actor.extent.y)
            animated = False
            if self.animate_radius > 0 and (centres or sumo_actor_id in self.adopted):
                reach = self.animate_radius + (10.0 if carla_actor_id in self.carla._physics else 0.0)
                animated = sumo_actor_id in self.adopted or (carla_actor_id not in touching and any(
                    (loc.x - c.x) ** 2 + (loc.y - c.y) ** 2 <= reach * reach for c in (centres or ())))
            if self.sync_vehicle_lights:
                # coupled-carla: the lights the bridge last gave it, instead of asking the server for every car
                # every tick (a blocking round trip each)
                carla_lights = BridgeHelper.get_carla_lights_state(self.carla.current_lights(carla_actor_id),
                                                                   sumo_actor.signals)
            else:
                carla_lights = None

            if animated:
                self.carla.follow_vehicle(carla_actor_id, carla_transform, sumo_actor.speed, yaw_rate,
                                          2.0 * sumo_actor.extent.x, carla_lights)
            else:
                self.carla.place_vehicle(carla_actor_id, carla_transform, carla_lights)
        for gone in [i for i in self._prev_yaw if i not in self.velocity]:
            del self._prev_yaw[gone]

    # ---------------------------------------------------------------------------------------------------------------
    # coupled-carla: the pipelined tick (bridge/pipeline.py)
    # ---------------------------------------------------------------------------------------------------------------

    def configure_pipeline(self, ratio=1, threaded=False, near=False, near_m=None):
        """SUMO every `ratio` CARLA ticks with interpolated poses in between; with `threaded` that step runs while
        CARLA ticks; with `near` SUMO is asked only about vehicles within the level-of-detail radius (+30 m) and
        traffic lights within near_m (default radius + 50 m) of the participants. All off: the plain tick."""
        self.ratio, self.threaded = max(1, int(ratio)), bool(threaded)
        self.near = bool(near) and bool(self.focus and self.radius)
        self.pipelined = self.ratio > 1 or self.threaded or self.near
        self.near_m = float(near_m or ((self.radius or 250.0) + 50.0))
        self._n = 0
        self._cur, self._prev = {}, {}
        self._acc = pipeline.Accumulator()
        self._tls_xy = None
        self._boundaries = 0
        self._new_sumo = set()
        self.hooks = getattr(self, "hooks", [])
        self.sumo_stepped = True

    def _sumo_points(self, centres):
        off = BridgeHelper.offset
        return [(c.x + off[0], off[1] - c.y) for c in (centres or ())]

    def _tick_pipelined(self):
        import carla  # pylint: disable=import-outside-toplevel,import-error
        laps = self.laps
        k = self.ratio
        j = self._n % k
        self._n += 1
        centres = self.focus() if (self.focus and self.radius) else None
        if j == 0:
            # the SUMO-step boundary: the step begun at the last one has finished (or runs now, the first time)
            if self._n == 1:
                self.sumo.tick()
            else:
                self.sumo.wait_step()
            laps("sumo_step")
            self.sumo.frozen = None
            lost_twins = self._boundary_sumo_events()
            self._boundary_carla_to_sumo(lost_twins)
            laps("carla_to_sumo")
            self._boundary_lod(centres)
            laps("lod")
            self._boundary_lights(centres)
            laps("lights")
            self._prev, cur = self._cur, {}
            for sumo_actor_id in self.sumo2carla_ids:
                try:
                    cur[sumo_actor_id] = self.sumo.read_actor(sumo_actor_id)
                except KeyError:
                    pass
            self._cur = cur
            self.sumo.frozen = cur
            self.sumo_stepped = True
            for hook in self.hooks:
                hook()
            laps("hooks")
            # the next SUMO step: in a thread while CARLA ticks, or now
            if self.threaded:
                self.sumo.start_step()
            else:
                self.sumo.tick()
            laps("sumo_step")
            self._boundaries += 1
        else:
            self.sumo_stepped = False
            self.arrived_adopted = set()
        f = (j + 1) / float(k)
        cur, prev = self._cur, self._prev

        def make(x, y, z, pitch, yaw):
            return carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch, yaw, 0.0))

        def get(sumo_actor_id):
            return pipeline.lerp_actor(prev.get(sumo_actor_id), cur[sumo_actor_id], f, make)
        self._place_sumo_actors(get, centres)
        laps("sumo_to_carla")
        self.carla.tick()
        self._acc.add(self.carla.spawned_actors, self.carla.destroyed_actors)
        laps("carla_tick")

    def _boundary_sumo_events(self):
        """SUMO's departures and arrivals of the step that just ran (the plain tick's first part); returns the CARLA
        actors whose SUMO twin SUMO removed."""
        lod = bool(self.focus and self.radius)
        spawned = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sorted(spawned & self.adopted.keys()):
            self.sumo.subscribe(sumo_actor_id)
            self.sumo2carla_ids[sumo_actor_id] = self.adopted[sumo_actor_id]
        spawned -= self.adopted.keys()
        for sumo_actor_id in sorted(spawned):
            if not self.near:        # near: the focus context subscriptions see them
                if lod:
                    self.sumo.subscribe_pose(sumo_actor_id)
                else:
                    self.sumo.subscribe(sumo_actor_id)
            self._sumo_live.add(sumo_actor_id)
        self._new_sumo = spawned
        self.arrived_adopted = self.sumo.destroyed_actors & self.adopted.keys()
        for sumo_actor_id in self.arrived_adopted:
            self.sumo2carla_ids.pop(sumo_actor_id, None)
        for sumo_actor_id in sorted(self.sumo.destroyed_actors - self.adopted.keys()):
            self._sumo_live.discard(sumo_actor_id)
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))
        lost_twins = {carla_actor_id for carla_actor_id, sumo_actor_id in self.carla2sumo_ids.items()
                      if sumo_actor_id in self.sumo.destroyed_actors}
        for carla_actor_id in sorted(lost_twins):
            logging.warning('SUMO removed the twin %s of CARLA actor %s; adding a new one',
                            self.carla2sumo_ids.pop(carla_actor_id), carla_actor_id)
        return lost_twins

    def _spawn_in_carla(self, sumo_actor_id, carla_transform, lod):
        """The plain tick's spawn of a SUMO actor in CARLA (subscribed to everything first)."""
        if lod:
            self.sumo.subscribe(sumo_actor_id)
        sumo_actor = self.sumo.read_actor(sumo_actor_id)
        carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
        if carla_blueprint is None:
            self.sumo.unsubscribe(sumo_actor_id)
            self._sumo_live.discard(sumo_actor_id)
            return
        carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
        if carla_actor_id != INVALID_ACTOR_ID:
            self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
        elif lod:
            self.sumo.unsubscribe(sumo_actor_id)
            if not self.near:
                self.sumo.subscribe_pose(sumo_actor_id)

    def _despawn(self, sumo_actor_id, lod):
        self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))
        if lod:
            self.sumo.unsubscribe(sumo_actor_id)
            if not self.near:
                self.sumo.subscribe_pose(sumo_actor_id)

    def _boundary_lod(self, centres):
        """Level of detail at a SUMO-step boundary: near SUMO actors into CARLA, far ones out (as the plain tick's)."""
        lod = bool(self.focus and self.radius)
        if self.near:
            near = self.sumo.near_poses()       # within radius + 30 m of the focus points of the last step
            self.sumo.set_focus(self._sumo_points(centres), self.radius + 30.0)
            off = BridgeHelper.offset
            for sumo_actor_id in sorted(near):
                if sumo_actor_id in self.adopted or sumo_actor_id not in self._sumo_live \
                        or sumo_actor_id in self.sumo2carla_ids:
                    continue
                x, y = near[sumo_actor_id]
                cx, cy = x - off[0], off[1] - y
                if any((cx - c.x) ** 2 + (cy - c.y) ** 2 <= self.radius ** 2 for c in (centres or ())):
                    self.sumo.subscribe(sumo_actor_id)
                    pose = self.sumo.get_pose(sumo_actor_id)
                    if pose is None:
                        self.sumo.unsubscribe(sumo_actor_id)
                        continue
                    self._spawn_in_carla(sumo_actor_id, BridgeHelper.get_carla_transform(*pose), False)
                    if sumo_actor_id not in self.sumo2carla_ids:
                        self.sumo.unsubscribe(sumo_actor_id)
            for sumo_actor_id in sorted(self.sumo2carla_ids):
                if sumo_actor_id not in self.adopted and sumo_actor_id not in near and self._boundaries > 0:
                    self._despawn(sumo_actor_id, lod)
            return
        every = max(1, int(round(5.0 / self.ratio)))
        check = self._sumo_live if self._boundaries % every == 0 else self._new_sumo & self._sumo_live
        for sumo_actor_id in sorted(check):
            pose = self.sumo.get_pose(sumo_actor_id)
            if pose is None:
                continue
            carla_transform = BridgeHelper.get_carla_transform(*pose)
            loc = carla_transform.location
            if centres is None:
                near = far = None
            else:
                d = min((((loc.x - c.x) ** 2 + (loc.y - c.y) ** 2) ** 0.5 for c in centres), default=1e9)
                near, far = d <= self.radius, d > self.radius + 30.0
            in_carla = sumo_actor_id in self.sumo2carla_ids
            if not in_carla and (centres is None or near):
                self._spawn_in_carla(sumo_actor_id, carla_transform, lod)
            elif in_carla and far:
                self._despawn(sumo_actor_id, lod)

    def _boundary_carla_to_sumo(self, lost_twins):
        """The plain tick's carla-->sumo part, for the CARLA ticks since the last boundary."""
        spawned, destroyed = self._acc.take(self.carla._active_actors)
        carla_spawned_actors = (spawned | (lost_twins - destroyed)) - \
            set(self.sumo2carla_ids.values()) - set(self.adopted.values())
        for carla_actor_id in sorted(carla_spawned_actors):
            carla_actor = self.carla.get_actor(carla_actor_id)
            if carla_actor is None:
                continue
            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)
        for carla_actor_id in sorted(destroyed):
            for sid in [k for k, v in self.adopted.items() if v == carla_actor_id]:
                self.adopted.pop(sid)
                if self.sumo2carla_ids.pop(sid, None) is not None:
                    self.sumo.destroy_actor(sid)
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))
        for carla_actor_id in sorted(self.carla2sumo_ids):
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]
            carla_actor = self.carla.get_actor(carla_actor_id)
            try:
                sumo_actor = self.sumo.read_actor(sumo_actor_id)
            except KeyError:          # added at this boundary: no results before its first step
                sumo_actor = None
            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            sumo_lights = None
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                self.carla.frame_lights[carla_actor_id] = carla_lights
                if carla_lights is not None and sumo_actor is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals, carla_lights)
            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in sorted(common_landmarks):
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def _tls_positions(self):
        """{light id: (x, y)} in SUMO's net: the mean end of its incoming lanes."""
        if self._tls_xy is None:
            out = {}
            net = self.sumo.net
            for tls in (net.getTrafficLights() if net is not None else ()):
                pts = [conn[0].getEdge().getToNode().getCoord() for conn in tls.getConnections()]
                if pts:
                    out[tls.getID()] = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
            self._tls_xy = out
        return self._tls_xy

    def _boundary_lights(self, centres):
        """SUMO's light states to CARLA (the plain tick's), with near only for the lights near the participants."""
        if self.tls_manager != 'sumo':
            return
        sumo_ids = self.sumo.traffic_light_ids
        if self._common_landmarks is None or self._common_landmarks[0] != sumo_ids:
            self._common_landmarks = (sumo_ids, sumo_ids & self.carla.traffic_light_ids)
        landmarks = self._common_landmarks[1]
        if self.near:
            tm = self.sumo.traffic_light_manager
            near_tls = pipeline.near_ids(self._tls_positions(), self._sumo_points(centres), self.near_m)
            tm.set_near(near_tls)
            index = tm._landmark_index()
            landmarks = {l for l in landmarks if any(t in near_tls for t, _ in index.get(l, ()))}
        for landmark_id in sorted(landmarks):
            sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
            carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)
            self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

    def adopt(self, sumo_actor_id, carla_actor_id):
        """
        coupled-carla: SUMO's vehicle sumo_actor_id (added to SUMO, perhaps not yet inserted) drives the existing CARLA
        actor carla_actor_id from now on; that actor gets no SUMO twin. Call it before the actor's first tick() here.
        """
        self.adopted[sumo_actor_id] = carla_actor_id
        twin = self.carla2sumo_ids.pop(carla_actor_id, None)
        if twin is not None:
            self.sumo.destroy_actor(twin)

    def close(self):
        """
        Cleans synchronization.
        """
        try:      # coupled-carla: a SUMO step still running in its thread
            self.sumo.wait_step()
        except Exception:  # pylint: disable=broad-except
            pass
        self.sumo.frozen = None
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()
