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
                 animate_radius=0.0,
                 smooth_paths=True,
                 driver_mix_seed=None,
                 red_running=False,
                 depart_hide_m=0.0):

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
        # coupled-carla: each SUMO vehicle in CARLA drives a smoothed copy of its SUMO path (animate.PathSmoother:
        # lateral acceleration and steering rate of a human driver, a short lag), so a kink in a SUMO lane or a
        # sharp junction shape is not shown as a swerve; a participant on the SUMO driver gets the gentler limits
        # coupled-carla: each new SUMO vehicle gets its own driver (drivermix.py); None: the vTypes' values for all
        self.driver_mix_seed = driver_mix_seed
        self.red_running = red_running
        self.depart_hide_m = depart_hide_m
        self.smooth_paths = bool(smooth_paths)
        self.smoothers = {}
        self.lane_shapes = None
        if self.smooth_paths and getattr(self.sumo, "net_file", None):
            try:
                self.lane_shapes = animate.LaneShapes(self.sumo.net_file)
            except Exception as err:  # noqa: BLE001 - no look-ahead: kinks are smoothed over the body only
                logging.warning("lane shapes for the look-ahead not read (%s)", err)
        # coupled-carla: CARLA actors whose SUMO vehicle drives them (a participant on the SUMO driver, inputs.py):
        # {sumo id: carla id}; mapped like SUMO's own vehicles once SUMO has inserted them, never spawned, despawned
        # or destroyed by the bridge. arrived_adopted: those SUMO took off the network this tick.
        self.adopted, self.arrived_adopted = {}, set()
        self._sumo_live = set()   # SUMO-controlled actors currently in SUMO (subscribed)
        self._lod_tick = 0
        self._common_landmarks = None   # coupled-carla: traffic lights both sides know (cached, see tick())
        # coupled-carla: laps(name) is called after each part of tick() (run.py times them for its profile line)
        self.laps = lambda name: None

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
        sumo_spawned_actors = self._drop_departures_in_sight(sumo_spawned_actors)
        if getattr(self, "driver_mix_seed", None) is not None:
            import traci  # pylint: disable=import-outside-toplevel
            from coupledcarla import drivermix  # pylint: disable=import-outside-toplevel
            for sumo_actor_id in sumo_spawned_actors:
                if sumo_actor_id.startswith("carla"):      # a CARLA actor's twin: driven by CARLA
                    continue
                try:
                    drivermix.apply(sumo_actor_id, traci.vehicle.getVehicleClass(sumo_actor_id), self.driver_mix_seed,
                                    red_running=getattr(self, "red_running", False))
                except traci.TraCIException:
                    pass
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
                sumo_actor = self.sumo.get_actor(sumo_actor_id)
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
            speed = sumo_actor.speed
            if getattr(self, "smooth_paths", False):
                sm = self.smoothers.get(carla_actor_id)
                if sm is None:
                    sm = self.smoothers[carla_actor_id] = (animate.PathSmoother.participant()
                                                           if sumo_actor_id in self.adopted else animate.PathSmoother())
                aim = self._aim(sumo_actor_id, sumo_actor.speed)
                loc.x, loc.y, yaw, yaw_rate, speed = sm.step((loc.x, loc.y, yaw), sumo_actor.speed,
                                                             2.0 * sumo_actor.extent.x, self.carla.step_length,
                                                             aim=aim)
                carla_transform.rotation.yaw = yaw
            self.velocity[carla_actor_id] = animate.velocity(speed, yaw)
            self.poses[carla_actor_id] = (loc.x, loc.y, yaw, speed, sumo_actor.extent.x,
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
                self.carla.follow_vehicle(carla_actor_id, carla_transform, speed, yaw_rate,
                                          2.0 * sumo_actor.extent.x, carla_lights)
            else:
                self.carla.place_vehicle(carla_actor_id, carla_transform, carla_lights)
        for gone in [i for i in self._prev_yaw if i not in self.velocity]:
            del self._prev_yaw[gone]
        for gone in [i for i in getattr(self, "smoothers", {}) if i not in self.velocity]:
            del self.smoothers[gone]
        if getattr(self, "lane_shapes", None) is not None and len(self.lane_shapes.cache) > 2 * len(self.sumo2carla_ids) + 50:
            live = set(self.sumo2carla_ids)
            for vid in [v for v in self.lane_shapes.cache if v not in live]:
                self.lane_shapes.forget(vid)
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

    def _drop_departures_in_sight(self, departed):
        """coupled-carla: a car or bike that SUMO inserts within depart_hide_m of a participant or free camera (a
        trip starting on an inner street) would appear out of nowhere in front of them (Tokyo clip: a car 40 m from
        the camera): it is taken out of SUMO before it is shown. Buses (their line's first stop) and CARLA actors'
        twins stay. Returns the departures that remain."""
        hide = float(getattr(self, "depart_hide_m", 0.0) or 0.0)
        if hide <= 0 or not departed or not self.focus:
            return departed
        centres = self.focus() or []
        if not centres:
            return departed
        import traci  # pylint: disable=import-outside-toplevel
        off = BridgeHelper.offset
        keep = set()
        for vid in departed:
            try:
                if vid.startswith("carla") or traci.vehicle.getVehicleClass(vid) == "bus":
                    keep.add(vid)
                    continue
                x, y = traci.vehicle.getPosition(vid)
            except traci.TraCIException:
                keep.add(vid)
                continue
            cx, cy = x - off[0], -(y - off[1])
            if any((cx - c.x) ** 2 + (cy - c.y) ** 2 < hide * hide for c in centres):
                try:
                    traci.vehicle.remove(vid)
                    self.departures_hidden = getattr(self, "departures_hidden", 0) + 1
                    continue
                except traci.TraCIException:
                    pass
            keep.add(vid)
        return keep

    def _aim(self, sumo_actor_id, speed):
        """coupled-carla: the CARLA point on the vehicle's SUMO path animate.look_ahead(speed) beyond its front
        bumper (its lane, then the lanes it takes next), or None."""
        if getattr(self, "lane_shapes", None) is None:
            return None
        st = self.sumo.get_lane_state(sumo_actor_id)
        if st is None:
            return None
        lane, pos, lat = st
        import traci  # pylint: disable=import-outside-toplevel
        shapes = self.lane_shapes.ahead(sumo_actor_id, lane, lambda: traci.vehicle.getNextLinks(sumo_actor_id))
        q = animate.path_point(shapes, pos, animate.look_ahead(speed), lat)
        if q is None:
            return None
        off = BridgeHelper.offset
        return q[0] - off[0], -(q[1] - off[1])

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
