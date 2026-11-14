#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging
import time

import carla  # pylint: disable=import-error

from .constants import INVALID_ACTOR_ID, SPAWN_OFFSET_Z

# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, host, port, step_length):
        self.client = carla.Client(host, port)
        self.client.set_timeout(30.0)  # coupled-carla: 2 s is too short for a loaded server

        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.step_length = step_length

        # The following sets contain updated information for the current frame.
        self._active_actors = set()
        # coupled-carla: batched updates and caches, see synchronize_vehicle() and tick()
        self._pending, self._lights, self._kinds, self._tl_states = [], {}, {}, {}
        self.frame_lights = {}   # coupled-carla: light state of each CARLA-driven vehicle at this frame (logger)
        self._side_tls = {}      # coupled-carla: the light actors on the side connection (use_side)
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # Set traffic lights.
        self._tls = {}  # {landmark_id: traffic_ligth_actor}

        tmp_map = self.world.get_map()
        for landmark in tmp_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_ligth = self.world.get_traffic_light(landmark)
                if traffic_ligth is not None:
                    self._tls[landmark.id] = traffic_ligth
        # coupled-carla: in a world generated from OpenDRIVE (generate_opendrive_world) get_traffic_light()
        # finds none of the lights, so they were never driven by SUMO and stayed red. Each light actor knows
        # its OpenDRIVE signal id, which is the landmark id SUMO uses.
        for actor in self.world.get_actors().filter('traffic.traffic_light'):
            opendrive_id = actor.get_opendrive_id()
            if opendrive_id and opendrive_id not in self._tls:
                self._tls[opendrive_id] = actor

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)

    # This is a workaround to fix synchronization issues when other carla clients remove an actor in
    # carla without waiting for tick (e.g., running sumo co-simulation and manual control at the
    # same time)
    def get_actor_light_state(self, actor_id):
        """
        Accessor for carla actor light state.

        If the actor is not alive, returns None.
        """
        try:
            actor = self.get_actor(actor_id)
            return actor.get_light_state()
        except RuntimeError:
            return None

    def current_lights(self, actor_id):
        """
        coupled-carla: the light state of a SUMO-driven vehicle, as the bridge last set it; asked from the server
        only the first time (nothing else changes those lights).
        """
        lights = self._lights.get(actor_id)
        if lights is None:
            lights = self.get_actor_light_state(actor_id)
            if lights is None:
                return carla.VehicleLightState.NONE
            self._lights[actor_id] = lights
        return lights

    def queue(self, command):
        """
        coupled-carla: a carla.command sent with this tick's batch, just before the world ticks. On CARLA 1.0 two
        fire-and-forget calls in a row stall the next blocking one by ~40 ms (its rpclib lost TCP_NODELAY, so
        Nagle's algorithm waits for a delayed ACK); one batch per tick never does.
        """
        self._pending.append(command)

    def use_side(self, side_world):
        """
        coupled-carla: send traffic-light states over a second connection (see coupledcarla.side), where several
        in a row cannot stall the ticks. Fetches the light actors there once, now.
        """
        ids = [tl.id for tl in self._tls.values()]
        by_id = {a.id: a for a in side_world.get_actors(ids)} if ids else {}
        self._side_tls = {lid: by_id[tl.id] for lid, tl in self._tls.items() if tl.id in by_id}

    @property
    def traffic_light_ids(self):
        return set(self._tls.keys())

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        if landmark_id not in self._tls:
            return None
        return self._tls[landmark_id].state

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(True)
                # We set the traffic light to 'green' because 'off' state sets the traffic light to
                # 'red'.
                actor.set_state(carla.TrafficLightState.Green)

    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new actor.

            :param blueprint: blueprint of the actor to be spawned.
            :param transform: transform where the actor will be spawned.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        transform = carla.Transform(transform.location + carla.Location(0, 0, SPAWN_OFFSET_Z),
                                    transform.rotation)

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]
        response = self.client.apply_batch_sync(batch, False)[0]
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor.
        """
        actor = self.world.get_actor(actor_id)
        if actor is not None:
            return actor.destroy()
        return False

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param lights: new vehicle light state.
            :return: True if successfully updated. Otherwise, False.
        """
        # coupled-carla: queued and sent as one batch in tick(); one RPC per vehicle per tick was the
        # bottleneck (real-time factor fell to 0.2 with ~100 SUMO vehicles at 50 Hz).
        self._pending.append(carla.command.ApplyTransform(vehicle_id, transform))
        if lights is not None and self._lights.get(vehicle_id) != lights:
            self._lights[vehicle_id] = lights
            self._pending.append(carla.command.SetVehicleLightState(vehicle_id, carla.VehicleLightState(lights)))
        return True

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param landmark_id: id of the landmark to be updated.
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        if not landmark_id in self._tls:
            logging.warning('Landmark %s not found in carla', landmark_id)
            return False

        if self._tl_states.get(landmark_id) == state:   # coupled-carla: only send changes
            return True
        self._tl_states[landmark_id] = state
        traffic_light = self._side_tls.get(landmark_id)
        if traffic_light is None:
            traffic_light = self._tls[landmark_id]
        traffic_light.set_state(state)
        return True

    def tick(self):
        """
        Tick to carla simulation.
        """
        if self._pending:
            self.client.apply_batch(self._pending)
            self._pending = []
        self.tick_frame = self.world.tick()
        self.tick_wall = time.time()      # coupled-carla: when this frame's tick returned (station latency)
        self.frame_lights = {}

        # Update data structures for the current frame. coupled-carla: from the local snapshot, asking the
        # server only about actors not seen before (get_actors() every tick was another bottleneck).
        ids = {a.id for a in self.world.get_snapshot()}
        for new_id in ids - self._kinds.keys():
            actor = self.world.get_actor(new_id)
            self._kinds[new_id] = actor is not None and actor.type_id.startswith('vehicle.')
        for gone in self._kinds.keys() - ids:
            del self._kinds[gone]
            self._lights.pop(gone, None)
        current_actors = {i for i in ids if self._kinds.get(i)}
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

    def close(self):
        """
        Closes carla client.
        """
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(False)
