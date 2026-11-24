#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the sumo simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import collections
import enum
import logging
import math
import os
import threading

import carla  # pylint: disable=import-error
import sumolib  # pylint: disable=import-error
import traci  # pylint: disable=import-error

from .constants import INVALID_ACTOR_ID
from coupledcarla.lanesnap import ASK  # pylint: disable=wrong-import-order

import lxml.etree as ET  # pylint: disable=import-error

# ==================================================================================================
# -- sumo definitions ------------------------------------------------------------------------------
# ==================================================================================================


# https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
class SumoSignalState(object):
    """
    SumoSignalState contains the different traffic light states.
    """
    RED = 'r'
    YELLOW = 'y'
    GREEN = 'G'
    GREEN_WITHOUT_PRIORITY = 'g'
    GREEN_RIGHT_TURN = 's'
    RED_YELLOW = 'u'
    OFF_BLINKING = 'o'
    OFF = 'O'


# https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html
class SumoVehSignal(object):
    """
    SumoVehSignal contains the different sumo vehicle signals.
    """
    BLINKER_RIGHT = 1 << 0
    BLINKER_LEFT = 1 << 1
    BLINKER_EMERGENCY = 1 << 2
    BRAKELIGHT = 1 << 3
    FRONTLIGHT = 1 << 4
    FOGLIGHT = 1 << 5
    HIGHBEAM = 1 << 6
    BACKDRIVE = 1 << 7
    WIPER = 1 << 8
    DOOR_OPEN_LEFT = 1 << 9
    DOOR_OPEN_RIGHT = 1 << 10
    EMERGENCY_BLUE = 1 << 11
    EMERGENCY_RED = 1 << 12
    EMERGENCY_YELLOW = 1 << 13


# https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
class SumoActorClass(enum.Enum):
    """
    SumoActorClass enumerates the different sumo actor classes.
    """
    IGNORING = "ignoring"
    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PEDESTRIAN = "pedestrian"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    EVEHICLE = "evehicle"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"


# coupled-carla: speed (m/s) too, for the CARLA side (a SUMO vehicle in CARLA has physics off and reads 0 m/s there)
SumoActor = collections.namedtuple('SumoActor', 'type_id vclass transform signals extent color speed',
                                   defaults=(0.0,))

# ==================================================================================================
# -- sumo traffic lights ---------------------------------------------------------------------------
# ==================================================================================================


class SumoTLLogic(object):
    """
    SumoTLLogic holds the data relative to a traffic light in sumo.
    """
    def __init__(self, tlid, states, parameters):
        self.tlid = tlid
        self.states = states

        self._landmark2link = {}
        self._link2landmark = {}
        for link_index, landmark_id in parameters.items():
            # Link index information is added in the parameter as 'linkSignalID:x'
            link_index = int(link_index.split(':')[1])

            if landmark_id not in self._landmark2link:
                self._landmark2link[landmark_id] = []
            self._landmark2link[landmark_id].append((tlid, link_index))
            self._link2landmark[(tlid, link_index)] = landmark_id

    def get_number_signals(self):
        """
        Returns number of internal signals of the traffic light.
        """
        if len(self.states) > 0:
            return len(self.states[0])
        return 0

    def get_all_signals(self):
        """
        Returns all the signals of the traffic light.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return [(self.tlid, i) for i in range(self.get_number_signals())]

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with this traffic light.
        """
        return self._landmark2link.keys()

    def get_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return self._landmark2link.get(landmark_id, [])


class SumoTLManager(object):
    """
    SumoTLManager is responsible for the management of the sumo traffic lights (i.e., keeps control
    of the current program, phase, ...)
    """
    def __init__(self):
        self._tls = {}  # {tlid: {program_id: SumoTLLogic}
        self._current_program = {}  # {tlid: program_id}
        self._current_phase = {}  # {tlid: index_phase}

        for tlid in traci.trafficlight.getIDList():
            self.subscribe(tlid)

            self._tls[tlid] = {}
            for tllogic in traci.trafficlight.getAllProgramLogics(tlid):
                states = [phase.state for phase in tllogic.getPhases()]
                parameters = tllogic.getParameters()
                tl = SumoTLLogic(tlid, states, parameters)
                self._tls[tlid][tllogic.programID] = tl

            # Get current status of the traffic lights.
            self._current_program[tlid] = traci.trafficlight.getProgram(tlid)
            self._current_phase[tlid] = traci.trafficlight.getPhase(tlid)

        self._off = False
        # coupled-carla: the light ids never change during a run, and the landmark -> signals index only when a
        # light switches program. Rebuilding both for every landmark every tick (each landmark scanned all ~140
        # lights of the twin) cost 7 ms per tick.
        self._tl_ids = list(self._tls)
        self._index = None    # {landmark_id: [(tlid, link_index)]} for the current programs

    @staticmethod
    def subscribe(tlid):
        """
        Subscribe the given traffic ligth to the following variables:

            * Current program.
            * Current phase.
        """
        traci.trafficlight.subscribe(tlid, [
            traci.constants.TL_CURRENT_PROGRAM,
            traci.constants.TL_CURRENT_PHASE,
        ])

    @staticmethod
    def unsubscribe(tlid):
        """
        Unsubscribe the given traffic ligth from receiving updated information each step.
        """
        traci.trafficlight.unsubscribe(tlid)

    def get_all_signals(self):
        """
        Returns all the traffic light signals.
        """
        signals = set()
        for tlid, program_id in self._current_program.items():
            signals.update(self._tls[tlid][program_id].get_all_signals())
        return signals

    def _landmark_index(self):
        # coupled-carla: built once per set of current programs (see __init__)
        if self._index is None:
            index = {}
            for tlid, program_id in self._current_program.items():
                for landmark_id, signals in self._tls[tlid][program_id]._landmark2link.items():
                    index.setdefault(landmark_id, set()).update(signals)
            self._index = index
        return self._index

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with a traffic light in the simulation.
        """
        return set(self._landmark_index())

    def get_all_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return set(self._landmark_index().get(landmark_id, ()))

    def get_state(self, landmark_id):
        """
        Returns the traffic light state of the signals associated with the given landmark.
        """
        states = set()
        for tlid, link_index in self._landmark_index().get(landmark_id, ()):
            current_program = self._current_program[tlid]
            current_phase = self._current_phase[tlid]

            tl = self._tls[tlid][current_program]
            states.update(tl.states[current_phase][link_index])

        if len(states) == 1:
            return states.pop()
        elif len(states) > 1:
            logging.warning('Landmark %s is associated with signals with different states',
                            landmark_id)
            return SumoSignalState.RED
        else:
            return None

    def set_state(self, landmark_id, state):
        """
        Updates the state of all the signals associated with the given landmark.
        """
        for tlid, link_index in self.get_all_associated_signals(landmark_id):
            traci.trafficlight.setLinkState(tlid, link_index, state)
        return True

    def switch_off(self):
        """
        Switch off all traffic lights.
        """
        for tlid, link_index in self.get_all_signals():
            traci.trafficlight.setLinkState(tlid, link_index, SumoSignalState.OFF)
        self._off = True

    def set_near(self, tl_ids):
        """coupled-carla (simulation.sumo_near): only these lights are subscribed and read each step (the bridge
        passes those near the participants); the others keep their last state."""
        tl_ids = [t for t in sorted(tl_ids) if t in self._tls]
        old, new = set(self._tl_ids), set(tl_ids)
        for tlid in sorted(old - new):
            self.unsubscribe(tlid)
        for tlid in sorted(new - old):
            self.subscribe(tlid)
        self._tl_ids = tl_ids

    def tick(self):
        """
        Tick to traffic light manager
        """
        if self._off is False:
            for tl_id in self._tl_ids:     # coupled-carla: was traci.trafficlight.getIDList(), a round trip
                results = traci.trafficlight.getSubscriptionResults(tl_id)
                if traci.constants.TL_CURRENT_PROGRAM not in results:   # coupled-carla: subscribed this step
                    continue
                current_program = results[traci.constants.TL_CURRENT_PROGRAM]
                current_phase = results[traci.constants.TL_CURRENT_PHASE]

                if current_program != 'online':
                    if self._current_program.get(tl_id) != current_program:
                        self._index = None
                    self._current_program[tl_id] = current_program
                    self._current_phase[tl_id] = current_phase


# ==================================================================================================
# -- sumo simulation -------------------------------------------------------------------------------
# ==================================================================================================

def _get_sumo_net(cfg_file):
    """
    Returns sumo net.

    This method reads the sumo configuration file and retrieve the sumo net filename to create the
    net.
    """
    cfg_file = os.path.join(os.getcwd(), cfg_file)

    tree = ET.parse(cfg_file)
    tag = tree.find('.//net-file')
    if tag is None:
        return None

    net_file = os.path.join(os.path.dirname(cfg_file), tag.get('value'))
    logging.debug('Reading net file: %s', net_file)

    sumo_net = sumolib.net.readNet(net_file)
    return sumo_net

def _sumo_args():
    from coupledcarla.drivers import SUMO_ARGS  # pylint: disable=import-outside-toplevel
    return SUMO_ARGS


LANE_SNAP_M = 0.6   # coupled-carla: farthest a CARLA-driven vehicle may sit from its SUMO lane's centre (see below)


class SumoSimulation(object):
    """
    SumoSimulation is responsible for the management of the sumo simulation.
    """
    def __init__(self, cfg_file, step_length, host=None, port=None, sumo_gui=False, client_order=1):
        if sumo_gui is True:
            sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            sumo_binary = sumolib.checkBinary('sumo')

        if host is None or port is None:
            logging.info('Starting new sumo server...')
            if sumo_gui is True:
                logging.info('Remember to press the play button to start the simulation')

            traci.start([sumo_binary,
                '--configuration-file', cfg_file,
                '--step-length', str(step_length),
                *_sumo_args(),          # coupled-carla: sublane, collisions, impatience (drivers.SUMO_ARGS)
                # coupled-carla: keep SUMO's own messages next to the trial's SUMO files
                '--log', os.path.join(os.path.dirname(os.path.abspath(cfg_file)), 'sumo.log')
            ],
                # coupled-carla: COUPLED_SUMO_TRACE=1 records every TraCI call (replay a SUMO crash without CARLA)
                traceFile=(os.path.join(os.path.dirname(os.path.abspath(cfg_file)), 'traci_trace.py')
                           if os.environ.get('COUPLED_SUMO_TRACE') else None),
                traceGetters=False)

        else:
            logging.info('Connection to sumo server. Host: %s Port: %s', host, port)
            traci.init(host=host, port=port)

        traci.setOrder(client_order)

        # Retrieving net from configuration file.
        self.net = _get_sumo_net(cfg_file)
        try:      # coupled-carla: the net file, for the lane shapes ahead of a vehicle (bridge/animate.LaneShapes)
            self.net_file = os.path.join(os.path.dirname(os.path.abspath(cfg_file)),
                                         ET.parse(cfg_file).find('.//net-file').get('value'))
        except (AttributeError, ET.ParseError, OSError):
            self.net_file = None
        # coupled-carla: convertRoad + convert2D for the lane snap below, in Python (coupledcarla/lanesnap.py)
        self.lane_index = None
        try:
            from coupledcarla.lanesnap import LaneIndex  # pylint: disable=import-outside-toplevel
            net_file = ET.parse(cfg_file).find('.//net-file').get('value')
            self.lane_index = LaneIndex(os.path.join(os.path.dirname(os.path.abspath(cfg_file)), net_file))
        except Exception as err:  # pylint: disable=broad-except
            logging.warning('lane index not built (%s); the lane snap asks SUMO', err)

        # To keep track of the vehicle classes for which a route has been generated in sumo.
        self._routes = set()

        # Variable to asign an id to new added actors.
        self._sequential_id = 0

        # Structures to keep track of the spawned and destroyed vehicles at each time step.
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # Traffic light manager.
        self.traffic_light_manager = SumoTLManager()

        # coupled-carla: the pipelined bridge (bridge/pipeline.py): a snapshot get_actor() answers from, the
        # thread that runs a SUMO step while CARLA ticks, and the participants' focus points (context subscriptions)
        self.frozen = None
        self._stepper = None
        self._step_error = None
        self._gate_on = False
        self._focus = {}
        self._off_net = set()

    # -- coupled-carla: SUMO stepped in a thread (simulation.sumo_thread) --------------------------------------------

    # every Connection method that writes the shared outgoing buffer (self._string), sends, or reads subscription
    # results: a call from another thread must not begin packing its command while the step's thread is mid-message
    _GATED = ("_sendCmd", "_sendExact", "_subscribe", "_subscribeContext", "_addSubscriptionFilter",
              "_getSubscriptionResults", "simulationStep", "load", "setOrder", "close")

    def _gate(self):
        """Makes every TraCI call wait for a running step first, unless it comes from the step's own thread (the
        connection is not thread-safe, and the call then sees the step's results, as without the thread)."""
        if self._gate_on:
            return
        conn = traci.getConnection()
        sim = self

        def gated(fn):
            def call(*args, **kwargs):
                t = sim._stepper
                if t is not None and threading.current_thread() is not t:
                    sim.wait_step()
                return fn(*args, **kwargs)
            return call
        for name in self._GATED:
            if hasattr(conn, name):
                setattr(conn, name, gated(getattr(conn, name)))
        self._gate_on = True

    def start_step(self):
        """Runs tick() in a thread; wait_step() joins it."""
        self._gate()

        def run():
            try:
                self.tick()
            except BaseException as err:  # noqa: BLE001 - raised again in wait_step
                self._step_error = err
        self._stepper = threading.Thread(target=run, name="sumo-step", daemon=True)
        self._stepper.start()

    def wait_step(self):
        """Waits for the step start_step() began (if any); raises what it raised."""
        t = getattr(self, "_stepper", None)
        if t is None:
            return
        t.join()
        self._stepper = None
        if self._step_error is not None:
            err, self._step_error = self._step_error, None
            raise err

    # -- coupled-carla: what is near the participants (simulation.sumo_near) -----------------------------------------

    def set_focus(self, points, radius):
        """One context subscription per point (SUMO x, y): every vehicle within radius, with its pose. A POI per
        point carries it; moved, added or removed as the points change."""
        for i, (x, y) in enumerate(points):
            pid = f"coupled_focus{i}"
            if pid not in self._focus:
                traci.poi.add(pid, x, y, (0, 0, 0, 0), layer=-1)
                traci.poi.subscribeContext(pid, traci.constants.CMD_GET_VEHICLE_VARIABLE, radius, self.POSE_VARS)
            elif self._focus[pid] != (x, y):
                traci.poi.setPosition(pid, x, y)
            self._focus[pid] = (x, y)
        for pid in [p for p in self._focus if int(p[len("coupled_focus"):]) >= len(points)]:
            traci.poi.unsubscribeContext(pid, traci.constants.CMD_GET_VEHICLE_VARIABLE, radius)
            traci.poi.remove(pid)
            del self._focus[pid]

    def near_poses(self):
        """{sumo id: (x, y)} of the vehicles the focus subscriptions saw at the last step."""
        out = {}
        for pid in self._focus:
            for vid, res in (traci.poi.getContextSubscriptionResults(pid) or {}).items():
                pos = res.get(traci.constants.VAR_POSITION3D)
                if pos is not None:
                    out[vid] = (pos[0], pos[1])
        return out

    @property
    def traffic_light_ids(self):
        return self.traffic_light_manager.get_all_landmarks()

    @staticmethod
    def subscribe(actor_id):
        """
        Subscribe the given actor to the following variables:

            * Type.
            * Vehicle class.
            * Color.
            * Length, Width, Height.
            * Position3D (i.e., x, y, z).
            * Angle, Slope.
            * Speed.
            * Lateral speed.
            * Signals.
        """
        traci.vehicle.subscribe(actor_id, [
            traci.constants.VAR_TYPE, traci.constants.VAR_VEHICLECLASS, traci.constants.VAR_COLOR,
            traci.constants.VAR_LENGTH, traci.constants.VAR_WIDTH, traci.constants.VAR_HEIGHT,
            traci.constants.VAR_POSITION3D, traci.constants.VAR_ANGLE, traci.constants.VAR_SLOPE,
            traci.constants.VAR_SPEED, traci.constants.VAR_SPEED_LAT, traci.constants.VAR_SIGNALS,
            # coupled-carla: where on which lane, for the smoothed path's look-ahead (animate.path_point)
            traci.constants.VAR_LANE_ID, traci.constants.VAR_LANEPOSITION, traci.constants.VAR_LANEPOSITION_LAT
        ])

    @staticmethod
    def get_lane_state(actor_id):
        """coupled-carla: (lane id, lane position, lateral position) of a vehicle subscribed with subscribe(), or
        None."""
        r = traci.vehicle.getSubscriptionResults(actor_id)
        lane = r.get(traci.constants.VAR_LANE_ID)
        if not lane:
            return None
        return lane, r.get(traci.constants.VAR_LANEPOSITION, 0.0), r.get(traci.constants.VAR_LANEPOSITION_LAT, 0.0)

    # coupled-carla: a SUMO vehicle that is not (yet) in CARLA is only needed for where it is (the level-of-detail
    # check); all twelve variables of every vehicle in the network cost 3 ms of parsing per tick in the twin.
    POSE_VARS = [traci.constants.VAR_POSITION3D, traci.constants.VAR_ANGLE, traci.constants.VAR_SLOPE,
                 traci.constants.VAR_LENGTH, traci.constants.VAR_SPEED]

    @staticmethod
    def subscribe_pose(actor_id):
        """
        Subscribe the given actor to its pose only (subscribe() adds the rest). SUMO adds variables to an existing
        subscription, so going back from subscribe() to this needs unsubscribe() first.
        """
        traci.vehicle.subscribe(actor_id, SumoSimulation.POSE_VARS)

    @staticmethod
    def get_pose(actor_id):
        """
        (transform, extent) of an actor subscribed with subscribe() or subscribe_pose(), as get_actor() gives
        them (extent carries the half length only), or None when SUMO sent nothing for it.
        """
        results = traci.vehicle.getSubscriptionResults(actor_id)
        try:
            x, y, z = results[traci.constants.VAR_POSITION3D]
            transform = carla.Transform(carla.Location(x, y, z),
                                        carla.Rotation(results[traci.constants.VAR_SLOPE],
                                                       results[traci.constants.VAR_ANGLE], 0.0))
            return transform, carla.Vector3D(results[traci.constants.VAR_LENGTH] / 2.0, 0.0, 0.0)
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def get_speed(actor_id):
        """coupled-carla: the speed (m/s) of a subscribed actor, 0 when SUMO sent none."""
        return traci.vehicle.getSubscriptionResults(actor_id).get(traci.constants.VAR_SPEED, 0.0)

    @staticmethod
    def unsubscribe(actor_id):
        """
        Unsubscribe the given actor from receiving updated information each step.
        """
        traci.vehicle.unsubscribe(actor_id)

    def get_net_offset(self):
        """
        Accessor for sumo net offset.
        """
        if self.net is None:
            return (0, 0)
        return self.net.getLocationOffset()

    def get_actor(self, actor_id):
        """
        Accessor for sumo actor.

        coupled-carla: while the pipelined bridge holds a snapshot (frozen: {sumo id: SumoActor} taken at the last
        SUMO-step boundary), the actor from it, or None when it has none; SUMO may be stepping meanwhile.
        """
        frozen = getattr(self, "frozen", None)
        if frozen is not None:
            return frozen.get(actor_id)
        return self.read_actor(actor_id)

    @staticmethod
    def read_actor(actor_id):
        """The actor from its subscription results (KeyError without them)."""
        results = traci.vehicle.getSubscriptionResults(actor_id)

        type_id = results[traci.constants.VAR_TYPE]
        vclass = SumoActorClass(results[traci.constants.VAR_VEHICLECLASS])
        color = results[traci.constants.VAR_COLOR]

        length = results[traci.constants.VAR_LENGTH]
        width = results[traci.constants.VAR_WIDTH]
        height = results[traci.constants.VAR_HEIGHT]

        location = list(results[traci.constants.VAR_POSITION3D])
        rotation = [results[traci.constants.VAR_SLOPE], results[traci.constants.VAR_ANGLE], 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = results[traci.constants.VAR_SIGNALS]
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)

        return SumoActor(type_id, vclass, transform, signals, extent, color,
                         results.get(traci.constants.VAR_SPEED, 0.0))

    def spawn_actor(self, type_id, color=None):
        """
        Spawns a new actor.

            :param type_id: vtype to be spawned.
            :param color: color attribute for this specific actor.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        actor_id = 'carla' + str(self._sequential_id)
        try:
            vclass = traci.vehicletype.getVehicleClass(type_id)
            if vclass not in self._routes:
                logging.debug('Creating route for %s vehicle class', vclass)
                allowed_edges = [e for e in self.net.getEdges() if e.allows(vclass)]
                if allowed_edges:
                    traci.route.add("carla_route_{}".format(vclass), [allowed_edges[0].getID()])
                    self._routes.add(vclass)
                else:
                    logging.error(
                        'Could not found a route for %s. No vehicle will be spawned in sumo',
                        type_id)
                    return INVALID_ACTOR_ID

            traci.vehicle.add(actor_id, 'carla_route_{}'.format(vclass), typeID=type_id)
        except traci.exceptions.TraCIException as error:
            logging.error('Spawn sumo actor failed: %s', error)
            return INVALID_ACTOR_ID

        if color is not None:
            color = color.split(',')
            traci.vehicle.setColor(actor_id, color)

        self._sequential_id += 1

        return actor_id

    @staticmethod
    def destroy_actor(actor_id):
        """
        Destroys the given actor.
        """
        traci.vehicle.remove(actor_id)

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        return self.traffic_light_manager.get_state(landmark_id)

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        self.traffic_light_manager.switch_off()

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param signals: new vehicle signals.
            :return: True if successfully updated. Otherwise, False.
        """
        loc_x, loc_y = transform.location.x, transform.location.y
        yaw = transform.rotation.yaw

        # coupled-carla: keep the vehicle inside a SUMO lane. Placed straddling two lanes (CARLA's junction paths are
        # not SUMO's internal lanes), the sublane model gives it a shadow lane, and SUMO 1.27/1.28 then segfaults in
        # MSVehicle::getBackPositionOnLane (a shadow further lane with no further lanes) when another vehicle looks
        # at it as a leader across a junction. Pull it to within LANE_SNAP_M of the lane centre.
        try:
            # coupled-carla: the same point from lanesnap, without SUMO's 1.6 ms scan of every lane
            centre = self.lane_index.nearest_centre(loc_x, loc_y) if self.lane_index is not None else ASK
            if centre is ASK:
                edge, pos, lane = traci.simulation.convertRoad(loc_x, loc_y, False, "passenger")
                centre = traci.simulation.convert2D(edge, pos, lane)
            elif centre is None:
                raise traci.exceptions.TraCIException("no lane")
            cx, cy = centre
            off = math.hypot(loc_x - cx, loc_y - cy)
            if off > LANE_SNAP_M:
                k = LANE_SNAP_M / off
                loc_x, loc_y = cx + (loc_x - cx) * k, cy + (loc_y - cy) * k
        except traci.exceptions.TraCIException:
            pass
        # coupled-carla: map onto a lane its class may use (keepRoute=0) first. keepRoute=2 ignores permissions and,
        # where a lane closed to cars is nearest (an internal bike or bus lane), leaves the car on no lane at all
        # while counting it as on the road: SUMO then segfaults on the next query or FCD output (Leuven).
        try:
            traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=0)
        except traci.exceptions.TraCIException:     # no lane for its class nearby (off the road): place it anyway
            try:
                traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=2)
            except traci.exceptions.TraCIException:   # coupled-carla: no lane at all (out of SUMO's area,
                off_net = self.__dict__.setdefault('_off_net', set())   # traffic.crop_radius_m): it stays
                if vehicle_id not in off_net:                              # where it was
                    off_net.add(vehicle_id)
                    logging.warning('CARLA car %s is off SUMO\'s roads (outside traffic.crop_radius_m?): its SUMO '
                                    'twin stays at the last place SUMO had for it', vehicle_id)
                return False
        if signals is not None:
            traci.vehicle.setSignals(vehicle_id, signals)
        return True

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param tl_id: id of the traffic light to be updated (logic id, link index).
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        self.traffic_light_manager.set_state(landmark_id, state)

    def tick(self):
        """
        Tick to sumo simulation.
        """
        traci.simulationStep()
        self.traffic_light_manager.tick()

        # Update data structures for the current frame.
        self.spawned_actors = set(traci.simulation.getDepartedIDList())
        self.destroyed_actors = set(traci.simulation.getArrivedIDList())

    @staticmethod
    def close():
        """
        Closes traci client.
        """
        traci.close()
