"""The SUMO driver: a participant's car on autopilot driven by SUMO, mirrored into CARLA like SUMO's own traffic.

The test autopilot (CARLA's BasicAgent on random lanes, inputs.Autopilot) steers by CARLA's waypoints with its own
controller and checks only vehicles and (SUMO's) lights. On the OSM-derived junctions it cut diagonally across
(BasicAgent picks a random junction path and its local planner cuts the corner of the first waypoint it has passed),
ran over islands into scenery (a YIELD sign on the Eindhoven rush-hour clip) and stood there until the runner put it
back on a lane; SUMO's vehicles knew it only through moveToXY (no route, so no right of way at junctions: a bus
crossed close in front). Driven by SUMO instead it follows SUMO's lanes and junction paths with the same car-following
(EIDM), junction, signal and lane-change rules as the traffic (drivers.py), yields to the CARLA walkers like it
(yielding.py), and the bridge moves its CARLA car with physics on (bridge/animate.py), so its wheels turn and steer.

Route: participant `route: [[lat, lon], ...]` (via points, in order: SUMO's shortest path through the nearest edge
of each) or `destination: {lat, lon}`; without them a random destination 600-1500 m away, and when a route nears its
end a new random destination (the car never arrives, so it never leaves CARLA). `hold` (film.py): the car is put into
SUMO only when the hold ends, at its spawn lane (the traffic built up meanwhile does not queue behind a parked car);
until then the CARLA car waits below the road.

The CARLA car does not copy SUMO's pose: it drives a smoothed copy of it (bridge/animate.PathSmoother, the gentle
participant limits: 3 m/s^2 lateral, a slow steering rate, up to 8 m off SUMO's path for a moment), so a kink or a
sideways jump in a SUMO junction lane (Eindhoven rush hour: 8 m across in an 8.8 m junction, 22 m/s^2) is driven as
a curve; the nets avoid such lanes too (netsmooth.py).

Keyboard and wheel participants are unchanged. simulation.autopilot: sumo (default) | carla picks this or BasicAgent
for `input: autopilot`; a participant's own `autopilot:` wins.
"""
import math
import random

import carla
import traci

from . import levels

STUCK_S = 10.0      # standing this long where its route cannot go on from its lane: take another road


def sumo_xy(offset, x, y):
    """CARLA (x, y) -> SUMO net (x, y) (bridge_helper: CARLA = SUMO - offset, y flipped)."""
    return x + offset[0], offset[1] - y


def lane_heading(lane, pos):
    """SUMO heading (deg, CARLA convention: 0 = +x, clockwise positive as y points south) of a lane at pos."""
    shape = lane.getShape()
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if acc + seg >= pos or (x1, y1) == shape[-1]:
            return math.degrees(math.atan2(-(y1 - y0), x1 - x0))
        acc += seg
    return 0.0


def pick_lane(net, x, y, yaw=None, vclass="passenger", radius=25.0, z=None):
    """(lane, position) of the SUMO lane nearest to SUMO (x, y) that `vclass` may use and (yaw given) runs within
    60 deg of the CARLA yaw, or None. z given (the car's height, or the level of the route so far): lanes higher or
    lower than the nearest-in-height one cost their extra height as distance (levels.py), so on a stacked road the
    car's own level wins; nets without heights rank as before."""
    cands = []
    for lane, dist in net.getNeighboringLanes(x, y, radius):
        if not lane.allows(vclass) or lane.getEdge().getFunction() == "internal":
            continue
        pos, _ = lane.getClosestLanePosAndDist((x, y))
        diff = abs((lane_heading(lane, pos) - yaw + 180.0) % 360.0 - 180.0) if yaw is not None else 0.0
        if diff > 60.0:
            continue
        cands.append((dist + diff / 30.0, lane, pos, levels.lane_z(lane, pos)))
    if not cands:
        return None
    keys = levels.level_keys([(c[0], c[3]) for c in cands], z)
    best = min(range(len(cands)), key=lambda i: keys[i])
    return cands[best][1], cands[best][2]


def plan_route(net, start_edge, vias, vclass="passenger"):
    """Edge ids from start_edge through the via edges in order (SUMO's shortest paths), or None when one leg has
    no path."""
    edges, here = [start_edge.getID()], start_edge
    for via in vias:
        if via.getID() == here.getID():
            continue
        path, _ = net.getShortestPath(here, via, vClass=vclass)
        if not path:
            return None
        edges += [e.getID() for e in path[1:]]
        here = via
    return edges


def drivable_lane(net, x, y, rng, yaw=None, vclass="passenger", radius=80.0, z=None):
    """(lane, position, route) of the nearest lane within radius of SUMO (x, y) from which a random destination
    can be reached, or None: a spawn lane can be a dead end (Salamanca's Plaza Mayor: a 16 m edge with no way
    out, so the car arrived at once and stood still). Lanes within 60 deg of yaw first, then any heading."""
    cands = []
    for lane, dist in net.getNeighboringLanes(x, y, radius):
        if not lane.allows(vclass) or lane.getEdge().getFunction() == "internal":
            continue
        pos, _ = lane.getClosestLanePosAndDist((x, y))
        diff = abs((lane_heading(lane, pos) - yaw + 180.0) % 360.0 - 180.0) if yaw is not None else 0.0
        cands.append((diff > 60.0, dist + (abs(levels.lane_z(lane, pos) - z) if z is not None else 0.0), lane, pos))
    seen = set()
    for _, _, lane, pos in sorted(cands, key=lambda c: (c[0], c[1])):
        edge = lane.getEdge()
        if edge.getID() in seen:
            continue
        seen.add(edge.getID())
        route = random_destination(net, edge, rng, vclass, tries=12)
        if route:
            return lane, pos, route
    return None


def has_u_turn(path):
    """Whether a path (sumolib edges) turns back onto the road it came on (an edge followed by the edge between the
    same two junctions the other way): at a dead end SUMO's turnaround lane swings round beyond the road's surface
    (Zurich HB twin: the participant's car turned through a building for 4 s at -231 -> 231)."""
    try:
        return any(b.getFromNode() == a.getToNode() and b.getToNode() == a.getFromNode()
                   for a, b in zip(path, path[1:]))
    except AttributeError:          # edges without their junctions (tests' stand-ins): none found
        return False


def random_destination(net, start_edge, rng, vclass="passenger", lo=600.0, hi=1500.0, tries=60):
    """A route (edge ids) from start_edge to a random edge lo-hi m away (straight line) without a U-turn, or None
    (with a U-turn only when no destination can be reached without one)."""
    sx, sy = start_edge.getToNode().getCoord()
    edges = [e for e in net.getEdges() if e.allows(vclass) and e.getFunction() != "internal"]
    far = [e for e in edges if lo <= math.hypot(e.getToNode().getCoord()[0] - sx,
                                               e.getToNode().getCoord()[1] - sy) <= hi] or edges
    fallback = None
    for _ in range(tries):
        dest = rng.choice(far)
        path, _ = net.getShortestPath(start_edge, dest, vClass=vclass)
        if path and len(path) > 2:
            if not has_u_turn(path):
                return [e.getID() for e in path]
            fallback = fallback or [e.getID() for e in path]
    return fallback


class SumoDriver:
    """inputs.make("sumo", vehicle): control() returns None (the bridge moves the car); attach() before the first
    bridge tick."""
    needs_display = False

    def __init__(self, vehicle, seed=1):
        self.vehicle = vehicle
        self.light, self.quit, self.stuck = None, False, False
        self.sid = None
        self.rng = random.Random(seed)
        self.hold_until = None
        self.inserted = False
        self.events = []        # (name, detail) for the trial log
        self._planned = None

    def hold(self, until):
        """film.py: stand still (out of SUMO) until simulation time `until`."""
        self.hold_until = float(until)
        if self.inserted and traci.simulation.getTime() < self.hold_until:   # added before the hold was known
            try:
                traci.vehicle.remove(self.sid)
            except traci.TraCIException:
                pass
            self.inserted = False
            self._hide()

    def _hide(self):
        tf = self.vehicle.get_transform()
        self.vehicle.set_simulate_physics(False)
        self.vehicle.set_transform(carla.Transform(tf.location - carla.Location(z=30.0), tf.rotation))

    def attach(self, sync, net, geo, p, offset, log=print):
        """Plans the route from the car's spawn lane; adds the SUMO vehicle now or when the hold ends."""
        self.sync, self.net, self.offset, self.log = sync, net, offset, log
        self.sid = f"ego_{p['role']}"
        tf = self.vehicle.get_transform()
        x, y = sumo_xy(offset, tf.location.x, tf.location.y)
        z = tf.location.z         # SUMO's heights are CARLA's (same OpenDRIVE); a net without heights ranks as before
        got = pick_lane(net, x, y, tf.rotation.yaw, z=z) or pick_lane(net, x, y, tf.rotation.yaw, radius=60.0, z=z)
        if got is None:
            raise RuntimeError(f"{p['role']}: no SUMO lane near the spawn for the SUMO driver")
        self.lane, self.pos = got
        vias = []
        for lat, lon in (p.get("route") or []) + ([[p["destination"]["lat"], p["destination"]["lon"]]]
                                                   if p.get("destination") else []):
            cx, cy = geo.to_carla(lat, lon)
            # a via point has no height: it keeps the level of the route so far (on a stacked road, the street
            # under the expressway stays the street), and the next point goes on from this lane's height
            near = pick_lane(net, *sumo_xy(offset, cx, cy), radius=40.0, z=z)
            if near is None:
                log(f"{p['role']}: route point {lat:.5f},{lon:.5f} has no lane within 40 m; skipped")
                continue
            vias.append(near[0].getEdge())
            z = levels.lane_z(near[0], near[1])
        start = self.lane.getEdge()
        route = plan_route(net, start, vias) if vias else None
        if vias and route is None:
            log(f"{p['role']}: no SUMO route through the given points; a random destination instead")
        self._planned = bool(route)
        route = route or random_destination(net, start, self.rng)
        if not route:           # a dead end: start from the nearest lane that leads somewhere
            alt = drivable_lane(net, x, y, self.rng, tf.rotation.yaw, z=z)
            if alt is not None:
                lane, pos, route = alt
                log(f"{p['role']}: no way on from lane {self.lane.getID()}; starting on {lane.getID()} instead")
                self.lane, self.pos = lane, pos
                self._place_on_lane(offset)
        route = route or [self.lane.getEdge().getID()]
        self.route = route
        self.vtype = self._vtype()
        length = sum(net.getEdge(e).getLength() for e in route)
        log(f"{p['role']}: SUMO driver, route of {len(route)} edges ({length:.0f} m"
            f"{', planned' if self._planned else ', random destination'}), from lane {self.lane.getID()}")
        sync.adopt(self.sid, self.vehicle.id)      # no SUMO twin for this car, now or while it waits
        if getattr(sync, "pipelined", False):      # SUMO steps less often or in a thread: drive at its steps
            sync.hooks.append(lambda: self._drive(self.vehicle))
        if self.hold_until is None:
            self._insert()
        else:   # out of the way until the hold ends (below the road, no physics)
            self._hide()

    def _place_on_lane(self, offset):
        """Moves the CARLA car to its SUMO start (self.lane at self.pos), facing along the lane."""
        sx, sy = self.lane.getShape()[0]
        try:
            import sumolib.geomhelper as gh
            sx, sy = gh.positionAtShapeOffset(self.lane.getShape(), self.pos)
        except ImportError:
            pass
        lz = levels.lane_z(self.lane, self.pos)     # SUMO's heights are CARLA's; 0 for a net without heights
        loc = carla.Location(x=sx - offset[0], y=offset[1] - sy,
                             z=(lz if lz else self.vehicle.get_transform().location.z) + 0.5)
        self.vehicle.set_transform(carla.Transform(loc, carla.Rotation(yaw=lane_heading(self.lane, self.pos))))

    def _vtype(self):
        vid = "ego:" + self.vehicle.type_id
        if vid in traci.vehicletype.getIDList():
            return vid
        cars = [t for t in traci.vehicletype.getIDList()
                if t.startswith("vehicle.") and traci.vehicletype.getVehicleClass(t) == "passenger"]
        traci.vehicletype.copy(cars[0] if cars else "DEFAULT_VEHTYPE", vid)
        ext = self.vehicle.bounding_box.extent
        traci.vehicletype.setLength(vid, 2.0 * ext.x)
        traci.vehicletype.setWidth(vid, min(2.1, 2.0 * ext.y))
        traci.vehicletype.setSpeedFactor(vid, 1.0)
        traci.vehicletype.setSpeedDeviation(vid, 0.0)
        return vid

    def _insert(self):
        rid = f"{self.sid}_r{self.rng.randrange(10 ** 6)}"
        traci.route.add(rid, self.route)
        traci.vehicle.add(self.sid, rid, typeID=self.vtype, depart="now", departLane=str(self.lane.getIndex()),
                          departPos=f"{max(0.0, min(self.pos, self.lane.getLength() - 0.5)):.1f}",
                          departSpeed="0")
        self.inserted = True
        self.events.append(("sumo_driver", f"{self.sid} into SUMO on {self.lane.getID()}"))

    def _unblock(self, route, idx, now):
        """Stuck for STUCK_S at the end of a lane from which its route's next edge cannot be reached (it waited for
        a lane change that the queue beside it never allowed, Eindhoven rush hour: 100 s at a green light): it
        takes a road that this lane leads to and a new destination from there, as a driver who missed the turn."""
        try:
            speed = traci.vehicle.getSpeed(self.sid)
            lane_id = traci.vehicle.getLaneID(self.sid)
        except traci.TraCIException:
            return
        if speed > 0.1:
            self._still_since = None
            return
        self._still_since = getattr(self, "_still_since", None) or now
        if now - self._still_since < STUCK_S or lane_id.startswith(":") or idx < 0 or idx + 1 >= len(route):
            return
        try:
            lane = self.net.getLane(lane_id)
        except KeyError:
            return
        reach = [c.getTo() for c in lane.getOutgoing() if c.getTo().allows("passenger")]
        if not reach or route[idx + 1] in {e.getID() for e in reach}:
            return
        nxt = self.rng.choice(reach)
        tail = random_destination(self.net, nxt, self.rng) or [nxt.getID()]
        try:
            traci.vehicle.setRoute(self.sid, [route[idx]] + tail)
            self.events.append(("sumo_driver", f"{self.sid} rerouted via {nxt.getID()}: could not change lanes "
                                               f"for {now - self._still_since:.0f} s"))
            self._still_since = None
        except traci.TraCIException:
            pass

    def control(self, vehicle, events=()):
        if self.sid is None or getattr(getattr(self, "sync", None), "pipelined", False):
            return None     # the pipelined bridge calls _drive at each SUMO step (a TraCI call here would wait)
        return self._drive(vehicle)

    def _drive(self, vehicle):
        """Inserts, re-inserts, unblocks and extends the SUMO vehicle's route (TraCI); once per SUMO step."""
        if self.sid is None:
            return None
        now = traci.simulation.getTime()
        if not self.inserted:
            if self.hold_until is not None and now >= self.hold_until:
                self._insert()
            return None
        if self.sid in self.sync.arrived_adopted:          # arrived anyway (or was removed): again from here
            tf = vehicle.get_transform()
            got = pick_lane(self.net, *sumo_xy(self.offset, tf.location.x, tf.location.y), tf.rotation.yaw,
                            z=tf.location.z)
            if got:
                self.lane, self.pos = got
                self.route = random_destination(self.net, self.lane.getEdge(), self.rng) or [self.lane.getEdge().getID()]
                self._insert()
            return None
        try:
            route = traci.vehicle.getRoute(self.sid)
            idx = traci.vehicle.getRouteIndex(self.sid)
        except traci.TraCIException:
            return None                                     # not inserted yet (its lane was taken)
        self._unblock(route, idx, now)
        if idx >= 0 and len(route) - idx <= 3:                # near the end: drive on to a new destination
            nxt = random_destination(self.net, self.net.getEdge(route[-1]), self.rng)
            if nxt:
                try:
                    traci.vehicle.setRoute(self.sid, list(route[idx:]) + nxt[1:])
                    self.events.append(("sumo_driver", f"{self.sid} new destination {nxt[-1]}"))
                except traci.TraCIException:
                    pass
        return None
