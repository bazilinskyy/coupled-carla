"""Traffic lights from SUMO, drawn in CARLA.

CARLA's traffic-light actors in a world built from OpenDRIVE (generate_opendrive_world) are not usable: they
have no OpenDRIVE id, so the bridge cannot link them to SUMO, and set_state() does not change them; they stay
red. SUMO is therefore the only source of truth for signals here:
- each signalled lane's light is drawn in the world at its stop line (a lamp on a short pole on the right),
  coloured from SUMO, for lights near a participant; debug drawings show in every camera;
- next_light() gives a participant's next light (id, distance, state) from its SUMO twin, for the autopilot,
  the log and the HUD.
"""
import math

import carla
import traci

COLOURS = {"G": carla.Color(0, 255, 0), "g": carla.Color(0, 255, 0), "y": carla.Color(255, 190, 0),
           "Y": carla.Color(255, 190, 0), "r": carla.Color(255, 0, 0), "R": carla.Color(255, 0, 0),
           "u": carla.Color(255, 120, 0), "s": carla.Color(255, 0, 0), "o": carla.Color(80, 80, 80),
           "O": carla.Color(80, 80, 80)}
RANK = {"G": 5, "g": 4, "y": 3, "Y": 3, "u": 2, "r": 1, "R": 1, "s": 1, "o": 0, "O": 0}


class Signals:
    def __init__(self, world, net, offset, radius=120.0, every=5, step=0.02, debug=None):
        """net: the sumolib net SUMO runs (map.net.xml); offset: the bridge's SUMO->CARLA shift; debug: where to
        draw (the runner passes the side connection's, see side.py), else world.debug."""
        self.debug = world.debug if debug is None else debug
        cmap = world.get_map()
        self.radius, self.every = radius, every
        self.life = every * step * 1.6
        self.heads = []   # (tls_id, [link indices], carla Location of the lamp)
        self.lanes = {}   # SUMO lane id -> (tls_id, [link indices], lane length)
        for tls in traci.trafficlight.getIDList():
            by_lane = {}
            for idx, links in enumerate(traci.trafficlight.getControlledLinks(tls)):
                for in_lane, _out, _via in links:
                    by_lane.setdefault(in_lane, []).append(idx)
            for lane_id, idxs in by_lane.items():
                try:
                    lane = net.getLane(lane_id)
                except KeyError:
                    continue
                self.lanes[lane_id] = (tls, idxs, lane.getLength())
                shape = lane.getShape()
                (x0, y0), (x1, y1) = shape[-2], shape[-1]
                d = math.hypot(x1 - x0, y1 - y0) or 1.0
                rx, ry = (y1 - y0) / d, -(x1 - x0) / d          # unit vector to the right of travel (SUMO)
                side = lane.getWidth() / 2 + 0.6
                sx, sy = x1 + rx * side, y1 + ry * side
                loc = carla.Location(x=sx - offset[0], y=-(sy - offset[1]))
                wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
                loc.z = wp.transform.location.z if wp is not None else 0.0      # the road's height (elevation)
                self.heads.append((tls, idxs, loc))
        self._tick = 0

    def draw(self, centres):
        self._tick += 1
        if self._tick % self.every or not centres:
            return
        states = {}
        for tls, idxs, loc in self.heads:
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            if tls not in states:
                states[tls] = traci.trafficlight.getRedYellowGreenState(tls)
            st = max((states[tls][i] for i in idxs if i < len(states[tls])), key=lambda s: RANK.get(s, 0),
                     default="o")
            colour = COLOURS.get(st, COLOURS["o"])
            self.debug.draw_line(loc, loc + carla.Location(z=3.0), thickness=0.08,
                                 color=carla.Color(40, 40, 40), life_time=self.life)
            self.debug.draw_point(loc + carla.Location(z=3.3), size=0.25, color=colour, life_time=self.life)


    def next_light(self, sumo_vehicle_id):
        """(tls id, distance m, state char) of the light at the end of the vehicle's SUMO lane, or None.

        SUMO's getNextTLS() returns nothing for vehicles placed with moveToXY (the participants' twins), so
        this looks at the lane the twin is on: the light that ends it, if any.
        """
        try:
            lane = traci.vehicle.getLaneID(sumo_vehicle_id)
            hit = self.lanes.get(lane)
            if hit is None:
                return None
            tls, idxs, length = hit
            dist = max(0.0, length - traci.vehicle.getLanePosition(sumo_vehicle_id))
            state = traci.trafficlight.getRedYellowGreenState(tls)
        except traci.TraCIException:
            return None
        st = max((state[i] for i in idxs if i < len(state)), key=lambda c: RANK.get(c, 0), default="o")
        return tls, dist, st
