"""Traffic lights from SUMO, drawn in CARLA.

CARLA's traffic-light actors in a world built from OpenDRIVE (generate_opendrive_world) are not usable: they
have no OpenDRIVE id, so the bridge cannot link them to SUMO, and set_state() does not change them; they stay
red. SUMO is therefore the only source of truth for signals here:
- each approach's signal stands at its stop line, coloured from SUMO, for lights near a participant. Style "head"
  (simulation.traffic_lights.style): a signal head on a pole at the kerb (CARLA 0.9.16's traffic-light backboard
  and pole meshes, spawned once), one per group of an approach's lanes that always show the same colour, its
  active lamp drawn as a small debug point (one draw call per head and redraw). Where the meshes are missing
  (another CARLA version) the head is drawn: a dark box on a dark line. Style "marker": as before, a line pole
  with a big glowing point per signalled lane;
- next_light() gives a participant's next light (id, distance, state) from its SUMO twin, for the autopilot,
  the log and the HUD.

Debug colours go through look.debug_colour: CARLA 0.9.16 shows a debug channel of 1 as about 200 of 255 and
saturates from 3, so the old (0, 255, 0) was a blinding square with a bloom halo.
"""
import math

import carla
import traci

from . import look

COLOURS = {"G": carla.Color(0, 255, 0), "g": carla.Color(0, 255, 0), "y": carla.Color(255, 190, 0),
           "Y": carla.Color(255, 190, 0), "r": carla.Color(255, 0, 0), "R": carla.Color(255, 0, 0),
           "u": carla.Color(255, 120, 0), "s": carla.Color(255, 0, 0), "o": carla.Color(80, 80, 80),
           "O": carla.Color(80, 80, 80)}
RANK = {"G": 5, "g": 4, "y": 3, "Y": 3, "u": 2, "r": 1, "R": 1, "s": 1, "o": 0, "O": 0}

# head style: the lamp of each state, its place (top, middle, bottom) and wanted colour (look.debug_colour)
LAMP_ROW = {"r": 0, "R": 0, "s": 0, "y": 1, "Y": 1, "u": 1, "g": 2, "G": 2}
LAMP_RGB = {0: (230, 20, 0), 1: (255, 160, 0), 2: (0, 220, 60)}
POLE_MESH = "/Game/Carla/Static/Pole/TrafficLight/SM_TrafficLight__VPole_Main.SM_TrafficLight__VPole_Main"
HEAD_MESH = "/Game/Carla/Static/TrafficLight/SM_TrafficLight_holster.SM_TrafficLight_holster"
POLE_SCALE, HEAD_SCALE = 0.5, 0.65       # a 3.1 m pole; a 0.5 x 0.9 m backboard
POLE_DZ = -0.23                          # the pole mesh starts 0.47 m above its pivot
HEAD_Z = 2.6                             # the backboard's pivot above the road; its centre is 0.13 m lower
LAMP_DZ = (0.33, 0.08, -0.17)            # top, middle, bottom lamp from the backboard's centre (by eye)
LAMP_FRONT = 0.28                        # in front of the pole, on the backboard's face (its local +y)
LAMP_SIZE = 0.15                         # debug point size: a square of about 0.2 m


def lamp_row(state):
    """0 (red, top), 1 (amber), 2 (green, bottom) or None (off) for a SUMO signal state character."""
    return LAMP_ROW.get(state)


def head_points(x, y, z, travel_yaw):
    """Where a head on a kerb pole at (x, y, z) facing traffic that travels along travel_yaw (CARLA deg) has its
    pole, backboard and three lamps: {"yaw": the meshes' yaw, "pole": (x, y, z), "head": (x, y, z),
    "lamps": [(x, y, z) top, middle, bottom]}. The backboard's face is its local +y, so it turns by travel + 90."""
    yaw = (travel_yaw + 90.0) % 360.0
    r = math.radians(yaw)
    fx, fy = -math.sin(r), math.cos(r)              # the face's normal: towards the oncoming traffic
    centre = z + HEAD_Z - 0.2 * HEAD_SCALE
    lamps = [(x + fx * LAMP_FRONT, y + fy * LAMP_FRONT, centre + dz) for dz in LAMP_DZ]
    return {"yaw": yaw, "pole": (x, y, z + POLE_DZ), "head": (x, y, z + HEAD_Z), "lamps": lamps}


def group_lanes(lanes, phases):
    """Group an approach's signalled lanes into signals: lanes whose state is the same in every phase share a head.

    lanes: [(lane index on its edge, [link indices])]; phases: [state string] of the light's program. Returns
    [[lane index, ...]] sorted from the kerb (lane 0) outwards."""
    def sig(idxs):
        return tuple(max((p[i] for i in idxs if i < len(p)), key=lambda s: RANK.get(s, 0), default="o").lower()
                     for p in phases)
    groups = {}
    for index, idxs in sorted(lanes):
        groups.setdefault(sig(idxs) if phases else index, []).append(index)
    return sorted(groups.values())


class Signals:
    def __init__(self, world, net, offset, radius=120.0, every=5, step=0.02, debug=None, style="head",
                 lefthand=False, log=print):
        """net: the sumolib net SUMO runs (map.net.xml); offset: the bridge's SUMO->CARLA shift; debug: where to
        draw (the runner passes the side connection's, see side.py), else world.debug; style: "head" or
        "marker" (or "none": no drawing, next_light only); lefthand: the kerb is on the left of the lanes."""
        self.debug = world.debug if debug is None else debug
        cmap = world.get_map()
        self.radius, self.every = radius, every
        self.life = every * step * 1.6
        # "none": nothing drawn, the level's own traffic-light actors show SUMO's states (a CARLA town, towns.py)
        self.style = style if style in ("head", "marker", "none") else "head"
        self.heads = []   # (tls_id, [link indices], carla Location at the stop line, travel yaw)
        self.lanes = {}   # SUMO lane id -> (tls_id, [link indices], lane length)
        self.actors = []
        approaches = {}   # (tls, edge id) -> [(lane index, [link indices], lane)]
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
                approaches.setdefault((tls, lane.getEdge().getID()), []).append((lane.getIndex(), idxs, lane))

        def place(lane, kerb):
            """The stop line's end on the lane's kerb side (kerb True) or its other side, 0.6 m beyond the lane."""
            shape = lane.getShape()
            (x0, y0), (x1, y1) = shape[-2], shape[-1]
            d = math.hypot(x1 - x0, y1 - y0) or 1.0
            rx, ry = (y1 - y0) / d, -(x1 - x0) / d          # unit vector to the right of travel (SUMO)
            side = (lane.getWidth() / 2 + 0.6) * (1 if kerb != lefthand else -1)
            sx, sy = x1 + rx * side, y1 + ry * side
            loc = carla.Location(x=sx - offset[0], y=-(sy - offset[1]))
            wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
            loc.z = wp.transform.location.z if wp is not None else 0.0      # the road's height (elevation)
            yaw = math.degrees(math.atan2(-(y1 - y0), x1 - x0))           # CARLA yaw of travel (y flipped)
            return loc, yaw

        for (tls, _edge), items in approaches.items():
            if self.style == "marker":          # one marker per signalled lane, as before
                for _index, idxs, lane in items:
                    self.heads.append((tls, idxs, *place(lane, True)))
                continue
            try:
                phases = [p.state for p in traci.trafficlight.getAllProgramLogics(tls)[0].phases]
            except (traci.TraCIException, IndexError):
                phases = []
            by_index = {index: (idxs, lane) for index, idxs, lane in items}
            for k, group in enumerate(group_lanes([(i, idxs) for i, idxs, _ in items], phases)):
                idxs = sorted({i for index in group for i in by_index[index][0]})
                kerb = k == 0      # the group with the kerb lane on the kerb, the others across their lanes
                lane = by_index[group[0] if kerb else group[-1]][1]
                self.heads.append((tls, idxs, *place(lane, kerb)))
        if self.style == "none":
            self.heads = []
        self.meshes = self.style == "head" and self._spawn_heads(world, log)
        self._tick = 0

    def _spawn_heads(self, world, log):
        """Pole and backboard meshes at every head (static, spawned once). False when the server lacks them."""
        try:
            bp = world.get_blueprint_library().find("static.prop.mesh")
        except (RuntimeError, IndexError):
            return False
        for tls, idxs, loc, yaw in self.heads:
            p = head_points(loc.x, loc.y, loc.z, yaw)
            for mesh, scale, at in ((POLE_MESH, POLE_SCALE, p["pole"]), (HEAD_MESH, HEAD_SCALE, p["head"])):
                bp.set_attribute("mesh_path", mesh)
                bp.set_attribute("scale", str(scale))
                a = world.try_spawn_actor(bp, carla.Transform(carla.Location(*at), carla.Rotation(yaw=p["yaw"])))
                if a is None:
                    if not self.actors:
                        log(f"traffic lights: no {mesh.rsplit('.', 1)[-1]} on this server; heads are drawn")
                        return False
                    continue
                self.actors.append(a)
        return True

    def destroy(self):
        for a in self.actors:
            try:
                a.destroy()
            except RuntimeError:
                pass
        self.actors = []

    def _state(self, states, tls, idxs):
        if tls not in states:
            states[tls] = traci.trafficlight.getRedYellowGreenState(tls)
        return max((states[tls][i] for i in idxs if i < len(states[tls])), key=lambda s: RANK.get(s, 0), default="o")

    def draw(self, centres):
        self._tick += 1
        if self._tick % self.every or not centres:
            return
        states = {}
        for tls, idxs, loc, yaw in self.heads:
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            st = self._state(states, tls, idxs)
            if self.style == "marker":
                self.debug.draw_line(loc, loc + carla.Location(z=3.0), thickness=0.08,
                                     color=carla.Color(40, 40, 40), life_time=self.life)
                self.debug.draw_point(loc + carla.Location(z=3.3), size=0.25, color=COLOURS.get(st, COLOURS["o"]),
                                      life_time=self.life)
                continue
            p = head_points(loc.x, loc.y, loc.z, yaw)
            if not self.meshes:     # no meshes on this server: a dark pole and housing, drawn
                self.debug.draw_line(loc, carla.Location(*p["head"]) - carla.Location(z=0.5), thickness=0.1,
                                     color=carla.Color(0, 0, 0), life_time=self.life)
                self.debug.draw_box(carla.BoundingBox(carla.Location(*p["lamps"][1]), carla.Vector3D(0.12, 0.12, 0.42)),
                                    carla.Rotation(yaw=p["yaw"]), thickness=0.12, color=carla.Color(0, 0, 0),
                                    life_time=self.life)
            row = lamp_row(st)
            if row is not None:
                self.debug.draw_point(carla.Location(*p["lamps"][row]), size=LAMP_SIZE,
                                      color=carla.Color(*look.debug_colour(LAMP_RGB[row])), life_time=self.life)

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
