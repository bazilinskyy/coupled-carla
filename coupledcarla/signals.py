"""Traffic lights from SUMO, drawn in CARLA.

CARLA's traffic-light actors in a world built from OpenDRIVE (generate_opendrive_world) are not usable: they
have no OpenDRIVE id, so the bridge cannot link them to SUMO, and set_state() does not change them; they stay
red. SUMO is therefore the only source of truth for signals here:
- each approach's signal stands at its stop line, coloured from SUMO, for lights near a participant. Style "head"
  (simulation.traffic_lights.style): a signal head on a pole at the kerb (CARLA 0.9.16's traffic-light backboard
  and pole meshes, spawned once), one per group of an approach's lanes that always show the same colour, its
  active lamp drawn as a short thick debug line, 0.2 m square in the world (one draw call per head and redraw). Where the meshes are missing
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
POLE_LEN, POLE_BASE = 6.2, 0.47          # the pole mesh's length and its foot above the pivot at scale 1
HEAD_Z = 2.6                             # the backboard's pivot above the road; its centre is 0.13 m lower
LAMP_DZ = (0.33, 0.08, -0.17)            # top, middle, bottom lamp from the backboard's centre (by eye)
LAMP_FRONT = 0.28                        # in front of the pole, on the backboard's face (its local +y)
LAMP_SIZE = 0.2                          # the lit lamp: a 0.2 x 0.2 m square in the world (a thick debug line;
                                         # a debug point keeps its size on screen, so far lamps were big blobs)


def lamp_row(state):
    """0 (red, top), 1 (amber), 2 (green, bottom) or None (off) for a SUMO signal state character."""
    return LAMP_ROW.get(state)


def head_points(x, y, z, travel_yaw, ground=None):
    """Where a head on a kerb pole at (x, y), z the road's height, facing traffic that travels along travel_yaw
    (CARLA deg) has its pole, backboard and three lamps: {"yaw": the meshes' yaw, "pole": (x, y, z), "pole_scale",
    "head": (x, y, z), "lamps": [(x, y, z) top, middle, bottom]}. The backboard's face is its local +y, so it turns
    by travel + 90. ground: the height of the ground under the pole when it is not the road's (the scenery's
    ground tiles lie 0.15 m or more under the road): the pole reaches down to it, a little longer (its top and the
    head stay where they are; at the road's height, poles stood up to 0.5 m in the air)."""
    yaw = (travel_yaw + 90.0) % 360.0
    r = math.radians(yaw)
    fx, fy = -math.sin(r), math.cos(r)              # the face's normal: towards the oncoming traffic
    centre = z + HEAD_Z - 0.2 * HEAD_SCALE
    lamps = [(x + fx * LAMP_FRONT, y + fy * LAMP_FRONT, centre + dz) for dz in LAMP_DZ]
    scale, pz = POLE_SCALE, z + POLE_DZ
    if ground is not None and abs(ground - z) > 0.02:
        top = pz + POLE_SCALE * (POLE_BASE + POLE_LEN)      # the pole's top as it stands on the road
        scale = max(0.3, (top - ground) / POLE_LEN)
        pz = ground - scale * POLE_BASE                     # the pivot, under the mesh's foot
    return {"yaw": yaw, "pole": (x, y, pz), "pole_scale": scale, "head": (x, y, z + HEAD_Z), "lamps": lamps}


def lamp_segment(lamp, yaw, size=LAMP_SIZE):
    """The ends of the short thick debug line that draws a lit lamp at `lamp` (x, y, z) on a backboard turned to
    `yaw`: across the face, so the line's thickness gives a lamp of size x size metres. draw_point is no use for
    it: CARLA draws a point at a constant size on the screen (Unreal's batched points), so a lamp 100 m away
    showed as a glowing block as big as a car."""
    r = math.radians(yaw)
    ux, uy = math.cos(r) * size / 2, math.sin(r) * size / 2      # the backboard's local x: along its face
    x, y, z = lamp
    return (x - ux, y - uy, z), (x + ux, y + uy, z)


def group_lanes(lanes, phases):
    """Group an approach's signalled lanes into signals: lanes whose state is the same in every phase share a head.

    lanes: [(lane index on its edge, [link indices])]; phases: [state string] of the light's program. Returns
    [[lane index, ...]] sorted from the kerb (lane 0) outwards."""
    return group_links([(index, [("", i) for i in idxs]) for index, idxs in lanes], {"": phases} if phases else {})


def group_links(lanes, phases):
    """group_lanes for an approach whose lanes several traffic lights control (on maps from OpenDRIVE every signal
    is a SUMO programme of its own, so one approach has a light per turn): lanes [(lane index, [(tls, link index)])],
    phases {tls: [state string]}. Lanes whose links show the same states in every phase of every light share a head;
    without programmes, a head per lane. (Grouped per light, an approach stood up to four heads on one spot.)"""
    def sig(links):
        by = {}
        for tls, i in links:
            by.setdefault(tls, []).append(i)
        return tuple((tls, tuple(max((p[i] for i in idxs if i < len(p)), key=lambda s: RANK.get(s, 0),
                                     default="o").lower() for p in phases[tls]))
                     for tls, idxs in sorted(by.items()))
    groups = {}
    for index, links in sorted(lanes, key=lambda t: t[0]):
        known = phases and all(tls in phases for tls, _ in links)
        groups.setdefault(sig(links) if known else ("lane", index), []).append(index)
    return sorted(groups.values())


SIDE_BY_SIDE = 0.7       # a second head at the same kerb stands this much further out (backboards 0.5 m wide)


def head_spot(k, groups, by_index, place, on_road):
    """(location, travel yaw) of group k's head: the kerb group (k 0) at the kerb; the others on the far side of
    the approach's innermost lane (a median or the far kerb of a one-way street), each on its own spot, or, where
    that spot is on a lane (no median: the other direction's lanes are right there), at the kerb beside the first
    head. (Placing a middle group's head beside its own lanes stood it on the road between two lanes.)
    place(lane, kerb, out) and on_road(location) come from Signals."""
    if k == 0:
        return place(by_index[groups[0][0]][1], True)
    inner = by_index[max(by_index)][1]
    far = near = 0
    for j in range(1, k + 1):       # where the earlier groups went decides this one's spot
        loc, yaw = place(inner, False, SIDE_BY_SIDE * far)
        if on_road(loc):
            near += 1
            loc, yaw = place(by_index[groups[0][0]][1], True, SIDE_BY_SIDE * near)
        else:
            far += 1
    return loc, yaw


class Signals:
    def __init__(self, world, net, offset, radius=120.0, every=5, step=0.02, debug=None, style="head",
                 lefthand=False, log=print, surface=None):
        """net: the sumolib net SUMO runs (map.net.xml); offset: the bridge's SUMO->CARLA shift; debug: where to
        draw (the runner passes the side connection's, see side.py), else world.debug; style: "head" or
        "marker"; lefthand: the kerb is on the left of the lanes; surface: placement.GroundSurface of the scenery's
        ground tiles (the poles stand on them)."""
        self.debug = world.debug if debug is None else debug
        cmap = world.get_map()
        self.surface = surface
        self._on_road_any = lambda loc: cmap.get_waypoint(loc, project_to_road=False,
                                                          lane_type=carla.LaneType.Any) is not None
        self.radius, self.every = radius, every
        self.life = every * step * 1.6
        self.style = style if style in ("head", "marker") else "head"
        self.heads = []   # (tls_id, [link indices], carla Location at the stop line, travel yaw): the first light
        self.head_links = []    # per head every (tls_id, [link indices]) it shows (several lights per approach)
        self.lanes = {}   # SUMO lane id -> [(tls_id, [link indices], lane length, {edges the links lead to})]
        self.actors = []
        approaches = {}   # edge id -> {lane index: (lane, [(tls, link index)])}
        for tls in traci.trafficlight.getIDList():
            by_lane, out_edge = {}, {}
            for idx, links in enumerate(traci.trafficlight.getControlledLinks(tls)):
                for in_lane, out_lane, _via in links:
                    by_lane.setdefault(in_lane, []).append(idx)
                    out_edge[idx] = out_lane.rsplit("_", 1)[0]
            for lane_id, idxs in by_lane.items():
                try:
                    lane = net.getLane(lane_id)
                except KeyError:
                    continue
                self.lanes.setdefault(lane_id, []).append(
                    (tls, idxs, lane.getLength(), {out_edge.get(i) for i in idxs}))
                entry = approaches.setdefault(lane.getEdge().getID(), {}).setdefault(lane.getIndex(), (lane, []))
                entry[1].extend((tls, i) for i in idxs)

        road = carla.LaneType.Driving | carla.LaneType.Biking | carla.LaneType.Bidirectional

        def on_road(loc):
            return cmap.get_waypoint(loc, project_to_road=False, lane_type=road) is not None

        def place(lane, kerb, out=0.0):
            """The stop line's end on the lane's kerb side (kerb True) or its other side, 0.6 m (+ out) beyond the
            lane; on the kerb side pushed further out while it is on a lane of CARLA's map (a cycle lane beyond)."""
            shape = lane.getShape()
            (x0, y0), (x1, y1) = shape[-2], shape[-1]
            d = math.hypot(x1 - x0, y1 - y0) or 1.0
            rx, ry = (y1 - y0) / d, -(x1 - x0) / d          # unit vector to the right of travel (SUMO)
            sign = 1 if kerb != lefthand else -1
            for extra in ((0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0) if kerb else (0.0,)):
                side = (lane.getWidth() / 2 + 0.6 + out + extra) * sign
                sx, sy = x1 + rx * side, y1 + ry * side
                loc = carla.Location(x=sx - offset[0], y=-(sy - offset[1]))
                if not on_road(loc):
                    break
            wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
            loc.z = wp.transform.location.z if wp is not None else 0.0      # the road's height (elevation)
            yaw = math.degrees(math.atan2(-(y1 - y0), x1 - x0))           # CARLA yaw of travel (y flipped)
            return loc, yaw

        def add(links, loc, yaw):
            by = {}
            for tls, i in links:
                by.setdefault(tls, []).append(i)
            pairs = [(tls, sorted(set(idxs))) for tls, idxs in sorted(by.items())]
            self.heads.append((pairs[0][0], pairs[0][1], loc, yaw))
            self.head_links.append(pairs)

        for _edge, lanes in approaches.items():
            if self.style == "marker":          # one marker per signalled lane, as before
                for _index, (lane, links) in sorted(lanes.items()):
                    add(links, *place(lane, True))
                continue
            phases = {}
            for tls in sorted({t for _lane, links in lanes.values() for t, _ in links}):
                try:
                    phases[tls] = [p.state for p in traci.trafficlight.getAllProgramLogics(tls)[0].phases]
                except (traci.TraCIException, IndexError):
                    pass
            by_index = {index: ([], lane) for index, (lane, _links) in lanes.items()}
            groups = group_links([(index, links) for index, (_lane, links) in lanes.items()], phases)
            for k, group in enumerate(groups):
                links = [lk for index in group for lk in lanes[index][1]]
                add(links, *head_spot(k, groups, by_index, place, on_road))
        self.meshes = self.style == "head" and self._spawn_heads(world, log)
        self._tick = 0

    def _spawn_heads(self, world, log):
        """Pole and backboard meshes at every head (static, spawned once). False when the server lacks them."""
        try:
            bp = world.get_blueprint_library().find("static.prop.mesh")
        except (RuntimeError, IndexError):
            return False
        for tls, idxs, loc, yaw in self.heads:
            p = head_points(loc.x, loc.y, loc.z, yaw, self._ground(loc))
            for mesh, scale, at in ((POLE_MESH, p["pole_scale"], p["pole"]), (HEAD_MESH, HEAD_SCALE, p["head"])):
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

    def _ground(self, loc):
        """The ground tiles' top under a pole off the lanes (scenery.json's ground_planes), else None (the road's)."""
        if self.surface is None or self._on_road_any(loc):
            return None
        return self.surface.z(loc.x, loc.y)

    def destroy(self):
        for a in self.actors:
            try:
                a.destroy()
            except RuntimeError:
                pass
        self.actors = []

    def _state(self, states, links):
        """The state a head shows: the most permissive of its links' (green if any of its turns may go)."""
        out = "o"
        for tls, idxs in links:
            if tls not in states:
                states[tls] = traci.trafficlight.getRedYellowGreenState(tls)
            out = max([out] + [states[tls][i] for i in idxs if i < len(states[tls])], key=lambda s: RANK.get(s, 0))
        return out

    def draw(self, centres):
        self._tick += 1
        if self._tick % self.every or not centres:
            return
        states = {}
        for (_tls, _idxs, loc, yaw), links in zip(self.heads, self.head_links):
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            st = self._state(states, links)
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
                a, b = lamp_segment(p["lamps"][row], p["yaw"])
                self.debug.draw_line(carla.Location(*a), carla.Location(*b), thickness=LAMP_SIZE,
                                     color=carla.Color(*look.debug_colour(LAMP_RGB[row])), life_time=self.life)

    def next_light(self, sumo_vehicle_id):
        """(tls id, distance m, state char) of the light at the end of the vehicle's SUMO lane, or None.

        SUMO's getNextTLS() returns nothing for vehicles placed with moveToXY (the participants' twins), so
        this looks at the lane the twin is on: the light that ends it, if any.
        """
        try:
            lane = traci.vehicle.getLaneID(sumo_vehicle_id)
            hits = self.lanes.get(lane)
            if not hits:
                return None
            dist = max(0.0, hits[0][2] - traci.vehicle.getLanePosition(sumo_vehicle_id))
            if len(hits) > 1:       # a light per turn: the one towards the twin's next edge, if its route says
                route, ri = traci.vehicle.getRoute(sumo_vehicle_id), traci.vehicle.getRouteIndex(sumo_vehicle_id)
                nxt = route[ri + 1] if 0 <= ri < len(route) - 1 else None
                hits = [h for h in hits if nxt in h[3]] or hits
            best = None
            for tls, idxs, _length, _to in hits:    # else the most restrictive of them (stop if any turn is red)
                state = traci.trafficlight.getRedYellowGreenState(tls)
                st = max((state[i] for i in idxs if i < len(state)), key=lambda c: RANK.get(c, 0), default="o")
                if best is None or RANK.get(st, 0) < RANK.get(best[1], 0):
                    best = (tls, st)
        except traci.TraCIException:
            return None
        return best[0], dist, best[1]
