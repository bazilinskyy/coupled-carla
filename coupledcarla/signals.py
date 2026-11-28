"""Traffic lights from SUMO, drawn in CARLA.

CARLA's traffic-light actors in a world built from OpenDRIVE (generate_opendrive_world) are not usable: they
have no OpenDRIVE id, so the bridge cannot link them to SUMO, and set_state() does not change them; they stay
red. SUMO is therefore the only source of truth for signals here:
- each approach's signal stands at its stop line, coloured from SUMO, for lights near a participant. Style "head"
  (simulation.traffic_lights.style): a signal head on a pole at the kerb (CARLA 0.9.16's traffic-light backboard
  and pole meshes, spawned once), one per group of an approach's lanes that always show the same colour, its
  active lamp drawn as a round lens of debug lines (signalfaces.py: a disc of six thick lines; until 2026-11-28 a
  single square line, which showed as a flat rectangle). Where the meshes are missing (another CARLA version) the
  head is drawn: a dark box on a dark line. Style "marker": a line pole with a big glowing disc per signalled lane;
- the heads stand at the stop line, which is before the crosswalk where one crosses the approach (stoplines.py:
  SUMO's lane stopOffset), not at the end of SUMO's lane;
- next_light() gives a participant's next light (id, distance, state) from its SUMO twin, for the autopilot,
  the log and the HUD.

On CARLA 1.0 with the CoupledCarla street content (streetlook.py, data/street_props.json) the heads are the
region's own (region: "NL", "UK", "JP"): the Dutch black-and-white banded pole with a car head (three 200 mm lenses
on a yellow back plate) or, over a cycle lane, the cyclist head (200 mm lenses with a bicycle in each); the UK's car
head (200 mm) and Japan's (300 mm, horizontal) on their own poles. The lit lens is drawn on the mesh's own lens (its
centre and diameter from street_props.json): a round disc, or on a pictogram lens (cyclist, pedestrian) the lit
pictogram (signalfaces.py). A server whose content predates a head mesh (street_props.json "replaces": the 300 mm
NL/UK car heads before 2026-11-28) spawns the older mesh with its own lens positions. Where the server lacks the
content (CARLA 0.9.16, a 1.0 package without it) CARLA's pole and backboard as before.

Debug colours go through look.debug_colour: CARLA 0.9.16 shows a debug channel of 1 as about 200 of 255 and
saturates from 3, so the old (0, 255, 0) was a blinding square with a bloom halo.
"""
import math

import carla
import traci

from . import look
from . import signalfaces
from . import stoplines

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
LAMP_SIZE = 0.2                          # the lit lamp: a 0.2 m round lens in the world (thick debug lines;
                                         # a debug point keeps its size on screen, so far lamps were big blobs)


# regional heads (CARLA 1.0 street content): the head's origin (its pole's axis, at the back plate) this high above the
# road, so a car head's plate (1.44 m) spans 2.0-3.5 m on the 3.6 m pole and a cyclist head's (1.14 m) 1.9-3.0 m
REGION_HEAD_Z = {"car": 2.75, "bike": 2.45, "ped": 2.3}
LENS_D = {"car": 0.30, "bike": 0.20, "ped": 0.20}          # lens diameter (m) of a manifest entry without lens_d
LENS_FRONT = signalfaces.LIT_FRONT                         # the lit lens this far in front of the mesh's lens (m)


def _head_style(head, pole, mode):
    lenses = [tuple(v) for v in head["lenses"]]
    return {"pole": pole, "head": head["path"], "head_z": REGION_HEAD_Z[mode], "lenses": lenses,
            "lens_d": float(head.get("lens_d") or LENS_D[mode]),
            "faces": list(head.get("faces") or signalfaces.faces_for(mode, len(lenses))), "mode": mode}


def regional_style(region, mode, props=None):
    """How a region's head of a mode looks on CARLA 1.0: {"pole": mesh path or None (CARLA's), "head": mesh path,
    "head_z", "lenses": [(x, y, z) of each lens from the head's origin, red first], "lens_d": lens diameter (m),
    "faces": [pictogram key or None per lens], "mode", "alt": the style of the mesh it replaces (a server whose
    content predates it) or None}, or None (no such head: CARLA's). props: streetlook.props()."""
    if not region:
        return None
    if props is None:
        from . import streetlook
        props = streetlook.props()

    def of(role):           # by region and role, not name: a head's mesh name says its lens size
        return next((v for _k, v in sorted(props.items()) if v.get("region") == region and v.get("role") == role),
                    None)
    mode = mode if mode in ("bike", "ped") else "car"
    head = of(f"signal_{mode}")
    if not head and mode != "car":          # no cyclist head in this region: the car head
        head, mode = of("signal_car"), "car"
    if not head or not head.get("lenses"):
        return None
    pole = of("signal_pole")
    pole = pole["path"] if pole else None
    st = _head_style(head, pole, mode)
    old = head.get("replaces")
    st["alt"] = _head_style(old, pole, mode) if old and old.get("lenses") else None
    return st


def regional_points(x, y, z, travel_yaw, style, ground=None):
    """head_points for a regional head: {"yaw", "pole" (its foot on the ground), "head" (origin), "lamps" (each
    lens's centre, a little in front of it)}. The head and its lenses face +y of the mesh, towards the traffic."""
    yaw = (travel_yaw + 90.0) % 360.0
    r = math.radians(yaw)
    c, sn = math.cos(r), math.sin(r)
    hz = z + style["head_z"]
    lamps = [(x + lx * c - (ly + LENS_FRONT) * sn, y + lx * sn + (ly + LENS_FRONT) * c, hz + lz)
             for lx, ly, lz in style["lenses"]]
    return {"yaw": yaw, "pole": (x, y, z if ground is None else ground), "head": (x, y, hz), "lamps": lamps}


def lens_of(row, n_lenses):
    """The lens that shows a lamp row (0 red, 1 amber, 2 green) on a head of n lenses (a two-lens pedestrian head:
    red, green; no amber), or None."""
    if row is None:
        return None
    if n_lenses == 2:
        return {0: 0, 2: 1}.get(row)
    return row if row < n_lenses else None


def lane_mode(lane):
    """"bike" for a sumolib lane only cyclists use, else "car"."""
    try:
        return "bike" if lane.allows("bicycle") and not lane.allows("passenger") and not lane.allows("bus") \
            else "car"
    except Exception:  # noqa: BLE001
        return "car"


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
    """The ends of a debug line `size` long across a backboard turned to `yaw`, centred at `lamp` (x, y, z): one
    thick line of this kind drew the lit lamp as a size x size square until 2026-11-28 (a flat rectangle on the
    screen); signalfaces.disc_lines now draws it round from several. draw_point is no use for a lamp: CARLA draws
    a point at a constant size on the screen (Unreal's batched points), so a lamp 100 m away showed as a glowing
    block as big as a car."""
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


def head_candidates(kerb, max_back=5.0):
    """(metres back from the stop line, metres further out) to try for a head's pole, nearest first: at the stop
    line, then out (kerb side only) and back."""
    outs = (0.0, 0.5, 1.0, 1.5, 2.0) if kerb else (0.0,)
    backs = [b for b in (0.0, 1.0, 2.0, 3.5, 5.0) if b <= max(0.0, max_back)]
    return [(b, o) for b in backs for o in outs]


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
                 lefthand=False, log=print, surface=None, region=None, stop_offsets=None):
        """net: the sumolib net SUMO runs (map.net.xml); offset: the bridge's SUMO->CARLA shift; debug: where to
        draw (the runner passes the side connection's, see side.py), else world.debug; style: "head" or
        "marker" (or "none": no drawing, next_light only); lefthand: the kerb is on the left of the lanes; surface:
        placement.GroundSurface of the scenery's ground tiles (the poles stand on them); region: "NL", "UK", "JP"
        (streetlook.region) for the region's heads on a CARLA 1.0 server with the street content, None: CARLA's;
        stop_offsets: {lane id: m} the net's stop lines before the lanes' ends (stoplines.read_offsets of the net
        SUMO runs): the heads stand there and next_light() measures to them."""
        self.debug = world.debug if debug is None else debug
        cmap = world.get_map()
        self.surface = surface
        from .placement import paved
        surfaced = paved(carla)
        self._on_road_any = lambda loc: cmap.get_waypoint(loc, project_to_road=False,
                                                          lane_type=surfaced) is not None
        self.radius, self.every = radius, every
        self.life = every * step * 1.6
        # "none": nothing drawn or spawned, the level's own traffic-light actors show SUMO's states (a CARLA town,
        # towns.py); the heads are still placed (the dashboard's live map shows them)
        self.style = style if style in ("head", "marker", "none") else "head"
        self.heads = []   # (tls_id, [link indices], carla Location at the stop line, travel yaw): the first light
        self.head_links = []    # per head every (tls_id, [link indices]) it shows (several lights per approach)
        self.head_modes = []    # per head "car" or "bike" (its lanes are cyclists' only)
        self.region = region
        self.styles = []        # per head its regional_style once spawned, else None (CARLA's head)
        self.stop_offsets = dict(stop_offsets or {})
        self.lanes = {}   # SUMO lane id -> [(tls_id, [link indices], stop line's position, {edges the links lead to})]
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
                stop_at = lane.getLength() - self.stop_offsets.get(lane_id, 0.0)     # the stop line (stoplines.py)
                self.lanes.setdefault(lane_id, []).append((tls, idxs, stop_at, {out_edge.get(i) for i in idxs}))
                entry = approaches.setdefault(lane.getEdge().getID(), {}).setdefault(lane.getIndex(), (lane, []))
                entry[1].extend((tls, i) for i in idxs)

        road = carla.LaneType.Driving | carla.LaneType.Biking | carla.LaneType.Bidirectional

        def on_road(loc):
            return cmap.get_waypoint(loc, project_to_road=False, lane_type=road) is not None

        def place(lane, kerb, out=0.0):
            """The stop line's end on the lane's kerb side (kerb True) or its other side, 0.6 m (+ out) beyond the
            lane; where that is on a lane of CARLA's map (a cycle lane beyond, or a junction's turning lanes,
            which flare out past the stop line), further out and up to 5 m back from the stop line. The stop line
            is the lane's end, or its stop offset before it (stoplines.py: before the crosswalk; on the lane
            before when the offset is longer than the lane)."""
            off = self.stop_offsets.get(lane.getID(), 0.0)
            chain, cur, total = [lane.getShape()], lane, lane.getLength()
            while total < off + 6.0 and len(chain) < 6:
                try:
                    inc = cur.getIncoming()
                except AttributeError:
                    break
                if not inc:
                    break
                cur = inc[0]
                chain.append(cur.getShape())
                total += cur.getLength()
            sign = 1 if kerb != lefthand else -1
            first = None
            for back, extra in head_candidates(kerb, min(5.0, total - off - 1.0)):
                x1, y1, ux, uy = stoplines.point_back(chain, off + back)
                rx, ry = uy, -ux                            # unit vector to the right of travel (SUMO)
                side = (lane.getWidth() / 2 + 0.6 + out + extra) * sign
                sx, sy = x1 + rx * side, y1 + ry * side
                loc = carla.Location(x=sx - offset[0], y=-(sy - offset[1]))
                if first is None:
                    first = (loc, ux, uy)
                if not on_road(loc):
                    break
            else:
                loc, ux, uy = first
            wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
            loc.z = wp.transform.location.z if wp is not None else 0.0      # the road's height (elevation)
            yaw = math.degrees(math.atan2(-uy, ux))                       # CARLA yaw of travel (y flipped)
            return loc, yaw

        def add(links, loc, yaw, mode="car"):
            by = {}
            for tls, i in links:
                by.setdefault(tls, []).append(i)
            pairs = [(tls, sorted(set(idxs))) for tls, idxs in sorted(by.items())]
            self.heads.append((pairs[0][0], pairs[0][1], loc, yaw))
            self.head_links.append(pairs)
            self.head_modes.append(mode)

        for _edge, lanes in approaches.items():
            if self.style == "marker":          # one marker per signalled lane, as before
                for _index, (lane, links) in sorted(lanes.items()):
                    add(links, *place(lane, True), lane_mode(lane))
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
                mode = "bike" if all(lane_mode(lanes[index][0]) == "bike" for index in group) else "car"
                add(links, *head_spot(k, groups, by_index, place, on_road), mode)
        self.meshes = self.style == "head" and self._spawn_heads(world, log)
        self._tick = 0

    def _spawn_heads(self, world, log):
        """Pole and backboard meshes at every head (static, spawned once). False when the server lacks them."""
        try:
            bp = world.get_blueprint_library().find("static.prop.mesh")
        except (RuntimeError, IndexError):
            return False
        self.styles = [None] * len(self.heads)
        self._have, self._missing = set(), set()       # head meshes this server has spawned / lacks
        regional = self.region is not None
        for h, (tls, idxs, loc, yaw) in enumerate(self.heads):
            st = regional_style(self.region, self.head_modes[h]) if regional else None
            if st is not None and st["head"] in self._missing and st.get("alt"):
                st = st["alt"]
            if st is not None:
                ok = self._spawn_regional(world, bp, h, loc, yaw, st)
                if ok is None and st.get("alt"):      # content older than this head: the mesh it replaces
                    self._missing.add(st["head"])
                    ok = self._spawn_regional(world, bp, h, loc, yaw, st["alt"])
                    if ok:
                        log(f"traffic lights: no {st['head'].rsplit('.', 1)[-1]} on this server; its older "
                            f"{st['alt']['head'].rsplit('.', 1)[-1]} (re-import the street props)")
                if ok:
                    continue
                if ok is None and not self._have:     # the server lacks the content: CARLA's heads from here on
                    log(f"traffic lights: no {self.region} signal meshes on this server; CARLA's heads")
                    regional = False
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
        n = sum(st is not None for st in self.styles)
        if n:
            log(f"traffic lights: {n} of {len(self.heads)} heads are {self.region} ones "
                f"({sum(st is not None and st['mode'] == 'bike' for st in self.styles)} for cyclists)")
        return True

    def _spawn_regional(self, world, bp, h, loc, yaw, st):
        """The region's pole and head at head h: True; False when one did not spawn here (blocked: CARLA's head);
        None when the server seems to lack the head mesh (it never spawned on this server)."""
        p = regional_points(loc.x, loc.y, loc.z, yaw, st, self._ground(loc))
        parts = [(st["head"], 1.0, p["head"], p["yaw"])]
        if st["pole"]:
            parts.insert(0, (st["pole"], 1.0, p["pole"], p["yaw"]))
        else:                   # CARLA's pole, the head on it
            c = head_points(loc.x, loc.y, loc.z, yaw, self._ground(loc))
            parts.insert(0, (POLE_MESH, c["pole_scale"], c["pole"], c["yaw"]))
        spawned = []
        for mesh, scale, at, myaw in parts:
            bp.set_attribute("mesh_path", mesh)
            bp.set_attribute("scale", str(scale))
            a = world.try_spawn_actor(bp, carla.Transform(carla.Location(*at), carla.Rotation(yaw=myaw)))
            if a is None:
                for b in spawned:
                    b.destroy()
                return False if st["head"] in getattr(self, "_have", ()) else None
            spawned.append(a)
        self.actors += spawned
        self.styles[h] = st
        getattr(self, "_have", set()).add(st["head"])
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

    def start_log(self, path, clock):
        """Write every drawn head's state (sim time from clock(), head, its first light, lamp state) to a CSV, to
        check against SUMO's own record of its lights (tls_states.xml, signalplan.tls_states_additional)."""
        self._log = open(path, "w", buffering=1)
        self._log.write("time,head,links,state\n")
        self._clock = clock

    def stop_log(self):
        if getattr(self, "_log", None):
            self._log.close()
            self._log = None

    def prefetch(self, centres):
        """The states of the lights whose heads are near the centres, read now (the pipelined bridge calls it at
        each SUMO step, where TraCI calls cost no wait); draw() then uses them."""
        if self.style == "none" or not centres:
            self.cached = {}
            return
        states = {}
        for (_tls, _idxs, loc, _yaw), links in zip(self.heads, self.head_links):
            if any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                for tls, _ in links:
                    if tls not in states:
                        states[tls] = traci.trafficlight.getRedYellowGreenState(tls)
        self.cached = states

    def draw(self, centres):
        if self.style == "none":        # the level's own light actors show SUMO's states (heads kept: livemap)
            return
        self._tick += 1
        if self._tick % self.every or not centres:
            return
        cached = getattr(self, "cached", None)
        states = dict(cached) if cached is not None else {}
        log = getattr(self, "_log", None)
        now = self._clock() if log else None
        for h, ((_tls, _idxs, loc, yaw), links) in enumerate(zip(self.heads, self.head_links)):
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            st = self._state(states, links)
            if log:
                log.write(f"{now:.2f},{h},{'|'.join(t + ':' + ' '.join(map(str, i)) for t, i in links)},{st}\n")
            if self.style == "marker":
                self.debug.draw_line(loc, loc + carla.Location(z=3.0), thickness=0.08,
                                     color=carla.Color(40, 40, 40), life_time=self.life)
                self._lit((loc.x, loc.y, loc.z + 3.3), yaw + 90.0, 0.35, None,    # world-sized, not a
                          COLOURS.get(st, COLOURS["o"]))                          # screen-sized point
                continue
            rst = self.styles[h] if h < len(self.styles) else None
            if rst is not None:     # the region's head: its own lens lit
                p = regional_points(loc.x, loc.y, loc.z, yaw, rst)
                lens = lens_of(lamp_row(st), len(rst["lenses"]))
                if lens is not None:
                    self._lit(p["lamps"][lens], p["yaw"], rst["lens_d"], rst["faces"][lens],
                              carla.Color(*look.debug_colour(LAMP_RGB[lamp_row(st)])))
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
                self._lit(p["lamps"][row], p["yaw"], LAMP_SIZE, None, carla.Color(*look.debug_colour(LAMP_RGB[row])))

    def _lit(self, centre, face_yaw, diameter, face, colour):
        """A lit lens at centre on a face turned to face_yaw: a round disc of thick lines, or its lit pictogram
        (signalfaces.lit_lines)."""
        for a, b, t in signalfaces.lit_lines(centre, face_yaw, diameter, face):
            self.debug.draw_line(carla.Location(*a), carla.Location(*b), thickness=t, color=colour,
                                 life_time=self.life)

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
