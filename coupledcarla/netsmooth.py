"""Smoother junctions for SUMO's net (mapbuild.sumo_net): no sideways jumps inside junctions, slower sharp turns.

Where a road widens (Eindhoven, Fellenoord: 2 lanes become 4 at junction 452), netconvert connects the left lane to
each new lane through the junction, so a vehicle bound for the far one crosses two lanes (8 m sideways) inside an
8.8 m junction: SUMO turns its heading 77 -> 27 -> 74 deg there, and the participant's car on the SUMO driver, which
follows its SUMO twin, swerved at 36 km/h (22 m/s^2 lateral). Such a connection is dropped when the same lane has
another one, at least SPARE_M less sideways, to the same road: the vehicle takes that one and changes lanes on the
road after the junction, where the sublane model spreads the move over tens of metres. A sideways connection
without such an alternative (a lane that merges) is kept, with a speed at which its S-bend stays under A_LAT.
Turns: netconvert limits a turn's speed to an average lateral acceleration of 5.5 m/s^2 by default; urban drivers
turn at about 2-3.5 (Xu et al. 2015, "Establishing style-oriented driver models by imitating human driving
behaviors", lateral acceleration in turns; AASHTO's design side friction at 20-40 km/h, 0.3-0.4 g, is a ceiling, not
the norm), so TURN_ACC (--junctions.limit-turn-speed) is passed with the net's other options.

The OpenDRIVE (CARLA's roads) is not changed; only map.net.xml. tools/junction_check.py-style numbers: README.
"""
import math
import subprocess
import tempfile
from pathlib import Path

TURN_ACC = 3.0        # m/s^2: --junctions.limit-turn-speed (netconvert's default 5.5)
SHIFT_M = 1.6         # sideways more than this (half a lane) within a junction ...
SHIFT_SLOPE = 0.25    # ... and more than this per metre along it: a sharp lane shift
STRAIGHT_DEG = 30.0   # only connections that keep their direction (a turn is not a shift)
SPARE_M = 1.5         # an alternative to the same road must be this much less sideways
A_LAT = 3.0           # m/s^2 at the peak of a kept S-bend (sinusoidal: 2 pi shift v^2 / length^2)
MIN_SPEED = 4.0       # m/s: never slower than this through a kept shift
CURVE_ACC = 3.0       # m/s^2: a lane's speed is cut to sqrt(CURVE_ACC * its tightest radius) (SUMO has no curve
                      # speeds of its own: a car took an 85 deg bend of an OSM road at 49 km/h, 30 m/s^2 lateral)
CURVE_WINDOW = 6.0    # m: the radius over this much of the lane's shape
CURVE_MIN_SPEED = 4.0 # m/s
NET_ARGS = ["--junctions.limit-turn-speed", f"{TURN_ACC:g}"]       # turnarounds: turnarounds() below
KINK_DEG = 8.0        # a junction lane with a corner sharper than this (at its ends or inside) is redrawn
BEZIER_STEP = 1.0     # m between the points of a redrawn junction lane
JUNCTION_MARGIN_M = 1.0   # a redrawn junction lane stays within the junction's outline and this margin (a 120 deg
                          # turn's curve bulged over the stop line of a road beside it: a bus across a waiting car)
FILLET_R = 25.0       # m: corners of a road's lane line are rounded with this radius ...
FILLET_MAX_M = 0.5    # ... or a smaller one, so the line moves at most this far from the corner (CARLA's road)
FILLET_MIN_DEG = 4.0  # corners gentler than this are left alone


def shift_of(shape, before, after):
    """(sideways, along, angle_deg) of an internal lane's shape: sideways and along its incoming lane's direction
    (before: the last segment of the incoming lane), angle between that and the outgoing lane's first segment."""
    dx, dy = before[1][0] - before[0][0], before[1][1] - before[0][1]
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n
    ex, ey = shape[-1][0] - shape[0][0], shape[-1][1] - shape[0][1]
    side, along = abs(-ex * uy + ey * ux), ex * ux + ey * uy
    ax, ay = after[1][0] - after[0][0], after[1][1] - after[0][1]
    ang = abs((math.degrees(math.atan2(ay, ax) - math.atan2(dy, dx)) + 180.0) % 360.0 - 180.0)
    return side, along, ang


def is_sharp(side, along, ang):
    return ang < STRAIGHT_DEG and side > SHIFT_M and side > SHIFT_SLOPE * max(along, 0.1)


def bend_speed(side, along, a_lat=A_LAT):
    """The speed (m/s) at which a sideways move of `side` over `along` metres peaks at a_lat."""
    if side <= 0:
        return None
    return max(MIN_SPEED, max(along, 1.0) * math.sqrt(a_lat / (2.0 * math.pi * side)))


def resample(shape, step=1.0):
    """Points every `step` metres along a polyline."""
    out = [shape[0]]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        d = step - carry
        while d <= seg:
            u = d / seg
            out.append((x0 + u * (x1 - x0), y0 + u * (y1 - y0)))
            d += step
        carry = seg - (d - step)
    if out[-1] != shape[-1]:
        out.append(shape[-1])
    return out


def min_radius(shape, window=CURVE_WINDOW):
    """The tightest radius (m) of a lane's shape: its heading change over `window` metres; None when straight or
    shorter than the window."""
    pts = resample(shape, 1.0)
    n = int(window)
    if len(pts) < n + 2:
        return None
    hd = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    best = None
    for i in range(len(hd) - n):
        turn = abs((hd[i + n] - hd[i] + math.pi) % (2 * math.pi) - math.pi)
        if turn > 1e-3:
            r = n / turn
            best = r if best is None or r < best else best
    return best


def curve_speed(radius, speed, a=CURVE_ACC):
    """The lane speed for its tightest radius (None: leave it)."""
    if radius is None:
        return None
    v = max(CURVE_MIN_SPEED, math.sqrt(a * radius))
    return round(v, 2) if v < speed - 0.1 else None


def plan(conns):
    """conns: [{from, to, fromLane, toLane, side, along, angle, speed}] -> (deletes, slowed): the sharp shifts with a
    straighter alternative from the same lane to the same road, and the remaining sharp ones with a new speed."""
    groups = {}
    for c in conns:
        groups.setdefault((c["from"], c["fromLane"], c["to"]), []).append(c)
    deletes, slowed = [], []
    for group in groups.values():
        least = min(c["side"] for c in group)
        for c in group:
            if not is_sharp(c["side"], c["along"], c["angle"]):
                continue
            if c["side"] - least >= SPARE_M:
                deletes.append(c)
            else:
                v = bend_speed(c["side"], c["along"])
                if c.get("speed") is None or v < c["speed"]:
                    slowed.append(dict(c, new_speed=round(v, 2)))
    return deletes, slowed


def connections(net_path):
    """The net's connections with their internal lane's sideways shift (sumolib)."""
    import sumolib
    net = sumolib.net.readNet(str(net_path), withInternal=True, withPedestrianConnections=False)
    out = []
    for e in net.getEdges():
        if e.getFunction() == "internal":
            continue
        for lane in e.getLanes():
            for c in lane.getOutgoing():
                via = c.getViaLaneID()
                if not via:
                    continue
                try:
                    il = net.getLane(via)
                except KeyError:
                    continue
                shape, before, after = il.getShape(), lane.getShape()[-2:], c.getToLane().getShape()[:2]
                if len(shape) < 2 or len(before) < 2 or len(after) < 2:
                    continue
                side, along, ang = shift_of(shape, before, after)
                out.append({"from": e.getID(), "to": c.getTo().getID(), "fromLane": lane.getIndex(),
                            "toLane": c.getToLane().getIndex(), "side": side, "along": along, "angle": ang,
                            "speed": il.getSpeed()})
    return out


def curves(net_path):
    """[(edge, lane index, new speed, radius)] for the normal lanes of a net whose bends need a lower speed."""
    import sumolib
    net = sumolib.net.readNet(str(net_path), withInternal=False, withPedestrianConnections=False)
    out = []
    for e in net.getEdges():
        for lane in e.getLanes():
            r = min_radius(lane.getShape())
            v = curve_speed(r, lane.getSpeed())
            if v is not None:
                out.append((e.getID(), lane.getIndex(), v, round(r, 1)))
    return out


def edge_xml(slow):
    lines = ["<edges>"]
    by_edge = {}
    for eid, idx, v, _ in slow:
        by_edge.setdefault(eid, []).append((idx, v))
    for eid, lanes in by_edge.items():
        lines.append(f'    <edge id="{eid}">')
        lines += [f'        <lane index="{i}" speed="{v}"/>' for i, v in lanes]
        lines.append("    </edge>")
    lines.append("</edges>")
    return "\n".join(lines) + "\n"


def _parse_shape(text):
    return [tuple(float(v) for v in p.split(",")) for p in text.split()]


def _fmt_shape(pts):
    return " ".join(",".join(f"{v:.2f}" for v in p) for p in pts)


def _heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _angle(a, b):
    return abs(math.degrees((a - b + math.pi) % (2 * math.pi) - math.pi))


def _inside(pt, poly, margin):
    """Whether pt lies in polygon poly or within margin metres of its outline."""
    x, y = pt[0], pt[1]
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][:2]
        x2, y2 = poly[(i + 1) % n][:2]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    if inside:
        return True
    for i in range(n):
        ax, ay = poly[i][:2]
        bx, by = poly[(i + 1) % n][:2]
        L = (bx - ax) ** 2 + (by - ay) ** 2
        u = 0.0 if L == 0 else max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L))
        if math.hypot(x - ax - u * (bx - ax), y - ay - u * (by - ay)) <= margin:
            return True
    return False


def bezier(p0, d0, p3, d1, step=BEZIER_STEP, handle=1.0 / 3.0):
    """A cubic Bezier from p0 leaving along unit d0 to p3 arriving along unit d1 (3D points: z linear), sampled
    every ~step metres. Its handles are a third of the chord, so the curve meets both lanes with their direction
    (no kink) and bends as evenly as a cubic can."""
    c = math.dist(p0[:2], p3[:2]) * handle
    p1 = (p0[0] + d0[0] * c, p0[1] + d0[1] * c)
    p2 = (p3[0] - d1[0] * c, p3[1] - d1[1] * c)
    n = max(2, int(math.ceil(3.0 * c / step)) + 1)
    z0, z3 = (p0[2] if len(p0) > 2 else None), (p3[2] if len(p3) > 2 else None)
    out = []
    for i in range(n + 1):
        u = i / n
        a, b, cc, d = (1 - u) ** 3, 3 * u * (1 - u) ** 2, 3 * u * u * (1 - u), u ** 3
        x = a * p0[0] + b * p1[0] + cc * p2[0] + d * p3[0]
        y = a * p0[1] + b * p1[1] + cc * p2[1] + d * p3[1]
        out.append((x, y) if z0 is None else (x, y, z0 + (z3 - z0) * u))
    return out


def polyline_length(pts):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:]))


def fillet(shape, radius=FILLET_R, max_move=FILLET_MAX_M, min_deg=FILLET_MIN_DEG, step=BEZIER_STEP):
    """A polyline with its corners rounded into circular arcs (tangent to both segments). Each arc's radius is
    `radius`, or smaller where the line would move more than max_move from the corner or the arc would need more
    than half of a neighbouring segment. The ends stay where they are. Points may be 2D or 3D (z interpolated)."""
    if len(shape) < 3:
        return list(shape)
    out = [shape[0]]
    for i in range(1, len(shape) - 1):
        a, b, c = shape[i - 1], shape[i], shape[i + 1]
        la, lc = math.dist(a[:2], b[:2]), math.dist(b[:2], c[:2])
        if la < 1e-6 or lc < 1e-6:
            continue
        ua = ((b[0] - a[0]) / la, (b[1] - a[1]) / la)
        uc = ((c[0] - b[0]) / lc, (c[1] - b[1]) / lc)
        turn = math.acos(max(-1.0, min(1.0, ua[0] * uc[0] + ua[1] * uc[1])))
        if math.degrees(turn) < min_deg or turn > math.radians(170):
            out.append(b)
            continue
        half = turn / 2.0
        r = min(radius, max_move / (1.0 / math.cos(half) - 1.0) if half > 1e-6 else radius)
        t = r * math.tan(half)                      # from the corner to where the arc starts
        t_max = 0.5 * min(la, lc)
        if t > t_max:
            t, r = t_max, t_max / math.tan(half)
        if t < 0.05:
            out.append(b)
            continue
        p0 = (b[0] - ua[0] * t, b[1] - ua[1] * t)
        p1 = (b[0] + uc[0] * t, b[1] + uc[1] * t)
        n = max(2, int(math.ceil(r * turn / step)), int(math.ceil(math.degrees(turn) / 10.0)))
        side = 1.0 if ua[0] * uc[1] - ua[1] * uc[0] > 0 else -1.0
        cx, cy = p0[0] - side * ua[1] * r, p0[1] + side * ua[0] * r
        a0 = math.atan2(p0[1] - cy, p0[0] - cx)
        z = b[2] if len(b) > 2 else None
        for k in range(n + 1):
            ang = a0 + side * turn * k / n
            q = (cx + r * math.cos(ang), cy + r * math.sin(ang))
            out.append(q if z is None else (q[0], q[1], z))
    out.append(shape[-1])
    clean = [out[0]]
    for q in out[1:]:                   # SUMO turns a zero-length segment into a NaN heading
        if math.dist(q[:2], clean[-1][:2]) >= 0.05:
            clean.append(q)
    if len(clean) < 2 or math.dist(clean[-1][:2], shape[-1][:2]) > 1e-6:
        clean = clean[:-1] + [shape[-1]] if len(clean) >= 2 else [shape[0], shape[-1]]
    return clean


def round_lane_corners(root):
    """Rounds the corners of every normal lane's shape (fillet()); the lane keeps its length (SUMO scales positions
    onto the shape), so routes, positions and the bridge's lane mapping are unchanged. Returns how many changed."""
    n = 0
    for edge in root.iter("edge"):
        if edge.get("function") in ("internal", "walkingarea", "crossing"):
            continue
        for ln in edge.iter("lane"):
            shape = _parse_shape(ln.get("shape", ""))
            new = fillet(shape)
            if len(new) != len(shape):
                ln.set("shape", _fmt_shape(new))
                n += 1
    return n


def redraw_junction_lanes(net_path, log=print):
    """Junction (internal) lanes that meet their lanes with a kink get a smooth cubic Bezier shape, a length to
    match and a speed for their tightest bend (TURN_ACC). netconvert draws some of them as straight chords
    (Eindhoven's station junction: a 120 deg right turn as a 37 m straight line, so a bus's front swung round 120
    deg in one step). Edits the XML in place (netconvert would draw them again); returns how many were redrawn."""
    import xml.etree.ElementTree as ET
    net_path = Path(net_path)
    tree = ET.parse(net_path)
    root = tree.getroot()
    rounded = round_lane_corners(root)
    junctions = {j.get("id"): _parse_shape(j.get("shape", "")) for j in root.iter("junction") if j.get("shape")}
    outside = 0
    lanes = {}
    for edge in root.iter("edge"):
        for ln in edge.iter("lane"):
            lanes[ln.get("id")] = ln
    n = 0
    for c in root.iter("connection"):
        via = c.get("via")
        if not via or via not in lanes:
            continue
        lin, lout = lanes.get(f'{c.get("from")}_{c.get("fromLane")}'), lanes.get(f'{c.get("to")}_{c.get("toLane")}')
        if lin is None or lout is None:
            continue
        vs, ins, outs = (_parse_shape(x.get("shape", "")) for x in (lanes[via], lin, lout))
        if len(vs) < 2 or len(ins) < 2 or len(outs) < 2:
            continue
        h_in, h_out = _heading(ins[-2], ins[-1]), _heading(outs[0], outs[1])
        pts = [vs[0]] + [q for a, q in zip(vs, vs[1:]) if math.dist(a[:2], q[:2]) > 0.05]
        hs = [h_in] + [_heading(a, b) for a, b in zip(pts, pts[1:])] + [h_out]
        kink = max(_angle(a, b) for a, b in zip(hs, hs[1:]))       # at the joins and inside
        if kink <= KINK_DEG or math.dist(vs[0][:2], vs[-1][:2]) < 1.0:
            continue
        poly = junctions.get(via[1:].rsplit("_", 2)[0])
        new = None
        for handle in (1.0 / 3.0, 0.2, 0.1):     # a sharp turn's curve bulges: keep it on the junction's area
            cand = bezier(vs[0], (math.cos(h_in), math.sin(h_in)), vs[-1], (math.cos(h_out), math.sin(h_out)),
                          handle=handle)
            if not poly or all(_inside(q, poly, JUNCTION_MARGIN_M) for q in cand):
                new = cand
                break
        if new is None:
            outside += 1
            continue
        ln = lanes[via]
        ln.set("shape", _fmt_shape(new))
        ln.set("length", f"{max(0.1, polyline_length(new)):.2f}")
        r = min_radius([p[:2] for p in new], window=4.0)
        if r is not None:
            v = max(CURVE_MIN_SPEED, math.sqrt(TURN_ACC * r))
            if v < float(ln.get("speed", 99)):
                ln.set("speed", f"{v:.2f}")
        n += 1
    if n or rounded:
        tree.write(net_path, encoding="UTF-8", xml_declaration=True)
    log(f"lanes: {rounded} with rounded corners; junctions: {n} junction lanes redrawn as smooth curves "
        f"(kinks over {KINK_DEG:g} deg), {outside} left as they were (a curve would leave the junction's area)")
    return n


def turnarounds(net_path):
    """The U-turn connections (dir "t") of a net that are not at a dead end: SUMO turns a car round on the spot within
    its own length (Eindhoven rush hour: a car reversed its heading at 33 km/h in front of the camera). Dead ends keep
    theirs, as there is no other way out. netconvert's --no-turnarounds does not remove the connections of a loaded
    net, so they are deleted like the lane shifts."""
    import sumolib
    net = sumolib.net.readNet(str(net_path), withInternal=False, withPedestrianConnections=False)
    out = []
    for e in net.getEdges():
        node = e.getToNode()
        ways_on = [o for o in node.getOutgoing()
                   if o.getToNode() != e.getFromNode() and any(ln.allows("passenger") or ln.allows("bicycle")
                                                               for ln in o.getLanes())]
        if not ways_on:
            continue                      # a dead end: the turnaround is the only way back
        for lane in e.getLanes():
            for c in lane.getOutgoing():
                if c.getDirection() == "t":
                    out.append({"from": e.getID(), "to": c.getTo().getID(), "fromLane": lane.getIndex(),
                                "toLane": c.getToLane().getIndex()})
    return out


def connection_xml(deletes, slowed):
    lines = ["<connections>"]
    for c in deletes:
        lines.append(f'    <delete from="{c["from"]}" to="{c["to"]}" fromLane="{c["fromLane"]}" toLane="{c["toLane"]}"/>')
    for c in slowed:
        lines.append(f'    <connection from="{c["from"]}" to="{c["to"]}" fromLane="{c["fromLane"]}" '
                     f'toLane="{c["toLane"]}" speed="{c["new_speed"]}"/>')
    lines.append("</connections>")
    return "\n".join(lines) + "\n"


def smooth(net_path, extra=(), log=print):
    """Rewrites net_path: sharp lane shifts dropped or slowed (netconvert, with the net's own options `extra`
    applied again), then in the XML: lane corners rounded, kinked junction lanes redrawn, bends slowed (see the
    module doc). Returns the counts."""
    net_path = Path(net_path)
    deletes, slowed = plan(connections(net_path))
    uturns = turnarounds(net_path)
    deletes = deletes + uturns
    with tempfile.TemporaryDirectory() as tmp:
        con = Path(tmp) / "smooth.con.xml"
        con.write_text(connection_xml(deletes, slowed))
        out = Path(tmp) / "smoothed.net.xml"
        subprocess.check_call(["netconvert", "--no-warnings", "--sumo-net-file", str(net_path),
                               "--connection-files", str(con), *extra, "-o", str(out)])
        net_path.write_bytes(out.read_bytes())
    redrawn = redraw_junction_lanes(net_path, log)
    slow = slow_bends(net_path)
    log(f"junctions: {len(deletes) - len(uturns)} sharp lane shifts dropped (a straighter connection to the same road "
        f"exists), {len(slowed)} slowed to stay under {A_LAT:g} m/s^2, {len(uturns)} U-turns away from dead ends "
        f"dropped; {slow} lanes slowed for their bends")
    return {"deleted": len(deletes) - len(uturns), "uturns": len(uturns), "slowed": len(slowed), "curves": slow,
            "redrawn": redrawn}


def slow_bends(net_path):
    """Normal lanes get the speed of their tightest bend (curve_speed, on the shapes as they are now); in place."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(net_path)
    n = 0
    for edge in tree.getroot().iter("edge"):
        if edge.get("function") in ("internal", "walkingarea", "crossing"):
            continue
        for ln in edge.iter("lane"):
            v = curve_speed(min_radius([q[:2] for q in _parse_shape(ln.get("shape", ""))]), float(ln.get("speed", 99)))
            if v is not None:
                ln.set("speed", f"{v:.2f}")
                n += 1
    if n:
        tree.write(net_path, encoding="UTF-8", xml_declaration=True)
    return n
