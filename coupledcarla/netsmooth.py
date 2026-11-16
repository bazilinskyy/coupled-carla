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
NET_ARGS = ["--junctions.limit-turn-speed", f"{TURN_ACC:g}"]


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
    """Rewrites net_path without its sharp lane shifts, with sharp bends and S-bends slowed (see the module doc);
    returns the counts. extra: the net's own netconvert options (applied again when netconvert rewrites it)."""
    net_path = Path(net_path)
    deletes, slowed = plan(connections(net_path))
    slow = curves(net_path)
    with tempfile.TemporaryDirectory() as tmp:
        con, edg = Path(tmp) / "smooth.con.xml", Path(tmp) / "smooth.edg.xml"
        con.write_text(connection_xml(deletes, slowed))
        edg.write_text(edge_xml(slow))
        out = Path(tmp) / "smoothed.net.xml"
        subprocess.check_call(["netconvert", "--no-warnings", "--sumo-net-file", str(net_path),
                               "--connection-files", str(con), "--edge-files", str(edg), *extra, "-o", str(out)])
        net_path.write_bytes(out.read_bytes())
    log(f"junctions: {len(deletes)} sharp lane shifts dropped (a straighter connection to the same road exists), "
        f"{len(slowed)} slowed to stay under {A_LAT:g} m/s^2; {len(slow)} lanes slowed for their bends")
    return {"deleted": len(deletes), "slowed": len(slowed), "curves": len(slow)}
