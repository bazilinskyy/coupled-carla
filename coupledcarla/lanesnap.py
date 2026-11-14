"""The point traci.simulation.convertRoad + convert2D give for an x, y, worked out in Python.

The bridge keeps each CARLA-driven vehicle within a few decimetres of a SUMO lane (sumo_simulation.LANE_SNAP_M)
with convertRoad + convert2D every tick. SUMO answers convertRoad by measuring every lane within 1 km of the point
(libsumo Helper::convertCartesianToRoadMap): 1.6 ms a call on the Eindhoven twin's 4,700 lanes, the largest part
of the bridge's own time per tick. LaneIndex follows the same steps (SUMO 1.28's code) over a grid of the lanes'
segments: lanes the vehicle class may use, internal (junction) lanes included; distance to the centre line, less
half the lane width where the point lies beside a segment (patchShapeDistance); the smallest lane id on a tie (so
of all lanes the point lies on, the smallest id); then that lane's nearest centre-line point as convertRoad's lane
position and convert2D turn it into one (2D offset along the shape, walked again in 3D).
"""
import math
import xml.etree.ElementTree as ET

POSITION_EPS = 0.1     # SUMO's, the least shape length in MSLane's length/geometry factor
ASK = object()         # nearest_centre's answer when only SUMO can tell (a lane end, to the last bit)


def _allowed(allow, disallow, vclass):
    """SUMO's lane permissions for one class from the net's allow / disallow attributes."""
    if allow is None and disallow is None:
        return True
    if disallow is None:
        words = allow.split()
        return vclass in words or "all" in words
    return disallow != "all" and vclass not in disallow.split()


def _seg_offset(a, b, px, py, perpendicular):
    """GeomHelper::nearest_offset_on_line_to_point2D (None for INVALID_OFFSET)."""
    length = math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))
    if length == 0.0:
        return 0.0
    u = ((px - a[0]) * (b[0] - a[0]) + (py - a[1]) * (b[1] - a[1])) / length
    if u < 0.0 or u > length:
        if perpendicular:
            return None
        return 0.0 if u < 0.0 else length
    return u


def _len2(a, b):
    return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))


def _len3(a, b):
    return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]))


def _at_offset(shape, pos, dist):
    """PositionVector::positionAtOffset(2D) (dist = _len2 or _len3), x and y only."""
    if len(shape) == 1:
        return shape[0][:2]
    seen = 0.0
    for a, b in zip(shape, shape[1:]):
        nxt = dist(a, b)
        if seen + nxt > pos:
            p = pos - seen
            if p == 0.0:
                return a[:2]
            f = p / nxt
            return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        seen += nxt
    return shape[-1][:2]


def _nearest_offset(shape, px, py, perpendicular):
    """PositionVector::nearest_offset_to_point2D (None for INVALID_OFFSET)."""
    best, best_d2, seen = None, math.inf, 0.0
    for k in range(len(shape) - 1):
        a, b = shape[k], shape[k + 1]
        u = _seg_offset(a, b, px, py, perpendicular)
        if u is not None:
            length = _len2(a, b)
            if u == 0.0 or length == 0.0:
                qx, qy = a[0], a[1]
            else:
                f = u / length
                qx, qy = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
            d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy)
            if d2 < best_d2:
                best, best_d2 = u + seen, d2
        elif k > 0:   # perpendicular: the corner of a bend can still be nearest
            d2 = (px - a[0]) * (px - a[0]) + (py - a[1]) * (py - a[1])
            if d2 < best_d2:
                z = shape[k - 1]
                if _seg_offset(z, a, px, py, False) == _len2(z, a) and _seg_offset(a, b, px, py, False) == 0.0:
                    best, best_d2 = seen, d2
        seen += _len2(a, b)
    return best


class LaneIndex:
    def __init__(self, net_file, vclass="passenger", cell=20.0):
        self.cell = cell
        self.lanes = []          # (id, [(x, y, z)], width, geometry/length factor, length)
        self.boxes = []          # (min x, min y, max x, max y, half width)
        self.grid = {}           # (i, j) -> {lane index}
        self.max_half_width = 0.0
        for _, el in ET.iterparse(str(net_file)):
            if el.tag != "lane":
                continue
            a = el.attrib
            if a.get("shape") and _allowed(a.get("allow"), a.get("disallow"), vclass):
                shape = []
                for p in a["shape"].split():
                    v = [float(c) for c in p.split(",")]
                    shape.append((v[0], v[1], v[2] if len(v) > 2 else 0.0))
                width = float(a.get("width", 3.2))
                length3 = sum(_len3(p, q) for p, q in zip(shape, shape[1:]))
                length = float(a["length"])
                n = len(self.lanes)
                self.lanes.append((a["id"], shape, width, max(POSITION_EPS, length3) / length, length))
                self.boxes.append((min(p[0] for p in shape), min(p[1] for p in shape),
                                   max(p[0] for p in shape), max(p[1] for p in shape), width / 2))
                self.max_half_width = max(self.max_half_width, width / 2)
                for p, q in zip(shape, shape[1:] or shape):
                    for i in range(math.floor(min(p[0], q[0]) / cell), math.floor(max(p[0], q[0]) / cell) + 1):
                        for j in range(math.floor(min(p[1], q[1]) / cell), math.floor(max(p[1], q[1]) / cell) + 1):
                            self.grid.setdefault((i, j), set()).add(n)
            el.clear()

    def _distance(self, n, px, py):
        """Helper::convertCartesianToRoadMap's distance of lane n to the point."""
        _, shape, width, _, _ = self.lanes[n]
        if len(shape) == 1:
            return _len2(shape[0], (px, py))
        x, y = _at_offset(shape, _nearest_offset(shape, px, py, False), _len2)
        d = math.sqrt((px - x) * (px - x) + (py - y) * (py - y))
        if _nearest_offset(shape, px, py, True) is not None:      # patchShapeDistance
            d = max(0.0, d - width * 0.5)
        return d

    def lane_at(self, x, y):
        """Index of the lane convertRoad picks for x, y (None: no lane for the class)."""
        cell = self.cell
        ci, cj = math.floor(x / cell), math.floor(y / cell)
        best, seen, ring = None, set(), 0
        while len(seen) < len(self.lanes):
            for i in range(ci - ring, ci + ring + 1):
                for j in range(cj - ring, cj + ring + 1):
                    if max(abs(i - ci), abs(j - cj)) != ring:
                        continue
                    for n in self.grid.get((i, j), ()):
                        if n in seen:
                            continue
                        seen.add(n)
                        if best is not None:     # no nearer than its box, less half its width: skip if farther
                            x0, y0, x1, y1, hw = self.boxes[n]
                            dx, dy = max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1)
                            if math.sqrt(dx * dx + dy * dy) - hw > best[0][0]:
                                continue
                        key = (self._distance(n, x, y), self.lanes[n][0])
                        if best is None or key < best[0]:
                            best = (key, n)
            # every lane not met yet lies outside the cells searched, so at least `margin` from the point (less
            # half its width once patched): none can be nearer, nor tie
            margin = min(x - (ci - ring) * cell, (ci + ring + 1) * cell - x, y - (cj - ring) * cell,
                         (cj + ring + 1) * cell - y)
            if best is not None and best[0][0] < margin - self.max_half_width:
                break
            ring += 1
        return None if best is None else best[1]

    def nearest_centre(self, x, y):
        """(x, y) that convert2D(*convertRoad(x, y, False, vclass)) returns; None where SUMO finds no lane or
        convert2D refuses convertRoad's position; ASK at a lane's very end, where SUMO's rounding decides whether
        convert2D refuses it (on random points SUMO's and this rounding differed in 2 of 56,000)."""
        n = self.lane_at(x, y)
        if n is None:
            return None
        _, shape, _, factor, length = self.lanes[n]
        offset = _nearest_offset(shape, x, y, False) if len(shape) > 1 else 0.0
        pos = offset / factor            # convertRoad's lane position
        if abs(pos - length) < 1e-9:
            return ASK
        if pos < 0 or pos > length:      # convert2D: "Position on lane invalid"
            return None
        return _at_offset(shape, pos * factor, _len3)
