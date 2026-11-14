"""Designed eHMIs (coupled-sim's eHMI designer, #131): an eHMI from a design file instead of code.

The design (designs.py: a panel of width x height metres, elements per eHMI state) is turned into strokes, thick
line segments in the panel, and the strokes are placed on the car: on its front (hood), rear, roof (both ways),
windshield, as a light bar on the bumper, or projected flat on the road ahead (1.8 m from the bumper on). Debug
lines have no back face: a panel is seen mirrored from behind (roof: read from the front; roof_rear: from behind). The host
draws them into the world with debug lines (ai.draw_ehmi's way, so every station's camera sees them); a station
can draw the same strokes over its own camera image instead (`ehmi.render: station`), which keeps fine detail and
colours exact but ignores occlusion (a building between the car and the camera does not hide it).

Experiment options (all optional; without `design` the eHMI is drawn by ai.draw_ehmi as before):

    ehmi:
      design: light_bar_cyan        # data/ehmi/<name>.json|yaml, a path, or an inline design dict
      placement: [front]            # overrides the design's own placement
      render: world                 # world (debug lines, default) | station (over the camera image) | both
    agents:
      - id: av1
        ehmi_design: text_walk      # per car (and ehmi_placement)

Design states use the eHMI state names (yielding, go, driving, ...); coupled-sim's WALK / STOP / DISABLED work
too (WALK = yielding or go, STOP = driving or wait). State "off" draws nothing.
"""
import math

from . import designs
from .strokefont import GLYPHS

PLACEMENTS = ("front", "hood", "rear", "roof", "roof_rear", "windshield", "bumper", "road")
MAX_THICK = 0.08          # metres: thicker areas are filled with several lines
MIN_LIGHT = 0.08          # strokes dimmer than this are not drawn (a blinking element's off phase)


# -- strokes in the panel -----------------------------------------------------------------------------------------

def _fill(out, x0, y0, w, h, rgb, f, max_thick=MAX_THICK):
    """A filled rectangle as parallel lines along its longer side."""
    if w <= 0 or h <= 0:
        return
    along_u = w >= h
    short = h if along_u else w
    n = max(1, int(math.ceil(short / max_thick - 1e-9)))
    th = short / n
    for k in range(n):
        o = (k + 0.5) * th
        if along_u:
            out.append((x0, y0 + o, x0 + w, y0 + o, th, rgb, f))
        else:
            out.append((x0 + o, y0, x0 + o, y0 + h, th, rgb, f))


def _ring(out, cx, cy, r, th, rgb, f, n=16):
    pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
    for (a, b), (c, d) in zip(pts, pts[1:]):
        out.append((a, b, c, d, th, rgb, f))


def _directed(direction, x0, y0, w, h):
    """Maps (s along the direction 0..1, q across -0.5..0.5) into the box; returns (map, length, breadth)."""
    d = (direction or "right").lower()
    if d == "left":
        return (lambda s, q: (x0 + (1 - s) * w, y0 + h / 2 + q * h)), w, h
    if d == "up":
        return (lambda s, q: (x0 + w / 2 + q * w, y0 + s * h)), h, w
    if d == "down":
        return (lambda s, q: (x0 + w / 2 + q * w, y0 + (1 - s) * h)), h, w
    return (lambda s, q: (x0 + s * w, y0 + h / 2 + q * h)), w, h


def text_strokes(out, text, x0, y0, w, h, rgb, f):
    """Text centred in the box, as large as fits (line font, 5.5 units per letter, 6 units high)."""
    text = str(text).upper()
    if not text:
        return
    unit = min(h * 0.8 / 6.0, w / max(1e-6, 5.5 * len(text) - 1.5))
    th = unit * 0.65
    width = unit * (5.5 * len(text) - 1.5)
    ox, oy = x0 + (w - width) / 2, y0 + (h - 6 * unit) / 2
    for i, ch in enumerate(text):
        bx = ox + i * 5.5 * unit
        for stroke in GLYPHS.get(ch, []):
            pts = [(bx + x * unit, oy + y * unit) for x, y in stroke]
            if len(pts) == 1:
                pts = pts * 2
            for (a, b), (c, d) in zip(pts, pts[1:]):
                out.append((a, b, c, d, th, rgb, f))


def element_strokes(e, W, H, t, values=None, out=None):
    """The strokes of one element at time t (s since its state started): (u0, v0, u1, v1, thickness, rgb,
    brightness) in metres from the panel's bottom left."""
    out = [] if out is None else out
    x0, y0, w, h = e["x"] * W, e["y"] * H, e["w"] * W, e["h"] * H
    r, g, b, a = e["rgba"]
    rgb = (r, g, b)
    alpha = a / 255.0
    typ = e["type"]
    anim, per = e["animation"], e["period"]

    def lit(pos=0.5):
        return designs.brightness(anim, per, t, pos) * alpha

    if typ == "rect":
        _fill(out, x0, y0, w, h, rgb, lit())
    elif typ == "lightbar":
        n = max(1, e["segments"])
        sw = w / n
        for i in range(n):
            _fill(out, x0 + (i + 0.1) * sw, y0, 0.8 * sw, h, rgb, lit((i + 0.5) / n))
    elif typ == "circle":
        rad = min(w, h) / 2
        n = max(1, int(math.ceil(rad / (MAX_THICK / 2))))
        for k in range(n):
            _ring(out, x0 + w / 2, y0 + h / 2, (k + 0.5) * rad / n, rad / n * 1.15, rgb, lit())
    elif typ == "arrow":
        m, length, breadth = _directed(e["direction"], x0, y0, w, h)
        fb = lit()
        a0, a1 = m(0.0, 0.0), m(0.55, 0.0)
        out.append((a0[0], a0[1], a1[0], a1[1], 0.3 * breadth, rgb, fb))
        tip = m(1.0, 0.0)
        k = max(3, int(math.ceil(breadth / MAX_THICK)) * 2)
        for i in range(k + 1):
            q = -0.5 + i / k
            base = m(0.5, q)
            out.append((base[0], base[1], tip[0], tip[1], breadth / k * 1.2, rgb, fb))
    elif typ == "chevrons":
        n = max(1, e["segments"])
        m, length, breadth = _directed(e["direction"], x0, y0, w, h)
        for i in range(n):
            fb = lit((i + 0.5) / n)
            s0, s1 = (i + 0.15) / n, (i + 0.75) / n
            p0, p1, p2 = m(s0, 0.5), m(s1, 0.0), m(s0, -0.5)
            th = 0.18 * length / n
            out.append((p0[0], p0[1], p1[0], p1[1], th, rgb, fb))
            out.append((p1[0], p1[1], p2[0], p2[1], th, rgb, fb))
    elif typ == "text":
        text_strokes(out, designs.fill(e["text"], values or {}), x0, y0, w, h, rgb, lit())
    # gauge, bar, speedsign: in-car displays only (incar.py)
    return out


def strokes(design, state, t, values=None, conds=None):
    """All strokes of a design in a state: its background (when at least half opaque), then the elements of ANY
    and of the state, in order."""
    W, H = design["width"], design["height"]
    out = []
    els = designs.elements_for(design, state)
    if not els and state in (None, "", "off"):
        return out
    bg = design["background"]
    if bg[3] >= 128 and els:
        _fill(out, 0.0, 0.0, W, H, bg[:3], bg[3] / 255.0)
    for e in els:
        if designs.holds(e["when"], conds or {}):
            element_strokes(e, W, H, t, values, out)
    return out


# -- placement on a vehicle ---------------------------------------------------------------------------------------

def frames(placement, loc, yaw_deg, ext, W, H, offset=None, road_gap=1.8):
    """Panel frames (centre, right, up) for a placement on a vehicle at loc (x, y, z of its origin, at road level)
    heading yaw_deg, with bounding-box half extents ext (x, y, z). `right` is the panel's +u as its reader sees it,
    `up` its +v."""
    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -fy, fx                         # the car's right (CARLA: y right of x, seen from above)
    ex, ey, ez = ext
    of, orr, ou = (list(offset) + [0, 0, 0])[:3] if offset else (0.0, 0.0, 0.0)

    def at(fwd, right, z):
        return (loc[0] + fx * (fwd + of) + rx * (right + orr), loc[1] + fy * (fwd + of) + ry * (right + orr),
                loc[2] + z + ou)

    up = (0.0, 0.0, 1.0)
    face_front = (-rx, -ry, 0.0)             # reader in front of the car, looking at it: their right = car's left
    face_rear = (rx, ry, 0.0)
    p = (placement or "front").lower()
    if p in ("front", "hood"):
        return [(at(ex + 0.06, 0.0, ez * 1.05), face_front, up)]
    if p == "bumper":
        return [(at(ex + 0.08, 0.0, 0.45), face_front, up)]
    if p == "windshield":
        return [(at(ex * 0.25, 0.0, ez * 1.55), face_front, up)]
    if p == "rear":
        return [(at(-(ex + 0.06), 0.0, ez * 1.05), face_rear, up)]
    if p in ("roof", "roof_rear"):      # a sign on the roof, read from the front (roof_rear: from behind); debug
        z = 2 * ez + 0.1 + H / 2        # lines have no back face, so each is seen mirrored from the other side
        return [(at(0.0, 0.0, z), face_front if p == "roof" else face_rear, up)]
    if p == "road":                          # flat on the road; its top (+v) towards the car
        return [(at(ex + road_gap + H / 2, 0.0, 0.06), face_front, (-fx, -fy, 0.0))]
    raise ValueError(f"unknown eHMI placement {placement!r} (one of {', '.join(PLACEMENTS)})")


def to_world(strokes_, frame, W, H):
    """Panel strokes in a frame -> world segments (p0, p1, thickness, rgb, brightness)."""
    (cx, cy, cz), (rx, ry, rz), (ux, uy, uz) = frame
    out = []
    for u0, v0, u1, v1, th, rgb, f in strokes_:
        a, b, c, d = u0 - W / 2, v0 - H / 2, u1 - W / 2, v1 - H / 2
        p0 = (cx + rx * a + ux * b, cy + ry * a + uy * b, cz + rz * a + uz * b)
        p1 = (cx + rx * c + ux * d, cy + ry * c + uy * d, cz + rz * c + uz * d)
        out.append((p0, p1, th, rgb, f))
    return out


class DesignedEhmi:
    """One car's designed eHMI: remembers when its state started (animations run from there) and makes its world
    segments."""

    def __init__(self, design, placement=None, offset=None):
        self.design = design
        pl = placement or design.get("placement") or ["front"]
        self.placements = [pl] if isinstance(pl, str) else list(pl)
        self.offset = offset if offset is not None else design.get("offset")
        self.state, self.since = None, 0.0

    def segments(self, state, t, loc, yaw, ext, values=None):
        if state != self.state:
            self.state, self.since = state, t
        if state in (None, "", "off"):
            return []
        W, H = self.design["width"], self.design["height"]
        st = strokes(self.design, state, t - self.since, values)
        out = []
        for pl in self.placements:
            for fr in frames(pl, loc, yaw, ext, W, H, self.offset):
                out += to_world(st, fr, W, H)
        return out


def draw_world(debug, vehicle, state, ehmi, t, life):
    """Draws a car's designed eHMI with debug lines (host side)."""
    import carla
    tf = vehicle.get_transform()
    e = vehicle.bounding_box.extent
    segs = ehmi.segments(state, t, (tf.location.x, tf.location.y, tf.location.z), tf.rotation.yaw, (e.x, e.y, e.z))
    for p0, p1, th, rgb, f in segs:
        if f < MIN_LIGHT:
            continue
        col = carla.Color(*(int(min(255, c * f)) for c in rgb))
        debug.draw_line(carla.Location(*p0), carla.Location(*p1), thickness=th, color=col, life_time=life)
    return len(segs)


def for_agents(tc, base=None, log=print):
    """{agent id: {design, placement, offset}} for the scripted cars with a designed eHMI (`ehmi.design` for every
    car, `agents[].ehmi_design` per car); a design that cannot be read is logged and the car keeps ai.draw_ehmi."""
    e = tc.get("ehmi") or {}
    out = {}
    for a in tc.get("agents") or []:
        if a.get("kind", "car") != "car":
            continue
        name = a.get("ehmi_design", e.get("design"))
        if not name:
            continue
        try:
            d = designs.load(name, "ehmi", base)
        except (OSError, ValueError) as ex:
            log(f"eHMI design for {a.get('id')}: {ex}; drawing the plain eHMI")
            continue
        out[a["id"]] = {"design": d, "placement": a.get("ehmi_placement", e.get("placement")),
                        "offset": a.get("ehmi_offset", e.get("offset"))}
    return out


def drawers(specs):
    return {aid: DesignedEhmi(s["design"], s.get("placement"), s.get("offset")) for aid, s in specs.items()}


# -- drawn by a station over its camera image ------------------------------------------------------------------------

def project(point, cam, size, fov):
    """Pixel (u, v) and depth of a world point seen by a camera at cam = (x, y, z, pitch, yaw, roll) degrees (CARLA:
    x forward, y right, z up); None behind the camera."""
    x, y, z = point[0] - cam[0], point[1] - cam[1], point[2] - cam[2]
    p, yw, r = (math.radians(a) for a in cam[3:6])
    # world -> camera: undo yaw (z), pitch (y), roll (x)
    cy_, sy_ = math.cos(yw), math.sin(yw)
    x, y = x * cy_ + y * sy_, -x * sy_ + y * cy_
    cp, sp = math.cos(p), math.sin(p)
    x, z = x * cp + z * sp, -x * sp + z * cp
    cr, sr = math.cos(r), math.sin(r)
    y, z = y * cr + z * sr, -y * sr + z * cr
    if x < 0.2:
        return None
    f = size[0] / (2.0 * math.tan(math.radians(fov) / 2.0))
    return size[0] / 2 + f * y / x, size[1] / 2 - f * z / x, x


def project_segments(segs, cam, size, fov):
    """World segments -> screen segments (u0, v0, u1, v1, width px, rgb, brightness), nearest last."""
    f = size[0] / (2.0 * math.tan(math.radians(fov) / 2.0))
    out = []
    for p0, p1, th, rgb, b in segs:
        if b < MIN_LIGHT:
            continue
        a, c = project(p0, cam, size, fov), project(p1, cam, size, fov)
        if a is None or c is None:
            continue
        depth = (a[2] + c[2]) / 2
        out.append((depth, (a[0], a[1], c[0], c[1], max(1, int(round(f * th / depth))), rgb, b)))
    out.sort(key=lambda x: -x[0])
    return [s for _d, s in out]
