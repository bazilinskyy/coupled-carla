"""What the occupants of a car see of its own eHMI (`ehmi.inside`), and their own indicator of it (`ehmi.indicator`).

An eHMI is meant for the people outside. Its panels on the front, hood, windshield and roof face away from the car,
and CARLA's debug lines have no back face, so from the passenger seat the AV's own message floated over the hood in
mirror writing ("DAEHA OG"). A real display seen from behind shows its housing (an LED panel, e-paper) or, if it is
see-through (a film on the windscreen, a projector), its message mirrored and dim. Debug lines are drawn for every
camera alike (measured on CARLA 0.9.16: a dark backing drawn 4 cm behind the text hid the text from the front as well,
whatever the drawing order, so lines cannot give a panel a back), so a car whose occupant has a station gets those
panels drawn by the stations instead: every station draws them over its camera image as the world would (as
`ehmi.render: station` does; occlusion by buildings is not applied), and the occupant's own station draws them by
`ehmi.inside`:

    hide    (default) not at all: the back of an opaque display, out of the occupant's line of sight
    dim     mirrored at a quarter of the light: a see-through display from behind
    mirror  as before: the host draws the whole eHMI into the world and everyone sees it, mirrored from behind

Panels that face elsewhere stay in the world for every camera (rear, roof_rear, bumper, the light bars, the eyes,
the road projection, which the occupants see the right way up as a pedestrian's mirror image, as in reality).

`ehmi.indicator` (default on): an AV passenger's station shows what the car shows outside on a small strip at the top
of the view, as a robotaxi's roof console does, the right way round, in the eHMI's own design and colours.

Pure functions here; run.py (host) and stationux.py (stations) call them. Same on CARLA 0.9.16 and 1.0: the stations
draw with pygame over their own images.
"""
import math

from . import designs, ehmi_design
from .strokefont import GLYPHS

MODES = ("hide", "dim", "mirror")
DIM = 0.25                                                   # dim: share of the light seen through the back
CABIN_PLACEMENTS = ("front", "hood", "windshield", "roof")   # designed panels the front seats look at from behind
CABIN_SLOTS = ("front", "roof")                              # ai.draw_ehmi's slots whose text they see mirrored


def settings(tc):
    """(mode, indicator) of an experiment; an unknown mode is read as hide."""
    e = tc.get("ehmi") or {}
    mode = str(e.get("inside", "hide")).lower()
    return (mode if mode in MODES else "hide"), bool(e.get("indicator", True))


def occupied(tc):
    """Ids of the scripted cars an AV passenger with a station rides in."""
    return {p.get("agent") for p in tc.get("participants") or []
            if p.get("kind") == "av_passenger" and p.get("station") and p.get("agent")}


def split(placements):
    """(world, cabin) placements of a designed eHMI: the panels the occupants would see from behind."""
    pl = [placements] if isinstance(placements, str) else list(placements or ["front"])
    return [p for p in pl if p not in CABIN_PLACEMENTS], [p for p in pl if p in CABIN_PLACEMENTS]


def world_slots(slots):
    """The plain eHMI's slots whose text the host still draws into the world for an occupied car."""
    return tuple(s for s in slots if s not in CABIN_SLOTS)


def host_plan(tc, specs, slots_of, mode=None):
    """What the host leaves to the stations: {agent id: {"placement": [cabin placements]} for a designed eHMI drawn
    in the world, or {"slots": [cabin slots]} for the plain one}. specs: ehmi_design.for_agents(); slots_of(agent
    cfg) -> its plain slots. Empty when mode is mirror or no car is occupied."""
    mode = mode or settings(tc)[0]
    if mode == "mirror":
        return {}
    render = (tc.get("ehmi") or {}).get("render", "world")
    out = {}
    for a in tc.get("agents") or []:
        aid = a.get("id")
        if aid not in occupied(tc) or a.get("kind", "car") != "car":
            continue
        if aid in specs:
            pl = specs[aid].get("placement") or specs[aid]["design"].get("placement") or ["front"]
            cabin = split(pl)[1]
            if cabin and render == "world":      # station / both: the stations draw every panel anyway
                out[aid] = {"placement": cabin}
        else:
            cabin = [s for s in slots_of(a) if s in CABIN_SLOTS]
            if cabin:
                out[aid] = {"slots": cabin}
    return out


def welcome(tc, specs, slots_of, states, pcfg=None, kind=None):
    """The `ehmi_inside` entry of a station's welcome: mode, indicator, the cars whose cabin panels the stations
    draw (with their design or plain states) and, for an AV passenger, its own car's eHMI for the indicator."""
    mode, indicator = settings(tc)
    plan = host_plan(tc, specs, slots_of, mode)
    plain_states = {k: {"text": v.get("text", ""), "color": list(v.get("color", [0, 0, 0]))}
                    for k, v in (states or {}).items()}
    cars = {}
    for aid, what in plan.items():
        if "placement" in what:
            s = specs[aid]
            cars[aid] = {"design": s["design"], "placement": what["placement"], "offset": s.get("offset")}
        else:
            cars[aid] = {"plain": plain_states, "slots": what["slots"]}
    out = {"mode": mode, "indicator": indicator, "cars": cars}
    own = (pcfg or {}).get("agent") if kind == "av_passenger" else None
    if own and indicator:
        out["own"] = {"agent": own, "design": specs[own]["design"]} if own in specs else \
            {"agent": own, "plain": plain_states}
    return out


# -- the plain eHMI's text (ai.draw_ehmi) as segments ---------------------------------------------------------------

def _text_segments(centre, along, up, text, height, rgb, thick):
    """ai.stroke_text's lines as world segments (p0, p1, thickness, rgb, brightness)."""
    unit = height / 6.0
    advance = 5.5 * unit
    width = advance * len(text) - 1.5 * unit
    ox = [centre[i] - along[i] * (width / 2) - up[i] * (height / 2) for i in range(3)]
    out = []
    for k, ch in enumerate(str(text).upper()):
        base = [ox[i] + along[i] * (k * advance) for i in range(3)]
        for stroke in GLYPHS.get(ch, []):
            pts = [tuple(base[i] + along[i] * (x * unit) + up[i] * (y * unit) for i in range(3)) for x, y in stroke]
            for a, b in zip(pts, pts[1:]):
                out.append((a, b, thick, tuple(rgb), 1.0))
    return out


def plain_segments(state, states, slots, loc, yaw_deg, ext):
    """The text of ai.draw_ehmi's front/rear/roof slots for a car at loc (its origin, road level) heading yaw_deg with
    bounding-box half extents ext, as world segments: where draw_ehmi puts it, in the state's colour (draw_ehmi's
    debug lines glow; drawn over an image, the colour itself matches them best)."""
    spec = (states or {}).get(state) or {}
    text, rgb = spec.get("text", ""), list(spec.get("color", [0, 0, 0]))
    if not state or state == "off" or not text or rgb == [0, 0, 0]:
        return []
    dim = [int(c) for c in rgb]
    y = math.radians(yaw_deg)
    fwd, right = (math.cos(y), math.sin(y), 0.0), (-math.sin(y), math.cos(y), 0.0)
    up = (0.0, 0.0, 1.0)
    ex, ey, ez = ext

    def facing(d):           # letters advance to the right of a reader looking along -d (at the car)
        return (d[1], -d[0], 0.0)

    def at(*terms):
        return tuple(loc[i] + sum(k * v[i] for k, v in terms) for i in range(3))

    out = []
    for slot in slots:
        if slot in ("front", "rear"):
            d = fwd if slot == "front" else tuple(-c for c in fwd)
            c = at((ex + 0.06 - 0.05, d), (ez * 1.05 + 0.42, up))
            out += _text_segments(c, facing(d), up, text, 0.32, dim, 0.035)
        elif slot == "roof":
            neg = lambda v: tuple(-c for c in v)  # noqa: E731
            for d, half in ((fwd, ex * 0.45), (neg(fwd), ex * 0.45), (right, ey * 0.85), (neg(right), ey * 0.85)):
                c = at((half + 0.02, d), (2 * ez + 0.08 + 0.22, up))
                out += _text_segments(c, facing(d), up, text, 0.22, dim, 0.03)
    return out


class PlainText:
    """A station-side drawer of the plain eHMI's cabin slots, with DesignedEhmi's segments() interface."""

    def __init__(self, states, slots):
        self.states, self.slots = states, list(slots)

    def segments(self, state, t, loc, yaw, ext, values=None):
        return plain_segments(state, self.states, self.slots, loc, yaw, ext)


def drawer(spec):
    """The station-side drawer for one entry of welcome()["cars"]."""
    if spec.get("design") is not None:
        return ehmi_design.DesignedEhmi(designs.normalise(spec["design"]), spec.get("placement"), spec.get("offset"))
    return PlainText(spec.get("plain") or {}, spec.get("slots") or [])


def for_occupant(segs, mode):
    """The occupant's own car's cabin panels as its station draws them: none (hide), a quarter of the light (dim)
    or unchanged (mirror)."""
    if mode == "hide":
        return []
    if mode == "dim":
        return [(p0, p1, th, rgb, b * DIM) for p0, p1, th, rgb, b in segs]
    return list(segs)


# -- the occupant's indicator ------------------------------------------------------------------------------------

def plain_design(states):
    """A design for the indicator of a plain eHMI: the state's light bar and its text, in its colour."""
    st = {}
    for name, spec in (states or {}).items():
        rgb = list(spec.get("color", [0, 0, 0]))
        if name == "off" or rgb == [0, 0, 0]:
            continue
        col = "#%02X%02X%02X" % tuple(int(c) for c in rgb[:3])
        els = [{"type": "lightbar", "segments": 12, "color": col, "x": 0.0, "y": 0.0, "w": 1.0, "h": 0.2}]
        if spec.get("text"):
            els.append({"type": "text", "text": spec["text"], "color": col, "x": 0.0, "y": 0.32, "w": 1.0, "h": 0.6})
        st[name] = els
    return designs.normalise({"name": "indicator", "width": 1.4, "height": 0.42, "background": "#00000000",
                              "states": st})


def indicator_rect(size, design, width_share=0.26, top_share=0.03):
    """(x, y, w, h) in pixels of the indicator strip: top centre, width_share of the view, the design's aspect."""
    w = size[0] * width_share
    h = w * float(design["height"]) / max(1e-6, float(design["width"]))
    return (size[0] - w) / 2.0, size[1] * top_share, w, h


def indicator_segments(design, state, t, rect, pad=0.08):
    """The design's strokes in a state, fitted into the pixel rect (x, y, w, h) the right way round: screen segments
    (u0, v0, u1, v1, width px, rgb, brightness) as ehmi_design.project_segments gives them."""
    W, H = float(design["width"]), float(design["height"])
    x, y, w, h = rect
    s = min(w * (1 - pad) / W, h * (1 - pad) / H)
    ox, oy = x + (w - W * s) / 2.0, y + (h - H * s) / 2.0
    out = []
    for u0, v0, u1, v1, th, rgb, b in ehmi_design.strokes(design, state, t):
        if b < ehmi_design.MIN_LIGHT:
            continue
        out.append((ox + u0 * s, oy + (H - v0) * s, ox + u1 * s, oy + (H - v1) * s, max(1, int(round(th * s))),
                    tuple(rgb), b))
    return out
