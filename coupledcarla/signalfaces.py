"""What a lit signal lens looks like: a round lens, or a lit pictogram inside it (no CARLA, no SUMO here).

signals.py draws the lit aspect of every head with CARLA's debug lines (the head meshes are static props that cannot
change their material at run time). CARLA's debug drawing has lines, points, boxes, arrows and text, but no filled
disc, and a thick line is drawn as a flat quad: the line's length one way, its thickness the other. Until
2026-11-28 the lit aspect was one such line as long as it was thick, a 0.24 m square in front of a 300 mm lens: on
the Eindhoven twin clip a flat bright-green rectangle with the visor's black edges cut into its corners (the
user: "traffic lights show an unrealistic rectangle for the lights"). Now:

- a plain lens (car heads) is lit as a round disc: DISC_ANGLES give the union of six thick lines across the face,
  rectangles whose corners lie on the lens's circle (a 24-cornered disc, LIT_FILL of the lens's diameter, inside
  the visor), so the lit aspect is as round and as big as the lens (disc_lines);
- a pictogram lens (Dutch cyclist heads: a bicycle; pedestrian heads: a standing red and a walking green figure)
  lights only its pictogram, in thick strokes on the dark lens, as the real heads' LEDs behind a black mask do
  (pictogram_lines). The head meshes (tools/street_props.py) carry the same pictograms (PICTOGRAMS) on a black mask
  disc in the lens's colour, so the unlit lens shows the faint symbol and the lit one the bright symbol on top of it.

Pictograms are in units of the lens's radius: x to the viewer's right (the head's local +x: the face looks along its
local +y), z up, the lens's centre at (0, 0). A primitive is ("seg", (x0, z0), (x1, z1), width) or
("dot", (x, z), radius).
"""
import math

DISC_ANGLES = (13.0, 26.0, 39.0, 51.0, 64.0, 77.0)  # the lit disc's rectangles' corners (deg from horizontal)
LIT_FILL = 0.92         # the lit disc's diameter / the lens's (the lens's rim and visor stay dark)
PICTO_FILL = 0.82       # a pictogram's unit circle / the lens's radius (inside the black mask's rim)
MASK_FRONT = 0.003      # the head mesh's black mask in front of its lens (m), its pictogram 1 mm further
LIT_FRONT = 0.015       # a lit disc or pictogram this far in front of the lens (m): in front of the mask, behind
                        # the visor's edge (16 cm out)


def _ring(cx, cz, r, n, w):
    pts = [(cx + r * math.cos(2 * math.pi * k / n), cz + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
    return [("seg", a, b, w) for a, b in zip(pts, pts[1:])]


# the Dutch cyclist light's bicycle (RVV 1990 art. 72 / BABW: a bicycle symbol in the lens), riding to the right
_R, _F, _B, _S, _H = (-0.46, -0.28), (0.46, -0.28), (-0.06, -0.28), (-0.22, 0.14), (0.30, 0.14)
BICYCLE = (_ring(*_R, 0.30, 8, 0.10) + _ring(*_F, 0.30, 8, 0.10) +
           [("seg", _R, _B, 0.10), ("seg", _R, _S, 0.10), ("seg", _B, _S, 0.10), ("seg", _S, _H, 0.10),
            ("seg", _B, _H, 0.10), ("seg", _H, _F, 0.10), ("seg", (0.24, 0.30), (0.42, 0.30), 0.10),
            ("seg", _H, (0.30, 0.30), 0.10), ("seg", (-0.34, 0.22), (-0.10, 0.22), 0.11)])
# pedestrian heads (BABW: a standing red figure, a walking green one)
PED_STAND = [("dot", (0.0, 0.62), 0.14), ("seg", (0.0, 0.44), (0.0, -0.12), 0.30),
             ("seg", (-0.08, -0.12), (-0.09, -0.80), 0.13), ("seg", (0.08, -0.12), (0.09, -0.80), 0.13),
             ("seg", (-0.21, 0.40), (-0.23, -0.10), 0.10), ("seg", (0.21, 0.40), (0.23, -0.10), 0.10)]
PED_WALK = [("dot", (0.06, 0.62), 0.14), ("seg", (0.04, 0.44), (-0.02, -0.10), 0.26),
            ("seg", (-0.02, -0.10), (-0.36, -0.80), 0.13), ("seg", (-0.02, -0.10), (0.30, -0.80), 0.13),
            ("seg", (0.02, 0.38), (-0.32, -0.02), 0.10), ("seg", (0.04, 0.38), (0.34, 0.04), 0.10)]
PICTOGRAMS = {"bike": BICYCLE, "ped_stand": PED_STAND, "ped_walk": PED_WALK}


def faces_for(mode, n_lenses):
    """The pictogram of each lens of a head (None: a plain lens): cyclist heads a bicycle in every lens, pedestrian
    heads (two lenses) a standing then a walking figure, car heads none."""
    if mode == "bike":
        return ["bike"] * n_lenses
    if mode == "ped" and n_lenses == 2:
        return ["ped_stand", "ped_walk"]
    return [None] * n_lenses


def disc_rects(radius, angles=DISC_ANGLES):
    """[(half width, half height)] of the rectangles whose union is the lit disc (their corners on the circle)."""
    return [(radius * math.cos(math.radians(a)), radius * math.sin(math.radians(a))) for a in angles]


def _face(centre, yaw):
    """A function (u, v) -> world (x, y, z) on a head's face: u along the face (its local +x), v up."""
    r = math.radians(yaw)
    ux, uy = math.cos(r), math.sin(r)
    x, y, z = centre
    return lambda u, v: (x + u * ux, y + u * uy, z + v)


def disc_lines(centre, yaw, diameter, angles=DISC_ANGLES):
    """The debug lines [(a, b, thickness)] that draw a lit round lens of `diameter` (m) centred at `centre` on a
    face turned to `yaw` (the head mesh's yaw: its local +x along the face): horizontal lines across the face, each
    as thick as its rectangle is tall."""
    at = _face(centre, yaw)
    return [(at(-w, 0.0), at(w, 0.0), 2 * h) for w, h in disc_rects(diameter / 2.0, angles)]


def pictogram_lines(centre, yaw, diameter, key, fill=PICTO_FILL):
    """The debug lines [(a, b, thickness)] that draw pictogram `key` lit inside a lens of `diameter` (m)."""
    s = diameter / 2.0 * fill
    at = _face(centre, yaw)
    out = []
    for prim in PICTOGRAMS[key]:
        if prim[0] == "seg":
            (x0, z0), (x1, z1), w = prim[1], prim[2], prim[3]
            out.append((at(x0 * s, z0 * s), at(x1 * s, z1 * s), w * s))
        else:
            (cx, cz), r = prim[1], prim[2]
            for w, h in disc_rects(r * s, (30.0, 60.0)):
                out.append((at(cx * s - w, cz * s), at(cx * s + w, cz * s), 2 * h))
    return out


def lit_lines(centre, yaw, diameter, face=None):
    """The lit aspect of a lens: its pictogram's strokes, or (face None) a round disc filling LIT_FILL of it."""
    if face:
        return pictogram_lines(centre, yaw, diameter, face)
    return disc_lines(centre, yaw, diameter * LIT_FILL)


def covered(lines, centre, yaw, step=0.004):
    """For tests and tools/signal_faces checks: the set of grid cells (u, v indices, `step` m) on the face that the
    debug lines cover, each line drawn as the quad its thickness gives (across the line, in the face's plane)."""
    r = math.radians(yaw)
    ux, uy = math.cos(r), math.sin(r)
    cx, cy, cz = centre

    def uv(p):
        return (p[0] - cx) * ux + (p[1] - cy) * uy, p[2] - cz
    cells = set()
    for a, b, t in lines:
        (u0, v0), (u1, v1) = uv(a), uv(b)
        du, dv = u1 - u0, v1 - v0
        L = math.hypot(du, dv) or 1e-9
        nu, nv = -dv / L, du / L
        umin = min(u0, u1) - t / 2
        umax = max(u0, u1) + t / 2
        vmin = min(v0, v1) - t / 2
        vmax = max(v0, v1) + t / 2
        i0, i1 = int(math.floor(umin / step)), int(math.ceil(umax / step))
        j0, j1 = int(math.floor(vmin / step)), int(math.ceil(vmax / step))
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                pu, pv = (i + 0.5) * step, (j + 0.5) * step
                along = ((pu - u0) * du + (pv - v0) * dv) / L
                across = (pu - u0) * nu + (pv - v0) * nv
                if 0.0 <= along <= L and abs(across) <= t / 2:
                    cells.add((i, j))
    return cells
