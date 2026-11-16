"""Regional street furniture for CARLA 1.0 as glTF meshes, made with numpy from real dimensions (no third-party
model): the Dutch set first (lamp posts by the Eindhoven register's mast types, bollards incl. the
"Amsterdammertje", the "Model Eindhoven" bench, Bammens-type waste bins, bike stands ("nietjes"), Dutch traffic
light heads with yellow back plates on black-and-white poles, shark-teeth give-way markings, red cycle-path
asphalt), small UK and Japanese sets, and region-free road wear (cracks, repair patches, potholes, oil spots,
autumn leaves) and rooftop clutter (air-conditioning units, stair bulkheads, vents).

    python tools/street_props.py OUT_DIR       # SM_CC_*.glb, T_CC_*.png (decal textures), street_props.json
    python tools/street_props.py OUT_DIR --data     # also coupledcarla/data/street_props.json and the meshes'
                                                    # entries in coupledcarla/data/meshes_1.0.json

Conventions (as tools/drone_mesh.py, whose primitives and glTF writer this uses): model x forward, y left, z up,
metres, origin at the foot (ground) centre; Unreal sees (x, -y, z) * 100 after the glTF import. Per kind:
  lamps          the arm (and the light) towards +x, as CARLA's own lamps (scenery LAMP_YAW 0)
  benches, bins  the seat / the bin's mouth towards Unreal -y (model +y): scenery turns furniture with its back
                 to the buildings (yaw = direction to the road + 90, so Unreal -y faces the road)
  bike stands    the hoop across the row (model y), the row along x (0.9 m apart)
  signal heads   lenses towards Unreal +y (as CARLA's sign plates: scenery and signals face +y to the traffic);
                 origin at the middle of the back plate's back, so a head hangs on a pole at that point
  markings       flat, 6 mm thick, the shark teeth's tips towards +x (towards the driver who must give way)
  decals         flat quads 4 mm above the origin, no collision (tools/ue_street_props.py)

Material slots are named after the PALETTE entries below (colour, metalness, roughness, optional texture);
tools/ue_street_props.py makes one material instance per slot. The manifest street_props.json lists each mesh with
its Unreal path, the semantic folder (CARLA's tagger labels a prop by the nearest folder named Pole, Static,
TrafficLight, RoadLine, Road, Building), half extents and bounding-box centre in Unreal metres (the catalogue's
"ext"/"off", as data/meshes_1.0.json), triangle count, the stock CARLA mesh a server without this content spawns
instead, and the licence (all procedural here: MIT, this repository). Lamp heights and arm lengths follow the
register's types (data.eindhoven.nl data-openbare-verlichting: type_drager_mast "AL 3,5 CONISCH", "ST 9
STANDAARD", type_drager_uithouder "ST ENKEL 1,50 M", "DUBBEL"); bench, bin and bike stand sizes the common Dutch
products (zitbanken0 "Model Eindhoven", afvalbakken0 "Bammens Prestige" 50 l, the galvanised 48 mm "nietje").
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from drone_mesh import Mesh, box, disc, sphere, tube, write_glb  # noqa: E402

ROOT = "/Game/CoupledCarla/Street"
LICENCE = "procedural, coupled-carla (MIT): tools/street_props.py"

# slot -> (linear base colour, metallic, roughness[, texture key]); sRGB values noted where they come from a RAL
PALETTE = {
    "AluGrey": ((0.36, 0.37, 0.38), 0.85, 0.42),            # natural aluminium masts (AL ...)
    "Galvanised": ((0.42, 0.42, 0.40), 0.9, 0.55),          # hot-dip galvanised steel (ST ...)
    "Anthracite": ((0.035, 0.040, 0.045), 0.35, 0.5),       # RAL 7016
    "CastIron": ((0.020, 0.026, 0.022), 0.4, 0.55),         # dark green-black cast iron (GY ...)
    "LampHousing": ((0.10, 0.105, 0.11), 0.6, 0.45),        # luminaire housings, grey RAL 7043
    "LampGlass": ((0.75, 0.75, 0.72), 0.0, 0.08),           # the diffuser / lantern glass (lit at night by season.py lights)
    "AmsterdamRed": ((0.16, 0.016, 0.018), 0.3, 0.5),       # the Amsterdammertje's red-brown, sRGB ~ (110, 33, 35)
    "AmsterdamCross": ((0.30, 0.05, 0.05), 0.3, 0.45),      # its three St Andrew's crosses, a lighter cast
    "ReflectWhite": ((0.80, 0.80, 0.78), 0.0, 0.35),        # retroreflective bands
    "Concrete": ((0.33, 0.32, 0.30), 0.0, 0.85),
    "SlatDarkBrown": ((0.055, 0.027, 0.014), 0.0, 0.6),     # register kleur "Donker bruin" (recycled plastic)
    "SlatLightBrown": ((0.28, 0.15, 0.07), 0.0, 0.65),      # "Licht bruin" (hardwood)
    "BinGreen": ((0.018, 0.036, 0.024), 0.3, 0.45),         # RAL 6009 fir green, register kleur "Groen"
    "BinGrey": ((0.20, 0.205, 0.20), 0.3, 0.45),            # RAL 7037, "Grijs"
    "BinBlack": ((0.015, 0.015, 0.016), 0.2, 0.45),
    "BinGold": ((0.45, 0.30, 0.06), 0.8, 0.35),             # the UK litter bin's band
    "SignalBlack": ((0.012, 0.012, 0.013), 0.1, 0.4),
    "SignalYellow": ((0.86, 0.50, 0.0), 0.0, 0.4),          # the Dutch yellow back plate, sRGB ~ (240, 188, 0)
    "SignalWhite": ((0.80, 0.80, 0.80), 0.0, 0.4),          # UK back plates' white border
    "LensRed": ((0.12, 0.004, 0.004), 0.0, 0.1),
    "LensAmber": ((0.14, 0.05, 0.0), 0.0, 0.1),
    "LensGreen": ((0.0, 0.07, 0.045), 0.0, 0.1),
    "PoleWhite": ((0.78, 0.78, 0.77), 0.0, 0.4),
    "PoleBlack": ((0.015, 0.015, 0.016), 0.0, 0.4),
    "RoadWhite": ((0.70, 0.70, 0.68), 0.0, 0.62),           # thermoplastic road marking
    "RedAsphalt": ((0.20, 0.035, 0.025), 0.0, 0.85, "redasphalt"),  # Dutch fietspad, sRGB ~ (125, 52, 44)
    "Klinker": ((1.0, 1.0, 1.0), 0.0, 0.8, "klinker"),     # grey concrete pavers (betonklinkers), stretcher bond
    "Crack": ((1.0, 1.0, 1.0), 0.0, 0.8, "crack"),
    "Patch": ((1.0, 1.0, 1.0), 0.0, 0.75, "patch"),
    "Pothole": ((1.0, 1.0, 1.0), 0.0, 0.9, "pothole"),
    "Oil": ((1.0, 1.0, 1.0), 0.0, 0.25, "oil"),
    "Leaves": ((1.0, 1.0, 1.0), 0.0, 0.8, "leaves"),
    "RoofUnit": ((0.55, 0.56, 0.55), 0.3, 0.5),             # galvanised / grey-white AC casings
    "RoofDark": ((0.04, 0.04, 0.045), 0.2, 0.6),            # fan grilles, roof felt, doors
    "RoofRender": ((0.38, 0.37, 0.35), 0.0, 0.85),          # bulkhead walls
    "JPGrey": ((0.30, 0.31, 0.31), 0.6, 0.45),              # Japanese galvanised / grey poles
}
DECAL = {"Crack", "Patch", "Pothole", "Oil", "Leaves", "RedAsphalt"}    # masked, two-sided, no collision


# -- extra primitives ----------------------------------------------------------------------------------------------

def rot(yaw=0.0, pitch=0.0, roll=0.0):
    """Rotation matrix: roll about x, then pitch about y, then yaw about z (radians, model frame)."""
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll),
                              math.sin(roll))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def obox(m, mat, centre, size, R):
    """A box of size (x, y, z) turned by R about its centre."""
    sub = Mesh()
    box(sub, mat, (0, 0, 0), size)
    for v, n, uv, ix in zip(*[sub.parts[mat][k] for k in range(4)]):
        m.add(mat, v @ np.asarray(R).T + np.asarray(centre, float), n @ np.asarray(R).T, uv, ix - ix.min())


def quad(m, mat, corners, normal, uvs=((0, 0), (1, 0), (1, 1), (0, 1)), both=False):
    v = np.asarray(corners, float)
    m.add(mat, v, np.tile(normal, (4, 1)), uvs, [0, 1, 2, 0, 2, 3])
    if both:
        m.add(mat, v, np.tile(-np.asarray(normal, float), (4, 1)), uvs, [0, 2, 1, 0, 3, 2])


def flat(m, mat, centre, size, yaw=0.0, z=0.004, uv_scale=(1.0, 1.0)):
    """A horizontal quad (decal) of size (x, y), facing up."""
    hx, hy = size[0] / 2, size[1] / 2
    R = rot(yaw)
    pts = [np.asarray(centre, float) + R @ np.array([px, py, z]) for px, py in ((-hx, -hy), (hx, -hy), (hx, hy),
                                                                              (-hx, hy))]
    su, sv = uv_scale
    quad(m, mat, pts, (0, 0, 1), ((0, sv), (su, sv), (su, 0), (0, 0)))


def prism(m, mat, pts2d, z0, z1):
    """A vertical prism over a convex polygon (counter-clockwise in x, y)."""
    p = [np.array([x, y]) for x, y in pts2d]
    n = len(p)
    for k in range(n):
        a, b = p[k], p[(k + 1) % n]
        e = b - a
        nrm = np.array([e[1], -e[0], 0.0]) / max(1e-9, np.hypot(*e))
        quad(m, mat, [(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)], nrm)
    c = np.mean(p, axis=0)
    for z, nz in ((z1, 1.0), (z0, -1.0)):
        V = [(c[0], c[1], z)] + [(q[0], q[1], z) for q in p]
        idx = []
        for k in range(n):
            tri = [0, k + 1, (k + 1) % n + 1]
            idx += tri if nz > 0 else [tri[0], tri[2], tri[1]]
        m.add(mat, V, np.tile((0, 0, nz), (len(V), 1)), [(0.5, 0.5)] * len(V), idx)


def polyline_tube(m, mat, pts, r, seg=10):
    pts = [np.asarray(p, float) for p in pts]
    for a, b in zip(pts, pts[1:]):
        tube(m, mat, a, b, r, seg=seg, caps=False)
    for p in pts[1:-1]:
        sphere(m, mat, p, r, nu=seg, nv=4)


def arc(c, r, a0, a1, n, plane="yz"):
    """Points on a circular arc of radius r about c, angles in radians, in the y-z (or x-z) plane."""
    out = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        d = (0.0, math.cos(a) * r, math.sin(a) * r) if plane == "yz" else (math.cos(a) * r, 0.0, math.sin(a) * r)
        out.append(np.asarray(c, float) + np.array(d))
    return out


# -- lamps -----------------------------------------------------------------------------------------------------

def _foot(m, mat, r, h=0.5):
    tube(m, mat, (0, 0, 0), (0, 0, h), r * 1.25, r * 1.1, seg=16)


def lamp_post_top(h=4.0, mat="AluGrey"):
    """AL 3,5 / 4 CONISCH: a conical aluminium mast with a post-top luminaire (residential streets; 12,034 + 2,036 +
    3,354 ... of Eindhoven's 58,507 light points). Light point at h + 0.1."""
    m = Mesh()
    _foot(m, mat, 0.065)
    tube(m, mat, (0, 0, 0.5), (0, 0, h), 0.060, 0.038, seg=16)
    tube(m, "LampHousing", (0, 0, h - 0.02), (0, 0, h + 0.05), 0.05, 0.12, seg=24)     # neck flaring out
    tube(m, "LampGlass", (0, 0, h + 0.05), (0, 0, h + 0.17), 0.21, 0.22, seg=32)       # diffuser ring
    tube(m, "LampHousing", (0, 0, h + 0.17), (0, 0, h + 0.30), 0.235, 0.06, seg=32)    # conical hat
    disc(m, "LampGlass", (0, 0, h + 0.05), 0.21, seg=32, both=False)
    return m, {"light": [0.0, 0.0, h + 0.1]}


def lamp_top_led(h=6.0, mat="AluGrey"):
    """AL 6 / AL 7 NEDAL with an "opzetstuk" (top spigot) and a flat LED street luminaire (Digistreet, Iridium):
    collector roads. The luminaire reaches 0.55 m out towards +x."""
    m = Mesh()
    _foot(m, mat, 0.08)
    tube(m, mat, (0, 0, 0.5), (0, 0, h), 0.075, 0.045, seg=16)
    tube(m, mat, (0, 0, h - 0.05), (0.18, 0, h + 0.03), 0.03, seg=10)
    _luminaire(m, (0.18, 0, h + 0.03), 0.62, 0.26, 0.085, tilt=math.radians(5))
    return m, {"light": [0.5, 0.0, h - 0.02]}


def _luminaire(m, at, length, width, height, tilt=0.0):
    """A flat LED street luminaire from its mounting point at `at` towards +x: a rounded housing, glass below."""
    R = rot(pitch=-tilt)
    c = np.asarray(at, float) + R @ np.array([length / 2, 0, 0])
    obox(m, "LampHousing", c, (length, width, height), R)
    obox(m, "LampHousing", c + R @ np.array([0, 0, height / 2]), (length * 0.92, width * 0.86, height * 0.4), R)
    obox(m, "LampGlass", c - R @ np.array([0.03, 0, height / 2 + 0.004]), (length * 0.72, width * 0.8, 0.008), R)


def lamp_arm(h=9.0, arm=1.5, mat="Galvanised", double=False):
    """ST 9 STANDAARD (and 8-12 m) steel masts with a single (ST ENKEL 1 M / 1,50 M) or double (ST DUBBEL 1,50 M)
    arm and an LED luminaire at its end: main roads. The arm leaves the mast 0.4 m below its top and rises 8 deg."""
    m = Mesh()
    _foot(m, mat, 0.10, 0.7)
    tube(m, mat, (0, 0, 0.7), (0, 0, h), 0.084, 0.050, seg=16)
    sides = (1, -1) if double else (1,)
    lights = []
    for s in sides:
        rise = math.tan(math.radians(8)) * arm
        a = np.array([0, 0, h - 0.4])
        mid = np.array([s * arm * 0.35, 0, h - 0.4 + rise * 0.6])
        end = np.array([s * arm, 0, h - 0.4 + rise])
        polyline_tube(m, mat, [a, mid, end], 0.032)
        R = rot(yaw=0.0 if s > 0 else math.pi)
        lum = Mesh()
        _luminaire(lum, (0, 0, 0), 0.72, 0.30, 0.10, tilt=math.radians(5))
        for k, (vs, ns, uvs, ids, _) in lum.parts.items():
            for v, n, uv, ix in zip(vs, ns, uvs, ids):
                m.add(k, v @ R.T + end, n @ R.T, uv, ix - ix.min())
        lights.append([float(s * (arm + 0.36)), 0.0, float(h - 0.4 + rise - 0.06)])
    return m, {"light": lights[0], "lights": lights}


def lamp_heritage(h=3.6):
    """GY 3,5 (cast iron, "met deur"): a fluted base, a slim column and a four-sided lantern (centres, old
    streets; the Annecy / CitySpirit luminaires)."""
    m = Mesh()
    tube(m, "CastIron", (0, 0, 0), (0, 0, 0.12), 0.17, 0.16, seg=8)
    tube(m, "CastIron", (0, 0, 0.12), (0, 0, 0.95), 0.13, 0.085, seg=12)
    for k in range(3):
        z = 0.95 + k * 0.05
        tube(m, "CastIron", (0, 0, z), (0, 0, z + 0.03), 0.095, 0.095, seg=12)
    tube(m, "CastIron", (0, 0, 1.1), (0, 0, h - 0.35), 0.055, 0.045, seg=12)
    tube(m, "CastIron", (0, 0, h - 0.35), (0, 0, h - 0.25), 0.06, 0.11, seg=12)
    # the lantern: a tapered four-sided glass body under a pyramid roof and a finial
    zb, zt = h - 0.25, h + 0.30
    rb, rt = 0.12, 0.20
    for k in range(4):
        a0, a1 = k * math.pi / 2 + math.pi / 4, (k + 1) * math.pi / 2 + math.pi / 4
        p = [(math.cos(a0) * rb, math.sin(a0) * rb, zb), (math.cos(a1) * rb, math.sin(a1) * rb, zb),
             (math.cos(a1) * rt, math.sin(a1) * rt, zt), (math.cos(a0) * rt, math.sin(a0) * rt, zt)]
        am = (a0 + a1) / 2
        quad(m, "LampGlass", p, (math.cos(am), math.sin(am), 0.3), both=True)
        tube(m, "CastIron", p[0], p[3], 0.012, seg=6)
    tube(m, "CastIron", (0, 0, zt), (0, 0, zt + 0.04), rt * 1.45, rt * 1.45, seg=4)
    tube(m, "CastIron", (0, 0, zt + 0.04), (0, 0, zt + 0.22), rt * 1.4, 0.02, seg=4)
    sphere(m, "CastIron", (0, 0, zt + 0.27), 0.035, nu=10, nv=6)
    return m, {"light": [0.0, 0.0, (zb + zt) / 2]}


# -- bollards, benches, bins, bike stands ----------------------------------------------------------------------

def bollard_nl():
    """A plain Dutch steel bollard: 0.9 m, 102 mm, anthracite with two retroreflective bands, a rounded top."""
    m = Mesh()
    tube(m, "Anthracite", (0, 0, 0), (0, 0, 0.62), 0.051, seg=16)
    tube(m, "ReflectWhite", (0, 0, 0.62), (0, 0, 0.68), 0.0515, seg=16, caps=False)
    tube(m, "Anthracite", (0, 0, 0.68), (0, 0, 0.74), 0.051, seg=16, caps=False)
    tube(m, "ReflectWhite", (0, 0, 0.74), (0, 0, 0.80), 0.0515, seg=16, caps=False)
    tube(m, "Anthracite", (0, 0, 0.80), (0, 0, 0.86), 0.051, seg=16, caps=False)
    sphere(m, "Anthracite", (0, 0, 0.86), 0.051, nu=16, nv=6, z_scale=0.8, half=True)
    return m, {}


def amsterdammertje():
    """The Amsterdam bollard: cast iron, red-brown, about 1.0 m, tapering from 0.14 m to 0.11 m, with the city's
    three St Andrew's crosses on its front (model +y and -y) under a knobbed top."""
    m = Mesh()
    tube(m, "AmsterdamRed", (0, 0, 0), (0, 0, 0.80), 0.072, 0.056, seg=20)
    tube(m, "AmsterdamRed", (0, 0, 0.80), (0, 0, 0.84), 0.062, 0.062, seg=20)
    tube(m, "AmsterdamRed", (0, 0, 0.84), (0, 0, 0.92), 0.055, 0.050, seg=20)
    tube(m, "AmsterdamRed", (0, 0, 0.92), (0, 0, 0.95), 0.056, 0.040, seg=20, caps=False)
    sphere(m, "AmsterdamRed", (0, 0, 0.97), 0.038, nu=16, nv=8)
    for side in (1, -1):
        for k in range(3):
            z = 0.66 - k * 0.11
            r = 0.059 + (0.80 - z) / 0.80 * 0.016       # the body's radius there
            for a in (math.radians(40), math.radians(-40)):
                obox(m, "AmsterdamCross", (0, side * r, z), (0.075, 0.012, 0.014), rot(roll=0) @ rot(pitch=a))
    return m, {}


def bench_eindhoven(slat="SlatDarkBrown"):
    """"Model Eindhoven" (zitbanken0: 1,953 of 3,963 benches): two concrete feet, steel brackets, three seat slats
    at 0.45 m and two backrest slats, 1.8 m long; the seat faces model +y (Unreal -y)."""
    m = Mesh()
    L = 1.8
    for x in (-0.65, 0.65):
        prism(m, "Concrete", [(x - 0.07, -0.26), (x + 0.07, -0.26), (x + 0.07, 0.22), (x - 0.07, 0.22)], 0, 0.38)
        obox(m, "Anthracite", (x, 0.0, 0.405), (0.06, 0.48, 0.05), np.eye(3))
        obox(m, "Anthracite", (x, -0.22, 0.62), (0.05, 0.05, 0.46), rot(roll=math.radians(-12)))
    for k, y in enumerate((-0.15, 0.0, 0.15)):
        obox(m, slat, (0, y, 0.45), (L, 0.13, 0.045), np.eye(3))
    R = rot(roll=math.radians(-12))
    for z in (0.62, 0.76):
        c = np.array([0, -0.22 - (z - 0.45) * math.tan(math.radians(12)) - 0.035, z])
        obox(m, slat, c, (L, 0.035, 0.12), R)
    return m, {}


def bin_round(colour="BinGreen"):
    """Bammens Prestige type (afvalbakken0: 2,756 of 4,445), 50 l: a round steel bin, 0.42 m across, on a post, the
    opening under the lid towards model +y; top 0.95 m."""
    m = Mesh()
    tube(m, colour, (0, -0.02, 0), (0, -0.02, 0.32), 0.035, seg=12)
    tube(m, colour, (0, 0, 0.30), (0, 0, 0.80), 0.21, seg=28)
    tube(m, "BinBlack", (0, 0, 0.80), (0, 0, 0.84), 0.205, seg=28, caps=False)       # the mouth's shadow line
    tube(m, colour, (0, 0, 0.84), (0, 0, 0.90), 0.215, 0.215, seg=28)
    sphere(m, colour, (0, 0, 0.90), 0.215, nu=28, nv=6, z_scale=0.25, half=True)
    obox(m, "BinBlack", (0, 0.20, 0.82), (0.26, 0.03, 0.05), np.eye(3))                # the slot at the front
    return m, {}


def bin_square(colour="BinGrey"):
    """A square street bin (Grijsen Constructo, Falco type): 0.45 x 0.40 m, 1.0 m, a sloping lid with the slot."""
    m = Mesh()
    prism(m, colour, [(-0.225, -0.2), (0.225, -0.2), (0.225, 0.2), (-0.225, 0.2)], 0.03, 0.88)
    prism(m, "BinBlack", [(-0.2, -0.18), (0.2, -0.18), (0.2, 0.18), (-0.2, 0.18)], 0.0, 0.03)
    obox(m, colour, (0, -0.02, 0.93), (0.47, 0.44, 0.08), rot(roll=math.radians(8)))
    obox(m, "BinBlack", (0, 0.205, 0.80), (0.30, 0.01, 0.07), np.eye(3))
    return m, {}


def bin_uk():
    """A British cast litter bin: black, round, 1.0 m, a gold band under the domed lid."""
    m = Mesh()
    tube(m, "BinBlack", (0, 0, 0), (0, 0, 0.06), 0.27, seg=24)
    tube(m, "BinBlack", (0, 0, 0.06), (0, 0, 0.78), 0.25, 0.26, seg=24)
    tube(m, "BinGold", (0, 0, 0.70), (0, 0, 0.74), 0.262, seg=24, caps=False)
    tube(m, "BinBlack", (0, 0, 0.78), (0, 0, 0.86), 0.24, 0.24, seg=24)
    sphere(m, "BinBlack", (0, 0, 0.86), 0.25, nu=24, nv=6, z_scale=0.45, half=True)
    obox(m, "SignalBlack", (0, 0.235, 0.82), (0.3, 0.03, 0.06), np.eye(3))
    return m, {}


def nietjes(n=1, pitch=0.9):
    """n galvanised bike stands ("nietjes": an inverted U of 48 mm tube, 0.75 m high, 0.70 m wide) in a row along x;
    each hoop spans model y, the bikes stand on both sides of it."""
    m = Mesh()
    for k in range(n):
        x = (k - (n - 1) / 2) * pitch
        r = 0.12
        pts = [(x, -0.35, 0.0), (x, -0.35, 0.75 - r)]
        pts += [tuple(p) for p in arc((x, -0.35 + r, 0.75 - r), r, math.pi, math.pi / 2, 4)][1:]
        pts += [tuple(p) for p in arc((x, 0.35 - r, 0.75 - r), r, math.pi / 2, 0.0, 4)]
        pts += [(x, 0.35, 0.0)]
        polyline_tube(m, "Galvanised", pts, 0.024, seg=8)
    return m, {}


# -- traffic lights --------------------------------------------------------------------------------------------

def signal_pole(h=3.6, band=0.25):
    """The Dutch traffic-light pole: 114 mm, black and white bands (0.25 m) over its whole height, a black cap."""
    m = Mesh()
    n = int(h / band)
    for k in range(n):
        z0, z1 = k * band, min(h, (k + 1) * band)
        tube(m, "PoleBlack" if k % 2 == 0 else "PoleWhite", (0, 0, z0), (0, 0, z1), 0.057, seg=16, caps=k == 0)
    tube(m, "PoleBlack", (0, 0, n * band), (0, 0, h + 0.05), 0.057, 0.04, seg=16)
    return m, {"top": [0.0, 0.0, h]}


def signal_head(lenses=3, d=0.30, plate="SignalYellow", border=None, horizontal=False, pole_r=0.057):
    """A signal head: a black body with `lenses` round lenses (diameter d: 300 mm car heads, 200 mm cyclist and
    pedestrian heads) under visors, on a back plate (Dutch yellow, or black with a white border as the UK's). The
    lenses face model -y (Unreal +y); origin at the middle of the back plate's back, which stands pole_r in front
    of the pole's axis when the head hangs on a pole at x = 0 (y = 0 is the pole's axis)."""
    m = Mesh()
    pitch = d + 0.06
    n = lenses
    long_, short = n * pitch + 0.06, d + 0.12
    bw, bh = (long_, short) if horizontal else (short, long_)
    y0 = -pole_r                         # the plate's back
    pw, ph = bw + 0.30, bh + 0.30        # back plate
    if border:
        obox(m, border, (0, y0 - 0.006, 0), (pw, 0.012, ph), np.eye(3))
        obox(m, plate, (0, y0 - 0.013, 0), (pw - 0.14, 0.004, ph - 0.14), np.eye(3))
    else:
        obox(m, plate, (0, y0 - 0.008, 0), (pw, 0.016, ph), np.eye(3))
    depth = 0.22
    obox(m, "SignalBlack", (0, y0 - 0.016 - depth / 2, 0), (bw, depth, bh), np.eye(3))
    lens_mats = {3: ["LensRed", "LensAmber", "LensGreen"], 2: ["LensRed", "LensGreen"], 1: ["LensAmber"]}[n]
    centres = []
    for k in range(n):
        off = ((n - 1) / 2 - k) * pitch
        cx, cz = (-off, 0.0) if horizontal else (0.0, off)
        yf = y0 - 0.016 - depth
        disc_c = np.array([cx, yf - 0.002, cz])
        sub = Mesh()
        disc(sub, lens_mats[k], (0, 0, 0), d / 2, seg=28, both=False)
        R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])        # disc normal +z -> -y
        for v, nn, uv, ix in zip(*[sub.parts[lens_mats[k]][j] for j in range(4)]):
            m.add(lens_mats[k], v @ R.T + disc_c, nn @ R.T, uv, ix - ix.min())
        # visor: a half tube over the lens
        for a0 in range(6):
            a = math.pi * a0 / 6
            a1 = math.pi * (a0 + 1) / 6
            r = d / 2 + 0.02
            p0 = (cx + math.cos(a) * r, cz + math.sin(a) * r)
            p1 = (cx + math.cos(a1) * r, cz + math.sin(a1) * r)
            quad(m, "SignalBlack", [(p0[0], yf, p0[1]), (p1[0], yf, p1[1]), (p1[0], yf - 0.16, p1[1]),
                                    (p0[0], yf - 0.16, p0[1])],
                 (math.cos((a + a1) / 2), 0, math.sin((a + a1) / 2)), both=True)
        centres.append([round(cx, 3), round(-(yf - 0.002), 3), round(cz, 3)])     # Unreal (x, y, z)
    return m, {"lenses": centres, "plate": [round(pw, 3), round(ph, 3)]}


# -- markings and road surface -----------------------------------------------------------------------------------

def shark_teeth(n=4, base=0.5, length=0.6, gap=0.25):
    """Give-way triangles ("haaientanden", RVV / BABW: 0.50 m base, 0.60 m long, 0.25 m apart), n in a row across
    the lane (model y), the tips towards +x. 6 mm thick."""
    m = Mesh()
    pitch = base + gap
    for k in range(n):
        y = (k - (n - 1) / 2) * pitch
        tri = [(-length / 2, y - base / 2), (length / 2, y), (-length / 2, y + base / 2)]
        _flat_poly(m, "RoadWhite", tri, 0.006)
    return m, {"width": round(n * pitch - gap, 3)}


def _flat_poly(m, mat, pts, h):
    """A thin polygon slab from z 0 to h, top facing up (the sides are left out: 6 mm do not show)."""
    V = [(x, y, h) for x, y in pts]
    idx = [k for i in range(1, len(pts) - 1) for k in (0, i, i + 1)]
    a = np.array(pts)
    area = 0.5 * sum(a[i, 0] * a[(i + 1) % len(a), 1] - a[(i + 1) % len(a), 0] * a[i, 1] for i in range(len(a)))
    if area < 0:
        idx = [idx[i + j] for i in range(0, len(idx), 3) for j in (0, 2, 1)]
    m.add(mat, V, np.tile((0, 0, 1), (len(V), 1)), [(x, y) for x, y in pts], idx)


def block_marking():
    """One block of a Dutch block marking ("blokmarkering", 0.3 m wide, 0.3 m long, at 0.6 m intervals), the dashed
    line along an exit or a cycle crossing ("fietsoversteek"). 6 mm thick."""
    m = Mesh()
    _flat_poly(m, "RoadWhite", [(-0.15, -0.15), (0.15, -0.15), (0.15, 0.15), (-0.15, 0.15)], 0.006)
    return m, {}


def red_strip(w=1.0, l=6.0):
    """Red asphalt overlay for a cycle lane: w x l (scaled by the lane's width at placement), 8 mm up."""
    m = Mesh()
    flat(m, "RedAsphalt", (0, 0, 0), (l, w), z=0.008, uv_scale=(l / 2.0, w / 2.0))
    return m, {}


def paving(T, tile_m=2.1):
    """A T x T m paving square (Dutch grey concrete pavers), flat, top at z 0, the texture repeating every tile_m
    metres (so the pavers keep their real 21 x 10.5 cm at every square size): scenery lays ground squares of 40, 20
    and 10 m (BGT "open verharding": footways, parking bays, squares)."""
    m = Mesh()
    flat(m, "Klinker", (0, 0, 0), (T, T), z=0.0, uv_scale=(T / tile_m, T / tile_m))
    return m, {}


def decal(mat, size, z=0.004):
    m = Mesh()
    flat(m, mat, (0, 0, 0), size, z=z)
    return m, {}


# -- rooftop clutter ---------------------------------------------------------------------------------------------

def roof_ac():
    """An outdoor air-conditioning / heat-pump unit on a roof: 1.0 x 0.45 x 0.85 m on two rails, the fan grille
    towards model +y."""
    m = Mesh()
    for x in (-0.35, 0.35):
        obox(m, "RoofDark", (x, 0, 0.04), (0.06, 0.6, 0.08), np.eye(3))
    obox(m, "RoofUnit", (0, 0, 0.08 + 0.425), (1.0, 0.45, 0.85), np.eye(3))
    sub = Mesh()
    disc(sub, "RoofDark", (0, 0, 0), 0.3, seg=24, both=False)
    R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])        # +z -> +y
    for v, n, uv, ix in zip(*[sub.parts["RoofDark"][j] for j in range(4)]):
        m.add("RoofDark", v @ R.T + np.array([-0.15, 0.226, 0.5]), n @ R.T, uv, ix - ix.min())
    return m, {}


def roof_ac_large():
    """A large rooftop condenser / air handler: 2.2 x 1.1 x 1.2 m with two fans on top."""
    m = Mesh()
    obox(m, "RoofUnit", (0, 0, 0.62), (2.2, 1.1, 1.2), np.eye(3))
    obox(m, "RoofDark", (0, 0, 0.05), (2.3, 1.2, 0.1), np.eye(3))
    for x in (-0.55, 0.55):
        tube(m, "RoofDark", (x, 0, 1.22), (x, 0, 1.30), 0.42, seg=24)
    return m, {}


def roof_bulkhead():
    """A stair / lift bulkhead ("trappenhuisopbouw"): 3.0 x 2.6 m, 2.6 m high, a door towards model +y."""
    m = Mesh()
    obox(m, "RoofRender", (0, 0, 1.3), (3.0, 2.6, 2.6), np.eye(3))
    obox(m, "RoofDark", (0, 0, 2.63), (3.1, 2.7, 0.06), np.eye(3))
    obox(m, "RoofDark", (0.6, 1.305, 1.05), (0.9, 0.02, 2.1), np.eye(3))
    return m, {}


def roof_vent():
    """A mushroom vent: a 0.25 m pipe 0.7 m high under a round cap."""
    m = Mesh()
    tube(m, "RoofUnit", (0, 0, 0), (0, 0, 0.7), 0.125, seg=16)
    tube(m, "RoofUnit", (0, 0, 0.75), (0, 0, 0.82), 0.22, 0.05, seg=16)
    tube(m, "RoofDark", (0, 0, 0.7), (0, 0, 0.75), 0.11, seg=16, caps=False)
    return m, {}


# -- UK and Japan -------------------------------------------------------------------------------------------------

def lamp_uk(h=8.0):
    """A British galvanised lighting column (BS EN 40) with a short upswept bracket and an LED lantern."""
    m = Mesh()
    _foot(m, "Galvanised", 0.09, 0.9)
    tube(m, "Galvanised", (0, 0, 0.9), (0, 0, h - 0.3), 0.076, 0.06, seg=16)
    polyline_tube(m, "Galvanised", [(0, 0, h - 0.3), (0.15, 0, h - 0.05), (0.6, 0, h + 0.05)], 0.03)
    _luminaire(m, (0.6, 0, h + 0.05), 0.6, 0.25, 0.09, tilt=math.radians(3))
    return m, {"light": [1.0, 0.0, h - 0.02]}


def lamp_jp(h=8.0):
    """A Japanese street lamp: a grey steel pole with a long curved (gooseneck) arm and a flat LED head."""
    m = Mesh()
    _foot(m, "JPGrey", 0.08, 0.5)
    tube(m, "JPGrey", (0, 0, 0.5), (0, 0, h - 0.8), 0.07, 0.055, seg=16)
    pts = [np.array([0, 0, h - 0.8])] + [np.array(p) for p in arc((0.8, 0, h - 0.8), 0.8, math.pi, math.pi / 2, 6, plane="xz")][1:]
    pts += [np.array([1.6, 0, h])]
    polyline_tube(m, "JPGrey", pts, 0.045)
    _luminaire(m, (1.6, 0, h - 0.02), 0.55, 0.28, 0.09)
    return m, {"light": [1.9, 0.0, h - 0.08]}


# -- the catalogue ------------------------------------------------------------------------------------------------

def specs():
    """[(name, region, folder, role, maker, fallback stock mesh (1.0 catalogue name), note)]"""
    S = []

    def add(name, region, folder, role, make, fallback=None, note=""):
        S.append((name, region, folder, role, make, fallback, note))
    # Dutch lamps (Pole): by register mast type and height (streetlook.lamp_for)
    add("LampPostTop4", "NL", "Pole", "lamp_post_top", lambda: lamp_post_top(4.0), "SM_StreetLight03",
        "AL 3,5 / AL 4 CONISCH, post-top luminaire (Ceder, CitySoul)")
    add("LampTopLed6", "NL", "Pole", "lamp_top", lambda: lamp_top_led(6.0), "SM_StreetLight06_A",
        "AL 6 / AL 7 NEDAL, top spigot + LED (Digistreet, Iridium)")
    add("LampArm9", "NL", "Pole", "lamp_arm", lambda: lamp_arm(9.0, 1.5), "SM_StreetLight06_A",
        "ST 8-10 STANDAARD with ST ENKEL 1,50 M")
    add("LampArm12", "NL", "Pole", "lamp_arm", lambda: lamp_arm(12.0, 2.0), "SM_StreetLight06_A",
        "ST 11-16 with a 2 m arm")
    add("LampDouble10", "NL", "Pole", "lamp_double", lambda: lamp_arm(10.0, 1.5, double=True), "SM_StreetLight06_A",
        "ST 9-11 M.V.U. DUBBEL 1,50 M (medians)")
    add("LampHeritage4", "NL", "Pole", "lamp_heritage", lambda: lamp_heritage(3.6), "SM_StreetLight03",
        "GY 3,5 cast iron with a lantern (old centres)")
    # Dutch furniture (Static)
    add("Bollard", "NL", "Static", "bollard", bollard_nl, "SM_Bollard_Grey")
    add("Amsterdammertje", "NL", "Static", "bollard_amsterdam", amsterdammertje, "SM_Bollard_Grey")
    add("BenchEindhoven_DarkBrown", "NL", "Static", "bench", lambda: bench_eindhoven("SlatDarkBrown"), "SM_Bench01")
    add("BenchEindhoven_LightBrown", "NL", "Static", "bench", lambda: bench_eindhoven("SlatLightBrown"),
        "SM_Bench01")
    add("BinRound_Green", "NL", "Static", "bin", lambda: bin_round("BinGreen"), "SM_TrashCan01")
    add("BinRound_Grey", "NL", "Static", "bin", lambda: bin_round("BinGrey"), "SM_TrashCan01")
    add("BinSquare_Grey", "NL", "Static", "bin", lambda: bin_square("BinGrey"), "SM_TrashCan02")
    add("Nietje_x1", "NL", "Static", "bike_stand", lambda: nietjes(1), "SM_BikeParking01")
    add("Nietje_x3", "NL", "Static", "bike_stand", lambda: nietjes(3), "SM_BikeParking01")
    add("Nietje_x6", "NL", "Static", "bike_stand", lambda: nietjes(6), "SM_BikeParking02")
    # Dutch traffic lights (TrafficLight): for signals.py's head style
    add("SignalPole", "NL", "TrafficLight", "signal_pole", signal_pole)
    add("SignalCar", "NL", "TrafficLight", "signal_car", lambda: signal_head(3, 0.30))
    add("SignalBike", "NL", "TrafficLight", "signal_bike", lambda: signal_head(3, 0.20))
    add("SignalPed", "NL", "TrafficLight", "signal_ped", lambda: signal_head(2, 0.20))
    # markings and surfaces (RoadLine, Road)
    for n in (3, 4, 5):
        add(f"SharkTeeth_{n}", "NL", "RoadLine", "shark_teeth", lambda n=n: shark_teeth(n))
    add("BlockMarking", "NL", "RoadLine", "block_marking", block_marking)
    add("RedAsphalt", "NL", "Road", "red_cycle", red_strip)
    for T in (40, 20, 10):
        add(f"Paving_{T}", "NL", "SideWalk", "paving", lambda T=T: paving(float(T)))
    # road wear (Road), any region
    add("Crack_A", "Any", "Road", "wear_crack", lambda: decal("Crack", (2.0, 2.0)))
    add("Patch_A", "Any", "Road", "wear_patch", lambda: decal("Patch", (2.4, 1.4), z=0.005))
    add("Pothole_A", "Any", "Road", "wear_pothole", lambda: decal("Pothole", (0.9, 0.9), z=0.006))
    add("Oil_A", "Any", "Road", "wear_oil", lambda: decal("Oil", (1.2, 1.6), z=0.004))
    add("Leaves_A", "Any", "Road", "wear_leaves", lambda: decal("Leaves", (2.0, 2.0), z=0.007))
    # rooftop clutter (Building), any region
    add("RoofAC", "Any", "Building", "roof_ac", roof_ac)
    add("RoofACLarge", "Any", "Building", "roof_ac_large", roof_ac_large)
    add("RoofBulkhead", "Any", "Building", "roof_bulkhead", roof_bulkhead)
    add("RoofVent", "Any", "Building", "roof_vent", roof_vent)
    # UK and Japan
    add("LampColumn8", "UK", "Pole", "lamp_arm", lambda: lamp_uk(8.0), "SM_StreetLight06_A", "BS EN 40 column")
    add("BinCast", "UK", "Static", "bin", bin_uk, "SM_TrashCan01")
    add("SignalCar", "UK", "TrafficLight", "signal_car",
        lambda: signal_head(3, 0.30, plate="SignalBlack", border="SignalWhite"))
    add("LampGooseneck8", "JP", "Pole", "lamp_arm", lambda: lamp_jp(8.0), "SM_StreetLight06_A")
    add("SignalCar", "JP", "TrafficLight", "signal_car",
        lambda: signal_head(3, 0.30, plate="SignalBlack", horizontal=True))
    return S


def mesh_name(region, name):
    return f"SM_CC_{region}_{name}"


def mesh_path(region, folder, name):
    n = mesh_name(region, name)
    return f"{ROOT}/{region}/{folder}/{n}.{n}"


def unreal_bounds(mesh):
    """(ext, off) in Unreal metres, as data/meshes_1.0.json: half extents and the box centre from the pivot."""
    vs = np.concatenate([np.concatenate(p[0]) for p in mesh.parts.values()])
    u = np.stack([vs[:, 0], -vs[:, 1], vs[:, 2]], axis=1)
    lo, hi = u.min(axis=0), u.max(axis=0)
    return [round(float(v), 3) for v in (hi - lo) / 2], [round(float(v), 3) for v in (hi + lo) / 2]


# -- decal textures --------------------------------------------------------------------------------------------

def textures(out, size=512, seed=7):
    """RGBA PNGs for the masked decals and the red asphalt (alpha = opacity mask), drawn with numpy and Pillow."""
    from PIL import Image, ImageDraw, ImageFilter
    rng = np.random.default_rng(seed)
    out = Path(out)

    def noise(s, octaves=5):
        acc = np.zeros((s, s))
        for o in range(octaves):
            n = 4 * 2 ** o
            g = rng.random((n + 1, n + 1))
            img = Image.fromarray((g * 255).astype(np.uint8)).resize((s, s), Image.BICUBIC)
            acc += np.asarray(img, float) / 255.0 / 2 ** o
        return acc / acc.max()

    def save(name, rgb, a):
        arr = np.dstack([np.clip(rgb, 0, 255), np.clip(a, 0, 255)]).astype(np.uint8)
        Image.fromarray(arr, "RGBA").save(out / f"T_CC_{name}.png")
    s = size
    # red asphalt: fine grain around sRGB (125, 52, 44), opaque, tiles every 2 m
    g = noise(s, 6)
    grain = rng.random((s, s))
    base = np.array([125, 52, 44], float)
    rgb = base[None, None, :] * (0.82 + 0.25 * g[..., None] + 0.12 * (grain[..., None] - 0.5))
    rgb[grain > 0.985] = [70, 66, 62]          # dark stones showing through
    save("redasphalt", rgb, np.full((s, s), 255))
    # grey concrete pavers 21 x 10.5 cm in stretcher bond, 2.1 m per texture (10 x 20 pavers: it tiles), dark joints
    img = Image.new("RGB", (s, s), (70, 70, 68))
    d = ImageDraw.Draw(img)
    pl_, rows = s / 10.0, 20
    for r in range(rows):
        y0 = r * s / rows
        off = (pl_ / 2) if r % 2 else 0.0
        x = -off
        while x < s:
            g = int(rng.normal(128, 9))
            tint = (g, g - 1, g - 4)
            d.rectangle([x + 1.2, y0 + 1.2, x + pl_ - 1.2, y0 + s / rows - 1.2], fill=tint)
            x += pl_
    arr = np.asarray(img, float) * (0.9 + 0.2 * noise(s, 6)[..., None])
    save("klinker", arr, np.full((s, s), 255))
    # cracks: dark branching random walks on transparent
    a = Image.new("L", (s, s), 0)
    d = ImageDraw.Draw(a)
    for _ in range(3):
        x, y = rng.uniform(0.2, 0.8) * s, rng.uniform(0.2, 0.8) * s
        ang = rng.uniform(0, 2 * math.pi)
        for step in range(60):
            ang += rng.normal(0, 0.45)
            nx, ny = x + math.cos(ang) * 7, y + math.sin(ang) * 7
            d.line([(x, y), (nx, ny)], fill=255, width=int(rng.integers(2, 5)))
            if rng.random() < 0.06:
                bx, by, ba = nx, ny, ang + rng.choice([-1, 1]) * rng.uniform(0.6, 1.2)
                for _ in range(15):
                    ba += rng.normal(0, 0.4)
                    cx, cy = bx + math.cos(ba) * 6, by + math.sin(ba) * 6
                    d.line([(bx, by), (cx, cy)], fill=200, width=2)
                    bx, by = cx, cy
            x, y = nx, ny
    a = np.asarray(a.filter(ImageFilter.GaussianBlur(0.8)), float)
    save("crack", np.full((s, s, 3), 18.0), a)
    # repair patch: darker, smoother asphalt with a lighter sealed edge
    yy, xx = np.mgrid[0:s, 0:s] / s
    edge = np.minimum.reduce([xx, 1 - xx, yy, 1 - yy])
    inside = edge > 0.03
    rgb = np.full((s, s, 3), 40.0) * (0.85 + 0.3 * noise(s, 5)[..., None])
    seal = (edge > 0.03) & (edge < 0.055)
    rgb[seal] = [62, 60, 58]
    save("patch", rgb, np.where(inside, 255, 0))
    # pothole: a dark broken hole with a crumbled rim
    r = np.hypot(xx - 0.5, yy - 0.5) / 0.5
    wob = 0.75 + 0.2 * noise(s, 4)
    hole = r < wob * 0.85
    rim = (r >= wob * 0.85) & (r < wob)
    rgb = np.zeros((s, s, 3))
    rgb[hole] = np.array([22, 20, 18]) * (0.7 + 0.6 * noise(s, 6)[hole][:, None])
    rgb[rim] = [58, 55, 52]
    save("pothole", rgb, np.where(hole | rim, 255, 0))
    # oil stain: irregular dark translucent blot
    blob = np.clip(1.0 - r / (0.65 + 0.35 * noise(s, 3)), 0, 1)
    a = (blob ** 0.6) * 170 * (0.7 + 0.3 * noise(s, 6))
    save("oil", np.full((s, s, 3), 10.0), a)
    # autumn leaves: many small leaves in oranges, yellows and browns
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    cols = [(176, 92, 24), (196, 140, 30), (120, 60, 20), (150, 40, 20), (90, 70, 30), (200, 120, 40)]
    for _ in range(420):
        x, y = rng.normal(0.5, 0.2) * s, rng.normal(0.5, 0.2) * s
        if not (8 < x < s - 8 and 8 < y < s - 8):
            continue
        L, W = rng.uniform(9, 18), rng.uniform(5, 9)
        ang = rng.uniform(0, math.pi)
        pts = [(x + L * math.cos(t) * math.cos(ang) - W * math.sin(t) * math.sin(ang),
                y + L * math.cos(t) * math.sin(ang) + W * math.sin(t) * math.cos(ang))
               for t in np.linspace(0, 2 * math.pi, 9)[:-1]]
        c = cols[int(rng.integers(len(cols)))]
        d.polygon(pts, fill=c + (255,))
    save("leaves", np.asarray(img, float)[..., :3], np.asarray(img, float)[..., 3])


def build(out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    meshes = {}
    for name, region, folder, role, make, fallback, note in specs():
        m, extra = make()
        n = mesh_name(region, name)
        tris = write_glb(m, out / f"{n}.glb")
        ext, off = unreal_bounds(m)
        slots = sorted(m.parts)
        meshes[n] = {"path": mesh_path(region, folder, name), "region": region, "folder": folder, "role": role,
                     "ext": ext, "off": off, "triangles": tris, "slots": slots, "fallback": fallback,
                     "decal": any(s in DECAL for s in slots), "note": note, "licence": LICENCE, **extra}
        print(f"{n}: {tris} triangles, {2 * ext[0]:.2f} x {2 * ext[1]:.2f} x {2 * ext[2]:.2f} m")
    textures(out)
    manifest = {"root": ROOT, "palette": {k: list(v[0]) + list(v[1:3]) + ([v[3]] if len(v) > 3 else [])
                                          for k, v in PALETTE.items()},
                "decal_slots": sorted(DECAL), "meshes": meshes}
    (out / "street_props.json").write_text(json.dumps(manifest, indent=1))
    (out / "SOURCES.md").write_text(sources_md(manifest))
    return manifest


SOURCES_HEAD = """# Street props: sources and licences

Every mesh and texture of `/Game/CoupledCarla/Street` is made by `tools/street_props.py` (numpy geometry, Pillow
textures) from published dimensions and register data; no third-party model, texture or photo is in them, so all
are MIT like the rest of this repository. Generated by `python tools/street_props.py OUT --data`; the meshes
themselves are not in git (`tools/ue_street_props.py` imports them into a CARLA 1.0 tree, about 4 MB of uassets).

Data the shapes and the choices follow (facts, not copied works):
- Gemeente Eindhoven open data (data.eindhoven.nl, CC0 / public open data): `data-openbare-verlichting`
  (type_drager_mast, type_drager_uithouder, type_armatuur, lph_armatuur: mast heights and arms), `zitbanken0`
  (bench model "Model Eindhoven", colours), `afvalbakken0` (bin models "Bammens Prestige" 50 l, colours).
- Dutch road marking and signal practice (RVV 1990 / BABW: shark teeth 0.50 x 0.60 m, block markings; black and
  white banded signal poles; yellow back plates as the brief asks), British BS EN 40 columns, Japanese gooseneck
  lamps: general street knowledge, approximate.
- Ideas for road wear, rooftop clutter and furniture by country standard: BeamNG.Earth
  (https://github.com/TecnaGamer/BeamNG.Earth, MIT, TecnaGamer); no code or data of it is used.
- Considered and not used: Sketchfab "City parking asset Amsterdammertje" (ustoopia, CC BY 4.0) and "Dutch Traffic
  Light" (tickinggears, CC BY 4.0); the procedural ones are exact in size, light and need no attribution in the
  package.

| Mesh | Unreal path | Size (m) | Triangles | Stock fallback | Based on | Licence |
|---|---|---|---|---|---|---|
"""


def sources_md(manifest):
    rows = []
    for n, s_ in sorted(manifest["meshes"].items()):
        e = s_["ext"]
        rows.append(f"| {n} | `{s_['path'].split('.')[0]}` | {2 * e[0]:.2f} x {2 * e[1]:.2f} x {2 * e[2]:.2f} | "
                    f"{s_['triangles']} | {s_['fallback'] or '-'} | {s_['note'] or s_['role']} | MIT (procedural) |")
    return SOURCES_HEAD + "\n".join(rows) + "\n"


CAT_OF = {"Pole": "lamp", "Static": "furniture", "TrafficLight": "furniture", "RoadLine": "marking",
          "Road": "marking", "Building": "furniture", "SideWalk": "ground"}


def write_data(manifest, repo=Path(__file__).resolve().parent.parent):
    """The manifest into coupledcarla/data/street_props.json and each mesh (ext, off) into data/meshes_1.0.json,
    as tools/mesh_catalog.py measures CARLA's own (the editor import checks the bounds against these)."""
    data = repo / "coupledcarla" / "data"
    (data / "street_props.json").write_text(json.dumps(manifest, indent=1) + "\n")
    cat_file = data / "meshes_1.0.json"
    cat = json.loads(cat_file.read_text())
    meshes = {k: v for k, v in cat["meshes"].items() if not k.startswith(ROOT + "/")}
    for n, spec in manifest["meshes"].items():
        meshes[spec["path"]] = {"cat": CAT_OF.get(spec["folder"], "furniture"), "ext": spec["ext"], "off": spec["off"]}
    cat["meshes"] = meshes
    cat_file.write_text(json.dumps(cat, indent=0))     # the catalogue's own layout (tools/mesh_catalog.py)
    src = repo / "unreal" / "StreetProps" / "SOURCES.md"
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_text(sources_md(manifest))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    m = build(args[0] if args else "build/street_props")
    if "--data" in sys.argv:
        write_data(m)
