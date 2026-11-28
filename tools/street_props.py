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
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(1, str(Path(__file__).resolve().parent.parent))
from drone_mesh import Mesh, box, disc, sphere, tube, write_glb  # noqa: E402
from coupledcarla import signalfaces  # noqa: E402

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
    "PoleWhite": ((0.80, 0.80, 0.78), 0.3, 0.35),           # white powder-coated aluminium (flagpoles)
    "SignalBlack": ((0.012, 0.012, 0.013), 0.1, 0.4),
    "SignalYellow": ((0.86, 0.50, 0.0), 0.0, 0.4),          # the Dutch yellow back plate, sRGB ~ (240, 188, 0)
    "SignalWhite": ((0.80, 0.80, 0.80), 0.0, 0.4),          # UK back plates' white border
    "LensRed": ((0.12, 0.004, 0.004), 0.0, 0.1),
    "LensAmber": ((0.14, 0.05, 0.0), 0.0, 0.1),
    "LensGreen": ((0.0, 0.07, 0.045), 0.0, 0.1),
    "LensWhite": ((0.07, 0.07, 0.065), 0.0, 0.1),           # the bar signals' unlit lens (lit white)
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
    # round 2: bikes, columns, signs, road surfaces and grime
    "FrameBlack": ((0.012, 0.012, 0.013), 0.3, 0.35),
    "FrameNavy": ((0.012, 0.022, 0.06), 0.3, 0.35),
    "FrameGrey": ((0.16, 0.165, 0.17), 0.5, 0.35),
    "FrameGreen": ((0.012, 0.045, 0.025), 0.3, 0.35),
    "FrameRed": ((0.17, 0.012, 0.014), 0.3, 0.35),
    "TyreBlack": ((0.02, 0.02, 0.02), 0.0, 0.8),
    "TyreBlue": ((0.02, 0.18, 0.62), 0.0, 0.6),             # the Swapfiets front tyre
    "Chrome": ((0.55, 0.55, 0.56), 1.0, 0.25),
    "Saddle": ((0.025, 0.02, 0.018), 0.0, 0.55),
    "Spokes": ((1.0, 1.0, 1.0), 0.8, 0.3, "spokes"),
    "ColumnGreen": ((0.012, 0.05, 0.03), 0.3, 0.45),        # the column's cast iron, dark green
    "Poster_A": ((1.0, 1.0, 1.0), 0.0, 0.6, "poster_a"),
    "Poster_B": ((1.0, 1.0, 1.0), 0.0, 0.6, "poster_b"),
    "Poster_C": ((1.0, 1.0, 1.0), 0.0, 0.6, "poster_c"),
    "SignBack": ((0.05, 0.05, 0.055), 0.4, 0.5),
    "Clinker": ((1.0, 1.0, 1.0), 0.0, 0.8, "clinker"),      # red-brown clay road clinkers, herringbone
    "Sett": ((1.0, 1.0, 1.0), 0.0, 0.75, "sett"),           # grey granite setts (cobbles)
    "Grime": ((1.0, 1.0, 1.0), 0.0, 0.9, "grime"),
    "Puddle": ((1.0, 1.0, 1.0), 0.2, 0.03, "puddle"),       # wet: nearly mirror-smooth
    "Stains": ((1.0, 1.0, 1.0), 0.0, 0.7, "stains"),
    "Litter": ((1.0, 1.0, 1.0), 0.0, 0.7, "litter"),
}
SIGN_KEYS = ["bakkerij", "cafe", "restaurant", "kapper", "apotheek", "mode", "fietsen", "bloemen", "supermarkt",
             "slagerij", "optiek", "snackbar", "lunchroom", "boeken", "schoenen", "winkel"]
PALETTE.update({f"Sign_{k}": ((1.0, 1.0, 1.0), 0.0, 0.5, f"sign_{k}") for k in SIGN_KEYS})
# round 3: big screens and vertical shop signs (screens.py): emissive, so they read by day and glow at night. The extra
# entries are the emissive strength and the texture's pan (u, v per second): a screen's content scrolls slowly.
SCREEN_KEYS = ["a", "b", "c", "d"]
PALETTE.update({f"Screen_{k.upper()}": ((1.0, 1.0, 1.0), 0.0, 0.25, f"screen_{k}", 6.0, 0.0, 0.02) for k in SCREEN_KEYS})
# vertical shop signs (tategaki) of Japanese streets: generic words, no names or brands (screens.SIGNV_OF)
SIGNV_WORDS = {"izakaya": "居酒屋", "ramen": "ラーメン", "sushi": "寿司", "shokudo": "食堂", "cafe": "カフェ",
               "karaoke": "カラオケ", "kusuri": "くすり", "biyo": "美容室", "shoten": "書店", "fuku": "洋服",
               "mise": "営業中"}
SIGNV_COLOURS = {"izakaya": ((150, 20, 20), (255, 236, 200)), "ramen": ((235, 190, 20), (60, 20, 10)),
                 "sushi": ((245, 245, 240), (20, 30, 60)), "shokudo": ((30, 70, 140), (255, 255, 255)),
                 "cafe": ((60, 40, 30), (250, 230, 200)), "karaoke": ((220, 30, 120), (255, 255, 255)),
                 "kusuri": ((20, 120, 70), (255, 255, 255)), "biyo": ((250, 250, 250), (140, 40, 100)),
                 "shoten": ((20, 60, 40), (240, 220, 160)), "fuku": ((20, 20, 25), (255, 255, 255)),
                 "mise": ((200, 60, 20), (255, 255, 255))}
PALETTE.update({f"SignV_{k}": ((1.0, 1.0, 1.0), 0.0, 0.4, f"signv_{k}", 2.0, 0.0, 0.0) for k in SIGNV_WORDS})
# round 4: generated advertising and shop signage (tools/signage_art.py: invented brands, shop names and ads in the
# local language; the placement is coupledcarla/signage.py). Lit items use the glow material (no pan, except the
# region screens), printed ones the textured decal material.
import signage_art  # noqa: E402
for _it in signage_art.catalog():
    PALETTE[_it["slot"]] = (((1.0, 1.0, 1.0), 0.0, 0.35, _it["tex"], _it["lit"]) + tuple(_it["pan"])) if _it["lit"] \
        else ((1.0, 1.0, 1.0), 0.0, 0.6, _it["tex"])
DECAL = {"Crack", "Patch", "Pothole", "Oil", "Leaves", "RedAsphalt", "Clinker", "Sett", "Grime", "Puddle", "Stains",
         "Litter"}    # masked, two-sided, no collision


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


def bin_post(colour="Anthracite"):
    """A campus / park bin on a post (Vepa, Bammens Aqua type): a rectangular steel body 0.44 x 0.30 m and 0.62 m
    high hanging on a 60 mm square post behind it, under a rounded hood with the opening at the front (model +y);
    top 1.05 m. The TU/e campus's bins (furniture_areas.json)."""
    m = Mesh()
    obox(m, "Galvanised", (0, -0.19, 0.50), (0.06, 0.06, 1.0), np.eye(3))                 # the post
    prism(m, colour, [(-0.22, -0.15), (0.22, -0.15), (0.22, 0.15), (-0.22, 0.15)], 0.28, 0.86)
    obox(m, "BinBlack", (0, 0.152, 0.83), (0.36, 0.006, 0.06), np.eye(3))              # the hood's shadow line
    # the hood: a half cylinder across the body (axis along x), a little proud of it
    n = 10
    for k in range(n):
        a0, a1 = math.pi * k / n, math.pi * (k + 1) / n
        y0, z0 = 0.16 * math.cos(a0), 0.86 + 0.13 * math.sin(a0)
        y1, z1 = 0.16 * math.cos(a1), 0.86 + 0.13 * math.sin(a1)
        c = ((y0 + y1) / 2, (z0 + z1) / 2)
        ln = math.hypot(y1 - y0, z1 - z0)
        ang = math.atan2(z1 - z0, y1 - y0)
        obox(m, colour, (0, c[0], c[1]), (0.46, ln, 0.012), rot(roll=ang))
    for x in (-0.23, 0.23):                                                              # the hood's ends
        prism_pts = [(0.16 * math.cos(math.pi * k / n), 0.86 + 0.13 * math.sin(math.pi * k / n)) for k in range(n + 1)]
        for k in range(n):
            (ya, za), (yb, zb) = prism_pts[k], prism_pts[k + 1]
            quad(m, colour, [(x, 0, 0.86), (x, ya, za), (x, yb, zb), (x, yb, zb)], (1 if x > 0 else -1, 0, 0),
                 both=True)
    return m, {}


def flagpole(h=8.0):
    """A white aluminium flagpole (tapered 90 to 55 mm) with a gilt ball and the halyard, on a ground sleeve."""
    m = Mesh()
    tube(m, "Concrete", (0, 0, 0), (0, 0, 0.05), 0.14, seg=16)
    tube(m, "PoleWhite", (0, 0, 0.05), (0, 0, h), 0.045, 0.0275, seg=16)
    sphere(m, "BinGold", (0, 0, h + 0.05), 0.06, nu=14, nv=8)
    tube(m, "Anthracite", (0.035, 0, 1.2), (0.035, 0, h - 0.1), 0.004, seg=4)             # the halyard
    obox(m, "Galvanised", (0.05, 0, 1.2), (0.03, 0.02, 0.12), np.eye(3))                  # its cleat
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


def signal_pole_plain(mat, h=3.6):
    """A plain traffic-light pole (114 mm, one colour, a cap): grey galvanised in Japan, black in the UK. CARLA 1.0
    has no stand-alone pole mesh for signals.py (0.9's SM_TrafficLight__VPole_Main is gone), so every region with
    regional heads brings its own."""
    m = Mesh()
    tube(m, mat, (0, 0, 0), (0, 0, h), 0.057, seg=16, caps=True)
    tube(m, mat, (0, 0, h), (0, 0, h + 0.05), 0.057, 0.04, seg=16)
    return m, {"top": [0.0, 0.0, h]}


def _face_disc(m, mat, centre, r, seg=28):
    """A disc of radius r facing model -y (Unreal +y) at centre (x, y, z)."""
    sub = Mesh()
    disc(sub, mat, (0, 0, 0), r, seg=seg, both=False)
    R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])        # disc normal +z -> -y
    for v, nn, uv, ix in zip(*[sub.parts[mat][j] for j in range(4)]):
        m.add(mat, v @ R.T + np.asarray(centre, float), nn @ R.T, uv, ix - ix.min())


def pictogram(m, mat, cx, y, cz, d, key):
    """signalfaces.PICTOGRAMS[key] as flat strokes facing model -y in a lens of diameter d centred at (cx, y, cz):
    the same strokes signals.py lights with debug lines in front of it."""
    s = d / 2 * signalfaces.PICTO_FILL
    for prim in signalfaces.PICTOGRAMS[key]:
        if prim[0] == "dot":
            (px, pz), r = prim[1], prim[2]
            _face_disc(m, mat, (cx + px * s, y, cz + pz * s), r * s, seg=12)
            continue
        (x0, z0), (x1, z1), w = prim[1], prim[2], prim[3]
        L = math.hypot(x1 - x0, z1 - z0) or 1e-9
        nx, nz = -(z1 - z0) / L * w * s / 2, (x1 - x0) / L * w * s / 2
        a, b = (cx + x0 * s, cz + z0 * s), (cx + x1 * s, cz + z1 * s)
        quad(m, mat, [(a[0] - nx, y, a[1] - nz), (b[0] - nx, y, b[1] - nz), (b[0] + nx, y, b[1] + nz),
                      (a[0] + nx, y, a[1] + nz)], (0, -1, 0), both=True)


def signal_head(lenses=3, d=0.20, plate="SignalYellow", border=None, horizontal=False, pole_r=0.057, faces=None,
                lens_mat=None):
    """A signal head: a black body with `lenses` round lenses (diameter d: 200 mm Dutch and British car heads,
    cyclist and pedestrian heads; 300 mm Japanese car heads) under visors, on a back plate (Dutch yellow, or black
    with a white border as the UK's; plate None: none, the body on the pole). lens_mat: one material for every lens
    (the bar signals' "LensWhite"), else red, amber, green. faces: a signalfaces pictogram per lens (None: a plain
    lens): a black mask
    over the lens with the pictogram in the lens's colour (the Dutch cyclist head's bicycle, the pedestrian
    head's standing and walking figures), so the unlit lens shows it faintly; signals.py lights the pictogram (or
    a plain lens as a round disc) in front of it. The lenses face model -y (Unreal +y); origin at the middle of
    the back plate's back, which stands pole_r in front of the pole's axis when the head hangs on a pole at x = 0
    (y = 0 is the pole's axis)."""
    m = Mesh()
    pitch = d + 0.06
    n = lenses
    faces = list(faces) if faces else [None] * n
    long_, short = n * pitch + 0.06, d + 0.12
    bw, bh = (long_, short) if horizontal else (short, long_)
    y0 = -pole_r                         # the plate's back
    pw, ph = bw + 0.30, bh + 0.30        # back plate
    if plate is None:
        pw, ph = bw, bh
        obox(m, "SignalBlack", (0, y0 - 0.008, 0), (0.08, 0.016, min(bh, 0.3)), np.eye(3))     # the pole clamp
    elif border:
        obox(m, border, (0, y0 - 0.006, 0), (pw, 0.012, ph), np.eye(3))
        obox(m, plate, (0, y0 - 0.013, 0), (pw - 0.14, 0.004, ph - 0.14), np.eye(3))
    else:
        obox(m, plate, (0, y0 - 0.008, 0), (pw, 0.016, ph), np.eye(3))
    depth = 0.22
    obox(m, "SignalBlack", (0, y0 - 0.016 - depth / 2, 0), (bw, depth, bh), np.eye(3))
    lens_mats = [lens_mat] * n if lens_mat else \
        {3: ["LensRed", "LensAmber", "LensGreen"], 2: ["LensRed", "LensGreen"], 1: ["LensAmber"]}[n]
    centres = []
    for k in range(n):
        off = ((n - 1) / 2 - k) * pitch
        cx, cz = (-off, 0.0) if horizontal else (0.0, off)
        yf = y0 - 0.016 - depth
        _face_disc(m, lens_mats[k], (cx, yf - 0.002, cz), d / 2)
        if faces[k]:                    # the black mask, the pictogram in the lens's colour on it
            ym = yf - 0.002 - signalfaces.MASK_FRONT
            _face_disc(m, "SignalBlack", (cx, ym, cz), d / 2 * 0.97)
            pictogram(m, lens_mats[k], cx, ym - 0.001, cz, d, faces[k])
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
    return m, {"lenses": centres, "plate": [round(pw, 3), round(ph, 3)], "lens_d": d, "faces": faces}


def replacing(made, region, name, old):
    """A head mesh `made` (signal_head's (mesh, extra)) that replaces an older one of another name: its manifest
    entry says which ("replaces": the older mesh's path, lens centres and diameter), so signals.py spawns the older
    one, with its own lens positions, on a server whose content predates this one."""
    m, extra = made
    _, old_extra = old
    extra = dict(extra, replaces={"path": mesh_path(region, "TrafficLight", name), "lenses": old_extra["lenses"],
                                  "lens_d": old_extra["lens_d"], "faces": old_extra["faces"]})
    return m, extra


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


# -- round 2: Dutch street life (bikes, columns, shop signs, clinker roads, grime) ------------------------------------

def torus(m, mat, centre, R, r, nu=22, nv=6):
    """A torus in the model's x-z plane (a wheel's tyre), axis along y."""
    c = np.asarray(centre, float)
    V, N, UV, idx = [], [], [], []
    for i in range(nu + 1):
        u = 2 * math.pi * i / nu
        cu, su = math.cos(u), math.sin(u)
        for j in range(nv + 1):
            v = 2 * math.pi * j / nv
            cv, sv = math.cos(v), math.sin(v)
            n = np.array([cu * cv, sv, su * cv])
            V.append(c + np.array([cu * R, 0, su * R]) + n * r)
            N.append(n)
            UV.append((i / nu, j / nv))
    w = nv + 1
    for i in range(nu):
        for j in range(nv):
            p = i * w + j
            idx += [p, p + w, p + w + 1, p, p + w + 1, p + 1]
    m.add(mat, V, N, UV, idx)


def _disc_xz(m, mat, centre, r, seg=20):
    """A two-sided disc in the x-z plane (spokes, posters)."""
    sub = Mesh()
    disc(sub, mat, (0, 0, 0), r, seg=seg, both=True)
    R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    for v, n, uv, ix in zip(*[sub.parts[mat][k] for k in range(4)]):
        m.add(mat, v @ R.T + np.asarray(centre, float), n @ R.T, uv, ix - ix.min())


SURFACE_WIDTHS = (2, 3, 4, 6, 8, 12)
BIKE_COLOURS = {"Black": "FrameBlack", "Navy": "FrameNavy", "Grey": "FrameGrey", "Green": "FrameGreen",
                "Red": "FrameRed"}


def dutch_bike(frame="FrameBlack", step_through=True, front_tyre="TyreBlack", lean_deg=7.0):
    """A Dutch city bike ("omafiets" with a step-through frame, or the diamond "opafiets"): 28-inch wheels, upright
    swept-back bars, mudguards, a rear rack, a closed chain guard, a kickstand; parked, leaning lean_deg to its left
    (model +y, the kickstand side). 1.85 m long, x forward. front_tyre "TyreBlue" is a Swapfiets."""
    m = Mesh()
    wf, wr, zc = np.array([0.56, 0, 0.355]), np.array([-0.57, 0, 0.355]), 0.355
    for c, tyre in ((wf, front_tyre), (wr, "TyreBlack")):
        torus(m, tyre, c, 0.333, 0.022)
        torus(m, "Chrome", c, 0.305, 0.007, nu=20, nv=4)
        _disc_xz(m, "Spokes", c, 0.30)
        tube(m, "Chrome", c + (0, -0.05, 0), c + (0, 0.05, 0), 0.02, seg=8)
        # mudguard: a thin arc over the top half
        pts = [c + np.array([math.cos(a) * 0.37, 0, math.sin(a) * 0.37])
               for a in np.linspace(math.radians(-20), math.radians(200), 9)]
        for a_, b_ in zip(pts, pts[1:]):
            mid = (a_ + b_) / 2
            ang = math.atan2(b_[2] - a_[2], b_[0] - a_[0])
            obox(m, frame, mid, (np.linalg.norm(b_ - a_) + 0.01, 0.055, 0.008), rot(pitch=-ang))
    bb = np.array([-0.04, 0, 0.30])
    head_lo, head_hi = np.array([0.38, 0, 0.66]), np.array([0.33, 0, 0.88])
    seat_top = np.array([-0.21, 0, 0.84])
    r = 0.019
    tube(m, frame, head_lo, head_hi, 0.024, seg=8)
    tube(m, frame, bb, seat_top, r, seg=8)                                  # seat tube
    polyline_tube(m, frame, [bb, wr + (0, 0.04, 0)], 0.012, seg=6)         # chain stays
    polyline_tube(m, frame, [bb, wr + (0, -0.04, 0)], 0.012, seg=6)
    polyline_tube(m, frame, [seat_top + (0, 0, -0.06), wr + (0, 0.04, 0)], 0.011, seg=6)   # seat stays
    polyline_tube(m, frame, [seat_top + (0, 0, -0.06), wr + (0, -0.04, 0)], 0.011, seg=6)
    if step_through:        # the double low tubes of an omafiets
        for dz in (0.0, 0.07):
            polyline_tube(m, frame, [head_lo + (0, 0, 0.12 + dz), np.array([0.15, 0, 0.42 + dz]), bb + (0.03, 0, dz)],
                          r, seg=6)
    else:
        polyline_tube(m, frame, [head_lo + (0, 0, 0.18), seat_top + (0, 0, -0.04)], r, seg=6)    # top tube
        polyline_tube(m, frame, [head_lo, bb], 0.022, seg=6)                                     # down tube
    for s in (1, -1):       # fork
        polyline_tube(m, frame, [head_lo + (0, s * 0.03, 0), wf + (-0.04, s * 0.05, 0.15), wf + (0, s * 0.05, 0)],
                      0.012, seg=6)
    stem_top = np.array([0.30, 0, 1.06])
    tube(m, "Chrome", head_hi, stem_top, 0.012, seg=6)
    for s in (1, -1):       # upright, swept-back bars and grips
        polyline_tube(m, "Chrome", [stem_top, stem_top + (-0.05, s * 0.18, 0.03), stem_top + (-0.20, s * 0.29, 0.0)],
                      0.011, seg=6)
        tube(m, "Saddle", stem_top + (-0.20, s * 0.27, 0.0), stem_top + (-0.23, s * 0.33, -0.005), 0.017, seg=6)
    tube(m, "Chrome", seat_top, seat_top + (-0.02, 0, 0.10), 0.013, seg=6)          # seat post
    obox(m, "Saddle", seat_top + (-0.05, 0, 0.13), (0.27, 0.19, 0.07), np.eye(3))
    # rear rack over the rear wheel, with stays to the axle
    for s in (1, -1):
        obox(m, frame, (-0.52, s * 0.07, 0.80), (0.48, 0.012, 0.012), np.eye(3))
        polyline_tube(m, frame, [np.array([-0.72, s * 0.07, 0.80]), wr + (0, s * 0.06, 0)], 0.007, seg=4)
    for x in (-0.70, -0.55, -0.40):
        obox(m, frame, (x, 0, 0.80), (0.012, 0.15, 0.012), np.eye(3))
    # closed chain guard (kettingkast) from the bottom bracket to the rear axle, on the right (model -y)
    mid = (bb + wr) / 2 + (0, -0.06, 0)
    ang = math.atan2(wr[2] - bb[2], wr[0] - bb[0])
    obox(m, "FrameBlack", mid, (np.linalg.norm(wr - bb) + 0.24, 0.05, 0.17), rot(pitch=-ang))
    tube(m, "Chrome", bb + (0, -0.12, 0), bb + (0, 0.12, 0), 0.03, seg=8)
    for s, ph in ((1, 0.0), (-1, math.pi)):      # pedals
        a = bb + np.array([0.12 * math.cos(ph), s * 0.13, 0.12 * math.sin(ph)])
        obox(m, "Saddle", a + (0, s * 0.05, 0), (0.1, 0.09, 0.02), np.eye(3))
    tube(m, "Chrome", bb + (-0.10, 0.04, -0.02), np.array([-0.28, 0.12, 0.0]), 0.008, seg=5)   # kickstand
    obox(m, "LampHousing", head_lo + (0.07, 0, 0.0), (0.06, 0.06, 0.06), np.eye(3))       # front light
    obox(m, "LensRed", (-0.78, 0, 0.82), (0.02, 0.05, 0.03), np.eye(3))                    # rear light
    if lean_deg:        # stand on the tyres' contact line, leaning to the kickstand side
        R = rot(roll=math.radians(lean_deg))
        for k, (vs, ns, uvs, ids, _) in list(m.parts.items()):
            m.parts[k][0] = [v @ R.T for v in vs]
            m.parts[k][1] = [n @ R.T for n in ns]
    return m, {"length": 1.85}


def ad_column(poster="Poster_A"):
    """A Morris (advertising) column, "reclamezuil": a 1.2 m drum of posters (a texture of three posters around it)
    on a dark green plinth under a fluted cornice and a dome, 3.6 m."""
    m = Mesh()
    tube(m, "ColumnGreen", (0, 0, 0), (0, 0, 0.35), 0.66, 0.64, seg=32)
    tube(m, "ColumnGreen", (0, 0, 0.35), (0, 0, 0.42), 0.62, 0.62, seg=32)
    # the poster drum: its own UVs, u around once, v bottom to top
    n, z0, z1, R = 36, 0.42, 2.95, 0.6
    V, N, UV, idx = [], [], [], []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        d = np.array([math.cos(a), math.sin(a), 0.0])
        V += [d * R + (0, 0, z0), d * R + (0, 0, z1)]
        N += [d, d]
        UV += [(i / n, 1.0), (i / n, 0.0)]
    for i in range(n):
        p = 2 * i
        idx += [p, p + 2, p + 3, p, p + 3, p + 1]
    m.add(poster, V, N, UV, idx)
    tube(m, "ColumnGreen", (0, 0, 2.95), (0, 0, 3.05), 0.62, 0.70, seg=32)
    tube(m, "ColumnGreen", (0, 0, 3.05), (0, 0, 3.15), 0.70, 0.70, seg=32)
    sphere(m, "ColumnGreen", (0, 0, 3.15), 0.62, nu=32, nv=6, z_scale=0.55, half=True)
    tube(m, "ColumnGreen", (0, 0, 3.45), (0, 0, 3.62), 0.04, 0.02, seg=8)
    sphere(m, "ColumnGreen", (0, 0, 3.63), 0.05, nu=10, nv=5)
    return m, {}


def shop_sign(face, width=3.0, height=0.6, depth=0.08):
    """A fascia sign: a board width x height x depth m (a lit signboard box: 0.2 m and more), its face (texture)
    towards model +y (Unreal -y, away from the wall: placed with yaw so that -y in Unreal points out of the facade),
    origin at the middle of its back."""
    m = Mesh()
    obox(m, "SignBack", (0, depth / 2, 0), (width, depth, height), np.eye(3))
    y = depth + 0.0005
    quad(m, face, [(width / 2 - 0.02, y, -height / 2 + 0.02), (-width / 2 + 0.02, y, -height / 2 + 0.02),
                   (-width / 2 + 0.02, y, height / 2 - 0.02), (width / 2 - 0.02, y, height / 2 - 0.02)],
         (0, 1, 0), ((0, 1), (1, 1), (1, 0), (0, 0)))
    return m, {"width": width, "height": height}


def screen(face, width=4.0, height=2.25, depth=0.12):
    """A big screen on a facade: a dark housing width x height x depth, its face (an emissive texture that scrolls,
    M_CC_Street_Glow) towards model +y (out of the wall, like shop_sign); origin at the middle of its back. Placed at
    the scale of the screen it stands for (screens.py: 6-18 m wide)."""
    m = Mesh()
    obox(m, "SignBack", (0, depth / 2, 0), (width + 0.08, depth, height + 0.08), np.eye(3))
    quad(m, face, [(width / 2, depth + 0.001, -height / 2), (-width / 2, depth + 0.001, -height / 2),
                   (-width / 2, depth + 0.001, height / 2), (width / 2, depth + 0.001, height / 2)],
         (0, 1, 0), ((0, 1), (1, 1), (1, 0), (0, 0)))
    return m, {"width": width, "height": height}


def blade_sign(face, out=0.7, height=3.6, thick=0.12, gap=0.15):
    """A vertical shop sign (tategaki) sticking out of a facade: a board `out` m from the wall (after a `gap` on two
    brackets) by `height`, its text on both faces, readable from either direction along the street. The wall is the
    model's y = 0 plane, the board along +y (out of the wall, as shop_sign's face); origin at the wall, mid height."""
    m = Mesh()
    y0, y1, t, h = gap, gap + out, thick / 2, height / 2
    obox(m, "SignBack", (0, (y0 + y1) / 2, 0), (thick, out, height), np.eye(3))
    for zb in (-h + 0.25, h - 0.25):
        obox(m, "SignBack", (0, y0 / 2, zb), (0.05, y0, 0.05), np.eye(3))
    # +x face: u along +y (the viewer's right), v top to bottom; -x face: u along -y
    quad(m, face, [(t + 0.001, y0, -h), (t + 0.001, y1, -h), (t + 0.001, y1, h), (t + 0.001, y0, h)], (1, 0, 0),
         ((0, 1), (1, 1), (1, 0), (0, 0)))
    quad(m, face, [(-t - 0.001, y1, -h), (-t - 0.001, y0, -h), (-t - 0.001, y0, h), (-t - 0.001, y1, h)], (-1, 0, 0),
         ((0, 1), (1, 1), (1, 0), (0, 0)))
    return m, {"out": y1}


def face2(m, face, w, h, y, zc, xc=0.0, back=True):
    """A texture face w x h in the plane y (towards +y), centred at (xc, zc); back: the same face towards -y at -y,
    readable from that side too (u to the viewer's right on both)."""
    x0, x1, z0, z1 = xc - w / 2, xc + w / 2, zc - h / 2, zc + h / 2
    quad(m, face, [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)], (0, 1, 0), ((0, 1), (1, 1), (1, 0), (0, 0)))
    if back:
        quad(m, face, [(x0, -y, z0), (x1, -y, z0), (x1, -y, z1), (x0, -y, z1)], (0, -1, 0),
             ((0, 1), (1, 1), (1, 0), (0, 0)))


def nobori(face, h=2.7, fw=0.45, fh=1.8):
    """A cloth banner (nobori) on a thin pole in a weighted foot: the flag beside the pole in the model's x-z plane,
    its text readable from both sides (placed across the pavement, so passers-by see it)."""
    m = Mesh()
    tube(m, "JPGrey", (0, 0, 0.0), (0, 0, 0.12), 0.14, 0.12, seg=16)
    tube(m, "JPGrey", (0, 0, 0.12), (0, 0, h), 0.016, seg=8)
    tube(m, "JPGrey", (0, 0, h - 0.06), (fw + 0.04, 0, h - 0.06), 0.01, seg=6)
    face2(m, face, fw, fh, 0.002, h - 0.1 - fh / 2, 0.03 + fw / 2)
    return m, {}


def stand_box(face, w=0.45, d=0.22, h=1.1):
    """A lit stand sign on the pavement: a box on a flat foot, the face on both broad sides."""
    m = Mesh()
    obox(m, "SignBack", (0, 0, 0.03), (w + 0.1, d + 0.15, 0.06), np.eye(3))
    obox(m, "SignBack", (0, 0, 0.08 + h / 2), (w, d, h), np.eye(3))
    face2(m, face, w - 0.03, h - 0.03, d / 2 + 0.001, 0.08 + h / 2)
    return m, {}


def a_board(face, w=0.6, h=1.0, spread=0.5):
    """A folding A-board (pavement chalkboard): two boards leaning on each other, the face on both outer sides."""
    m = Mesh()
    yb, yt = spread / 2, 0.03
    for s_ in (1, -1):
        pts = [(w / 2, s_ * yb, 0.0), (-w / 2, s_ * yb, 0.0), (-w / 2, s_ * yt, h), (w / 2, s_ * yt, h)]
        if s_ < 0:
            pts = [(-w / 2, -yb, 0.0), (w / 2, -yb, 0.0), (w / 2, -yt, h), (-w / 2, -yt, h)]
        nrm = np.array([0.0, s_ * h, (yb - yt)]) / math.hypot(h, yb - yt)
        quad(m, face, pts, nrm, ((0, 1), (1, 1), (1, 0), (0, 0)))
        m.add("SignBack", pts, np.tile(-nrm, (4, 1)), ((0, 1), (1, 1), (1, 0), (0, 0)), [0, 2, 1, 0, 3, 2])  # inside
    return m, {}


def mupi(face, w=1.18, h=1.76):
    """A lit poster case (MUPI, abri panel) on a foot: the poster on both sides."""
    m = Mesh()
    obox(m, "SignBack", (0, 0, 0.18), (0.3, 0.2, 0.36), np.eye(3))
    obox(m, "SignBack", (0, 0, 0.36 + (h + 0.2) / 2), (w + 0.1, 0.16, h + 0.2), np.eye(3))
    face2(m, face, w, h, 0.081, 0.36 + (h + 0.2) / 2)
    return m, {}


def billboard_stand(face, w=6.0, h=3.0, clear=2.6):
    """A roadside billboard on two posts: the poster towards +y, a plain back."""
    m = Mesh()
    for x in (-w * 0.32, w * 0.32):
        tube(m, "Galvanised", (x, -0.2, 0.0), (x, -0.2, clear + h / 2), 0.11, seg=12)
    obox(m, "SignBack", (0, 0, clear + h / 2), (w + 0.2, 0.22, h + 0.2), np.eye(3))
    face2(m, face, w, h, 0.111, clear + h / 2, back=False)
    return m, {}


def roof_billboard(face, w=9.0, h=3.0, clear=2.0):
    """A rooftop billboard on a steel frame (legs, braces), the poster towards +y; stands on the roof (z = 0)."""
    m = Mesh()
    for x in np.linspace(-w / 2 + 0.3, w / 2 - 0.3, 4):
        tube(m, "Galvanised", (x, -0.15, 0.0), (x, -0.15, clear + h), 0.07, seg=8)
        tube(m, "Galvanised", (x, -1.4, 0.0), (x, -0.15, clear + h * 0.6), 0.05, seg=8)
    tube(m, "Galvanised", (-w / 2, -0.15, clear * 0.5), (w / 2, -0.15, clear * 0.5), 0.05, seg=8)
    obox(m, "SignBack", (0, 0, clear + h / 2), (w + 0.2, 0.2, h + 0.2), np.eye(3))
    face2(m, face, w, h, 0.101, clear + h / 2, back=False)
    return m, {"out": 0.1}


def surface_strip(mat, w=1.0, l=2.0, tile_m=2.0, z=0.009):
    """A road-surface overlay w x l (scaled by the lane width at placement: one per 1.9 widths), its texture
    repeating every tile_m metres at scale 1."""
    m = Mesh()
    flat(m, mat, (0, 0, 0), (l, w), z=z, uv_scale=(l / tile_m, w / tile_m))
    return m, {}


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
    add("BinPost_Anthracite", "NL", "Static", "bin", lambda: bin_post("Anthracite"), "SM_TrashCan02",
        "campus / park bin on a post (TU/e campus: data/furniture_areas.json)")
    add("Nietje_x1", "NL", "Static", "bike_stand", lambda: nietjes(1), "SM_BikeParking01")
    add("Nietje_x3", "NL", "Static", "bike_stand", lambda: nietjes(3), "SM_BikeParking01")
    add("Nietje_x6", "NL", "Static", "bike_stand", lambda: nietjes(6), "SM_BikeParking02")
    # Dutch traffic lights (TrafficLight): for signals.py's head style
    add("SignalPole", "NL", "TrafficLight", "signal_pole", signal_pole)
    # 200 mm lenses (Dutch urban car heads; 300 mm ones are for roads outside built-up areas): until 2026-11-28 a
    # 300 mm head as SignalCar, which signals.py still spawns where the server's content predates SignalCar200
    add("SignalCar200", "NL", "TrafficLight", "signal_car",
        lambda: replacing(signal_head(3, 0.20), "NL", "SignalCar", signal_head(3, 0.30)), None,
        "200 mm car head, yellow back plate")
    add("SignalBike", "NL", "TrafficLight", "signal_bike",
        lambda: signal_head(3, 0.20, faces=signalfaces.faces_for("bike", 3)), None,
        "200 mm cyclist head, a bicycle in each lens")
    add("SignalPed", "NL", "TrafficLight", "signal_ped",
        lambda: signal_head(2, 0.20, faces=signalfaces.faces_for("ped", 2)), None,
        "200 mm pedestrian head, standing red and walking green figures")
    add("SignalBus", "NL", "TrafficLight", "signal_bus",
        lambda: signal_head(3, 0.20, faces=signalfaces.faces_for("bus", 3), lens_mat="LensWhite"), None,
        "public-transport bar signal (OV-licht): white horizontal bar, dot, vertical bar")
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
    # round 2: Dutch street life
    for col, slot in BIKE_COLOURS.items():
        add(f"Bike_{col}", "NL", "Static", "bike", lambda slot=slot: dutch_bike(slot, True))
        add(f"BikeMen_{col}", "NL", "Static", "bike", lambda slot=slot: dutch_bike(slot, False))
    add("Bike_Swapfiets", "NL", "Static", "bike_swapfiets", lambda: dutch_bike("FrameBlack", True, "TyreBlue"))
    add("BikeMen_Swapfiets", "NL", "Static", "bike_swapfiets", lambda: dutch_bike("FrameBlack", False, "TyreBlue"))
    add("Flagpole8", "Any", "Static", "flagpole", lambda: flagpole(8.0), None, "8 m aluminium flagpole (OSM man_made=flagpole)")
    for key in "ABC":
        add(f"AdColumn_{key}", "Any", "Static", "ad_column", lambda key=key: ad_column(f"Poster_{key}"),
            "SM_Billboard1", "Morris column (reclamezuil, Litfass)")
    for k in SIGN_KEYS:
        add(f"ShopSign_{k}", "NL", "Static", "shop_sign", lambda k=k: shop_sign(f"Sign_{k}"))
    for w in SURFACE_WIDTHS:      # widths so that a strip stands at about scale 1 (the stones keep their size)
        add(f"Clinker_W{w}", "NL", "Road", "clinker_road", lambda w=w: surface_strip("Clinker", w, 2 * w))
        add(f"Sett_W{w}", "Any", "Road", "sett_road", lambda w=w: surface_strip("Sett", w, 2 * w))
    add("Grime_A", "Any", "Road", "grime", lambda: decal("Grime", (6.0, 0.6), z=0.010))
    add("Puddle_A", "Any", "Road", "puddle", lambda: decal("Puddle", (1.4, 1.0), z=0.011))
    add("Stains_A", "Any", "Road", "stains", lambda: decal("Stains", (1.5, 1.5), z=0.006))
    add("Litter_A", "Any", "Road", "litter", lambda: decal("Litter", (1.5, 1.5), z=0.012))
    # UK and Japan
    add("LampColumn8", "UK", "Pole", "lamp_arm", lambda: lamp_uk(8.0), "SM_StreetLight06_A", "BS EN 40 column")
    add("BinCast", "UK", "Static", "bin", bin_uk, "SM_TrashCan01")
    add("SignalPole", "UK", "TrafficLight", "signal_pole", lambda: signal_pole_plain("PoleBlack"))
    add("SignalCar200", "UK", "TrafficLight", "signal_car",       # 200 mm (300 mm on fast roads only)
        lambda: replacing(signal_head(3, 0.20, plate="SignalBlack", border="SignalWhite"), "UK", "SignalCar",
                          signal_head(3, 0.30, plate="SignalBlack", border="SignalWhite")), None,
        "200 mm car head, black back plate with a white border")
    add("SignalPed", "UK", "TrafficLight", "signal_ped",
        lambda: signal_head(2, 0.20, plate=None, faces=signalfaces.faces_for("ped", 2)), None,
        "far-side pedestrian head: red standing, green walking figure")
    add("LampGooseneck8", "JP", "Pole", "lamp_arm", lambda: lamp_jp(8.0), "SM_StreetLight06_A")
    # round 3: big screens (any region, screens.py) and Japanese vertical shop signs
    for k in SCREEN_KEYS:
        add(f"Screen_{k.upper()}", "Any", "Static", "screen", lambda k=k: screen(f"Screen_{k.upper()}"), None,
            "facade screen, 16:9, scrolling generic content (scaled to 6-18 m)")
        add(f"ScreenTall_{k.upper()}", "Any", "Static", "screen_tall",
            lambda k=k: screen(f"Screen_{k.upper()}", 2.25, 4.0), None, "facade screen, 9:16")
    for k in SIGNV_WORDS:
        add(f"SignV_{k}", "JP", "Static", "sign_vertical", lambda k=k: blade_sign(f"SignV_{k}"), None,
            f"vertical shop sign: {SIGNV_WORDS[k]} (a generic word)")
    # round 4: generated advertising and shop signage, per locale (tools/signage_art.py, coupledcarla/signage.py)
    makers = {"shop_sign": shop_sign, "screen": screen, "blade_sign": blade_sign, "nobori": nobori,
              "stand_box": stand_box, "a_board": a_board, "mupi": mupi, "billboard_stand": billboard_stand,
              "roof_billboard": roof_billboard, "ad_column": ad_column}
    for it in signage_art.catalog():
        add(it["name"], it["loc"], it["folder"], it["role"],
            lambda it=it: makers[it["maker"]](it["slot"], **it["kw"]), None, it["note"])
    add("SignalPole", "JP", "TrafficLight", "signal_pole", lambda: signal_pole_plain("Galvanised"))
    add("SignalCar", "JP", "TrafficLight", "signal_car",            # Japan's are 250-300 mm
        lambda: signal_head(3, 0.30, plate="SignalBlack", horizontal=True), None, "300 mm horizontal car head")
    add("SignalPed", "JP", "TrafficLight", "signal_ped",
        lambda: signal_head(2, 0.25, plate=None, faces=signalfaces.faces_for("ped", 2)), None,
        "pedestrian head: red standing, blue-green walking figure")
    # anywhere else: a plain dark pole and a pedestrian head without a back plate (CARLA has neither)
    add("SignalPole", "Any", "TrafficLight", "signal_pole", lambda: signal_pole_plain("Anthracite", 3.0))
    add("SignalPed", "Any", "TrafficLight", "signal_ped",
        lambda: signal_head(2, 0.20, plate=None, faces=signalfaces.faces_for("ped", 2)), None,
        "generic pedestrian head: red standing, green walking figure")
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


# Dutch shop-sign words by OSM shop / amenity value: generic trade names, no brands (streetlook.SIGN_OF)
SIGN_WORDS = {"bakkerij": "BAKKERIJ", "cafe": "CAFÉ", "restaurant": "RESTAURANT", "kapper": "KAPSALON",
              "apotheek": "APOTHEEK", "mode": "MODE", "fietsen": "FIETSEN", "bloemen": "BLOEMEN",
              "supermarkt": "SUPERMARKT", "slagerij": "SLAGERIJ", "optiek": "OPTIEK", "snackbar": "SNACKBAR",
              "lunchroom": "LUNCHROOM", "boeken": "BOEKHANDEL", "schoenen": "SCHOENEN", "winkel": "WINKEL"}
SIGN_STYLE = [((20, 40, 70), (235, 225, 200)), ((120, 20, 30), (245, 240, 230)), ((240, 238, 230), (30, 30, 35)),
              ((25, 70, 45), (235, 220, 160)), ((15, 15, 18), (225, 190, 90)), ((200, 120, 40), (255, 255, 255))]


def textures2(out, seed=11):
    """Round 2 textures: spokes, posters, shop-sign faces, clinker and sett paving, grime, litter, puddles, stains,
    bike tyres (RGBA PNGs, alpha = opacity mask)."""
    from PIL import Image, ImageDraw, ImageFilter, ImageFont
    import matplotlib
    rng = np.random.default_rng(seed)
    out = Path(out)
    font_dir = Path(matplotlib.__file__).parent / "mpl-data" / "fonts" / "ttf"     # DejaVu (Bitstream Vera licence)

    def save(name, img):
        img.save(out / f"T_CC_{name}.png")

    def noise(s, octaves=5):
        acc = np.zeros((s, s))
        for o in range(octaves):
            n = 4 * 2 ** o
            g = rng.random((n + 1, n + 1))
            acc += np.asarray(Image.fromarray((g * 255).astype(np.uint8)).resize((s, s), Image.BICUBIC),
                              float) / 255.0 / 2 ** o
        return acc / acc.max()
    # spokes: 36 thin radial lines on transparent, a hub ring
    s = 512
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    for k in range(36):
        a = 2 * math.pi * k / 36
        d.line([(s / 2 + math.cos(a) * 18, s / 2 + math.sin(a) * 18),
                (s / 2 + math.cos(a + 0.35) * 250, s / 2 + math.sin(a + 0.35) * 250)], fill=(190, 190, 195, 255),
               width=2)
    d.ellipse([s / 2 - 22, s / 2 - 22, s / 2 + 22, s / 2 + 22], outline=(170, 170, 175, 255), width=6)
    save("spokes", img)
    # posters around a column: three posters side by side (1024 x 512 wraps the drum once)
    font = ImageFont.truetype(str(font_dir / "DejaVuSans-Bold.ttf"), 54)
    small = ImageFont.truetype(str(font_dir / "DejaVuSans.ttf"), 26)
    words = [["CONCERT", "TOUR 2026"], ["EXPO", "MUSEUM"], ["FESTIVAL", "ZOMER"], ["THEATER", "PREMIERE"],
             ["OPERA", "VOORSTELLING"], ["MARKT", "ZATERDAG"], ["FILM", "NU IN DE BIOS"], ["DANS", "LIVE"],
             ["KERMIS", "STADSFEEST"]]
    for v, key in enumerate("ABC"):
        img = Image.new("RGB", (1024, 512), (230, 228, 220))
        d = ImageDraw.Draw(img)
        for p in range(3):
            x0 = p * 1024 // 3
            bg = tuple(int(c) for c in rng.integers(30, 230, 3))
            d.rectangle([x0 + 6, 14, x0 + 1024 // 3 - 6, 498], fill=bg)
            for _ in range(3):      # abstract shapes
                cx, cy, r = x0 + rng.integers(40, 300), rng.integers(140, 380), rng.integers(30, 110)
                col = tuple(int(c) for c in rng.integers(0, 255, 3))
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
            w1, w2 = words[(v * 3 + p) % len(words)]
            fg = (255, 255, 255) if sum(bg) < 380 else (20, 20, 20)
            d.text((x0 + 22, 30), w1, font=font, fill=fg)
            d.text((x0 + 24, 440), w2, font=small, fill=fg)
        img = img.filter(ImageFilter.GaussianBlur(0.6))
        save(f"poster_{key.lower()}", img)
    # shop-sign faces: the trade word on a plain board, a thin border
    big = ImageFont.truetype(str(font_dir / "DejaVuSans-Bold.ttf"), 150)
    for i, (key, word) in enumerate(SIGN_WORDS.items()):
        bg, fg = SIGN_STYLE[i % len(SIGN_STYLE)]
        img = Image.new("RGB", (1024, 205), bg)
        d = ImageDraw.Draw(img)
        d.rectangle([6, 6, 1017, 198], outline=fg, width=5)
        size = 150
        f = big
        while d.textlength(word, font=f) > 940 and size > 40:
            size -= 8
            f = ImageFont.truetype(str(font_dir / "DejaVuSans-Bold.ttf"), size)
        tw = d.textlength(word, font=f)
        d.text(((1024 - tw) / 2, (205 - size * 1.17) / 2), word, font=f, fill=fg)
        save(f"sign_{key}", img)
    # red-brown clay clinkers (waalformaat 200 x 100 mm) in herringbone, 2 m per texture: bricks of 0.1 m units,
    # H_i = [i, i+2] x [i, i+1], V_i = [i-1, i] x [i, i+2], bands every 4 units (it tiles over 20 x 20 units)
    s, U = 1024, 1024 / 20
    img = Image.new("RGB", (s, s), (82, 66, 58))
    d = ImageDraw.Draw(img)
    for b in range(-12, 12):
        for i in range(-30, 30):
            for (x0, x1, y0, y1) in ((i + 4 * b, i + 2 + 4 * b, i, i + 1), (i - 1 + 4 * b, i + 4 * b, i, i + 2)):
                for ox in (0, -20, 20):
                    for oy in (0, -20, 20):
                        X0, X1, Y0, Y1 = (x0 + ox) * U, (x1 + ox) * U, (y0 + oy) * U, (y1 + oy) * U
                        if X1 < 0 or Y1 < 0 or X0 > s or Y0 > s:
                            continue
                        t = rng.normal(0, 1)
                        col = (int(np.clip(132 + 18 * t, 60, 200)), int(np.clip(64 + 9 * t, 30, 120)),
                               int(np.clip(48 + 7 * t, 20, 100)))
                        d.rectangle([X0 + 2, Y0 + 2, X1 - 2, Y1 - 2], fill=col)
    arr = np.asarray(img, float) * (0.88 + 0.22 * noise(s, 6)[..., None])
    Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)).save(out / "T_CC_clinker.png")
    # grey granite setts (10 x 10 cm, rows), 2 m per texture
    img = Image.new("RGB", (s, s), (60, 60, 58))
    d = ImageDraw.Draw(img)
    for r_ in range(20):
        off = (U / 2) * (r_ % 2)
        for c_ in range(-1, 21):
            g = int(np.clip(rng.normal(118, 14), 70, 170))
            x0 = c_ * U + off
            d.rounded_rectangle([x0 + 3, r_ * U + 3, x0 + U - 3, r_ * U + U - 3], radius=8, fill=(g, g - 2, g - 6))
    arr = np.asarray(img, float) * (0.85 + 0.3 * noise(s, 6)[..., None])
    Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)).save(out / "T_CC_sett.png")
    # gutter grime: a dark band fading out from the kerb (v = 0) over the strip's width, streaks along it
    s = 512
    yy, xx = np.mgrid[0:s, 0:s] / s
    n1 = noise(s, 6)
    a = np.clip((1.0 - yy) ** 2.2 * (0.6 + 0.8 * n1) * 255 * 0.9, 0, 210)
    rgb = np.dstack([40 + 20 * n1, 36 + 16 * n1, 30 + 12 * n1])
    Image.fromarray(np.dstack([rgb, a]).astype(np.uint8), "RGBA").save(out / "T_CC_grime.png")
    # puddle: a dark, irregular wet patch
    r = np.hypot(xx - 0.5, yy - 0.5) / 0.5
    edge = 0.62 + 0.3 * noise(s, 3)
    a = np.clip((edge - r) * 6, 0, 1) * 235
    rgb = np.dstack([np.full((s, s), 28.0), np.full((s, s), 30.0), np.full((s, s), 32.0)])
    Image.fromarray(np.dstack([rgb, a]).astype(np.uint8), "RGBA").save(out / "T_CC_puddle.png")
    # stains: chewing gum spots and a dark blotch
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    for _ in range(90):
        x, y, rr = rng.uniform(20, s - 20), rng.uniform(20, s - 20), rng.uniform(2.5, 6)
        g = int(rng.uniform(25, 70))
        d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=(g, g, g, 230))
    blot = np.clip(1.0 - r / (0.45 + 0.25 * noise(s, 3)), 0, 1) ** 0.8 * 120
    base = np.asarray(img, float)
    base[..., 3] = np.maximum(base[..., 3], blot)
    base[..., :3] = np.where(base[..., 3:4] > 0, np.maximum(base[..., :3], 25), base[..., :3])
    Image.fromarray(base.astype(np.uint8), "RGBA").save(out / "T_CC_stains.png")
    # litter: cigarette butts, paper scraps, a can, bottle caps on transparent
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    for _ in range(40):         # butts: 25 mm long at 512 px per 1.5 m
        x, y, a_ = rng.uniform(20, s - 20), rng.uniform(20, s - 20), rng.uniform(0, math.pi)
        L = 8.5
        x1, y1 = x + math.cos(a_) * L, y + math.sin(a_) * L
        d.line([(x, y), (x1, y1)], fill=(235, 230, 220, 255), width=3)
        d.line([(x1, y1), (x1 + math.cos(a_) * 3, y1 + math.sin(a_) * 3)], fill=(200, 140, 60, 255), width=3)
    for _ in range(9):          # paper scraps
        x, y = rng.uniform(30, s - 30), rng.uniform(30, s - 30)
        pts = [(x + rng.uniform(-16, 16), y + rng.uniform(-16, 16)) for _ in range(5)]
        g = int(rng.uniform(190, 245))
        d.polygon(pts, fill=(g, g, g - 10, 255))
    for _ in range(2):
        x, y = rng.uniform(40, s - 40), rng.uniform(40, s - 40)
        d.rectangle([x, y, x + 22, y + 11], fill=(160, 20, 20, 255))
    for _ in range(6):
        x, y = rng.uniform(20, s - 20), rng.uniform(20, s - 20)
        d.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(180, 170, 60, 255))
    save("litter", img)


TEXTURES3 = Path(__file__).resolve().parent.parent / "unreal" / "StreetProps" / "textures"
CJK_FONTS = ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",       # Noto Sans CJK (SIL Open Font Licence)
             "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc", "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Bold.ttc")


def textures3(out, render=False, seed=23):
    """Round 3 textures: the screens' scrolling content (four 16:9 frames stacked, so a vertical pan loops) and the
    vertical shop signs' faces. Generic only: colour, shapes and common words, no brand, logo, name or real ad.
    The Japanese text needs a CJK font (Noto Sans CJK, SIL OFL), which Windows has not: the PNGs are kept in git
    (unreal/StreetProps/textures, made with --render-textures where the font is) and copied from there."""
    import shutil
    names = [f"screen_{k}" for k in SCREEN_KEYS] + [f"signv_{k}" for k in SIGNV_WORDS]
    if not render and all((TEXTURES3 / f"T_CC_{n}.png").exists() for n in names):
        for n in names:
            shutil.copyfile(TEXTURES3 / f"T_CC_{n}.png", out / f"T_CC_{n}.png")
        return
    from PIL import Image, ImageDraw, ImageFont
    import matplotlib
    font_path = os.environ.get("CC_CJK_FONT") or next((f for f in CJK_FONTS if Path(f).exists()), None)
    if font_path is None:
        raise SystemExit("textures3: no CJK font (Noto Sans CJK) for the Japanese signs; set CC_CJK_FONT, or use the "
                         f"PNGs in {TEXTURES3}")
    latin = str(Path(matplotlib.__file__).parent / "mpl-data" / "fonts" / "ttf" / "DejaVuSans-Bold.ttf")

    def cjk(size):
        return ImageFont.truetype(font_path, size, index=0)     # face 0 of the collection: Japanese
    rng = np.random.default_rng(seed)
    W, H = 768, 432
    words = [("NEW", "新発売"), ("OPEN", "ようこそ"), ("SALE", "セール"), ("NEWS", "ニュース"), ("LIVE", "本日"),
             ("MUSIC", "音楽"), ("TODAY", "天気"), ("CITY", "夏")]
    palettes = [((255, 70, 120), (40, 20, 120)), ((20, 200, 255), (10, 30, 80)), ((255, 200, 40), (200, 40, 20)),
                ((80, 255, 160), (10, 60, 90)), ((250, 250, 250), (230, 30, 60)), ((170, 90, 255), (20, 10, 40))]

    def frame(kind):
        img = Image.new("RGB", (W, H))
        d = ImageDraw.Draw(img)
        a, b = palettes[rng.integers(len(palettes))]
        for y in range(H):              # a vertical gradient between the two colours
            t = y / (H - 1)
            d.line([(0, y), (W, y)], fill=tuple(int(a[i] * (1 - t) + b[i] * t) for i in range(3)))
        if kind == 0:                   # a big common word, Latin and Japanese, and lines of "small print"
            lw, jw = words[rng.integers(len(words))]
            d.text((40, 60), lw, font=ImageFont.truetype(latin, 150), fill=(255, 255, 255))
            d.text((44, 250), jw, font=cjk(96), fill=(255, 255, 255))
            for k in range(3):
                d.rectangle([W - 260, 300 + k * 30, W - 40 - rng.integers(0, 120), 314 + k * 30], fill=(255, 255, 255))
        elif kind == 1:                 # abstract: overlapping discs
            for _ in range(9):
                r = int(rng.integers(40, 170))
                x, y = int(rng.integers(0, W)), int(rng.integers(0, H))
                c = tuple(int(v) for v in rng.integers(60, 256, 3))
                d.ellipse([x - r, y - r, x + r, y + r], fill=c)
        elif kind == 2:                 # a ticker: a dark panel, a headline bar and rows of text-like bars
            d.rectangle([0, 0, W, H], fill=(12, 14, 22))
            d.rectangle([0, 0, W, 86], fill=b)
            d.text((24, 10), words[rng.integers(len(words))][1], font=cjk(60), fill=(255, 255, 255))
            for k in range(7):
                y = 120 + k * 42
                d.rectangle([24, y, 24 + int(rng.integers(260, 700)), y + 18], fill=(200, 205, 215))
        else:                           # bold stripes and a square "product" block
            for k in range(0, W, 64):
                if (k // 64) % 2:
                    d.polygon([(k, 0), (k + 48, 0), (k - 120, H), (k - 168, H)], fill=a)
            d.rounded_rectangle([W // 2 - 110, H // 2 - 110, W // 2 + 110, H // 2 + 110], 30, fill=(250, 250, 250))
            d.text((W // 2 - 80, H // 2 - 58), words[rng.integers(len(words))][1][:2], font=cjk(110), fill=b)
        return img
    for i, k in enumerate(SCREEN_KEYS):
        atlas = Image.new("RGB", (W, 4 * H))
        for j in range(4):
            atlas.paste(frame((i + j) % 4), (0, j * H))
        atlas.save(out / f"T_CC_screen_{k}.png", optimize=True)
    for k, word in SIGNV_WORDS.items():
        bg, fg = SIGNV_COLOURS[k]
        img = Image.new("RGB", (256, 1280), bg)
        d = ImageDraw.Draw(img)
        d.rectangle([10, 10, 245, 1269], outline=fg, width=8)
        chars = [("丨" if c == "ー" else c) for c in word]      # the long vowel mark stands upright in vertical text
        size = min(190, int(1150 / len(chars) * 0.86))
        f = cjk(size)
        step = 1150 / len(chars)
        for n, c in enumerate(chars):
            w = d.textlength(c, font=f)
            d.text(((256 - w) / 2, 60 + n * step + (step - size * 1.16) / 2), c, font=f, fill=fg)
        img.save(out / f"T_CC_signv_{k}.png", optimize=True)


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
    textures2(out)
    textures3(out)
    signage_art.textures(out)
    manifest = {"root": ROOT, "palette": {k: list(v[0]) + list(v[1:3]) + list(v[3:]) for k, v in PALETTE.items()},
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
- Shop signs and posters: lettering in DejaVu Sans Bold (Bitstream Vera / DejaVu licence, free to embed; the copy
  in matplotlib), generic Dutch trade words and invented poster titles, no brand or real event. Bikes: a generic
  Dutch city bike; the Swapfiets look is only its well-known blue front tyre (no logo).
- Generated signage and advertising (round 4, `tools/signage_art.py`, coupledcarla/signage.py): every shop name,
  brand, product, film, show and slogan is invented and the layouts are procedural; no real advertisement, logo or
  trademark and no photo is in them (open street photos were looked at for the style only). Letters: Noto Sans CJK JP
  Bold and Noto Serif CJK JP Bold (Adobe / Google, SIL Open Font Licence 1.1; subsets with only the characters drawn
  in `unreal/StreetProps/fonts`, licence beside them), DejaVu Sans / Serif and STIX (free licences, from matplotlib).
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
    if "--render-textures" in sys.argv:      # the round 3 PNGs into git (needs a CJK font: Noto Sans CJK)
        TEXTURES3.mkdir(parents=True, exist_ok=True)
        textures3(TEXTURES3, render=True)
    m = build(args[0] if args else "build/street_props")
    if "--data" in sys.argv:
        write_data(m)
