"""The coupled-carla quadrotors as glTF meshes, made with numpy (no third-party model): a consumer camera drone
(about 0.9 kg, 0.38 m motor to motor) and an industrial delivery drone (about 6.3 kg, 0.9 m), the airframes of
coupledcarla/dronemodel.py.

    python tools/drone_mesh.py OUT_DIR          # writes SM_CCDrone_<Airframe>_{Body,Prop,Disc,Full}.glb

Each airframe has
  _Body  the airframe without propellers: shell, arms, motor bells, landing gear, battery, gimbal and camera, front
         sensors, navigation lights (red front, green rear), anti-collision strobe on top (industrial: GPS mast,
         carbon tubes, skids and a parcel box). Origin at the feet (the flight model's position), x forward,
         y left, z up, metres.
  _Prop  one two-bladed propeller (twisted, tapered blades and a hub) about its own axis, origin at the hub.
  _Disc  the blur disc of a spinning propeller (a flat disc with radial UVs: the material fades it out).
  _Full  body + four still propellers + four discs in one mesh (the `mesh` look of drones.py).
Material slots are named (Body, Dark, Metal, Carbon, Accent, Lens, NavRed, NavGreen, Strobe, Prop, Disc, Parcel);
tools/ue_drone_assets.py imports the files into /Game/CoupledCarla/Drones and assigns the materials.
The prop positions (PROPS) are written to OUT_DIR/drone_props.json for the blueprint builder.
"""
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

ANGLES = [math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4]     # FL, RL, RR, FR (model)
SPIN = [1, -1, 1, -1]


class Mesh:
    """Triangles grouped by material slot."""

    def __init__(self):
        self.parts = {}

    def add(self, mat, v, n, uv, idx):
        p = self.parts.setdefault(mat, [[], [], [], [], 0])
        p[0].append(np.asarray(v, np.float32))
        p[1].append(np.asarray(n, np.float32))
        p[2].append(np.asarray(uv, np.float32))
        p[3].append(np.asarray(idx, np.uint32) + p[4])
        p[4] += len(v)

    def extend(self, other, offset=(0, 0, 0), rot_z=0.0):
        c, s = math.cos(rot_z), math.sin(rot_z)
        r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], np.float32)
        for mat, (vs, ns, uvs, ids, _) in other.parts.items():
            for v, n, uv, ix in zip(vs, ns, uvs, ids):
                base = ix - ix.min() if len(ix) else ix
                self.add(mat, v @ r.T + np.asarray(offset, np.float32), n @ r.T, uv, base)

    def merged(self):
        out = {}
        for mat, (vs, ns, uvs, ids, _) in self.parts.items():
            out[mat] = (np.concatenate(vs), np.concatenate(ns), np.concatenate(uvs), np.concatenate(ids))
        return out


# -- primitives ----------------------------------------------------------------------------------------------------

def _frame(axis):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, t)
    u /= np.linalg.norm(u)
    return a, u, np.cross(a, u)


def box(m, mat, centre, size, rot_z=0.0):
    c = np.asarray(centre, float)
    hx, hy, hz = np.asarray(size, float) / 2
    cz, sz = math.cos(rot_z), math.sin(rot_z)
    R = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    faces = [((1, 0, 0), (0, 1, 0), (0, 0, 1)), ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
             ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
             ((0, 0, 1), (1, 0, 0), (0, 1, 0)), ((0, 0, -1), (1, 0, 0), (0, -1, 0))]
    h = np.array([hx, hy, hz])
    for nrm, u, w in faces:
        nrm, u, w = np.array(nrm, float), np.array(u, float), np.array(w, float)
        pts = [nrm * h + su * u * h + sw * w * h for su, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        v = np.array(pts) @ R.T + c
        m.add(mat, v, np.tile(nrm @ R.T, (4, 1)), [[0, 0], [1, 0], [1, 1], [0, 1]], [0, 1, 2, 0, 2, 3])


def superellipsoid(m, mat, centre, radii, e1=0.35, e2=0.35, nu=24, nv=16, squash_bottom=1.0):
    """A rounded box (a superellipsoid): smooth shell for drone bodies."""
    c, (a, b, cc) = np.asarray(centre, float), radii
    us = np.linspace(-math.pi, math.pi, nu + 1)
    vs = np.linspace(-math.pi / 2, math.pi / 2, nv + 1)

    def sp(x, e):
        return np.sign(x) * np.abs(x) ** e
    V, N, UV = [], [], []
    for j, v in enumerate(vs):
        for i, u in enumerate(us):
            x = a * sp(math.cos(v), e1) * sp(math.cos(u), e2)
            y = b * sp(math.cos(v), e1) * sp(math.sin(u), e2)
            z = cc * sp(math.sin(v), e1) * (squash_bottom if v < 0 else 1.0)
            V.append((x, y, z))
            # normal of the implicit surface |x/a|^(2/e2) + |y/b|^(2/e2))^(e2/e1) + |z/c|^(2/e1) = 1
            nx = sp(math.cos(v), 2 - e1) * sp(math.cos(u), 2 - e2) / a
            ny = sp(math.cos(v), 2 - e1) * sp(math.sin(u), 2 - e2) / b
            nz = sp(math.sin(v), 2 - e1) / cc
            n = np.array([nx, ny, nz])
            ln = np.linalg.norm(n)
            N.append(n / ln if ln > 1e-9 else (0, 0, 1 if v > 0 else -1))
            UV.append((i / nu, j / nv))
    idx = []
    w = nu + 1
    for j in range(nv):
        for i in range(nu):
            p = j * w + i
            idx += [p, p + 1, p + w + 1, p, p + w + 1, p + w]
    m.add(mat, np.array(V) + c, N, UV, idx)


def tube(m, mat, a, b, r0, r1=None, seg=16, caps=True):
    """A (tapered) cylinder from a to b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    r1 = r0 if r1 is None else r1
    ax, u, w = _frame(b - a)
    th = np.linspace(0, 2 * math.pi, seg + 1)
    V, N, UV, idx = [], [], [], []
    for k, t in enumerate(th):
        d = math.cos(t) * u + math.sin(t) * w
        V += [a + d * r0, b + d * r1]
        N += [d, d]
        UV += [(k / seg, 0), (k / seg, 1)]
    for k in range(seg):
        p = 2 * k
        idx += [p, p + 2, p + 3, p, p + 3, p + 1]
    m.add(mat, V, N, UV, idx)
    if caps:
        for centre, r, nrm in ((a, r0, -ax), (b, r1, ax)):
            V = [centre] + [centre + (math.cos(t) * u + math.sin(t) * w) * r for t in th]
            uv = [(0.5, 0.5)] + [(0.5 + 0.5 * math.cos(t), 0.5 + 0.5 * math.sin(t)) for t in th]
            idx = []
            for k in range(seg):
                idx += ([0, k + 1, k + 2] if nrm @ ax > 0 else [0, k + 2, k + 1])
            m.add(mat, V, np.tile(nrm, (len(V), 1)), uv, idx)


def sphere(m, mat, centre, r, nu=16, nv=10, z_scale=1.0, half=False):
    c = np.asarray(centre, float)
    V, N, UV, idx = [], [], [], []
    v0 = 0.0 if half else -math.pi / 2
    for j, v in enumerate(np.linspace(v0, math.pi / 2, nv + 1)):
        for i, u in enumerate(np.linspace(0, 2 * math.pi, nu + 1)):
            d = np.array([math.cos(v) * math.cos(u), math.cos(v) * math.sin(u), math.sin(v)])
            V.append(c + d * r * np.array([1, 1, z_scale]))
            N.append(d)
            UV.append((i / nu, j / nv))
    w = nu + 1
    for j in range(nv):
        for i in range(nu):
            p = j * w + i
            idx += [p, p + 1, p + w + 1, p, p + w + 1, p + w]
    m.add(mat, V, N, UV, idx)


def disc(m, mat, centre, r, seg=48, both=True):
    c = np.asarray(centre, float)
    th = np.linspace(0, 2 * math.pi, seg + 1)
    V = [c] + [c + np.array([math.cos(t) * r, math.sin(t) * r, 0]) for t in th]
    uv = [(0.5, 0.5)] + [(0.5 + 0.5 * math.cos(t), 0.5 + 0.5 * math.sin(t)) for t in th]
    up = [k for s in range(seg) for k in (0, s + 1, s + 2)]
    m.add(mat, V, np.tile((0, 0, 1), (len(V), 1)), uv, up)
    if both:
        m.add(mat, V, np.tile((0, 0, -1), (len(V), 1)), uv, [k for s in range(seg) for k in (0, s + 2, s + 1)])


def blade_pair(m, mat, radius, hub_r, chord, thickness, spin=1, n=14):
    """Two tapered, twisted blades (a simple airfoil plate), origin at the hub, spinning about +z."""
    for side in (1, -1):
        top, bot = [], []
        for k in range(n + 1):
            f = k / n
            r = hub_r + (radius - hub_r) * f
            c = chord * (0.55 + 1.6 * f * (1 - f) * 1.2) * (1.0 - 0.55 * f ** 3)
            twist = math.radians(22 - 14 * f) * spin
            for (sgn, lst) in ((1, top), (-1, bot)):
                for e in (-0.5, 0.5):          # leading / trailing edge
                    # camber: the upper surface bulges
                    y = e * c
                    z = (thickness * (0.5 if sgn > 0 else -0.5)) * (1 - 4 * e * e) + 0.12 * thickness * sgn
                    yy, zz = y * math.cos(twist) - z * math.sin(twist), y * math.sin(twist) + z * math.cos(twist)
                    lst.append((side * r, side * yy, zz + 0.25 * thickness * f))
        for lst, nz in ((top, 1.0), (bot, -1.0)):
            V = np.array(lst)
            N = np.tile((0, 0, nz), (len(V), 1))
            UV = [((i // 2) / n, i % 2) for i in range(len(V))]
            idx = []
            for k in range(n):
                p = 2 * k
                tri = [p, p + 1, p + 3, p, p + 3, p + 2]
                if (nz > 0) != (side > 0):
                    tri = [p, p + 3, p + 1, p, p + 2, p + 3]
                idx += tri
            m.add(mat, V, N, UV, idx)
    tube(m, mat, (0, 0, -thickness), (0, 0, thickness * 1.5), hub_r, hub_r * 0.8, seg=12)


# -- the airframes -------------------------------------------------------------------------------------------------

def consumer():
    """A folding camera drone in the style of the DJI Mavic class: light grey shell, dark arms, front legs under the
    front motors, a 3-axis gimbal under the nose, stereo sensors at the front, battery on the back."""
    arm, R = 0.19, 0.12
    body = Mesh()
    zc = 0.062
    superellipsoid(body, "Body", (0.0, 0, zc), (0.105, 0.048, 0.03), 0.45, 0.5, squash_bottom=0.8)
    superellipsoid(body, "Body", (0.02, 0, zc + 0.022), (0.075, 0.04, 0.014), 0.4, 0.45)          # top deck
    superellipsoid(body, "Dark", (-0.045, 0, zc + 0.028), (0.06, 0.036, 0.012), 0.3, 0.3)          # battery
    box(body, "Dark", (-0.045, 0, zc + 0.041), (0.03, 0.05, 0.004))                               # battery latch
    for y in (-0.018, 0.018):                                                                      # front stereo
        sphere(body, "Lens", (0.1, y, zc + 0.005), 0.007, z_scale=0.8)
    sphere(body, "Lens", (-0.1, 0, zc + 0.002), 0.006)                                             # rear sensor
    # gimbal and camera under the nose
    tube(body, "Dark", (0.08, 0, zc - 0.02), (0.08, 0, zc - 0.03), 0.012)
    superellipsoid(body, "Dark", (0.095, 0, zc - 0.042), (0.018, 0.022, 0.016), 0.5, 0.5)
    tube(body, "Lens", (0.11, 0, zc - 0.042), (0.118, 0, zc - 0.042), 0.009, 0.0095, seg=20)
    disc_m = Mesh()
    props = []
    for k, a in enumerate(ANGLES):
        tip = np.array([arm * math.cos(a), arm * math.sin(a)])
        root = np.array([0.045 * math.cos(a), 0.03 * math.sin(a)])
        front = k in (0, 3)
        z_arm = zc + (0.012 if front else -0.006)
        tube(body, "Dark", (*root, z_arm), (*tip, z_arm), 0.011, 0.008, seg=12)
        box(body, "Dark", (*((root + tip) / 2), z_arm - 0.004), (0.006, 0.006, 0.006))
        # motor bell: dark metal can on a dark base, a silver shaft
        tube(body, "Dark", (*tip, z_arm - 0.012), (*tip, z_arm + 0.006), 0.014, seg=20)
        tube(body, "Metal", (*tip, z_arm + 0.006), (*tip, z_arm + 0.02), 0.0135, 0.013, seg=20)
        tube(body, "Metal", (*tip, z_arm + 0.02), (*tip, z_arm + 0.026), 0.0025, seg=8)
        # navigation light under each motor: red at the front, green at the back
        sphere(body, "NavRed" if front else "NavGreen", (*tip, z_arm - 0.015), 0.005, half=False)
        # legs: under the front motors, folding legs under the rear arms
        if front:
            tube(body, "Dark", (*tip, z_arm - 0.012), (tip[0] * 0.95, tip[1] * 0.95, 0.004), 0.004, 0.003)
            sphere(body, "Dark", (tip[0] * 0.95, tip[1] * 0.95, 0.004), 0.005)
        else:
            foot = np.array([tip[0] * 0.75, tip[1] * 0.75])
            tube(body, "Dark", (*foot, z_arm - 0.004), (*foot, 0.004), 0.004, 0.003)
            sphere(body, "Dark", (*foot, 0.004), 0.005)
        props.append({"x": float(tip[0]), "y": float(tip[1]), "z": float(z_arm + 0.027), "spin": SPIN[k]})
    sphere(body, "Strobe", (-0.01, 0, zc + 0.037), 0.006, half=True)                              # strobe on top
    prop = Mesh()
    blade_pair(prop, "Prop", R, 0.008, 0.024, 0.0025)
    disc(disc_m, "Disc", (0, 0, 0.001), R * 1.02)
    return body, prop, disc_m, props


def industrial():
    """An industrial delivery quadrotor in the style of the DJI M300 / Matrice class: dark grey body with an orange
    accent, carbon arms on folding hinges, big motors, a GPS mast, two batteries on top, landing gear with skids,
    a downward camera and a parcel box under the body."""
    arm, R = 0.45, 0.27
    body = Mesh()
    zc = 0.33
    superellipsoid(body, "Body", (0, 0, zc), (0.2, 0.15, 0.07), 0.35, 0.4, squash_bottom=0.7)
    superellipsoid(body, "Accent", (0.17, 0, zc + 0.01), (0.04, 0.11, 0.035), 0.4, 0.4)           # nose stripe
    for y in (-0.06, 0.06):                                                                        # two batteries
        superellipsoid(body, "Dark", (-0.02, y, zc + 0.085), (0.11, 0.045, 0.03), 0.25, 0.25)
    tube(body, "Carbon", (0, 0, zc + 0.06), (0, 0, zc + 0.2), 0.008, seg=10)                      # GPS mast
    tube(body, "Body", (0, 0, zc + 0.2), (0, 0, zc + 0.215), 0.045, 0.04, seg=24)                 # GPS puck
    sphere(body, "Strobe", (-0.12, 0, zc + 0.075), 0.012, half=True)                              # strobe
    for y in (-0.05, 0.05):
        sphere(body, "Lens", (0.2, y, zc + 0.0), 0.012, z_scale=0.8)                               # FPV / sensors
    tube(body, "Dark", (0.12, 0, zc - 0.06), (0.12, 0, zc - 0.09), 0.02)                          # gimbal
    superellipsoid(body, "Dark", (0.14, 0, zc - 0.12), (0.045, 0.05, 0.04), 0.5, 0.5)
    tube(body, "Lens", (0.18, 0, zc - 0.12), (0.192, 0, zc - 0.12), 0.022, 0.024, seg=24)
    # parcel box (delivery) under the body
    box(body, "Parcel", (-0.04, 0, zc - 0.17), (0.26, 0.2, 0.16))
    box(body, "Dark", (-0.04, 0, zc - 0.085), (0.1, 0.1, 0.012))                                   # release hook
    disc_m = Mesh()
    props = []
    for k, a in enumerate(ANGLES):
        tip = np.array([arm * math.cos(a), arm * math.sin(a)])
        root = np.array([0.17 * math.cos(a), 0.13 * math.sin(a)])
        z_arm = zc + 0.02
        tube(body, "Carbon", (*root, z_arm), (*tip, z_arm), 0.016, 0.015, seg=14)
        box(body, "Dark", (*(root * 1.15), z_arm), (0.05, 0.05, 0.045), rot_z=a)                  # folding hinge
        tube(body, "Dark", (*tip, z_arm - 0.03), (*tip, z_arm + 0.01), 0.032, seg=24)            # motor mount
        tube(body, "Metal", (*tip, z_arm + 0.01), (*tip, z_arm + 0.045), 0.034, 0.033, seg=24)    # bell
        tube(body, "Metal", (*tip, z_arm + 0.045), (*tip, z_arm + 0.055), 0.005, seg=8)
        front = k in (0, 3)
        sphere(body, "NavRed" if front else "NavGreen", (*tip, z_arm - 0.04), 0.011)
        props.append({"x": float(tip[0]), "y": float(tip[1]), "z": float(z_arm + 0.057), "spin": SPIN[k]})
    # landing gear: two legs each side down to a skid
    for side in (-1, 1):
        y_top, y_skid = 0.12 * side, 0.24 * side
        for x in (-0.12, 0.12):
            tube(body, "Carbon", (x, y_top, zc - 0.04), (x * 1.2, y_skid, 0.02), 0.011, 0.01, seg=10)
        tube(body, "Carbon", (-0.24, y_skid, 0.015), (0.24, y_skid, 0.015), 0.012, seg=12)
        sphere(body, "Dark", (-0.24, y_skid, 0.015), 0.013)
        sphere(body, "Dark", (0.24, y_skid, 0.015), 0.013)
    prop = Mesh()
    blade_pair(prop, "Prop", R, 0.02, 0.055, 0.005)
    disc(disc_m, "Disc", (0, 0, 0.002), R * 1.02)
    return body, prop, disc_m, props


AIRFRAMES = {"Consumer": consumer, "Industrial": industrial}


def full(body, prop, disc_m, props):
    m = Mesh()
    m.extend(body)
    for p in props:
        m.extend(prop, (p["x"], p["y"], p["z"]), rot_z=0.4 if p["spin"] > 0 else -0.4)
        m.extend(disc_m, (p["x"], p["y"], p["z"]))
    return m


# -- glTF ------------------------------------------------------------------------------------------------------

def to_gltf_frame(v):
    """Model (x forward, y left, z up) -> glTF axes such that Unreal's glTF import (Interchange: Unreal x = glTF x,
    y = glTF z, z = glTF y, metres to cm) gives Unreal x forward, y right, z up: glTF (x, z, -y). The axis check
    mesh (axis_check()) verifies it in the editor (tools/ue_drone_assets.py)."""
    v = np.asarray(v, np.float32)
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], axis=1)


def axis_check():
    """A small box forward-left-up of the origin, at model (0.5, 0.2, 0.1): Unreal should see it at (50, -20, 10)."""
    m = Mesh()
    box(m, "Body", (0.5, 0.2, 0.1), (0.02, 0.02, 0.02))
    return m


def write_glb(mesh, path, frame=to_gltf_frame):
    parts = mesh.merged()
    blobs, views, accessors, prims, mats = [], [], [], [], []
    off = 0

    def push(arr, target, comp, typ, minmax=False):
        nonlocal off
        b = arr.tobytes()
        pad = (-len(b)) % 4
        blobs.append(b + b"\0" * pad)
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(b), **({"target": target} if target else {})})
        acc = {"bufferView": len(views) - 1, "componentType": comp, "count": len(arr), "type": typ}
        if minmax:
            acc["min"] = arr.min(axis=0).tolist()
            acc["max"] = arr.max(axis=0).tolist()
        accessors.append(acc)
        off += len(b) + pad
        return len(accessors) - 1
    for k, (mat, (v, n, uv, idx)) in enumerate(sorted(parts.items())):
        vv, nn = frame(v), frame(n)
        nn = nn / np.maximum(1e-9, np.linalg.norm(nn, axis=1, keepdims=True))
        # (x, z, -y) is a rotation (determinant 1): the winding stays counter-clockwise
        a_pos = push(vv.astype(np.float32), 34962, 5126, "VEC3", True)
        a_nrm = push(nn.astype(np.float32), 34962, 5126, "VEC3")
        a_uv = push(uv.astype(np.float32), 34962, 5126, "VEC2")
        a_idx = push(idx.astype(np.uint32), 34963, 5125, "SCALAR")
        mats.append({"name": mat, "pbrMetallicRoughness": {"baseColorFactor": [0.5, 0.5, 0.5, 1.0]},
                     "doubleSided": mat in ("Disc", "Prop")})
        prims.append({"attributes": {"POSITION": a_pos, "NORMAL": a_nrm, "TEXCOORD_0": a_uv}, "indices": a_idx,
                      "material": k})
    name = Path(path).stem
    gltf = {"asset": {"version": "2.0", "generator": "coupled-carla tools/drone_mesh.py"},
            "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0, "name": name}],
            "meshes": [{"name": name, "primitives": prims}], "materials": mats,
            "buffers": [{"byteLength": off}], "bufferViews": views, "accessors": accessors}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    binb = b"".join(blobs)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(binb)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(binb), 0x004E4942) + binb)
    return sum(len(p[3]) // 3 for p in parts.values())


def build(out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    info = {}
    for name, make in AIRFRAMES.items():
        body, prop, disc_m, props = make()
        tris = {}
        for suffix, m in (("Body", body), ("Prop", prop), ("Disc", disc_m), ("Full", full(body, prop, disc_m, props))):
            tris[suffix] = write_glb(m, out / f"SM_CCDrone_{name}_{suffix}.glb")
        info[name] = {"props": props, "triangles": tris}
        print(f"{name}: " + ", ".join(f"{k} {v} triangles" for k, v in tris.items()))
    write_glb(axis_check(), out / "SM_CCDrone_AxisCheck.glb")
    (out / "drone_props.json").write_text(json.dumps(info, indent=1))
    return info


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "build/drones")
