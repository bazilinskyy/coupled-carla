"""The coupled-carla ships as glTF meshes, made with numpy from the hull's particulars (no third-party model):
for now the KVLCC2 tanker (320 m) of coupledcarla/shipmodel.py.

    python -m tools.ship_mesh OUT_DIR          # writes SM_CCShip_<Hull>.glb and ship_meshes.json

The hull is lofted from sections: a parallel middle body (KVLCC2 is full, C_b 0.81), a blunt bow with a bulb and a
stern with a transom above the water; the sections are boxy amidships (a tight bilge) and rounder towards the ends.
On it: a deck with a forecastle, a pipe rack and catwalk, the accommodation aft (five decks, the bridge with wings),
the funnel, masts, the rudder and the propeller, and navigation lights (red port, green starboard, white masthead and
stern). It shows a generic VLCC of KVLCC2's dimensions, not a particular ship. Origin amidships on the waterline,
x forward, y left, z up, metres (the drones' convention: tools/drone_mesh.py writes the glTF axes).
Material slots: Antifouling, Boottop, Hull, Deck, House, Glass, Funnel, FunnelBand, Mast, Rudder, Propeller, NavRed,
NavGreen, NavWhite (tools/ue_ship_assets.py assigns the materials).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from coupledcarla import shipmodel as sm                                  # noqa: E402
from tools.drone_mesh import Mesh, box, superellipsoid, tube, write_glb  # noqa: E402

COLOURS = {      # for previews (Unreal gets materials): sRGB 0-1
    "Antifouling": (0.55, 0.13, 0.10), "Boottop": (0.08, 0.08, 0.09), "Hull": (0.13, 0.17, 0.24),
    "Deck": (0.33, 0.45, 0.32), "House": (0.92, 0.92, 0.90), "Glass": (0.10, 0.14, 0.18),
    "Funnel": (0.90, 0.90, 0.88), "FunnelBand": (0.10, 0.25, 0.55), "Mast": (0.75, 0.62, 0.22),
    "Rudder": (0.55, 0.13, 0.10), "Propeller": (0.72, 0.53, 0.25), "NavRed": (1.0, 0.1, 0.1),
    "NavGreen": (0.1, 1.0, 0.2), "NavWhite": (1.0, 1.0, 0.95)}
BOOTTOP_M = 2.0          # black band above the waterline (the light-ballast waterline lies below the design one)


# -- hull form ---------------------------------------------------------------------------------------------------

class HullForm:
    """Half-breadth y(x, z) of a full-form tanker from its particulars. x from midship (forward +), z from the design
    waterline (up +)."""

    def __init__(self, hull):
        self.h = hull
        L, B = hull.lpp, hull.beam
        self.L, self.b, self.d, self.fb = L, B / 2, hull.draught, hull.freeboard
        mid = min(0.6, max(0.0, 2.4 * hull.cb - 1.4))          # parallel middle body share: about 0.54 for C_b 0.81
        self.x_fwd = (0.5 * mid + 0.04) * L                     # where the bow starts to narrow
        self.x_aft = -(0.5 * mid - 0.02) * L                    # where the run starts
        self.x_fp = L / 2 + 0.006 * L                           # the stem at the deck (the hull ends at FP + a bit)
        self.x_ap = -L / 2 - 0.012 * L                          # the transom at the deck
        self.loa = self.x_fp - self.x_ap

    def plan(self, x, z):
        """Share (0-1) of the half beam at x, z: 1 in the middle body, falling to the stem and the stern."""
        if self.x_aft <= x <= self.x_fwd:
            return 1.0
        if x > self.x_fwd:              # bow: blunt (a superellipse of exponent 2.3 in plan), finer low down
            t = min(1.0, (x - self.x_fwd) / (self.x_fp - self.x_fwd))
            p = 2.3 - 0.5 * max(0.0, -z / self.d)
            return max(0.0, 1.0 - t ** p) ** (1.0 / p)
        t = min(1.0, (self.x_aft - x) / (self.x_aft - self.x_ap))     # stern: a finer run below, a transom above
        transom = 0.62 * min(1.0, max(0.0, (z + 0.15 * self.d) / (0.15 * self.d + self.fb)))
        p = 2.0 if z > 0 else 1.6
        return transom + (1.0 - transom) * max(0.0, 1.0 - t ** p) ** (1.0 / p)

    def section(self, x, z):
        """Half breadth (m): below the water a superellipse of exponent n (8 amidships: a tight bilge; 2.6 at the
        ends: U-shaped), above it straight sides with a little flare at the bow."""
        b = self.b * self.plan(x, z)
        end = max(0.0, (abs(x) - 0.3 * self.L) / (0.2 * self.L))
        if z < 0:
            n = 8.0 - 5.4 * min(1.0, end)
            return b * max(0.0, 1.0 - (-z / self.d) ** n) ** (1.0 / n)
        flare = 1.0 + (0.06 * min(1.0, end) if x > 0 else 0.0) * z / self.fb
        return min(self.b, b * flare)

    def deck_z(self, x):
        """The deck: the forecastle 3 m higher over the bow."""
        return self.fb + (3.0 if x > self.L / 2 - 0.09 * self.L else 0.0)

    def waterline(self, n=200):
        xs = np.linspace(self.x_ap, self.x_fp, n)
        return xs, np.array([self.section(x, 0.0) for x in xs])


def _strip(m, mat, a, b, flip=False):
    """Quads between two polylines a and b (k x 3 each), normals from the quads."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    for i in range(len(a) - 1):
        p = np.array([a[i], a[i + 1], b[i + 1], b[i]])
        nrm = np.cross(p[1] - p[0], p[3] - p[0])
        if np.linalg.norm(nrm) < 1e-9:
            nrm = np.cross(p[2] - p[1], p[3] - p[1])
        nrm = nrm / max(1e-9, np.linalg.norm(nrm))
        idx = [0, 2, 1, 0, 3, 2] if flip else [0, 1, 2, 0, 2, 3]
        m.add(mat, p, np.tile(-nrm if flip else nrm, (4, 1)), [[0, 0], [1, 0], [1, 1], [0, 1]], idx)


def hull_shell(m, f, nx=140):
    """The shell, both sides, with the antifouling / boot-top / topside split, the deck, the transom."""
    # stations: denser at the ends
    u = np.linspace(-1.0, 1.0, nx)
    xs = 0.5 * (f.x_fp + f.x_ap) + 0.5 * f.loa * np.sign(u) * (1.0 - (1.0 - np.abs(u)) ** 1.6)
    zs = np.unique(np.concatenate([np.linspace(-f.d, 0.0, 18), [BOOTTOP_M], np.linspace(BOOTTOP_M, 1.0, 2)]))
    for k in range(len(zs) - 1):
        z0, z1 = zs[k], zs[k + 1]
        mat = "Antifouling" if z1 <= 0 else "Boottop"
        for side in (1, -1):
            a = [(x, side * f.section(x, z0), z0) for x in xs]
            b = [(x, side * f.section(x, z1), z1) for x in xs]
            _strip(m, mat, a, b, flip=side < 0)
    # topsides up to the deck (the forecastle steps up)
    top = [(x, f.deck_z(x)) for x in xs]
    for side in (1, -1):
        a = [(x, side * f.section(x, BOOTTOP_M), BOOTTOP_M) for x in xs]
        b = [(x, side * f.section(x, min(z, f.fb)), z) for x, z in top]
        _strip(m, "Hull", a, b, flip=side < 0)
    # deck
    a = [(x, -f.section(x, f.fb), z) for x, z in top]
    b = [(x, f.section(x, f.fb), z) for x, z in top]
    _strip(m, "Deck", a, b)
    # transom: the stern's last section, a fan from its centre line
    x = xs[0]
    ring = [(x, f.section(x, z), z) for z in np.linspace(-f.d, f.deck_z(x), 24)]
    ring = ring + [(x, -y, z) for (_, y, z) in ring[::-1]]
    c = np.mean(np.array(ring), axis=0)
    for i in range(len(ring) - 1):
        p = np.array([c, ring[i + 1], ring[i]])
        m.add("Hull", p, np.tile((-1.0, 0.0, 0.0), (3, 1)), [[0.5, 0.5], [0, 0], [1, 0]], [0, 1, 2])


def superstructure(m, f):
    L, B, fb = f.L, 2 * f.b, f.fb
    x0 = f.x_ap + 0.025 * L                          # aft face of the accommodation
    house_l, house_w, deck_h = 0.075 * L, 0.72 * B, 2.9
    decks = 5
    for k in range(decks):                           # stacked decks, each a little shorter
        lk = house_l * (1 - 0.05 * k)
        box(m, "House", (x0 + lk / 2, 0, fb + deck_h * (k + 0.5)), (lk, house_w, deck_h))
        box(m, "Glass", (x0 + lk + 0.02, 0, fb + deck_h * (k + 0.55)), (0.06, house_w * 0.92, 0.9))
        for side in (1, -1):
            box(m, "Glass", (x0 + lk / 2, side * (house_w / 2 + 0.02), fb + deck_h * (k + 0.55)), (lk * 0.85, 0.06, 0.9))
    zb = fb + deck_h * decks
    bl = 0.6 * house_l                                # the bridge, full width with its wings
    box(m, "House", (x0 + house_l - bl / 2, 0, zb + deck_h / 2), (bl, house_w, deck_h))
    box(m, "House", (x0 + house_l - 3.0, 0, zb + deck_h * 0.75), (4.0, B * 0.98, deck_h * 0.5))      # wings
    box(m, "Glass", (x0 + house_l + 0.03, 0, zb + deck_h * 0.6), (0.06, house_w * 0.96, 1.4))
    box(m, "House", (x0 + house_l - bl / 2, 0, zb + deck_h + 0.3), (bl * 1.05, house_w * 1.02, 0.6))  # roof
    # radar mast on the bridge roof, lights
    top = zb + deck_h + 0.6
    mx = x0 + house_l - bl / 2
    tube(m, "Mast", (mx, 0, top), (mx, 0, top + 9.0), 0.35, 0.25)
    box(m, "Mast", (mx, 0, top + 6.0), (0.4, 7.0, 0.3))
    box(m, "NavWhite", (mx, 0, top + 9.2), (0.6, 0.6, 0.6))
    for side, mat in ((1, "NavRed"), (-1, "NavGreen")):           # y left: port red, starboard green
        box(m, mat, (x0 + house_l - 3.0, side * (B * 0.49 - 0.4), zb + deck_h + 0.2), (0.8, 0.5, 0.6))
    # funnel aft of the house, a band near its top
    fx = x0 + 0.25 * house_l
    fz = zb + deck_h
    tube(m, "Funnel", (fx, 0, fz - 2), (fx - 1.0, 0, fz + 10.0), 3.8, 3.4)
    tube(m, "FunnelBand", (fx - 0.7, 0, fz + 6.0), (fx - 0.85, 0, fz + 8.0), 3.62, 3.58)
    # stern light
    box(m, "NavWhite", (f.x_ap + 0.4, 0, fb + 1.5), (0.4, 0.6, 0.6))
    return {"bridge_x": x0 + house_l, "bridge_z": zb + deck_h, "house_x0": x0, "house_len": house_l}


def deck_gear(m, f):
    L, fb = f.L, f.fb
    x0, x1 = f.x_ap + 0.12 * L, f.L / 2 - 0.1 * L
    box(m, "Mast", ((x0 + x1) / 2, 0, fb + 1.4), (x1 - x0, 1.6, 0.25))             # catwalk
    for y in (-3.0, -1.5, 1.5, 3.0):                                               # pipes along the deck
        tube(m, "Deck", (x0, y, fb + 0.6), (x1, y, fb + 0.6), 0.35, seg=10, caps=False)
    for dx in (-1.5, 1.5):                                                         # manifold pipes and its crane
        tube(m, "Deck", (0.02 * L + dx, -f.b * 0.85, fb + 1.2), (0.02 * L + dx, f.b * 0.85, fb + 1.2), 0.45, seg=10)
    box(m, "Mast", (0.02 * L, 0, fb + 0.5), (5.0, 2 * f.b * 0.85, 1.0))
    tube(m, "Mast", (0.02 * L, f.b * 0.6, fb), (0.02 * L, f.b * 0.6, fb + 12.0), 0.5)
    # foremast with the forward masthead light, anchor windlasses on the forecastle
    fx = f.L / 2 - 0.035 * L
    tube(m, "Mast", (fx, 0, f.deck_z(fx)), (fx, 0, f.deck_z(fx) + 14.0), 0.45, 0.3)
    box(m, "NavWhite", (fx, 0, f.deck_z(fx) + 14.3), (0.6, 0.6, 0.6))
    for y in (-6.0, 6.0):
        box(m, "Mast", (f.L / 2 - 0.02 * L, y, f.deck_z(fx) + 0.9), (3.0, 2.5, 1.8))


def appendages(m, f):
    h = f.h
    xr = f.x_ap + 0.01 * f.L + 2.0                                                   # rudder at the stern
    chord, span = 8.65, h.h_rudder                                                   # Table 2: B_R 8.65 m
    box(m, "Rudder", (xr, 0, -f.d + 0.8 + span / 2), (chord, 1.2, span))
    xp = xr + chord / 2 + 3.0                                                        # propeller ahead of it
    zp = -f.d + h.d_prop / 2 + 0.6
    tube(m, "Propeller", (xp - 1.0, 0, zp), (xp + 1.2, 0, zp), 1.0, 1.1)             # hub
    for k in range(4):
        a = k * math.pi / 2 + math.pi / 4
        tip = (xp, h.d_prop / 2 * math.cos(a), zp + h.d_prop / 2 * math.sin(a))
        tube(m, "Propeller", (xp, 0, zp), tip, 0.9, 0.5, seg=8)
    superellipsoid(m, "Antifouling", (f.x_fp - 1.0, 0, -0.62 * f.d), (7.0, 4.6, 4.6), e1=1.0, e2=1.0)    # bulb


def build_ship(hull):
    f = HullForm(hull)
    m = Mesh()
    hull_shell(m, f)
    info = superstructure(m, f)
    deck_gear(m, f)
    appendages(m, f)
    info.update({"loa": f.loa, "x_ap": f.x_ap, "x_fp": f.x_fp, "freeboard": f.fb, "draught": f.d,
                 "beam": 2 * f.b})
    return m, f, info


def build(out_dir, hulls=("KVLCC2",)):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    info = {}
    for name in hulls:
        m, f, meta = build_ship(sm.preset(name))
        meta["triangles"] = write_glb(m, out / f"SM_CCShip_{name}.glb")
        info[name] = meta
        print(f"{name}: {meta['triangles']} triangles, Loa {f.loa:.1f} m")
    (out / "ship_meshes.json").write_text(json.dumps(info, indent=1))
    return info


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "build/ships")
