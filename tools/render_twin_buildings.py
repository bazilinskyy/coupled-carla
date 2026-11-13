"""OSM extrusions vs 3DBAG LoD 2.2 for the same digital twin, side by side (matplotlib, flat shaded).

    python tools/render_twin_buildings.py <twin build dir> <twin build dir>/ue/meshes/osm_buildings out.png [x,y]

Run with the carla-digitaltwins environment (trimesh, matplotlib) after threedbag_buildings.py; x,y (metres in
the twin's frame) centres the 150 m view, else the median of the 3DBAG vertices.
"""
import glob, sys
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

build, osm_dir, out = sys.argv[1], sys.argv[2], sys.argv[3]
FOCUS = [float(x) for x in sys.argv[4].split(",")] if len(sys.argv) > 4 else None
LIGHT = np.array([0.4, -0.5, 0.75]); LIGHT /= np.linalg.norm(LIGHT)

def load(files):
    ms = [trimesh.load(f, force="mesh") for f in files]
    m = trimesh.util.concatenate(ms)
    v = m.vertices[:, [0, 2, 1]].copy(); v[:, 1] *= -1     # glTF y-up -> x east, y north, z up
    return v, m.faces

def draw(ax, ground, bld, title, colour):
    tris, cols = [], []
    for (v, f), base, shade in ((bld, colour, True),):
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
        lum = 0.4 + 0.6 * np.clip(np.abs(n @ LIGHT), 0, 1) if shade else np.full(len(tri), 1.0)
        tris.append(tri); cols.append(np.clip(np.array(base)[None, :] * lum[:, None], 0, 1))
    # one collection, so matplotlib sorts every face by depth together
    ax.add_collection3d(Poly3DCollection(np.vstack(tris), facecolors=np.vstack(cols), edgecolors="none",
                                         linewidths=0))
    lo, hi = bld[0].min(0), bld[0].max(0)
    c = np.array(FOCUS + [0.0]); r = 75.0
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(lo[2] - 2, lo[2] - 2 + 2 * r * 0.5)
    ax.set_box_aspect((1, 1, 0.5))
    ax.view_init(elev=28, azim=-55)
    ax.set_axis_off()
    ax.set_title(title, fontsize=15, pad=0)

ground = load([f for f in glob.glob(f"{build}/ue/meshes/*.glb") if any(k in f for k in ("drivable", "sidewalk", "crossing"))])
bag = load(sorted(glob.glob(f"{build}/ue/meshes/*building*.glb")))
if FOCUS is None:     # the densest block: the median of the 3DBAG vertices
    FOCUS = list(np.median(bag[0][:, :2], axis=0))
osm = load(sorted(glob.glob(f"{osm_dir}/*building*.glb")))
fig = plt.figure(figsize=(16, 7.2), dpi=110)
for i, (b, t, col) in enumerate(((osm, "OpenStreetMap footprints, extruded (flat roofs)", (0.85, 0.72, 0.55)),
                                  (bag, "3DBAG LoD 2.2 (roof shapes from AHN lidar)", (0.86, 0.55, 0.45)))):
    ax = fig.add_subplot(1, 2, i + 1, projection="3d")
    draw(ax, ground, b, t, col)
plt.subplots_adjust(left=0, right=1, top=0.95, bottom=0, wspace=0)
fig.savefig(out, facecolor="white")
print("wrote", out, len(osm[1]), "vs", len(bag[1]), "triangles")
