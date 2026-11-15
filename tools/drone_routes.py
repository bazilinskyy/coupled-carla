"""AI drone routes on a map's real buildings and trees, offline (no CARLA): the old way (straight, `clearance_m` above
everything under the route) against the routing band (droneplan.Router), with the numbers and a top-down picture.

    python -m tools.drone_routes maps/eindhoven_centre [--random 40] [--png docs/img/drone_routes.png] [--fly]

Routes: the deliveries of experiments/eindhoven_drones.yaml (depot, three drones, two addresses each, picked as
drones.py picks them), `--random` more homes within 700 m of the depot, and the yaml's patrol loop. For each: cruise
height above the ground, horizontal length, estimated energy (dronemodel.steady_power), planning time and clearance to
buildings and trees (exact, from the footprints: the 3D distance from the cruise path, and the horizontal distance to
anything reaching above the cruise height less the vertical clearance; both outside 15 m of the ends, where a drone
climbs or lowers its parcel beside a building). `--fly` flies every delivery in the flight model (dronemodel.Fleet)
too: energy from the battery, flight time and the clearance of the flown track.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from coupledcarla import dronemodel as dm
from coupledcarla import droneplan as plan
from coupledcarla import drones

DEPOT = (51.44270, 5.48240)          # experiments/eindhoven_drones.yaml
PILOT = (51.43898, 5.47870)          # the missions' centre there (the pilot's spawn)
PATROL = ((51.43950, 5.47850), 500.0)


def ground_fn(mdir):
    """Ground z (CARLA x, y) from the nearest building bases (buildings.json z): enough on a flat map."""
    b = json.loads((Path(mdir) / "buildings.json").read_text())
    cs = np.array([np.mean(np.asarray(r["p"], float), axis=0) for r in b])
    zs = np.array([float(r.get("z") or 0.0) for r in b])

    def ground(x, y):
        d = (cs[:, 0] - x) ** 2 + (cs[:, 1] - y) ** 2
        k = np.argsort(d)[:6]
        w = 1.0 / np.maximum(1.0, d[k])
        return float(np.sum(zs[k] * w) / np.sum(w))
    return ground


class Clearance:
    """Exact clearance from building footprints (model frame) and tree discs."""

    def __init__(self, obs):
        from shapely.geometry import Point, Polygon
        from shapely.strtree import STRtree
        self.geoms, self.tops = [], []
        for pts, top, _k in obs.polys:
            g = Polygon(pts)
            if not g.is_valid:
                g = g.buffer(0)
            self.geoms.append(g)
            self.tops.append(top)
        for x, y, top, r in obs.trees:
            self.geoms.append(Point(x, y).buffer(r, 8))
            self.tops.append(top)
        self.tops = np.array(self.tops)
        self.tree = STRtree(self.geoms)

    def at(self, xyz, ends=(), skip=15.0, reach=None, skip_h=None):
        """(min 3D clearance, min horizontal distance to obstacles reaching above z - reach) over points xyz (N, 3),
        leaving out points within `skip` of the ends (`skip_h` for the horizontal distance)."""
        from shapely.geometry import Point
        best3, besth = math.inf, math.inf
        skip_h = skip if skip_h is None else skip_h
        for p in xyz:
            dend = min((math.hypot(p[0] - e[0], p[1] - e[1]) for e in ends), default=math.inf)
            if dend < skip:
                continue
            pt = Point(p[0], p[1])
            for k in self.tree.query(pt.buffer(80.0)):
                dh = self.geoms[k].distance(pt)
                dz = max(0.0, p[2] - self.tops[k])
                best3 = min(best3, math.hypot(dh, dz))
                if reach is not None and self.tops[k] > p[2] - reach and dend >= skip_h:
                    besth = min(besth, dh)
        return best3, besth


def densify(points, step=2.0):
    out = []
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.linalg.norm(b[:2] - a[:2]) / step))
        out.extend(a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False))
    out.append(np.asarray(points[-1], float))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("map", help="map folder (buildings.json, scenery.json, geo.net.xml)")
    ap.add_argument("--random", type=int, default=40, help="more addresses within 700 m of the depot")
    ap.add_argument("--png", help="write the top-down picture here")
    ap.add_argument("--json", help="write the numbers here")
    ap.add_argument("--fly", action="store_true", help="fly the deliveries in the flight model")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    mdir = Path(a.map)
    from coupledcarla.geo import Geo
    geo = Geo(mdir / "geo.net.xml")
    gz = ground_fn(mdir)
    obs = drones.load_obstacles(mdir, {"map": {}}, gz, print)
    clr = Clearance(obs)
    af, speed = dm.INDUSTRIAL, 12.0
    lat, ver = plan.margins(af, speed)
    print(f"margins for the industrial drone at {speed} m/s: lateral {lat:.1f} m, vertical {ver:.1f} m")

    def model(ll):
        x, y = geo.to_carla(*ll)
        return np.array([x, -y, gz(x, y)])
    depot = model(DEPOT)
    centre = model(PILOT)
    rng = plan.make_rng(a.seed)
    routes = []              # (name, pad, drop, k)
    for k in range(3):       # as drones.py: three drones, two addresses each, within 450 m of the centre
        picks = plan.pick_addresses(obs, 2, rng, centre[:2], 450.0)
        pad = depot + np.array([3.0 * math.cos(2.4 * k), 3.0 * math.sin(2.4 * k), 0.0]) * (k > 0)
        pad[2] = gz(pad[0], -pad[1])
        for n, (_bi, xy) in enumerate(picks):
            routes.append((f"parcels_{k}.{n}", pad, np.array([xy[0], xy[1], gz(xy[0], -xy[1])]), k))
        rng.uniform(-math.pi, math.pi)
    rr = plan.make_rng(a.seed + 100)
    for n, (_bi, xy) in enumerate(plan.pick_addresses(obs, a.random, rr, depot[:2], 700.0)):
        routes.append((f"random_{n}", depot, np.array([xy[0], xy[1], gz(xy[0], -xy[1])]), 0))

    old = plan.Router(obs, {"mode": "over"})
    new = plan.Router(obs)
    rows = []
    t_all = time.perf_counter()
    for name, pad, drop, k in routes:
        row = {"name": name}
        for tag, r in (("before", old), ("after", new)):
            legs = (r.leg(pad, drop, af, speed, drop[2] + 7.0, k=k), r.leg(drop, pad, af, speed, pad[2] + 0.3, k=k))
            c3, ch = math.inf, math.inf
            for lg, ends in zip(legs, ((pad, drop), (drop, pad))):
                x3, xh = clr.at(densify(lg.points), ends, reach=ver, skip_h=lat + 8.0)
                c3, ch = min(c3, x3), min(ch, xh)
            row[tag] = {"z_agl": [round(float(lg.z - max(pad[2], drop[2])), 1) for lg in legs],
                        "length_m": round(sum(lg.length for lg in legs), 1),
                        "straight_m": round(sum(lg.straight for lg in legs), 1),
                        "energy_wh": round(sum(lg.energy_wh for lg in legs), 2),
                        "ms": round(sum(lg.ms for lg in legs), 1), "how": [lg.how for lg in legs],
                        "clear3d_m": round(c3, 1), "clear_h_m": round(ch, 1) if ch < math.inf else None,
                        "legs": legs}
        rows.append(row)
    print(f"planned {len(routes)} deliveries (2 legs each) before and after in {time.perf_counter() - t_all:.1f} s")

    # the yaml's patrol: a 24-point circle of 0.45 * radius round its centre
    pc = model(PATROL[0])
    ring = plan.loop_points(pc, PATROL[1] * 0.45, 24, 0.0)
    pstart = np.array([*plan.open_point_near(obs, pc[0], pc[1], plan.make_rng(5), 40.0), pc[2]])
    paf, pspeed = dm.CONSUMER, 8.0
    tops = [obs.corridor_top(p, q, 10.0) for p, q in zip(ring, ring[1:] + ring[:1])]
    z_old = max(max(tops), pstart[2]) + 35.0
    lp = new.loop(ring, pstart, paf, pspeed)
    plat, pver = plan.margins(paf, pspeed)
    old_ring = [(x, y, z_old) for x, y in ring]
    patrol = {"before": {"z_agl": round(z_old - pstart[2], 1), "length_m": round(plan.path_length(ring + ring[:1]), 1),
                         "clear3d_m": round(clr.at(densify(old_ring + old_ring[:1]))[0], 1)},
              "after": {"z_agl": round(lp.z - pstart[2], 1), "length_m": round(lp.length, 1), "ms": round(lp.ms, 1),
                        "how": lp.how}}
    c3, ch = clr.at(densify(lp.points + lp.points[:1]), reach=pver)
    patrol["after"].update(clear3d_m=round(c3, 1), clear_h_m=round(ch, 1) if ch < math.inf else None)
    for tag in ("before", "after"):
        L, z = patrol[tag]["length_m"], patrol[tag]["z_agl"]
        patrol[tag]["energy_wh_per_lap"] = round(plan.leg_energy_wh(paf, L, 0, 0, pspeed), 2)
        patrol[tag]["climb_wh"] = round(plan.leg_energy_wh(paf, 0, z, 0, pspeed), 2)
    print(f"patrol (consumer, 8 m/s; margins {plat:.1f} / {pver:.1f} m): {json.dumps(patrol)}")

    if a.fly:
        fly(rows, routes, obs, clr, af, speed, ver, lat)
    summary(rows)
    if a.json:
        out = [{k: ({kk: vv for kk, vv in v.items() if kk != "legs"} if isinstance(v, dict) else v)
                for k, v in r.items()} for r in rows]
        Path(a.json).write_text(json.dumps({"deliveries": out, "patrol": patrol}, indent=1))
    if a.png:
        picture(a.png, obs, rows, depot, old_ring, lp, pstart, new, lat, ver)


def fly(rows, routes, obs, clr, af, speed, ver, lat, dt=0.02, limit_s=600.0):
    """Every delivery, before and after, as one trip (out, winch drop, back) in one fleet."""
    f = dm.Fleet()
    ms = []
    for row, (name, pad, drop, k) in zip(rows, routes):
        for tag in ("before", "after"):
            i = f.add(af, tuple(pad), 0.0)
            f.ground[i] = pad[2]          # it lands back on its pad (drones.py sets this from the map)
            legs = row[tag]["legs"]
            m = plan.Delivery(f"{name}/{tag}", f, i, tuple(pad), [tuple(drop)], None, None, dwell_s=4.0, cruise=speed,
                              drop="winch", winch_m=7.0, loop=False, legs=[legs])
            ms.append((row, tag, i, m, pad, drop, []))
    t0 = time.perf_counter()
    t = 0.0
    while t < limit_s and not all(m.phase == "done" for _r, _t, _i, m, *_ in ms):
        for row, tag, i, m, pad, drop, track in ms:
            m.step(t, dt)
        f.step(dt)
        t += dt
        if int(round(t / dt)) % 10 == 0:
            for row, tag, i, m, pad, drop, track in ms:
                if m.phase != "done":
                    track.append(f.pos[i].copy())
                    row[tag].setdefault("t_s", 0.0)
                    row[tag]["t_s"] = t
    print(f"flew {len(ms)} trips in the flight model: {t:.0f} s simulated in {time.perf_counter() - t0:.0f} s")
    for row, tag, i, m, pad, drop, track in ms:
        tr = np.array(track)
        c3, ch = clr.at(tr[tr[:, 2] > max(pad[2], drop[2]) + 10.0], (pad, drop), reach=ver, skip_h=lat + 8.0) \
            if len(tr) else (0, 0)
        row[tag].update(flown_wh=round(float(f.energy_wh[i]), 2), flown_s=round(row[tag].get("t_s", 0.0), 1),
                        delivered=m.delivered, flown_clear3d_m=round(c3, 1),
                        flown_clear_h_m=round(ch, 1) if ch < math.inf else None)


def summary(rows):
    def stats(tag, key, fn=np.mean):
        v = [r[tag][key] for r in rows if r[tag].get(key) is not None]
        return fn(v) if v else float("nan")
    print(f"{'route':<14}{'before z':>12}{'after z':>12}{'before m':>10}{'after m':>9}{'before Wh':>10}{'after Wh':>9}"
          f"{'clr before':>11}{'clr after':>10}{'lat after':>10}  how")
    for r in rows:
        b, a = r["before"], r["after"]
        print(f"{r['name']:<14}{str(b['z_agl']):>12}{str(a['z_agl']):>12}{b['length_m']:>10.0f}{a['length_m']:>9.0f}"
              f"{b['energy_wh']:>10.1f}{a['energy_wh']:>9.1f}{b['clear3d_m']:>11.1f}{a['clear3d_m']:>10.1f}"
              f"{str(a['clear_h_m']):>10}  {'/'.join(a['how'])}" +
              (f"  flown {b.get('flown_wh')} -> {a.get('flown_wh')} Wh, {b.get('flown_s')} -> {a.get('flown_s')} s"
               if "flown_wh" in a else ""))
    for tag in ("before", "after"):
        zs = [z for r in rows for z in r[tag]["z_agl"]]
        print(f"{tag}: cruise {np.min(zs):.0f}-{np.max(zs):.0f} m above the ground (median {np.median(zs):.0f}), "
              f"length {stats(tag, 'length_m', np.sum):.0f} m in all (straight {stats(tag, 'straight_m', np.sum):.0f}),"
              f" energy {stats(tag, 'energy_wh', np.sum):.0f} Wh (est.)" +
              (f", flown {stats(tag, 'flown_wh', np.sum):.0f} Wh in {stats(tag, 'flown_s', np.sum):.0f} s"
               if "flown_wh" in rows[0][tag] else "") +
              f", min clearance {stats(tag, 'clear3d_m', np.min):.1f} m (3D), planning {stats(tag, 'ms'):.1f} ms per "
              f"route (max {stats(tag, 'ms', np.max):.0f})")


def picture(path, obs, rows, depot, old_ring, lp, pstart, router, lat, ver):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    top = obs.top.copy()
    h = np.where(np.isfinite(top), top - depot[2], np.nan)
    # the routes that changed most, and the yaml's own deliveries
    changed = sorted(rows, key=lambda r: -(max(r["before"]["z_agl"]) - max(r["after"]["z_agl"])))
    show = [r for r in rows if r["name"].startswith("parcels")] + \
        [r for r in changed if not r["name"].startswith("parcels")][:4]
    pts = np.array([p[:2] for r in show for tag in ("before", "after") for lg in r[tag]["legs"] for p in lg.points] +
                   [p[:2] for p in lp.points] + [depot[:2]])
    x0, y0 = pts.min(axis=0) - 80
    x1, y1 = pts.max(axis=0) + 80
    j0, j1 = int((x0 - obs.x0) / obs.cell), int((x1 - obs.x0) / obs.cell)
    i0, i1 = int((y0 - obs.y0) / obs.cell), int((y1 - obs.y0) / obs.cell)
    i0, j0 = max(0, i0), max(0, j0)
    sub = h[i0:i1, j0:j1]
    fig, ax = plt.subplots(figsize=(7.4, 6.6), dpi=100)
    cmap = ListedColormap(plt.get_cmap("Greys")(np.linspace(0.18, 0.95, 12)))
    cmap.set_bad("white")
    im = ax.imshow(sub, origin="lower", cmap=cmap, vmin=0, vmax=60,
                   extent=(obs.x0 + j0 * obs.cell, obs.x0 + j1 * obs.cell, obs.y0 + i0 * obs.cell,
                           obs.y0 + i1 * obs.cell), interpolation="nearest")
    cb = fig.colorbar(im, ax=ax, shrink=0.7, pad=0.02)
    cb.set_label("roof / tree top above the ground (m)")
    blocked, _core = router.masks(depot[2] + 40.0, lat, ver)     # what the band's floor keeps clear of
    ny, nx = blocked.shape
    ax.imshow(np.ma.masked_where(~blocked, np.ones(blocked.shape)), origin="lower", cmap=ListedColormap(["#e4572e"]),
              alpha=0.22, extent=(router.x0, router.x0 + nx * router.cell, router.y0, router.y0 + ny * router.cell),
              interpolation="nearest")
    col = plt.get_cmap("tab10")
    for n, r in enumerate(show):
        c = col(n % 10)
        for tag, ls, lw in (("before", (0, (3, 2)), 1.0), ("after", "-", 1.8)):
            for lg in r[tag]["legs"][:1]:
                p = np.array(lg.points)
                ax.plot(p[:, 0], p[:, 1], ls=ls, lw=lw, color=c)
        b, a = r["before"], r["after"]
        end = np.array(r["after"]["legs"][0].points[-1])
        ax.annotate(f"{b['z_agl'][0]:.0f}→{a['z_agl'][0]:.0f} m", end[:2], fontsize=7, color=c,
                    xytext=(5, -9 + 10 * (n % 2)), textcoords="offset points")
    p = np.array(old_ring + old_ring[:1])
    ax.plot(p[:, 0], p[:, 1], ls=(0, (3, 2)), lw=1.0, color="0.35")
    p = np.array(lp.points + lp.points[:1])
    ax.plot(p[:, 0], p[:, 1], lw=1.8, color="0.15")
    ax.annotate(f"patrol {old_ring[0][2] - pstart[2]:.0f}→{lp.z - pstart[2]:.0f} m", p[0][:2], fontsize=7,
                color="0.15", xytext=(4, -10), textcoords="offset points")
    ax.plot(*depot[:2], marker="s", ms=7, color="black")
    ax.annotate("depot", depot[:2], fontsize=8, xytext=(5, 5), textcoords="offset points")
    ax.plot([], [], ls=(0, (3, 2)), color="0.3", label="before: straight, 20 m above the highest roof")
    ax.plot([], [], ls="-", lw=1.8, color="0.3", label="after: 40-60 m band, round what reaches into it")
    ax.fill([], [], color="#e4572e", alpha=0.3, label=f"kept clear at 40 m ({lat:.0f} m round tops above "
            f"{40 - ver:.0f} m)")
    ax.legend(loc="lower left", fontsize=7, framealpha=0.85)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Delivery and patrol routes over Eindhoven (3DBAG heights): cruise height before → after",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    try:     # a palette PNG is a third of the size
        from PIL import Image
        Image.open(path).convert("RGB").quantize(colors=64, method=Image.Quantize.MEDIANCUT).save(path, optimize=True)
    except Exception:  # noqa: BLE001
        pass
    print(f"wrote {path} ({Path(path).stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
