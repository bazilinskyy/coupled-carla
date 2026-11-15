"""How close is a map's render to the real street? Real street-level photos (Panoramax, KartaView, Wikimedia
Commons: open licences, no keys) at known positions and headings, the same views rendered in CARLA, and a
comparison of the two (coupledcarla/realism.py: semantic layout, skyline, image statistics).

    # 1. views and real images (internet, no CARLA): <out>/views.csv, <out>/real/<id>.jpg
    python tools/realism_check.py prepare experiments/eindhoven_twin.yaml --out logs/realism
    # 2. renders (a running CARLA server with the map; GPU): <out>/renders/<id>.png, <id>_sem.png, renders.csv
    python tools/realism_check.py render experiments/eindhoven_twin.yaml --out logs/realism --host 127.0.0.1
    # 3. comparison (CPU is fine; torch + transformers): <out>/compare.csv, summary.json, sheets/*.jpg
    python tools/realism_check.py compare --out logs/realism
    # validation without CARLA: real photo against real photo (same place / neighbour / random place)
    python tools/realism_check.py pairs experiments/eindhoven_twin.yaml --out logs/realism

render works from views.csv alone (it builds it from the cached photo metadata when missing); compare downloads
real images it lacks. Contact sheets and real images stay out of git (logs/).
"""
import argparse
import csv
import json
import math
import random
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import realism as R  # noqa: E402


def _map(experiment):
    from coupledcarla import config
    cfg = config.load(experiment)
    mdir = config.map_dir(cfg)
    meta = json.loads((mdir / "meta.json").read_text())
    return cfg, mdir, meta["bbox"]


def build_views(args, mdir, bbox):
    recs = R.fetch(mdir, bbox, sources=args.sources.split(","), refresh=args.refresh)
    offsets = tuple(float(x) for x in args.pano_offsets.split(","))
    return R.select(recs, bbox, spacing_m=args.spacing, pano_offsets=offsets, max_views=args.max_views,
                    include_action=args.action_cams, size=args.size)


# ---------------------------------------------------------------- real images

def _download(url, tries=3):
    import requests
    from coupledcarla.realworld import UA
    for k in range(tries):
        try:
            r = requests.get(url, headers=UA, timeout=120)
            r.raise_for_status()
            return r.content
        except Exception:  # noqa: BLE001 - retried, then the caller logs it
            if k == tries - 1:
                raise
            time.sleep(2 + 3 * k)


def real_image(v, out, sd=False):
    """<out>/real/<id>.jpg: a panorama cut to the view, or the photo scaled to the view's width."""
    import io
    from PIL import Image, ImageOps
    path = Path(out) / "real" / f"{v['id']}.jpg"
    if path.exists():
        im = Image.open(path)
        if im.width != v["width"]:      # prepared at another --size: scaled, not fetched again
            im.convert("RGB").resize((v["width"], int(round(im.height * v["width"] / im.width))),
                                     Image.LANCZOS).save(path, quality=92)
        return path
    urls = [u for u in ((v.get("url_sd"), v["url"]) if sd else (v["url"], v.get("url_sd"))) if u]
    data = None
    for k, url in enumerate(urls):      # KartaView's storage URLs are sometimes gone where its CDN still has them
        try:
            data = _download(url, tries=2)
            break
        except Exception:  # noqa: BLE001
            if k == len(urls) - 1:
                raise
    im = ImageOps.exif_transpose(Image.open(io.BytesIO(data))).convert("RGB")
    if v["projection"] == "equirect":
        arr = R.reproject(np.asarray(im), "equirect", v["width"], v["height"], v["hfov"],
                          src_yaw=float(v["src_heading"]) - float(v["heading"]))
        im = Image.fromarray(arr)
    else:
        im = im.resize((v["width"], int(round(im.height * v["width"] / im.width))), Image.LANCZOS)
    path.parent.mkdir(parents=True, exist_ok=True)
    im.save(path, quality=92)
    return path


def fetch_images(views, out, threads=4, sd=False):
    ok, failed = 0, 0

    def one(v):
        try:
            real_image(v, out, sd)
            return True
        except Exception as err:  # noqa: BLE001 - one missing photo is not fatal
            print(f"{v['id']}: image not fetched ({err})", flush=True)
            return False
    with ThreadPoolExecutor(threads) as pool:
        for good in pool.map(one, views):
            ok += good
            failed += not good
    print(f"real images: {ok} ready, {failed} failed", flush=True)


def cmd_prepare(args):
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    views = build_views(args, mdir, bbox)
    R.write_views(out / "views.csv", views)
    by = {}
    for v in views:
        by[(v["source"], v["kind"])] = by.get((v["source"], v["kind"]), 0) + 1
    print("views by source and camera: " + ", ".join(f"{s}/{k} {n}" for (s, k), n in sorted(by.items())), flush=True)
    if not args.no_images:
        fetch_images(views, out, args.threads, args.sd)
    write_attribution(views, out / "ATTRIBUTION.csv")


def write_attribution(views, path):
    with Path(path).open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["id", "source", "author", "licence", "page"])
        for v in views:
            wr.writerow([v["id"], v["source"], v["author"], v["licence"], v["page"]])


# ---------------------------------------------------------------- rendering (CARLA)

def cmd_render(args):
    import queue

    import carla
    from coupledcarla import compat
    from coupledcarla.geo import Geo, heading_to_yaw
    from coupledcarla.scenery import Scenery
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    vpath = out / "views.csv"
    if vpath.exists():
        views = R.read_views(vpath)
    else:
        views = build_views(args, mdir, bbox)
        R.write_views(vpath, views)
    if args.max_views and len(views) > args.max_views:
        views = views[:args.max_views]
    rdir = out / "renders"
    rdir.mkdir(parents=True, exist_ok=True)
    done = {p.stem for p in rdir.glob("*.png") if not p.stem.endswith("_sem")}
    todo = [v for v in views if v["id"] not in done or args.again]
    print(f"{len(todo)} views to render ({len(views) - len(todo)} already there)", flush=True)
    geo = Geo(mdir / "geo.net.xml")
    t0 = time.time()
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    level = cfg["map"].get("carla_level")
    if not level or not world.get_map().name.endswith(level):
        from coupledcarla.run import load_world
        print(f"loading {level or 'the OpenDRIVE map'}", flush=True)
        world = load_world(client, mdir / "map.xodr", cfg["map"], slab=not level)
    old = world.get_settings()
    st = world.get_settings()
    st.synchronous_mode, st.fixed_delta_seconds = True, args.delta
    world.apply_settings(st)
    actors, spawned = [], []
    try:
        world.tick()
        if not args.no_scenery and (mdir / "scenery.json").exists():
            scen = Scenery(client, world, mdir, {"radius_m": 0})
            scen.update([])
            spawned.append(scen)
            print(f"scenery: {scen.n} meshes", flush=True)
        parked_done = False
        cmap = world.get_map()
        lib = world.get_blueprint_library()
        rows = []
        wide = {}
        for v in todo:
            wide[v["id"]] = R.wide_size(v["width"], v["height"], v["hfov"], args.yaw_margin, args.pitch_margin)
        sizes = sorted({(w, h, round(f, 2)) for w, h, f in wide.values()})
        cams = {}

        def camera(size):
            if size not in cams:
                w, h, f = size
                bp = lib.find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(w))
                bp.set_attribute("image_size_y", str(h))
                bp.set_attribute("fov", str(f))
                if bp.has_attribute("motion_blur_intensity"):
                    bp.set_attribute("motion_blur_intensity", "0")
                compat.setup_camera(bp, 0.0, args.profile)
                sb = lib.find("sensor.camera.semantic_segmentation")
                sb.set_attribute("image_size_x", str(w))
                sb.set_attribute("image_size_y", str(h))
                sb.set_attribute("fov", str(f))
                tf = carla.Transform(carla.Location(z=500))
                rgb, sem = world.spawn_actor(bp, tf), world.spawn_actor(sb, tf)
                q = queue.Queue()
                rgb.listen(q.put)
                actors.extend([rgb, sem])
                cams[size] = (rgb, sem, q)
            return cams[size]
        print(f"{len(sizes)} camera sizes; setup {time.time() - t0:.0f} s; about "
              f"{len(todo) * args.settle * 0.035 / 60:.0f} min to render", flush=True)
        weather_key = None
        spectator = world.get_spectator()
        t_render = time.time()
        for k, v in enumerate(todo):
            if args.budget_min and time.time() - t_render > args.budget_min * 60:
                print(f"time budget of {args.budget_min} min reached: {k} of {len(todo)} rendered; run again to "
                      f"render the rest", flush=True)
                break
            hour = v["local_time"][:13]
            if hour and hour != weather_key and args.weather != "fixed":
                from coupledcarla.run import set_weather
                sc = dict(cfg["simulation"], weather=args.weather, time=v["local_time"])
                set_weather(world, sc, (v["lat"], v["lon"]), mdir / "real", log=lambda m: None)
                weather_key = hour
            if not parked_done and not args.no_parked:
                parked_done = True
                try:
                    from coupledcarla import parking
                    tc = dict(cfg, map=dict(cfg["map"], bbox=bbox),
                              simulation=dict(cfg["simulation"], time=v["local_time"] or None))
                    data = parking.trial_data(mdir, tc, log=lambda m: None)
                    if data and data["items"]:
                        pk = Scenery(client, world, mdir, {"radius_m": 0, "kinds": ["parked"]}, data=data)
                        pk.update([])
                        spawned.append(pk)
                        print(f"parked cars: {pk.n}", flush=True)
                except Exception as err:  # noqa: BLE001 - views without parked cars
                    print(f"parked cars left out ({err})", flush=True)
            x, y = geo.to_carla(v["lat"], v["lon"])
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Any)
            z0 = wp.transform.location.z if wp is not None else 0.0
            gz = z0
            try:
                gp = world.ground_projection(carla.Location(x=x, y=y, z=z0 + 1.5), 6.0)
                if gp is not None:
                    gz = gp.location.z
            except Exception:  # noqa: BLE001 - older API: the lane height
                pass
            loc = carla.Location(x=x, y=y, z=gz + v["height_m"])
            tf = carla.Transform(loc, carla.Rotation(yaw=heading_to_yaw(v["heading"]), pitch=v["pitch"], roll=0.0))
            rgb, sem, q = camera((*wide[v["id"]][:2], round(wide[v["id"]][2], 2)))
            rgb.set_transform(tf)
            sem.set_transform(tf)
            spectator.set_transform(tf)
            n = max(3, args.first_settle if k == 0 else args.settle)
            for _ in range(n - 2):
                world.tick()
            sq = queue.Queue()
            sem.listen(sq.put)        # the labels need no exposure: streamed for the last two ticks only
            world.tick()
            frame = world.tick()
            img = simg = None
            deadline = time.time() + 10
            while time.time() < deadline and (img is None or simg is None):
                try:
                    while img is None or img.frame < frame:
                        img = q.get(timeout=2.0)
                    while simg is None or simg.frame < frame:
                        simg = sq.get(timeout=2.0)
                except queue.Empty:
                    break
            sem.stop()
            while not q.empty():
                q.get_nowait()
            if img is None or simg is None:
                print(f"{v['id']}: no image", flush=True)
                continue
            from PIL import Image
            a = np.frombuffer(img.raw_data, np.uint8).reshape(img.height, img.width, 4)[..., [2, 1, 0]]
            Image.fromarray(a).save(rdir / f"{v['id']}.png")
            s = np.frombuffer(simg.raw_data, np.uint8).reshape(simg.height, simg.width, 4)[..., 2]
            Image.fromarray(s.copy()).save(rdir / f"{v['id']}_sem.png")
            w_, h_, f_ = wide[v["id"]]
            rows.append({"id": v["id"], "x": round(x, 2), "y": round(y, 2), "ground_z": round(gz, 2),
                         "lane_z": round(z0, 2), "cam_z": round(loc.z, 2), "yaw": round(tf.rotation.yaw, 2),
                         "wide_w": w_, "wide_h": h_, "wide_hfov": round(f_, 2), "frame": frame,
                         "sun_alt": round(world.get_weather().sun_altitude_angle, 1)})
            if (k + 1) % 25 == 0:
                el = time.time() - t_render
                print(f"{k + 1}/{len(todo)} rendered, {el / (k + 1):.2f} s per view", flush=True)
        _merge_csv(out / "renders.csv", rows, "id")
        print(f"rendered {len(rows)} views in {time.time() - t0:.0f} s", flush=True)
    finally:
        for a in actors:
            try:
                if a.is_listening:
                    a.stop()
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        for sc in spawned:        # leave the server as it was: scenery and parked cars removed
            ids = [i for key in sc.loaded for i in sc.loaded[key]]
            client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
        world.apply_settings(old)


def _merge_csv(path, rows, key):
    old = []
    if path.exists():
        with path.open(newline="") as f:
            old = list(csv.DictReader(f))
    have = {r[key] for r in rows}
    allrows = [r for r in old if r[key] not in have] + rows
    if not allrows:
        return
    fields = list(allrows[-1].keys())
    with path.open("w", newline="") as f:
        wr = csv.DictWriter(f, fields, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(allrows)


# ---------------------------------------------------------------- comparison

class Labels:
    """Segmentation with a cache of label maps (<out>/seg/<name>.png)."""

    def __init__(self, out, model, device):
        self.dir = Path(out) / "seg"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.model, self.device, self.seg = model, device, None

    def __call__(self, name, rgb):
        from PIL import Image
        p = self.dir / f"{name}.png"
        if p.exists():
            return np.asarray(Image.open(p))
        if self.seg is None:
            self.seg = R.Segmenter(self.model, self.device)
        lab = self.seg(rgb)
        Image.fromarray(lab).save(p)
        return lab


def _load(path):
    from PIL import Image
    return np.asarray(Image.open(path).convert("RGB"))


def view_metrics(real_rgb, wide_rgb, real_lab, wide_lab, v, wide_hfov, gt_wide=None, args=None):
    """All metrics of one view: nominal pose and best alignment."""
    w, h, hfov = v["width"], v["height"], v["hfov"]
    rg = R.groups_of(real_lab)
    wg = R.groups_of(wide_lab)
    if rg.shape != (h, w):
        from PIL import Image
        rg = np.asarray(Image.fromarray(rg).resize((w, h), Image.NEAREST))
        real_rgb = np.asarray(Image.fromarray(real_rgb).resize((w, h), Image.BILINEAR))
    nominal_g = R.reproject(wg, "pinhole", w, h, hfov, src_hfov=wide_hfov, nearest=True, fill=R.IGNORE)
    m = {f"nom_{k}": val for k, val in R.compare_labels(rg, nominal_g, hfov).items()}
    yr = args.yaw_margin if args else 12.0
    pr = args.pitch_margin if args else 6.0
    dy, dp, al_g, am = R.align(rg, wg, hfov, w, h, wide_hfov, yaw_range=min(12.0, yr), pitch_range=min(6.0, pr))
    m.update({f"al_{k}": val for k, val in am.items()})
    m.update(align_dyaw=dy, align_dpitch=dp)
    al_rgb = R.reproject(wide_rgb, "pinhole", w, h, hfov, yaw=dy, pitch=dp, src_hfov=wide_hfov)
    mask = (rg != R.IGNORE) & (al_g != R.IGNORE)
    sr, sd = R.image_stats(real_rgb, mask), R.image_stats(al_rgb, mask)
    for k in sr:
        m[f"real_{k}"], m[f"render_{k}"] = sr[k], sd[k]
    m["colour_hist_sim"] = R.colour_hist_sim(real_rgb, al_rgb, mask)
    if gt_wide is not None:     # the model on the render against CARLA's own labels: how far to trust it there
        gt = R.reproject(R.groups_of(gt_wide, R.CARLA_TO_GROUP), "pinhole", w, h, hfov, src_hfov=wide_hfov,
                         nearest=True, fill=R.IGNORE)
        g = R.compare_labels(gt, nominal_g, hfov)
        m["model_vs_carla_pix_acc"], m["model_vs_carla_miou"] = g["pix_acc"], g["miou"]
    return m, rg, al_g, al_rgb, real_rgb


def contact_sheets(rows, out, name, per_sheet=12, thumb=384):
    """Real | render | real layout | render layout, one view per row, with its scores."""
    from PIL import Image, ImageDraw
    sdir = Path(out) / "sheets"
    sdir.mkdir(parents=True, exist_ok=True)
    paths = []
    for s in range(0, len(rows), per_sheet):
        chunk = rows[s:s + per_sheet]
        hs = [int(round(thumb * r["imgs"][0].shape[0] / r["imgs"][0].shape[1])) for r in chunk]
        sheet = Image.new("RGB", (thumb * 4 + 8, sum(hs) + 22 * len(chunk)), (255, 255, 255))
        dr = ImageDraw.Draw(sheet)
        y = 0
        for r, hh in zip(chunk, hs):
            dr.text((4, y + 4), r["caption"], fill=(0, 0, 0))
            y += 22
            for i, im in enumerate(r["imgs"]):
                sheet.paste(Image.fromarray(im).resize((thumb, hh)), (i * (thumb + 2), y))
            y += hh
        p = sdir / f"{name}_{s // per_sheet + 1:02d}.jpg"
        sheet.save(p, quality=85)
        paths.append(p)
    return paths


SUMMARY_KEYS = ["nom_miou", "al_miou", "nom_pix_acc", "al_pix_acc", "nom_layout_tv", "al_layout_tv",
                "nom_skyline_mae_deg", "al_skyline_mae_deg", "colour_hist_sim", "model_vs_carla_miou"]


def summarise(rows, keys=SUMMARY_KEYS, by="source"):
    def med(xs):
        xs = [float(x) for x in xs if x not in ("", None) and not math.isnan(float(x))]
        return round(float(np.median(xs)), 3) if xs else None
    groups = {"all": rows}
    for r in rows:
        groups.setdefault(r.get(by, "?"), []).append(r)
    out = {}
    for g, rs in groups.items():
        d = {"n": len(rs)}
        for k in keys:
            if k in rs[0]:
                d[k] = med([r.get(k) for r in rs])
        for name in R.GROUPS:     # median class areas, real vs render
            if f"al_real_{name}" in rs[0]:
                d[f"area_{name}"] = [med([r[f"al_real_{name}"] for r in rs]), med([r[f"al_render_{name}"] for r in rs])]
        for k in ("lum_mean", "lum_std", "sat_mean", "colourfulness", "edge_density"):
            if f"real_{k}" in rs[0]:
                d[k] = [med([r[f"real_{k}"] for r in rs]), med([r[f"render_{k}"] for r in rs])]
        out[g] = d
    return out


def _write_rows(path, rows):
    rows = [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k not in ("imgs", "caption")}
            for r in rows]
    if not rows:
        return
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open("w", newline="") as f:
        wr = csv.DictWriter(f, fields)
        wr.writeheader()
        wr.writerows(rows)


def cmd_compare(args):
    out = Path(args.out)
    views = {v["id"]: v for v in R.read_views(out / "views.csv")}
    renders = {}
    with (out / "renders.csv").open(newline="") as f:
        for r in csv.DictReader(f):
            renders[r["id"]] = r
    labels = Labels(out, args.model, args.device)
    rows, sheet_rows = [], []
    t0 = time.time()
    for k, (vid, rr) in enumerate(renders.items()):
        v = views.get(vid)
        rp = out / "renders" / f"{vid}.png"
        if v is None or not rp.exists():
            continue
        try:
            real_path = real_image(v, out, args.sd)
        except Exception as err:  # noqa: BLE001
            print(f"{vid}: no real image ({err})", flush=True)
            continue
        real_rgb, wide_rgb = _load(real_path), _load(rp)
        sp = out / "renders" / f"{vid}_sem.png"
        from PIL import Image
        gt = np.asarray(Image.open(sp)) if sp.exists() else None
        m, rg, ag, arg, real_rgb = view_metrics(real_rgb, wide_rgb, labels(f"real_{vid}", real_rgb),
                                                labels(f"render_{vid}", wide_rgb), v, float(rr["wide_hfov"]), gt, args)
        row = {"id": vid, "source": v["source"], "kind": v["kind"], "lat": v["lat"], "lon": v["lon"],
               "heading": v["heading"], "local_time": v["local_time"], **m}
        rows.append(row)
        sheet_rows.append({"imgs": [real_rgb, arg, R.colourise(rg), R.colourise(ag)],
                           "caption": f"{vid} {v['source']} {v['local_time']}  mIoU {m['al_miou']:.2f} "
                                      f"(nominal {m['nom_miou']:.2f}, yaw {m['align_dyaw']:+.0f} pitch "
                                      f"{m['align_dpitch']:+.0f})  skyline {m['al_skyline_mae_deg']:.1f} deg"})
        if (k + 1) % 20 == 0:
            print(f"{k + 1}/{len(renders)} compared ({(time.time() - t0) / (k + 1):.1f} s each)", flush=True)
    _write_rows(out / "compare.csv", rows)
    summ = {"views": summarise(rows), "model": args.model,
            "groups": R.GROUPS, "ignored": "person, rider, motorcycle, bicycle (both images)"}
    pairs = out / "pairs_summary.json"
    if pairs.exists():
        summ["real_vs_real"] = json.loads(pairs.read_text())
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    order = sorted(range(len(rows)), key=lambda i: -float(rows[i]["al_miou"]) if not math.isnan(rows[i]["al_miou"])
                   else 1.0)     # best match first
    paths = contact_sheets([sheet_rows[i] for i in order], out, "twin")
    print(json.dumps(summ["views"]["all"], indent=1))
    print(f"{len(rows)} views compared; {out / 'compare.csv'}, {out / 'summary.json'}, {len(paths)} sheets in "
          f"{out / 'sheets'}", flush=True)


# ---------------------------------------------------------------- validation: real against real

def make_pairs(views, records, max_pairs=60, seed=1):
    """(a, b, kind) pairs of views: 'same' = a panorama of another sequence within 8 m (cut at a's heading),
    'neighbour' = the same sequence 10-25 m on, 'random' = a view over 300 m away."""
    rng = random.Random(seed)
    panos = [r for r in records if r["projection"] == "equirect"]
    out = []
    for v in [v for v in views if v["projection"] == "equirect"]:
        cand_same = [r for r in panos if r["sequence"] != v["sequence"] and
                     R.metres(r["lat"], r["lon"], v["lat"], v["lon"]) < 8.0 and r["src_id"] != v["src_id"]]
        cand_next = [r for r in panos if r["sequence"] == v["sequence"] and
                     10.0 <= R.metres(r["lat"], r["lon"], v["lat"], v["lon"]) <= 25.0 and
                     abs((r["heading"] - v["heading"] + 180) % 360 - 180) < 30]
        for kind, cands in (("same", cand_same), ("neighbour", cand_next)):
            if cands:
                out.append((v, min(cands, key=lambda r: R.metres(r["lat"], r["lon"], v["lat"], v["lon"])), kind))
    by_kind = {}
    for p in out:
        by_kind.setdefault(p[2], []).append(p)
    pairs = []
    for kind, ps in by_kind.items():
        rng.shuffle(ps)
        pairs += ps[:max_pairs]
    for v, _, _ in [p for p in pairs if p[2] == "same"] or pairs[:max_pairs]:
        far = [r for r in panos if R.metres(r["lat"], r["lon"], v["lat"], v["lon"]) > 300]
        if far:
            pairs.append((v, rng.choice(far), "random"))
    return pairs


def cmd_pairs(args):
    """Real-vs-real baseline: what the metrics give for two real photos of the same place, of places 10-25 m
    apart, and of random places; a twin render should score between 'random' and 'same'."""
    import io

    from PIL import Image
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    recs = R.fetch(mdir, bbox, sources=("panoramax", "kartaview"), refresh=False)
    views = R.read_views(out / "views.csv")
    pairs = make_pairs(views, R.daytime(recs), args.max_pairs)
    print(f"{len(pairs)} pairs: " + ", ".join(f"{k} {sum(p[2] == k for p in pairs)}"
                                             for k in ("same", "neighbour", "random")), flush=True)
    labels = Labels(out, args.model, args.device)
    cache = Path(out) / "pairs_src"
    cache.mkdir(parents=True, exist_ok=True)
    rows, sheet_rows = [], []
    for k, (v, r, kind) in enumerate(pairs):
        try:
            real_rgb = _load(real_image(v, out, args.sd))
            p = cache / f"{r['src_id']}.jpg"
            if not p.exists():
                im = Image.open(io.BytesIO(_download(r["url_sd"] if args.sd else (r["url"] or r["url_sd"]))))
                im.convert("RGB").save(p, quality=92)
            pano = _load(p)
        except Exception as err:  # noqa: BLE001
            print(f"pair {v['id']}: skipped ({err})", flush=True)
            continue
        w2, h2, f2 = R.wide_size(v["width"], v["height"], v["hfov"], args.yaw_margin, args.pitch_margin)
        # b cut as a wide 'render' at a's heading (b's own heading for random places)
        head = v["heading"] if kind != "random" else r["heading"]
        wide = R.reproject(pano, "equirect", w2, h2, f2, src_yaw=r["heading"] - head)
        m, rg, ag, arg, real_rgb = view_metrics(real_rgb, wide, labels(f"real_{v['id']}", real_rgb),
                                                labels(f"pair_{r['src_id']}_{int(head)}", wide), v, f2, None, args)
        rows.append({"id": v["id"], "other": r["src_id"], "kind": kind,
                     "dist_m": round(R.metres(r["lat"], r["lon"], v["lat"], v["lon"]), 1), **m})
        if k < 36:
            sheet_rows.append({"imgs": [real_rgb, arg, R.colourise(rg), R.colourise(ag)],
                               "caption": f"{kind} {v['id']} vs {r['src_id'][:8]} ({rows[-1]['dist_m']} m)  "
                                          f"mIoU {m['al_miou']:.2f} (nominal {m['nom_miou']:.2f})"})
        if (k + 1) % 20 == 0:
            print(f"{k + 1}/{len(pairs)} pairs", flush=True)
    _write_rows(out / "pairs.csv", rows)
    summ = summarise(rows, by="kind")
    (out / "pairs_summary.json").write_text(json.dumps(summ, indent=1))
    contact_sheets(sheet_rows, out, "pairs")
    for kind in ("same", "neighbour", "random"):
        if kind in summ:
            print(kind, json.dumps({k: summ[kind].get(k) for k in ["n"] + SUMMARY_KEYS if k in summ[kind]}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--out", default="logs/realism")
    common.add_argument("--yaw-margin", type=float, default=12.0, help="render this much wider each side (deg)")
    common.add_argument("--pitch-margin", type=float, default=6.0)
    common.add_argument("--sd", action="store_true", help="2048 px panoramas instead of full resolution")
    views = argparse.ArgumentParser(add_help=False)
    views.add_argument("experiment")
    views.add_argument("--sources", default="panoramax,kartaview,commons")
    views.add_argument("--refresh", action="store_true", help="fetch the photo metadata again")
    views.add_argument("--spacing", type=float, default=20.0, help="one view per this many metres and direction")
    views.add_argument("--pano-offsets", default="0", help="views cut from each panorama, deg from its heading")
    views.add_argument("--max-views", type=int, default=350)
    views.add_argument("--action-cams", action="store_true", help="also handlebar action cams (roll, fisheye)")
    views.add_argument("--size", type=int, default=768, help="view width (px)")
    seg = argparse.ArgumentParser(add_help=False)
    seg.add_argument("--model", default="nvidia/segformer-b2-finetuned-cityscapes-1024-1024")
    seg.add_argument("--device", default=None)
    p = sub.add_parser("prepare", parents=[common, views])
    p.add_argument("--no-images", action="store_true")
    p.add_argument("--threads", type=int, default=4)
    p = sub.add_parser("render", parents=[common, views])
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--delta", type=float, default=0.05, help="fixed time step while rendering (s)")
    p.add_argument("--settle", type=int, default=30, help="ticks after each move (auto exposure adapts)")
    p.add_argument("--first-settle", type=int, default=150)
    p.add_argument("--profile", default="Dashcam", help="CARLA 1.0 post-process profile")
    p.add_argument("--weather", default="real", help="'real' (Open-Meteo at the photo's hour), a preset or 'fixed'")
    p.add_argument("--budget-min", type=float, default=10.0, help="stop rendering after this many minutes")
    p.add_argument("--no-scenery", action="store_true")
    p.add_argument("--no-parked", action="store_true")
    p.add_argument("--again", action="store_true", help="render views that already have a render")
    p = sub.add_parser("compare", parents=[common, seg])
    p = sub.add_parser("pairs", parents=[common, seg])
    p.add_argument("experiment")
    p.add_argument("--max-pairs", type=int, default=60)
    args = ap.parse_args()
    {"prepare": cmd_prepare, "render": cmd_render, "compare": cmd_compare, "pairs": cmd_pairs}[args.cmd](args)


if __name__ == "__main__":
    main()
