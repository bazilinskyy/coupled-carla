"""How close is a map's render to the real street? Real street-level photos (Panoramax, KartaView, Wikimedia
Commons: open licences, no keys; Mapillary with the token mapillary_token) at known positions and headings, the
same views rendered in CARLA, and a comparison of the two (coupledcarla/realism.py: semantic layout, skyline,
image statistics; coupledcarla/streetphotos.py: edge alignment, DINOv2 similarity, the differences in words, a
score per spot and per map). The street objects the photos show (Mapillary map features, poles in Panoramax
panoramas) are matched with the map's lamps, signs and furniture, and can correct them (objects --apply).

    # 0. what each source has in the map's box: <photos>/coverage.json
    python tools/realism_check.py sources experiments/eindhoven_centre.yaml

    # 1. views and real images (internet, no CARLA): <out>/views.csv, <out>/real/<id>.jpg
    python tools/realism_check.py prepare experiments/eindhoven_twin.yaml --out logs/realism
    # 2. renders (a running CARLA server with the map; GPU): <out>/renders/<id>.png, <id>_sem.png, renders.csv
    python tools/realism_check.py render experiments/eindhoven_twin.yaml --out logs/realism --host 127.0.0.1
    # 3. comparison (CPU is fine; torch + transformers): <out>/compare.csv, summary.json, sheets/*.jpg
    python tools/realism_check.py compare --out logs/realism
    # validation without CARLA: real photo against real photo (same place / neighbour / random place)
    python tools/realism_check.py pairs experiments/eindhoven_twin.yaml --out logs/realism
    # several compared runs (e.g. before and after a change) side by side over their common views
    python tools/realism_check.py table logs/realism_before logs/realism_after
    # street objects: Mapillary features / panorama poles against scenery.json, and what a correction would change
    python tools/realism_check.py objects experiments/chernihiv.yaml --evidence mapillary [--apply]

render works from views.csv alone (it builds it from the cached photo metadata when missing); compare downloads
real images it lacks. Photos, panoramas, their labels and the Mapillary tiles are cached under the map's
real/streetphotos/ (run.json in <out> says where); renders, sheets and spots/ (one image per spot) stay in <out>
(logs/, not in git). Photographers are credited in ATTRIBUTION.csv and on every sheet. docs/street_photos.md.
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
    if getattr(args, "box", None):      # only the views in a part of the map (e.g. the TU/e campus)
        s, w, n, e = (float(x) for x in args.box.split(","))
        bbox = [max(bbox[0], s), max(bbox[1], w), min(bbox[2], n), min(bbox[3], e)]
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


PANO_W = 4096     # panoramas are kept at this width (768 px per 90 deg view needs 3072)


def photos_dir(out):
    """Where a run's real photos, panoramas and their labels are cached: the map's real/streetphotos/ (run.json
    says so), or the run folder itself for runs made before that."""
    p = Path(out) / "run.json"
    if p.exists():
        d = json.loads(p.read_text()).get("photos")
        if d:
            return Path(d)
    return Path(out)


def set_photos_dir(out, mdir, photos=None):
    """A new run caches its photos under the map (shared by runs); a run that already has panos/ or real/ keeps them."""
    out = Path(out)
    p = out / "run.json"
    if not p.exists() and ((out / "panos").exists() or (out / "real").exists()):
        return out
    run = json.loads(p.read_text()) if p.exists() else {}
    run["photos"] = str(Path(photos or Path(mdir) / "real" / "streetphotos").resolve())
    run["map"] = str(Path(mdir).resolve())
    out.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(run, indent=1))
    return Path(run["photos"])


def _fresh_url(v, out):
    """Mapillary's thumbnail URLs are signed and expire: a new one, found by searching around the image's spot
    (None when it cannot be had; the stored URL is tried then)."""
    if v.get("source") != "mapillary":
        return None
    try:
        from coupledcarla.streetphotos import Mapillary
        return Mapillary(photos_dir(out)).url_for(v["src_id"], float(v["lat"]), float(v["lon"]))
    except Exception as err:  # noqa: BLE001 - the stored URL is tried
        print(f"{v['id']}: no fresh Mapillary URL ({err})", flush=True)
        return None


def _fetch_photo(v, sd=False):
    import io
    from PIL import Image, ImageOps
    urls = [u for u in ((v.get("url_sd"), v["url"]) if sd else (v["url"], v.get("url_sd"))) if u]
    for k, url in enumerate(urls):      # KartaView's storage URLs are sometimes gone where its CDN still has them
        try:
            return ImageOps.exif_transpose(Image.open(io.BytesIO(_download(url, tries=2)))).convert("RGB")
        except Exception:  # noqa: BLE001
            if k == len(urls) - 1:
                raise


def pano_image(v, out, sd=False):
    """<out>/panos/<src_id>.jpg: the panorama of an equirect view (at most PANO_W wide), fetched once."""
    from PIL import Image
    path = photos_dir(out) / "panos" / f"{v['src_id']}.jpg"
    if not path.exists():
        url = _fresh_url(v, out)
        im = _fetch_photo(dict(v, url=url) if url else v, sd)
        if im.width > PANO_W:
            im = im.resize((PANO_W, PANO_W // 2), Image.LANCZOS)
        path.parent.mkdir(parents=True, exist_ok=True)
        im.save(path, quality=92)
    return path


def real_image(v, out, sd=False):
    """<out>/real/<id>.jpg: a panorama cut to the view, or the photo scaled to the view's width."""
    from PIL import Image
    path = photos_dir(out) / "real" / f"{v['id']}.jpg"
    if path.exists() and (v["projection"] != "equirect" or (photos_dir(out) / "panos" / f"{v['src_id']}.jpg").exists()):
        im = Image.open(path)
        if im.width != v["width"]:      # prepared at another --size: scaled, not fetched again
            im.convert("RGB").resize((v["width"], int(round(im.height * v["width"] / im.width))),
                                     Image.LANCZOS).save(path, quality=92)
        return path
    if v["projection"] == "equirect":
        pano = np.asarray(Image.open(pano_image(v, out, sd)).convert("RGB"))
        im = Image.fromarray(R.reproject(pano, "equirect", v["width"], v["height"], v["hfov"],
                                         src_yaw=float(v["src_heading"]) - float(v["heading"])))
    else:
        url = _fresh_url(v, out)
        im = _fetch_photo(dict(v, url=url) if url else v, sd)
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
    set_photos_dir(out, mdir, getattr(args, "photos", None))
    views = build_views(args, mdir, bbox)
    R.write_views(out / "views.csv", views)
    by = {}
    for v in views:
        by[(v["source"], v["kind"])] = by.get((v["source"], v["kind"]), 0) + 1
    print("views by source and camera: " + ", ".join(f"{s}/{k} {n}" for (s, k), n in sorted(by.items())), flush=True)
    if not args.no_images:
        fetch_images(views, out, args.threads, args.sd)
        for v in [v for v in views if v["projection"] == "equirect"]:     # the panoramas themselves, for alignment
            try:
                pano_image(v, out, args.sd)
            except Exception as err:  # noqa: BLE001
                print(f"{v['id']}: panorama not fetched ({err})", flush=True)
    write_attribution(views, out / "ATTRIBUTION.csv")
    if args.labels:      # label the real side now, so compare only has the renders left to segment
        labels, t0 = Labels(out, args.model, args.device), time.time()
        for k, v in enumerate(views):
            try:
                real_spec(v, out, labels, args.sd)
            except Exception as err:  # noqa: BLE001
                print(f"{v['id']}: not labelled ({err})", flush=True)
            if (k + 1) % 50 == 0:
                print(f"{k + 1}/{len(views)} real views labelled ({time.time() - t0:.0f} s)", flush=True)


def write_attribution(views, path):
    with Path(path).open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["id", "source", "author", "licence", "page"])
        for v in views:
            wr.writerow([v["id"], v["source"], v["author"], v["licence"], v["page"]])


# ---------------------------------------------------------------- rendering (CARLA)

class MovingCars:
    """A few cars on the driving lanes in each view, as a street photo has them: vehicle actors with physics off at
    points of the lanes 10-80 m in front of the camera and inside its field of view, cars_per_km per km of lane
    (in both directions) and at least 8 m apart, drawn anew for every view (seeded by its id) and removed after it.
    eindhoven_centre (0.9.16, 350 views, with its parked cars): more than 0.5 % of the view is car in 22 % of the
    renders without moving cars, 32 % at 6 per km, 45 % at 20 per km (the default), and in 58 % of the photos."""

    SPACING = 4.0

    def __init__(self, client, world, cmap, cars_per_km, seed=1):
        import carla
        self.client, self.world, self.carla = client, world, carla
        self.p = float(cars_per_km) * self.SPACING / 1000.0
        self.seed = seed
        pts = [w.transform for w in cmap.generate_waypoints(self.SPACING) if w.lane_type == carla.LaneType.Driving]
        self.xy = np.array([[t.location.x, t.location.y] for t in pts]) if pts else np.zeros((0, 2))
        self.tf = pts
        lib = world.get_blueprint_library()
        self.bps = [b for b in lib.filter("vehicle.*")
                    if (not b.has_attribute("number_of_wheels") or b.get_attribute("number_of_wheels").as_int() == 4)
                    and (not b.has_attribute("base_type") or b.get_attribute("base_type").as_str().lower()
                         in ("car", ""))]
        self.loaded = {"cars": []}
        self.n = 0
        print(f"moving cars: {cars_per_km} per km of lane, {len(self.tf)} lane points, {len(self.bps)} car models",
              flush=True)

    def place(self, vid, loc, yaw, hfov):
        carla = self.carla
        self.clear()
        if not len(self.xy) or not self.bps:
            return
        rng = random.Random(f"{self.seed}:{vid}")
        d = self.xy - np.array([loc.x, loc.y])
        dist = np.hypot(d[:, 0], d[:, 1])
        ang = (np.degrees(np.arctan2(d[:, 1], d[:, 0])) - yaw + 180.0) % 360.0 - 180.0
        cand = np.where((dist > 10.0) & (dist < 80.0) & (np.abs(ang) < hfov / 2.0 + 5.0))[0]
        chosen, cmds = [], []
        for i in cand[np.argsort(dist[cand])]:
            if rng.random() >= self.p or any(np.hypot(*(self.xy[i] - self.xy[j])) < 8.0 for j in chosen):
                continue
            chosen.append(i)
            t = self.tf[i]
            bp = rng.choice(self.bps)
            if bp.has_attribute("color"):
                vals = bp.get_attribute("color").recommended_values
                if vals:
                    bp.set_attribute("color", rng.choice(vals))
            tf = carla.Transform(carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 0.05),
                                 carla.Rotation(yaw=t.rotation.yaw))
            cmds.append(carla.command.SpawnActor(bp, tf).then(carla.command.SetSimulatePhysics(
                carla.command.FutureActor, False)))
        if cmds:
            res = self.client.apply_batch_sync(cmds, False)
            self.loaded["cars"] = [r.actor_id for r in res if not r.error]
        self.n = len(self.loaded["cars"])

    def clear(self):
        if self.loaded["cars"]:
            self.client.apply_batch_sync([self.carla.command.DestroyActor(i) for i in self.loaded["cars"]], False)
        self.loaded["cars"], self.n = [], 0


def cmd_render(args):
    import queue

    import carla
    from coupledcarla import compat, look
    from coupledcarla.geo import Geo, heading_to_yaw
    from coupledcarla.scenery import Scenery
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    set_photos_dir(out, mdir, getattr(args, "photos", None))
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
    photo = None
    try:
        world.tick()
        if (cfg["map"] or {}).get("photoreal"):   # Google's 3D tiles over the twin (photoreal.py): one root request
            from coupledcarla import photoreal
            photo = photoreal.attach(world, cfg, False, lambda m: print(m, flush=True))
            for _ in range(100):       # let the tileset start before the first view
                world.tick()
        seasons = {}        # --season photo: each view's leaf state on its photo's date (season.py)
        if getattr(args, "season", "leaf") == "photo":
            from datetime import date as _date
            from coupledcarla import season as season_mod
            ph = season_mod.phenology(bbox, mdir / "real", quick=True)
            for v in todo:
                try:
                    st_, pr_ = season_mod.leaf_state(_date.fromisoformat(v["local_time"][:10]), ph)
                except ValueError:
                    st_, pr_ = "leaf", 0.0
                seasons[v["id"]] = (st_, round(pr_, 1))
            todo.sort(key=lambda v: seasons[v["id"]])
            print("leaf states of the views: " + ", ".join(
                f"{k[0]} {sum(1 for x in seasons.values() if x == k)}" for k in sorted(set(seasons.values()))),
                flush=True)
        scen, scen_state = None, None

        def scenery_for(state):
            """The scenery spawned for a leaf state (state None: as built, trees in leaf)."""
            nonlocal scen, scen_state
            if args.no_scenery or not (mdir / "scenery.json").exists() or (scen is not None and state == scen_state):
                return
            if scen is not None:
                ids = [i for key in scen.loaded for i in scen.loaded[key]]
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
                spawned.remove(scen)
            sea = {"state": state[0], "progress": state[1], "bare": "small"} if state else None
            scen = Scenery(client, world, mdir, {"radius_m": 0}, season=sea, log=lambda m: None)
            scen.update([])
            spawned.append(scen)
            scen_state = state
            print(f"scenery: {scen.n} meshes" + (f" (leaves: {state[0]})" if state else ""), flush=True)
        scenery_for(seasons[todo[0]["id"]] if seasons and todo else None)
        parked_done = False
        cmap = world.get_map()
        lib = world.get_blueprint_library()
        capture_09 = lib.find("sensor.camera.rgb").has_attribute("exposure_mode")
        traffic = MovingCars(client, world, cmap, args.moving_cars, args.seed) if args.moving_cars > 0 else None
        if traffic is not None:
            spawned.append(traffic)
        rows = []
        wide = {}
        for v in todo:
            wide[v["id"]] = R.wide_size(v["width"], v["height"], v["hfov"], args.yaw_margin, args.pitch_margin)
        sizes = sorted({(w, h, round(f, 2)) for w, h, f in wide.values()})
        # views grouped by camera size, and one camera pair alive at a time: every camera streams a frame each
        # tick, and cameras kept for other sizes filled their queues until WSL ran out of memory (47 GB, the
        # crowd tmux service went down with it; 2026-11-15)
        todo.sort(key=lambda v: (seasons.get(v["id"], ("", 0.0)), wide[v["id"]][0], wide[v["id"]][1],
                                 round(wide[v["id"]][2], 2)))      # by leaf state first: one scenery per state
        cams = {}
        extra_attrs = dict(kv.split("=", 1) for kv in args.camera_attr.split(",") if "=" in kv)

        def camera(size, ev=None):
            if (size, ev) not in cams:
                for old_size in list(cams):          # the previous size's pair: destroyed, not left streaming
                    o_rgb, o_sem, _ = cams.pop(old_size)
                    for a in (o_rgb, o_sem):
                        try:
                            if a.is_listening:
                                a.stop()
                            a.destroy()
                        except RuntimeError:
                            pass
                        if a in actors:
                            actors.remove(a)
                w, h, f = size
                bp = lib.find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(w))
                bp.set_attribute("image_size_y", str(h))
                bp.set_attribute("fov", str(f))
                if bp.has_attribute("motion_blur_intensity"):
                    bp.set_attribute("motion_blur_intensity", "0")
                compat.setup_camera(bp, 0.0, args.profile)
                if ev is not None:      # 0.9.x capture look (look.py): no blur, flare or fringes, exposure ev
                    look.apply_09(bp, {"ev_bias": 0.0}, None, look.shutter_for_ev(ev))
                for k_, v_ in extra_attrs.items():
                    if bp.has_attribute(k_):
                        bp.set_attribute(k_, v_)
                sb = lib.find("sensor.camera.semantic_segmentation")
                sb.set_attribute("image_size_x", str(w))
                sb.set_attribute("image_size_y", str(h))
                sb.set_attribute("fov", str(f))
                tf = carla.Transform(carla.Location(z=500))
                rgb, sem = world.spawn_actor(bp, tf), world.spawn_actor(sb, tf)
                q = queue.Queue(maxsize=8)       # bounded: a frame nobody reads is dropped, not kept

                def put(img, q=q):
                    try:
                        q.put_nowait(img)
                    except queue.Full:
                        try:
                            q.get_nowait()
                            q.put_nowait(img)
                        except (queue.Empty, queue.Full):
                            pass
                rgb.listen(put)
                actors.extend([rgb, sem])
                cams[(size, ev)] = (rgb, sem, q)
            return cams[(size, ev)]
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
            if seasons:
                scenery_for(seasons[v["id"]])
            hour = v["local_time"][:13]
            if hour and hour != weather_key and args.weather != "fixed":
                from coupledcarla.run import set_weather
                sc = dict(cfg["simulation"], weather=args.weather, time=v["local_time"])
                if args.cloud_scale is not None:
                    sc["cloud_scale"] = args.cloud_scale
                set_weather(world, sc, (v["lat"], v["lon"]), mdir / "real", log=lambda m: None)
                weather_key = hour
            if not parked_done and not args.no_parked:
                parked_done = True
                try:
                    from coupledcarla import parking
                    tc = dict(cfg, map=dict(cfg["map"], bbox=bbox),
                              simulation=dict(cfg["simulation"], time=v["local_time"] or None))
                    data = parking.trial_data(mdir, tc, log=lambda m: None)
                    if not (mdir / "parked.json").exists():
                        print(f"no parked cars: {mdir / 'parked.json'} is missing (python -m coupledcarla.mapbuild "
                              f"{args.experiment} --scenery makes it)", flush=True)
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
            ev = None
            if args.look == "capture" and capture_09:
                wt = world.get_weather()
                ev = round(4 * (look.exposure_ev(wt.sun_altitude_angle, wt.cloudiness) + args.ev_bias)) / 4
            rgb, sem, q = camera((*wide[v["id"]][:2], round(wide[v["id"]][2], 2)), ev)
            rgb.set_transform(tf)
            sem.set_transform(tf)
            spectator.set_transform(tf)
            if traffic is not None:
                traffic.place(v["id"], loc, tf.rotation.yaw, wide[v["id"]][2])
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
                         "sun_alt": round(world.get_weather().sun_altitude_angle, 1),
                         "cloudiness": round(world.get_weather().cloudiness, 1), "ev": "" if ev is None else ev,
                         "moving_cars": traffic.n if traffic is not None else 0})
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
        if photo is not None:
            photo.destroy()
        for sc in spawned:        # leave the server as it was: scenery, parked and moving cars removed
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
    """Segmentation with a cache of label maps: the renders' in <out>/seg/, the photos' (real_*, pano_*) with the
    photos (photos_dir: the map's real/streetphotos/seg/), where every run finds them."""

    def __init__(self, out, model, device):
        self.dir = Path(out) / "seg"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.pdir = photos_dir(out) / "seg"
        self.pdir.mkdir(parents=True, exist_ok=True)
        self.model, self.device, self.seg = model, device, None

    def __call__(self, name, rgb, width=None):
        from PIL import Image
        p = (self.dir if name.startswith("render_") else self.pdir) / f"{name}.png"
        if p.exists():
            return np.asarray(Image.open(p))
        if self.seg is None:
            self.seg = R.Segmenter(self.model, self.device)
        lab = self.seg(rgb, width)
        Image.fromarray(lab).save(p)
        return lab

    def pano(self, src_id, path):
        """Labels of a whole panorama, inferred at 2048 x 1024 (Cityscapes' frame size), and the image at that size."""
        from PIL import Image
        im = Image.open(path).convert("RGB")
        rgb = np.asarray(im.resize((2048, 1024), Image.BILINEAR)) if im.width != 2048 else np.asarray(im)
        return self(f"pano_{src_id}", rgb, 2048), np.asarray(im)


def _load(path):
    from PIL import Image
    return np.asarray(Image.open(path).convert("RGB"))


def _fit(g, w, h):
    from PIL import Image
    return g if g.shape[:2] == (h, w) else np.asarray(Image.fromarray(g).resize((w, h), Image.NEAREST))


def view_metrics(v, real, wide_rgb, wide_lab, wide_hfov, gt_wide=None, yaw_range=12.0, pitch_range=6.0,
                 pano_yaw_range=30.0, fixed=None):
    """All metrics of one view, at the stated pose (nom_) and at the best alignment (al_), or at `fixed` = (heading
    error, pitch error) taken from an earlier round's compare.csv (no search: the same alignment in both rounds).

    real: ("photo", rgb, labels) for a pinhole photo (the render is cut at offsets from its wider frame), or
    ("pano", pano_rgb, pano_labels, src_yaw) for a panorama (the photo is cut at offsets, the render stays)."""
    w, h, hfov = v["width"], v["height"], v["hfov"]
    wg = R.groups_of(wide_lab)
    nominal_g = R.reproject(wg, "pinhole", w, h, hfov, src_hfov=wide_hfov, nearest=True, fill=R.IGNORE)
    nominal_rgb = R.reproject(wide_rgb, "pinhole", w, h, hfov, src_hfov=wide_hfov)
    if real[0] == "pano":
        _, pano_rgb, pano_lab, src_yaw = real
        pg = R.groups_of(pano_lab)
        p0 = float(v.get("pitch") or 0.0)      # the render's camera pitch
        rg = R.reproject(pg, "equirect", w, h, hfov, pitch=p0, src_yaw=src_yaw, nearest=True)
        m = {f"nom_{k}": val for k, val in R.compare_labels(rg, nominal_g, hfov).items()}
        if fixed is None:
            err, perr, real_al_g, am, (cy, cp) = R.align_pano(pg, nominal_g, hfov, w, h, src_yaw, pano_yaw_range,
                                                              pitch_range, pitch0=p0)
        else:       # align_pano reports the cut negated
            err, perr, real_al_g, am, (cy, cp) = R.align_pano(pg, nominal_g, hfov, w, h, src_yaw, 0.0, 0.0,
                                                              pitch0=p0, center=(-fixed[0], -fixed[1]))
        real_rgb = R.reproject(pano_rgb, "equirect", w, h, hfov, yaw=cy, pitch=p0 + cp, src_yaw=src_yaw)
        ren_g, ren_rgb, real_g = nominal_g, nominal_rgb, real_al_g
    else:
        _, real_rgb, real_lab = real
        rg = _fit(R.groups_of(real_lab), w, h)
        if real_rgb.shape[:2] != (h, w):
            from PIL import Image
            real_rgb = np.asarray(Image.fromarray(real_rgb).resize((w, h), Image.BILINEAR))
        m = {f"nom_{k}": val for k, val in R.compare_labels(rg, nominal_g, hfov).items()}
        if fixed is None:
            err, perr, ren_g, am = R.align(rg, wg, hfov, w, h, wide_hfov, yaw_range, pitch_range)
        else:
            err, perr, ren_g, am = R.align(rg, wg, hfov, w, h, wide_hfov, 0.0, 0.0, center=fixed)
        ren_rgb = R.reproject(wide_rgb, "pinhole", w, h, hfov, yaw=err, pitch=perr, src_hfov=wide_hfov)
        real_g = rg
    m.update({f"al_{k}": val for k, val in am.items()})
    m.update(heading_error=err, pitch_error=perr, heading_error_abs=abs(err))
    mask = (real_g != R.IGNORE) & (ren_g != R.IGNORE)
    sr, sd = R.image_stats(real_rgb, mask), R.image_stats(ren_rgb, mask)
    for k in sr:
        m[f"real_{k}"], m[f"render_{k}"] = sr[k], sd[k]
    m["colour_hist_sim"] = R.colour_hist_sim(real_rgb, ren_rgb, mask)
    if gt_wide is not None:     # the model on the render against CARLA's own labels: how far to trust it there
        gt = R.reproject(R.groups_of(gt_wide, R.CARLA_TO_GROUP), "pinhole", w, h, hfov, src_hfov=wide_hfov,
                         nearest=True, fill=R.IGNORE)
        g = R.compare_labels(gt, nominal_g, hfov)
        m["model_vs_carla_pix_acc"], m["model_vs_carla_miou_coarse"] = g["pix_acc"], g["miou_coarse"]
    return m, real_g, ren_g, ren_rgb, real_rgb


def real_spec(v, out, labels, sd=False):
    """The real side of a view for view_metrics (fetched and labelled as needed)."""
    if v["projection"] == "equirect":
        pl, prgb = labels.pano(v["src_id"], pano_image(v, out, sd))
        return ("pano", prgb, pl, float(v["src_heading"]) - float(v["heading"]))     # labels at 2048 x 1024
    rgb = _load(real_image(v, out, sd))
    return ("photo", rgb, labels(f"real_{v['id']}", rgb))


def _font(size):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=size)
    except TypeError:       # Pillow < 10.1: the fixed bitmap font
        return ImageFont.load_default()


CREDIT = "Photos: their authors via {src} ({lic}); per-photo credits in ATTRIBUTION.csv"


def contact_sheets(rows, out, name, per_sheet=12, thumb=384, credit=None):
    """Real | render | real layout | render layout, one view per row, with its scores (caption lines split at \n)
    and the photo credit at the bottom."""
    from PIL import Image, ImageDraw
    sdir = Path(out) / "sheets"
    sdir.mkdir(parents=True, exist_ok=True)
    paths = []
    font, lh = _font(13), 16
    for s in range(0, len(rows), per_sheet):
        chunk = rows[s:s + per_sheet]
        hs = [int(round(thumb * r["imgs"][0].shape[0] / r["imgs"][0].shape[1])) for r in chunk]
        caps = [r["caption"].split("\n") for r in chunk]
        sheet = Image.new("RGB", (thumb * 4 + 8, sum(hs) + sum(6 + lh * len(c) for c in caps) + 24), (255, 255, 255))
        dr = ImageDraw.Draw(sheet)
        y = 0
        for r, hh, cap in zip(chunk, hs, caps):
            for li, line in enumerate(cap):
                dr.text((4, y + 3 + li * lh), line, fill=(0, 0, 0) if li == 0 else (150, 30, 30), font=font)
            y += 6 + lh * len(cap)
            for i, im in enumerate(r["imgs"]):
                sheet.paste(Image.fromarray(im).resize((thumb, hh)), (i * (thumb + 2), y))
            y += hh
        dr.text((4, y + 5), credit or CREDIT.format(src="Panoramax, KartaView, Wikimedia Commons, Mapillary",
                                                    lic="CC BY-SA 4.0 or per file"), fill=(60, 60, 60), font=font)
        p = sdir / f"{name}_{s // per_sheet + 1:02d}.jpg"
        sheet.save(p, quality=85)
        paths.append(p)
    return paths


SUMMARY_KEYS = ["nom_miou_coarse", "al_miou_coarse", "nom_pix_acc_coarse", "al_pix_acc_coarse", "nom_miou", "al_miou",
                "nom_pix_acc", "al_pix_acc", "nom_layout_tv", "al_layout_tv", "nom_skyline_mae_deg",
                "al_skyline_mae_deg", "colour_hist_sim", "heading_error_abs", "model_vs_carla_pix_acc",
                "model_vs_carla_miou_coarse", "boundary_f", "dino_global", "dino_patch", "spot_score"]


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


NOT_STREET_GROUND, NOT_STREET_SKY = 0.03, 0.02


def load_exclusions(path):
    """{view id: reason} from a YAML file of {place: {view id: reason}} (view ids are the sources' own, unique)."""
    if not path or not Path(path).exists():
        return {}
    import yaml
    data = yaml.safe_load(Path(path).read_text()) or {}
    return {str(i): str(r) for ids in data.values() for i, r in (ids or {}).items()}


def not_street(row, listed=None):
    """Why a reference photo is not a street view, or "". Listed (looked at: indoors, a close-up of one object,
    people filling the frame, a view from high up, sky only), or seen by its own labels at the stated pose: no ground
    (road + sidewalk under 3 %) and no sky (under 2 %), i.e. a wall, a hedge or an interior fills it. Such a photo
    says nothing about the street the twin has to match; its views are left out of the summary and the index."""
    if listed and str(row.get("id")) in listed:
        return listed[str(row["id"])]
    if row.get("nom_real_road") in (None, ""):
        return ""
    try:
        ground = float(row.get("nom_real_road") or 0) + float(row.get("nom_real_sidewalk") or 0)
        sky = float(row.get("nom_real_sky") or 0)
    except (TypeError, ValueError):
        return ""
    if ground < NOT_STREET_GROUND and sky < NOT_STREET_SKY:
        return "no ground and no sky in the photo (a wall, a hedge or an interior)"
    return ""


def read_alignment(path):
    """{view id: (heading error, pitch error)} from an earlier round's compare.csv."""
    out = {}
    if path and Path(path).exists():
        with Path(path).open(newline="") as f:
            for r in csv.DictReader(f):
                try:
                    out[r["id"]] = (float(r["heading_error"]), float(r["pitch_error"]))
                except (KeyError, TypeError, ValueError):
                    pass
    return out


def cmd_compare(args):
    from PIL import Image
    out = Path(args.out)
    views = {v["id"]: v for v in R.read_views(out / "views.csv")}
    renders = {}
    with (out / "renders.csv").open(newline="") as f:
        for r in csv.DictReader(f):
            renders[r["id"]] = r
    labels = Labels(out, args.model, args.device)
    perc = Perceptual(args) if args.perceptual else None
    listed = load_exclusions(args.exclude)
    fixed = read_alignment(args.align_from)
    if args.align_from:
        print(f"alignment of {len(fixed)} views taken from {args.align_from}", flush=True)
    rows, sheet_rows = [], []
    t0 = time.time()
    for k, (vid, rr) in enumerate(renders.items()):
        v = views.get(vid)
        rp = out / "renders" / f"{vid}.png"
        if v is None or not rp.exists():
            continue
        try:
            real = real_spec(v, out, labels, args.sd)
        except Exception as err:  # noqa: BLE001
            print(f"{vid}: no real image ({err})", flush=True)
            continue
        wide_rgb = _load(rp)
        sp = out / "renders" / f"{vid}_sem.png"
        gt = np.asarray(Image.open(sp)) if sp.exists() else None
        m, rg, ag, arg, real_rgb = view_metrics(v, real, wide_rgb, labels(f"render_{vid}", wide_rgb),
                                                float(rr["wide_hfov"]), gt, args.yaw_margin, args.pitch_margin,
                                                args.pano_yaw, fixed=fixed.get(vid))
        extra_metrics(m, rg, ag, real_rgb, arg, v["hfov"], perc)
        m["excluded"] = not_street({"id": vid, **m}, listed)
        if gt is not None:      # CARLA's own labels: a view that is nearly all wall stands in or against a building
            from coupledcarla import streetphotos as SP
            m["wall_share"] = SP.wall_share(gt)
            if m["wall_share"] > SP.WALL_SHARE:
                m["pose_suspect"] = 1
                m["differences"] = "camera in or against a building (photo position off?)" + (
                    "; " + m["differences"] if m["differences"] else "")
        rows.append({"id": vid, "source": v["source"], "kind": v["kind"], "lat": v["lat"], "lon": v["lon"],
                     "heading": v["heading"], "local_time": v["local_time"], **m})
        imgs = [real_rgb, arg, R.colourise(rg), R.colourise(ag)]
        cap = _caption(vid, v["source"] + " " + v["local_time"], m)
        sheet_rows.append({"imgs": imgs, "caption": cap})
        spot_image(out / "spots" / f"{vid}.jpg", imgs, cap, v)
        if (k + 1) % 20 == 0:
            print(f"{k + 1}/{len(renders)} compared ({(time.time() - t0) / (k + 1):.1f} s each)", flush=True)
    _write_rows(out / "compare.csv", rows)
    from coupledcarla import streetphotos as SP
    kept = [r for r in rows if not r.get("excluded")]       # not street views: in compare.csv, not in the summary
    summ = {"views": summarise(kept), "model": args.model, "groups": R.GROUPS,
            "ignored": "person, rider, motorcycle, bicycle (both images)", "map": SP.map_score(kept),
            "perceptual_model": args.dino_model if args.perceptual else None,
            "excluded": {r["id"]: r["excluded"] for r in rows if r.get("excluded")},
            "aligned_from": args.align_from or None,
            "worst": [{"id": r["id"], "spot_score": round(r["spot_score"], 3), "differences": r["differences"]}
                      for r in sorted(kept, key=lambda r: _num(r["spot_score"]))[:20]]}
    pairs = out / "pairs_summary.json"
    if pairs.exists():
        summ["real_vs_real"] = json.loads(pairs.read_text())
        summ["realism_index"] = realism_index(summ["views"]["all"], summ["real_vs_real"])
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    order = sorted(range(len(rows)), key=lambda i: -_num(rows[i]["spot_score"]))     # best match first
    paths = contact_sheets([sheet_rows[i] for i in order], out, "twin")
    print(f"map score {summ['map']['score']} (median spot score x 100 over {summ['map']['spots']} spots); most "
          f"frequent differences: " + "; ".join(f"{k} {n}" for k, n in list(summ["map"]["differences"].items())[:6]),
          flush=True)
    print(json.dumps({k: summ["views"]["all"].get(k) for k in ["n"] + SUMMARY_KEYS}, indent=1))
    if "realism_index" in summ:
        print("realism index (0 = like a random real place, 1 = like a real photo of the same place):",
              json.dumps(summ["realism_index"]))
    print(f"{len(rows)} views compared; {out / 'compare.csv'}, {out / 'summary.json'}, {len(paths)} sheets in "
          f"{out / 'sheets'}", flush=True)


def _num(x):
    try:
        x = float(x)
        return x if not math.isnan(x) else -1.0
    except (TypeError, ValueError):
        return -1.0


def _caption(vid, what, m):
    cap = (f"{vid} {what}  coarse mIoU {m['al_miou_coarse']:.2f} (stated pose {m['nom_miou_coarse']:.2f}; heading "
           f"{m['heading_error']:+.0f}, pitch {m['pitch_error']:+.0f})  pix {m['al_pix_acc']:.2f}  skyline "
           f"{m['al_skyline_mae_deg']:.1f} deg")
    if "spot_score" in m:
        cap += (f"  edges F {_num(m.get('boundary_f')):.2f}" +
                (f"  DINO {m['dino_patch']:.2f}" if "dino_patch" in m else "") + f"  spot {m['spot_score']:.2f}")
        if m.get("differences"):
            cap += "\n" + m["differences"]
    return cap


class Perceptual:
    """The DINOv2 perceptual similarity (streetphotos.Perceptual), loaded on first use."""

    def __init__(self, args):
        self.args, self.p = args, None

    def __call__(self, a, b):
        if self.p is None:
            from coupledcarla.streetphotos import Perceptual as P
            self.p = P(self.args.dino_model, self.args.device)
        return self.p(a, b)


def extra_metrics(m, real_g, ren_g, real_rgb, ren_rgb, hfov, perc=None):
    """Adds the boundary F-score, the differences in words, the DINOv2 similarity and the spot score to m."""
    from coupledcarla import streetphotos as SP
    m["boundary_f"] = SP.boundary_f(R.COARSE[real_g], R.COARSE[ren_g])
    m["differences"] = "; ".join(t for _, t in SP.differences(real_g, ren_g, float(hfov), R.GROUPS))
    if perc is not None:
        m.update(perc(real_rgb, ren_rgb))
    m["spot_score"] = SP.spot_score(m)
    return m


def spot_image(path, imgs, caption, v):
    """One spot side by side: photo | render over photo layout | render layout, the scores and differences below,
    and the photo's credit."""
    from PIL import Image, ImageDraw
    import textwrap
    w, h = imgs[0].shape[1], imgs[0].shape[0]
    width = max(40, int((2 * w + 4) / 8.2))         # characters per line of the 15 px font
    first, *rest = caption.split("\n")
    lines = textwrap.wrap(first, width) + [x for r in rest for x in textwrap.wrap(r, width)]
    n_head = len(textwrap.wrap(first, width))
    credit = f"Photo: {v.get('author') or 'unknown'}, {v.get('licence') or '?'}, {v.get('source')} {v.get('page')}"
    lh = 18
    im = Image.new("RGB", (2 * w + 4, 2 * h + 4 + lh * (len(lines) + 1) + 10), (255, 255, 255))
    for i, a in enumerate(imgs):
        im.paste(Image.fromarray(a).resize((w, h)), ((i % 2) * (w + 4), (i // 2) * (h + 4)))
    dr = ImageDraw.Draw(im)
    font = _font(15)
    y = 2 * h + 8
    for li, line in enumerate(lines + [credit]):
        dr.text((6, y + li * lh), line, fill=(0, 0, 0) if li < n_head else (150, 30, 30) if li < len(lines)
                else (70, 70, 70), font=font)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    im.save(path, quality=88)


def realism_index(twin, rr, ref="neighbour"):
    """Per metric: where the twin's median lies between real photos of random places (0) and real photos of the same
    street 10-25 m on ('neighbour', 1)."""
    out = {}
    lo, hi = rr.get("random") or {}, rr.get(ref) or {}
    for k in ("al_miou_coarse", "al_pix_acc_coarse", "al_miou", "al_layout_tv", "al_skyline_mae_deg",
              "colour_hist_sim", "boundary_f", "dino_global", "dino_patch", "spot_score"):
        a, b, t = lo.get(k), hi.get(k), twin.get(k)
        if None not in (a, b, t) and abs(b - a) > 1e-6:
            out[k] = round((t - a) / (b - a), 2)
    return out


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
    """Real-vs-real baseline: what the metrics give for two real photos of the same place (another ride within 8 m),
    of the same street 10-25 m on, and of random places; a twin render should score between 'random' and the others."""
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    set_photos_dir(out, mdir, getattr(args, "photos", None))
    recs = R.fetch(mdir, bbox, sources=("panoramax", "kartaview"), refresh=False)
    views = R.read_views(out / "views.csv")
    pairs = make_pairs(views, R.daytime(recs), args.max_pairs)
    print(f"{len(pairs)} pairs: " + ", ".join(f"{k} {sum(p[2] == k for p in pairs)}"
                                             for k in ("same", "neighbour", "random")), flush=True)
    labels = Labels(out, args.model, args.device)
    perc = Perceptual(args) if args.perceptual else None
    rows, sheet_rows = [], []
    for k, (v, r, kind) in enumerate(pairs):
        try:
            real = real_spec(v, out, labels, args.sd)
            ov = dict(r, id="", projection="equirect")
            ppath = pano_image(ov, out, args.sd)
            ol, orgb = labels.pano(r["src_id"], ppath)
        except Exception as err:  # noqa: BLE001
            print(f"pair {v['id']}: skipped ({err})", flush=True)
            continue
        # b plays the render: cut wide at a's heading (b's own heading for a random place), labels from its panorama
        w2, h2, f2 = R.wide_size(v["width"], v["height"], v["hfov"], args.yaw_margin, args.pitch_margin)
        head = v["heading"] if kind != "random" else r["heading"]
        wide = R.reproject(orgb, "equirect", w2, h2, f2, src_yaw=r["heading"] - head)
        wide_lab = R.reproject(ol, "equirect", w2, h2, f2, src_yaw=r["heading"] - head, nearest=True)
        m, rg, ag, arg, real_rgb = view_metrics(v, real, wide, wide_lab, f2, None, args.yaw_margin,
                                                args.pitch_margin, args.pano_yaw)
        extra_metrics(m, rg, ag, real_rgb, arg, v["hfov"], perc)
        rows.append({"id": v["id"], "other": r["src_id"], "kind": kind,
                     "dist_m": round(R.metres(r["lat"], r["lon"], v["lat"], v["lon"]), 1), **m})
        if len(sheet_rows) < 36:
            sheet_rows.append({"imgs": [real_rgb, arg, R.colourise(rg), R.colourise(ag)],
                               "caption": _caption(v["id"], f"{kind} vs {r['src_id'][:8]} ({rows[-1]['dist_m']} m)", m)})
        if (k + 1) % 20 == 0:
            print(f"{k + 1}/{len(pairs)} pairs", flush=True)
    _write_rows(out / "pairs.csv", rows)
    summ = summarise(rows, by="kind")
    (out / "pairs_summary.json").write_text(json.dumps(summ, indent=1))
    contact_sheets(sheet_rows, out, "pairs")
    for kind in ("same", "neighbour", "random"):
        if kind in summ:
            print(kind, json.dumps({k: summ[kind].get(k) for k in ["n"] + SUMMARY_KEYS if k in summ[kind]}))


TABLE_KEYS = [("al_miou_coarse", "coarse IoU"), ("al_pix_acc_coarse", "coarse pixel acc."),
              ("al_skyline_mae_deg", "skyline error (deg)"), ("colour_hist_sim", "colour hist. sim."),
              ("lum_mean", "luminance mean"), ("lum_std", "luminance spread"), ("sat_mean", "saturation"),
              ("edge_density", "edge density"), ("area_road", "road share"), ("area_vegetation", "vegetation share"),
              ("area_vehicle", "vehicle share"), ("area_sky", "sky share")]


def cmd_table(args):
    """Several compared runs side by side, over the views all of them have (medians; [photo, render] pairs give the
    render's value, the photo's in the first column). Writes <first out>/table.json when --json is given."""
    runs = {}
    for out in args.runs:
        with (Path(out) / "compare.csv").open(newline="") as f:
            runs[out] = {r["id"]: r for r in csv.DictReader(f)}
    common = set.intersection(*(set(r) for r in runs.values()))
    res = {}
    for out, rows in runs.items():
        summ = summarise([rows[i] for i in sorted(common)])["all"]
        res[out] = summ
    names = [Path(o).name for o in runs]
    print(f"{len(common)} views in all of {len(runs)} runs")
    print("| Measure | Photos | " + " | ".join(names) + " |")
    print("|---|---|" + "---|" * len(names))
    for k, label in TABLE_KEYS:
        vals = [res[o].get(k) for o in runs]
        if all(v is None for v in vals):
            continue
        photo = next((v[0] for v in vals if isinstance(v, list)), "")
        cells = [(v[1] if isinstance(v, list) else v) for v in vals]
        fmt = (lambda x: "" if x is None else (f"{x:.3f}" if abs(x) < 10 else f"{x:.1f}"))
        print(f"| {label} | {fmt(photo) if photo != '' else ''} | " + " | ".join(fmt(c) for c in cells) + " |")
    if args.json:
        Path(args.json).write_text(json.dumps({"views": len(common), "runs": res}, indent=1))


# ---------------------------------------------------------------- sources and street objects

def cmd_sources(args):
    """How many street photos each source has in the map's box (by year and camera), and what Mapillary offers:
    written to <photos>/coverage.json."""
    from collections import Counter

    from coupledcarla import streetphotos as SP
    cfg, mdir, bbox = _map(args.experiment)
    pdir = Path(args.photos or mdir / "real" / "streetphotos")
    cov = {"bbox": bbox, "made": time.strftime("%Y-%m-%d %H:%M"), "sources": {}}
    for src in [x for x in args.sources.split(",") if x != "mapillary"]:
        recs = R.fetch(mdir, bbox, sources=(src,), refresh=args.refresh)
        day = R.daytime(recs)
        cov["sources"][src] = {
            "photos": len(recs), "daytime": len(day), "panoramas": sum(r["projection"] == "equirect" for r in recs),
            "years": dict(sorted(Counter((r.get("time") or "?")[:4] for r in recs).items())),
            "licence": R.SOURCES[src][1]}
    if "mapillary" in args.sources.split(","):
        try:
            m = SP.Mapillary(pdir)
            pts = [p for p in m.points("mly1_public", bbox) if p.get("layer") == "image"]
            feats = m.features(bbox)
            try:
                kept = len(m.images(bbox))
            except SP.FetchError:
                kept = 0
            cov["sources"]["mapillary"] = {
                "photos": len(pts), "panoramas": sum(bool(p.get("is_pano")) for p in pts),
                "years": dict(sorted(Counter(time.strftime("%Y", time.gmtime((p.get("captured_at") or 0) / 1000))
                                             for p in pts).items())),
                "map_features": len(feats),
                "feature_classes": dict(Counter(SP.mly_class(f["value"], f["layer"]) or "other" for f in feats)),
                "image_search": m.last_stats, "usable": kept,
                "licence": SP.MAPILLARY[1]}
        except SP.FetchError as err:
            cov["sources"]["mapillary"] = {"error": str(err)}
    pdir.mkdir(parents=True, exist_ok=True)
    (pdir / "coverage.json").write_text(json.dumps(cov, indent=1))
    print(f"{cfg['name']} {bbox}")
    for src, c in cov["sources"].items():
        print(f"  {src}: " + json.dumps(c))


def _spot_pair(path, w, h):
    """(photo, aligned render) cut back out of a spot image (photo | render over the two layouts)."""
    a = _load(path)
    return a[:h, :w].copy(), a[:h, w + 4:2 * w + 4].copy()


def cmd_critique(args):
    """A vision-language model lists what a person would notice between photo and render, per spot, as JSON
    (category, what, where, severity); per run the themes that make the renders look fake, ranked. Off by default
    (this command only). Claude with the key anthropic_api_key in the secret file, else (or --backend local) an
    open model on this machine's GPU. Writes <out>/critique/<id>.json, critique_summary.json and summary.json's
    'critique'."""
    from coupledcarla import streetphotos as SP
    from coupledcarla.settings import get_secret
    out = Path(args.out)
    views = {v["id"]: v for v in R.read_views(out / "views.csv")}
    with (out / "compare.csv").open(newline="") as f:
        rows = [r for r in csv.DictReader(f) if (out / "spots" / f"{r['id']}.jpg").exists()]
    rows.sort(key=lambda r: _num(r.get("spot_score")))
    if args.max_spots and len(rows) > args.max_spots:     # spread over the scores, worst to best
        step = len(rows) / args.max_spots
        rows = [rows[int(i * step)] for i in range(args.max_spots)]
    if args.backend == "claude" and not get_secret("anthropic_api_key")[0]:
        print("critique skipped: no anthropic_api_key in the project's secret file (python -m coupledcarla.settings "
              "--set anthropic_api_key), and --backend claude asked for Claude; --backend local runs an open model "
              "on this machine's GPU instead", flush=True)
        return
    cdir = out / "critique"
    cdir.mkdir(parents=True, exist_ok=True)
    critic, done = None, []
    t0 = time.time()
    for k, r in enumerate(rows):
        path = cdir / f"{r['id']}.json"
        if path.exists() and not args.again:     # parsed again from the raw answer (the parser may have improved)
            old = json.loads(path.read_text())
            done.append((r["id"], SP.parse_critique(old.get("raw")) if old.get("raw") else old["differences"]))
            continue
        if critic is None:
            kw = {"quant": args.quant} if args.backend != "claude" and not (
                args.backend == "auto" and get_secret("anthropic_api_key")[0]) else {"effort": args.effort}
            critic = SP.make_critic(args.backend, args.vlm, **kw)
            print(f"critic: {critic.name} {getattr(critic, 'model_id', getattr(critic, 'model', ''))}", flush=True)
        v = views[r["id"]]
        photo, render = _spot_pair(out / "spots" / f"{r['id']}.jpg", int(v["width"]), int(v["height"]))
        try:
            diffs, raw = critic(photo, render)
        except Exception as err:  # noqa: BLE001 - one spot the model fails on is not fatal
            print(f"{r['id']}: no critique ({type(err).__name__}: {err})", flush=True)
            continue
        path.write_text(json.dumps({"id": r["id"], "critic": critic.name, "spot_score": r.get("spot_score"),
                                    "differences": diffs, "raw": raw}, indent=1))
        done.append((r["id"], diffs))
        if (k + 1) % 10 == 0:
            print(f"{k + 1}/{len(rows)} spots critiqued ({(time.time() - t0) / (k + 1):.1f} s each)", flush=True)
    agg = SP.aggregate_critiques(done)
    agg["critic"] = critic.name if critic else "cached"
    (out / "critique_summary.json").write_text(json.dumps(agg, indent=1))
    sp = out / "summary.json"
    if sp.exists():
        summ = json.loads(sp.read_text())
        summ["critique"] = {k: agg[k] for k in ("critic", "spots", "categories")}
        summ["critique"]["themes"] = agg["themes"][:10]
        sp.write_text(json.dumps(summ, indent=1))
    print(f"what makes it look fake ({agg['spots']} spots; weight = summed severity of the spots naming it):")
    for t in agg["themes"]:
        print(f"  {t['weight']:3d}  {t['category']:9s} {t['theme']:12s} {t['spots']:3d} spots   "
              f"e.g. {t['examples'][0]}")


def _lane_check(mdir, cfg):
    """on_lane(x, y): True on a driving lane of the map's OpenDRIVE (carla.Map offline), or None without carla."""
    try:
        import carla
    except ImportError:
        return None
    xodr = Path(mdir) / "map.xodr"
    if not xodr.exists():
        return None
    cmap = carla.Map(cfg["map"].get("name") or cfg["name"], xodr.read_text(encoding="utf-8"))

    def on_lane(x, y):
        return cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.0), project_to_road=False,
                                 lane_type=carla.LaneType.Driving) is not None
    return on_lane


def _ground(mdir, geo):
    meta = json.loads((Path(mdir) / "meta.json").read_text())
    elev = meta.get("elevation")
    dem_path = Path(mdir) / "real" / "dem.npz"
    if not elev or not dem_path.exists():
        return None
    from coupledcarla import realworld
    dem = realworld.Dem.load(dem_path)

    def ground(x, y):
        return float(dem.sample(*geo.to_latlon(x, y))) - float(elev.get("base_m") or 0.0)
    return ground


def cmd_objects(args):
    """Street objects in the photos against the map's scenery.json: Mapillary's map features and/or poles found in
    Panoramax panoramas, matched by class; how many lamps, signs and pieces of furniture a correction pass would move,
    add or remove; --apply writes the --actions (default add, remove) into scenery.json."""
    from coupledcarla import streetphotos as SP
    from coupledcarla.geo import Geo
    cfg, mdir, bbox = _map(args.experiment)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    geo = Geo(mdir / "geo.net.xml")
    srcs = args.evidence.split(",")
    for src in srcs:
        if SP.evidence_path(mdir, src).exists() and not args.refresh:
            print(f"{src}: evidence cached ({SP.evidence_path(mdir, src)}; --refresh makes it again)", flush=True)
            continue
        if src == "mapillary":
            ev, cams = SP.mapillary_evidence(SP.Mapillary(mdir / "real" / "streetphotos"), bbox, geo)
        elif src in ("panoramax", "mapillary_panos"):      # poles found in the panoramas of a source
            set_photos_dir(out, mdir, getattr(args, "photos", None))
            psrc = "panoramax" if src == "panoramax" else "mapillary"
            recs = [r for r in R.daytime(R.fetch(mdir, bbox, sources=(psrc,)))
                    if r["projection"] == "equirect"]
            recs = _thin(recs, args.pano_spacing)[:args.max_panos]
            labels = Labels(out, args.model, args.device)

            def lab(r):
                try:
                    return labels.pano(r["src_id"], pano_image(dict(r, id=""), out, args.sd))[0]
                except Exception as err:  # noqa: BLE001 - a panorama that cannot be fetched is skipped
                    print(f"{r['src_id']}: skipped ({err})", flush=True)
                    return None
            ev, cams = SP.pano_evidence(recs, geo, lab, cam_h=R.HEIGHTS["pano" if psrc == "panoramax" else "mly_pano"])
        else:
            raise SystemExit(f"unknown evidence source {src} (mapillary, panoramax, mapillary_panos)")
        SP.save_evidence(mdir, src, ev, cams)
    protect = None if args.protect == "auto" else [x for x in args.protect.split(",") if x and x != "none"]
    actions = tuple(a for a in args.actions.split(",") if a)
    res = SP.correct_map(mdir, srcs, actions, protect, apply=args.apply,
                         on_lane=_lane_check(mdir, cfg) if args.apply else None,
                         ground=_ground(mdir, geo) if args.apply else None)
    if args.validate:       # the rules against a register: spoil it, correct it from the photos, compare
        data = json.loads((mdir / "scenery.json").read_text())
        for src in srcs:
            ev = json.loads(SP.evidence_path(mdir, src).read_text())
            obs = SP.Observations([tuple(c) for c in ev["cameras"]])
            res[src]["validation"] = [SP.validate_corrections(SP.scenery_objects(data), ev["evidence"], obs, cls,
                                                              shift_m=shift, seed=seed)
                                      for cls in args.validate.split(",") for shift in (3.0, 6.0) for seed in (1, 2)]
            for v in res[src]["validation"]:
                print(f"validation {src} {v['class']} shift {v['shift_m']} m: " + json.dumps(
                    {k: v[k] for k in ("before", "after", "moves", "moves_closer", "removes", "removes_of_made_up",
                                       "adds", "adds_at_a_register_object")}), flush=True)
    (out / "objects.json").write_text(json.dumps(res, indent=1))
    for src, r in res.items():
        prot = ", ".join(r["protected"]) or "none"
        print(f"\n{cfg['name']}: {src} evidence against scenery.json (protected: {prot})")
        print("| Class | Photos | Scenery (photographed) | Matched | Median offset (m) | Recall | Precision | "
              "Would move | add | remove |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for cls, st in r["stats"].items():
            c = r["summary"].get(cls, {})
            print(f"| {cls} | {st['evidence']} | {st['scenery']} ({st['scenery_photographed']}) | {st['matched']} | "
                  f"{st['median_offset_m'] if st['median_offset_m'] is not None else ''} | "
                  f"{st['recall'] if st['recall'] is not None else ''} | "
                  f"{st['precision'] if st['precision'] is not None else ''} | "
                  f"{'protected' if st['protected'] else c.get('move', 0)} | {c.get('add', 0)} | "
                  f"{c.get('remove', 0)} |")
        if r.get("applied"):
            print(f"applied to {mdir / 'scenery.json'}: {r['applied']} (old: scenery.before_photos.json)")
    print(f"{out / 'objects.json'}", flush=True)


def _thin(recs, spacing):
    """One record per spacing metres along each sequence."""
    out, last = [], {}
    for r in sorted(recs, key=lambda r: (r.get("sequence") or "", r.get("time") or "", r["src_id"])):
        q = last.get(r.get("sequence"))
        if q is not None and R.metres(q["lat"], q["lon"], r["lat"], r["lon"]) < spacing:
            continue
        last[r.get("sequence")] = r
        out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--out", default="logs/realism")
    common.add_argument("--yaw-margin", type=float, default=12.0, help="render this much wider each side (deg)")
    common.add_argument("--pitch-margin", type=float, default=6.0)
    common.add_argument("--sd", action="store_true", help="2048 px panoramas instead of full resolution")
    common.add_argument("--pano-yaw", type=float, default=30.0, help="heading search for panoramas (+- deg)")
    common.add_argument("--photos", default=None, help="photo cache (default: the map's real/streetphotos/)")
    views = argparse.ArgumentParser(add_help=False)
    views.add_argument("experiment")
    views.add_argument("--sources", default="panoramax,kartaview,commons")
    views.add_argument("--refresh", action="store_true", help="fetch the photo metadata again")
    views.add_argument("--spacing", type=float, default=20.0, help="one view per this many metres and direction")
    views.add_argument("--pano-offsets", default="0", help="views cut from each panorama, deg from its heading")
    views.add_argument("--max-views", type=int, default=350)
    views.add_argument("--action-cams", action="store_true", help="also handlebar action cams (roll, fisheye)")
    views.add_argument("--size", type=int, default=768, help="view width (px)")
    views.add_argument("--box", default=None, help="s,w,n,e: only views inside this part of the map")
    seg = argparse.ArgumentParser(add_help=False)
    seg.add_argument("--model", default="nvidia/segformer-b2-finetuned-cityscapes-1024-1024")
    seg.add_argument("--device", default=None)
    seg.add_argument("--no-perceptual", dest="perceptual", action="store_false",
                     help="leave out the DINOv2 similarity (transformers model, ~350 MB)")
    seg.add_argument("--dino-model", default="facebook/dinov2-base")
    p = sub.add_parser("prepare", parents=[common, views, seg])
    p.add_argument("--no-images", action="store_true")
    p.add_argument("--labels", action="store_true", help="also segment the real images (torch, transformers)")
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
    p.add_argument("--moving-cars", type=float, default=20.0, help="cars per km of driving lane in view (0: none)")
    p.add_argument("--seed", type=int, default=1, help="draw of the moving cars")
    p.add_argument("--look", default="capture", choices=["capture", "auto"],
                   help="CARLA 0.9.x: capture (look.py: manual exposure from sun and clouds, no blur or flare) or "
                        "auto (CARLA's auto exposure); CARLA 1.0 uses --profile")
    p.add_argument("--ev-bias", type=float, default=0.0, help="CARLA 0.9.x capture look: exposure offset (EV)")
    p.add_argument("--camera-attr", default="", help="extra RGB camera attributes, e.g. gamma=2.2,temp=7000")
    p.add_argument("--cloud-scale", type=float, default=None,
                   help="CARLA cloudiness per % of real cloud cover (default simulation.cloud_scale)")
    p.add_argument("--again", action="store_true", help="render views that already have a render")
    p.add_argument("--season", default="leaf", choices=["leaf", "photo"],
                   help="trees in leaf (default), or each view's leaf state on its photo's date (season.py)")
    p = sub.add_parser("compare", parents=[common, seg])
    p.add_argument("--exclude", default=None,
                   help="YAML {place: {view id: reason}}: reference photos that are not street views (left out of the "
                        "summary; photos with no ground and no sky are left out anyway)")
    p.add_argument("--align-from", default=None,
                   help="an earlier round's compare.csv: each view keeps that round's alignment (no search), so an "
                        "A/B difference cannot come from a re-picked alignment")
    p = sub.add_parser("pairs", parents=[common, seg])
    p.add_argument("experiment")
    p.add_argument("--max-pairs", type=int, default=60)
    p = sub.add_parser("sources", parents=[common], help="street photos per source in the map's box")
    p.add_argument("experiment")
    p.add_argument("--sources", default="panoramax,kartaview,commons,mapillary")
    p.add_argument("--refresh", action="store_true")
    p = sub.add_parser("critique", parents=[common],
                       help="a vision-language model's differences per spot, ranked per run (optional)")
    p.add_argument("--backend", default="auto", choices=["auto", "claude", "local"],
                   help="auto: Claude when anthropic_api_key is set, else the local model")
    p.add_argument("--vlm", default=None, help="model id (Claude: claude-opus-5-5; local: Qwen/Qwen2.5-VL-7B-Instruct)")
    p.add_argument("--quant", default="4bit", choices=["4bit", "8bit", "none"], help="local model quantisation")
    p.add_argument("--effort", default="medium", help="Claude's effort")
    p.add_argument("--max-spots", type=int, default=40, help="spots to critique, spread from worst to best (0: all)")
    p.add_argument("--again", action="store_true")
    p = sub.add_parser("objects", parents=[common, seg], help="street objects in the photos against scenery.json")
    p.add_argument("experiment")
    p.add_argument("--evidence", default="mapillary",
                   help="mapillary (its map features), panoramax or mapillary_panos (poles found in their panoramas)")
    p.add_argument("--refresh", action="store_true", help="find the evidence again (else the saved one)")
    p.add_argument("--max-panos", type=int, default=600)
    p.add_argument("--pano-spacing", type=float, default=6.0, help="one panorama per this many metres of a ride")
    p.add_argument("--protect", default="auto",
                   help="classes never corrected: auto (registers: city lamps, NDW signs), none, or e.g. lamp,sign")
    p.add_argument("--apply", action="store_true", help="write the corrections (--actions) into scenery.json")
    p.add_argument("--actions", default="add,remove",
                   help="what --apply does: add, remove, move (moves made Eindhoven's lamps worse: "
                        "docs/street_photos.md)")
    p.add_argument("--validate", default="", help="classes from a register (e.g. lamp): test the rules on it")
    p = sub.add_parser("table", help="compared runs side by side over their common views")
    p.add_argument("runs", nargs="+", help="output folders of compared runs (the first is the reference)")
    p.add_argument("--json", default=None)
    args = ap.parse_args()
    {"prepare": cmd_prepare, "render": cmd_render, "compare": cmd_compare, "pairs": cmd_pairs,
     "table": cmd_table, "sources": cmd_sources, "objects": cmd_objects, "critique": cmd_critique}[args.cmd](args)


if __name__ == "__main__":
    main()
