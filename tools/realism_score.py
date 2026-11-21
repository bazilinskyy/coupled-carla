"""The realism pilot's score: one table per round over a frozen set of places, views and top-down tiles.

    # once: draw the views (street photos) and top-down tiles (open aerial photos) and fix them
    python tools/realism_score.py freeze [--places a,b]
    # each round: renders (a running CARLA server with the maps; GPU) ...
    python tools/realism_score.py render r0 --port 2500 [--places a,b]
    # ... then the comparison (CPU or GPU) and the table
    python tools/realism_score.py score r0
    python tools/realism_score.py table r0 [r1 ...]

Street level is tools/realism_check.py on the frozen views (realism index: 0 = like a photo of a random place in the
box, 1 = like a second real photo of the same street; docs/street_photos.md). From above, a 200 m square rendered
straight down from 600 m is compared with the open aerial photo of the same square (coupledcarla/aerial.py):
vegetation cover IoU, colour histogram, DINOv2 similarity; its baselines are the aerial photo of another tile in
the box ("random") and, in the Netherlands, an earlier year's photo of the same tile ("same").
Places, roles (train / held_out) and counts: experiments/pilot/realism_pilot.yaml. Outputs under --root
(default logs/pilot): frozen/<place>/ and rounds/<round>/<place>/; table.md per round.
"""
import argparse
import csv
import json
import math
import random
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from coupledcarla import aerial  # noqa: E402
from coupledcarla import realism as R  # noqa: E402
from coupledcarla.provenance import MEASURED_BUILDINGS as MEASURED  # noqa: E402

PILOT = ROOT / "experiments" / "pilot" / "realism_pilot.yaml"
CHECK = [sys.executable, str(ROOT / "tools" / "realism_check.py")]
STREET_KEYS = ("spot_score", "al_miou_coarse", "boundary_f", "dino_patch", "dino_global", "al_skyline_mae_deg",
               "colour_hist_sim")
TOP_KEYS = ("veg_iou", "dino_patch", "dino_global", "colour_hist_sim", "bld_iou", "bld_iou_measured", "veg_recall",
            "green_excess_render", "green_excess_photo")
CAM_H = 600.0
MIN_PAIRS = 30          # a place's own real-vs-real scale needs this many neighbour and random pairs


def load(path=PILOT):
    return yaml.safe_load(Path(path).read_text())


def places(cfg, only=None):
    want = set(only.split(",")) if only else None
    return [p for p in cfg["places"] if want is None or p["name"] in want]


def _map(p):
    import realism_check as RC
    cfg, mdir, bbox = RC._map(str(ROOT / p["experiment"]))
    box = p.get("box") or bbox      # a part of the map, never beyond it (nothing to render there)
    return cfg, mdir, [max(box[0], bbox[0]), max(box[1], bbox[1]), min(box[2], bbox[2]), min(box[3], bbox[3])]


def spread(views, n, seed):
    """n views spread over the box: farthest-point order from a seeded start, all sources mixed."""
    if len(views) <= n:
        return list(views)
    rng = random.Random(seed)
    lat0 = sum(v["lat"] for v in views) / len(views)
    xy = np.array([[v["lon"] * 111320 * math.cos(math.radians(lat0)), v["lat"] * 110540] for v in views])
    first = rng.randrange(len(views))
    chosen = [first]
    d = np.hypot(*(xy - xy[first]).T)
    while len(chosen) < n:
        k = int(np.argmax(d))
        chosen.append(k)
        d = np.minimum(d, np.hypot(*(xy - xy[k]).T))
    return [views[k] for k in sorted(chosen)]


def tile_views(p, mdir, box, n, tile_m):
    """Top-down 'views': n tile centres on a grid inside the box (each tile wholly inside), camera 600 m up."""
    from coupledcarla.geo import Geo
    geo = Geo(mdir / "geo.net.xml")
    s, w, nn, e = box
    x0, y0 = geo.to_carla(s, w)
    x1, y1 = geo.to_carla(nn, e)
    xs, ys = sorted((x0, x1)), sorted((y0, y1))
    half = tile_m / 2
    k = max(1, math.ceil(math.sqrt(n)))
    cx = np.linspace(xs[0] + half, xs[1] - half, k) if xs[1] - xs[0] > tile_m else [sum(xs) / 2]
    cy = np.linspace(ys[0] + half, ys[1] - half, k) if ys[1] - ys[0] > tile_m else [sum(ys) / 2]
    hfov = math.degrees(2 * math.atan(half / CAM_H))
    out = []
    for i, x in enumerate(cx):
        for j, y in enumerate(cy):
            if len(out) >= n:
                break
            lat, lon = geo.to_latlon(float(x), float(y))
            corners = [geo.to_latlon(float(x) + dx, float(y) + dy) for dx in (-half, half) for dy in (-half, half)]
            out.append({"id": f"top_{i}{j}", "source": "aerial", "src_id": f"{p['name']}_{i}{j}", "kind": "topdown",
                        "lat": round(lat, 7), "lon": round(lon, 7), "heading": 0.0, "pitch": -90.0, "roll": 0.0,
                        "hfov": round(hfov, 3), "width": 512, "height": 512, "height_m": CAM_H,
                        "projection": "pinhole", "src_heading": 0.0, "utc": "2024-06-21T11:00:00Z",
                        "local_time": "2024-06-21 13:00", "sun_alt": "", "url": "", "url_sd": "", "page": "",
                        "licence": aerial.CREDIT.get(p.get("country"), ""), "author": "", "camera": "", "sequence": "",
                        "accuracy_m": "", "_bbox": [min(c[0] for c in corners), min(c[1] for c in corners),
                                                    max(c[0] for c in corners), max(c[1] for c in corners)]})
    return out


def cmd_freeze(args):
    import realism_check as RC
    cfg = load(args.pilot)
    root = Path(args.root)
    for p in places(cfg, args.places):
        out = root / "frozen" / p["name"]
        out.mkdir(parents=True, exist_ok=True)
        ecfg, mdir, box = _map(p)
        print(f"== {p['name']} ({p['role']}): {mdir.name}, box {box}", flush=True)
        RC.set_photos_dir(out, mdir)
        if not (out / "views.csv").exists() or args.again:
            recs = R.fetch(mdir, json.loads((mdir / "meta.json").read_text())["bbox"],
                           sources=p.get("sources", "panoramax,kartaview,commons").split(","))
            allv = R.select(recs, box, spacing_m=20.0, size=768)
            views = R.route_order(spread(allv, int(cfg["views"]), cfg["seed"]))
            R.write_views(out / "views.csv", views)
            print(f"{len(views)} of {len(allv)} views frozen", flush=True)
        views = R.read_views(out / "views.csv")
        RC.fetch_images(views, out, 4, False)
        for v in [v for v in views if v["projection"] == "equirect"]:
            try:
                RC.pano_image(v, out, False)
            except Exception as err:  # noqa: BLE001
                print(f"{v['id']}: panorama not fetched ({err})", flush=True)
        RC.write_attribution(views, out / "ATTRIBUTION.csv")
        if args.labels:
            labels = RC.Labels(out, args.model, args.device)
            for v in views:
                try:
                    RC.real_spec(v, out, labels, False)
                except Exception as err:  # noqa: BLE001
                    print(f"{v['id']}: not labelled ({err})", flush=True)
        # top-down tiles and their aerial photos
        tiles = tile_views(p, mdir, box, int(cfg["tiles"]), float(cfg["tile_m"]))
        top = out / "top"
        top.mkdir(exist_ok=True)
        R.write_views(top / "views.csv", tiles)
        (top / "tiles.json").write_text(json.dumps({t["id"]: t["_bbox"] for t in tiles}, indent=1))
        shutil.copy(out / "run.json", top / "run.json")
        c = p.get("country", "none")
        for t in tiles:
            for cc, sub in ((c, "aerial"), (aerial.PREVIOUS.get(c), "aerial_prev")):
                if cc and aerial.available(cc):
                    try:
                        aerial.ortho(cc, *t["_bbox"], 512, cache=top / sub / f"{t['id']}.jpg")
                    except Exception as err:  # noqa: BLE001
                        print(f"{t['id']}: {sub} not fetched ({err})", flush=True)
        if p.get("pairs", False) and not (out / "pairs_summary.json").exists():
            if any(v["projection"] == "equirect" for v in views):
                subprocess.run(CHECK + ["pairs", str(ROOT / p["experiment"]), "--out", str(out)], check=False)
        print(f"{p['name']}: {len(views)} views, {len(tiles)} tiles"
              + (f", aerial {c}" if aerial.available(c) else ", no open aerial photo"), flush=True)


def cmd_freeze_top(args):
    """A further frozen top-down set: the same squares (and aerial photos) as "top", the camera at another height
    with the field of view that still covers the square (lower: less sky-atmosphere haze, more building lean)."""
    cfg = load(args.pilot)
    for p in places(cfg, args.places):
        fz = Path(args.root) / "frozen" / p["name"]
        src = fz / "top"
        if not (src / "views.csv").exists():
            continue
        dst = fz / args.name
        dst.mkdir(exist_ok=True)
        vs = R.read_views(src / "views.csv")
        half = float(cfg["tile_m"]) / 2
        for v in vs:
            v["height_m"] = args.height
            v["hfov"] = round(math.degrees(2 * math.atan(half / args.height)), 3)
        R.write_views(dst / "views.csv", vs)
        shutil.copy(src / "run.json", dst / "run.json")
        print(f"{p['name']}: {len(vs)} squares at {args.height:.0f} m ({vs[0]['hfov'] if vs else '-'} deg) in {dst}")


def _round_dir(args, rnd, name):
    return Path(args.root) / "rounds" / rnd / name


def cmd_render(args):
    cfg = load(args.pilot)
    for p in places(cfg, args.places):
        if args.twins and not p.get("twin"):
            continue
        fz = Path(args.root) / "frozen" / p["name"]
        out = _round_dir(args, args.round, p["name"])
        (out / "top").mkdir(parents=True, exist_ok=True)
        for f in ("views.csv", "run.json", "pairs_summary.json", "ATTRIBUTION.csv"):
            if (fz / f).exists():
                shutil.copy(fz / f, out / f)
        for f in ("views.csv", "run.json"):
            shutil.copy(fz / "top" / f, out / "top" / f)
        extra_sets = [t for t in args.top_sets.split(",")[1:] if (fz / t / "views.csv").exists()]
        for t in extra_sets:
            (out / t).mkdir(parents=True, exist_ok=True)
            for f in ("views.csv", "run.json"):
                shutil.copy(fz / t / f, out / t / f)
        exp = str(ROOT / (p["twin"] if args.twins else p["experiment"]))
        if args.twins:      # the twin's box is smaller than the 0.9.16 map's: only the views and squares inside it
            import realism_check as RC
            _, tdir, tb = RC._map(exp)
            for f in [out / "views.csv", out / "top" / "views.csv"] + [out / t / "views.csv" for t in extra_sets]:
                vs = [v for v in R.read_views(f) if tb[0] <= v["lat"] <= tb[2] and tb[1] <= v["lon"] <= tb[3]]
                R.write_views(f, vs)
                print(f"{p['name']}: {len(vs)} views in the twin's box ({f.parent.name})", flush=True)
        print(f"== {p['name']}: street views", flush=True)
        subprocess.run(CHECK + ["render", exp, "--out", str(out), "--port", str(args.port),
                                "--budget-min", str(args.budget_min)] + args.extra, check=False)
        print(f"== {p['name']}: top-down tiles", flush=True)
        subprocess.run(CHECK + ["render", exp, "--out", str(out / "top"), "--port", str(args.port), "--yaw-margin", "0",
                                "--pitch-margin", "0", "--weather", "ClearNoon", "--moving-cars", "0",
                                "--first-settle", "80", "--settle", "20"] + args.extra, check=False)
        for t in extra_sets:
            print(f"== {p['name']}: top-down set {t}", flush=True)
            subprocess.run(CHECK + ["render", exp, "--out", str(out / t), "--port", str(args.port), "--yaw-margin", "0",
                                "--pitch-margin", "0", "--weather", "ClearNoon", "--moving-cars", "0",
                                "--first-settle", "80", "--settle", "20"] + args.extra, check=False)


def footprint_iou(mdir, render_row, sem):
    """IoU of the render's building pixels (CARLA's labels) with the map's building footprints (buildings.json:
    3DBAG / national data where measured, else OSM / Overture) rasterised in the same frame (north up, ground scale);
    also the IoU restricted to measured footprints. Tests where the generator put buildings, not how they look."""
    from PIL import Image, ImageDraw
    b = getattr(footprint_iou, "_cache", {}).get(str(mdir))
    if b is None:
        f = Path(mdir) / "buildings.json"
        b = json.loads(f.read_text()) if f.exists() else []
        footprint_iou._cache = {str(mdir): b}
    if not b:
        return {}
    W = sem.shape[1]
    cx, cy = float(render_row["x"]), float(render_row["y"])
    dz = float(render_row["cam_z"]) - float(render_row["ground_z"])
    half = dz * math.tan(math.radians(float(render_row["wide_hfov"])) / 2)
    k = W / (2 * half)
    out = {}
    for name, keep in (("bld_iou", lambda r: True), ("bld_iou_measured", lambda r: r.get("s") in MEASURED)):
        im = Image.new("L", (W, sem.shape[0]), 0)
        d = ImageDraw.Draw(im)
        n = 0
        for r in b:
            if not keep(r):
                continue
            pts = [((x - cx) * k + W / 2, (y - cy) * k + sem.shape[0] / 2) for x, y in r["p"]]
            if any(0 <= px < W and 0 <= py < sem.shape[0] for px, py in pts):
                d.polygon(pts, fill=1)
                n += 1
        if not n:
            continue
        fp = np.asarray(im) > 0
        bl = np.isin(sem, (3, 4))       # CARLA 0.9.14+ labels: building, wall
        union = (fp | bl).sum()
        out[name] = round(float((fp & bl).sum() / union), 4) if union else None
    return out


def _top_scores(p, fz, out, perc, sub="top"):
    from PIL import Image
    _, mdir, _ = _map(p)
    rinfo = {}
    if (out / sub / "renders.csv").exists():
        with (out / sub / "renders.csv").open(newline="") as f:
            rinfo = {r["id"]: r for r in csv.DictReader(f)}
    rows = []
    tiles = R.read_views(fz / sub / "views.csv")
    a_dir, prev_dir = fz / "top" / "aerial", fz / "top" / "aerial_prev"
    with_photo = [t["id"] for t in tiles if (a_dir / f"{t['id']}.jpg").exists()]
    ids = with_photo or [t["id"] for t in tiles]       # no open aerial photo: the footprint check only
    for k, tid in enumerate(ids):
        row = {"id": tid}
        rp = out / sub / "renders" / f"{tid}.png"
        if not with_photo:
            sp = out / sub / "renders" / f"{tid}_sem.png"
            if sp.exists() and tid in rinfo:
                row.update({f"render_{m}": v for m, v in footprint_iou(mdir, rinfo[tid],
                                                                         np.asarray(Image.open(sp))).items()})
            rows.append(row)
            continue
        real = np.asarray(Image.open(a_dir / f"{tid}.jpg").convert("RGB"))
        if rp.exists():
            row.update({f"render_{m}": v for m, v in aerial.topdown_metrics(real, np.asarray(
                Image.open(rp).convert("RGB")), perc).items()})
            sp = out / sub / "renders" / f"{tid}_sem.png"
            if sp.exists() and tid in rinfo:
                row.update({f"render_{m}": v for m, v in footprint_iou(mdir, rinfo[tid],
                                                                         np.asarray(Image.open(sp))).items()})
        other = ids[(k + 1) % len(ids)]       # another tile of the box: the "random place" baseline
        if other != tid:
            o = np.asarray(Image.open(a_dir / f"{other}.jpg").convert("RGB"))
            row.update({f"random_{m}": v for m, v in aerial.topdown_metrics(real, o, perc).items()})
        if (prev_dir / f"{tid}.jpg").exists():
            o = np.asarray(Image.open(prev_dir / f"{tid}.jpg").convert("RGB"))
            row.update({f"same_{m}": v for m, v in aerial.topdown_metrics(real, o, perc).items()})
        rows.append(row)
    if not rows:
        return {}
    with (out / sub / "topdown.csv").open("w", newline="") as f:
        keys = sorted({k for r in rows for k in r})
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)
    summ = {}
    for kind in ("render", "random", "same"):
        for m in TOP_KEYS:
            xs = [r[f"{kind}_{m}"] for r in rows if f"{kind}_{m}" in r]
            if xs:
                summ[f"{kind}_{m}"] = round(float(np.median(xs)), 4)
    summ["tiles"] = sum(1 for r in rows if "render_veg_iou" in r or "render_bld_iou" in r)
    for m in TOP_KEYS:      # index where an earlier photo of the same place exists, else the lift over random
        r_, lo, hi = summ.get(f"render_{m}"), summ.get(f"random_{m}"), summ.get(f"same_{m}")
        if r_ is not None and lo is not None:
            summ[f"lift_{m}"] = round(r_ - lo, 4)
            if hi is not None and abs(hi - lo) > 1e-6:
                summ[f"index_{m}"] = round((r_ - lo) / (hi - lo), 3)
    return summ


def cmd_score(args):
    cfg = load(args.pilot)
    ref_pairs = Path(args.root) / "frozen" / args.pairs_ref / "pairs_scale.json"
    perc = None
    if args.perceptual:
        from coupledcarla.streetphotos import Perceptual
        perc = Perceptual(args.dino_model, args.device)
    for p in places(cfg, args.places):
        fz = Path(args.root) / "frozen" / p["name"]
        out = _round_dir(args, args.round, p["name"])
        if not out.exists():
            print(f"{p['name']}: no renders in {out}", flush=True)
            continue
        own = fz / "pairs_summary.json"
        own_ok = own.exists() and min(json.loads(own.read_text()).get(k, {}).get("n", 0)
                                      for k in ("neighbour", "random")) >= MIN_PAIRS
        if own_ok:
            shutil.copy(own, out / "pairs_summary.json")
        elif ref_pairs.exists():      # too few panoramas here: Eindhoven's real-vs-real scale (100 pairs)
            shutil.copy(ref_pairs, out / "pairs_summary.json")
        if (out / "renders.csv").exists() and (args.again or not (out / "summary.json").exists()):
            cmd = CHECK + ["compare", "--out", str(out)]
            if not args.perceptual:
                cmd.append("--no-perceptual")
            subprocess.run(cmd + (["--device", args.device] if args.device else []), check=False)
            if not (out / "summary.json").exists() and args.device != "cpu":
                # the GPU is shared (another helper's CARLA server can take most of it): again on the CPU
                print(f"{p['name']}: compare failed on the GPU; again on the CPU", flush=True)
                subprocess.run(cmd + ["--device", "cpu"], check=False)
        score = {"place": p["name"], "role": p["role"], "round": args.round}
        if (out / "summary.json").exists():
            s = json.loads((out / "summary.json").read_text())
            allv = s["views"]["all"]
            score["street"] = {k: allv.get(k) for k in ("n",) + STREET_KEYS}
            score["street_index"] = s.get("realism_index", {})
            score["map_score"] = s["map"]["score"]
            score["differences"] = dict(list(s["map"].get("differences", {}).items())[:8])
            score["pairs_scale"] = "own" if own_ok else args.pairs_ref
        score["top"] = _top_scores(p, fz, out, perc)
        for sub in args.top_sets.split(",")[1:]:        # further top-down sets (e.g. top150: the same squares, lower)
            if (out / sub).exists():
                score[sub] = _top_scores(p, fz, out, perc, sub)
        (out / "score.json").write_text(json.dumps(score, indent=1))
        print(f"{p['name']}: street index {score.get('street_index', {}).get('spot_score')}, top veg IoU "
              f"{score['top'].get('render_veg_iou')} (random {score['top'].get('random_veg_iou')})", flush=True)


def _fmt(x, nd=2):
    return "–" if x is None else f"{x:.{nd}f}" if isinstance(x, (int, float)) else str(x)


def table_md(root, rounds, cfg):
    lines = []
    cols = ["place", "role", "views", "street index", "IoU idx", "edges idx", "DINO patch idx", "skyline °",
            "map score", "top tiles", "veg IoU (random, same)", "top DINO patch (random)", "top colour (random)",
            "footprint IoU (measured)"]
    for rnd in rounds:
        lines += [f"### Round {rnd}", "", "| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
        agg = {"train": [], "held_out": []}
        for p in cfg["places"]:
            f = Path(root) / "rounds" / rnd / p["name"] / "score.json"
            if not f.exists():
                lines.append(f"| {p['name']} | {p['role']} | not scored |" + " |" * (len(cols) - 3))
                continue
            s = json.loads(f.read_text())
            si, st, t = s.get("street_index", {}), s.get("street", {}), s.get("top", {})
            agg[p["role"]].append(si.get("spot_score"))
            lines.append("| " + " | ".join([
                p["name"], p["role"], _fmt(st.get("n"), 0), _fmt(si.get("spot_score")), _fmt(si.get("al_miou_coarse")),
                _fmt(si.get("boundary_f")), _fmt(si.get("dino_patch")), _fmt(st.get("al_skyline_mae_deg"), 1),
                _fmt(s.get("map_score"), 1), _fmt(t.get("tiles"), 0),
                f"{_fmt(t.get('render_veg_iou'))} ({_fmt(t.get('random_veg_iou'))}, {_fmt(t.get('same_veg_iou'))})",
                f"{_fmt(t.get('render_dino_patch'))} ({_fmt(t.get('random_dino_patch'))})",
                f"{_fmt(t.get('render_colour_hist_sim'))} ({_fmt(t.get('random_colour_hist_sim'))})",
                f"{_fmt(t.get('render_bld_iou'))} ({_fmt(t.get('render_bld_iou_measured'))})"]) + " |")
        for role, xs in agg.items():
            xs = [x for x in xs if x is not None]
            mean = _fmt(sum(xs) / len(xs)) if xs else "–"
            lines.append(f"| **mean {role}** | | | **{mean}** |" + " |" * (len(cols) - 4))
        lines.append("")
        lines += ["Most frequent differences per place (street level, mentions over the views):", ""]
        for p in cfg["places"]:
            f = Path(root) / "rounds" / rnd / p["name"] / "score.json"
            if f.exists():
                d = json.loads(f.read_text()).get("differences", {})
                lines.append(f"- {p['name']}: " + "; ".join(f"{k} {v}" for k, v in list(d.items())[:5]))
        lines.append("")
    return "\n".join(lines)


def cmd_table(args):
    cfg = load(args.pilot)
    md = table_md(args.root, args.rounds, cfg)
    print(md)
    (Path(args.root) / f"table_{'_'.join(args.rounds)}.md").write_text(md)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pilot", default=str(PILOT))
    ap.add_argument("--root", default="logs/pilot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("freeze")
    p.add_argument("--places", default=None)
    p.add_argument("--again", action="store_true", help="draw the views again (changes the evaluation!)")
    p.add_argument("--labels", action="store_true", help="segment the real photos now")
    p.add_argument("--model", default="nvidia/segformer-b2-finetuned-cityscapes-1024-1024")
    p.add_argument("--device", default=None)
    p = sub.add_parser("freeze-top", help="a further top-down set from the frozen squares at another height")
    p.add_argument("name")
    p.add_argument("height", type=float)
    p.add_argument("--places", default=None)
    p = sub.add_parser("render")
    p.add_argument("round")
    p.add_argument("--places", default=None)
    p.add_argument("--port", type=int, default=2500)
    p.add_argument("--budget-min", type=float, default=30.0)
    p.add_argument("--top-sets", default="top", help="top-down sets to render, e.g. top,top150")
    p.add_argument("--twins", action="store_true", help="the places' CARLA 1.0 twins (map 'twin'), not the 0.9.16 maps")
    p.add_argument("--render-args", default="", help="more realism_check render options, e.g. '--profile Dashcam'")
    p = sub.add_parser("score")
    p.add_argument("round")
    p.add_argument("--places", default=None)
    p.add_argument("--pairs-ref", default="ehv_centre", help="real-vs-real scale for places without panoramas")
    p.add_argument("--top-sets", default="top,top150", help="top-down sets to score (the first is score['top'])")
    p.add_argument("--no-perceptual", dest="perceptual", action="store_false")
    p.add_argument("--dino-model", default="facebook/dinov2-base")
    p.add_argument("--device", default=None)
    p.add_argument("--again", action="store_true")
    p = sub.add_parser("table")
    p.add_argument("rounds", nargs="+")
    args = ap.parse_args()
    args.extra = getattr(args, "render_args", "").split()
    {"freeze": cmd_freeze, "freeze-top": cmd_freeze_top, "render": cmd_render, "score": cmd_score,
     "table": cmd_table}[args.cmd](args)


if __name__ == "__main__":
    main()
