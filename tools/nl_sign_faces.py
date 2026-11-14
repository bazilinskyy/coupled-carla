"""Dutch traffic sign faces for CARLA 1.0: the RVV 1990 designs from Wikimedia Commons as PNG textures.

    python tools/nl_sign_faces.py [--cache DIR] [--size 512] [--rsvg rsvg-convert]
    python tools/nl_sign_faces.py --plates-from sections.json     (tools/ue_nl_signs.py --probe's output)

Wikimedia Commons has SVG diagrams of all Dutch traffic signs (category "Diagrams of road signs of the
Netherlands"), most named "Nederlands verkeersbord <code>[ variant].svg". The category tree misses many of them,
so the files are listed by name prefix (Commons API list=allimages, keyless, realworld.UA). Each file name is
parsed into an RVV code and a variant, and one file is picked per face key:

    B6, D2, C4-R, ...           the sign itself (C4 and L19 come as -L and -R, left and right)
    A1-50, A1-130               a sign with its number (speed)
    A1-30-ZB, A2-30-ZE, E1-ZB   a zone board: begin (ZB) or end (ZE) of a zone, with its number where it has one

Newer or other designs ("2023" files, "mb"/"zb" variants with an extra plate, historic sets, Frisian, school
variants) are left out: the classic RVV 1990 designs are what stands in the streets. Speed signs Commons lacks
(A1 15, 20, 30, ...; zone boards for other speeds) are made from the Commons file whose number is SVG text (A1-50,
A1-60-ZB, A2-60-ZE) by changing that number ("derived" in the manifest; same licence). The licence and author of
each file come from the Commons API (extmetadata) and are kept in the manifest; Dutch statutory sign designs are
not protected by copyright (Auteurswet art. 11) and nearly all files are tagged public domain or CC0. A file
whose licence is neither these nor CC BY / CC BY-SA is left out; for the CC BY-SA ones the author is recorded.

Each SVG is rendered with rsvg-convert (librsvg) and mapped onto the front face of a CARLA 1.0 blank plate of its
outline (round, triangle, inverted triangle, diamond, octagon, or the rectangle whose front is closest to the
drawing's width/height): the plate's front-face UV map (an affine map of its x/z, tools/nl_sign_plates.json,
measured with tools/ue_nl_signs.py --probe) says where each texel lands on the plate, the drawing's opaque box is
stretched to the front face's box, and texels off the front take the nearest drawing colour (no dark fringes in
the mip maps). The textures are square RGB PNGs (size x size), in ~/.cache/coupled-carla/nl_signs/textures/ (not
in git); coupledcarla/data/nl_sign_faces.json (face key -> file, licence, author, plate, mesh) is in git.
tools/ue_nl_signs.py imports them into the CARLA 1.0 editor as one texture, material instance and plate mesh each.
"""
import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import requests

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from coupledcarla.realworld import UA  # noqa: E402

API = "https://commons.wikimedia.org/w/api.php"
PREFIXES = ["Nederlands verkeersbord", "Nederlands Verkeersbord", "Nederlandse verkeersbord", "NL verkeersbord",
            "Netherlands traffic sign"]
CACHE = Path.home() / ".cache" / "coupled-carla" / "nl_signs"
MANIFEST = ROOT / "coupledcarla" / "data" / "nl_sign_faces.json"
PLATES_FILE = ROOT / "tools" / "nl_sign_plates.json"
FOLDER = "/Game/CoupledCarla/Signs/NL"
TITLE = re.compile(r"^File:(Nederlandse?[ -][Vv]erkeersbord|NL[ -][Vv]erkeersbord|Netherlands traffic sign)[ -]+"
                   r"(?:NL )?(OB|BB|BW|[A-L])0*(\d+)([a-z]?\d*)(.*)\.svg$")
FREE = ("public domain", "cc0")
OPEN = ("cc by",)
# speed signs made from a Commons file whose number is SVG text: key template -> (source key, its number)
DERIVE = {"A1-{}": ("A1-50", "50"), "A1-{}-ZB": ("A1-60-ZB", "60"), "A2-{}-ZE": ("A2-60-ZE", "60")}
SPEEDS = {"A1-{}": [15, 20, 30, 40, 60, 70, 80, 90, 100, 120], "A1-{}-ZB": [15, 20, 50],
          "A2-{}-ZE": [15, 20, 50]}


# --- which files -------------------------------------------------------------------------------------------------

def variant(rest):
    """The face-key suffix for the part of a file name after the code, or None for a design we leave out."""
    r = rest.strip()
    if r == "":
        return ""
    m = re.fullmatch(r"-?(\d{1,3})", r)
    if m:
        return f"-{int(m.group(1))}" if r.startswith("-") or len(m.group(1)) >= 2 and len(m.group(1)) != 4 else None
    m = re.fullmatch(r"-?(\d{1,3})[ -](Zone|ZB|Eindzone|ZE)", r)
    if m:
        return f"-{int(m.group(1))}-{'ZB' if m.group(2) in ('Zone', 'ZB') else 'ZE'}"
    if r in ("Zone", "Eindzone"):
        return "-ZB" if r == "Zone" else "-ZE"
    if r in ("(links)", "(rechts)"):
        return "-L" if r == "(links)" else "-R"
    if r == "zonder onderbord":         # J37 is drawn with its sub-plate; this file is the sign alone
        return ""
    return None


def parse_title(title):
    """'File:Nederlands verkeersbord A1 30 Zone.svg' -> ('A1-30-ZB', 'A1', prefix rank); None for other files and
    for designs left out (2023, mb, zb, historic, ...)."""
    m = TITLE.match(title)
    if not m:
        return None
    v = variant(m.group(5))
    if v is None:
        return None
    code = f"{m.group(2)}{int(m.group(3))}{m.group(4)}"
    rank = 0 if m.group(1).lower().startswith("nederlands verkeersbord") else 1
    if "onderbord" in m.group(5):
        rank = -1
    return code + v, code, rank


def list_titles(session, log=print):
    out = set()
    for pre in PREFIXES:
        cont = {}
        while True:
            r = session.get(API, params={"action": "query", "list": "allimages", "aiprefix": pre, "ailimit": 500,
                                         "format": "json", "formatversion": 2, **cont}, timeout=60)
            r.raise_for_status()
            d = r.json()
            out.update(m["title"] for m in d["query"]["allimages"])
            if "continue" not in d:
                break
            cont = {"aicontinue": d["continue"]["aicontinue"]}
    log(f"Commons: {len(out)} files with a Dutch sign prefix")
    return sorted(out)


def pick(titles):
    """{face key: (title, code)}: per key the file of the main prefix ("Nederlands verkeersbord"), else the
    shortest name."""
    best = {}
    for t in titles:
        p = parse_title(t)
        if p is None:
            continue
        key, code, rank = p
        cur = best.get(key)
        if cur is None or (rank, len(t), t) < cur[0]:
            best[key] = ((rank, len(t), t), code)
    return {k: (v[0][2], v[1]) for k, v in best.items()}


def _strip(html):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", "", html or "")).strip()


def file_info(session, titles):
    """{title: {url, page, sha1, licence, licence_url, author}} from the Commons API (50 titles per request)."""
    out = {}
    for i in range(0, len(titles), 50):
        r = session.get(API, params={"action": "query", "prop": "imageinfo", "titles": "|".join(titles[i:i + 50]),
                                     "iiprop": "url|extmetadata|sha1", "format": "json", "formatversion": 2},
                        timeout=60)
        r.raise_for_status()
        for p in r.json()["query"]["pages"]:
            ii = (p.get("imageinfo") or [{}])[0]
            em = ii.get("extmetadata") or {}
            out[p["title"]] = {
                "url": ii.get("url"), "page": ii.get("descriptionurl"), "sha1": ii.get("sha1"),
                "licence": _strip(em.get("LicenseShortName", {}).get("value")),
                "licence_url": em.get("LicenseUrl", {}).get("value"),
                "author": _strip(em.get("Artist", {}).get("value"))[:200],
            }
        time.sleep(0.5)
    return out


def licence_ok(lic):
    lic = (lic or "").lower()
    return any(f in lic for f in FREE) or any(lic.startswith(o) for o in OPEN)


def download(session, url, tries=6):
    """The file's bytes; upload.wikimedia.org answers 429 to fast clients: wait (Retry-After, else doubling from
    10 s) and try again. One file every 2 s stays under its limit."""
    wait = 10.0
    for i in range(tries):
        r = session.get(url, timeout=60)
        if r.status_code == 429 and i < tries - 1:
            time.sleep(float(r.headers.get("Retry-After") or wait))
            wait *= 2
            continue
        r.raise_for_status()
        time.sleep(2.0)
        return r.content
    raise RuntimeError("unreachable")


def derive_svg(src, old, new):
    """The SVG with its number (a <tspan> whose text is `old`) changed to `new`; three digits in a smaller font."""
    m = re.search(r'(<tspan[^>]*font-size=")(\d+)("[^>]*>)' + re.escape(old) + r"(</tspan>)", src)
    if not m:
        return None
    size = int(m.group(2))
    if len(new) > len(old):
        size = round(size * len(old) / len(new) * 1.15)
    return src[:m.start()] + f"{m.group(1)}{size}{m.group(3)}{new}{m.group(4)}" + src[m.end():]


# --- the plates --------------------------------------------------------------------------------------------------

# blank plates of CARLA 1.0 (/Game/Carla/Static/TrafficSign/Signs) by outline; every SM_HorRect_*/SM_VertRect_*
# (rectangles of other proportions) is a "rect" too
PLATE_SHAPES = {"SM_CircleShape": "round", "SM_DangerSignShape": "triangle",
                "SM_InvertedTriangleShape": "inv_triangle", "SM_RomboidShape": "diamond",
                "SM_OctogonalShape": "octagon", "SM_SquareShape": "rect", "SM_SlightRectangleShape": "rect",
                "SM_LargeSlightRectangleShape": "rect", "SM_VerticalRectangle": "rect", "SM_SmallRectangle": "rect"}
SIGNS_DIR = "/Game/Carla/Static/TrafficSign/Signs"


def plates_from_sections(sections):
    """tools/nl_sign_plates.json from ue_nl_signs.py --probe's per-polygon dump: for each plate its front face
    (material slot 0's triangles facing +y, the side scenery.py turns to the traffic) as triangles of (x, z, u, v)
    (cm, UV0), its x/z box, and the mesh's bounding box for the catalogue. "flat": the front's UVs follow one
    affine map of x/z (most plates; SM_SquareShape's do not, which the triangles handle as well)."""
    import numpy as np
    out = {}
    for name, m in sorted(sections.items()):
        shape = PLATE_SHAPES.get(name) or ("rect" if re.match(r"SM_(Hor|Vert)Rect_\d+$", name) else None)
        if shape is None or "tris" not in m or name == "SM_HorRect_13":     # 13: SM_SquareShape again
            continue
        front, allp = [], []
        for g, tri in m["tris"]:
            P = np.array(tri, dtype=float)
            allp.append(P)
            if g != 0 or len(P) != 3:
                continue
            n = np.cross(P[1, :3] - P[0, :3], P[2, :3] - P[0, :3])
            if np.linalg.norm(n) < 1e-9:
                continue
            # Unreal winds front faces clockwise: this cross product points into the mesh, -y for a +y face
            if n[1] / np.linalg.norm(n) < -0.9:
                front.append(P)
        if not front:
            continue
        P = np.concatenate(front)
        A = np.c_[P[:, 0], P[:, 2], np.ones(len(P))]
        res = max(float(np.abs(A @ np.linalg.lstsq(A, P[:, k], rcond=None)[0] - P[:, k]).max()) for k in (3, 4))
        Q = np.concatenate(allp)
        lo, hi = Q[:, :3].min(0) / 100.0, Q[:, :3].max(0) / 100.0
        x0, x1, z0, z1 = P[:, 0].min(), P[:, 0].max(), P[:, 2].min(), P[:, 2].max()
        out[name] = {"shape": shape, "path": f"{SIGNS_DIR}/{name}.{name}",
                     "front": [round(float(v), 3) for v in (x0, x1, z0, z1)],
                     "aspect": round(float((x1 - x0) / (z1 - z0)), 4), "flat": res < 0.01,
                     "tris": [[[round(float(t[0]), 3), round(float(t[2]), 3), round(float(t[3]), 5),
                                round(float(t[4]), 5)] for t in T] for T in front],
                     "ext": [round(float(v), 3) for v in (hi - lo) / 2],
                     "off": [round(float(v), 3) for v in (hi + lo) / 2]}
    return out


def plates():
    return json.loads(PLATES_FILE.read_text())


def outline(aspect, fill, top, bottom, left=0.0, right=0.0):
    """The plate outline for a drawing, from its opaque box: width/height, the share of the box it fills (a circle
    0.79, an octagon 0.83, a triangle or diamond 0.5, a rectangle ~1) and how much of the box's width its top and
    bottom rows fill (a triangle's apex is narrow), and its left and right columns of the height (a diamond's
    corners). Anything else is a rectangle."""
    if fill > 0.9:
        return "rect"
    square = abs(aspect - 1) <= 0.08
    if square and 0.72 < fill < 0.81:
        return "round"
    if square and 0.81 <= fill <= 0.9:
        return "octagon"
    if 0.42 < fill < 0.62 and 0.9 < aspect < 1.35:
        if top < 0.3 and bottom < 0.3 and left < 0.3 and right < 0.3:
            return "diamond"
        if top < 0.3 < 0.7 < bottom:
            return "triangle"
        if bottom < 0.3 < 0.7 < top:
            return "inv_triangle"
    return "rect"


def choose_plate(aspect, fill, top, bottom, left, right, table):
    """(plate name, plate) for a drawing: the plate of its outline; for rectangles the one whose front is closest
    to the drawing's width/height (log ratio)."""
    shape = outline(aspect, fill, top, bottom, left, right)
    cands = [(n, p) for n, p in table.items() if p["shape"] == shape] or \
            [(n, p) for n, p in table.items() if p["shape"] == "rect"]
    return min(cands, key=lambda np_: (abs(math.log(np_[1]["aspect"] / aspect)), np_[0]))


# --- rendering ---------------------------------------------------------------------------------------------------

def render(svg, png, size, rsvg):
    """SVG -> PNG of `size` px on its long side (aspect kept)."""
    subprocess.run([rsvg, "--keep-aspect-ratio", "-w", str(size), "-h", str(size), "-f", "png", "-o", str(png),
                    str(svg)], check=True, capture_output=True)


def alpha_box(png):
    """(aspect, fill, top, bottom, left, right) of the drawing: width / height of its opaque box, the opaque share
    of that box, the opaque share of the box's width in its top and bottom 5 % rows and of its height in its left
    and right 5 % columns."""
    import numpy as np
    from PIL import Image
    a = np.asarray(Image.open(png).convert("RGBA"))[..., 3] > 128
    ys, xs = np.nonzero(a)
    if len(xs) == 0:
        return 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
    box = a[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    h, w = box.shape
    k, j = max(1, round(h * 0.05)), max(1, round(w * 0.05))
    return (float(w / h), float(box.mean()), float(box[:k].any(0).mean()), float(box[-k:].any(0).mean()),
            float(box[:, :j].any(1).mean()), float(box[:, -j:].any(1).mean()))


def fill_transparent(rgba, ring=16):
    """RGB where every transparent pixel takes the colour of the nearest opaque ones: grown out from the drawing
    `ring` pixels (each step the mean of the filled 4-neighbours), the rest from a pull-push pyramid. A texel just
    off the drawing (UV seams, mip maps, the plate's rim) then has the edge colour, not black."""
    import numpy as np
    a = rgba[..., 3].astype(np.float64) / 255.0
    c = rgba[..., :3].astype(np.float64) / 255.0
    have = a > 0.5
    col = np.where(have[..., None], c, 0.0)
    for _ in range(ring):
        if have.all():
            break
        acc, n = np.zeros_like(col), np.zeros(have.shape)
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            h = np.zeros_like(have)
            v = np.zeros_like(col)
            ys = slice(max(dy, 0), have.shape[0] + min(dy, 0))
            yd = slice(max(-dy, 0), have.shape[0] + min(-dy, 0))
            xs = slice(max(dx, 0), have.shape[1] + min(dx, 0))
            xd = slice(max(-dx, 0), have.shape[1] + min(-dx, 0))
            h[yd, xd], v[yd, xd] = have[ys, xs], col[ys, xs]
            acc += v * h[..., None]
            n += h
        new = ~have & (n > 0)
        col[new] = acc[new] / n[new][:, None]
        have = have | new
    if not have.all():          # pull-push for what the ring did not reach
        w = have[..., None].astype(np.float64)
        levels = [(col * w, w)]
        while min(levels[-1][1].shape[:2]) > 1:
            cc, ww = levels[-1]
            hh, wd = cc.shape[0] // 2 * 2, cc.shape[1] // 2 * 2
            levels.append(tuple((x[0:hh:2, 0:wd:2] + x[1:hh:2, 0:wd:2] + x[0:hh:2, 1:wd:2] + x[1:hh:2, 1:wd:2]) / 4
                                for x in (cc, ww)))
        cc, ww = levels[-1]
        fill = cc / np.maximum(ww, 1e-9)
        for cc, ww in reversed(levels[:-1]):
            up = np.repeat(np.repeat(fill, 2, 0), 2, 1)
            up = np.pad(up, ((0, cc.shape[0] - up.shape[0]), (0, cc.shape[1] - up.shape[1]), (0, 0)), mode="edge")
            fill = cc + up * (1 - ww)
        col = np.where(have[..., None], col, fill)
    out = c * a[..., None] + col * (1 - a[..., None])
    return np.clip(out * 255 + 0.5, 0, 255).astype(np.uint8)


def front_points(plate, size):
    """(x, z, covered) per texel of a size x size texture: the point of the plate's front each texel lands on, by
    rasterising the front's triangles in UV space (barycentric); covered False where no front triangle is."""
    import numpy as np
    X, Z = np.zeros((size, size)), np.zeros((size, size))
    cov = np.zeros((size, size), dtype=bool)
    for tri in plate["tris"]:
        T = np.array(tri)
        uv = T[:, 2:] * size - 0.5                   # texel centres at integer coordinates
        j0, i0 = np.floor(uv.min(0)).astype(int)
        j1, i1 = np.ceil(uv.max(0)).astype(int)
        j0, i0, j1, i1 = max(j0, 0), max(i0, 0), min(j1, size - 1), min(i1, size - 1)
        if j1 < j0 or i1 < i0:
            continue
        jj, ii = np.meshgrid(np.arange(j0, j1 + 1), np.arange(i0, i1 + 1))
        (ua, va), (ub, vb), (uc, vc) = uv
        den = (vb - vc) * (ua - uc) + (uc - ub) * (va - vc)
        if abs(den) < 1e-12:
            continue
        la = ((vb - vc) * (jj - uc) + (uc - ub) * (ii - vc)) / den
        lb = ((vc - va) * (jj - uc) + (ua - uc) * (ii - vc)) / den
        lc = 1 - la - lb
        inside = (la >= -1e-3) & (lb >= -1e-3) & (lc >= -1e-3)
        X[ii[inside], jj[inside]] = (la * T[0, 0] + lb * T[1, 0] + lc * T[2, 0])[inside]
        Z[ii[inside], jj[inside]] = (la * T[0, 1] + lb * T[1, 1] + lc * T[2, 1])[inside]
        cov[ii[inside], jj[inside]] = True
    return X, Z, cov


def compose(png, out, plate, size):
    """The texture of a face: each texel that lands on the plate's front shows the drawing there (its opaque box
    stretched to the front's box, bilinear); the other texels (the rim, UV gaps) take the nearest front colour."""
    import numpy as np
    from PIL import Image
    im = np.asarray(Image.open(png).convert("RGBA"))
    ys, xs = np.nonzero(im[..., 3] > 8)
    im = im[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    rgb = fill_transparent(im).astype(np.float64)
    H, W = rgb.shape[:2]
    X, Z, cov = front_points(plate, size)
    x0, x1, z0, z1 = plate["front"]
    sx = np.clip((X - x0) / (x1 - x0) * W - 0.5, 0, W - 1)    # drawing pixels: left to right, top to bottom
    sy = np.clip((z1 - Z) / (z1 - z0) * H - 0.5, 0, H - 1)
    i0, j0 = np.floor(sy).astype(int), np.floor(sx).astype(int)
    i1, j1 = np.minimum(i0 + 1, H - 1), np.minimum(j0 + 1, W - 1)
    fy, fx = (sy - i0)[..., None], (sx - j0)[..., None]
    img = (rgb[i0, j0] * (1 - fx) * (1 - fy) + rgb[i0, j1] * fx * (1 - fy) + rgb[i1, j0] * (1 - fx) * fy
           + rgb[i1, j1] * fx * fy)
    rgba = np.concatenate([np.clip(img + 0.5, 0, 255), cov[..., None] * 255.0], axis=2).astype(np.uint8)
    Image.fromarray(fill_transparent(rgba), "RGB").save(out, optimize=True)


def width_m(shape, aspect):
    """The plate width (m) a face is placed at: ndwsigns.WIDTH for its outline; a rectangle as wide as a 0.6 x 0.6
    m board of the same area (0.36 m2), between 0.4 and 1.2 m."""
    from coupledcarla.ndwsigns import WIDTH
    if shape == "rect":
        return round(min(max(math.sqrt(0.36 * aspect), 0.4), 1.2), 2)
    return WIDTH[shape]


CATALOGUE = ROOT / "coupledcarla" / "data" / "meshes_1.0.json"


def update_catalogue(faces, path=CATALOGUE):
    """Put each face's mesh into the CARLA 1.0 mesh catalogue with its plate's measured size (the same geometry:
    the mesh is a duplicate of the plate); faces no longer made are taken out."""
    d = json.loads(path.read_text())
    meshes = {k: v for k, v in d["meshes"].items() if not k.startswith(FOLDER + "/")}
    by_name = {k.rsplit(".", 1)[-1]: v for k, v in meshes.items()}
    n = 0
    for f in faces.values():
        base = by_name.get(f["plate"])
        if base is None:
            continue
        meshes[f"{FOLDER}/Meshes/{f['mesh']}.{f['mesh']}"] = dict(base)
        n += 1
    d["meshes"] = meshes
    path.write_text(json.dumps(d, indent=0, sort_keys=True))     # as tools/mesh_catalog.py writes it
    return n


def asset_name(key):
    return key.replace("-", "_")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cache", type=Path, default=CACHE)
    ap.add_argument("--size", type=int, default=512, help="texture size (px, square, power of two)")
    ap.add_argument("--rsvg", default=shutil.which("rsvg-convert") or "rsvg-convert")
    ap.add_argument("--manifest", type=Path, default=MANIFEST)
    ap.add_argument("--plates-from", type=Path, help="write tools/nl_sign_plates.json from a --probe dump, then stop")
    args = ap.parse_args(argv)
    if args.plates_from:
        table = plates_from_sections(json.loads(args.plates_from.read_text()))
        PLATES_FILE.write_text(json.dumps(table, sort_keys=True, separators=(",", ":")))
        print(f"{len(table)} plates -> {PLATES_FILE}")
        return
    svgdir, rawdir, texdir = args.cache / "svg", args.cache / "raw", args.cache / "textures"
    for d in (svgdir, rawdir, texdir):
        d.mkdir(parents=True, exist_ok=True)
    s = requests.Session()
    s.headers.update(UA)
    chosen = pick(list_titles(s))
    info = file_info(s, sorted(t for t, _ in chosen.values()))
    table = plates()
    faces, dropped, sources = {}, {}, {}
    for key in sorted(chosen):
        title, code = chosen[key]
        fi = info.get(title) or {}
        if not fi.get("url"):
            dropped[key] = "no url"
            continue
        if not licence_ok(fi["licence"]):
            dropped[key] = f"licence {fi['licence']!r}"
            continue
        svg, meta = svgdir / f"{key}.svg", svgdir / f"{key}.json"
        if not svg.exists() or not meta.exists() or json.loads(meta.read_text()).get("sha1") != fi.get("sha1"):
            svg.write_bytes(download(s, fi["url"]))
            meta.write_text(json.dumps({"sha1": fi.get("sha1")}))
        sources[key] = (svg, code, {"file": title[5:], "page": fi["page"], "licence": fi["licence"],
                                    "licence_url": fi["licence_url"], "author": fi["author"], "sha1": fi["sha1"]})
    for tmpl, (src, old) in DERIVE.items():         # speeds Commons lacks, from a file with the number as text
        if src not in sources:
            continue
        text = sources[src][0].read_text()
        for v in SPEEDS[tmpl]:
            key = tmpl.format(v)
            if key in sources:
                continue
            new = derive_svg(text, old, str(v))
            if new is None:
                dropped[key] = f"{src} has no number {old} as text"
                continue
            p = svgdir / f"{key}.svg"
            p.write_text(new)
            sources[key] = (p, sources[src][1], dict(sources[src][2], derived=f"{src} with {v} for {old}"))
    for key, (svg, code, meta) in sorted(sources.items()):
        raw = rawdir / f"{key}.png"
        try:
            render(svg, raw, args.size, args.rsvg)
        except subprocess.CalledProcessError as err:
            dropped[key] = f"render failed: {err.stderr.decode(errors='replace')[:120]}"
            continue
        aspect, fill, *edges = alpha_box(raw)
        pname, plate = choose_plate(aspect, fill, *edges, table)
        if plate["shape"] == "rect" and fill < 0.9:     # a cut-out drawing (beacons J12/J13, finger posts K4/K6/K7)
            dropped[key] = f"not a plate: fills {fill:.2f} of its box"
            continue
        name = asset_name(key)
        compose(raw, texdir / f"T_NL_{name}.png", plate, args.size)
        faces[key] = dict(meta, code=code, plate=pname, shape=plate["shape"], aspect=round(aspect, 3),
                          fill=round(fill, 3), width_m=width_m(plate["shape"], plate["aspect"]),
                          texture=f"T_NL_{name}", material=f"MI_NL_{name}", mesh=f"SM_NL_{name}")
    for old in texdir.glob("T_NL_*.png"):          # textures of faces no longer made
        if old.stem not in {f["texture"] for f in faces.values()}:
            old.unlink()
    lic = {}
    for f in faces.values():
        lic[f["licence"]] = lic.get(f["licence"], 0) + 1
    args.manifest.write_text(json.dumps({
        "version": 1, "fetched": time.strftime("%Y-%m-%d"), "source": "Wikimedia Commons (keyless API)",
        "renderer": subprocess.run([args.rsvg, "--version"], capture_output=True, text=True).stdout.split("\n")[0],
        "size": args.size, "folder": FOLDER, "licences": lic, "faces": faces, "dropped": dropped},
        indent=1, sort_keys=True))
    print(f"catalogue: {update_catalogue(faces)} face meshes in {CATALOGUE.name}")
    print(f"{len(faces)} faces ({sum('derived' in f for f in faces.values())} derived), {len(dropped)} left out; "
          f"licences: {lic}")
    for k, why in sorted(dropped.items()):
        print(f"  left out {k}: {why}")


if __name__ == "__main__":
    main()
