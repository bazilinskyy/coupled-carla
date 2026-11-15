"""Measure the static car meshes coupledcarla.parking places, on a running CARLA, into data/parked_cars.json.

    python tools/parked_catalog.py [--content $CARLA_ROOT/Unreal/CarlaUnreal/Content] [--port 2000]
    python tools/parked_catalog.py --paints-only      # the paint colours alone, no server (CARLA 1.0 source tree)

Spawns each car mesh of parking.CARS_09 / CARS_10 (the server's version picks the table) once as static.prop.mesh in
an empty OpenDRIVE world and records its bounding box relative to the actor (pivot offset and half extents, metres),
as tools/mesh_catalog.py does for the scenery. Meshes whose box is empty or infinite are left out.

CARLA 1.0 keeps the windows of most parked cars in a separate mesh beside the body (SM_LincolnMKZ_Parked and
SM_LincolnMKZ_Glass_Parked, same pivot): a body without it has holes for windows. The glass mesh in the same Parked
folder whose name is the body's plus "glass" is measured too and recorded as the body's "glass" when its box lies
inside the body's.

Paint: a static mesh keeps the paint it was made with, and parking.Chooser prefers the meshes whose paint is common
in the vehicle register. On CARLA 1.0 (an uncooked source tree, whose .uasset files keep tagged properties) the
colour is read from the mesh's car-paint material instance: its "Base Color" vector parameter (linear RGB), else the
average of the source PNG of its base-colour texture, else a colour word in the texture's name (MatteGrey). The sRGB
colour is named with fleet.COLORS' words by hue, saturation and value (paint_name). This reads the .uasset files
heuristically (the name table and the VectorParameterValues array); it needs no editor.
"""
import argparse
import colorsys
import json
import math
import os
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from coupledcarla import parking  # noqa: E402

CONTENT = {"0.9": "CarlaUE4/Content", "1.0": "Unreal/CarlaUnreal/Content"}


# --- reading .uasset files (uncooked, tagged properties) -------------------------------------------------------

def name_table(b):
    """The package's name map: the longest run of FStrings (int32 length, ASCII, NUL) each followed by a 4-byte
    hash, as UE5 writes it."""
    def fstring(i):
        if i + 4 > len(b):
            return None
        n = struct.unpack_from("<i", b, i)[0]
        if not (1 <= n < 300 and i + 4 + n <= len(b) and b[i + 3 + n] == 0):
            return None
        s = b[i + 4:i + 3 + n]
        return s.decode() if all(32 <= c < 127 for c in s) else None

    best, i, seen = [], 0, set()
    while i < len(b) - 8:
        if i in seen or fstring(i) is None:
            i += 1
            continue
        run, j = [], i
        while True:
            s = fstring(j)
            if s is None:
                break
            seen.add(j)
            run.append(s)
            j += 4 + len(s) + 1 + 4
        if len(run) > len(best):
            best = run
        i = j if len(run) > 1 else i + 1
    return best


STRUCTURAL = {"None", "Name", "NameProperty", "StructProperty", "ArrayProperty", "ByteProperty", "IntProperty",
              "ParameterInfo", "MaterialParameterInfo", "Association", "EMaterialParameterAssociation",
              "GlobalParameter", "Index", "ParameterValue", "LinearColor", "ExpressionGUID", "Guid",
              "VectorParameterValue", "VectorParameterValues", "ObjectProperty", "FloatProperty", "BoolProperty",
              "SoftObjectPath", "SoftObjectProperty"}
AFTER_VECTORS = ("ScalarParameterValues", "TextureParameterValues", "BasePropertyOverrides", "Parent",
                 "StaticParametersRuntime", "EditorOnlyData", "CachedReferencedTextures", "PhysMaterial",
                 "bHasStaticPermutationResource", "LightingGuid", "ReferencedTextureGuids",
                 "ScalarParameterAtlasInstanceData")


def vector_params(path):
    """({parameter: (r, g, b, a) linear}, name table) of a material instance's VectorParameterValues. Each element
    is ParameterInfo (whose Name is the parameter's FName) then ParameterValue (a LinearColor, four floats)."""
    b = Path(path).read_bytes()
    nm = name_table(b)
    idx = {s: k for k, s in enumerate(nm)}
    out = {}
    if "VectorParameterValues" not in idx or "ParameterValue" not in idx:
        return out, nm

    def ref(s):
        return struct.pack("<ii", idx[s], 0)
    a0 = b.find(ref("VectorParameterValues"))
    if a0 < 0:
        return out, nm
    ends = [e for e in (b.find(ref(s), a0 + 8) for s in AFTER_VECTORS if s in idx) if e > a0]
    a1 = min(ends) if ends else len(b)
    good = {k for k, s in enumerate(nm) if s not in STRUCTURAL and not s.startswith("/")}
    pv, p = ref("ParameterValue"), a0
    while True:
        q = b.find(pv, p + 1, a1)
        if q < 0:
            break
        pname, k = None, q - 8
        while k > p:        # the parameter's name: the last reference to a non-structural name before the value
            i, number = struct.unpack_from("<ii", b, k)
            if number == 0 and i in good:
                pname = nm[i]
                break
            k -= 1
        for k in range(q + 8, q + 90):      # the four floats after the value's property tag
            v = struct.unpack_from("<4f", b, k)
            if all(math.isfinite(x) and 0.0 <= x <= 64.0 for x in v) and 0.5 <= v[3] <= 1.0001 \
                    and all(x == 0 or abs(x) > 1e-6 for x in v):
                if pname:
                    out[pname] = v
                break
        p = q
    return out, nm


def png_average(path):
    """Median sRGB colour of the source PNG a texture .uasset carries (editor-only source art), or None."""
    b = Path(path).read_bytes()
    i = b.find(b"\x89PNG\r\n\x1a\n")
    j = b.find(b"IEND", i)
    if i < 0 or j < 0:
        return None
    try:
        import io
        from PIL import Image
        im = Image.open(io.BytesIO(b[i:j + 8])).convert("RGB").resize((64, 64))
    except Exception:  # noqa: BLE001 - no PIL, or not a readable PNG
        return None
    raw = im.tobytes()
    px = [raw[i:i + 3] for i in range(0, len(raw), 3) if max(raw[i:i + 3]) > 6]
    if not px:          # (pure black is the UV padding between the islands, not paint)
        return None
    return tuple(sorted(p[c] for p in px)[len(px) // 2] for c in range(3))


def srgb(lin):
    return tuple(round(255 * (12.92 * x if x <= 0.0031308 else 1.055 * min(1.0, x) ** (1 / 2.4) - 0.055))
                 for x in lin[:3])


def paint_name(rgb):
    """A fleet.COLORS word for an sRGB car paint colour."""
    h, s, v = colorsys.rgb_to_hsv(*(c / 255.0 for c in rgb))
    h *= 360.0
    if v < 0.18:
        return "black"
    if s < 0.2:
        return "grey" if v < 0.6 else "silver" if v < 0.82 else "white"
    if h < 15 or h >= 345:
        return "maroon" if v < 0.5 else "red"
    if h < 40:
        return "brown" if v < 0.5 else "orange" if s > 0.6 else "beige"
    if h < 70:
        return "gold" if v < 0.75 else "yellow"
    if h < 170:
        return "green"
    if h < 260:
        return "blue"
    if h < 320:
        return "purple"
    return "pink"


NAME_HINTS = {"grey": "grey", "gray": "grey", "black": "black", "white": "white", "silver": "silver", "red": "red",
              "blue": "blue", "green": "green", "yellow": "yellow", "orange": "orange", "brown": "brown"}


def references(path):
    """/Game/... object paths a .uasset refers to (its imports)."""
    b = Path(path).read_bytes()
    return sorted({m.decode() for m in re.findall(rb"/Game/[A-Za-z0-9_/\-]+", b)})


def mesh_paint(content, mesh_file):
    """{"paint", "rgb", "from"} of a car mesh, from its car-paint materials; None when none is readable. With several
    paints (two-tone), the body's own, else the most saturated one."""
    found = []
    for r in references(mesh_file):
        name = r.rsplit("/", 1)[-1]
        if name == Path(mesh_file).stem or "glass" in name.lower() or "/Textures/" in r and name.startswith("T_"):
            continue
        f = Path(content) / (r[len("/Game/"):] + ".uasset")
        if not f.exists():
            continue
        vec, nm = vector_params(f)
        paint_mi = any("CarPaint" in s for s in nm if s.startswith("/Game/") and s != nm[0])
        body_like = re.search(r"body|carpaint|exterior", name, re.I)
        if "Base Color" in vec and (paint_mi or body_like) and not re.search(r"emissive|light|interior|detail", name,
                                                                             re.I):
            found.append((srgb(vec["Base Color"]), name))
        elif body_like and not vec:
            # a plain material (no instance parameters): its base-colour texture's source art, else its name
            for t in (s for s in nm if s.startswith("/Game/") and s.rsplit("/", 1)[-1].lower().endswith("_d")):
                tf = Path(content) / (t[len("/Game/"):] + ".uasset")
                rgb = png_average(tf) if tf.exists() else None
                if rgb:
                    found.append((rgb, f"{name}: {t.rsplit('/', 1)[-1]}"))
                    break
                hint = next((w for k, w in NAME_HINTS.items() if k in t.rsplit("/", 1)[-1].lower()), None)
                if hint:
                    found.append((fleet_rgb(hint), f"{name}: {t.rsplit('/', 1)[-1]} (its name)"))
                    break
    if not found:
        return None
    # the body's own material first (M_JeepWranglerRubicon_Bodywork over a generic paint on a part), then the most
    # saturated paint
    rgb, src = max(found, key=lambda f: (bool(re.search(r"body|exterior", f[1], re.I)),
                                         colorsys.rgb_to_hsv(*(c / 255 for c in f[0]))[1]))
    return {"paint": paint_name(rgb), "rgb": list(rgb), "from": src,
            **({"paints": [[list(c), s] for c, s in found]} if len(found) > 1 else {})}


def fleet_rgb(word):
    from coupledcarla import fleet
    return fleet.COLORS[word]


# --- the meshes ----------------------------------------------------------------------------------------------

def candidates(content, table):
    """{mesh name: .uasset path} of the table's car meshes and the glass meshes beside them."""
    base = Path(content) / "Carla" / "Static"
    out = {}
    for f in sorted(base.rglob("SM_*.uasset"), key=lambda f: ("/Parked/" not in f.as_posix(), str(f))):
        if "__External" in str(f):
            continue
        if f.stem in table and f.stem not in out:
            out[f.stem] = f
    for name, f in list(out.items()):
        for g in sorted(f.parent.glob("SM_*.uasset")):
            if "glass" in g.stem.lower() and glass_key(g.stem) == glass_key(name):
                out.setdefault(g.stem, g)
    return out


def glass_key(name):
    s = name.lower()
    for w in ("_glasses", "_glass", "glass", "_parked", "parked"):
        s = s.replace(w, "")
    return s.strip("_")


def inside(inner, outer, margin=0.1):
    """Is the box (off, ext) inner within outer (both relative to the same pivot)?"""
    return all(inner["off"][k] - inner["ext"][k] >= outer["off"][k] - outer["ext"][k] - margin and
               inner["off"][k] + inner["ext"][k] <= outer["off"][k] + outer["ext"][k] + margin for k in range(3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--content", default=None, help="the Content folder (default: from $CARLA_ROOT)")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--xodr", default=str(ROOT / "maps" / "eindhoven_centre" / "map.xodr"))
    ap.add_argument("--paints-only", action="store_true", help="only read the paints (CARLA 1.0, no server)")
    args = ap.parse_args()
    out_f = ROOT / "coupledcarla" / "data" / "parked_cars.json"
    data = json.loads(out_f.read_text())
    if args.paints_only:
        version, release, client = "1.0", "1.0", None
    else:
        import carla
        client = carla.Client("127.0.0.1", args.port)
        client.set_timeout(60)
        version = client.get_server_version()
        release = version.split("-")[0]
    gen = "0.9" if release.startswith("0.") else "1.0"
    table = parking.CARS_09 if gen == "0.9" else parking.CARS_10
    content = args.content or str(Path(os.environ.get("CARLA_ROOT", ".")) / CONTENT[gen])
    cands = candidates(content, table)
    missing = sorted(set(table) - set(cands))
    print(f"{len(cands)} meshes ({sum(1 for n in cands if n in table)} cars of {len(table)}; "
          f"missing {missing or 'none'}) in {content}", file=sys.stderr)
    block = {}
    if args.paints_only:        # and update the paints of the measured block
        block = data.get(release, {})
        for name, f in cands.items():
            if name in table:
                p = mesh_paint(content, f)
                print(name, p)
                for path, rec in block.items():
                    if path.rsplit(".", 1)[-1] == name:
                        for k in ("paint", "rgb", "from", "paints"):
                            rec.pop(k, None)
                        rec.update(p or {})
        if block:
            out_f.write_text(json.dumps(data, indent=0, sort_keys=True) + "\n")
        return
    import carla
    client.set_timeout(600)
    world = client.get_world()
    if not world.get_map().name.lower().startswith("opendrive"):
        world = client.generate_opendrive_world(Path(args.xodr).read_text(), carla.OpendriveGenerationParameters(
            enable_mesh_visibility=True, enable_pedestrian_navigation=False))
    bp = world.get_blueprint_library().find("static.prop.mesh")
    sizes = {}
    for i, (name, f) in enumerate(cands.items()):
        rel = f.relative_to(content).with_suffix("")
        path = f"/Game/{rel.as_posix()}.{name}"
        bp.set_attribute("mesh_path", path)
        bp.set_attribute("scale", "1")
        loc = carla.Location(x=5000 + 50 * (i % 20), y=5000 + 50 * (i // 20), z=0)
        actor = world.try_spawn_actor(bp, carla.Transform(loc))
        if actor is None:
            print(f"  {name}: spawn failed", file=sys.stderr)
            continue
        world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
        bb = actor.bounding_box
        e, c = bb.extent, bb.location
        if abs(c.x - loc.x) < 1000 and abs(c.x) > 1000:     # props: some versions give world coordinates
            c = carla.Location(c.x - loc.x, c.y - loc.y, c.z - loc.z)
        actor.destroy()
        ext = [e.x, e.y, e.z]
        if not all(math.isfinite(v) and 0 < v < 100 for v in ext):
            print(f"  {name}: no usable bounding box {ext}", file=sys.stderr)
            continue
        sizes[name] = (path, {"ext": [round(v, 2) for v in ext], "off": [round(c.x, 2), round(c.y, 2), round(c.z, 2)]})
    for name, (path, m) in sizes.items():
        if name not in table:
            continue
        rec = {"cat": "car", **m}
        glass = [g for g in sizes if g not in table and glass_key(g) == glass_key(name)
                 and cands[g].parent == cands[name].parent and inside(sizes[g][1], m)]
        if glass:
            rec["glass"] = sizes[sorted(glass, key=len)[0]][0]
            rec["glass_box"] = sizes[sorted(glass, key=len)[0]][1]
        if gen == "1.0":
            p = mesh_paint(content, cands[name])
            if p:
                rec.update(p)
        block[path] = rec
        print(f"  {name}: {2 * m['ext'][0]:.2f} x {2 * m['ext'][1]:.2f} x {2 * m['ext'][2]:.2f} m, "
              f"glass {'yes' if glass else 'no'}, paint {rec.get('paint')} {rec.get('rgb')} ({rec.get('from')})",
              file=sys.stderr)
    data[release] = block
    out_f.write_text(json.dumps(data, indent=0, sort_keys=True) + "\n")
    print(f"{len(block)} car meshes for CARLA {release} -> {out_f}", file=sys.stderr)


if __name__ == "__main__":
    main()
