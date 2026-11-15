r"""Dutch sign faces into a CARLA 1.0 (Unreal Engine 5.8) source tree: one texture, material instance and plate mesh
per face of coupledcarla/data/nl_sign_faces.json, under /Game/CoupledCarla/Signs/NL. Editor Python, headless:

    UnrealEditor-Cmd <CarlaUnreal.uproject> -run=pythonscript -script="<abs>/ue_nl_signs.py --textures <dir>
        --manifest <abs>/nl_sign_faces.json [--report <file>] [--only B6,E4]"
    ... -script="<abs>/ue_nl_signs.py --verify --manifest ... --report <file>"     (reads, writes nothing)
    ... -script="<abs>/ue_nl_signs.py --probe <dir>"     (the blank plates' polygons -> <dir>/sections.json, for
                                                          tools/nl_sign_faces.py --plates-from)

What it makes (idempotent: existing assets are updated in place):
  Materials/M_NL_SignFace      the parent: texture parameter "Face" (UV0) -> base colour, scalar "Roughness"
  Textures/T_NL_<key>          the PNG of tools/nl_sign_faces.py (sRGB, default compression, never streamed: a
                               plate spawned at run time has no texture streaming data, and the streamer would
                               keep it at a low mip)
  Materials/MI_NL_<key>        a constant instance of M_NL_SignFace with Face = T_NL_<key>
  Meshes/SM_NL_<key>           a duplicate of the CARLA blank plate the face was drawn for (same geometry and
                               pivot), its material slot 0 (FrontSide) set to MI_NL_<key>; slot 1 (BackSide)
                               keeps CARLA's grey back

static.prop.mesh keeps a mesh's own materials, so each face needs its own mesh (CARLA 1.0's StaticMeshFactory has
mesh_path, mass and scale, no material attribute). For a package the folder has to be cooked:
+DirectoriesToAlwaysCook=(Path="/Game/CoupledCarla") in Config/DefaultGame.ini.

On win5090 (2026-11-15) it ran in C:\cc58 as the scheduled task cc58-dutchsigns (S4U, below-normal priority):
C:\cc58\dutchsigns\job.ps1 calls UnrealEditor-Cmd.exe C:\cc58\carla\Unreal\CarlaUnreal\CarlaUnreal.uproject
-run=pythonscript "-script=C:/cc58/dutchsigns/ue_nl_signs.py --textures C:/cc58/dutchsigns/textures --manifest
C:/cc58/dutchsigns/nl_sign_faces.json --report C:/cc58/dutchsigns/build_report.json" -unattended -nop4 -nosplash
(about 70 s; 248 faces, 32 MB of uassets). The textures come from tools/nl_sign_faces.py on any machine.
"""
import json
import os
import sys
import time

import unreal

ROOT = "/Game/CoupledCarla/Signs/NL"
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
PARENT = ROOT + "/Materials/M_NL_SignFace"
SIGNS = "/Game/Carla/Static/TrafficSign/Signs"


def log(msg):
    unreal.log("[nl_signs] " + msg)


def arg(argv, name, default=None):
    if name in argv:
        i = argv.index(name)
        return argv[i + 1] if i + 1 < len(argv) and not argv[i + 1].startswith("--") else True
    return default


def tools():
    return unreal.AssetToolsHelpers.get_asset_tools()


def load(path):
    return EAL.load_asset(path) if EAL.does_asset_exist(path) else None


def parent_material():
    mat = load(PARENT)
    if mat is not None:
        return mat
    folder, name = PARENT.rsplit("/", 1)
    EAL.make_directory(folder)
    mat = tools().create_asset(name, folder, unreal.Material, unreal.MaterialFactoryNew())
    tex = MEL.create_material_expression(mat, unreal.MaterialExpressionTextureSampleParameter2D, -500, 0)
    tex.set_editor_property("parameter_name", "Face")
    tex.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/DefaultTexture.DefaultTexture"))
    MEL.connect_material_property(tex, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    rough = MEL.create_material_expression(mat, unreal.MaterialExpressionScalarParameter, -500, 300)
    rough.set_editor_property("parameter_name", "Roughness")
    rough.set_editor_property("default_value", 0.45)     # painted retroreflective sheeting: fairly smooth
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.recompile_material(mat)
    EAL.save_asset(PARENT, only_if_is_dirty=False)
    log("created " + PARENT)
    return mat


def import_textures(faces, texdir):
    folder = ROOT + "/Textures"
    EAL.make_directory(folder)
    tasks = []
    for key, f in sorted(faces.items()):
        png = os.path.join(texdir, f["texture"] + ".png")
        if not os.path.exists(png):
            unreal.log_warning("[nl_signs] missing " + png)
            continue
        t = unreal.AssetImportTask()
        t.set_editor_property("filename", png)
        t.set_editor_property("destination_path", folder)
        t.set_editor_property("destination_name", f["texture"])
        t.set_editor_property("automated", True)
        t.set_editor_property("replace_existing", True)
        t.set_editor_property("save", False)
        tasks.append(t)
    t0 = time.time()
    for i in range(0, len(tasks), 50):
        tools().import_asset_tasks(tasks[i:i + 50])
    out = {}
    for key, f in sorted(faces.items()):
        path = folder + "/" + f["texture"]
        tex = load(path)
        if tex is None:
            continue
        tex.set_editor_property("srgb", True)
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_DEFAULT)
        tex.set_editor_property("never_stream", True)
        EAL.save_asset(path, only_if_is_dirty=False)
        out[key] = tex
    log("textures: %d of %d imported in %.0f s" % (len(out), len(faces), time.time() - t0))
    return out


def material_instance(f, tex, parent):
    folder = ROOT + "/Materials"
    EAL.make_directory(folder)
    path = folder + "/" + f["material"]
    mi = load(path) or tools().create_asset(f["material"], folder, unreal.MaterialInstanceConstant,
                                            unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(mi, parent)
    MEL.set_material_instance_texture_parameter_value(mi, "Face", tex)
    MEL.update_material_instance(mi)
    EAL.save_asset(path, only_if_is_dirty=False)
    return mi


def plate_mesh(f, mi):
    folder = ROOT + "/Meshes"
    EAL.make_directory(folder)
    path = folder + "/" + f["mesh"]
    mesh = load(path)
    if mesh is None:
        mesh = EAL.duplicate_asset(SIGNS + "/" + f["plate"], path)
        if mesh is None:
            raise RuntimeError("could not duplicate %s/%s" % (SIGNS, f["plate"]))
    mesh.set_material(0, mi)
    EAL.save_asset(path, only_if_is_dirty=False)
    return mesh


def tex_size(tex):
    for get in (lambda t: (t.blueprint_get_size_x(), t.blueprint_get_size_y()),
                lambda t: tuple(int(v) for v in (t.get_editor_property("imported_size").x,
                                                 t.get_editor_property("imported_size").y))):
        try:
            w, h = get(tex)
            if w and h:
                return [int(w), int(h)]
        except Exception:
            pass
    return None


def verify(faces):
    """Reload every face's mesh, material instance and texture and check they hang together."""
    rep, bad = {}, []
    for key, f in sorted(faces.items()):
        r = {"mesh": ROOT + "/Meshes/" + f["mesh"]}
        mesh = load(r["mesh"])
        if mesh is None:
            r["error"] = "no mesh"
            bad.append(key)
            rep[key] = r
            continue
        slots = mesh.get_editor_property("static_materials")
        r["slots"] = [(str(s.get_editor_property("material_slot_name")),
                       s.get_editor_property("material_interface").get_path_name()
                       if s.get_editor_property("material_interface") else None) for s in slots]
        mi = slots[0].get_editor_property("material_interface") if slots else None
        ok = mi is not None and mi.get_name() == f["material"]
        if ok:
            par = mi.get_editor_property("parent")
            r["parent"] = par.get_path_name() if par else None
            tex = MEL.get_material_instance_texture_parameter_value(mi, "Face")
            r["texture"] = tex.get_path_name() if tex else None
            ok = ok and tex is not None and tex.get_name() == f["texture"] and r["parent"].startswith(PARENT)
            if tex is not None:
                r["size"] = tex_size(tex)
                r["never_stream"] = bool(tex.get_editor_property("never_stream"))
        bb = mesh.get_bounding_box()
        r["bbox_cm"] = [round(bb.min.x, 2), round(bb.min.y, 2), round(bb.min.z, 2),
                        round(bb.max.x, 2), round(bb.max.y, 2), round(bb.max.z, 2)]
        r["ok"] = bool(ok)
        if not ok:
            bad.append(key)
        rep[key] = r
    par = load(PARENT)
    rep["_parent"] = {"path": PARENT, "exists": par is not None,
                      "texture_params": [str(n) for n in MEL.get_texture_parameter_names(par)] if par else [],
                      "scalar_params": [str(n) for n in MEL.get_scalar_parameter_names(par)] if par else []}
    log("verify: parent %s; %d faces, %d ok, %d bad %s" % (rep["_parent"], len(faces), len(faces) - len(bad),
                                                            len(bad), ",".join(bad[:30])))
    return rep


def probe(outdir):
    """The blank plates' polygons: material group, positions and UV0 of each vertex (MeshDescription)."""
    os.makedirs(outdir, exist_ok=True)
    res = {}
    for p in [p.split(".")[0] for p in EAL.list_assets(SIGNS, recursive=False)]:
        m = EAL.load_asset(p)
        if not isinstance(m, unreal.StaticMesh):
            continue
        d = m.get_static_mesh_description(0)
        tris = []
        for i in range(d.get_polygon_count()):
            pid = unreal.PolygonID()
            pid.set_editor_property("id_value", i)
            if not d.is_polygon_valid(pid):
                continue
            g = d.get_polygon_polygon_group(pid).get_editor_property("id_value")
            pts = []
            for vi in d.get_polygon_vertex_instances(pid):
                pos = d.get_vertex_position(d.get_vertex_instance_vertex(vi))
                uv = d.get_vertex_instance_uv(vi, 0)
                pts.append([round(pos.x, 3), round(pos.y, 3), round(pos.z, 3), round(uv.x, 5), round(uv.y, 5)])
            tris.append([g, pts])
        res[p.rsplit("/", 1)[-1]] = {"tris": tris, "slots": [
            str(s.get_editor_property("material_slot_name")) for s in m.get_editor_property("static_materials")]}
    with open(os.path.join(outdir, "sections.json"), "w") as f:
        json.dump(res, f)
    log("probe: %d plates -> %s" % (len(res), outdir))


def main(argv):
    if arg(argv, "--probe"):
        probe(arg(argv, "--probe"))
        return
    man = json.load(open(arg(argv, "--manifest")))
    faces = man["faces"]
    only = arg(argv, "--only")
    if only:
        faces = {k: v for k, v in faces.items() if k in only.split(",")}
    report = arg(argv, "--report")
    if not arg(argv, "--verify"):
        t0 = time.time()
        parent = parent_material()
        texs = import_textures(faces, arg(argv, "--textures"))
        n = 0
        for key, f in sorted(faces.items()):
            if key not in texs:
                continue
            try:
                plate_mesh(f, material_instance(f, texs[key], parent))
                n += 1
            except Exception as err:
                unreal.log_error("[nl_signs] %s: %s" % (key, err))
        log("built %d of %d faces in %.0f s" % (n, len(faces), time.time() - t0))
    rep = verify(faces)
    if report:
        with open(report, "w") as fh:
            json.dump(rep, fh, indent=1)
    log("DONE")


main(sys.argv[1:])
