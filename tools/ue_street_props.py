"""Imports tools/street_props.py's meshes into a CARLA 1.0 (UE 5.8) tree as spawnable static.prop.mesh props, headless:

    python tools/street_props.py /tmp/street_props
    UnrealEditor-Cmd <tree>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \\
        -script="<repo>/tools/ue_street_props.py --src /tmp/street_props [--report <file>] [--only SM_CC_NL_Bollard]" \\
        -unattended -nosplash -nullrhi -nosound

Writes /Game/CoupledCarla/Street (cook it: +DirectoriesToAlwaysCook=(Path="/Game/CoupledCarla") in
Config/DefaultGame.ini, which C:\\cc58 has since the Dutch sign faces):
  Materials/M_CC_Street_Lit       BaseColor, Metallic, Roughness parameters
  Materials/M_CC_Street_Decal     masked, two-sided: texture "Tex" (RGBA, alpha = opacity mask) x Tint, Roughness
  Materials/M_CC_Street_Glow      emissive: "Tex" x Tint x Emissive, panned (PanU, PanV) per second (screens, signs)
  Materials/MI_CC_<slot>          one constant instance per PALETTE slot of the manifest
  Textures/T_CC_<key>             the decal textures (sRGB, never streamed: a prop spawned at run time has no
                                  streaming data and would keep a low mip)
  <Region>/<Folder>/SM_CC_<Region>_<Name>   the meshes; the folder (Pole, Static, TrafficLight, RoadLine, Road,
                                  Building) is what CARLA's tagger reads as the semantic label
Collision: the render triangles as collision (complex as simple), so a lamp's arm over the road has no box under it
down to the ground; decals and markings get none. Nanite off (small meshes, and the -nullrhi editor). Each mesh's
measured bounds are checked against the manifest (Unreal axes) and written to the report.
"""
import argparse
import json
import os
import shlex
import sys

import unreal

ROOT = "/Game/CoupledCarla/Street"
MATS = ROOT + "/Materials"
TEX = ROOT + "/Textures"
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary


def log(msg):
    unreal.log_warning("[street] " + str(msg))     # a commandlet's log keeps warnings


def tools():
    return unreal.AssetToolsHelpers.get_asset_tools()


def load(path):
    return EAL.load_asset(path) if EAL.does_asset_exist(path) else None


# -- materials ----------------------------------------------------------------------------------------------------

def expr(m, cls, x, y, **props):
    e = MEL.create_material_expression(m, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def new_material(name):
    path = "%s/%s" % (MATS, name)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    return tools().create_asset(name, MATS, unreal.Material, unreal.MaterialFactoryNew())


def m_lit():
    m = new_material("M_CC_Street_Lit")
    mp = unreal.MaterialProperty
    MEL.connect_material_property(expr(m, unreal.MaterialExpressionVectorParameter, -400, 0, parameter_name="BaseColor",
                                       default_value=unreal.LinearColor(0.5, 0.5, 0.5, 1)), "", mp.MP_BASE_COLOR)
    MEL.connect_material_property(expr(m, unreal.MaterialExpressionScalarParameter, -400, 150,
                                       parameter_name="Metallic", default_value=0.0), "", mp.MP_METALLIC)
    MEL.connect_material_property(expr(m, unreal.MaterialExpressionScalarParameter, -400, 250,
                                       parameter_name="Roughness", default_value=0.5), "", mp.MP_ROUGHNESS)
    MEL.recompile_material(m)
    EAL.save_asset(MATS + "/M_CC_Street_Lit", only_if_is_dirty=False)
    return m


def m_decal():
    m = new_material("M_CC_Street_Decal")
    m.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
    m.set_editor_property("two_sided", True)
    m.set_editor_property("opacity_mask_clip_value", 0.35)
    mp = unreal.MaterialProperty
    tex = expr(m, unreal.MaterialExpressionTextureSampleParameter2D, -700, 0, parameter_name="Tex")
    tex.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/DefaultTexture.DefaultTexture"))
    tint = expr(m, unreal.MaterialExpressionVectorParameter, -700, 250, parameter_name="Tint",
                default_value=unreal.LinearColor(1, 1, 1, 1))
    mul = expr(m, unreal.MaterialExpressionMultiply, -350, 50)
    MEL.connect_material_expressions(tex, "RGB", mul, "A")
    MEL.connect_material_expressions(tint, "", mul, "B")
    MEL.connect_material_property(mul, "", mp.MP_BASE_COLOR)
    MEL.connect_material_property(tex, "A", mp.MP_OPACITY_MASK)
    MEL.connect_material_property(expr(m, unreal.MaterialExpressionScalarParameter, -400, 350,
                                       parameter_name="Roughness", default_value=0.8), "", mp.MP_ROUGHNESS)
    MEL.recompile_material(m)
    EAL.save_asset(MATS + "/M_CC_Street_Decal", only_if_is_dirty=False)
    return m


def m_glow():
    """Emissive and lit at once (a screen, a lit sign): BaseColor = Tex x Tint x 0.2, Emissive = Tex x Tint x
    Emissive, the texture panned by (PanU, PanV) per second (a screen's content scrolls; 0 for signs)."""
    m = new_material("M_CC_Street_Glow")
    mp = unreal.MaterialProperty
    tc = expr(m, unreal.MaterialExpressionTextureCoordinate, -1300, 0)
    t = expr(m, unreal.MaterialExpressionTime, -1300, 150)
    pu = expr(m, unreal.MaterialExpressionScalarParameter, -1300, 250, parameter_name="PanU", default_value=0.0)
    pv = expr(m, unreal.MaterialExpressionScalarParameter, -1300, 350, parameter_name="PanV", default_value=0.0)
    speed = expr(m, unreal.MaterialExpressionAppendVector, -1100, 300)
    MEL.connect_material_expressions(pu, "", speed, "A")
    MEL.connect_material_expressions(pv, "", speed, "B")
    shift = expr(m, unreal.MaterialExpressionMultiply, -950, 200)
    MEL.connect_material_expressions(t, "", shift, "A")
    MEL.connect_material_expressions(speed, "", shift, "B")
    uv = expr(m, unreal.MaterialExpressionAdd, -800, 50)
    MEL.connect_material_expressions(tc, "", uv, "A")
    MEL.connect_material_expressions(shift, "", uv, "B")
    tex = expr(m, unreal.MaterialExpressionTextureSampleParameter2D, -600, 0, parameter_name="Tex")
    tex.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/DefaultTexture.DefaultTexture"))
    MEL.connect_material_expressions(uv, "", tex, "UVs")
    tint = expr(m, unreal.MaterialExpressionVectorParameter, -600, 250, parameter_name="Tint",
                default_value=unreal.LinearColor(1, 1, 1, 1))
    col = expr(m, unreal.MaterialExpressionMultiply, -350, 50)
    MEL.connect_material_expressions(tex, "RGB", col, "A")
    MEL.connect_material_expressions(tint, "", col, "B")
    base = expr(m, unreal.MaterialExpressionMultiply, -150, 0)
    MEL.connect_material_expressions(col, "", base, "A")
    base.set_editor_property("const_b", 0.2)
    MEL.connect_material_property(base, "", mp.MP_BASE_COLOR)
    glow = expr(m, unreal.MaterialExpressionScalarParameter, -350, 250, parameter_name="Emissive", default_value=4.0)
    em = expr(m, unreal.MaterialExpressionMultiply, -150, 150)
    MEL.connect_material_expressions(col, "", em, "A")
    MEL.connect_material_expressions(glow, "", em, "B")
    MEL.connect_material_property(em, "", mp.MP_EMISSIVE_COLOR)
    MEL.connect_material_property(expr(m, unreal.MaterialExpressionScalarParameter, -350, 400,
                                       parameter_name="Roughness", default_value=0.3), "", mp.MP_ROUGHNESS)
    MEL.recompile_material(m)
    EAL.save_asset(MATS + "/M_CC_Street_Glow", only_if_is_dirty=False)
    return m


def import_texture(src, name):
    t = unreal.AssetImportTask()
    t.set_editor_property("filename", src)
    t.set_editor_property("destination_path", TEX)
    t.set_editor_property("destination_name", name)
    t.set_editor_property("automated", True)
    t.set_editor_property("replace_existing", True)
    t.set_editor_property("save", False)
    tools().import_asset_tasks([t])
    tx = load("%s/%s" % (TEX, name))
    if tx is None:      # Interchange may name it after the file
        found = [p for p in EAL.list_assets(TEX, recursive=False) if name in p]
        tx = EAL.load_asset(found[0].split(".")[0]) if found else None
    if tx is None:
        raise RuntimeError("texture %s did not import" % src)
    tx.set_editor_property("srgb", True)
    tx.set_editor_property("never_stream", True)
    EAL.save_asset(tx.get_path_name().split(".")[0], only_if_is_dirty=False)
    return tx


def instances(manifest, src):
    lit, dec = m_lit(), m_decal()
    glow_m = m_glow() if any(len(spec) > 6 for spec in manifest["palette"].values()) else None
    out = {}
    for slot, spec in manifest["palette"].items():
        rgb, metal, rough = spec[0:3], spec[3], spec[4]
        tex_key = spec[5] if len(spec) > 5 else None
        name = "MI_CC_" + slot
        path = "%s/%s" % (MATS, name)
        inst = load(path) or tools().create_asset(name, MATS, unreal.MaterialInstanceConstant,
                                                  unreal.MaterialInstanceConstantFactoryNew())
        if tex_key and len(spec) > 6:        # [.., texture, emissive, pan u, pan v]: a screen or a lit sign
            MEL.set_material_instance_parent(inst, glow_m)
            tx = import_texture(os.path.join(src, "T_CC_%s.png" % tex_key), "T_CC_%s" % tex_key)
            MEL.set_material_instance_texture_parameter_value(inst, "Tex", tx)
            MEL.set_material_instance_vector_parameter_value(inst, "Tint", unreal.LinearColor(*rgb, 1.0))
            MEL.set_material_instance_scalar_parameter_value(inst, "Emissive", float(spec[6]))
            MEL.set_material_instance_scalar_parameter_value(inst, "PanU", float(spec[7]) if len(spec) > 7 else 0.0)
            MEL.set_material_instance_scalar_parameter_value(inst, "PanV", float(spec[8]) if len(spec) > 8 else 0.0)
        elif tex_key:
            MEL.set_material_instance_parent(inst, dec)
            tx = import_texture(os.path.join(src, "T_CC_%s.png" % tex_key), "T_CC_%s" % tex_key)
            MEL.set_material_instance_texture_parameter_value(inst, "Tex", tx)
            MEL.set_material_instance_vector_parameter_value(inst, "Tint", unreal.LinearColor(*rgb, 1.0))
        else:
            MEL.set_material_instance_parent(inst, lit)
            MEL.set_material_instance_vector_parameter_value(inst, "BaseColor", unreal.LinearColor(*rgb, 1.0))
            MEL.set_material_instance_scalar_parameter_value(inst, "Metallic", float(metal))
        MEL.set_material_instance_scalar_parameter_value(inst, "Roughness", float(rough))
        MEL.update_material_instance(inst)
        EAL.save_asset(path, only_if_is_dirty=False)
        out[slot] = inst
    return out


# -- meshes -------------------------------------------------------------------------------------------------------

def import_glb(src, dest, name):
    t = unreal.AssetImportTask()
    t.set_editor_property("filename", src)
    t.set_editor_property("destination_path", dest)
    t.set_editor_property("destination_name", name)
    t.set_editor_property("automated", True)
    t.set_editor_property("replace_existing", True)
    t.set_editor_property("save", False)
    try:
        pipe = unreal.InterchangeGenericAssetsPipeline()
        mp = pipe.get_editor_property("mesh_pipeline")
        mp.set_editor_property("import_static_meshes", True)
        mp.set_editor_property("import_skeletal_meshes", False)
        mp.set_editor_property("build_nanite", False)
        try:
            mp.set_editor_property("collision", False)      # set below: complex as simple
        except Exception:
            pass
        pipe.get_editor_property("material_pipeline").set_editor_property("import_materials", False)
        common = pipe.get_editor_property("common_meshes_properties")
        try:
            common.set_editor_property("force_all_mesh_as_type", unreal.InterchangeForceMeshType.IFMT_STATIC_MESH)
        except Exception:
            pass
        t.set_editor_property("options", pipe)
    except Exception as exc:  # noqa: BLE001
        log("interchange options: %s" % exc)
    tools().import_asset_tasks([t])
    got = [str(p) for p in (t.get_editor_property("imported_object_paths") or [])]
    meshes = [m for m in (unreal.load_object(None, p) for p in got) if isinstance(m, unreal.StaticMesh)]
    if not meshes:
        found = [p for p in EAL.list_assets(dest, recursive=False) if name in p.split("/")[-1]]
        meshes = [m for m in (EAL.load_asset(p.split(".")[0]) for p in found) if isinstance(m, unreal.StaticMesh)]
    if not meshes:
        raise RuntimeError("import of %s gave nothing (%s)" % (src, got))
    mesh = meshes[0]
    path = mesh.get_path_name().split(".")[0]
    want = "%s/%s" % (dest, name)
    if path != want:
        if EAL.does_asset_exist(want):
            EAL.delete_asset(want)
        EAL.rename_asset(path, want)
        mesh = EAL.load_asset(want)
    try:
        mesh.set_editor_property("nanite_settings", unreal.MeshNaniteSettings(enabled=False))
    except Exception:
        pass
    return mesh


def set_collision(mesh, decal):
    """Render triangles as collision for solid props; none for flat decals and markings."""
    body = mesh.get_editor_property("body_setup")
    if body is None:
        return "no body setup"
    flag = unreal.CollisionTraceFlag
    body.set_editor_property("collision_trace_flag", flag.CTF_USE_SIMPLE_AS_COMPLEX if decal
                             else flag.CTF_USE_COMPLEX_AS_SIMPLE)
    if decal:
        try:
            unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem).remove_collisions(mesh)
        except Exception:  # noqa: BLE001
            pass
    return "none" if decal else "complex as simple"


def set_lods(mesh):
    """Three LODs (100 %, 30 %, 8 % of the triangles at screen sizes 1, 0.3, 0.08) for the heavier props (bikes,
    columns, lamps): thousands of parked bikes stand in a street. Returns the LOD count."""
    try:
        settings = [unreal.EditorScriptingMeshReductionSettings(percent_triangles=p_, screen_size=s_)
                    for p_, s_ in ((1.0, 1.0), (0.3, 0.3), (0.08, 0.08))]
        opts = unreal.EditorScriptingMeshReductionOptions(auto_compute_lod_screen_size=False,
                                                          reduction_settings=settings)
        sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
        return sub.set_lods(mesh, opts)
    except Exception as err:  # noqa: BLE001 - one LOD then
        log("LODs for %s: %s" % (mesh.get_name(), err))
        return 1


def assign(mesh, mats):
    missing = []
    for k, sm in enumerate(mesh.get_editor_property("static_materials")):
        slot = str(sm.get_editor_property("material_slot_name"))
        key = next((s for s in sorted(mats, key=len, reverse=True) if slot == s or slot.startswith(s)), None)
        if key is None:
            missing.append(slot)
            continue
        mesh.set_material(k, mats[key])
    return missing


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="folder with tools/street_props.py's output")
    ap.add_argument("--report", default=None)
    ap.add_argument("--only", default=None, help="comma-separated mesh names")
    a = ap.parse_args(argv)
    manifest = json.load(open(os.path.join(a.src, "street_props.json")))
    for d in (ROOT, MATS, TEX):
        EAL.make_directory(d)
    mats = instances(manifest, a.src)
    only = set(a.only.split(",")) if a.only else None
    report = {}
    for name, spec in sorted(manifest["meshes"].items()):
        if only and name not in only:
            continue
        dest = spec["path"].rsplit("/", 1)[0]
        EAL.make_directory(dest)
        try:
            mesh = import_glb(os.path.join(a.src, name + ".glb"), dest, name)
        except Exception as err:  # noqa: BLE001 - report it, go on with the others
            report[name] = {"error": str(err)}
            log("%s: %s" % (name, err))
            continue
        missing = assign(mesh, mats)
        lods = set_lods(mesh) if spec.get("triangles", 0) >= 800 else 1
        coll = set_collision(mesh, spec.get("decal") or spec["folder"] == "RoadLine")
        EAL.save_asset(dest + "/" + name, only_if_is_dirty=False)
        bb = mesh.get_bounding_box()
        ext = [(bb.max.x - bb.min.x) / 200, (bb.max.y - bb.min.y) / 200, (bb.max.z - bb.min.z) / 200]
        off = [(bb.max.x + bb.min.x) / 200, (bb.max.y + bb.min.y) / 200, (bb.max.z + bb.min.z) / 200]
        err = max(abs(p - q) for p, q in zip(ext + off, spec["ext"] + spec["off"]))
        report[name] = {"path": mesh.get_path_name(), "ext": [round(v, 3) for v in ext],
                        "off": [round(v, 3) for v in off], "max_err_m": round(err, 3), "missing_slots": missing,
                        "collision": coll, "lods": lods}
        log("%s: ext %s off %s (manifest error %.3f m)%s, collision %s" % (
            name, report[name]["ext"], report[name]["off"], err,
            ("; slots without material: %s" % missing) if missing else "", coll))
    # Interchange makes placeholder materials for the glTF's slots even with import_materials off: delete them
    for p in EAL.list_assets(ROOT, recursive=True):
        n = p.split("/")[-1].split(".")[0]
        if not n.startswith(("M_CC_Street", "MI_CC_", "SM_CC_", "T_CC_")):
            EAL.delete_asset(p.split(".")[0])
    unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
    if a.report:
        with open(a.report, "w") as f:
            json.dump(report, f, indent=1)
    bad = [n for n, r in report.items() if "error" in r or r.get("max_err_m", 0) > 0.02]
    log("done: %d meshes, %d with problems %s" % (len(report), len(bad), bad))


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 1 and " " in args[0]:
        args = shlex.split(args[0])
    main(args)
