"""Builds the coupled-carla ships as Unreal assets in a CARLA 1.0 (UE 5.8) tree, headless:

    python -m tools.ship_mesh /tmp/ships           # the glTF meshes (numpy)
    UnrealEditor-Cmd <tree>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \\
        -script="<repo>/tools/ue_ship_assets.py --src /tmp/ships" -unattended -nosplash -nullrhi -nosound

Writes /Game/CoupledCarla/Ships:
  SM_CCShip_<Hull>          the ships (KVLCC2, KCS), no collision, for ships.py's `mesh` look (static.prop.mesh + mass)
  M_CCShip_Lit, M_CCShip_Light and MI_CCShip_* instances: antifouling, boot-top, topsides, deck, accommodation,
      glass, funnel, masts, rudder, propeller (lit), and the navigation lights (emissive: red, green, white)
Prints the imported mesh's bounds so the axes can be checked (x forward, y right, z up in Unreal, origin amidships
on the waterline). The materials follow tools/ue_drone_assets.py; tools/content_pack.py packs /Game/CoupledCarla
for the release package (an editor server reads the tree directly).
"""
import argparse
import os
import shlex
import sys

import unreal

DEST = "/Game/CoupledCarla/Ships"
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
HULLS = ("KVLCC2", "KCS")
# linear base colours (the preview's sRGB colours of tools/ship_mesh.py, squared about)
LIT = {
    "Antifouling": ((0.30, 0.025, 0.015), 0.7, 0.0), "Boottop": ((0.010, 0.010, 0.012), 0.6, 0.0),
    "Hull": ((0.018, 0.028, 0.055), 0.45, 0.0), "Deck": ((0.10, 0.18, 0.10), 0.75, 0.0),
    "House": ((0.80, 0.80, 0.78), 0.5, 0.0), "Glass": ((0.008, 0.012, 0.018), 0.08, 0.0),
    "Funnel": ((0.78, 0.78, 0.76), 0.5, 0.0), "FunnelBand": ((0.012, 0.05, 0.25), 0.5, 0.0),
    "Mast": ((0.50, 0.36, 0.06), 0.6, 0.0), "Rudder": ((0.30, 0.025, 0.015), 0.7, 0.0),
    "Propeller": ((0.55, 0.32, 0.10), 0.3, 1.0),
    "BoxRed": ((0.35, 0.03, 0.015), 0.6, 0.3), "BoxBlue": ((0.015, 0.06, 0.25), 0.6, 0.3),
    "BoxGreen": ((0.03, 0.15, 0.05), 0.6, 0.3), "BoxGrey": ((0.27, 0.28, 0.29), 0.6, 0.3),
    "BoxWhite": ((0.70, 0.70, 0.68), 0.6, 0.3), "BoxOrange": ((0.69, 0.18, 0.015), 0.6, 0.3)}
LIGHTS = {"NavRed": ((1.0, 0.03, 0.01), 60.0), "NavGreen": ((0.02, 1.0, 0.06), 60.0),
          "NavWhite": ((1.0, 0.95, 0.85), 80.0)}


def log(msg):
    unreal.log_warning("[ships] " + str(msg))


def tools():
    return unreal.AssetToolsHelpers.get_asset_tools()


def import_glb(src, name):
    t = unreal.AssetImportTask()
    t.set_editor_property("filename", src)
    t.set_editor_property("destination_path", DEST)
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
            mp.set_editor_property("collision", False)
        except Exception:
            pass
        pipe.get_editor_property("material_pipeline").set_editor_property("import_materials", False)
        try:
            pipe.get_editor_property("common_meshes_properties").set_editor_property(
                "force_all_mesh_as_type", unreal.InterchangeForceMeshType.IFMT_STATIC_MESH)
        except Exception:
            pass
        t.set_editor_property("options", pipe)
    except Exception as exc:  # noqa: BLE001
        log("interchange options: %s" % exc)
    tools().import_asset_tasks([t])
    got = [str(p) for p in (t.get_editor_property("imported_object_paths") or [])]
    meshes = [m for m in (unreal.load_object(None, p) for p in got) if isinstance(m, unreal.StaticMesh)]
    if not meshes:
        found = [p for p in EAL.list_assets(DEST, recursive=False) if name in p.split("/")[-1]]
        meshes = [m for m in (EAL.load_asset(p.split(".")[0]) for p in found) if isinstance(m, unreal.StaticMesh)]
    if not meshes:
        raise RuntimeError("import of %s gave nothing (%s)" % (src, got))
    mesh = meshes[0]
    path = mesh.get_path_name().split(".")[0]
    if path != "%s/%s" % (DEST, name):
        if EAL.does_asset_exist("%s/%s" % (DEST, name)):
            EAL.delete_asset("%s/%s" % (DEST, name))
        EAL.rename_asset(path, "%s/%s" % (DEST, name))
        mesh = EAL.load_asset("%s/%s" % (DEST, name))
    try:
        mesh.set_editor_property("nanite_settings", unreal.MeshNaniteSettings(enabled=False))
    except Exception:
        pass
    bb = mesh.get_bounding_box()
    slots = [str(m.get_editor_property("material_slot_name")) for m in mesh.get_editor_property("static_materials")]
    log("%s: bounds min (%.0f, %.0f, %.0f) max (%.0f, %.0f, %.0f) cm, slots %s" % (
        name, bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z, slots))
    return mesh


def new_material(name):
    path = "%s/%s" % (DEST, name)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    return tools().create_asset(name, DEST, unreal.Material, unreal.MaterialFactoryNew())


def expr(m, cls, x, y, **props):
    e = MEL.create_material_expression(m, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def vparam(m, name, rgb, x, y):
    return expr(m, unreal.MaterialExpressionVectorParameter, x, y, parameter_name=name,
                default_value=unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))


def sparam(m, name, v, x, y):
    return expr(m, unreal.MaterialExpressionScalarParameter, x, y, parameter_name=name, default_value=float(v))


def m_lit():
    m = new_material("M_CCShip_Lit")
    mp = unreal.MaterialProperty
    MEL.connect_material_property(vparam(m, "BaseColor", (0.5, 0.5, 0.5), -400, 0), "", mp.MP_BASE_COLOR)
    MEL.connect_material_property(sparam(m, "Metallic", 0.0, -400, 150), "", mp.MP_METALLIC)
    MEL.connect_material_property(sparam(m, "Roughness", 0.5, -400, 250), "", mp.MP_ROUGHNESS)
    MEL.recompile_material(m)
    return m


def m_light():
    m = new_material("M_CCShip_Light")
    mp = unreal.MaterialProperty
    col = vparam(m, "Color", (1.0, 1.0, 1.0), -600, 0)
    inten = sparam(m, "Intensity", 60.0, -600, 150)
    mul = expr(m, unreal.MaterialExpressionMultiply, -300, 50)
    MEL.connect_material_expressions(col, "", mul, "A")
    MEL.connect_material_expressions(inten, "", mul, "B")
    MEL.connect_material_property(mul, "", mp.MP_EMISSIVE_COLOR)
    MEL.connect_material_property(col, "", mp.MP_BASE_COLOR)
    MEL.recompile_material(m)
    return m


def mi(name, parent, vectors=None, scalars=None):
    path = "%s/%s" % (DEST, name)
    inst = EAL.load_asset(path) if EAL.does_asset_exist(path) else tools().create_asset(
        name, DEST, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(inst, parent)
    for k, v in (vectors or {}).items():
        MEL.set_material_instance_vector_parameter_value(inst, k, unreal.LinearColor(v[0], v[1], v[2], 1.0))
    for k, v in (scalars or {}).items():
        MEL.set_material_instance_scalar_parameter_value(inst, k, float(v))
    MEL.update_material_instance(inst)
    EAL.save_asset(path, only_if_is_dirty=False)
    return inst


def materials():
    lit, light = m_lit(), m_light()
    for mm in (lit, light):
        EAL.save_asset(mm.get_path_name().split(".")[0], only_if_is_dirty=False)
    out = {k: mi("MI_CCShip_" + k, lit, {"BaseColor": c}, {"Roughness": r, "Metallic": met})
           for k, (c, r, met) in LIT.items()}
    out.update({k: mi("MI_CCShip_" + k, light, {"Color": c}, {"Intensity": i}) for k, (c, i) in LIGHTS.items()})
    return out


def assign(mesh, mats):
    for k, s in enumerate(mesh.get_editor_property("static_materials")):
        slot = str(s.get_editor_property("material_slot_name"))
        key = next((n for n in sorted(mats, key=len, reverse=True) if slot.startswith(n)), None)
        if key is None:
            log("slot %s of %s has no material" % (slot, mesh.get_name()))
            continue
        mesh.set_material(k, mats[key])


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    a = ap.parse_args(argv)
    if not EAL.does_directory_exist(DEST):
        EAL.make_directory(DEST)
    mats = materials()
    for hull in HULLS:
        name = "SM_CCShip_%s" % hull
        mesh = import_glb(os.path.join(a.src, name + ".glb"), name)
        assign(mesh, mats)
        EAL.save_asset("%s/%s" % (DEST, name), only_if_is_dirty=False)
    for p in EAL.list_assets(DEST, recursive=True):          # what Interchange made besides ours (slot materials)
        n = p.split("/")[-1].split(".")[0]
        if not n.startswith(("M_CCShip", "MI_CCShip", "SM_CCShip")):
            EAL.delete_asset(p.split(".")[0])
    unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
    log("done: %s" % ", ".join(sorted(p.split("/")[-1].split(".")[0] for p in EAL.list_assets(DEST))))


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 1 and " " in args[0]:        # -script="file --src x" arrives as one string on some builds
        args = shlex.split(args[0])
    main(args)
