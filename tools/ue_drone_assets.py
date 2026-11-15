"""Builds the coupled-carla drones as Unreal assets in a CARLA 1.0 (UE 5.8) tree, headless:

    python tools/drone_mesh.py /tmp/drones          # the glTF meshes (numpy)
    UnrealEditor-Cmd <tree>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \\
        -script="<repo>/tools/ue_drone_assets.py --src /tmp/drones" -unattended -nosplash -nullrhi -nosound

Writes /Game/CoupledCarla/Drones:
  SM_CCDrone_<Airframe>_{Body,Prop,Disc,Full}   the meshes (Consumer, Industrial), no collision
  M_CCDrone_Lit, M_CCDrone_Light, M_CCDrone_PropSpin, M_CCDrone_Disc and their MI_CCDrone_* instances: plastic,
      carbon, metal, lens, parcel; navigation lights (red, green; emissive) and the white anti-collision strobe
      (a 60 ms flash a second, each drone's phase from its position); spinning blades (turned about their hub in the
      material's world position offset, translucent) and the blur disc (translucent, fading to the tips)
  BP_CCDrone_<Airframe>          a movable actor: the body, four spinning propellers and their blur discs, all
                                 NoCollision (spawned as blueprint.cc_drone_<airframe> through the content pack)
  BP_CCDrone_<Airframe>_Parked   the same on the ground: propellers still, no discs, lights dimmed, strobe off
  SM_CCDrone_<Airframe>          (the _Full mesh, renamed) for drones.py's `mesh` look (static.prop.mesh + mass)
Prints the imported meshes' bounds so the axis convention can be checked (x forward, y right, z up in Unreal).
"""
import argparse
import json
import os
import shlex
import sys

import unreal

DEST = "/Game/CoupledCarla/Drones"
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
AIRFRAMES = ("Consumer", "Industrial")
# drone_mesh.py writes the model's (x forward, y left, z up) so that Unreal's glTF import gives x forward, y right,
# z up (checked with SM_CCDrone_AxisCheck below): Unreal = (x, -y, z) * 100.
UE_FROM_MODEL = (1.0, -1.0, 1.0)


def log(msg):
    unreal.log_warning("[drones] " + str(msg))     # a commandlet's log keeps warnings, not LogPython's displays


def tools():
    return unreal.AssetToolsHelpers.get_asset_tools()


def load(path):
    return EAL.load_asset(path) if EAL.does_asset_exist(path) else None


# -- meshes ----------------------------------------------------------------------------------------------------

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
        mat = pipe.get_editor_property("material_pipeline")
        mat.set_editor_property("import_materials", False)
        common = pipe.get_editor_property("common_meshes_properties")
        try:
            common.set_editor_property("force_all_mesh_as_type", unreal.InterchangeForceMeshType.IFMT_STATIC_MESH)
        except Exception:
            pass
        t.set_editor_property("options", pipe)
    except Exception as exc:  # noqa: BLE001 - older engines: the default pipeline
        log("interchange options: %s" % exc)
    tools().import_asset_tasks([t])
    got = [str(p) for p in (t.get_editor_property("imported_object_paths") or [])]
    meshes = [unreal.load_object(None, p) for p in got]
    meshes = [m for m in meshes if isinstance(m, unreal.StaticMesh)]
    if not meshes:     # Interchange may prefix the name (SM_SM_...): look for it
        found = [p for p in EAL.list_assets(DEST, recursive=False) if name in p.split("/")[-1]]
        meshes = [m for m in (EAL.load_asset(p.split(".")[0]) for p in found) if isinstance(m, unreal.StaticMesh)]
    if not meshes:
        raise RuntimeError("import of %s gave nothing (%s)" % (src, got))
    mesh = meshes[0]
    path = mesh.get_path_name().split(".")[0]
    if path != "%s/%s" % (DEST, name):      # our name, whatever Interchange called it
        if EAL.does_asset_exist("%s/%s" % (DEST, name)):
            EAL.delete_asset("%s/%s" % (DEST, name))
        EAL.rename_asset(path, "%s/%s" % (DEST, name))
        mesh = EAL.load_asset("%s/%s" % (DEST, name))
        log("%s imported as %s, renamed" % (name, path))
    try:
        mesh.set_editor_property("nanite_settings", unreal.MeshNaniteSettings(enabled=False))
    except Exception:
        pass
    bb = mesh.get_bounding_box()
    slots = [str(m.get_editor_property("material_slot_name")) for m in mesh.get_editor_property("static_materials")]
    log("%s: bounds min %s max %s, slots %s" % (name, _v(bb.min), _v(bb.max), slots))
    return mesh


def _v(v):
    return "(%.1f, %.1f, %.1f)" % (v.x, v.y, v.z)


# -- materials -------------------------------------------------------------------------------------------------

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


def link(a, out, b, inp):
    if not MEL.connect_material_expressions(a, out, b, inp):
        log("could not connect %s.%s -> %s.%s" % (a.get_name(), out, b.get_name(), inp))


def prop(e, out, m, p):
    if not MEL.connect_material_property(e, out, p):
        log("could not connect %s.%s -> %s" % (e.get_name(), out, p))


def vparam(m, name, rgb, x, y):
    return expr(m, unreal.MaterialExpressionVectorParameter, x, y, parameter_name=name,
                default_value=unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))


def sparam(m, name, v, x, y):
    return expr(m, unreal.MaterialExpressionScalarParameter, x, y, parameter_name=name, default_value=float(v))


def m_lit():
    m = new_material("M_CCDrone_Lit")
    mp = unreal.MaterialProperty
    prop(vparam(m, "BaseColor", (0.8, 0.8, 0.8), -400, 0), "", m, mp.MP_BASE_COLOR)
    prop(sparam(m, "Metallic", 0.0, -400, 150), "", m, mp.MP_METALLIC)
    prop(sparam(m, "Roughness", 0.4, -400, 250), "", m, mp.MP_ROUGHNESS)
    prop(sparam(m, "Specular", 0.5, -400, 350), "", m, mp.MP_SPECULAR)
    MEL.recompile_material(m)
    return m


def m_light():
    """Emissive light: Color x Intensity; with Strobe 1 only during the first Duty of every 1/Rate seconds, phase
    from the object's position so that drones do not flash together."""
    m = new_material("M_CCDrone_Light")
    mp = unreal.MaterialProperty
    col = vparam(m, "Color", (1.0, 0.05, 0.02), -900, 0)
    inten = sparam(m, "Intensity", 60.0, -900, 120)
    strobe = sparam(m, "Strobe", 0.0, -900, 220)
    rate = sparam(m, "Rate", 1.0, -900, 320)
    duty = sparam(m, "Duty", 0.06, -900, 420)
    time = expr(m, unreal.MaterialExpressionTime, -1100, 520)
    opos = expr(m, unreal.MaterialExpressionObjectPositionWS, -1100, 620)
    mask = expr(m, unreal.MaterialExpressionComponentMask, -950, 620, r=True, g=True, b=False, a=False)
    link(opos, "", mask, "")
    dot = expr(m, unreal.MaterialExpressionDotProduct, -800, 620)
    link(mask, "", dot, "A")
    k = expr(m, unreal.MaterialExpressionConstant2Vector, -950, 720, r=0.0007, g=0.0011)
    link(k, "", dot, "B")
    tr = expr(m, unreal.MaterialExpressionMultiply, -800, 520)
    link(time, "", tr, "A")
    link(rate, "", tr, "B")
    ph = expr(m, unreal.MaterialExpressionAdd, -650, 560)
    link(tr, "", ph, "A")
    link(dot, "", ph, "B")
    fr = expr(m, unreal.MaterialExpressionFrac, -520, 560)
    link(ph, "", fr, "")
    flash = expr(m, unreal.MaterialExpressionIf, -380, 480, const_b=0.06)
    link(fr, "", flash, "A")
    link(duty, "", flash, "B")
    one = expr(m, unreal.MaterialExpressionConstant, -520, 400, r=1.0)
    zero = expr(m, unreal.MaterialExpressionConstant, -520, 460, r=0.0)
    link(zero, "", flash, "A > B")
    link(zero, "", flash, "A == B")
    link(one, "", flash, "A < B")
    lerp = expr(m, unreal.MaterialExpressionLinearInterpolate, -240, 400)
    link(one, "", lerp, "A")
    link(flash, "", lerp, "B")
    link(strobe, "", lerp, "Alpha")
    e1 = expr(m, unreal.MaterialExpressionMultiply, -600, 60)
    link(col, "", e1, "A")
    link(inten, "", e1, "B")
    e2 = expr(m, unreal.MaterialExpressionMultiply, -100, 200)
    link(e1, "", e2, "A")
    link(lerp, "", e2, "B")
    prop(e2, "", m, mp.MP_EMISSIVE_COLOR)
    base = expr(m, unreal.MaterialExpressionMultiply, -300, -100, const_b=0.3)
    link(col, "", base, "A")
    prop(base, "", m, mp.MP_BASE_COLOR)
    prop(expr(m, unreal.MaterialExpressionConstant, -300, -200, r=0.2), "", m, mp.MP_ROUGHNESS)
    MEL.recompile_material(m)
    return m


def m_propspin():
    """Translucent blades turned about their hub (the component's pivot) in the world position offset: RevPerSec
    turns a second times Spin (+1 counter-clockwise, -1 clockwise seen from above). The visual rate is lower than
    a real rotor's (100 rev/s would alias into a random flicker at 50 Hz): a camera-friendly turn inside the disc."""
    m = new_material("M_CCDrone_PropSpin")
    m.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
    m.set_editor_property("two_sided", True)
    mp = unreal.MaterialProperty
    prop(vparam(m, "BaseColor", (0.03, 0.03, 0.035), -400, -200), "", m, mp.MP_BASE_COLOR)
    prop(sparam(m, "Opacity", 0.45, -400, -100), "", m, mp.MP_OPACITY)
    prop(expr(m, unreal.MaterialExpressionConstant, -400, -300, r=0.5), "", m, mp.MP_ROUGHNESS)
    time = expr(m, unreal.MaterialExpressionTime, -1000, 100)
    rev = sparam(m, "RevPerSec", 4.7, -1000, 200)
    spin = sparam(m, "Spin", 1.0, -1000, 300)
    a1 = expr(m, unreal.MaterialExpressionMultiply, -850, 150)
    link(time, "", a1, "A")
    link(rev, "", a1, "B")
    a2 = expr(m, unreal.MaterialExpressionMultiply, -700, 200)
    link(a1, "", a2, "A")
    link(spin, "", a2, "B")
    up = expr(m, unreal.MaterialExpressionConstant3Vector, -1000, 400, constant=unreal.LinearColor(0, 0, 1, 0))
    tw = expr(m, unreal.MaterialExpressionTransform, -850, 400,
              transform_source_type=unreal.MaterialVectorCoordTransformSource.TRANSFORMSOURCE_LOCAL,
              transform_type=unreal.MaterialVectorCoordTransform.TRANSFORM_WORLD)
    link(up, "", tw, "")
    nrm = expr(m, unreal.MaterialExpressionNormalize, -700, 400)
    link(tw, "", nrm, "")
    rot = expr(m, unreal.MaterialExpressionRotateAboutAxis, -450, 300)
    link(nrm, "", rot, "NormalizedRotationAxis")
    link(a2, "", rot, "RotationAngle")
    link(expr(m, unreal.MaterialExpressionObjectPositionWS, -700, 520), "", rot, "PivotPoint")
    link(expr(m, unreal.MaterialExpressionWorldPosition, -700, 620), "", rot, "Position")
    prop(rot, "", m, mp.MP_WORLD_POSITION_OFFSET)
    MEL.recompile_material(m)
    return m


def m_disc():
    """The blur of spinning blades: translucent, darker near the hub, fading towards and at the tips."""
    m = new_material("M_CCDrone_Disc")
    m.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
    m.set_editor_property("two_sided", True)
    mp = unreal.MaterialProperty
    prop(vparam(m, "BaseColor", (0.04, 0.04, 0.045), -400, -200), "", m, mp.MP_BASE_COLOR)
    prop(expr(m, unreal.MaterialExpressionConstant, -400, -300, r=0.6), "", m, mp.MP_ROUGHNESS)
    uv = expr(m, unreal.MaterialExpressionTextureCoordinate, -1100, 0)
    c = expr(m, unreal.MaterialExpressionConstant2Vector, -1100, 100, r=0.5, g=0.5)
    d = expr(m, unreal.MaterialExpressionDistance, -950, 50)
    link(uv, "", d, "A")
    link(c, "", d, "B")
    r = expr(m, unreal.MaterialExpressionMultiply, -800, 50, const_b=2.0)
    link(d, "", r, "A")
    pw = expr(m, unreal.MaterialExpressionPower, -650, 50, const_exponent=1.5)
    link(r, "", pw, "Base")
    inv = expr(m, unreal.MaterialExpressionOneMinus, -520, 50)
    link(pw, "", inv, "")
    dens = sparam(m, "Density", 0.28, -520, 150)
    o1 = expr(m, unreal.MaterialExpressionMultiply, -380, 80)
    link(inv, "", o1, "A")
    link(dens, "", o1, "B")
    o2 = expr(m, unreal.MaterialExpressionAdd, -250, 80, const_b=0.04)
    link(o1, "", o2, "A")
    # edge: nothing outside r = 1 (the disc mesh is a little larger than the blades)
    edge = expr(m, unreal.MaterialExpressionIf, -250, 250, const_b=0.98)
    link(r, "", edge, "A")
    one = expr(m, unreal.MaterialExpressionConstant, -400, 300, r=1.0)
    zero = expr(m, unreal.MaterialExpressionConstant, -400, 360, r=0.0)
    link(zero, "", edge, "A > B")
    link(one, "", edge, "A == B")
    link(one, "", edge, "A < B")
    o3 = expr(m, unreal.MaterialExpressionMultiply, -100, 150)
    link(o2, "", o3, "A")
    link(edge, "", o3, "B")
    prop(o3, "", m, mp.MP_OPACITY)
    MEL.recompile_material(m)
    return m


def mi(name, parent, vectors=None, scalars=None):
    path = "%s/%s" % (DEST, name)
    inst = load(path) or tools().create_asset(name, DEST, unreal.MaterialInstanceConstant,
                                              unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(inst, parent)
    for k, v in (vectors or {}).items():
        MEL.set_material_instance_vector_parameter_value(inst, k, unreal.LinearColor(v[0], v[1], v[2], 1.0))
    for k, v in (scalars or {}).items():
        MEL.set_material_instance_scalar_parameter_value(inst, k, float(v))
    MEL.update_material_instance(inst)
    EAL.save_asset(path, only_if_is_dirty=False)
    return inst


def materials():
    lit, light, spin, disc = m_lit(), m_light(), m_propspin(), m_disc()
    for mm in (lit, light, spin, disc):
        EAL.save_asset(mm.get_path_name().split(".")[0], only_if_is_dirty=False)
    out = {
        "BodyLight": mi("MI_CCDrone_BodyLight", lit, {"BaseColor": (0.62, 0.63, 0.64)}, {"Roughness": 0.32,
                                                                                         "Specular": 0.55}),
        "BodyDark": mi("MI_CCDrone_BodyDark", lit, {"BaseColor": (0.045, 0.047, 0.05)}, {"Roughness": 0.4}),
        "Dark": mi("MI_CCDrone_Dark", lit, {"BaseColor": (0.025, 0.025, 0.028)}, {"Roughness": 0.55}),
        "Metal": mi("MI_CCDrone_Metal", lit, {"BaseColor": (0.12, 0.12, 0.13)}, {"Metallic": 1.0, "Roughness": 0.3}),
        "Carbon": mi("MI_CCDrone_Carbon", lit, {"BaseColor": (0.015, 0.015, 0.017)}, {"Roughness": 0.22,
                                                                                    "Specular": 0.7}),
        "Accent": mi("MI_CCDrone_Accent", lit, {"BaseColor": (0.85, 0.22, 0.01)}, {"Roughness": 0.35}),
        "Lens": mi("MI_CCDrone_Lens", lit, {"BaseColor": (0.004, 0.004, 0.006)}, {"Roughness": 0.04,
                                                                                "Specular": 1.0}),
        "Parcel": mi("MI_CCDrone_Parcel", lit, {"BaseColor": (0.42, 0.29, 0.15)}, {"Roughness": 0.85}),
        "PropStill": mi("MI_CCDrone_PropStill", lit, {"BaseColor": (0.03, 0.03, 0.033)}, {"Roughness": 0.45}),
        "NavRed": mi("MI_CCDrone_NavRed", light, {"Color": (1.0, 0.03, 0.01)}, {"Intensity": 80.0}),
        "NavGreen": mi("MI_CCDrone_NavGreen", light, {"Color": (0.02, 1.0, 0.06)}, {"Intensity": 80.0}),
        "Strobe": mi("MI_CCDrone_Strobe", light, {"Color": (1.0, 1.0, 1.0)},
                     {"Intensity": 1500.0, "Strobe": 1.0, "Rate": 1.0, "Duty": 0.06}),
        "NavRedDim": mi("MI_CCDrone_NavRedDim", light, {"Color": (1.0, 0.03, 0.01)}, {"Intensity": 12.0}),
        "NavGreenDim": mi("MI_CCDrone_NavGreenDim", light, {"Color": (0.02, 1.0, 0.06)}, {"Intensity": 12.0}),
        "StrobeOff": mi("MI_CCDrone_StrobeOff", light, {"Color": (1.0, 1.0, 1.0)}, {"Intensity": 0.0}),
        "PropCCW": mi("MI_CCDrone_PropCCW", spin, scalars={"Spin": 1.0}),
        "PropCW": mi("MI_CCDrone_PropCW", spin, scalars={"Spin": -1.0}),
        "Disc": mi("MI_CCDrone_Disc", disc),
    }
    return out


SLOT_MI = {"Body": None, "Dark": "Dark", "Metal": "Metal", "Carbon": "Carbon", "Accent": "Accent", "Lens": "Lens",
           "NavRed": "NavRed", "NavGreen": "NavGreen", "Strobe": "Strobe", "Prop": "PropStill", "Disc": "Disc",
           "Parcel": "Parcel"}


def assign(mesh, mats, body_key):
    for k, sm in enumerate(mesh.get_editor_property("static_materials")):
        slot = str(sm.get_editor_property("material_slot_name"))
        key = next((v for s, v in SLOT_MI.items() if slot.startswith(s)), None)
        key = body_key if slot.startswith("Body") else key
        if key is None:
            log("slot %s of %s has no material" % (slot, mesh.get_name()))
            continue
        mesh.set_material(k, mats[key])


# -- blueprints ------------------------------------------------------------------------------------------------

def new_blueprint(name):
    path = "%s/%s" % (DEST, name)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    f = unreal.BlueprintFactory()
    f.set_editor_property("parent_class", unreal.Actor)
    return tools().create_asset(name, DEST, unreal.Blueprint, f)


def add_mesh_component(sub, bp, parent, name, mesh, loc=(0, 0, 0), materials=None, shadows=True):
    params = unreal.AddNewSubobjectParams(parent_handle=parent, new_class=unreal.StaticMeshComponent,
                                          blueprint_context=bp)
    handle, fail = sub.add_new_subobject(params)
    if fail and str(fail):
        log("add %s: %s" % (name, fail))
    sub.rename_subobject(handle, unreal.Text(name))
    data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(handle)
    comp = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data)
    comp.set_editor_property("static_mesh", mesh)
    comp.set_editor_property("relative_location", unreal.Vector(*loc))
    comp.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    comp.set_collision_profile_name("NoCollision")
    comp.set_editor_property("generate_overlap_events", False)
    comp.set_editor_property("can_character_step_up_on", unreal.CanBeCharacterBase.ECB_NO)
    comp.set_editor_property("cast_shadow", shadows)
    for idx, m in (materials or {}).items():
        comp.set_material(idx, m)
    return handle


def build_blueprint(name, meshes, mats, props, parked, body_key):
    sub = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    bp = new_blueprint(name)
    handles = sub.k2_gather_subobject_data_for_blueprint(bp)
    root = handles[0]
    for h in handles:      # the default scene root (handles[0] is the actor itself): movable, the parent of all
        o = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(unreal.SubobjectDataBlueprintFunctionLibrary.get_data(h))
        if isinstance(o, unreal.SceneComponent):
            o.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
            root = h
            break
    body = meshes["Body"]
    over = {}
    if parked:      # lights dimmed, strobe off on the ground
        for slot, key in (("NavRed", "NavRedDim"), ("NavGreen", "NavGreenDim"), ("Strobe", "StrobeOff")):
            i = body.get_material_index(slot)
            if i >= 0:
                over[i] = mats[key]
    add_mesh_component(sub, bp, root, "Body", body, materials=over)
    for k, p in enumerate(props):
        loc = (p["x"] * 100 * UE_FROM_MODEL[0], p["y"] * 100 * UE_FROM_MODEL[1], p["z"] * 100 * UE_FROM_MODEL[2])
        pm = mats["PropStill"] if parked else (mats["PropCCW"] if p["spin"] < 0 else mats["PropCW"])
        # reaction torque +z (spin +1 in the model) means the rotor turns clockwise seen from above
        add_mesh_component(sub, bp, root, "Prop%d" % k, meshes["Prop"], loc, {0: pm})
        if not parked:
            add_mesh_component(sub, bp, root, "Disc%d" % k, meshes["Disc"], loc, {0: mats["Disc"]}, shadows=False)
    unreal.BlueprintEditorLibrary.compile_blueprint(bp)
    EAL.save_asset("%s/%s" % (DEST, name), only_if_is_dirty=False)
    log("%s: %d components" % (name, len(sub.k2_gather_subobject_data_for_blueprint(bp))))
    return bp


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="folder with tools/drone_mesh.py's output")
    a = ap.parse_args(argv)
    EAL.make_directory(DEST)
    info = json.load(open(os.path.join(a.src, "drone_props.json")))
    check = import_glb(os.path.join(a.src, "SM_CCDrone_AxisCheck.glb"), "SM_CCDrone_AxisCheck")
    c = check.get_bounding_box()
    centre = ((c.min.x + c.max.x) / 2, (c.min.y + c.max.y) / 2, (c.min.z + c.max.z) / 2)
    EAL.delete_asset(check.get_path_name().split(".")[0])
    ok = abs(centre[0] - 50) < 2 and abs(centre[1] + 20) < 2 and abs(centre[2] - 10) < 2
    log("axis check: a box at model (0.5, 0.2, 0.1) m is at Unreal (%.1f, %.1f, %.1f) cm: %s"
        % (centre + ("as expected (50, -20, 10)" if ok else "WRONG, expected (50, -20, 10)",)))
    if not ok:
        raise RuntimeError("glTF axes: fix drone_mesh.to_gltf_frame")
    mats = materials()
    for af in AIRFRAMES:
        meshes = {}
        for part in ("Body", "Prop", "Disc", "Full"):
            name = "SM_CCDrone_%s_%s" % (af, part) if part != "Full" else "SM_CCDrone_%s" % af
            meshes[part] = import_glb(os.path.join(a.src, "SM_CCDrone_%s_%s.glb" % (af, part)), name)
        body_key = "BodyLight" if af == "Consumer" else "BodyDark"
        for part, m in meshes.items():
            if part == "Prop":
                m.set_material(0, mats["PropStill"])
            elif part == "Disc":
                m.set_material(0, mats["Disc"])
            else:
                assign(m, mats, body_key)
            EAL.save_asset(m.get_path_name().split(".")[0], only_if_is_dirty=False)
        props = info[af]["props"]
        build_blueprint("BP_CCDrone_%s" % af, meshes, mats, props, False, body_key)
        build_blueprint("BP_CCDrone_%s_Parked" % af, meshes, mats, props, True, body_key)
    # Interchange makes placeholder materials for the glTF's slots, even with import_materials off: delete them
    for p in EAL.list_assets(DEST, recursive=True):
        n = p.split("/")[-1].split(".")[0]
        if not n.startswith(("M_CCDrone", "MI_CCDrone", "SM_CCDrone", "BP_CCDrone")):
            EAL.delete_asset(p.split(".")[0])
    unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
    log("done: %s" % ", ".join(sorted(p.split("/")[-1].split(".")[0] for p in EAL.list_assets(DEST))))


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 1 and " " in args[0]:
        args = shlex.split(args[0])
    main(args)
