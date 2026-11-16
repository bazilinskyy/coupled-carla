"""Street-light blueprints for the regional lamp meshes (streetlook.py), so at night the real lamp shapes give light:

    UnrealEditor-Cmd <tree>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \\
        -script="<repo>/tools/ue_lamp_lights.py --props <repo>/coupledcarla/data/street_props.json [--inspect] \\
                 [--report <file>]" -unattended -nosplash -nullrhi -nosound

For each regional lamp in data/street_props.json that lists a "light" (SM_CC_NL_LampArm9, ...), it duplicates CARLA's
lit lamp blueprint BP_StreetLight06_A (a static lamp mesh with a spot light and a CarlaLight component: the light
manager's street light, switched on at night) into /Game/CoupledCarla/Street/Lights/BP_CC_<Region>_<Name>, puts the
regional mesh in place of CARLA's, moves the spot light to the lamp's luminaire (the manifest's "light", metres,
Unreal axes) and adds a second one for a second luminaire ("lights": the double-arm mast). The lamp glass slot takes
CARLA's lamp glass material (MI_StreetLight_GlassInt; CARLA's separate glass meshes are dropped), whose emissive
the CarlaLight gates on at night;
the CarlaLight keeps CARLA's lamp emissive (BP_StreetLight06_A's Light Defaults: 100,000). The blueprints are listed in
the content pack (coupledcarla/data/carla_packs/CoupledCarla/Content/Config/BlueprintParameters.json) as
blueprint.cc_<region>_<name>, and season.LAMP_LIGHTS spawns them at night in place of the plain meshes.

--inspect only prints BP_StreetLight06_A's components (class, mesh, materials, light settings) to the log/report.
Cook /Game/CoupledCarla (DirectoriesToAlwaysCook, as for the street props) for a packaged server.
"""
import argparse
import json
import shlex
import sys

import unreal

SRC = "/Game/Carla/Static/StreetLight/BP_StreetLight06_A"
DST = "/Game/CoupledCarla/Street/Lights"
EAL = unreal.EditorAssetLibrary
SDS = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
LIB = unreal.SubobjectDataBlueprintFunctionLibrary
LAMP_EMISSIVE = 100000.0     # BP_StreetLight06_A's emissiveIntensity in CARLA's Light Defaults
GLASS_SLOT = "LampGlass"


def log(msg):
    unreal.log_warning("[lamplights] " + str(msg))


def components(bp):
    """[(handle, component template)] of a blueprint's components."""
    out = []
    for h in SDS.k2_gather_subobject_data_for_blueprint(bp):
        obj = LIB.get_object(LIB.get_data(h))
        if isinstance(obj, unreal.ActorComponent):
            out.append((h, obj))
    return out


def describe(comp):
    d = {"name": comp.get_name(), "class": comp.get_class().get_name()}
    if isinstance(comp, unreal.SceneComponent):
        loc, rot = comp.get_editor_property("relative_location"), comp.get_editor_property("relative_rotation")
        d.update(location=[round(loc.x, 2), round(loc.y, 2), round(loc.z, 2)],
                 rotation=[round(rot.pitch, 2), round(rot.yaw, 2), round(rot.roll, 2)],
                 mobility=str(comp.get_editor_property("mobility")))
    if isinstance(comp, unreal.StaticMeshComponent):
        sm = comp.get_editor_property("static_mesh")
        d["mesh"] = sm.get_path_name() if sm else None
        d["materials"] = [m.get_path_name() if m else None for m in comp.get_materials()]
        if sm:
            d["slots"] = [str(s.get_editor_property("material_slot_name"))
                          for s in sm.get_editor_property("static_materials")]
    if isinstance(comp, unreal.LocalLightComponent):
        for k in ("intensity", "intensity_units", "attenuation_radius", "light_color", "cast_shadows"):
            try:
                d[k] = str(comp.get_editor_property(k))
            except Exception:  # noqa: BLE001
                pass
        if isinstance(comp, unreal.SpotLightComponent):
            d["cone"] = [comp.get_editor_property("inner_cone_angle"), comp.get_editor_property("outer_cone_angle")]
    if comp.get_class().get_name() == "CarlaLight":
        for k in ("light_intensity", "emissive_intensity", "light_type", "light_color"):
            try:
                d[k] = str(comp.get_editor_property(k))
            except Exception:  # noqa: BLE001
                pass
    return d


def make(name, spec, src):
    """BP_CC_<...> for the regional lamp `name` (manifest spec); returns its report."""
    bp_name = "BP_" + name[3:]       # SM_CC_NL_LampArm9 -> BP_CC_NL_LampArm9
    path = "%s/%s" % (DST, bp_name)
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    bp = EAL.duplicate_asset(SRC, path)
    mesh = EAL.load_asset(spec["path"].split(".")[0])
    comps = components(bp)
    meshes = [(h, c) for h, c in comps if isinstance(c, unreal.StaticMeshComponent)]
    spots = [(h, c) for h, c in comps if isinstance(c, unreal.LocalLightComponent)]
    carla = [c for _, c in comps if not isinstance(c, unreal.SceneComponent)]      # BP_Lights (a CarlaLight)
    roots = [(h, c) for h, c in meshes if c.get_name().startswith("StreetLight_Mesh")]
    if not roots or not spots:
        raise RuntimeError("%s: no StreetLight_Mesh or light component in %s" % (bp_name, SRC))
    root_h, root = roots[0]
    glass = None
    for _, c in meshes:     # CARLA's lamp glass (GlassInt/GlassExt meshes): hidden; its material lights our glass
        if c is root or c.get_name() == root.get_name():
            continue
        mats = c.get_materials()
        if mats and mats[0] and (glass is None or "GlassInt" in mats[0].get_name()):
            glass = mats[0]
        c.set_editor_property("static_mesh", None)
    root.set_editor_property("static_mesh", mesh)
    slots = [str(s_.get_editor_property("material_slot_name"))
             for s_ in mesh.get_editor_property("static_materials")]
    mats = [None] * len(slots)
    if glass is not None and GLASS_SLOT in slots:
        mats[slots.index(GLASS_SLOT)] = glass
    root.set_editor_property("override_materials", mats)
    pos = spec.get("lights") or [spec["light"]]
    first = spots[0][1]
    for _, other in spots:     # the template is listed twice: one light component
        if other.get_name() != first.get_name():
            raise RuntimeError("%s: %d lights in %s" % (bp_name, len(spots), SRC))
    first.set_editor_property("relative_location", unreal.Vector(pos[0][0] * 100, pos[0][1] * 100, pos[0][2] * 100))
    for p in pos[1:]:       # a second luminaire: a copy of the first light
        params = unreal.AddNewSubobjectParams(parent_handle=root_h, new_class=type(first).static_class(),
                                              blueprint_context=bp)
        h, fail = SDS.add_new_subobject(params)
        if not LIB.is_handle_valid(h):
            raise RuntimeError("%s: second light not added (%s)" % (bp_name, fail))
        c = LIB.get_object(LIB.get_data(h))
        for k in ("intensity", "intensity_units", "attenuation_radius", "light_color", "cast_shadows",
                  "relative_rotation", "mobility", "inner_cone_angle", "outer_cone_angle", "source_radius"):
            try:
                c.set_editor_property(k, first.get_editor_property(k))
            except Exception:  # noqa: BLE001
                pass
        c.set_editor_property("relative_location", unreal.Vector(p[0] * 100, p[1] * 100, p[2] * 100))
    for c in carla:
        for k in ("emissive_intensity", "EmissiveIntensity"):
            try:
                c.set_editor_property(k, LAMP_EMISSIVE)
                break
            except Exception:  # noqa: BLE001
                pass
    unreal.BlueprintEditorLibrary.compile_blueprint(bp)
    EAL.save_asset(path, only_if_is_dirty=False)
    rep = {"blueprint": path, "mesh": spec["path"], "lights": pos, "glass": glass.get_path_name() if glass else None,
           "components": [describe(c) for _, c in components(bp)]}
    log(json.dumps(rep))
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--props", required=True)
    ap.add_argument("--inspect", action="store_true")
    ap.add_argument("--report")
    ap.add_argument("--only")
    a = ap.parse_args(shlex.split(" ".join(sys.argv[1:])))
    src = EAL.load_asset(SRC)
    out = {"source": SRC, "source_components": [describe(c) for _, c in components(src)], "made": []}
    log(json.dumps(out["source_components"]))
    if not a.inspect:
        specs = json.load(open(a.props))["meshes"]
        for name, spec in sorted(specs.items()):
            if not spec.get("light") or (a.only and name != a.only):
                continue
            if not EAL.does_asset_exist(spec["path"].split(".")[0]):
                log("%s: mesh not in this tree, skipped" % name)
                continue
            try:
                out["made"].append(make(name, spec, src))
            except Exception as err:  # noqa: BLE001
                log("%s: FAILED %s" % (name, err))
                out["made"].append({"name": name, "error": str(err)})
    if a.report:
        with open(a.report, "w") as f:
            json.dump(out, f, indent=1, default=str)
    log("DONE")


main()
