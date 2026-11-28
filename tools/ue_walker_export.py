"""In-editor helper for importing walkers (README "Who the pedestrians are", coverage gaps); runs inside a FULL
UnrealEditor via -ExecutePythonScript (skeletal-mesh and animation FBX export assert in a commandlet; see the CARLA
repo skill Skills/ue58/import-carla-ue58-walker). Reads a job JSON (CC_WALKER_JOB):

  {"result": "<json>", "export": [{"asset": "/Game/...", "fbx": "<path>"}, ...],
   "inspect": ["/Game/Carla/Blueprints/Walkers/BP_..."], "list": ["/Game/Carla/Static/Pedestrian/Animations"],
   "export_matching": [{"path": <a listed path>, "contains": "walk", "skeleton": "/Game/...", "dir": "<dir>"}],
   "register": [{"mesh": "/Game/...SK" | "fbx": "<fbx>", "dest": "/Game/...", "skeleton": "/Game/...Skel",
                 "donor": "/Game/...BP", "blueprint": "/Game/...BP_new", "capsule_from": "/Game/...BP" | null,
                 "scale": 1.0}]}

export: SkeletalMesh or AnimSequence to FBX. inspect: a walker blueprint's mesh, skeleton, anim class, capsule and
mesh offset. list: assets under a path with their class and skeleton. register: duplicates a donor walker blueprint
(the skill's method: no recompile) and points it at an existing SkeletalMesh, optionally copying the capsule size
and mesh offset of another blueprint (an adult mesh on a child donor). Writes the result JSON after every step.
"""
import json
import os
import traceback

import unreal

JOB = json.load(open(os.environ["CC_WALKER_JOB"]))
state = {"steps": [], "exported": [], "inspected": {}, "listed": {}, "registered": []}


def mark(step, **kw):
    state["steps"].append(dict(step=step, **kw))
    with open(JOB["result"], "w") as f:
        json.dump(state, f, indent=2)


def _cdo(bp_path):
    bp = unreal.load_asset(bp_path)
    return bp, unreal.get_default_object(bp.generated_class())


def _comp(cdo, names):
    for n in names:
        try:
            c = cdo.get_editor_property(n)
        except Exception:
            continue
        if c:
            return n, c
    return None, None


def do_export(item):
    obj = unreal.load_asset(item["asset"])
    if obj is None:
        raise RuntimeError("cannot load " + item["asset"])
    task = unreal.AssetExportTask()
    task.set_editor_property("object", obj)
    task.set_editor_property("filename", item["fbx"])
    task.set_editor_property("automated", True)
    task.set_editor_property("prompt", False)
    task.set_editor_property("replace_identical", True)
    opt = unreal.FbxExportOption()
    opt.set_editor_property("collision", False)
    opt.set_editor_property("level_of_detail", False)
    task.set_editor_property("options", opt)
    ok = bool(unreal.Exporter.run_asset_export_task(task))
    sk = None
    try:
        s = obj.get_editor_property("skeleton")
        sk = str(s.get_path_name()) if s else None
    except Exception:
        pass
    state["exported"].append({"asset": item["asset"], "fbx": item["fbx"], "ok": ok, "class": type(obj).__name__,
                              "skeleton": sk, "bytes": os.path.getsize(item["fbx"]) if os.path.isfile(item["fbx"]) else 0})
    mark("exported", asset=item["asset"])


def do_inspect(bp_path):
    bp, cdo = _cdo(bp_path)
    info = {"cdo_class": type(cdo).__name__}
    n, mesh = _comp(cdo, ("mesh", "Mesh", "skeletal_mesh_component"))
    if mesh:
        sm = mesh.get_editor_property("skeletal_mesh")
        info["mesh"] = str(sm.get_path_name()) if sm else None
        sk = sm.get_editor_property("skeleton") if sm else None
        info["skeleton"] = str(sk.get_path_name()) if sk else None
        loc = mesh.get_editor_property("relative_location")
        rot = mesh.get_editor_property("relative_rotation")
        info["mesh_location"] = [loc.x, loc.y, loc.z]
        info["mesh_rotation"] = [rot.pitch, rot.yaw, rot.roll]
        try:
            ac = mesh.get_editor_property("anim_class")
            info["anim_class"] = str(ac.get_path_name()) if ac else None
        except Exception as e:
            info["anim_class"] = "?" + str(e)[:60]
        if sm:
            ext = sm.get_bounds().box_extent
            info["mesh_size_m"] = [round(2 * ext.x / 100, 3), round(2 * ext.y / 100, 3), round(2 * ext.z / 100, 3)]
    n, cap = _comp(cdo, ("capsule_component", "CapsuleComponent"))
    if cap:
        info["capsule"] = [cap.get_editor_property("capsule_radius"), cap.get_editor_property("capsule_half_height")]
    state["inspected"][bp_path] = info
    mark("inspected", bp=bp_path)


def do_list(path):
    out = []
    for p in unreal.EditorAssetLibrary.list_assets(path, recursive=True, include_folder=False):
        data = unreal.EditorAssetLibrary.find_asset_data(p)
        cls = str(data.asset_class_path.asset_name) if hasattr(data, "asset_class_path") else str(data.asset_class)
        rec = {"path": str(p), "class": cls}
        if cls in ("AnimSequence", "SkeletalMesh", "AnimBlueprint", "BlendSpace"):
            try:
                a = unreal.load_asset(p)
                s = a.get_editor_property("skeleton") if cls != "AnimBlueprint" else a.get_editor_property("target_skeleton")
                rec["skeleton"] = str(s.get_path_name()) if s else None
            except Exception:
                pass
        out.append(rec)
    state["listed"][path] = out
    mark("listed", path=path, n=len(out))


def do_dump_bones(item):
    """A skeletal mesh's reference skeleton (name, parent, component-space and local transforms, UE cm) to JSON,
    through the SkeletonModifier (works in a commandlet, where FBX export of a skeletal mesh asserts)."""
    mesh = unreal.load_asset(item["mesh"])
    mod = unreal.SkeletonModifier()
    mod.set_skeletal_mesh(mesh)
    bones = []
    for n in mod.get_all_bone_names():
        n = str(n)
        g = mod.get_bone_transform(n, True)
        l = mod.get_bone_transform(n, False)
        def tr(x):
            q = x.rotation
            return {"t": [x.translation.x, x.translation.y, x.translation.z], "q": [q.x, q.y, q.z, q.w],
                    "s": [x.scale3d.x, x.scale3d.y, x.scale3d.z]}
        bones.append({"name": n, "parent": str(mod.get_parent_name(n)), "global": tr(g), "local": tr(l)})
    with open(item["json"], "w") as f:
        json.dump({"mesh": item["mesh"], "bones": bones}, f, indent=1)
    mark("dumped_bones", mesh=item["mesh"], n=len(bones))


def do_import(item):
    """The skill's import (bound to the SHARED skeleton, or the walker will not animate), with the FBX's embedded
    textures imported too: {"fbx", "dest", "skeleton"}; returns the mesh path."""
    skeleton = unreal.load_asset(item["skeleton"])
    if skeleton is None:
        raise RuntimeError("skeleton not found: " + item["skeleton"])
    opt = unreal.FbxImportUI()
    opt.set_editor_property("import_mesh", True)
    opt.set_editor_property("import_as_skeletal", True)
    opt.set_editor_property("import_materials", True)
    opt.set_editor_property("import_textures", True)
    opt.set_editor_property("import_animations", False)
    opt.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_SKELETAL_MESH)
    opt.set_editor_property("skeleton", skeleton)
    data = opt.skeletal_mesh_import_data
    data.set_editor_property("import_morph_targets", False)
    data.set_editor_property("update_skeleton_reference_pose", False)
    data.set_editor_property("use_t0_as_ref_pose", False)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", item["fbx"])
    task.set_editor_property("destination_path", item["dest"])
    task.set_editor_property("automated", True)
    task.set_editor_property("save", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("options", opt)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    mesh_path, bound = None, None
    created = []
    for path in unreal.EditorAssetLibrary.list_assets(item["dest"], recursive=True, include_folder=False):
        asset = unreal.load_asset(path)
        created.append(type(asset).__name__)
        if isinstance(asset, unreal.SkeletalMesh) and mesh_path is None:
            mesh_path = str(path).split(".")[0]
            b = asset.get_editor_property("skeleton")
            bound = str(b.get_path_name()) if b else None
            ext = asset.get_bounds().box_extent
            size = [round(2 * ext.x / 100, 3), round(2 * ext.y / 100, 3), round(2 * ext.z / 100, 3)]
    rec = {"fbx": item["fbx"], "mesh": mesh_path, "skeleton": bound, "shared": bound == item["skeleton"],
           "created": sorted(set(created)), "size_m": size if mesh_path else None}
    state.setdefault("imported", []).append(rec)
    mark("imported", **{k: rec[k] for k in ("fbx", "mesh", "shared")})
    if not rec["shared"]:
        raise RuntimeError("mesh bound to {} instead of the shared {}: it would not animate".format(bound, item["skeleton"]))
    return mesh_path


def do_register(item):
    new = item["blueprint"]
    # a blueprint made by an earlier run is edited in place (deleting and duplicating again fails in a commandlet)
    if not unreal.EditorAssetLibrary.does_asset_exist(new) and \
            not unreal.EditorAssetLibrary.duplicate_asset(item["donor"], new):
        raise RuntimeError("duplicate_asset failed: " + item["donor"])
    bp, cdo = _cdo(new)
    n, mesh = _comp(cdo, ("mesh", "Mesh", "skeletal_mesh_component"))
    if item.get("fbx"):
        item["mesh"] = do_import(item)
    sm = unreal.load_asset(item["mesh"])
    if mesh is None or sm is None:
        raise RuntimeError("no mesh component or mesh " + item["mesh"])
    mesh.set_editor_property("skeletal_mesh", sm)
    rec = {"blueprint": new, "mesh": item["mesh"]}
    if item.get("capsule_from"):
        _, src = _cdo(item["capsule_from"])
        _, scap = _comp(src, ("capsule_component", "CapsuleComponent"))
        _, smesh = _comp(src, ("mesh", "Mesh", "skeletal_mesh_component"))
        _, cap = _comp(cdo, ("capsule_component", "CapsuleComponent"))
        cap.set_editor_property("capsule_radius", scap.get_editor_property("capsule_radius"))
        cap.set_editor_property("capsule_half_height", scap.get_editor_property("capsule_half_height"))
        mesh.set_editor_property("relative_location", smesh.get_editor_property("relative_location"))
        rec["capsule"] = [cap.get_editor_property("capsule_radius"), cap.get_editor_property("capsule_half_height")]
    # size: the generation-3 donors are CHILD blueprints that shrink the (adult-proportioned) mesh with the mesh
    # component's scale; set the walker's own: scale s (a mesh made larger to fit the skeleton's legs is shown at its
    # own size), a capsule as tall as the scaled mesh, the feet at the capsule's bottom
    s = float(item.get("scale", 1.0))
    _, cap = _comp(cdo, ("capsule_component", "CapsuleComponent"))
    donor_scale = mesh.get_editor_property("relative_scale3d")
    rec["donor_scale"] = [donor_scale.x, donor_scale.y, donor_scale.z]
    ext = sm.get_bounds().box_extent              # half sizes, cm, the mesh's rest pose
    hh = ext.z * s
    loc = mesh.get_editor_property("relative_location")
    mesh.set_editor_property("relative_scale3d", unreal.Vector(s, s, s))
    cap.set_editor_property("capsule_half_height", hh)
    mesh.set_editor_property("relative_location", unreal.Vector(loc.x, loc.y, -hh))
    rec["scale"], rec["capsule_half_height"], rec["height_cm"] = s, hh, 2 * hh
    unreal.EditorAssetLibrary.save_asset(new, only_if_is_dirty=False)
    rec["blueprint_class"] = "{}.{}_C".format(new, new.rsplit("/", 1)[1])
    state["registered"].append(rec)
    mark("registered", blueprint=new)


try:
    mark("start", engine=str(unreal.SystemLibrary.get_engine_version()))
    for p in JOB.get("list", []):
        do_list(p)
    for b in JOB.get("inspect", []):
        do_inspect(b)
    for it in JOB.get("dump_bones", []):
        do_dump_bones(it)
    for it in JOB.get("export", []):
        do_export(it)
    for m in JOB.get("export_matching", []):       # e.g. the walk animations of a skeleton
        for rec in state["listed"].get(m["path"], []):
            if rec["class"] == m.get("class", "AnimSequence") and m["contains"].lower() in rec["path"].lower() \
                    and (not m.get("skeleton") or rec.get("skeleton") == m["skeleton"]):
                name = rec["path"].rsplit("/", 1)[1].split(".")[0]
                do_export({"asset": rec["path"], "fbx": os.path.join(m["dir"], name + ".fbx")})
    for path in JOB.get("delete", []):              # e.g. a test walker's mesh folder and blueprint
        if unreal.EditorAssetLibrary.does_directory_exist(path):
            unreal.EditorAssetLibrary.delete_directory(path)
        elif unreal.EditorAssetLibrary.does_asset_exist(path):
            unreal.EditorAssetLibrary.delete_asset(path)
        mark("deleted", path=path)
    for it in JOB.get("register", []):
        do_register(it)
    mark("done")
except Exception:
    state["error"] = traceback.format_exc()
    mark("ERROR")
finally:
    if JOB.get("quit", True):
        try:
            unreal.SystemLibrary.quit_editor()
        except Exception:
            unreal.SystemLibrary.execute_console_command(None, "QUIT_EDITOR")
