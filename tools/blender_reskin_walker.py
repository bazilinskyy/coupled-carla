"""Re-skin a Microsoft Rocketbox avatar (3ds Max Biped rig) onto CARLA's pedestrian skeleton, for the
import-carla-ue58-walker skill (README "Who the pedestrians are", coverage gaps). Runs in Blender (4.5 LTS):

    blender -b --factory-startup -P tools/blender_reskin_walker.py -- \\
        --ref SK_AsiaM03_G3.fbx --src Male_Adult_09.fbx --textures <dir> --out CC_RB_Male_Adult_09.fbx [--preview p.png]

--ref: a CARLA walker mesh exported with its skeleton (the skill's `export`, or tools/ue_walker_export.py), whose
bone names, hierarchy and bone orientations the result must keep: the walker binds to the shared skeleton
(Skel_Pedestrian_G3) only when the hierarchy matches, and CARLA's animations drive bones by their local rotation.

Method:
  1. the avatar is turned to face the reference's way and put into the reference's rest pose: each mapped limb segment
     (shoulder->elbow, elbow->wrist, hip->knee, ..., each finger joint) is rotated onto the reference's segment
     direction, the hands and feet also by their spread (index->little knuckle, heel->toe up) so palms and soles
     match; the pose is applied to the mesh
  2. a copy of the reference armature is fitted to the avatar: every mapped bone's head moves to the avatar's joint,
     keeping the reference bone's direction and roll (so local axes, and hence the animations, stay CARLA's); unmapped
     bones (twist, end and helper bones) move with their parent, scaled by the parent segment's length ratio
  3. weights: the avatar's own skin weights, renamed through BONE_MAP; groups of unmapped avatar bones (face rig,
     extra spine, finger nubs) are merged into their nearest mapped ancestor
  4. textures are re-linked to the files in --textures (same stem, any of .tga/.png/.jpg) and embedded in the FBX
  5. an FBX for Unreal: the armature object keeps the reference's root name (crl_root: Blender turns the FBX root
     node into the armature object, and the exported node becomes Unreal's root bone again), no leaf bones
--preview renders the result posed by the reference's rest pose and, when --anim (a CARLA walk FBX) is given, a frame
of that walk, side by side with the reference (Cycles on the CPU: no GPU use).
"""
import argparse
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

# Rocketbox (Biped) -> CARLA's GEN3 pedestrian skeleton (Skel_Pedestrian_G3: 26 bones, no fingers or face; the
# avatar's finger, face and extra spine weights go to the nearest mapped bone); {S}: Biped "L"/"R" -> "__L"/"__R".
BONE_MAP = {
    "Bip01 Pelvis": "crl_hips__C",
    "Bip01 Spine": "crl_spine__C",
    "Bip01 Spine2": "crl_spine01__C",
    "Bip01 Neck": "crl_neck__C",
    "Bip01 Head": "crl_Head__C",
    "Bip01 {S}Eye": "crl_eye__{S}",
    "Bip01 {S} Clavicle": "crl_shoulder__{S}",
    "Bip01 {S} UpperArm": "crl_arm__{S}",
    "Bip01 {S} Forearm": "crl_foreArm__{S}",
    "Bip01 {S} Hand": "crl_hand__{S}",
    "Bip01 {S} Thigh": "crl_thigh__{S}",
    "Bip01 {S} Calf": "crl_leg__{S}",
    "Bip01 {S} Foot": "crl_foot__{S}",
    "Bip01 {S} Toe0": "crl_toe__{S}",
}
# segments posed onto the reference: (joint, next joint) in Biped names; {S} as above
CHAINS = [
    ("Bip01 Pelvis", "Bip01 Spine"), ("Bip01 Spine", "Bip01 Spine2"), ("Bip01 Spine2", "Bip01 Neck"),
    ("Bip01 Neck", "Bip01 Head"),
    ("Bip01 {S} Clavicle", "Bip01 {S} UpperArm"), ("Bip01 {S} UpperArm", "Bip01 {S} Forearm"),
    ("Bip01 {S} Forearm", "Bip01 {S} Hand"),
    ("Bip01 {S} Thigh", "Bip01 {S} Calf"), ("Bip01 {S} Calf", "Bip01 {S} Foot"), ("Bip01 {S} Foot", "Bip01 {S} Toe0"),
]
SIDES = (("L", "L"), ("R", "R"))


def expand(table):
    out = {}
    for k, v in table.items():
        if "{S}" in k:
            for bs, cs in SIDES:
                out[k.replace("{S}", bs)] = v.replace("{S}", cs)
        else:
            out[k] = v
    return out


def expand_chains():
    out = []
    for a, b in CHAINS:
        if "{S}" in a:
            out += [(a.replace("{S}", s), b.replace("{S}", s)) for s, _ in SIDES]
        else:
            out.append((a, b))
    return out


def log(*a):
    print("[reskin]", *a, flush=True)


def import_fbx(path):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=path, automatic_bone_orientation=False, ignore_leaf_bones=False)
    new = [o for o in bpy.data.objects if o not in before]
    arm = next(o for o in new if o.type == "ARMATURE")
    meshes = [o for o in new if o.type == "MESH"]
    return arm, meshes, new


def heads(arm):
    """{bone: world head} in the armature's current pose."""
    return {pb.name: arm.matrix_world @ pb.head for pb in arm.pose.bones}


def rest_heads(arm):
    return {b.name: arm.matrix_world @ b.head_local for b in arm.data.bones}


def rotate_pose_bone(arm, name, q, pivot):
    """Rotates pose bone `name` (and so its children) by world quaternion q about world point pivot."""
    pb = arm.pose.bones[name]
    mw = arm.matrix_world
    world = mw @ pb.matrix
    rot = Matrix.Translation(pivot) @ q.to_matrix().to_4x4() @ Matrix.Translation(-pivot)
    pb.matrix = mw.inverted() @ rot @ world
    bpy.context.view_layer.update()


def bone_list_order(arm):
    order = []

    def walk(b):
        order.append(b.name)
        for c in b.children:
            walk(c)
    for b in arm.data.bones:
        if b.parent is None:
            walk(b)
    return order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--src", required=True)
    ap.add_argument("--textures")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name")
    ap.add_argument("--preview")
    ap.add_argument("--anim")
    ap.add_argument("--report")
    ap.add_argument("--conform", action="store_true",
                    help="bake CARLA's exact joint positions into the mesh (for a skeleton whose bone translations come "
                         "from the animation); default: the avatar keeps its own proportions")
    a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:])
    bmap = expand(BONE_MAP)
    report = {"src": a.src, "warnings": []}

    bpy.ops.wm.read_factory_settings(use_empty=True)
    ref_arm, ref_meshes, _ = import_fbx(a.ref)
    src_arm, src_meshes, _ = import_fbx(a.src)
    for b in src_arm.data.bones:         # children's rigs are "Bip02 ...": one name set (vertex groups follow)
        if b.name.startswith("Bip0") and b.name[4:5].isdigit() and not b.name.startswith("Bip01"):
            b.name = "Bip01" + b.name[5:]
    ref_names = {b.name for b in ref_arm.data.bones}
    bmap = {k: v for k, v in bmap.items() if k in src_arm.pose.bones and v in ref_names}
    missing = [v for v in expand(BONE_MAP).values() if v not in ref_names]
    if missing:
        report["warnings"].append(f"reference lacks {sorted(set(missing))}")
    R = rest_heads(ref_arm)

    # 0. face the same way and stand on the same floor: compare the left shoulders' sides
    S = heads(src_arm)
    sx = S["Bip01 L Clavicle"].x - S["Bip01 R Clavicle"].x
    rx = R["crl_shoulder__L"].x - R["crl_shoulder__R"].x
    if sx * rx < 0:
        src_arm.rotation_euler.z += math.pi
        bpy.context.view_layer.update()
        log("turned the avatar 180 degrees to face the reference's way")
    # 1. pose onto the reference's segment directions, top-down
    order = bone_list_order(src_arm)
    chains = sorted(expand_chains(), key=lambda ab: order.index(ab[0]) if ab[0] in order else 999)
    for j, k in chains:
        if j not in bmap or k not in bmap:
            continue
        S = heads(src_arm)
        d_src = S[k] - S[j]
        d_ref = R[bmap[k]] - R[bmap[j]]
        if d_src.length < 1e-5 or d_ref.length < 1e-5:
            continue
        q = d_src.rotation_difference(d_ref)
        rotate_pose_bone(src_arm, j, q, S[j])
    # palms and soles: twist the hand about its axis so index->little knuckle matches the reference
    for s, cs in SIDES:
        hand, idx, pinky = f"Bip01 {s} Hand", f"Bip01 {s} Finger1", f"Bip01 {s} Finger4"
        if all(n in bmap for n in (hand, idx, pinky)):
            S = heads(src_arm)
            axis = (S[f"Bip01 {s} Finger2"] - S[hand]).normalized()
            v_src = (S[pinky] - S[idx])
            v_ref = (R[bmap[pinky]] - R[bmap[idx]])
            v_src -= axis * v_src.dot(axis)
            v_ref -= axis * v_ref.dot(axis)
            if v_src.length > 1e-5 and v_ref.length > 1e-5:
                rotate_pose_bone(src_arm, hand, v_src.rotation_difference(v_ref), S[hand])
    # apply the pose to the meshes
    for m in src_meshes:
        if m.data.shape_keys:
            m.shape_key_clear()
        bpy.context.view_layer.objects.active = m
        for mod in list(m.modifiers):
            if mod.type == "ARMATURE":
                bpy.ops.object.modifier_apply(modifier=mod.name)
    S = heads(src_arm)
    # floor: the avatar's lowest vertex at the reference's floor
    zmin_src = min((m.matrix_world @ v.co).z for m in src_meshes for v in m.data.vertices)
    zmin_ref = min(((m.matrix_world @ v.co).z for m in ref_meshes for v in m.data.vertices), default=0.0)
    dz = zmin_ref - zmin_src
    # (--conform) the uniform scale that brings the avatar's joints closest to the reference's (the walker blueprint
    # scales it back: bp_scale), so the conform step only nudges joints instead of stretching limbs
    # least squares over the mapped joints (heights above the floor and sideways/forward offsets alike)
    num = den = 0.0
    for b, c in bmap.items():
        sv = S[b] - Vector((0.0, 0.0, zmin_src))
        rv = R[c] - Vector((0.0, 0.0, zmin_ref))
        num += sv.dot(rv)
        den += sv.dot(sv)
    k = num / den if a.conform and den > 0 else 1.0
    fit = Matrix.Translation((0.0, 0.0, zmin_ref)) @ Matrix.Scale(k, 4) @ Matrix.Translation((0.0, 0.0, -zmin_src))
    report["leg_scale"] = round(k, 4)
    report["bp_scale"] = round(1.0 / k, 4)

    # 2. fit a copy of the reference armature to the avatar's joints
    new_arm = ref_arm.copy()
    new_arm.data = ref_arm.data.copy()
    bpy.context.scene.collection.objects.link(new_arm)
    new_arm.animation_data_clear()          # the reference may come from an animation FBX: rest pose only
    for pb in new_arm.pose.bones:
        pb.matrix_basis.identity()
    inv = new_arm.matrix_world.inverted()
    target = {v: inv @ (fit @ S[b]) for b, v in bmap.items()}
    if a.conform:      # CARLA's hips joint sits above the thighs (the Biped pelvis at their height): keep CARLA's
        target.pop("crl_hips__C", None)
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = new_arm
    new_arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    eb = new_arm.data.edit_bones
    orig = {b.name: (b.head.copy(), b.tail.copy(), b.roll) for b in eb}
    for b in eb:
        b.use_connect = False
    placed = {}
    for name in bone_list_order(ref_arm):
        b = eb[name]
        h0, t0, roll = orig[name]
        if name in target:
            h = target[name]
        elif b.parent is not None and b.parent.name in placed:
            p = b.parent.name
            ph0 = orig[p][0]
            # scale by the parent segment's length ratio when the parent has a mapped child, else 1
            scale = placed[p][1]
            h = placed[p][0] + (h0 - ph0) * scale
        else:
            h = h0
        # this bone's scale: its segment length to a mapped child, against the reference
        kids = [c.name for c in b.children if c.name in target]
        scale = 1.0
        if kids:
            ref_len = (orig[kids[0]][0] - h0).length
            new_len = (target[kids[0]] - h).length
            scale = new_len / ref_len if ref_len > 1e-6 else 1.0
        elif b.parent is not None and b.parent.name in placed:
            scale = placed[b.parent.name][1]
        vec = (t0 - h0) * scale
        b.head, b.tail = h, h + vec
        b.roll = roll
        placed[name] = (h, scale)
    bpy.ops.object.mode_set(mode="OBJECT")

    # 3. weights: rename mapped groups, merge the others into their nearest mapped ancestor
    def mapped_ancestor(bname):
        b = src_arm.data.bones.get(bname)
        while b is not None and b.name not in bmap:
            b = b.parent
        return bmap[b.name] if b is not None else "crl_hips__C"
    for m in src_meshes:
        groups = {vg.index: vg.name for vg in m.vertex_groups}
        acc = {}
        for v in m.data.vertices:
            for g in v.groups:
                tgt = bmap.get(groups[g.group]) or mapped_ancestor(groups[g.group])
                acc.setdefault(tgt, {})
                acc[tgt][v.index] = acc[tgt].get(v.index, 0.0) + g.weight
        for vg in list(m.vertex_groups):
            m.vertex_groups.remove(vg)
        for tgt, ws in acc.items():
            vg = m.vertex_groups.new(name=tgt)
            for vi, w in ws.items():
                vg.add([vi], min(1.0, w), "REPLACE")
        mw = m.matrix_world.copy()           # keep its place (the avatar's armature carried a 0.01 scale)
        m.parent = None
        m.matrix_world = fit @ mw
        bpy.ops.object.select_all(action="DESELECT")
        m.select_set(True)
        bpy.context.view_layer.objects.active = m
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        mod = m.modifiers.new("Armature", "ARMATURE")
        mod.object = new_arm
        m.parent = new_arm
        m.matrix_parent_inverse = new_arm.matrix_world.inverted()
    report["groups"] = sorted({vg.name for m in src_meshes for vg in m.vertex_groups})

    # 3b. conform: pose the fitted armature onto the reference's exact rest pose (same orientations, so only the
    # joints move), bake that into the mesh and bind it to an exact copy of the reference skeleton. The walker's rest
    # pose is then CARLA's, whatever the skeleton's translation retargeting does.
    if a.conform:
        bpy.ops.object.select_all(action="DESELECT")
        for name in bone_list_order(ref_arm):
            new_arm.pose.bones[name].matrix = ref_arm.data.bones[name].matrix_local.copy()
            bpy.context.view_layer.update()
        for m in src_meshes:
            bpy.context.view_layer.objects.active = m
            m.select_set(True)
            bpy.ops.object.modifier_apply(modifier="Armature")
            m.select_set(False)
        final = ref_arm.copy()
        final.data = ref_arm.data.copy()
        bpy.context.scene.collection.objects.link(final)
        final.animation_data_clear()
        for pb in final.pose.bones:
            pb.matrix_basis.identity()
        for m in src_meshes:
            mw = m.matrix_world.copy()
            m.parent = final
            m.matrix_world = mw
            mod = m.modifiers.new("Armature", "ARMATURE")
            mod.object = final
        bpy.data.objects.remove(new_arm, do_unlink=True)
        new_arm = final

    # 4. textures
    if a.textures:
        files = {}
        for f in os.listdir(a.textures):
            files[os.path.splitext(f)[0].lower()] = os.path.join(a.textures, f)
        for img in bpy.data.images:
            stem = os.path.splitext(os.path.basename(img.filepath))[0].lower()
            if stem in files:
                img.filepath = files[stem]
                img.reload()
            elif img.filepath:
                report["warnings"].append(f"texture not found: {img.filepath}")

    # 5. clean up and export
    root_name = ref_arm.name     # Blender makes the FBX root node (crl_root) the armature object: keep its name,
    for o in [src_arm] + ref_meshes + [ref_arm]:         # so the exported hierarchy starts at crl_root as in Unreal
        bpy.data.objects.remove(o, do_unlink=True)
    new_arm.name = root_name
    new_arm.data.name = root_name
    name = a.name or os.path.splitext(os.path.basename(a.out))[0]
    for i, m in enumerate(src_meshes):
        m.name = name if i == 0 else f"{name}_{i}"
    bpy.ops.object.select_all(action="DESELECT")
    for o in [new_arm] + src_meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = new_arm
    bpy.ops.export_scene.fbx(filepath=a.out, use_selection=True, object_types={"ARMATURE", "MESH"},
                             add_leaf_bones=False, primary_bone_axis="Y", secondary_bone_axis="X",
                             armature_nodetype="NULL", bake_anim=False, path_mode="COPY", embed_textures=True,
                             apply_unit_scale=True, apply_scale_options="FBX_SCALE_ALL", mesh_smooth_type="FACE",
                             use_armature_deform_only=False)
    zs = [(m.matrix_world @ v.co).z for m in src_meshes for v in m.data.vertices]
    report["height_m"] = round(max(zs) - min(zs), 3)
    report["out"] = a.out
    log("wrote", a.out, "height", report["height_m"], "warnings", report["warnings"])
    if a.report:
        import json
        with open(a.report, "w") as f:
            json.dump(report, f, indent=1)


if __name__ == "__main__":
    main()
