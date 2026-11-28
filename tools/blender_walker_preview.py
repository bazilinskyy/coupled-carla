"""Renders a re-skinned walker FBX (tools/blender_reskin_walker.py) in its rest pose and driven by a CARLA walk
animation FBX, to check the skinning before importing it: Cycles on the CPU (no GPU), a few small frames.

    blender -b --factory-startup -P tools/blender_walker_preview.py -- --fbx CC_X.fbx --anim AS_walking04_G3.fbx \\
        --out /tmp/prev_X [--frames 0,8,16,24]

Writes <out>_rest.png and <out>_f<frame>.png (front three-quarter view). The animation's bone rotations are applied
by bone name (the two armatures share CARLA's bone orientations), as CARLA's animation blueprint does.
"""
import argparse
import math
import sys

import bpy
from mathutils import Vector


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fbx", required=True)
    ap.add_argument("--anim")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", default="0,8,16,24")
    ap.add_argument("--res", type=int, default=420)
    a = ap.parse_args(sys.argv[sys.argv.index("--") + 1:])
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=a.fbx, automatic_bone_orientation=False, ignore_leaf_bones=False)
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    action = None
    if a.anim:
        before = set(bpy.data.objects)
        bpy.ops.import_scene.fbx(filepath=a.anim, automatic_bone_orientation=False, ignore_leaf_bones=False)
        anim_objs = [o for o in bpy.data.objects if o not in before]
        src = next(o for o in anim_objs if o.type == "ARMATURE")
        action = src.animation_data.action if src.animation_data else None
        for o in anim_objs:
            bpy.data.objects.remove(o, do_unlink=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 24
    sc.render.resolution_x, sc.render.resolution_y = a.res, int(a.res * 1.4)
    pts = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
    zmax = max(p.z for p in pts)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    # the face's side: the eyes' y (or the toes') against the hips
    bones = {b.name: arm.matrix_world @ b.head_local for b in arm.data.bones}
    fwd = 1.0 if bones.get("crl_toe__L", Vector((0, 1, 0))).y > bones.get("crl_foot__L", Vector()).y else -1.0
    d = 3.2 * max(1.0, zmax / 1.8)
    cam.location = (d * 0.5, fwd * d * 0.85, zmax * 0.55)
    cam.rotation_euler = (math.radians(88), 0, math.radians(30 if fwd > 0 else 150) + (math.pi if fwd > 0 else 0))
    direction = Vector((0, 0, zmax * 0.5)) - cam.location
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 4.0
    sun.rotation_euler = (math.radians(40), 0, math.radians(30))
    sc.collection.objects.link(sun)
    world = bpy.data.worlds.new("w")
    world.color = (0.6, 0.6, 0.65)
    sc.world = world
    sc.render.filepath = a.out + "_rest.png"
    bpy.ops.render.render(write_still=True)
    if action is not None:
        arm.animation_data_create()
        arm.animation_data.action = action
        if hasattr(arm.animation_data, "action_slot") and getattr(action, "slots", None):
            arm.animation_data.action_slot = action.slots[0]      # Blender 4.4+: slotted actions
        for f in [int(x) for x in a.frames.split(",")]:
            sc.frame_set(int(action.frame_range[0]) + f)
            sc.render.filepath = f"{a.out}_f{f}.png"
            bpy.ops.render.render(write_still=True)
    print("[preview] done", a.out)


if __name__ == "__main__":
    main()
