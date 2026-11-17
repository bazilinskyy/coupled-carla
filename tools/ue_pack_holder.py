"""Holder level for a content pack: one actor per asset of a /Game folder, so the pack's DLC cook takes them along.

    UnrealEditor-Cmd <tree>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \\
        -script="<repo>/tools/ue_pack_holder.py --pack CoupledCarlaStreet --src /Game/CoupledCarla --result <file>" \\
        -EnablePlugins=CoupledCarlaStreet -NullRHI -unattended -nosplash -nopause

(with "ExplicitlyLoaded": false in the pack's .uplugin for the session; tools/content_pack.py does all of this.)

A CARLA content pack cooks everything under its own mount point (/<Pack>/) plus, with -DLCIncludeEngineContent,
the /Game assets that content references and the base release did not cook. coupled-carla's street props, regional
lamps and drones (/Game/CoupledCarla) are spawned at runtime by path (static.prop.mesh's mesh_path) or by blueprint
id (blueprint.cc_*), so no map references them and a pack would leave them out. This writes /<Pack>/Maps/<name>, a
small level (not World Partition, not registered as a CARLA map) holding a StaticMeshActor per mesh, an actor per
blueprint and a cube per material, 20 km below the origin. The cook then ships them at their /Game paths, where
coupled-carla and the pack's BlueprintParameters.json find them.
"""
import argparse
import json
import shlex
import sys

import unreal


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", required=True)
    ap.add_argument("--src", default="/Game/CoupledCarla")
    ap.add_argument("--name", default="CCAssetHolder")
    ap.add_argument("--result", required=True)
    ap.add_argument("--also", default="", help="more blueprints to hold, comma-separated object paths (Pkg.Name): "
                    "catalog entries outside --src that the base release did not cook")
    a = ap.parse_args(argv)
    level = "/{}/Maps/{}".format(a.pack, a.name)
    out = {"level": level, "src": a.src, "meshes": 0, "blueprints": 0, "materials": 0, "skipped": [], "ok": False}
    try:
        reg = unreal.AssetRegistryHelpers.get_asset_registry()
        reg.scan_paths_synchronous([a.src, "/" + a.pack], True)
        assets = list(reg.get_assets_by_path(a.src, recursive=True))
        also = [p for p in a.also.split(",") if p]
        les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        if not les.new_level(level, False):
            raise RuntimeError("new_level failed for " + level)
        actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
        n = 0
        for ad in assets:
            cls = str(ad.asset_class_path.asset_name)
            path = "{}.{}".format(ad.package_name, ad.asset_name)
            if cls not in ("StaticMesh", "Blueprint", "Material", "MaterialInstanceConstant"):
                continue
            n += 1
            where = unreal.Vector((n % 40) * 600.0, (n // 40) * 600.0, -2000000.0)
            try:
                if cls == "Blueprint":
                    bp = unreal.load_object(None, "{}.{}_C".format(ad.package_name, ad.asset_name))
                    actors.spawn_actor_from_class(bp, where, unreal.Rotator(0, 0, 0))
                    out["blueprints"] += 1
                    continue
                actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, where, unreal.Rotator(0, 0, 0))
                comp = actor.static_mesh_component
                if cls == "StaticMesh":
                    comp.set_static_mesh(unreal.load_object(None, path))
                    out["meshes"] += 1
                else:
                    comp.set_static_mesh(cube)
                    comp.set_material(0, unreal.load_object(None, path))
                    out["materials"] += 1
            except Exception as e:  # noqa: BLE001 - keep going, report it
                out["skipped"].append("{} ({}): {}".format(path, cls, e))
        for path in also:
            n += 1
            where = unreal.Vector((n % 40) * 600.0, (n // 40) * 600.0, -2000000.0)
            try:
                actors.spawn_actor_from_class(unreal.load_object(None, path + "_C"), where, unreal.Rotator(0, 0, 0))
                out["blueprints"] += 1
            except Exception as e:  # noqa: BLE001
                out["skipped"].append("{} (also): {}".format(path, e))
        les.save_current_level()
        out["ok"] = True
    except Exception as e:  # noqa: BLE001
        out["error"] = str(e)
    with open(a.result, "w") as f:
        json.dump(out, f, indent=1)
    unreal.log_warning("[packholder] " + json.dumps({k: out[k] for k in ("meshes", "blueprints", "materials", "ok")}))


if __name__ == "__main__":
    main(shlex.split(" ".join(sys.argv[1:])) if len(sys.argv) > 1 else [])
