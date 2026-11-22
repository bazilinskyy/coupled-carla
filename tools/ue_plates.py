"""Import the country number plates (tools/plate_faces.py) into a CARLA 1.0 project, as the server's licence plate
code (CARLA LicensePlates.cpp) expects them: /Game/CoupledCarla/Plates/<CC>/T_Plate_<CC>_NN (texture) and
MI_Plate_<CC>_NN, a material instance of CARLA's shared plate material MI_LicensePlate with only its Diffuse texture
replaced (same UVs, same surface).

Editor Python (headless):
    UnrealEditor-Cmd <CarlaUnreal.uproject> -run=pythonscript -script="<abs>/tools/ue_plates.py <abs plates dir>"

Idempotent: existing assets are re-imported and updated. Cook /Game/CoupledCarla/Plates into packages (the server
loads the plates by path).
"""
import sys
from pathlib import Path

import unreal

ROOT = "/Game/CoupledCarla/Plates"
PARENT = "/Game/Carla/Static/GenericMaterials/Vehicles/MI_LicensePlate"
EAL = unreal.EditorAssetLibrary


def import_texture(png: Path, dest: str):
    task = unreal.AssetImportTask()
    task.filename = str(png)
    task.destination_path = dest
    task.destination_name = png.stem
    task.replace_existing = True
    task.automated = True
    task.save = False
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    tex = EAL.load_asset(f"{dest}/{png.stem}")
    if tex is not None:
        tex.set_editor_property("srgb", True)
        tex.set_editor_property("mip_gen_settings", unreal.TextureMipGenSettings.TMGS_FROM_TEXTURE_GROUP)
        tex.set_editor_property("lod_group", unreal.TextureGroup.TEXTUREGROUP_VEHICLE)
    return tex


def main(src: Path):
    parent = EAL.load_asset(PARENT)
    if parent is None:
        raise RuntimeError("missing " + PARENT)
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    made = 0
    for cdir in sorted(p for p in src.iterdir() if p.is_dir()):
        cc = cdir.name
        dest = f"{ROOT}/{cc}"
        for png in sorted(cdir.glob(f"T_Plate_{cc}_*.png")):
            tex = import_texture(png, dest)
            if tex is None:
                unreal.log_error("import failed: %s" % png)
                continue
            name = png.stem.replace("T_Plate_", "MI_Plate_")
            path = f"{dest}/{name}"
            mi = EAL.load_asset(path) if EAL.does_asset_exist(path) else \
                tools.create_asset(name, dest, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
            mi.set_editor_property("parent", parent)
            unreal.MaterialEditingLibrary.set_material_instance_texture_parameter_value(mi, "Diffuse", tex)
            unreal.MaterialEditingLibrary.update_material_instance(mi)
            made += 1
        EAL.save_directory(dest, only_if_is_dirty=False, recursive=True)
        unreal.log("PLATES %s: %d" % (cc, len(list(cdir.glob(f"T_Plate_{cc}_*.png")))))
    unreal.log("PLATESDONE %d material instances" % made)


if __name__ == "__main__":
    main(Path(sys.argv[-1]))
