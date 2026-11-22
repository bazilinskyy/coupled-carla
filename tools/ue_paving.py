"""Import the road paving materials into a CARLA 1.0 project: /Game/CoupledCarla/Paving/MI_Surface_<style> for the
generated maps' paved lanes (coupledcarla/road_surfaces.py writes OpenDRIVE <material surface="<style>">; CARLA with
the road-surface change gives such lanes <folder>/MI_Surface_<style>, server option
-carla-road-surface-materials=/Game/CoupledCarla/Paving).

Textures: carla-digitaltwins TwinModel/tools/paving_textures.py (T_<prefix>_paving_<style>_{d,n,orm}.png, 2 x 2 m
tiles). Each material is an instance of CARLA's M_GeneralMaster projected in world space (World Position + Triplanar:
CARLA's generated road meshes carry no UVs), 2 m per texture repeat, no recolouring, weather wetness kept.

Editor Python (headless):
    UnrealEditor-Cmd <CarlaUnreal.uproject> -run=pythonscript -script="<abs>/tools/ue_paving.py <abs textures dir>"

Idempotent. Cook /Game/CoupledCarla/Paving into packages (the server loads the materials by path).
"""
import sys
from pathlib import Path

import unreal

ROOT = "/Game/CoupledCarla/Paving"
PARENT = "/Game/Carla/Static/GenericMaterials/000_Masters/M_GeneralMaster"
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
TILE_CM = 200.0          # one texture repeat (paving_textures.TILE_M)
SLOTS = {"d": ("BaseColor", True, unreal.TextureCompressionSettings.TC_DEFAULT),
         "n": ("Normal", False, unreal.TextureCompressionSettings.TC_NORMALMAP),
         "orm": ("AO / Roughness / Metallic / Emissive", False, unreal.TextureCompressionSettings.TC_MASKS)}


def import_texture(png: Path, srgb: bool, comp):
    task = unreal.AssetImportTask()
    task.filename = str(png)
    task.destination_path = ROOT
    task.destination_name = png.stem
    task.replace_existing = True
    task.automated = True
    task.save = False
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    tex = EAL.load_asset(f"{ROOT}/{png.stem}")
    if tex is not None:
        tex.set_editor_property("srgb", srgb)
        tex.set_editor_property("compression_settings", comp)
        tex.set_editor_property("lod_group", unreal.TextureGroup.TEXTUREGROUP_WORLD)
    return tex


def main(src: Path):
    parent = EAL.load_asset(PARENT)
    if parent is None:
        raise RuntimeError("missing " + PARENT)
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    made = []
    for d in sorted(src.glob("T_*_paving_*_d.png")):
        style = d.stem.split("_paving_")[1][:-2]
        name = f"MI_Surface_{style}"
        path = f"{ROOT}/{name}"
        mi = EAL.load_asset(path) if EAL.does_asset_exist(path) else \
            tools.create_asset(name, ROOT, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        MEL.set_material_instance_parent(mi, parent)
        for suf, (param, srgb, comp) in SLOTS.items():
            png = d.with_name(d.name[:-len("_d.png")] + f"_{suf}.png")
            tex = import_texture(png, srgb, comp) if png.exists() else None
            if tex is None:
                unreal.log_error("PAVING %s: no %s" % (style, png.name))
                continue
            MEL.set_material_instance_texture_parameter_value(mi, param, tex)
        for sw, val in (("World Position", True), ("Triplanar", True), ("Change Color", False),
                        ("Normal On/Off", True), ("AddVariations", False)):
            MEL.set_material_instance_static_switch_parameter_value(mi, sw, val)
        for sc, val in (("Scale X", 1.0 / TILE_CM), ("Scale Y", 1.0 / TILE_CM), ("Normal Intensity", 1.0),
                        ("Saturation", 1.0), ("Brightness", 1.0), ("Contrast", 1.0)):
            MEL.set_material_instance_scalar_parameter_value(mi, sc, val)
        MEL.update_material_instance(mi)
        made.append(name)
    EAL.save_directory(ROOT, only_if_is_dirty=False, recursive=True)
    unreal.log("PAVINGDONE %s" % ", ".join(made))


if __name__ == "__main__":
    main(Path(sys.argv[-1]))
