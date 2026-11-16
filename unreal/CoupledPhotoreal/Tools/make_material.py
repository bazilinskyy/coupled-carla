"""Builds the plugin's tile material in the Unreal editor (UE 5.8, CARLA's fork): run it in an editor of a CARLA tree
with this plugin enabled, e.g. headless

    UnrealEditor-Cmd <carla>/Unreal/CarlaUnreal/CarlaUnreal.uproject -run=pythonscript \
        -script=<carla>/Unreal/CarlaUnreal/Plugins/CoupledPhotoreal/Tools/make_material.py \
        -EnablePlugins=CoupledPhotoreal -unattended -nullrhi -nosplash

It writes /CoupledPhotoreal/Materials/M_CCPhotorealTiles (and two 4 x 4 default textures) into the plugin's Content
folder. The material replaces Cesium's base material on the photo tiles (ACCPhotorealTiles_C sets it): Cesium sets the
glTF parameters by name on a material without layers (baseColorTexture, baseColorFactor, baseColorTextureCoordinateIndex,
metallicFactor, roughnessFactor) and a raster overlay keyed "Clipping" as Clipping_Texture, Clipping_TranslationScale,
Clipping_TextureCoordinateIndex (the polygon overlay: R8, 1 inside a polygon). The plugin sets the cut grid
(CCCutTex: R = signed distance to the cut's outline in m, G = road height in m), its transform from world cm to UV
(CCCutXform: u = X a + b, v = Y c + d) and CCCutOn, CCCutTop (m above the road), CCCutGrow (m).

Lit like Cesium's own (default lit; Cesium gives KHR_materials_unlit primitives up-facing normals, metallic 0,
roughness 1), masked at 0.5: a pixel is clipped where it lies inside the cut (distance < grow) and lower than the road
height + cut top, or where the polygon overlay covers it.
"""
import os
import struct
import tempfile
import zlib

import unreal

PKG = "/CoupledPhotoreal/Materials"
NAME = "M_CCPhotorealTiles"
AT = unreal.AssetToolsHelpers.get_asset_tools()
MEL = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary


def png(path, rgba, size=4):
    raw = b"".join(b"\x00" + bytes(rgba) * size for _ in range(size))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def existing(path):
    """The asset at path, or None (a commandlet's asset registry may not have scanned the plugin yet)."""
    try:
        return unreal.load_asset(path)
    except Exception:  # noqa: BLE001
        return None


def texture(name, rgba, srgb):
    path = f"{PKG}/{name}"
    old = existing(path)
    if old is not None:
        return old
    tmp = os.path.join(tempfile.mkdtemp(), name + ".png")
    png(tmp, rgba)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", tmp)
    task.set_editor_property("destination_path", PKG)
    task.set_editor_property("destination_name", name)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", False)
    AT.import_asset_tasks([task])
    tex = unreal.load_asset(path)
    tex.set_editor_property("srgb", srgb)
    tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_DEFAULT)
    tex.set_editor_property("mip_gen_settings", unreal.TextureMipGenSettings.TMGS_NO_MIPMAPS)
    EAL.save_asset(path, only_if_is_dirty=False)
    return tex


def node(mat, cls, x, y, **props):
    e = MEL.create_material_expression(mat, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def custom(mat, x, y, code, inputs, out_type, desc):
    c = node(mat, unreal.MaterialExpressionCustom, x, y, code=code, output_type=out_type, description=desc)
    ins = []
    for n in inputs:
        ci = unreal.CustomInput()
        ci.set_editor_property("input_name", n)
        ins.append(ci)
    c.set_editor_property("inputs", ins)
    return c


def link(src, dst, name, out=""):
    if not MEL.connect_material_expressions(src, out, dst, name):
        raise RuntimeError(f"cannot connect {out or 'output'} into {name}")


def main():
    white = texture("T_CCWhite", (255, 255, 255, 255), True)
    black = texture("T_CCBlackLinear", (0, 0, 0, 255), False)
    path = f"{PKG}/{NAME}"
    mat = existing(path)
    if mat is not None:     # rebuilt in place (deleting or re-creating an asset fails unattended)
        MEL.delete_all_material_expressions(mat)
    else:
        mat = AT.create_asset(NAME, PKG, unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
    mat.set_editor_property("opacity_mask_clip_value", 0.5)
    mat.set_editor_property("used_with_instanced_static_meshes", True)

    uvs = [node(mat, unreal.MaterialExpressionTextureCoordinate, -1400, 200 * i, coordinate_index=i) for i in range(4)]
    ctex = unreal.MaterialSamplerType.SAMPLERTYPE_COLOR
    ltex = unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR
    base_tex = node(mat, unreal.MaterialExpressionTextureObjectParameter, -1100, -400, parameter_name="baseColorTexture",
                    texture=white, sampler_type=ctex)
    base_idx = node(mat, unreal.MaterialExpressionScalarParameter, -1100, -250,
                    parameter_name="baseColorTextureCoordinateIndex", default_value=0.0)
    base_fac = node(mat, unreal.MaterialExpressionVectorParameter, -1100, -150, parameter_name="baseColorFactor",
                    default_value=unreal.LinearColor(1, 1, 1, 1))
    metal = node(mat, unreal.MaterialExpressionScalarParameter, -400, 0, parameter_name="metallicFactor",
                 default_value=0.0)
    rough = node(mat, unreal.MaterialExpressionScalarParameter, -400, 100, parameter_name="roughnessFactor",
                 default_value=1.0)
    base = custom(mat, -700, -300, """
float2 uv = Idx < 0.5 ? UV0 : (Idx < 1.5 ? UV1 : (Idx < 2.5 ? UV2 : UV3));
return Texture2DSample(Tex, TexSampler, uv).rgb * Factor.rgb;   // (SampleLevel in ray-tracing stages)
""", ["Tex", "Idx", "Factor", "UV0", "UV1", "UV2", "UV3"], unreal.CustomMaterialOutputType.CMOT_FLOAT3,
                  "glTF base colour (Cesium's parameters)")
    link(base_tex, base, "Tex")
    link(base_idx, base, "Idx")
    link(base_fac, base, "Factor", "RGBA")
    for i, uv in enumerate(uvs):
        link(uv, base, f"UV{i}")

    wp = node(mat, unreal.MaterialExpressionWorldPosition, -1100, 400)
    cut_tex = node(mat, unreal.MaterialExpressionTextureObjectParameter, -1100, 500, parameter_name="CCCutTex",
                   texture=black, sampler_type=ltex)
    xform = node(mat, unreal.MaterialExpressionVectorParameter, -1100, 650, parameter_name="CCCutXform",
                 default_value=unreal.LinearColor(0, 0, 0, 0))
    on = node(mat, unreal.MaterialExpressionScalarParameter, -1100, 800, parameter_name="CCCutOn", default_value=0.0)
    top = node(mat, unreal.MaterialExpressionScalarParameter, -1100, 900, parameter_name="CCCutTop",
               default_value=2.8)
    grow = node(mat, unreal.MaterialExpressionScalarParameter, -1100, 1000, parameter_name="CCCutGrow",
                default_value=0.0)
    p_tex = node(mat, unreal.MaterialExpressionTextureObjectParameter, -1100, 1150, parameter_name="Clipping_Texture",
                 texture=black, sampler_type=ltex)
    p_ts = node(mat, unreal.MaterialExpressionVectorParameter, -1100, 1300, parameter_name="Clipping_TranslationScale",
                default_value=unreal.LinearColor(0, 0, 1, 1))
    p_idx = node(mat, unreal.MaterialExpressionScalarParameter, -1100, 1450,
                 parameter_name="Clipping_TextureCoordinateIndex", default_value=0.0)
    clip = custom(mat, -600, 700, """
float keep = 1.0;
if (On > 0.5)
{
    float2 uv = float2(WP.x * X.r + X.g, WP.y * X.b + X.a);
    if (all(uv > 0.0) && all(uv < 1.0))
    {
        float2 g = Cut.SampleLevel(CutSampler, uv, 0).rg;   // signed distance (m), road height (m)
        if (g.r < Grow && WP.z * 0.01 < g.g + Top)
            keep = 0.0;
    }
}
float2 puv = PIdx < 0.5 ? UV0 : (PIdx < 1.5 ? UV1 : (PIdx < 2.5 ? UV2 : UV3));
if (PTex.SampleLevel(PTexSampler, puv * PTS.zw + PTS.xy, 0).r > 0.5)
    keep = 0.0;
return keep;
""", ["WP", "Cut", "X", "On", "Top", "Grow", "PTex", "PTS", "PIdx", "UV0", "UV1", "UV2", "UV3"],
                  unreal.CustomMaterialOutputType.CMOT_FLOAT1, "the road cut: height-limited mask, polygon overlay")
    for src, name, pin in ((wp, "WP", ""), (cut_tex, "Cut", ""), (xform, "X", "RGBA"), (on, "On", ""), (top, "Top", ""),
                           (grow, "Grow", ""), (p_tex, "PTex", ""), (p_ts, "PTS", "RGBA"), (p_idx, "PIdx", "")):
        link(src, clip, name, pin)   # a vector parameter's default output is its RGB
    for i, uv in enumerate(uvs):
        link(uv, clip, f"UV{i}")

    MEL.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.connect_material_property(metal, "", unreal.MaterialProperty.MP_METALLIC)
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.connect_material_property(clip, "", unreal.MaterialProperty.MP_OPACITY_MASK)
    MEL.recompile_material(mat)
    EAL.save_asset(path, only_if_is_dirty=False)
    unreal.log(f"CCPHOTOREAL_MATERIAL_SAVED {path}")


main()
