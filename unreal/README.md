# CoupledPhotoreal: Google Photorealistic 3D Tiles in a CARLA 1.0 level

Unreal plugin behind `map.photoreal: google` (see `coupledcarla/photoreal.py` for what it does and costs). It needs
Cesium for Unreal, the CoupledCarla content pack and, on the server machine only, the user's own Google Maps Platform
key. Nothing from Google is stored in content: tiles stream at run time.

## Build (CARLA `ue58-dev` source tree)

1. Cesium for Unreal **v2.31.0** (Apache 2.0), the UE 5.8 source zip from
   https://github.com/CesiumGS/cesium-unreal/releases/tag/v2.31.0 (`CesiumForUnreal-58-SourceOnly-v2.31.0.zip`, 248 MB;
   it has the prebuilt cesium-native libraries for every platform). Unzip `CesiumForUnreal/` into
   `<engine>/Engine/Plugins/Marketplace/` (lin5080) or `<carla>/Unreal/CarlaUnreal/Plugins/` (a packaging tree).
2. **Linux only:** copy `patches/CesiumCarlaRttiShim.cpp` into `CesiumForUnreal/Source/CesiumRuntime/Private/`. CARLA's
   engine fork compiles every module with RTTI and the prebuilt cesium-native libraries lack one typeinfo
   (`undefined symbol: typeinfo for Cesium3DTilesSelection::TileOcclusionRendererProxyPool`). Windows needs nothing.
3. Copy `CoupledPhotoreal/` into `<carla>/Unreal/CarlaUnreal/Plugins/`. It is `EnabledByDefault: false`, so the tree
   behaves as before until a server or the project enables it.
4. Build the two plugins for the editor target, one at a time (a foreign-plugin build only compiles that plugin):

   ```bash
   env DLSS_SDK=disabled <engine>/Engine/Build/BatchFiles/Linux/Build.sh CarlaUnrealEditor Linux Development \
       -project=<carla>/Unreal/CarlaUnreal/CarlaUnreal.uproject -game -buildscw -NoUBTMakefiles \
       -Plugin=<path>/CesiumForUnreal.uplugin          # then -Plugin=<carla>/.../Plugins/CoupledPhotoreal/CoupledPhotoreal.uplugin
   ```

   On lin5080 (2026-11-15): Cesium 182 actions in 2.5 min with 6 parallel actions, CoupledPhotoreal 20 s.
5. Run a server with them: `-EnablePlugins=CesiumForUnreal,CoupledPhotoreal -carla-packs=<coupled-carla>/coupledcarla/data/carla_packs`
   (the folder that holds `CoupledCarla/`).

For a packaged (cooked) server add both to the project instead, so the package build compiles and stages them:
`CarlaUnreal.uproject` "Plugins": `{"Name": "CesiumForUnreal", "Enabled": true}, {"Name": "CoupledPhotoreal", "Enabled": true}`,
and cook Cesium's materials and this plugin's tile material, which only C++ refers to: `Config/DefaultGame.ini`, section
`[/Script/UnrealEd.ProjectPackagingSettings]`, `+DirectoriesToAlwaysCook=(Path="/CesiumForUnreal")` and
`+DirectoriesToAlwaysCook=(Path="/CoupledPhotoreal")`.

## Tile material (the height-limited cut)

`Content/Materials/M_CCPhotorealTiles` (with two 4 x 4 default textures) replaces Cesium's base material on the photo
tiles. It is made by `Tools/make_material.py` in an editor of the tree (headless: `UnrealEditor-Cmd <uproject>
-run=pythonscript -script=<plugin>/Tools/make_material.py -EnablePlugins=CoupledPhotoreal -unattended -nullrhi`) and
checked by cooking it alone (`-run=cook -targetplatform=Linux -cooksinglepackagenorefs
-package=/CoupledPhotoreal/Materials/M_CCPhotorealTiles`: the shader errors show there, not in the editor with
-nullrhi). It reads Cesium's glTF parameters by name (a material without layers), clips where the level's cut grid
(`<Level>_cut.bin`, `python -m coupledcarla.photoreal cut <Level> <xodr>`) says the pixel is over the twin's lanes and
lower than the road + `cut_top`, and also where Cesium's polygon overlay (`Clipping_*`, `cut=polygons`, night
footprints) covers it. A server without the material (an older package) falls back to `cut=polygons`.

## Profiling and A/B tests

`prof=<s>` appends a row every s seconds to `google_tiles_profile.csv` in the config folder (frame, game, render, RHI
thread and GPU ms per frame, visible and loaded tiles, cameras, cut). `control=1` applies `photoreal_control.txt` there
whenever it changes (`key=value` lines: `phase`, `cut` mask/polygons/none, `cut_top`, `grow`, `tiles` 0/1 (the twin
alone), `sse`, `culled_sse`, `forbid_holes`, `loads`, `cache_mb`, `fog`, `occlusion`, `load_ms`, `clip_sse`, `clip_tex`,
`cams`, `player`, `viewport`, `cvar.<name>`) and acknowledges it in `photoreal_control.ack`; none of these reloads the
tileset, so a whole comparison costs one root request. `tools/photoreal_profile.py` drives it.

## Level data

`python -m coupledcarla.photoreal prepare <experiment> --footprints <carla-digitaltwins>/build/<twin>/ue/manifest.json`
writes `coupledcarla/data/carla_packs/CoupledCarla/Content/Config/Photoreal/<Level>.json`: the georeference origin,
the road carve polygons (from the level's OpenDRIVE) and the building footprints (night). Copy the pack to the server.

## Key and usage (server machine)

`GOOGLE_MAPS_API_KEY`, or the first line of `~/.config/coupled-carla/google_maps_key` (Windows:
`%USERPROFILE%\.config\coupled-carla\google_maps_key`), mode 600. `COUPLED_CARLA_CONFIG_DIR` moves that folder.
Root requests are counted in `google_tiles_usage.json` there; the credits seen last are in `google_tiles_credits.txt`.

## Check without a key

`tools/photoreal_check.py` against a server started with `env -u GOOGLE_MAPS_API_KEY COUPLED_CARLA_CONFIG_DIR=<empty dir>`:
blueprint present, no key = fail soft, a local mock tileset = exactly one root request, caps and reloads enforced.
