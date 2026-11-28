# Third-party assets

Assets that coupled-carla imports into CARLA's content but does not keep in this repository (they are fetched by a
script and built into a CARLA package). The code's own licence is in [LICENSE](LICENSE); data sources are credited
in the README ("Credits, data and licences").

## Walkers: Microsoft Rocketbox avatars (MIT)

Eleven avatars of the [Microsoft Rocketbox avatar library](https://github.com/microsoft/Microsoft-Rocketbox)
(commit 0943055db6ec570bcef9f2c8b41c9e5467c808f9), re-skinned to CARLA's GEN3 pedestrian skeleton
(`tools/blender_reskin_walker.py`) and imported as CARLA 1.0 walkers `walker.pedestrian.0100` and `0102`-`0111`
(`tools/import_walkers.py`; README "Who the pedestrians are"). Fetched by `tools/fetch_walker_assets.py`.

| Walker | Rocketbox avatar | Changes |
|---|---|---|
| 0100 | Adults/Female_Adult_03 | re-posed to CARLA's T pose, joints fitted to the GEN3 skeleton, weights renamed and merged, textures as JPEG/PNG |
| 0102 | Adults/Male_Adult_09 | as above |
| 0103 | Adults/Male_Adult_10 | as above |
| 0104 | Professions/Business_Male_02 | as above |
| 0105 | Adults/Female_Adult_06 | as above |
| 0106 | Adults/Female_Adult_10 | as above |
| 0107 | Children/Female_Child_02 | as above |
| 0108 | Adults/Male_Adult_15 | as above |
| 0109 | Adults/Male_Adult_19 | as above |
| 0110 | Adults/Male_Adult_17 | as above |
| 0111 | Professions/Business_Female_01 | as above |

`walker.pedestrian.0101` is CARLA's own `SK_AsiaM03_G3` mesh (CARLA content, MIT), given a blueprint.

```
MIT License

Copyright (c) 2020 Microsoft

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Not used

- MakeHuman / MPFB 2 (CC0 models) for East Asian children: not made yet (see the README's coverage gaps).
