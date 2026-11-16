# Street photos: how close is a map to the real street?

`tools/realism_check.py` puts real street-level photos next to CARLA renders taken from the same place, height,
heading and field of view. It scores how alike they are and says in words where they differ. A second part checks
the map's street objects against what the photos show: lamps, signs, benches, bins and the like. It reports what a
correction would change, and it can apply the changes. The code is in `coupledcarla/realism.py` (sources, views,
layout metrics) and `coupledcarla/streetphotos.py` (Mapillary, the extra metrics, the objects and the corrections).

## Sources

| Source | Key | Licence | What it gives |
|---|---|---|---|
| [Panoramax](https://panoramax.xyz) | none | CC BY-SA 4.0 (per picture, read from its metadata) | 360° panoramas and action-cam photos with position, heading and camera |
| [KartaView](https://kartaview.org) | none | CC BY-SA 4.0 | phone and dashcam photos (the keyless v2 API lists at most 500 m around a point) |
| [Wikimedia Commons](https://commons.wikimedia.org) | none | per file (CC BY, CC BY-SA, CC0, PD) | geotagged photos with a camera heading |
| [Mapillary](https://www.mapillary.com) | `mapillary_token` | CC BY-SA 4.0 | photos and panoramas, and *map features*: street lights, signs, poles, bins, benches, hydrants and more, placed from many photos |

Coverage of the boxes used so far (2026-11-16; `realism_check.py sources <experiment>` writes it to
`maps/<name>/real/streetphotos/coverage.json`):

| Box | Panoramax | KartaView | Commons (with heading) | Mapillary photos (panoramas) | Mapillary map features |
|---|---|---|---|---|---|
| Eindhoven centre (`eindhoven_centre`, 1.6 x 1.6 km) | 1,727 (846 panoramas, 2024-25) | 85 | 218 | 64,219 (19,901), 2009-2026 | 28,121 |
| TU/e campus (51.4453-51.4524 N, 5.4813-5.4991 E) | 791 (188 panoramas, 2022-26) | 6 | 1 | 11,738 (1,758) | 3,883 |
| Chernihiv (`chernihiv`, 1.4 x 1.4 km) | 0 | 0 | 71 (22 dated in daylight) | 11,993 (4,868), 2018-2022 | 4,721 |

Requests are spaced per host (Mapillary tiles 0.2 s, graph 0.1 s, Panoramax 0.5 s) and retried with backoff on
429 and 5xx. Mapillary's published limits are far above that (forum figures: 50,000 graph requests per minute,
50,000 tile requests per day). Everything is cached under the map folder, which is not in git:

- `real/streetview_<source>.json`: the photo metadata.
- `real/streetphotos/`: photos (`real/`), panoramas (`panos/`), their labels (`seg/`), Mapillary tiles and image
  records (`mapillary/`), coverage and the street-object evidence.

Mapillary tiles are fetched again after 30 days. Each run credits the photographers in `ATTRIBUTION.csv`, on every
contact sheet and on every spot image.

### Getting a Mapillary token

1. Log in at mapillary.com (a free account), open **Dashboard > Developers** and **Register application**. Any
   name will do. Tick **Read** under what the application may do: without it the graph API refuses every image
   (see below).
2. Copy the application's **client token**. It starts with `MLY|`.
3. Put it in the project's `secret` file as `"mapillary_token"`. `python -m coupledcarla.settings --set
   mapillary_token` asks for it without echo. It can also go in `$MAPILLARY_TOKEN`, or in the older
   `~/.config/coupled-carla/mapillary_token` (mode 600). `python -m coupledcarla.settings` shows whether it is set,
   without printing it.

Never paste the token into a chat, a commit, an issue or a log. The code sends it only to Mapillary. The tiles
take it in the URL, so the HTTP client's error messages never include the URL.

**What the token unlocks.**

- *Map features*, from the tiles: positions and classes of lamps, signs, poles and furniture. This works with the
  current token, on every map.
- *Photos and panoramas*, which need the graph API (`graph.mapillary.com/<image id>`) for the camera, the heading
  corrected by structure from motion, and the image URL. With the token set on 2026-11-16, the tiles answer but
  the graph API refuses every image and map feature ("does not exist, cannot be loaded due to missing permissions")
  and its bbox search returns empty lists. The application probably lacks read access. Until that is fixed, the
  tool counts Mapillary's photos but cannot compare them. This matters most for Chernihiv, where Mapillary has
  4,868 panoramas from 2022 and the open sources have none. `realism_check.py sources` tells whether the graph
  answers.

## Running it

```bash
export COUPLED_CARLA_MUTE=1 SDL_AUDIODRIVER=dummy           # nothing on the speakers
python tools/realism_check.py sources  experiments/chernihiv.yaml                      # what each source has
python tools/realism_check.py prepare  experiments/eindhoven_centre.yaml --out logs/sp --labels [--box s,w,n,e]
python tools/realism_check.py render   experiments/eindhoven_centre.yaml --out logs/sp --port 2500   # CARLA, GPU
python tools/realism_check.py pairs    experiments/eindhoven_centre.yaml --out logs/sp   # real-vs-real baseline
python tools/realism_check.py compare  --out logs/sp                                     # metrics, sheets, spots
python tools/realism_check.py objects  experiments/chernihiv.yaml --evidence mapillary   # street objects
python tools/realism_check.py table    logs/sp_before logs/sp_after                     # runs side by side
```

- **prepare** picks the views: one per 20 m, direction and source; daylight only; no handlebar action cams.
  Panoramas are cut to a 90° view. `--box` keeps only part of the map (for example the TU/e campus inside
  `eindhoven_centre`). `--labels` segments the photos now, so that compare only has the renders left.
- **render** works on CARLA 0.9.16 (an OpenDRIVE map with its scenery) and on CARLA 1.0 levels
  (`map.carla_level`). It renders each view wider than the photo, so that photo heading errors can be searched
  (±12° yaw, ±6° pitch; panoramas ±30°). The camera is set at the source's typical height:
  - 2.1 m: Panoramax panoramas, from a pole on a cyclist's backpack
  - 2.2 m: Mapillary panoramas, mostly car roofs
  - 1.4 m: dashcams
  - 1.5 to 1.6 m: phones and Commons photos

  Weather and sun are set from the photo's hour.
- **compare** segments both images with SegFormer-B2 (Cityscapes; NVIDIA's research licence). With
  `--no-perceptual` it skips DINOv2-base (Apache 2.0). Every model runs on CPU too.
- **objects** is described under "Street objects" below.

Outputs in `--out`:

- `compare.csv`: one row per spot.
- `summary.json`: medians, the map score, the most frequent differences, the worst spots and the realism index.
- `sheets/twin_NN.jpg`: contact sheets, best spot first.
- `spots/<id>.jpg`: each spot side by side: photo, render, both layouts, scores, differences and credit.

## What the scores mean

All scores compare the photo and the render after the alignment search.

| Score | Meaning |
|---|---|
| `al_miou_coarse` | IoU of where ground, buildings, vegetation and sky are (0-1). Road and pavement are merged, because Dutch red cycle paths and brick pavements swap between them in the model. |
| `boundary_f` | Boundary F-score (0-1): the share of each image's outlines (ground/building/vegetation/sky edges, poles, cars) within 1 % of the width of the other's. Tests whether the street's edges, kerbs and skyline line up, which area overlap misses for thin strips. |
| `al_skyline_mae_deg` | Mean difference, in degrees, of the highest non-sky pixel per column. |
| `dino_global` | Cosine similarity of the two images' DINOv2 image tokens. Answers "is it the same kind of place?"; a perceptual measure that reacts to looks as well as to layout. |
| `dino_patch` | Mean cosine similarity of DINOv2 patch tokens at the same places. Answers "do the same things appear in the same parts of the view?" |
| `colour_hist_sim` and image statistics | Colour, contrast, saturation and fine detail (see the README). |
| `spot_score` | Mean of `al_miou_coarse`, `boundary_f` and `dino_patch`, one number per spot (0-1). |
| map score | Median spot score × 100. |
| realism index | Per measure: where the map's median lies between real photos of random places (0) and a second real photo of the same street 10-25 m on (1). The scale comes from `pairs`. Maps without panoramas use Eindhoven's scale. |

The real-vs-real scale (Eindhoven, 100 pairs of Panoramax panoramas):

| Pair | Coarse IoU | Boundary F | DINO global | DINO patch | Spot score | Skyline (°) |
|---|---|---|---|---|---|---|
| Same place, another ride | 0.544 | 0.445 | 0.781 | 0.477 | 0.467 | 2.4 |
| Same street, 10-25 m on | 0.536 | 0.431 | 0.849 | 0.525 | 0.498 | 3.8 |
| Random place in the box | 0.328 | 0.298 | 0.376 | 0.335 | 0.305 | 6.6 |

So a map score of about 50 means "as alike as two photos of the same street", and about 30 means "as alike as a
photo of somewhere else".

**Differences in words** (`differences` in `compare.csv`; the map's counts are in `summary.json`). Each image is
cut into thirds (left, middle, right) and a lower half (the street). A difference is reported when a class's share
differs by more than these limits (in percentage points):

- **missing building / a building where the photo has none** (left, middle, right): building share differs by more
  than 12.
- **missing trees / trees or greenery where the photo has none**: vegetation share differs by more than 10.
- **road too wide / too narrow**: road share of the lower half differs by more than 12.
- **pavement missing or too narrow / more pavement than the photo**: pavement share of the lower half differs by
  more than 8.
- **cars in the photo, few or none in the render**: cars cover more than 3 % of the photo and under a third of
  that in the render.
- **poles, lamps or signs missing**: the photo has more than one percentage point more of them, and more than
  twice the render's share.
- **skyline N° too low / too high**: the mean skyline difference is more than 3°.
- **more open sky than the photo**: sky share differs by more than 10, when the skyline rule did not fire.

The four largest are kept per spot, largest first. They describe the view, not the cause. A tree in the photo in
front of a building reads as "missing trees"; it can also come from the photo's position (the GPS is off by 2-5 m).
Use them to find the spots worth a look, then look at the spot image.

## Street objects

`objects` compares the photos' evidence with what `scenery.json` places:

- lamps
- signs (a post and its plates count as one)
- benches, bins, bike racks, hydrants, post boxes and billboards, by their mesh

Two kinds of evidence:

- **Mapillary map features** (token; `--evidence mapillary`), from the tiles. A feature is placed by Mapillary from
  many photos. Signs seen from the front and the back are merged within 1.5 m.
- **Poles in Panoramax panoramas** (no key; `--evidence panoramax`). The Cityscapes model labels each panorama, and
  every pole whose foot is seen on the ground gets a direction and a distance (camera height 2.1 m over the angle
  of the foot below the horizon). It counts when two or more panoramas put it within 2.5 m. It is a lamp when it
  is taller than 5 m or leaves the image at the top, a sign when a sign touches it, else a pole.

Objects are matched one-to-one within 6 m (lamps), 5 m (signs) or 4 m (furniture). From the matches come the
proposed corrections:

- **move**: matched more than 1.5 m apart, the evidence seen on two dates or more;
- **add**: evidence with no scenery object of its class within twice the match radius, seen on two dates or more
  (lamps, benches, bins, bike racks, hydrants, post boxes);
- **remove**: a scenery object with no evidence of its class within 10 m, although 20 or more photos on 3 or more
  days were taken within 20 m of it.

Classes whose positions come from an official register are never corrected: a city's lighting register
(`real/municipal.json`, Eindhoven) and the Dutch NDW sign register.

`--apply` writes the corrections into `scenery.json`; the old file stays as `scenery.before_photos.json`. It applies
`--actions add,remove` by default; `move` must be asked for. Added objects copy the mesh, turn and height above
ground of the nearest object of their class. With `carla` installed, none is put on a driving lane. To have
`mapbuild` apply the saved evidence after every scenery build, set `map.scenery.photo_corrections:
[mapillary]`. It is off by default.

**Validated against Eindhoven's lighting register** (`--validate lamp`). The register (2,311 lamps in the box) is
spoiled the way guessed or OSM positions are: each lamp shifted, 30 % left out, 30 % made-up lamps added. The
corrections are then made from Mapillary's lamps and the result is compared with the register:

| Spoiled by | Lamps within 4 m of a register lamp | Moves that came closer | Removals that hit a made-up lamp | Adds at a register lamp |
|---|---|---|---|---|
| up to 3 m, 30 % / 30 % | 1,610 -> 1,544 | 84 of 404 | 117 of 218 | 129 of 643 |
| up to 6 m, 30 % / 30 % | 1,110 -> 1,156 | 158 of 369 | 117 of 218 | 127 of 643 |
| up to 10 m, 50 % / 50 % | 493 -> 658 | 94 of 240 | 195 of 287 | 204 of 751 |
| up to 16 m, 60 % / 80 % (a guess) | 306 -> 527 | 50 of 185 | 300 of 398 | 270 of 815 |

Mapillary's lamps lie a median 2.6 m from the register's. Moving an object that is already within a few metres
makes it worse. Removals are right two times in three, and right three times in four where the map's lamps are
guesses. Only a third of the adds land on a register lamp. Some of the others are real lamps the city does not
own (station, campus, facades), but most are not. Stricter rules (moves only beyond 4.5 m, evidence seen 180+
days apart, adds only 3 match radii from any lamp) did not change the picture.

So the correction pass stays off by default, with moves off even when it is on. It helps where the map's own
positions are guesses, as in Chernihiv: 723 lamps placed every 32 m along the kerbs, of which only 63 match one of
Mapillary's 562. There it would add 92 lamps and remove 154. On maps with registers it should not be used for
lamps or signs.

## Results

FILLED BY THE RUNS
