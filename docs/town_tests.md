Smoke test of the CARLA towns, 2026-11-15: 40 s headless autopilot trial per town, CARLA 0.9.16 at Epic, 50 Hz, on lin4080 (RTX 4080 SUPER), the SUMO-driven autopilot, with SUMO traffic, the town's traffic lights run by SUMO, parked cars, a crowd of 20 and 20 CARLA AI walkers.

| Town | Lane links | Traffic lights run by SUMO | Parking stalls | Parked cars shown | Buildings (sound, drones) | AI walkers | SUMO vehicles shown | Real-time factor | Left road | Stuck | Put back on lane | Collisions | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Town01_Opt | 160 | 36 of 36 | 0 | 0 | 373 | 20 | 7 | 0.908 | 0 | 0 | 0 | 0 | ok |
| Town02_Opt | 112 | 24 of 24 | 0 | 0 | 445 | 20 | 7 | 0.905 | 0 | 0 | 0 | 0 | ok |
| Town03_Opt | 503 | 38 of 38 | 335 | 163 | 726 | 20 | 19 | 0.985 | 0 | 0 | 0 | 0 | ok |
| Town04_Opt | 559 | 43 of 43 | 77 | 0 | 118 | 19 | 19 | 0.982 | 0 | 0 | 0 | 0 | ok |
| Town05_Opt | 604 | 54 of 54 | 107 | 48 | 719 | 20 | 17 | 0.975 | 0 | 0 | 0 | 0 | ok |
| Town06_Opt | 490 | – of – | 0 | – | – | 0 | – | – | 0 | 0 | 0 | – | FAILED |
| Town07_Opt | 360 | – of – | 0 | – | – | 0 | – | – | 0 | 0 | 0 | – | FAILED |
| Town10HD_Opt | 200 | 15 of 15 | 120 | 61 | 324 | 20 | 14 | 0.993 | 0 | 0 | 0 | 0 | ok |

- Town01_Opt: a small town with a river and bridges, T-junctions
- Town02_Opt: a small residential and commercial town, T-junctions
- Town03_Opt: the largest urban map: a roundabout, an underpass, ramps, a large junction
- Town04_Opt: a small town in the mountains inside a figure-of-eight highway
- Town05_Opt: a squared grid town with multi-lane roads, a bridge and a highway
- Town06_Opt: long highways with many entrances and exits, Michigan lefts
- Town07_Opt: a rural village with narrow roads, barns and few traffic lights
- Town10HD_Opt: downtown: avenues, skyscrapers and a waterfront

Notes:
- Town06 and Town07 are not in the CARLA 0.9.16 release on lin4080: they come with CARLA's AdditionalMaps package.
  Their maps build (OpenDRIVE: 490 and 360 lane links; SUMO nets of 59 and 115 edges with 8 and 5 signal programs,
  167 and 35 vehicles inserted in a 300 s SUMO-only run) and the run now says which levels the server has.
- "Traffic lights run by SUMO": the town's CARLA traffic-light actors whose OpenDRIVE signal id SUMO's net carries
  (all of them in every town); none are drawn.
- "Parking stalls": Coupled CARLA's stalls in the OpenDRIVE's parking lanes and 2-3 m kerb shoulders (Town01 and
  Town02 have 0.3 m shoulders, so none; there the level's own parked cars stay); "Parked cars shown": those taken
  at 08:00 within 250 m of the driver.
- "Buildings": the level's environment objects of the Buildings label (boxes over 2 m tall, off the roads), written
  to buildings.json for sound occlusion and drone obstacles; tree tops for the drones come from the Vegetation boxes
  (Town01 2,194, Town02 649, Town03 149, Town04 9,377, Town05 3,990, Town10HD 189).
- This run had the GPU nearly to itself; earlier runs the same day with CARLA's BasicAgent as the autopilot gave the
  same counts (real-time factors 0.97-0.995 alone, 0.23-0.995 beside three other CARLA servers). Town04's 0 parked
  cars shown: the SUMO driver's random route stayed away from its lakeside road.
- The crowd's kerbs lie on the town's pavements (Town03: 739 of 780 kerb points moved onto sidewalk lanes) and its
  marked crossings are the OpenDRIVE's crosswalks.
- SUMO vehicles shown: those that came within 250 m of the driver in the 40 s (demand starts at t = 0).
