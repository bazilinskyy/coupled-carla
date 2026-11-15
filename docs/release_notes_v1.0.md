# Coupled CARLA 1.0

Coupled CARLA is a human-factors driving simulator for real places. Type a place, get a drivable
[CARLA](https://carla.org) world built from open data, fill it with [SUMO](https://eclipse.dev/sumo/) traffic, and
put drivers, pedestrians, cyclists, AV passengers and drone pilots in it together, each at their own station, with
every trial logged at 50 Hz. It is the successor of [coupled-sim](https://github.com/bazilinskyy/coupled-sim) and
is free software under the MIT License.

Project page with the demo videos: <https://bazilinskyy.github.io/coupled-carla/> ·
Code and full documentation: <https://github.com/bazilinskyy/coupled-carla>

## What is in 1.0

**Maps of real places**
- [Make a map of any place](https://github.com/bazilinskyy/coupled-carla#make-a-map-of-any-place): the map maker
  (a local web page or one command) finds a place in any language, guesses the driving side and the traffic, and
  builds an OpenDRIVE map for CARLA with a matching SUMO network in a few minutes. No programming needed
  ([step by step](https://github.com/bazilinskyy/coupled-carla/blob/main/docs/make_a_map.md)).
- [Real-world scenery, terrain and weather](https://github.com/bazilinskyy/coupled-carla#real-world-scenery-terrain-and-weather)
  from free open data: buildings at their real footprints and heights (3DBAG, BD TOPO, NYC, PLATEAU, Overture),
  street trees, furniture, lamps, road signs, ground cover, lidar or global terrain, parked cars, seasons, and the
  weather and sun of a real place and time.
- [19 cities](https://github.com/bazilinskyy/coupled-carla#cities) ready to drive (Eindhoven plus three per inhabited
  continent, left- and right-hand traffic), and [CARLA's own towns](https://github.com/bazilinskyy/coupled-carla#standard-carla-maps)
  (Town01-Town07, Town10HD) with every feature.

**Traffic**
- [SUMO traffic](https://github.com/bazilinskyy/coupled-carla#traffic) with EIDM car following and the sublane
  model, car fleets from open vehicle registers (NL, New York, New Zealand, Brazil), demand fitted to open traffic
  counts, cyclists on the real cycle network, urban speed limits per country.
- [Real public transport](https://github.com/bazilinskyy/coupled-carla#public-transport): OSM bus lines on their real
  streets at their GTFS frequencies, live bus positions from GTFS-Realtime.
- A [pedestrian crowd](https://github.com/bazilinskyy/coupled-carla#coupled-experiments-phase-2) calibrated per country
  with the CROWD dataset (speeds, crossing rates, mid-block crossings, accepted gaps).

**Experiments with people**
- [Coupled experiments](https://github.com/bazilinskyy/coupled-carla#coupled-experiments-phase-2): stations for a
  driver (keyboard or Logitech G25 with force feedback), a pedestrian, an AV passenger and a drone pilot on any
  machine of the lab network; scripted AI cars and walkers with coupled-sim-style triggers; eHMIs on the front, rear,
  roof, road or as eyes; eye-contact events; visual sync; latency logging.
- [Study tools](https://github.com/bazilinskyy/coupled-carla#study-tools): counterbalanced trial plans, an experiment
  validator, continuous responses, take-over requests, and the
  [session dashboard](https://github.com/bazilinskyy/coupled-carla#session-dashboard) to run a whole session from a
  browser with a live map; a [lab guide](https://github.com/bazilinskyy/coupled-carla/blob/main/docs/lab_guide.md) and
  [session checklist](https://github.com/bazilinskyy/coupled-carla/blob/main/docs/session_checklist.md).
- [Sound, in-car displays and eHMIs](https://github.com/bazilinskyy/coupled-carla#sound-in-car-displays-and-designed-ehmis):
  a calibrated soundscape from published acoustic models (CNOSSOS-EU, ISO 9613, binaural), audio for videos,
  instrument cluster, HUD and centre screen on each car's own panels, haptic cues, eHMIs from design files.
- [Drones](https://github.com/bazilinskyy/coupled-carla#drones): a pilot station and AI missions (delivery, swarm,
  patrol, hover) with coupled-sim's flight model, routes kept clear of buildings at SORA clearances.

**Data**
- [Logging](https://github.com/bazilinskyy/coupled-carla#logging-and-data): coupled-sim-style CSV at 50 Hz, events,
  live JSON over UDP, SUMO trajectories of the whole map, CARLA's recorder.
- [Analysis](https://github.com/bazilinskyy/coupled-carla#analysis) of every encounter (gap, time to collision, PET,
  accepted gap, stops, eye contact) and a browser [trial viewer](https://github.com/bazilinskyy/coupled-carla#trial-viewer).

**CARLA 1.0 and digital twins**
- The same experiment files run on [CARLA 1.0](https://github.com/bazilinskyy/coupled-carla#running-on-carla-10)
  (Unreal Engine 5.8), with blueprint stand-ins, a 1.0 mesh catalogue and a fix for its RPC stall.
- [The Eindhoven digital twin](https://github.com/bazilinskyy/coupled-carla#the-eindhoven-digital-twin): central
  Eindhoven baked into a CARLA 1.0 level from Dutch open data (AHN, PDOK aerial photos, 3DBAG with facade styles,
  lit windows at night), in real time with SUMO traffic; downloadable as a
  [content pack](https://github.com/bazilinskyy/coupled-carla/releases/tag/twin-eindhoven-2026-11-15).
  [A twin of any place](https://github.com/bazilinskyy/coupled-carla#a-twin-of-any-place) from the best open data
  of its country, and optional [photorealistic tiles](https://github.com/bazilinskyy/coupled-carla#photorealistic-tiles-optional)
  with your own Google key.

**Easier install (new in 1.0)**
- `pip install -e ".[carla,maps]"` from a clone (a `pyproject.toml`), with commands `coupled-carla-map`,
  `coupled-carla-run`, `coupled-carla-session`, `coupled-carla-station`, `coupled-carla-build`,
  `coupled-carla-validate` and more; `python -m coupledcarla.<module>` works as before. Extras: `carla` (the 0.9.16
  client), `maps` (Overture, AHN, GTFS-Realtime), `tools`, `dev`. On CARLA 1.0 install `.[maps]` and the 1.0 wheel.
- A [Dockerfile](https://github.com/bazilinskyy/coupled-carla/blob/main/docs/docker.md) for the map maker and the
  SUMO-side tools on Linux (CARLA stays on the host).
- A [project page](https://bazilinskyy.github.io/coupled-carla/) where the demo videos play in the browser, a
  contributing guide and issue templates.

## Tested on

| What | Where | Versions |
|---|---|---|
| Driving, the 19 cities, the towns, coupled experiments, session dashboard, videos | Ubuntu 22.04, RTX 4080 SUPER (driver 560) | CARLA 0.9.16 release package, Epic quality, 50 Hz; Python 3.12; SUMO 1.28 (eclipse-sumo wheel) |
| CARLA 1.0 features (13 checks), twins, drones | Ubuntu 22.04, RTX 5080 (driver 595), headless (`-nullrhi`) | CARLA `ue58-dev` source build (Unreal Engine 5.8) |
| Rendered twin, stations at the rendering server | Windows 11, RTX 5090; client in WSL 2 (Ubuntu 24.04) | CARLA 1.0 Shipping package, built from source |
| Map maker on five new places | Ubuntu 22.04 | Python 3.12 |
| Unit tests (about 670, no CARLA or SUMO needed) | GitHub Actions, Ubuntu | Python 3.10 and 3.12 |

Maps can be made on macOS (the launcher `Make a map.command`); CARLA does not run on a Mac, so they are driven on a
Windows or Linux PC.

## Known limits

- On OSM maps the buildings are CARLA's stand-in models at the real footprints and heights, not the real facades;
  there are no road markings, and lamps give no light on CARLA 0.9.16. Real facades, aerial-photo ground and markings
  need a CARLA 1.0 twin.
- Pedestrians walk by direct control (CARLA 0.9.16 serves no navigation mesh for OpenDRIVE worlds); SUMO persons are
  not mirrored; eHMIs are drawn with debug lines (they glow, and an AV passenger sees them mirrored).
- Signal programs are CARLA's converter defaults (42 s green per approach), not the real ones; public transport is
  buses only.
- CARLA 1.0 needs a recent NVIDIA driver (560 fails at start-up, 595 works); its towns are not tested yet; the SUMO
  bridge on 1.0 is slower than on 0.9.16 unless the RPC fix is used.
- Remote stations receive raw camera images (about one full-HD station per gigabit link, wired only); multi-GPU
  secondaries are written but not tested on GPUs; no VR yet; no study with human participants yet (bot stations
  only).
- Photorealistic tiles are built and checked without a key, not yet seen with one on a rendering server.
- The Docker image has not been built on a test machine yet (none had Docker); the same install was tested in a
  fresh Python environment.

More in the README's [Known limits](https://github.com/bazilinskyy/coupled-carla#known-limits) and
[Roadmap](https://github.com/bazilinskyy/coupled-carla#roadmap).

## Downloads

- Source code: the archives below, or `git clone https://github.com/bazilinskyy/coupled-carla.git`.
- Demo videos: release [demo-videos](https://github.com/bazilinskyy/coupled-carla/releases/tag/demo-videos).
- The Eindhoven twin for CARLA 1.0 (Linux and Windows content packs):
  [twin-eindhoven-2026-11-15](https://github.com/bazilinskyy/coupled-carla/releases/tag/twin-eindhoven-2026-11-15).

## Credits

Coupled CARLA is made at Eindhoven University of Technology by Pavlo Bazilinskyy. It builds on CARLA (MIT; its SUMO
co-simulation bridge is vendored and modified), Eclipse SUMO (EPL-2.0), coupled-sim, and the open data of
OpenStreetMap contributors (ODbL; built maps are derived databases under the ODbL) and many national and municipal
data publishers, listed with their licences in
[Credits, data and licences](https://github.com/bazilinskyy/coupled-carla#credits-data-and-licences).

## How to cite

> Bazilinskyy, P. (2026). Coupled CARLA: a human-factors driving simulator for real places (Version 1.0.0)
> [Computer software]. https://github.com/bazilinskyy/coupled-carla

and the coupled simulator it builds on:

> Bazilinskyy, P., Kooijman, L., Dodou, D., & De Winter, J. C. F. (2020). Coupled simulator for research on the
> interaction between pedestrians and (automated) vehicles. 19th Driving Simulation Conference (DSC). Antibes, France.

GitHub's "Cite this repository" button gives the software citation in APA and BibTeX
([`CITATION.cff`](https://github.com/bazilinskyy/coupled-carla/blob/main/CITATION.cff)).
