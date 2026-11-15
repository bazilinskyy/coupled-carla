# Coupled CARLA on CARLA 1.0: test matrix

`tools/test_carla10.py` runs the features that had been tested only on CARLA 0.9.16 against a CARLA 1.0 (Unreal
Engine 5.8, `ue58-dev`) server, one short trial each, and prints the table below. Run on 2026-11-15 on lin5080: the
CARLA `ue58-dev` (38b72cc52) editor binary in `-game -nullrhi` mode (no rendering, CPU only; the RTX 5080 was busy
with other jobs), RPC port 2080, the client in the same machine's CARLA 1.0 venv (Python 3.10), SUMO 1.28. Trials of
60 s with `simulation.profile: true` and no real-time sleep (so RTF above 1 is headroom), stations in real time.

```bash
python tools/test_carla10.py --port 2080 --launch scratch/srv58fg.sh --duration 60 \
    --mapmaker-exp experiments/places/strijp_s_eindhoven.yaml          # 13 tests, about 25 minutes
python tools/test_carla10.py --list
```

`--launch CMD` starts the server itself (CMD runs it in the foreground), restarts it when a test needs another map
or the server passed 17 GB, waits for 28 GB of available memory before each start and kills it at 22 GB RSS or under
10 GB available (the 1.0 editor server grows with every world it loads). Without `--launch` the tests attach to a
running server.

## Results (headless, `-nullrhi`)

| Feature | Result | Numbers | Notes |
|---|---|---|---|
| Eindhoven twin (baked `EhvCentre`), `eindhoven_crowd.yaml`'s scene with `--local` bots: pedestrian, AV passenger, driver, two AI cars, two AI walkers; SUMO traffic, real bus lines, parked cars, scenery (1.0 catalogue, snapped), signals, CROWD pedestrians | ok | RTF 1.15, 17.4 ms per tick (server tick 9.9, `carla_to_sumo` 0.90), 5 SUMO vehicles shown, 0 collisions, 2,824 scenery meshes, 739 parked cars, 165 signalled lanes, 24 crowd walkers, level load 9 s | no bus within 250 m in 60 s (13 buses running at 14:00 on 60 lines over the 2 km box) |
| CARLA 1.0: World Partition streaming, rpclib stall | ok | ground under a point 1.1 km from the origin: none with the spectator parked away, found 0.2 s after the spectator moved over it; tick 7.8 ms alone, 7.4 ms after two fire-and-forget calls, 7.4 ms after `apply_batch_sync`; 25 vehicle blueprints (bike 3, bus 1, car 13, motorbike 4, truck 2, van 2 as the soundscape classes them) | no 40 ms stall with client and server on one Linux machine |
| Sound `--record-audio`, driver first | ok | `audio.wav` 60.04 s for 3,002 ticks: 2,647,764 samples, exactly ticks x 0.02 s x 44,100; `start_frame` in `audio.json`; 2.4 ms per tick | |
| Sound `--record-audio`, pedestrian first | ok | the same, 2.4 ms per tick, 165 signal poles for accessible pedestrian signals | |
| Weather: real, time now | ok | Open-Meteo 17:00: 100 % cloud, 0.1 mm/h, 14.8 km/h, 9 C -> cloudiness 60, wind 29.6; sun -4 deg, night; 61 lamps lit | GTFS-Realtime needs `gtfs-realtime-bindings` (missing in that venv): buses spread by the timetable |
| Night: street lights, headlights | ok | sun -51 deg, 61 lamp blueprints lit within 200 m, RTF 1.47 | the lights themselves need frames (render checks) |
| Season: winter | ok | 5,383 deciduous trees bare (small), RTF 1.30 | |
| Drones (`eindhoven_drones.yaml`): bot pilot and AI missions | ok | 13 AI drones in 6 missions, 16 phase changes, 3 parcels delivered, 0 crashes, drones 4.4 ms per tick, RTF 1.67, look `blueprint` (the content pack) | observer camera and FPV need frames |
| Film cameras (`make_videos.py render eindhoven_pedestrian_ehmi --no-launch`) | ok | trial rc 0, RTF 0.96, 402 camera poses (the 5 fps grid of 40 frames covered), `--record-audio` 17.54 s = start + length + tail, clean exit; 0 images (the server cannot render) | images, encoding, poster need a rendering server |
| Stations (`station_dryrun.py`, three bot stations) | ok | all three joined in 0.3 s, loops at 49.8 of 50 Hz, round trip 0.2-0.5 ms, host RTF 0.99; exit 1 (no images: ready_s and image_share fail, and the report says the server cannot render) | images, latency, ready time need a rendering server |
| OpenDRIVE world: `eindhoven_centre` (`generate_opendrive_world`) | ok | load 20.7 s, 2,246 spawn points, RTF 1.17, 17.1 ms per tick (server 10.8, `carla_to_sumo` 0.64), 27 SUMO vehicles shown, 0 collisions, 5,337 scenery meshes, 798 parked, 175 signalled lanes | |
| OpenDRIVE world: map-maker map (`mapmaker "Strijp-S, Eindhoven" --size small`, built on lin5080) | ok | load 10.2 s, 1,941 spawn points, RTF 1.62, 12.3 ms per tick (server 7.8, `carla_to_sumo` 0.45), 13 SUMO vehicles and a bus shown, 0 collisions, 4,439 scenery meshes, 596 parked, 139 signal heads | run after merging main (see below) |
| A camera on a `-nullrhi` server | ok | spawn works, `listen` raises "this server cannot render (-nullrhi) and no rendering secondary server is connected to own this camera sensor", the server stays up | older notes said the server crashed (SIGSEGV); not on 38b72cc52 |

After merging main (new traffic-light heads, look, drone routing) the twin, probe, pedestrian sound, drones, film,
stations, `eindhoven_centre` and the map-maker map ran again with the same results (twin: RTF 1.05, 144 signal
heads; `eindhoven_centre`: 11 SUMO vehicles, its bus now logged as a bus; drones: 3 deliveries, 0 crashes; film 45 s
now that the crowd's places are cached).

Server memory: 11-14 GB after start, 14.8 GB after the twin, never near the 22 GB watchdog (two restarts per run:
twin -> OpenDRIVE maps -> camera probe).

## Bugs found and fixed

- **Cameras on a `-nullrhi` server** stopped a trial (film), a drone pilot's view or a station with an exception from
  `listen`. `compat.listen()` catches exactly that refusal: the camera actor stays (it can still be moved, film keeps
  its poses and timing grid, `film.json` says `images: false`), stations report `no_render` and the dry run says why
  images are missing.
- **Buses logged, heard and filmed as cars on 1.0**: the runner compared the blueprint with `traffic.bus_type`
  (`vehicle.mitsubishi.fusorosa`, shown on 1.0 as `vehicle.fuso.mitsubishi`); the kind now comes from the SUMO vehicle
  class. The soundscape gave 1.0 buses a car's CNOSSOS category and the 1.0 police car no siren; film `bus` targets
  found none.
- **Sound vs picture weather**: a preset weather with `simulation.time` played Open-Meteo's rain (1.4 mm/h, wet
  roads) under a dry sky; rain, wind and wetness now follow the visible weather unless `weather: real`.
- **Crowd crossings on baked levels**: no `map.osm`, so no marked crossings; the twin's OpenDRIVE crosswalks and
  pedestrian signals are used (211 of its 241 snap onto kerb segments).
- **Overture places fetched in the middle of a trial**: a first trial with walkers queried Overture over S3 after the
  world had loaded (57 s on the twin, about 5 minutes on `eindhoven_centre`); now fetched when the map is built.

## Still open

- The SUMO network starts empty, so 60 s trials show few SUMO vehicles near the participants (5 on the twin scene,
  27 on `eindhoven_centre`); not 1.0-specific.
- The tree has no Dutch sign faces (`/Game/CoupledCarla/Signs` is not in lin5080's `~/carla58/carla`): 56-65 register
  signs spawn as blank plates. The 1.0 server there also lacks `SM_TrafficLight__VPole_Main`, so traffic-light heads
  are drawn.
- A map-maker build spent about 90 minutes in Overture's S3 queries on lin5080 (the place search, OSM, 3DBAG, BGT and
  the network took 5 minutes).

## Render checks (need a GPU server)

On win5090 (the packaged CARLA 1.0 with EhvCentre and the content pack on Windows, the client in WSL):

```bash
bash tools/render_checks_carla10.sh env10.sh <port> .venv10/bin/python
# = python tools/test_carla10.py --render-checks --host 127.0.0.1 --port <port> --duration 40
```

| Check | What it looks at |
|---|---|
| film | `eindhoven_pedestrian_ehmi` (pedestrian's eyes, `--record-audio`): 40 frames on the 5 fps sim-time grid, the encoded mp4 with the recorded sound, poster and preview |
| stations | three bot stations at 1280x720 every tick: ready time, image share, tick-to-image latency (labcheck thresholds) |
| drones | the pilot's FPV frames and the observer camera on the square: counts and brightness |
| night | the driver's frames every 2 s at 22:30: brightness and crushed share with the street lights |
| season | the same on a winter day |
