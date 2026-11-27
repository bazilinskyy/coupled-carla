# Ships in Coupled CARLA (design, draft)

Status: proposal for discussion, nothing implemented. Branch `design/ships`.

Decision (Shadab, 2026-11-27): the first use is **ships seen from the shore**. Participants stay on land (driver,
pedestrian, cyclist, AV passenger) and the ships are AI. Steering a ship from its bridge comes later.

## Goal

Ships as a new kind of actor, sailing on the water of a map, the way drones fly above it:

- AI ships sail routes past the participants, with realistic manoeuvring, lights and sound;
- several hull types, starting with the two standard research hulls **KVLCC2** and **KCS**, with more added as
  parameter sets;
- every ship is logged like the drones are (its own CSV next to `frames.csv`), and in `frames.csv` while near a
  participant, so the analysis sees it;
- later: a participant steers one from a station (helm and engine telegraph, a view from the bridge).

## Why it fits: the drone pattern

Drones (`drones.py`, `dronemodel.py`, `droneinput.py`, `droneplan.py`) already solve the hard parts:

| Drones today | Ships would do the same |
|---|---|
| Motion from an engine-free numpy model (`dronemodel.Fleet`), not CARLA physics | `shipmodel.py`: an MMG manoeuvring model, vectorised over all ships |
| Integrated on the host; each tick the pose goes into the tick's batch as `ApplyTransform` | Same: `ShipWorld.step()` before `sync.tick()`, pose queued with `carla_sim.queue` |
| Three looks: content-pack blueprint, `static.prop.mesh` with a mass (kinematic), simple shapes on 0.9.16 | Same three, with hull meshes generated from public hull geometry |
| Participant kind `drone`; station sends sticks, host moves the station's camera | Participant kind `ship`; station sends helm and telegraph, host moves the bridge camera |
| AI drones on missions (`drones.missions`) | AI ships on routes (`ships.routes`) |
| `drones.csv`, `drones.check_config` in the validator | `ships.csv`, `ships.check_config` |

A moving `static.prop.mesh` needs a mass to be movable after spawning (README, Lessons learned); `drones.visual`
already does this with physics and collisions switched off. A 320 m hull works the same way.

## Ship motion model (`coupledcarla/shipmodel.py`)

The MMG standard method (Yasukawa & Yoshimura 2015, *Introduction of MMG standard method for ship maneuvering
predictions*, J. Mar. Sci. Technol. 20(1), 37-52): three degrees of freedom (surge, sway, yaw), with the forces
split into hull, propeller and rudder parts.

- **State per ship:** position and heading in the earth frame, surge u, sway v, yaw rate r, rudder angle,
  propeller revolutions.
- **Hull:** resistance and the manoeuvring derivatives (X'vv, Y'v, Y'r, N'v, N'r, ... up to third order).
- **Propeller:** thrust from the open-water curve K_T(J), with the wake fraction and thrust deduction.
- **Rudder:** normal force from the effective inflow (flow straightening, propeller race), with the hull-rudder
  interaction factors (t_R, a_H, x'_H).
- **Actuators:** the rudder turns at a limited rate (2.32 deg/s, the SOLAS 35-to-30 degrees in 28 s rule) up to
  +-35 deg; revolutions follow the telegraph with a first-order lag.
- **Integration:** RK4 at the host tick (0.02 s) or a few sub-steps; ships are slow, so this is cheap.
- **Environment:** deep water with a uniform current, wind and the mean wave drift (done 2026-11-27); see
  "Wind, waves and current" below. Shallow water, bank and ship-ship interaction: later, if a
  study needs them.

Hull types are dataclasses like `dronemodel.Airframe`, with a `PRESETS` dict:

| | KVLCC2 (tanker) | KCS (container ship) |
|---|---|---|
| Lpp | 320.0 m | 230.0 m |
| Beam | 58.0 m | 32.2 m |
| Draught | 20.8 m | 10.8 m |
| Block coefficient | 0.810 | 0.651 |
| Design speed | 15.5 kn | 24 kn |
| MMG coefficients | Yasukawa & Yoshimura (2015), full set | to be sourced (published MMG sets from SIMMAN captive tests) |

The particulars come from the SIMMAN workshop pages; the coefficients must be copied from the papers with their
reference in the code, not typed from memory. A third hull (for example a ferry or an inland barge) is one more
preset, once its coefficients are published.

**Status of step 1 (2026-11-27):** `coupledcarla/shipmodel.py` and `tests/test_shipmodel.py` (38 tests with
current, wind and waves, 2 s, no CARLA). KVLCC2 coefficients are copied from the paper's Tables 1 and 3 and Sect. 5.1. The propeller position
x'_P is printed only in a figure, so it comes from the MIT-licensed shipmmg project's KVLCC2 example (-0.690).
Against the paper's own calculation for the 7 m model (Tables 4 and 5): turning advance and tactical diameter
4-8 % smaller, zig-zag overshoot angles within 1.6 deg, the same port/starboard asymmetry. Full scale meets the
IMO criteria, but turns about 17 % tighter than the paper's full-scale run (3.0 vs 3.6 L). That is because the
paper's full-scale resistance (Schoenherr extrapolation) is not published, so the model's R'_0 = 0.022 is kept.
The ships are a small list of scalar models, not a vectorised fleet: there are few of them.

### More ship types (wanted: catamarans, cruise ships, ...)

Each type needs a full set of MMG coefficients. Two routes:

1. **Published sets** (best): KCS and KVLCC1 (SIMMAN), the ONR tumblehome, and others in the literature. Few
   exist for catamarans or cruise ships, and catamarans have two hulls, two propellers and two rudders, which
   needs the twin-propeller, twin-rudder form of MMG.
2. **Estimated from main particulars:** empirical regressions give the hull derivatives from L, B, d and C_b
   (for example Kijima et al. 1990 for the linear and nonlinear derivatives, Clarke et al. 1983 for the linear
   ones). That gives any hull a plausible behaviour, including a cruise ship, at the cost of accuracy. Such hulls
   would be marked "estimated" in the preset's source and in the logs, as the scenery marks generated objects.

What each candidate needs, to be sourced before coding (papers, with the tables copied, not values from memory):

| Type | Example | Route | Notes |
|---|---|---|---|
| Container ship | KCS (230 m) | published (SIMMAN captive tests) | single screw, like KVLCC2 |
| Cruise ship | a generic 300 m cruise ship | estimated, unless a published set is found | twin screw, twin rudder, large windage |
| Catamaran | a fast ferry (40-100 m) | published twin-hull sets are rare: search first | twin hulls; waterjets on many fast ferries |
| Ferry / RoPax | a published RoPax set, if any | published or estimated | twin screw; also the ship passenger's first ride |
| Inland barge | for canal scenes | published inland-vessel sets exist (shallow water) | shallow water matters on canals |

Looks for each type come from the same mesh tool (step 5): hull and superstructure from the main particulars.

### Wind, waves and current

What can be added to the MMG model, in order of cost:

- **Current** (done 2026-11-27): `Environment(current_mps, current_to_deg)`. The ship's u, v_m are through the
  water, so the hull, propeller and rudder forces are unchanged and the current only adds to the track over the
  ground: exact for a uniform, steady current, and tested that way. A field along a river (speed varying with
  position) is a later extension.
- **Wind** (done 2026-11-27): `Environment(wind_mps, wind_from_deg)`, wind at 10 m from the meteorological
  direction, like Open-Meteo's `wind_speed_10m` / `wind_direction_10m`. Forces from Blendermann (1994), whose
  parameters cover 17 ship types (tanker, container ship, ferry, passenger liner, speed boat, ...), as tabulated in
  Fossen's MIT-licensed MSS toolbox; each hull carries a `Windage` (frontal and lateral areas, length overall,
  centroid of the lateral area). KVLCC2 uses the areas reported for it by Yasukawa et al. (2018) (frontal 1161 m²,
  lateral 4258 m², superstructure estimated; their full text is paywalled, so these were checked only against the
  hull's freeboard of 9.2 m plus an aft accommodation block); Loa and the centroid are estimated. The relative wind
  uses the ship's velocity over the ground. In Beaufort 9 (22.7 m/s) a head wind costs about 0.8 kn at fixed
  revolutions, and a beam wind gives about 0.4 deg of drift with about 2 deg of check helm. The autopilot has an
  integral term so it holds its heading in steady wind. Not yet modelled: gusts, and wind from the trial's own
  weather (that is wiring in `ships.py`: the same Open-Meteo values as `set_weather`).
- **Waves, effect on the course** (done 2026-11-27): the mean wave drift in a long-crested JONSWAP sea (peak
  factor 3.3), `Environment(wave_hs, wave_tp, wave_from_deg)` with Open-Meteo marine's conventions. Along the ship:
  the added resistance in regular waves of IMO MEPC.1/Circ.850/Rev.3 (2021, appendix to appendix 2; the SNNM
  method of Liu & Papanikolaou), a motion part (head to beam seas) and a reflection part per waterline segment,
  integrated over the spectrum. Checks: the head-sea curve peaks at a wave length of about one ship length and
  levels off at about 2 in short waves, as KVLCC2 tank tests show; in head seas x 1.3 (IMO's allowance for
  spreading) it is within 10 % of IMO's simple formula at Beaufort 7-9. Across the ship: an estimate, not from
  IMO: full reflection on the side (1/2 rho g A^2 per metre) in short waves, with IMO's draught factor; it is an
  upper bound, and gives KVLCC2 about 3 deg of drift in Hs 5.5 m beam seas. Estimated for KVLCC2: the waterline
  entrance and run lengths (0.2 and 0.25 Lpp) and the pitch gyration (0.25 Lpp). Not modelled: a yaw moment from
  waves, spreading (short-crested seas), and the ship's oscillations (next bullet).
- **Waves, motions** (expensive): heave, roll and pitch need a seakeeping model (response amplitude operators per
  hull, or a strip-theory solution). Seen from the shore, a ship's roll and pitch in swell is visible, so a simple
  RAO-based motion per hull may be worth it later.
- **Waves, looks:** CARLA's water is flat tiles. Moving water needs a water material or plug-in on CARLA 1.0
  (UE 5's water system); that is a rendering task separate from the ship dynamics.

**Tests (`tests/test_shipmodel.py`), no CARLA needed**, as `test_dronemodel.py` does:

- straight run at the design revolutions reaches the design speed;
- 35 deg turning circle: advance and tactical diameter close to the published free-running results, and within
  the IMO criteria (MSC.137(76): advance < 4.5 L, tactical diameter < 5 L);
- 10/10 and 20/20 zig-zag: first and second overshoot angles close to the published results;
- rudder rate and angle limits hold; zero rudder keeps a straight course.

## Frames

MMG works in an earth frame x north, y east, heading psi clockwise from north; body x forward, y starboard.
CARLA is x east, y south, yaw clockwise from east. So `carla_x = y_east`, `carla_y = -x_north`,
`carla_yaw = psi - 90 deg`. One pair of functions, tested both ways, as `drones.to_carla` / `from_carla`.

## Water and maps

What exists: `scenery.py` lays water tiles (`SM_TileLake`) 0.5 m below the ground where OSM has `natural=water`,
`landuse=basin|reservoir` or `waterway=riverbank`. The water is flat and has no collision role for anything.

What ships need:

1. **Navigable area:** the same OSM water polygons, saved at map build time (`ships_water.json`), so the model
   can tell water from land without CARLA. Leaving it is a grounding (logged, the ship stops), like a drone's
   crash check. Water mapped only as a `waterway=river` line has no area: such maps need the area mapped, or a
   buffer around the line as a fallback.
2. **Fairways (optional):** OSM `waterway=fairway` and buoys (`seamark:*`) for AI routes and for the scenery.
3. **Water level:** one height per map (the water tiles' top), which is the ship's waterline.
4. **A port map:** KVLCC2 and KCS do not fit Eindhoven's canals. A first test map with deep water and real
   traffic, for example the Nieuwe Waterweg / Maasvlakte (Rotterdam) or the Scheldt near Antwerp. The map maker
   would need a "water" check that warns when a hull is too large for the map's water.
5. **Sight distance:** a 300 m ship is visible from the shore kilometres away, while SUMO vehicles appear only
   within `lod_radius_m` (250 m) and scenery within `radius_m` (400 m). Ships get their own, larger radius
   (`ships.visible_m`, for example 3 km): there are few of them, so all ships within it can exist in CARLA.
   The far shore itself stays outside the scenery radius; a cheap far-away look (buildings only) is a later
   step. (From a bridge, in the later ship participant, the same applies more strongly.)
6. **Maps with almost no roads:** `mapbuild.prune()` keeps the largest connected road network; a map of open
   water may have very few roads. To check: what happens to a map whose road network is tiny.

## Looks

- **Meshes:** `tools/ship_mesh.py`, like `tools/drone_mesh.py`: a glTF hull from the published hull geometry
  (SIMMAN offsets/IGES), plus a simple superstructure, funnel, deck cargo for KCS, navigation lights (red port,
  green starboard, white masthead and stern). Origin at the waterline amidships.
- **Content pack:** `BP_CCShip_KVLCC2`, `BP_CCShip_KCS` through `tools/ue_drone_assets.py`'s route (a
  `tools/ue_ship_assets.py`), listed in `BlueprintParameters.json` as `cc_ship_kvlcc2`, `cc_ship_kcs`.
- **Fallbacks:** `static.prop.mesh` with the mesh and a mass (kinematic); on 0.9.16 a box hull of the right length,
  beam and freeboard. `ships.visual: auto | blueprint | mesh | shapes | none`, as for drones.
- **Not in version 1:** waves, wake, spray, ship motions in waves (heave, roll, pitch).

## Seen from the shore (first use)

What a participant on land needs from a passing ship:

- **Placement in sight:** routes are planned so ships pass the participants when the study needs them:
  `start_s` or a trigger (a participant reaches a point, as coupled-sim-style triggers do for AI cars), so a ship
  is in view at the moment of interest rather than somewhere on its route.
- **Night:** navigation lights (masthead, sidelights, stern) drawn near participants, like the drones' lights,
  and deck lights on CARLA 1.0.
- **Sound:** engine and propeller hum, and the horn (sound signals for manoeuvres: one short blast for altering
  to starboard, etc.) through `soundscape.py` as `sound_event`s, as the living city's horns are.
- **Logging and analysis:** ship rows in `frames.csv` within `logging.radius_m` of a participant (kind `ship`),
  events `ship_in_view` / `ship_passed` with the closest distance, and the ships in the trial viewer.
- **Scenes worth a first map:** a quay or dyke road along a busy fairway (Nieuwe Waterweg at Maassluis or Hoek
  van Holland, where KVLCC2- and KCS-sized ships pass close to the road), and a canal with inland barges and a
  movable bridge (needs an inland hull; see Open questions).

## Ship passenger (like the AV passenger)

Asked for by Shadab (2026-11-27): a participant who rides on a ship without steering it, as `av_passenger` rides
in a scripted AV.

- **Kind `ship_passenger`**, with `ship: <AI ship id>` and a place on board (`deck: bridge | bow | stern |
  promenade` or an explicit `{x, y, z}` in the ship's frame).
- The AI ship sails its route; the host moves the passenger's camera with the ship every tick, as it moves a
  drone pilot's camera (`ApplyTransform` in the tick's batch, so camera and ship never disagree).
- The passenger looks around (mouse or VR head tracking); walking on deck could follow later (a walker moved in
  the ship's frame).
- With wave motions (above), the passenger feels them in VR; without, the deck is steady.
- Logging: the passenger's view direction and position on board, the ship's state, as `av_passenger` is logged.

This comes right after the AI ships, since it reuses them and needs no ship controls.

## Participants and stations (later)

- **Kind `ship`** in `validate.KINDS`, with `hull: kvlcc2 | kcs`, `spawn: {lat, lon, heading}`, `speed_kn` at
  start, and `input: keyboard | gamepad | bot`.
- **Controls (station):** A/D rudder in steps (or an axis), W/S engine telegraph in steps (full astern ...
  full ahead), autopilot heading hold on/off; Esc, F (visual sync) as everywhere.
- **Station message:** `ship_input {rudder, telegraph, autopilot}`, handled in `ShipWorld.on_message`.
- **View:** the host moves the station's camera to the bridge (eye position per hull, configurable); views
  `bridge`, `bridge_wing`, `chase`.
- **HUD:** heading, speed over ground, rate of turn, rudder angle, ordered and actual revolutions (a minimal
  conning display); later the in-car display machinery (`incar.py`) can draw a proper one.
- **Bot:** follows waypoints with a heading autopilot (PD on heading, rudder limited) and a speed schedule.

## AI ships

`ships.routes`: `[{hull, path: [[lat, lon], ...], speed_kn, start_s}]`, sailed by the same model with the bot
autopilot. Later, real tracks: open historical AIS (for example the Danish Maritime Authority's, or NOAA's
MarineCadastre for US waters) replayed as routes, as the buses replay GTFS, and live AIS (aisstream.io, an API
with a free key) for `time: now`, as the buses use GTFS-Realtime. (aiboatapp.com, suggested 2026-11-27, is a
closed consumer app with nothing reusable; its AIS comes from aisstream.io.)

## Code changes (where)

| File | Change |
|---|---|
| `coupledcarla/shipmodel.py` (new) | MMG model, hulls, autopilot, standard manoeuvres, current, wind and wave drift (done) |
| `coupledcarla/ships.py` (new) | `ShipWorld`: spawn looks, step, grounding checks, camera, HUD, `ships.csv`, `check_config` |
| `coupledcarla/shipinput.py` (new) | keyboard/gamepad helm and telegraph, bot |
| `coupledcarla/run.py` | create `ShipWorld` beside `DroneWorld`; `add_pilot` for kind `ship`; step before `sync.tick()`; messages, logging, summary, close |
| `coupledcarla/station.py` | kind `ship`: send `ship_camera`, `ship_input`; show the HUD |
| `coupledcarla/config.py` | a `ships` section with defaults |
| `coupledcarla/validate.py` | kind `ship`, `ships.check_config` |
| `coupledcarla/mapbuild.py` / `scenery.py` | save the navigable water polygons and water level |
| `coupledcarla/logger.py` | ship rows, or `ships.csv` written by `ships.py` as `drones.csv` is |
| `tools/ship_mesh.py`, `tools/ue_ship_assets.py` (new) | meshes and content-pack blueprints |
| `tests/test_shipmodel.py`, `tests/test_ships.py` (new) | model tests; config, frames, grounding tests |
| `experiments/rotterdam_ships.yaml` (new) | a port map with one ship participant and AI ships |
| `README.md` | a Ships section |

## Steps

1. `shipmodel.py` with KVLCC2 and its tests (turning circle, zig-zag). No CARLA. **Done.**
2. More hulls (KCS first, then cruise ship, catamaran, ferry), once their coefficients are sourced. Current, wind
   and wave drift in the model: **done** (2026-11-27); README figures from `tools/ship_figures.py`.
3. `ships.py`: AI ships on routes with the `shapes` look and `ships.visible_m`, a port map with a quay road,
   `ships.csv` and ship rows in `frames.csv`. CARLA 0.9.16.
4. Seen from the shore: triggers, navigation lights, sound, `ship_in_view` / `ship_passed` events, the trial viewer.
   Then the ship passenger.
5. Meshes and content-pack blueprints (CARLA 1.0), since ships seen from the shore must look like ships.
6. Later: AIS replay, a current field along a river, gusts, an inland hull for canals, the ship participant at a station.

## Open questions

- **KCS coefficients:** which published MMG set to use.
- **First scene:** a fairway quay (Nieuwe Waterweg) for KVLCC2 and KCS, or a city canal, which needs an inland
  hull with published MMG coefficients instead.
- **Interaction with roads:** movable bridges and ferries touch SUMO traffic; out of scope for now?
- **Pavlo:** does this fit the project's roadmap, and should it live in this repository or as a plug-in beside it?
