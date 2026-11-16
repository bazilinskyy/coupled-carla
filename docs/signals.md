# Traffic lights: real data, Dutch programmes and what access to real timings needs

Coupled CARLA runs every traffic light in SUMO and draws it in CARLA (README, [Traffic lights](../README.md#traffic-lights)).
This page covers where the signal programmes come from (`traffic.signals`), which real signal data exists for
Eindhoven and how to get it, the data formats Coupled CARLA reads, and how the programmes were checked. Code:
`coupledcarla/signalplan.py` (controllers, Dutch programmes), `coupledcarla/signaldata.py` (real data: replay and
live), `tools/signal_check.py` (before/after in SUMO), `tools/signal_heads.py` (CARLA's heads against SUMO).

## The modes

| `traffic.signals` | What runs | When |
|---|---|---|
| `dutch` | one controller per junction, matched to the city's list of signalised junctions; Dutch signal groups, conflict-free stages, CROW amber, computed all-red, vehicle-actuated | `auto` in the Netherlands without real data |
| `real` | the recorded phases of the chosen hour (`signal_data`), or a live feed; `dutch` where the data has no junction | `auto` when `signal_data.log` + `groups` or `signal_data.live` are set |
| `sumo` | netconvert's programmes, as before: one light per OpenDRIVE signal, 42 s green, 3 s amber, 3 s red, all in step | `auto` outside the Netherlands |

Before this, the SUMO net made from the OpenDRIVE (`mapbuild.sumo_net`, `netconvert_carla.py`) had a light per
OpenDRIVE signal: on `eindhoven_centre`, 210 one-link lights at 23 nodes, all green for 42 s and red for 6 s at the
same moments. Every stream of a junction was green together (crossing roads, left turns and cyclists sorted out by
right of way only), and links the OSM signal did not reach had no light at all (at Kennedylaan/Fellenoord 17 of 43
links were signalised).

## Real signal data for Eindhoven (researched 2026-11-16)

No live signal timing (SPaT) and no recorded phase logs of Eindhoven are open. What is open, and what needs the
municipality's permission:

| Source | What | Time resolution | Licence | Access |
|---|---|---|---|---|
| [data.eindhoven.nl `locaties-verkeerlichten`](https://data.eindhoven.nl/explore/dataset/locaties-verkeerlichten/) | 180 signalised junctions: name, point | static (updated at most twice a year) | Public Domain | open; used (fetched once per map, `real/signal_locations.json`; a copy is bundled) |
| Gemeente Eindhoven, *Nota Verkeerslichten 2024* (Bijlage 6) | the VRI number of each junction (174), the city's timing policy | static | public document | open; the numbers are bundled (`coupledcarla/data/signals/eindhoven_vri.csv`), the policy values are used |
| [map.udap.nl](https://map.udap.nl/) (UDAP, NDW / Ministerie van IenW, run by Monotch) | the iVRIs (intelligent traffic lights) of the Netherlands: 1,389, 13 of them Eindhoven's | static list | not stated | open, anonymous JSON; used only to mark junctions that publish SPaT (`ivri_eindhoven_region.csv`) |
| UDAP live stream (Talking Traffic) | live SPaT (signal phase and timing), MAP (intersection topology), SSM | SPaT on change, up to 10/s (about 1/s), 3 s time-to-live; MAP on connect and on change | not stated; each road authority decides who gets its data | account and the road authority's permission; ASN.1 UPER over TLS with a token |
| NDW Dexter, VLOG product | recorded V-Log (every signal state and detector event, `.vlg` with `.vlt`/`.vlc` configuration), waiting-time reports | event level | not stated | Dexter account; only road authorities that deliver V-Log are in it (Den Haag, Stedendriehoek); Eindhoven is not listed |
| Gemeente Eindhoven's own V-Log and CCOL programmes | the city stores V-Log of every VRI (streaming and file logging are required, central system MobiMaestro), the control programmes (CCOL) and their parameters | event level | none published | on request to the municipality |
| CROW Handboek verkeerslichtenregelingen 2022 | amber times, design rules | | cited values | the values used are below |

Coverage of the boxes the experiments use:

| Box | Listed junctions (city) | of which iVRI (could stream SPaT) |
|---|---|---|
| `eindhoven_centre` (51.4360 5.4700 51.4500 5.4930) | 16 | 0 |
| TU/e campus (51.444 5.479 51.4535 5.501) | 5 | 2 (VRI 2006 O.L. Vrouwestraat - J.F. Kennedylaan, EIN2007 O.L. Vrouwestraat - Th. Fliednerstraat) |
| Ring area (51.418 5.437 51.462 5.508) | 85 | 13 (all on the Ring: EIN2001-2004, 2006, EIN2007, EIN2088, EIN2089, EIN3008-3012) |

Being listed on UDAP does not prove a junction streams (nationally 1,451 iVRIs were registered in 2024, 930 of them
exchanging data), and the Nota says Eindhoven's iVRIs mostly send only the current state (red, amber, green) for now,
time-to-green "only once very accurate". Rijkswaterstaat's and the province's iVRIs near Eindhoven (A2/A67
Leenderheide, Veldhoven N2) are outside the boxes.

### What you would have to request (nothing has been requested)

1. **Recorded phases (the best route, for `replay`)**: ask the Gemeente Eindhoven, verkeersmanagement, via
   data@eindhoven.nl, for V-Log recordings (`.vlg` with their `.vlt`/`.vlc`) and the CCOL control programmes (with the
   signal-group layout, `fc` numbers per approach and turn) of the junctions you model (`eindhoven_centre`: VRI 1001,
   1002, 1004, 1015, 1016, 1017, 1019, 1020, 1082, 2025, 2026, 2028, 2030, 2034, 2035 and 2115, and 1021 at its
   western edge; the TU/e campus: 2006-2009 and 2030; the Ring: the iVRIs above), for a few weekday hours (e.g. 07:00-09:00 and 16:00-18:00) and a night hour. Say it is for research
   and how the data will be stored and published (aggregated programmes, not raw logs, unless they agree). If they
   ask for a formal route, it is a Woo request (Wet open overheid) to the same municipality.
2. **Live SPaT (for `live`)**: the municipality's written permission for the junctions (above), then an e-mail to
   the Ministerie van IenW at info@talking-traffic.com with the organisation, a contact name, e-mail and phone; that
   opens a test-access ticket, and Monotch's UDAP support creates the account. A listener is built on the TLEX broker
   interfaces (docs: kennisbank.crow.nl, "UDAP Deployment"; monotch-support.github.io/subject-interface-v1), followed
   by a self-assessment before approval. The stream is ASN.1 UPER; it needs a decoder that writes the rows below.
3. **NDW Dexter** has V-Log only for the authorities that deliver it; to ask whether Eindhoven's could be added:
   mail@servicedeskndw.nu.

TLEX Interchange (Monotch, Azure Marketplace, from $40 a day) is a message platform, not a source of Dutch SPaT.

## Data formats `real` reads

- **Phase log** (`traffic.signal_data.log`, a file or a folder of them): CSV with a header
  `time,controller,signal_group,state`, or NDJSON lines with those keys; one row per state change.
  `time`: ISO 8601 (local time when it has no zone) or Unix seconds; `controller`: the VRI number (`1001`, `EIN2001`,
  `vri1001`) or a controller id of the plan; `signal_group`: the controller's number (`02`, `fc22`); `state`: `G`,
  `g`, `Y`, `R`, `O` or SPaT event states (`protected-Movement-Allowed`, `permissive-Movement-Allowed`,
  `protected-clearance`, `permissive-clearance`, `stop-And-Remain`, `stop-Then-Proceed`, `dark`). A V-Log converts to
  it with its configuration (the state changes of the `fc` signal groups); a SPaT decoder writes the same rows.
- **Signal groups** (`traffic.signal_data.groups`): JSON
  `{"1001": {"02": {"from": 0, "turns": "sr", "mode": "car"}, "22": {"from": 0, "turns": "lsr", "mode": "bike"}}}`:
  for each signal group the compass bearing its traffic comes from (0: from the north), the turns it signals and the
  mode (car, bike, bus); from the CCOL layout or the MAP message. SUMO's links are matched to them by approach
  direction (within 45 deg), turn and mode; a link without a group copies its Dutch group's match.

```yaml
traffic:
  signals: real
  signal_data:
    log: data/vlog_2026-11-02.csv     # relative to the project folder
    groups: data/signal_groups.json
    replay: program                   # or exact
    date: "2026-11-02"
    hour: 8                           # 08:00-09:00; without date/hour: simulation.time
    live: null                        # e.g. tcp://127.0.0.1:9000 or a file a decoder appends to
```

- `replay: program` writes a programme per covered junction (`signals_real.add.xml` in the trial's `sumo/`): the
  typical cycle (the shift that best repeats all the junction's states; cycles start where a group green once a cycle
  turns green), one cycle folded over all cycles of the hour (each
  second the most common state of each group), the offset at the trial's start, `actuated` when the cycles vary by
  more than 10% (coefficient of variation) or a group's greens by more than 15%, each green then between the 10th and
  90th percentile of its recorded lengths.
- `replay: exact` sets every recorded state at its time through TraCI (simulation time 0 = the recorded start).
- `live` sets the rows of a feed as they come; when the feed stops, the junctions go back to their Dutch programmes.

## The Dutch programme (`dutch`)

- **Controllers.** The signalised nodes of SUMO's net are matched to the nearest listed junction within 45 m; all
  nodes of one junction (dual carriageways) are one controller named after its VRI number (`vri2030`); a listed
  junction without a signalised node gets one at road nodes within 30 m (OSM missed the signal); signalised nodes
  near no listed junction keep their OSM signal (`tls<node>`, joined within 30 m). The net is copied with the new
  lights (connections' `tl`/`linkIndex`, junction type, `tlLogic`); lanes and geometry are unchanged (netconvert is
  not run again: re-reading the net re-shaped 110 lanes and dropped 3). Cached in `maps/<map>/signals/`.
- **Signal groups.** Per approach and mode: a left turn from its own lane (protected), the other car lanes (through,
  right, lefts from a shared lane), cyclists; buses on bus lanes their own. An approach whose lanes' paths cross
  inside the junction (net artefacts) is split.
- **Stages.** Conflicts come from SUMO's request matrix and the paths' shapes. A turning stream may share a green
  with a stream it yields to (a 'g': right turns and straight-on cyclists, lefts from a shared lane and oncoming
  traffic, turning cyclists), as Dutch "deelconflicten"; a left from its own lane and straight streams never do.
  Stages are the fewest conflict-free sets (greedy colouring over 300 orders), each filled with every group that fits,
  opposite through streams together, ordered for the least lost time.
- **Amber** (CROW 2022): straight on 3.5 s at 50 km/h, 4.0 at 60, 4.5 at 70, 5.0 at 80; turning 3.0 s; cyclists
  (with mopeds) 3.0 s; buses 3.0 s (2.0 s only with selective detection).
- **All-red** (clearance): the last vehicle of the ending stream leaving at 10 m/s (turning 7, cyclists 5, buses 8)
  must clear the conflict point plus its length (6, 2, 12 m) before the first of the starting stream, starting at
  6 m/s (cyclists 4, buses 5), reaches it; the worst pair, rounded up to 0.5 s (as Eindhoven rounds), at most 6 s.
- **Cycle** (Nota Verkeerslichten 2024): base 50 s for two stages, 65 for three, 80 for four, 95 for more (+15 s on
  main roads: 50 km/h and two lanes or more); greens split by each stage's heaviest group (lanes, cars count double);
  minimum greens 6 s (cyclists and buses 5 s). **Vehicle-actuated** (SUMO `actuated`, detectors 3 s gap): each
  green between its minimum and a maximum that keeps the longest cycle within 90 s inside the Ring and 120 s on main
  roads. A junction with one stage (a mid-block crossing) gets a 12 s red for the road (9 m at 1.2 m/s, the Regeling's
  walking speed).
- **Pedestrians** are not in SUMO's net (no sidewalks); the CARLA crowd crosses at a junction when the streams over
  its crossing show red (`pedsignals.py`), and the accessible signals tick (`sound.aps`).

Not modelled: green waves (offsets) on the Ring, public-transport priority (KAR), "cyclists twice green", night
programmes and flashing amber, skipped stages without demand (SUMO's actuated control serves every stage).

## Checks

`tools/signal_check.py` runs the trial's SUMO traffic without CARLA for each mode and measures at the signals;
`tools/signal_heads.py` compares the heads CARLA drew (`simulation.traffic_lights.log: true`) with SUMO's own record
of its lights (`tls_states.xml`, written in every trial's `sumo/`). Results on `eindhoven_centre`: README,
[Traffic lights](../README.md#traffic-lights).
