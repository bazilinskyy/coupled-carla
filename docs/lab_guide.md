# Running a session with people

A practical guide to running Coupled CARLA with human participants on the win5090 rendering server: which
machine does what, what each station needs, the network, the order to start things in, the checks before a
session, what to watch during it, where the data ends up, and what to do when something goes wrong. The one-page
list to keep next to the keyboard is [session_checklist.md](session_checklist.md).

Written 2026-11-15 for the setup of that day: the packaged CARLA 1.0 Shipping server on win5090, the host and
SUMO in WSL on the same machine. Bot stations have run against it; **no session with people has run yet**, so
treat the numbers below as the targets the dry run checks, not as measured lab results.

## Machines

| What | Where | Started by |
|---|---|---|
| CARLA 1.0 server (Shipping package, renders every camera) | win5090, Windows: `C:\cc58\carla\Build\Package\Carla-1.0-Win64-Shipping` | scheduled task `cc58-pkgserver` (`C:\cc58\pkgserver.ps1`: `-RenderOffScreen -nosound -carla-rpc-port=2000 -carla-streaming-port=2001 -quality-level=Epic -carla-packs=C:\cc58\carla_packs`) |
| Host: the world, SUMO, AI agents, the log (`coupledcarla.run`) | win5090, WSL (`ssh wsl5090`, user `crowd`): `~/cc58/coupled-carla`, environment `source ~/cc58/env58.sh` | the experimenter |
| Experimenter dashboard (`--dashboard`) | the host process, http://127.0.0.1:8780/ | `run ... --dashboard` |
| Stations (`coupledcarla.station`), one per participant | win5090 itself (monitors on its RTX 5090) or lab PCs on the wired LAN | the experimenter, per station |

win5090 is `TUE055687.campus.tue.nl`, 100.80.224.203. WSL runs with mirrored networking, so the host in WSL and
the server on Windows reach each other at 127.0.0.1, and a lab PC reaches both at win5090's address.

### Where a station can run

A station renders nothing itself: the server renders its camera and sends it raw (BGRA, 4 bytes a pixel) over
TCP. That decides where stations can sit:

| Station on | Camera stream | Fits |
|---|---|---|
| win5090 itself (Windows Python or WSL) | loopback | any size and rate; the screens hang on win5090's GPU (3 DisplayPort + 1 HDMI) |
| a lab PC on gigabit Ethernet | over the LAN | 1280x720 is 3.7 MB an image: 184 MB/s at 50 Hz, but gigabit carries 117 MB/s (measured win5090 -> lin4080). One remote station at 1280x720 needs `--camera-hz 25` (92 MB/s); all remote stations share win5090's one link, so two need 12 Hz each, or 960x540 at 25 Hz |
| a PC on Wi-Fi | over the air | no: lin5080 (USB Wi-Fi, 131.155.229.241) drops packets above ~1,300-1,350 bytes without telling the sender (path-MTU black hole, checked again 2026-11-15 with `ping -M do -s 1300/1400` from win5090), so CARLA's image stream stalls while the control channel still works |

So for 1280x720 at the full rate, put the stations on win5090 (one monitor each). Remote stations at full rate
need multi-GPU secondaries on their own PC ([below](#rendering-on-more-gpus-multi-gpu-untested)), which is not
tested yet.

Three limits of several stations on one PC:

- **Keyboard and mouse go to the focused window only.** One PC can have one station driven by keyboard and mouse
  (the pedestrian: W/S/A/D and mouse; the AV passenger: keys 1-9). The driver on a wheel works in a window without
  focus (the wheel module sets `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS`), but the driver's keys (R, Z/X) and F
  (visual sync) work only with focus.
- **Responses read every joystick of the PC** (`responses.py` opens all of them). With two responding stations on
  one PC, the driver's wheel paddles also move the pedestrian's slider. Put responding stations on separate PCs,
  or give only one station a `response`.
- **Each station plays sound on the default output** unless `--audio-device NAME` (names from
  `python -m coupledcarla.station --list-devices`). Two stations on one PC need two outputs (two USB headsets).

Candidate remote station PCs today: **lin4080** (wired, full 1500-byte MTU and 117 MB/s to win5090; a CARLA 1.0
station environment is ready in `~/cc-wt-labprep`: `source env58.sh`). **mini** cannot run a station: there is no
CARLA Python client for macOS. **lin5080** is on Wi-Fi (above). wsl5090 is win5090 itself.

## What each station machine needs

| | Windows lab PC (or win5090's Windows side) | Linux lab PC |
|---|---|---|
| Python | 3.10 exactly (the wheel is cp310), e.g. `uv venv --python 3.10 .venv58` | 3.10 (`uv venv --python 3.10 .venv58`) |
| CARLA client | `C:\cc58\carla\Build\Package\Carla-1.0-Win64-Shipping\PythonAPI\carla\dist\carla-1.0-cp310-cp310-win_amd64.whl` (copy it from win5090) | `carla-1.0-cp310-cp310-linux_x86_64.whl` built on lin5080 from the same commit (copies: wsl5090 `~/cc58/`, lin4080 `~/cc-wt-labprep/`) |
| Packages | `pip install <wheel> numpy pygame pyyaml` (+ `shapely networkx` for `--input bot` drivers) | the same |
| Coupled CARLA | a checkout of the same commit as the host (the station imports its modules) | the same |
| `CARLA_ROOT` | only for `--input bot` drivers: a folder with `PythonAPI/carla/agents` from the ue58-dev tree | the same (lin4080: `~/cc-wt-labprep/carla_root`) |
| Speech (spoken eHMI cues) | built in (SAPI) | `espeak-ng` |
| Wheel | Logitech G HUB, the G25 at 900 degrees, force feedback at the top | the kernel driver; force feedback through SDL |
| Audio | headphones on a known output; Windows volume fixed for the whole study | the same |
| Network | wired; the server and host ports open on win5090 (below) | wired |

The client must match the server: a carla 0.9.16 client, or the UE5-weekly package's wheel on lin4080
(`~/carla58-pkg`), is not this server's build. Check with
`python -c "import carla; c = carla.Client('<win5090>', 2000); c.set_timeout(5); print(c.get_server_version())"`.

Check a station machine once with `python -m coupledcarla.station --list-devices`: it lists the audio outputs, the
joysticks (the G25, gamepads) and the screens (`--display N`, `--fullscreen`).

## Network

| Port | What | Process on win5090 | Needed by |
|---|---|---|---|
| 2000 TCP | CARLA RPC | the server (Windows) | the host, every station |
| 2001 TCP | CARLA streaming (camera images, world snapshots) | the server (Windows) | every station |
| 2002 TCP | CARLA secondary port | the server (Windows) | multi-GPU secondaries only |
| 2100 TCP | Coupled CARLA control channel | the host (WSL) | every station |
| 8780 TCP | experimenter dashboard | the host (WSL) | a browser on another PC (`--dashboard-host`) |
| 3000-3002, 4000-4002, ... | multi-GPU secondaries | secondaries | stations whose camera such a secondary renders |

Stations on win5090 itself need nothing opened. **Stations on other PCs need firewall rules on win5090 that do not
exist yet** (checked 2026-11-15: Windows Firewall is on, inbound blocked by default, the network profile is
Domain; there is a rule for an older CARLA 0.10 in the user's folder, none for `C:\cc58`, and the only rule for
WSL is port 2222 for ssh). Pavlo or TU/e IT has to add them, in an administrator PowerShell on win5090:

```powershell
# the CARLA 1.0 server (a Windows process)
New-NetFirewallRule -DisplayName "coupled-carla CARLA 1.0 server" -Direction Inbound -Action Allow -Profile Domain `
  -Program "C:\cc58\carla\Build\Package\Carla-1.0-Win64-Shipping\CarlaUnreal\Binaries\Win64\CarlaUnreal-Win64-Shipping.exe" `
  -Protocol TCP -LocalPort 2000-2002
# the host's control channel and dashboard (WSL, mirrored networking: a Windows rule and a Hyper-V rule, as for ssh 2222)
New-NetFirewallRule -DisplayName "coupled-carla control and dashboard" -Direction Inbound -Action Allow -Profile Domain `
  -Protocol TCP -LocalPort 2100,8780
New-NetFirewallHyperVRule -Name coupled-carla-wsl -DisplayName "coupled-carla control and dashboard (WSL)" `
  -Direction Inbound -Action Allow -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 2100,8780
```

Add `-RemoteAddress <lab PC addresses>` to each to allow only the lab PCs. With multi-GPU secondaries on win5090
and stations elsewhere, add the secondaries' ports (3000-3002, ...) to the first rule.

From a lab PC, check before the session (PowerShell `Test-NetConnection TUE055687.campus.tue.nl -Port 2000`, Linux
`nc -zv 100.80.224.203 2000`) for 2000, 2001 and 2100, and the path MTU: `ping -f -l 1472 100.80.224.203` (Windows)
or `ping -M do -s 1472 100.80.224.203` (Linux) must get answers. If 1472 fails but 1200 works, the station will
get no images (the Wi-Fi problem above).

## Before the first session (once)

1. **Stop win5090's yield scripts from pausing Coupled CARLA.** win5090 pauses background work while someone uses
   its desktop (keyboard or mouse input in 12 of the last 30 20-s slots), or while a CARLA server started at the
   desk runs: `wsl_yield.py` (wsl5090 `~/compute-dash/`, cron) sends SIGSTOP to every `python`, `sumo` and
   `netconvert` of the `crowd` user in WSL, which includes the host, SUMO and WSL stations; `win_yield.pyw`
   (scheduled task "dashboard win yield") suspends every background (session 0) process under `C:\cc58`, which
   includes the server started by the task `cc58-pkgserver`. With participants or the experimenter at win5090's
   keyboard the session would freeze within a minute. Decide one of (this changes the shared dashboard-workstations
   setup, so it is Pavlo's call):
   - in `wsl_yield.py` set `KEEP_GOING = [os.path.expanduser('~/cc58/coupled-carla')]` (jobs whose working folder
     is the Coupled CARLA checkout keep going), and start the server at the desk instead of through the task
     (programs started at the desk are never suspended; a CARLA server running at the desk also counts as "a
     simulation", which frees the GPU from the crowd workers), or
   - drop `C:\cc58` from `PROJECTS` in `win_yield.pyw` for the study period, plus the `KEEP_GOING` change.
2. **Firewall rules** (above), if any station sits on another PC.
3. **Station machines** set up (above); `python -m coupledcarla.station --list-devices` shows the wheel, the
   headsets and the screens.
4. **Experiment file checked**: `python -m coupledcarla.validate experiments/<study>.yaml --participants 1-24`
   (exit 0; read the warnings and every participant's plan).
5. **Dry run** passes (below).

## Start order

1. **GPU window** (only the lead starts it): `ssh wsl5090 'tmux new -d -s gpuwin ~/cc58/gpu_window.sh'`. It pauses
   the crowd workers, ends the jaywalking run and frees the RTX 5090 for 2 h; `rm ~/cc58/gpu_flag` ends it early and
   everything is restored.
2. **CARLA server**: `schtasks /run /tn cc58-pkgserver` on win5090 (or at the desk, see above). It takes ~30 s
   (9.7 GB RAM, 2.6 GB GPU memory). Check: `Test-NetConnection 127.0.0.1 -Port 2000`.
3. **Host** in WSL, with the dashboard:
   ```bash
   cd ~/cc58/coupled-carla && source ~/cc58/env58.sh
   python -m coupledcarla.run experiments/<study>.yaml --participant 7 --dashboard
   ```
   It prints the validator's findings and the participant's order, writes `logs/<study>/session_P07_<time>.csv`,
   builds the first trial and then waits for the stations (`coupling.wait_for_stations_s`, 600 s).
4. **Stations**, one per participant, in any order (they retry the host for 5 minutes and stay connected across
   trials):
   ```bash
   # on win5090 (Windows, in the coupled-carla checkout with the 3.10 venv)
   python -m coupledcarla.station --host 127.0.0.1 --role driver --input wheel --display 1 --fullscreen --audio-device "<headset 1>"
   # on a lab PC: the server and the host are both win5090
   python -m coupledcarla.station --host 100.80.224.203 --role pedestrian --display 0 --fullscreen --camera-hz 25
   ```
   Each window says "waiting for the experiment to start" until its trial runs.
5. **Dashboard**: open http://127.0.0.1:8780/ on win5090's desktop (WSL's localhost is reachable from Windows), or
   from another PC with `--dashboard-host 0.0.0.0` and the link with the token the host prints. Each trial waits
   for **Start trial** once it is loaded (`dashboard.gate_trials`).

The host restarts nothing on the packaged server between trials (it attaches to it; `server.launch` only starts a
server when none answers on port 2000). Every trial runs the stations in fresh child processes; they rejoin by
themselves.

## The dry run

`tools/station_dryrun.py` runs one short trial with bot stations and says per station whether it is ready:

```bash
# on wsl5090, during a GPU window, with the server up: three local bot stations
python tools/station_dryrun.py experiments/eindhoven_crossing_study.yaml --duration 40
# plus one station on lin4080, started over ssh (its station environment in ~/cc-wt-labprep)
python tools/station_dryrun.py experiments/eindhoven_crossing_study.yaml --duration 40 \
    --ssh pedestrian=lin4080:cc-wt-labprep --ssh-env env58.sh --camera-hz 25
# or print the command for a station on another PC and wait for it
python tools/station_dryrun.py experiments/eindhoven_crossing_study.yaml --remote pedestrian=labpc1
```

It prints, per station: the time to join the host, the time from the trial start to the first camera image, the
round trip to the host, the loop and camera frame rates against the host's tick rate, the tick-to-image latency
(median and 95th percentile), the world ticks it missed, the share of camera images it got, and whether sound
(and an audio device), in-car displays and station-drawn eHMIs are on; then the trial's real-time factor. The
exit code is 0 when all is within the thresholds, 1 when one is missed, 2 when a station never ran the trial.

| Threshold | Default | Option |
|---|---|---|
| join the host | 20 s | `--max-join-s` |
| trial start to first image | 15 s | `--max-ready-s` |
| round trip to the host, median | 30 ms | `--max-rtt-ms` |
| station loop rate / host tick rate | 0.9 | `--min-fps-share` |
| camera images received / expected | 0.9 | `--min-image-share` |
| tick to image, 95th percentile | 250 ms | `--max-image-p95-ms` |
| world ticks missed | 5% | `--max-dropped-share` |
| real-time factor | 0.95 | `--min-rtf` |

The bots render at the full rate (`--camera-hz 0`, as a human's window does), unlike `--headless` stations in
other tests (1 image a second). `--report logs/<study>/<time>_P07_trial2` checks a real session's trial the same
way afterwards; the stations report these numbers every 5 s (`station_stats` events) whether they are bots or
not.

## Before each session

- [ ] Start order above; the dashboard shows every station connected.
- [ ] **Audio calibration** (experiments with `sound`): set the PC's volume once and never touch it again (note the
  Windows volume %). During a practice trial press **F3** at the station: it plays a 1 kHz tone at full scale
  (F3 again stops it). Measure it at the participant's ear (a sound level meter at the ear cup or an artificial
  ear) and set `sound.reference_db` (or the participant's `sound.reference_db`) to the reading; the soundscape is
  then in dB SPL (tyres 68 dB at 50 km/h and 7.5 m). F3 is logged in the station's `_cues.csv`.
- [ ] **Wheel calibration** (driver): G HUB at 900 degrees, force feedback at the top. With the station closed,
  `python -m coupledcarla.inputs.steering_wheel` prints the inputs: steering -1 to +1 lock to lock and 0.00
  centred, throttle and brake 0 to 1, clutch, gear. Centre the wheel before each trial (the force feedback's
  spring does it at a standstill).
- [ ] **Latency check**: the dry run, or the practice trial with the dashboard open: round trip below 30 ms, frame
  rate at the host's tick rate (50), image lag about 100-200 ms (CARLA renders and reads back a camera 4-6 frames
  after its tick). The display adds its own lag on top: a photodiode on the screen's top edge sees the visual-sync
  bar (F or `coupling.visual_sync_at_s`) whose frame is logged (`visual_sync` event).
- [ ] Screens: each station full screen on its own monitor (`--display N --fullscreen`); nothing else on those
  monitors; Windows notifications off (Focus).
- [ ] Response devices: press the button and move the slider once in the practice trial; the dashboard shows the
  response per participant.
- [ ] Disk: the host writes 10-20 MB per 2-minute trial (frames.csv at 50 Hz, SUMO's FCD; 21 MB for 120 s on the
  twin); check `df -h ~` on wsl5090 once a day.

## During the session

Watch the dashboard:

| Field | Healthy | If not |
|---|---|---|
| state, t / duration | running, counting | paused: someone pressed Pause; loading for minutes: the host waits for a station |
| RTF | 0.95-1.0 | below 0.9: the server or host is behind (GPU shared? the yield scripts? scenery radius?) |
| per participant: connected | yes | a station left: it rejoins by itself at the next trial; the current trial goes on without it |
| round trip | < 30 ms | high or rising: network trouble (Wi-Fi, a busy link) |
| fps | ~ the tick rate (50) | low: that station's PC is too slow, or its window is minimised/hidden |
| image lag | 100-200 ms, steady | rising: images queue on the link (remote station at too high a rate: `--camera-hz`) |
| collisions | as the scenario expects | unexpected ones: check the event list |
| events | trial_start, triggers, visual_sync, ehmi, take-overs | `stations_missing` at the start: a station did not join in time |

Buttons: **Start trial** (each loaded trial waits for it), **Pause / Resume** (the world, SUMO and the stations stop;
paused time does not count), **End trial**, **Stop session** (no more trials), **Visual sync**, **Take-over
request** per automated driver. Esc at a station leaves the session for that station.

## After the session

| What | Where |
|---|---|
| Each trial: config.yaml (with the `study` block), events.csv, frames.csv (50 Hz), summary.json, weather.json, responses.csv, takeover.csv, sumo/ (SUMO's FCD), analysis/ (encounters, timeline) | wsl5090 `~/cc58/coupled-carla/logs/<study>/<time>_P07_trial<i>/` |
| The participant's order | `~/cc58/coupled-carla/logs/<study>/session_P07_<time>.csv` |
| Each station's per-frame log, cues, sound recordings | `<station's coupled-carla folder>/logs/stations/<role>_<time>.csv`, `_cues.csv`, `.wav` (with `sound.record`) |
| CARLA's own recording (replay) | win5090 `C:\Users\20255250\AppData\Local\CarlaUnreal\Saved\coupledcarla_<study>_<time>_trial<i>.log` |
| Study summaries | `python -m coupledcarla.analysis logs/<study>` writes `study_responses.csv`, `study_takeovers.csv` |

Backup the same day: copy the participant's trial folders, the session CSV and the stations' logs off win5090
(e.g. `rsync -a wsl5090:cc58/coupled-carla/logs/<study>/ <backup>/<study>/` and each station's `logs/stations/`),
then check the copy. The logs hold no names; keep the participant number to name key on paper or in the study's
own protected storage, never in the logs or the repository. `python tools/station_dryrun.py --report <trial>`
checks a trial's station numbers afterwards; `python -m coupledcarla.export_trial <trial>` makes the viewer file.

End the GPU window (`rm ~/cc58/gpu_flag` on wsl5090) so the crowd workers continue.

## Troubleshooting

**A station cannot connect**
- "waiting for the host" for long: the control channel (2100) is not reachable. On win5090 check the host runs;
  from a lab PC check the port (`Test-NetConnection ... -Port 2100`) and the firewall rules (the Hyper-V rule for
  WSL as well as the Windows one).
- It joins (the dashboard shows it) but shows no image: port 2001 blocked, a different CARLA build (the client's
  wheel must be this server's), or the path-MTU problem (Wi-Fi): check `ping` with 1472 bytes and don't fragment.
- `version mismatch` warnings or a crash at connect: the carla wheel is not the server's build.
- "no participant with role X": the station's `--role` is not a `station: true` participant of the experiment.

**Low frame rate or high image lag**
- The RTF on the dashboard is low too: the server or host is behind. Is the GPU window on (crowd workers paused),
  did a yield script pause the host (`cat ~/.wsl_yield.json` on wsl5090: `"paused": true`), is someone's job on the
  GPU (`nvidia-smi`)? `simulation.profile: true` prints where each tick's milliseconds go.
- Only one station is slow: its PC (a laptop on battery, an integrated GPU drawing a full-screen pygame window) or
  its link. A remote station at 1280x720 and 50 Hz needs more than gigabit: `--camera-hz 25` or `--size 960x540`.
- Fewer images than ticks, rising image lag: the link is full (see the bandwidth table above).

**Stalls**
- Everything freezes for seconds while someone uses win5090's keyboard or mouse: the yield scripts (see
  [Before the first session](#before-the-first-session-once)).
- Ticks stall ~40 ms each: the CARLA 1.0 rpclib delayed ACK; the code works around it (one `apply_batch_sync` per
  tick, drawings on a second connection), so a stall means an extra fire-and-forget call slipped in (README, Real
  time on CARLA 1.0).
- A station's process ends with a CARLA abort between trials: expected when the server restarts; the supervisor
  starts a fresh child and it rejoins.
- The host waits for stations forever: a station's role is missing or it runs against another host's port;
  `coupling.wait_for_stations_s` (600 s) ends the wait and logs `stations_missing`.
- The server stops with "GPU crashed or D3D device removed": the GPU ran out of memory (another job on it, as on
  2026-11-14 with the editor server). Restart the server task and the session from the next trial
  (`--participant N --trial K`).

## Rendering on more GPUs (multi-GPU, untested)

CARLA 1.0 can split a simulation over a primary server (physics, actors, clients; `-nullrhi`, no GPU) and
secondary servers that render the sensors on their own GPUs. Coupled CARLA supports it through
`server.multigpu` (`coupledcarla/multigpu.py`); the configuration, the command lines, the relay and the camera
turns are tested without CARLA, **nothing has run on real GPUs yet**.

```yaml
server:
  multigpu:
    primary_host: 100.80.224.203      # the primary's LAN address, for secondaries on other PCs
    secondaries:
      - {name: gpu0, host: 127.0.0.1, rpc_port: 3000, gpu: 0, launch: false}           # on win5090
      - {name: ped, host: <lab PC>, rpc_port: 4000, gpu: 0, roles: [pedestrian]}      # on the pedestrian's PC
```

`python -m coupledcarla.multigpu experiments/<study>.yaml --windows` prints the plan: the primary's extra
arguments (`-nullrhi`), each secondary's ports and command line, and the checks. On a station PC,
`python -m coupledcarla.multigpu experiments/<study>.yaml --secondary ped --root <CARLA 1.0 package> --primary
100.80.224.203` runs that secondary and restarts it whenever the primary restarts. Every server needs the same
CARLA build and the same content packs (`-carla-packs`, and the twin's `OpenDrive/` and `TM/` folders).

What CARLA decides and how Coupled CARLA deals with it:

- The primary gives each new camera to the next connected secondary, round-robin; a client cannot choose. A
  station whose role a secondary lists asks the host for a camera turn, listens, sees through its relay which
  server took the stream, and respawns the camera until it lands on its own secondary (at most once round).
- A secondary's stream token has a port but no address; the client fills in the address it used for the primary.
  So a station whose plan has secondaries on other machines connects through a relay on 127.0.0.1: its own
  secondary is then reached directly, the primary and the others through the relay. Every server needs its own
  streaming port (secondary *i* defaults to 3000 + 1000 *i*); the validator checks it.
- The first secondary to connect switches the primary to synchronous mode at 1/20 s; the host puts its own step
  back after the stations joined and checks every 5 s.
- Debug drawings are not mirrored: the eHMI light bars and text drawn in the world do not show in cameras a
  secondary renders. Use a designed eHMI with `ehmi.render: station`.

The test to run when a GPU is free: on win5090 stop `cc58-pkgserver`, start the package as a primary with
`-nullrhi -carla-rpc-port=2000` and a secondary with `-carla-rpc-port=3000 -carla-primary-host=127.0.0.1
-carla-primary-port=2002 -graphicsadapter=0 -RenderOffScreen` (both with the `-carla-packs` of pkgserver.ps1),
then the dry run with `server.multigpu.secondaries: [{name: gpu0, launch: false}]` and three local bot stations.
Compare RTF, frame rates and image latency with the single server. Then a secondary on a station PC with a GPU
of its own (lin4080's driver 560 cannot run CARLA 1.0; a Windows lab PC with a recent NVIDIA driver can).
