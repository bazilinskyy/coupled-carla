# Session checklist (one participant)

Participant number: ______  Date: __________  Experimenter: ______  Study file: `experiments/__________.yaml`

Details and fixes: [lab_guide.md](lab_guide.md).

## Before the participant arrives (30 min)

- [ ] GPU window on: `ssh wsl5090 'tmux new -d -s gpuwin ~/cc58/gpu_window.sh'` (lead only)
- [ ] win5090's yield scripts leave Coupled CARLA alone (lab guide, "Before the first session")
- [ ] CARLA server up: `schtasks /run /tn cc58-pkgserver` (or at the desk); port 2000 answers
- [ ] Host: `cd ~/cc58/coupled-carla && source ~/cc58/env58.sh`
      `python -m coupledcarla.run experiments/<study>.yaml --participant <N> --dashboard`
      (validator prints no errors; the order printed matches the session sheet)
- [ ] Stations started, one per role, each on its own screen (`--display N --fullscreen`), headsets on their own
      outputs (`--audio-device`); every window says "waiting for the experiment to start"
- [ ] VR stations (`view: vr` or `--display vr`): headset charged / cable in, lenses clean, a hygiene cover on; the
      runtime is the active OpenXR runtime (Quest Link / Varjo Base / SteamVR); `python -m coupledcarla.vrxr` passes
      (runtime ~ the headset's rate) before the station starts
- [ ] Dashboard http://127.0.0.1:8780/: every station **connected**
- [ ] Practice trial (Start trial):
  - [ ] round trip < 30 ms, fps ~ 50, image lag 100-200 ms and steady, RTF >= 0.95
  - [ ] F3 at each station with sound: 1 kHz tone measured at the ear = `sound.reference_db`; volume fixed
  - [ ] wheel centred, steers both ways, pedals respond (driver)
  - [ ] response button and slider move on the dashboard
  - [ ] visual sync (F or the button): red bar on every screen (photodiode reads it, if used)
  - [ ] VR: the participant sets the headset's IPD (or note `vr.ipd_mm`), sits upright, looks straight ahead: Home
        recentres; the speed/prompt panel is readable; the VR station's stats show `vr_fps` ~ the host's tick rate
        and `vr_pose_to_image_ms` steady
- [ ] Phones silent, notifications off on the station PCs

## With the participant

- [ ] Consent signed; participant number (not the name) used everywhere
- [ ] Instructions read; practice trial run with the participant
- [ ] Each trial: Start trial, watch RTF / connected / round trip / image lag / collisions, note anything odd:

| Trial | Condition | Start time | Notes (sickness, stops, glitches) |
|---|---|---|---|
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |

- [ ] Simulator sickness: ask between trials (in VR after every trial, e.g. the MISC 0-10 scale; stop at 6 or on any
      request); stop at once if needed (End trial, then Stop session)
- [ ] Questionnaires between / after trials as the protocol says

## After

- [ ] Stop session on the dashboard; stations closed (Esc)
- [ ] Logs present: `logs/<study>/<time>_P<NN>_trial*/` (one per trial), `session_P<NN>_<time>.csv`, each station's
      `logs/stations/<role>_<time>.csv` (VR stations also `<role>_<time>_vr.csv`: head poses, motion-to-photon)
- [ ] `python tools/station_dryrun.py --report logs/<study>/<time>_P<NN>_trial<i>` for any trial that felt wrong
- [ ] Backup copied off win5090 and checked (trial folders, session CSV, station logs)
- [ ] GPU window ended: `rm ~/cc58/gpu_flag` on wsl5090 (crowd workers continue)
- [ ] Wheel, headsets and screens back in place for the next session
