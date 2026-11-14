# Sounds, designs and their sources

coupled-carla ships **no audio files**. Every sound a station plays is synthesised at start-up with numpy in
`coupledcarla/audio.py` (code under the repository's MIT licence), so there is nothing to credit and nothing to
download:

| Sound | How it is made | Used for |
|---|---|---|
| Tyre-road noise | pink noise (-3 dB/octave), looped; pitch and low-pass follow speed and distance | every vehicle; the own car inside |
| Engine | four-cylinder firing frequency (rpm / 30) with three harmonics and roughness, continuous phase | combustion vehicles |
| Wind | band noise 80-3000 Hz | the own car inside, above 10 km/h |
| AVAS | 220 Hz two-tone hum with an octave and 2 Hz modulation, pitch +1 % per km/h | electric / automated cars up to 20 km/h |
| City ambience | brown noise (distant traffic) with a low-passed pink bed and a slow swell | `sound.ambience.kind: city` |
| Rural ambience | band noise with gusts (wind in trees) | `sound.ambience.kind: rural` |
| Chime, beeps, ding | bell tones E5+C5, three 1.5 kHz beeps, an A5 bell (coupled-sim #95's cue sounds, re-synthesised) | auditory eHMIs, messages |
| Blinker tick | 2.4 kHz + 900 Hz decaying click | the own car's turn signal |
| Take-over alarm | alternating 2000/1600 Hz tone, 1.2 s | take-over requests |
| Calibration tone | 1 kHz sine at full scale | F3 at a station |

Spoken messages (`say:` in an auditory eHMI cue or a message) come from the station machine's own speech
synthesiser at run time (espeak-ng, espeak, pico2wave, macOS `say` or Windows SAPI) and are cached in
`~/.cache/coupled-carla/speech/`; nothing is fetched. Sound files an experiment names (`file:`,
`sound.ambience.file`) are the experimenter's own and stay on the station machine.

## coupled-sim's audio files: checked, not reused

| File in coupled-sim | Source and licence | Reused? |
|---|---|---|
| `Assets/Sounds/CanEngine_E_midlow-register.wav` | no source or licence recorded | no: unknown licence |
| `Assets/Vehicle_Essentials/.../Vehicle_Car_Drive_Interior_Loop_01.wav`, `..._Exterior_Loop.wav` | Vehicle - Essentials by Nox Sound, Unity Asset Store EULA | no: the EULA does not allow redistribution outside a built application |

coupled-sim's own soundscape (#135) and auditory eHMIs (#95) are procedural too; coupled-carla follows their
level model (tyres 68 dB at 50 km/h and 7.5 m, +30 log10 v; engines 64 dB; interior 6 dB quieter; 1 kHz
full-scale calibration tone) and cue sounds.

## Design files

`data/ehmi/light_bar_cyan.json`, `text_walk.json`, `directional_chevrons.json`, `road_projection.json` and
`data/incar/cluster.json`, `hud_speed.json`, `centre_screen.json` are coupled-sim's eHMI and in-car display
designs (github.com/bazilinskyy/coupled-sim, MIT licence, Pavlo Bazilinskyy and coupled-sim contributors), with a
placement, a speed-limit sign and messages added. `data/ehmi/stopping_bar.yaml` is new.
