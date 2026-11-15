# Sounds, models, recordings and their sources

coupled-carla ships **no audio files**. Every sound is synthesised in code (`coupledcarla/synth.py`, `audio.py`,
MIT licence) from published acoustic models, except a few ambience recordings and an HRTF set that a station or the
host fetches at run time from their open sources and caches in `~/.cache/coupled-carla/sounds/` (never committed).
Offline, everything falls back to synthesis.

## Models (all synthesised)

| Sound | Model | Source |
|---|---|---|
| Vehicle sound power (cars, vans, buses/trucks, mopeds, motorbikes) | rolling A_R + B_R lg(v/70) and propulsion A_P + B_P (v-70)/70 per octave band 63 Hz-8 kHz; road surface, temperature and junction corrections | CNOSSOS-EU, Commission Directive (EU) 2015/996 Annex II as amended by Commission Delegated Directive (EU) 2021/1226 (EU legislation, reusable under Commission Decision 2011/833/EU) |
| Acceleration and wet roads | Delta L_P = C_P a (a clipped to -1..a_max); Delta L_wet = max(15 lg f - 12 lg(v/70) - 48, 0) | IMAGINE Deliverable D11 (M+P, 2007), sections 4.3.1 and 4.3.5 |
| Tyre-road noise | octave bands of looped pink noise at the CNOSSOS band levels | Pieren, Bütler & Heutschi (2016), Applied Sciences 6(1):5 (tyre noise as broadband noise) |
| Engine | firing frequency (cylinders/2 x rpm/60) and its orders and half orders, load-dependent tilt, exhaust and intake formants, silencer roll-off, cycle-to-cycle speed and amplitude jitter, firing-pulsed residual noise; gearbox model for rpm | Pieren et al. (2016); Baldan, Lachambre, Delle Monache & Boussard (2015), IEEE SIVE; Jagla, Maillard & Martin (2012), JASA 132(5) |
| Air absorption | per band from temperature, humidity, pressure | ISO 9613-1:1993 |
| Ground, buildings | ground factor G; barrier D_z = 10 lg(3 + 20 z C3/lambda) over the roofs and round the sides | ISO 9613-2:1996 (Maekawa-type barrier law) |
| Reflections, reverberation | first-order image sources off the nearest facades; diffuse tail sized by street width and height (Sabine on a canyon segment, open top and ends) | Kang (2000), JASA 107(3) for street-canyon reverberation |
| Inside the car | own tyres and engine through a cabin insertion loss rising with frequency; wind noise 60 lg v; rain on the roof | interior levels of mid-size cars (~40 dB(A) idle, ~60 at 50 km/h, ~67 at 100 km/h); dipole sound power ~v^6 |
| Drones | blade-passing tones and harmonics (two detuned rotor pairs) over broadband noise, at the EU 2019/945 class sound power | Cabell et al. (2016), NASA; Schäffer et al. (2021), IJERPH 18:5940 |
| AVAS | tonal hum rising with speed, 56 dB(A) at 2 m up to 20 km/h | UN Regulation No. 138, EU 540/2014 |
| Footsteps | heel and toe impacts filtered by the surface, cadence from walking speed | Cook (2002), Modeling Bill's gait, AES 22nd Int. Conf. |
| Pedestrian signal ticking | a ~1.6 kHz 'tok', 1/s while red, ~8/s while green (Dutch rateltikker, German Akustischer Signalgeber) | synthesised |
| Bicycle freewheel, bell | pawl clicks at 24 per wheel turn while coasting; inharmonic struck-dome partials (fallback for the recorded bell) | synthesised |
| Sirens, horn, bus air | two-tone 440/587 Hz (Europe) or wail (North America); two-tone 420/500 Hz horn; high-passed air burst | synthesised |
| Rain, birds (fallback) | Poisson drops (clicks with bubble resonances), shaped hiss; FM chirps | synthesised |
| Distant traffic | CNOSSOS car spectrum at 250 m behind buildings, level following an urban hourly flow curve, plus the simulated vehicles beyond the heard ones | synthesised |
| Chime, beeps, ding, blinker tick, take-over alarm, calibration tone | coupled-sim #95's cue sounds re-synthesised; 1 kHz full-scale sine | synthesised |

## Recordings fetched at run time (Wikimedia Commons, no key)

The licence of each file is read from the Commons API at fetch time; only CC0, public domain, CC BY and CC BY-SA
are used. Every recording and every trial audio that used one gets the attribution in `audio_credits.txt`.

| Role | File (first choice, then fallback) | Author | Licence |
|---|---|---|---|
| Dawn chorus | [Réveil des oiseaux.ogg](https://commons.wikimedia.org/wiki/File:R%C3%A9veil_des_oiseaux.ogg) | Joseph Sardin | CC0 |
| | [Reggelirigók.ogg](https://commons.wikimedia.org/wiki/File:Reggelirig%C3%B3k.ogg) | Ödönke | Public domain |
| Daytime birdsong | [Male Blackbird song 01.ogg](https://commons.wikimedia.org/wiki/File:Male_Blackbird_song_01.ogg) | Tony Wills | CC BY-SA 3.0 |
| | [Common Blackbird song (Turdus merula).ogg](https://commons.wikimedia.org/wiki/File:Common_Blackbird_song_(Turdus_merula).ogg) | Diana Tudor | CC BY 4.0 |
| Light rain | [Sound of light rainfall.ogg](https://commons.wikimedia.org/wiki/File:Sound_of_light_rainfall.ogg) | Mijesty | CC BY-SA 4.0 |
| Heavy rain | [Rain (1).ogg](https://commons.wikimedia.org/wiki/File:Rain_(1).ogg) | ezwa | Public domain |
| | [Heavy Rain in Summer.ogg](https://commons.wikimedia.org/wiki/File:Heavy_Rain_in_Summer.ogg) | Fiver, der Hellseher | CC BY-SA 3.0 |
| Bicycle bell | [Bicycle-bell-1.wav](https://commons.wikimedia.org/wiki/File:Bicycle-bell-1.wav), [Bicycle-bell-3.wav](https://commons.wikimedia.org/wiki/File:Bicycle-bell-3.wav) | Soundscape_Leuphana | CC0 |

Bird recordings are high-passed at 1.2 kHz and rain at 200 Hz (what lies below is the recordist's background), then
played as loops at levels set by the models above. Candidates rejected after a spectrogram check: band-limited
files (e.g. 'Heavy rain in Glenshaw, PA', cut at 4.5 kHz), files with thunder, voices or traffic. Freesound has
more, but its API needs a key, so it is not used.

## HRTFs (binaural output, `sound.spatial: binaural`)

[MIT KEMAR compact set](https://sound.media.mit.edu/resources/KEMAR.html), Bill Gardner and Keith Martin, MIT Media
Lab (1994): "provided free with no restrictions on use, provided the authors are cited". Fetched once (226 kB) and
diffuse-field equalised (so binaural and stereo output have the same level within ~1.5 dB). Without it, the
spherical-head model of Brown & Duda (1998), IEEE Trans. Speech Audio Process. 6(5).

## Speech, experimenters' files

Spoken messages (`say:` in an auditory eHMI cue or a message) come from the station machine's own speech
synthesiser at run time (espeak-ng, espeak, pico2wave, macOS `say` or Windows SAPI) and are cached in
`~/.cache/coupled-carla/speech/`. Sound files an experiment names (`file:`, `sound.ambience.file`) are the
experimenter's own and stay on their machine.

## coupled-sim's audio files: checked, not reused

| File in coupled-sim | Source and licence | Reused? |
|---|---|---|
| `Assets/Sounds/CanEngine_E_midlow-register.wav` | no source or licence recorded | no: unknown licence |
| `Assets/Vehicle_Essentials/.../Vehicle_Car_Drive_Interior_Loop_01.wav`, `..._Exterior_Loop.wav` | Vehicle - Essentials by Nox Sound, Unity Asset Store EULA | no: the EULA does not allow redistribution outside a built application |

coupled-sim's own soundscape (#135) and auditory eHMIs (#95) are procedural too; coupled-carla keeps their cue sounds
and the 1 kHz full-scale calibration, and replaced their level model (tyres 68 dB at 50 km/h and 7.5 m, +30 lg v)
with CNOSSOS-EU (a car at 50 km/h and 7.5 m: 72.9 dB(A)).

## Design files

`data/ehmi/light_bar_cyan.json`, `text_walk.json`, `directional_chevrons.json`, `road_projection.json` and
`data/incar/cluster.json`, `hud_speed.json`, `centre_screen.json` are coupled-sim's eHMI and in-car display
designs (github.com/bazilinskyy/coupled-sim, MIT licence, Pavlo Bazilinskyy and coupled-sim contributors), with a
placement, a speed-limit sign and messages added. `data/ehmi/stopping_bar.yaml` is new.
