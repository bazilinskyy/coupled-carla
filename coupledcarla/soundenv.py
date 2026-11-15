"""The ambience of a soundscape from the real conditions of the trial: time of day and season (birds at dawn and in
spring, a quieter city at night), the weather (rain on the ground or on the car roof, wind in the trees) and the
traffic beyond the vehicles heard one by one (the far vehicles of the simulation summed into one distant hum, plus
the city beyond the map at a level that follows the hour).

The weather comes from the trial (weather.json: Open-Meteo's temperature, humidity, rain, wind with `weather: real`);
recordings of birds and rain from Wikimedia Commons (soundfetch.py) when they can be had, else synthesis.
"""
import math
import wave
from datetime import datetime

import numpy as np

from . import acoustics as ac
from . import audio, synth

# relative hourly traffic flow in a European city centre (share of the peak hour), 0..23 h: the familiar diurnal
# curve of urban traffic counts (night trough ~0.05-0.1 of the peak, morning and evening peaks)
HOURLY_FLOW = [0.12, 0.07, 0.05, 0.04, 0.05, 0.12, 0.35, 0.75, 0.95, 0.8, 0.75, 0.78, 0.8, 0.8, 0.82, 0.88, 0.97,
               1.0, 0.9, 0.7, 0.55, 0.45, 0.35, 0.22]


class Env:
    """What the soundscape knows of the trial's conditions (all optional): temperature (C), humidity (%), rain
    (mm/h), wind (m/s at 10 m), wet (0..1), local time at t = 0, latitude, night, sun altitude, country."""

    def __init__(self, d=None):
        d = dict(d or {})
        self.temp = float(d.get("temperature", 15.0) if d.get("temperature") is not None else 15.0)
        self.rh = float(d.get("humidity", 70.0) if d.get("humidity") is not None else 70.0)
        self.rain = float(d.get("rain") or 0.0)
        self.wind = float(d.get("wind") or 0.0)
        wet = d.get("wet")
        self.wet = float(wet) if wet is not None else (1.0 if self.rain >= 0.1 else 0.0)
        self.lat = d.get("lat")
        self.country = (d.get("country") or "").upper()
        self.sun = d.get("sun_altitude")
        self.night = bool(d.get("night")) if d.get("night") is not None else (self.sun is not None and self.sun < -3)
        self.local = None
        lt = d.get("local_time")
        if lt:
            for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M", "%Y-%m-%d %H:%M:%S"):
                try:
                    self.local = datetime.strptime(str(lt), fmt)
                    break
                except ValueError:
                    pass

    @classmethod
    def from_weather(cls, wx, meta=None, carla_weather=None, override=None):
        """From a trial's weather.json record (run.set_weather: Open-Meteo's hour with `weather: real`), else from
        CARLA's weather parameters (a preset: precipitation 20 per mm/h, wind intensity 2 per km/h, as
        realworld.carla_weather_fields maps them the other way), the map's meta.json, and `override` (the
        experiment's sound.weather: rain, wind, temperature, humidity, wet)."""
        wx = wx or {}
        om = wx.get("open_meteo") or {}
        bbox = (meta or {}).get("bbox")
        d = {"temperature": om.get("temperature_2m"), "humidity": om.get("relative_humidity_2m"),
             "rain": om.get("rain") if om.get("rain") is not None else om.get("precipitation"),
             "wind": (om.get("wind_speed_10m") or 0.0) / 3.6, "local_time": wx.get("local_time"),
             "sun_altitude": wx.get("sun_altitude"), "night": wx.get("night"),
             "country": ((meta or {}).get("speed_defaults") or {}).get("country")}
        p3 = om.get("precipitation_3h")
        if p3 is not None:
            d["wet"] = max(0.0, min(1.0, float(p3) / 1.0))
        if not om and carla_weather is not None:
            cw = carla_weather
            d["rain"] = float(getattr(cw, "precipitation", 0.0) or 0.0) / 20.0
            d["wind"] = float(getattr(cw, "wind_intensity", 0.0) or 0.0) / 2.0 / 3.6
            d["wet"] = max(float(getattr(cw, "wetness", 0.0) or 0.0),
                           float(getattr(cw, "precipitation_deposits", 0.0) or 0.0)) / 100.0
            if d.get("sun_altitude") is None and getattr(cw, "sun_altitude_angle", None) is not None:
                d["sun_altitude"] = float(cw.sun_altitude_angle)
                d["night"] = d["sun_altitude"] < 3.0
        for k, v in (override or {}).items():
            if k in ("rain", "wind", "temperature", "humidity", "wet", "local_time", "night", "sun_altitude"):
                d[k] = v
        if bbox:
            d["lat"] = (bbox[0] + bbox[2]) / 2
        return cls(d)

    def as_dict(self):
        return {"temperature": self.temp, "humidity": self.rh, "rain": self.rain, "wind": self.wind, "wet": self.wet,
                "lat": self.lat, "country": self.country, "sun_altitude": self.sun, "night": self.night,
                "local_time": self.local.strftime("%Y-%m-%d %H:%M") if self.local else None}

    def hour(self, t=0.0):
        if self.local is None:
            return 21.0 if self.night else 12.0
        h = self.local.hour + self.local.minute / 60.0 + t / 3600.0
        return h % 24.0

    def flow(self, t=0.0):
        h = self.hour(t)
        i = int(h) % 24
        f = h - int(h)
        return HOURLY_FLOW[i] * (1 - f) + HOURLY_FLOW[(i + 1) % 24] * f

    def season(self):
        """Bird song by month (northern breeding season March-July; mirrored south of the equator; weaker
        seasonality within 23 deg of the equator)."""
        if self.local is None:
            return 0.5
        m = self.local.month
        if self.lat is not None and self.lat < 0:
            m = (m + 5) % 12 + 1
        s = {1: 0.15, 2: 0.3, 3: 0.7, 4: 1.0, 5: 1.0, 6: 0.9, 7: 0.6, 8: 0.35, 9: 0.3, 10: 0.2, 11: 0.15, 12: 0.12}[m]
        if self.lat is not None and abs(self.lat) < 23:
            s = 0.5 + 0.5 * s
        return s

    def dawn(self, t=0.0):
        """0..1: the dawn chorus, from an hour before sunrise to two hours after (by the sun's altitude when known,
        else by the clock)."""
        if self.sun is not None and self.local is not None:
            h = self.hour(t)
            if h > 12:
                return 0.0
            return max(0.0, 1.0 - abs(self.sun - 2.0) / 10.0)
        h = self.hour(t)
        return max(0.0, 1.0 - abs(h - 6.0) / 2.0)


def load_recording(path, seconds=60.0, highpass=None, rate=audio.RATE):
    """A decoded recording (44.1 kHz WAV) as a mono loop of at most `seconds`, high-passed (birds: their song is
    above 1 kHz; what lies below is the recordist's background) and normalised to full-scale RMS."""
    with wave.open(str(path)) as w:
        ch, width, fr = w.getnchannels(), w.getsampwidth(), w.getframerate()
        n = min(w.getnframes(), int(seconds * fr) + int(2 * fr))
        raw = w.readframes(n)
    x = np.frombuffer(raw, "<i2").astype(np.float64).reshape(-1, ch).mean(axis=1) / 32768.0
    if fr != rate:
        x = np.interp(np.arange(0, len(x) - 1, fr / rate), np.arange(len(x)), x)
    x = x[int(0.5 * rate):]          # skip the start (handling noise)
    if highpass:
        spec = np.fft.rfft(x)
        f = np.fft.rfftfreq(len(x), 1.0 / rate)
        spec *= 1.0 / (1.0 + (highpass / np.maximum(f, 1.0)) ** 4)
        x = np.fft.irfft(spec, len(x))
    return audio._norm(audio._seamless(x, rate, 0.5))


class Ambience:
    """The ambience voices of a soundscape. update() each tick with where the listener is (inside a car or not),
    the share of trees around and the power of the far traffic per band."""

    def __init__(self, scape, cfg, env, log=print):
        self.sc, self.cfg, self.env, self.log = scape, cfg, env, log
        self.credits = {}
        self.used = set()          # recording roles actually heard (for the credits)
        self.voices = {}
        m = scape.mixer
        rec = cfg.get("recordings", True)
        # distant city and far simulated traffic: two decorrelated noise voices left and right
        if cfg.get("distant_traffic", "auto"):
            for k, az in (("hum_l", -1.0), ("hum_r", 1.0)):
                v = synth.BandNoise("pink", seed=101 + len(self.voices), flutter_db=1.5, flutter_hz=0.15,
                                    kinds=("direct",))
                v.paths[0].az = az
                self.voices[k] = m.add(("amb", k), v)
            r, p = ac.vehicle_power("1", 45.0)
            lw = ac.db_sum(np.vstack([r, p]), axis=0)
            far = lw - (20 * math.log10(250.0) + 11) - ac.air_absorption(env.temp, env.rh) * 250.0 \
                - ac.barrier_attenuation(8.0, double=True, e=15.0)
            self.hum_shape = far - ac.a_weighted(far)          # a distant-traffic spectrum, 0 dB(A)
        # birds
        self.birds = None
        if cfg.get("birds", "auto"):
            bufs = {}
            for role in ("birds_dawn", "birds_day"):
                path, info = self._recording(role, rec)
                if path is not None:
                    try:
                        bufs[role] = load_recording(path, 60.0, highpass=1200.0)
                        self.credits[role] = info
                    except Exception as e:  # noqa: BLE001
                        log(f"sound: cannot read the {role} recording ({e})")
            for role, dens in (("birds_dawn", 3.0), ("birds_day", 0.8)):
                if role not in bufs:
                    bufs[role] = synth.birds_synth(12.0, seed=4 if role == "birds_dawn" else 8, density=dens)
            self.birds = {}
            for role, buf in bufs.items():
                for side, az in (("l", -0.9), ("r", 0.9)):
                    v = synth.ShapedMono(buf, loop=True, start=(0 if side == "l" else len(buf) // 2),
                                         kinds=("direct", "reverb"))
                    v.paths[0].az = az
                    self.birds[(role, side)] = m.add(("amb", role, side), v)
        # wind in the trees (and at the ear)
        if cfg.get("wind", True):
            self.voices["wind"] = m.add(("amb", "wind"), synth.BandNoise("pink", seed=55, flutter_db=0.0,
                                                                          kinds=("direct",)))
            self.gust = 0.0
        # rain
        self.rain = None
        if env.rain >= 0.05 and cfg.get("rain", True):
            self.rain = {}
            for role in ("rain_light", "rain_heavy"):
                path, info = self._recording(role, rec)
                if path is not None:
                    try:
                        buf = load_recording(path, 45.0, highpass=200.0)
                        self.credits[role] = info
                        for side, az in (("l", -1.0), ("r", 1.0)):
                            v = synth.ShapedMono(buf, loop=True, start=0 if side == "l" else len(buf) // 3,
                                                 kinds=("direct",))
                            v.paths[0].az = az
                            self.rain[(role, side)] = m.add(("amb", role, side), v)
                    except Exception as e:  # noqa: BLE001
                        log(f"sound: cannot read the {role} recording ({e})")
            if not self.rain:            # synthesis: a hiss shaped like rain on hard ground
                for side, az in (("l", -1.0), ("r", 1.0)):
                    v = synth.BandNoise("white", seed=77 if side == "l" else 78, flutter_db=1.0, kinds=("direct",))
                    v.paths[0].az = az
                    self.rain[("synth", side)] = m.add(("amb", "rain", side), v)
            self.drops = m.add(("amb", "drops"), synth.EventVoice(synth.tick_buffers("drop", 6), seed=3,
                                                                  poisson=True, gain_db_sd=4.0, kinds=("direct",)))
            self.roof = m.add(("amb", "roof"), synth.EventVoice(synth.tick_buffers("roof", 6), seed=4,
                                                                poisson=True, gain_db_sd=4.0, kinds=("direct",)))
            log(f"sound: rain {env.rain:.1f} mm/h ({'recorded' if self.credits.keys() & {'rain_light', 'rain_heavy'} else 'synthesised'})")
        self.rain_shape = np.array([-20.0, -14.0, -9.0, -5.0, -2.0, 0.0, -1.0, -5.0])   # rain on hard ground

    def _recording(self, role, allowed):
        if not allowed:
            return None, None
        from . import soundfetch
        try:
            return soundfetch.recording(role, log=self.log)
        except Exception as e:  # noqa: BLE001 - never take the soundscape down for a recording
            self.log(f"sound: no {role} recording ({type(e).__name__}: {e}); synthesised")
            return None, None

    def update(self, t, inside=False, trees=0.5, far_band_power=None, il=None):
        """inside: the listener is in a car (cabin insertion loss `il` per band applies); trees 0..1 around;
        far_band_power: the power per band (full-scale units at ref) of the simulated traffic too far to be
        heard one by one."""
        sc, env, cfg = self.sc, self.env, self.cfg
        ref = sc.ref
        il = np.zeros(ac.NB) if il is None or not inside else np.asarray(il, float)
        mute = sc.muted or sc.calibrating
        info = {}
        # distant traffic: the city beyond the map at the hour's flow, plus the far simulated vehicles
        if "hum_l" in self.voices:
            base = float(cfg.get("distant_traffic_db", 50.0)) if cfg.get("distant_traffic", "auto") in ("auto", True) \
                else float(cfg.get("distant_traffic"))
            lvl = base + 10 * math.log10(max(env.flow(t), 0.02))
            P = synth.band_db_to_power(self.hum_shape + lvl - il, ref)
            if far_band_power is not None:
                P = P + np.asarray(far_band_power) * 10 ** (-il / 10)
            info["distant_traffic_dba"] = ac.a_weighted(10 * np.log10(np.maximum(P, 1e-30)) + ref)
            w = np.sqrt(P / 2.0).astype(np.float32) * (0.0 if mute else 1.0)
            self.voices["hum_l"].paths[0].w = w
            self.voices["hum_r"].paths[0].w = w
        # birds
        if self.birds:
            season = env.season()
            dawn = env.dawn(t)
            night = env.night and dawn == 0.0
            calm = 1.0 / (1.0 + max(0.0, env.wind - 6.0) / 3.0) * (0.3 if env.rain > 0.5 else 1.0)
            tr = max(0.15, min(1.0, trees))
            f_dawn = dawn * season * tr * calm
            f_day = season * tr * calm * (0.0 if night else 1.0)
            lev_dawn = float(cfg.get("birds_dawn_db", 58.0)) + 10 * math.log10(f_dawn) if f_dawn > 1e-3 else -99.0
            lev_day = float(cfg.get("birds_day_db", 47.0)) + 10 * math.log10(f_day) if f_day > 1e-3 else -99.0
            info["birds_dba"] = float(ac.db_sum([lev_dawn, lev_day]))
            for (role, side), v in self.birds.items():
                lev = (lev_dawn if role == "birds_dawn" else lev_day) - 3.0
                bd = v.band_db(lev, il)
                w = v.weights(bd, ref) * (0.0 if mute or lev < 10 else 1.0)
                if lev >= 10:
                    self.used.add(role)
                v.paths[0].w = w
                v.paths[1].w = w * 0.4
        # wind: leaves rustle (with the trees around) and wind at the ear, with gusts
        if "wind" in self.voices:
            u = env.wind
            dt = 0.05
            self.gust += (np.random.default_rng(int(t * 20) + 9).standard_normal() * 0.6 - self.gust) * dt / 2.0
            g = max(0.3, 1.0 + 0.35 * self.gust)
            uu = u * g
            if uu > 1.0 and not inside:
                rustle = 22.0 + 32.0 * math.log10(uu) + 10 * math.log10(max(0.1, trees))
                shape = np.array([-25.0, -18.0, -10.0, -4.0, 0.0, -1.0, -5.0, -12.0])
                bd = shape - ac.a_weighted(shape) + rustle
                if uu > 4.0:              # flow noise at the ear: low rumble
                    ear = 38.0 + 40.0 * math.log10(uu / 4.0)
                    eshape = np.array([0.0, -2.0, -6.0, -14.0, -24.0, -34.0, -44.0, -54.0])
                    bd = 10 * np.log10(10 ** (bd / 10) + 10 ** ((eshape - ac.a_weighted(eshape) + ear) / 10))
                info["wind_dba"] = ac.a_weighted(bd)
                self.voices["wind"].paths[0].w = self.voices["wind"].weights(bd, ref) * (0.0 if mute else 1.0)
            else:
                self.voices["wind"].paths[0].w = np.zeros(ac.NB, np.float32)
        # rain
        if self.rain is not None:
            R = env.rain
            out = ac.rain_level(R, inside=False)
            heavy = max(0.0, min(1.0, (R - 1.0) / 6.0))
            for (role, side), v in self.rain.items():
                share = (1.0 - heavy) if role == "rain_light" else heavy if role == "rain_heavy" else 1.0
                if not any(r == "rain_heavy" for r, _ in self.rain) and role == "rain_light":
                    share = 1.0
                if not any(r == "rain_light" for r, _ in self.rain) and role == "rain_heavy":
                    share = 1.0
                lev = out - 3.0 + 10 * math.log10(max(share, 1e-4)) - 3.0       # the drops below carry ~half
                if isinstance(v, synth.BandNoise):
                    bd = self.rain_shape - ac.a_weighted(self.rain_shape) + lev - il
                else:
                    bd = v.band_db(lev, il)
                v.paths[0].w = v.weights(bd, ref) * (0.0 if mute else 1.0)
                if share > 0.01:
                    self.used.add(role)
            # nearby drops on hard surfaces (outside) or on the roof (inside)
            drops = self.drops
            drops.period = 1.0 / max(1.0, 25.0 * R) if not inside else None
            dl = ac.rain_level(R) - 3.0 - 10 * math.log10(max(1.0, 25.0 * R) * 0.03)
            drops.paths[0].w = drops.weights(drops.band_db(dl, il), ref) * (0.0 if mute or inside else 1.0)
            roof = self.roof
            roof.period = 1.0 / max(1.0, 60.0 * R) if inside else None
            rl = ac.rain_level(R, inside=True) - 10 * math.log10(max(1.0, 60.0 * R) * 0.05)
            roof.paths[0].w = roof.weights(roof.band_db(rl), ref) * (0.0 if mute or not inside else 1.0)
            info["rain_dba"] = ac.rain_level(R, inside=inside)
        return info
