"""Night lights and rain: street lamps as the city lists them, vehicle lights by the Dutch rules, fewer cyclists in
rain.

Street lamps. A city's lighting register says, per light point, how it is mounted, which lamp it carries and how much
light that lamp gives. Eindhoven's (data.eindhoven.nl `data-openbare-verlichting`, public domain, 58,507 lights;
3,603 in the centre box) has the mast or bracket (`type_drager_mast`), the arm (`type_drager_uithouder`), the light
point's height (`lph_armatuur`, cm), the lamp kind (`soort_lamp`: led, son, sont, tl, pl, cdm-t, ...), the lamp model
(`type_lamp`, e.g. "CorePro LED Urban PLL Mains 13W 830", "SON-T PIA PLUS 150W"), its wattage and its lumens. OSM
lamps carry some of the same as tags (support, lamp_mount, lamp_type, light:colour). From them:
- mount (`mount`): "pole" (a mast, which the scenery places as a lamp mesh), or "wall" (on a facade or a column),
  "canopy" (under a ceiling or canopy), "wire" (hung on a span wire over the street), "ground" (set in the paving or a
  bench) and "special" (art works, special constructions). 29 % of the centre's lights are not on poles (566 on
  walls, 229 under canopies, 66 in the paving, 42 on wires): before, each of them stood as a street lamp on its own
  pole, often in front of a shop window. Only "pole" lamps are placed now.
- colour temperature (`cct`): from the lamp model's code where it has one (LED "830" = CRI 80+, 3000 K; "740" =
  4000 K; "3000K", "3K", "WW"), else from the lamp kind: high-pressure sodium (SON, SON-T) 2000 K, low-pressure sodium
  (SOX) 1800 K, white SON (SDW-T) 2500 K, ceramic metal halide 3000 K (CDM) or 2800 K (CosmoPolis CDO/CPO), quartz
  metal halide and mercury 4000 K, fluorescent (TL, PL) 4000 K, LED 3000 K (the register's own codes: 1,009 of the
  centre's 1,314 coded LEDs are 2700-3000 K), else the colour word (geel/yellow 2000 K, oranje/orange 1800 K).
- luminous flux (`lumen`): the register's lamp lumens (median 3,500 lm in the centre), else watts x a typical
  efficacy of the lamp kind (LED 110 lm/W, sodium 100, metal halide 85, fluorescent 75, mercury 50), capped at
  MAX_LUMEN (sports-field floodlights of 220,000 lm stand on the same kind of mast mesh as a street lamp).

On CARLA 1.0 the lamps that carry a light (season.LAMP_LIGHTS) get that colour and flux through the LightManager
(`apply_lamp_lights`). Note: CARLA ue58-dev applies a client's set_intensity/set_color only to its own record, not
to the light components, until UCarlaLight::SetLightState applies them (unreal/patches/CarlaLight_SetLightState.patch);
without the patch the lamps keep CARLA's 27,000 lm and warm white. CARLA's street-light intensity is lumens, but
values under 1,000 are read as kilolumens (UCarlaLight::StreetIntensityToLumens), so at least 1,000 lm is sent.
CARLA 0.9.16 cannot add a light at run time: its lamp meshes stay dark.

Vehicle lights (RVV 1990, art. 32 and 34). Dipped beam at night, and by day "indien het zicht ernstig wordt belemmerd"
(art. 32(1)); rear lights whenever dipped beam is on (art. 32(3)); the rear fog light only in fog or snowfall that
limits visibility to under 50 m (art. 34(2)), never for rain. The law gives no number for seriously impaired
visibility; here: visibility under 1,000 m (fog in the meteorological sense) or rain of at least RAIN_LIGHTS_MMH
(the start of "moderate rain", 2.5 mm/h, American Meteorological Society glossary). Night: the sun below 3 degrees,
as before (a little before sunset, when most drivers switch on).

Cyclists in rain. Wessel (2020, "Using weather forecasts to forecast whether bikes are used", Institute of Transport
Economics Muenster, working paper 32; hourly counts at 188 German counting stations 2017-2018, log-linear) found
per class of the hour's rain: drizzle under 0.5 mm/h -15 %, 0.5-1 mm/h -18 %, 1-2 mm/h -23 %, 2-5 mm/h -26 %,
5-10 mm/h -25 %, and -21 % more when it rained in the three hours before (exp of the coefficients -0.158, -0.199,
-0.264, -0.303, -0.285 and -0.241). His class above 10 mm/h (-0.095) rests on 0.04 % of the hours; the 5-10 mm/h
value is used for it. Bike demand (cycling.py and the plain random bike trips) is scaled by this factor.
"""
import math
import re

# --- street lamps -------------------------------------------------------------------------------------------------

MOUNTS = ("pole", "wall", "canopy", "wire", "ground", "special")
# register mast types (Eindhoven type_drager_mast, upper case) -> mount; checked in order, first match wins
_REG_MOUNT = [
    ("WAND/GEVEL", "wall"), ("PIJLER/KOLOM", "wall"), ("OP DAK", "wall"),
    ("PLAFOND/LUIFEL", "canopy"), ("LUIFEL", "canopy"),
    ("SPANDRAAD", "wire"), ("OVERSPANNING", "wire"),
    ("BESTRATING", "ground"), ("INGROUND", "ground"), ("TEGEL", "ground"), ("BODEMSTUK", "ground"),
    ("ZITBANK", "ground"),
    ("SPECIALE CONSTRUCTIE", "special"), ("KUNSTWERK", "special"),
]
# OSM support= / lamp_mount= values -> mount (anything else, or none: a pole)
_OSM_MOUNT = {"wall_mounted": "wall", "wall": "wall", "facade": "wall", "suspended": "wire", "wire": "wire",
              "catenary": "wire", "ceiling": "canopy", "ground": "ground", "recessed": "ground"}

SODIUM_K = 2000             # high-pressure sodium (SON, SON-T)
KIND_K = [                  # lamp kind (register soort_lamp, OSM lamp_type, the model's first word) -> K
    (r"^sox|low_pressure_sodium", 1800), (r"^sdw", 2500), (r"^son|sodium|^hps", SODIUM_K),
    (r"^cdo|^cpo|cosmo", 2800), (r"^cdm|ceramic", 3000), (r"^mhn|^hpi|^hqi|metal_halide|halide", 4000),
    (r"^hpl|^hp\b|^hp$|mercury", 4000), (r"^tl|^pl|fluorescent|^plt|^pll", 4000), (r"^led|led$", 3000),
    (r"gas", 2700), (r"halogen|^ha\b|incandescent|^gls", 2800),
]
COLOUR_K = {"geel": 2000, "yellow": 2000, "oranje": 1800, "orange": 1800}
EFFICACY = [                # lm/W by kind (typical lamp efficacies), for a lamp listed with watts only
    (r"^led|led$", 110.0), (r"^sox|^son|sodium|^hps|^sdw", 100.0), (r"^cd|^cpo|^mhn|^hpi|halide|cosmo", 85.0),
    (r"^tl|^pl|fluorescent", 75.0), (r"^hp|mercury", 50.0),
]
MAX_LUMEN = 40000.0         # a street-lamp mesh with a floodlight's 220,000 lm would light a whole block
CARLA_MIN_LUMEN = 1000.0    # CARLA 1.0 reads street-light values under 1,000 as kilolumens


def _num(v):
    try:
        x = float(str(v).replace(",", "."))
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def mount(tags):
    """How a lamp is mounted: one of MOUNTS ("pole" when nothing says otherwise)."""
    reg = (tags.get("support:register") or "").upper()
    if reg:
        for key, m in _REG_MOUNT:
            if key in reg:
                return m
        return "pole"
    for k in ("support", "lamp_mount"):
        v = (tags.get(k) or "").lower()
        if v in _OSM_MOUNT:
            return _OSM_MOUNT[v]
    return "pole"


def _kind(tags):
    kind = (tags.get("lamp_type") or "").strip().lower()
    if kind in ("", "electric"):
        model = (tags.get("lamp_model") or "").strip().lower()
        kind = model.split()[0] if model else kind
    return kind


def cct(tags):
    """Colour temperature (K) of a lamp, or None when nothing tells it."""
    model = (tags.get("lamp_model") or "").lower()
    m = re.search(r"(\d{4})\s?k\b", model)
    if m and 1500 <= int(m.group(1)) <= 6500:
        return int(m.group(1))
    m = re.search(r"\b([2-6])k\b", model)
    if m:
        return int(m.group(1)) * 1000
    m = re.search(r"(?<![\d.,])[6789]([2-6]\d)(?![\d])", model)     # CRI digit + CCT/100: 830, 740, 727
    if m:
        return int(m.group(1)) * 100
    if re.search(r"\bww\b|warm ?white", model):
        return 3000
    if re.search(r"\bnw\b|neutral ?white", model):
        return 4000
    kind = _kind(tags)
    for pat, k in KIND_K:
        if kind and re.search(pat, kind):
            return k
    return COLOUR_K.get((tags.get("light:colour") or "").strip().lower())


def lumen(tags):
    """Luminous flux (lm) of a lamp, or None: the listed lumens, else watts x the kind's efficacy (capped)."""
    lm = _num(tags.get("light:lumen"))
    if not lm or lm <= 0:
        w = _num(tags.get("light:wattage"))
        kind = _kind(tags)
        eff = next((e for pat, e in EFFICACY if kind and re.search(pat, kind)), None)
        lm = w * eff if w and w > 0 and eff else None
    return min(MAX_LUMEN, lm) if lm else None


def lamp_spec(tags):
    """{"mount", "cct" (K or None), "lumen" (lm or None)} of a lamp from its register or OSM tags."""
    return {"mount": mount(tags), "cct": cct(tags), "lumen": lumen(tags)}


def kelvin_rgb(k):
    """Linear-ish RGB (0-1, brightest channel 1) of a black body at k kelvin (Tanner Helland's fit to the CIE 1964
    10-degree colour matching functions, 1000-40000 K)."""
    t = max(1000.0, min(40000.0, float(k))) / 100.0
    if t <= 66:
        r = 255.0
        g = 99.4708025861 * math.log(t) - 161.1195681661
        b = 0.0 if t <= 19 else 138.5177312231 * math.log(t - 10) - 305.0447927307
    else:
        r = 329.698727446 * (t - 60) ** -0.1332047592
        g = 288.1221695283 * (t - 60) ** -0.0755148492
        b = 255.0
    rgb = [max(0.0, min(255.0, c)) / 255.0 for c in (r, g, b)]
    top = max(rgb)
    return tuple(round(c / top, 4) for c in rgb)


def carla_lumen(lm, scale=1.0):
    """The intensity to send to a CARLA 1.0 street light for lm lumens (at least CARLA_MIN_LUMEN, see above)."""
    return max(CARLA_MIN_LUMEN, float(lm) * float(scale))


def spec_rows(items, specs):
    """scenery.json's "lamp_specs": [x, y, cct or 0, lumen or 0] per placed lamp (items: the lamp items,
    specs: their lamp_spec dicts, aligned)."""
    return [[it[1], it[2], int(s.get("cct") or 0), int(round(s.get("lumen") or 0))]
            for it, s in zip(items, specs) if s and (s.get("cct") or s.get("lumen"))]


class SpecIndex:
    """Lamp specs by position (scenery.json "lamp_specs"): the spec of the lamp standing at (x, y), within 0.5 m."""

    def __init__(self, rows):
        self.cells = {}
        for x, y, k, lm in rows or ():
            self.cells.setdefault((int(x // 2), int(y // 2)), []).append((x, y, k or None, lm or None))

    def __len__(self):
        return sum(len(v) for v in self.cells.values())

    def at(self, x, y, tol=0.5):
        best, bd = None, tol
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for row in self.cells.get((int(x // 2) + i, int(y // 2) + j), ()):
                    d = math.hypot(row[0] - x, row[1] - y)
                    if d <= bd:
                        best, bd = row, d
        return best


def lamp_settings(lights, index, scale=1.0, colours=True):
    """For CARLA lights (objects with .location.x/.y) near lamps of the index: [(light, (r, g, b) or None,
    intensity or None)]; lights with no listed spec are left out."""
    out = []
    for lt in lights:
        row = index.at(lt.location.x, lt.location.y)
        if row is None:
            continue
        _, _, k, lm = row
        col = kelvin_rgb(k) if (k and colours) else None
        inten = carla_lumen(lm, scale) if lm else None
        if col or inten:
            out.append((lt, col, inten))
    return out


def apply_lamp_lights(world, actor_ids, index, scale=1.0, colours=True, log=print):
    """Give the street lights of the lamps just spawned (actor_ids) the colour and flux of their register entry.
    Returns how many lights were set. Fails soft (a server without the light API keeps CARLA's defaults)."""
    if not index or not actor_ids:
        return 0
    import carla
    try:
        # the lamps just spawned, by position (a CARLA light reports its owner actor's location)
        where = [a.get_location() for a in world.get_actors(list(actor_ids))]
        spawned = SpecIndex([[p.x, p.y, 0, 0] for p in where])
        lm = world.get_lightmanager()
        mine = [lt for lt in lm.get_all_lights(carla.LightGroup.Street)
                if spawned.at(lt.location.x, lt.location.y, 1.0) is not None]
        sets = lamp_settings(mine, index, scale, colours)
        col = [(lt, c) for lt, c, _ in sets if c]
        inten = [(lt, v) for lt, _, v in sets if v]
        if col:
            lm.set_colors([lt for lt, _ in col], [carla.Color(int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
                                                  for _, c in col])
        if inten:
            lm.set_intensities([lt for lt, _ in inten], [float(v) for _, v in inten])
        return len(sets)
    except Exception as err:  # noqa: BLE001 - the lamps keep CARLA's own light
        log(f"scenery: lamp colours and lumens not set ({err})")
        return 0


# --- vehicle lights -----------------------------------------------------------------------------------------------

NIGHT_SUN = 3.0             # sun below this (deg): night lights (as run.set_weather's "night")
RAIN_LIGHTS_MMH = 2.5       # rain at least this (mm/h): visibility seriously impaired (moderate rain, AMS)
FOG_LIGHTS_M = 1000.0       # visibility under this (m): seriously impaired (fog)
REAR_FOG_M = 50.0           # visibility under this (m) in fog: the rear fog light (RVV art. 34(2))


def visibility_from_fog(fog_density):
    """Visibility (m) for a CARLA fog_density, the inverse of realworld.carla_weather_fields' mapping
    (fog = 100 (1 - vis/10 km)^1.5); None without fog."""
    f = float(fog_density or 0.0)
    if f <= 0:
        return None
    return round(10000.0 * (1.0 - min(1.0, f / 100.0) ** (2.0 / 3.0)), 1)


def vehicle_lights(sun_altitude, rain_mmh=0.0, visibility_m=None, mode="auto"):
    """Which lights cars drive with: {"low_beam", "position", "fog_rear": bool, "why": text}. mode "on"/"off"
    forces dipped beam (simulation.headlights)."""
    night = sun_altitude is not None and sun_altitude < NIGHT_SUN
    fog = visibility_m is not None and visibility_m < FOG_LIGHTS_M
    rain = (rain_mmh or 0.0) >= RAIN_LIGHTS_MMH
    why = [w for w, on in (("night", night), (f"visibility {visibility_m:.0f} m" if fog else "", fog),
                           (f"rain {rain_mmh:.1f} mm/h" if rain else "", rain)) if on]
    low = night or fog or rain
    if mode in ("on", True):
        low, why = True, ["simulation.headlights on"]
    elif mode in ("off", False):
        low, why = False, ["simulation.headlights off"]
    rear_fog = low and visibility_m is not None and visibility_m < REAR_FOG_M and not rain
    return {"low_beam": low, "position": low, "fog_rear": rear_fog, "why": ", ".join(why)}


def weather_lights(weather, rec=None, mode="auto"):
    """vehicle_lights for an applied carla.WeatherParameters (or an object with its fields) and, when the weather is
    real, its Open-Meteo hour (rain mm/h and visibility m from there)."""
    rec = rec or {}
    rain = rec.get("rain") if rec.get("rain") is not None else (getattr(weather, "precipitation", 0.0) or 0.0) / 20.0
    vis = rec.get("visibility")
    if vis is None:
        vis = visibility_from_fog(getattr(weather, "fog_density", 0.0))
    return vehicle_lights(getattr(weather, "sun_altitude_angle", 90.0), float(rain or 0.0), vis, mode)


def light_bits(lights, carla):
    """The carla.VehicleLightState bits for vehicle_lights' answer."""
    LS = carla.VehicleLightState
    bits = 0
    if lights.get("low_beam"):
        bits |= int(LS.LowBeam)
    if lights.get("position"):
        bits |= int(LS.Position)
    if lights.get("fog_rear"):
        bits |= int(LS.Fog)
    return bits


def brake_bits(current, braking, carla):
    """A car's light state with the brake light following its control (the rest as it was)."""
    LS = carla.VehicleLightState
    keep = int(current) & ~int(LS.Brake)
    return LS(keep | (int(LS.Brake) if braking else 0))


# --- cyclists in rain ---------------------------------------------------------------------------------------------

WESSEL_RAIN = [(0.5, -0.158), (1.0, -0.199), (2.0, -0.264), (5.0, -0.303), (float("inf"), -0.285)]
WESSEL_RAIN_3H = -0.241
RAIN_MIN_MMH = 0.1          # Open-Meteo's smallest reported amount: below it, dry


def bike_rain_factor(rain_mmh, rain_before_3h_mm=0.0):
    """Multiplier on cycling demand for the hour's rain (mm/h) and the rain of the three hours before (mm)."""
    b = 0.0
    r = float(rain_mmh or 0.0)
    if r >= RAIN_MIN_MMH:
        b += next(c for top, c in WESSEL_RAIN if r < top)
    if float(rain_before_3h_mm or 0.0) >= RAIN_MIN_MMH:
        b += WESSEL_RAIN_3H
    return round(math.exp(b), 4)


PRESET_RAIN = [("hardrain", 100.0), ("midrain", 60.0), ("softrain", 30.0)]   # CARLA presets' precipitation


def trial_rain(cfg, map_dir, log=print):
    """(rain mm/h, rain in the 3 hours before in mm, source) for a trial: Open-Meteo's hour with weather "real",
    else the preset's or the weather dict's precipitation (CARLA precipitation 20 = 1 mm/h, as the real mapping)."""
    sc = cfg["simulation"]
    w = sc.get("weather")
    if w == "real":
        from datetime import datetime
        from pathlib import Path
        from . import realworld, transit
        bb = cfg["map"]["bbox"]
        lat, lon = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
        t = sc.get("time") or "now"
        when = "now" if str(t).strip() == "now" else transit.trial_time(t, (lat, lon), Path(map_dir) / "real")
        try:
            rec, _, _ = realworld.weather_at(lat, lon, when if isinstance(when, datetime) else "now",
                                             Path(map_dir) / "real")
        except Exception as err:  # noqa: BLE001 - offline: as if dry
            log(f"cycling: no weather for the rain factor ({err})")
            return 0.0, 0.0, "none"
        rain = float(rec.get("rain") or 0.0) + 0.7 * float(rec.get("snowfall") or 0.0)
        return rain, float(rec.get("precipitation_before_3h") or 0.0), "open-meteo"
    if isinstance(w, dict):
        return float(w.get("precipitation") or 0.0) / 20.0, 0.0, "weather"
    name = str(w or "").lower()
    for key, p in PRESET_RAIN:
        if key in name:
            return p / 20.0, 0.0, "preset"
    return 0.0, 0.0, "preset"


def bike_weather_factor(cfg, map_dir, log=print):
    """traffic.cycling.weather: the rain factor on bike demand (1.0 when off or dry), logged."""
    cc = cfg["traffic"].get("cycling") or {}
    if cc.get("weather", "auto") in (False, None, "off"):
        return 1.0
    rain, before, src = trial_rain(cfg, map_dir, log)
    f = bike_rain_factor(rain, before)
    if f < 1.0:
        log(f"cycling: rain {rain:.1f} mm/h ({before:.1f} mm in the 3 h before, {src}): bike demand x {f:.2f} "
            "(Wessel 2020)")
    return f


class BrakeLight:
    """Lights a car's brake lights while its control brakes (the bot driver and scripted cars; keyboard and wheel
    inputs do their own). Sends a light state only when braking starts or ends, with the car's other lights as they
    are then (headlights, blinkers)."""

    def __init__(self, vehicle, send):
        self.vehicle, self.send, self.on = vehicle, send, False

    def update(self, ctl, carla):
        braking = bool(ctl is not None and getattr(ctl, "brake", 0.0) > 0.0)
        if braking == self.on:
            return
        self.on = braking
        try:
            cur = self.vehicle.get_light_state()
        except (RuntimeError, AttributeError):
            cur = 0
        self.send(carla.command.SetVehicleLightState(self.vehicle.id, brake_bits(cur, braking, carla)))
