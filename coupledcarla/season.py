"""Leaf state of the trees by the trial's date and place, and street lamps that light at night (CARLA 1.0).

Leaves. Deciduous trees are in leaf, in autumn colour or bare depending on the date and the place:
- The dates follow latitude (MODEL below): leaf-out on day 113 (23 April) at 50 degrees, the European mean start
  of the growing season from the phenological gardens (Rotzer and Chmielewski 2001, Climate Research 18: 249-257;
  most of Europe 10-25 April), leaf colouring from day 280 and leaf fall on day 305 (1 November), as Germany's
  phenological clock (DWD: "Vollherbst" in October, "Spatherbst" with the leaf fall at the turn of the month).
  Towards the poles leaf-out comes later and leaf fall earlier (Hopkins' bioclimatic law, 1918: about 4 days per
  degree in spring in North America, where continentality adds to latitude). Here 1.5 days per degree in spring and
  2 in autumn, which keeps the project's cities roughly at their usual dates: Tokyo's ginkgos and zelkovas leaf out
  in early April and drop in early December, Ushuaia's lenga beeches leaf out late October and turn in April. The
  southern hemisphere is half a year apart; within 25 degrees of the equator trees stay in leaf. Latitude alone
  misses continental climates (Burlington, Vermont leafs out about four weeks later than the model's mid-April).
- Satellite dates where wanted (scenery.leaves "modis"): NASA's MODIS Land Cover Dynamics (MCD12Q2 v6.1, 500 m,
  yearly) is keyless through the ORNL DAAC REST service, read for a 4 x 4 km box at the map's centre (medians of
  MidGreenup, Senescence and MidGreendown over 2019-2023, cached in real/phenology.json). It is not the default:
  each band takes 40-60 s and the service often answers 502/503, and in city centres the 500 m pixels follow lawns
  and crops more than trees (Senescence in June-July in London and Leuven, MidGreendown on 18 September in
  Westminster; Stellenbosch's grass greens with the winter rains in June). Dates more than MAX_OFF days from the
  model are refused. Copernicus HR-VPP (10 m, Europe) would see street trees but needs a CDSE/WEkEO account.
- Only deciduous trees change: genus and species from OSM or the city register (leaf_cycle where mapped), with
  evergreen genera (pines, spruces, cypresses, hollies, holm and cork oaks, eucalypts, figs, palms, ...) excluded
  and deciduous conifers (larch, dawn redwood, swamp cypress) and ginkgo included.

CARLA has no bare or winter tree meshes (0.9.16's cooked content and 1.0's source tree have none: the "Base"
trees are the same trees with a tree pit), so a bare tree is a stand-in: the same family at half its size by
default (scenery.bare "small"), or left out ("drop"), or kept in leaf ("keep"). Autumn has real meshes: CARLA's
sugar maple in autumn colour (0.9.16 SM_Acer_Red_01, 1.0 SM_Acer01_B; its leaf textures are the
Acer_Saccharum_*_Autumn ones) and the red Japanese maples; during the colouring weeks a growing share of the
deciduous trees take one of them. There is no snow in either CARLA (WeatherParameters has no snow or frost field
in 0.9.16 or 1.0), and the ground tiles cannot change colour at run time.

Lamps. CARLA 1.0's street lamp blueprints (BP_StreetLight06_A is the scenery's SM_StreetLight06_A mesh with a spot
light and a CarlaLight, BP_StreetLight03 the park lamp's) are not in the blueprint library, but a content pack can
list them (data/carla_packs/CoupledCarla: Content/Config/BlueprintParameters.json, loaded by the server started
with -carla-packs=<that folder> or CARLA_PACKS). Spawned as blueprint.cc_streetlight06_a they register with the
LightManager as street lights, and the server switches every street light on when the sun is at or below
the horizon (measured: on at altitude 0, off at 1).
"""
import json
import math
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from datetime import date, datetime, timedelta
from pathlib import Path

import requests

MODIS = "https://modis.ornl.gov/rst/api/v1/MCD12Q2/subset"
BANDS = {"greenup": "MidGreenup.Num_Modes_01", "senescence": "Senescence.Num_Modes_01",
         "greendown": "MidGreendown.Num_Modes_01"}
YEARS = (2019, 2023)        # the five most recent years MCD12Q2 v6.1 had for every pixel when this was written
BOX_KM = 2                  # km above/below and left/right of the centre
FILL = 32767
SOURCES = {
    "modis_phenology": ("NASA MODIS/Terra+Aqua Land Cover Dynamics MCD12Q2 v6.1 (Friedl, Gray, Sulla-Menashe 2019) "
                        "via the ORNL DAAC MODIS web service", "NASA EOSDIS open data (no restrictions)"),
}
TROPICS = 25.0              # |latitude| under this: no leafless season
# Leaf dates (day of year, northern hemisphere) at |latitude| 50 and the change per degree further from the equator
# (see the module text): leaf-out, start of the autumn colour, leaf fall (bare from then on)
MODEL = {"greenup": (113.0, 1.5), "senescence": (280.0, -2.0), "greendown": (305.0, -2.0)}
SEASON_MIN, SEASON_MAX = 120, 300   # a plausible leaf season (days) for a temperate deciduous cycle
MAX_OFF = 45                # MODIS dates further than this (days) from the latitude model: not the trees

# --- deciduous or evergreen --------------------------------------------------------------------------------
EVERGREEN_GENERA = {
    "pinus", "picea", "abies", "taxus", "pseudotsuga", "cedrus", "sequoia", "sequoiadendron", "araucaria", "tsuga",
    "cupressus", "thuja", "chamaecyparis", "juniperus", "cryptomeria", "podocarpus", "agathis", "cunninghamia",
    "calocedrus", "x cupressocyparis", "cupressocyparis", "hesperocyparis", "pittosporum",
    "ilex", "olea", "laurus", "ligustrum", "eucalyptus", "corymbia", "citrus", "cinnamomum", "ficus", "arbutus",
    "photinia", "metrosideros", "callistemon", "melaleuca", "schinus", "ceratonia", "casuarina", "grevillea",
    "tristaniopsis", "lophostemon", "syzygium", "acmena", "waterhousea", "elaeocarpus", "brachychiton",
    "cocos", "phoenix", "washingtonia", "trachycarpus", "roystonea", "archontophoenix", "syagrus", "livistona",
    "butia", "chamaerops", "dypsis", "mangifera", "terminalia", "tabebuia", "handroanthus", "spathodea",
    "aporosa", "pongamia", "michelia", "camellia", "viburnum", "osmanthus", "quillaja",
    "acacia", "banksia", "agonis", "angophora", "harpephyllum", "afrocarpus", "nerium",
}
EVERGREEN_SPECIES = {
    "quercus ilex", "quercus suber", "quercus virginiana", "quercus agrifolia", "quercus coccifera",
    "quercus rotundifolia", "quercus myrsinifolia", "quercus glauca", "magnolia grandiflora",
    "prunus laurocerasus", "prunus lusitanica", "nothofagus betuloides", "nothofagus dombeyi",
    "nothofagus cunninghamii", "lithocarpus edulis", "cinnamomum camphora", "elaeagnus pungens",
}
DECIDUOUS_CONIFERS = {"larix", "metasequoia", "taxodium", "glyptostrobus", "ginkgo", "pseudolarix"}
EVERGREEN_FAMILIES = {"pine", "cypress", "palm", "tropical"}

# Autumn-coloured meshes per CARLA generation (0: 0.9.x, 1: 1.0): the sugar maple with Acer_Saccharum autumn leaf
# textures (orange, mean RGB 121/98/45 against the summer leaf's 77/105/62) and the red Japanese maples
AUTUMN = {0: ["SM_Acer_Red_01", "SM_Japanese_Maple_01", "SM_Japanese_Maple_03", "SM_Japanese_Maple_04"],
          1: ["SM_Acer01_B", "SM_JapaneseMaple01_A", "SM_JapaneseMaple01_B", "SM_JapaneseMaple01_C",
              "SM_JapaneseMaple01_D"]}
BARE_SCALE = 0.5            # bare "small": the tree at this share of its height
AUTUMN_PEAK = 0.7           # share of deciduous trees in autumn colour at MidGreendown (the rest already fallen)

# --- lamps that light (CARLA 1.0) ---------------------------------------------------------------------------
PACKS = Path(__file__).resolve().parent / "data" / "carla_packs"
LAMP_LIGHTS = {             # scenery lamp mesh (1.0 name) -> the blueprint (content pack) with that mesh and a light;
    # both checked on a 1.0 server: the same bounding box and pivot as the static.prop.mesh of the mesh
    "SM_StreetLight06_A": "blueprint.cc_streetlight06_a", "SM_StreetLight03": "blueprint.cc_streetlight03",
}
NIGHT_SUN = 0.0             # the sun at or below this: lamps carry their light (a 1.0 server switches
                            # every street light on at altitude 0 and off at 1, measured)


def log(msg):
    print(f"[season] {msg}", flush=True)


def _ua():
    from .realworld import UA
    return UA


# --- phenology ------------------------------------------------------------------------------------------------

def _circ_median(doys):
    """Median of days of the year on the circle (an autumn date of the southern hemisphere wraps the new year)."""
    if not doys:
        return None
    a = [2 * math.pi * d / 365.25 for d in doys]
    mean = math.atan2(sum(map(math.sin, a)), sum(map(math.cos, a)))
    ref = mean * 365.25 / (2 * math.pi)
    rel = sorted(((d - ref + 182.625) % 365.25) - 182.625 for d in doys)
    n = len(rel)
    med = rel[n // 2] if n % 2 else (rel[n // 2 - 1] + rel[n // 2]) / 2
    return round((ref + med) % 365.25, 1)


def _band(lat, lon, band, years=YEARS, km=BOX_KM, timeout=180, tries=3):
    params = {"latitude": f"{lat:.4f}", "longitude": f"{lon:.4f}", "band": band,
              "startDate": f"A{years[0]}001", "endDate": f"A{years[1]}001", "kmAboveBelow": km, "kmLeftRight": km}
    last = None
    for attempt in range(tries):
        try:
            r = requests.get(MODIS, params=params, headers={**_ua(), "Accept": "application/json"}, timeout=timeout)
            r.raise_for_status()
            return r.json()
        except (requests.RequestException, ValueError) as err:
            last = err
            if attempt + 1 < tries:
                time.sleep(5 * (attempt + 1))
    raise RuntimeError(f"MODIS {band}: {last}")


def _doys(sub):
    """Days of the year of a subset's valid pixels, all years together."""
    out = []
    for s in sub.get("subset", []):
        for v in s.get("data", []):
            if v is not None and v != FILL:
                out.append((date(1970, 1, 1) + timedelta(days=int(v))).timetuple().tm_yday)
    return out


def modis_phenology(lat, lon, years=YEARS, km=BOX_KM, quick=False):
    """The place's typical leaf-out, colouring and leaf-fall days of the year from MCD12Q2 (raises when the service
    fails; returns the dict with the medians, None for a band without data). The service answers in 40-60 s;
    quick: one try per band with a 90 s timeout (at trial time), else three tries."""
    kw = {"timeout": 90, "tries": 1} if quick else {}
    with ThreadPoolExecutor(len(BANDS)) as ex:
        subs = dict(zip(BANDS, ex.map(lambda b: _band(lat, lon, b, years, km, **kw), BANDS.values())))
    out = {"source": "modis", "years": list(years), "box_km": 2 * km}
    for k, sub in subs.items():
        d = _doys(sub)
        out[k] = _circ_median(d)
        out[f"{k}_n"] = len(d)
    return out


def model_phenology(lat):
    """Typical dates from latitude alone (MODEL), None within TROPICS of the equator."""
    if abs(lat) < TROPICS:
        return None
    d = abs(lat) - 50.0
    shift = 0.0 if lat >= 0 else 182.6
    out = {"source": "latitude"}
    for k, (a, b) in MODEL.items():
        out[k] = round((a + b * d + shift) % 365.25, 1)
    return out


def _cycle(ph, lat):
    """A temperate deciduous cycle: leaf-out in the hemisphere's spring and a leaf season of SEASON_MIN-MAX days."""
    if not ph or any(ph.get(k) is None for k in BANDS):
        return False
    spring = ph["greenup"] if lat >= 0 else (ph["greenup"] + 182.6) % 365.25
    length = (ph["greendown"] - ph["greenup"]) % 365.25
    return abs(lat) >= TROPICS and 20 <= spring <= 180 and SEASON_MIN <= length <= SEASON_MAX


def plausible(ph, lat):
    """MODIS dates that describe the trees: a temperate cycle (_cycle) whose three dates are each within MAX_OFF
    days of the latitude model. MODIS sees whatever greens a 500 m pixel: grass that greens with the winter rains
    (Stellenbosch: MidGreenup in June), lawns that brown in summer (London: Senescence in late June) or paddies
    (Taipei: MidGreendown in September) are refused."""
    if not _cycle(ph, lat):
        return False
    m = model_phenology(lat)
    return all(abs(((ph[k] - m[k] + 182.6) % 365.25) - 182.6) <= MAX_OFF for k in BANDS)


def phenology(bbox, real_dir=None, fetch=True, quick=False, modis=False):
    """The map's leaf dates: the latitude model, or with modis MCD12Q2's (cached in real/phenology.json) where
    plausible. Fails soft: offline or a broken service falls back to the model with one log line; a failure is
    noted in real/phenology_failed.json and not tried again that day."""
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    if not modis:
        return model_phenology(lat)
    path = Path(real_dir) / "phenology.json" if real_dir else None
    failed = Path(real_dir) / "phenology_failed.json" if real_dir else None
    ph = None
    if path and path.exists():
        ph = json.loads(path.read_text())
    elif fetch and failed and failed.exists() and \
            json.loads(failed.read_text()).get("date") == date.today().isoformat():
        log("MODIS phenology failed earlier today; leaf dates from latitude")
    elif fetch:
        t0 = time.time()
        try:
            ph = modis_phenology(lat, lon, quick=quick)
            ph.update(fetched=datetime.now().isoformat(timespec="minutes"), what=SOURCES["modis_phenology"][0],
                      licence=SOURCES["modis_phenology"][1])
            if path:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(ph, indent=1))
            log(f"MODIS MCD12Q2 at {lat:.3f}, {lon:.3f}: leaf-out day {ph['greenup']}, colouring "
                f"{ph['senescence']}, leaf fall {ph['greendown']} ({ph['greenup_n']} pixel-years, "
                f"{time.time() - t0:.0f} s)")
        except Exception as err:  # noqa: BLE001 - offline or the service is down: the latitude model
            log(f"MODIS phenology failed ({err}); leaf dates from latitude")
            if failed:
                failed.parent.mkdir(parents=True, exist_ok=True)
                failed.write_text(json.dumps({"date": date.today().isoformat(), "error": str(err)[:300]}))
    if plausible(ph, lat):
        return ph
    if ph is not None:
        log(f"MODIS sees no temperate leaf cycle here ({ph.get('greenup')}-{ph.get('greendown')}); "
            "leaf dates from latitude")
    return model_phenology(lat)


def leaf_state(day, ph):
    """("leaf" | "autumn" | "bare", progress) on a date: autumn from Senescence to MidGreendown (progress 0-1
    through it), bare from MidGreendown to MidGreenup."""
    if ph is None:
        return "leaf", 0.0
    doy = day.timetuple().tm_yday
    up, sen, down = ph["greenup"], ph["senescence"], ph["greendown"]
    since_up = (doy - up) % 365.25
    if since_up < (sen - up) % 365.25:
        return "leaf", 0.0
    if since_up < (down - up) % 365.25:
        span = max(1.0, (down - sen) % 365.25)
        return "autumn", round(((doy - sen) % 365.25) / span, 3)
    return "bare", 0.0


def deciduous(tags, family):
    """True for a tree that loses its leaves in winter: OSM leaf_cycle, then species, genus, leaf type, family."""
    cycle = (tags.get("leaf_cycle") or "").lower()
    if cycle in ("evergreen", "semi_evergreen"):
        return False
    if cycle in ("deciduous", "semi_deciduous"):
        return True
    name = (tags.get("species") or tags.get("taxon") or tags.get("genus") or "").lower().replace("×", "x").strip()
    parts = name.split()
    species = " ".join(parts[:2]) if len(parts) > 1 else ""
    genus = (tags.get("genus") or (parts[0] if parts else "")).lower()
    if species in EVERGREEN_SPECIES:
        return False
    if genus in DECIDUOUS_CONIFERS:
        return True
    if genus in EVERGREEN_GENERA:
        return False
    if tags.get("leaf_type") == "needleleaved":
        return False
    return family not in EVERGREEN_FAMILIES


def variants(cat, ms, tags, family, mesh, scale, key):
    """A deciduous tree's stand-ins as {"autumn": mesh name or None, "autumn_scale", "bare": mesh, "bare_scale"},
    or None for an evergreen one. Sizes follow the placed tree's height."""
    from . import scenery
    if isinstance(family, list):
        family = family[scenery._h(key, "fam") % len(family)]
    if not deciduous(tags, family):
        return None
    height = cat.size(mesh)[2] * scale
    out = {}
    autumn = [m for m in AUTUMN[scenery.major(cat.version)] if m in cat]
    if autumn:
        a = min(autumn, key=lambda m: abs(math.log(cat.size(m)[2] / height)) + (scenery._h(key, m) % 10) / 100.0)
        out["autumn"], out["autumn_scale"] = a, max(0.5, min(1.6, height / cat.size(a)[2]))
    b, s = scenery._tree(cat, family, height * BARE_SCALE, key)
    out["bare"], out["bare_scale"] = b, s
    return out


def record(pl, cat, tags, family, mesh, scale, key, cx, cy):
    """At scenery build: note the stand-ins of the tree just placed (pl.items[-1]) if it is deciduous."""
    alt = variants(cat, cat.m, tags, family, mesh, scale, key)
    if alt is None:
        return
    item = pl.items[-1]
    yaw = item[4]
    x0, y0, z0, _ = cat.place(mesh, cx, cy, 0.0, yaw, scale, 0.1)

    def rel(m, s):      # the stand-in's transform relative to the tree's (snap_scenery.py moves only item z)
        x, y, z, _ = cat.place(m, cx, cy, 0.0, yaw, s, 0.1)
        return [m, round(s, 3), round(x - x0, 2), round(y - y0, 2), round(z - z0, 2)]

    a = rel(alt["autumn"], alt["autumn_scale"]) if alt.get("autumn") else [None, 0.0, 0.0, 0.0, 0.0]
    if not hasattr(pl, "leaf"):
        pl.leaf = []
    pl.leaf.append([len(pl.items) - 1] + a + rel(alt["bare"], alt["bare_scale"]))


def finish(pl):
    """scenery.json's "leaf" list: [item, autumn mesh index (-1: none), scale, dx, dy, dz, bare mesh index, scale,
    dx, dy, dz] per deciduous tree. The stand-in meshes join the mesh list after every placed one."""
    out = []
    for row in getattr(pl, "leaf", []):
        row = list(row)
        for j in (1, 6):
            row[j] = -1 if row[j] is None else pl.mesh_index.setdefault(row[j], len(pl.mesh_index))
        out.append(row)
    if out:
        pl.counts["tree_deciduous"] = len(out)
    return out


# --- run time -------------------------------------------------------------------------------------------------

def trial_leaves(cfg, mdir, log=log):
    """The leaf state for a trial: (state, progress, info). "auto" needs simulation.time (no date: in leaf)."""
    sc = cfg["simulation"].get("scenery") or {}
    want = sc.get("leaves", "auto")
    info = {"leaves": want}
    if want in (False, None, "off", "leaf"):
        return "leaf", 0.0, info
    if want in ("autumn", "bare"):
        return want, 0.5 if want == "autumn" else 0.0, info
    if not cfg["simulation"].get("time"):
        info["note"] = "no simulation.time: trees in leaf"
        return "leaf", 0.0, info
    from . import transit
    bbox = cfg["map"]["bbox"]
    latlon = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    when = transit.trial_time(cfg["simulation"].get("time"), latlon, Path(mdir) / "real")
    ph = phenology(bbox, Path(mdir) / "real", quick=True, modis=want == "modis")
    state, progress = leaf_state(when.date(), ph)
    info.update(date=when.date().isoformat(), state=state, progress=progress, phenology=ph)
    log(f"leaves on {when:%Y-%m-%d}: {state}" + (f" ({progress:.0%} through the colouring)" if state == "autumn"
                                                   else "") + (f", dates from {ph['source']}" if ph else
                                                               " (tropics: always in leaf)"))
    return state, progress, info


def choose(item_key, alt, state, progress, bare_mode):
    """For one deciduous tree: None (as placed), "drop", or ("autumn"|"bare", mesh, scale)."""
    if state == "autumn" and alt.get("autumn"):
        from . import scenery
        share = AUTUMN_PEAK * min(1.0, max(0.0, progress))
        if (scenery._h(item_key, "autumn") % 1000) / 1000.0 < share:
            return "autumn"
        return None
    if state == "bare":
        if bare_mode == "drop":
            return "drop"
        if bare_mode == "small":
            return "bare"
    return None


def apply(items, leaf, season, log=log):
    """scenery.json's items with the deciduous trees as the season wants them (a copy; None: left out)."""
    if not leaf:
        log("scenery.json has no deciduous-tree stand-ins (built before season.py): rebuild it with "
            "mapbuild --scenery for the season's trees")
        return items
    state, progress, mode = season["state"], season.get("progress", 0.0), season.get("bare", "small")
    out = list(items)
    n = defaultdict(int)
    for row in leaf:
        i = row[0]
        it = items[i]
        pick = choose((round(it[1]), round(it[2])), {"autumn": row[1] >= 0}, state, progress, mode)
        if pick == "drop":
            out[i] = None
        elif pick in ("autumn", "bare"):
            idx, s, dx, dy, dz = row[1:6] if pick == "autumn" else row[6:11]
            out[i] = [idx, round(it[1] + dx, 2), round(it[2] + dy, 2), round(it[3] + dz, 2), it[4], s] + it[6:]
        n[pick or "kept"] += 1
    log(f"season {state}: {len(leaf)} deciduous trees: " + ", ".join(f"{k} {v}" for k, v in sorted(n.items())))
    return out


def for_trial(cfg, mdir, world, log=log):
    """What Scenery needs for a trial's season: leaf state, the bare mode and (CARLA 1.0 at night) the lamp
    blueprints that carry a light. Returns (season dict, info for season.json)."""
    sc = cfg["simulation"].get("scenery") or {}
    state, progress, info = trial_leaves(cfg, mdir, log)
    season = {"state": state, "progress": progress, "bare": sc.get("bare", "small"),
              "lights_radius_m": float(sc.get("lights_radius_m", 200.0))}
    sun = world.get_weather().sun_altitude_angle
    want = sc.get("lamp_lights", "auto")
    if want and (want is True or sun <= NIGHT_SUN):
        season["lights"] = light_blueprints(world, log)
    info.update(bare=season["bare"], sun_altitude=round(sun, 2), lamp_lights=bool(season.get("lights")),
                lights_radius_m=season["lights_radius_m"])
    return season, info


def light_blueprints(world, log=log):
    """Lamp mesh name -> light blueprint id for the blueprints this server has (CARLA 1.0 with the content pack)."""
    lib = world.get_blueprint_library()
    have = {b.id for b in lib.filter("blueprint.cc_*")}
    out = {m: b for m, b in LAMP_LIGHTS.items() if b in have}
    if not out:
        log("lamps without light: start the CARLA 1.0 server with -carla-packs=" + str(PACKS)
            + " (coupled-carla does when it starts the server) or set CARLA_PACKS")
    return out


def pack_args(exe_name):
    """Server command-line arguments for the content pack (CARLA 1.0 only; 0.9.x has no content packs)."""
    return [f"-carla-packs={PACKS}"] if "CarlaUnreal" in str(exe_name) and PACKS.is_dir() else []


# --- tools ----------------------------------------------------------------------------------------------------

def main():
    import argparse
    ap = argparse.ArgumentParser(description="Leaf dates and the leaf state for an experiment's place.")
    ap.add_argument("experiment")
    ap.add_argument("--date", help="YYYY-MM-DD (default: the experiment's simulation.time)")
    ap.add_argument("--modis", action="store_true", help="MODIS MCD12Q2 dates (where plausible)")
    a = ap.parse_args()
    from . import config
    cfg = config.load(a.experiment)
    mdir = config.map_dir(cfg)
    ph = phenology(cfg["map"]["bbox"], mdir / "real", modis=a.modis)
    print(json.dumps(ph, indent=1))
    if a.date:
        print(a.date, leaf_state(date.fromisoformat(a.date), ph))
    else:
        print(trial_leaves(cfg, mdir)[:2])


if __name__ == "__main__":
    main()
