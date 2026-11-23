"""A CARLA 1.0 digital twin of any place from the map maker: python -m coupledcarla.mapmaker "<place>" --twin

The map maker builds an OSM map that CARLA 0.9.16 generates on the fly. A digital twin goes further (real building
shapes with facade styles and photo roofs, the aerial photo on the ground, the national terrain model, CARLA's road
materials with lane markings, lit windows at night) and is a level baked into a CARLA 1.0 (Unreal Engine 5.8)
source tree by carla-digitaltwins' TwinModel pipeline (``python -m twinmodel twin``), with the best open data of the
place's country (twinmodel.regions: PDOK/AHN/3DBAG in the Netherlands, PNOA/MDT/Catastro in Spain, BD ORTHO/RGE
ALTI/BD TOPO in France, NAIP/3DEP in the US, ...; Overture Maps and the AWS Terrain Tiles elsewhere).

That needs a toolchain this module does not install, named in a small config file (``~/.config/coupled-carla/
twin.json``, or the file in COUPLED_CARLA_TWIN_CONFIG):

    {"digitaltwins": "~/carla-digitaltwins",          # the carla-digitaltwins checkout (TwinModel/ inside)
     "python": "~/carla-digitaltwins/.venv/bin/python",  # a Python with twinmodel installed (pip install -e TwinModel)
     "carla_root": "~/carla58/carla",                 # CARLA ue58-dev source tree, built with the editor
     "engine": "~/carla58/UnrealEngine5_carla",       # its Unreal Engine 5.8
     "carla_python": "~/carla58/venv/bin/python",     # a Python with the CARLA 1.0 wheel (Traffic Manager cache)
     "builds": "~/twins",                             # where the twins are built (default <digitaltwins>/TwinModel/build)
     "no_nav_build": false, "max_rss_gb": 24, "min_avail_gb": 8}

Without ``carla_root``/``engine`` the twin stops before the bake (its manifest can be baked later). make_twin runs
the pipeline with plain-language progress, copies the twin's OpenDRIVE to twins/<Name>.xodr and writes an
experiment like experiments/eindhoven_twin.yaml (map.opendrive, map.carla_level) as experiments/places/
<name>_twin.yaml; the map maker then builds its SUMO net from that OpenDRIVE.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
PLACES = ROOT / "experiments" / "places"
TWINS = ROOT / "twins"
CONFIG = Path.home() / ".config" / "coupled-carla" / "twin.json"
# twinmodel twin's steps, in the map maker's words (its "[twin] <kind> <step>: <text>" lines carry these keys)
STEPS = [
    ("twin_build", "Building roads, terrain and aerial photo from the country's open data"),
    ("twin_export", "Cutting the meshes for Unreal"),
    ("twin_buildings", "Putting in the real buildings with their facades"),
    ("twin_ground", "Laying the aerial photo on the ground"),
    ("twin_clearance", "Keeping the roads and sidewalks clear of buildings"),
    ("twin_detail", "Adding entrances, steps and open ground floors"),
    ("twin_gaps", "Closing gaps between road and pavement surfaces"),
    ("twin_paving", "Paving brick and cobblestone streets in the road surface"),
    ("twin_credits", "Writing the data credits"),
    ("twin_bake", "Baking the CARLA 1.0 level (Unreal editor; the longest step)"),
    ("twin_nav", "Building the pedestrian navigation"),
]
TITLES = dict(STEPS)
LINE = re.compile(r"^\[twin\] (start|done|note|skip|failed|advice)(?: (\w+))?:\s*(.*)$")
TWIN_MINUTES = 35        # a 1 km twin on lin5080: build ~10 min, buildings and photo ~5, bake ~10-20


class TwinError(Exception):
    def __init__(self, message, advice=""):
        super().__init__(message)
        self.advice = advice


def config_path():
    return Path(os.environ.get("COUPLED_CARLA_TWIN_CONFIG") or CONFIG).expanduser()


def load_config():
    """The toolchain config (paths expanded), or None when there is none."""
    p = config_path()
    try:
        cfg = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(cfg, dict):
        return None
    for k in ("digitaltwins", "python", "carla_root", "engine", "carla_python", "builds", "cache"):
        if cfg.get(k):
            cfg[k] = str(Path(str(cfg[k])).expanduser())
    return cfg


def status(cfg=None):
    """Whether twins can be made here: {configured, ok, bake, why, how}. The page shows its checkbox only when
    configured."""
    cfg = load_config() if cfg is None else cfg
    if not cfg:
        return {"configured": False, "ok": False, "bake": False, "why": "No digital-twin toolchain is set up.",
                "how": f"See docs/make_a_map.md, \"A digital twin (CARLA 1.0)\": it needs carla-digitaltwins and a "
                       f"CARLA 1.0 source build with the editor, named in {config_path()}."}
    dt = Path(cfg.get("digitaltwins") or "")
    if not (dt / "TwinModel" / "twinmodel" / "twin.py").exists():
        return {"configured": True, "ok": False, "bake": False,
                "why": f"carla-digitaltwins with the twin command is not at {dt}.",
                "how": "Set \"digitaltwins\" in the config to a checkout of carla-digitaltwins (branch with "
                       "TwinModel/twinmodel/twin.py)."}
    py = cfg.get("python") or sys.executable
    if not Path(py).exists() and not shutil.which(py):
        return {"configured": True, "ok": False, "bake": False, "why": f"The Python {py} does not exist.",
                "how": "Set \"python\" to an interpreter with twinmodel installed (pip install -e TwinModel)."}
    bake = bool(cfg.get("carla_root") and cfg.get("engine") and
                (Path(cfg["carla_root"]) / "Unreal" / "CarlaUnreal" / "CarlaUnreal.uproject").exists())
    return {"configured": True, "ok": True, "bake": bake,
            "why": "ready" if bake else "ready, without the bake (no CARLA 1.0 editor configured)",
            "how": "" if bake else "Set carla_root and engine to a built CARLA ue58-dev tree to bake the level."}


def level_name(slug):
    """'plaza_mayor_salamanca' -> 'PlazaMayorSalamanca' (a valid Unreal level name, at most 32 characters)."""
    name = "".join(w[:1].upper() + w[1:] for w in re.split(r"[^A-Za-z0-9]+", slug) if w)[:32]
    return name if name and name[0].isalpha() else ("Twin" + name)[:32]


def build_dir(cfg, name):
    base = Path(cfg.get("builds") or Path(cfg["digitaltwins"]) / "TwinModel" / "build")
    return base / re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def command(cfg, p, name, force=False):
    """The twinmodel twin command for a map maker plan (its box, not the place text: the square is the plan's)."""
    py = cfg.get("python") or sys.executable
    cmd = [py, "-u", "-m", "twinmodel", "twin", "--bbox", *map(str, p["bbox"]), "--name", name,
           "--out", str(build_dir(cfg, name)),
           "--cache", str(cfg.get("cache") or Path(cfg["digitaltwins"]) / "TwinModel" / "data")]
    st = status(cfg)
    if st["bake"]:
        cmd += ["--bake", "--carla-root", cfg["carla_root"], "--engine", cfg["engine"]]
        if cfg.get("carla_python"):
            cmd += ["--carla-python", cfg["carla_python"]]
        for key, flag in (("max_rss_gb", "--max-rss-gb"), ("min_avail_gb", "--min-avail-gb"),
                          ("min_start_gb", "--min-start-gb")):
            if cfg.get(key) is not None:
                cmd += [flag, str(cfg[key])]
        if cfg.get("no_nav_build"):
            cmd.append("--no-nav-build")
    if force:
        cmd.append("--force")
    return cmd


def parse_line(line):
    """('start' | 'done' | 'note' | 'skip' | 'failed' | 'advice', step key or None, text) of a twinmodel twin
    progress line, else None."""
    m = LINE.match(line.strip())
    if not m:
        return None
    return m.group(1), m.group(2), m.group(3).strip()


def twin_experiment(p, name, summ):
    """The experiment of a baked twin, in the shape of experiments/eindhoven_twin.yaml."""
    exp = {
        "name": f"{p['slug']}_twin",
        "city": {"name": p["label"], "country": p["country"] or "unknown", "continent": p["continent"],
                 "size": p["town"], "population": p["population"] or 0, "lat": round(p["lat"], 5),
                 "lon": round(p["lon"], 5), "drive": p["drive"], "country_code": p["country_code"]},
        "map": {"opendrive": f"twins/{name}.xodr", "carla_level": name, "bbox": p["bbox"],
                "lefthand": p["drive"] == "left"},
        "server": {"quality": "Epic"},
        "simulation": {"step": 0.02, "weather": "CloudyNoon"},
        "traffic": dict(p["traffic"]),
        "participants": [{"role": "driver", "vehicle": "vehicle.lincoln.mkz_2020", "input": "keyboard",
                          "view": "driver"}],
        "trials": [{"duration_s": 300}],
    }
    if p.get("pedestrians"):
        exp["traffic"]["walkers"] = {"count": "auto"}
    return exp


def real_experiment(p, name, summ):
    """The "as real as it gets" experiment of a twin (mapmaker --realistic): the same level and SUMO net, Google's
    Photorealistic 3D Tiles streamed over it as the look (the twin keeps the roads, collision, navigation and the lit
    buildings at night), and every live data stream switched on: the sun and weather of now, traffic fitted to open
    counts, the real buses where they are now, cyclists from counters, the national car fleet, a calibrated crowd."""
    exp = twin_experiment(p, name, summ)
    exp["name"] = f"{p['slug']}_real"
    exp["map"]["name"] = f"{p['slug']}_twin"          # the twin's map folder (SUMO net, scenery, parked cars)
    exp["map"]["photoreal"] = {"provider": "google"}
    exp["map"]["scenery"] = {"trees": False, "buildings": False}    # the photo mesh has the real ones
    exp["simulation"] = {"step": 0.02, "time": "now", "weather": "real"}
    traffic = dict(exp["traffic"])
    traffic.update({"cars_per_hour": "auto", "counts": "auto", "walkers": {"count": "auto"}})
    exp["traffic"] = traffic
    return exp


def real_experiment_text(p, name, summ):
    head = [
        f"# {p['label']}{', ' + p['country'] if p['country'] else ''}, as real as Coupled CARLA gets: the twin "
        f"{name} with Google's Photorealistic 3D Tiles over it",
        f"# (made by coupledcarla.mapmaker --realistic on {time.strftime('%Y-%m-%d')}). The photo mesh is what you see; "
        "the twin keeps the roads,",
        "# collision, LiDAR, navigation and, at night, its buildings with lit windows. Live data: the sun and weather "
        "of now (Open-Meteo),",
        "# traffic fitted to open counts, the real buses where they are now (GTFS-Realtime), cyclists from counters, "
        "the country's car fleet,",
        "# a crowd calibrated with CROWD. Needs YOUR Google Maps Platform key with billing on the server machine "
        "(README, \"Photorealistic",
        "# tiles\"; each level load is one billed root request) and the level data in the server's content pack. "
        "Without a key it runs on the twin.",
        f"# Drive it:  python -m coupledcarla.run experiments/places/{p['slug']}_real.yaml",
    ]
    return "\n".join(head) + "\n" + yaml.safe_dump(real_experiment(p, name, summ), sort_keys=False,
                                                   allow_unicode=True)


def prepare_photoreal(exp_path, out, emit=None):
    """The level data for the tiles (origin, road cut, footprints) into the repo's content pack; None or the file.
    Sends nothing to Google (no key needed). A failure is a note, not the end of the twin."""
    from . import config, photoreal
    try:
        cfg = config.load(exp_path)
        manifest = Path(out) / "ue" / "manifest.json"
        return photoreal.prepare(cfg, footprints=str(manifest) if manifest.exists() else None)
    except Exception as err:          # noqa: BLE001 - the twin is still usable without the tiles
        if emit:
            emit({"step": "place", "title": "", "state": "note", "at": 0.0,
                  "text": f"The data for the photorealistic tiles could not be written ({err}); the twin works without."})
        return None


def experiment_text(p, name, summ):
    blds = (summ or {}).get("buildings")
    src = (summ or {}).get("building_source")
    head = [
        f"# {p['label']}{', ' + p['country'] if p['country'] else ''} as a baked CARLA 1.0 level: the "
        f"carla-digitaltwins twin {name}",
        f"# (a {p['km']:g} km square around {p['lat']:.5f}, {p['lon']:.5f}; made by coupledcarla.mapmaker --twin on "
        f"{time.strftime('%Y-%m-%d')}).",
        f"# Roads from OpenStreetMap with lane markings, kerbs and sidewalks; terrain {summ.get('elevation') or '-'}; "
        f"aerial photo {summ.get('imagery') or 'none'};",
        f"# {blds if blds is not None else 'no'} real buildings from {src or 'OSM'} with facade styles. Credits: "
        f"ATTRIBUTION.md next to the level.",
        "# Needs a CARLA 1.0 server that has the level (baked into its source tree, or a cooked package or content "
        "pack holding it; see README, \"A twin of any place\").",
        f"# Drive it:  python -m coupledcarla.run experiments/places/{p['slug']}_twin.yaml",
    ]
    return "\n".join(head) + "\n" + yaml.safe_dump(twin_experiment(p, name, summ), sort_keys=False,
                                                   allow_unicode=True)


def make_twin(p, emit, force=False, cfg=None, runner=None):
    """Make the twin of a map maker plan. Returns the experiment file (experiments/places/<slug>_twin.yaml).
    emit gets {step, title, state, text, at} as the map maker's steps; raises TwinError with advice."""
    cfg = load_config() if cfg is None else cfg
    st = status(cfg)
    if not st["ok"]:
        raise TwinError(st["why"], st["how"])
    name = level_name(p["slug"])
    out = build_dir(cfg, name)
    out.mkdir(parents=True, exist_ok=True)
    cmd = command(cfg, p, name, force=force)
    t0 = time.time()
    log_path = out / "mapmaker_twin.log"
    state = {k: "pending" for k, _ in STEPS}
    failure = {"msg": None, "advice": ""}

    def ev(step, s, text=""):
        state[step] = s
        emit({"step": step, "title": TITLES.get(step, step), "state": s, "text": text,
              "at": round(time.time() - t0, 1)})

    def on_line(line):
        got = parse_line(line)
        if not got:
            return
        kind, key, text = got
        step = f"twin_{key}" if key else None
        if kind == "advice":
            failure["advice"] = text
            return
        if step not in TITLES:
            if kind == "failed":
                failure["msg"] = text
            elif kind == "note":
                emit({"step": "place", "title": "", "state": "note", "text": text, "at": 0.0})
            return
        if kind == "start":
            ev(step, "running", "")
        elif kind == "done":
            ev(step, "done", text)
        elif kind == "skip":
            ev(step, "skipped" if "done before" not in text else "done", text)
        elif kind == "note":
            ev(step, "running" if state[step] == "running" else state[step], text)
        elif kind == "failed":
            failure["msg"] = text
            ev(step, "failed", text)

    rc = (runner or _run)(cmd, cfg, log_path, on_line)
    if rc != 0:
        cur = next((k for k, _ in STEPS if state[k] == "running"), None)
        msg = failure["msg"] or "The digital twin could not be made."
        if cur:
            ev(cur, "failed", msg)
        raise TwinError(msg, (failure["advice"] + " " if failure["advice"] else "") + f"The log is {log_path}.")
    summ = {}
    try:
        summ = json.loads((out / "twin.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        pass
    xodr = Path(summ.get("xodr") or out / f"{name}.xodr")
    if not xodr.exists():
        raise TwinError("The twin has no OpenDRIVE.", f"The log is {log_path}.")
    TWINS.mkdir(exist_ok=True)
    shutil.copyfile(xodr, TWINS / f"{name}.xodr")
    for k, _ in STEPS:
        if state[k] in ("pending", "running"):
            ev(k, "done" if state[k] == "running" else "skipped", "done" if state[k] == "running" else "")
    path = PLACES / f"{p['slug']}_twin.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(experiment_text(p, name, summ), encoding="utf-8")
    info = {"experiment": str(path), "level": name}
    if p.get("realistic"):
        real = PLACES / f"{p['slug']}_real.yaml"
        real.write_text(real_experiment_text(p, name, summ), encoding="utf-8")
        info["realistic"] = str(real)
        data = prepare_photoreal(real, out, emit)
        if data:
            info["photoreal_data"] = str(data)
            emit({"step": "place", "title": "", "state": "note", "at": round(time.time() - t0, 1),
                  "text": f"Photorealistic tiles: the level data is in {data}; with your Google key on the server, "
                          f"experiments/places/{real.name} streams them over the twin."})
    (out / "mapmaker.json").write_text(json.dumps(info, indent=1))
    return path


def twin_env():
    """The environment for the twin pipeline: ours without PROJ_LIB/PROJ_DATA/GDAL_DATA, which point at SUMO's own
    (older) PROJ database for the map builder and break the twin's pyproj and rasterio."""
    env = {k: v for k, v in os.environ.items() if k not in ("PROJ_LIB", "PROJ_DATA", "GDAL_DATA")}
    env["PYTHONUNBUFFERED"] = "1"
    return env


def _run(cmd, cfg, log_path, on_line):
    """Run twinmodel twin from the TwinModel folder, its output to the log and each line to on_line."""
    cwd = Path(cfg["digitaltwins"]) / "TwinModel"
    with open(log_path, "a", encoding="utf-8") as logf:
        logf.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        try:
            proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                    encoding="utf-8", errors="replace", bufsize=1, env=twin_env())
        except OSError as err:
            raise TwinError("The twin pipeline could not start.", f"Check \"python\" in {config_path()} ({err}).")
        for line in proc.stdout:
            logf.write(line)
            logf.flush()
            on_line(line)
        return proc.wait()


def summary_lines(exp_cfg, mdir):
    """Lines for the map maker's summary of a twin map (its level, buildings and where the credits are)."""
    mc = (exp_cfg or {}).get("map") or {}
    if not mc.get("carla_level"):
        return []
    cfg = load_config() or {}
    out = build_dir(cfg, mc["carla_level"]) if cfg.get("digitaltwins") or cfg.get("builds") else None
    summ = {}
    if out is not None:
        try:
            summ = json.loads((out / "twin.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            pass
    lines = [f"  Digital twin: CARLA 1.0 level {mc['carla_level']}"
             + (f" ({summ['level']})" if summ.get("level") else "")]
    if summ.get("buildings") is not None:
        lines.append(f"    {summ['buildings']:,} real buildings from {summ.get('building_source')}; terrain "
                     f"{summ.get('elevation') or '-'}; aerial photo {summ.get('imagery') or 'none'}")
    if out is not None and (out / "ATTRIBUTION.md").exists():
        lines.append(f"    Data credits: {out / 'ATTRIBUTION.md'}")
    if summ and not (summ.get("bake") or {}).get("level_saved"):
        lines.append("    Not baked yet: bake it on a machine with the CARLA 1.0 editor (python -m twinmodel twin "
                     "... --bake, or ue/bake_level.py)")
    return lines
