"""Check an experiment file before running it, and print each participant's trial plan.

    python -m coupledcarla.validate experiments/eindhoven_crossing.yaml
    python -m coupledcarla.validate exp.yaml --participants 1-12 [--order full] [--no-roads] [--quiet]

Checks (errors stop a run going wrong; warnings are worth a look):
  keys       unknown keys anywhere (with the nearest known one), against config.DEFAULTS and the keys participants,
             agents, triggers, bots, trials, conditions, responses and automation take
  map        the map folder is built (else the run builds it, which needs map.bbox and the network); map.opendrive
             exists
  spawns     every spawn, route, path, trigger and POI point inside the map's box; with a built map (and sumolib)
             their distance to the nearest lane (a car spawns on the nearest driving lane, a walker where it is);
             two vehicles starting within 4 m of each other (the second spawn fails)
  roles      unique roles and agent ids, known kinds and inputs, an av_passenger's agent is a car, patches name
             existing agents and roles
  eHMI       every eHMI state used (agents' triggers, auto mode, AV-passenger bot schedules) is defined
  durations  positive trial durations; visual syncs, take-over requests and bot schedules within their trial
  design     conditions used are defined, factors and block_by, the counterbalancing order
  multi-GPU  server.multigpu: secondary names, roles, port clashes, launch only on this machine (multigpu.py)
  photoreal  map.photoreal: its options and caps, a baked level and its OpenDRIVE (photoreal.py)

Exit code 1 when there are errors.
"""
import argparse
import ast
import difflib
import math
import re
import sys
from pathlib import Path

import yaml

from . import config, drones, multigpu, photoreal, responses, study, takeover

OVERRIDES = ("simulation", "traffic", "participants", "agents", "logging", "coupling", "ehmi", "eye_contact",
             "sound", "displays", "haptics", "drones")
TRIAL_KEYS = set(OVERRIDES) | {"duration_s", "name", "block", "condition", "conditions", "patch"}
TOP_EXTRA = {"city", "pois", "eye_contact"}
PART_KEYS = {"role", "kind", "station", "vehicle", "spawn", "input", "view", "window", "bot", "agent", "blueprint",
             "seed", "color", "response", "automation", "sound", "displays", "haptics", "myosuite",
             "route", "destination", "autopilot", "fov", "screens"} | drones.PILOT_KEYS   # SUMO driver; cockpit view
KINDS = {"driver", "pedestrian", "av_passenger", "drone"}
INPUTS = {"keyboard", "wheel", "autopilot", "bot", "gamepad", "rc", "myosuite"}
SPAWN_KEYS = {"lat", "lon", "heading", "x", "y"}      # x, y: CARLA coordinates (towns.resolve_xy)
BOT_KEYS = {"path", "start_s", "speed", "look", "ehmi", "response", "sticks", "land", "absolute", "after_crash"}
AGENT_KEYS = {"id", "kind", "vehicle", "color", "route", "speed_kmh", "accel", "yield_to_pedestrians", "ehmi",
              "ehmi_slots", "triggers", "at_end", "path", "speed", "start_s", "loop", "ehmi_design", "ehmi_placement",
              "ehmi_offset"}
TRIGGER_KEYS = {"at", "at_m", "speed_kmh", "accel", "yield_s", "blinker", "ehmi", "takeover"}
RESPONSE_KEYS = {"button", "slider", "slider_start", "slider_speed", "hidden", "press_threshold", "joystick"}
EHMI_KEYS = {"states", "slots", "design", "placement", "render", "offset", "inside", "indicator"}
POI_KEYS = {"name", "lat", "lon", "heading", "radius_m", "radius", "x", "y"}
FREE = {("traffic", "car_types"), ("traffic", "bike_types"), ("conditions",), ("design", "factors"), ("drones", "centre"),
        ("design", "trial"), ("design", "practice"), ("design", "after"), ("ehmi",), ("eye_contact",),
        ("film", "camera"), ("film", "cameras"), ("film", "hold"), ("film", "attributes")}
VEHICLE_GAP_M = 4.0
TRIAL_PREFIX = re.compile(r"^trial \d+( \([^)]*\))? ?")


def default_ehmi():
    """ai.DEFAULT_EHMI's state names, read from the source (importing ai needs CARLA)."""
    src = (Path(__file__).parent / "ai.py").read_text(encoding="utf-8")
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "DEFAULT_EHMI" for t in node.targets):
            return ast.literal_eval(node.value)
    return {"off": {}, "driving": {}, "yielding": {}, "go": {}}


class Report:
    def __init__(self):
        self.items = []

    def error(self, where, msg):
        self.items.append(("error", where, msg))

    def warn(self, where, msg):
        self.items.append(("warning", where, msg))

    @property
    def errors(self):
        return [i for i in self.items if i[0] == "error"]


def _unknown(rep, where, got, known):
    for k in got:
        if k not in known:
            near = difflib.get_close_matches(str(k), [str(x) for x in known], n=1)
            rep.error(where, f"unknown key {k!r}" + (f" (did you mean {near[0]!r}?)" if near else ""))


def _keys(rep, raw, defaults, path=()):
    """Unknown keys of `raw` against the defaults dict, recursively."""
    if not isinstance(raw, dict):
        return
    where = ".".join(path) or "top level"
    known = set(defaults) | (TOP_EXTRA if not path else set())
    _unknown(rep, where, raw, known)
    for k, v in raw.items():
        sub = path + (k,)
        if sub in FREE or k not in defaults:
            continue
        if isinstance(defaults[k], dict) and isinstance(v, dict):
            _keys(rep, v, defaults[k], sub)


def _override_keys(rep, where, over):
    """The sections of a trial or condition, against the defaults they override."""
    for sec in OVERRIDES:
        v = over.get(sec)
        if sec == "ehmi" and isinstance(v, dict):
            _unknown(rep, f"{where}.ehmi", v, EHMI_KEYS)
        elif isinstance(v, dict) and config.DEFAULTS.get(sec) and isinstance(config.DEFAULTS[sec], dict):
            _keys(rep, v, config.DEFAULTS[sec], (where, sec))


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _metres(a, b):
    lat = math.radians((a[0] + b[0]) / 2)
    return 6371000.0 * math.hypot(math.radians(b[0] - a[0]), math.radians(b[1] - a[1]) * math.cos(lat))


class Roads:
    """Distance from a lat/lon to the nearest lane of the map's geo.net.xml (sumolib), or None per point."""

    def __init__(self, net_path):
        import sumolib
        self.net = sumolib.net.readNet(str(net_path), withInternal=False, withPedestrianConnections=False)

    def distance(self, lat, lon, kind="car"):
        x, y = self.net.convertLonLat2XY(lon, lat)
        best = None
        for lane, d in self.net.getNeighboringLanes(x, y, 200.0):
            if kind == "car" and not lane.allows("passenger"):
                continue
            best = d if best is None else min(best, d)
        return best


def _points(rep, where, pts, n_min=1):
    """A list of [lat, lon] (or None with an error)."""
    if not isinstance(pts, list) or len(pts) < n_min or not all(
            isinstance(p, (list, tuple)) and len(p) >= 2 and _num(p[0]) and _num(p[1]) for p in pts):
        rep.error(where, f"needs a list of at least {n_min} [lat, lon] point{'s' if n_min > 1 else ''}")
        return []
    return [(float(p[0]), float(p[1])) for p in pts]


def _schedule(rep, where, sched, duration, states=None):
    if not isinstance(sched, list):
        rep.error(where, "needs a list of [time, ...] entries")
        return
    for e in sched:
        if not isinstance(e, (list, tuple)) or not e or not _num(e[0]):
            rep.error(where, f"entry {e!r} is not [time, ...]")
            continue
        if duration is not None and e[0] >= duration:
            rep.warn(where, f"at {e[0]} s, after the trial's end ({duration:g} s)")
        if states is not None and len(e) > 1 and e[1] not in states:
            rep.error(where, f"eHMI state {e[1]!r} is not defined (defined: {', '.join(sorted(states))})")


def check_trial_config(rep, tc, where, duration, geo_checks):
    """Participants, agents, eHMI and schedules of one trial's config."""
    states = set(((tc.get("ehmi") or {}).get("states") or default_ehmi()).keys()) | {"off"}
    agents = tc.get("agents") or []
    ids = [a.get("id") for a in agents if isinstance(a, dict)]
    for a in agents:
        if not isinstance(a, dict):
            rep.error(f"{where} agents", f"{a!r} is not a mapping")
            continue
        aw = f"{where} agent {a.get('id')}"
        _unknown(rep, aw, a, AGENT_KEYS)
        if not a.get("id"):
            rep.error(aw, "needs an id")
        design = a.get("ehmi_design", (tc.get("ehmi") or {}).get("design"))
        if design and a.get("kind", "car") == "car":       # a designed eHMI (ehmi_design.py): the file must exist
            from . import designs
            try:
                designs.load(design, "ehmi", Path(tc["_path"]).parent if tc.get("_path") else None)
            except (OSError, ValueError) as e:
                rep.error(aw, f"eHMI design: {e}")
        kind = a.get("kind", "car")
        if kind not in ("car", "walker"):
            rep.error(aw, f"kind {kind!r} is neither car nor walker")
        if kind == "car":
            geo_checks.append((f"{aw} route", _points(rep, f"{aw} route", a.get("route"), 2), "car",
                               a.get("id")))
            mode = a.get("ehmi", "off")
            if mode not in ("off", "auto", "manual"):
                rep.error(aw, f"ehmi {mode!r} is not off, auto or manual")
            if mode == "auto":
                for s in ("yielding", "driving"):
                    if s not in states:
                        rep.error(aw, f"ehmi auto shows {s!r}, which ehmi.states does not define")
            for trg in a.get("triggers") or []:
                if not isinstance(trg, dict):
                    rep.error(f"{aw} trigger", f"{trg!r} is not a mapping")
                    continue
                _unknown(rep, f"{aw} trigger", trg, TRIGGER_KEYS)
                if "at" not in trg and "at_m" not in trg:
                    rep.warn(f"{aw} trigger", "has neither at nor at_m: fires at the start")
                if "at" in trg:
                    geo_checks.append((f"{aw} trigger at", _points(rep, f"{aw} trigger at", [trg["at"]]), "any", None))
                if "ehmi" in trg and trg["ehmi"] not in states:
                    rep.error(f"{aw} trigger", f"eHMI state {trg['ehmi']!r} is not defined "
                                               f"(defined: {', '.join(sorted(states))})")
        else:
            geo_checks.append((f"{aw} path", _points(rep, f"{aw} path", a.get("path"), 1), "walk", None))
            if _num(a.get("start_s")) and duration and a["start_s"] >= duration:
                rep.warn(aw, f"starts at {a['start_s']} s, after the trial's end")
    if len(set(ids)) != len(ids):
        rep.error(f"{where} agents", "agent ids are not unique")
    parts = tc.get("participants") or []
    roles = [p.get("role") for p in parts if isinstance(p, dict)]
    if len(set(roles)) != len(roles):
        rep.error(f"{where} participants", "roles are not unique")
    for p in parts:
        if not isinstance(p, dict):
            rep.error(f"{where} participants", f"{p!r} is not a mapping")
            continue
        pw = f"{where} participant {p.get('role')}"
        _unknown(rep, pw, p, PART_KEYS)
        kind = p.get("kind", "driver")
        if not p.get("role"):
            rep.error(pw, "needs a role")
        if kind not in KINDS:
            rep.error(pw, f"kind {kind!r} is not one of {', '.join(sorted(KINDS))}")
        if p.get("input") is not None and p["input"] not in INPUTS:
            rep.error(pw, f"input {p['input']!r} is not one of {', '.join(sorted(INPUTS))}")
        sp = p.get("spawn")
        if sp is not None:
            if not isinstance(sp, dict):
                rep.error(pw, "spawn needs {lat, lon, heading}")
            else:
                _unknown(rep, f"{pw} spawn", sp, SPAWN_KEYS)
                if ("lat" in sp) != ("lon" in sp):
                    rep.error(f"{pw} spawn", "needs both lat and lon")
                elif "lat" in sp:
                    geo_checks.append((f"{pw} spawn", _points(rep, f"{pw} spawn", [[sp["lat"], sp["lon"]]]),
                                       "car" if kind == "driver" else "any" if kind == "drone" else "walk",
                                       p.get("role") if kind == "driver"
                                       else None))
        if kind == "pedestrian" and not (isinstance(sp, dict) and ("lat" in sp or "x" in sp)):
            rep.error(pw, "a pedestrian needs spawn: {lat, lon}")
        if kind == "av_passenger":
            ag = next((a for a in agents if isinstance(a, dict) and a.get("id") == p.get("agent")), None)
            if ag is None:
                rep.error(pw, f"agent {p.get('agent')!r} is not one of the agents ({', '.join(map(str, ids)) or 'none'})")
            elif ag.get("kind", "car") != "car":
                rep.error(pw, f"agent {ag['id']!r} is a walker, not a car")
        bot = p.get("bot")
        if bot is not None:
            if not isinstance(bot, dict):
                rep.error(f"{pw} bot", "needs a mapping")
            else:
                _unknown(rep, f"{pw} bot", bot, BOT_KEYS)
                if "path" in bot and kind != "drone":       # a drone's: drones.check_config
                    geo_checks.append((f"{pw} bot path", _points(rep, f"{pw} bot path", bot["path"]), "walk", None))
                if "ehmi" in bot:
                    _schedule(rep, f"{pw} bot ehmi", bot["ehmi"], duration, states)
                if "response" in bot:
                    _schedule(rep, f"{pw} bot response", bot["response"], duration)
                if bot.get("look") and bot["look"] not in ids + roles:
                    rep.warn(f"{pw} bot", f"look {bot['look']!r} is neither an agent nor a participant")
        if p.get("response") not in (None, False):
            r = p["response"]
            if isinstance(r, dict):
                _unknown(rep, f"{pw} response", r, RESPONSE_KEYS)
                s0 = r.get("slider_start", 50)
                if not _num(s0) or not 0 <= s0 <= 100:
                    rep.error(f"{pw} response", "slider_start must be within 0-100")
            elif isinstance(r, str):
                bad = {x.strip() for x in r.split(",")} - {"button", "slider", "hidden"}
                if bad:
                    rep.error(f"{pw} response", f"unknown {', '.join(sorted(bad))} (button, slider, hidden)")
            elif r is not True:
                rep.error(f"{pw} response", "needs true, 'button,slider' or a mapping")
            if responses.spec(r) is None and r not in (None, False):
                rep.warn(f"{pw} response", "asks for neither a button nor a slider")
        if p.get("automation"):
            check_automation(rep, pw, p, duration, geo_checks)
        if p.get("myosuite") is not None or p.get("input") == "myosuite":
            from . import myodriver      # imports no MuJoCo: options only
            if kind != "driver":
                rep.error(pw, "input myosuite drives a car: kind driver")
            if not isinstance(p.get("myosuite") or {}, dict):
                rep.error(f"{pw} myosuite", "needs a mapping")
            else:
                _unknown(rep, f"{pw} myosuite", p.get("myosuite") or {}, myodriver.DEFAULTS)
                if (p.get("myosuite") or {}).get("preset", "fast") not in ("fast", "full"):
                    rep.error(f"{pw} myosuite", "preset is fast or full")
    # drones: missions, observer cameras, drone participants (drones.check_config)
    drones.check_config(tc, where, duration, rep.error, rep.warn,
                        lambda w, pts, kind: geo_checks.append((w, _points(rep, w, pts), kind, None)))


def check_automation(rep, pw, p, duration, geo_checks):
    a = p["automation"]
    if not isinstance(a, dict):
        rep.error(f"{pw} automation", "needs a mapping")
        return
    if p.get("kind", "driver") != "driver":
        rep.error(f"{pw} automation", "only a driver can ride in an automated car")
    _unknown(rep, f"{pw} automation", a, takeover.DEFAULTS)
    _unknown(rep, f"{pw} automation takeover", a.get("takeover") or {}, takeover.DEFAULTS["takeover"])
    geo_checks.append((f"{pw} automation route", _points(rep, f"{pw} automation route", a.get("route"), 2), "car",
                       p.get("role")))
    if p.get("spawn"):
        rep.warn(f"{pw} automation", "spawn is ignored: the automated car starts at its route's first point")
    tk = takeover.spec(a)["takeover"]
    if str(tk.get("no_reaction")).lower() not in ("brake", "keep"):
        rep.error(f"{pw} automation takeover", "no_reaction must be brake or keep")
    if not _num(tk.get("budget_s")) or tk["budget_s"] <= 0:
        rep.error(f"{pw} automation takeover", "budget_s must be a positive number")
    at = tk.get("at_s")
    for x in (at if isinstance(at, list) else [at]):
        if x is None:
            continue
        if not _num(x):
            rep.error(f"{pw} automation takeover", f"at_s {x!r} is not a number")
        elif duration is not None and x >= duration:
            rep.warn(f"{pw} automation takeover", f"request at {x} s, after the trial's end ({duration:g} s)")
    trg = [t for t in a.get("triggers") or [] if isinstance(t, dict)]
    for t in trg:
        _unknown(rep, f"{pw} automation trigger", t, TRIGGER_KEYS)
    if at is None and not any(t.get("takeover") for t in trg):
        rep.warn(f"{pw} automation", "no take-over request is scheduled (takeover.at_s or a trigger with "
                                     "takeover: true); only the experimenter can request one")


def check_geometry(rep, geo_checks, bbox, roads):
    if bbox:
        s, w, n, e = bbox
        for where, pts, _, _ in geo_checks:
            for lat, lon in pts:
                if not (s <= lat <= n and w <= lon <= e):
                    rep.error(where, f"{lat:.6f}, {lon:.6f} is outside the map's box [{s}, {w}, {n}, {e}]")
    starts = [(where, pts[0], name) for where, pts, kind, name in geo_checks if kind == "car" and name and pts]
    seen = set()
    for i, (wa, pa, na) in enumerate(starts):
        for wb, pb, nb in starts[i + 1:]:
            if na == nb or (na, nb) in seen:
                continue
            d = _metres(pa, pb)
            if d < VEHICLE_GAP_M:
                seen.add((na, nb))
                rep.error(wb, f"starts {d:.1f} m from {na} ({wa}): the second car cannot spawn there")
    if roads is None:
        return
    for where, pts, kind, _ in geo_checks:
        if kind == "any":
            continue
        for lat, lon in pts:
            try:
                d = roads.distance(lat, lon, "car" if kind == "car" else "any")
            except Exception as err:  # noqa: BLE001 - outside the projection etc.
                rep.warn(where, f"road distance not checked ({err})")
                return
            if d is None:
                rep.error(where, f"{lat:.6f}, {lon:.6f}: no {'driving ' if kind == 'car' else ''}lane within 200 m")
            elif kind == "car" and d > 50:
                rep.error(where, f"{lat:.6f}, {lon:.6f} is {d:.0f} m from the nearest driving lane")
            elif kind == "car" and d > 15:
                rep.warn(where, f"{lat:.6f}, {lon:.6f} is {d:.0f} m from the nearest driving lane (moved onto it)")
            elif kind == "walk" and d > 40:
                rep.warn(where, f"{lat:.6f}, {lon:.6f} is {d:.0f} m from the nearest road")


def check(raw, cfg, roads=True, participants=None, order=None):
    """All checks on a loaded experiment (raw YAML and config.load's merge). Returns (Report, plans by participant
    number)."""
    rep = Report()
    if not isinstance(raw, dict):
        rep.error("file", "is not a YAML mapping")
        return rep, {}
    _keys(rep, raw, config.DEFAULTS)
    _resolve_xy(cfg)          # points given as CARLA {x, y} (a CARLA town): as lat, lon when the map is built
    for level, msg in multigpu.problems(cfg):        # server.multigpu: names, roles, ports
        (rep.error if level == "error" else rep.warn)("server.multigpu", msg)
    for level, msg in photoreal.problems(cfg):       # map.photoreal: keys, caps, needs a baked level
        (rep.error if level == "error" else rep.warn)("map.photoreal", msg)
    ehmi = raw.get("ehmi")
    if isinstance(ehmi, dict):
        _unknown(rep, "ehmi", ehmi, EHMI_KEYS)
        if ehmi.get("render", "world") not in ("world", "station", "both"):
            rep.error("ehmi.render", f"{ehmi.get('render')!r} is not world, station or both")
        if str(ehmi.get("inside", "hide")).lower() not in ("hide", "dim", "mirror"):
            rep.error("ehmi.inside", f"{ehmi.get('inside')!r} is not hide, dim or mirror")
        for name, st in (ehmi.get("states") or {}).items():
            if not isinstance(st, dict) or not set(st) <= {"text", "color"}:
                rep.warn(f"ehmi.states.{name}", "takes {text, color: [r, g, b]}")
    for i, poi in enumerate(raw.get("pois") or []):
        if isinstance(poi, dict):
            _unknown(rep, f"pois[{i}]", poi, POI_KEYS)
    # map
    mdir = config.map_dir(cfg)
    mc = cfg["map"]
    bbox = mc.get("bbox")
    built = (mdir / "meta.json").exists()
    if built and not bbox:
        import json
        bbox = json.loads((mdir / "meta.json").read_text()).get("bbox")
    if not built:
        if mc.get("carla_town") and not mc.get("opendrive"):
            rep.warn("map", f"maps/{mc['name']} is not built yet: the run builds it from {mc['carla_town']}'s "
                            f"OpenDRIVE (CARLA_ROOT or the server)")
        elif mc.get("opendrive"):
            if not Path(mc["opendrive"]).exists() and not (Path(cfg["_root"]) / mc["opendrive"]).exists():
                rep.error("map.opendrive", f"{mc['opendrive']} does not exist")
        elif not bbox:
            rep.error("map", f"maps/{mc['name']} is not built and map.bbox is not set: nothing to build it from")
        else:
            rep.warn("map", f"maps/{mc['name']} is not built yet: the run builds it from OSM (needs the network)")
    road_index = roads if hasattr(roads, "distance") else None      # tests hand in their own
    if roads is True and (mdir / "geo.net.xml").exists():
        try:
            road_index = Roads(mdir / "geo.net.xml")
        except Exception as err:  # noqa: BLE001 - no sumolib (CI) or no pyproj
            rep.warn("map", f"distances to the roads not checked ({type(err).__name__}: {err})")
    # design and conditions
    conds = cfg.get("conditions") or {}
    for name, c in conds.items():
        if not isinstance(c, dict):
            rep.error(f"conditions.{name}", "needs a mapping of trial overrides")
            continue
        _unknown(rep, f"conditions.{name}", c, TRIAL_KEYS - {"condition", "conditions", "block"})
        _override_keys(rep, f"conditions.{name}", c)
    design = cfg.get("design") or {}
    rd = raw.get("design") or {}
    if design.get("order") not in study.ORDERS:
        rep.error("design.order", f"{design.get('order')!r} is not one of {', '.join(study.ORDERS)}")
    factors = design.get("factors") or {}
    for f, levels in factors.items():
        if not isinstance(levels, list) or not levels:
            rep.error(f"design.factors.{f}", "needs a list of condition names")
            continue
        for lv in levels:
            if lv not in conds:
                rep.error(f"design.factors.{f}", f"condition {lv!r} is not defined in conditions")
    if design.get("block_by") and design["block_by"] not in factors:
        rep.error("design.block_by", f"{design['block_by']!r} is not one of the factors")
    if factors and "trials" in raw:
        rep.warn("trials", "design.factors generates the trials: the trials list is ignored")
    if rd and not factors and any(k in rd for k in ("repeat", "practice", "after", "trial", "block_by")):
        rep.warn("design", "has no factors: only design.order is used (on the trials' blocks)")
    for where, trials in (("trials", raw.get("trials") if not factors else None),
                          ("design.practice", design.get("practice")), ("design.after", design.get("after"))):
        for i, t in enumerate(trials or []):
            if not isinstance(t, dict):
                rep.error(f"{where}[{i}]", "needs a mapping")
                continue
            _unknown(rep, f"{where}[{i}]", t, TRIAL_KEYS)
            _override_keys(rep, f"{where}[{i}]", t)
            names = t.get("conditions") if t.get("conditions") is not None else (
                [t["condition"]] if t.get("condition") is not None else [])
            for n in [names] if isinstance(names, str) else names:
                if n not in conds:
                    rep.error(f"{where}[{i}]", f"condition {n!r} is not defined in conditions")
    if any("is not defined in conditions" in m or "design.order" == w for _, w, m in rep.errors):
        return rep, {}      # the plan needs every condition
    trials = study.file_trials(cfg)
    if not trials:
        rep.error("trials", "there are no trials")
        return rep, {}
    blocks = sorted({t["_block"] for t in trials if t["_block"] is not None})
    order = order or design.get("order") or "latin"
    if order == "full" and len(blocks) > 7:
        rep.warn("design.order", f"full counterbalancing of {len(blocks)} blocks needs "
                                 f"{math.factorial(len(blocks))} participants; latin needs {study.square_rows(len(blocks), 'latin')}")
    # every trial's resolved config
    seen_geo = set()
    for t in trials:
        where = f"trial {t['_index']}" + (f" ({t['_condition']})" if t.get("_condition") else "")
        d = t.get("duration_s", 300)
        if not _num(d) or d <= 0:
            rep.error(where, f"duration_s {d!r} must be a positive number")
            d = None
        elif d > 3600:
            rep.warn(where, f"lasts {d / 60:.0f} minutes")
        for kind, key in (("agents", "id"), ("participants", "role")):
            names = {x.get(key) for x in cfg.get(kind) or [] if isinstance(x, dict)}
            names |= {x.get(key) for x in t.get(kind) or [] if isinstance(x, dict)}
            for name in ((t.get("patch") or {}).get(kind) or {}):
                if name not in names:
                    rep.error(f"{where} patch", f"{kind[:-1]} {name!r} does not exist")
        _unknown(rep, f"{where} patch", t.get("patch") or {}, {"agents", "participants"})
        tc = config.trial_config(cfg, t)
        for x in (tc.get("coupling") or {}).get("visual_sync_at_s") or []:
            if d is not None and _num(x) and x >= d:
                rep.warn(f"{where} coupling", f"visual sync at {x} s, after the trial's end ({d:g} s)")
        geo_checks = []
        check_trial_config(rep, tc, where, d, geo_checks)
        for i, poi in enumerate(tc.get("pois") or []):
            if isinstance(poi, dict) and "lat" in poi:
                geo_checks.append((f"pois[{i}]", _points(rep, f"pois[{i}]", [[poi["lat"], poi.get("lon")]]), "any",
                                   None))
        key = repr([(TRIAL_PREFIX.sub("", g[0]), g[1]) for g in geo_checks])
        if key not in seen_geo:            # trials that only change the weather share their geometry
            seen_geo.add(key)
            check_geometry(rep, geo_checks, bbox, road_index)
    # dedupe (the same problem in every trial)
    out, seen = [], set()
    for item in rep.items:
        k = (item[0], TRIAL_PREFIX.sub("", item[1]), item[2])
        if k not in seen:
            seen.add(k)
            out.append(item)
    rep.items = out
    if participants is None:
        n = len(blocks)
        participants = range(1, (min(study.square_rows(n, order), 24) if n else 1) + 1)
    plans = {p: study.plan(cfg, p, order) for p in participants}
    return rep, plans


def _resolve_xy(cfg):
    """towns.resolve_xy on the experiment and its trials with the built map's geo.net.xml (no-op otherwise)."""
    f = config.map_dir(cfg) / "geo.net.xml"
    if not f.exists():
        return
    try:
        from .geo import Geo
        from . import towns
        geo = Geo(f)
    except Exception:  # noqa: BLE001 - no sumolib (CI): the points stay as they are
        return
    towns.resolve_xy(cfg, geo)
    for t in cfg.get("trials") or []:
        if isinstance(t, dict):
            towns.resolve_xy(t, geo)


def load(path):
    raw = yaml.safe_load(Path(path).read_text())
    return raw, config.load(path)


def _range(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--participants", help="e.g. 1-12 or 3,7 (default: one counterbalanced set)")
    ap.add_argument("--order", choices=study.ORDERS, help="override design.order")
    ap.add_argument("--no-roads", action="store_true", help="skip the distances to the roads (fast)")
    ap.add_argument("--quiet", action="store_true", help="problems only, no plan")
    args = ap.parse_args(argv)
    raw, cfg = load(args.experiment)
    rep, plans = check(raw, cfg, roads=not args.no_roads,
                       participants=_range(args.participants) if args.participants else None, order=args.order)
    for level, where, msg in rep.items:
        print(f"{level:7s}  {where}: {msg}")
    n_err, n_warn = len(rep.errors), len(rep.items) - len(rep.errors)
    print(f"{args.experiment}: {n_err} error{'s' * (n_err != 1)}, {n_warn} warning{'s' * (n_warn != 1)}")
    if plans and not args.quiet:
        trials = study.file_trials(cfg)
        blocks = list(dict.fromkeys(t["_block"] for t in trials if t["_block"] is not None))
        order = args.order or (cfg.get("design") or {}).get("order") or "latin"
        print(f"\n{len(trials)} trials, {len(blocks)} blocks, order {order if blocks else 'as written'}"
              + (f" ({study.square_rows(len(blocks), order)} participants per full set)"
                 if blocks and order in ('latin', 'full') else ""))
        for p, pl in plans.items():
            total = sum(float(t.get("duration_s", 300)) for t in pl)
            print(f"participant {p}  ({total / 60:.1f} min of trials)")
            for t in pl:
                print("  " + study.label(t))
    return 1 if rep.errors else 0


if __name__ == "__main__":
    sys.exit(main())
