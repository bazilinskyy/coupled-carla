"""Who the pedestrians are: walkers that look like the people on the street of the map's place.

Every walker coupled-carla spawns on its own (the CROWD crowd, the crowd window's walkers, the living city's groups,
children, wheelchair users, bus passengers and door walkers, the old beside-the-road crowd, CARLA-town AI walkers,
scripted walkers given a blueprint pattern) gets its blueprint from here instead of a uniform pick from CARLA's
library:

  1. the place     the nearest place in data/walker_demographics.yaml whose radius holds the map's centre (Tokyo,
                   Eindhoven, Amsterdam, Salamanca, Zurich, Helsinki, New York, Lyon, Kigali, Chernihiv, ...), else
                   the map's country (crowdpeds.locate: the experiment's city.country, CROWD's localities, OSM) from
                   "countries", else its region; a CARLA town or an unknown place uses the world mix ("generic")
  2. the mix       the residents' mix (census or registry counts by nationality / origin / race and ethnicity,
                   turned into broad appearance groups) and the visitors' mix (tourism statistics by origin), weighted
                   by the share of visitors on the street (visitor nights per year / 365 against the population, times
                   the place's hotspot factor); every number with its source and year in the YAML
  3. the walkers   data/walker_looks.csv tags every walker blueprint of CARLA 0.9.16 and 1.0 (and imported ones) with
                   a broad appearance group, the groups it can also pass for, a coarse skin tone, age group,
                   presentation and clothing: appearance only, from CARLA's own catalogue stills
  4. the sampling  each walking unit (a single walker, or a group: livingcity.dress_groups) is a resident or a visitor
                   (the visitor share), its leader's group is drawn from that component's mix, and its members share
                   the leader's group (family_same_group, default 0.9) or draw again from the same component; a
                   child is a child walker of that group (none: the member walks as an adult of the group), a
                   wheelchair user one that can use a wheelchair (none: the walker walks)

Groups: east_asian (East and Southeast Asian), south_asian, black, white, mena (Middle Eastern / North African),
latin_american, mixed_unclear. Appearance is not nationality, and the numbers are approximate: the goal is a crowd
that looks plausible for the place (README "Who the pedestrians are").

traffic.walkers.looks: {enabled: true, place: auto | <place key> | <ISO3> | generic | uniform,
                        mix: null | {group: share}, family_same_group: 0.9, also_weight: 0.25, seed: null}
"""
import csv
import math
import random
import zlib
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"
GROUPS = ("east_asian", "south_asian", "black", "white", "mena", "latin_american", "mixed_unclear")
DEFAULTS = {"enabled": True, "place": "auto", "mix": None, "family_same_group": 0.9, "also_weight": 0.25,
            "seed": None}
MAX_VISITOR_SHARE = 0.6
UNKNOWN_WEIGHT = 0.5          # a walker blueprint the catalogue does not know: mixed_unclear, at this weight


def _log(msg):
    print(f"[walkers] {msg}", flush=True)


# --- data --------------------------------------------------------------------------------------------------------

@lru_cache(maxsize=None)
def catalogue():
    """[row dict] of data/walker_looks.csv (lines starting with # are comments)."""
    with open(DATA / "walker_looks.csv", newline="", encoding="utf-8") as f:
        lines = [ln for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
    rows = []
    for r in csv.DictReader(lines):
        r["also_reads_as"] = tuple(g for g in (r.get("also_reads_as") or "").split(";") if g)
        r["skin_tone"] = int(r["skin_tone"]) if r.get("skin_tone") else None
        r["crowd_weight"] = float(r.get("crowd_weight") or 1.0)
        rows.append(r)
    return rows


@lru_cache(maxsize=None)
def demographics():
    import yaml
    return yaml.safe_load((DATA / "walker_demographics.yaml").read_text(encoding="utf-8"))


def normalise(mix):
    """{group: share} summing to 1 over GROUPS (unknown keys are an error, negatives 0)."""
    bad = [g for g in mix if g not in GROUPS]
    if bad:
        raise ValueError(f"unknown appearance group(s) {bad}; groups are {GROUPS}")
    tot = sum(max(0.0, float(v)) for v in mix.values())
    if tot <= 0:
        raise ValueError("an appearance mix needs a positive share")
    return {g: max(0.0, float(mix.get(g, 0.0))) / tot for g in GROUPS}


def region_of(iso3, data=None):
    data = data or demographics()
    for name, reg in data["regions"].items():
        if iso3 in reg.get("countries", ()):
            return name
    return None


def resolve(origin, data=None):
    """The appearance mix of an origin: a group name, a named mix, an ISO3 country, a region, an alias of one, or
    other_rest (the world mix)."""
    data = data or demographics()
    origin = data.get("aliases", {}).get(origin, origin)
    if origin in GROUPS:
        return {g: 1.0 if g == origin else 0.0 for g in GROUPS}
    if origin in ("other_rest", "generic"):
        return normalise(data["generic"]["mix"])
    for section in ("mixes", "countries", "regions"):
        if origin in data.get(section, {}):
            return normalise(data[section][origin]["mix"])
    if isinstance(origin, str) and len(origin) == 3 and origin.isupper():
        reg = region_of(origin, data)
        if reg:
            return normalise(data["regions"][reg]["mix"])
    raise KeyError(f"walker_demographics.yaml: no mix for origin {origin!r}")


def blend(weighted):
    """{group: share} of [(weight, mix)]."""
    out = {g: 0.0 for g in GROUPS}
    tot = sum(w for w, _ in weighted)
    for w, m in weighted:
        for g in GROUPS:
            out[g] += w * m.get(g, 0.0) / (tot or 1.0)
    return out


def part_mix(part, data=None):
    """A residents or visitors entry's mix: from 'shares' ({group: share}) or 'counts'/'origins' ({origin: n})."""
    data = data or demographics()
    if part.get("shares"):
        return normalise(part["shares"])
    counts = part.get("counts") or part.get("origins") or {}
    return blend([(float(n), resolve(o, data)) for o, n in counts.items() if float(n) > 0])


def visitor_share(vis):
    """The share of the people on the place's street who are foreign visitors (see the YAML's header)."""
    if not vis:
        return 0.0
    if vis.get("visitor_share") is not None:
        return min(MAX_VISITOR_SHARE, max(0.0, float(vis["visitor_share"])))
    nights = vis.get("foreign_nights_per_year")
    if nights is None:
        nights = float(vis.get("foreign_visitors_per_year", 0)) * float(vis.get("nights", 1))
    present = float(nights) / 365.0
    pop = float(vis.get("population") or 0)
    if present <= 0 or pop <= 0:
        return 0.0
    return min(MAX_VISITOR_SHARE, present / (pop + present) * float(vis.get("hotspot_factor", 1.0)))


def street(key, data=None):
    """{key, name, residents, visitors, visitor_share, mix, sources} for a place key, 'country:ISO3', 'region:name',
    'generic' or 'uniform' (mix None: every blueprint equally likely, as before)."""
    data = data or demographics()
    if key == "uniform":
        return {"key": key, "name": "uniform (every walker blueprint equally likely)", "residents": None,
                "visitors": None, "visitor_share": 0.0, "mix": None, "sources": []}
    if key == "generic":
        g = data["generic"]
        m = normalise(g["mix"])
        return {"key": key, "name": g["name"], "residents": m, "visitors": None, "visitor_share": 0.0, "mix": m,
                "sources": [g.get("source", "")]}
    if key.startswith("country:") or key.startswith("region:"):
        kind, code = key.split(":", 1)
        sect = data["countries" if kind == "country" else "regions"].get(code)
        if sect is None and kind == "country":
            reg = region_of(code, data)
            if reg is None:
                return street("generic", data)
            sect, key = data["regions"][reg], f"region:{reg}"
        m = normalise(sect["mix"])
        src = sect.get("source") or data.get("region_note", "")
        return {"key": key, "name": key, "residents": m, "visitors": None, "visitor_share": 0.0, "mix": m,
                "sources": [src]}
    p = data["places"][key]
    res = part_mix(p["residents"], data)
    vis = part_mix(p["visitors"], data) if p.get("visitors") and (p["visitors"].get("origins") or
                                                                   p["visitors"].get("shares")) else None
    v = visitor_share(p.get("visitors")) if vis else 0.0
    mix = blend([(1.0 - v, res)] + ([(v, vis)] if vis else []))
    sources = [s for part in ("residents", "visitors") for s in (p.get(part) or {}).get("sources", [])]
    return {"key": key, "name": p.get("name", key), "residents": res, "visitors": vis, "visitor_share": v,
            "mix": mix, "sources": sources}


def _km(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 12742.0 * math.asin(min(1.0, math.sqrt(a)))


def place_at(lat, lon, iso3=None, data=None):
    """(key, how) of the place for a map centred at (lat, lon) in country iso3 (or None)."""
    data = data or demographics()
    best, bd = None, None
    for key, p in data["places"].items():
        m = p["match"]
        d = _km(lat, lon, m["lat"], m["lon"])
        if d <= m["km"] and (bd is None or d < bd):
            best, bd = key, d
    if best:
        return best, f"place {data['places'][best].get('name', best)} ({bd:.0f} km)"
    if iso3 and iso3 in data["countries"]:
        return f"country:{iso3}", f"country {iso3}"
    if iso3 and region_of(iso3, data):
        return f"region:{region_of(iso3, data)}", f"region of {iso3}"
    return "generic", "no known place or country: the world mix"


def settings(tc):
    w = ((tc or {}).get("traffic") or {}).get("walkers") or {}
    got = w.get("looks")
    if got is False:
        got = {"enabled": False}
    return {**DEFAULTS, **(got or {})}


def street_for(tc, mdir=None, log=_log):
    """The street mix for a trial: traffic.walkers.looks.place/mix, else located from the map (see place_at)."""
    cfg = settings(tc)
    data = demographics()
    if cfg.get("mix"):
        m = normalise(cfg["mix"])
        return {"key": "config", "name": "traffic.walkers.looks.mix", "residents": m, "visitors": None,
                "visitor_share": 0.0, "mix": m, "sources": ["experiment config"], "how": "experiment config"}
    want = cfg.get("place", "auto")
    if want and want != "auto":
        key = want if (want in data["places"] or want in ("generic", "uniform") or ":" in want) else \
            (f"country:{want.upper()}" if len(want) == 3 else want)
        s = street(key, data)
        s["how"] = "traffic.walkers.looks.place"
        return s
    m = (tc or {}).get("map") or {}
    if m.get("carla_town"):
        s = street("generic", data)
        s["how"] = "a CARLA town (no real place): the world mix"
        return s
    bbox = m.get("bbox")
    if not bbox and mdir is not None and (Path(mdir) / "meta.json").exists():
        import json
        bbox = json.loads((Path(mdir) / "meta.json").read_text()).get("bbox")
    if not bbox:
        s = street("generic", data)
        s["how"] = "no map box: the world mix"
        return s
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    iso3 = None
    try:
        from . import crowdpeds
        cache = Path(mdir) / "real" / "crowdpeds_country.json" if mdir is not None else None
        iso3 = crowdpeds.locate(tc, bbox, cache)[0]
    except Exception:  # noqa: BLE001 - no country: places by position only, else the world mix
        iso3 = None
    key, how = place_at(lat, lon, iso3, data)
    s = street(key, data)
    s["how"] = how
    return s


# --- the walkers -------------------------------------------------------------------------------------------------

def _attr(bp, name):
    try:
        if bp.has_attribute(name):
            a = bp.get_attribute(name)
            return a.as_str() if hasattr(a, "as_str") else str(a)
    except (RuntimeError, AttributeError):
        pass
    return ""


def library_walkers(lib):
    """[{id, age, gender, wheelchair}] of the walker blueprints a server offers."""
    out = []
    for bp in sorted(lib.filter("walker.pedestrian.*"), key=lambda b: b.id):
        out.append({"id": bp.id, "age": _attr(bp, "age").lower(), "gender": _attr(bp, "gender").lower(),
                    "wheelchair": _attr(bp, "can_use_wheelchair").lower() == "true"})
    return out


def entries_for(walkers, rows=None):
    """The catalogue entries of the walkers a server offers: the CARLA version whose rows agree best with the
    blueprints' ids and age attributes (0.9.16 and 1.0 number the same people differently), plus imported
    walkers (rows of any other version), plus untagged walkers as mixed_unclear. Returns (entries, version,
    untagged ids)."""
    rows = catalogue() if rows is None else rows
    by_ver = {}
    for r in rows:
        by_ver.setdefault(r["carla"], {})[r["blueprint"]] = r
    have = {w["id"]: w for w in walkers}

    def score(ver):
        s = 0
        for bid, r in by_ver[ver].items():
            w = have.get(bid)
            if w is None:
                s -= 1
            elif not w["age"] or not r["carla_age"] or w["age"] == r["carla_age"]:
                s += 1
            else:
                s -= 2
        return s
    stock = [v for v in by_ver if v in ("0.9.16", "1.0")]
    version = max(stock, key=score) if stock else None
    out, untagged = [], []
    for w in walkers:
        r = (by_ver.get(version) or {}).get(w["id"])
        if r is None:      # an imported walker: a row of another (non-stock) version
            r = next((by_ver[v][w["id"]] for v in by_ver if v not in stock and w["id"] in by_ver[v]), None)
        if r is None:
            untagged.append(w["id"])
            r = {"blueprint": w["id"], "group": "mixed_unclear", "also_reads_as": (), "crowd_weight": UNKNOWN_WEIGHT,
                 "age_group": "child" if w["age"] == "child" else "adult", "presentation": w["gender"],
                 "clothing": "", "special": "", "skin_tone": None}
        child = (w["age"] == "child") if w["age"] else r.get("age_group") == "child"
        wheel = w["wheelchair"] if w["age"] else str(r.get("wheelchair", "")).lower() == "true"
        out.append({"id": w["id"], "group": r["group"], "also": tuple(r.get("also_reads_as") or ()),
                    "weight": float(r.get("crowd_weight", 1.0)), "child": child, "wheelchair": wheel,
                    "presentation": r.get("presentation") or w["gender"], "clothing": r.get("clothing", ""),
                    "special": r.get("special", "")})
    return out, version, untagged


class Looks:
    """Chooses walker blueprints for a street mix (see the module doc). entries: entries_for()[0]; street: street()."""

    def __init__(self, entries, street, cfg=None, seed=1):
        self.cfg = {**DEFAULTS, **(cfg or {})}
        self.entries, self.street = list(entries), street
        self.rng = random.Random(seed if self.cfg.get("seed") is None else self.cfg["seed"])
        self.gaps = {"child": 0, "wheelchair": 0, "group": {}}
        self.picked, self.drawn = [], []      # blueprint ids, and the group each was drawn for

    # candidates ---------------------------------------------------------------------------------------------------
    def candidates(self, group, child=False, wheelchair=False):
        """[(entry, weight)] for a group: walkers tagged with it, then (at also_weight) those that also pass for it;
        within the tagged ones female and male presentations get equal halves when both exist."""
        pool = [e for e in self.entries if e["child"] == child and (not wheelchair or e["wheelchair"])]
        prim = [e for e in pool if e["group"] == group]
        also = [e for e in pool if e["group"] != group and group in e["also"]]
        out = []
        pres = {}
        for e in prim:
            pres.setdefault(e["presentation"], []).append(e)
        for es in pres.values():
            tot = sum(e["weight"] for e in es) or 1.0
            out += [(e, e["weight"] / tot / len(pres)) for e in es]
        if also:       # each counts also_weight of one tagged walker (times its crowd weight); alone when none is
            per = 1.0 / len(prim) if prim else 1.0
            aw = float(self.cfg["also_weight"]) if prim else 1.0
            out += [(e, aw * e["weight"] * per) for e in also]
        return out

    def _draw(self, cands, rng):
        es, ws = zip(*cands)
        return rng.choices(es, weights=ws)[0]

    def draw_group(self, visitor, rng=None):
        rng = rng or self.rng
        mix = (self.street["visitors"] if visitor and self.street.get("visitors") else
               self.street["residents"]) or self.street["mix"]
        return rng.choices(GROUPS, weights=[mix[g] for g in GROUPS])[0]

    def choose(self, group, child=False, wheelchair=False, rng=None):
        """A blueprint id for a walker of this group (None: no child / wheelchair walker of it exists). An adult of a
        group no walker can pass for falls back to mixed_unclear, then to any adult."""
        rng = rng or self.rng
        if self.street.get("mix") is None:            # uniform
            pool = [e for e in self.entries if e["child"] == child and (not wheelchair or e["wheelchair"])]
            return rng.choice(pool)["id"] if pool else None
        c = self.candidates(group, child, wheelchair)
        if c:
            return self._draw(c, rng)["id"]
        if child or wheelchair:
            return None
        self.gaps["group"][group] = self.gaps["group"].get(group, 0) + 1
        c = self.candidates("mixed_unclear") or [(e, e["weight"]) for e in self.entries if not e["child"]]
        return self._draw(c, rng)["id"] if c else None

    # plans --------------------------------------------------------------------------------------------------------
    def dress(self, walkers):
        """Sets "bp" (and "look", "visitor") on crowdpeds plan dicts, group-aware (a member's "follow" names its
        leader's index in this list). A living-city child or wheelchair user keeps its role when the group has such
        a walker, else walks as an adult / on foot. Plans with another explicit "bp" are left alone."""
        v = float(self.street.get("visitor_share") or 0.0)
        same = float(self.cfg["family_same_group"])
        for i, w in enumerate(walkers):
            if w.get("bp") and not (w.get("child") or w.get("wheelchair")):
                continue
            f = w.get("follow")
            lead = walkers[f[0]] if f is not None and 0 <= f[0] < len(walkers) and f[0] != i else None
            if lead is not None and lead.get("look"):
                visitor = bool(lead.get("visitor"))
                group = lead["look"] if self.rng.random() < same else self.draw_group(visitor)
            else:
                visitor = self.rng.random() < v
                group = self.draw_group(visitor)
            bp = None
            if w.get("child"):
                bp = self.choose(group, child=True)
                if bp is None:
                    self.gaps["child"] += 1
                    w["child"] = False
                    if w.get("follow") and lead is not None:      # back to an adult's place beside the leader
                        w["follow"] = (f[0], 0.75 if f[1] >= 0 else -0.75, max(0.0, f[2] - 0.2))
            if bp is None and w.get("wheelchair"):
                bp = self.choose(group, wheelchair=True)
                if bp is None:
                    self.gaps["wheelchair"] += 1
                    w["wheelchair"] = False
                    w.pop("speed_factor", None)
            if bp is None:
                bp = self.choose(group)
            w["bp"], w["look"], w["visitor"] = bp, group, visitor
            self.picked.append(bp)
            self.drawn.append(group)
        return walkers

    def pick(self, w=None, rng=None):
        """A blueprint id for one walker spawned on its own (w: its plan, if any: follows its "look")."""
        rng = rng or self.rng
        group = (w or {}).get("look") or self.draw_group(rng.random() < float(self.street.get("visitor_share") or 0),
                                                         rng)
        bp = self.choose(group, child=bool((w or {}).get("child")), rng=rng) or self.choose(group, rng=rng)
        self.picked.append(bp)
        self.drawn.append(group)
        return bp

    def pick_keyed(self, key):
        """A blueprint id fixed by a key (a scripted walker's id): the same walker every run."""
        return self.pick(rng=random.Random(zlib.crc32(str(key).encode())))

    # reports ------------------------------------------------------------------------------------------------------
    def group_of(self, bp_id):
        for e in self.entries:
            if e["id"] == bp_id:
                return e["group"]
        return None

    def realised(self, bp_ids=None):
        """{group: share} of the walkers' primary groups (bp_ids: blueprint ids, default every pick so far)."""
        ids = [b for b in (self.picked if bp_ids is None else bp_ids) if b]
        out = {g: 0.0 for g in GROUPS}
        for b in ids:
            g = self.group_of(b)
            if g in out:
                out[g] += 1.0 / len(ids)
        return out, len(ids)

    def summary(self, bp_ids=None):
        """The walkers' mix by their catalogue group against the target, and (for the walkers drawn so far) by the
        group they were drawn for: they differ where a walker that also passes for a group stood in for it."""
        real, n = self.realised(bp_ids)
        tgt = self.street.get("mix")
        drawn = {g: self.drawn.count(g) / len(self.drawn) for g in GROUPS} if self.drawn else None
        parts = []
        for g in GROUPS:
            t = tgt[g] if tgt else None
            if (t or 0) >= 0.005 or real[g] >= 0.005:
                parts.append(f"{g} {real[g] * 100:.0f} %" + (f" (target {t * 100:.0f} %" if t is not None else "")
                             + (f", drawn {drawn[g] * 100:.0f} %" if drawn and t is not None else "")
                             + (")" if t is not None else ""))
        gaps = []
        if self.gaps["child"]:
            gaps.append(f"{self.gaps['child']} children walk as adults (no child walker of their group)")
        if self.gaps["wheelchair"]:
            gaps.append(f"{self.gaps['wheelchair']} wheelchair users walk (no wheelchair walker of their group)")
        for g, k in self.gaps["group"].items():
            gaps.append(f"{k} {g} walkers drawn from mixed_unclear (no walker of the group)")
        return f"{n} walkers: " + ", ".join(parts) + ("; " + "; ".join(gaps) if gaps else "")


# --- the trial's looks (set by run.py, used by every spawner) ----------------------------------------------------

_street = None
_cfg = dict(DEFAULTS)
_bound = {}


def setup(tc, mdir=None, log=_log):
    """Works out this trial's street mix (before the world is loaded); returns it, or None when looks are off."""
    global _street, _cfg
    _bound.clear()
    _cfg = settings(tc)
    if not _cfg.get("enabled", True):
        _street = None
        log("walker looks: off (every walker blueprint equally likely)")
        return None
    try:
        _street = street_for(tc, mdir, log)
    except Exception as err:  # noqa: BLE001 - a broken data file must not stop a trial
        log(f"walker looks: no street mix ({type(err).__name__}: {err}); every walker equally likely")
        _street = None
        return None
    s = _street
    if s.get("mix"):
        top = sorted(s["mix"].items(), key=lambda kv: -kv[1])
        log(f"walker looks: {s['name']} ({s.get('how', '')}); street mix "
            + ", ".join(f"{g} {p * 100:.0f} %" for g, p in top if p >= 0.005)
            + (f"; visitors {s['visitor_share'] * 100:.0f} % of the people" if s.get("visitor_share") else ""))
    else:
        log(f"walker looks: {s['name']}")
    return s


def for_library(lib, seed=1, log=_log):
    """The trial's Looks bound to the server's blueprint library on first use (one server per trial; None when looks
    are off or not set up)."""
    if _street is None:
        return None
    key = "trial"
    if key not in _bound:
        entries, version, untagged = entries_for(library_walkers(lib))
        if not entries:
            return None
        lk = Looks(entries, _street, _cfg, seed)
        lk.version, lk.untagged = version, untagged
        have = {g: sum(1 for e in entries if e["group"] == g and not e["child"]) for g in GROUPS}
        log(f"walker looks: CARLA {version} catalogue, {len(entries)} walker blueprints; adults per group "
            + ", ".join(f"{g} {k}" for g, k in have.items() if k))
        if untagged:
            log(f"walker looks: {len(untagged)} walker blueprints not in data/walker_looks.csv (as mixed_unclear): "
                + ", ".join(untagged[:8]) + (" ..." if len(untagged) > 8 else ""))
        _bound[key] = lk
    return _bound[key]


def choose_blueprint(lib, rng, bps=None, key=None, plan=None):
    """A walker blueprint for a spawner that used to pick rng.choice(bps): the trial's looks when set, else that
    uniform pick (bps: the walker blueprints, default the library's; key: a fixed pick, e.g. a scripted walker's
    id; plan: a crowd plan whose "look" or "child" it follows)."""
    lk = for_library(lib)
    if lk is not None:
        bid = lk.pick_keyed(key) if key is not None else lk.pick(plan, rng=rng)
        if bid:
            try:
                return lib.find(bid)
            except (RuntimeError, IndexError):
                pass
    bps = bps if bps is not None else sorted(lib.filter("walker.pedestrian.*"), key=lambda b: b.id)
    if key is not None:
        return bps[zlib.crc32(str(key).encode()) % len(bps)]
    return rng.choice(bps)


def report(actors, log=_log, lib=None, what="crowd"):
    """Logs the realised mix of spawned walker actors against the target; returns (realised, n)."""
    if _street is None or not _bound:
        return None
    lk = _bound["trial"]
    ids = []
    for a in actors:
        try:
            ids.append(a.type_id)
        except RuntimeError:
            pass
    log(f"walker looks ({what}): " + lk.summary(ids))
    return lk.realised(ids)
