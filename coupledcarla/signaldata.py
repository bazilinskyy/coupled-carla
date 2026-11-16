"""Real signal data for the traffic lights (traffic.signals: real): recorded phases replayed, or a live feed.

No real phase data of Eindhoven is open (README, Traffic lights; docs/signals.md says what to ask for). This module
takes such data once there is some, in a plain form any source can be converted to:

- a phase log: CSV with a header `time,controller,signal_group,state` (or NDJSON lines with those keys): one row per
  state change of a signal group. time: ISO 8601 (local time when it has no zone) or Unix seconds; controller: the
  VRI number ("1001", "EIN2001", "vri1001") or a controller id of the plan (signals/<key>.plan.json); signal_group:
  the controller's own number ("02", "fc22", ...); state: G/g/Y/R/O, or the SPaT movement event states of ETSI
  TS 103 301 / SAE J2735 (protected-Movement-Allowed, permissive-Movement-Allowed, protected-clearance,
  permissive-clearance, stop-And-Remain, stop-Then-Proceed, caution-Conflicting-Traffic, dark). A V-Log (.vlg)
  converts to it with the controller's .vlt/.vlc configuration (state changes of the "fc" signal groups);
- the signal groups: JSON {controller: {signal_group: {"from": compass bearing the stream comes from (0 = from the
  north), "turns": "lsr" (the turns it signals), "mode": "car" | "bike" | "bus"}}}, from the controller's MAP
  topology (SPaT's companion: lanes with their signal group) or its intersection drawing. SUMO's links are matched to
  them by approach direction, turn and mode.

Three ways to use them:
- replay "program" (default): per controller the SUMO programme of the chosen hour (simulation.time or
  signal_data.date / hour): the typical cycle (the shift that best repeats all its groups' states), each cycle
  starting where a group green once a cycle turns green, the phases of a cycle folded over all cycles of the hour
  (each second the most common state of every group), the offset of the cycle at the trial's start, and type
  "actuated" when the cycles or greens vary (coefficient of variation over 0.1 or 0.15) with each green between the
  10th and 90th percentile of its recorded length;
- replay "exact": TraCI sets every recorded state at its time (ExactDriver), as it happened;
- live: a feed of the same rows (NDJSON lines from a file being written, an http(s) stream or tcp://host:port), set
  through TraCI as they come (LiveDriver). A UDAP (Talking Traffic) SPaT stream is ASN.1 UPER over TLS with a token;
  a decoder turning it into these lines is all that is missing once there is an account.
Controllers without data keep their Dutch programme (signalplan.py); if the feed fails, they go back to it.
"""
import csv
import json
import math
import statistics
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta
from pathlib import Path

STATE = {"g": "G", "G": "G", "green": "G", "protected-movement-allowed": "G", "protected": "G",
         "permissive-movement-allowed": "g", "permissive": "g",
         "y": "y", "Y": "y", "a": "y", "A": "y", "amber": "y", "yellow": "y", "geel": "y",
         "protected-clearance": "y", "permissive-clearance": "y", "caution-conflicting-traffic": "o",
         "r": "r", "R": "r", "red": "r", "rood": "r", "stop-and-remain": "r", "stop-then-proceed": "r",
         "pre-movement": "r", "o": "o", "O": "o", "dark": "o", "off": "o", "unavailable": "o", "groen": "G"}
CV_ACTUATED = 0.1         # cycles varying more than this (coefficient of variation) ...
GREEN_CV_ACTUATED = 0.15  # ... or a group's greens more than this: an actuated programme
BEARING_TOL = 45.0        # a signal group's approach direction matches a SUMO approach within this (deg)
TZ = "Europe/Amsterdam"


def _log(msg):
    print(f"[signals] {msg}", flush=True)


def norm_state(s):
    """A state of any of the accepted spellings -> SUMO's G, g, y, r or o (None when unknown)."""
    s = str(s).strip()
    return STATE.get(s) or STATE.get(s.lower()) or STATE.get(s.replace("_", "-").lower())


def norm_controller(c):
    """"EIN2001", "2001", "vri2001" -> "vri2001"; other ids as they are."""
    c = str(c).strip()
    digits = "".join(ch for ch in c if ch.isdigit())
    if digits and (c.isdigit() or c.lower().startswith(("ein", "vri", "ehv"))):
        return f"vri{int(digits)}"
    return c


def norm_group(g):
    """"fc02", "02", "2" -> "2"; "fc22" -> "22"."""
    g = str(g).strip().lower()
    digits = "".join(ch for ch in g if ch.isdigit())
    return str(int(digits)) if digits else g


def parse_time(v):
    """ISO 8601 or Unix seconds -> a naive local datetime (Europe/Amsterdam)."""
    from zoneinfo import ZoneInfo
    tz = ZoneInfo(TZ)
    if isinstance(v, (int, float)) or (isinstance(v, str) and v.replace(".", "", 1).isdigit()):
        return datetime.fromtimestamp(float(v), tz).replace(tzinfo=None)
    d = datetime.fromisoformat(str(v).strip().replace("Z", "+00:00"))
    if d.tzinfo is not None:
        d = d.astimezone(tz).replace(tzinfo=None)
    return d


def read_rows(lines_or_path):
    """Rows of a phase log (CSV with a header or NDJSON) -> [(datetime, controller, group, state)], sorted."""
    if isinstance(lines_or_path, (str, Path)) and Path(lines_or_path).is_dir():
        out = []
        for p in sorted(Path(lines_or_path).glob("*")):
            if p.suffix.lower() in (".csv", ".ndjson", ".jsonl", ".json"):
                out += read_rows(p)
        return sorted(out)
    if isinstance(lines_or_path, (str, Path)):
        lines = Path(lines_or_path).read_text().splitlines()
    else:
        lines = list(lines_or_path)
    lines = [ln for ln in lines if ln.strip()]
    out = []
    if lines and lines[0].lstrip().startswith("{"):
        recs = (json.loads(ln) for ln in lines)
    else:
        recs = csv.DictReader(lines)
    for r in recs:
        try:
            st = norm_state(r.get("state") or r.get("eventState"))
            if st is None:
                continue
            out.append((parse_time(r.get("time") or r.get("timestamp")),
                        norm_controller(r.get("controller") or r.get("intersection")),
                        norm_group(r.get("signal_group") or r.get("signalGroup")), st))
        except (ValueError, TypeError, AttributeError):
            continue
    return sorted(out)


def timelines(rows, controller):
    """{group: [(datetime, state)]} of one controller."""
    out = {}
    for t, c, g, s in rows:
        if c == controller:
            out.setdefault(g, []).append((t, s))
    return out


def state_at(tl, t):
    """The state of a timeline [(datetime, state)] at datetime t (None before its first row)."""
    lo, hi = 0, len(tl)
    while lo < hi:
        mid = (lo + hi) // 2
        if tl[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return tl[lo - 1][1] if lo else None


def green_starts(tl, t0, t1):
    return [t for (t, s), (_tp, sp) in zip(tl[1:], tl[:-1]) if s in "Gg" and sp not in "Gg" and t0 <= t < t1]


def greens(tl, t0, t1):
    """Lengths (s) of the greens that start in [t0, t1)."""
    out = []
    for i, (t, s) in enumerate(tl):
        if s in "Gg" and (i == 0 or tl[i - 1][1] not in "Gg") and t0 <= t < t1:
            end = next((tt for tt, ss in tl[i + 1:] if ss not in "Gg"), None)
            if end is not None:
                out.append((end - t).total_seconds())
    return out


def _pct(vals, p):
    v = sorted(vals)
    if not v:
        return None
    i = (len(v) - 1) * p / 100.0
    lo = int(math.floor(i))
    return v[lo] + (v[min(lo + 1, len(v) - 1)] - v[lo]) * (i - lo)


# --- matching a controller's signal groups to SUMO's links --------------------------------------------------------

def match_groups(prog, groups):
    """{link index: signal group} of a plan's controller (signalplan programme: links with bearing, turn, mode) for a
    signal-group description {group: {"from", "turns", "mode"}}. A link matches the group of its mode whose approach
    direction is nearest (within BEARING_TOL) among those that signal its turn."""
    out = {}
    for lk in prog["links"]:
        came_from = (lk["bearing"] + 180.0) % 360.0
        best = None
        for g, d in groups.items():
            if d.get("mode", "car") != lk["mode"] or lk["turn"] not in str(d.get("turns", "lsr")):
                continue
            diff = abs((float(d["from"]) - came_from + 180.0) % 360.0 - 180.0)
            if diff <= BEARING_TOL and (best is None or diff < best[0]):
                best = (diff, norm_group(g))
        if best:
            out[lk["k"]] = best[1]
    return out


def fill_unmatched(prog, link_group):
    """Links without a signal group copy one of their Dutch group's links that has one (else they stay red):
    {link: group or None}."""
    out = dict(link_group)
    for g in prog["groups"]:
        known = [link_group[k] for k in g["links"] if k in link_group]
        for k in g["links"]:
            if k not in out:
                out[k] = known[0] if known else None
    for lk in prog["links"]:
        out.setdefault(lk["k"], None)
    return out


def link_states(n_links, link_group, group_state, yields=()):
    """The SUMO state string for group states {group: G|g|y|r|o}: 'r' for links without a group; a 'G' link that must
    yield to another green link (the plan's yield pairs) shows 'g'."""
    st = []
    for k in range(n_links):
        g = link_group.get(k)
        st.append(group_state.get(g, "r") if g is not None else "r")
    for a, b in yields:
        if a < n_links and b < n_links and st[a] == "G" and st[b] in "Gg":
            st[a] = "g"
    return "".join(st)


# --- a programme from the recorded hour ---------------------------------------------------------------------------

def program_from_log(prog, tls, link_group, t_start, hours=1.0, step=1.0):
    """The SUMO programme of one controller for the recorded hour from t_start (datetime): {id, type, offset, phases,
    cycle, stats} or None (too few cycles). tls: timelines(); link_group: fill_unmatched()."""
    used = {g for g in link_group.values() if g is not None and g in tls}
    if not used:
        return None
    last = max(tl[-1][0] for g, tl in tls.items() if g in used)
    t1 = min(t_start + timedelta(hours=hours), last)
    period = typical_period(tls, used, t_start, t1)
    if period is None:
        return None
    span = (t1 - t_start).total_seconds()
    # the reference: a group green once per cycle (its green starts nearest span / period), the longest green first
    ref = min(used, key=lambda g: (abs(len(green_starts(tls[g], t_start, t1)) - span / period),
                                   -sum(greens(tls[g], t_start, t1)), g))
    starts = green_starts(tls[ref], t_start - timedelta(minutes=10), t1)
    starts = [s for s in starts if s >= t_start - timedelta(minutes=5)]
    if len(starts) < 3:
        return None
    cycles = [(b - a).total_seconds() for a, b in zip(starts, starts[1:])]
    cyc = statistics.median(cycles)
    cv = statistics.pstdev(cycles) / statistics.mean(cycles) if len(cycles) > 1 else 0.0
    n = max(1, int(round(cyc / step)))
    columns = []
    for i in range(n):                    # each second of the cycle: every group's most common state
        frac = i / n
        votes = {g: {} for g in used}
        for a, b in zip(starts, starts[1:]):
            t = a + timedelta(seconds=frac * (b - a).total_seconds())
            for g in used:
                s = state_at(tls[g], t) or "r"
                votes[g][s] = votes[g].get(s, 0) + 1
        columns.append({g: max(v.items(), key=lambda kv: (kv[1], kv[0] == "r"))[0] for g, v in votes.items()})
    yields = prog.get("yields") or []
    nl = len(prog["links"])
    states = [link_states(nl, link_group, col, yields) for col in columns]
    phases = []
    for s in states:
        if phases and phases[-1]["state"] == s:
            phases[-1]["duration"] += step
        else:
            phases.append({"duration": step, "state": s, "minDur": None, "maxDur": None, "name": ""})
    if len(phases) > 1 and phases[0]["state"] == phases[-1]["state"]:
        phases[0]["duration"] += phases.pop()["duration"]
    green_cv = {}
    for g in used:
        gs = greens(tls[g], t_start, t1)
        if len(gs) >= 5 and statistics.mean(gs) > 0:
            green_cv[g] = statistics.pstdev(gs) / statistics.mean(gs)
    actuated = cv > CV_ACTUATED or max(green_cv.values(), default=0.0) > GREEN_CV_ACTUATED
    if actuated:
        for p in phases:
            lead = [g for g in used if any(link_group.get(k) == g and c in "Gg" for k, c in enumerate(p["state"]))]
            obs = [x for g in lead for x in greens(tls[g], t_start, t1)]
            if obs and p["duration"] >= 4:
                lo, hi = _pct(obs, 10), _pct(obs, 90)
                p["minDur"] = round(max(4.0, min(lo, p["duration"])), 1)
                p["maxDur"] = round(max(hi, p["duration"]), 1)
    # where the cycle stands at the trial's start: the last green start of ref before it
    before = [s for s in starts if s <= t_start]
    pos = ((t_start - before[-1]).total_seconds() if before else cyc - (starts[0] - t_start).total_seconds()) % cyc
    # the phases start where ref turns green (fold origin); SUMO's programme time at simulation time t is t - offset
    out = {"id": prog["id"], "type": "actuated" if actuated else "static", "offset": round(-pos, 1) % cyc,
           "phases": phases, "links": prog["links"], "nodes": prog["nodes"], "cycle": round(cyc, 1),
           "stats": {"cycles": len(cycles), "cycle_s": round(cyc, 1), "cycle_cv": round(cv, 3), "reference": ref,
                     "green_cv_max": round(max(green_cv.values(), default=0.0), 3),
                     "groups": {g: {"green_median_s": round(statistics.median(greens(tls[g], t_start, t1)), 1)
                                    if greens(tls[g], t_start, t1) else None} for g in sorted(used)}}}
    return out


def typical_period(tls, used, t0, t1, lo=30, hi=180):
    """The cycle (s) of a controller's recorded states: the shift that best repeats all its groups' states
    (autocorrelation over the window, 1 s samples); the shortest shift within 2% of the best wins (not a multiple).
    Groups green twice a cycle (cyclists, a group of several stages) made the green-start intervals half a cycle."""
    n = int((t1 - t0).total_seconds())
    if n < 2 * lo:
        return None
    cols = []
    for g in sorted(used):
        tl = tls[g]
        cols.append([state_at(tl, t0 + timedelta(seconds=i)) or "r" for i in range(n)])
    rows = list(zip(*cols))
    best = {}
    for p in range(lo, min(hi, n // 2) + 1):
        m = n - p
        best[p] = sum(rows[i] == rows[i + p] for i in range(0, m)) / m
    top = max(best.values())
    return float(min(p for p, v in best.items() if v >= top - 0.02))


def sumo_offset(pos, cycle):
    """SUMO's tlLogic offset so that the programme is `pos` seconds into its cycle at simulation time 0."""
    return round((-pos) % cycle, 1)


# --- configuration and drivers ------------------------------------------------------------------------------------

class Source:
    """traffic.signal_data: where the real data is and how it is used."""

    def __init__(self, sd, root=None, log=_log):
        self.sd, self.log = sd or {}, log
        base = Path(root) if root else Path.cwd()

        def path(key):
            v = self.sd.get(key)
            if not v:
                return None
            p = Path(v).expanduser()
            return p if p.is_absolute() else base / p
        self.log_path, self.groups_path = path("log"), path("groups")
        self.live = self.sd.get("live")
        self.replay = self.sd.get("replay", "program")

    def available(self):
        return bool(self.live) or bool(self.log_path and self.log_path.exists() and self.groups_path
                                       and self.groups_path.exists())

    def start_time(self, when):
        """The recorded moment the trial starts at: signal_data.date (YYYY-MM-DD) and hour, else simulation.time."""
        d, h = self.sd.get("date"), self.sd.get("hour")
        if d:
            base = datetime.strptime(str(d), "%Y-%m-%d")
            hh = float(h) if h is not None else (when.hour + when.minute / 60.0 if when else 8.0)
            return base + timedelta(hours=hh)
        return when or datetime.now().replace(hour=8, minute=0, second=0, microsecond=0)

    def groups(self):
        raw = json.loads(self.groups_path.read_text()) if self.groups_path and self.groups_path.exists() else {}
        return {norm_controller(c): g for c, g in raw.items()}

    def programs(self, progs, work, when, log=_log):
        """Replay "program": an additional file with the recorded-hour programme of each controller the log covers
        (programID "real": SUMO runs the programme loaded last). Returns (file name or None, info)."""
        if not (self.log_path and self.log_path.exists()):
            return None, {"replayed": 0, "reason": "no phase log"}
        rows = read_rows(self.log_path)
        groups = self.groups()
        t0 = self.start_time(when)
        out, info = [], {"start": t0.isoformat(), "replayed": 0, "controllers": {}}
        for prog in progs:
            gdesc = groups.get(prog["id"]) or groups.get(norm_controller(prog.get("vri") or "")) or {}
            if not gdesc:
                continue
            ctl = prog["id"] if any(c == prog["id"] for _t, c, _g, _s in rows) else \
                norm_controller(prog.get("vri") or prog["id"])
            tls = timelines(rows, ctl)
            if not tls:
                continue
            lg = fill_unmatched(prog, match_groups(prog, gdesc))
            if self.replay == "exact":
                info["controllers"][prog["id"]] = {"exact": True, "links_matched": sum(v is not None
                                                                                         for v in lg.values())}
                continue
            p = program_from_log(prog, tls, lg, t0)
            if p is None:
                info["controllers"][prog["id"]] = {"error": "fewer than 3 cycles in the hour"}
                continue
            out.append(p)
            info["controllers"][prog["id"]] = {**p["stats"], "type": p["type"],
                                               "links_matched": sum(v is not None for v in lg.values()),
                                               "links": len(prog["links"])}
        info["replayed"] = len(out)
        if not out:
            if self.replay != "exact":
                log("traffic lights (real): the phase log covers none of the map's junctions; Dutch programmes")
            return None, info
        from .signalplan import tl_logic_xml
        root = ET.Element("additional")
        for p in out:
            root.append(tl_logic_xml(p, "real"))
        ET.indent(root, space="  ")
        name = "signals_real.add.xml"
        Path(work).mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(str(Path(work) / name), encoding="UTF-8", xml_declaration=True)
        log(f"traffic lights (real): {len(out)} junctions replay {t0:%Y-%m-%d %H:%M} (" +
            ", ".join(f"{p['id']} {p['stats']['cycle_s']:.0f} s {p['type']}" for p in out) + ")")
        return name, info

    def driver_spec(self, when):
        if self.live:
            return {"kind": "live", "source": self.live, "groups": str(self.groups_path or "")}
        if self.replay == "exact" and self.available():
            return {"kind": "exact", "log": str(self.log_path), "groups": str(self.groups_path),
                    "start": self.start_time(when).isoformat()}
        return None


class _Applier:
    """Sets controllers' states through TraCI: group states -> link state strings; on failure the programme back."""

    def __init__(self, progs, groups, log=_log):
        self.log = log
        self.progs = {}
        for prog in progs:
            gdesc = groups.get(prog["id"]) or groups.get(norm_controller(prog.get("vri") or "")) or {}
            if gdesc:
                self.progs[prog["id"]] = (prog, fill_unmatched(prog, match_groups(prog, gdesc)))
        self.alias = {}
        for cid, (prog, _lg) in self.progs.items():
            self.alias[cid] = cid
            if prog.get("vri"):
                self.alias[norm_controller(prog["vri"])] = cid
        self.state = {cid: {} for cid in self.progs}
        self.shown = {}
        self.online = set()

    def set(self, controller, group, state):
        cid = self.alias.get(controller)
        if cid is not None:
            self.state[cid][group] = state
            return cid
        return None

    def push(self, cids, traci):
        for cid in cids:
            prog, lg = self.progs[cid]
            s = link_states(len(prog["links"]), lg, self.state[cid], prog.get("yields") or [])
            if self.shown.get(cid) != s:
                traci.trafficlight.setRedYellowGreenState(cid, s)
                self.shown[cid] = s
                self.online.add(cid)

    def release(self, traci):
        """Back to the programmes (the feed stopped)."""
        for cid in sorted(self.online):
            try:
                traci.trafficlight.setProgram(cid, "dutch")
            except Exception:  # noqa: BLE001
                pass
        self.online.clear()
        self.shown.clear()


class ExactDriver:
    """Replay "exact": every recorded state at its time (simulation time 0 = the recorded start)."""

    def __init__(self, progs, rows, groups, start, log=_log):
        self.ap = _Applier(progs, groups, log)
        self.rows = [(max(0.0, (t - start).total_seconds()), c, g, s) for t, c, g, s in rows if
                     c in self.ap.alias and t <= start + timedelta(hours=6)]
        self.rows.sort()
        self.i = 0
        self.log = log
        log(f"traffic lights (real, exact): {len(self.ap.progs)} junctions, {len(self.rows)} state changes from "
            f"{start:%Y-%m-%d %H:%M}")

    def step(self, sim_t, traci):
        touched = set()
        while self.i < len(self.rows) and self.rows[self.i][0] <= sim_t:
            _t, c, g, s = self.rows[self.i]
            cid = self.ap.set(c, g, s)
            if cid:
                touched.add(cid)
            self.i += 1
        if touched:
            self.ap.push(touched, traci)

    def close(self, traci):
        pass


class LiveDriver:
    """Live: rows from a feed (NDJSON lines: a file being written, http(s) stream, tcp://host:port), read in a
    thread and set as they come. A feed that fails hands the junctions back to their programmes."""

    def __init__(self, progs, source, groups, log=_log, opener=None):
        import queue
        import threading
        self.ap = _Applier(progs, groups, log)
        self.q = queue.Queue()
        self.failed = None
        self.log = log
        self.n = 0
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._read, args=(source, opener), daemon=True)
        self.thread.start()

    def _lines(self, source):
        import time
        src = str(source)
        if src.startswith(("http://", "https://")):
            import requests
            from .realworld import UA
            with requests.get(src, stream=True, headers=UA, timeout=30) as r:
                r.raise_for_status()
                for ln in r.iter_lines(decode_unicode=True):
                    yield ln
        elif src.startswith("tcp://"):
            import socket
            host, port = src[6:].rsplit(":", 1)
            with socket.create_connection((host, int(port)), timeout=30) as s:
                f = s.makefile("r")
                for ln in f:
                    yield ln
        else:
            with open(Path(src).expanduser()) as f:         # a file being written: follow it
                while not self._stop.is_set():
                    ln = f.readline()
                    if ln:
                        yield ln
                    else:
                        time.sleep(0.05)

    def _read(self, source, opener):
        try:
            for ln in (opener(source) if opener else self._lines(source)):
                if self._stop.is_set():
                    return
                ln = ln.strip() if isinstance(ln, str) else ln
                if not ln:
                    continue
                for _t, c, g, s in read_rows([ln]):
                    self.q.put((c, g, s))
        except Exception as err:  # noqa: BLE001 - the feed is gone: the programmes take over
            self.failed = f"{type(err).__name__}: {err}"
        else:
            self.failed = self.failed or "feed ended"

    def step(self, sim_t, traci):
        touched = set()
        while not self.q.empty():
            c, g, s = self.q.get_nowait()
            cid = self.ap.set(c, g, s)
            if cid:
                touched.add(cid)
                self.n += 1
        if touched:
            self.ap.push(touched, traci)
        if self.failed and self.ap.online:
            self.log(f"traffic lights (live): feed stopped ({self.failed}); Dutch programmes take over")
            self.ap.release(traci)

    def close(self, traci):
        self._stop.set()


def driver(work, log=_log):
    """The TraCI driver of a trial (run.py calls step(sim_t, traci) after each SUMO step), or None: from
    work/signals.json written by signalplan.prepare."""
    p = Path(work) / "signals.json"
    if not p.exists():
        return None
    info = json.loads(p.read_text())
    spec = info.get("driver")
    if not spec:
        return None
    plan_path = Path(info["net"]).with_name(Path(info["net"]).name.replace(".net.xml", ".plan.json"))
    progs = json.loads(plan_path.read_text())["programs"]
    gpath = Path(spec.get("groups") or "")
    groups = {norm_controller(c): g for c, g in json.loads(gpath.read_text()).items()} \
        if spec.get("groups") and gpath.is_file() else {}
    try:
        if spec["kind"] == "exact":
            return ExactDriver(progs, read_rows(spec["log"]), groups, datetime.fromisoformat(spec["start"]), log)
        if spec["kind"] == "live":
            log(f"traffic lights (live): reading {spec['source']}")
            return LiveDriver(progs, spec["source"], groups, log)
    except Exception as err:  # noqa: BLE001 - the programmes run
        log(f"traffic lights: {spec['kind']} data not used ({type(err).__name__}: {err})")
    return None
