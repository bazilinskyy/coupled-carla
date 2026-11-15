"""The session dashboard: run a whole coupled experiment from a browser, as with coupled-sim's experimenter dashboard.

    python -m coupledcarla.session                         # open http://127.0.0.1:8790/
    python -m coupledcarla.session --host 0.0.0.0          # from tablets and other PCs (prints a link with a token)
    python -m coupledcarla.station --host <this machine>   # on every station PC: no --role, the dashboard gives it

Unlike `run ... --dashboard` (dashboard.py), which watches one host run, this page stays up for the whole session:

  Session    choose an experiment (experiments/, cities/, places/ from the map maker, twins) or make a map of any
             place (a name, an address or lat,lon[,radius]; the map maker builds it with its progress shown); the
             participant number (their counterbalanced trial plan, study.py); the CARLA server (started here, or one
             already running at host:port); quality, real time, host window. The experiment is validated
             (validate.py) and its problems shown in plain words.
  Stations   up to 8 stations that joined the lobby (machine, address, input devices, GPU, round trip), each given a
             role from the experiment's participants by dropdown, ready once its trial process waits; bot stations
             started from here for tests.
  Trial      Initialise (loads the trial: CARLA world, SUMO, agents; waits for the stations), Start, Pause, Next
             trial, End trial, Stop session; the plan with what is done.
  Live       a map of the place (its own roads and buildings over an open aerial photo where one exists, else
             OpenStreetMap) with every participant ringed and labelled, AVs, scripted cars, SUMO cars, bicycles,
             buses, AI walkers, the pedestrian crowd, drones with their height and traffic-light states; an agents
             table; eHMI states for the AVs, take-over requests, the weather and time of day, visual sync, events.
  After      every finished trial with its numbers, opened in the trial viewer or downloaded as a zip.

Each trial runs in its own host process (python -m coupledcarla.run --trial N --control-relay ...), so a trial that
crashes does not take the dashboard or the stations with it, and every trial starts fresh as run.py's trials do.
The stations stay connected to this process (hub.py) across trials and participants.

Security as dashboard.py: it listens on 127.0.0.1 unless --host says otherwise; then every request needs the token
printed at the start (the link carries it, and the page keeps it in a SameSite=Strict cookie for its links); commands
are POSTs with an X-Coupled-Carla header, which a page on another site cannot send; a local dashboard answers only
requests addressed to localhost. Station connections (port 2100) are the plain control channel, as with run.py.
"""
import argparse
import collections
import csv
import io
import json
import os
import re
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import webbrowser
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import yaml

from . import livemap
from .dashboard import COOKIE, LOCAL, _lan_ip, request_allowed
from .hub import MAX, Hub

ROOT = Path(__file__).resolve().parent.parent
PAGE = Path(__file__).resolve().parent / "data" / "session.html"
WINDOWS = os.name == "nt"
HOSTNAME = re.compile(r"^[A-Za-z0-9.\-:]{1,253}$")
TIME = re.compile(r"^([01]?\d|2[0-3]):[0-5]\d$")
COMMANDS = ("experiment", "participant", "options", "assign", "bot", "remove_bot", "initialise", "start", "pause",
            "resume", "next", "abort", "stop", "sync", "takeover", "ehmi", "weather", "place", "export", "refresh")
TRIAL_COMMANDS = ("start", "pause", "resume", "sync", "takeover", "ehmi", "weather")
DEFAULT_OPTIONS = {"server": "launch", "server_host": "127.0.0.1", "server_port": 2000, "quality": "Epic",
                   "real_time": True, "host_window": False, "bots_for_missing": True, "auto_next": False,
                   "duration": None}
PHASES = {"choose": "choose an experiment", "lobby": "lobby", "loading": "loading the trial",
          "ready": "ready to start", "running": "running", "paused": "paused", "ending": "ending the trial",
          "between": "between trials", "done": "all trials done", "failed": "trial failed",
          "stopped": "session stopped"}


class SessionError(Exception):
    """A refused command, in plain words, with what to do instead."""

    def __init__(self, msg, advice=""):
        super().__init__(msg)
        self.advice = advice


def _rel(path, root=ROOT):
    try:
        return Path(path).resolve().relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return str(path)


def list_experiments(root=ROOT):
    """Every experiment file the dashboard offers: [{path, label, group, map, built, roles, stations}]."""
    root = Path(root)
    out = []
    for group, folder in (("Experiments", root / "experiments"), ("Cities", root / "experiments" / "cities"),
                          ("Your places (map maker)", root / "experiments" / "places")):
        for f in sorted(folder.glob("*.yaml")) if folder.exists() else []:
            if f.name == "cities.yaml":
                continue
            try:
                raw = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
            except (OSError, yaml.YAMLError):
                continue
            if not isinstance(raw, dict) or not (raw.get("map") or raw.get("name")):
                continue
            m = raw.get("map") or {}
            name = m.get("name") or raw.get("name") or f.stem
            parts = [p for p in raw.get("participants") or [] if isinstance(p, dict)]
            city = raw.get("city") or {}
            g = "Digital twins" if (m.get("carla_level") or m.get("opendrive")) else group
            out.append({"path": _rel(f, root), "label": city.get("name") or raw.get("name") or f.stem,
                        "file": f.stem, "group": g, "map": name, "built": (root / "maps" / name / "meta.json").exists(),
                        "roles": len(parts), "stations": sum(1 for p in parts if p.get("station"))})
    return out


def experiment_roles(cfg):
    """The participants a station can take, from the top level and every trial's own list: [{role, kind, station}]."""
    from . import config
    seen = {}
    lists = [cfg.get("participants") or []]
    for t in cfg.get("trials") or []:
        lists.append(config.trial_config(cfg, t).get("participants") or [])
    for ps in lists:
        for p in ps:
            if isinstance(p, dict) and p.get("role") and p["role"] not in seen:
                seen[p["role"]] = {"role": p["role"], "kind": p.get("kind", "driver"), "station": bool(p.get("station")),
                                   "automated": bool(p.get("automation"))}
    return list(seen.values())


def child_args(exp_path, position, participant, order, link_port, token, station_roles, options):
    """The trial host's command line (after `python -m coupledcarla.run`)."""
    cmd = [exp_path, "--trial", str(position), "--participant", str(participant),
           "--control-relay", f"127.0.0.1:{link_port}", "--relay-token", token,
           "--station-roles=" + ",".join(sorted(station_roles)), "--quality", options["quality"],
           "--real-time", "on" if options["real_time"] else "off"]
    if order:
        cmd += ["--order", order]
    if not options["host_window"]:
        cmd.append("--headless")
    if options["server"] == "attach":
        cmd += ["--no-launch", "--server-host", options["server_host"], "--server-port", str(options["server_port"])]
    elif int(options["server_port"]) != 2000:
        cmd += ["--server-port", str(options["server_port"])]
    if options.get("duration"):
        cmd += ["--duration", str(options["duration"])]
    return cmd


def check_options(cur, body):
    """The options with the body's changes, each checked; raises SessionError in plain words."""
    out = dict(cur)
    for k, v in body.items():
        if k not in DEFAULT_OPTIONS:
            continue
        if k == "server":
            if v not in ("launch", "attach"):
                raise SessionError("The CARLA server is either started here or one that is already running.")
        elif k == "server_host":
            v = str(v).strip()
            if not HOSTNAME.match(v):
                raise SessionError(f"{v!r} is not a computer name or address.", "Type e.g. 127.0.0.1 or lin4080.")
        elif k == "server_port":
            try:
                v = int(v)
            except (TypeError, ValueError):
                v = 0
            if not 1024 <= v <= 65000:
                raise SessionError("The CARLA port is a number between 1024 and 65000 (usually 2000).")
        elif k == "quality":
            if v not in ("Epic", "Low"):
                raise SessionError("Quality is Epic or Low.")
        elif k == "duration":
            if v in (None, "", 0, "0"):
                v = None
            else:
                try:
                    v = float(v)
                except (TypeError, ValueError):
                    v = -1
                if not 5 <= v <= 7200:
                    raise SessionError("A trial lasts between 5 s and 2 hours.", "Leave it empty for the file's length.")
        else:
            v = bool(v)
        out[k] = v
    return out


def safe_trial_dir(rel, logs):
    """A trial folder named by the page, only inside logs/ and only one that has a config.yaml (else None)."""
    try:
        p = (Path(logs) / unquote(str(rel))).resolve()
        p.relative_to(Path(logs).resolve())
    except (ValueError, OSError):
        return None
    return p if p.is_dir() and (p / "config.yaml").exists() else None


def summarise_trial(d):
    """The numbers of a finished trial for the dashboard, from its summary.json."""
    try:
        s = json.loads((Path(d) / "summary.json").read_text())
    except (OSError, ValueError):
        return None
    st = s.get("station_stats") or {}
    return {"sim_s": s.get("sim_s"), "rtf": s.get("rtf"), "collisions": s.get("collisions"),
            "sumo": s.get("sumo_vehicles_spawned"), "stations": s.get("stations") or [], "aborted": s.get("aborted"),
            "takeovers": len(s.get("takeovers") or []), "rows": s.get("rows"),
            "rtt_ms": {r: v.get("rtt_ms") for r, v in st.items() if isinstance(v, dict)}}


class TrialRun:
    """One trial's host process and what it prints."""

    def __init__(self, cmd, env, log_path, position, trial):
        self.position, self.trial = position, trial
        self.log_path = log_path
        self.lines = collections.deque(maxlen=400)
        self.server_pids, self.log_dir = set(), None
        self.started, self.stop_sent = time.time(), None
        self.logf = open(log_path, "w", encoding="utf-8")
        self.logf.write(" ".join(cmd) + "\n")
        self.logf.flush()
        self.proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding="utf-8", errors="replace", bufsize=1,
                                     **({} if WINDOWS else {"start_new_session": True}))
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.logf.write(line)
            self.logf.flush()
            line = line.rstrip()
            self.lines.append(line)
            m = re.search(r"started Carla\w*\.(?:sh|exe) \(pid (\d+)\)", line)
            if m:
                self.server_pids.add(int(m.group(1)))
            m = re.search(r"\] trial \d+ -> (.+)$", line)
            if m:
                self.log_dir = m.group(1).strip()
        self.logf.close()

    def alive(self):
        return self.proc.poll() is None

    def last_line(self):
        for line in reversed(self.lines):
            text = re.sub(r"^\[\s*[\d.]+s\]\s*", "", line).strip()
            if text and not text.startswith(("Traceback", "  File")):
                return text
        return ""

    def error_line(self):
        for line in reversed(self.lines):
            if re.search(r"(Error|Exception|error:)", line):
                return line.strip()[:300]
        return self.last_line()[:300]

    def terminate(self, grace=30.0):
        """SIGTERM (run.py cleans up: CARLA server, SUMO), then a kill; then any CARLA server it started."""
        if self.alive():
            try:
                self.proc.terminate()
                self.proc.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                if WINDOWS:
                    subprocess.call(["taskkill", "/T", "/F", "/PID", str(self.proc.pid)], stdout=subprocess.DEVNULL)
                else:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(timeout=10)
        for pid in self.server_pids:       # a server left behind by a crashed host (its own process group)
            try:
                if WINDOWS:
                    subprocess.call(["taskkill", "/T", "/F", "/PID", str(pid)], stdout=subprocess.DEVNULL)
                else:
                    os.killpg(pid, signal.SIGKILL)
            except (OSError, ProcessLookupError):
                pass


class Session:
    def __init__(self, root=ROOT, stations_port=2100, log=print, carla_root=None, python=None):
        self.root = Path(root)
        self.logs = self.root / "logs"
        self.python = python or sys.executable
        self.lock = threading.RLock()
        self.messages = collections.deque(maxlen=60)          # what happened, for the page: [time, text]
        (self.logs / "sessions").mkdir(parents=True, exist_ok=True)
        self.stamp = time.strftime("%Y%m%d-%H%M%S")
        self.logf = open(self.logs / "sessions" / f"session_{self.stamp}.log", "a", encoding="utf-8")
        self._print = log
        self.options = dict(DEFAULT_OPTIONS)
        self.hub = Hub(stations_port, log=self.say, carla=self._carla_info(), on_host=self._from_host)
        self.experiments = list_experiments(self.root)
        self.exp = None
        self.participant, self.order = 0, None
        self.plan, self.position = [], 0
        self.run = None                # the current TrialRun
        self.status = self.live = None  # from the current trial's host
        self.finished = []
        self.after_exit = None          # "next": initialise the next trial when this one has ended
        self.stopped = False
        self.bots = {}                  # station id -> {proc, role, log}
        self.exports = {}               # trial dir (relative to logs) -> running | done | failed
        self.layers, self.layers_state = {}, {}
        self.validation = {"state": "none", "items": []}
        self.carla_root = carla_root or os.environ.get("CARLA_ROOT")
        if not self.carla_root:
            try:
                from .mapmaker import find_carla
                self.carla_root = find_carla()
            except Exception:  # noqa: BLE001 - the map maker's packages missing: no CARLA path
                self.carla_root = None
        self.jobs = None
        self._job_seen = None
        self.closed = False
        threading.Thread(target=self._watch, daemon=True).start()

    # -- log ----------------------------------------------------------------------------------------------------
    def say(self, text):
        line = f"{time.strftime('%H:%M:%S')} {text}"
        self.messages.append([time.strftime("%H:%M:%S"), str(text)])
        self.logf.write(line + "\n")
        self.logf.flush()
        self._print(f"[session] {text}")

    def _carla_info(self):
        o = self.options
        host = o["server_host"] if o["server"] == "attach" and o["server_host"] not in LOCAL else None
        return {"host": host, "port": int(o["server_port"])}

    # -- the trial host's messages (hub thread) -----------------------------------------------------------------
    def _from_host(self, msg):
        kind = msg.get("type")
        if kind == "status":
            self.status = msg.get("status")
            if self.run is not None and self.status and self.status.get("log_dir"):
                self.run.log_dir = self.status["log_dir"]
        elif kind == "live":
            self.live = msg

    # -- commands -----------------------------------------------------------------------------------------------
    def command(self, name, body):
        if name not in COMMANDS:
            raise SessionError("Unknown command.")
        body = body if isinstance(body, dict) else {}
        with self.lock:
            return getattr(self, f"cmd_{name}")(body) or {}

    def _trial_alive(self):
        return self.run is not None and self.run.alive()

    def _need_no_trial(self, what):
        if self._trial_alive():
            raise SessionError(f"{what} only between trials.", "End the trial (or wait until it is over) first.")

    def cmd_refresh(self, body):
        self.experiments = list_experiments(self.root)

    def cmd_experiment(self, body):
        self._need_no_trial("The experiment can be changed")
        path = str(body.get("path") or "")
        if path not in {e["path"] for e in self.experiments}:
            self.experiments = list_experiments(self.root)
            if path not in {e["path"] for e in self.experiments}:
                raise SessionError("That experiment file is not in experiments/.", "Pick one from the list.")
        from . import config
        try:
            cfg = config.load(self.root / path)
        except Exception as err:  # noqa: BLE001 - a broken YAML file: say so
            raise SessionError(f"The experiment file cannot be read: {err}", "Fix the file, then choose it again.")
        info = next(e for e in self.experiments if e["path"] == path)
        ehmi = (cfg.get("ehmi") or {}).get("states") or {"driving": 0, "yielding": 0, "go": 0}
        riders = {p.get("agent") for p in cfg.get("participants") or [] if p.get("kind") == "av_passenger"}
        self.exp = {"path": path, "name": cfg["name"], "map": cfg["map"]["name"], "built": info["built"],
                    "label": info["label"], "roles": experiment_roles(cfg),
                    "country": (cfg.get("city") or {}).get("country"),
                    "ehmi_states": ["off"] + [s for s in ehmi if s != "off"],
                    "ehmi_agents": [a["id"] for a in cfg.get("agents") or [] if isinstance(a, dict) and
                                    a.get("kind", "car") == "car" and (a.get("ehmi", "off") != "off" or
                                                                       a.get("id") in riders)],
                    "weather": cfg["simulation"].get("weather") if isinstance(cfg["simulation"].get("weather"), str)
                    else "custom", "time": cfg["simulation"].get("time"), "_cfg": cfg}
        self.position, self.stopped = 0, False
        self._replan()
        roles = {r["role"] for r in self.exp["roles"]}
        for st in self.hub.stations_view():
            if st["role"] and st["role"] not in roles and not st["fixed"]:
                self.hub.assign(st["id"], None)
        self.say(f"experiment {path}: {len(self.plan)} trials, roles {', '.join(sorted(roles)) or 'none'}")
        threading.Thread(target=self._validate, args=(path, cfg, info["built"]), daemon=True).start()
        if info["built"]:
            self._load_layers(self.exp["map"], self.exp["country"])

    def _replan(self):
        from . import study
        try:
            self.plan = study.plan(self.exp["_cfg"], self.participant, self.order)
        except (KeyError, ValueError) as err:
            self.plan = []
            self.validation = {"state": "done", "items": [["error", "design", f"no trial plan: {err}"]]}

    def _validate(self, path, cfg, built):
        self.validation = {"state": "running", "items": []}
        items = []
        try:
            from . import validate
            raw = yaml.safe_load((self.root / path).read_text(encoding="utf-8"))
            report, _ = validate.check(raw, cfg, roads=built, participants=[])
            items = [list(i) for i in report.items]
        except Exception as err:  # noqa: BLE001 - the validator itself failed: show it, do not block
            items = [["warning", "validator", f"could not check the file ({type(err).__name__}: {err})"]]
        if not built:
            items.insert(0, ["warning", "map", f"the map {cfg['map']['name']} is not built yet: Initialise builds it "
                                               "first, which takes a few minutes (or use the map maker)"])
        if self.exp and self.exp["path"] == path:
            self.validation = {"state": "done", "items": items}

    def _load_layers(self, name, country=None):
        if name in self.layers or self.layers_state.get(name) == "running":
            return
        self.layers_state[name] = "running"

        def work():
            try:
                self.layers[name] = livemap.map_layers(self.root / "maps" / name, country)
                self.layers_state[name] = "done"
            except Exception as err:  # noqa: BLE001 - no map underlay: the live map still shows the actors
                self.layers_state[name] = "failed"
                self.say(f"map layers of {name}: {type(err).__name__}: {err}")
        threading.Thread(target=work, daemon=True).start()

    def cmd_participant(self, body):
        self._need_no_trial("The participant can be changed")
        try:
            n = int(body.get("participant", 0))
        except (TypeError, ValueError):
            n = -1
        if not 0 <= n <= 9999:
            raise SessionError("The participant number is a whole number from 0 (the file's order) up.")
        order = body.get("order") or None
        from . import study
        if order is not None and order not in study.ORDERS:
            raise SessionError(f"The order is one of {', '.join(study.ORDERS)}.")
        self.participant, self.order = n, order
        self.position, self.stopped = 0, False
        if self.exp:
            self._replan()
        self.say(f"participant {n}" + (f", order {order}" if order else ""))

    def cmd_options(self, body):
        new = check_options(self.options, body)
        if self._trial_alive() and any(new[k] != self.options[k] for k in new if k not in ("auto_next",)):
            raise SessionError("These settings apply from the next trial on.", "Change them between trials.")
        self.options = new
        self.hub.carla = self._carla_info()

    def _roles(self):
        return {r["role"] for r in (self.exp or {}).get("roles", [])}

    def cmd_assign(self, body):
        self._need_no_trial("Roles can be changed")
        sid, role = str(body.get("station") or ""), body.get("role") or None
        rows = {s["id"]: s for s in self.hub.stations_view()}
        if sid not in rows and sid not in self.bots:
            raise SessionError("That station is not connected any more.")
        if sid in rows and rows[sid]["fixed"]:
            raise SessionError("This station was started with --role: its role is fixed.",
                               "Start it without --role to choose its role here.")
        if role is not None and role not in self._roles():
            raise SessionError(f"The experiment has no participant {role!r}.")
        if role is not None:
            for other in rows.values():
                if other["role"] == role and other["id"] != sid:
                    if other["fixed"]:
                        raise SessionError(f"{role} is taken by a station started with --role {role}.")
                    self.hub.assign(other["id"], None)
                    self.say(f"{other['name']}: no role ({role} went to another station)")
        self.hub.assign(sid, role)
        self.say(f"{rows.get(sid, {}).get('name', sid)}: {role or 'no role'}")

    def cmd_bot(self, body):
        role = body.get("role") or None
        if role is not None and role not in self._roles():
            raise SessionError(f"The experiment has no participant {role!r}.")
        if len({s["id"] for s in self.hub.stations_view()} | set(self.bots)) >= MAX:
            raise SessionError(f"The session has {MAX} stations already.", "Remove one first.")
        n = 1
        while f"bot-{n}" in self.bots:
            n += 1
        sid = f"bot-{n}"
        if role is not None:
            self._need_no_trial("Roles can be changed")
            for other in self.hub.stations_view():
                if other["role"] == role and not other["fixed"]:
                    self.hub.assign(other["id"], None)
            self.hub.assign(sid, role)
        cmd = [self.python, "-m", "coupledcarla.station", "--host", "127.0.0.1", "--control-port", str(self.hub.port),
               "--input", "bot", "--headless", "--name", sid, "--station-id", sid, "--size", "640x360",
               "--camera-hz", "5", "--out", str(self.logs)]
        log = self.logs / "sessions" / f"{self.stamp}_{sid}.log"
        f = open(log, "w", encoding="utf-8")
        proc = subprocess.Popen(cmd, cwd=self.root, stdout=f, stderr=subprocess.STDOUT, env=self._env(),
                                **({} if WINDOWS else {"start_new_session": True}))
        self.bots[sid] = {"proc": proc, "role": role, "log": str(log), "file": f}
        self.say(f"bot station {sid} started" + (f" as {role}" if role else ""))
        return {"station": sid}

    def cmd_remove_bot(self, body):
        sid = str(body.get("station") or "")
        if sid not in self.bots:
            raise SessionError("Only bot stations started here can be removed here.")
        self._need_no_trial("Bot stations can be removed")
        self._kill_bot(sid)
        self.hub.assign(sid, None)
        self.say(f"bot station {sid} removed")

    def _kill_bot(self, sid):
        b = self.bots.pop(sid, None)
        if b is None:
            return
        if b["proc"].poll() is None:
            try:
                if WINDOWS:
                    subprocess.call(["taskkill", "/T", "/F", "/PID", str(b["proc"].pid)], stdout=subprocess.DEVNULL)
                else:
                    os.killpg(b["proc"].pid, signal.SIGTERM)
                b["proc"].wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                b["proc"].kill()
        b["file"].close()

    def _env(self):
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
        try:
            from .mapmaker import sumo_env
            env = dict(sumo_env(), PYTHONUNBUFFERED="1")
        except Exception:  # noqa: BLE001 - the environment as it is
            pass
        if self.carla_root:
            env["CARLA_ROOT"] = str(self.carla_root)
        return env

    def station_roles(self):
        """Roles with a station for the next trial: those given out here, plus (when missing roles do not run as
        bots) every station participant of the file, which the host then waits for."""
        given = {s["role"] for s in self.hub.stations_view() if s["role"]} & self._roles()
        if not self.options["bots_for_missing"]:
            given |= {r["role"] for r in self.exp["roles"] if r["station"]}
        return given

    def cmd_initialise(self, body):
        if not self.exp:
            raise SessionError("Choose an experiment first.")
        self._need_no_trial("A trial can be loaded")
        if self.position >= len(self.plan):
            raise SessionError("Every trial of this participant's plan is done.",
                               "Enter the next participant's number to start their plan.")
        errors = [i for i in self.validation["items"] if i[0] == "error"]
        if errors:
            raise SessionError(f"The experiment file has {len(errors)} error(s).",
                               "Fix them in the file (see Problems), then choose the experiment again.")
        trial = self.plan[self.position]
        if self.participant and self.position == 0:
            self._session_file()
        cmd = [self.python, "-u", "-m", "coupledcarla.run",
               *child_args(self.exp["path"], self.position, self.participant, self.order, self.hub.link_port,
                           self.hub.token, self.station_roles(), self.options)]
        log = self.logs / "sessions" / f"{self.stamp}_P{self.participant:02d}_pos{self.position}.log"
        self.status = self.live = None
        self.after_exit, self.stopped = None, False
        self.run = TrialRun(cmd, self._env(), log, self.position, trial)
        self.say(f"loading trial {self.position + 1} of {len(self.plan)} (file trial {trial.get('_index')}"
                 f"{', ' + trial['_condition'] if trial.get('_condition') else ''}); host log {_rel(log, self.root)}")

    def _session_file(self):
        from . import study
        sdir = self.logs / self.exp["name"]
        sdir.mkdir(parents=True, exist_ok=True)
        rows = study.session_rows(self.plan)
        with open(sdir / f"session_P{self.participant:02d}_{time.strftime('%Y%m%d-%H%M%S')}.csv", "w",
                  newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]), delimiter=";")
            w.writeheader()
            w.writerows(rows)

    def _send(self, cmd):
        if not self._trial_alive():
            raise SessionError("No trial is loaded.", "Press Initialise first.")
        if not self.hub.command(cmd):
            raise SessionError("The trial is still loading (CARLA world, traffic).", "Wait a moment and try again.")

    def cmd_start(self, body):
        if (self.status or {}).get("state") not in ("ready",):
            raise SessionError("The trial is not ready yet." if self._trial_alive() else "No trial is loaded.",
                               "Wait until the stations are in and the state says ready to start.")
        self._send({"cmd": "start"})

    def cmd_pause(self, body):
        self._send({"cmd": "pause"})

    def cmd_resume(self, body):
        self._send({"cmd": "resume"})

    def cmd_sync(self, body):
        self._send({"cmd": "sync"})

    def cmd_takeover(self, body):
        role = body.get("role") or None
        if role is not None and role not in ((self.status or {}).get("automated") or []):
            raise SessionError(f"{role} does not drive automated in this trial.")
        self._send({"cmd": "takeover", **({"role": role} if role else {})})

    def cmd_ehmi(self, body):
        states = ((self.live or {}).get("ehmi") or {}).get("states") or (self.exp or {}).get("ehmi_states") or []
        state = str(body.get("state") or "")
        if state not in states:
            raise SessionError(f"The experiment has no eHMI state {state!r}.")
        agent = body.get("agent") or None
        if agent is not None and not isinstance(agent, str):
            raise SessionError("Unknown car.")
        self._send({"cmd": "ehmi", "state": state, **({"agent": agent[:64]} if agent else {})})
        self.say(f"eHMI {state}" + (f" on {agent}" if agent else " on every AV"))

    def cmd_weather(self, body):
        preset = body.get("preset") or None
        tod = (body.get("time") or "").strip() or None
        if preset is not None and preset not in livemap.WEATHER_PRESETS:
            raise SessionError("Unknown weather.", "Pick one from the list.")
        if tod is not None and not TIME.match(tod):
            raise SessionError("The time of day is hours:minutes, e.g. 14:30.")
        if not preset and not tod:
            raise SessionError("Choose a weather or a time of day.")
        self._send({"cmd": "weather", **({"preset": preset} if preset else {}), **({"time": tod} if tod else {})})
        self.say(f"weather {preset or 'as is'}{' at ' + tod if tod else ''}")

    def cmd_abort(self, body):
        self._end("abort")

    def cmd_stop(self, body):
        self.stopped = True
        self._end("stop")

    def _end(self, how):
        if not self._trial_alive():
            raise SessionError("No trial is loaded.")
        self.run.stop_sent = time.time()
        if not self.hub.command({"cmd": how}):          # still loading: end the host process itself
            threading.Thread(target=self.run.terminate, daemon=True).start()
        self.say("ending the trial" if how == "abort" else "stopping the session after this trial")

    def cmd_next(self, body):
        if not self.exp:
            raise SessionError("Choose an experiment first.")
        if self._trial_alive():
            self.after_exit = "next"
            self._end("abort")
            return {"note": "The trial ends; the next one loads after it."}
        last = self.finished[-1] if self.finished else None
        if last and not last["ok"] and last["position"] == self.position:
            self.position += 1                       # skip the trial that failed
        self.cmd_initialise({})

    def cmd_place(self, body):
        from . import mapmaker as mm
        from . import mapmaker_web
        text = str(body.get("text") or "").strip()
        if not text:
            raise SessionError("Type a place: a name, an address or lat,lon[,radius in m].")
        if self.jobs is None:
            self.jobs = mapmaker_web.Jobs()
        m = re.fullmatch(r"\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*(\d+(?:\.\d+)?)\s*(?:m)?)?\s*", text)
        size = str(body.get("size") or "medium")
        req = {"size": size, "query": text}
        if m:
            req.update(lat=float(m.group(1)), lon=float(m.group(2)))
            if m.group(3):
                req["size"] = str(max(0.3, min(4.0, 2 * float(m.group(3)) / 1000.0)))
        else:
            try:
                found = mm.search(text, limit=1)
            except mm.MapMakerError as err:
                raise SessionError(str(err), err.advice)
            if not found:
                raise SessionError(f"I found no place called {text!r}.", "Try the town's name or lat,lon.")
            req["found"] = found[0]
        try:
            job = self.jobs.start(req)
        except mm.MapMakerError as err:
            raise SessionError(str(err), err.advice)
        self._job_seen = None
        self.say(f"making a map of {job['plan']['place']['full'] if job['plan']['place'].get('full') else text}")

    def cmd_export(self, body):
        d = safe_trial_dir(body.get("dir"), self.logs)
        if d is None:
            raise SessionError("That trial folder is not in logs/.")
        rel = _rel(d, self.logs)
        if (d / "trial.json").exists():
            self.exports[rel] = "done"
            return {"viewer": viewer_url(rel)}
        if self.exports.get(rel) == "running":
            return {}
        self.exports[rel] = "running"

        def work():
            code = subprocess.call([self.python, "-m", "coupledcarla.export_trial", str(d)], cwd=self.root,
                                   env=self._env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.exports[rel] = "done" if code == 0 and (d / "trial.json").exists() else "failed"
        threading.Thread(target=work, daemon=True).start()
        return {}

    # -- watching the trial process, bots and the map maker -----------------------------------------------------
    def _watch(self):
        while not self.closed:
            time.sleep(0.3)
            try:
                with self.lock:
                    self._check_run()
                    for sid, b in list(self.bots.items()):
                        if b["proc"].poll() is not None:
                            self.say(f"bot station {sid} stopped (code {b['proc'].returncode}; log {b['log']})")
                            b["file"].close()
                            self.bots.pop(sid, None)
                    self._check_job()
            except Exception as err:  # noqa: BLE001 - the watcher must keep going
                self.say(f"watcher: {type(err).__name__}: {err}")

    def _check_run(self):
        run = self.run
        if run is None or run.alive():
            if run is not None and run.stop_sent and time.time() - run.stop_sent > 90 and run.alive():
                self.say("the trial's host did not end within 90 s: stopping it")
                run.stop_sent = time.time() + 1e6
                threading.Thread(target=run.terminate, daemon=True).start()
            return
        code = run.proc.returncode
        d = (self.root / run.log_dir) if run.log_dir and not Path(run.log_dir).is_absolute() else \
            (Path(run.log_dir) if run.log_dir else None)
        summ = summarise_trial(d) if d else None
        ok = code == 0 and summ is not None
        rec = {"position": run.position, "index": run.trial.get("_index"), "condition": run.trial.get("_condition"),
               "name": run.trial.get("name"), "participant": self.participant, "code": code, "ok": ok,
               "dir": _rel(d, self.logs) if d else None, "summary": summ, "log": _rel(run.log_path, self.root),
               "started": time.strftime("%H:%M:%S", time.localtime(run.started)),
               "wall_s": round(time.time() - run.started)}
        if not ok:
            rec["error"] = run.error_line()
        self.finished.append(rec)
        self.run = None
        self.live = None
        if ok:
            self.position += 1
            if (summ or {}).get("aborted") == "session stopped":
                self.stopped = True
            self.say(f"trial {run.position + 1} finished: {summ['sim_s']} s, real-time factor {summ['rtf']}, "
                     f"{summ['collisions']} collisions" + (f" ({summ['aborted']})" if summ.get("aborted") else ""))
        else:
            self.say(f"trial {run.position + 1} failed (exit code {code}): {rec['error']}")
        nxt = self.after_exit == "next" or (self.options["auto_next"] and ok and not self.stopped)
        self.after_exit = None
        if nxt and self.position < len(self.plan) and not self.stopped:
            if not ok:
                self.position += 1
            if self.position < len(self.plan):
                try:
                    self.cmd_initialise({})
                except SessionError as err:
                    self.say(f"next trial not loaded: {err}")

    def _check_job(self):
        if self.jobs is None:
            return
        j = self.jobs.job
        if j is None or j["state"] == "running" or self._job_seen == j["id"]:
            return
        self._job_seen = j["id"]
        if j["state"] == "done":
            self.experiments = list_experiments(self.root)
            path = j["plan"]["experiment"].replace(os.sep, "/")
            self.say(f"map ready: {path}")
            if not self._trial_alive():
                try:
                    self.cmd_experiment({"path": path})
                except SessionError as err:
                    self.say(str(err))
        else:
            self.say(f"map maker: {j.get('error')} {j.get('advice') or ''}")

    # -- the page's view ------------------------------------------------------------------------------------------
    def phase(self):
        if self.run is not None and self.run.alive():
            if self.run.stop_sent:
                return "ending"
            st = (self.status or {}).get("state")
            return st if st in ("ready", "running", "paused") else "loading"
        if not self.exp:
            return "choose"
        if self.stopped:
            return "stopped"
        if self.position >= len(self.plan) and self.plan:
            return "done"
        if self.finished and not self.finished[-1]["ok"]:
            return "failed"
        return "between" if self.finished else "lobby"

    def state(self):
        with self.lock:
            phase = self.phase()
            run = self.run
            stations = self.hub.stations_view()
            ids = {s["id"] for s in stations}
            for sid, b in self.bots.items():
                for s in stations:
                    if s["id"] == sid:
                        s["bot"] = True
                if sid not in ids:
                    stations.append({"id": sid, "name": sid, "ip": "127.0.0.1", "info": {"bot": True}, "rtt_ms": None,
                                     "role": b["role"], "fixed": False, "ready": False, "starting": True, "bot": True})
            exp = {k: v for k, v in self.exp.items() if not k.startswith("_")} if self.exp else None
            if exp:
                exp["layers"] = self.layers_state.get(exp["map"], "none")
            past = []
            if self.exp:
                d = self.logs / self.exp["name"]
                for t in sorted((x for x in d.glob("*_trial*") if x.is_dir()), reverse=True)[:12] if d.exists() else []:
                    past.append({"dir": _rel(t, self.logs), "summary": summarise_trial(t),
                                 "viewer": viewer_url(_rel(t, self.logs)) if (t / "trial.json").exists() else None})
            for f in self.finished:
                if f["dir"] and self.exports.get(f["dir"]) == "done":
                    f["viewer"] = viewer_url(f["dir"])
            return {
                "phase": phase, "phase_text": PHASES.get(phase, phase),
                "detail": run.last_line() if run is not None and phase == "loading" else "",
                "experiments": self.experiments, "experiment": exp, "participant": self.participant,
                "order": self.order, "plan": [{"position": t["_position"], "index": t["_index"],
                                               "condition": t.get("_condition") or t.get("name") or "",
                                               "block": t.get("_block"), "duration_s": t.get("duration_s", 300)}
                                              for t in self.plan],
                "position": self.position, "options": self.options, "validation": self.validation,
                "carla_root": self.carla_root, "stations": stations, "stations_port": self.hub.port,
                "lan_ip": _lan_ip(), "trial": self.status if run is not None or phase in ("between", "done") else None,
                "live": self.live, "log": list(run.lines)[-14:] if run is not None else [],
                "finished": self.finished[-30:], "past": past, "exports": self.exports,
                "mapjob": self.jobs.view() if self.jobs is not None and self.jobs.job is not None else None,
                "messages": list(self.messages)[-25:], "weather_presets": livemap.WEATHER_PRESETS,
                "orders": ["latin", "full", "random", "none"]}

    def close(self):
        self.closed = True
        with self.lock:
            run = self.run
        if run is not None and run.alive():
            self.hub.command({"cmd": "stop"})
            try:
                run.proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                pass
            run.terminate()
        for sid in list(self.bots):
            self._kill_bot(sid)
        self.hub.close()
        self.logf.close()


def viewer_url(rel):
    return f"/viewer/?file=/logs/{rel}/trial.json"


def zip_trial(d):
    """A trial folder as a zip file (a spooled temporary file, rewound)."""
    buf = tempfile.SpooledTemporaryFile(max_size=64 * 1024 * 1024)
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(Path(d).rglob("*")):
            if f.is_file():
                z.write(f, f"{Path(d).name}/{f.relative_to(d).as_posix()}")
    buf.seek(0, io.SEEK_END)
    size = buf.tell()
    buf.seek(0)
    return buf, size


def make_handler(session, local, token):
    viewer_dir = (ROOT / "viewer").resolve()
    logs = session.logs

    class Handler(BaseHTTPRequestHandler):
        server_version = "coupled-carla-session"

        def log_message(self, *a):
            pass

        def _allowed(self):
            return request_allowed(self, local, token, cookie=True)

        def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
            data = body if isinstance(body, bytes) else (
                body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False, default=str).encode())
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):  # noqa: N802
            if not self._allowed():
                return self._send(403, {"error": "forbidden"})
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            path = u.path
            if path in ("/", "/index.html"):
                extra = {}
                if token:          # the page's links (viewer, zip) carry the token in a cookie
                    extra["Set-Cookie"] = f"{COOKIE}={token}; Path=/; HttpOnly; SameSite=Strict"
                return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8", extra)
            if path == "/api/state":
                return self._send(200, session.state())
            if path == "/api/layers":
                name = q.get("map", "")
                lay = session.layers.get(name)
                return self._send(200, lay) if lay else self._send(404, {"error": "not ready"})
            if path == "/api/zip":
                d = safe_trial_dir(q.get("dir"), logs)
                if d is None:
                    return self._send(404, {"error": "no such trial"})
                buf, size = zip_trial(d)
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Length", str(size))
                self.send_header("Content-Disposition", f'attachment; filename="{d.name}.zip"')
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                while True:
                    chunk = buf.read(1 << 20)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                buf.close()
                return None
            if path == "/viewer" or path.startswith("/viewer/"):
                rel = path[len("/viewer/"):] or "index.html"
                f = (viewer_dir / unquote(rel)).resolve()
                if viewer_dir in f.parents and f.is_file():
                    ctype = {".html": "text/html", ".json": "application/json", ".js": "text/javascript",
                             ".css": "text/css", ".png": "image/png"}.get(f.suffix, "application/octet-stream")
                    return self._send(200, f.read_bytes(), ctype + ("; charset=utf-8" if "text" in ctype else ""))
                return self._send(404, {"error": "not found"})
            if path.startswith("/logs/") and path.endswith("/trial.json"):
                d = safe_trial_dir(path[len("/logs/"):-len("/trial.json")], logs)
                if d is not None and (d / "trial.json").exists():
                    return self._send(200, (d / "trial.json").read_bytes())
                return self._send(404, {"error": "not found"})
            return self._send(404, {"error": "not found"})

        def do_POST(self):  # noqa: N802
            if not self._allowed() or self.headers.get("X-Coupled-Carla") != "1":
                return self._send(403, {"error": "forbidden"})
            path = urlparse(self.path).path
            name = path.rsplit("/", 1)[-1]
            if not path.startswith("/api/") or name not in COMMANDS:
                return self._send(404, {"error": "unknown command"})
            try:
                n = min(int(self.headers.get("Content-Length") or 0), 16384)
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._send(400, {"error": "That request was not understood."})
            try:
                return self._send(200, {"ok": True, **session.command(name, body)})
            except SessionError as err:
                return self._send(409, {"error": str(err), "advice": err.advice})
            except Exception as err:  # noqa: BLE001 - shown on the page in plain words
                session.say(f"{name} failed: {type(err).__name__}: {err}")
                return self._send(500, {"error": f"That did not work ({type(err).__name__}: {err}).",
                                        "advice": "The session log (logs/sessions/) has the details."})

    return Handler


def serve(session, host="127.0.0.1", port=8790, token=None):
    """Start the page's server in a thread; returns (httpd, url)."""
    local = host in LOCAL
    token = None if local else (token or secrets.token_urlsafe(12))
    server = ThreadingHTTPServer
    if ":" in host:
        server = type("ThreadingHTTPServer6", (ThreadingHTTPServer,), {"address_family": socket.AF_INET6})
    httpd = server((host, port), make_handler(session, local, token))
    httpd.daemon_threads = True
    shown = "127.0.0.1" if local and host != "::1" else (host if host not in ("0.0.0.0", "::") else _lan_ip())
    shown = f"[{shown}]" if ":" in shown else shown
    url = f"http://{shown}:{httpd.server_address[1]}/" + (f"?token={token}" if token else "")
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, url


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1", help="where the page listens (0.0.0.0: every network, with a token)")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--stations-port", type=int, default=2100, help="the control channel stations connect to")
    ap.add_argument("--experiment", help="choose this experiment file at the start")
    ap.add_argument("--participant", type=int, default=0)
    ap.add_argument("--carla-root", help="the CARLA folder to start the server from (default: CARLA_ROOT)")
    ap.add_argument("--attach", metavar="HOST:PORT", help="use the CARLA server already running there")
    ap.add_argument("--carla-port", type=int, help="the port of the CARLA server started here (default 2000)")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    session = Session(ROOT, args.stations_port, carla_root=args.carla_root)
    if args.attach:
        h, _, p = args.attach.rpartition(":")
        session.cmd_options({"server": "attach", "server_host": h or "127.0.0.1", "server_port": p})
    elif args.carla_port:
        session.cmd_options({"server_port": args.carla_port})
    if args.participant:
        session.cmd_participant({"participant": args.participant})
    if args.experiment:
        session.cmd_experiment({"path": _rel(Path(args.experiment).resolve())})
    httpd, url = serve(session, args.host, args.port)
    session.say(f"session dashboard: {url}")
    session.say(f"stations: python -m coupledcarla.station --host {_lan_ip()}"
                + (f" --control-port {args.stations_port}" if args.stations_port != 2100 else "")
                + "   (no --role: give each its role on the page)")
    if not args.no_browser and args.host in LOCAL:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        session.say("closing: ending the trial, stopping the bot stations")
    finally:
        session.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
