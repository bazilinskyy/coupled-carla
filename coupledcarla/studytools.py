"""The study tools' hooks into the host (run.py) and the station (station.py): continuous responses
(responses.py), take-over requests (takeover.py) and the experimenter dashboard (dashboard.py).

HostStudy lives for one trial on the host; run.py calls it at a few points of its loop:
  welcome(part)          extra welcome fields for a station (its response and automation settings)
  spawn(...)             the car of an automated driver (an ai.ScriptedCar on the participant's route)
  bind(...)              after spawning: the command queue, the logger, the stations' stats
  ready()                the dashboard's "Start trial" (when it gates trials); False: skip the trial
  hold()                 blocks while the dashboard paused the trial
  intercept(part, ...)   a host-local driver's control while the automation drives (take-over detection)
  step(t, ...)           after the AI agents: automated cars, requests, dashboard commands
  on_message(msg, ...)   station messages it handles (responses, take-over, engage); True when handled
  log(t, frame)          after the frame is logged: responses.csv, the dashboard's status
  close(summary)         take-overs and responses into summary.json
StationStudy does the same at a station: samples the responses, decides whether the station drives, draws.
"""
import collections
import math
import time

from . import responses, takeover
from .takeover import AUTOMATED, MANUAL, REQUESTED

ENGAGE_KEYS = ("return", "enter")        # pygame key names of the automation button on the keyboard
REQUEST_KEY = "t"                        # the experimenter's take-over request at the participant's machine


def _buttons(sp):
    b = (sp or {}).get("takeover", {}).get("button")
    if b is None:
        return [takeover.WHEEL_BUTTON, takeover.PAD_BUTTON]
    return list(b) if isinstance(b, (list, tuple)) else [b]


def keys_pressed(events, sp):
    """(automation button pressed, experimenter request pressed) among this frame's pygame events."""
    if not events:
        return False, False
    import pygame
    button = request = False
    for e in events:
        if e.type == pygame.KEYDOWN:
            name = pygame.key.name(e.key)
            button = button or name in ENGAGE_KEYS
            request = request or name == REQUEST_KEY
        elif e.type == pygame.JOYBUTTONDOWN and e.button in _buttons(sp):
            button = True
    return button, request


def _speed(actor):
    v = actor.get_velocity()
    return math.hypot(v.x, v.y)


class HostStudy:
    def __init__(self, tc, trial, parts, out, log=print, control=None, dashboard=None, plan=None):
        self.tc, self.trial, self.parts, self.out, self.say = tc, trial, parts, out, log
        self.control, self.dash, self.plan = control, dashboard, plan or []
        self.by_role = {p.role: p for p in parts}
        self.rspec = {p.role: s for p in parts if (s := responses.spec(p.cfg.get("response")))}
        self.aspec = {p.role: s for p in parts
                      if p.kind == "driver" and (s := takeover.spec(p.cfg.get("automation")))}
        self.cars, self.tor, self.fired = {}, {}, set()
        self.responses = responses.ResponseLog(out / "responses.csv", self.rspec) if self.rspec or control else None
        self.tor_file = self.tor_csv = None
        self.logger = self.queue = self.view = self.sync_cb = None
        self.station_stats = {}
        self.events = collections.deque(maxlen=40)
        self.collisions = 0
        self.paused = self.abort = self.stop = False
        self.state = "loading"
        self.t, self.frame, self.n = 0.0, 0, 0
        self.t_wall0 = None
        self.last_local = time.time()
        if self.dash is not None:
            self.dash.set_status(self.status())

    # -- set-up ---------------------------------------------------------------------------------------------
    def welcome(self, part):
        out = {}
        if part.role in self.rspec:
            out["response"] = self.rspec[part.role]
        if part.role in self.aspec:
            out["automation"] = self.aspec[part.role]
        return out

    def spawn(self, world, geo, part, planner=None):
        """The automated driver's car, at the start of its route; None for any other participant."""
        sp = self.aspec.get(part.role)
        if not sp:
            return None
        if not sp.get("route") or len(sp["route"]) < 2:
            raise ValueError(f"{part.role}: automation needs a route of at least two [lat, lon] points")
        from .ai import ScriptedCar
        cfg = {"id": part.role, "vehicle": part.cfg.get("vehicle", "vehicle.lincoln.mkz_2020"),
               "color": part.cfg.get("color"), "route": sp["route"], "speed_kmh": sp["speed_kmh"],
               "accel": sp["accel"], "yield_to_pedestrians": sp["yield_to_pedestrians"],
               "triggers": sp.get("triggers") or [], "ehmi": "off", "at_end": "stop"}
        car = ScriptedCar(world, geo, cfg, planner)
        self.cars[part.role] = car
        self.say(f"{part.role}: automated along {car.route_length():.0f} m at {sp['speed_kmh']:g} km/h; "
                 f"take-over at {sp['takeover'].get('at_s')} s, budget {sp['takeover']['budget_s']:g} s")
        return car.vehicle

    def bind(self, logger, queue, station_stats=None, sync_cb=None, view=None):
        self.logger, self.queue, self.view, self.sync_cb = logger, queue, view, sync_cb
        self.station_stats = station_stats if station_stats is not None else {}
        logger.listeners.append(self._on_event)
        if self.cars:
            import carla
            self.tor_file, self.tor_csv = takeover.open_csv(self.out / "takeover.csv")
        for role, car in self.cars.items():
            vid = car.vehicle.id
            car.apply = lambda c, i=vid: queue(carla.command.ApplyVehicleControl(i, c))
            self.tor[role] = takeover.TakeOver(
                role, self.aspec[role], lambda t, name, detail, i=vid: logger.event(t, self.frame, name, i, detail),
                self.tor_csv, where=lambda v=car.vehicle: self._where(v))
            part = self.by_role[role]
            if hasattr(part.input, "send"):      # keyboard lights only while the participant drives
                part.input.send = lambda c, r=role: queue(c) if self.tor[r].state == MANUAL else None
        for p in self.parts:
            if p.role in self.rspec and not p.station:
                bot = (p.cfg.get("bot") or {}).get("response")
                self.responses.local[p.role] = responses.ResponseInput(self.rspec[p.role], bot)

    @staticmethod
    def _where(vehicle):
        loc = vehicle.get_location()
        return 3.6 * _speed(vehicle), loc.x, loc.y

    def _on_event(self, ts, frame, name, actor_id, detail):
        if name == "spawn" or name == "station_stats":
            return
        if name == "collision":
            self.collisions += 1
        self.events.append([round(ts, 2), name, str(detail)[:140]])

    # -- trial flow -----------------------------------------------------------------------------------------
    def ready(self):
        """Waits for the dashboard's Start trial when it gates trials. False: the experimenter ended the trial or
        the session instead."""
        self.state = "ready"
        if self.dash is None or not (self.tc.get("dashboard") or {}).get("gate_trials", True):
            return True
        self.say(f"trial ready: press Start trial on the dashboard ({self.dash.url})")
        while True:
            self.dash.set_status(self.status())
            for cmd in self.dash.commands():
                if cmd["cmd"] == "start":
                    return True
                if cmd["cmd"] in ("abort", "stop"):
                    self.abort, self.stop = True, cmd["cmd"] == "stop"
                    return False
            time.sleep(0.1)

    def started(self):
        self.state = "running"
        self.t_wall0 = time.time()

    def hold(self):
        """While the dashboard paused the trial: no ticks (the world, SUMO and the stations wait)."""
        if not self.paused:
            return
        self.logger.event(self.t, self.frame, "paused", "", "dashboard")
        self._tell_stations({"type": "pause", "paused": True})
        t_pause = time.time()
        while self.paused and not self.abort:
            self._commands()
            if self.dash is not None:
                self.dash.set_status(self.status())
            time.sleep(0.1)
        if self.t_wall0 is not None:
            self.t_wall0 += time.time() - t_pause
        self.logger.event(self.t, self.frame, "resumed", "", f"after {time.time() - t_pause:.1f} s")
        self._tell_stations({"type": "pause", "paused": False})

    def _tell_stations(self, msg):
        if self.control is not None:
            self.control.broadcast(msg)

    def _commands(self):
        if self.dash is None:
            return
        for cmd in self.dash.commands():
            c = cmd["cmd"]
            if c == "pause":
                self.paused = True
            elif c == "resume":
                self.paused = False
            elif c in ("abort", "stop"):
                self.abort = True
                self.stop = self.stop or c == "stop"
                self.paused = False
                self.logger.event(self.t, self.frame, "trial_aborted" if c == "abort" else "session_stopped", "",
                                  "dashboard")
            elif c == "takeover":
                roles = [cmd["role"]] if cmd.get("role") in self.tor else list(self.tor)
                for r in roles:
                    self.tor[r].request(self.t, "dashboard")
            elif c == "sync" and self.sync_cb is not None:
                self.sync_cb(self.t)

    # -- per tick -------------------------------------------------------------------------------------------
    def drives(self, part):
        """The automation (not the participant) controls this participant's car now."""
        tor = self.tor.get(part.role)
        return tor is not None and tor.state != MANUAL

    def intercept(self, part, ctl, t, events=()):
        """A host-local driver's control: True while the automation drives (the control is only read)."""
        tor = self.tor.get(part.role)
        if tor is None:
            return False
        sp = self.aspec[part.role]
        button, request = keys_pressed(events, sp)
        if request:
            tor.request(t, "key")
        if tor.state == REQUESTED:
            if type(part.input).__name__ == "Autopilot":      # a bot (inputs.Autopilot) takes over after a delay
                bot_rt = float(sp["takeover"].get("bot_reaction_s", 1.5))
                how = "bot" if t - tor.t_request >= bot_rt else None
            else:
                how = takeover.detect(ctl, sp, button)
            if how and tor.taken_over(t, how):
                if hasattr(part.input, "restart"):
                    part.input.restart(part.actor.get_world().get_map().get_waypoint(part.actor.get_location()))
                return False
        elif tor.state == MANUAL and button and tor.engage(t):
            self._resume(self.cars[part.role])
        return tor.state != MANUAL

    def step(self, t, dt, walkers=(), frame=None):
        self.t = t
        self.frame = frame if frame is not None else self.frame
        self._commands()
        for role, car in self.cars.items():
            tor, part = self.tor[role], self.by_role[role]
            car.light = part.light
            tor.tick(t)
            if tor.automated:
                part.ctl = car.step(t, dt, walkers)
                for name, detail in car.events:
                    self.logger.event(t, self.frame, name, car.vehicle.id, detail)
                car.events.clear()
                for i, trg in enumerate(car.triggers):
                    if trg[2] and trg[1].get("takeover") and (role, i) not in self.fired:
                        self.fired.add((role, i))
                        tor.request(t, "trigger")
            elif tor.stopping:
                import carla
                ctl = carla.VehicleControl(throttle=0.0, brake=takeover.brake_for(
                    _speed(car.vehicle), float(self.aspec[role]["takeover"]["stop_decel"])))
                car.apply(ctl)
                part.ctl = ctl
            elif part.station:
                part.ctl = None          # the logger reads the control the station applied
            if tor.changed and self.control is not None:
                self.control.send(role, tor.message())
                tor.changed = False

    @staticmethod
    def _resume(car):
        """The automation continues from the route point nearest the car."""
        loc = car.vehicle.get_location()
        pts = [(w.transform.location.x, w.transform.location.y) for w, _ in car.plan]
        i = takeover.nearest_index(pts, loc.x, loc.y)
        car.progress = i
        car.agent.set_global_plan(car.plan[i:], stop_waypoint_creation=True, clean_queue=True)
        car.target = 3.6 * _speed(car.vehicle)

    def on_message(self, msg, t, frame):
        kind, role = msg.get("type"), msg.get("role")
        if kind == "response":
            if self.responses is not None:
                self.responses.update(role, msg)
            return True
        if kind == "takeover" and role in self.tor:
            self.tor[role].taken_over(t, msg.get("how", "?"), msg.get("reaction_s"))
            return True
        if kind == "engage" and role in self.tor:
            if self.tor[role].engage(t):
                self._resume(self.cars[role])
            return True
        if kind == "takeover_request":
            roles = [role] if role in self.tor else list(self.tor)
            for r in roles:
                self.tor[r].request(t, "key")
            return True
        if kind == "hello" and role in self.tor:
            self.tor[role].changed = True        # a station that (re)joins needs the automation state
        return False

    def log(self, t, frame):
        self.n += 1
        if self.responses is not None:
            now = time.time()
            for role, r in self.responses.local.items():
                window = self.view is not None and self.view.window and self.parts and self.parts[0].role == role
                if window:
                    r.read_pygame(now - self.last_local)
                else:
                    r.bot(t)
            self.last_local = now
            self.responses.log(t, frame, now)
        if self.dash is not None and self.n % 10 == 0:
            self.dash.set_status(self.status())

    def hud_lines(self, part):
        """Extra lines for the host's own window (a host-local participant)."""
        lines = []
        tor = self.tor.get(part.role)
        if tor is not None:
            lines.append({AUTOMATED: "AUTOMATED", MANUAL: "manual",
                          REQUESTED: "!!! TAKE OVER !!! (steer, brake or Enter)"}[tor.state])
        r = self.responses.local.get(part.role) if self.responses is not None else None
        if r is not None and not r.sp.get("hidden"):
            bits = []
            if r.sp.get("button"):
                bits.append(f"button {'HELD' if r.pressed else '-'} (B)")
            if r.sp.get("slider"):
                bits.append(f"slider {r.slider:.0f} (N/M)")
            lines.append("  ".join(bits))
        return lines

    def status(self):
        """The dashboard's view of the trial."""
        connected = self.control.connected() if self.control is not None else set()
        parts = []
        for p in self.parts:
            row = {"role": p.role, "kind": p.kind, "station": p.station, "connected": p.role in connected or not p.station}
            try:
                row["speed_kmh"] = round(3.6 * _speed(p.actor), 1) if p.actor is not None else None
            except Exception:  # noqa: BLE001 - the actor is gone (trial over)
                row["speed_kmh"] = None
            st = self.station_stats.get(p.role) or {}
            lat = (st.get("latency") or {}).get("tick_to_image_ms") or {}
            row.update(rtt_ms=st.get("rtt_ms"), fps=st.get("fps"), image_lag_ms=lat.get("median"))
            if self.responses is not None and (p.role in self.rspec or p.role in self.responses.latest):
                row["response"] = self.responses.state(p.role)
            if p.role in self.tor:
                tor = self.tor[p.role]
                row["automation"] = tor.state + (" (braking to a stop)" if tor.stopping else "")
                row["takeovers"] = tor.records
            parts.append(row)
        wall = time.time() - self.t_wall0 if self.t_wall0 else 0.0
        trial = self.trial
        return {"experiment": self.tc.get("name"), "state": "paused" if self.paused else self.state,
                "participant": trial.get("_participant", 0), "position": trial.get("_position"),
                "trial": trial.get("_index"), "condition": trial.get("_condition") or "",
                "block": trial.get("_block") or "", "t": round(self.t, 2),
                "duration_s": float(trial.get("duration_s", 300)), "rtf": round(self.t / wall, 2) if wall > 1 else None,
                "frame": self.frame, "collisions": self.collisions, "participants": parts,
                "automated": list(self.tor), "events": list(self.events)[-15:],
                "plan": [[t.get("_position"), t.get("_index"), t.get("_condition") or t.get("name") or "",
                          float(t.get("duration_s", 300))] for t in self.plan]}

    def close(self, summary):
        self.state = "ended"
        if self.tor:
            summary["takeovers"] = [r for tor in self.tor.values() for r in tor.records]
        if self.responses is not None and self.responses.rows:
            summary["response_rows"] = self.responses.rows
            self.responses.close()
        if self.tor_file is not None:
            self.tor_file.close()
        if self.abort:
            summary["aborted"] = "session stopped" if self.stop else "trial ended early"
        if self.dash is not None:
            self.dash.set_status(self.status())


class StationStudy:
    """A station's responses and take-over: sample() every frame, drive() before applying a control, draw()."""

    def __init__(self, welcome, args, send):
        self.send = send
        self.bot = getattr(args, "input", "") == "bot"
        rsp = responses.merge_cli(welcome.get("response"), getattr(args, "response", None))
        bot = (welcome.get("bot") or {}).get("response") if self.bot else None
        self.resp = responses.ResponseInput(rsp, bot) if rsp else None
        asp = welcome.get("automation")
        self.tor = takeover.StationTakeOver(asp, send, bot=self.bot) if asp else None
        self.input = self.actor = self.world = None
        self.last = time.time()
        self.paused = False

    def start(self, actor, world, inp, client=None):
        self.actor, self.world, self.input = actor, world, inp
        if self.tor is not None and inp is not None and hasattr(inp, "send") and client is not None:
            inp.send = lambda c: client.apply_batch([c]) if self.tor.state == MANUAL else None

    def on_message(self, msg):
        if msg.get("type") == "pause":
            self.paused = bool(msg.get("paused"))
            return True
        return self.tor.on_message(msg) if self.tor is not None else False

    def drive(self, ctl, events, now=None):
        """True when the station applies its own control this frame (no automation, or taken over)."""
        if self.tor is None:
            return True
        now = time.time() if now is None else now
        button, request = keys_pressed(events, self.tor.sp)
        if request:
            self.send({"type": "takeover_request"})
        was = self.tor.state
        drive = self.tor.may_drive(ctl, button, now)
        if was == REQUESTED and self.tor.state == MANUAL and self.bot and hasattr(self.input, "restart"):
            self.input.restart(self.world.get_map().get_waypoint(self.actor.get_location()))
        return drive

    def sample(self, t, frame, screen=None):
        """Read and send the responses; the cells for the station's CSV (button..slider, automation state)."""
        now = time.time()
        if self.resp is not None:
            if self.bot:
                self.resp.bot(t)
            elif screen is not None:
                self.resp.read_pygame(now - self.last)
            self.send(self.resp.message(frame, t))
        self.last = now
        cells = self.resp.cells() if self.resp is not None else ["", "", "", ""]
        return cells + [self.tor.state if self.tor is not None else ""]

    def draw(self, screen, font, big):
        if self.resp is not None:
            self.resp.draw(screen, font)
        if self.tor is not None:
            self.tor.draw(screen, big, font, time.time())
            self.tor.beep()


CSV_COLUMNS = ["button", "button_value", "presses", "slider", "automation"]


def _write(path, fields, rows):
    import csv
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, delimiter=";", extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def analyse(trial_dir, out):
    """<trial>/analysis/responses.csv and takeovers.csv (analysis.analyse_trial calls this)."""
    res, tor = responses.summarise(trial_dir), takeover.summarise(trial_dir)
    if res:
        _write(out / "responses.csv", responses.SUMMARY, res)
    if tor:
        _write(out / "takeovers.csv", takeover.SUMMARY, tor)
    return res, tor


def collect(dirs, path):
    """Every trial's responses and take-overs in one table each for an experiment folder (study_responses.csv,
    study_takeovers.csv), with the participant, position and condition the trial had (config.yaml's study)."""
    import yaml
    keys = ["trial_dir", "participant", "position", "trial", "condition"]
    res_all, tor_all = [], []
    for d in dirs:
        cfg = d / "config.yaml"
        st = ((yaml.safe_load(cfg.read_text()) or {}).get("study") or {}) if cfg.exists() else {}
        info = {"trial_dir": d.name, "participant": st.get("participant", ""), "position": st.get("position", ""),
                "trial": st.get("index", ""), "condition": st.get("condition", "")}
        res_all += [{**info, **r} for r in responses.summarise(d)]
        tor_all += [{**info, **r} for r in takeover.summarise(d)]
    if res_all:
        _write(path / "study_responses.csv", keys + responses.SUMMARY, res_all)
    if tor_all:
        _write(path / "study_takeovers.csv", keys + takeover.SUMMARY, tor_all)
    return res_all, tor_all
