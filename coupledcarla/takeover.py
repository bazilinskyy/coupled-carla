"""Automated driving with take-over requests (coupled-sim's AutomatedDriving, #94).

A driver participant with `automation` rides in a car that drives itself along a route (ai.ScriptedCar: CARLA's
BasicAgent on the planned lanes, stopping for SUMO's red lights and for pedestrians), until a take-over request
(TOR) hands it over:

    participants:
      - role: driver
        station: true
        automation:
          route: [[51.44309, 5.49081], [51.44283, 5.48899], [51.44246, 5.48640]]   # lat, lon; starts at the first
          speed_kmh: 30
          triggers: [{at_m: 150, takeover: true}]     # a request when the car has driven 150 m (also speed_kmh,
                                                      # blinker, ... as ScriptedCar's triggers)
          takeover: {at_s: 20, budget_s: 7, no_reaction: brake, beep: true}

The request shows a flashing red TAKE OVER in the participant's view, with three 1 kHz beeps a second (beep:
false for none). The driver takes over by steering (|steer| above steer_threshold), braking or pressing the
throttle (above pedal_threshold), or with the automation button (Enter; on a wheel the G25's autopilot button, 2,
on a gamepad button 7, `button` sets another), and then drives. Without a reaction within budget_s the car brakes to
a stop at stop_decel m/s2 (no_reaction: brake, a minimal-risk manoeuvre; the driver can still take over) or the
request ends and the automation drives on (no_reaction: keep). With engage_again the automation button hands the car
back to the automation, which continues from the nearest point of its route. Requests also come from the
experimenter: T at the station, or "Take-over request" on the dashboard. A bot driver (--input bot, --local)
takes over bot_reaction_s after the request.

Logged: events.csv takeover_request / takeover / takeover_no_reaction / automation_engaged, and <trial>/takeover.csv
in coupled-sim's TakeOver-*.csv columns (time,event,detail,speed_kmh, then x,y in CARLA metres instead of Unity's
x,z), with the event names request / takeover / no reaction / engaged and the detail "steer; reaction 1.234 s".
The reaction time is measured at the station, from the first frame that showed the warning to the input; the host's
own (request to the take-over arriving, simulation time) is in the events too.
"""
import csv

DEFAULTS = {
    "route": None,                 # [[lat, lon], ...]
    "speed_kmh": 30.0,
    "accel": 2.0,
    "yield_to_pedestrians": True,
    "triggers": [],                # ScriptedCar triggers; takeover: true issues a request there
    "takeover": {
        "at_s": None,              # a request this many seconds into the trial (or a list of times)
        "budget_s": 7.0,           # time to react before the minimal-risk manoeuvre
        "no_reaction": "brake",    # brake (to a stop) | keep (drive on)
        "stop_decel": 3.0,         # m/s2 while braking to a stop
        "beep": True,
        "steer_threshold": 0.1,    # |steer| (-1..1) that counts as taking over
        "pedal_threshold": 0.1,    # throttle or brake (0..1)
        "button": None,            # joystick button(s) for take-over and engage (default by device, see below)
        "engage_again": True,      # the automation button hands the car back
        "bot_reaction_s": 1.5,     # a bot driver takes over this long after the request
    },
}
AUTOMATED, REQUESTED, MANUAL = "automated", "requested", "manual"
WHEEL_BUTTON, PAD_BUTTON = 2, 7    # G25 autopilot button (inputs/steering_wheel.py), gamepad Start


def spec(value):
    """A participant's `automation` setting merged over DEFAULTS, or None."""
    if not value:
        return None
    from .config import _merge
    return _merge(DEFAULTS, value)


def detect(ctl, sp, button=False):
    """How the driver took over, from their own control input (throttle, steer, brake) and the button; or None."""
    if button:
        return "button"
    if ctl is None:
        return None
    t = sp["takeover"]
    if abs(getattr(ctl, "steer", 0.0)) > t["steer_threshold"]:
        return "steer"
    if getattr(ctl, "brake", 0.0) > t["pedal_threshold"]:
        return "brake"
    if getattr(ctl, "throttle", 0.0) > t["pedal_threshold"]:
        return "throttle"
    return None


def brake_for(speed_mps, decel):
    """A VehicleControl's brake (0-1) that slows a car at about decel m/s2; full brake once (nearly) stopped."""
    return 1.0 if speed_mps < 0.3 else min(1.0, 0.1 + decel / 9.0)


class TakeOver:
    """The host's state of one automated car: automated -> requested -> manual (-> automated by engaging).

    emit(t, event, detail) logs an event; where() returns (speed km/h, x, y) for takeover.csv."""

    def __init__(self, role, sp, emit, writer=None, where=None):
        self.role, self.sp, self.emit = role, sp, emit
        self.where = where or (lambda: (0.0, 0.0, 0.0))
        self.state = AUTOMATED
        self.t_request = self.source = None
        self.stopping = False
        at = sp["takeover"].get("at_s")
        self.schedule = sorted(float(x) for x in (at if isinstance(at, (list, tuple)) else [at]) if x is not None)
        self.records = []           # one dict per request, for summary.json
        self.w = writer             # takeover.csv (open_csv), shared by the trial's automated cars
        self.changed = True          # the stations need the new state

    @property
    def automated(self):
        """The automation steers the car (also while a request waits, unless it brakes to a stop)."""
        return self.state == AUTOMATED or (self.state == REQUESTED and not self.stopping)

    def _row(self, t, event, detail=""):
        if self.w is not None:
            speed, x, y = self.where()
            self.w.writerow([f"{t:.3f}", event, detail, f"{speed:.1f}", f"{x:.2f}", f"{y:.2f}", self.role])

    def request(self, t, source):
        if self.state != AUTOMATED:
            return False
        self.state, self.t_request, self.source, self.stopping = REQUESTED, t, source, False
        self.records.append({"role": self.role, "t_request": round(t, 3), "source": source, "how": None,
                             "reaction_s": None, "host_reaction_s": None, "no_reaction": False})
        self.emit(t, "takeover_request", f"{self.role} source={source}")
        self._row(t, "request", source)
        self.changed = True
        return True

    def taken_over(self, t, how, reaction_s=None):
        if self.state != REQUESTED:
            return False
        host_rt = t - self.t_request
        rt = host_rt if reaction_s is None else float(reaction_s)
        self.state, self.stopping = MANUAL, False
        rec = self.records[-1]
        rec.update(how=how, reaction_s=round(rt, 3), host_reaction_s=round(host_rt, 3))
        self.emit(t, "takeover", f"{self.role} {how}; reaction {rt:.3f} s (host {host_rt:.3f} s)")
        self._row(t, "takeover", f"{how}; reaction {rt:.3f} s")
        self.changed = True
        return True

    def engage(self, t):
        if self.state != MANUAL or not self.sp["takeover"].get("engage_again", True):
            return False
        self.state = AUTOMATED
        self.emit(t, "automation_engaged", self.role)
        self._row(t, "engaged")
        self.changed = True
        return True

    def tick(self, t):
        """Scheduled requests and the reaction budget."""
        while self.schedule and self.schedule[0] <= t:
            self.schedule.pop(0)
            self.request(t, "at_s")
        tk = self.sp["takeover"]
        if self.state == REQUESTED and not self.stopping and t - self.t_request > float(tk["budget_s"]) and \
                not self.records[-1]["no_reaction"]:
            self.records[-1]["no_reaction"] = True
            keep = str(tk.get("no_reaction", "brake")).lower() == "keep"
            self.emit(t, "takeover_no_reaction", f"{self.role} {'keep driving' if keep else 'brake to a stop'}")
            self._row(t, "no reaction", "KeepDriving" if keep else "BrakeToStop")
            if keep:
                self.state = AUTOMATED
            else:
                self.stopping = True
            self.changed = True

    def message(self):
        return {"type": "takeover", "state": self.state, "stopping": self.stopping, "source": self.source}


def open_csv(path):
    """takeover.csv: coupled-sim's TakeOver-*.csv columns, then the role. Returns (file, csv writer)."""
    f = open(path, "w", newline="", buffering=1)
    w = csv.writer(f)
    w.writerow(["time", "event", "detail", "speed_kmh", "x", "y", "role"])
    return f, w


class StationTakeOver:
    """The station's side: warning, beep, take-over detection and the automation button.

    may_drive(ctl, button, now) says whether the station applies its own control this frame; it sends
    {"type": "takeover", how, reaction_s} and {"type": "engage"} through send()."""

    def __init__(self, sp, send, bot=False):
        self.sp, self.send, self.bot = sp, send, bot
        self.state = AUTOMATED
        self.stopping = False
        self.shown_at = None           # station time of the first frame showing the warning
        self.sound = None
        self.sound_on = False

    def on_message(self, msg):
        if msg.get("type") != "takeover":
            return False
        new = msg.get("state", self.state)
        if new != REQUESTED:
            self.shown_at = None
        if new == REQUESTED and self.state != REQUESTED:
            self.shown_at = None       # set at the next rendered frame
        self.state, self.stopping = new, bool(msg.get("stopping"))
        return True

    def frame_shown(self, now):
        if self.state == REQUESTED and self.shown_at is None:
            self.shown_at = now

    def may_drive(self, ctl, button, now):
        if self.state == MANUAL:
            if button and self.sp["takeover"].get("engage_again", True):
                self.state = AUTOMATED       # the host confirms; until then the automation drives
                self.send({"type": "engage"})
                return False
            return True
        if self.state == REQUESTED:
            self.frame_shown(now)
            if self.bot:
                how = "bot" if now - self.shown_at >= float(self.sp["takeover"].get("bot_reaction_s", 1.5)) else None
            else:
                how = detect(ctl, self.sp, button)
            if how:
                self.state = MANUAL
                self.send({"type": "takeover", "how": how, "reaction_s": round(now - self.shown_at, 3)})
                return True
        return False

    def beep(self):
        """Three short 1 kHz beeps a second while a request waits (pygame.mixer; silent without audio)."""
        want = self.state == REQUESTED and self.sp["takeover"].get("beep", True)
        if want == self.sound_on:
            return
        self.sound_on = want
        try:
            import pygame
            if self.sound is None:
                if not pygame.mixer.get_init():
                    pygame.mixer.init()
                self.sound = pygame.mixer.Sound(buffer=beeps(*pygame.mixer.get_init()))
            if want:
                self.sound.play(loops=-1)
            else:
                self.sound.stop()
        except Exception as err:  # noqa: BLE001 - no audio device: the warning alone
            print(f"[station] no take-over beep ({err})", flush=True)
            self.sp["takeover"]["beep"] = False

    def draw(self, screen, big, font, now):
        import pygame
        w, h = screen.get_size()
        label = {AUTOMATED: ("AUTOMATED", (0, 170, 255)), MANUAL: ("MANUAL", (200, 200, 200)),
                 REQUESTED: ("TAKE OVER", (255, 40, 30))}[self.state]
        badge = font.render(label[0] if self.state != REQUESTED else "AUTOMATION ENDING", True, label[1])
        screen.blit(badge, (w - badge.get_width() - 16, 12))
        if self.state == REQUESTED:
            self.frame_shown(now)
            a = 0.4 + 0.6 * abs(((now * 3.0) % 2.0) - 1.0)      # pulses three times a second
            box = pygame.Surface((min(520, w - 40), 110), pygame.SRCALPHA)
            box.fill((255, 30, 20, int(200 * a)))
            screen.blit(box, ((w - box.get_width()) // 2, h // 2 - 150))
            text = big.render("TAKE OVER", True, (255, 255, 255))
            screen.blit(text, ((w - text.get_width()) // 2, h // 2 - 150 + (110 - text.get_height()) // 2))


def beeps(freq=44100, size=-16, channels=2):
    """One second of three 0.15 s 1 kHz beeps as raw 16-bit samples for pygame.mixer.Sound(buffer=...)."""
    import numpy as np
    t = np.arange(int(freq)) / float(freq)
    on = (t % (1.0 / 3.0)) < 0.15
    wave = (0.35 * 32767 * np.sin(2 * np.pi * 1000.0 * t) * on).astype(np.int16)
    if channels and channels > 1:
        wave = np.repeat(wave[:, None], channels, axis=1)
    return wave.tobytes()


def nearest_index(points, x, y):
    """Index of the route point nearest to (x, y): where the automation continues after engaging again."""
    return min(range(len(points)), key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2) if points else 0


SUMMARY = ["role", "t_request", "source", "how", "reaction_s", "no_reaction", "speed_kmh_at_request"]


def summarise(trial_dir):
    """One row per take-over request of a trial, from its takeover.csv."""
    path = trial_dir / "takeover.csv"
    if not path.exists():
        return []
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            role = r.get("role", "")
            if r["event"] == "request":
                out.append({"role": role, "t_request": float(r["time"]), "source": r["detail"], "how": "",
                            "reaction_s": "", "no_reaction": 0, "speed_kmh_at_request": float(r["speed_kmh"])})
                continue
            last = next((o for o in reversed(out) if o["role"] == role), None)
            if last is None:
                continue
            if r["event"] == "takeover" and not last["how"]:
                how, _, rt = r["detail"].partition("; reaction ")
                last["how"], last["reaction_s"] = how, float(rt.split()[0]) if rt else ""
            elif r["event"] == "no reaction":
                last["no_reaction"] = 1
    return out
