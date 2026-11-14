"""Continuous participant responses (coupled-sim's ResponseInput, PR #130): a hold-to-respond button ("I would
cross now", "I feel safe") and a slider from 0 to 100 (trust, perceived risk), sampled every frame.

A participant gets them with `response` in the experiment:

    participants:
      - role: pedestrian
        response: {button: "Hold while you would cross", slider: "How safe do you feel?", slider_start: 50}

(`response: button,slider` or `true` for both with default texts; `hidden: true` logs without showing anything.)
At a station (or on the host, for a host-local participant with a window) the button is B on the keyboard, the
slider N (down) / M (up); on a gamepad the right shoulder button (and the right trigger, whose analog value is
logged) and the d-pad up/down; on a steering wheel (G25/G27/G29) the right button on the wheel and the paddles
(right up, left down). `joystick: {buttons: [..], axis: N, axis_rest: -1, hat: 0, slider_up: [..],
slider_down: [..]}` sets other ones. A small panel at the bottom of the view shows the text, a lamp while the
button is held and the slider.

A station can add them with --response button,slider. The station sends its state to the host every frame ({"type": "response", ...}); the host writes
<trial>/responses.csv, one row per host frame per responding participant, in coupled-sim's Responses-*.csv
columns (utc;time;frame;button;button_value;presses;slider), then role, the station's frame and the sample's age.
The station's own CSV (logs/stations/) has the same four values on each of its frames.
"""
import csv
import time
from datetime import datetime, timezone

COLUMNS = ["utc", "time", "frame", "button", "button_value", "presses", "slider", "role", "station_frame",
           "age_ms"]
DEFAULT_TEXT = {"button": "Hold the button while you feel safe", "slider": "Slider: how safe do you feel?"}
KEYS = {"button": "b", "slider_down": "n", "slider_up": "m"}
WHEELS = ("g25", "g27", "g29", "g920", "g923", "wheel", "driving force")
WHEEL_JOY = {"buttons": [6], "slider_up": [4], "slider_down": [5], "axis": None, "hat": None}
PAD_JOY = {"buttons": [5], "slider_up": [], "slider_down": [], "axis": 5, "axis_rest": -1.0, "hat": 0}


def spec(value):
    """A participant's `response` setting as a dict (or None when it has none)."""
    if value in (None, False, "", "none"):
        return None
    if value is True:
        value = "button,slider"
    if isinstance(value, str):
        parts = {p.strip().lower() for p in value.split(",")}
        value = {"button": "button" in parts, "slider": "slider" in parts, "hidden": "hidden" in parts}
    out = {"button": None, "slider": None, "slider_start": 50.0, "slider_speed": 40.0, "hidden": False,
           "press_threshold": 0.5, "joystick": None}
    for k, v in value.items():
        out[k] = v
    for k in ("button", "slider"):
        if out[k] is True:
            out[k] = DEFAULT_TEXT[k]
        elif not out[k]:
            out[k] = None
    if not out["button"] and not out["slider"]:
        return None
    return out


def merge_cli(sp, text):
    """A station's --response button,slider[,hidden] on top of what the host said."""
    extra = spec(text) if text else None
    if not extra:
        return sp
    if not sp:
        return extra
    sp = dict(sp)
    for k in ("button", "slider"):
        if extra.get(k) and not sp.get(k):
            sp[k] = extra[k]
    sp["hidden"] = bool(sp.get("hidden") or extra.get("hidden"))
    return sp


def joystick_state(joy, js=None):
    """(button held, analog 0-1, slider direction -1/0/1) from one pygame joystick, by the joystick's kind."""
    name = (joy.get_name() or "").lower()
    js = dict(WHEEL_JOY if any(w in name for w in WHEELS) else PAD_JOY, **(js or {}))
    nb = joy.get_numbuttons()

    def held(ids):
        return any(0 <= b < nb and joy.get_button(b) for b in ids or [])

    value = 1.0 if held(js.get("buttons")) else 0.0
    ax = js.get("axis")
    if ax is not None and 0 <= ax < joy.get_numaxes():
        rest = float(js.get("axis_rest", -1.0))
        a = (joy.get_axis(ax) - rest) / max(1e-6, 1.0 - rest)
        value = max(value, min(1.0, max(0.0, a)))
    d = (1 if held(js.get("slider_up")) else 0) - (1 if held(js.get("slider_down")) else 0)
    hat = js.get("hat")
    if hat is not None and 0 <= hat < joy.get_numhats():
        d += joy.get_hat(hat)[1]          # pygame: up is +1
    return value >= 1.0, value, max(-1, min(1, d))


class ResponseInput:
    """One participant's button and slider. step() each frame with what is held; bot() replays a schedule."""

    def __init__(self, sp, bot=None):
        self.sp = sp
        self.pressed = False
        self.value = 0.0
        self.presses = 0
        self.slider = min(100.0, max(0.0, float(sp.get("slider_start", 50.0))))
        self.schedule = sorted([list(x) for x in bot or []], key=lambda x: x[0])
        self.joysticks = None

    def step(self, dt, key=False, analog=0.0, slider_dir=0):
        """key: the button key (or a joystick button) held; analog: a trigger's 0-1; slider_dir: -1, 0, 1."""
        value = 1.0 if key else max(0.0, min(1.0, float(analog)))
        pressed = bool(key) or value >= float(self.sp.get("press_threshold", 0.5))
        if not self.sp.get("button"):
            pressed, value = False, 0.0
        if pressed and not self.pressed:
            self.presses += 1
        self.pressed, self.value = pressed, value
        if self.sp.get("slider") and slider_dir:
            rate = max(-1.0, min(1.0, float(slider_dir)))
            self.slider = min(100.0, max(0.0, self.slider + rate * float(self.sp.get("slider_speed", 40.0)) * dt))

    def bot(self, t):
        """Replay [[t, button 0/1, slider or null], ...] (tests, `--input bot`)."""
        while self.schedule and self.schedule[0][0] <= t:
            _, b, s = (self.schedule.pop(0) + [None, None])[:3]
            if b is not None and self.sp.get("button"):
                b = bool(b)
                if b and not self.pressed:
                    self.presses += 1
                self.pressed, self.value = b, 1.0 if b else 0.0
            if s is not None and self.sp.get("slider"):
                self.slider = min(100.0, max(0.0, float(s)))

    def read_pygame(self, dt):
        """Read the keyboard and every joystick (pygame must be initialised, with a window for the keys)."""
        import pygame
        k = pygame.key.get_pressed()
        key = k[pygame.key.key_code(KEYS["button"])]
        d = (1 if k[pygame.key.key_code(KEYS["slider_up"])] else 0) - \
            (1 if k[pygame.key.key_code(KEYS["slider_down"])] else 0)
        analog = 0.0
        if self.joysticks is None:
            if not pygame.joystick.get_init():
                pygame.joystick.init()
            self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            for j in self.joysticks:
                j.init()
        for j in self.joysticks:
            held, a, jd = joystick_state(j, self.sp.get("joystick"))
            key = key or held
            analog = max(analog, a)
            d += jd
        self.step(dt, key, analog, max(-1, min(1, d)))

    def cells(self):
        """button;button_value;presses;slider as coupled-sim writes them (empty when not asked for)."""
        b, s = self.sp.get("button"), self.sp.get("slider")
        return [int(self.pressed) if b else "", f"{self.value:.3f}" if b else "", self.presses if b else "",
                f"{self.slider:.1f}" if s else ""]

    def message(self, frame, t):
        b = bool(self.sp.get("button"))
        return {"type": "response", "b": int(self.pressed) if b else None, "v": round(self.value, 3) if b else None,
                "n": self.presses if b else None, "s": round(self.slider, 1) if self.sp.get("slider") else None,
                "f": frame, "t": round(t, 3)}

    def draw(self, screen, font):
        """The panel at the bottom centre of a pygame surface: text, lamp, slider."""
        if self.sp.get("hidden"):
            return
        import pygame
        w, h = screen.get_size()
        lines = [x for x in (self.sp.get("button"), self.sp.get("slider")) if x]
        pw, ph = min(560, w - 40), 16 + 28 * len(lines) + (26 if self.sp.get("slider") else 0)
        x0, y0 = (w - pw) // 2, h - ph - 24
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((16, 20, 26, 190))
        screen.blit(panel, (x0, y0))
        y = y0 + 8
        if self.sp.get("button"):
            colour = (50, 215, 80) if self.pressed else (140, 140, 140)
            pygame.draw.circle(screen, colour, (x0 + 22, y + 12), 10)
            screen.blit(font.render(self.sp["button"], True, (240, 240, 240)), (x0 + 42, y))
            y += 28
        if self.sp.get("slider"):
            screen.blit(font.render(f"{self.sp['slider']}  {self.slider:.0f}", True, (240, 240, 240)), (x0 + 12, y))
            y += 30
            pygame.draw.rect(screen, (90, 90, 90), (x0 + 12, y, pw - 24, 12), border_radius=6)
            pygame.draw.rect(screen, (0, 190, 255), (x0 + 12, y, int((pw - 24) * self.slider / 100.0), 12),
                             border_radius=6)


def _utc(ts=None):
    return datetime.fromtimestamp(ts if ts is not None else time.time(), timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%S.%f")[:-3]


class ResponseLog:
    """The host's responses.csv: the latest state of every responding participant, one row per host frame."""

    def __init__(self, path, roles):
        self.roles = list(roles)
        self.latest = {}                 # role -> (message, host time received)
        self.local = {}                  # role -> ResponseInput on the host
        self.path, self.f, self.w = path, None, None      # opened with the first row
        self.rows = 0

    def update(self, role, msg):
        if role not in self.roles:          # a station given --response without the experiment asking
            self.roles.append(role)
        self.latest[role] = (msg, msg.get("recv", time.time()))

    def state(self, role):
        """{button, value, presses, slider} now (for the dashboard), or None before the first sample."""
        if role in self.local:
            r = self.local[role]
            return {"button": int(r.pressed), "value": round(r.value, 3), "presses": r.presses,
                    "slider": round(r.slider, 1) if r.sp.get("slider") else None}
        got = self.latest.get(role)
        if not got:
            return None
        m = got[0]
        return {"button": m.get("b"), "value": m.get("v"), "presses": m.get("n"), "slider": m.get("s")}

    def log(self, t, frame, now=None):
        now = time.time() if now is None else now
        if self.w is None:
            if not self.local and not self.latest:
                return
            self.f = open(self.path, "w", newline="", buffering=1)
            self.w = csv.writer(self.f, delimiter=";")
            self.w.writerow(COLUMNS)
        utc = _utc(now)
        for role in self.roles:
            if role in self.local:
                self.w.writerow([utc, f"{t:.4f}", frame, *self.local[role].cells(), role, frame, 0])
            elif role in self.latest:
                m, recv = self.latest[role]
                self.w.writerow([utc, f"{t:.4f}", frame, m.get("b", ""), m.get("v", ""), m.get("n", ""),
                                 "" if m.get("s") is None else m["s"], role, m.get("f", ""),
                                 round(1000 * (now - recv))])
            else:
                continue
            self.rows += 1

    def close(self):
        if self.f is not None:
            self.f.close()


SUMMARY = ["role", "rows", "duration_s", "button_held_s", "button_share", "presses", "first_press_s",
           "slider_mean", "slider_min", "slider_max", "slider_end"]


def _rows(trial_dir):
    path = trial_dir / "responses.csv"
    if not path.exists():
        return {}
    by_role = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            by_role.setdefault(r["role"], []).append(r)
    return by_role


def summarise(trial_dir):
    """Per responding participant of a trial: how long the button was held, presses, the slider's mean and range."""
    out = []
    for role, rows in _rows(trial_dir).items():
        ts = [float(r["time"]) for r in rows]
        held, first, presses = 0.0, None, ""
        for i, r in enumerate(rows):
            if r["button"] == "1":
                held += (ts[i + 1] - ts[i]) if i + 1 < len(rows) else 0.0
                first = ts[i] if first is None else first
            if r["presses"] not in ("", None):
                presses = int(r["presses"])
        sl = [float(r["slider"]) for r in rows if r["slider"] not in ("", None)]
        dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
        has_button = any(r["button"] not in ("", None) for r in rows)
        out.append({"role": role, "rows": len(rows), "duration_s": round(dur, 2),
                    "button_held_s": round(held, 2) if has_button else "",
                    "button_share": round(held / dur, 3) if has_button and dur else "",
                    "presses": presses, "first_press_s": "" if first is None else round(first, 2),
                    "slider_mean": round(sum(sl) / len(sl), 1) if sl else "", "slider_min": min(sl) if sl else "",
                    "slider_max": max(sl) if sl else "", "slider_end": sl[-1] if sl else ""})
    return out


def series(trial_dir, t0=0.0, every=0.1):
    """{role: [[t, button, slider], ...]} for the trial viewer: every `every` s and at each button change."""
    out = {}
    for role, rows in _rows(trial_dir).items():
        s, last_t, last_b = [], None, None
        for r in rows:
            t, b = float(r["time"]), r["button"]
            if last_t is None or t - last_t >= every - 1e-6 or b != last_b:
                s.append([round(t + t0, 2), int(b) if b not in ("", None) else None,
                          float(r["slider"]) if r["slider"] not in ("", None) else None])
                last_t, last_b = t, b
        out[role] = s
    return out
