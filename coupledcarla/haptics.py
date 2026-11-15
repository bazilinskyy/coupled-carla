"""Haptic cues at a station (coupled-sim #133): a named cue with an intensity (0-1), a duration and a pattern,
felt as
- a vibration of the steering wheel (the G25's force feedback, steering_wheel.SteeringWheel.vibrate),
- gamepad rumble (pygame's Joystick.rumble, on every gamepad that is not the wheel),
- an external device (vibrating seat, belt, wristband): a UDP datagram in JSON,
  {"cue": "takeover", "intensity": 1.0, "duration": 1.2, "pattern": "triple", "time": 12.34, "role": "driver"},
  to each `udp` address (host:port), as coupled-sim's -hapticsUdp.

    haptics:
      enabled: true
      wheel: true
      gamepad: true
      udp: ["192.168.1.50:9000"]
      cues:                         # besides the take-over request ("takeover", triple, 1.2 s) and messages
        - {on: over_limit, pattern: double, intensity: 0.6, duration: 0.6}
        - {at: 30, name: attention, pattern: pulse}
        - {on: "ehmi:yielding", pattern: pulse, intensity: 0.4}   # the nearest automated car within 40 m

Patterns: pulse (one), double, triple, continuous. Every cue is logged by the station.
"""
import json
import socket

PATTERNS = {        # on-intervals as fractions of the duration
    "pulse": [(0.0, 1.0)],
    "double": [(0.0, 0.35), (0.6, 0.95)],
    "triple": [(0.0, 0.22), (0.39, 0.61), (0.78, 1.0)],
    "continuous": [(0.0, 1.0)],
}
DEFAULT_CUES = {"takeover": {"intensity": 1.0, "duration": 1.2, "pattern": "triple"},
                "message": {"intensity": 0.5, "duration": 0.3, "pattern": "pulse"},
                "over_limit": {"intensity": 0.6, "duration": 0.6, "pattern": "double"}}


def strength(cues, now):
    """The vibration strength now: the strongest of the running cues in one of their on-intervals."""
    s = 0.0
    for c in cues:
        f = (now - c["t"]) / max(1e-3, c["duration"])
        if 0.0 <= f < 1.0 and any(a <= f < b for a, b in PATTERNS.get(c["pattern"], PATTERNS["pulse"])):
            s = max(s, c["intensity"])
    return s


class Haptics:
    def __init__(self, cfg, wheel=None, role="", log=print, on_cue=None):
        self.cfg = cfg or {}
        self.wheel = wheel if self.cfg.get("wheel", True) else None
        self.role, self.log = role, log
        self.on_cue = on_cue or (lambda *a: None)
        self.running = []
        self.pads = []
        self.targets = []
        self.sock = None
        self.fired = set()
        self.cues = [dict(c) for c in self.cfg.get("cues") or []]
        for c in self.cues:         # YAML reads an unquoted `on:` key as true (and the welcome's JSON as "true")
            for k in (True, "true"):
                if "on" not in c and k in c:
                    c["on"] = c.pop(k)
        self._last = 0.0
        self._pad_on = False
        if self.cfg.get("gamepad", True):
            self.pads = self._gamepads()
        for t in self.cfg.get("udp") or []:
            try:
                host, port = str(t).rsplit(":", 1)
                self.targets.append((host, int(port)))
            except ValueError:
                log(f"haptics: bad udp address {t!r} (host:port)")
        if self.targets:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        outs = (["wheel"] if self.wheel is not None and hasattr(self.wheel, "vibrate") else []) + \
               [f"{len(self.pads)} gamepad(s)"] * bool(self.pads) + [f"UDP {t[0]}:{t[1]}" for t in self.targets]
        log(f"haptics: {', '.join(outs) or 'no output device (cues are logged only)'}")

    def _gamepads(self):
        try:
            import pygame
            pygame.joystick.init()
            pads = []
            wheel_js = getattr(self.wheel, "joystick", None)
            for i in range(pygame.joystick.get_count()):
                js = pygame.joystick.Joystick(i)
                js.init()
                if wheel_js is not None and js.get_instance_id() == wheel_js.get_instance_id():
                    continue
                if hasattr(js, "rumble"):
                    pads.append(js)
            return pads
        except Exception:
            return []

    def cue(self, name, now, intensity=None, duration=None, pattern=None, detail=""):
        d = dict(DEFAULT_CUES.get(name, {"intensity": 1.0, "duration": 0.5, "pattern": "pulse"}))
        if intensity is not None:
            d["intensity"] = float(intensity)
        if duration is not None:
            d["duration"] = float(duration)
        if pattern:
            d["pattern"] = str(pattern)
        if d["pattern"] not in PATTERNS:
            d["pattern"] = "pulse"
        d["intensity"] = max(0.0, min(1.0, d["intensity"]))
        c = {"name": name, "t": now, **d}
        self.running.append(c)
        self.on_cue("haptic", name, f"{d['pattern']} {d['intensity']:.2f} {d['duration']:.2f} s {detail}".strip())
        if self.sock is not None:
            msg = json.dumps({"cue": name, "intensity": round(d["intensity"], 2), "duration": round(d["duration"], 2),
                              "pattern": d["pattern"], "time": round(now, 3), "role": self.role}).encode()
            for t in self.targets:
                try:
                    self.sock.sendto(msg, t)
                except OSError as e:
                    self.log(f"haptics: UDP to {t[0]}:{t[1]}: {e}")
        return c

    def events(self, now, conds, ehmi_near=None):
        """Fires the configured cues: at a time (`at`), or on a condition's rising edge (`on`: over_limit, takeover,
        message, ehmi:<state>)."""
        for i, c in enumerate(self.cues):
            on = c.get("on")
            if "at" in c:
                hit = now >= float(c["at"]) and i not in self.fired
                if hit:
                    self.fired.add(i)
            elif on:
                if on.startswith("ehmi:"):
                    cond = ehmi_near == on.split(":", 1)[1]
                else:
                    cond = bool(conds.get(on))
                hit = cond and (i, "on") not in self.fired
                if cond:
                    self.fired.add((i, "on"))
                else:
                    self.fired.discard((i, "on"))
            else:
                continue
            if hit:
                self.cue(c.get("name", on or "cue"), now, c.get("intensity"), c.get("duration"), c.get("pattern"))

    def tick(self, now):
        self.running = [c for c in self.running if now - c["t"] < c["duration"]]
        s = strength(self.running, now)
        if self.wheel is not None and hasattr(self.wheel, "vibrate"):
            self.wheel.vibrate(s)
        if self.pads and (s > 0) != self._pad_on:
            self._pad_on = s > 0
            for js in self.pads:
                try:
                    if s > 0:
                        js.rumble(s * 0.8, s, 0)       # until stopped
                    else:
                        js.stop_rumble()
                except Exception:
                    pass
        return s

    def close(self):
        for js in self.pads:
            try:
                js.stop_rumble()
            except Exception:
                pass
        if self.wheel is not None and hasattr(self.wheel, "vibrate"):
            self.wheel.vibrate(0.0)
        if self.sock is not None:
            self.sock.close()
