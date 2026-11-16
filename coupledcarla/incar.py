"""In-car displays of a driver or AV-passenger station (coupled-sim #133 and #157): an instrument cluster, a head-up
display and a centre screen drawn over the station's camera view, timed in-car messages (take-over requests among
them), and the speed limit of the road from the CARLA map.

    displays:
      cluster: cluster              # a design (data/incar/<name>.json|yaml, a path or an inline dict); true = default
      hud: hud_speed
      centre: centre_screen
      messages:                     # shown on a display at a time, a distance driven or when a state starts
        - {at: 5, seconds: 4, display: hud, text: "Turn right at the next junction"}
        - {at_m: 150, seconds: 5, display: centre, text: "Roadworks ahead", color: "#FFD60A"}
        - {at: 20, seconds: 6, takeover: true, text: "TAKE OVER"}   # a scripted warning: TAKEOVER, a cue
      messages_file: experiments/messages.csv    # coupled-sim's CSV (at;seconds;display;text;color;atDistance;onState)

A display design is the same format as an eHMI design (designs.py). Its states are MANUAL (a driver), AUTOMATED
(an AV passenger, or a driver whose car has `automation`) and TAKEOVER (during a take-over request); ANY is always
shown. Real take-over requests of an automated car come from takeover.py (its own warning, beeps and reaction
time); the displays follow its state, with {time_left} counting down its budget. A message with `takeover: true`
is a scripted warning only (no automation hands over), with its reaction (first brake, throttle or steering) logged. Live values in text, gauges and
bars: {speed} {limit} {rpm} {gear} {power} {clock} {trial_time} {distance} {message} {mode} {time_left} {reaction}.
An electric own car (set_electric, from sound.electric) has no rpm (0) and no gear number (P, D, R); its default
cluster is cluster_ev (a power meter, % of full power, regeneration below 0, instead of the rev counter).
Conditions (`when`): left right blinkers headlights braking reverse automated manual takeover message limit
over_limit; "!x" negates. Messages for a display that is not configured go to the centre screen (a plain message
panel when there is none). Every message shown, take-over request and response is logged by the station.
"""
import math
import re
import threading
import time
import xml.etree.ElementTree as ET

from . import designs

RECTS = {   # default place on the screen: (x of the centre, y of the centre or None, width), fractions
    "cluster": (0.5, None, 0.34),     # at the bottom, centred
    "hud": (0.5, 0.56, 0.28),         # just below the horizon, over the road
    "centre": (0.82, None, 0.24),     # at the bottom right
}
DEFAULT_DESIGNS = {"cluster": "cluster", "hud": "hud_speed", "centre": "centre_screen"}
LOW_BEAM, HIGH_BEAM, BRAKE_LIGHT, RIGHT_BLINKER, LEFT_BLINKER, REVERSE_LIGHT = 2, 4, 8, 16, 32, 64


# -- speed limits from the OpenDRIVE --------------------------------------------------------------------------------

def _kmh(value, unit):
    v = float(value)
    u = (unit or "m/s").lower()
    k = v if u in ("km/h", "kmh") else v * 1.609344 if u == "mph" else v * 3.6
    r = round(k)
    five = 5 * round(k / 5)
    return int(five if abs(k - five) <= 1.0 else r)


class SpeedLimits:
    """Speed limits per road, lane section and lane from an OpenDRIVE text (CARLA's world.get_map().to_opendrive():
    netconvert writes each lane's <speed max> in m/s), with the road type's speed as a fallback."""

    def __init__(self, xodr_text):
        self.roads = {}       # road id -> ([(s, {lane id: km/h})], [(s, km/h)] road type speeds, junction?)
        root = ET.fromstring(xodr_text)
        for road in root.iter("road"):
            rid = road.get("id")
            types = []
            for t in road.findall("type"):
                sp = t.find("speed")
                if sp is not None and sp.get("max") not in (None, "no limit", "undefined"):
                    try:
                        types.append((float(t.get("s", 0)), _kmh(sp.get("max"), sp.get("unit") or "m/s")))
                    except ValueError:
                        pass
            sections = []
            lanes = road.find("lanes")
            for sec in (lanes.findall("laneSection") if lanes is not None else []):
                speeds = {}
                for lane in sec.iter("lane"):
                    sp = lane.find("speed")
                    if sp is not None and sp.get("max"):
                        try:
                            speeds[int(lane.get("id"))] = _kmh(sp.get("max"), sp.get("unit"))
                        except ValueError:
                            pass
                sections.append((float(sec.get("s", 0)), speeds))
            self.roads[rid] = (sections, types, road.get("junction", "-1") not in ("-1", None))

    def __len__(self):
        return len(self.roads)

    def lookup(self, road_id, s, lane_id):
        """km/h on that lane at that s (None if unknown), and whether the road is a junction connector."""
        r = self.roads.get(str(road_id))
        if r is None:
            return None, False
        sections, types, junction = r
        best = None
        for s0, speeds in sections:
            if s0 <= s + 1e-6:
                best = speeds
        if best and int(lane_id) in best:
            return best[int(lane_id)], junction
        tv = None
        for s0, v in types:
            if s0 <= s + 1e-6:
                tv = v
        return tv, junction


# -- drawing designs with pygame ------------------------------------------------------------------------------------

class Painter:
    """Draws designs into pygame surfaces (fonts cached by size)."""

    def __init__(self, pygame):
        self.pg = pygame
        pygame.font.init()
        self.fonts = {}

    def font(self, px, bold=True):
        px = max(6, int(px))
        f = self.fonts.get((px, bold))
        if f is None:
            f = self.fonts[(px, bold)] = self.pg.font.SysFont("dejavusans,arial,helvetica,sans", px, bold=bold)
        return f

    def _blend(self, surf, rgba, box, draw):
        """Draws a shape with alpha blending (through a temporary surface when it is translucent)."""
        pg = self.pg
        x, y, w, h = box
        if w < 1 or h < 1 or rgba[3] <= 0:
            return
        if rgba[3] >= 255:
            draw(surf, rgba[:3], (0, 0), (x, y, w, h))
            return
        tmp = pg.Surface((int(math.ceil(w)) + 2, int(math.ceil(h)) + 2), pg.SRCALPHA)
        draw(tmp, rgba, (x - 1, y - 1), (1, 1, w, h))
        surf.blit(tmp, (int(x) - 1, int(y) - 1))

    def text(self, surf, text, rgba, box, align="center"):
        x, y, w, h = box
        if not text or w < 2 or h < 2:
            return
        f = self.font(h * 0.86)
        img = f.render(str(text), True, rgba[:3])
        if img.get_width() > w:
            f = self.font(h * 0.86 * w / img.get_width())
            img = f.render(str(text), True, rgba[:3])
        if rgba[3] < 255:
            img.set_alpha(rgba[3])
        ix = x + (w - img.get_width()) / 2 if align == "center" else x
        surf.blit(img, (ix, y + (h - img.get_height()) / 2))

    def element(self, surf, e, pw, ph, t, values, limit=None):
        pg = self.pg
        x, w = e["x"] * pw, e["w"] * pw
        h = e["h"] * ph
        y = ph - e["y"] * ph - h
        r, g, b, a = e["rgba"]
        typ = e["type"]

        def rgba_at(pos=0.5):
            return (r, g, b, int(a * designs.brightness(e["animation"], e["period"], t, pos)))

        def rect(s, col, off, bx):
            pg.draw.rect(s, col, (bx[0], bx[1], math.ceil(bx[2]), math.ceil(bx[3])))

        def ellipse(s, col, off, bx):
            pg.draw.ellipse(s, col, (bx[0], bx[1], math.ceil(bx[2]), math.ceil(bx[3])))

        if typ == "rect":
            self._blend(surf, rgba_at(), (x, y, w, h), rect)
        elif typ == "lightbar":
            n = max(1, e["segments"])
            sw = w / n
            for i in range(n):
                self._blend(surf, rgba_at((i + 0.5) / n), (x + (i + 0.1) * sw, y, 0.8 * sw, h), rect)
        elif typ == "circle":
            self._blend(surf, rgba_at(), (x, y, w, h), ellipse)
        elif typ in ("arrow", "chevrons"):
            self._directional(surf, e, x, y, w, h, rgba_at)
        elif typ == "text":
            self.text(surf, designs.fill(e["text"], values), rgba_at(), (x, y, w, h))
        elif typ == "gauge":
            self._gauge(surf, e, x, y, w, h, values)
        elif typ == "bar":
            v = _num(designs.fill(e["value"], values))
            frac = max(0.0, min(1.0, (v - e["min"]) / max(1e-6, e["max"] - e["min"])))
            col = rgba_at()
            pg.draw.rect(surf, col[:3], (x, y, w, h), 1)
            fill = (x, y, w * frac, h) if w >= h else (x, y + h * (1 - frac), w, h * frac)
            self._blend(surf, col, fill, rect)
        elif typ == "speedsign":
            if limit is None:
                return
            d = min(w, h)
            cx, cy = x + w / 2, y + h / 2
            pg.draw.circle(surf, (255, 255, 255), (int(cx), int(cy)), int(d / 2))
            pg.draw.circle(surf, (210, 20, 20), (int(cx), int(cy)), int(d / 2), max(2, int(d * 0.12)))
            self.text(surf, str(limit), (0, 0, 0, 255), (cx - d * 0.3, cy - d * 0.25, d * 0.6, d * 0.5))

    def _directional(self, surf, e, x, y, w, h, rgba_at):
        pg = self.pg
        d = (e["direction"] or "right").lower()

        def m(s, q):   # s along the direction 0..1, q across -0.5..0.5 (screen y grows down)
            if d == "left":
                return x + (1 - s) * w, y + h / 2 - q * h
            if d == "up":
                return x + w / 2 + q * w, y + (1 - s) * h
            if d == "down":
                return x + w / 2 + q * w, y + s * h
            return x + s * w, y + h / 2 - q * h

        length = w if d in ("left", "right") else h
        if e["type"] == "arrow":
            col = rgba_at()
            shaft = [m(0, 0.15), m(0.55, 0.15), m(0.55, 0.5), m(1, 0), m(0.55, -0.5), m(0.55, -0.15), m(0, -0.15)]
            self._poly(surf, col, shaft)
        else:
            n = max(1, e["segments"])
            for i in range(n):
                s0, s1 = (i + 0.15) / n, (i + 0.75) / n
                col = rgba_at((i + 0.5) / n)
                if col[3] <= 0:
                    continue
                pg.draw.lines(surf, col[:3], False, [m(s0, 0.5), m(s1, 0), m(s0, -0.5)],
                              max(1, int(0.18 * length / n)))

    def _poly(self, surf, col, pts):
        pg = self.pg
        if col[3] <= 0:
            return
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        x0, y0 = min(xs), min(ys)
        if col[3] >= 255:
            pg.draw.polygon(surf, col[:3], pts)
            return
        tmp = pg.Surface((int(max(xs) - x0) + 3, int(max(ys) - y0) + 3), pg.SRCALPHA)
        pg.draw.polygon(tmp, col, [(px - x0 + 1, py - y0 + 1) for px, py in pts])
        surf.blit(tmp, (x0 - 1, y0 - 1))

    def _gauge(self, surf, e, x, y, w, h, values):
        pg = self.pg
        col = e["rgba"][:3]
        cx, cy = x + w / 2, y + h / 2
        rad = min(w, h) / 2 * 0.95
        a0, a1 = e["startAngle"], e["endAngle"]
        lo, hi = e["min"], e["max"]

        def pt(frac, rr):
            ang = math.radians(a0 + (a1 - a0) * frac)
            return cx + rr * math.cos(ang), cy - rr * math.sin(ang)

        majors = max(1, e["segments"])
        for k in range(majors * 2 + 1):
            frac = k / (majors * 2)
            val = lo + (hi - lo) * frac
            red = val >= e["redFrom"]
            inner = rad * (0.82 if k % 2 == 0 else 0.9)
            pg.draw.line(surf, (230, 40, 40) if red else col, pt(frac, inner), pt(frac, rad), 2 if k % 2 == 0 else 1)
        if e["labels"] > 1:
            for k in range(e["labels"]):
                frac = k / (e["labels"] - 1)
                val = (lo + (hi - lo) * frac) * e["labelScale"]
                px, py = pt(frac, rad * 0.66)
                s = rad * 0.16
                self.text(surf, f"{val:.0f}", (*col, 255), (px - s * 1.6, py - s / 2, s * 3.2, s))
        v = max(lo, min(hi, _num(designs.fill(e["value"], values))))
        frac = (v - lo) / max(1e-6, hi - lo)
        pg.draw.line(surf, (255, 90, 40), (cx, cy), pt(frac, rad * 0.86), max(2, int(rad * 0.04)))
        pg.draw.circle(surf, (40, 44, 50), (int(cx), int(cy)), max(2, int(rad * 0.08)))

    def panel(self, design, size, state, t, values, conds, extra=(), limit=None):
        """A display as a pygame surface (with alpha) of `size` pixels."""
        pg = self.pg
        pw, ph = size
        surf = pg.Surface((pw, ph), pg.SRCALPHA)
        surf.fill(design["background"])
        for e in list(designs.elements_for(design, state)) + list(extra):
            if designs.holds(e["when"], conds):
                self.element(surf, e, pw, ph, t, values, limit)
        return surf


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        m = re.search(r"-?\d+(\.\d+)?", str(s or ""))
        return float(m.group()) if m else 0.0


# -- the displays of one station -------------------------------------------------------------------------------------

MESSAGE_PANEL = {"name": "message panel", "width": 0.24, "height": 0.06, "background": "#00000000", "states": []}


def _clock_start(sim_time):
    """Seconds after midnight at the trial start, from simulation.time ("HH:MM", "YYYY-MM-DD HH:MM", "now")."""
    m = re.search(r"(\d{1,2}):(\d{2})", str(sim_time or ""))
    if m and str(sim_time) != "now":
        return int(m.group(1)) * 3600 + int(m.group(2)) * 60
    lt = time.localtime()
    return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec


class InCar:
    """The displays and messages of one station. update() once per tick with the own vehicle's state; draw()
    composes the displays onto the station's screen (or a saved frame)."""

    def __init__(self, cfg, kind, size, log=print, on_event=None, base=None, sim_time=None):
        self.cfg, self.kind, self.size, self.log = cfg or {}, kind, size, log
        self.on_event = on_event or (lambda *a: None)
        self.displays = {}
        for name in ("cluster", "hud", "centre"):
            spec = self.cfg.get(name)
            if not spec:
                continue
            design = DEFAULT_DESIGNS[name] if spec is True else spec.get("design", DEFAULT_DESIGNS[name]) \
                if isinstance(spec, dict) and "states" not in spec else spec
            try:
                d = designs.load(design, "incar", base)
            except (OSError, ValueError) as e:
                log(f"displays: {name}: {e}")
                continue
            rect = spec.get("rect") if isinstance(spec, dict) else None
            self.displays[name] = {"design": d, "rect": self._rect(name, d, rect), "own_rect": bool(rect),
                                   "default": design == DEFAULT_DESIGNS[name], "base": base}
        try:
            self.messages = designs.load_messages(self.cfg, base)
        except (OSError, ValueError) as e:
            log(f"displays: messages: {e}")
            self.messages = []
        for name, disp in self.displays.items():
            self.messages += [dict(m, display=m.get("display") or name) for m in disp["design"]["messages"]]
        if self.messages and "centre" not in self.displays and any(
                self._target(m) == "centre" for m in self.messages):
            d = designs.normalise(MESSAGE_PANEL)
            self.displays["centre"] = {"design": d, "rect": self._rect("centre", d, None)}
        self.fired = set()
        self.shown = []                   # [message, until, display]
        self.t0 = None
        self.distance = 0.0
        self.state = "AUTOMATED" if kind == "av_passenger" else "MANUAL"
        self.takeover = None              # (start, until) of a take-over request
        self.takeover_ctl = None
        self.reaction = None
        self.values, self.conds = {}, {}
        self.clock0 = _clock_start(sim_time)
        self.painter = None
        self._prev_state = None
        # displays.style (cockpit.py): "dashboard" draws into the car's own panels once a CabinView is set
        # (set_cabin, by stationux), "overlay" keeps flat panels at the screen's edges
        self.style = str(self.cfg.get("style") or "dashboard").lower()
        self.cabin = None
        self.wheel_deg = 0.0
        self.max_steer = 70.0        # the car's front wheels' steering lock (deg; stationux reads the physics)
        self.screen_gain, self.hud_gain = 0.92, 0.9
        if self.displays or self.messages:
            log(f"displays: {', '.join(self.displays) or 'none'}; {len(self.messages)} timed messages")

    def set_electric(self, electric=True):
        """The own car is electric: no rev counter (the default cluster becomes cluster_ev, a power meter), no
        engine rpm and no gear number."""
        electric = bool(electric)
        if electric == getattr(self, "electric", False):
            return
        self.electric = electric
        disp = self.displays.get("cluster")
        if disp is not None and disp.get("default"):
            try:
                disp["design"] = designs.load("cluster_ev" if electric else "cluster", "incar", disp.get("base"))
            except (OSError, ValueError) as e:
                self.log(f"displays: cluster_ev: {e}")

    def set_cabin(self, cabin):
        """The station's cockpit.CabinView (dashboard style), or None for flat panels."""
        self.cabin = cabin if self.style != "overlay" else None

    def _rect(self, name, design, rect):
        sw, sh = self.size
        aspect = design["height"] / max(1e-6, design["width"])
        if rect:
            x, y, w = rect[0], rect[1], rect[2]
            h = rect[3] if len(rect) > 3 else w * sw * aspect / sh
            return (int(x * sw), int(y * sh), int(w * sw), int(h * sh))
        cx, cy, w = RECTS[name]
        pw = int(w * sw)
        ph = int(pw * aspect)
        y = int(cy * sh - ph / 2) if cy is not None else int(sh - ph - 0.02 * sh)
        return (int(cx * sw - pw / 2), y, pw, ph)

    def _target(self, m):
        d = (m.get("display") or "").strip().lower()
        if d in ("centrescreen", "center", "centre_screen"):
            d = "centre"
        if d in ("", "all"):
            return "all"
        return d if d in self.displays else "centre"

    def update(self, t, kmh, lights=0, control=None, limit=None, dt=0.05, automation=None, time_left=None):
        """automation: takeover.py's state of the participant's car (automated / requested / manual), when it has
        `automation`; a request shows as TAKEOVER with time_left (s) counting down."""
        if self.t0 is None:
            self.t0 = t
        tt = t - self.t0
        self.distance += max(0.0, kmh) / 3.6 * max(0.0, dt)
        brake = float(getattr(control, "brake", 0.0) or 0.0) if control is not None else 0.0
        throttle = float(getattr(control, "throttle", 0.0) or 0.0) if control is not None else 0.0
        steer = float(getattr(control, "steer", 0.0) or 0.0) if control is not None else 0.0
        reverse = bool(getattr(control, "reverse", False)) if control is not None else False
        if control is not None:      # the steering wheel's angle in the image (cockpit.WheelSpin)
            from .cockpit import wheel_degrees
            self.wheel_deg = wheel_degrees(steer, self.max_steer)
        # take-over request: over at its end; a response is the first brake, throttle or steering after it
        if self.takeover is not None:
            start, until = self.takeover
            if self.takeover_ctl is not None and self.reaction is None and control is not None:
                b0, th0, s0 = self.takeover_ctl
                if brake > b0 + 0.2 or throttle > th0 + 0.2 or abs(steer - s0) > 0.1:
                    self.reaction = tt - start
                    self.on_event("takeover_response", "", f"{self.reaction:.2f} s")
            if tt >= until:
                self.takeover = None
        base = {"automated": "AUTOMATED", "requested": "TAKEOVER", "manual": "MANUAL"}.get(automation) or \
            ("AUTOMATED" if self.kind == "av_passenger" else "MANUAL")
        state = "TAKEOVER" if self.takeover is not None else base
        started = state != self._prev_state
        self._prev_state = state
        self.state = state
        for i, m in enumerate(self.messages):
            if i in self.fired:
                continue
            due = (m["at"] >= 0 and tt >= m["at"]) or (m["atDistance"] >= 0 and self.distance >= m["atDistance"]) \
                or (started and m["onState"] and m["onState"].upper() == state)
            if not due:
                continue
            self.fired.add(i)
            self.shown.append([m, tt + max(0.1, m["seconds"]), self._target(m)])
            if _truthy(m.get("takeover")):
                self.takeover = (tt, tt + max(0.1, m["seconds"]))
                self.takeover_ctl = (brake, throttle, steer) if control is not None else None
                self.reaction = None
                self.on_event("takeover_request", m["text"], f"{m['seconds']:.1f} s")
            self.on_event("message", m["text"], f"{self._target(m)} {m['seconds']:.1f} s", m)
        self.shown = [s for s in self.shown if tt < s[1]]
        gear, rpm = _gear_rpm(kmh, throttle, reverse, control)
        power = 0
        if getattr(self, "electric", False):      # no engine: no rpm, a single fixed gear
            gear = "R" if reverse else ("P" if kmh < 0.5 and throttle < 0.05 else "D")
            rpm = 0.0
            power = int(round(100 * throttle if throttle > 0.02 else (-40 * brake if kmh > 3 else 0)))
        left_time = max(0.0, self.takeover[1] - tt) if self.takeover is not None else \
            max(0.0, time_left) if (state == "TAKEOVER" and time_left is not None) else 0.0
        blink_on = (t * 1.5) % 1.0 < 0.5
        lights = int(lights or 0)
        clock = self.clock0 + tt
        self.values = {"speed": int(round(kmh)), "limit": limit if limit is not None else "--", "rpm": int(rpm),
                       "gear": gear, "power": power, "clock": f"{int(clock // 3600) % 24:02d}:{int(clock // 60) % 60:02d}",
                       "trial_time": f"{tt:.0f}", "distance": f"{self.distance / 1000:.1f}",
                       "message": self.shown[-1][0]["text"] if self.shown else "",
                       "mode": {"AUTOMATED": "Automated", "TAKEOVER": "Take over"}.get(state, "Manual"),
                       "time_left": f"{left_time:.1f}", "reaction": f"{self.reaction:.2f}" if self.reaction else "-",
                       "slider": "-"}
        self.limit = limit
        self.conds = {"left": bool(lights & LEFT_BLINKER) and blink_on, "right": bool(lights & RIGHT_BLINKER) and blink_on,
                      "blinkers": bool(lights & (LEFT_BLINKER | RIGHT_BLINKER)),
                      "headlights": bool(lights & (LOW_BEAM | HIGH_BEAM)), "braking": brake > 0.1, "reverse": reverse,
                      "automated": state == "AUTOMATED", "manual": state == "MANUAL", "takeover": state == "TAKEOVER",
                      "message": bool(self.shown), "limit": limit is not None,
                      "over_limit": limit is not None and kmh > limit + 3}
        self.tt = tt

    def _message_elements(self, name):
        els = []
        for m, _until, target in self.shown:
            if target not in ("all", name):
                continue
            mx, my, mw, mh = m["x"], m["y"], m["w"], m["h"]
            if name == "cluster" and abs(mx - 0.05) < 1e-6 and abs(mw - 0.9) < 1e-6:
                mx, my, mw, mh = 0.375, 0.02, 0.25, 0.27     # between the dials, as coupled-sim
            if name == "centre" and self.displays[name]["design"]["name"] == "message panel":
                mx, my, mw, mh = 0.0, 0.0, 1.0, 1.0
            back = designs._norm_element({"type": "rect", "x": mx, "y": my, "w": mw, "h": mh,
                                          "color": m.get("background", "#000000B0"),
                                          "animation": m.get("animation"), "period": m.get("period", 1.0)})
            txt = designs._norm_element({"type": "text", "x": mx + mw * 0.03, "y": my + mh * 0.1, "w": mw * 0.94,
                                         "h": mh * 0.8, "color": m.get("color", "#FFFFFF"), "text": m["text"],
                                         "animation": m.get("animation"), "period": m.get("period", 1.0)})
            els += [back, txt]
        return els

    def _panel_for(self, name, disp, size):
        mine = [m for m, _u, target in self.shown if target in ("all", name)]
        values = dict(self.values, message=mine[-1]["text"] if mine else "")    # {message}: this display's
        conds = dict(self.conds, message=bool(mine))
        return self.painter.panel(disp["design"], size, self.state, self.tt, values, conds,
                                  self._message_elements(name), self.limit)

    def draw(self, surface):
        if self.t0 is None:
            return
        import pygame
        if self.painter is None:
            self.painter = Painter(pygame)
        flat = dict(self.displays)
        if self.cabin is not None:          # displays.style dashboard (cockpit.py): on the car's own panels
            flat = self._draw_dashboard(surface, pygame)
        for name, disp in flat.items():
            x, y, w, h = disp["rect"]
            surface.blit(self._panel_for(name, disp, (w, h)), (x, y))

    def _draw_dashboard(self, surface, pygame):
        """Draws the displays into the car's cluster, centre-screen and HUD panels (cockpit.CabinView) and turns
        the steering wheel; returns the displays still to draw flat (those with their own `rect`)."""
        import numpy as np
        cab = self.cabin
        left = {}
        arr = pygame.surfarray.pixels3d(surface)
        try:
            img = arr.swapaxes(0, 1)
            spinning = cab.spin is not None and cab.spin.lift(img, self.wheel_deg)
            for name, disp in self.displays.items():
                if disp.get("own_rect"):
                    left[name] = disp
                    continue
                d = disp["design"]
                aspect = d["height"] / max(1e-6, d["width"])
                tex = cab.tex_size(name, aspect)
                if tex is None:
                    continue
                panel = self._panel_for(name, disp, tex)
                rgba = np.dstack([pygame.surfarray.array3d(panel), pygame.surfarray.array_alpha(panel)])
                rgba = np.ascontiguousarray(rgba.swapaxes(0, 1))
                if name == "hud":
                    cab.draw(img, name, rgba, additive=True, gain=self.hud_gain)
                else:
                    cab.draw(img, name, rgba, gain=self.screen_gain, spinning=spinning)
            if spinning:
                cab.spin.put_back(img)
        finally:
            del arr
        return left


def _truthy(v):
    return v is True or str(v).strip().lower() in ("1", "true", "yes", "on")


def _gear_rpm(kmh, throttle, reverse, control):
    from .soundscape import engine_state
    if reverse:
        return "R", 1200.0
    g, rpm = engine_state(kmh, throttle)
    if g == 0:
        return ("P" if throttle < 0.05 else "D"), rpm
    return f"D{g}", rpm


class LimitWatcher:
    """The speed limit where the own vehicle drives, from the map's OpenDRIVE (parsed in a thread at the start),
    looked up twice a second; on a junction connector the last road's limit stays."""

    def __init__(self, cmap, log=print):
        self.cmap, self.log = cmap, log
        self.limits = None
        self.value = None
        self.t_next = 0.0
        threading.Thread(target=self._load, name="speed limits", daemon=True).start()

    def _load(self):
        try:
            t0 = time.time()
            self.limits = SpeedLimits(self.cmap.to_opendrive())
            self.log(f"displays: speed limits of {len(self.limits)} roads ({time.time() - t0:.1f} s)")
        except Exception as e:
            self.log(f"displays: no speed limits from the map ({e})")

    def update(self, t, location):
        if self.limits is None or t < self.t_next:
            return self.value
        self.t_next = t + 0.5
        try:
            wp = self.cmap.get_waypoint(location)
        except RuntimeError:
            return self.value
        if wp is None:
            return self.value
        v, junction = self.limits.lookup(wp.road_id, wp.s, wp.lane_id)
        if v is not None and not (junction or wp.is_junction):
            self.value = v
        return self.value
