"""Design files for eHMIs and in-car displays (coupled-sim's eHMI designer, #131, and in-car displays, #133/#157).

A design is a JSON or YAML file (the same format as coupled-sim's StreamingAssets/eHMI and InCar files, so those
files work here unchanged):

    name: "Cyan light bar"
    width: 1.2                      # metres (an eHMI) or the panel's aspect (a display)
    height: 0.08
    background: "#101010"           # #RRGGBB or #RRGGBBAA, or [r, g, b(, a)]; #00000000 = none
    placement: front                # eHMIs: front, rear, roof, windshield, bumper, road (or a list)
    states:
      - state: yielding             # an eHMI state (coupled-carla's names, or coupled-sim's WALK/STOP/DISABLED);
        elements:                   # displays: MANUAL, AUTOMATED, TAKEOVER; ANY is always shown
          - {type: lightbar, segments: 16, color: "#00E5FF", animation: pulse, period: 1.2}
    messages: []                    # displays: timed messages (see incar.py)

Elements (x, y, w, h from 0 to 1 in the panel, from the bottom left): lightbar (segments), rect, circle, arrow and
chevrons (direction left/right/up/down, segments), text (with {placeholders}), and on displays gauge (value,
min, max, labels, redFrom, labelScale, startAngle, endAngle), bar (value, min, max) and speedsign ({limit}).
Animations: steady, pulse, blink, sweep, sweepleft, sweepout, sweepin, each with a period in seconds. `when`
shows an element only while a condition holds ("left", "!message", ...).
"""
import csv
import io
import json
import math
from pathlib import Path

import yaml

DATA = Path(__file__).resolve().parent / "data"
ANIMATIONS = ("steady", "pulse", "blink", "sweep", "sweepleft", "sweepout", "sweepin")
# coupled-sim's eHMI states and coupled-carla's: a design written for one works with the other
STATE_ALIASES = {
    "yielding": ["WALK"], "go": ["WALK"], "driving": ["STOP"], "wait": ["STOP"], "off": ["DISABLED"],
    "WALK": ["yielding", "go"], "STOP": ["driving", "wait"], "DISABLED": ["off"],
}

ELEMENT_DEFAULTS = {"type": "rect", "x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0, "color": "#00FFFF", "text": "",
                    "segments": 12, "direction": "right", "animation": "steady", "period": 1.0, "when": "",
                    "value": "", "min": 0.0, "max": 100.0, "startAngle": 225.0, "endAngle": -45.0, "labels": 0,
                    "redFrom": 1e30, "labelScale": 1.0}
MESSAGE_DEFAULTS = {"at": -1.0, "atDistance": -1.0, "onState": "", "seconds": 5.0, "display": "", "text": "",
                    "color": "#FFFFFF", "background": "#000000B0", "x": 0.05, "y": 0.05, "w": 0.9, "h": 0.22,
                    "animation": "steady", "period": 1.0}


def parse_color(c, default=(0, 255, 255, 255)):
    """(r, g, b, a) 0-255 from "#RGB", "#RRGGBB", "#RRGGBBAA", [r, g, b] or [r, g, b, a]."""
    if c is None:
        return default
    if isinstance(c, (list, tuple)):
        v = [int(round(float(x))) for x in c] + [255]
        return tuple(max(0, min(255, x)) for x in v[:4])
    s = str(c).strip().lstrip("#")
    try:
        if len(s) == 3:
            s = "".join(ch * 2 for ch in s)
        if len(s) == 6:
            s += "FF"
        if len(s) == 8:
            return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4, 6))
    except ValueError:
        pass
    return default


def brightness(animation, period, t, pos=0.5):
    """Brightness 0..1 of an animated element at time t (s since its state started); pos 0..1 along it (sweeps).
    As coupled-sim's DesignedEhmi: unlit lamps of a sweep stay at 0.12."""
    ph = (t / max(0.05, float(period or 1.0))) % 1.0
    a = (animation or "steady").lower()
    if a == "pulse":
        return 0.25 + 0.75 * (0.5 - 0.5 * math.cos(ph * 2 * math.pi))
    if a == "blink":
        return 1.0 if ph < 0.5 else 0.0
    if a == "sweep":
        return 1.0 if pos <= ph else 0.12
    if a == "sweepleft":
        return 1.0 if 1.0 - pos <= ph else 0.12
    if a == "sweepout":
        return 1.0 if abs(pos - 0.5) * 2.0 <= ph else 0.12
    if a == "sweepin":
        return 1.0 if 1.0 - abs(pos - 0.5) * 2.0 <= ph else 0.12
    return 1.0


def _norm_element(e):
    out = dict(ELEMENT_DEFAULTS)
    out.update(e or {})
    out["type"] = str(out["type"]).lower()
    for k in ("x", "y", "w", "h", "period", "min", "max", "startAngle", "endAngle", "labelScale"):
        out[k] = float(out[k])
    out["redFrom"] = min(1e30, float(out["redFrom"])) if out["redFrom"] is not None else 1e30   # JSON-safe
    out["segments"] = int(out["segments"])
    out["labels"] = int(out["labels"])
    out["rgba"] = parse_color(out["color"])
    return out


def norm_message(m):
    """A message with every field, accepting coupled-carla's snake_case (at_m, on_state) as well."""
    out = dict(MESSAGE_DEFAULTS)
    m = dict(m or {})
    if "at_m" in m:
        m["atDistance"] = m.pop("at_m")
    if "on_state" in m:
        m["onState"] = m.pop("on_state")
    out.update(m)
    for k in ("at", "atDistance", "seconds", "x", "y", "w", "h", "period"):
        out[k] = float(out[k]) if out[k] not in (None, "") else MESSAGE_DEFAULTS[k]
    out["text"] = str(out.get("text") or "")
    return out


def normalise(d, name=""):
    """A design dict with defaults filled in and states keyed by name: {"states": {name: [elements]}}."""
    d = dict(d or {})
    out = {"name": d.get("name", name), "width": float(d.get("width", 1.2)), "height": float(d.get("height", 0.15)),
           "background": parse_color(d.get("background", "#000000")), "placement": d.get("placement"),
           "offset": d.get("offset"), "states": {}, "messages": [norm_message(m) for m in d.get("messages") or []]}
    states = d.get("states") or []
    if isinstance(states, dict):        # YAML shorthand: {state: [elements]}
        states = [{"state": k, "elements": v} for k, v in states.items()]
    for s in states:
        out["states"][str(s.get("state", "ANY"))] = [_norm_element(e) for e in s.get("elements") or []]
    return out


def find(name, kind="ehmi", base=None):
    """The file of a design: a path (absolute, or relative to `base` or the project folder), or a name in
    coupledcarla/data/<kind>/ (.json, .yaml or .yml)."""
    p = Path(str(name)).expanduser()
    cands = []
    if p.suffix:
        cands += [p] if p.is_absolute() else ([Path(base) / p] if base else []) + [DATA.parent.parent / p, p]
    else:
        for ext in (".json", ".yaml", ".yml"):
            cands += [DATA / kind / f"{name}{ext}"]
            if base:
                cands += [Path(base) / f"{name}{ext}"]
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError(f"no {kind} design {name!r} (looked in {', '.join(str(c) for c in cands[:4])})")


def load(name_or_dict, kind="ehmi", base=None):
    """A normalised design from a dict (inline in the experiment), a name in data/<kind>/ or a file path."""
    if isinstance(name_or_dict, dict):
        return normalise(name_or_dict, name_or_dict.get("name", "inline"))
    path = find(name_or_dict, kind, base)
    text = path.read_text(encoding="utf-8")
    d = json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)
    return normalise(d, path.stem)


def elements_for(design, state):
    """The elements shown in a state: the state's own (by name, case-insensitive, or by its coupled-sim/
    coupled-carla alias) after those of ANY."""
    st = design["states"]
    out = list(st.get("ANY", []))
    if state is None:
        return out
    names = {k.lower(): k for k in st}
    key = names.get(str(state).lower())
    if key is None:
        for alias in STATE_ALIASES.get(str(state), []) + STATE_ALIASES.get(str(state).lower(), []):
            key = names.get(alias.lower())
            if key:
                break
    if key is not None and key != "ANY":
        out += st[key]
    return out


def state_names(design):
    return [s for s in design["states"] if s not in ("ANY",)]


def fill(text, values):
    """Text with {placeholders} filled from `values` (unknown ones stay as written)."""
    if not text or "{" not in text:
        return text
    for k, v in values.items():
        token = "{" + k + "}"
        if token in text:
            text = text.replace(token, "" if v is None else str(v))
    return text


def holds(condition, conds):
    """Whether a `when` condition holds; "!x" negates; unknown conditions hold."""
    c = (condition or "").strip()
    if not c:
        return True
    neg = c.startswith("!")
    c = c.lstrip("!").lower()
    v = bool(conds.get(c, True))
    return not v if neg else v


def parse_schedule(text):
    """A message schedule in coupled-sim's CSV format: a header (at, seconds, display, text, and optionally
    color, atDistance, onState, background; ";" or "," separated) and one message per row; # comments."""
    lines = [ln for ln in text.replace("\r", "").split("\n")]
    if not lines or not lines[0].strip():
        return []
    sep = ";" if ";" in lines[0] else ","
    rows = list(csv.reader(io.StringIO("\n".join(ln for ln in lines if ln.strip() and not ln.lstrip().startswith("#"))),
                           delimiter=sep))
    head = [h.strip().lower() for h in rows[0]]
    if "text" not in head:
        raise ValueError("a message schedule needs a 'text' column")
    names = {"at": "at", "seconds": "seconds", "display": "display", "text": "text", "color": "color",
             "atdistance": "atDistance", "at_m": "atDistance", "onstate": "onState", "on_state": "onState",
             "background": "background", "takeover": "takeover", "haptic": "haptic", "sound": "sound", "say": "say"}
    out = []
    for r in rows[1:]:
        m = {}
        for h, v in zip(head, r):
            v = v.strip()
            if h in names and v != "":
                m[names[h]] = v
        out.append(norm_message(m))
    return out


def load_messages(displays_cfg, base=None):
    """The timed messages of a `displays` section: its `messages` list plus its `messages_file` (CSV)."""
    msgs = [norm_message(m) for m in (displays_cfg.get("messages") or [])]
    f = displays_cfg.get("messages_file")
    if f:
        p = Path(f)
        if not p.is_absolute():
            p = (Path(base) / p) if base and (Path(base) / p).exists() else DATA.parent.parent / p
        msgs += parse_schedule(p.read_text(encoding="utf-8"))
    return msgs
