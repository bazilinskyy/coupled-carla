"""Numbers for running sessions with people: what each station reports, and whether a trial met the thresholds.

Stations send `stats` every 5 s (logged as `station_stats` events and the last one per station in summary.json):
round trip to the host, loop and camera frame rates, how many world ticks and camera images they got, the
tick-to-image latency, how long the camera took to deliver its first image after the trial started, and which of
sound, in-car displays, haptics and station-drawn eHMIs are on. tools/station_dryrun.py runs a short trial with
bot stations and checks those numbers against THRESHOLDS; summarise() and check() also read the logs of a real
session (python tools/station_dryrun.py --report logs/<experiment>/<time>_trial0).

No CARLA here (the tests run without it).
"""
import csv
import json
import time
from pathlib import Path

LINK_MB_S = 117.0          # what gigabit Ethernet carried between win5090 and lin4080 (TCP, 2026-11-15)

THRESHOLDS = {
    "max_join_s": 20.0,        # station start -> welcome from the host (control channel up; local stations only)
    "max_ready_s": 15.0,       # trial running -> the station's first camera image (connect, spawn, first frame)
    "max_rtt_ms": 30.0,        # median round trip to the host (13 Nov: 0.3-20 ms local, 12-23 ms on the LAN)
    "min_fps_share": 0.9,      # the station's loop rate against the host's tick rate
    "min_image_share": 0.9,    # camera images received against those expected (ticks, or camera_hz)
    "max_image_p95_ms": 250.0,  # tick -> camera image at the station, 95th percentile
    "max_dropped_share": 0.05,  # world ticks the station did not see (gaps in its frame numbers)
    "min_rtf": 0.95,           # the trial's real-time factor (the host)
}


def stream_mb_s(size, hz):
    """MB/s of one camera stream: CARLA sends raw BGRA (4 bytes a pixel), one image per tick unless sensor_tick."""
    w, h = (int(v) for v in str(size).lower().split("x"))
    return w * h * 4 * float(hz) / 1e6


def ux_flags(welcome, ux):
    """Which of the station's extras are on: sound (and whether it reaches an audio device), in-car displays,
    haptics, station-drawn eHMIs. welcome: the host's welcome; ux: the station's StationUX or None."""
    w = welcome or {}
    snd = bool((w.get("sound") or {}).get("enabled"))
    out = getattr(ux, "output", None)
    disp = w.get("displays") or {}
    return {"sound": snd,
            "audio_device": bool(out is not None and getattr(out, "channel", None) is not None),
            "displays": bool(getattr(ux, "incar", None) is not None) if ux is not None else
            any(disp.get(k) for k in ("cluster", "hud", "centre", "messages")),
            "haptics": bool(getattr(ux, "haptics", None) is not None) if ux is not None else
            bool((w.get("haptics") or {}).get("enabled")),
            "ehmi_station": bool(getattr(ux, "ehmi_draw", None)) if ux is not None else
            (bool(w.get("ehmi_designs")) and w.get("ehmi_render", "world") in ("station", "both"))}


class Counter:
    """A station's frame and image counts for its stats (thread-safe enough: the image callback only adds)."""

    def __init__(self, now=time.time):
        self.now = now
        self.first_frame = self.last_frame = None
        self.frames = 0                 # world ticks seen (snapshots)
        self.images = 0
        self.t_running = None           # when the trial started here
        self.ready_s = None             # trial start -> first camera image
        self._win_t0, self._win_frames, self._win_images = now(), 0, 0
        self.fps = self.image_fps = 0.0

    def running(self):
        self.t_running = self.now()

    def frame(self, frame):
        self.first_frame = frame if self.first_frame is None else self.first_frame
        self.last_frame = frame
        self.frames += 1
        self._win_frames += 1

    def image(self):
        self.images += 1
        self._win_images += 1
        if self.ready_s is None and self.t_running is not None:
            self.ready_s = self.now() - self.t_running

    def window(self, every=5.0):
        """True (and the rates updated) once every `every` seconds."""
        dt = self.now() - self._win_t0
        if dt < every:
            return False
        self.fps, self.image_fps = self._win_frames / dt, self._win_images / dt
        self._win_t0, self._win_frames, self._win_images = self.now(), 0, 0
        return True

    def stats(self):
        span = (self.last_frame - self.first_frame + 1) if self.first_frame is not None else 0
        return {"fps": round(self.fps, 1), "image_fps": round(self.image_fps, 1), "frames": self.frames,
                "images": self.images, "span": span,
                "ready_s": round(self.ready_s, 2) if self.ready_s is not None else None}


# -- reading a trial ------------------------------------------------------------------------------------------

def read_trial(trial_dir):
    """A trial folder's station numbers: {trial: summary fields, step, stations: {role: {joined, left, stats: [..]}},
    missing: [roles]}."""
    d = Path(trial_dir)
    summary = json.loads((d / "summary.json").read_text()) if (d / "summary.json").exists() else {}
    step = None
    if (d / "config.yaml").exists():
        try:
            import yaml
            step = (yaml.safe_load((d / "config.yaml").read_text()).get("simulation") or {}).get("step")
        except Exception:  # noqa: BLE001 - a broken config only loses the step
            step = None
    stations, missing = {}, []
    for role in summary.get("stations") or []:
        stations.setdefault(role, {"joined": None, "left": None, "stats": []})
    if (d / "events.csv").exists():
        with open(d / "events.csv", newline="") as f:
            for row in csv.DictReader(f, delimiter=";"):
                ev, detail = row.get("event"), row.get("detail") or ""
                t = float(row.get("timestamp") or 0)
                if ev == "station_stats":
                    try:
                        st = json.loads(detail)
                    except ValueError:
                        continue
                    st["t"] = t
                    stations.setdefault(st.get("role"), {"joined": None, "left": None, "stats": []})["stats"].append(st)
                elif ev in ("station_joined", "station_left"):
                    s = stations.setdefault(detail, {"joined": None, "left": None, "stats": []})
                    s["joined" if ev == "station_joined" else "left"] = t
                elif ev == "stations_missing":
                    missing += [r for r in detail.split(",") if r]
                elif ev == "trial_start" and detail:
                    try:
                        for role in json.loads(detail).get("stations") or []:
                            stations.setdefault(role, {"joined": None, "left": None, "stats": []})
                    except ValueError:
                        pass
    for role, st in (summary.get("station_stats") or {}).items():
        s = stations.setdefault(role, {"joined": None, "left": None, "stats": []})
        if not s["stats"]:
            s["stats"].append(st)
    return {"dir": str(d), "trial": summary, "step": step or 0.02, "stations": stations, "missing": missing}


def _median(xs):
    xs = sorted(x for x in xs if x is not None)
    return xs[len(xs) // 2] if xs else None


def summarise(trial, join_s=None, camera_hz=None):
    """One row per station: the numbers check() looks at. join_s: {role: s} measured by the dry run;
    camera_hz: {role: Hz} the stations' camera rate (0 or None: every tick)."""
    summ = trial["trial"]
    step = float(trial.get("step") or 0.02)
    wall = summ.get("wall_s") or 0
    host_tps = (summ.get("ticks") or 0) / wall if wall else None
    rows = []
    for role, s in sorted(trial["stations"].items()):
        stats = s["stats"]
        last = stats[-1] if stats else {}
        steady = stats[1:] if len(stats) > 2 else stats     # the first comes at the start (rates 0)
        lat = (last.get("latency") or {}).get("tick_to_image_ms") or {}
        hz = last.get("camera_hz", (camera_hz or {}).get(role))
        span, frames, images = last.get("span"), last.get("ticks_seen", last.get("frames")), last.get("images")
        if images is not None and span:
            expected = span if not hz else min(span, span * step * float(hz))
            image_share = images / expected if expected else None
        else:
            image_share = None
        rows.append({
            "role": role, "host": last.get("host"), "joined": s["joined"] is not None or bool(stats),
            "join_s": (join_s or {}).get(role), "ready_s": last.get("ready_s"),
            "rtt_ms": _median([x.get("rtt_ms") for x in steady]),
            "rtt_max_ms": max([x.get("rtt_ms") for x in stats if x.get("rtt_ms") is not None], default=None),
            "fps": _median([x.get("fps") for x in steady]),
            "image_fps": _median([x.get("image_fps") for x in steady]),
            "host_tps": round(host_tps, 1) if host_tps else None,
            "image_median_ms": lat.get("median"), "image_p95_ms": lat.get("p95"),
            "dropped": (span - frames) if span and frames is not None else None,
            "dropped_share": (span - frames) / span if span and frames is not None else None,
            "image_share": image_share, "camera_hz": hz, "camera_on": last.get("camera_on"),
            "ux": last.get("ux"), "stats": len(stats), "left": s["left"],
        })
    return rows


def check(row, th=None):
    """[(metric, value, limit, ok)] for one summarise() row; a missing value fails only where it means the
    station did not work (no stats, no images)."""
    th = {**THRESHOLDS, **(th or {})}
    out = []

    def add(name, value, limit, ok):
        out.append((name, value, limit, bool(ok)))
    add("joined", row["joined"], True, row["joined"])
    add("stats", row["stats"], "> 0 (the station ran the trial)", row["stats"] > 0)
    if row.get("join_s") is not None:
        add("join_s", row["join_s"], th["max_join_s"], row["join_s"] <= th["max_join_s"])
    add("ready_s", row["ready_s"], th["max_ready_s"],
        row["ready_s"] is not None and row["ready_s"] <= th["max_ready_s"])
    if row["rtt_ms"] is not None:
        add("rtt_ms", row["rtt_ms"], th["max_rtt_ms"], row["rtt_ms"] <= th["max_rtt_ms"])
    if row["fps"] is not None and row["host_tps"]:
        share = row["fps"] / row["host_tps"]
        add("fps_share", round(share, 2), th["min_fps_share"], share >= th["min_fps_share"])
    add("image_share", None if row["image_share"] is None else round(row["image_share"], 2), th["min_image_share"],
        row["image_share"] is not None and row["image_share"] >= th["min_image_share"])
    if row["image_p95_ms"] is not None:
        add("image_p95_ms", row["image_p95_ms"], th["max_image_p95_ms"], row["image_p95_ms"] <= th["max_image_p95_ms"])
    if row["dropped_share"] is not None:
        add("dropped_share", round(row["dropped_share"], 3), th["max_dropped_share"],
            row["dropped_share"] <= th["max_dropped_share"])
    return out


def check_trial(trial, th=None):
    th = {**THRESHOLDS, **(th or {})}
    rtf = trial["trial"].get("rtf")
    return [("rtf", rtf, th["min_rtf"], rtf is not None and rtf >= th["min_rtf"])]


def _fmt(v, nd=1):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def table(rows, checks):
    """The per-station report as plain text."""
    head = ["role", "machine", "join s", "ready s", "rtt ms", "fps", "img fps", "img ms med/p95", "dropped",
            "images", "sound", "audio dev", "displays", "ehmi@st", "result"]
    lines = []
    for row, chk in zip(rows, checks):
        ux = row.get("ux") or {}
        bad = [c[0] for c in chk if not c[3]]
        lines.append([row["role"], row.get("host") or "-", _fmt(row.get("join_s")), _fmt(row.get("ready_s")),
                      _fmt(row["rtt_ms"]), f"{_fmt(row['fps'])}/{_fmt(row['host_tps'])}", _fmt(row["image_fps"]),
                      f"{_fmt(row['image_median_ms'], 0)}/{_fmt(row['image_p95_ms'], 0)}",
                      "-" if row["dropped"] is None else f"{row['dropped']} ({100 * row['dropped_share']:.1f}%)",
                      "-" if row["image_share"] is None else f"{100 * row['image_share']:.0f}%",
                      _fmt(ux.get("sound")), _fmt(ux.get("audio_device")), _fmt(ux.get("displays")),
                      _fmt(ux.get("ehmi_station")), "ok" if not bad else "FAIL " + ",".join(bad)])
    widths = [max(len(str(r[i])) for r in [head] + lines) for i in range(len(head))]
    fmt = "  ".join(f"{{:{w}}}" for w in widths)
    return "\n".join([fmt.format(*head)] + [fmt.format(*r) for r in lines])
