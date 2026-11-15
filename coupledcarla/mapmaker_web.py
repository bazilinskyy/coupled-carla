"""The map maker as a page in the browser: python -m coupledcarla.mapmaker --web

A small local web server (Python's standard library only) on 127.0.0.1: a world map to search or click a place on,
a square that follows the chosen size, the guessed country, driving side and traffic, a "Build this map" button with
live progress in plain words, then the preview picture, the summary and how to drive it. It listens on this computer
only and refuses requests that come from other web sites.
"""
import json
import re
import threading
import time
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from . import mapmaker as mm

PAGE = Path(__file__).resolve().parent / "data" / "mapmaker.html"
NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,60}$")


class Jobs:
    """One map build at a time, in a thread; the page polls its state."""

    def __init__(self):
        self.lock = threading.Lock()
        self.job = None
        self.drive_proc = None
        self.drive_name = None

    def running(self):
        return self.job is not None and self.job["state"] == "running"

    def start(self, req):
        with self.lock:
            if self.running():
                raise mm.MapMakerError("A map is already being built.",
                                       "Wait until it is ready; then build the next one.")
            p = make_plan(req)
            self.job = {"id": uuid.uuid4().hex[:8], "state": "running", "started": time.time(), "plan": p,
                        "steps": [{"step": s, "title": t, "state": "pending", "text": ""} for s, t in mm.STEPS],
                        "notes": [], "summary": None, "error": None, "advice": None, "finished": None}
            self.job["steps"][0].update(state="done", text=p["place"]["full"])
            threading.Thread(target=self._run, args=(self.job, req), daemon=True).start()
            return self.job

    def _emit(self, job):
        def emit(ev):
            if ev["state"] == "note":
                job["notes"].append(ev["text"])
                return
            for s in job["steps"]:
                if s["step"] == ev["step"]:
                    s.update(state=ev["state"], text=ev["text"])
        return emit

    def _run(self, job, req):
        try:
            p = job["plan"]
            for w in p["warnings"]:
                job["notes"].append(w)
            exp = mm.write_experiment(p, query=req.get("query") or None)
            job["summary"] = mm.build(exp, self._emit(job), force=bool(req.get("force")))
            job["state"] = "done"
        except mm.MapMakerError as err:
            job.update(state="failed", error=str(err), advice=err.advice)
        except Exception as err:  # noqa: BLE001 - shown on the page in plain words
            job.update(state="failed", error="Something went wrong while building the map.",
                       advice=f"Try again once; if it fails again, send the build log to whoever set this up "
                              f"({type(err).__name__}: {err}).")
        finally:
            job["finished"] = time.time()

    def view(self):
        j = self.job
        if j is None:
            return {"state": "idle"}
        end = j["finished"] or time.time()
        return dict(j, elapsed=round(end - j["started"]))

    def drive(self, name):
        st = mm.carla_status()
        exp = mm.PLACES / f"{name}.yaml"
        cmd = mm.shown_command(mm.drive_command(exp))
        if not exp.exists():
            raise mm.MapMakerError("That map is not here any more.", "Build it again.")
        if not st["ok"]:
            return {"started": False, "why": st["why"], "how": st["how"], "command": cmd}
        with self.lock:
            if self.drive_proc is not None and self.drive_proc.poll() is None:
                return {"started": False, "why": f"A drive of {self.drive_name} is still running.",
                        "how": "Close its window (Esc) first.", "command": cmd}
            self.drive_proc = mm.drive(exp, wait=False)
            self.drive_name = name
        return {"started": True, "command": cmd,
                "how": "CARLA is starting; its window opens in about a minute. Drive with W/S/A/D or the arrow keys, "
                       "Esc to stop."}


def make_plan(req):
    size = req.get("size") or "medium"
    name = (req.get("name") or "").strip() or None
    found = req.get("found")
    if isinstance(found, dict):          # a result of /api/search, sent back by the page
        try:
            found = dict(found, lat=float(found["lat"]), lon=float(found["lon"]))
        except (KeyError, TypeError, ValueError):
            found = None
    else:
        found = None
    if found is None:
        try:
            lat, lon = float(req["lat"]), float(req["lon"])
        except (KeyError, TypeError, ValueError):
            raise mm.MapMakerError("Choose a place first.", "Search for it or click on the map.")
        if not (-85 <= lat <= 85 and -180 <= lon <= 180):
            raise mm.MapMakerError("That point is off the map.", "Click on land between the polar circles.")
        found = mm.reverse(lat, lon)
    return mm.plan(size=size, name=name, pedestrians=bool(req.get("pedestrians")), found=found)


def make_handler(jobs, port):
    allowed = {f"127.0.0.1:{port}", f"localhost:{port}"}

    class Handler(BaseHTTPRequestHandler):
        server_version = "coupled-carla-mapmaker"

        def log_message(self, fmt, *args):     # quiet: the terminal is for the user
            pass

        def _send(self, code, body, ctype="application/json; charset=utf-8", cache=False):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False, default=str).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Cache-Control", "max-age=60" if cache else "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _err(self, err, code=400):
            self._send(code, {"error": str(err), "advice": getattr(err, "advice", "")})

        def _host_ok(self):
            # DNS rebinding: only our own address in Host
            return self.headers.get("Host", "") in allowed

        def do_GET(self):  # noqa: N802
            if not self._host_ok():
                return self._send(403, {"error": "forbidden"})
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path in ("/", "/index.html"):
                    return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
                if u.path == "/api/search":
                    return self._send(200, {"results": mm.search(q.get("q", ""), limit=5)})
                if u.path == "/api/job":
                    return self._send(200, jobs.view())
                if u.path == "/api/maps":
                    return self._send(200, {"maps": mm.built_maps()})
                if u.path == "/api/carla":
                    return self._send(200, mm.carla_status())
                m = re.fullmatch(r"/maps/([a-z0-9_]+)/preview\.png", u.path)
                if m and NAME_RE.match(m.group(1)):
                    f = mm.ROOT / "maps" / m.group(1) / "preview.png"
                    if f.exists():
                        return self._send(200, f.read_bytes(), "image/png")
                return self._send(404, {"error": "not found"})
            except mm.MapMakerError as err:
                return self._err(err)

        def do_POST(self):  # noqa: N802
            origin = self.headers.get("Origin")
            if not self._host_ok() or (origin and origin.split("//", 1)[-1] not in allowed) or \
                    not self.headers.get("Content-Type", "").startswith("application/json"):
                return self._send(403, {"error": "forbidden"})
            try:
                n = min(int(self.headers.get("Content-Length") or 0), 100_000)
                req = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(req, dict):
                    raise ValueError("not an object")
            except ValueError:
                return self._send(400, {"error": "bad request"})
            u = urlparse(self.path)
            try:
                if u.path == "/api/plan":
                    return self._send(200, make_plan(req))
                if u.path == "/api/build":
                    job = jobs.start(req)
                    return self._send(200, {"job": job["id"]})
                if u.path == "/api/drive":
                    name = str(req.get("name", ""))
                    if not NAME_RE.match(name):
                        return self._send(400, {"error": "bad name"})
                    return self._send(200, jobs.drive(name))
                return self._send(404, {"error": "not found"})
            except mm.MapMakerError as err:
                return self._err(err, 409 if "already" in str(err) else 400)

    return Handler


def serve(port=8777, open_browser=True):
    """Serve the page on 127.0.0.1 (the first free port from `port`) until Ctrl+C."""
    jobs = Jobs()
    httpd = None
    for p in range(port, port + 20):
        if p == 8000:
            continue
        try:
            httpd = ThreadingHTTPServer(("127.0.0.1", p), make_handler(jobs, p))
            port = p
            break
        except OSError:
            continue
    if httpd is None:
        mm.say(f"No free port between {port} and {port + 19}; close the other map maker first.")
        return 1
    url = f"http://127.0.0.1:{port}/"
    mm.say(f"The map maker is open at {url}\n(only on this computer). Keep this window open while you use it; "
           "press Ctrl+C here to stop.")
    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        mm.say("\nStopped.")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    serve()
