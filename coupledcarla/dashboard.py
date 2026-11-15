"""Experimenter dashboard (coupled-sim's ExperimentDashboard, #88): a local web page to watch and steer a running
experiment, served by the host with the standard library only.

    python -m coupledcarla.run experiments/eindhoven_crossing.yaml --participant 7 --dashboard
    # open http://127.0.0.1:8780/

It shows the experiment, the participant and their trial plan, the trial's state, time and real-time factor, each
participant (connected, speed, the station's round trip, frame rate and image lag, the responses, the automation
state and take-overs), collisions and the latest events. Buttons: Start trial (each trial waits for it once it
is loaded, unless dashboard.gate_trials is false), Pause / Resume (the world, SUMO and the stations stop ticking;
paused time does not count), End trial, Stop session (the current trial ends and no more start), Visual sync
(red bar on every station) and a take-over request per automated driver.

It listens on 127.0.0.1 unless --dashboard-host (or dashboard.host) says otherwise; then every request needs the
token printed at the start (the link carries it). Commands are POSTs with an X-Coupled-Carla header (a page on
another site cannot send that without the browser asking first, and this server never allows it), and a local
dashboard only answers requests addressed to localhost (no DNS rebinding).
"""
import json
import queue
import secrets
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

LOCAL = ("127.0.0.1", "localhost", "::1")
COMMANDS = ("start", "pause", "resume", "abort", "stop", "sync", "takeover")
COOKIE = "cc_token"


def request_allowed(handler, local, token, cookie=False):
    """The rule for every request to a dashboard (this one and the session dashboard, session.py): a local one
    answers only requests addressed to localhost (no DNS rebinding); any other needs the token, in the X-Token
    header, the ?token= query or (cookie=True) the SameSite=Strict cookie the page set."""
    if local:
        hostname = (handler.headers.get("Host") or "").rsplit(":", 1)[0].strip("[]")
        return hostname in LOCAL
    q = parse_qs(urlparse(handler.path).query)
    given = handler.headers.get("X-Token") or (q.get("token") or [""])[0]
    if not given and cookie:
        for part in (handler.headers.get("Cookie") or "").split(";"):
            k, _, v = part.strip().partition("=")
            if k == COOKIE:
                given = v
    return bool(token) and secrets.compare_digest(given.encode("utf-8", "replace"), token.encode())


class Dashboard:
    def __init__(self, host="127.0.0.1", port=8780, log=print, token=None):
        self.local = host in LOCAL
        self.token = None if self.local else (token or secrets.token_urlsafe(12))
        self._status = {"state": "starting"}
        self._lock = threading.Lock()
        self._cmds = queue.Queue()
        self.log = log
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _allowed(self):
                return request_allowed(self, dash.local, dash.token)

            def _send(self, code, body, ctype="application/json"):
                data = body if isinstance(body, bytes) else body.encode()
                self.send_response(code)
                self.send_header("Content-Type", ctype + "; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if not self._allowed():
                    return self._send(403, '{"error": "forbidden"}')
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    return self._send(200, PAGE, "text/html")
                if path == "/status.json":
                    return self._send(200, json.dumps(dash.status()))
                return self._send(404, '{"error": "not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                if not self._allowed() or self.headers.get("X-Coupled-Carla") != "1":
                    return self._send(403, '{"error": "forbidden"}')
                cmd = path.rsplit("/", 1)[-1]
                if not path.startswith("/api/") or cmd not in COMMANDS:
                    return self._send(404, '{"error": "unknown command"}')
                try:
                    n = min(int(self.headers.get("Content-Length") or 0), 4096)
                    body = json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    body = {}
                item = {"cmd": cmd}
                if isinstance(body, dict) and isinstance(body.get("role"), str):
                    item["role"] = body["role"][:64]
                dash._cmds.put(item)
                dash.log(f"dashboard: {cmd}" + (f" {item['role']}" if "role" in item else ""))
                return self._send(200, '{"ok": true}')

        server = ThreadingHTTPServer
        if ":" in host:                      # an IPv6 address (::1, ::)
            server = type("ThreadingHTTPServer6", (ThreadingHTTPServer,), {"address_family": socket.AF_INET6})
        self.httpd = server((host, port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        shown = "127.0.0.1" if self.local and host != "::1" else (
            host if host not in ("0.0.0.0", "::") else _lan_ip())
        shown = f"[{shown}]" if ":" in shown else shown
        self.url = f"http://{shown}:{self.port}/" + (f"?token={self.token}" if self.token else "")
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        log(f"experimenter dashboard: {self.url}")

    def set_status(self, status):
        with self._lock:
            self._status = dict(status)

    def update(self, **fields):
        with self._lock:
            self._status = {**self._status, **fields}

    def status(self):
        with self._lock:
            return dict(self._status)

    def commands(self):
        out = []
        while True:
            try:
                out.append(self._cmds.get_nowait())
            except queue.Empty:
                return out

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def _lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))      # no packet is sent: picks the interface of the default route
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Experiment dashboard</title>
<style>
:root { --bg:#f3f5f8; --panel:#fff; --ink:#1d2733; --muted:#5b6b7c; --line:#d8dee6; --accent:#c8102e; --ok:#1f8f4e;
  --warn:#c77700; }
@media (prefers-color-scheme: dark) { :root { --bg:#14181d; --panel:#1d232a; --ink:#e6ebf0; --muted:#93a1b0;
  --line:#2e3742; --accent:#ff5a6e; --ok:#4cc27e; --warn:#f0a43a; } }
* { box-sizing: border-box; }
body { margin:0; padding:16px; background:var(--bg); color:var(--ink);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }
h1 { font-size:18px; margin:0 0 2px; } .muted { color:var(--muted); }
.grid { display:grid; gap:12px; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); margin-top:12px; }
.panel { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px; overflow-x:auto; }
.panel h2 { font-size:13px; text-transform:uppercase; letter-spacing:.04em; color:var(--muted); margin:0 0 8px; }
.big { font-size:28px; font-weight:600; font-variant-numeric:tabular-nums; }
.bar { height:8px; background:var(--line); border-radius:4px; overflow:hidden; margin:6px 0; }
.bar > div { height:100%; background:var(--accent); width:0; }
button { font:inherit; padding:7px 12px; border-radius:6px; border:1px solid var(--line); background:var(--panel);
  color:var(--ink); cursor:pointer; margin:0 6px 6px 0; } button:hover { border-color:var(--muted); }
button.primary { background:var(--ok); color:#fff; border-color:var(--ok); }
button.danger { color:var(--accent); }
table { border-collapse:collapse; width:100%; font-variant-numeric:tabular-nums; }
td, th { text-align:left; padding:4px 6px; border-bottom:1px solid var(--line); white-space:nowrap; }
th { color:var(--muted); font-weight:500; }
.state { display:inline-block; padding:1px 8px; border-radius:10px; background:var(--line); }
.state.running { background:var(--ok); color:#fff; } .state.paused, .state.ready { background:var(--warn); color:#fff; }
.cur { font-weight:600; color:var(--accent); }
.lamp { display:inline-block; width:10px; height:10px; border-radius:50%; background:var(--line); vertical-align:-1px; }
.lamp.on { background:var(--ok); }
#events td:first-child { color:var(--muted); }
#err { color:var(--accent); }
</style></head><body>
<h1 id="title">Experiment dashboard</h1>
<div class="muted" id="sub">connecting...</div> <div id="err"></div>
<div class="grid">
  <div class="panel"><h2>Trial</h2>
    <div><span class="state" id="state">-</span> <span id="cond" class="muted"></span></div>
    <div class="big" id="clock">-</div><div class="bar"><div id="prog"></div></div>
    <div class="muted" id="rtf"></div>
    <div style="margin-top:10px">
      <button class="primary" data-cmd="start">Start trial</button>
      <button data-cmd="pause" id="pause">Pause</button>
      <button data-cmd="sync">Visual sync</button>
      <button class="danger" data-cmd="abort" data-confirm="End this trial now?">End trial</button>
      <button class="danger" data-cmd="stop" data-confirm="End this trial and stop the session?">Stop session</button>
    </div><div id="tor"></div>
  </div>
  <div class="panel"><h2>Plan</h2><table id="plan"></table></div>
</div>
<div class="grid">
  <div class="panel" style="grid-column:1/-1"><h2>Participants</h2><table id="parts"></table></div>
  <div class="panel" style="grid-column:1/-1"><h2>Events <span class="muted" id="cols"></span></h2><table id="events"></table></div>
</div>
<script>
const token = new URLSearchParams(location.search).get("token") || "";
const H = {"X-Coupled-Carla": "1", "Content-Type": "application/json"}; if (token) H["X-Token"] = token;
const q = token ? "?token=" + encodeURIComponent(token) : "";
const $ = id => document.getElementById(id);
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
let last = {};
async function send(cmd, role) {
  try { await fetch("/api/" + cmd + q, {method: "POST", headers: H, body: JSON.stringify(role ? {role} : {})}); }
  catch (e) { $("err").textContent = "command failed: " + e; }
}
document.querySelectorAll("button[data-cmd]").forEach(b => b.onclick = () => {
  let cmd = b.dataset.cmd;
  if (cmd === "pause" && last.state === "paused") cmd = "resume";
  if (b.dataset.confirm && !confirm(b.dataset.confirm)) return;
  send(cmd);
});
function fmt(v, d=1) { return v === null || v === undefined ? "–" : Number(v).toFixed(d); }
function render(s) {
  last = s;
  $("title").textContent = s.experiment || "Experiment dashboard";
  $("sub").textContent = (s.participant ? "participant " + s.participant + " · " : "") +
    (s.position !== undefined && s.position !== null ? "trial " + (s.position + 1) + " of " + (s.plan || []).length : "");
  $("state").textContent = s.state; $("state").className = "state " + s.state;
  $("cond").textContent = [s.condition, s.block ? "block " + s.block : ""].filter(x => x).join(" · ");
  $("clock").textContent = fmt(s.t) + " / " + fmt(s.duration_s, 0) + " s";
  $("prog").style.width = Math.min(100, 100 * (s.t || 0) / (s.duration_s || 1)) + "%";
  $("rtf").textContent = (s.rtf ? "real-time factor " + s.rtf + " · " : "") + "frame " + (s.frame ?? "–") +
    " · collisions " + (s.collisions ?? 0);
  $("pause").textContent = s.state === "paused" ? "Resume" : "Pause";
  $("tor").innerHTML = (s.automated || []).map(r =>
    `<button data-role="${esc(r)}">Take-over request: ${esc(r)}</button>`).join("");
  $("tor").querySelectorAll("button").forEach(b => b.onclick = () => send("takeover", b.dataset.role));
  $("plan").innerHTML = "<tr><th>#</th><th>trial</th><th>condition</th><th>s</th></tr>" + (s.plan || []).map(p =>
    `<tr class="${p[0] === s.position ? "cur" : ""}"><td>${p[0] + 1}</td><td>${p[1]}</td><td>${esc(p[2])}</td><td>${p[3]}</td></tr>`).join("");
  $("parts").innerHTML = "<tr><th>role</th><th>kind</th><th>station</th><th>km/h</th><th>rtt ms</th><th>fps</th>" +
    "<th>image lag ms</th><th>button</th><th>slider</th><th>automation</th><th>take-overs</th></tr>" +
    (s.participants || []).map(p => {
      const r = p.response || {};
      const tk = (p.takeovers || []).map(x => x.how ? `${esc(x.how)} ${fmt(x.reaction_s, 2)} s` :
        (x.no_reaction ? "no reaction" : "waiting")).join(", ");
      return `<tr><td>${esc(p.role)}</td><td>${esc(p.kind)}</td>` +
        `<td>${p.station ? (p.connected ? "connected" : "<b>missing</b>") : "host"}</td>` +
        `<td>${fmt(p.speed_kmh)}</td><td>${fmt(p.rtt_ms)}</td><td>${fmt(p.fps, 0)}</td><td>${fmt(p.image_lag_ms)}</td>` +
        `<td>${p.response ? `<span class="lamp ${r.button ? "on" : ""}"></span> ${esc(r.presses)}` : ""}</td>` +
        `<td>${esc(r.slider)}</td><td>${esc(p.automation || "")}</td><td>${tk}</td></tr>`;
    }).join("");
  $("events").innerHTML = (s.events || []).slice().reverse().map(e =>
    `<tr><td>${fmt(e[0], 2)}</td><td>${esc(e[1])}</td><td>${esc(e[2])}</td></tr>`).join("");
}
async function poll() {
  try { const r = await fetch("/status.json" + q, {cache: "no-store"});
        if (r.ok) { render(await r.json()); $("err").textContent = ""; }
        else $("err").textContent = "status: HTTP " + r.status; }
  catch (e) { $("err").textContent = "host not reachable (experiment over?)"; }
  setTimeout(poll, 500);
}
poll();
</script></body></html>
"""
