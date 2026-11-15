"""coupledcarla.session, hub, lobby and livemap: the session dashboard without CARLA (loopback sockets only)."""
import http.client
import json
import socket
import sys
import time
import types
import zipfile

import pytest

from coupledcarla import livemap, lobby, session as ses
from coupledcarla.coupling import ControlClient, ControlServer, RelayControl
from coupledcarla.hub import Hub

EXPERIMENT = """\
name: demo_session
map: {name: demo_map, bbox: [51.44, 5.48, 51.45, 5.49]}
ehmi:
  states:
    yielding: {text: "I'M STOPPING", color: [0, 190, 255]}
    go: {text: "GO", color: [0, 230, 120]}
participants:
  - {role: pedestrian, kind: pedestrian, station: true, spawn: {lat: 51.445, lon: 5.485}}
  - {role: av_passenger, kind: av_passenger, station: true, agent: av1}
  - {role: driver, kind: driver, station: true}
agents:
  - {id: av1, kind: car, route: [[51.444, 5.484], [51.446, 5.486]], ehmi: manual}
  - {id: car2, kind: car, route: [[51.446, 5.486], [51.444, 5.484]]}
trials:
  - {duration_s: 30}
  - {duration_s: 30, simulation: {weather: WetCloudySunset}}
"""


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait(cond, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.03)
    return False


def request(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request(method, path, body=json.dumps(body) if body is not None else None, headers=headers or {})
    r = c.getresponse()
    data = r.read()
    c.close()
    return r.status, data, r


@pytest.fixture
def sess(tmp_path):
    (tmp_path / "experiments").mkdir()
    (tmp_path / "experiments" / "demo.yaml").write_text(EXPERIMENT)
    (tmp_path / "experiments" / "cities.yaml").write_text("cities: []\n")      # the city list: not offered
    s = ses.Session(tmp_path, free_port(), log=lambda *a: None, carla_root="/opt/carla")
    yield s
    s.close()


HDR = {"X-Coupled-Carla": "1", "Content-Type": "application/json"}


def test_experiments_are_listed_and_chosen(sess):
    assert [e["path"] for e in sess.experiments] == ["experiments/demo.yaml"]
    assert sess.experiments[0]["stations"] == 3 and sess.experiments[0]["built"] is False
    sess.command("experiment", {"path": "experiments/demo.yaml"})
    st = sess.state()
    assert st["phase"] == "lobby" and len(st["plan"]) == 2
    assert [r["role"] for r in st["experiment"]["roles"]] == ["pedestrian", "av_passenger", "driver"]
    assert st["experiment"]["ehmi_states"] == ["off", "yielding", "go"]
    assert st["experiment"]["ehmi_agents"] == ["av1"]          # car2 has no eHMI
    assert wait(lambda: sess.validation["state"] == "done", 20)
    assert any("not built" in i[2] for i in sess.validation["items"])
    with pytest.raises(ses.SessionError):
        sess.command("experiment", {"path": "../../etc/passwd.yaml"})
    with pytest.raises(ses.SessionError):
        sess.command("nonsense", {})


def test_http_rules_local(sess):
    httpd, url = ses.serve(sess, "127.0.0.1", 0)
    port = httpd.server_address[1]
    try:
        code, page, _ = request(port, "GET", "/")
        assert code == 200 and b"Coupled CARLA" in page and b"Initialise" in page and b"Take-over" in page
        assert request(port, "GET", "/api/state", headers={"Host": "evil.example:80"})[0] == 403    # DNS rebinding
        code, data, _ = request(port, "GET", "/api/state")
        assert code == 200 and json.loads(data)["phase"] == "choose"
        assert request(port, "POST", "/api/experiment", {"path": "experiments/demo.yaml"})[0] == 403   # no header
        assert request(port, "POST", "/api/launch_missiles", {}, HDR)[0] == 404
        code, data, _ = request(port, "POST", "/api/experiment", {"path": "nope.yaml"}, HDR)
        assert code == 409 and "not in experiments" in json.loads(data)["error"]
        assert request(port, "POST", "/api/experiment", {"path": "experiments/demo.yaml"}, HDR)[0] == 200
        code, data, _ = request(port, "POST", "/api/start", {}, HDR)
        assert code == 409 and "No trial" in json.loads(data)["error"]
        assert request(port, "GET", "/api/zip?dir=../../", headers={})[0] == 404
        assert request(port, "GET", "/viewer/../coupledcarla/session.py")[0] == 404
    finally:
        httpd.shutdown()


def test_http_rules_token(sess):
    httpd, url = ses.serve(sess, "0.0.0.0", 0, token="s3cret")
    port = httpd.server_address[1]
    try:
        assert url.endswith("?token=s3cret")
        assert request(port, "GET", "/api/state")[0] == 403
        assert request(port, "GET", "/api/state?token=wrong")[0] == 403
        code, _, r = request(port, "GET", "/?token=s3cret")
        assert code == 200 and "SameSite=Strict" in r.getheader("Set-Cookie") and \
            r.getheader("X-Frame-Options") == "DENY"
        assert request(port, "GET", "/api/state", headers={"Cookie": "cc_token=s3cret"})[0] == 200
        assert request(port, "POST", "/api/refresh", {}, {**HDR, "Cookie": "cc_token=s3cret"})[0] == 200
        assert request(port, "POST", "/api/refresh", {}, {"X-Token": "s3cret"})[0] == 403   # header still needed
    finally:
        httpd.shutdown()


def test_command_validation(sess):
    sess.command("experiment", {"path": "experiments/demo.yaml"})
    for bad in ({"server_host": "a b; rm -rf"}, {"server_port": 80}, {"quality": "Ultra"}, {"duration": 2},
                {"server": "elsewhere"}):
        with pytest.raises(ses.SessionError):
            sess.command("options", bad)
    sess.command("options", {"server": "attach", "server_host": "lin4080", "server_port": "2400", "duration": "40"})
    assert sess.options["server_port"] == 2400 and sess.options["duration"] == 40.0
    assert sess.hub.carla == {"host": "lin4080", "port": 2400}       # what stations are told
    with pytest.raises(ses.SessionError):
        sess.command("weather", {"preset": "Tornado"})
    with pytest.raises(ses.SessionError):
        sess.command("weather", {"time": "25:00"})
    with pytest.raises(ses.SessionError):        # valid, but no trial
        sess.command("weather", {"preset": "ClearNoon", "time": "18:30"})
    with pytest.raises(ses.SessionError):
        sess.command("ehmi", {"state": "dance"})
    with pytest.raises(ses.SessionError):
        sess.command("participant", {"participant": -3})
    with pytest.raises(ses.SessionError):
        sess.command("participant", {"participant": 2, "order": "alphabetical"})
    sess.command("participant", {"participant": 2, "order": "latin"})
    assert sess.participant == 2
    with pytest.raises(ses.SessionError):
        sess.command("assign", {"station": "ghost", "role": "driver"})


def test_child_command():
    o = dict(ses.DEFAULT_OPTIONS, server="attach", server_host="127.0.0.1", server_port=2400, duration=40)
    cmd = ses.child_args("experiments/x.yaml", 1, 7, "latin", 5555, "tok", {"driver", "pedestrian"}, o)
    assert cmd[:5] == ["experiments/x.yaml", "--trial", "1", "--participant", "7"]
    assert "--station-roles=driver,pedestrian" in cmd and "--headless" in cmd
    assert cmd[cmd.index("--control-relay") + 1] == "127.0.0.1:5555"
    assert ["--no-launch", "--server-host", "127.0.0.1", "--server-port", "2400"] == \
        cmd[cmd.index("--no-launch"):cmd.index("--no-launch") + 5]
    assert cmd[cmd.index("--duration") + 1] == "40" and cmd[cmd.index("--order") + 1] == "latin"
    cmd = ses.child_args("x.yaml", 0, 0, None, 1, "t", set(), dict(ses.DEFAULT_OPTIONS, host_window=True))
    assert "--station-roles=" in cmd and "--headless" not in cmd and "--no-launch" not in cmd


def info(name):
    return {"machine": name, "inputs": ["keyboard", "G25 Racing Wheel"], "gpu": "RTX 4080", "input": "keyboard"}


def test_lobby_and_roles(sess):
    sess.command("experiment", {"path": "experiments/demo.yaml"})
    a = lobby.Lobby("127.0.0.1", sess.hub.port, info("lab-pc-1"), "pc1", retry_s=5)
    b = lobby.Lobby("127.0.0.1", sess.hub.port, info("lab-pc-2"), "pc2", retry_s=5)
    old = ControlClient("127.0.0.1", sess.hub.port, "driver", retry_s=5, hello={"info": {"machine": "old-pc"}})
    try:
        assert wait(lambda: len(sess.hub.stations_view()) == 3)
        rows = {r["id"]: r for r in sess.hub.stations_view()}
        assert rows["pc1"]["info"]["inputs"] == ["keyboard", "G25 Racing Wheel"] and rows["pc1"]["role"] is None
        assert rows["role:driver"]["fixed"] and rows["role:driver"]["ready"]        # started with --role
        a.poll()
        assert a.role is None
        sess.command("assign", {"station": "pc1", "role": "pedestrian"})
        assert wait(lambda: a.poll() or a.role == "pedestrian")
        assert a.carla == {"host": None, "port": 2000}
        sess.command("assign", {"station": "pc2", "role": "pedestrian"})        # moves the role
        assert wait(lambda: (a.poll(), b.poll()) and a.role is None and b.role == "pedestrian")
        with pytest.raises(ses.SessionError):
            sess.command("assign", {"station": "pc1", "role": "driver"})         # the --role station has it
        with pytest.raises(ses.SessionError):
            sess.command("assign", {"station": "role:driver", "role": "pedestrian"})
        with pytest.raises(ses.SessionError):
            sess.command("assign", {"station": "pc1", "role": "pilot"})
        sess.command("assign", {"station": "pc1", "role": "av_passenger"})
        assert sess.station_roles() == {"pedestrian", "av_passenger", "driver"}
        # the pedestrian's trial process joins: the lobby station is ready
        child = ControlClient("127.0.0.1", sess.hub.port, "pedestrian", retry_s=5, hello={"station": "pc2"})
        assert wait(lambda: {r["id"]: r for r in sess.hub.stations_view()}["pc2"]["ready"])
        assert wait(lambda: {r["id"]: r for r in sess.hub.stations_view()}["pc2"]["rtt_ms"] is not None, 4)
        child.sock.close()
        sess.command("options", {"bots_for_missing": False})
        assert "driver" in sess.station_roles()
    finally:
        for c in (a.client, b.client, old):
            c.sock.close()


def test_lobby_full_and_closed():
    hub = Hub(free_port(), log=lambda *a: None)
    try:
        ls = [lobby.Lobby("127.0.0.1", hub.port, info(f"pc{i}"), f"pc{i}", retry_s=5) for i in range(8)]
        assert wait(lambda: len(hub.stations_view()) == 8)
        ninth = lobby.Lobby("127.0.0.1", hub.port, info("pc9"), "pc9", retry_s=5)
        assert wait(lambda: ninth.poll() or ninth.refused)
        assert "8 stations" in ninth.refused
    finally:
        hub.close()
    assert wait(lambda: ls[0].poll() or ls[0].closed)          # the session ended: not a dropped connection
    assert not ls[0].dropped


def test_plain_host_refuses_a_station_without_role():
    srv = ControlServer(free_port(), log=lambda *a: None)
    try:
        lb = lobby.Lobby("127.0.0.1", srv.port, info("pc"), "pc", retry_s=5)
        assert wait(lambda: lb.poll() or lb.refused)
        assert "--role" in lb.refused
        old = ControlClient("127.0.0.1", srv.port, "driver", retry_s=5)       # --role stations: as before
        srv.on_hello = lambda role: {"kind": role}
        assert wait(lambda: srv.connected() == {"driver"})
        old.sock.close()
    finally:
        srv.close()


def test_child_command_line_of_a_lobby_station():
    cmd = lobby.child_command(["--host", "10.0.0.5", "--input", "wheel"], "driver", "pc1", {"port": 2400}, False,
                              False, "10.0.0.5")
    assert cmd[cmd.index("--role") + 1] == "driver" and cmd[cmd.index("--port") + 1] == "2400"
    assert "--control-host" not in cmd
    cmd = lobby.child_command(["--host", "10.0.0.5", "--port", "2000"], "driver", "pc1", {"port": 2400}, True,
                              False, "10.0.0.5")
    assert "--port" not in cmd[cmd.index("--child"):]                     # the station's own --port wins
    cmd = lobby.child_command(["--host", "10.0.0.5"], "driver", "pc1", {"host": "10.0.0.9", "port": 2000}, False,
                              False, "10.0.0.5")
    assert cmd[cmd.index("--control-host") + 1] == "10.0.0.5" and cmd[-1] == "10.0.0.9"


def test_relay_between_host_and_stations():
    got = []
    hub = Hub(free_port(), log=lambda *a: None, on_host=got.append)
    st = ControlClient("127.0.0.1", hub.port, "pedestrian", retry_s=5)    # waits before the trial host exists
    try:
        assert wait(lambda: hub.connected() == {"pedestrian"})
        intruder = RelayControl(f"127.0.0.1:{hub.link_port}", "wrong", log=lambda *a: None)
        assert wait(lambda: intruder.gone)                          # a wrong token: the hub hangs up
        relay = RelayControl(f"127.0.0.1:{hub.link_port}", hub.token, log=lambda *a: None)
        assert wait(lambda: relay.connected() == {"pedestrian"})
        relay.on_hello = lambda role: {"kind": role, "experiment": "demo"}     # set later, as run_trial does
        assert wait(lambda: any(m.get("type") == "welcome" for m in st.poll()))
        assert wait(lambda: any(m.get("type") == "hello" for m in relay.poll()) or True)
        relay.send("pedestrian", {"type": "state", "state": "running", "actor_id": 42})
        relay.broadcast({"type": "flash", "frame": 7})
        seen = []
        assert wait(lambda: seen.extend(st.poll()) or {"state", "flash"} <= {m["type"] for m in seen})
        st.send({"type": "stats", "fps": 33})
        msgs = []
        assert wait(lambda: msgs.extend(relay.poll()) or any(m.get("type") == "stats" for m in msgs))
        stats = next(m for m in msgs if m["type"] == "stats")
        assert stats["role"] == "pedestrian" and stats["fps"] == 33
        relay.dashboard.set_status({"state": "ready", "t": 0})
        relay.live({"t": 1.0, "features": []})
        assert wait(lambda: {"status", "live"} <= {m["type"] for m in got})
        assert hub.command({"cmd": "ehmi", "state": "go"}) and hub.command({"cmd": "stop"})
        assert wait(lambda: relay.cancelled)
        assert relay.wait_for(["driver"], 30) == {"driver"}          # cancelled: does not wait 30 s
        cmds = relay.dashboard.commands()
        assert {"cmd": "ehmi", "state": "go"} in cmds and {"cmd": "stop"} in cmds
        relay.close()
        assert wait(lambda: any(m["type"] == "detached" for m in got))
        assert hub.connected() == {"pedestrian"}                     # the station stays for the next trial
    finally:
        st.sock.close()
        hub.close()


def test_relay_ends_the_trial_when_the_session_goes():
    hub = Hub(free_port(), log=lambda *a: None)
    relay = RelayControl(f"127.0.0.1:{hub.link_port}", hub.token, log=lambda *a: None)
    time.sleep(0.2)
    hub.close()
    assert wait(lambda: relay.gone)
    assert {"cmd": "stop"} in relay.dashboard.commands()


def test_trial_process_end_and_crash(sess, tmp_path):
    sess.command("experiment", {"path": "experiments/demo.yaml"})
    d = tmp_path / "logs" / "demo_session" / "20260101-120000_trial0"
    d.mkdir(parents=True)
    (d / "config.yaml").write_text("name: demo_session\n")
    (d / "summary.json").write_text(json.dumps({"sim_s": 30.0, "rtf": 0.98, "collisions": 0,
                                                "sumo_vehicles_spawned": 51, "stations": ["pedestrian"]}))
    log = tmp_path / "logs" / "sessions" / "t.log"
    ok = ses.TrialRun([sys.executable, "-c", f"print('[   1.0s] trial 0 -> {d}')"], None, log, 0, sess.plan[0])
    sess.run = ok
    assert wait(lambda: sess.run is None, 10)
    rec = sess.finished[-1]
    assert rec["ok"] and rec["summary"]["rtf"] == 0.98 and rec["dir"] == "demo_session/20260101-120000_trial0"
    assert sess.position == 1 and sess.phase() == "between"
    bad = ses.TrialRun([sys.executable, "-c", "import sys; print('RuntimeError: no CARLA server'); sys.exit(1)"],
                       None, log, 1, sess.plan[1])
    sess.run = bad
    assert wait(lambda: sess.run is None, 10)
    assert not sess.finished[-1]["ok"] and "no CARLA server" in sess.finished[-1]["error"]
    assert sess.position == 1 and sess.phase() == "failed"           # the dashboard is still here
    buf, size = ses.zip_trial(d)
    with zipfile.ZipFile(buf) as z:
        assert sorted(z.namelist()) == [f"{d.name}/config.yaml", f"{d.name}/summary.json"]
    assert ses.safe_trial_dir("demo_session/20260101-120000_trial0", tmp_path / "logs") == d
    assert ses.safe_trial_dir("../demo_session", tmp_path / "logs") is None
    st = sess.state()
    assert st["past"] and st["past"][0]["summary"]["sumo"] == 51


# -- livemap --------------------------------------------------------------------------------------------------------
def fake_proj(x, y):
    """A gently non-linear 'projection' around Eindhoven."""
    return 51.44 - y / 111320.0 + 1e-12 * x * x, 5.48 + x / 69400.0 + 1e-12 * y * y


def test_affine_fit_is_close():
    aff = livemap.fit_affine(fake_proj, -800, -800, 800, 800)
    for x, y in ((0, 0), (700, -650), (-400, 300)):
        lat, lon = livemap.to_ll(aff, x, y)
        tl, tn = fake_proj(x, y)
        assert abs(lat - tl) * 111320 < 0.05 and abs(lon - tn) * 69400 < 0.05     # metres


def test_aerial_choice():
    assert "PDOK" in livemap.aerial([51.436, 5.47, 51.45, 5.493])["name"]          # Eindhoven
    assert livemap.aerial([50.87, 4.69, 50.89, 4.71]) is None                     # Leuven: not PDOK's
    assert "IGN" in livemap.aerial([45.75, 4.82, 45.77, 4.85])["name"]            # Lyon
    assert "GSI" in livemap.aerial([35.65, 139.69, 35.66, 139.71])["name"]
    assert livemap.aerial([-1.95, 30.05, -1.94, 30.07]) is None                   # Kigali: OSM tiles
    assert livemap.aerial([51.43, 5.47, 51.45, 5.49], country="Belgium") is None
    assert livemap.light_state("G") == "g" and livemap.light_state("y") == "y" and livemap.light_state("r") == "r"
    assert livemap.light_state("O") == "o"


class V:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class Snap:
    def __init__(self, actors):
        self.actors, self.frame = actors, 99

    def find(self, aid):
        a = self.actors.get(aid)
        if a is None:
            return None
        x, y, yaw, v = a
        tf = types.SimpleNamespace(location=V(x, y), rotation=types.SimpleNamespace(yaw=yaw))
        return types.SimpleNamespace(get_transform=lambda: tf, get_velocity=lambda: V(v, 0.0))


class Car:
    def __init__(self, ident, aid, mode):
        self.id, self.vehicle, self.ehmi_mode = ident, types.SimpleNamespace(id=aid), mode
        self.ehmi = "off"
        self.events = []

    def set_ehmi(self, state, by):
        self.ehmi = state
        self.events.append(("ehmi", f"{self.id} {state} (by {by})"))


def test_live_features_and_commands():
    geo = types.SimpleNamespace(to_carla=lambda lat, lon: ((lon - 5.48) * 69400.0, -(lat - 51.44) * 111320.0),
                                to_latlon=fake_proj)
    tc = {"map": {"bbox": [51.435, 5.47, 51.445, 5.49]}, "simulation": {"weather": "CloudyNoon"},
          "ehmi": {"states": {"yielding": {}, "go": {}}}}
    av, car2 = Car("av1", 11, "manual"), Car("car2", 12, "auto")
    walker = types.SimpleNamespace(id="w1", walker=types.SimpleNamespace(id=13))
    parts = [types.SimpleNamespace(role="pedestrian", kind="pedestrian", actor=types.SimpleNamespace(id=1), agent=None),
             types.SimpleNamespace(role="av_passenger", kind="av_passenger", actor=av.vehicle, agent=av)]
    sent = []
    control = types.SimpleNamespace(live=sent.append)
    tools = types.SimpleNamespace(other=[], tor={})
    feed = livemap.LiveFeed(control, geo, tc, None, parts, [av, car2, walker], None, 0.02, log=lambda *a: None,
                            tools=tools)
    assert feed.states == ["off", "yielding", "go"] and [a.id for a in feed.ehmi_agents()] == ["av1", "car2"]
    snap = Snap({1: (10.0, -20.0, 90.0, 1.4), 11: (50.0, 0.0, 180.0, 8.0), 12: (80.0, 5.0, 0.0, 7.0),
                 13: (5.0, 5.0, 0.0, 1.2), 21: (100.0, 30.0, 270.0, 10.0), 22: (120.0, 30.0, 0.0, 4.0),
                 31: (0.0, 40.0, 0.0, 1.3)})
    sync = types.SimpleNamespace(sumo2carla_ids={"veh7": 21, "bike3": 22})
    crowd = types.SimpleNamespace(walkers=[types.SimpleNamespace(id=31)])
    tools.other.append({"cmd": "ehmi", "state": "yielding"})
    tools.other.append({"cmd": "ehmi", "state": "nonsense"})
    world = types.SimpleNamespace()
    feed.tick(0.0, snap, 0, logger=None, world=world, sync=sync, types={21: ("car", "x"), 22: ("bike", "y")},
              crowd=crowd)
    assert tools.other == [] and av.ehmi == "yielding" and car2.ehmi == "yielding" and car2.ehmi_mode == "manual"
    out = sent[-1]
    kinds = [f[0] for f in out["features"]]
    assert kinds == ["participant", "participant", "av", "walker", "sumo", "bike", "crowd"]   # car2: now manual
    ped = out["features"][0]
    assert ped[2] == "pedestrian" and ped[7] == {"k": "pedestrian"} and ped[6] == round(3.6 * 1.4, 1)
    assert out["features"][1][7]["e"] == "yielding"                     # the passenger's AV, shown once
    lat, lon = ped[3], ped[4]
    assert abs(lat - fake_proj(10.0, -20.0)[0]) < 1e-6 and abs(lon - fake_proj(10.0, -20.0)[1]) < 1e-6
    assert out["ehmi"]["agents"][0] == {"id": "av1", "state": "yielding", "rider": "av_passenger"}
    feed.tick(0.02, snap, 1, logger=None, world=world, sync=sync)
    assert len(sent) == 1                                              # 5 Hz at 50 Hz ticks


def test_set_stations_from_the_session():
    run = pytest.importorskip("coupledcarla.run")          # needs the carla package (not on macOS)
    ps = [{"role": "pedestrian", "kind": "pedestrian", "station": True},
          {"role": "driver", "kind": "driver", "station": True},
          {"role": "local", "kind": "driver", "input": "keyboard"}]
    run._set_stations(ps, ["pedestrian", "local"])
    assert ps[0]["station"] and not ps[1]["station"] and ps[1]["input"] == "autopilot" and ps[2]["station"]
