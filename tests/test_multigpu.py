"""coupledcarla.multigpu: configuration, checks, command lines, the relay, camera placement and the camera turn of
the control channel (no CARLA, no GPU)."""
import socket
import threading
import time

import pytest

from coupledcarla import config, multigpu
from coupledcarla.coupling import ControlClient, ControlServer


def cfg_with(secondaries, **mg):
    cfg = config._merge(config.DEFAULTS, {
        "name": "mg", "participants": [{"role": "pedestrian", "kind": "pedestrian", "station": True},
                                       {"role": "driver", "kind": "driver", "station": True}],
        "server": {"multigpu": {"secondaries": secondaries, **mg}}})
    return cfg


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_defaults_and_settings():
    assert not multigpu.enabled(config.DEFAULTS["server"])
    assert multigpu.primary_args(config.DEFAULTS["server"]) == []
    assert multigpu.problems(config.DEFAULTS) == []
    sc = cfg_with([{}, {"name": "ped", "host": "10.0.0.9", "roles": ["pedestrian"]}])["server"]
    s = multigpu.settings(sc)
    a, b = s["secondaries"]
    assert (a.name, a.rpc_port, a.streaming_port, a.local, a.launch) == ("s0", 3000, 3001, True, True)
    assert (b.name, b.rpc_port, b.local, b.launch, b.roles) == ("ped", 4000, False, False, ["pedestrian"])
    assert s["secondary_port"] == 2002
    assert multigpu.primary_args(sc) == ["-nullrhi"]
    sc["multigpu"]["primary_renders"] = True
    sc["multigpu"]["secondary_port"] = 2502
    assert multigpu.primary_args(sc) == ["-carla-secondary-port=2502"]


def test_secondary_command_lines():
    sc = cfg_with([{"name": "gpu1", "gpu": 1, "packs": "C:\\cc58\\carla_packs", "os": "windows"},
                   {"name": "ped", "host": "10.0.0.9", "rpc_port": 4000, "os": "linux", "extra_args": ["-x"]}],
                  primary_host="10.0.0.1")["server"]
    gpu1, ped = multigpu.settings(sc)["secondaries"]
    argv = multigpu.secondary_command(sc, gpu1, root="C:\\carla")
    assert argv[0] == "C:\\carla\\CarlaUnreal.exe"
    assert "-carla-rpc-port=3000" in argv and "-carla-primary-host=127.0.0.1" in argv
    assert "-carla-primary-port=2002" in argv and "-graphicsadapter=1" in argv and "-RenderOffScreen" in argv
    assert "-carla-packs=C:\\cc58\\carla_packs" in argv and not any(a == "-nullrhi" for a in argv)
    text = multigpu.command_text(argv, windows=True)
    assert text.startswith("& 'C:\\carla\\CarlaUnreal.exe' -carla-rpc-port=3000")
    argv = multigpu.secondary_command(sc, ped, root="/opt/carla")
    assert argv[0] == "/opt/carla/CarlaUnreal.sh" and "-carla-primary-host=10.0.0.1" in argv and argv[-1] == "-x"
    assert multigpu.command_text(argv, windows=False).startswith("/opt/carla/CarlaUnreal.sh -carla-rpc-port=4000")


def test_problems():
    msgs = lambda c: [m for _l, m in multigpu.problems(c)]      # noqa: E731
    errs = lambda c: [m for lv, m in multigpu.problems(c) if lv == "error"]      # noqa: E731
    assert errs(cfg_with([{}, {"name": "b", "host": "10.0.0.9"}], primary_host="10.0.0.1")) == []
    assert any("used twice" in m for m in errs(cfg_with([{"name": "a"}, {"name": "a", "rpc_port": 5000}])))
    assert any("not a participant" in m for m in errs(cfg_with([{"roles": ["cyclist"]}])))
    assert any("listed by" in m for m in errs(cfg_with([{"roles": ["driver"]}, {"roles": ["driver"]}])))
    assert any("launch is only possible" in m
               for m in errs(cfg_with([{"host": "10.0.0.9", "launch": True}], primary_host="10.0.0.1")))
    # a secondary on the primary's ports, two on one machine sharing ports, and the control channel's port
    assert any("both use port 2001" in m for m in errs(cfg_with([{"rpc_port": 2000}])))
    assert any("both use port" in m for m in errs(cfg_with([{"rpc_port": 3000}, {"rpc_port": 3001}])))
    assert any("streaming port 3001" in m
               for m in errs(cfg_with([{"host": "10.0.0.8"}, {"host": "10.0.0.9", "rpc_port": 3000}],
                                      primary_host="10.0.0.1")))
    assert any("control channel" in m for m in errs(cfg_with([{"rpc_port": 2099}])))
    assert any("primary_host" in m for m in msgs(cfg_with([{"host": "10.0.0.9"}])))
    c = cfg_with([{}])
    c["agents"] = [{"id": "av1", "kind": "car", "ehmi": "manual"}]
    assert any("not mirrored" in m for m in msgs(c))
    c["ehmi"] = {"render": "station"}
    assert not any("not mirrored" in m for m in msgs(c))


def test_validate_reports_multigpu(tmp_path):
    from coupledcarla import validate
    raw = {"name": "mg", "map": {"name": "mg", "bbox": [51.43, 5.47, 51.45, 5.49]},
           "server": {"multigpu": {"secondaries": [{"name": "a", "rpc_port": 2000}], "colour": 1}}}
    cfg = config._merge(config.DEFAULTS, raw)
    cfg["_root"] = str(tmp_path)
    rep, _ = validate.check(raw, cfg, roads=False, participants=[])
    texts = [f"{w}: {m}" for _lv, w, m in rep.items]
    assert any("server.multigpu" in t and "port 2001" in t for t in texts)
    assert any("colour" in t for t in texts)          # unknown key in server.multigpu


def test_welcome():
    sc = cfg_with([{"name": "here"}, {"name": "ped", "host": "10.0.0.9", "roles": ["pedestrian"]}])["server"]
    w = multigpu.welcome(sc, "pedestrian")["multigpu"]
    assert w["own"] == "ped" and w["rpc_port"] == 2000 and w["streaming_port"] == 2001
    assert w["secondaries"] == [{"name": "here", "host": "server", "streaming_port": 3001},
                                {"name": "ped", "host": "10.0.0.9", "streaming_port": 4001}]
    assert multigpu.welcome(sc, "driver")["multigpu"]["own"] is None
    assert multigpu.welcome(config.DEFAULTS["server"], "driver") == {}


class Echo:
    """A TCP server that answers every chunk with its port number and the chunk."""

    def __init__(self):
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.sock.accept()
            except OSError:
                return
            threading.Thread(target=self._echo, args=(c,), daemon=True).start()

    def _echo(self, c):
        while True:
            d = c.recv(65536)
            if not d:
                c.close()
                return
            c.sendall(f"{self.port}:".encode() + d)


def test_relay_forwards_and_counts():
    up = Echo()
    local = free_port()
    relay = multigpu.Relay({local: ("127.0.0.1", up.port)})
    try:
        for _ in range(2):
            c = socket.create_connection(("127.0.0.1", local), timeout=5)
            c.sendall(b"x" * 100000)
            got = b""
            while len(got) < 100000 + len(f"{up.port}:"):
                got += c.recv(1 << 20)
            assert got.startswith(f"{up.port}:".encode())
            c.close()
        time.sleep(0.1)
        assert relay.counts()[local] == 2
    finally:
        relay.close()


def test_relay_process_forwards_while_the_gil_is_held():
    """The relay runs in a child process: it forwards while this process holds the GIL (as CARLA 1.0's client does
    while it waits for get_world's answer), counts the connections and ends with close()."""
    up = Echo()
    local = free_port()
    relay = multigpu.Relay({local: ("127.0.0.1", up.port)})
    try:
        assert relay.proc is not None and relay.proc.is_alive()
        c = socket.create_connection(("127.0.0.1", local), timeout=5)
        c.sendall(b"ping")
        assert c.recv(100) == f"{up.port}:ping".encode()
        c.close()
        deadline = time.time() + 5
        while relay.counts()[local] < 1 and time.time() < deadline:
            time.sleep(0.05)
        assert relay.counts()[local] == 1
    finally:
        relay.close()
    assert relay.proc is None
    with pytest.raises(OSError):        # the port is taken: the child says so and the parent raises
        busy = socket.socket()
        busy.bind(("127.0.0.1", 0))
        busy.listen(1)
        try:
            multigpu.Relay({busy.getsockname()[1]: ("127.0.0.1", up.port)})
        finally:
            busy.close()


def test_station_routing_direct_without_remote_secondaries():
    sc = cfg_with([{"name": "here"}])["server"]
    r = multigpu.StationRouting(multigpu.welcome(sc, "driver")["multigpu"], "10.9.9.9", 2000, log=lambda m: None)
    assert r.relay is None and r.address() == ("10.9.9.9", 2000) and not r.can_place()
    assert multigpu.StationRouting({}, "10.9.9.9", 2000).address() == ("10.9.9.9", 2000)


def test_station_routing_relays_and_places(monkeypatch):
    """A station on its own PC, with its own secondary there: the client goes through 127.0.0.1, the primary's
    ports and the other secondaries are relayed, and a camera that lands elsewhere is spawned again."""
    p_rpc, p_stream, s_other, s_own = free_port(), free_port(), free_port(), free_port()
    own_server = socket.socket()
    own_server.bind(("127.0.0.1", s_own))      # "the own secondary's streaming port" on this machine
    own_server.listen(4)
    plan = {"secondaries": [{"name": "srv", "host": "server", "streaming_port": s_other},
                            {"name": "mine", "host": "10.0.0.50", "streaming_port": s_own}],
            "own": "mine", "rpc_port": p_rpc, "streaming_port": p_stream, "relay": "auto", "placement": True}
    monkeypatch.setattr(multigpu, "_is_local", lambda h: h in multigpu.LOCAL_HOSTS)
    monkeypatch.setattr(multigpu, "RELAY_PROCESS", False)       # its counts are bumped by hand below
    r = multigpu.StationRouting(plan, "10.0.0.1", p_rpc, log=lambda m: None)
    try:
        assert r.relay is not None and r.own_local and r.can_place()
        assert set(r.relay.routes) == {p_rpc, p_stream, s_other}
        assert r.relay.routes[s_other] == ("10.0.0.1", s_other)
        assert r.address() == ("127.0.0.1", p_rpc)
        # first camera: its stream connects to the relayed secondary "srv"; the second gets images directly
        got = threading.Event()
        spawned, destroyed = [], []

        def spawn():
            spawned.append(len(spawned))
            return len(spawned)

        def listen(cam):
            if cam == 1:
                with r.relay.lock:
                    r.relay.connections[s_other] += 1
            else:
                got.set()
        cam = r.place_camera(spawn, listen, destroyed.append, got, timeout=1.0)
        assert cam == 2 and destroyed == [1] and r.placed == "mine"
    finally:
        r.close()
        own_server.close()


def test_place_camera_once_without_placement():
    r = multigpu.StationRouting({}, "127.0.0.1", 2000)
    calls = []
    cam = r.place_camera(lambda: "cam", calls.append, None, threading.Event())
    assert cam == "cam" and calls == ["cam"]


def test_camera_turns_one_at_a_time():
    server = ControlServer(free_port(), log=lambda *a: None)
    got = {}
    clients = {}
    try:
        for role in ("a", "b"):
            ev = threading.Event()
            got[role] = ev
            clients[role] = ControlClient(
                "127.0.0.1", server.port, role, retry_s=5,
                on_message=lambda m, ev=ev: ev.set() if m.get("type") == "camera_go" else None)
        t0 = time.time()
        while server.connected() != {"a", "b"} and time.time() - t0 < 5:
            time.sleep(0.05)
        clients["a"].send({"type": "camera_turn"})
        assert got["a"].wait(5)
        clients["b"].send({"type": "camera_turn"})
        assert not got["b"].wait(0.5)            # a holds the turn
        clients["a"].send({"type": "camera_done"})
        assert got["b"].wait(5)
    finally:
        server.close()


def test_camera_turn_released_when_the_holder_leaves():
    server = ControlServer(free_port(), log=lambda *a: None)
    go_b = threading.Event()
    try:
        a = ControlClient("127.0.0.1", server.port, "a", retry_s=5)
        b = ControlClient("127.0.0.1", server.port, "b", retry_s=5,
                          on_message=lambda m: go_b.set() if m.get("type") == "camera_go" else None)
        t0 = time.time()
        while server.connected() != {"a", "b"} and time.time() - t0 < 5:
            time.sleep(0.05)
        a.send({"type": "camera_turn"})
        time.sleep(0.3)
        b.send({"type": "camera_turn"})
        time.sleep(0.3)
        assert not go_b.is_set()
        a.sock.shutdown(socket.SHUT_RDWR)     # the holder's station dies
        a.sock.close()
        assert go_b.wait(5)
    finally:
        server.close()


def test_camera_turn_context_sends_turn_and_done():
    sent = []
    go = threading.Event()
    go.set()
    with multigpu.CameraTurn(sent.append, go, timeout=1) as turn:
        turn.result["secondary"] = "mine"
    assert sent == [{"type": "camera_turn"}, {"type": "camera_done", "secondary": "mine"}]


def test_supervise_restarts_with_the_primary(monkeypatch):
    """The secondary starts once the primary is up, stops when it goes away, starts again when it is back."""
    up = iter([False, True, True, False, True, True])
    monkeypatch.setattr(multigpu, "_port_open", lambda h, p, timeout=1.0: next(up))
    killed, started = [], []

    class P:
        def __init__(self, argv, **kw):
            self.pid, self.returncode = len(started) + 100, None
            started.append(argv)

        def poll(self):
            return self.returncode

    monkeypatch.setattr(multigpu, "_kill", lambda p: killed.append(p.pid) or setattr(p, "returncode", -9))
    ticks = [0]

    def stop():
        ticks[0] += 1
        return ticks[0] > 6
    multigpu.supervise(["carla"], "10.0.0.1", 2002, log=lambda m: None, poll_s=0, stop=stop, popen=P)
    assert len(started) == 2 and killed[0] == 100


def test_guard_settings():
    class S:
        synchronous_mode, fixed_delta_seconds = True, 0.05

    class W:
        def __init__(self):
            self.s, self.applied = S(), []

        def get_settings(self):
            return self.s

        def apply_settings(self, s):
            self.applied.append((s.synchronous_mode, s.fixed_delta_seconds))
    assert multigpu.guard_settings(config.DEFAULTS["server"], 0.02) is None
    g = multigpu.guard_settings(cfg_with([{}])["server"], 0.02, log=lambda m: None)
    w = W()
    assert g(w, force=True) is True and w.applied == [(True, 0.02)]
    assert g(w) is False                     # checked again only after SETTINGS_EVERY_S
    assert g(w, force=True) is False         # already right


def test_plan_text_and_cli(capsys, tmp_path):
    path = tmp_path / "mg.yaml"
    path.write_text("""name: mg
participants: [{role: pedestrian, kind: pedestrian, station: true}]
server:
  multigpu:
    primary_host: 10.0.0.1
    secondaries:
      - {name: gpu0, gpu: 0}
      - {name: ped, host: 10.0.0.9, rpc_port: 4000, roles: [pedestrian], os: windows}
""")
    assert multigpu.main([str(path), "--windows"]) == 0
    out = capsys.readouterr().out
    assert "-nullrhi" in out and "gpu0" in out and "ped" in out and "--secondary ped" in out
    assert multigpu.main([str(path), "--secondary", "ped", "--root", "C:\\carla", "--dry"]) == 0
    out = capsys.readouterr().out.strip()
    assert "-carla-primary-host=10.0.0.1" in out and "-carla-rpc-port=4000" in out
    assert multigpu.main([str(path), "--secondary", "nope"]) == 2


@pytest.mark.parametrize("host", ["127.0.0.1", "localhost", ""])
def test_local_hosts(host):
    assert multigpu._is_local(host)
