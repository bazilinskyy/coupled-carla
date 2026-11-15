"""coupledcarla.dashboard: status, commands, and who may send them."""
import http.client
import json
import threading
import types

from coupledcarla.dashboard import Dashboard
from coupledcarla.studytools import HostStudy
from test_responses import FakeLogger, part


def request(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request(method, path, body=json.dumps(body) if body is not None else None, headers=headers or {})
    r = c.getresponse()
    data = r.read()
    c.close()
    return r.status, data


def test_local_dashboard_serves_and_takes_commands():
    d = Dashboard("127.0.0.1", 0, log=lambda *a: None)
    try:
        assert d.token is None and d.url == f"http://127.0.0.1:{d.port}/"
        code, page = request(d.port, "GET", "/")
        assert code == 200 and b"Take-over request" in page and b"Stop session" in page
        d.set_status({"state": "running", "t": 3.5})
        code, data = request(d.port, "GET", "/status.json")
        assert code == 200 and json.loads(data) == {"state": "running", "t": 3.5}
        hdr = {"X-Coupled-Carla": "1", "Content-Type": "application/json"}
        assert request(d.port, "POST", "/api/pause", {})[0] == 403                    # no header: a form post
        assert request(d.port, "POST", "/api/pause", {}, {**hdr, "Host": "evil.example:80"})[0] == 403  # rebinding
        assert request(d.port, "POST", "/api/launch", {}, hdr)[0] == 404
        assert request(d.port, "POST", "/api/takeover", {"role": "driver"}, hdr)[0] == 200
        assert request(d.port, "POST", "/api/pause", None, hdr)[0] == 200
        assert d.commands() == [{"cmd": "takeover", "role": "driver"}, {"cmd": "pause"}]
        assert d.commands() == []
    finally:
        d.close()


def test_other_hosts_need_the_token():
    d = Dashboard("0.0.0.0", 0, log=lambda *a: None, token="s3cret")
    try:
        assert d.url.endswith("?token=s3cret")
        assert request(d.port, "GET", "/status.json")[0] == 403
        assert request(d.port, "GET", "/status.json?token=nope")[0] == 403
        assert request(d.port, "GET", "/status.json?token=s3cret")[0] == 200
        hdr = {"X-Coupled-Carla": "1"}
        assert request(d.port, "POST", "/api/stop", {}, hdr)[0] == 403
        assert request(d.port, "POST", "/api/stop", {}, {**hdr, "X-Token": "s3cret"})[0] == 200
        assert d.commands() == [{"cmd": "stop"}]
    finally:
        d.close()


def test_gate_pause_and_stop_through_the_host_hooks(tmp_path):
    d = Dashboard("127.0.0.1", 0, log=lambda *a: None)
    hdr = {"X-Coupled-Carla": "1"}
    try:
        plan = [{"_position": 0, "_index": 2, "_condition": "ehmi_on", "duration_s": 45},
                {"_position": 1, "_index": 0, "_condition": "ehmi_off", "duration_s": 45}]
        tools = HostStudy({"name": "demo", "dashboard": {"gate_trials": True}}, dict(plan[0], _participant=7),
                          [part("pedestrian")], tmp_path, log=lambda *a: None, dashboard=d, plan=plan)
        logger = FakeLogger()
        tools.bind(logger, queue=lambda c: None)
        result = []
        th = threading.Thread(target=lambda: result.append(tools.ready()))
        th.start()
        status = json.loads(request(d.port, "GET", "/status.json")[1])
        assert status["state"] in ("loading", "ready") and status["participant"] == 7
        assert status["plan"][0] == [0, 2, "ehmi_on", 45.0] and status["condition"] == "ehmi_on"
        request(d.port, "POST", "/api/start", {}, hdr)
        th.join(5)
        assert result == [True]
        tools.started()
        request(d.port, "POST", "/api/pause", {}, hdr)
        tools.step(1.0, 0.02, frame=10)
        assert tools.paused
        th = threading.Thread(target=tools.hold)
        th.start()
        assert json.loads(request(d.port, "GET", "/status.json")[1])["state"] == "paused" or th.is_alive()
        request(d.port, "POST", "/api/resume", {}, hdr)
        th.join(5)
        assert not th.is_alive() and not tools.paused
        request(d.port, "POST", "/api/stop", {}, hdr)
        tools.step(1.02, 0.02, frame=11)
        assert tools.abort and tools.stop
        names = [r[2] for r in logger.rows]
        assert names == ["paused", "resumed", "session_stopped"]
        summary = {}
        tools.close(summary)
        assert summary["aborted"] == "session stopped"
        assert [e[1] for e in d.status()["events"]] == names
        # the next trial's gate sees a stop at once
        tools2 = HostStudy({"name": "demo"}, plan[1], [], tmp_path, log=lambda *a: None, dashboard=d, plan=plan)
        d._cmds.put({"cmd": "stop"})
        assert tools2.ready() is False and tools2.stop
    finally:
        d.close()


def test_no_dashboard_means_no_gate(tmp_path):
    tools = HostStudy({"name": "demo"}, {"duration_s": 5}, [], tmp_path, log=lambda *a: None)
    assert tools.ready() is True
    tools.hold()                 # not paused: returns at once
    assert tools.status()["state"] == "ready"
    assert types.SimpleNamespace(**tools.status()).collisions == 0


def test_token_check_survives_odd_tokens_and_ipv6_loopback():
    """A non-ASCII token is refused (403), not an exception in the handler; ::1 (in LOCAL) can be bound."""
    import socket
    import pytest
    d = Dashboard("0.0.0.0", 0, log=lambda *a: None, token="s3cret")
    try:
        assert request(d.port, "GET", "/status.json?token=%C3%A9t%C3%A9")[0] == 403
        assert request(d.port, "GET", "/status.json", headers={"X-Token": "s3cret"})[0] == 200
    finally:
        d.close()
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.bind(("::1", 0))
        s.close()
    except OSError as err:        # no IPv6 loopback here
        pytest.skip(f"no ::1 ({err})")
    d = Dashboard("::1", 0, log=lambda *a: None)
    try:
        assert d.local and d.token is None and d.url == f"http://[::1]:{d.port}/"
        c = http.client.HTTPConnection("::1", d.port, timeout=5)
        c.request("GET", "/status.json", headers={"Host": f"[::1]:{d.port}"})
        assert c.getresponse().status == 200
        c.close()
    finally:
        d.close()


def test_page_escapes_what_stations_send():
    """Response values come from the stations over the LAN: the page escapes them like every other field."""
    from coupledcarla.dashboard import PAGE
    assert "${r.presses" not in PAGE and "${r.slider" not in PAGE
    assert "${esc(r.presses)}" in PAGE and "${esc(r.slider)}" in PAGE
