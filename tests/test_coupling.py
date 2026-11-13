"""coupledcarla.coupling: host-station control channel over loopback, clock offset and lag."""
import socket
import time

from coupledcarla.coupling import ControlClient, ControlServer


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
        time.sleep(0.05)
    return False


def test_hello_ping_and_ticks():
    server = ControlServer(free_port(), log=lambda *a: None)
    server.on_hello = lambda role: {"experiment": "demo"}
    client = ControlClient("127.0.0.1", server.port, "pedestrian", retry_s=5)
    try:
        assert wait(lambda: server.connected() == {"pedestrian"})
        assert wait(lambda: client.rtt() is not None)
        assert abs(client.clock_offset()) < 0.05          # same machine: the clocks agree
        welcome = []
        assert wait(lambda: welcome.extend(m for m in client.poll() if m["type"] == "welcome") or welcome)
        assert welcome[0]["experiment"] == "demo"
        t = time.time()
        server.broadcast({"type": "tick", "frame": 7, "t": t})
        assert wait(lambda: 7 in client.host_ticks)
        lag = client.lag_ms(7, time.time())
        assert lag is not None and 0 <= lag < 1000
        client.send({"type": "gaze", "target": "av1"})
        msgs = []
        assert wait(lambda: msgs.extend(server.poll()) or any(m["type"] == "gaze" for m in msgs))
        assert next(m for m in msgs if m["type"] == "gaze")["role"] == "pedestrian"
    finally:
        server.close()


def test_clock_offset_uses_the_fastest_ping():
    c = ControlClient.__new__(ControlClient)
    c.clock = [(0.040, 2.0), (0.004, 0.5), (0.020, 1.0)]
    c.host_ticks = {10: 100.0}
    assert c.clock_offset() == 0.5
    assert abs(c.lag_ms(10, 99.6) - 100.0) < 1e-6
    assert c.lag_ms(11, 99.6) is None
