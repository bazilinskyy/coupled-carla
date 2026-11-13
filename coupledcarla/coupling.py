"""Host <-> station control channel for coupled experiments (coupled-sim's NetworkingManager, over TCP).

The host (coupledcarla.run) owns the CARLA world, SUMO, AI agents and the log. Every human sits at a station
(coupledcarla.station) that connects twice: to the CARLA server, to render its own camera and apply its own
controls directly, and to the host on this channel, for everything else: which actor is theirs, when a trial
starts and ends, visual-sync flashes, eHMI commands from an AV passenger, and latency.

Messages are JSON objects, one per line. Station -> host: hello {role}, ping {t}, ehmi {state}, sync {},
stats {rtt_ms, fps, frames, latency}, bye. Host -> station: welcome {role, ...}, state {state, actor_id, ...},
pong {t, th}, tick {frame, t}, flash {frame}, ehmi {state} (echo to the passenger).

Latency: every pong carries the host's clock (th), so the station estimates the clock offset from the ping with
the shortest round trip (NTP's rule: offset = th - (sent + received) / 2). The host sends `tick` (frame number,
host time when the world tick returned) a few times a second; the station times when it sees that frame's
snapshot and when that frame's camera image arrives, both on the host's clock.
"""
import json
import queue
import socket
import threading
import time


def _send(sock, lock, msg):
    data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
    with lock:
        sock.sendall(data)


class ControlServer:
    """Accepts stations; answers pings in its own threads (so measured round trips do not include the tick)."""

    def __init__(self, port, log=print):
        self.log = log
        self.inbox = queue.Queue()
        self.stations = {}            # role -> (socket, lock, address)
        self._lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.listen(16)
        self.port = port
        self.on_hello = None          # callable(role) -> welcome dict (set by the host)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._serve, args=(conn, addr), daemon=True).start()

    def _serve(self, conn, addr):
        role, lock = None, threading.Lock()
        try:
            for line in conn.makefile("r"):
                msg = json.loads(line)
                kind = msg.get("type")
                if kind == "ping":                      # answered here, at once
                    _send(conn, lock, {"type": "pong", "t": msg.get("t"), "th": time.time()})
                    continue
                if kind == "hello":
                    role = msg["role"]
                    with self._lock:
                        self.stations[role] = (conn, lock, addr)
                    welcome = self.on_hello(role) if self.on_hello else {}
                    _send(conn, lock, {"type": "welcome", "role": role, **(welcome or {})})
                    self.log(f"station '{role}' connected from {addr[0]}")
                msg["role"] = role
                msg["recv"] = time.time()
                self.inbox.put(msg)
        except (OSError, ValueError):
            pass
        finally:
            if role:
                with self._lock:
                    if self.stations.get(role, (None,))[0] is conn:
                        del self.stations[role]
                self.inbox.put({"type": "bye", "role": role, "recv": time.time()})
                self.log(f"station '{role}' disconnected")
            conn.close()

    def send(self, role, msg):
        with self._lock:
            st = self.stations.get(role)
        if st:
            try:
                _send(st[0], st[1], msg)
            except OSError:
                pass

    def broadcast(self, msg):
        with self._lock:
            roles = list(self.stations)
        for role in roles:
            self.send(role, msg)

    def poll(self):
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def connected(self):
        with self._lock:
            return set(self.stations)

    def wait_for(self, roles, timeout, log_every=15.0):
        """Block until every role in `roles` has a station, or timeout (s). Returns the roles still missing."""
        t0, last = time.time(), 0.0
        while True:
            missing = set(roles) - self.connected()
            if not missing or time.time() - t0 > timeout:
                return missing
            if time.time() - last > log_every:
                self.log(f"waiting for stations: {', '.join(sorted(missing))} (port {self.port})")
                last = time.time()
            time.sleep(0.2)

    def close(self):
        self.broadcast({"type": "state", "state": "closed"})
        try:
            self.sock.close()
        except OSError:
            pass


class ControlClient:
    """The station's end: connects, says hello, keeps reading messages into a queue, pings every second."""

    def __init__(self, host, port, role, retry_s=300, on_message=None):
        self.role = role
        self.on_message = on_message      # called in the reader thread, before the message is queued
        self.inbox = queue.Queue()
        self.rtt_ms = []
        self.clock = []                   # (rtt s, host minus station clock s), last 60 pings
        self.host_ticks = {}              # frame -> host time when its tick returned (host clock)
        self.lock = threading.Lock()
        t0 = time.time()
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=5)
                break
            except OSError:
                if time.time() - t0 > retry_s:
                    raise
                time.sleep(2)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send({"type": "hello", "role": role})
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._ping, daemon=True).start()

    def _read(self):
        try:
            for line in self.sock.makefile("r"):
                msg = json.loads(line)
                if msg.get("type") == "pong":
                    now = time.time()
                    self.rtt_ms.append(1000.0 * (now - msg["t"]))
                    self.rtt_ms = self.rtt_ms[-30:]
                    if msg.get("th") is not None:
                        self.clock.append((now - msg["t"], msg["th"] - (msg["t"] + now) / 2.0))
                        self.clock = self.clock[-60:]
                    continue
                if msg.get("type") == "tick":
                    self.host_ticks[msg["frame"]] = msg["t"]
                    if len(self.host_ticks) > 400:
                        for f in sorted(self.host_ticks)[:200]:
                            self.host_ticks.pop(f, None)
                    continue
                if self.on_message:
                    self.on_message(msg)
                self.inbox.put(msg)
        except (OSError, ValueError):
            pass
        self.inbox.put({"type": "state", "state": "closed"})

    def _ping(self):
        while True:
            try:
                self.send({"type": "ping", "t": time.time()})
            except OSError:
                return
            time.sleep(1.0)

    def send(self, msg):
        _send(self.sock, self.lock, msg)

    def poll(self):
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def rtt(self):
        return sum(self.rtt_ms) / len(self.rtt_ms) if self.rtt_ms else None

    def clock_offset(self):
        """Host clock minus this station's clock (s), from the ping with the shortest round trip; None at first."""
        clock = list(self.clock)
        return min(clock)[1] if clock else None

    def lag_ms(self, frame, t_local):
        """How long after the host's tick of `frame` (host clock) something happened here at t_local."""
        th, off = self.host_ticks.get(frame), self.clock_offset()
        if th is None or off is None:
            return None
        return 1000.0 * (t_local + off - th)
