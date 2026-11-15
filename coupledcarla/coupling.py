"""Host <-> station control channel for coupled experiments (coupled-sim's NetworkingManager, over TCP).

The host (coupledcarla.run) owns the CARLA world, SUMO, AI agents and the log. Every human sits at a station
(coupledcarla.station) that connects twice: to the CARLA server, to render its own camera and apply its own
controls directly, and to the host on this channel, for everything else: which actor is theirs, when a trial
starts and ends, visual-sync flashes, eHMI commands from an AV passenger, and latency.

Messages are JSON objects, one per line. Station -> host: hello {role}, ping {t}, ehmi {state}, sync {},
stats {rtt_ms, fps, frames, latency, ...}, camera_turn, camera_done, bye. Host -> station: welcome {role, ...},
state {state, actor_id, ...}, pong {t, th}, tick {frame, t}, flash {frame}, ehmi {state} (echo to the passenger),
camera_go.

Camera turns (multi-GPU, multigpu.py): a station that must see which secondary takes its camera asks for a turn
(camera_turn); the server grants one station at a time (camera_go) until it says camera_done, leaves, or holds
the turn longer than TURN_S.

Lobby (session.py, the session dashboard): a station started without --role says hello without a role (with its
machine, input devices and GPU) and waits; the session answers `lobby` and, once the experimenter gives it a role,
`assign {role, carla}`; the station then runs its trials with that role, each trial's process saying hello with the
role as before. A plain host (run.py) has no lobby: it answers such a hello with `lobby_refused` and a reason.
With a session, the trial's host process (run.py --control-relay) does not listen for stations itself: RelayControl
talks to the session over a loopback link, and the session passes the stations' messages both ways.

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

TURN_S = 30.0       # a camera turn not given back after this long goes to the next station


def _send(sock, lock, msg):
    data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
    with lock:
        sock.sendall(data)


LOBBY_REFUSED = ("This host gives no roles: start the station with --role (driver, pedestrian, ...), or run the "
                 "session dashboard (python -m coupledcarla.session) and give it a role there.")


class _HostSide:
    """What run.py uses of the host's end, shared by ControlServer and RelayControl: needs connected(), send(),
    on_hello, log and port."""
    on_hello = None

    def rewelcome(self, roles):
        """Sends each connected station of `roles` a welcome made now (on_hello). A station's next trial process
        says hello while the host is still closing the previous trial, so its welcome came from that trial's
        on_hello: the previous trial's eHMI states, sound, displays, responses and take-over settings."""
        if self.on_hello is None:
            return
        for role in sorted(set(roles) & self.connected()):
            self.send(role, {"type": "welcome", "role": role, **(self.on_hello(role) or {})})

    def wait_for(self, roles, timeout, log_every=15.0):
        """Block until every role in `roles` has a station, or timeout (s), or the session cancels the trial.
        Returns the roles still missing."""
        t0, last = time.time(), 0.0
        while True:
            missing = set(roles) - self.connected()
            if not missing or time.time() - t0 > timeout or getattr(self, "cancelled", False):
                return missing
            if time.time() - last > log_every:
                self.log(f"waiting for stations: {', '.join(sorted(missing))} (port {self.port})")
                last = time.time()
            time.sleep(0.2)


class ControlServer(_HostSide):
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
        self._turn, self._turn_t, self._turn_queue = None, 0.0, []      # camera turns (see the module doc)
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
                if kind in ("camera_turn", "camera_done"):
                    self._camera(role, kind)
                if kind == "hello" and not msg.get("role"):      # a station waiting for a role: not here
                    _send(conn, lock, {"type": "lobby_refused", "why": LOBBY_REFUSED})
                    self.log(f"a station without a role connected from {addr[0]}: told to start with --role")
                    return
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
                self._camera(role, "camera_done")
                self.inbox.put({"type": "bye", "role": role, "recv": time.time()})
                self.log(f"station '{role}' disconnected")
            conn.close()

    def _camera(self, role, kind):
        """Camera turns: one station at a time between camera_go and its camera_done."""
        grant = None
        with self._lock:
            if kind == "camera_turn":
                if role not in self._turn_queue and role != self._turn:
                    self._turn_queue.append(role)
            elif role == self._turn:
                self._turn = None
            else:
                self._turn_queue = [r for r in self._turn_queue if r != role]
            if self._turn is not None and time.time() - self._turn_t > TURN_S:
                self.log(f"station '{self._turn}' kept its camera turn over {TURN_S:.0f} s: next")
                self._turn = None
            if self._turn is None and self._turn_queue:
                self._turn, self._turn_t = self._turn_queue.pop(0), time.time()
                grant = self._turn
        if grant is not None:
            self.send(grant, {"type": "camera_go"})

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

    def close(self):
        self.broadcast({"type": "state", "state": "closed"})
        try:
            self.sock.close()
        except OSError:
            pass


class ControlClient:
    """The station's end: connects, says hello, keeps reading messages into a queue, pings every second."""

    def __init__(self, host, port, role, retry_s=300, on_message=None, hello=None):
        self.role = role                  # None: a station waiting in a session's lobby (hello carries its info)
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
        self.send({"type": "hello", "role": role, **(hello or {})})
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
        self.inbox.put({"type": "state", "state": "closed", "eof": True})     # eof: the connection dropped

    def _ping(self):
        while True:
            rtt = self.rtt()
            try:
                self.send({"type": "ping", "t": time.time(), **({"rtt_ms": round(rtt, 1)} if rtt is not None else {})})
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


class RelayDashboard:
    """The dashboard object HostStudy (studytools.py) talks to when a session dashboard runs the trial: its status
    goes to the session, the session's commands come back (see RelayControl)."""

    url = "the session dashboard"

    def __init__(self, relay):
        self.relay = relay
        self._cmds = queue.Queue()
        self._status = {}

    def set_status(self, status):
        self._status = dict(status)
        self.relay.link({"type": "status", "status": self._status})

    def update(self, **fields):
        self.set_status({**self._status, **fields})

    def status(self):
        return dict(self._status)

    def commands(self):
        out = []
        while True:
            try:
                out.append(self._cmds.get_nowait())
            except queue.Empty:
                return out

    def close(self):
        pass


class RelayControl(_HostSide):
    """The host's end of the control channel when a session dashboard (session.py) owns the stations: run.py
    --control-relay HOST:PORT. Same interface as ControlServer (send, broadcast, poll, connected, wait_for,
    rewelcome, on_hello, close); messages go over one loopback link to the session, which passes them to and from
    the stations. Link messages (JSON lines) host -> session: attach {token}, to {role, msg}, all {msg},
    status {status}, live {...}; session -> host: connected {roles}, station {msg} (a station's message, with its
    role and receive time), cmd {cmd, ...} (the experimenter's commands). Closing it does not close the stations:
    they stay with the session for the next trial."""

    def __init__(self, address, token, log=print, timeout=10.0):
        host, port = str(address).rsplit(":", 1)
        self.log, self.port = log, int(port)
        self.sock = socket.create_connection((host, int(port)), timeout=timeout)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.lock = threading.Lock()
        self.inbox = queue.Queue()
        self._connected = set()
        self._hello_fn, self._pending = None, []
        self._plock = threading.Lock()
        self.cancelled = False        # the session ended the trial or the session (wait_for stops waiting)
        self.gone = False             # the session went away: the trial ends
        self.dashboard = RelayDashboard(self)
        self.link({"type": "attach", "token": token})
        threading.Thread(target=self._read, daemon=True).start()

    # on_hello is set by run_trial after the link is up; hellos that arrive before get their welcome then
    @property
    def on_hello(self):
        return self._hello_fn

    @on_hello.setter
    def on_hello(self, fn):
        with self._plock:
            self._hello_fn = fn
            pending, self._pending = self._pending, []
        for role in pending:
            self._welcome(role)

    def _welcome(self, role):
        fn = self._hello_fn
        self.send(role, {"type": "welcome", "role": role, **((fn(role) if fn else None) or {})})

    def link(self, msg):
        if self.gone:
            return
        try:
            _send(self.sock, self.lock, msg)
        except OSError:
            self._lost()

    def _lost(self):
        if not self.gone:
            self.gone = self.cancelled = True
            self.log("the session dashboard went away: ending the trial")
            self.dashboard._cmds.put({"cmd": "stop"})

    def _read(self):
        try:
            for line in self.sock.makefile("r"):
                msg = json.loads(line)
                kind = msg.get("type")
                if kind == "connected":
                    self._connected = set(msg.get("roles") or [])
                elif kind == "station":
                    m = msg.get("msg") or {}
                    if m.get("type") == "hello" and m.get("role"):
                        with self._plock:
                            later = self._hello_fn is None
                            if later:
                                self._pending.append(m["role"])
                        if not later:
                            self._welcome(m["role"])
                    self.inbox.put(m)
                elif kind == "cmd":
                    cmd = {k: v for k, v in msg.items() if k != "type"}
                    if cmd.get("cmd") in ("abort", "stop"):
                        self.cancelled = True
                    self.dashboard._cmds.put(cmd)
        except (OSError, ValueError):
            pass
        self._lost()

    def send(self, role, msg):
        self.link({"type": "to", "role": role, "msg": msg})

    def broadcast(self, msg):
        self.link({"type": "all", "msg": msg})

    def live(self, frame):
        self.link({"type": "live", **frame})

    def poll(self):
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def connected(self):
        return set(self._connected)

    def close(self):
        self.gone = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)     # a plain close keeps the fd open while the reader's file has it
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
