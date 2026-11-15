"""The session dashboard's end of the control channel (session.py): stations join a lobby and get their role from the
experimenter; each trial's host process (run.py --control-relay) attaches over a loopback link and talks to the
stations through here.

Two kinds of station connection arrive on the coupling port (2100):
  lobby   a station started without --role (coupledcarla.lobby): hello {role: null, lobby, station: id, info:
          {machine, inputs, gpu, input, platform}}. It is listed, gets `lobby {station, role, carla}` back and later
          `assign {role, carla}` when the experimenter picks its role. Up to MAX stations.
  trial   a station's trial process: hello {role, station?} as with a plain host (run.py), so stations started with
          --role work unchanged. Its messages go to the attached trial host; the host's messages for that role come
          back to it.

Pings are answered here at once (as ControlServer does), so a station's round trip is to this process, which runs
on the host machine. Camera turns (multigpu.py) are ControlServer's.
"""
import json
import re
import secrets
import socket
import threading
import time
import uuid

from .coupling import ControlServer, _send

MAX = 8                     # stations in the lobby, as coupled-sim's host plus seven players
SAFE_ID = re.compile(r"[^A-Za-z0-9_.-]")


def clean_info(info):
    """What a station says about itself, trimmed to short strings (it ends up in a web page and in logs)."""
    info = info if isinstance(info, dict) else {}

    def s(v, n=64):
        return str(v)[:n] if v not in (None, "") else None
    inputs = info.get("inputs") if isinstance(info.get("inputs"), list) else []
    return {"machine": s(info.get("machine")), "gpu": s(info.get("gpu"), 80), "input": s(info.get("input"), 16),
            "platform": s(info.get("platform"), 24), "bot": bool(info.get("bot")),
            "inputs": [str(x)[:64] for x in inputs[:8]]}


class Hub(ControlServer):
    def __init__(self, port, log=print, carla=None, on_host=None, on_change=None):
        self.lobby = {}           # station id -> {conn, lock, addr, info, rtt_ms, since}
        self.roles_of = {}        # station id -> the role the experimenter gave it
        self.trial = {}           # role -> {station, addr, since, info, rtt_ms} of its trial connection
        self.carla = dict(carla or {})    # {host, port} of the CARLA server, sent with every role
        self.on_host = on_host    # callable(msg): status, live and detached messages from the trial host
        self.on_change = on_change
        self.host = None
        self._hlock = threading.Lock()
        self.token = secrets.token_urlsafe(16)
        self.link = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.link.bind(("127.0.0.1", 0))
        self.link.listen(4)
        self.link_port = self.link.getsockname()[1]
        super().__init__(port, log)
        threading.Thread(target=self._accept_host, daemon=True).start()

    # -- stations -------------------------------------------------------------------------------------------------
    def _serve(self, conn, addr):
        role, sid, lobby, lock = None, None, False, threading.Lock()
        try:
            for line in conn.makefile("r"):
                msg = json.loads(line)
                kind = msg.get("type")
                if kind == "ping":                      # answered here, at once
                    _send(conn, lock, {"type": "pong", "t": msg.get("t"), "th": time.time()})
                    if msg.get("rtt_ms") is not None:
                        self._rtt(sid if lobby else role, lobby, msg["rtt_ms"])
                    continue
                if lobby:
                    continue                            # a lobby connection only pings
                if kind == "hello" and not msg.get("role") and role is None:
                    sid = self._join(conn, lock, addr, msg)
                    if sid is None:
                        return
                    lobby = True
                    continue
                if kind in ("camera_turn", "camera_done"):
                    self._camera(role, kind)
                if kind == "hello":
                    role = str(msg["role"])[:64]
                    with self._lock:
                        self.stations[role] = (conn, lock, addr)
                        self.trial[role] = {"station": SAFE_ID.sub("", str(msg.get("station") or ""))[:32] or None,
                                            "addr": addr[0], "since": time.time(),
                                            "info": clean_info(msg.get("info")), "rtt_ms": None}
                    self.log(f"station '{role}' connected from {addr[0]}")
                    self._changed()
                msg["role"] = role
                msg["recv"] = time.time()
                self.to_host({"type": "station", "msg": msg})
        except (OSError, ValueError, KeyError):
            pass
        finally:
            if lobby:
                with self._lock:
                    if (self.lobby.get(sid) or {}).get("conn") is conn:
                        del self.lobby[sid]
                self.log(f"station {sid} left the lobby")
                self._changed()
            if role:
                with self._lock:
                    if self.stations.get(role, (None,))[0] is conn:
                        del self.stations[role]
                        self.trial.pop(role, None)
                self._camera(role, "camera_done")
                self.to_host({"type": "station", "msg": {"type": "bye", "role": role, "recv": time.time()}})
                self.log(f"station '{role}' disconnected")
                self._changed()
            conn.close()

    def _join(self, conn, lock, addr, msg):
        sid = SAFE_ID.sub("", str(msg.get("station") or ""))[:32] or uuid.uuid4().hex[:8]
        with self._lock:
            if sid not in self.lobby and len(self.lobby) >= MAX:
                full = True
            else:
                full = False
                self.lobby[sid] = {"conn": conn, "lock": lock, "addr": addr[0], "info": clean_info(msg.get("info")),
                                   "rtt_ms": None, "since": time.time()}
            role = self.roles_of.get(sid)
        if full:
            _send(conn, lock, {"type": "lobby_refused",
                               "why": f"The session already has {MAX} stations; remove one on the dashboard first."})
            return None
        _send(conn, lock, {"type": "lobby", "station": sid, "role": role, "carla": self.carla})
        self.log(f"station {sid} ({self.lobby[sid]['info'].get('machine') or addr[0]}) joined the lobby")
        self._changed()
        return sid

    def _rtt(self, key, lobby, rtt):
        try:
            rtt = round(float(rtt), 1)
        except (TypeError, ValueError):
            return
        with self._lock:
            entry = (self.lobby if lobby else self.trial).get(key)
            if entry is not None:
                entry["rtt_ms"] = rtt

    def assign(self, sid, role):
        """Gives a lobby station its role (None: no role); it starts its trial process with it."""
        with self._lock:
            if role:
                self.roles_of[sid] = role
            else:
                self.roles_of.pop(sid, None)
            entry = self.lobby.get(sid)
        if entry is not None:
            try:
                _send(entry["conn"], entry["lock"], {"type": "assign", "role": role, "carla": self.carla})
            except OSError:
                pass
        self._changed()

    def stations_view(self):
        """Lobby stations and stations started with --role, in the order they came, for the dashboard."""
        with self._lock:
            lobby = {k: dict(v) for k, v in self.lobby.items()}
            trial = {k: dict(v) for k, v in self.trial.items()}
            roles_of = dict(self.roles_of)
        out = []
        for sid, e in lobby.items():
            role = roles_of.get(sid)
            tr = trial.get(role) if role else None
            out.append({"id": sid, "name": e["info"].get("machine") or e["addr"], "ip": e["addr"], "info": e["info"],
                        "rtt_ms": (tr or {}).get("rtt_ms") or e["rtt_ms"], "role": role, "fixed": False,
                        "ready": bool(tr and tr["station"] == sid), "since": e["since"]})
        known = set(lobby)
        for role, tr in trial.items():
            if tr["station"] in known:
                continue
            out.append({"id": f"role:{role}", "name": tr["info"].get("machine") or tr["addr"], "ip": tr["addr"],
                        "info": tr["info"], "rtt_ms": tr["rtt_ms"], "role": role, "fixed": True, "ready": True,
                        "since": tr["since"]})
        return sorted(out, key=lambda r: r["since"])

    def _changed(self):
        self.to_host({"type": "connected", "roles": sorted(self.connected())})
        if self.on_change is not None:
            self.on_change()

    # -- the trial host -------------------------------------------------------------------------------------------
    def _accept_host(self):
        while True:
            try:
                conn, _ = self.link.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._serve_host, args=(conn,), daemon=True).start()

    def _serve_host(self, conn):
        lock, attached = threading.Lock(), False
        try:
            conn.settimeout(10)
            f = conn.makefile("r")
            first = json.loads(f.readline() or "{}")
            if first.get("type") != "attach" or \
                    not secrets.compare_digest(str(first.get("token", "")).encode(), self.token.encode()):
                return
            conn.settimeout(None)
            with self._hlock:
                old, self.host = self.host, (conn, lock)
            if old is not None:
                old[0].close()
            attached = True
            self.log("trial host attached")
            _send(conn, lock, {"type": "connected", "roles": sorted(self.connected())})
            for role in sorted(self.connected()):        # stations already waiting get this trial's welcome
                _send(conn, lock, {"type": "station", "msg": {"type": "hello", "role": role, "recv": time.time()}})
            for line in f:
                msg = json.loads(line)
                kind = msg.get("type")
                if kind == "to":
                    self.send(str(msg.get("role")), msg.get("msg") or {})
                elif kind == "all":
                    self.broadcast(msg.get("msg") or {})
                elif self.on_host is not None:
                    self.on_host(msg)
        except (OSError, ValueError):
            pass
        finally:
            with self._hlock:
                if self.host is not None and self.host[0] is conn:
                    self.host = None
            conn.close()
            if attached:
                self.log("trial host detached")
                if self.on_host is not None:
                    self.on_host({"type": "detached"})

    def to_host(self, msg):
        with self._hlock:
            h = self.host
        if h is None:
            return False
        try:
            _send(h[0], h[1], msg)
            return True
        except OSError:
            return False

    def command(self, cmd):
        """An experimenter's command for the trial host (HostStudy and livemap act on it)."""
        return self.to_host({"type": "cmd", **cmd})

    def close(self):
        super().close()
        with self._lock:
            lobby = list(self.lobby.values())
        for e in lobby:
            try:
                _send(e["conn"], e["lock"], {"type": "state", "state": "closed"})
            except OSError:
                pass
        try:
            self.link.close()
        except OSError:
            pass
