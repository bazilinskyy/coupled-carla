"""CARLA 1.0 multi-GPU: a primary server that simulates and secondary servers that render the cameras.

CARLA 1.0 (ue58-dev, Docs/core_multigpu.md) splits a simulation over several servers: the primary owns the world,
the physics and the clients; each secondary follows it frame by frame and renders the sensors the primary hands
it. The primary needs no GPU (`-nullrhi`). coupled-carla uses this to render the participants' cameras on other
GPUs than the simulation's: a second GPU of the server machine, or the GPU of a station's own PC, so that the
station's images do not cross the LAN (a 1280x720 camera at 50 Hz is 184 MB/s of raw pixels; gigabit Ethernet
carries 117 MB/s).

    server:
      multigpu:
        secondaries:
          - {name: gpu0, host: 127.0.0.1, rpc_port: 3000, gpu: 0}                    # on the server machine
          - {name: ped, host: 131.155.1.2, rpc_port: 4000, gpu: 0, roles: [pedestrian]}  # on the pedestrian's PC

    python -m coupledcarla.multigpu experiments/x.yaml                  # the plan: servers, ports, commands, checks
    python -m coupledcarla.multigpu experiments/x.yaml --secondary ped --root <CARLA 1.0 package> --primary <ip>
                                                                        # run secondary "ped" here, restarted
                                                                        # whenever the primary restarts

What CARLA decides, and what coupled-carla does about it:
- A server is a secondary when it gets `-carla-primary-host`; it connects to the primary's secondary port
  (`server.port` + 2, 2002). Start the primary first, then the secondaries, then the clients. The host starts the
  secondaries marked `launch` (on its own machine) right after the primary; the others are started on their
  machines with `--secondary NAME` and wait for the primary.
- The first secondary to connect switches the primary to synchronous mode at 1/20 s. The host applies its own
  settings after the secondaries are up and checks them every 5 s (guard_settings).
- The primary hands each new camera to the next connected secondary, round-robin in the order of `listen()`
  calls; a client cannot choose. A station whose role is listed in a secondary's `roles` therefore asks the host
  for a turn (no other station listens meanwhile), listens, sees which server its stream went to, and if it is
  not its own secondary destroys the camera and spawns a new one, which goes to the next secondary (at most once
  round all of them).
- A secondary's stream token carries a port but no address, and the client fills in the address it used for the
  primary. A camera rendered on another machine than the primary is therefore only reachable through a relay:
  a station whose plan names a secondary on another machine connects to the primary through 127.0.0.1 (relay
  below), so the address it fills in is its own machine: its own secondary is reached directly, the primary and
  the other secondaries through the relay. Every server's streaming port must be unique for that (problems()).
- Not mirrored to secondaries: debug drawings (`world.debug`: the eHMI light bars and text, drawn SUMO traffic
  lights), console variables and textures applied at run time. Designed eHMIs with `ehmi.render: station` are
  drawn by the stations and do show.

Untested on real GPUs (2026-11-15): the configuration, the command lines, the relay and the turn protocol are
tested without CARLA (tests/test_multigpu.py); see the README for the test to run when a GPU is free.
"""
import os
import shlex
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

LOCAL_HOSTS = {"127.0.0.1", "localhost", "::1", "0.0.0.0", ""}
RPC_BASE, RPC_STEP = 3000, 1000          # default ports of secondary i: 3000 + 1000 i (3000-3002, 4000-4002, ...)
SETTINGS_EVERY_S = 5.0


def _is_local(host):
    if host in LOCAL_HOSTS:
        return True
    try:
        names = {socket.gethostname(), socket.getfqdn()}
        own = set()
        for n in names:
            try:
                own.update(socket.gethostbyname_ex(n)[2])
            except OSError:
                pass
        return host in names or host in own
    except OSError:
        return False


def _port_open(host, port, timeout=1.0):
    try:
        socket.create_connection((host, port), timeout=timeout).close()
        return True
    except OSError:
        return False


class Secondary:
    """One secondary server of server.multigpu.secondaries (defaults from its position i)."""

    def __init__(self, spec, i):
        spec = spec or {}
        self.name = str(spec.get("name") or f"s{i}")
        self.host = str(spec.get("host") or "127.0.0.1")
        self.rpc_port = int(spec.get("rpc_port") or RPC_BASE + RPC_STEP * i)
        self.streaming_port = int(spec.get("streaming_port") or self.rpc_port + 1)
        self.own_secondary_port = self.rpc_port + 2          # a secondary opens it too; it must be free
        self.gpu = spec.get("gpu", 0)
        self.local = _is_local(self.host)
        self.launch = bool(spec.get("launch", self.local))
        self.roles = [str(r) for r in (spec.get("roles") or [])]
        self.extra_args = [str(a) for a in (spec.get("extra_args") or [])]
        self.root = spec.get("root")                         # CARLA 1.0 on that machine (printed commands)
        self.packs = spec.get("packs")                       # its content packs folder (-carla-packs)
        self.windows = spec.get("os", "windows" if os.name == "nt" else "linux") == "windows"
        self.spec = spec

    def ports(self):
        return [self.rpc_port, self.streaming_port, self.own_secondary_port]

    def __repr__(self):
        return f"Secondary({self.name} {self.host}:{self.rpc_port} gpu {self.gpu})"


def settings(sc):
    """The multi-GPU part of a `server` config, normalised: {secondaries: [Secondary], primary_renders,
    secondary_port, primary_host, relay, wait_s, placement}."""
    mg = (sc or {}).get("multigpu") or {}
    secs = [Secondary(s, i) for i, s in enumerate(mg.get("secondaries") or [])]
    port = int((sc or {}).get("port", 2000))
    return {"secondaries": secs, "primary_renders": bool(mg.get("primary_renders", False)),
            "rpc_port": port, "streaming_port": port + 1,
            "secondary_port": int(mg.get("secondary_port") or port + 2),
            "primary_host": mg.get("primary_host"), "relay": mg.get("relay", "auto"),
            "wait_s": float(mg.get("wait_s", 180)), "placement": mg.get("placement", True)}


def enabled(sc):
    return bool(((sc or {}).get("multigpu") or {}).get("secondaries"))


def problems(cfg):
    """[(level, message)] for server.multigpu of a loaded experiment (validate.py prints them)."""
    sc = cfg.get("server") or {}
    if not enabled(sc):
        return []
    out = []
    s = settings(sc)
    secs = s["secondaries"]
    names = [x.name for x in secs]
    for n in sorted({n for n in names if names.count(n) > 1}):
        out.append(("error", f"secondary name {n!r} is used twice"))
    roles = {p.get("role"): p for p in cfg.get("participants") or []}
    seen_roles = {}
    for x in secs:
        for r in x.roles:
            if roles and r not in roles:
                out.append(("error", f"{x.name}: role {r!r} is not a participant"))
            elif roles and not roles[r].get("station"):
                out.append(("warning", f"{x.name}: role {r!r} is not at a station (its camera is the host's)"))
            if r in seen_roles:
                out.append(("error", f"role {r!r} is listed by {seen_roles[r]} and {x.name}"))
            seen_roles[r] = x.name
        if x.launch and not x.local:
            out.append(("error", f"{x.name}: launch is only possible on this machine (host {x.host}); start it there "
                                 f"with python -m coupledcarla.multigpu <experiment> --secondary {x.name}"))
        try:
            if int(x.gpu) < -2:
                raise ValueError
        except (TypeError, ValueError):
            out.append(("error", f"{x.name}: gpu must be a device index (0, 1, ...; -1 any discrete GPU)"))
    # ports: servers on one machine must not share a port; streaming ports must be unique everywhere (a token
    # carries only the port, and a station's relay listens on each one at 127.0.0.1)
    primary = ("primary", "local", [s["rpc_port"], s["streaming_port"], s["secondary_port"]])
    servers = [primary] + [(x.name, "local" if x.local else x.host, x.ports()) for x in secs]
    taken = {}
    for name, host, ports in servers:
        for p in ports:
            other = taken.get((host, p))
            if other and other != name:
                out.append(("error", f"{name} and {other} both use port {p} on {host}"))
            taken[(host, p)] = name
    stream = [(s["streaming_port"], "primary")] + [(x.streaming_port, x.name) for x in secs]
    ports = [p for p, _ in stream]
    for p, n in stream:
        if ports.count(p) > 1:
            out.append(("error", f"streaming port {p} ({n}) is used by more than one server: give every secondary "
                                 f"its own rpc_port (3000, 4000, ...)"))
            break
    fixed = {int((cfg.get("coupling") or {}).get("port", 2100)): "the control channel",
             int(sc.get("tm_port", 8100)): "the Traffic Manager",
             int((cfg.get("dashboard") or {}).get("port", 8780)): "the dashboard"}
    for x in secs:
        for p in x.ports():
            if p in fixed:
                out.append(("error", f"{x.name}: port {p} is {fixed[p]}'s"))
    if any(not x.local for x in secs) and not s["primary_host"]:
        out.append(("warning", "secondaries on other machines need the primary's LAN address: set "
                               "server.multigpu.primary_host (or pass --primary to --secondary)"))
    ehmi = cfg.get("ehmi") or {}
    if any((a.get("ehmi") or "off") != "off" for a in cfg.get("agents") or []) and \
            ehmi.get("render", "world") == "world":
        out.append(("warning", "debug drawings are not mirrored to secondaries: the eHMI light bars and text drawn "
                               "in the world do not show in cameras they render (a designed eHMI with "
                               "ehmi.render: station does)"))
    return out


# -- command lines --------------------------------------------------------------------------------------------

def primary_args(sc):
    """Extra command-line arguments of the primary (added to server.extra_args when the host launches it)."""
    if not enabled(sc):
        return []
    s = settings(sc)
    out = [] if s["primary_renders"] else ["-nullrhi"]
    if s["secondary_port"] != s["rpc_port"] + 2:
        out.append(f"-carla-secondary-port={s['secondary_port']}")
    return out


def exe_path(root, windows):
    """The launcher in a CARLA 1.0 package (as server.py looks for it)."""
    root = str(root or ("<CARLA 1.0 package>" if windows else "<CARLA 1.0 package>"))
    sep = "\\" if windows else "/"
    return root.rstrip("/\\") + sep + ("CarlaUnreal.exe" if windows else "CarlaUnreal.sh")


def secondary_args(sc, sec, primary_host=None, quality=None):
    """The arguments (without the executable) that start `sec` as a secondary of the primary."""
    s = settings(sc)
    host = "127.0.0.1" if sec.local else (primary_host or s["primary_host"] or "<primary address>")
    args = [f"-carla-rpc-port={sec.rpc_port}"]
    if sec.streaming_port != sec.rpc_port + 1:
        args.append(f"-carla-streaming-port={sec.streaming_port}")
    args += [f"-carla-primary-host={host}", f"-carla-primary-port={s['secondary_port']}",
             f"-graphicsadapter={sec.gpu}", f"-quality-level={quality or (sc or {}).get('quality', 'Epic')}",
             "-RenderOffScreen", "-nosound", "-unattended"]
    if sec.packs:
        args.append(f"-carla-packs={sec.packs}")
    return args + sec.extra_args


def secondary_command(sc, sec, root=None, primary_host=None, packs=None):
    if packs and not sec.packs:
        sec.packs = packs
    return [exe_path(root or sec.root, sec.windows)] + secondary_args(sc, sec, primary_host)


def command_text(argv, windows):
    """A command line to paste: PowerShell on Windows, bash elsewhere."""
    if windows:
        exe, rest = argv[0], argv[1:]
        return "& '" + exe + "' " + " ".join(a if " " not in a else f"'{a}'" for a in rest)
    return shlex.join(argv)


# -- the host's side ------------------------------------------------------------------------------------------

class Launcher:
    """Starts the secondaries marked launch (this machine) after the primary is up, waits for every configured
    secondary's port, and stops the ones it started."""

    def __init__(self, sc, log=print, root=None, packs_args=()):
        self.sc, self.log = sc, log
        self.s = settings(sc)
        self.root = root
        self.packs_args = list(packs_args)
        self.procs = {}

    def start(self):
        windows = os.name == "nt"
        for sec in self.s["secondaries"]:
            if not sec.launch or _port_open("127.0.0.1", sec.rpc_port):
                continue
            exe = exe_path(sec.root or self.root or os.environ.get("CARLA_ROOT", "."), windows)
            if not Path(exe).exists():
                self.log(f"multi-GPU: cannot start secondary {sec.name}: {exe} does not exist (set its root, or "
                         f"launch: false and start it yourself)")
                continue
            argv = [exe] + secondary_args(self.sc, sec) + [a for a in self.packs_args if not sec.packs]
            self.procs[sec.name] = subprocess.Popen(argv, **({} if windows else {"start_new_session": True}))
            self.log(f"multi-GPU: started secondary {sec.name} on GPU {sec.gpu} (pid {self.procs[sec.name].pid}, "
                     f"port {sec.rpc_port})")
        return self.wait()

    def wait(self, timeout=None):
        """Waits for every secondary's RPC port; returns the names still missing (logged, not fatal: a station's
        secondary may come later, and the primary hands it the cameras that were waiting)."""
        timeout = self.s["wait_s"] if timeout is None else timeout
        t0, missing = time.time(), None
        while True:
            missing = [x.name for x in self.s["secondaries"]
                       if not _port_open("127.0.0.1" if x.local else x.host, x.rpc_port)]
            for name, p in self.procs.items():
                if p.poll() is not None and name in missing:
                    self.log(f"multi-GPU: secondary {name} exited during start-up (code {p.returncode})")
                    return missing
            if not missing or time.time() - t0 > timeout:
                break
            time.sleep(2)
        if missing:
            self.log(f"multi-GPU: secondaries not up after {timeout:.0f} s: {', '.join(missing)}")
        else:
            self.log(f"multi-GPU: {len(self.s['secondaries'])} secondaries up")
        return missing

    def stop(self):
        for name, p in self.procs.items():
            if p.poll() is None:
                if os.name == "nt":
                    subprocess.call(["taskkill", "/T", "/F", "/PID", str(p.pid)], stdout=subprocess.DEVNULL)
                else:
                    import signal
                    os.killpg(p.pid, signal.SIGKILL)
                try:
                    p.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    pass
                self.log(f"multi-GPU: secondary {name} stopped")
        self.procs = {}


def guard_settings(sc, step, log=print):
    """A function f(world) that puts back the host's synchronous mode and step when a secondary (connecting late)
    switched the primary to 1/20 s; it looks at most every SETTINGS_EVERY_S. None without secondaries."""
    if not enabled(sc):
        return None
    last = [0.0]

    def check(world, force=False):
        if not force and time.time() - last[0] < SETTINGS_EVERY_S:
            return False
        last[0] = time.time()
        st = world.get_settings()
        if st.synchronous_mode and st.fixed_delta_seconds and abs(st.fixed_delta_seconds - step) < 1e-6:
            return False
        log(f"multi-GPU: the primary ran at {st.fixed_delta_seconds} s "
            f"({'synchronous' if st.synchronous_mode else 'asynchronous'}; a secondary connected): back to {step} s")
        st.synchronous_mode, st.fixed_delta_seconds = True, step
        world.apply_settings(st)
        return True
    return check


def welcome(sc, role):
    """What a station needs to know about the secondaries (sent on hello): every secondary's name, host (as the
    server machine sees it: 127.0.0.1 is the server itself) and streaming port, which one is the role's own, and
    the primary's ports. {} without secondaries."""
    if not enabled(sc):
        return {}
    s = settings(sc)
    own = next((x.name for x in s["secondaries"] if role in x.roles), None)
    return {"multigpu": {
        "secondaries": [{"name": x.name, "host": "server" if x.local else x.host,
                         "streaming_port": x.streaming_port} for x in s["secondaries"]],
        "own": own, "rpc_port": s["rpc_port"], "streaming_port": s["streaming_port"],
        "relay": s["relay"], "placement": bool(s["placement"])}}


# -- the station's side ---------------------------------------------------------------------------------------

RELAY_PROCESS = True        # run a station's relay in a child process (see Relay)


def _relay_process(routes, bind, shared, ports, conn):
    """The relay's child process: forwards until the station's process ends (its end of the pipe closes)."""
    index = {p: i for i, p in enumerate(ports)}
    try:
        relay = Relay(routes, bind, process=False)
    except OSError as err:
        conn.send(f"{err}")
        return

    def bump(port):
        shared[index[port]] += 1
    relay.on_connect = bump
    conn.send("ok")
    try:
        conn.recv()             # nothing is sent: returns with EOFError once the station's process is gone
    except (EOFError, OSError):
        pass
    os._exit(0)


class Relay:
    """Forwards 127.0.0.1:port to (host, port) for each given pair; counts the connections per port, so a station
    can tell which server took its camera stream.

    By default it runs in a child process (RELAY_PROCESS): CARLA 1.0's Python client holds the GIL while it waits
    for some RPC answers (get_world), so a relay in the station's own threads could not forward that very request
    and every call through it timed out (measured on 2026-11-15 against a CARLA 1.0 primary: the request reached
    the relay at once and left it 15 s later, at the client's timeout). The child exits when the station's process
    ends, however it ends."""

    def __init__(self, routes, bind="127.0.0.1", process=None):
        self.routes = dict(routes)                # local port -> (host, port)
        self.ports = sorted(self.routes)
        self.connections = {p: 0 for p in self.routes}
        self.socks = []
        self.lock = threading.Lock()
        self.closed = False
        self.on_connect = None
        self.proc = None
        if RELAY_PROCESS if process is None else process:
            self._start_process(bind)
            return
        try:
            for port in self.routes:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if os.name != "nt":                # on Windows SO_REUSEADDR lets two sockets share a port
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((bind, port))
                s.listen(16)
                self.socks.append(s)
        except OSError:
            self.close()
            raise
        for s in self.socks:
            threading.Thread(target=self._accept, args=(s,), daemon=True).start()

    def _start_process(self, bind):
        import multiprocessing
        ctx = multiprocessing.get_context("spawn")
        self._shared = ctx.RawArray("i", len(self.ports))
        mine, theirs = ctx.Pipe()
        self.proc = ctx.Process(target=_relay_process, args=(self.routes, bind, self._shared, self.ports, theirs),
                                name="coupledcarla-relay", daemon=True)
        self.proc.start()
        theirs.close()
        msg = mine.recv() if mine.poll(30) else "the relay process did not start"
        if msg != "ok":
            if self.proc.is_alive():
                self.proc.kill()
            self.proc = None
            raise OSError(msg)
        self._pipe = mine           # kept open: the child relays as long as it is

    def _accept(self, srv):
        port = srv.getsockname()[1]
        while not self.closed:
            try:
                conn, _ = srv.accept()
            except OSError:
                return
            try:
                up = socket.create_connection(self.routes[port], timeout=10)
            except OSError:
                conn.close()
                continue
            up.settimeout(None)
            for c in (conn, up):
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.connections[port] += 1
            if self.on_connect is not None:
                self.on_connect(port)
            threading.Thread(target=self._pump, args=(conn, up), daemon=True).start()
            threading.Thread(target=self._pump, args=(up, conn), daemon=True).start()

    @staticmethod
    def _pump(a, b):
        try:
            while True:
                data = a.recv(1 << 20)
                if not data:
                    break
                b.sendall(data)
        except OSError:
            pass
        for s in (a, b):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            s.close()

    def counts(self):
        with self.lock:
            out = dict(self.connections)
        if self.proc is not None:
            for i, p in enumerate(self.ports):
                out[p] += self._shared[i]
        return out

    def close(self):
        self.closed = True
        if self.proc is not None:
            self._pipe.close()
            self.proc.join(2)
            if self.proc.is_alive():
                self.proc.kill()
            self.proc = None
        for s in self.socks:
            try:
                s.close()
            except OSError:
                pass


class StationRouting:
    """A station's view of the secondaries (the welcome's `multigpu`): where its carla.Client connects (direct,
    or 127.0.0.1 through a Relay when a secondary is on another machine than the primary) and where its camera
    lands (place_camera)."""

    def __init__(self, plan, host, port, log=print):
        self.plan, self.host, self.port, self.log = plan or {}, host, port, log
        self.relay = None
        self.own = self.plan.get("own")
        self.own_local = False
        self.on_primary = _is_local(host)       # this station runs on the primary's machine
        self.placed = None              # name of the server that renders this station's camera, when known
        if not self.plan:
            return
        servers = self._servers()
        # secondaries on this machine are reached directly at 127.0.0.1 (the address the client fills in)
        here = {n for n, h, p in servers
                if _is_local(h) or (n == self.own and _port_open("127.0.0.1", p, 0.5))}
        self.own_local = self.own in here
        away = [n for n, h, _p in servers if h != self.host]      # not on the primary's machine
        if self.plan.get("relay", "auto") in (False, "off") or not away:
            return              # every secondary is where the primary is: the client's own address works for all
        routes = {}
        if not self.on_primary:      # the client connects to the primary through 127.0.0.1 too
            routes[self.port] = (self.host, self.port)
            routes[int(self.plan["streaming_port"])] = (self.host, int(self.plan["streaming_port"]))
        for n, h, p in servers:
            if n not in here:
                routes[p] = (h, p)
        try:
            self.relay = Relay(routes)
            self.log(f"multi-GPU: relaying {', '.join(f'{p}->{h}' for p, (h, _) in sorted(routes.items()))}"
                     f"{'; own secondary ' + self.own + ' on this machine' if self.own_local else ''}")
        except OSError as err:
            self.log(f"multi-GPU: no relay ({err}); cameras on secondaries of other machines will not arrive")

    def _servers(self):
        """[(name, host as this station reaches it, streaming port)] of the secondaries."""
        return [(x["name"], self.host if x.get("host") in ("server",) or x.get("host") in LOCAL_HOSTS
                 else x["host"], int(x["streaming_port"])) for x in self.plan.get("secondaries") or []]

    def address(self):
        """(host, port) for carla.Client: 127.0.0.1 when relaying (or on the primary's machine), so that the
        address the client fills into stream tokens is this machine."""
        return ("127.0.0.1", self.port) if self.relay is not None or self.on_primary else (self.host, self.port)

    def can_place(self):
        """Whether this station can tell which server took its camera: its own secondary is on this machine
        and every other server is behind the relay."""
        return self.relay is not None and self.own_local and not self.on_primary and \
            bool(self.plan.get("placement", True))

    def which(self, before, after):
        """Name of the server whose relay port got a new connection between two Relay.counts() (None: none did,
        so the stream went to a server reached directly)."""
        by_port = {p: n for n, _h, p in self._servers()}
        by_port[self.plan.get("streaming_port")] = "primary"
        for p, n in after.items():
            if n > before.get(p, 0) and p != self.port:
                return by_port.get(p, f"port {p}")
        return None

    def place_camera(self, spawn, listen, destroy, got_image, turn=None, timeout=3.0):
        """Spawns and listens to the station's camera so that it lands on the role's own secondary. spawn() ->
        camera, listen(camera), destroy(camera); got_image: a threading.Event the image callback sets; turn: a
        context manager that keeps the other stations from listening meanwhile (the host's camera turn). Returns
        the camera."""
        tries = len(self.plan.get("secondaries") or []) if self.can_place() else 1
        with (turn if turn is not None and tries > 1 else _nothing()):
            cam = None
            for k in range(max(tries, 1)):
                got_image.clear()
                before = self.relay.counts() if self.relay is not None else {}
                cam = spawn()
                listen(cam)
                if tries <= 1:
                    return cam
                t0 = time.time()
                where = None
                while time.time() - t0 < timeout:
                    where = self.which(before, self.relay.counts())
                    if where is not None or got_image.is_set():
                        break
                    time.sleep(0.05)
                where = where or (self.own if got_image.is_set() else None)
                if where == self.own:
                    self.placed = where
                    self.log(f"multi-GPU: camera on {where} (try {k + 1})")
                    return cam
                self.log(f"multi-GPU: camera went to {where or 'no server (no image)'}; respawning it")
                if k < tries - 1:
                    destroy(cam)
            self.placed = None
            self.log(f"multi-GPU: camera not on {self.own} after {tries} tries; it stays where it is (relayed)")
            return cam

    def close(self):
        if self.relay is not None:
            self.relay.close()


class _nothing:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class CameraTurn:
    """The station's end of the host's camera turn (ControlServer grants one station at a time): a context
    manager; go is a threading.Event that the station's reader thread sets on `camera_go`."""

    def __init__(self, send, go, timeout=30.0):
        self.send, self.go, self.timeout = send, go, timeout
        self.result = {}

    def __enter__(self):
        self.go.clear()
        self.send({"type": "camera_turn"})
        if not self.go.wait(self.timeout):
            pass        # the host is older or busy: listen anyway
        return self

    def __exit__(self, *a):
        try:
            self.send({"type": "camera_done", **self.result})
        except OSError:
            pass
        return False


# -- secondary on this machine (a station's PC) ---------------------------------------------------------------

def supervise(argv, primary, primary_port, log=print, poll_s=2.0, stop=None, popen=subprocess.Popen):
    """Runs a secondary (argv) and restarts it whenever it exits or the primary goes away and comes back (a
    secondary that loses its primary stops following it until restarted). Returns when stop() is true."""
    stop = stop or (lambda: False)
    proc = None
    primary_up = False
    while not stop():
        up = _port_open(primary, primary_port)
        if proc is not None and (proc.poll() is not None or (primary_up and not up)):
            if proc.poll() is None:
                _kill(proc)
                log("multi-GPU: the primary went away: secondary stopped, waiting for the primary")
            else:
                log(f"multi-GPU: secondary exited (code {proc.returncode})")
            proc = None
        primary_up = up
        if proc is None and up:
            proc = popen(argv, **({} if os.name == "nt" else {"start_new_session": True}))
            log(f"multi-GPU: secondary started (pid {proc.pid}): {' '.join(argv)}")
        time.sleep(poll_s)
    if proc is not None:
        _kill(proc)
    return proc


def _kill(proc):
    if proc.poll() is not None:
        return
    if os.name == "nt":
        subprocess.call(["taskkill", "/T", "/F", "/PID", str(proc.pid)], stdout=subprocess.DEVNULL)
    else:
        import signal
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        pass


def plan_text(cfg, root=None, primary_host=None, windows=None):
    """The multi-GPU plan of an experiment as text: servers, ports, the commands, the checks."""
    sc = cfg.get("server") or {}
    if not enabled(sc):
        return "no server.multigpu.secondaries: one server renders everything"
    s = settings(sc)
    ph = primary_host or s["primary_host"]
    lines = [f"primary   {sc.get('host', '127.0.0.1')}:{s['rpc_port']} (streaming {s['streaming_port']}, "
             f"secondaries connect to {s['secondary_port']})"
             f"{'' if s['primary_renders'] else ', -nullrhi (renders nothing)'}",
             f"          extra arguments: {' '.join(primary_args(sc)) or '(none)'}"]
    for x in s["secondaries"]:
        win = x.windows if windows is None else windows
        where = "this machine (the host starts it)" if x.launch else \
            ("the server machine" if x.local else f"{x.host} (start it there)")
        lines.append(f"{x.name:9s} {where}; ports {x.rpc_port}-{x.own_secondary_port}, GPU {x.gpu}, cameras of "
                     f"{', '.join(x.roles) or 'any role (round-robin)'}")
        cmd = secondary_command(sc, x, root, ph)
        lines.append("          " + command_text(cmd, win))
        if not x.local:
            lines.append(f"          or: python -m coupledcarla.multigpu {cfg.get('_path', '<experiment>')} "
                         f"--secondary {x.name} --root <CARLA 1.0> --primary {ph or '<primary address>'}")
    for level, msg in problems(cfg):
        lines.append(f"{level:9s} {msg}")
    return "\n".join(lines)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="CARLA 1.0 multi-GPU secondaries of an experiment: the plan, or run "
                                             "one secondary on this machine.")
    ap.add_argument("experiment")
    ap.add_argument("--secondary", help="run this secondary here (restarted whenever the primary restarts)")
    ap.add_argument("--root", help="the CARLA 1.0 package on this machine (default: the secondary's root, CARLA_ROOT)")
    ap.add_argument("--primary", help="the primary's address as this machine reaches it")
    ap.add_argument("--packs", help="content packs folder (-carla-packs), the same packs as the primary's")
    ap.add_argument("--windows", action="store_true", help="print PowerShell commands")
    ap.add_argument("--dry", action="store_true", help="with --secondary: print the command, do not run it")
    a = ap.parse_args(argv)
    from . import config
    cfg = config.load(a.experiment)
    sc = cfg["server"]
    if not a.secondary:
        print(plan_text(cfg, a.root, a.primary, True if a.windows else None))
        return 1 if any(level == "error" for level, _ in problems(cfg)) else 0
    s = settings(sc)
    sec = next((x for x in s["secondaries"] if x.name == a.secondary), None)
    if sec is None:
        print(f"no secondary {a.secondary!r} in server.multigpu.secondaries", file=sys.stderr)
        return 2
    sec.windows = os.name == "nt"
    sec.local = False if a.primary else sec.local
    primary = a.primary or s["primary_host"] or "127.0.0.1"
    argv_ = secondary_command(sc, sec, a.root or sec.root or os.environ.get("CARLA_ROOT"), primary, a.packs)
    if a.dry:
        print(command_text(argv_, sec.windows))
        return 0
    if not Path(argv_[0]).exists():
        print(f"{argv_[0]} does not exist: pass --root <CARLA 1.0 package>", file=sys.stderr)
        return 2
    print(f"secondary {sec.name}: waiting for the primary at {primary}:{s['secondary_port']} (Ctrl+C stops)",
          flush=True)
    try:
        supervise(argv_, primary, s["secondary_port"], log=lambda m: print(m, flush=True))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
