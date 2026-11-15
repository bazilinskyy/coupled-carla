"""Checks CARLA 1.0's multi-GPU protocol and coupled-carla's side of it (multigpu.py) without a second GPU.

A real primary (CARLA 1.0, e.g. the editor binary with -nullrhi on a CPU-only box) talks to stand-in secondaries
("fakes") that speak CARLA's secondary protocol (LibCarla/source/carla/multigpu: length-prefixed messages, a
CommandHeader, LOAD_MAP answered with EPISODE_READY, GET_TOKEN with a 24-byte streaming token, YOU_ALIVE) and serve
their own streaming port; a fake started with --images sends a small camera image to whoever subscribes, as a
rendering secondary would. Then a station's routing (multigpu.StationRouting: the relay and place_camera) runs
against them with a real carla client, so these hold or fail on a real server:

  - a secondary connecting switches the primary to synchronous mode at 1/20 s (multigpu.guard_settings puts the
    host's step back),
  - the primary asks the secondaries for a camera's token round-robin (GET_TOKEN with the camera's actor id),
  - the token's address is ignored or unspecified and the client connects to the address it used for the primary
    (so a station on another PC needs the relay), and the station's camera ends up on its own secondary.

    python tools/multigpu_check.py fake --primary 131.155.229.241 --secondary-port 2101 --stream-port 4001 --images
    python tools/multigpu_check.py settings --primary 131.155.229.241 --port 2099 [--step 0.02]
    python tools/multigpu_check.py station --primary 131.155.229.241 --port 2099 --role pedestrian \\
        --plan gpu0@server:3001,ped@100.80.227.26:4001/pedestrian       # (or the welcome's JSON list)
    python tools/multigpu_check.py experiment --base experiments/x.yaml --out experiments/_mgpu_test.yaml \\
        --secondary gpu0@127.0.0.1:3000                                  # the experiment with server.multigpu

Every subcommand prints JSON lines. No GPU is used anywhere (the primary needs -nullrhi).
"""
import argparse
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

SEND_FRAME, LOAD_MAP, GET_TOKEN, ENABLE_ROS, DISABLE_ROS, IS_ENABLED_ROS, YOU_ALIVE = range(7)
NAMES = {0: "SEND_FRAME", 1: "LOAD_MAP", 2: "GET_TOKEN", 3: "ENABLE_ROS", 4: "DISABLE_ROS", 5: "IS_ENABLED_ROS",
         6: "YOU_ALIVE"}
SCENE_CAPTURE = 13          # ASceneCaptureCamera's place in LibCarla's SensorRegistry (its wire type id)


def say(**kw):
    print(json.dumps(kw, default=str), flush=True)


def _recv(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def send_message(sock, payload):
    """A LibCarla streaming message: uint32 size, then the payload."""
    sock.sendall(struct.pack("<I", len(payload)) + payload)


def token(stream_id, port, address=None):
    """carla::streaming::detail::token_data (packed, 24 bytes): stream id, port, protocol (tcp = 1), address type
    (0 not set, 1 IPv4), 16 address bytes."""
    if address is None:
        return struct.pack("<IHBB16s", stream_id, port, 1, 0, b"")
    return struct.pack("<IHBB16s", stream_id, port, 1, 1, socket.inet_aton(address))


def parse_token(b):
    sid, port, proto, atype = struct.unpack("<IHBB", b[:8])
    addr = socket.inet_ntoa(b[8:12]) if atype == 1 else None
    return {"stream_id": sid, "port": port, "protocol": proto, "address": addr}


def image_message(frame, w=16, h=16, fov=90.0):
    """A camera image as a secondary streams it: SensorHeaderSerializer::Header (sensor type, frame, timestamp,
    transform), ImageSerializer::ImageHeader (width, height, fov), BGRA pixels."""
    head = struct.pack("<QQd6f", SCENE_CAPTURE, frame, frame * 0.05, 0, 0, 0, 0, 0, 0)
    return head + struct.pack("<IIf", w, h, fov) + bytes([40, 80, 160, 255]) * (w * h)


class Fake:
    """A stand-in secondary: connects to the primary's secondary port, answers its commands, serves a streaming
    port (and, with images, sends a few images to each subscriber)."""

    def __init__(self, primary, secondary_port, stream_port, name, images=False, token_address=None):
        self.primary, self.secondary_port, self.stream_port, self.name = primary, secondary_port, stream_port, name
        self.images, self.token_address = images, token_address
        self.counts = {}
        self.frames = 0
        self.lock = threading.Lock()

    def serve_stream(self):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.stream_port))
        srv.listen(16)
        while True:
            conn, peer = srv.accept()
            threading.Thread(target=self._subscriber, args=(conn, peer), daemon=True).start()

    def _subscriber(self, conn, peer):
        try:
            sid = struct.unpack("<I", _recv(conn, 4))[0]
            say(fake=self.name, event="stream_subscribed", stream_id=sid, peer=peer[0])
            if self.images:
                for k in range(20):
                    send_message(conn, image_message(1000 + k))
                    time.sleep(0.05)
                say(fake=self.name, event="images_sent", stream_id=sid, n=20)
            while conn.recv(4096):
                pass
        except (OSError, ConnectionError):
            pass

    def run(self):
        threading.Thread(target=self.serve_stream, daemon=True).start()
        while True:
            try:
                s = socket.create_connection((self.primary, self.secondary_port), timeout=10)
            except OSError as e:
                say(fake=self.name, event="waiting_for_primary", error=str(e))
                time.sleep(2)
                continue
            s.settimeout(None)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            say(fake=self.name, event="connected", primary=f"{self.primary}:{self.secondary_port}")
            try:
                self._commands(s)
            except (OSError, ConnectionError) as e:
                say(fake=self.name, event="primary_gone", error=str(e), counts=self.counts)
                time.sleep(2)

    def _commands(self, s):
        last = time.time()
        while True:
            n = struct.unpack("<I", _recv(s, 4))[0]
            payload = _recv(s, n)
            cid, size = struct.unpack("<II", payload[:8])
            data = payload[8:8 + size]
            name = NAMES.get(cid, str(cid))
            self.counts[name] = self.counts.get(name, 0) + 1
            if cid == SEND_FRAME:
                self.frames += 1
                if time.time() - last > 5:
                    say(fake=self.name, event="frames", n=self.frames, last_bytes=len(data))
                    last = time.time()
            elif cid == LOAD_MAP:
                mapname = data.split(b"\0", 1)[0].decode(errors="replace")
                load_id = struct.unpack("<I", data[len(mapname) + 1:len(mapname) + 5])[0] \
                    if len(data) >= len(mapname) + 5 else 0
                say(fake=self.name, event="load_map", map=mapname, load_id=load_id)
                send_message(s, b"EPISODE_READY" + struct.pack("<I", load_id))
            elif cid == GET_TOKEN:
                sid, actor = struct.unpack("<II", data[:8])
                t = token(sid, self.stream_port, self.token_address)
                say(fake=self.name, event="get_token", stream_id=sid, actor_id=actor, answered_port=self.stream_port,
                    answered_address=self.token_address)
                send_message(s, t)
            elif cid == YOU_ALIVE:
                send_message(s, b"Yes, I'm alive")
            elif cid in (ENABLE_ROS, DISABLE_ROS, IS_ENABLED_ROS):
                send_message(s, b"\0")
            else:
                say(fake=self.name, event="unknown_command", id=cid, size=size)


def parse_plan(text):
    """Secondaries as the welcome lists them: a JSON list, or name@host:streaming_port[/role+role] items separated by
    commas (no quotes needed in any shell)."""
    text = text.strip()
    if text.startswith("["):
        return json.loads(text)
    out = []
    for item in filter(None, text.split(",")):
        head, _, roles = item.partition("/")
        name, _, where = head.partition("@")
        host, _, port = where.rpartition(":")
        out.append({"name": name, "host": host, "streaming_port": int(port),
                    "roles": [r for r in roles.split("+") if r]})
    return out


def cmd_experiment(a):
    """Writes the base experiment with server.multigpu.secondaries (name@host:rpc_port, launch false) and the server
    not launched (it runs already)."""
    import yaml
    cfg = yaml.safe_load(Path(a.base).read_text())
    secs = []
    for item in a.secondary:
        name, _, where = item.partition("@")
        host, _, port = where.rpartition(":")
        secs.append({"name": name, "host": host, "rpc_port": int(port), "launch": False})
    server = cfg.setdefault("server", {})
    server["launch"] = False
    server["multigpu"] = {"secondaries": secs}
    Path(a.out).write_text(yaml.safe_dump(cfg, sort_keys=False))
    say(check="experiment", out=a.out, secondaries=secs)


def cmd_settings(a):
    import carla
    from coupledcarla import multigpu
    c = carla.Client(a.primary, a.port)
    c.set_timeout(30)
    w = c.get_world()
    st = w.get_settings()
    say(check="settings", server=c.get_server_version(), synchronous=st.synchronous_mode,
        fixed_delta_seconds=st.fixed_delta_seconds)
    if a.step:
        guard = multigpu.guard_settings({"multigpu": {"secondaries": [{}]}}, a.step, log=lambda m: say(log=m))
        changed = guard(w, force=True)
        for _ in range(int(a.ticks)):
            w.tick(10.0)
        st = w.get_settings()
        say(check="guard_settings", changed=changed, synchronous=st.synchronous_mode,
            fixed_delta_seconds=st.fixed_delta_seconds, ticks=int(a.ticks), again=guard(w, force=True))
    if a.reset:
        st.synchronous_mode, st.fixed_delta_seconds = False, None
        w.apply_settings(st)
        say(check="reset", synchronous=False)


def cmd_station(a):
    import carla
    from coupledcarla import compat, multigpu
    plan = {"multigpu": {"secondaries": parse_plan(a.plan), "own": None, "rpc_port": a.port,
                         "streaming_port": a.port + 1, "relay": "auto", "placement": True}}
    for x in plan["multigpu"]["secondaries"]:
        if a.role in (x.get("roles") or []):
            plan["multigpu"]["own"] = x["name"]
    routing = multigpu.StationRouting(plan["multigpu"], a.primary, a.port, log=lambda m: say(log=m))
    say(check="routing", address=routing.address(), relay=sorted(routing.relay.routes) if routing.relay else None,
        own=routing.own, own_local=routing.own_local, on_primary=routing.on_primary, can_place=routing.can_place())
    c = carla.Client(*routing.address())
    c.set_timeout(30)
    w = c.get_world()
    stop = threading.Event()

    def ticker():           # the host's ticks (the primary is synchronous once a secondary is up)
        while not stop.is_set():
            try:
                if w.get_settings().synchronous_mode:
                    w.tick(5.0)
                else:
                    w.wait_for_tick(5.0)
            except RuntimeError:
                pass
            time.sleep(0.05)
    threading.Thread(target=ticker, daemon=True).start()
    bp = w.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "16")
    bp.set_attribute("image_size_y", "16")
    got = threading.Event()
    images = []

    def on_image(im):
        images.append((im.frame, im.width, im.height))
        got.set()
    spawned = []

    def spawn():
        cam = w.spawn_actor(bp, carla.Transform(carla.Location(z=50)))
        spawned.append(cam.id)
        say(event="camera_spawned", actor_id=cam.id)
        return cam

    def listen(cam):
        ok = compat.listen(cam, on_image, lambda m: say(log=m), "camera")
        say(event="camera_listen", actor_id=cam.id, streaming=ok)

    def destroy(cam):
        cam.stop()
        cam.destroy()
    t0 = time.time()
    cam = routing.place_camera(spawn, listen, destroy, got, turn=None, timeout=a.timeout)
    say(check="placed", placed=routing.placed, own=routing.own, actor_id=cam.id, cameras_spawned=spawned,
        images=len(images), first_image=images[0] if images else None, seconds=round(time.time() - t0, 1),
        relay_connections=routing.relay.counts() if routing.relay else None)
    time.sleep(1)
    stop.set()
    try:
        destroy(cam)
    except RuntimeError:
        pass
    routing.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fake", help="a stand-in secondary")
    f.add_argument("--primary", required=True)
    f.add_argument("--secondary-port", type=int, default=2002)
    f.add_argument("--stream-port", type=int, required=True)
    f.add_argument("--name", default="fake")
    f.add_argument("--images", action="store_true", help="send images to subscribers (a rendering secondary)")
    f.add_argument("--token-address", help="put this IPv4 address into the tokens (default: none)")
    s = sub.add_parser("settings", help="the primary's settings; with --step, multigpu.guard_settings")
    s.add_argument("--primary", required=True)
    s.add_argument("--port", type=int, default=2000)
    s.add_argument("--step", type=float)
    s.add_argument("--ticks", type=int, default=20)
    s.add_argument("--reset", action="store_true", help="back to asynchronous at the end")
    t = sub.add_parser("station", help="a station's routing and camera placement against the primary")
    t.add_argument("--primary", required=True)
    t.add_argument("--port", type=int, default=2000)
    t.add_argument("--role", default="pedestrian")
    t.add_argument("--plan", required=True, help="secondaries as in the welcome: name@host:streaming_port[/role+role],"
                                                 "... or a JSON list (name, host, streaming_port, roles)")
    t.add_argument("--timeout", type=float, default=4.0)
    e = sub.add_parser("experiment", help="an experiment with server.multigpu, for the dry run")
    e.add_argument("--base", required=True)
    e.add_argument("--out", required=True)
    e.add_argument("--secondary", action="append", required=True, help="name@host:rpc_port")
    a = ap.parse_args(argv)
    if a.cmd == "experiment":
        cmd_experiment(a)
    elif a.cmd == "fake":
        Fake(a.primary, a.secondary_port, a.stream_port, a.name, a.images, a.token_address).run()
    elif a.cmd == "settings":
        cmd_settings(a)
    else:
        cmd_station(a)


if __name__ == "__main__":
    main()
