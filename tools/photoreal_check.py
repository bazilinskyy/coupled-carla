"""Check the photorealistic-tiles path on a CARLA 1.0 server without a Google key (coupledcarla/photoreal.py).

Against a running server that has the Cesium for Unreal and CoupledPhotoreal plugins and the CoupledCarla content
pack (with Photoreal/<Level>.json), started WITHOUT a key (env -u GOOGLE_MAPS_API_KEY, COUPLED_CARLA_CONFIG_DIR set to
an empty folder), on this machine:

  1. blueprint   blueprint.cc_photoreal_google is in the blueprint library
  2. no key      the actor removes itself (fail soft) and the server log says why; nothing is requested
  3. mock        with url=<a local mock 3D Tiles server>: the actor stays, Cesium sends exactly ONE root request and
                 fetches the tile under it, the twin's meshes are hidden, and destroying the actor sends nothing more
  4. cap         with day_cap reached in the (test) ledger: the actor removes itself, no root request
  5. reload      a forced RefreshTileset (Cesium's reload, a second root request) is counted and, beyond run_cap,
                 stops the tiles and removes the actor

    python tools/photoreal_check.py --port 2096 --level EhvCentre --config-dir ~/photoreal/cfg --server-log LOG

Never sends anything to Google: the mock server listens on 127.0.0.1 and the plugin takes no other test URL.
"""
import argparse
import json
import math
import struct
import sys
import threading
import time
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import photoreal  # noqa: E402


def enu_to_ecef_matrix(lat, lon, h):
    """Column-major 4x4 (3D Tiles `transform`) from east-north-up at lat, lon, h to ECEF."""
    la, lo = math.radians(lat), math.radians(lon)
    o = photoreal._ecef(lat, lon, h)
    e = (-math.sin(lo), math.cos(lo), 0.0)
    n = (-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la))
    u = (math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la))
    return [*e, 0.0, *n, 0.0, *u, 0.0, *o, 1.0]


def quad_glb(size=100.0, up=10.0):
    """A glTF binary: one size x size m square `up` m above the tile's origin (glTF y-up: (e, u, -n))."""
    h = size / 2
    pos = [(-h, up, h), (h, up, h), (h, up, -h), (-h, up, -h)]
    idx = [0, 1, 2, 0, 2, 3]
    bpos = b"".join(struct.pack("<3f", *p) for p in pos)
    bidx = struct.pack("<6H", *idx) + b"\0\0"
    binc = bpos + bidx
    gltf = {"asset": {"version": "2.0", "copyright": "coupled-carla mock tile"}, "scene": 0,
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "material": 0}]}],
            "materials": [{"pbrMetallicRoughness": {"baseColorFactor": [0.8, 0.2, 0.2, 1.0]}}],
            "buffers": [{"byteLength": len(binc)}],
            "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(bpos), "target": 34962},
                            {"buffer": 0, "byteOffset": len(bpos), "byteLength": 12, "target": 34963}],
            "accessors": [{"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3",
                           "min": [-h, up, -h], "max": [h, up, h]},
                          {"bufferView": 1, "componentType": 5123, "count": 6, "type": "SCALAR"}]}
    js = json.dumps(gltf).encode()
    js += b" " * (-len(js) % 4)
    body = struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(binc), 0x004E4942) + binc
    return struct.pack("<III", 0x46546C67, 2, 12 + len(body)) + body


def mock_files(origin):
    tileset = {"asset": {"version": "1.0"}, "geometricError": 500,
               "root": {"transform": enu_to_ecef_matrix(origin["lat"], origin["lon"], origin["height"]),
                        "boundingVolume": {"box": [0, 0, 10, 500, 0, 0, 0, 500, 0, 0, 0, 50]},
                        "geometricError": 0, "refine": "REPLACE", "content": {"uri": "tile.glb?session=mock"}}}
    return {"/root.json": json.dumps(tileset).encode(), "/tile.glb": quad_glb()}


def serve(port, files):
    hits = Counter()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            hits[path] += 1
            hits["key header"] += int("x-goog-api-key" in {k.lower() for k in self.headers})
            body = files.get(path)
            if body is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json" if path.endswith(".json") else "model/gltf-binary")
            self.send_header("Cache-Control", "max-age=0" if path == "/root.json" else "private, max-age=14400")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, hits


def wait(world, seconds):
    t = time.time() + seconds
    while time.time() < t:
        try:
            world.wait_for_tick(2.0)
        except RuntimeError:
            time.sleep(0.2)


def alive(world, actor):
    try:
        return world.get_actor(actor.id) is not None and actor.is_alive
    except RuntimeError:
        return False


def log_since(path, mark):
    if not path:
        return ""
    try:
        with open(path, "rb") as f:
            f.seek(mark)
            return f.read().decode("utf-8", "replace")
    except OSError:
        return ""


def log_size(path):
    try:
        return Path(path).stat().st_size if path else 0
    except OSError:
        return 0


def main():
    import carla
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2096)
    ap.add_argument("--level", default="EhvCentre")
    ap.add_argument("--mock-port", type=int, default=8896)
    ap.add_argument("--config-dir", required=True, help="the server's COUPLED_CARLA_CONFIG_DIR (an empty test folder)")
    ap.add_argument("--server-log", help="the server's log file, to quote its CoupledPhotoreal lines")
    ap.add_argument("--pack", default=str(photoreal.PACK))
    ap.add_argument("--out", help="write the results here (JSON)")
    a = ap.parse_args()
    cfgdir = Path(a.config_dir).expanduser()
    key, _ = photoreal.read_key(cfgdir=cfgdir, env={})
    if key is not None:
        sys.exit(f"{cfgdir} holds a key: use an empty folder (this check must not reach Google)")
    res = {}
    client = carla.Client(a.host, a.port)
    client.set_timeout(600.0)
    world = client.get_world()
    if a.level not in world.get_map().name:
        print(f"loading {a.level} ...", flush=True)
        t0 = time.time()
        world = client.load_world(a.level)
        res["load_s"] = round(time.time() - t0, 1)
    client.set_timeout(30.0)
    res["map"] = world.get_map().name
    lib = world.get_blueprint_library()
    bp = next((b for b in lib.filter(photoreal.BLUEPRINT) if b.id == photoreal.BLUEPRINT), None)
    res["1_blueprint"] = bp is not None
    print("1 blueprint:", res["1_blueprint"], flush=True)
    if bp is None:
        print(json.dumps(res, indent=1))
        return 1
    s = photoreal.settings({"photoreal": "google"})

    def spawn(**extra):
        b = next(b for b in world.get_blueprint_library().filter(photoreal.BLUEPRINT) if b.id == photoreal.BLUEPRINT)
        b.set_attribute("role_name", photoreal.role_options(dict(s, **extra)))
        return world.try_spawn_actor(b, carla.Transform())

    # 2. no key: fail soft
    mark = log_size(a.server_log)
    actor = spawn()
    wait(world, 6)
    gone = not alive(world, actor)
    lines = [ln for ln in log_since(a.server_log, mark).splitlines() if "CoupledPhotoreal" in ln]
    res["2_no_key"] = {"actor_removed": gone, "log": lines[-3:]}
    print("2 no key:", res["2_no_key"], flush=True)

    # 3. mock tileset: one root request
    level = json.loads(photoreal.pack_file(a.level, a.pack).read_text())
    srv, hits = serve(a.mock_port, mock_files(level["origin"]))
    try:
        mark = log_size(a.server_log)
        actor = spawn(test_url=f"http://127.0.0.1:{a.mock_port}/root.json", day_cap=1000)
        wait(world, 25)
        on = alive(world, actor)
        roots_while_on = hits["/root.json"]
        if on:
            actor.destroy()
        wait(world, 5)
        lines = [ln for ln in log_since(a.server_log, mark).splitlines() if "CoupledPhotoreal" in ln]
        res["3_mock"] = {"actor_streaming": on, "root_requests": roots_while_on,
                         "root_requests_after_destroy": hits["/root.json"], "tile_requests": hits["/tile.glb"],
                         "key_header_sent": hits["key header"],
                         "ledger": photoreal.Usage(cfgdir / "google_tiles_usage_test.json").load().get("day_roots"),
                         "log": lines}
        print("3 mock:", json.dumps(res["3_mock"], indent=1), flush=True)

        # 4. cap reached (the test ledger counted the mock's root): refuses before any request
        mark = log_size(a.server_log)
        before = hits["/root.json"]
        actor = spawn(test_url=f"http://127.0.0.1:{a.mock_port}/root.json", day_cap=1)
        wait(world, 6)
        lines = [ln for ln in log_since(a.server_log, mark).splitlines() if "CoupledPhotoreal" in ln]
        res["4_cap"] = {"actor_removed": not alive(world, actor), "root_requests": hits["/root.json"] - before,
                        "log": lines[-2:]}
        print("4 cap:", res["4_cap"], flush=True)

        # 5. a reload (Cesium destroys and reloads the tileset): counted, and stopped beyond run_cap
        mark = log_size(a.server_log)
        before = hits["/root.json"]
        actor = spawn(test_url=f"http://127.0.0.1:{a.mock_port}/root.json", day_cap=1000, run_cap=1, test_refresh_s=5)
        wait(world, 15)
        lines = [ln for ln in log_since(a.server_log, mark).splitlines() if "CoupledPhotoreal" in ln]
        res["5_reload"] = {"actor_removed": not alive(world, actor), "root_requests": hits["/root.json"] - before,
                           "log": lines[-3:]}
        print("5 reload:", res["5_reload"], flush=True)
    finally:
        srv.shutdown()
    ok = (res["2_no_key"]["actor_removed"] and res["3_mock"]["actor_streaming"] and res["3_mock"]["root_requests"] == 1
          and res["3_mock"]["root_requests_after_destroy"] == 1 and res["3_mock"]["key_header_sent"] == 0
          and res["4_cap"]["actor_removed"] and res["4_cap"]["root_requests"] == 0
          and res["5_reload"]["actor_removed"] and res["5_reload"]["root_requests"] == 2)
    res["ok"] = bool(ok)
    text = json.dumps(res, indent=1)
    if a.out:
        Path(a.out).write_text(text)
    print("OK" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
