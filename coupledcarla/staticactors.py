"""Static props out of CARLA's per-tick world state (opt-in, server.omit_static_actors).

A CARLA 1.0 server built from the cc-static-snapshot change (C:\\cc58\\carla, not upstream) and started with
-carla-omit-static-actors no longer sends static meshes that cannot move (our scenery's static.prop.mesh) to every
client on every tick: on TUeCampus with 7,971 scenery actors a synchronous world.tick() takes 1.8 ms instead of
9.2 ms, and a 60 Hz host trial with the station runs at RTF 0.93 instead of 0.54 (2026-11-21).

The client must have the same change (its wheel fetches the static actors with the "get_static_actors" call). An
unpatched client attached to such a server sees no static props in world.get_actors(), so a mismatch is an error,
never silent:
- server.omit_static_actors on: the client wheel must be patched (require_client), and the server we launch gets the
  flag (server_args);
- after the first scenery spawn, check_after_spawn looks at one static prop: a server that omits it with an
  unpatched client stops the run; a server that still sends it while the setting is on is reported (an attached
  server started without the flag: it works, just without the saving).
Off by default until the change is upstream.
"""
import importlib.util
from pathlib import Path

FLAG = "-carla-omit-static-actors"
MARKER = b"get_static_actors"       # the RPC name the patched client calls; absent from stock wheels
_cache = {}


def client_module_path():
    """The carla extension module (.so / .pyd) this interpreter imports, or None."""
    spec = importlib.util.find_spec("carla")
    if spec is None:
        return None
    if spec.origin and spec.origin.endswith((".so", ".pyd")):
        return Path(spec.origin)
    pkg = Path(spec.origin).parent if spec.origin else None        # a package: its compiled core beside it
    if pkg is not None:
        for f in sorted(pkg.glob("*.so")) + sorted(pkg.glob("*.pyd")):
            return f
    return None


def client_supports(path=None):
    """True when the carla client binary has the static-actor change (looks for the RPC name in it)."""
    path = Path(path) if path is not None else client_module_path()
    if path is None or not path.exists():
        return False
    key = (str(path), path.stat().st_mtime)
    if key not in _cache:
        found, tail = False, b""
        with open(path, "rb") as f:
            while True:
                block = f.read(1 << 20)
                if not block:
                    break
                if MARKER in tail + block:
                    found = True
                    break
                tail = block[-len(MARKER):]
        _cache[key] = found
    return _cache[key]


def unpatched_message(path, why):
    return (f"{why}, but the carla client at {path or '(not found)'} is a stock wheel without the static-actor "
            f"change: static props would be missing from world.get_actors(). Install the patched client wheel "
            f"(built from the cc-static-snapshot branch of C:\\cc58\\carla: WSL ~/cc58/wheel_optc, Windows "
            f"C:\\cc58\\carla\\Build\\PythonAPI\\dist) or set server.omit_static_actors: false and start the server "
            f"without {FLAG}.")


def require_client(sc):
    """Raise SystemExit when server.omit_static_actors is on and the client wheel lacks the change."""
    if sc.get("omit_static_actors") and not client_supports():
        raise SystemExit(unpatched_message(client_module_path(), "server.omit_static_actors is on"))


def server_args(sc):
    """The server flag for a server we launch, when the setting is on."""
    return [FLAG] if sc.get("omit_static_actors") else []


def _tick(world):
    if world.get_settings().synchronous_mode:
        world.tick()
    else:
        world.wait_for_tick(10.0)


def check_after_spawn(world, actor_ids, sc, log=print):
    """After scenery spawned static props (actor_ids): does the server leave them out of the world state, and can
    this client cope? Raises SystemExit on a server that omits them with an unpatched client; logs when the setting
    is on but the server sends them anyway. Returns True when the server omits them, False when it sends them,
    None when there was nothing to look at."""
    actor_ids = [i for i in actor_ids if i]
    if not actor_ids:
        return None
    _tick(world)
    snap = world.get_snapshot()
    probe = actor_ids[:20]
    # after a tick a stock server lists every actor; one that omits static ones lists none of the static props
    # (a lit lamp blueprint among the probes may not count as static, so any missing one decides)
    missing = [i for i in probe if snap.find(i) is None]
    if not missing:        # the server sends static props every tick (stock behaviour)
        if sc.get("omit_static_actors"):
            log(f"static actors: server.omit_static_actors is on, but the server sends static props every tick "
                f"(attached to a server started without {FLAG}?): no saving")
        return False
    if not client_supports():
        raise SystemExit(unpatched_message(client_module_path(),
                                           f"the CARLA server leaves static actors out of the world state ({FLAG})"))
    log("static actors: the server leaves static props out of the per-tick world state "
        f"({FLAG}); the client fetches them when they change")
    return True
