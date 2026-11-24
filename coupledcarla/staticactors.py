"""Static actors out of CARLA's per-tick world state.

Every scenery mesh is a CARLA actor. A stock server sends every actor to every client on every tick: on TUeCampus
with 7,971 scenery actors a synchronous world.tick() takes 9.2 ms instead of 1.8 ms without them (2026-11-21).
Two kinds of server leave static actors (static.prop.mesh without mass, which is our scenery) out of that state:
- CARLA ue58-dev since f5fb36aed (carla-simulator/carla#10381, merged 24 Nov 2026), always, with no flag; traffic
  signs leave it too. Its client wheel lists them with world.get_static_actors() and in world.get_actors().
- our cc-static-snapshot tree (C:\\cc58\\carla, the first version of that change), only when started with
  -carla-omit-static-actors (server.omit_static_actors, off by default). Its wheel has the "get_static_actors"
  RPC but no Python call.

Client and server must come from the same tree: the two say which static actors changed in different ways, and a
stock client sees none of them. A client that cannot list them is an error, never silent:
- server.omit_static_actors on: the client wheel must have the change (require_client), and the server we launch
  gets the flag (server_args; a server from upstream ignores it and omits static actors anyway);
- after the first scenery spawn, check_after_spawn looks at the props: a server that leaves them out of the
  snapshot while this client's world.get_actors() does not list them stops the run, whatever the setting; the
  setting on with a server that still sends them is reported (no saving).
Only world.get_snapshot() (and on_tick) changes for a client that can list them: snapshot.find() returns None for a
static actor, so code that follows a static actor reads actor.get_transform() instead.
"""
import importlib.util
from pathlib import Path

FLAG = "-carla-omit-static-actors"  # our cc-static-snapshot tree only; upstream has no flag
MARKER = b"get_static_actors"       # the RPC name both changed clients call; absent from stock wheels
UPSTREAM = "f5fb36aed"              # carla-simulator/carla ue58-dev commit that made it the default (24 Nov 2026)
CC_WHEELS = "WSL ~/cc58/wheel_optc, Windows C:\\cc58\\carla\\Build\\PythonAPI\\dist"
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


def _python_call():
    """True when the imported carla module has world.get_static_actors() (upstream since f5fb36aed)."""
    try:
        import carla
    except ImportError:
        return False
    return hasattr(getattr(carla, "World", None), "get_static_actors")


def client_kind(path=None):
    """Which client this interpreter has: "upstream" (world.get_static_actors(): carla-simulator ue58-dev since
    f5fb36aed), "cc" (the RPC in the binary but no Python call: our cc-static-snapshot tree) or "stock"."""
    if path is None and _python_call():
        return "upstream"
    return "cc" if client_supports(path) else "stock"


WHICH_WHEEL = (f"Client and server must come from the same CARLA tree: a server built from carla-simulator ue58-dev "
               f"since {UPSTREAM} (24 Nov 2026) needs that tree's client wheel (it has world.get_static_actors()); "
               f"a server from our cc-static-snapshot tree (C:\\cc58\\carla) started with {FLAG} needs that tree's "
               f"wheel ({CC_WHEELS}).")


def unpatched_message(path, why):
    return (f"{why}, but the carla client at {path or '(not found)'} is a stock wheel without the static-actor "
            f"change: static props would be missing from world.get_actors(). {WHICH_WHEEL} Or set "
            f"server.omit_static_actors: false and start the server without {FLAG} (a server from upstream "
            f"ue58-dev since {UPSTREAM} cannot be told to send them).")


def mismatch_message(path, kind):
    return (f"the CARLA server leaves static actors out of the per-tick world state, but this client does not list "
            f"them in world.get_actors() (the {kind} carla client at {path or '(not found)'}): scenery would be "
            f"invisible to provenance, toggling and the logs. {WHICH_WHEEL}")


def require_client(sc):
    """Raise SystemExit when server.omit_static_actors is on and the client wheel lacks the change (either tree)."""
    if sc.get("omit_static_actors") and not client_supports():
        raise SystemExit(unpatched_message(client_module_path(), "server.omit_static_actors is on"))


def server_args(sc):
    """The server flag for a server we launch, when the setting is on (needed by our cc-static-snapshot tree;
    a server from upstream ue58-dev since f5fb36aed ignores it and leaves static actors out anyway)."""
    return [FLAG] if sc.get("omit_static_actors") else []


def _tick(world):
    if world.get_settings().synchronous_mode:
        world.tick()
    else:
        world.wait_for_tick(10.0)


def check_after_spawn(world, actor_ids, sc, log=print):
    """After scenery spawned static props (actor_ids): does the server leave them out of the world snapshot, and
    does this client still list them? Raises SystemExit when the server leaves them out and world.get_actors() does
    not list them (a stock client, or a wheel from the other tree); logs when the setting is on but the server sends
    them anyway. Returns True when the server omits them, False when it sends them, None when there was nothing to
    look at."""
    actor_ids = [i for i in actor_ids if i]
    if not actor_ids:
        return None
    _tick(world)
    snap = world.get_snapshot()
    probe = actor_ids[:20]
    # after a tick a stock server lists every actor; one that omits static ones lists none of the static props
    # (a lit lamp blueprint or a prop with mass among the probes is not static, so any missing one decides)
    missing = [i for i in probe if snap.find(i) is None]
    if not missing:        # the server sends static props every tick (stock behaviour)
        if sc.get("omit_static_actors"):
            log(f"static actors: server.omit_static_actors is on, but the server sends static props every tick "
                f"(attached to a server started without {FLAG}?): no saving")
        return False
    kind = client_kind()
    listed = {a.id for a in world.get_actors()}        # the snapshot's actors plus the client's static table
    if kind == "stock" or any(i not in listed for i in missing):
        raise SystemExit(mismatch_message(client_module_path(), kind))
    if kind == "upstream":
        log(f"static actors: the server leaves static props and traffic signs out of the per-tick world state "
            f"(CARLA ue58-dev since {UPSTREAM}); world.get_static_actors() lists them")
    else:
        log("static actors: the server leaves static props out of the per-tick world state "
            f"({FLAG}); the client fetches them when they change")
    return True
