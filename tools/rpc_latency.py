"""How long a small CARLA call takes from this client, and whether a fire-and-forget call stalls the next one.

CARLA 1.0's rpclib has no TCP_NODELAY (carla-simulator/carla#10305): after a fire-and-forget call (apply_control,
set_transform, a debug draw) the next blocking call can wait for the peer's delayed ACK, ~40 ms. Run the same test on a
native Windows client and on the WSL client against one server to see whether one of them pays it:

    python tools/rpc_latency.py --host 127.0.0.1 --port 2000 --n 300 --label windows-native --out rpc.json

Patterns (ms per call, median / 95th percentile / max):
  blocking         world.get_settings() alone
  ff_then_block    vehicle.apply_control() (fire-and-forget) then world.get_settings()
  batch_sync       client.apply_batch_sync([ApplyVehicleControl]) (blocking, one round trip)
Only carla is needed (no coupled-carla); spawns one car at a spawn point and destroys it.
"""
import argparse
import json
import sys
import time


def stats(ms):
    s = sorted(ms)
    if not s:
        return {}
    return {"median": round(s[len(s) // 2], 3), "p95": round(s[min(len(s) - 1, int(0.95 * len(s)))], 3),
            "max": round(s[-1], 3), "n": len(s)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--label", default=sys.platform)
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(60.0)
    world = client.get_world()
    bp = world.get_blueprint_library().filter("vehicle.*")[0]
    car = None
    for sp in world.get_map().get_spawn_points()[:20]:
        car = world.try_spawn_actor(bp, sp)
        if car:
            break
    if car is None:
        raise SystemExit("no free spawn point for the test car")
    ctl = carla.VehicleControl(throttle=0.0, brake=1.0)
    res = {}
    try:
        for name in ("blocking", "ff_then_block", "batch_sync"):
            ms = []
            for _ in range(a.n):
                t = time.perf_counter()
                if name == "blocking":
                    world.get_settings()
                elif name == "ff_then_block":
                    car.apply_control(ctl)
                    world.get_settings()
                else:
                    client.apply_batch_sync([carla.command.ApplyVehicleControl(car.id, ctl)])
                ms.append(1000 * (time.perf_counter() - t))
            res[name] = stats(ms)
    finally:
        car.destroy()
    out = {"client": a.label, "python": sys.version.split()[0], "server": client.get_server_version(), **res}
    print(json.dumps(out), flush=True)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
