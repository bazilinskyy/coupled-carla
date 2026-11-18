"""What a synchronous CARLA tick costs with N static props spawned, as the scenery's meshes are (one actor each).

    python tools/tick_probe.py --host 131.155.x.x --port 2000 [--counts 0 1000 2000 4000] [--map-dir maps/<map>]

On a server that already has its level loaded (nothing is loaded here; get_map, which downloads the OpenDRIVE, is
never called). For each count: spawns that many props in a 5 m grid around --x/--y (default: the spectator), then
times --ticks ticks: world.tick() (the server's step and the snapshot it sends) and the client's work on each
snapshot as the bridge does it (the ids of every actor in it), and destroys them. The props are static.prop.mesh with
the most common mesh of the map's scenery.json (--map-dir), else --blueprint. Restores the world's settings.
Prints a table and, with --json, writes it.
"""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import carla


def pick_blueprint(world, map_dir, fallback):
    lib = world.get_blueprint_library()
    if map_dir and (Path(map_dir) / "scenery.json").exists():
        data = json.loads((Path(map_dir) / "scenery.json").read_text())
        common = Counter(it[0] for it in data["items"] if it).most_common(1)
        if common:
            bp = lib.find("static.prop.mesh")
            bp.set_attribute("mesh_path", data["meshes"][common[0][0]])
            return bp, data["meshes"][common[0][0]]
    return lib.find(fallback), fallback


def grid(n, x0, y0, z, spacing=5.0):
    side = max(1, int(n ** 0.5 + 0.999))
    out = []
    for i in range(n):
        r, c = divmod(i, side)
        out.append(carla.Transform(carla.Location(x=x0 + (c - side / 2) * spacing, y=y0 + (r - side / 2) * spacing,
                                                  z=z)))
    return out


def timed_ticks(world, n):
    world.tick()
    t_tick = t_snap = 0.0
    actors = 0
    for _ in range(n):
        t0 = time.perf_counter()
        world.tick()
        t1 = time.perf_counter()
        snap = world.get_snapshot()
        ids = {a.id for a in snap}
        t2 = time.perf_counter()
        t_tick += t1 - t0
        t_snap += t2 - t1
        actors = len(ids)
    return 1000 * t_tick / n, 1000 * t_snap / n, actors


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--counts", type=int, nargs="+", default=[0, 1000, 2000, 4000])
    ap.add_argument("--ticks", type=int, default=150)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--map-dir", default=None)
    ap.add_argument("--blueprint", default="static.prop.streetbarrier")
    ap.add_argument("--x", type=float, default=None)
    ap.add_argument("--y", type=float, default=None)
    ap.add_argument("--z", type=float, default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    client = carla.Client(a.host, a.port)
    client.set_timeout(120.0)
    world = client.get_world()
    before = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, a.step
    world.apply_settings(s)
    rows = []
    try:
        bp, what = pick_blueprint(world, a.map_dir, a.blueprint)
        here = world.get_spectator().get_transform().location
        x0 = here.x if a.x is None else a.x
        y0 = here.y if a.y is None else a.y
        z = (here.z if a.z is None else a.z) + (0.0 if a.z is not None else -50.0)   # out of sight under the spectator
        print(f"props: {what} around ({x0:.0f}, {y0:.0f}, {z:.0f}); {len(world.get_actors())} actors before")
        for n in a.counts:
            ids = []
            if n:
                places = grid(n, x0, y0, z)
                for i in range(0, n, 500):
                    cmds = [carla.command.SpawnActor(bp, tf) for tf in places[i:i + 500]]
                    ids += [r.actor_id for r in client.apply_batch_sync(cmds, True) if not r.error]
            tick_ms, snap_ms, actors = timed_ticks(world, a.ticks)
            row = {"props": len(ids), "actors": actors, "tick_ms": round(tick_ms, 2), "snapshot_ms": round(snap_ms, 2)}
            rows.append(row)
            print(f"{len(ids):6d} props  {actors:6d} actors  world.tick {tick_ms:7.2f} ms  "
                  f"snapshot ids {snap_ms:6.2f} ms")
            if ids:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], True)
    finally:
        world.apply_settings(before)
    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
