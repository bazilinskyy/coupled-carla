"""Checks a CARLA 1.0 package can show the scenery: every catalogue mesh spawns, and the paving renders as paving.

Two checks on a running server with rendering (not -nullrhi):

  spawn   static.prop.mesh for each mesh of the scenery catalogue (coupledcarla/data/meshes_1.0.json) and of the
          street content, far below the map, destroyed again: the meshes the server refuses (not cooked into it)
  paving  SM_CC_NL_Paving_40 and CARLA's SideWalkCube (the scenery's ground squares) each spawned 30 m up, seen
          from 8 m above by a 256 px RGB camera, and the same view without them: the mesh has to change the view
          and must not be green (win5090's 2026-11-20 package drew pedestrian squares as lawn: their ground squares
          were missing, the lawn under them showed)

    python tools/package_check.py [--host 127.0.0.1 --port 2000] [--level TUeCampusWin] [--skip redirectors.json]
        [--json out.json]

Exit 0 when both pass, 1 otherwise. Prints one line per check.
"""
import argparse
import json
import queue
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CATALOG = HERE.parent / "coupledcarla" / "data" / "meshes_1.0.json"
PAVING = {"/Game/CoupledCarla/Street/NL/SideWalk/SM_CC_NL_Paving_40.SM_CC_NL_Paving_40": 1.0,
          "/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube": None}   # None: scale to ~10 m
GREEN_MAX = 12.0        # 2G-R-B of the mean colour: lawn renders at 30-45, grey pavers at about 0
CHANGE_MIN = 6.0        # mean |difference| (0-255) the mesh has to make to the view
SIZE = 256


def green_excess(rgb):
    """2G-R-B of the mean colour of the middle 60 % of an HxWx3 uint8 RGB array."""
    import numpy as np
    h, w = rgb.shape[:2]
    c = rgb[int(h * 0.2):int(h * 0.8), int(w * 0.2):int(w * 0.8)].reshape(-1, 3).astype(float).mean(0)
    return float(2 * c[1] - c[0] - c[2])


def verdict(base, frame):
    """(ok, reason) for the view with the mesh (frame) against the view without it (base): RGB uint8 arrays."""
    import numpy as np
    change = float(np.abs(frame.astype(float) - base.astype(float)).mean())
    g = green_excess(frame)
    mean = float(frame.mean())
    if change < CHANGE_MIN:
        return False, f"the mesh did not change the view (mean difference {change:.1f})"
    if g > GREEN_MAX:
        return False, f"green (2G-R-B {g:.1f} > {GREEN_MAX})"
    if mean < 15:
        return False, f"dark (mean {mean:.0f})"
    return True, f"ok (2G-R-B {g:.1f}, change {change:.1f}, mean {mean:.0f})"


def catalogue_paths(catalog=CATALOG):
    return list(json.loads(Path(catalog).read_text())["meshes"])


def spawn_check(client, world, paths, batch=200):
    """The mesh paths the server refuses as static.prop.mesh (spawned 2 km below the map, then destroyed)."""
    import carla
    bp = world.get_blueprint_library().find("static.prop.mesh")
    refused = []
    for k in range(0, len(paths), batch):
        part = paths[k:k + batch]
        cmds = []
        for i, p in enumerate(part):
            bp.set_attribute("mesh_path", p)
            cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(x=40.0 * i, y=0, z=-2000))))
        res = client.apply_batch_sync(cmds, False)
        refused += [p for p, r in zip(part, res) if r.error]
        client.apply_batch_sync([carla.command.DestroyActor(r.actor_id) for r in res if not r.error], False)
    return refused


def _rgb(image):
    import numpy as np
    a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    return a[:, :, 2::-1].copy()        # BGRA -> RGB


def _shot(world, q, ticks=8):
    img = None
    for _ in range(ticks):              # a few frames for exposure and streaming to settle
        world.tick()
        img = q.get(timeout=30)
    return _rgb(img)


def paving_check(client, world, meshes=PAVING):
    """{mesh path: (ok, reason)} for each paving mesh, rendered from above."""
    import carla
    lib = world.get_blueprint_library()
    spot = world.get_map().get_spawn_points()[0].location
    above = carla.Location(x=spot.x, y=spot.y, z=spot.z + 30.0)
    cam_bp = lib.find("sensor.camera.rgb")
    for k, v in (("image_size_x", SIZE), ("image_size_y", SIZE), ("fov", 60)):
        cam_bp.set_attribute(k, str(v))
    cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=above.x, y=above.y, z=above.z + 8.0),
                                                    carla.Rotation(pitch=-90.0)))
    q = queue.Queue()
    cam.listen(q.put)
    out = {}
    try:
        base = _shot(world, q)
        mesh_bp = lib.find("static.prop.mesh")
        for path, scale in meshes.items():
            mesh_bp.set_attribute("mesh_path", path)
            if scale is None:       # CARLA's 1 m-ish cube: scaled to cover the view (the scenery stretches it too)
                scale = 10.0
            mesh_bp.set_attribute("scale", str(scale))
            res = client.apply_batch_sync([carla.command.SpawnActor(mesh_bp, carla.Transform(above))], False)[0]
            if res.error:
                out[path] = (False, f"refused by the server ({res.error})")
                continue
            try:
                out[path] = verdict(base, _shot(world, q))
            finally:
                client.apply_batch_sync([carla.command.DestroyActor(res.actor_id)], False)
                _shot(world, q, ticks=2)
    finally:
        cam.stop()
        cam.destroy()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--level", default="", help="load this level first (default: the current one)")
    ap.add_argument("--catalog", default=str(CATALOG))
    ap.add_argument("--skip", default="", help="JSON list of mesh paths to leave out of the spawn check (the "
                    "catalogue's redirectors, from cook_catalog_dirs.py --redirectors-out)")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    import carla
    client = carla.Client(a.host, a.port)
    client.set_timeout(600.0)
    world = client.load_world(a.level) if a.level else client.get_world()
    st = world.get_settings()
    old = (st.synchronous_mode, st.fixed_delta_seconds)
    st.synchronous_mode, st.fixed_delta_seconds = True, 0.05
    world.apply_settings(st)
    t = time.time()
    try:
        w = world.get_weather()     # midday sun: the check is about the mesh, not the light
        w.sun_altitude_angle, w.cloudiness, w.precipitation = 50.0, 10.0, 0.0
        world.set_weather(w)
        skip = set(json.loads(Path(a.skip).read_text())) if a.skip else set()
        cat = catalogue_paths(a.catalog)
        paths = [p for p in cat if p not in skip] + [p for p in PAVING if p not in cat]
        refused = spawn_check(client, world, paths)
        pav = paving_check(client, world)
    finally:
        st.synchronous_mode, st.fixed_delta_seconds = old
        world.apply_settings(st)
    ok_spawn = not refused
    ok_pav = all(v[0] for v in pav.values())
    print(f"spawn check: {len(paths) - len(refused)} of {len(paths)} meshes spawn"
          + ("" if ok_spawn else f"; REFUSED {len(refused)}, e.g. {', '.join(p.rsplit('.', 1)[-1] for p in refused[:8])}"))
    for p, (ok, why) in pav.items():
        print(f"paving check {p.rsplit('.', 1)[-1]}: {'PASS' if ok else 'FAIL'} {why}")
    print(f"package check: {'PASS' if ok_spawn and ok_pav else 'FAIL'} ({time.time() - t:.0f} s)")
    if a.json:
        Path(a.json).write_text(json.dumps({"refused": refused, "paving": {p: list(v) for p, v in pav.items()},
                                            "pass": ok_spawn and ok_pav}, indent=1))
    return 0 if ok_spawn and ok_pav else 1


if __name__ == "__main__":
    sys.exit(main())
