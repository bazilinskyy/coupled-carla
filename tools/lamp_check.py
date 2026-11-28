"""Check that street lamps spawned by coupled-carla light up at night on a CARLA 1.0 server.

    python tools/lamp_check.py [--host 127.0.0.1] [--port 2000] [--map EhvCentreFacades] [--n 6]
        [--mode ground|lift|both] [--sun -38] [--image lamps.png] [--at X,Y] [--scenery maps/eindhoven_twin]

Needs a CARLA 1.0 server started with the content pack (-carla-packs=coupledcarla/data/carla_packs, as
coupled-carla's server.py does), so that blueprint.cc_streetlight06_a exists. It sets the sun below the horizon,
spawns N lamps (blueprint.cc_streetlight06_a) every 25 m along the kerb ahead of a spawn point, and prints for
each lamp where it was meant to stand, where the actor really is, and the light manager's view of the nearest
light (id, on/off, intensity, group, distance). With --image it also saves camera frames looking down the row of
lamps with the lamps switched off and on (exposure fixed), and prints the mean brightness of each: lamps that
really light make the "on" frame clearly brighter.

--mode lift spawns the lamps 100 m up and moves them down with ApplyTransform, as scenery.py did before the fix:
the lamp blueprints' root mesh is Static, so Unreal refuses the move and the lamps (with their lights) stay 100 m
up. --mode ground spawns them in place, as scenery.py does now. --mode both does both side by side (lifted lamps
on the left kerb, in-place lamps on the right).
--scenery MAPDIR runs scenery.py's own Scenery class instead (the map's scenery.json, lamps within 200 m of the
camera spawned as light blueprints at night) and reports its lamps the same way.

Leaves the world as it found it: the weather is restored and every actor it spawned is destroyed.
"""
import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import carla  # noqa: E402
from coupledcarla.compat import left_lane, right_lane  # noqa: E402
from coupledcarla.geo import nearest_waypoint  # noqa: E402

LAMP_BP = "blueprint.cc_streetlight06_a"
LIFT = 100.0


def tick(world, n=1):
    for _ in range(n):
        if world.get_settings().synchronous_mode:
            world.tick()
        else:
            world.wait_for_tick(10.0)


def kerb_row(cmap, start, n, spacing, side):
    """n lamp transforms every spacing m along the kerb on one side (+1 right, -1 left) ahead of start."""
    wp = cmap.get_waypoint(start.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    out = []
    for _ in range(n):
        edge = wp
        for _ in range(8):      # the outermost driving lane on that side
            nxt = right_lane(edge) if side > 0 else left_lane(edge)
            if nxt is None or nxt.lane_type != carla.LaneType.Driving or nxt.lane_id * edge.lane_id < 0:
                break
            edge = nxt
        tf = edge.transform
        r = tf.get_right_vector()
        off = side * (edge.lane_width / 2 + 1.0)
        loc = carla.Location(tf.location.x + r.x * off, tf.location.y + r.y * off, tf.location.z)
        yaw = tf.rotation.yaw + (270.0 if side > 0 else 90.0)      # the arm (+x) over the road
        out.append(carla.Transform(loc, carla.Rotation(yaw=yaw)))
        nxt = wp.next(spacing)
        if not nxt:
            break
        wp = nxt[0]
    return out


def ground(world, loc):
    """The ground under loc (the level's collision), else loc's own height."""
    try:
        hit = world.ground_projection(carla.Location(loc.x, loc.y, loc.z + 10.0), 50.0)
        if hit is not None:
            return hit.location.z
    except Exception:  # noqa: BLE001 - older clients have no ground_projection
        pass
    return loc.z


def spawn(client, world, bp, tfs, lift):
    """Spawn lamps as scenery.py does: in place, or (lift) 100 m up and then moved down with ApplyTransform."""
    cmds = []
    for tf in tfs:
        at = carla.Transform(carla.Location(tf.location.x, tf.location.y, tf.location.z + (LIFT if lift else 0.0)),
                             tf.rotation)
        cmds.append(carla.command.SpawnActor(bp, at))
    res = client.apply_batch_sync(cmds, False)
    for r in res:
        if r.error:
            print("  spawn refused:", r.error)
    if lift:
        client.apply_batch_sync([carla.command.ApplyTransform(r.actor_id, tf)
                                 for r, tf in zip(res, tfs) if not r.error], False)
    return [(tf, r.actor_id) for r, tf in zip(res, tfs) if not r.error]


def report(world, lamps, label):
    """Per lamp: meant location, the actor's real location, and the nearest light of the light manager."""
    lights = world.get_lightmanager().get_all_lights()
    print(f"--- {label}: {len(lamps)} lamps, the light manager has {len(lights)} lights")
    ok, up = 0, 0
    for k, (tf, aid) in enumerate(lamps):
        a = world.get_actor(aid)
        real = a.get_transform().location if a else None
        near = min(lights, key=lambda li: li.location.distance(tf.location), default=None)
        moved = real is not None and real.distance(tf.location) < 1.0
        lit = near is not None and near.location.distance(tf.location) < 15.0 and near.is_on
        ok += moved and lit
        up += real is not None and real.z - tf.location.z > 50.0
        if k >= 8:
            continue
        print(f"  actor {aid}: meant ({tf.location.x:.1f}, {tf.location.y:.1f}, {tf.location.z:.1f}) "
              + (f"is ({real.x:.1f}, {real.y:.1f}, {real.z:.1f})" if real else "gone")
              + (f"; nearest light id {near.id} {near.location.distance(tf.location):.1f} m away at "
                 f"z {near.location.z:.1f}: on {near.is_on}, intensity {near.intensity:.0f}, "
                 f"group {near.light_group}, colour ({near.color.r},{near.color.g},{near.color.b})"
                 if near else "; no lights"))
    if len(lamps) > 8:
        print(f"  ... ({len(lamps) - 8} more)")
    print(f"  {ok} of {len(lamps)} lamps stand where meant with a lit light within 15 m; {up} hang 50+ m up")
    return ok


def camera(world, at, out, lm, exposure):
    """Frames down the row of lamps with the street lights off and on; prints the mean brightness of each."""
    lib = world.get_blueprint_library()
    bp = lib.find("sensor.camera.rgb")
    attrs = [("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "90"), ("motion_blur_intensity", "0")]
    if exposure == "manual":    # EV100 about 1 (night street): f/1.4, 1/30 s, ISO 3200
        attrs += [("exposure_mode", "manual"), ("fstop", "1.4"), ("shutter_speed", "30"), ("iso", "3200")]
    else:
        attrs += [("exposure_mode", "histogram")]
    for k, v in attrs:
        if bp.has_attribute(k):
            bp.set_attribute(k, v)
    fwd = at.get_forward_vector()
    tf = carla.Transform(carla.Location(at.location.x - fwd.x * 8, at.location.y - fwd.y * 8, at.location.z + 1.8),
                         carla.Rotation(pitch=-4.0, yaw=at.rotation.yaw))
    cam = world.spawn_actor(bp, tf)
    frames = []
    res = {}
    try:
        try:
            cam.listen(frames.append)
        except RuntimeError as err:     # a -nullrhi server has no camera
            print(f"  no camera: {err}")
            return None
        ours = lm.get_all_lights(carla.LightGroup.Street)
        for state in ("off", "on"):
            (lm.turn_on if state == "on" else lm.turn_off)(ours)
            tick(world, 20)             # let the exposure settle
            frames.clear()
            t0 = time.time()
            while not frames and time.time() - t0 < 10:
                tick(world)
            if not frames:
                print("  no camera frame")
                return None
            img = frames[-1]
            path = str(Path(out).with_name(Path(out).stem + f"_{state}" + Path(out).suffix))
            img.save_to_disk(path)
            raw = bytes(img.raw_data)       # BGRA
            lum = [0.114 * raw[i] + 0.587 * raw[i + 1] + 0.299 * raw[i + 2] for i in range(0, len(raw), 4 * 7)]
            low = lum[len(lum) // 2:]       # the lower half: road and pavement
            res[state] = (sum(lum) / len(lum), sum(low) / len(low))
            print(f"  street lights {state}: mean brightness {res[state][0]:.1f} (lower half {res[state][1]:.1f}) "
                  f"-> {path}")
        lm.turn_on(ours)
        print(f"  lower half on/off: x{res['on'][1] / max(res['off'][1], 0.1):.1f} "
              "(about x1: the lamps light nothing in view)")
    finally:
        try:
            cam.stop()
        except RuntimeError:
            pass
        cam.destroy()
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default=None, help="load this map first (default: the one loaded)")
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--spacing", type=float, default=25.0)
    ap.add_argument("--mode", default="ground", choices=["ground", "lift", "both"])
    ap.add_argument("--sun", type=float, default=-38.0, help="sun altitude (degrees)")
    ap.add_argument("--at", default=None, help="X,Y in CARLA coordinates (default: the map's first spawn point)")
    ap.add_argument("--spawn-index", type=int, default=0)
    ap.add_argument("--image", default=None, help="save camera frames to NAME_off.png and NAME_on.png")
    ap.add_argument("--exposure", default="manual", choices=["manual", "histogram"],
                    help="camera exposure: fixed (default, makes off/on comparable) or CARLA's auto exposure")
    ap.add_argument("--scenery", default=None, help="a map folder with scenery.json: use scenery.py's Scenery")
    ap.add_argument("--keep", action="store_true", help="leave the lamps (and night) in place")
    a = ap.parse_args()

    client = carla.Client(a.host, a.port)
    client.set_timeout(120.0)
    world = client.load_world(a.map) if a.map else client.get_world()
    cmap = world.get_map()
    print("server", client.get_server_version(), "map", cmap.name)
    old_weather = world.get_weather()
    w = world.get_weather()
    w.sun_altitude_angle = a.sun
    world.set_weather(w)
    tick(world, 3)
    lm = world.get_lightmanager()
    before = lm.get_all_lights(carla.LightGroup.Street)
    print(f"night (sun {a.sun}): the level has {len(before)} street lights of its own, "
          f"{sum(1 for li in before if li.is_on)} on")

    if a.at:
        x, y = (float(v) for v in a.at.split(","))
        wp = nearest_waypoint(cmap, x, y)
        start = wp.transform
    else:
        start = cmap.get_spawn_points()[a.spawn_index]
    lib = world.get_blueprint_library()
    if not lib.filter(LAMP_BP):
        sys.exit(f"{LAMP_BP} is not in the blueprint library: start the server with "
                 f"-carla-packs={ROOT / 'coupledcarla' / 'data' / 'carla_packs'}")
    bp = lib.find(LAMP_BP)
    spawned, scen = [], None
    try:
        if a.scenery:
            from coupledcarla import season, scenery
            sc = {"radius_m": 250.0}
            seas = {"state": "leaf", "lights": season.light_blueprints(world), "lights_radius_m": 200.0}
            scen = scenery.Scenery(client, world, a.scenery, sc, log=print, season=seas)
            scen.update([start.location])
            tick(world, 3)
            lamps = []
            for key, ids in scen.lamps.items():
                if key not in scen.lit:
                    continue        # lamps beyond lights_radius_m: plain meshes
                items = [it for it in scen.cells[key] if it[6] == "lamp"]
                for aid in ids:
                    act = world.get_actor(aid)
                    if not act:
                        continue
                    tf = act.get_transform()    # where scenery.json meant it: the lamp item at the actor's x, y
                    it = min(items, key=lambda i: math.hypot(i[1] - tf.location.x, i[2] - tf.location.y))
                    lamps.append((carla.Transform(carla.Location(it[1], it[2], it[3]), tf.rotation), aid))
            print(f"Scenery: {scen.n} actors, {scen.n_lights} lit lamps in {len(scen.lit)} cells, "
                  f"{scen.n_lamp_specs} given their register colour and lumens (nightlights.py)")
            report(world, lamps, "scenery.py lamps")
        else:
            rows = []
            if a.mode in ("lift", "both"):
                rows.append(("lifted 100 m and moved down (old scenery.py)", True, -1 if a.mode == "both" else 1))
            if a.mode in ("ground", "both"):
                rows.append(("spawned in place (fixed scenery.py)", False, 1))
            for label, lift, side in rows:
                tfs = kerb_row(cmap, start, a.n, a.spacing, side)
                for tf in tfs:
                    tf.location.z = ground(world, tf.location)
                lamps = spawn(client, world, bp, tfs, lift)
                spawned += [aid for _, aid in lamps]
                tick(world, 3)
                lm.turn_on(lm.get_all_lights(carla.LightGroup.Street))
                tick(world, 2)
                report(world, lamps, label)
        if a.image:
            camera(world, start, a.image, lm, a.exposure)
    finally:
        if not a.keep:
            if scen:
                scen.destroy()
            if spawned:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in spawned], False)
            world.set_weather(old_weather)
            tick(world)


if __name__ == "__main__":
    main()
