"""tools/realism_check.py render against a fake CARLA (no server): camera sizes, placement on the ground, the frame it
keeps after the settle ticks, the files it writes, and that it leaves the server's settings as they were."""
import argparse
import csv
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

NET = """<net version="1.20"><location netOffset="0.00,0.00" convBoundary="-800,-800,800,800"
 origBoundary="0,0,0,0" projParameter="+proj=tmerc +lat_0=51.443 +lon_0=5.4815 +k=1 +x_0=0 +y_0=0 +datum=WGS84
 +units=m +no_defs"/></net>"""


def fake_carla():
    c = types.ModuleType("carla")

    class Location:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform:
        def __init__(self, location=None, rotation=None):
            self.location, self.rotation = location or Location(), rotation or Rotation()

    class BP:
        def __init__(self, id_):
            self.id, self.attrs = id_, {}

        def has_attribute(self, k):
            return k in ("image_size_x", "image_size_y", "fov", "post_process_profile")

        def set_attribute(self, k, v):
            self.attrs[k] = v

    class Image:
        def __init__(self, frame, w, h, value):
            self.frame, self.width, self.height = frame, w, h
            self.raw_data = bytes(np.full((h, w, 4), value, np.uint8))

    class Sensor:
        def __init__(self, world, bp, tf):
            self.world, self.bp, self.tf, self.cb, self.alive = world, bp, tf, None, True
            self.w, self.h = int(bp.attrs["image_size_x"]), int(bp.attrs["image_size_y"])

        @property
        def is_listening(self):
            return self.cb is not None

        def listen(self, cb):
            self.cb = cb

        def stop(self):
            self.cb = None

        def set_transform(self, tf):
            self.tf = tf
            self.world.moved_at = self.world.frame

        def destroy(self):
            self.alive = False

    class Map:
        name = "Carla/Maps/Twins/EhvCentre/EhvCentre"

        def get_waypoint(self, loc, project_to_road=True, lane_type=None):
            return types.SimpleNamespace(transform=Transform(Location(loc.x, loc.y, 16.0)))

    class World:
        def __init__(self):
            self.frame, self.moved_at, self.sensors, self.settings = 100, 0, [], types.SimpleNamespace(
                synchronous_mode=False, fixed_delta_seconds=None)
            self.applied = []

        def get_map(self):
            return Map()

        def get_settings(self):
            return types.SimpleNamespace(**vars(self.settings))

        def apply_settings(self, s):
            self.applied.append((s.synchronous_mode, s.fixed_delta_seconds))
            self.settings = s

        def get_blueprint_library(self):
            return types.SimpleNamespace(find=BP)

        def spawn_actor(self, bp, tf):
            s = Sensor(self, bp, tf)
            self.sensors.append(s)
            return s

        def tick(self):
            self.frame += 1
            for s in self.sensors:
                if s.cb:     # RGB: the number of ticks since the last move (auto exposure); labels: tag 3
                    s.cb(Image(self.frame, s.w, s.h, 3 if "semantic" in s.bp.id else min(250, self.frame - self.moved_at)))
            return self.frame

        def get_spectator(self):
            return types.SimpleNamespace(set_transform=lambda tf: None)

        def ground_projection(self, loc, dist):
            return types.SimpleNamespace(location=Location(loc.x, loc.y, 15.5))

        def get_weather(self):
            return types.SimpleNamespace(sun_altitude_angle=30.0)

    class Client:
        def __init__(self, host, port):
            self.world = World()
            c.last = self

        def set_timeout(self, t):
            pass

        def get_world(self):
            return self.world

        def apply_batch_sync(self, cmds, sync=False):
            return []

    c.Location, c.Rotation, c.Transform, c.Client = Location, Rotation, Transform, Client
    c.LaneType = types.SimpleNamespace(Any=0)
    c.command = types.SimpleNamespace(DestroyActor=lambda i: ("destroy", i))
    return c


def test_render_with_a_fake_server(tmp_path, monkeypatch):
    pytest.importorskip("sumolib")      # Geo reads the net with sumolib
    carla = fake_carla()
    monkeypatch.setitem(sys.modules, "carla", carla)
    import realism_check as T
    from coupledcarla import realism as R
    mdir = tmp_path / "map"
    mdir.mkdir()
    (mdir / "geo.net.xml").write_text(NET)
    cfg = {"map": {"carla_level": "EhvCentre", "name": "eindhoven_twin"}, "simulation": {}}
    monkeypatch.setattr(T, "_map", lambda exp: (cfg, mdir, [51.436, 5.47, 51.45, 5.493]))
    out = tmp_path / "out"
    views = []
    for i, (lat, lon, head) in enumerate([(51.443, 5.4815, 90.0), (51.4432, 5.4817, 0.0)]):
        views.append({"id": f"v{i}", "source": "panoramax", "src_id": str(i), "kind": "pano", "lat": lat, "lon": lon,
                      "heading": head, "pitch": 0.0, "roll": 0.0, "hfov": 90.0, "width": 64, "height": 36,
                      "height_m": 2.1, "projection": "equirect", "src_heading": head, "utc": "", "local_time": "",
                      "sun_alt": "", "url": "", "url_sd": "", "page": "", "licence": "", "author": "", "camera": "",
                      "sequence": "", "accuracy_m": ""})
    R.write_views(out / "views.csv", views)
    args = argparse.Namespace(experiment="x", out=str(out), yaw_margin=12.0, pitch_margin=6.0, sd=False,
                              sources="panoramax", refresh=False, spacing=20, pano_offsets="0", max_views=0,
                              action_cams=False, size=64, host="h", port=1, delta=0.05, settle=6, first_settle=9,
                              profile="Dashcam", weather="fixed", budget_min=10, no_scenery=True, no_parked=True,
                              again=False)
    T.cmd_render(args)
    from PIL import Image
    w, h, f = R.wide_size(64, 36, 90.0, 12.0, 6.0)
    rgb = np.asarray(Image.open(out / "renders" / "v0.png"))
    assert rgb.shape == (h, w, 3)
    assert rgb[0, 0, 0] == 9                      # the frame after all nine first settle ticks, not an earlier one
    assert np.asarray(Image.open(out / "renders" / "v1.png"))[0, 0, 0] == 6
    assert np.asarray(Image.open(out / "renders" / "v1_sem.png")).max() == 3
    rows = list(csv.DictReader((out / "renders.csv").open()))
    assert [r["id"] for r in rows] == ["v0", "v1"]
    assert float(rows[0]["ground_z"]) == 15.5 and float(rows[0]["cam_z"]) == pytest.approx(17.6)
    assert abs(float(rows[0]["x"])) < 0.5 and float(rows[0]["yaw"]) == 0.0      # heading east = CARLA yaw 0
    world = carla.last.world
    assert world.applied[0] == (True, 0.05) and world.applied[-1] == (False, None)
    assert all(not s.alive for s in world.sensors) and len(world.sensors) == 2
    assert world.sensors[0].bp.attrs["post_process_profile"] == "Dashcam"
    T.cmd_render(args)                            # again: nothing left to render
    assert len(list(csv.DictReader((out / "renders.csv").open()))) == 2
