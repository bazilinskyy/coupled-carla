"""coupledcarla.station with a stand-in carla module: the trial start (camera rate, camera placement hook), the
per-tick counting and the stats it sends (no CARLA server)."""
import importlib
import socket
import sys
import time
import types
from argparse import Namespace

import pytest


def fake_carla():
    m = types.ModuleType("carla")

    class V:
        def __init__(self, x=0.0, y=0.0, z=0.0, **kw):
            self.x, self.y, self.z = x, y, z

        def __add__(self, o):
            return V(self.x + o.x, self.y + o.y, self.z + o.z)

    class Rotation:
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform:
        def __init__(self, location=None, rotation=None):
            self.location, self.rotation = location or V(), rotation or Rotation()

    class BP:
        def __init__(self):
            self.attrs = {}

        def set_attribute(self, k, v):
            self.attrs[k] = v

        missing = {"post_process_profile"}       # a CARLA 0.9.x camera; a test sets CARLA 1.0's

        def has_attribute(self, k):
            return k not in self.missing

    class Camera:
        def __init__(self, bp):
            self.bp, self.cb, self.destroyed = bp, None, False

        def listen(self, cb):
            self.cb = cb

        def stop(self):
            pass

        def destroy(self):
            self.destroyed = True

    class Actor:
        id, type_id = 7, "vehicle.tesla.model3"

        def get_transform(self):
            return Transform()

        def get_location(self):
            return V()

        def get_velocity(self):
            return V(10.0, 0.0)

    class Snap:
        def __init__(self, frame):
            self.frame = frame
            self.timestamp = types.SimpleNamespace(elapsed_seconds=0.02 * frame)

    class World:
        def __init__(self):
            self.frame, self.spawned = 100, []

        def get_actor(self, i):
            return Actor()

        def wait_for_tick(self, timeout=None):
            self.frame += 1
            return Snap(self.frame)

        def get_blueprint_library(self):
            return types.SimpleNamespace(find=lambda name: BP())

        def spawn_actor(self, bp, tf, attach_to=None, attachment_type=None):
            cam = Camera(bp)
            self.spawned.append(cam)
            return cam

        def get_actors(self):
            return []

    world = World()

    class Client:
        def __init__(self, host, port):
            m.connected_to = (host, port)

        def set_timeout(self, t):
            pass

        def get_world(self):
            return world

    m.Location, m.Vector3D, m.Rotation, m.Transform, m.Client = V, V, Rotation, Transform, Client
    m.AttachmentType = types.SimpleNamespace(Rigid=0)
    m.world = world
    return m


class FakeControl:
    def __init__(self, *a, on_message=None, **kw):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def poll(self):
        return []

    def rtt(self):
        return 2.5

    def clock_offset(self):
        return 0.0

    def lag_ms(self, frame, t):
        return 90.0


@pytest.fixture
def station_mod(monkeypatch):
    carla = fake_carla()
    monkeypatch.setitem(sys.modules, "carla", carla)
    for name in ("coupledcarla.station",):
        sys.modules.pop(name, None)
    mod = importlib.import_module("coupledcarla.station")
    monkeypatch.setattr(mod, "ControlClient", FakeControl)
    yield mod, carla
    sys.modules.pop("coupledcarla.station", None)


def args(tmp_path, port, **kw):
    a = dict(host="127.0.0.1", port=port, control_host=None, control_port=2100, role="av_passenger", input="bot",
             size="640x360", fov=90.0, headless=True, frames_every=0.0, out=str(tmp_path), response=None,
             camera_hz=0.0, display=0, fullscreen=False, audio_device=None, child=True, list_devices=False)
    a.update(kw)
    return Namespace(**a)


def test_station_trial_counts_and_stats(station_mod, tmp_path):
    mod, carla = station_mod
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)          # "the CARLA server's port" the station waits for
    try:
        st = mod.Station(args(tmp_path, srv.getsockname()[1]))
        st.welcome = {"kind": "av_passenger", "bot": {"ehmi": [[0.1, "go"]]}, "ehmi_states": ["go"]}
        st.start({"state": "running", "actor_id": 7, "trial": 0})
        cam = carla.world.spawned[-1]
        assert "sensor_tick" not in cam.bp.attrs          # --camera-hz 0: every tick, as a human's window
        assert carla.connected_to == ("127.0.0.1", srv.getsockname()[1])      # no multi-GPU: direct
        image = types.SimpleNamespace(frame=101, timestamp=2.02, raw_data=b"", width=1, height=1)
        for _ in range(3):
            cam.cb(image)
        for _ in range(10):
            assert st.tick()
        st.fps_t0 = time.time() - 6          # the 5-s stats window is over
        assert st.tick()
        stats = [m for m in st.control.sent if m.get("type") == "stats"][-1]
        assert stats["images"] == 3 and stats["ticks_seen"] == 11 and stats["span"] == 11
        assert stats["ready_s"] is not None and stats["camera_hz"] == 0.0 and stats["size"] == "640x360"
        assert stats["ux"] == {"sound": False, "audio_device": False, "displays": False, "haptics": False,
                               "ehmi_station": False}
        assert stats["camera_on"] is None and stats["rtt_ms"] == 2.5
        assert any(m.get("type") == "ehmi" and m.get("state") == "go" for m in st.control.sent)   # the bot's eHMI
    finally:
        srv.close()


def test_station_camera_rate_headless_default(station_mod, tmp_path):
    mod, carla = station_mod
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)
    try:
        st = mod.Station(args(tmp_path, srv.getsockname()[1], camera_hz=None, frames_every=0.5))
        st.welcome = {"kind": "av_passenger"}
        assert st.camera_hz() == 2.0
        st.start({"state": "running", "actor_id": 7, "trial": 0})
        attrs = carla.world.spawned[-1].bp.attrs
        assert attrs["sensor_tick"] == "0.5" and attrs["exposure_mode"] == "manual"     # as before: few frames
        st2 = mod.Station(args(tmp_path, srv.getsockname()[1], camera_hz=25.0))
        st2.welcome = {"kind": "av_passenger"}
        st2.start({"state": "running", "actor_id": 7, "trial": 0})
        attrs = carla.world.spawned[-1].bp.attrs
        assert attrs["sensor_tick"] == "0.04" and "exposure_mode" not in attrs
        # CARLA 1.0 (no exposure attributes, post-process profiles): headless renders every tick so its auto
        # exposure adapts, with the welcome's profile (compat.setup_camera)
        bp_class = type(carla.world.spawned[-1].bp)
        bp_class.missing = {"exposure_mode", "shutter_speed"}
        try:
            st3 = mod.Station(args(tmp_path, srv.getsockname()[1], camera_hz=None, frames_every=0.5))
            st3.welcome = {"kind": "av_passenger", "camera_profile": "Dashcam"}
            st3.start({"state": "running", "actor_id": 7, "trial": 0})
            attrs = carla.world.spawned[-1].bp.attrs
            assert "sensor_tick" not in attrs and attrs["post_process_profile"] == "Dashcam"
            assert st3.camera_hz() == 0.0
        finally:
            bp_class.missing = {"post_process_profile"}
    finally:
        srv.close()
