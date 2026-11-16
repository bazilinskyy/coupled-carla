"""coupledcarla.station in VR (--display vr, view: vr) with a stand-in carla module and the simulated headset: two eye
cameras at the seat, moved in each tick's batch with their pose in the car, images paired and submitted, the VR log
and stats; and the --display argument (no CARLA server, no OpenXR)."""
import argparse
import csv
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
        missing = {"post_process_profile"}

        def __init__(self):
            self.attrs = {}

        def set_attribute(self, k, v):
            self.attrs[k] = v

        def has_attribute(self, k):
            return k not in self.missing

    class Camera:
        n = 100

        def __init__(self, bp, tf, parent):
            Camera.n += 1
            self.id, self.bp, self.tf, self.parent, self.cb = Camera.n, bp, tf, parent, None

        def listen(self, cb):
            self.cb = cb

        def stop(self):
            pass

        def destroy(self):
            pass

    class Actor:
        id, type_id, attributes = 7, "vehicle.tesla.model3", {}

        def get_transform(self):
            return Transform(V(50.0, 20.0, 0.0), Rotation(yaw=90.0))

        def get_location(self):
            return V(50.0, 20.0)

        def get_velocity(self):
            return V(5.0, 0.0)

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
            cam = Camera(bp, tf, attach_to)
            self.spawned.append(cam)
            return cam

        def get_actors(self):
            return []

    world = World()
    m.batches, m.async_batches = [], []

    class Client:
        def __init__(self, host, port):
            pass

        def set_timeout(self, t):
            pass

        def get_world(self):
            return world

        def get_server_version(self):
            return "0.9.16"

        def apply_batch_sync(self, cmds, due=False):
            m.batches.append(list(cmds))

        def apply_batch(self, cmds):
            m.async_batches.append(list(cmds))

    m.Location, m.Vector3D, m.Rotation, m.Transform, m.Client = V, V, Rotation, Transform, Client
    m.AttachmentType = types.SimpleNamespace(Rigid=0, SpringArm=1)
    m.command = types.SimpleNamespace(ApplyTransform=lambda i, tf: ("transform", i, tf))
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
        return 60.0


@pytest.fixture
def station_mod(monkeypatch):
    carla = fake_carla()
    monkeypatch.setitem(sys.modules, "carla", carla)
    sys.modules.pop("coupledcarla.station", None)
    mod = importlib.import_module("coupledcarla.station")
    monkeypatch.setattr(mod, "ControlClient", FakeControl)
    yield mod, carla
    sys.modules.pop("coupledcarla.station", None)


def make_args(tmp_path, port, **kw):
    a = dict(host="127.0.0.1", port=port, control_host=None, control_port=2100, role="av_passenger", input="bot",
             size="640x360", fov=None, headless=True, frames_every=0.0, out=str(tmp_path), response=None,
             camera_hz=0.0, display="vr", fullscreen=False, audio_device=None, child=True, list_devices=False,
             vr_runtime="sim")
    a.update(kw)
    return Namespace(**a)


def test_display_argument(station_mod):
    mod, _carla = station_mod
    assert mod.display_arg("vr") == "vr" and mod.display_arg("VR") == "vr" and mod.display_arg("2") == 2
    with pytest.raises(argparse.ArgumentTypeError):
        mod.display_arg("left")
    assert mod.screen_index("vr") == 0 and mod.screen_index(1) == 1


def test_vr_station_moves_both_eyes_and_submits(station_mod, tmp_path):
    pytest.importorskip("pygame")          # the station draws its overlay with pygame (not installed in CI)
    mod, carla = station_mod
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)
    try:
        st = mod.Station(make_args(tmp_path, srv.getsockname()[1]))
        welcome = {"kind": "av_passenger", "bot": {}, "ehmi_states": [],
                   "vr": {"size": [64, 72], "ipd_mm": 64, "sim_hz": 200}}
        st.welcome = welcome
        st._start_headset({"role": "av_passenger"})       # the first welcome can be empty (host not ready yet)
        assert st.vr_cfg["size"] is None and st.vr_cfg["runtime"] == "sim"
        st._vr_settings(welcome)                  # the full welcome later: its settings count at the trial start
        assert st.vr_cfg["size"] == [64, 72] and st.vr_cfg["runtime"] == "sim"
        st.start({"state": "running", "actor_id": 7, "trial": 0})
        left, right = st.vr.cams
        assert st.camera is left and left.parent is not None and right.parent is not None      # attached to the car
        assert left.bp.attrs["image_size_x"] == "64" and left.bp.attrs["image_size_y"] == "72"
        assert left.bp.attrs["exposure_mode"] == "manual"         # the cabin's fixed exposure, as a monitor view
        assert left.tf.location.y > 0                             # the passenger's seat (right)
        for _ in range(3):
            assert st.tick()
        # CARLA 0.9.x: the VR station's batch goes without waiting (apply_batch_sync would hold the GIL the eye
        # cameras' callbacks need)
        assert carla.batches == [] and carla.async_batches
        moves = [c for c in carla.async_batches[-1] if c[0] == "transform"]
        assert {c[1] for c in moves} == {left.id, right.id}
        lt, rt = (next(c[2] for c in moves if c[1] == cam.id) for cam in (left, right))
        # poses in the car (relative: CARLA 0.9.16), the IPD apart, at the passenger's seat
        gap = ((lt.location.x - rt.location.x) ** 2 + (lt.location.y - rt.location.y) ** 2 +
               (lt.location.z - rt.location.z) ** 2) ** 0.5
        assert gap == pytest.approx(0.064, abs=1e-6)
        assert 0.0 < lt.location.y < 1.0 and abs(lt.location.x) < 3
        # both eyes' images of the next frame: rendered where they were put (in the world: the car's pose then)
        frame = carla.world.frame
        from coupledcarla import vr
        rec = st.vr.history.get(frame + 1)
        assert st.tick()                # the station sees snapshot frame + 1 (the car's pose then) before its images
        for i, cam in enumerate((left, right)):
            x, y, z, p, yw, r = vr.to_transform(rec["world_eyes"][i])
            tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=p, yaw=yw, roll=r))
            cam.cb(types.SimpleNamespace(frame=frame + 1, raw_data=bytes(64 * 72 * 4), width=64, height=72,
                                         transform=tf))
        deadline = time.time() + 2                                # drawn and submitted by the VR worker thread
        while (st.headset.latest is None or frame + 1 not in st.vr.records) and time.time() < deadline:
            time.sleep(0.01)
        assert st.headset.latest is not None and st.headset.latest[0] == frame + 1
        assert st.count.images == 1                               # one pair = one image for the station's stats
        check = st.vr.records[frame + 1][7]
        assert check[0] < 0.01 and check[1] < 0.1                 # degrees, cm
        deadline = time.time() + 2
        while st.headset.first_shown(frame + 1) is None and time.time() < deadline:
            time.sleep(0.01)
        st.headset.shown[frame + 1] = (time.perf_counter(), rec["head"])     # shown (taken by the loop above)
        st.fps_t0 = time.time() - 6
        assert st.tick()
        stats = [m for m in st.control.sent if m.get("type") == "stats"][-1]
        assert stats["vr_size"] == "64x72" and stats["vr_runtime"] == "sim" and "vr_horizon_ms" in stats
        st.stop()
        st.headset.stop()
        rows = list(csv.DictReader(open(vr.log_path(st.log_f.name)), delimiter=";"))
        assert rows and rows[0]["frame"] == str(frame + 1) and float(rows[0]["carla_check_cm"]) < 0.1
        assert rows[0]["motion_to_photon_ms"] != ""
    finally:
        srv.close()


def test_vr_station_quits_without_a_runtime(station_mod, tmp_path, monkeypatch):
    mod, _carla = station_mod
    st = mod.Station(make_args(tmp_path, 1, vr_runtime=None))

    def missing(cfg, log=print):
        raise RuntimeError("VR needs pyopenxr")
    monkeypatch.setattr(mod.vr, "make_headset", missing)
    with pytest.raises(SystemExit):
        st._start_headset({"kind": "driver"})
    assert {"type": "bye"} in st.control.sent
