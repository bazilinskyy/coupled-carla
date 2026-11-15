"""tools/multigpu_check.py's pure parts: the compact plan syntax, the multi-GPU experiment it writes, and the bytes its
stand-in secondary sends (CARLA's token and sensor layouts), plus one command answered over a socket (no CARLA)."""
import importlib.util
import socket
import struct
import threading
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("multigpu_check", ROOT / "tools" / "multigpu_check.py")
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)


def test_parse_plan():
    plan = mc.parse_plan("gpu0@server:3001,ped@100.80.227.26:4001/pedestrian+driver")
    assert plan == [{"name": "gpu0", "host": "server", "streaming_port": 3001, "roles": []},
                    {"name": "ped", "host": "100.80.227.26", "streaming_port": 4001, "roles": ["pedestrian", "driver"]}]
    assert mc.parse_plan('[{"name": "a", "host": "server", "streaming_port": 1}]')[0]["name"] == "a"


def test_experiment_with_secondaries(tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text(yaml.safe_dump({"name": "x", "server": {"port": 2000}, "trials": [{"duration_s": 5}]}))
    out = tmp_path / "out.yaml"
    mc.main(["experiment", "--base", str(base), "--out", str(out), "--secondary", "gpu0@127.0.0.1:3000"])
    cfg = yaml.safe_load(out.read_text())
    assert cfg["server"] == {"port": 2000, "launch": False, "multigpu": {
        "secondaries": [{"name": "gpu0", "host": "127.0.0.1", "rpc_port": 3000, "launch": False}]}}
    from coupledcarla import config, multigpu
    loaded = config.load(out)
    assert [x.name for x in multigpu.settings(loaded["server"])["secondaries"]] == ["gpu0"]
    assert not [m for lvl, m in multigpu.problems(loaded) if lvl == "error"]


def test_wire_layouts():
    t = mc.token(7, 4001)
    assert len(t) == 24 and mc.parse_token(t) == {"stream_id": 7, "port": 4001, "protocol": 1, "address": None}
    assert mc.parse_token(mc.token(7, 4001, "10.1.2.3"))["address"] == "10.1.2.3"
    img = mc.image_message(1000, 4, 2)
    assert len(img) == 48 + 12 + 4 * 2 * 4                  # sensor header, image header, BGRA pixels
    sensor_type, frame = struct.unpack("<QQ", img[:16])
    assert sensor_type == mc.SCENE_CAPTURE and frame == 1000
    assert struct.unpack("<II", img[48:56]) == (4, 2)


def test_fake_answers_get_token_and_load_map():
    """The stand-in secondary against a stand-in primary: LOAD_MAP -> EPISODE_READY + load id, GET_TOKEN -> a token
    for its own streaming port."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    stream = socket.socket()
    stream.bind(("127.0.0.1", 0))
    sport = stream.getsockname()[1]
    stream.close()
    fake = mc.Fake("127.0.0.1", srv.getsockname()[1], sport, "t")
    threading.Thread(target=fake.run, daemon=True).start()
    conn, _ = srv.accept()

    def command(cid, data):
        payload = struct.pack("<II", cid, len(data)) + data
        conn.sendall(struct.pack("<I", len(payload)) + payload)

    def reply():
        n = struct.unpack("<I", mc._recv(conn, 4))[0]
        return mc._recv(conn, n)
    command(mc.LOAD_MAP, b"Town10HD_Opt\0" + struct.pack("<I", 3))
    assert reply() == b"EPISODE_READY" + struct.pack("<I", 3)
    command(mc.GET_TOKEN, struct.pack("<II", 5, 42))
    tok = mc.parse_token(reply())
    assert tok["stream_id"] == 5 and tok["port"] == sport and tok["address"] is None
    command(mc.YOU_ALIVE, b"Are you alive?")
    assert reply() == b"Yes, I'm alive"
    assert fake.counts == {"LOAD_MAP": 1, "GET_TOKEN": 1, "YOU_ALIVE": 1}
    conn.close()
    srv.close()
