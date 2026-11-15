"""tools/station_dryrun.py and coupledcarla.labcheck: the dry run over the real control channel with a fake host
and fake stations (no CARLA), the report of a trial folder, and the thresholds."""
import importlib.util
import json
import socket
import sys
from pathlib import Path

from coupledcarla import labcheck

ROOT = Path(__file__).resolve().parent.parent
FAKE = ROOT / "tests" / "fake_coupled.py"


def _tool():
    spec = importlib.util.spec_from_file_location("station_dryrun", ROOT / "tools" / "station_dryrun.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def experiment(tmp_path, port):
    path = tmp_path / "dry.yaml"
    path.write_text(f"""name: dry
map: {{name: none, bbox: [51.43, 5.47, 51.45, 5.49]}}
coupling: {{port: {port}}}
participants:
  - {{role: pedestrian, kind: pedestrian, station: true}}
  - {{role: driver, kind: driver, station: true}}
""")
    return path


def run_tool(tmp_path, extra_station="", extra=()):
    port = free_port()
    exp = experiment(tmp_path, port)
    py = sys.executable
    return _tool().main([str(exp), "--duration", "3", "--out", str(tmp_path / "out"), "--min-rtf", "0.3",
                         "--host-cmd", f"{py} {FAKE} host {{experiment}} {{duration}} {tmp_path / 'logs'}",
                         "--station-cmd", f"{py} {FAKE} station {{role}} {port} {extra_station}", *extra])


def test_dryrun_passes_with_good_stations(tmp_path, capsys):
    code = run_tool(tmp_path)
    out = capsys.readouterr().out
    assert code == 0, out
    assert "pedestrian" in out and "driver" in out and "result: ok" in out
    trial = next((tmp_path / "logs").iterdir())
    rows = labcheck.summarise(labcheck.read_trial(trial))
    ped = next(r for r in rows if r["role"] == "pedestrian")
    assert ped["ux"]["sound"] is True and ped["rtt_ms"] == 1.0 and ped["image_share"] > 0.9
    assert ped["ready_s"] is not None and ped["ready_s"] < 1.0


def test_dryrun_fails_a_poor_station(tmp_path, capsys):
    code = run_tool(tmp_path, extra_station="--bad driver")
    out = capsys.readouterr().out
    assert code == 1, out
    assert "driver: rtt_ms = 80.0" in out
    assert "image_share" in out and "image_p95_ms" in out


def test_dryrun_missing_station_exits_2(tmp_path, capsys):
    code = run_tool(tmp_path, extra=["--roles", "pedestrian"])      # nobody starts the driver's station
    assert code == 2, capsys.readouterr().out


def test_dryrun_prints_remote_commands(tmp_path, capsys):
    port = free_port()
    exp = experiment(tmp_path, port)
    py = sys.executable
    code = _tool().main([str(exp), "--duration", "2", "--out", str(tmp_path / "out"), "--min-rtf", "0.3",
                         "--host-ip", "10.1.2.3", "--remote", "driver=lin4080",
                         "--host-cmd", f"{py} {FAKE} host {{experiment}} {{duration}} {tmp_path / 'logs'}",
                         "--station-cmd", f"{py} {FAKE} station {{role}} {port}"])
    out = capsys.readouterr().out
    assert "start on lin4080" in out
    assert "--control-host 10.1.2.3" in out and "--role driver" in out and "--host 10.1.2.3" in out
    assert code == 2          # the printed station was never started (the fake host waits 6 s)


def write_trial(d, stats, rtf=1.0, ticks=2000, wall=40.0):
    d.mkdir(parents=True)
    (d / "config.yaml").write_text("simulation: {step: 0.02}\n")
    rows = ["timestamp;frame;event;actor_id;detail", '0.000;1;trial_start;;"{""stations"": [""pedestrian""]}"',
            "0.000;1;station_joined;;pedestrian"]
    for i, st in enumerate(stats):
        rows.append(f'{5 * i:.3f};{250 * i};station_stats;;"' + json.dumps(st).replace('"', '""') + '"')
    (d / "events.csv").write_text("\n".join(rows) + "\n")
    (d / "summary.json").write_text(json.dumps({"rtf": rtf, "ticks": ticks, "wall_s": wall, "sim_s": 40.0,
                                                "stations": ["pedestrian"],
                                                "station_stats": {"pedestrian": stats[-1]}}))


def stat(fps=50.0, rtt=2.0, images=1990, span=2000, ticks_seen=1995, p95=120.0):
    return {"role": "pedestrian", "rtt_ms": rtt, "fps": fps, "image_fps": 49.0, "frames": ticks_seen,
            "ticks_seen": ticks_seen, "images": images, "span": span, "ready_s": 2.5, "camera_hz": 0,
            "host": "lab1", "latency": {"tick_to_image_ms": {"median": 90.0, "p95": p95, "n": 500}},
            "ux": {"sound": True, "audio_device": True, "displays": False, "haptics": False, "ehmi_station": False}}


def test_report_of_a_real_looking_trial(tmp_path, capsys):
    write_trial(tmp_path / "t", [stat(fps=0.0), stat(), stat()])
    assert _tool().main(["--report", str(tmp_path / "t")]) == 0
    out = capsys.readouterr().out
    assert "lab1" in out and "ok" in out


def test_report_thresholds(tmp_path, capsys):
    write_trial(tmp_path / "t", [stat(fps=0.0), stat(fps=30.0, images=1000, p95=400.0)], rtf=0.8)
    assert _tool().main(["--report", str(tmp_path / "t")]) == 1
    out = capsys.readouterr().out
    assert "fps_share" in out and "image_share" in out and "image_p95_ms" in out and "RTF 0.8" in out
    # looser limits pass
    assert _tool().main(["--report", str(tmp_path / "t"), "--min-fps-share", "0.5", "--min-image-share", "0.4",
                         "--max-image-p95-ms", "500", "--min-rtf", "0.7"]) == 0


def test_report_says_why_a_headless_server_gave_no_images(tmp_path, capsys):
    # a CARLA 1.0 -nullrhi server: the station runs (ticks, rtt) but its camera streams nothing
    st = dict(stat(), images=0, image_fps=0.0, ready_s=None, latency={}, no_render=True)
    write_trial(tmp_path / "t", [dict(st, fps=0.0), st, st])
    assert _tool().main(["--report", str(tmp_path / "t")]) == 1
    out = capsys.readouterr().out
    assert "image_share" in out and "the server cannot render" in out


def test_summarise_camera_rate_and_dropped():
    trial = {"trial": {"ticks": 1000, "wall_s": 20.0}, "step": 0.02, "missing": [],
             "stations": {"driver": {"joined": 0.0, "left": None, "stats": [
                 {"rtt_ms": 3.0, "fps": 45.0, "span": 1000, "ticks_seen": 900, "images": 250, "camera_hz": 12.5}]}}}
    row = labcheck.summarise(trial)[0]
    assert row["dropped"] == 100 and abs(row["dropped_share"] - 0.1) < 1e-9
    assert abs(row["image_share"] - 1.0) < 1e-9        # 1000 ticks of 0.02 s at 12.5 Hz: 250 images expected
    names = {c[0]: c[3] for c in labcheck.check(row)}
    assert names["dropped_share"] is False and names["image_share"] is True and names["fps_share"] is True


def test_counter_and_ux_flags():
    t = [0.0]
    c = labcheck.Counter(now=lambda: t[0])
    c.running()
    t[0] = 1.5
    for f in (10, 11, 13):
        c.frame(f)
    c.image()
    t[0] = 5.0
    assert c.window(5.0)
    st = c.stats()
    assert st["span"] == 4 and st["images"] == 1 and st["ready_s"] == 1.5 and st["fps"] == 0.6

    class Out:
        channel = None

    class UX:
        output, incar, haptics, ehmi_draw = Out(), object(), None, {}
    flags = labcheck.ux_flags({"sound": {"enabled": True}}, UX())
    assert flags == {"sound": True, "audio_device": False, "displays": True, "haptics": False, "ehmi_station": False}
    assert labcheck.ux_flags({}, None)["sound"] is False


def test_stream_bandwidth():
    assert round(labcheck.stream_mb_s("1280x720", 50)) == 184      # why remote stations need a lower rate
    assert labcheck.stream_mb_s("960x540", 35) < labcheck.LINK_MB_S
