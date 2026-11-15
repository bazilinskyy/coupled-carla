"""The map maker's launchers (make_map.sh, "Make a map.command", make_map.bat): which Python they pick, that extra
arguments reach the map maker, that they do not wait for Enter without a terminal, and their exit code. The shell
launchers run with a stand-in Python; the batch file (Windows) is checked as text. The page server itself:
mapmaker_web.serve answering on 127.0.0.1."""
import os
import shutil
import socket
import subprocess
import threading
import time
import urllib.request
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SHELL = ["make_map.sh", "Make a map.command"]
FAKE = """#!/bin/sh
# a stand-in Python: records its arguments, says its version
if [ "$1" = "-c" ]; then exit {old}; fi
echo "$@" > "{log}"
exit 7
"""

needs_bash = pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="needs bash")


def launcher_copy(tmp_path, name):
    d = tmp_path / "coupled carla"            # a folder with a space, as "Documents/coupled carla"
    d.mkdir()
    shutil.copy(ROOT / name, d / name)
    return d / name


def fake_python(path, log, old=0):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(FAKE.format(log=log, old=old))
    path.chmod(0o755)


@needs_bash
@pytest.mark.parametrize("name", SHELL)
def test_shell_launcher_uses_the_venv_and_passes_arguments(tmp_path, name):
    script = launcher_copy(tmp_path, name)
    log = tmp_path / "args.txt"
    fake_python(script.parent / ".venv" / "bin" / "python", log)
    p = subprocess.run(["bash", str(script), "--no-browser", "--port", "8790"], stdin=subprocess.DEVNULL,
                       capture_output=True, text=True, timeout=20, env={"PATH": "/usr/bin:/bin"})
    assert log.read_text().split() == ["-m", "coupledcarla.mapmaker", "--web", "--no-browser", "--port", "8790"]
    assert p.returncode == 7                         # the map maker's exit code, and no wait for Enter
    assert "Press Enter" not in p.stdout


@needs_bash
@pytest.mark.parametrize("name", SHELL)
def test_shell_launcher_picks_a_new_enough_python(tmp_path, name):
    script = launcher_copy(tmp_path, name)
    log = tmp_path / "args.txt"
    bindir = tmp_path / "bin"
    fake_python(bindir / "python3", tmp_path / "old.txt", old=1)       # Python 3.9: too old
    fake_python(bindir / "python3.12", log)
    p = subprocess.run(["bash", str(script)], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                       timeout=20, env={"PATH": f"{bindir}:/usr/bin:/bin"})
    assert log.read_text().split() == ["-m", "coupledcarla.mapmaker", "--web"] and p.returncode == 7
    assert not (tmp_path / "old.txt").exists()


@needs_bash
@pytest.mark.parametrize("name", SHELL)
def test_shell_launcher_without_python_says_what_to_do(tmp_path, name):
    script = launcher_copy(tmp_path, name)
    bindir = tmp_path / "bin"
    fake_python(bindir / "python3", tmp_path / "old.txt", old=1)
    for tool in ("dirname", "bash"):
        if shutil.which(tool):
            (bindir / tool).symlink_to(shutil.which(tool))
    p = subprocess.run([shutil.which("bash"), str(script)], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                       timeout=20, env={"PATH": str(bindir)})
    assert p.returncode == 3 and "Python 3.10 or newer" in p.stdout and "make_a_map.md" in p.stdout


def test_batch_launcher():
    raw = (ROOT / "make_map.bat").read_bytes()
    assert b"\r\n" in raw and b"\n" not in raw.replace(b"\r\n", b"")          # CRLF only: cmd.exe needs it
    text = raw.decode()
    assert 'cd /d "%~dp0"' in text                                             # works from any folder, with spaces
    assert ".venv\\Scripts\\python.exe" in text and "py -3" in text
    assert "-m coupledcarla.mapmaker --web %*" in text                         # extra arguments pass through
    assert "sys.version_info < (3, 10)" in text                                # an old Python is not used
    assert "if not defined MAPMAKER_NO_PAUSE pause" in text and "exit /b %CODE%" in text


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_page_server_answers_on_localhost(monkeypatch):
    from coupledcarla import mapmaker_web
    port = free_port()
    served = {}

    class Server(mapmaker_web.ThreadingHTTPServer):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            served["server"] = self
    monkeypatch.setattr(mapmaker_web, "ThreadingHTTPServer", Server)
    t = threading.Thread(target=mapmaker_web.serve, args=(port, False), daemon=True)
    t.start()
    deadline = time.time() + 10
    body = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2) as r:
                body = r.read()
                assert r.status == 200
            break
        except OSError:
            time.sleep(0.1)
    try:
        assert body and b"<html" in body.lower()
        assert served["server"].server_address[0] == "127.0.0.1"            # this computer only
    finally:
        served["server"].shutdown()
