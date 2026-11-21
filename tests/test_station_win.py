"""The participant station on native Windows: its launcher and setup scripts, and settings (keys, mute) as Windows
sees them (USERPROFILE, no chmod)."""
import json
from pathlib import Path

from coupledcarla import settings

TOOLS = Path(__file__).resolve().parent.parent / "tools"


def test_launcher_and_setup_scripts():
    run = (TOOLS / "station_win.cmd").read_bytes()
    setup = (TOOLS / "station_win_setup.cmd").read_bytes()
    for b in (run, setup):
        assert b"\r\n" in b and b.count(b"\n") == b.count(b"\r\n")     # cmd.exe wants CRLF throughout
    run, setup = run.decode(), setup.decode()
    assert '-m coupledcarla.station --host 127.0.0.1 %*' in run         # later options win: --host can be overridden
    assert "station_venv" in run and 'cd /d "%~dp0.."' in run
    assert 'if "%STATION_VENV%"=="" set STATION_VENV=%CC58%\\station_venv' in run      # another venv on request
    assert '"%STATION_VENV%\\Scripts\\python.exe" -m coupledcarla.station' in run
    assert "cp310-win_amd64.whl" in setup and "UV_PYTHON_INSTALL_DIR=%CC58%\\uvpython" in setup
    assert "--user" not in setup and "setx" not in setup.lower()        # nothing system-wide or per-user global


class _WindowsOs:
    """settings' view of os on Windows (os.name "nt"); everything else is the real module (pathlib keeps working)."""
    name = "nt"

    def __getattr__(self, attr):
        import os
        return getattr(os, attr)


def test_keys_and_mute_on_windows(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "os", _WindowsOs())
    home = tmp_path / "home"
    (home / ".config" / "coupled-carla").mkdir(parents=True)
    (home / ".config" / "coupled-carla" / settings.LEGACY["google_maps_api_key"]).write_text("# key\nABC123\n")
    env = {"USERPROFILE": str(home)}
    root = tmp_path / "repo"
    root.mkdir()
    assert settings.get_secret("google_maps_api_key", root, env) == ("ABC123", "file")
    (root / "secret").write_text(json.dumps({"google_maps_api_key": "XYZ"}))
    (root / "secret").chmod(0o644)
    msg = settings.secret_status("google_maps_api_key", root, env)
    assert "secret" in msg and "chmod" not in msg and "XYZ" not in msg     # no POSIX permission advice on Windows
    assert settings.mute({"COUPLED_CARLA_MUTE": "1"}) is True
