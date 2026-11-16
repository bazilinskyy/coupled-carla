"""settings.py: the project's config and secret files (the user's convention: default.* templates committed,
config and secret ignored by git), env first, the older per-user key file last; nothing prints a key."""
import json
import subprocess

from coupledcarla import settings


def test_templates_list_every_key_and_are_empty():
    root = settings.ROOT
    sec = json.loads((root / "default.secret").read_text())
    assert set(sec) == set(settings.ENV) and all(v == "" for v in sec.values())
    assert "mute_sound" in json.loads((root / "default.config").read_text())
    if (root / ".git").exists():          # a git checkout (lin4080's copy is rsynced, not git)
        ignored = subprocess.run(["git", "check-ignore", "secret", "config"], cwd=root, capture_output=True, text=True)
        assert ignored.stdout.split() == ["secret", "config"]


def test_secret_lookup_order(tmp_path):
    (tmp_path / "secret").write_text(json.dumps({"mapillary_token": "MLY|abc123", "tomtom_api_key": ""}))
    legacy = tmp_path / "home" / ".config" / "coupled-carla"
    legacy.mkdir(parents=True)
    (legacy / "google_maps_key").write_text("# comment\nAIzaTEST\n")
    env = {"HOME": str(tmp_path / "home")}
    assert settings.get_secret("mapillary_token", tmp_path, env) == ("MLY|abc123", "secret")
    assert settings.get_secret("mapillary_token", tmp_path, {**env, "MAPILLARY_TOKEN": "fromenv"}) == ("fromenv", "env")
    assert settings.get_secret("google_maps_api_key", tmp_path, env) == ("AIzaTEST", "file")
    assert settings.get_secret("tomtom_api_key", tmp_path, env) == (None, None)
    line = settings.secret_status("mapillary_token", tmp_path, env)
    assert "abc123" not in line and "10 characters" in line
    assert "developer.tomtom.com" in settings.secret_status("tomtom_api_key", tmp_path, env)


def test_config_and_mute(tmp_path, monkeypatch):
    (tmp_path / "default.config").write_text(json.dumps({"mute_sound": False, "config_dir": ""}))
    assert settings.get_config("mute_sound", root=tmp_path) is False
    (tmp_path / "config").write_text(json.dumps({"mute_sound": True}))
    assert settings.get_config("mute_sound", root=tmp_path) is True
    assert settings.get_config("nothing", 5, root=tmp_path) == 5
    assert settings.mute({"COUPLED_CARLA_MUTE": "1"})
