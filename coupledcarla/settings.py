"""The machine's own settings and keys, in the project's root as in the user's other projects: `config` (copied from
default.config) and `secret` (copied from default.secret), both JSON and ignored by git; the default.* templates are
committed. Every entry may stay empty; a feature that needs a key says so when it is missing.

    cp default.config config && cp default.secret secret && chmod 600 secret      # then fill in what you use
    python -m coupledcarla.settings                                                 # which keys are set (lengths only)

secret (API keys; never printed, logged, committed or sent anywhere but to their own service):
  google_maps_api_key     Google Photorealistic 3D Tiles (photoreal.py; the CoupledPhotoreal plugin in the CARLA
                          server reads $GOOGLE_MAPS_API_KEY or ~/.config/coupled-carla/google_maps_key on the server
                          machine itself)
  mapillary_token         Mapillary street photos and map features (the street-photo comparison)
  sketchfab_api_token     Sketchfab downloads of openly licensed 3D models (street furniture)
  freesound_api_key       Freesound recordings (engines, horns, rain, city ambience)
  opentopography_api_key  OpenTopography elevation (Copernicus 30 m, national lidar) outside the countries we cover
  tomtom_api_key          TomTom traffic speeds where no open counts exist
A key is looked up in its environment variable first (GOOGLE_MAPS_API_KEY, MAPILLARY_TOKEN, ...), then in `secret`,
then in the older per-user file ~/.config/coupled-carla/<name> where there was one.

config (this machine):
  mute_sound   true: nothing is played on the speakers (as COUPLED_CARLA_MUTE=1); recordings still work
  config_dir   where the Google tiles ledger, credits and the plugin's key live (default ~/.config/coupled-carla)
"""
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ENV = {"google_maps_api_key": "GOOGLE_MAPS_API_KEY", "mapillary_token": "MAPILLARY_TOKEN",
       "sketchfab_api_token": "SKETCHFAB_API_TOKEN", "freesound_api_key": "FREESOUND_API_KEY",
       "opentopography_api_key": "OPENTOPOGRAPHY_API_KEY", "tomtom_api_key": "TOMTOM_API_KEY"}
LEGACY = {"google_maps_api_key": "google_maps_key", "mapillary_token": "mapillary_token"}
HOW = {"google_maps_api_key": "console.cloud.google.com: a project with billing, the Map Tiles API, an API key",
       "mapillary_token": "mapillary.com/dashboard/developers: register an app, its client token",
       "sketchfab_api_token": "sketchfab.com: Settings, Password & API, API token",
       "freesound_api_key": "freesound.org/apiv2/apply",
       "opentopography_api_key": "portal.opentopography.org: MyOpenTopo, Request API key",
       "tomtom_api_key": "developer.tomtom.com: the dashboard's key"}
_MISSING = object()


def _json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    except ValueError as err:
        raise SystemExit(f"{path} is not valid JSON ({err}); compare it with its default.* template") from None


def _legacy_dir(env):
    if env.get("COUPLED_CARLA_CONFIG_DIR"):
        return Path(env["COUPLED_CARLA_CONFIG_DIR"])
    home = env.get("USERPROFILE") if os.name == "nt" else env.get("HOME")
    return Path(home or Path.home()) / ".config" / "coupled-carla"


def get_secret(name, root=None, env=None):
    """(value, source) of a key: its environment variable, the project's `secret`, or the older per-user file;
    (None, None) when it is set nowhere. Never print the value."""
    env = os.environ if env is None else env
    v = (env.get(ENV.get(name, "")) or "").strip() if name in ENV else ""
    if v:
        return v, "env"
    v = str(_json(Path(root or ROOT) / "secret").get(name) or "").strip()
    if v:
        return v, "secret"
    if name in LEGACY:
        try:
            for line in (_legacy_dir(env) / LEGACY[name]).read_text(encoding="utf-8").splitlines():
                if line.strip() and not line.strip().startswith("#"):
                    return line.strip(), "file"
        except OSError:
            pass
    return None, None


def secret_status(name, root=None, env=None):
    """One line about a key without the key: where it was found and its length, or how to get one."""
    v, src = get_secret(name, root, env)
    if v is None:
        return f"{name}: not set (secret, ${ENV.get(name, '?')}); get one at {HOW.get(name, '?')}"
    where = {"env": f"${ENV[name]}", "secret": "secret", "file": f"~/.config/coupled-carla/{LEGACY.get(name)}"}[src]
    msg = f"{name}: set in {where} ({len(v)} characters)"
    path = Path(root or ROOT) / "secret"
    if src == "secret" and os.name != "nt" and path.stat().st_mode & 0o077:
        msg += f"; {path} is readable by others: chmod 600 it"
    return msg


def get_config(name, default=_MISSING, root=None):
    """An entry of the project's `config`, else of default.config, else `default` (KeyError without one)."""
    root = Path(root or ROOT)
    cfg = {**_json(root / "default.config"), **_json(root / "config")}
    if name in cfg and cfg[name] not in ("", None):
        return cfg[name]
    if default is not _MISSING:
        return default
    if name in cfg:
        return cfg[name]
    raise KeyError(name)


def mute(env=None):
    """Whether sound must stay off the speakers: COUPLED_CARLA_MUTE, else config's mute_sound."""
    env = os.environ if env is None else env
    if env.get("COUPLED_CARLA_MUTE", "") not in ("", "0"):
        return True
    try:
        return bool(get_config("mute_sound", False))
    except SystemExit:
        return False


def main():
    print(f"{ROOT}: config {'present' if (ROOT / 'config').exists() else 'missing (cp default.config config)'}, "
          f"secret {'present' if (ROOT / 'secret').exists() else 'missing (cp default.secret secret)'}")
    for name in ENV:
        print("  " + secret_status(name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
