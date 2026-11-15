import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # import coupledcarla without installing it

import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _no_sound_downloads(monkeypatch, tmp_path_factory):
    """Tests never fetch recordings or HRTFs: an empty sound cache and no network in soundfetch (tests that want a
    fetch monkeypatch soundfetch._get themselves)."""
    from coupledcarla import soundfetch
    cache = tmp_path_factory.mktemp("sounds")
    monkeypatch.setattr(soundfetch, "cache_dir", lambda: cache)

    def offline(url, timeout=60):
        raise OSError("no network in tests")

    monkeypatch.setattr(soundfetch, "_get", offline)
    soundfetch._failed.clear()
