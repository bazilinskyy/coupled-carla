"""The project page generator (site/build.py): gallery from clips.yaml, only clips with an mp4, no placeholders."""
import importlib.util
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("site_build", ROOT / "site" / "build.py")
site_build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(site_build)


def test_build_shows_the_clips_that_have_an_mp4(tmp_path):
    names = [c["name"] for c in site_build.clips()]
    assert "eindhoven_rush_hour" in names and any(n.startswith("town") for n in names)
    videos = tmp_path / "videos"
    videos.mkdir()
    for n in ["eindhoven_rush_hour", "town10_rush_hour"]:
        (videos / f"{n}.mp4").write_bytes(b"\0" * 1000)
    out = tmp_path / "site"
    missing = site_build.build(videos, out)
    page = (out / "index.html").read_text(encoding="utf-8")
    assert not re.search(r"\{\{\w+\}\}", page)
    assert page.count('class="clip"') == 2
    assert 'data-src="videos/eindhoven_rush_hour.mp4"' in page and "Rush hour in Eindhoven" in page
    assert set(missing) == set(names) - {"eindhoven_rush_hour", "town10_rush_hour"}
    assert (out / "videos" / "eindhoven_rush_hour.mp4").exists() and (out / "logo.svg").exists()
    # every local file the page refers to exists
    for ref in set(re.findall(r'(?:src|href|poster|data-src|data-poster)="([^":#?]+)"', page)):
        assert (out / ref).exists(), ref


def test_build_hides_the_towns_block_without_town_clips(tmp_path):
    videos = tmp_path / "videos"
    videos.mkdir()
    (videos / "tokyo_shibuya.mp4").write_bytes(b"\0" * 10)
    out = tmp_path / "site"
    site_build.build(videos, out)
    page = (out / "index.html").read_text(encoding="utf-8")
    assert '<div class="towns-block" hidden>' in page
    assert site_build.sentence("the AV slows") == "The AV slows"
