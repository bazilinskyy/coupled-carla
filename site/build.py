"""Builds the Coupled CARLA project page (GitHub Pages) into one folder of static files.

    python site/build.py --videos <folder with the demo mp4s> --out _site
    python -m http.server -d _site 8000          # then open http://localhost:8000/

The page is site/index.html with a few {{placeholders}} filled here: the video gallery from
experiments/videos/clips.yaml (title, place, role, what to notice, length), the version from coupledcarla/__init__.py and the
build date. The videos are never in git: the Pages workflow (.github/workflows/pages.yml) downloads them from the
`demo-videos` release into a folder and passes it as --videos. A clip without its mp4 there is left out of the
gallery (the build says which). Posters come from that folder when the release has them, else from
docs/img/videos/. Pictures the page uses (img/<name>) are copied from docs/img/, made smaller with Pillow when it is
installed. Needs PyYAML; Pillow is optional.
"""
import argparse
import datetime
import html
import re
import shutil
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPO = "https://github.com/bazilinskyy/coupled-carla"
RELEASE = f"{REPO}/releases/tag/demo-videos"
MAX_WIDTH = 1400          # pictures wider than this are scaled down (when Pillow is installed)
STATIC = ["style.css", "main.js"]


def version():
    text = (ROOT / "coupledcarla" / "__init__.py").read_text(encoding="utf-8")
    m = re.search(r'^__version__\s*=\s*"([^"]+)"', text, re.M)
    return m.group(1) if m else "dev"


def clips():
    data = yaml.safe_load((ROOT / "experiments" / "videos" / "clips.yaml").read_text(encoding="utf-8"))
    return data.get("clips", [])


def mb(n):
    return f"{n / 1e6:.1f} MB"


def sentence(text):
    """clips.yaml's notes start in lower case (they follow a dash in the README): a capital for the page."""
    text = str(text).strip()
    return text[:1].upper() + text[1:] if text else ""


def card(clip, size, poster):
    """One gallery card: a poster that plays the clip muted on hover and opens it with sound on click."""
    e = html.escape
    name = clip["name"]
    title = clip.get("title", name)
    meta = " · ".join(x for x in [clip.get("place", ""), clip.get("role", "")] if x)
    length = f"{int(round(float(clip.get('duration_s', 0))))} s" if clip.get("duration_s") else ""
    facts = " · ".join(x for x in [length, mb(size)] if x)
    return f"""      <li class="clip">
        <button class="clip-play" type="button" data-src="videos/{e(name)}.mp4" data-title="{e(title)}"
                data-poster="videos/{e(poster)}" aria-label="Play: {e(title)} ({e(facts)}, with sound)">
          <video muted loop playsinline preload="none" poster="videos/{e(poster)}" aria-hidden="true" tabindex="-1"
                 data-src="videos/{e(name)}.mp4"></video>
          <span class="clip-badge" aria-hidden="true">{e(length)}</span>
          <span class="clip-icon" aria-hidden="true"><svg viewBox="0 0 24 24" width="28" height="28"><path d="M8 5v14l11-7z" fill="currentColor"/></svg></span>
        </button>
        <div class="clip-text">
          <h3>{e(title)}</h3>
          <p class="clip-meta">{e(meta)}</p>
          <p>{e(sentence(clip.get('notice', '')))}</p>
          <p class="clip-links"><a href="videos/{e(name)}.mp4" download>Download mp4</a> <span>{e(facts)}</span></p>
        </div>
      </li>"""


HERO = ["crossing_four_views", "eindhoven_pedestrian_ehmi", "eindhoven_rush_hour"]   # the first one published
HERO_CAPTION = {"crossing_four_views": "The same moment from four stations in Eindhoven: a pedestrian, an AV passenger, "
                                       "a driver and a drone pilot (bot stations), frame-locked in simulation time."}


def hero(shown):
    """The clip at the top of the page (muted, plays while on screen), else a still picture."""
    e = html.escape
    for name in HERO:
        clip = shown.get(name)
        if clip:
            title = clip.get("title", name)
            caption = HERO_CAPTION.get(name) or f"{title}: {sentence(clip.get('notice', ''))}."
            return f"""    <figure class="hero-media">
      <button class="hero-play" type="button" data-src="videos/{name}.mp4" data-title="{e(title)}"
              data-poster="videos/{name}.jpg" aria-label="Play: {e(title)} (with sound)">
        <video id="hero-video" muted loop playsinline preload="none" poster="videos/{name}.jpg"
               data-src="videos/{name}.mp4" aria-hidden="true" tabindex="-1"></video>
        <span class="clip-icon" aria-hidden="true"><svg viewBox="0 0 24 24" width="32" height="32"><path d="M8 5v14l11-7z" fill="currentColor"/></svg></span>
      </button>
      <figcaption>{e(caption)}</figcaption>
    </figure>"""
    return """    <figure class="hero-media">
      <img src="img/stations_four_views.jpg" alt="Four stations in one trial at the same moment, east of Eindhoven station: the pedestrian, the AV passenger with the in-car display, a driver and a drone pilot">
      <figcaption>The same moment from four stations in Eindhoven: a pedestrian, an AV passenger, a driver and a
        drone pilot (bot stations).</figcaption>
    </figure>"""


def gallery(videos, out):
    """The two gallery lists (real places, CARLA's towns) and which clips were left out."""
    (out / "videos").mkdir(parents=True, exist_ok=True)
    # every poster, also of clips not published yet (the page uses some as pictures); the release's win
    for p in sorted((ROOT / "docs" / "img" / "videos").glob("*.jpg")) + (sorted(videos.glob("*.jpg")) if videos else []):
        shutil.copy2(p, out / "videos" / p.name)
    places, towns, missing, shown = [], [], [], {}
    for clip in clips():
        name = clip["name"]
        mp4 = videos / f"{name}.mp4" if videos else None
        if not mp4 or not mp4.exists():
            missing.append(name)
            continue
        shutil.copy2(mp4, out / "videos" / mp4.name)
        poster = f"{name}.jpg"
        if not (out / "videos" / poster).exists():
            print(f"  {name}: no poster (docs/img/videos/{name}.jpg); the first frame shows instead")
            poster = ""
        (towns if name.startswith("town") else places).append(card(clip, mp4.stat().st_size, poster))
        shown[name] = clip
    return "\n".join(places), "\n".join(towns), missing, shown


def copy_image(name, out):
    src = ROOT / "docs" / "img" / name
    dst = out / "img" / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
    except ImportError:
        shutil.copy2(src, dst)
        return
    with Image.open(src) as im:
        if im.width <= MAX_WIDTH:
            shutil.copy2(src, dst)
            return
        h = round(im.height * MAX_WIDTH / im.width)
        small = im.resize((MAX_WIDTH, h), Image.LANCZOS)
        if dst.suffix.lower() in (".jpg", ".jpeg"):
            small.convert("RGB").save(dst, quality=84, optimize=True, progressive=True)
        else:
            small.save(dst, optimize=True)
    if dst.stat().st_size > src.stat().st_size:
        shutil.copy2(src, dst)


def sizes(out):
    """Sets width and height of every <img> to the copied picture's own (no layout shift; needs Pillow)."""
    try:
        from PIL import Image
    except ImportError:
        return
    page = (out / "index.html").read_text(encoding="utf-8")

    def fix(m):
        tag = m.group(0)
        src = re.search(r'src="([^"]+)"', tag).group(1)
        f = out / src
        if not f.exists() or f.suffix == ".svg":
            return tag
        with Image.open(f) as im:
            w, h = im.size
        tag = re.sub(r'\s(width|height)="\d+"', "", tag)
        return tag.replace("<img ", f'<img width="{w}" height="{h}" ', 1)

    (out / "index.html").write_text(re.sub(r"<img\s[^>]*>", fix, page), encoding="utf-8")


def build(videos, out):
    out = Path(out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    page = (HERE / "index.html").read_text(encoding="utf-8")
    places, towns, missing, shown = gallery(Path(videos) if videos else None, out)
    page = (page.replace("{{hero}}", hero(shown))
                .replace("{{videos_places}}", places)
                .replace("{{videos_towns}}", towns)
                .replace("{{towns_hidden}}", "" if towns else " hidden")
                .replace("{{version}}", html.escape(version()))
                .replace("{{built}}", datetime.date.today().isoformat())
                .replace("{{release}}", RELEASE)
                .replace("{{repo}}", REPO))
    left = re.findall(r"\{\{\w+\}\}", page)
    if left:
        sys.exit(f"unfilled placeholders in site/index.html: {sorted(set(left))}")
    (out / "index.html").write_text(page, encoding="utf-8")
    for f in STATIC:
        shutil.copy2(HERE / f, out / f)
    shutil.copy2(ROOT / "docs" / "logo.svg", out / "logo.svg")
    images = sorted(set(re.findall(r'["\(]img/([\w.-]+\.(?:jpg|jpeg|png|svg|webp))', page)))
    for name in images:
        if not (ROOT / "docs" / "img" / name).exists():
            sys.exit(f"site/index.html uses img/{name}, which is not in docs/img/")
        copy_image(name, out)
    sizes(out)
    (out / ".nojekyll").write_text("", encoding="utf-8")
    shown = len(clips()) - len(missing)
    total = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    print(f"Built {out}/: {shown} videos, {len(images)} pictures, {mb(total)} in all")
    if missing:
        print(f"  left out (no mp4 in {videos or 'the --videos folder'}): {', '.join(missing)}")
    return missing


def main(argv=None):
    ap = argparse.ArgumentParser(description="Build the Coupled CARLA project page.")
    ap.add_argument("--videos", help="folder with the demo clips (<name>.mp4, optional <name>.jpg posters), "
                                     "e.g. from: gh release download demo-videos --dir videos")
    ap.add_argument("--out", default=str(ROOT / "_site"), help="output folder (emptied first)")
    ap.add_argument("--strict", action="store_true", help="fail when a clip of clips.yaml has no mp4")
    args = ap.parse_args(argv)
    missing = build(args.videos, args.out)
    return 1 if (args.strict and missing) else 0


if __name__ == "__main__":
    sys.exit(main())
