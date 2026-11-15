"""Openly licensed recordings for the ambience, fetched at run time from Wikimedia Commons (no key, no account) and
cached in ~/.cache/coupled-carla/sounds/; nothing is committed to the repository.

Each recording is a named file on Commons; its licence is read from the Commons API at fetch time and only CC0,
public domain, CC BY and CC BY-SA are accepted (the attribution is kept next to the cached file and written into the
credits file of each recording or trial that used it). Offline, or for a file whose licence is not accepted, the
caller falls back to synthesis with one log line. Freesound would offer more, but its API needs a key, so it is not
used.

Files are decoded to 44.1 kHz mono WAV with ffmpeg when it is installed, else with pygame (SDL's dummy audio
driver, so no sound device is needed); WAV originals need neither.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
import urllib.parse
import urllib.request
from pathlib import Path

API = "https://commons.wikimedia.org/w/api.php"
ACCEPT = re.compile(r"(cc0|cc-zero|cc0 1\.0|public domain|pd|cc by(-sa)?( \d\.\d)?( [a-z]{2,3})?)", re.I)

# role -> Commons file titles, best first (vetted by spectrogram 2026-11-15: full bandwidth, no thunder, voices or
# traffic; band-limited ones such as 'Heavy rain in Glenshaw, PA' were dropped). Licences are re-read at each fetch.
RECORDINGS = {
    "birds_dawn": ["File:Réveil des oiseaux.ogg", "File:Reggelirigók.ogg"],
    "birds_day": ["File:Male Blackbird song 01.ogg", "File:Common Blackbird song (Turdus merula).ogg"],
    "rain_light": ["File:Sound of light rainfall.ogg", "File:Rain (1).ogg"],
    "rain_heavy": ["File:Rain (1).ogg", "File:Heavy Rain in Summer.ogg"],
    "bicycle_bell": ["File:Bicycle-bell-1.wav", "File:Bicycle-bell-3.wav"],
}
_last_download = [0.0]
_failed = set()


def cache_dir():
    d = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "coupled-carla" / "sounds"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _ua():
    from .realworld import UA
    return dict(UA)


def _get(url, timeout=60):
    req = urllib.request.Request(url, headers=_ua())
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def fetch_bytes(url, name, log=print, timeout=60):
    """The bytes of `url`, cached as cache_dir()/name; None offline (one log line)."""
    p = cache_dir() / name
    if p.exists() and p.stat().st_size > 0:
        return p.read_bytes()
    if url in _failed:
        return None
    try:
        data = _get(url, timeout)
    except Exception as e:  # noqa: BLE001 - offline or the service is down: the caller falls back
        _failed.add(url)
        log(f"sound: cannot fetch {url} ({type(e).__name__}: {e})")
        return None
    tmp = p.with_suffix(p.suffix + ".part")
    tmp.write_bytes(data)
    tmp.replace(p)
    return data


def licence_ok(short):
    return bool(short) and bool(ACCEPT.fullmatch(short.strip()))


def commons_info(title, log=print):
    """{title, url, page, licence, licence_url, artist, credit} of a Commons file from the API, or None."""
    params = {"action": "query", "titles": title, "prop": "imageinfo", "iiprop": "url|extmetadata|size|mime",
              "iiextmetadatafilter": "LicenseShortName|LicenseUrl|Artist|Credit|AttributionRequired",
              "format": "json"}
    try:
        data = json.loads(_get(API + "?" + urllib.parse.urlencode(params), 30))
    except Exception as e:  # noqa: BLE001
        log(f"sound: Commons API unreachable ({type(e).__name__}: {e})")
        return None
    for page in (data.get("query") or {}).get("pages", {}).values():
        ii = (page.get("imageinfo") or [None])[0]
        if not ii:
            return None
        m = ii.get("extmetadata") or {}

        def val(k):
            return re.sub(r"<[^>]+>", "", (m.get(k) or {}).get("value", "")).strip()

        return {"title": page.get("title", title), "url": ii["url"], "page": ii.get("descriptionurl", ""),
                "licence": val("LicenseShortName"), "licence_url": val("LicenseUrl"),
                "artist": re.sub(r"\s+", " ", val("Artist"))[:200], "credit": re.sub(r"\s+", " ", val("Credit"))[:200],
                "size": ii.get("size"), "mime": ii.get("mime")}
    return None


def _decode(src, dst, rate=44100):
    """Decodes a sound file to mono 16-bit WAV at `rate`: ffmpeg, else pygame with the dummy driver."""
    if shutil.which("ffmpeg"):
        r = subprocess.run(["ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-i", str(src), "-ac", "1", "-ar",
                            str(rate), "-sample_fmt", "s16", str(dst)], capture_output=True, timeout=300)
        if r.returncode == 0 and dst.exists():
            return True
    try:
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
        import numpy as np
        import pygame
        from . import audio
        if not pygame.mixer.get_init():
            pygame.mixer.init(rate, -16, 2)
        fr = pygame.mixer.get_init()[0]
        a = pygame.sndarray.array(pygame.mixer.Sound(str(src))).astype(np.float64) / 32768
        x = a.mean(axis=1) if a.ndim == 2 else a
        if fr != rate:
            x = np.interp(np.arange(0, len(x) - 1, fr / rate), np.arange(len(x)), x)
        audio.write_wav(dst, np.stack([x, x], axis=1))
        return True
    except Exception:  # noqa: BLE001
        return False


def recording(role, log=print, max_mb=40):
    """(path of a decoded 44.1 kHz WAV, attribution dict) of the first usable recording for `role`, or (None,
    None). Cached: after the first run no network is needed."""
    d = cache_dir() / "commons"
    d.mkdir(exist_ok=True)
    for title in RECORDINGS.get(role, []):
        key = hashlib.sha1(title.encode()).hexdigest()[:12]
        wav, meta = d / f"{key}.wav", d / f"{key}.json"
        if wav.exists() and meta.exists():
            info = json.loads(meta.read_text())
            if licence_ok(info.get("licence")):
                return wav, info
            continue
        if title in _failed:
            continue
        info = commons_info(title, log)
        if info is None:
            _failed.add(title)
            continue
        if not licence_ok(info["licence"]):
            log(f"sound: {title} is {info['licence']!r}, not an accepted open licence; skipped")
            _failed.add(title)
            continue
        if (info.get("size") or 0) > max_mb * 1e6:
            _failed.add(title)
            continue
        wait = 2.0 - (time.time() - _last_download[0])      # upload.wikimedia.org throttles fast clients
        if wait > 0:
            time.sleep(wait)
        _last_download[0] = time.time()
        ext = Path(urllib.parse.urlparse(info["url"]).path).suffix or ".bin"
        raw = d / f"{key}.orig{ext}"
        try:
            raw.write_bytes(_get(info["url"], 120))
        except Exception as e:  # noqa: BLE001
            log(f"sound: cannot download {title} ({type(e).__name__}: {e})")
            _failed.add(title)
            continue
        if ext.lower() == ".wav" and not shutil.which("ffmpeg"):
            raw.replace(wav)              # read as it is (audio.load_sound / soundenv resample)
            ok = True
        else:
            ok = _decode(raw, wav)
        raw.unlink(missing_ok=True)
        if not ok:
            log(f"sound: cannot decode {title} (no ffmpeg, pygame failed)")
            _failed.add(title)
            continue
        meta.write_text(json.dumps(info, indent=1, ensure_ascii=False))
        log(f"sound: {role} = {info['title']} ({info['licence']}, {info['artist'] or 'unknown author'})")
        return wav, info
    return None, None


def credit_line(info):
    """One attribution line: title, author, licence, source page."""
    return (f"{info['title'].replace('File:', '')} by {info.get('artist') or 'unknown author'}, "
            f"{info['licence']} ({info.get('licence_url') or ''}), {info.get('page') or ''}")
