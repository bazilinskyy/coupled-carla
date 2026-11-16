"""Free-flow speeds from TomTom's Traffic Flow Segment Data where no open speed counts exist (speeds.py source "tomtom").

speeds.py fits the drivers' speed factor (desired speed / limit) to open spot speeds (Portland, Montevideo, Taipei).
Elsewhere it used SUMO's default N(1.0, 0.1). TomTom's flowSegmentData answers, for a point, the road segment's
free-flow speed (its typical speed when nobody holds the traffic up, from TomTom's floating-car data) and the
current speed. Tried as the desired speed (Lyon centre, 40 points, 2026-11-16): free-flow speed / limit median 0.73,
IQR 0.57-1.13, with 9 km/h on some 30 km/h streets: like NDW's, TomTom's segment speeds include the waits at the
segment's junctions, so they are a check of the simulated speeds (tools/speeds_check.py), not a factor
(speeds.SPOT leaves them out). A map gets SAMPLES points on the middles
of its longer car edges of every road class (counts.net_edges), one request each, with the segment's direction so
counts.match puts it on the right edge.

Key: settings.get_secret("tomtom_api_key") (env TOMTOM_API_KEY, the project's `secret` file); without one this source
is skipped. Requests: SAMPLES per map, cached for speeds.MAX_AGE_DAYS in maps/<name>/real/speeds.json (well inside
TomTom's free tier of 2,500 requests a day; only the derived speeds are kept, never map tiles). The key goes only to
api.tomtom.com and is never logged. Not used in the Netherlands (NDW covers it) or where an open source does.
"""
import json
import math
import time
import urllib.parse
import urllib.request
from pathlib import Path

URL = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
SAMPLES = 40
MIN_EDGE_M = 60.0
PAUSE_S = 0.25           # between requests (TomTom allows 5 per second on the free tier)


def api_key():
    try:
        from . import settings
        return settings.get_secret("tomtom_api_key")
    except Exception:  # noqa: BLE001 - an older checkout without settings.py: the environment only
        import os
        return os.environ.get("TOMTOM_API_KEY") or None


def sample_points(map_dir, n=None):
    """[(lat, lon, compass heading, road name, class)] at the middles of up to n car edges, spread over the classes."""
    from . import counts
    n = n or SAMPLES
    net, edges = counts.net_edges(map_dir)
    by_cls = {}
    for e in edges:
        if e["length"] >= MIN_EDGE_M:
            by_cls.setdefault(e["cls"], []).append(e)
    picked = []
    while len(picked) < n and any(by_cls.values()):
        for cls in sorted(by_cls):
            if by_cls[cls] and len(picked) < n:
                lst = by_cls[cls]
                picked.append(lst.pop(len(lst) // 2 if len(lst) % 2 else 0))
    out = []
    for e in picked:
        sh = e["shape"]
        half, acc = e["length"] / 2.0, 0.0
        for (x0, y0), (x1, y1) in zip(sh, sh[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= half and seg > 0:
                u = (half - acc) / seg
                x, y = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
                lon, lat = net.convertXY2LonLat(x, y)
                heading = (math.degrees(math.atan2(x1 - x0, y1 - y0)) + 360.0) % 360.0
                out.append((lat, lon, heading, e.get("name"), e["cls"]))
                break
            acc += seg
    return out


def parse(answer, lat, lon, heading, road):
    """A speeds.py spot site from one flowSegmentData answer, or None."""
    d = (answer or {}).get("flowSegmentData") or {}
    ff = d.get("freeFlowSpeed")
    if not ff or d.get("roadClosure"):
        return None
    return {"id": f"tomtom{lat:.5f},{lon:.5f}", "lat": lat, "lon": lon, "road": road, "heading": heading,
            "kind": "spot", "v50": float(ff), "v85": None, "frc": d.get("frc"),
            "current_kmh": d.get("currentSpeed"), "confidence": d.get("confidence")}


def fetch(map_dir, key=None, get=None):
    """The sites for a map (speeds.FETCHERS); raises when there is no key (speeds.load then keeps the default)."""
    key = key or api_key()
    if not key:
        raise RuntimeError("no tomtom_api_key in the secret file or TOMTOM_API_KEY")
    get = get or _get
    sites = []
    for lat, lon, heading, road, _cls in sample_points(map_dir):
        try:
            s = parse(get(lat, lon, key), lat, lon, heading, road)
        except OSError:
            s = None
        if s:
            sites.append(s)
        time.sleep(PAUSE_S)
    return sites


def _get(lat, lon, key):
    q = urllib.parse.urlencode({"point": f"{lat:.6f},{lon:.6f}", "unit": "KMPH", "key": key})
    req = urllib.request.Request(f"{URL}?{q}", headers={"User-Agent": "coupled-carla (research simulator)"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())
