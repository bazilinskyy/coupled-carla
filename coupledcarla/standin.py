"""A drivable stand-in world in seconds: the map's OpenDRIVE cut to a disc around the driver.

A baked city level (EhvCity: 70 km2) takes minutes to load, and CARLA's OpenDRIVE-only world of the whole city
builds meshes for every road of it. ``map.standin: {radius_m: 800}`` drives instead on the roads within that
distance of the first participant's spawn, generated from the map's own OpenDRIVE (the idea of BeamNG.Earth's
quick world after a teleport): the same lanes, ids and signals as the full map, so SUMO (which keeps the whole net)
and the logs line up; traffic beyond the disc stays in SUMO, outside the actors' level-of-detail radius anyway.

``crop_xodr`` keeps every road with a plan-view point within the radius, every junction one of whose incoming
roads is kept (with its connections from kept roads) and the connecting roads they name; links to anything left
out are removed, and so are controls of signals that are gone.
"""
from __future__ import annotations

import math
from pathlib import Path

from lxml import etree


def _road_points(road, step=25.0):
    """Plan-view points of a road: each geometry's start and points along it every ``step`` m (straight-line
    approximation along the geometry's start heading; enough to tell whether a road reaches a disc)."""
    pts = []
    for g in road.findall("planView/geometry"):
        x, y = float(g.get("x")), float(g.get("y"))
        hdg, length = float(g.get("hdg", 0.0)), float(g.get("length", 0.0))
        n = max(1, int(math.ceil(length / step)))
        for k in range(n + 1):
            s = length * k / n
            pts.append((x + s * math.cos(hdg), y + s * math.sin(hdg)))
    return pts


def crop_xodr(text: str, x: float, y: float, radius_m: float) -> tuple[str, dict]:
    """(OpenDRIVE text, stats) of the roads within ``radius_m`` of OpenDRIVE (x, y)."""
    root = etree.fromstring(text.encode("utf-8"))
    roads = root.findall("road")
    r2 = radius_m * radius_m
    plain = {r.get("id") for r in roads if r.get("junction", "-1") == "-1"
             and any((px - x) ** 2 + (py - y) ** 2 <= r2 for px, py in _road_points(r))}
    keep_j, conn_roads = set(), set()
    for j in root.findall("junction"):
        conns = [c for c in j.findall("connection") if c.get("incomingRoad") in plain]
        for c in j.findall("connection"):
            if c not in conns:
                j.remove(c)
        if conns:
            keep_j.add(j.get("id"))
            conn_roads |= {c.get("connectingRoad") for c in conns}
        else:
            root.remove(j)
    keep = plain | conn_roads
    n_all = len(roads)
    signals = set()
    for r in roads:
        if r.get("id") not in keep:
            root.remove(r)
            continue
        link = r.find("link")
        for el in (list(link) if link is not None else []):
            et, eid = el.get("elementType"), el.get("elementId")
            if (et == "road" and eid not in keep) or (et == "junction" and eid not in keep_j):
                link.remove(el)
        signals |= {s.get("id") for s in r.iter("signal")}
    for ctl in root.findall("controller"):
        for c in ctl.findall("control"):
            if c.get("signalId") not in signals:
                ctl.remove(c)
        if not ctl.findall("control"):
            root.remove(ctl)
    ctl_ids = {c.get("id") for c in root.findall("controller")}
    for j in root.findall("junction"):
        for c in j.findall("controller"):
            if c.get("id") not in ctl_ids:
                j.remove(c)
    out = etree.tostring(root, xml_declaration=True, encoding="UTF-8").decode("utf-8")
    return out, {"roads": len(keep), "of": n_all, "junctions": len(keep_j), "radius_m": radius_m}


def standin_xodr(mdir: Path, tc: dict, log=print) -> Path:
    """Write the stand-in OpenDRIVE of a trial (map.standin) next to the map and return its path: the map's
    map.xodr cut around the first participant's spawn (or the map's centre)."""
    from .geo import Geo
    st = tc["map"].get("standin")
    radius = float((st or {}).get("radius_m", 800.0)) if isinstance(st, dict) else 800.0
    geo = Geo(mdir / "geo.net.xml")
    sp = next((p.get("spawn") for p in tc.get("participants") or [] if p.get("spawn")), None)
    if sp and "lat" in sp:
        cx, cy = geo.to_carla(sp["lat"], sp["lon"])
    else:
        (x0, y0), (x1, y1) = geo.net.getBBoxXY()
        cx, cy = (x0 + x1) / 2.0, -(y0 + y1) / 2.0
    text, st_ = crop_xodr((mdir / "map.xodr").read_text(encoding="utf-8"), cx, -cy, radius)
    out = mdir / f"standin_{int(round(cx))}_{int(round(-cy))}_{int(radius)}.xodr"
    out.write_text(text, encoding="utf-8")
    log(f"stand-in world: {st_['roads']} of {st_['of']} roads, {st_['junctions']} junctions within "
        f"{radius:.0f} m of the start -> {out.name}")
    return out
