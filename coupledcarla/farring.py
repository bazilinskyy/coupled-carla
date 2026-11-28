"""The far landscape ring of a generated map: the real land around it out to the horizon.

A map made from OpenDRIVE (mapmaker) ends at its scenery's ground tiles; beyond them the run used to spawn a 20 km
paving slab (a flat grey plain to the horizon). carla-digitaltwins' ``tools/far_ring.py`` builds the real landscape
instead (terrain, aerial photo, low buildings to 3 km, forest canopy, ground to 60 km); this module

* works out what that tool needs from a map folder (``ring_args``): the OpenDRIVE geoReference and header offset
  (the map's model frame), the scenery ground's extent and edge heights (scenery.json ``ground_planes``) and the
  terrain base the map's heights are relative to (meta.json ``elevation.base_m``), and runs it (``build``) into
  ``<map>/far_ring/``;
* spawns the ring's meshes at run time (``spawn``) when the server has them cooked under
  ``/Game/Carla/Static/<Semantic>/FarRing/<Map>/`` (carla-digitaltwins ``ue/bake_far_ring.py --assets-only``; a
  package needs that folder and /Game/CoupledCarla/FarRing in DirectoriesToAlwaysCook). A server without them gets
  the slab as before.

The generated world's coordinates are the OpenDRIVE's (CARLA x = x, y = -y), the frame the ring is built in, so every
mesh stands at the origin.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Optional

MESH_ROOT = "/Game/Carla/Static/{semantic}/FarRing/{name}"   # carla-digitaltwins bake_far_ring.py --assets-only


def ring_name(mdir: Path) -> str:
    """The ring's asset name of a map folder (CamelCase of the folder name: eindhoven_centre -> EindhovenCentre)."""
    return "".join(p[:1].upper() + p[1:] for p in re.split(r"[^A-Za-z0-9]+", Path(mdir).name) if p) or "Map"


def georef(xodr_text: str):
    """(geoReference string, (offset x, offset y)) of an OpenDRIVE text."""
    m = re.search(r"<geoReference>\s*(?:<!\[CDATA\[)?\s*([^<\]]*?)\s*(?:\]\]>)?\s*</geoReference>", xodr_text)
    g = " ".join(m.group(1).split()) if m else ""
    o = re.search(r'<offset\s+x="([-0-9.eE]+)"\s+y="([-0-9.eE]+)"', xodr_text[:5000])
    return g, ((float(o.group(1)), float(o.group(2))) if o else (0.0, 0.0))


def ground_edge(planes, band: float = 60.0):
    """(inner rect in model metres (y north), edge points [[x, y, z], ...]) of scenery ground tiles (CARLA x, y south,
    side T, top z at the centre, slopes zx, zy): the tiles' outline and their corner heights within ``band`` of it."""
    xs0 = [p[0] - p[2] / 2 for p in planes]; xs1 = [p[0] + p[2] / 2 for p in planes]
    ys0 = [-(p[1] + p[2] / 2) for p in planes]; ys1 = [-(p[1] - p[2] / 2) for p in planes]
    rect = (min(xs0), min(ys0), max(xs1), max(ys1))
    pts = []
    for x, y, T, z, *sl in planes:
        zx, zy = (sl + [0.0, 0.0])[:2]
        for dx in (-T / 2, 0.0, T / 2):
            for dy in (-T / 2, 0.0, T / 2):
                mx, my = x + dx, -(y + dy)
                d = min(mx - rect[0], rect[2] - mx, my - rect[1], rect[3] - my)
                if d < band:
                    pts.append([round(mx, 2), round(my, 2), round(z + zx * dx + zy * dy, 3)])
    return rect, pts


def ring_args(mdir: Path) -> Optional[dict]:
    """What tools/far_ring.py needs for a map folder, or None (no scenery ground tiles, no geoReference)."""
    mdir = Path(mdir)
    try:
        planes = json.loads((mdir / "scenery.json").read_text()).get("ground_planes")
        text = (mdir / "map.xodr").read_text(encoding="utf-8", errors="replace")
    except (OSError, ValueError):
        return None
    g, off = georef(text)
    if not planes or not g:
        return None
    rect, pts = ground_edge(planes)
    meta = {}
    try:
        meta = json.loads((mdir / "meta.json").read_text())
    except (OSError, ValueError):
        pass
    base = float(((meta.get("elevation") or {}).get("base_m")) or 0.0)
    country = (meta.get("speed_defaults") or {}).get("country") or meta.get("country")
    return {"name": ring_name(mdir), "georef": g, "offset": off, "inner": rect, "seam_points": pts,
            "z_offset": -base, "country": country}


def build(mdir: Path, cdt: Path, python: str = sys.executable, extra=(), log=print, stdout=None) -> int:
    """Run carla-digitaltwins' tools/far_ring.py (``cdt`` = its TwinModel folder) for a map folder."""
    a = ring_args(mdir)
    if a is None:
        log(f"far ring: {mdir} has no scenery ground tiles or no geoReference: skipped")
        return 1
    out = Path(mdir) / "far_ring"
    out.mkdir(exist_ok=True)
    seam = out / "seam_points.json"
    seam.write_text(json.dumps(a["seam_points"]))
    cmd = [python, str(Path(cdt) / "tools" / "far_ring.py"), "--name", a["name"], "--georef", a["georef"],
           "--offset", str(a["offset"][0]), str(a["offset"][1]), "--inner", *[str(v) for v in a["inner"]],
           "--seam-points", str(seam), "--z-offset", str(a["z_offset"]), "--out", str(out),
           "--cache", str(Path(mdir) / "far_ring_cache")] + (["--country", a["country"]] if a["country"] else [])
    cmd += list(extra)
    log("far ring: " + " ".join(cmd[:3]) + " ...")
    return subprocess.call(cmd, stdout=stdout, stderr=subprocess.STDOUT if stdout is not None else None)


def spawn(world, mdir: Path, carla, log=print) -> int:
    """Spawn the map's ring meshes (static.prop.mesh at the origin) when <map>/far_ring/far_ring.json exists and the
    server has them cooked; returns how many were spawned (0: none, the caller lays its slab)."""
    p = Path(mdir) / "far_ring" / "far_ring.json"
    if not p.exists():
        return 0
    try:
        ring = json.loads(p.read_text())
    except ValueError:
        return 0
    name = ring.get("name") or ring_name(mdir)
    root = MESH_ROOT.format(semantic="<Semantic>", name=name)
    bp = world.get_blueprint_library().find("static.prop.mesh")
    spawned = []
    for a in ring.get("assets", []):
        folder = MESH_ROOT.format(semantic=a.get("semantic", "Terrain"), name=name)
        bp.set_attribute("mesh_path", f"{folder}/{a['asset']}.{a['asset']}")
        try:
            bp.set_attribute("scale", "1")
        except (IndexError, RuntimeError):
            pass
        actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(0, 0, 0)))
        if actor is None:
            if not spawned:
                log(f"far ring: {root} is not on this server (cook it into the package): the slab instead")
                return 0
            continue
        spawned.append(actor)
    log(f"far ring: {len(spawned)} meshes from {root}")
    return len(spawned)
