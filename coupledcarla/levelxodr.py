"""Is a baked level's map dir built from the level's own OpenDRIVE?

A map with map.carla_level (a carla-digitaltwins twin, a CARLA town) loads the level by name, and the server drives
on the level's own roads. Everything else coupled-carla puts there is built from the map dir's map.xodr: SUMO's net,
the scenery (surface tiles, parked cars, signs, furniture), the clinker and sett paving (the twinPaving mark in its
header), the speed limits. When map.xodr is an older export than the level (map.opendrive copied before the level
was rebaked, or before carla-digitaltwins' road_paving marked it), those land on roads that are not there: on
24 Nov 2026 a clinker surface tile stood 13 m up across the De Zaale passage on TUeCampus, placed on road 277 of a
stale map.xodr.

So after loading a level the run compares the server's OpenDRIVE (world.get_map().to_opendrive()) with map.xodr by
a signature of what matters (signature(): every road and junction with its numbers rounded to 1 mm, without the
roads' signals, and the twinPaving mark; not formatting, comments, the date or the PROJ string's geoid grid, which mapbuild strips). On a
difference it saves the level's copy as <map dir>/level.xodr and, with map.level_xodr (default "stop"), stops the
run naming what differs and the two commands that fix it; "warn" logs and runs on (the scenery may be misplaced).
tools/level_xodr.py refreshes the source (map.opendrive) from the level and rebuilds the map.
"""
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

NUM = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$")
GEOID = re.compile(r"\s*\+geoidgrids=[^\s\]<]+|\s*\+vunits=[^\s\]<]+")   # not into "]]>" or "</"
# a road's signals are left out: after each bake the level's copy loses the signals whose stock sign actors CARLA
# would spawn (C:\\cc58\\patches\\cc_drop_signs_one.py), while map.xodr keeps them, as coupled-carla places its own
# signs for them; roads, lanes, geometry, elevation, objects and junctions must match
SKIP = {"signals"}
PAVING = re.compile(r'<userData\s+code="twinPaving"\s+value="([^"]*)"')


def strip_geoid(text):
    """The OpenDRIVE text without geoid grids and vertical units in its PROJ string (as mapbuild writes map.xodr)."""
    return GEOID.sub("", text)


def _val(v):
    v = v.strip()
    if NUM.match(v):
        r = round(float(v), 3) + 0.0       # -0.0 -> 0.0
        return f"{r:.3f}".rstrip("0").rstrip(".") or "0"
    return v


def _canon(el, out):
    if el.tag in SKIP:
        return
    attrs = ",".join(f"{k}={_val(v)}" for k, v in sorted(el.attrib.items()))
    text = (el.text or "").strip()
    out.append(f"<{el.tag} {attrs}>{_val(text) if text else ''}")
    for c in el:
        _canon(c, out)
    out.append(f"</{el.tag}>")


def signature(text):
    """{"roads", "junctions", "paving", "hash", "road_hashes"} of an OpenDRIVE text: what the level drives on."""
    text = strip_geoid(text)
    m = PAVING.search(text[:20000])
    paving = sorted(s for s in (m.group(1).split(",") if m else []) if s)
    root = ET.fromstring(text.encode("utf-8") if isinstance(text, str) else text)
    road_hashes, whole = {}, hashlib.sha256()
    for tag in ("road", "junction"):
        for el in sorted(root.findall(tag), key=lambda e: (e.get("id") or "")):
            out = []
            _canon(el, out)
            h = hashlib.sha256("".join(out).encode("utf-8")).hexdigest()[:16]
            if tag == "road":
                road_hashes[el.get("id") or ""] = h
            whole.update(f"{tag}{el.get('id')}:{h};".encode())
    whole.update(("paving:" + ",".join(paving)).encode())
    return {"roads": len(road_hashes), "junctions": len(root.findall("junction")), "paving": paving,
            "hash": whole.hexdigest()[:20], "road_hashes": road_hashes}


def file_signature(path):
    return signature(Path(path).read_text(encoding="utf-8", errors="replace"))


def differences(a, b):
    """Plain-language list of what differs between two signatures (a: map.xodr, b: the level's)."""
    if a["hash"] == b["hash"]:
        return []
    out = []
    if a["roads"] != b["roads"] or a["junctions"] != b["junctions"]:
        out.append(f"{a['roads']} roads and {a['junctions']} junctions in map.xodr, {b['roads']} and "
                   f"{b['junctions']} in the level")
    ra, rb = a.get("road_hashes") or {}, b.get("road_hashes") or {}
    only_a, only_b = sorted(set(ra) - set(rb)), sorted(set(rb) - set(ra))
    changed = sorted(i for i in set(ra) & set(rb) if ra[i] != rb[i])
    if only_a:
        out.append(f"{len(only_a)} roads only in map.xodr (e.g. {', '.join(only_a[:5])})")
    if only_b:
        out.append(f"{len(only_b)} roads only in the level (e.g. {', '.join(only_b[:5])})")
    if changed:
        out.append(f"{len(changed)} roads with other geometry or lanes (e.g. {', '.join(changed[:5])})")
    if a["paving"] != b["paving"]:
        out.append(f"paving mark {','.join(a['paving']) or 'none'} in map.xodr, "
                   f"{','.join(b['paving']) or 'none'} in the level")
    return out or ["the junctions differ"]


def map_signature(mdir):
    """map.xodr's signature: from xodr_signature.json when mapbuild wrote it for this very file, else computed."""
    mdir = Path(mdir)
    xodr = mdir / "map.xodr"
    try:
        sig = json.loads((mdir / "xodr_signature.json").read_text())
        if sig.get("size") == xodr.stat().st_size and sig.get("road_hashes") is not None:
            return sig
    except (OSError, ValueError):
        pass
    return file_signature(xodr)


def store_signature(mdir):
    """mapbuild: keep map.xodr's signature (with its size, so a map.xodr replaced by hand is noticed). Returns the
    short form for meta.json."""
    mdir = Path(mdir)
    sig = file_signature(mdir / "map.xodr")
    sig["size"] = (mdir / "map.xodr").stat().st_size
    (mdir / "xodr_signature.json").write_text(json.dumps(sig))
    return {k: sig[k] for k in ("roads", "junctions", "paving", "hash")}


def fix_commands(cfg):
    exp = cfg.get("_path") or "<experiment>.yaml"
    return (f"python tools/level_xodr.py {exp} --from <the level's bake input, e.g. <Name>_pave.xodr, or "
            f"<map dir>/level.xodr> (replaces map.opendrive with it and rebuilds the map), then tools/snap_scenery.py; "
            f"or set map.level_xodr: warn to run anyway")


def check(world, mdir, cfg, log=print):
    """After a baked level is loaded: True when map.xodr is the level's own OpenDRIVE, False (or SystemExit, with
    map.level_xodr "stop", the default) when not. The level's copy is saved as <mdir>/level.xodr on a difference."""
    mc = cfg["map"]
    mdir = Path(mdir)
    if not mc.get("carla_level") or not (mdir / "map.xodr").exists():
        return None
    level_text = world.get_map().to_opendrive()
    lsig = signature(level_text)
    msig = map_signature(mdir)
    diff = differences(msig, lsig)
    if not diff:
        return True
    (mdir / "level.xodr").write_text(level_text, encoding="utf-8")
    msg = (f"{mdir / 'map.xodr'} is not the OpenDRIVE of the level {mc['carla_level']}: " + "; ".join(diff)
           + ". SUMO's net, the scenery, the paving and the parked cars were built from the other one and stand "
           f"on roads the level does not have. The level's copy is saved as {mdir / 'level.xodr'}. Fix: "
           + fix_commands(cfg) + ".")
    if (mc.get("level_xodr") or "stop") == "warn":
        log("WARNING " + msg)
        return False
    raise SystemExit(msg)
