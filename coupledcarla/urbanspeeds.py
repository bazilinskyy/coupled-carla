"""Urban default speed limits for OSM roads without maxspeed, at map build (`map.urban_speeds: auto`).

netconvert gives an OSM road without a `maxspeed` tag the speed of its type in SUMO's osmNetconvert.typ.xml, which
holds rural values: 100 km/h on trunk, primary and secondary roads, 80 on tertiary roads and links. In a city that
put 109 edges of Midtown Manhattan and 204 around Taipei Main Station at 100 km/h, in SUMO's net and in CARLA's
OpenDRIVE alike. Here the map gets its own copy of that type file (urban.typ.xml in the map folder) in which those
road types carry the country's urban default limit instead (speeds.URBAN_KMH: the legal limit inside built-up areas
where no sign says otherwise, after the OSM wiki's "Default speed limits" and the national codes). netconvert uses a
type's speed only where the way has no maxspeed, so every mapped limit stays as tagged. A type's speed is only ever
lowered (a living street keeps its 10 km/h, a residential street its 50 where the urban limit is 60), and motorways,
their links and service roads keep netconvert's values: an urban motorway has signed limits, which OSM maps.

The rest of the type file is SUMO's own, unchanged, so the net is the one netconvert builds without a type file
(checked: identical for Midtown Manhattan) except for those speeds. SUMO's osmNetconvertUrbanDe.typ.xml does the same
for Germany only (50 km/h on all of them); there is no other country's file in SUMO.

Maps built this way record it in meta.json ("speed_defaults"); speeds.plan then leaves them alone, and its variable
speed signs per trial remain the fallback for maps built before.
"""
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from . import speeds

FILE = "urban.typ.xml"


def base_typemap():
    """SUMO's osmNetconvert.typ.xml (SUMO_HOME, else the eclipse-sumo pip package), or None."""
    homes = [os.environ.get("SUMO_HOME")]
    try:
        import sumo                      # the eclipse-sumo wheel
        homes.append(getattr(sumo, "SUMO_HOME", None) or os.path.dirname(sumo.__file__))
    except ImportError:
        pass
    for home in homes:
        if home and (Path(home) / "data" / "typemap" / "osmNetconvert.typ.xml").exists():
            return Path(home) / "data" / "typemap" / "osmNetconvert.typ.xml"
    return None


def urban_speeds(country, base_text):
    """{OSM road type: new speed m/s} for the types of the base type file the country's urban default lowers."""
    out = {}
    root = ET.fromstring(base_text)
    for t in root.iter("type"):
        tid = t.get("id", "")
        road = tid.replace("highway.", "")
        if not tid.startswith("highway.") or road not in speeds.NETCONVERT_DEFAULT or t.get("speed") is None:
            continue
        v = round(speeds.urban_limit(country, road) / 3.6, 2)
        if v < float(t.get("speed")) - 0.05:
            out[road] = v
    return out


def write_typemap(path, country, base=None):
    """Write the base type file with the country's urban defaults to path -> {road type: km/h} changed, or None
    when SUMO's type file is not found (netconvert then uses its built-in defaults, as before)."""
    base = Path(base) if base else base_typemap()
    if base is None or not base.exists():
        return None
    text = base.read_text(encoding="utf-8")
    new = urban_speeds(country, text)
    root = ET.fromstring(text)
    for t in root.iter("type"):
        road = t.get("id", "").replace("highway.", "")
        if t.get("id", "").startswith("highway.") and road in new:
            t.set("speed", f"{new[road]:.2f}")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return {k: round(v * 3.6) for k, v in sorted(new.items())}


def netconvert_args(cfg, out, log=print):
    """(extra netconvert arguments for the OSM import, meta.json "speed_defaults" or None). Never fails: with
    map.urban_speeds off, without SUMO's type file or on an error, netconvert's own defaults (one log line)."""
    mc = cfg["map"]
    if mc.get("urban_speeds", "auto") in (False, None, "off"):
        return [], None
    try:
        country = speeds.country_of(cfg, mc["bbox"])
        kmh = write_typemap(Path(out) / FILE, country)
        if kmh is None:
            log("urban speeds: SUMO's osmNetconvert.typ.xml not found; roads without maxspeed keep netconvert's "
                "defaults")
            return [], None
        log(f"urban speeds: roads without an OSM maxspeed get the urban default of {country or 'most countries'} "
            f"({', '.join(f'{k} {v}' for k, v in kmh.items())} km/h) -> {FILE}")
        return ["--type-files", str(Path(out) / FILE)], {"country": country, "file": FILE, "kmh": kmh}
    except Exception as err:  # noqa: BLE001 - the map still builds, with netconvert's defaults
        log(f"urban speeds: not used ({type(err).__name__}: {err}); netconvert's defaults")
        return [], None
