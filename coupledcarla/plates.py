"""Country number plates on the vehicles Coupled CARLA spawns.

CARLA 1.0 with the plate code (CARLA LicensePlates.cpp: the vehicle blueprint attribute ``license_plate``, an
ISO 3166 alpha-2 code) mounts on each vehicle a plate of the country's format and real size from
/Game/CoupledCarla/Plates/<CC> (drawn by tools/plate_faces.py, imported by tools/ue_plates.py): Dutch yellow plates
in Eindhoven, Spanish in Salamanca, Japanese in Tokyo, New York's in New York, instead of CARLA's California
"CARLA" plate. A server without that code has no such attribute and nothing changes.

Vehicles whose meshes carry CARLA's plate material get the plate where that material is. CARLA 1.0's newer
vehicles (Lincoln MKZ, Mini Cooper, Kia Niro, Tesla Model 3, the Fuso bus ...) have none: the server measures
where their body is flat and upright at plate height (LicensePlates.cpp BodySpots) and logs the spots; collected
into data/plate_spots.json (tools/plate_spots.py), they travel with the country code ("ES;rear=...;front=...")
for servers that cannot read their meshes (a cooked package).

The country is the map's (crowdpeds.locate, as for the signal heads and street furniture; CARLA's own towns are
North American: US), or ``vehicles.license_plates`` in the experiment: "auto" (default), an ISO2 code, or false
(CARLA's plates).
"""
import json
from functools import lru_cache
from pathlib import Path

# countries with plate content (tools/plate_faces.py COUNTRIES), ISO3 -> ISO2
PLATES = {"NLD": "NL", "ESP": "ES", "FIN": "FI", "DEU": "DE", "FRA": "FR", "CHE": "CH", "JPN": "JP", "USA": "US",
          "UKR": "UA", "RWA": "RW"}
SPOTS_FILE = Path(__file__).resolve().parent / "data" / "plate_spots.json"

_country = None
_iso3 = None


def country_for(iso3):
    """The plate country (ISO2) for a map's ISO3 country, or None where there are no plates for it."""
    return PLATES.get((iso3 or "").upper())


def country_iso3(tc, mdir):
    """The map's country (ISO3) or None: CARLA's towns are North American (USA); else the experiment's
    city.country, the nearest CROWD locality or OSM (crowdpeds.locate)."""
    if (tc.get("map") or {}).get("carla_town"):
        return "USA"
    try:
        from . import crowdpeds
        bbox = tc["map"].get("bbox") or json.loads((Path(mdir) / "meta.json").read_text())["bbox"]
        return crowdpeds.locate(tc, bbox, Path(mdir) / "real" / "crowdpeds_country.json")[0]
    except Exception:  # noqa: BLE001 - country unknown
        return None


def setup(tc, mdir, log=print):
    """Sets the plate country of this trial from the experiment and the map; returns it (None: CARLA's plates).
    The map's ISO3 country stays in ``plates.iso3()`` for the signs (localsigns.py)."""
    global _country, _iso3
    want = (tc.get("vehicles") or {}).get("license_plates", "auto")
    _country = None
    _iso3 = country_iso3(tc, mdir)
    if want in (False, None, "off", "none"):
        return None
    if isinstance(want, str) and want.lower() != "auto":
        _country = want.upper()
    else:
        _country = country_for(_iso3)
    if _country:
        log(f"number plates: {_country}")
    elif _iso3:
        log(f"number plates: none drawn for {_iso3} (tools/plate_faces.py COUNTRIES): CARLA's own")
    return _country


def iso3():
    """The map's country (ISO3) found by the last setup(), or None."""
    return _iso3


@lru_cache(maxsize=None)
def spots():
    """{blueprint id: ";rear=...;front=..."} of data/plate_spots.json ({} without it)."""
    try:
        d = json.loads(SPOTS_FILE.read_text())
    except (OSError, ValueError):
        return {}
    return {bp: "".join(f";{side}={','.join(str(v) for v in s)}" for side, s in sorted(sides.items()))
            for bp, sides in (d.get("spots") or {}).items()}


def value_for(bp_id, country=None):
    """The license_plate attribute for a blueprint: the country code, plus the vehicle's measured plate spots when
    its mesh has no plate material (data/plate_spots.json)."""
    country = country or _country
    if not country:
        return ""
    return country + spots().get(str(bp_id), "")


def tag(bp):
    """Puts this trial's plate country on a vehicle blueprint (a no-op for walkers, props and old servers)."""
    if _country and bp.has_attribute("license_plate"):
        bp.set_attribute("license_plate", value_for(getattr(bp, "id", ""), _country))
    return bp
