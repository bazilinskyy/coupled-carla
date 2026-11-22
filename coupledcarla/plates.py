"""Country number plates on the vehicles Coupled CARLA spawns.

CARLA 1.0 with the plate code (CARLA LicensePlates.cpp: the vehicle blueprint attribute ``license_plate``, an
ISO 3166 alpha-2 code) swaps a vehicle's plate material for one of the country's plates in
/Game/CoupledCarla/Plates/<CC> (drawn by tools/plate_faces.py, imported by tools/ue_plates.py): Dutch yellow plates
in Eindhoven, Spanish in Salamanca, Japanese in Tokyo, New York's in New York, instead of CARLA's California
"CARLA" plate. A server without that code has no such attribute and nothing changes.

The country is the map's (crowdpeds.locate, as for the signal heads and street furniture), or
``vehicles.license_plates`` in the experiment: "auto" (default), an ISO2 code, or false (CARLA's plates).
"""
import json

# countries with plate content (tools/plate_faces.py COUNTRIES), ISO3 -> ISO2
PLATES = {"NLD": "NL", "ESP": "ES", "FIN": "FI", "DEU": "DE", "FRA": "FR", "CHE": "CH", "JPN": "JP", "USA": "US"}

_country = None


def country_for(iso3):
    """The plate country (ISO2) for a map's ISO3 country, or None where there are no plates for it."""
    return PLATES.get((iso3 or "").upper())


def setup(tc, mdir, log=print):
    """Sets the plate country of this trial from the experiment and the map; returns it (None: CARLA's plates)."""
    global _country
    want = (tc.get("vehicles") or {}).get("license_plates", "auto")
    _country = None
    if want in (False, None, "off", "none"):
        return None
    if isinstance(want, str) and want.lower() != "auto":
        _country = want.upper()
    else:
        try:
            from . import crowdpeds
            bbox = tc["map"].get("bbox") or json.loads((mdir / "meta.json").read_text())["bbox"]
            _country = country_for(crowdpeds.locate(tc, bbox, mdir / "real" / "crowdpeds_country.json")[0])
        except Exception:  # noqa: BLE001 - country unknown: CARLA's plates
            _country = None
    if _country:
        log(f"number plates: {_country}")
    return _country


def tag(bp):
    """Puts this trial's plate country on a vehicle blueprint (a no-op for walkers, props and old servers)."""
    if _country and bp.has_attribute("license_plate"):
        bp.set_attribute("license_plate", _country)
    return bp
