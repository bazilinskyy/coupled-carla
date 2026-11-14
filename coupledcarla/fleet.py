"""The car fleet of the place, from open vehicle registers: body types and colours of the registered cars.

Netherlands: RDW open data (all insured passenger cars, CC0). New York City: NY DMV registrations of Manhattan
(county NEW YORK), with its yellow cabs. Elsewhere the configured `traffic.car_types` stay, with a world colour mix.

The body types become weights over the CARLA blueprints that look like them (traffic.car_types), and each SUMO car
gets a colour drawn from the register's colour shares; the bridge passes it on to the CARLA car
(sync_vehicle_color). A register is the fleet that is registered, not the mix on the street.
"""
import json
import random
import time
from pathlib import Path

import requests

from . import realworld

CACHE = Path.home() / ".cache" / "coupled-carla" / "fleet"

# CARLA 0.9.16 cars by body type (names as the experiment files use them; compat.py maps them for CARLA 1.0)
BODY_BLUEPRINTS = {
    "hatchback": ["vehicle.mini.cooper_s_2021", "vehicle.nissan.micra", "vehicle.citroen.c3", "vehicle.audi.a2",
                  "vehicle.seat.leon", "vehicle.toyota.prius"],
    "estate": ["vehicle.bmw.grandtourer", "vehicle.seat.leon", "vehicle.toyota.prius"],
    "mpv": ["vehicle.bmw.grandtourer", "vehicle.audi.etron"],
    "suv": ["vehicle.audi.etron", "vehicle.nissan.patrol_2021", "vehicle.jeep.wrangler_rubicon"],
    "sedan": ["vehicle.tesla.model3", "vehicle.lincoln.mkz_2020", "vehicle.mercedes.coupe_2020"],
    "us_sedan": ["vehicle.lincoln.mkz_2020", "vehicle.tesla.model3", "vehicle.chevrolet.impala",
                 "vehicle.dodge.charger_2020"],
    "coupe": ["vehicle.audi.tt", "vehicle.mercedes.coupe_2020", "vehicle.ford.mustang"],
    "pickup": ["vehicle.tesla.cybertruck", "vehicle.nissan.patrol_2021"],
    "van": ["vehicle.mercedes.sprinter", "vehicle.volkswagen.t2_2021"],
    "taxi": ["vehicle.ford.crown"],
}

# register colour -> RGB (CARLA's color attribute), with a shade spread for each
COLORS = {
    "grey": (128, 128, 128), "black": (18, 18, 20), "white": (236, 236, 234), "blue": (30, 60, 140),
    "red": (160, 22, 22), "green": (30, 90, 45), "brown": (90, 60, 36), "yellow": (225, 190, 30),
    "orange": (215, 105, 25), "beige": (200, 180, 140), "purple": (85, 40, 110), "cream": (232, 222, 190),
    "pink": (220, 150, 170), "silver": (175, 178, 182), "gold": (180, 150, 70), "maroon": (100, 20, 30),
    "taxi_yellow": (250, 195, 0),
}
# typical car colours worldwide (no register): white, black, grey, silver, blue, red dominate
WORLD_COLORS = {"white": 35, "black": 19, "grey": 18, "silver": 8, "blue": 8, "red": 7, "brown": 2, "green": 1,
                "beige": 1, "yellow": 1}

RDW_COLORS = {"GRIJS": "grey", "ZWART": "black", "WIT": "white", "BLAUW": "blue", "ROOD": "red", "GROEN": "green",
              "BRUIN": "brown", "GEEL": "yellow", "ORANJE": "orange", "BEIGE": "beige", "PAARS": "purple",
              "CREME": "cream", "ROSE": "pink"}
RDW_BODY = {"hatchback": "hatchback", "stationwagen": "estate", "MPV": "mpv", "sedan": "sedan", "Sedan": "sedan",
            "cabriolet": "coupe", "coupe": "coupe", "terreinvoertuig": "suv"}
DMV_COLORS = {"GY": "grey", "LT GY": "grey", "DK GY": "grey", "BK": "black", "WH": "white", "BL": "blue",
              "RD": "red", "GR": "green", "BR": "brown", "YW": "yellow", "TN": "beige", "OR": "orange",
              "GL": "gold", "MR": "maroon", "SL": "silver", "SILVR": "silver", "PR": "purple", "PK": "pink"}
DMV_BODY = {"SUBN": "suv", "4DSD": "us_sedan", "SEDN": "us_sedan", "2DSD": "coupe", "CONV": "coupe",
            "VAN": "van", "PICK": "pickup", "UTIL": "suv", "DELV": "van", "TAXI": "taxi"}


def _soda(url, params):
    r = requests.get(url, params=params, headers=realworld.UA, timeout=180)
    r.raise_for_status()
    return r.json()


def _cached(name, fetch, days=30):
    path = CACHE / f"{name}.json"
    if path.exists() and time.time() - path.stat().st_mtime < days * 86400:
        return json.loads(path.read_text())
    data = fetch()
    CACHE.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1))
    return data


def _rdw():
    url = "https://opendata.rdw.nl/resource/m9d7-ebf2.json"
    where = "voertuigsoort='Personenauto' AND wam_verzekerd='Ja'"
    colors, body = {}, {}
    for r in _soda(url, {"$select": "eerste_kleur,count(*) as n", "$where": where, "$group": "eerste_kleur"}):
        c = RDW_COLORS.get(r.get("eerste_kleur"))
        if c:
            colors[c] = colors.get(c, 0) + int(r["n"])
    for r in _soda(url, {"$select": "inrichting,count(*) as n", "$where": where, "$group": "inrichting"}):
        b = RDW_BODY.get(r.get("inrichting"))
        if b:
            body[b] = body.get(b, 0) + int(r["n"])
    return {"source": "RDW open data, insured passenger cars (Netherlands)", "licence": "CC0 1.0",
            "colors": colors, "body": body}


def _nyc():
    url = "https://data.ny.gov/resource/w4pv-hbkt.json"
    where = "county='NEW YORK' AND record_type='VEH'"
    colors, body = {}, {}
    for r in _soda(url, {"$select": "color,count(*) as n", "$where": where + " AND registration_class='PAS'",
                         "$group": "color", "$limit": 200}):
        c = DMV_COLORS.get((r.get("color") or "").strip())
        if c:
            colors[c] = colors.get(c, 0) + int(r["n"])
    for r in _soda(url, {"$select": "body_type,count(*) as n", "$where": where + " AND registration_class='PAS'",
                         "$group": "body_type", "$limit": 200}):
        b = DMV_BODY.get(r.get("body_type"))
        if b:
            body[b] = body.get(b, 0) + int(r["n"])
    taxis = _soda(url, {"$select": "count(*) as n", "$where": where + " AND registration_class='OMT'"})
    body["taxi"] = int(taxis[0]["n"]) if taxis else 0
    return {"source": "NY DMV vehicle registrations, Manhattan (passenger cars and yellow cabs)",
            "licence": "NY Open Data terms", "colors": colors, "body": body}


def _in_nyc(bbox):
    s, w, n, e = bbox
    lat, lon = (s + n) / 2, (w + e) / 2
    return 40.68 <= lat <= 40.88 and -74.03 <= lon <= -73.90


def registry(bbox):
    """The register's fleet for the box, or None where there is none (cached for a month)."""
    try:
        if realworld.in_netherlands(bbox):
            return _cached("rdw_nl", _rdw)
        if _in_nyc(bbox):
            return _cached("ny_dmv_manhattan", _nyc)
    except Exception as err:  # noqa: BLE001 - offline: the configured fleet
        print(f"[fleet] register not reached ({err}); the configured fleet is used", flush=True)
    return None


def car_types(reg):
    """Body-type shares -> weights over CARLA blueprints."""
    total = sum(reg["body"].values()) or 1
    out = {}
    for body, n in reg["body"].items():
        bps = BODY_BLUEPRINTS.get(body, [])
        for bp in bps:
            out[bp] = out.get(bp, 0) + n / total / len(bps)
    return {bp: round(100 * w, 2) for bp, w in out.items() if w > 0}


class Painter:
    """Draws a colour for each car: the register's colour shares (or the world's), with a little shade spread."""

    def __init__(self, reg=None, seed=0):
        shares = (reg or {}).get("colors") or WORLD_COLORS
        self.names = [k for k in shares if k in COLORS]
        self.weights = [shares[k] for k in self.names]
        self.rng = random.Random(seed)

    def __call__(self, blueprint=""):
        if "ford.crown" in blueprint and self.taxis:
            return _rgb(COLORS["taxi_yellow"])
        name = self.rng.choices(self.names, self.weights)[0]
        r, g, b = COLORS[name]
        k = self.rng.uniform(0.85, 1.12)
        return _rgb((min(255, int(r * k)), min(255, int(g * k)), min(255, int(b * k))))

    taxis = False


def _rgb(c):
    return f"{c[0]},{c[1]},{c[2]}"


BIKE_COLORS = {"black": 40, "grey": 15, "blue": 12, "white": 10, "red": 8, "green": 7, "brown": 5, "silver": 3}


def paint_routes(path, painter, kind="car"):
    """Give every <vehicle> or <trip> in a SUMO route file a colour (SUMO's default is yellow, which the bridge
    would copy onto the CARLA car)."""
    import re
    text = Path(path).read_text()

    def repl(m):
        tag = m.group(0)
        if " color=" in tag:
            return tag
        vtype = re.search(r'type="([^"]*)"', tag)
        return tag.replace(">", f' color="{painter(vtype.group(1) if vtype else "")}">', 1) \
            if tag.endswith(">") and not tag.endswith("/>") else \
            tag.replace("/>", f' color="{painter(vtype.group(1) if vtype else "")}"/>', 1)
    text = re.sub(r"<(vehicle|trip) [^>]*?/?>", repl, text)
    Path(path).write_text(text)
