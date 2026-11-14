"""The car fleet of the place, from open vehicle registers: body types and colours of the registered cars.

Netherlands: RDW open data (all insured passenger cars, CC0). New York City: NY DMV registrations of Manhattan
(county NEW YORK), with its yellow cabs. New Zealand: Waka Kotahi's Motor Vehicle Register (CC BY 4.0), light cars,
utes and vans of the territorial authority (Auckland, Wellington, Christchurch) or the whole country. Brazil:
Senatran's RENAVAM fleet by municipality (vehicle type and colour). Elsewhere the configured `traffic.car_types`
stay, with a world colour mix.

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
# a register that does not tell hatchbacks from saloons (Brazil's "automovel")
BODY_BLUEPRINTS["car"] = list(dict.fromkeys(BODY_BLUEPRINTS["hatchback"] + BODY_BLUEPRINTS["sedan"]))

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


# Waka Kotahi (NZ Transport Agency) Motor Vehicle Register: an ArcGIS Online item whose feature service is
# replaced every month (MVR_<Mon><yy>); the item id stays, so the service is looked up from it
NZTA_ITEM = "7b4df667d5014f1a93e6050b31d18407"
NZ_TLAS = [   # territorial authority -> rough box (s, w, n, e); elsewhere in New Zealand the national fleet
    ("AUCKLAND", (-37.40, 174.15, -36.10, 175.55)), ("WELLINGTON CITY", (-41.37, 174.60, -41.14, 174.90)),
    ("CHRISTCHURCH CITY", (-43.65, 172.45, -43.38, 172.80)), ("HAMILTON CITY", (-37.85, 175.18, -37.70, 175.35)),
    ("DUNEDIN CITY", (-45.95, 170.40, -45.78, 170.65)),
]
NZ_COLORS = {c: c.lower() for c in ("WHITE", "SILVER", "BLACK", "GREY", "BLUE", "RED", "GREEN", "BROWN", "GOLD",
                                    "ORANGE", "PURPLE", "YELLOW", "CREAM", "PINK")}
NZ_BODY = {"HATCHBACK": "hatchback", "SALOON": "sedan", "SPORTS CAR": "coupe", "CONVERTIBLE": "coupe",
           "LIGHT VAN": "van", "UTILITY": "pickup"}
# the register files SUVs and people movers as STATION WAGON; the common models (80 % of the wagons) are named
# here, the rest of the wagons are shared out in the same proportions
NZ_SUV = {
    "2008", "3008", "4008", "ACADIA", "AIRTREK", "ASX", "ATTO 2", "ATTO 3", "C-HR", "CAPTIVA", "CAYENNE",
    "CHALLENGER", "CHEROKEE", "COLORADO 7", "COMPASS", "COUNTRYMAN", "CR-V", "CROSSROAD", "CROSSTREK", "CRV", "CX-3",
    "CX-30", "CX-5", "CX-60", "CX-7", "CX-8", "CX-80", "CX-9", "DEFENDER", "DISCOVERY", "DISCOVERY SPORT", "DUALIS",
    "E-PACE", "ECLIPSE CROSS", "ECOSPORT", "ENDURA", "ESCAPE", "EV3", "EV5", "EV6", "EVEREST", "EXPLORER", "F-PACE",
    "FORESTER", "FORMENTOR", "FORTUNER", "FREELANDER", "FRONX", "G4 REXTON", "GLC", "GLC-CLASS", "GLE", "GLS",
    "GRAND CHEROKEE", "H2", "HARRIER", "HAVAL H6", "HAVAL JOLION", "HIGHLANDER", "HILUX", "HILUX SURF", "HR-V", "HS",
    "ID.4", "IONIQ 5", "IX35", "J7", "JIMNY", "JOURNEY", "JUKE", "KAMIQ", "KAROQ", "KLUGER", "KODIAQ", "KOLEOS",
    "KONA", "KORANDO", "KUGA", "LANDCRUISER", "LBX", "MACAN", "ML", "MODEL Y", "MOKKA", "MU", "MURANO",
    "MUSTANG MACH-E", "NIRO", "NITRO", "NX", "NX300H", "OUTBACK", "OUTLANDER", "PAJERO", "PAJERO SPORT", "PATHFINDER",
    "PATROL", "PUMA", "Q2", "Q3", "Q5", "Q7", "Q8", "QASHQAI", "RANGE ROVER", "RANGE ROVER EVOQUE",
    "RANGE ROVER SPORT", "RANGE ROVER VELAR", "RAV4", "RENEGADE", "REXTON", "RUSH", "RX", "RX350", "RX450H",
    "SANTA FE", "SEALION 6", "SEALION 7", "SELTOS", "SORENTO", "SORENTO R", "SPORTAGE", "STONIC", "SURF", "T-CROSS",
    "T-ROC", "TANK", "TERIOS", "TERRITORY", "TIGGO 4 PRO", "TIGUAN", "TIVOLI", "TOUAREG", "TRAILBLAZER", "TRAX",
    "TRIBUTE", "TUCSON", "UX250H", "VANGUARD", "VENUE", "VEZEL", "VITARA", "WRANGLER", "X-TRAIL", "X1", "X2", "X3",
    "X5", "X7", "XC40", "XC60", "XC90", "XUV", "XV", "YARIS CROSS", "ZRV", "ZS"}
NZ_MPV = {"WISH", "PREMACY", "SERENA", "ESTIMA", "ODYSSEY", "ALPHARD", "MPV", "STREAM", "VELLFIRE", "CARNIVAL",
          "ELGRAND", "LAFESTA", "IPSUM", "VOXY", "NOAH", "SIENTA", "BIANTE", "PREVIA", "PRESAGE", "ISIS", "STEPWAGON",
          "GRANDIS", "EDIX", "ELYSION", "GRANVIA", "FREED", "SPACIO", "FUNCARGO", "RAUM", "CUBE", "RACTIS",
          "TOURAN", "ZAFIRA", "GRAND C4 PICASSO"}
NZ_ESTATE = {"COROLLA", "PRIUS", "LEGACY", "ATENZA", "CALDINA", "WINGROAD", "MAZDA6", "AVENSIS", "IMPREZA", "AXELA",
             "GOLF", "PASSAT", "A4", "A6", "A3", "COMMODORE", "MARKX", "OCTAVIA", "LEVORG", "FIELDER", "AIRWAVE",
             "SHUTTLE", "FALCON", "FIT", "FOCUS", "I30", "I40", "CAMRY", "320I", "V50", "V60", "V70", "STAGEA",
             "PRIMERA", "LANCER", "ALTEZZA", "FAMILIA", "AD", "CARIB", "C5", "DEMIO", "NOTE", "CRUZE", "S4", "RS4",
             "RS6", "MONDEO", "SUPERB", "CORONA", "CROWN", "ACCORD", "TOURING", "MAZDA3", "SKYLINE"}
NZ_VAN = {"HIACE", "CARAVAN", "TOWNACE", "BONGO"}

# Senatran (Brazil's national traffic department) publishes the RENAVAM fleet each month, by municipality and
# vehicle type and by municipality and colour (xlsx, a few MB, links on one page per year)
SENATRAN = "https://www.gov.br/transportes/pt-br/assuntos/transito/conteudo-Senatran/frota-de-veiculos-{year}"
BR_CITIES = [   # (municipality, state, state code, rough box (s, w, n, e))
    ("SAO PAULO", "SAO PAULO", "SP", (-24.01, -46.83, -23.36, -46.36)),
    ("RIO DE JANEIRO", "RIO DE JANEIRO", "RJ", (-23.08, -43.80, -22.75, -43.10)),
    ("BRASILIA", "DISTRITO FEDERAL", "DF", (-16.05, -48.29, -15.50, -47.31)),
]
BR_COLORS = {"BRANCA": "white", "PRETA": "black", "PRATA": "silver", "CINZA": "grey", "VERMELHA": "red",
             "AZUL": "blue", "VERDE": "green", "BEGE": "beige", "AMARELA": "yellow", "MARROM": "brown",
             "LARANJA": "orange", "DOURADA": "gold", "ROXA": "purple", "GRENA": "maroon", "ROSA": "pink"}
BR_BODY = {"AUTOMOVEL": "car", "CAMIONETA": "suv", "CAMINHONETE": "pickup", "UTILITARIO": "mpv"}


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


def _arcgis_counts(url, where, field):
    """{value: count} of an ArcGIS feature service's field, counted by the server (paged)."""
    out, offset = {}, 0
    while True:
        r = requests.get(url + "/query", params={
            "where": where, "groupByFieldsForStatistics": field, "orderByFields": field, "f": "json",
            "outStatistics": json.dumps([{"statisticType": "count", "onStatisticField": "OBJECTID",
                                          "outStatisticFieldName": "n"}]),
            "resultOffset": offset, "resultRecordCount": 2000}, headers=realworld.UA, timeout=300)
        r.raise_for_status()
        d = r.json()
        if "error" in d:
            raise RuntimeError(d["error"].get("message", d["error"]))
        feats = d.get("features", [])
        for f in feats:
            a = f["attributes"]
            out[a.get(field)] = out.get(a.get(field), 0) + int(a["n"])
        if not feats or not d.get("exceededTransferLimit"):
            return out
        offset += len(feats)


def _nzta(tla=None):
    item = requests.get(f"https://www.arcgis.com/sharing/rest/content/items/{NZTA_ITEM}", params={"f": "json"},
                        headers=realworld.UA, timeout=60).json()
    url = item["url"].rstrip("/") + "/0"
    where = ("VEHICLE_TYPE IN ('PASSENGER CAR/VAN', 'GOODS VAN/TRUCK/UTILITY') AND BODY_TYPE IN ("
             + ", ".join(f"'{b}'" for b in [*NZ_BODY, "STATION WAGON"])
             + ") AND (GROSS_VEHICLE_MASS IS NULL OR GROSS_VEHICLE_MASS <= 3500)")
    if tla:
        where += f" AND TLA = '{tla}'"
    colors, body = {}, {}
    for k, n in _arcgis_counts(url, where, "BASIC_COLOUR").items():
        if k in NZ_COLORS:
            colors[NZ_COLORS[k]] = colors.get(NZ_COLORS[k], 0) + n
    for k, n in _arcgis_counts(url, where, "BODY_TYPE").items():
        if k in NZ_BODY:
            body[NZ_BODY[k]] = body.get(NZ_BODY[k], 0) + n
    wagons = {"suv": 0, "mpv": 0, "estate": 0}
    rest = 0
    for model, n in _arcgis_counts(url, where + " AND BODY_TYPE = 'STATION WAGON'", "MODEL").items():
        model = (model or "").strip().upper()
        if model in NZ_SUV:
            wagons["suv"] += n
        elif model in NZ_MPV:
            wagons["mpv"] += n
        elif model in NZ_ESTATE:
            wagons["estate"] += n
        elif model in NZ_VAN:
            body["van"] = body.get("van", 0) + n
        else:
            rest += n
    named = sum(wagons.values()) or 1
    for k, n in wagons.items():   # the unnamed wagons in the named ones' proportions
        body[k] = body.get(k, 0) + round(n + rest * n / named)
    where_name = tla.title() if tla else "New Zealand"
    return {"source": f"Waka Kotahi Motor Vehicle Register, light cars, utes and vans ({where_name}; "
                      f"{item.get('title', 'MVR')}, {url.split('/services/')[-1].split('/')[0]})",
            "licence": "CC BY 4.0 (Waka Kotahi NZ Transport Agency)", "colors": colors, "body": body}


def _in_nz(bbox):
    lat, lon = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    return -47.5 <= lat <= -34.0 and 166.0 <= lon <= 178.8


def _nz_tla(bbox):
    return next((tla for tla, box in NZ_TLAS if realworld._in_box(bbox, *box)), None)


def _xlsx_rows(data):
    """The rows of an xlsx file's first sheet as lists of strings (stdlib only: zip of XML)."""
    import io
    import xml.etree.ElementTree as ET
    import zipfile
    ns = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"
    z = zipfile.ZipFile(io.BytesIO(data))
    strings = []
    if "xl/sharedStrings.xml" in z.namelist():
        for si in ET.fromstring(z.read("xl/sharedStrings.xml")).iter(ns + "si"):
            strings.append("".join(t.text or "" for t in si.iter(ns + "t")))
    sheet = sorted(n for n in z.namelist() if n.startswith("xl/worksheets/sheet"))[0]
    for _, el in ET.iterparse(z.open(sheet)):
        if el.tag == ns + "row":
            row = []
            for c in el.iter(ns + "c"):
                v = c.find(ns + "v")
                if c.get("t") == "s" and v is not None:
                    row.append(strings[int(v.text)])
                elif c.get("t") == "inlineStr":
                    row.append("".join(x.text or "" for x in c.iter(ns + "t")))
                else:
                    row.append(v.text if v is not None else None)
            yield [(x or "").strip() for x in row]
            el.clear()


def _senatran_links():
    """The newest colour and municipality-type files on Senatran's yearly pages."""
    import re
    year = time.gmtime().tm_year
    found = {}
    for y in (year, year - 1):
        r = requests.get(SENATRAN.format(year=y), headers=realworld.UA, timeout=60)
        if r.status_code != 200:
            continue
        hrefs = re.findall(r'href="([^"]+\.xlsx/@@download/file)"', r.text)
        for key, pat in (("color", r"_COR_"), ("type", r"Frota_?por_?munic[ií]pio_?e_?tipo")):
            hit = next((h for h in hrefs if re.search(pat, h, re.I)), None)
            if hit and key not in found:
                found[key] = "https://www.gov.br" + hit if hit.startswith("/") else hit
        if len(found) == 2:
            return found
    raise RuntimeError("Senatran fleet files not found")


def _senatran(city, state, uf):
    links = _senatran_links()
    get = lambda u: requests.get(u, headers=realworld.UA, timeout=300).content   # noqa: E731
    colors, body = {}, {}
    for r in _xlsx_rows(get(links["color"])):   # UF (name), Município, Cor Veículo, Qtd. Veículos
        if len(r) >= 4 and r[0] == state and r[1] == city and r[2] in BR_COLORS:
            colors[BR_COLORS[r[2]]] = colors.get(BR_COLORS[r[2]], 0) + int(float(r[3]))
    header = None
    for r in _xlsx_rows(get(links["type"])):   # UF (code), MUNICIPIO, TOTAL, AUTOMOVEL, ...
        if r[:2] == ["UF", "MUNICIPIO"]:
            header = r
        elif header and len(r) >= 2 and r[0] == uf and r[1] == city:
            for name, n in zip(header, r):
                if name in BR_BODY and n:
                    body[BR_BODY[name]] = body.get(BR_BODY[name], 0) + int(float(n))
            break
    if not colors or not body:
        raise RuntimeError(f"{city} not in the Senatran files")
    month = links["color"].split("_COR_")[-1].split(".xlsx")[0].replace("_", " ")
    return {"source": f"Senatran RENAVAM fleet, {city.title()} ({month}; colours of all vehicles)",
            "licence": "Brazilian federal open data (gov.br, CC BY-ND 3.0 site terms; only counts are used)",
            "colors": colors, "body": body}


def _br_city(bbox):
    return next((c for c in BR_CITIES if realworld._in_box(bbox, *c[3])), None)


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
        if _in_nz(bbox):
            tla = _nz_tla(bbox)
            return _cached("nzta_" + (tla or "new zealand").lower().replace(" ", "_"), lambda: _nzta(tla))
        br = _br_city(bbox)
        if br:
            return _cached("senatran_" + br[0].lower().replace(" ", "_"), lambda: _senatran(*br[:3]))
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
