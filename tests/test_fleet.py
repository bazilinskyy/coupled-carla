"""Fleet from vehicle registers: body types to CARLA blueprints, colours on SUMO cars (no network)."""
import re

from coupledcarla import fleet


def test_car_types_follow_body_shares():
    reg = {"body": {"hatchback": 60, "van": 40}, "colors": {"grey": 1}}
    w = fleet.car_types(reg)
    hatch = sum(v for k, v in w.items() if k in fleet.BODY_BLUEPRINTS["hatchback"])
    van = sum(v for k, v in w.items() if k in fleet.BODY_BLUEPRINTS["van"])
    assert abs(hatch - 60) < 0.1 and abs(van - 40) < 0.1


def test_painter_follows_register_colours_and_paints_taxis():
    p = fleet.Painter({"colors": {"black": 90, "white": 10}}, seed=3)
    cols = [p("vehicle.seat.leon") for _ in range(500)]
    dark = sum(1 for c in cols if int(c.split(",")[0]) < 60)
    assert 400 < dark < 490
    p.taxis = True
    assert p("vehicle.ford.crown") == "250,195,0"


def test_paint_routes(tmp_path):
    f = tmp_path / "car.rou.xml"
    f.write_text('<routes>\n  <vehicle id="car0" type="vehicle.seat.leon" depart="0.00">\n    <route edges="a"/>\n'
                 '  </vehicle>\n  <vehicle id="car1" type="x" depart="1" color="1,2,3"/>\n</routes>\n')
    fleet.paint_routes(f, lambda bp: "9,9,9")
    t = f.read_text()
    assert re.search(r'<vehicle id="car0"[^>]*color="9,9,9">', t) and 'color="1,2,3"' in t


class _Resp:
    def __init__(self, data=None, content=b"", text="", status=200):
        self._data, self.content, self.text, self.status_code = data, content, text, status

    def json(self):
        return self._data

    def raise_for_status(self):
        pass


def test_nzta_counts_on_the_server_and_splits_station_wagons(monkeypatch, tmp_path):
    import json as _json
    monkeypatch.setattr(fleet, "CACHE", tmp_path)
    counts = {"BASIC_COLOUR": {"WHITE": 50, "SILVER": 30, "MAUVE": 5},
              "BODY_TYPE": {"HATCHBACK": 40, "SALOON": 10, "UTILITY": 10, "STATION WAGON": 40},
              "MODEL": {"RAV4": 20, "COROLLA": 10, "ESTIMA": 5, "OBSCURA": 5}}
    seen = []

    def get(url, params=None, **kw):
        if "sharing/rest/content/items" in url:
            return _Resp({"url": "https://example.org/arcgis/rest/services/MVR_Jan26/FeatureServer",
                          "title": "Motor Vehicle Register"})
        seen.append(params["where"])
        field = params["groupByFieldsForStatistics"]
        assert _json.loads(params["outStatistics"])[0]["statisticType"] == "count"
        return _Resp({"features": [{"attributes": {field: k, "n": n}} for k, n in counts[field].items()]})
    monkeypatch.setattr(fleet.requests, "get", get)
    reg = fleet.registry([-36.855, 174.755, -36.842, 174.771])   # Auckland CBD
    assert all("TLA = 'AUCKLAND'" in w for w in seen)
    assert reg["colors"] == {"white": 50, "silver": 30}
    # 35 named wagons (20 SUV, 10 estate, 5 MPV); the 5 unnamed ones shared out in those proportions
    assert reg["body"]["hatchback"] == 40 and reg["body"]["pickup"] == 10
    assert reg["body"]["suv"] == 23 and reg["body"]["estate"] == 11 and reg["body"]["mpv"] == 6
    assert (tmp_path / "nzta_auckland.json").exists()


def _xlsx(rows):
    import io
    import zipfile
    from xml.sax.saxutils import escape

    def cell(v):
        if isinstance(v, (int, float)):
            return f"<c><v>{v}</v></c>"
        return f'<c t="inlineStr"><is><t>{escape(str(v))}</t></is></c>'
    sheet = ('<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>'
             + "".join("<row>" + "".join(cell(v) for v in r) + "</row>" for r in rows) + "</sheetData></worksheet>")
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("xl/worksheets/sheet1.xml", sheet)
    return buf.getvalue()


def test_senatran_reads_colour_and_type_sheets(monkeypatch, tmp_path):
    monkeypatch.setattr(fleet, "CACHE", tmp_path)
    page = ('<a href="/x/C_Frota_por_UF_Municipio_COR_Setembro_2026.xlsx/@@download/file">c</a>'
            '<a href="/x/Frota_por_municipio_e_tipo_Setembro_2026.xlsx/@@download/file">t</a>')
    color = _xlsx([["UF", "Município", "Cor Veículo", "Qtd. Veículos"], ["SAO PAULO", "SAO PAULO", "PRETA", 60],
                   ["SAO PAULO", "SAO PAULO", "BRANCA", 40], ["SAO PAULO", "SAO PAULO", "FANTASIA", 9],
                   ["SAO PAULO", "CAMPINAS", "AZUL", 99]])
    types = _xlsx([["Frota de veículos"], ["UF", "MUNICIPIO", "TOTAL", "AUTOMOVEL", "CAMINHONETE", "CAMIONETA",
                                            "MOTOCICLETA", "UTILITARIO"],
                   ["SP", "CAMPINAS", 9, 9, 0, 0, 0, 0], ["SP", "SAO PAULO", 200, 80, 5, 10, 100, 5]])

    def get(url, params=None, **kw):
        if url.endswith("2026"):
            return _Resp(text=page)
        return _Resp(content=color if "_COR_" in url else types)
    monkeypatch.setattr(fleet.requests, "get", get)
    reg = fleet.registry([-23.555, -46.64, -23.54, -46.625])   # Sé, São Paulo
    assert reg["colors"] == {"black": 60, "white": 40}
    assert reg["body"] == {"car": 80, "pickup": 5, "suv": 10, "mpv": 5}
    assert "Setembro 2026" in reg["source"]
    w = fleet.car_types(reg)
    assert abs(sum(w.values()) - 100) < 0.5 and "vehicle.tesla.model3" in w and "vehicle.nissan.micra" in w


def test_register_offline_falls_back(monkeypatch, tmp_path):
    monkeypatch.setattr(fleet, "CACHE", tmp_path)

    def down(*a, **kw):
        raise OSError("offline")
    monkeypatch.setattr(fleet.requests, "get", down)
    assert fleet.registry([-36.855, 174.755, -36.842, 174.771]) is None
    assert fleet.registry([-23.555, -46.64, -23.54, -46.625]) is None
    assert fleet.registry([-1.96, 30.05, -1.94, 30.07]) is None   # Kigali: no register
