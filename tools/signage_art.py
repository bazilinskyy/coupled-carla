"""Generated advertising and shop signage for the twins: every face, poster, billboard and screen frame is drawn here.

    python tools/street_props.py OUT          # builds these textures with the other street props (signage_art.textures)
    python tools/signage_art.py OUT [--sheet sheet.png]   # only these, plus a contact sheet to look at

Nothing in here is a real advertisement, brand, logo or trademark, and no photo is used: the brand names, products,
shop names, films, shows and slogans are invented, the layouts are procedural (Pillow), the letters are open fonts.
Real reference photos (Mapillary / Wikimedia Commons, CC BY-SA) were only looked at for the style: what kind of sign
hangs where, how dense, how big, which colours (README "Generated signage"). Where a real place's look comes from:

- Shibuya / Japan (locale JP): lit horizontal signboards over the shopfronts (kanban) in white, yellow, red and blue
  with black or white gothic or mincho letters, multi-tenant vertical signs listing the floors (2F, 3F, ...) on the
  building corners, cloth banners (nobori) and lit stand signs on the pavement, wall and rooftop billboards, video
  screens with Japanese copy. Letters: Noto Sans CJK JP Bold and Noto Serif CJK JP Bold (SIL Open Font Licence),
  as subsets in unreal/StreetProps/fonts (tools/signage_fonts.py makes them from the full fonts).
- The Netherlands (NL), Spain (ES), German-speaking Switzerland (DE), Finland (FI, Finnish with Swedish below) and
  the United States (EN, Times Square billboards and screens): painted or lit fascia boards with the trade and an
  invented shop name in the local language, A-board pavement signs at cafes, poster cases (MUPI), advertising
  columns and roadside billboards with local copy. Letters: DejaVu Sans / Serif and STIX (free licences, shipped with
  matplotlib).

catalog() lists every item (mesh maker, size, texture, lit or not) for tools/street_props.py; textures() draws them.
The runtime placement is coupledcarla/signage.py.
"""
from __future__ import annotations

import math
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FONT_DIR = ROOT / "unreal" / "StreetProps" / "fonts"
JP_FONTS = {"jp_sans": FONT_DIR / "NotoSansCJKjp-Bold-signage.otf",
            "jp_serif": FONT_DIR / "NotoSerifCJKjp-Bold-signage.otf"}
LATIN_FONTS = {"sans": "DejaVuSans-Bold.ttf", "serif": "DejaVuSerif-Bold.ttf", "sans_it": "DejaVuSans-BoldOblique.ttf",
               "serif_it": "DejaVuSerif-BoldItalic.ttf", "times": "STIXGeneralBol.ttf", "light": "DejaVuSans.ttf",
               "mono": "DejaVuSansMono-Bold.ttf"}
LOCALES = ("JP", "NL", "ES", "DE", "FI", "EN")
VARIANTS = 2            # fascia / kanban faces per trade and locale


# ---------------------------------------------------------------------------------------------------- the texts
#
# Shop fascias: kind -> [(trade word, invented name), ...] (VARIANTS of them). The kinds are coupledcarla.signage's
# (KIND_OF maps OSM tags onto them). Names are made up; none is a chain or a brand.

FASCIA = {
    "NL": {
        "bakery": [("BAKKERIJ", "De Korenschoof"), ("Bakker", "Het Zuurdesemhuis")],
        "cafe": [("CAFÉ", "De Vergulde Lantaarn"), ("Koffiehuis", "Bonenbrander")],
        "restaurant": [("RESTAURANT", "De Gouden Pollepel"), ("Eetcafé", "Het Hoekje")],
        "bar": [("BRUIN CAFÉ", "De Twee Zwanen"), ("Bar", "Nachtuil")],
        "fastfood": [("SNACKBAR", "De Frietkoning"), ("Shoarma & Grill", "Zuiderpoort")],
        "hair": [("KAPSALON", "Knipwerk"), ("Barbier", "Scheermes & Zeep")],
        "pharmacy": [("APOTHEEK", "De Lindeboom"), ("Drogisterij", "De Vijzel")],
        "fashion": [("MODE", "Atelier Noorderlicht"), ("Kleding", "Lapjeswinkel")],
        "books": [("BOEKHANDEL", "Het Leeshuis"), ("Antiquariaat", "Ezelsoor")],
        "florist": [("BLOEMEN", "Tulpenhof"), ("Bloemsierkunst", "Het Madeliefje")],
        "supermarket": [("BUURTSUPER", "Dagvers"), ("Supermarkt", "De Korf")],
        "shop": [("WINKEL", "'t Snuffelhoekje"), ("Cadeaus", "Prul & Pracht")],
    },
    "ES": {
        "bakery": [("PANADERÍA", "La Espiga"), ("Horno", "San Cristóbal")],
        "cafe": [("CAFETERÍA", "El Mirador"), ("Café", "La Tertulia")],
        "restaurant": [("RESTAURANTE", "Casa Aurelio"), ("Mesón", "El Arco")],
        "bar": [("BAR", "La Parra"), ("Taberna", "Los Faroles")],
        "fastfood": [("BOCADILLERÍA", "El Buen Bocado"), ("Kebab", "Plaza")],
        "hair": [("PELUQUERÍA", "Tijeras de Oro"), ("Barbería", "Navaja")],
        "pharmacy": [("FARMACIA", "Lda. M. Rivas"), ("Farmacia", "del Puente")],
        "fashion": [("MODAS", "Carmela"), ("Moda", "Lino y Algodón")],
        "books": [("LIBRERÍA", "Hojas Sueltas"), ("Librería", "El Quijote")],
        "florist": [("FLORISTERÍA", "Azahar"), ("Flores", "La Violeta")],
        "supermarket": [("ALIMENTACIÓN", "Ultramarinos Pilar"), ("Supermercado", "La Cesta")],
        "shop": [("REGALOS", "La Alacena"), ("Tienda", "El Rincón")],
    },
    "DE": {
        "bakery": [("BÄCKEREI", "Sonnenkorn"), ("Confiserie", "Zum Mühlerad")],
        "cafe": [("CAFÉ", "Zur Linde"), ("Kaffeebar", "Bohnenglück")],
        "restaurant": [("RESTAURANT", "Zum Goldenen Hirschen"), ("Gasthaus", "Alpenrose")],
        "bar": [("BAR", "Nachtfalter"), ("Weinstube", "Zum Fass")],
        "fastfood": [("IMBISS", "Wurstbude am Platz"), ("Take-away", "Falafel & Co.")],
        "hair": [("COIFFEUR", "Haarwerk"), ("Coiffeur", "Schnittpunkt")],
        "pharmacy": [("APOTHEKE", "am Platz"), ("Drogerie", "Kamille")],
        "fashion": [("MODE", "Stoffwechsel"), ("Boutique", "Silberdistel")],
        "books": [("BUCHHANDLUNG", "Seitenwind"), ("Antiquariat", "Eselsohr")],
        "florist": [("BLUMEN", "Enzian"), ("Floristik", "Löwenzahn")],
        "supermarket": [("LEBENSMITTEL", "Quartierladen"), ("Markt", "Frischeck")],
        "shop": [("GESCHENKE", "Schatzkiste"), ("Laden", "Allerlei")],
    },
    "FI": {      # Finnish, Swedish under it (Helsinki is bilingual)
        "bakery": [("LEIPOMO · BAGERI", "Ruiskukka"), ("Leipomo", "Kaurahiutale")],
        "cafe": [("KAHVILA · KAFÉ", "Pulla & Kuppi"), ("Kahvila", "Satamakivi")],
        "restaurant": [("RAVINTOLA · RESTAURANG", "Merituuli"), ("Ravintola", "Kuusikallio")],
        "bar": [("BAARI · BAR", "Revontuli"), ("Pub", "Vanha Ankkuri")],
        "fastfood": [("GRILLI · GRILL", "Nakkikioski"), ("Kebab", "Kauppatori")],
        "hair": [("KAMPAAMO · FRISÖR", "Saksiniekka"), ("Parturi", "Partaveitsi")],
        "pharmacy": [("APTEEKKI · APOTEK", "Kruununhaka"), ("Apteekki", "Senaatti")],
        "fashion": [("VAATTEET · KLÄDER", "Pellava"), ("Muoti", "Lumikide")],
        "books": [("KIRJAKAUPPA · BOKHANDEL", "Sivulause"), ("Antikvariaatti", "Koirankorva")],
        "florist": [("KUKKAKAUPPA · BLOMSTER", "Kielo"), ("Kukka", "Ruusunmarja")],
        "supermarket": [("RUOKAKAUPPA · LIVSMEDEL", "Kulmapuoti"), ("Lähikauppa", "Torikori")],
        "shop": [("LAHJAT · PRESENTER", "Aarrearkku"), ("Puoti", "Kaikkea Kivaa")],
    },
    "EN": {
        "bakery": [("BAKERY", "Rise & Crumb"), ("Bagels", "Hudson Street Ovens")],
        "cafe": [("COFFEE", "Grind & Gable"), ("Café", "The Corner Cup")],
        "restaurant": [("RESTAURANT", "Marlowe's Table"), ("Diner", "Eighth Avenue Grill")],
        "bar": [("BAR", "The Copper Lantern"), ("Tavern", "Old Pier Taproom")],
        "fastfood": [("PIZZA", "Gino & Sons Slice"), ("Deli", "Ninth Avenue Heroes")],
        "hair": [("SALON", "Snip & Sparrow"), ("Barbershop", "Ninth Street Shears")],
        "pharmacy": [("PHARMACY", "Midtown Chemists"), ("Drugs", "Lexington Rx")],
        "fashion": [("APPAREL", "Thread & Thimble"), ("Boutique", "Wren Lane")],
        "books": [("BOOKS", "Dog-Eared Pages"), ("Bookshop", "Ink & Paper")],
        "florist": [("FLOWERS", "Petalwise"), ("Florist", "Bloom Room")],
        "supermarket": [("DELI & GROCERY", "Fresh Corner"), ("Market", "24/7 Food Mart")],
        "shop": [("GIFTS", "Gotham Souvenirs"), ("Electronics", "Wired Up")],
    },
}

# Japanese lit signboards (kanban): kind -> [(shop name, line under it), ...]
KANBAN = {
    "ramen": [("麺屋 とりあかり", "ラーメン・つけ麺"), ("らあめん 月見坂", "濃厚とんこつ")],
    "sushi": [("寿司処 海凪", "江戸前 握り"), ("回転寿司 まるなみ", "一皿 120円より")],
    "izakaya": [("居酒屋 灯り家", "焼き鳥・地酒"), ("炉端 こはる", "生ビール 390円")],
    "cafe": [("喫茶 こもれび", "珈琲・軽食"), ("カフェ ほしのね", "ケーキ・ランチ")],
    "restaurant": [("ひだまり食堂", "日替わり定食"), ("洋食 グリル朝凪", "ハンバーグ・オムライス")],
    "karaoke": [("カラオケ うたまる", "全室 Wi-Fi 完備"), ("カラオケ ルミナ", "30分 100円から")],
    "pharmacy": [("ドラッグ みどり堂", "くすり・化粧品"), ("薬局 すずかぜ", "処方せん受付")],
    "hair": [("美容室 リュンヌ", "カット・カラー"), ("ヘアサロン 風花", "メンズ歓迎")],
    "fashion": [("古着屋 ミモザ", "USED CLOTHING"), ("セレクトショップ 縞", "NEW ARRIVAL")],
    "books": [("書店 ふみの森", "本・雑誌・文具"), ("古書 しおり堂", "買取いたします")],
    "supermarket": [("コンビニ ひなた", "24時間営業"), ("ミニマート あおぞら", "お弁当・飲料")],
    "shop": [("雑貨 てとて", "生活雑貨"), ("電器 かみなり堂", "スマホ修理")],
}
# multi-tenant vertical signs: the floors of one building, top to bottom
TENANTS = [
    [("2F", "居酒屋"), ("3F", "カラオケ"), ("4F", "美容室"), ("5F", "歯科")],
    [("B1", "バー"), ("2F", "焼肉"), ("3F", "整体"), ("4F", "英会話")],
    [("2F", "カフェ"), ("3F", "ネイル"), ("4F", "麻雀"), ("5F", "占い")],
    [("2F", "ラーメン"), ("3F", "ジム"), ("4F", "居酒屋"), ("5F", "眼科")],
    [("3F", "カラオケ"), ("4F", "個室"), ("5F", "バー"), ("6F", "事務所")],
    [("2F", "古着"), ("3F", "美容室"), ("4F", "塾"), ("5F", "整骨院")],
]
NOBORI = [("営業中", (200, 20, 30), (255, 255, 255)), ("ランチ", (250, 245, 235), (200, 30, 30)),
          ("生ビール", (255, 210, 0), (20, 20, 20)), ("ラーメン", (180, 20, 20), (255, 240, 200)),
          ("焼き鳥", (30, 30, 30), (255, 220, 120)), ("大盛無料", (20, 70, 160), (255, 255, 255)),
          ("期間限定", (240, 120, 20), (255, 255, 255)), ("新メニュー", (30, 120, 60), (255, 255, 255))]
STAND = [("カフェ 2F", (250, 250, 245), (40, 30, 20)), ("ランチ営業中", (200, 30, 30), (255, 255, 255)),
         ("カラオケ 3F", (220, 40, 130), (255, 255, 255)), ("本日のおすすめ", (20, 20, 25), (255, 220, 90)),
         ("居酒屋 地下1階", (120, 20, 20), (255, 236, 200)), ("美容室 4F", (245, 245, 250), (120, 40, 100))]
ABOARD = {
    "NL": [["Vandaag", "Koffie + appeltaart", "€ 4,50"], ["Lunch", "Tosti & soep", "Terras open!"]],
    "ES": [["Menú del día", "Primero, segundo", "y postre 12 €"], ["Tapas", "Caña + tapa", "2,50 €"]],
    "DE": [["Heute", "Kaffee & Gipfeli", "CHF 5.50"], ["Mittagsmenü", "Suppe, Salat", "CHF 18.50"]],
    "FI": [["Tänään · Idag", "Kahvi + pulla", "4,50 €"], ["Lounas · Lunch", "Keitto ja leipä", "11,90 €"]],
    "EN": [["Today", "Coffee $3", "Bagel $2.50"], ["Happy Hour", "4-7 pm", "Half-price drafts"]],
}

# Advertisements: headline, line, invented wordmark, the picture's shape, background (two colours) and text colour.
ADS = {
    "JP": [
        ("夏、はじける。", "新発売 ソーダ", "HOSHIBA SODA", "bottle", ((40, 170, 230), (10, 60, 140)), (255, 255, 255)),
        ("新しい、ひかりを。", "スマートフォン", "KUMORA X5", "phone", ((20, 20, 30), (70, 70, 110)), (255, 255, 255)),
        ("月影の約束", "全国ロードショー", "映画", "moon", ((10, 15, 40), (60, 30, 90)), (255, 230, 160)),
        ("最大50%OFF", "冬のセール開催中", "HANAMIZUKA", "tag", ((230, 30, 50), (150, 10, 30)), (255, 255, 255)),
        ("全国ツアー決定", "LIVE TOUR 2026", "MIZUKA", "star", ((240, 70, 150), (90, 20, 120)), (255, 255, 255)),
        ("うるおい、まとう。", "新しい化粧水", "RINKA BEAUTY", "drop", ((250, 235, 240), (230, 190, 210)), (120, 40, 80)),
        ("星の環", "好評配信中 新作RPG", "GAME", "ring", ((20, 30, 70), (10, 120, 160)), (255, 255, 255)),
        ("旅に出よう。", "沖縄 片道 9,800円から", "RINKAI AIR", "plane", ((120, 200, 240), (250, 250, 255)), (10, 50, 110)),
        ("バイト探しは", "シゴトノワ", "SHIGOTO NO WA", "circle", ((255, 200, 0), (255, 150, 0)), (30, 30, 30)),
        ("ずるっと、うまい。", "カップ麺 新登場", "ZURUZURU-DO", "cup", ((250, 240, 220), (240, 200, 120)), (150, 20, 20)),
        ("ぜんぶ、まるごと。", "新しい通信プラン", "SORAMICHI NET", "phone", ((0, 160, 120), (0, 90, 80)), (255, 255, 255)),
        ("のどごし、さわやか。", "冷たいお茶", "MIDORI CHA", "bottle", ((40, 140, 60), (200, 230, 160)), (255, 255, 255)),
    ],
    "EN": [
        ("See more of tonight.", "The new Novarell 9", "NOVARELL", "phone", ((15, 15, 25), (60, 60, 100)), (255, 255, 255)),
        ("Crisp. Cold. Yours.", "Ice-cold sparkling", "FIZZWELL", "bottle", ((220, 30, 40), (120, 10, 20)), (255, 255, 255)),
        ("THE GLASS ORCHARD", "A new musical", "NOW PLAYING", "star", ((20, 10, 30), (120, 30, 60)), (255, 215, 120)),
        ("HARBOR LIGHTS", "New season streaming Friday", "PELLOWAY+", "moon", ((5, 20, 40), (20, 80, 120)), (255, 255, 255)),
        ("Stride further.", "Zephyrine Runners", "ZEPHYRINE", "shoe", ((250, 250, 250), (200, 200, 210)), (20, 20, 20)),
        ("Aureline", "Eau de Parfum", "AURELINE", "drop", ((240, 220, 200), (190, 150, 110)), (60, 40, 20)),
        ("NYC - LONDON", "from $399", "FLY BLUEMERE", "plane", ((30, 90, 200), (120, 190, 250)), (255, 255, 255)),
        ("Covered. Calm.", "Home, car and life", "HARTWELL MUTUAL", "circle", ((0, 110, 90), (0, 60, 60)), (255, 255, 255)),
        ("SKYLINES", "An exhibition of city design", "MIDTOWN HALL", "ring", ((250, 200, 0), (250, 120, 0)), (20, 20, 20)),
        ("Read it first.", "The Morning Ledger", "LEDGER", "tag", ((240, 240, 235), (210, 210, 200)), (20, 20, 20)),
        ("Big taste. Little price.", "Burgers from $5", "STACKHOUSE", "cup", ((255, 190, 0), (220, 60, 20)), (60, 20, 0)),
        ("Wake up brighter.", "Cold brew in a can", "DAYBREAK", "bottle", ((40, 30, 25), (140, 90, 50)), (255, 235, 200)),
    ],
    "NL": [
        ("Zomerfestival 2026", "Muziek, dans en theater", "STADSFEEST", "star", ((250, 120, 30), (220, 40, 80)), (255, 255, 255)),
        ("Fris. Bruisend. Nieuw.", "Bronwater met een bubbel", "HELDERBRON", "bottle", ((40, 160, 210), (10, 70, 140)), (255, 255, 255)),
        ("Fiets, en kom verder.", "Elektrische stadsfiets", "TRAPLICHT", "circle", ((30, 120, 70), (10, 60, 40)), (255, 255, 255)),
        ("Nu in de bioscoop", "De Laatste Veerpont", "FILM", "moon", ((15, 20, 40), (60, 40, 90)), (255, 230, 160)),
        ("Lekker lokaal", "Kaas van de boerderij", "HOEVEKAAS", "tag", ((255, 205, 0), (240, 150, 0)), (60, 30, 0)),
    ],
    "ES": [
        ("Ciclo de conciertos", "Teatro municipal, otoño 2026", "MÚSICA", "star", ((150, 20, 40), (60, 10, 30)), (255, 230, 170)),
        ("Pura sierra.", "Agua mineral natural", "FONTERA", "bottle", ((60, 170, 220), (20, 80, 150)), (255, 255, 255)),
        ("Recicla bien.", "Separa en casa, gana la ciudad", "RECICLA", "circle", ((0, 140, 80), (0, 80, 50)), (255, 255, 255)),
        ("Estreno en cines", "La Última Vendimia", "CINE", "moon", ((20, 15, 30), (90, 40, 40)), (255, 220, 150)),
        ("Sabor de siempre", "Turrón artesano", "DULCEMIEL", "tag", ((240, 200, 120), (190, 120, 50)), (80, 30, 10)),
    ],
    "DE": [
        ("Sommerkonzerte 2026", "Am See, jeden Freitag", "KLANGSOMMER", "star", ((20, 60, 120), (10, 30, 60)), (255, 255, 255)),
        ("Neu: Alpenklar", "Mineralwasser mit Kohlensäure", "ALPENKLAR", "bottle", ((90, 180, 230), (20, 90, 160)), (255, 255, 255)),
        ("Velo fahren hält jung.", "Mit dem Velo zur Arbeit", "VELOSTADT", "circle", ((230, 30, 40), (150, 10, 20)), (255, 255, 255)),
        ("Jetzt im Kino", "Der Letzte Zug nach Chur", "KINO", "moon", ((15, 15, 30), (50, 50, 80)), (255, 230, 170)),
        ("Feinste Schokolade", "Seit 1891 von Hand", "BERGLI", "tag", ((90, 40, 20), (160, 90, 50)), (255, 230, 190)),
    ],
    "FI": [
        ("Kesäkonsertit 2026", "Sommarkonserter 2026", "KESÄSOI", "star", ((20, 50, 120), (60, 140, 200)), (255, 255, 255)),
        ("Uutuus: Kirkaslähde", "Nyhet: mineralvatten", "KIRKASLÄHDE", "bottle", ((100, 190, 230), (20, 80, 150)), (255, 255, 255)),
        ("Pyöräile kaupunkiin", "Cykla till stan", "POLKU", "circle", ((0, 130, 90), (0, 70, 60)), (255, 255, 255)),
        ("Elokuvateattereissa nyt", "Viimeinen lautta · Den sista färjan", "ELOKUVA", "moon", ((10, 20, 40), (50, 60, 100)), (255, 230, 170)),
        ("Aamun paras hetki", "Morgonens bästa stund", "SATAMAPAAHTIMO", "cup", ((60, 35, 20), (150, 100, 60)), (255, 240, 210)),
    ],
}
# advertising-column posters: a strip of three event posters per set
COLUMN = {
    "ES": [[("CONCIERTO", "Sábado 21 h"), ("EXPOSICIÓN", "Museo de la ciudad"), ("TEATRO", "Estreno")],
           [("FERIA", "Septiembre"), ("CINE", "Al aire libre"), ("DANZA", "Flamenco en vivo")]],
    "DE": [[("KONZERT", "Samstag 20 Uhr"), ("AUSSTELLUNG", "Kunsthaus"), ("THEATER", "Premiere")],
           [("FLOHMARKT", "Jeden Sonntag"), ("OPER", "Neuinszenierung"), ("ZIRKUS", "Nur 3 Tage")]],
    "FI": [[("KONSERTTI", "Lauantaina klo 19"), ("NÄYTTELY", "Taidemuseo"), ("TEATTERI", "Ensi-ilta")],
           [("MARKKINAT", "Kauppatori"), ("OOPPERA", "Uusi tuotanto"), ("TANSSI", "Nykytanssi")]],
}

# What the coupledcarla/signage.py placement asks for, per locale: (family, keys) -> mesh SM_CC_<loc>_<family>_<key>
SCREEN_LOCALES = ("JP", "EN")           # region-specific screen content (else the generic SM_CC_Any_Screen_*)
BILLBOARD_LOCALES = ("JP", "EN")        # wall billboards (dense commercial streets, Shibuya, Times Square)
N_BILLBOARD, N_BILLBOARD_TALL, N_ROOF, N_MUPI, N_STANDBB = 8, 4, 4, 3, 2


def jp_texts():
    """Every Japanese string drawn (the font subset must cover them all: tests/test_signage.py)."""
    out = []
    for v in KANBAN.values():
        for a, b in v:
            out += [a, b]
    for t in TENANTS:
        out += [a + b for a, b in t]
    out += [t for t, _, _ in NOBORI] + [t for t, _, _ in STAND]
    for a in ADS["JP"]:
        out += [a[0], a[1], a[2]]
    out += ["営業時間 11:00-23:00", "→", "↑", "↓", "円", "丨"]
    return out


# ---------------------------------------------------------------------------------------------------- the catalogue

def _ad(loc, i):
    return ADS[loc][i % len(ADS[loc])]


def catalog():
    """[dict]: name (mesh name without SM_CC_<loc>_), loc, folder, role, maker (tools/street_props maker name),
    kw (its arguments besides the face slot), slot, tex (texture key, T_CC_<tex>.png), lit (emissive strength or
    None for an unlit printed face), pan (u, v per second), draw (what textures() draws), note."""
    C = []

    def add(name, loc, maker, kw, draw, lit=None, pan=(0.0, 0.0), role="signage", note=""):
        slot = f"Sig_{loc}_{name}"
        C.append({"name": name, "loc": loc, "folder": "Static", "role": role, "maker": maker, "kw": kw, "slot": slot,
                  "tex": f"sig_{loc.lower()}_{name.lower()}", "lit": lit, "pan": pan, "draw": draw, "note": note})
    for loc, kinds in FASCIA.items():
        for kind, faces in kinds.items():
            for v, (trade, name) in enumerate(faces[:VARIANTS], 1):
                w, h = (3.6, 0.8) if loc == "EN" else (3.2, 0.6)
                add(f"Fascia_{kind}_{v}", loc, "shop_sign", {"width": w, "height": h},
                    ("fascia", loc, kind, v - 1), role="fascia", note=f"fascia: {trade} {name} (invented)")
    for kind, faces in KANBAN.items():
        for v, (name, sub) in enumerate(faces[:VARIANTS], 1):
            add(f"Kanban_{kind}_{v}", "JP", "shop_sign", {"width": 3.6, "height": 0.9, "depth": 0.22},
                ("kanban", kind, v - 1), lit=2.5, role="kanban", note=f"lit signboard: {name} (invented)")
    for v in range(len(TENANTS)):
        add(f"Tenant_{v + 1}", "JP", "blade_sign", {"out": 0.9, "height": 7.2, "thick": 0.3},
            ("tenant", v), lit=2.0, role="tenant_sign", note="multi-tenant vertical sign (floors, generic trades)")
    for v, (text, _, _) in enumerate(NOBORI):
        add(f"Nobori_{v + 1}", "JP", "nobori", {}, ("nobori", v), role="nobori", note=f"banner: {text}")
    for v, (text, _, _) in enumerate(STAND):
        add(f"Stand_{v + 1}", "JP", "stand_box", {}, ("stand", v), lit=2.0, role="stand_sign",
            note=f"lit stand sign: {text}")
    for loc, boards in ABOARD.items():
        for v in range(len(boards)):
            add(f"ABoard_{v + 1}", loc, "a_board", {}, ("aboard", loc, v), role="a_board", note="chalk A-board")
    for loc in BILLBOARD_LOCALES:
        for v in range(N_BILLBOARD):
            add(f"Billboard_{v + 1}", loc, "screen", {"width": 8.0, "height": 4.5, "depth": 0.35},
                ("ad", loc, v, 1280, 720), lit=1.6, role="billboard", note="wall billboard, invented ad, 16:9")
        for v in range(N_BILLBOARD_TALL):
            add(f"BillboardTall_{v + 1}", loc, "screen", {"width": 4.5, "height": 8.0, "depth": 0.35},
                ("ad", loc, v + N_BILLBOARD, 576, 1024), lit=1.6, role="billboard_tall",
                note="wall billboard, invented ad, 9:16")
    for v in range(N_ROOF):
        add(f"RoofBillboard_{v + 1}", "JP", "roof_billboard", {}, ("ad", "JP", v + 3, 1536, 512), lit=1.6,
            role="roof_billboard", note="rooftop billboard on a steel frame, invented ad")
    for loc in SCREEN_LOCALES:
        for k in "ABCD":
            add(f"Screen_{k}", loc, "screen", {}, ("screen", loc, "ABCD".index(k)), lit=6.0, pan=(0.0, 0.02),
                role="screen", note="facade screen, 16:9, invented ads in the local language")
            add(f"ScreenTall_{k}", loc, "screen", {"width": 2.25, "height": 4.0}, ("screen", loc, "ABCD".index(k)),
                lit=6.0, pan=(0.0, 0.02), role="screen_tall", note="facade screen, 9:16")
    for loc in ADS:
        for v in range(N_MUPI):
            add(f"Mupi_{v + 1}", loc, "mupi", {}, ("ad", loc, v + 1, 512, 768), lit=1.4, role="poster_box",
                note="lit poster case (OSM advertising=poster_box/board), invented ad")
        for v in range(N_STANDBB):
            add(f"BillboardStand_{v + 1}", loc, "billboard_stand", {}, ("ad", loc, v + 4, 1024, 512), role="billboard",
                note="roadside billboard (OSM advertising=billboard), invented ad")
    for loc, sets in COLUMN.items():
        for v in range(len(sets)):
            add(f"AdColumn_{v + 1}", loc, "ad_column", {}, ("column", loc, v), role="ad_column",
                note="Morris column with local event posters (invented)")
    return C


# ---------------------------------------------------------------------------------------------------- drawing

_FONTS = {}


def font(key, size):
    from PIL import ImageFont
    size = max(6, int(size))
    k = (key, size)
    if k not in _FONTS:
        if key in JP_FONTS:
            path = JP_FONTS[key]
            if not path.exists():
                raise SystemExit(f"signage: {path} missing (tools/signage_fonts.py makes it from Noto CJK)")
            _FONTS[k] = ImageFont.truetype(str(path), size)
        else:
            import matplotlib
            _FONTS[k] = ImageFont.truetype(str(Path(matplotlib.__file__).parent / "mpl-data" / "fonts" / "ttf"
                                               / LATIN_FONTS[key]), size)
    return _FONTS[k]


def _has_jp(s):
    return any(ord(c) > 0x2000 for c in s)


def fit(d, text, key, max_w, max_h, top=None):
    """The largest font of `key` (a Japanese face for text with Japanese characters) that fits max_w x max_h:
    (font, width, height, y offset of the ink's top)."""
    size = int(top or max_h)
    while True:
        f = font(key, size)
        x0, y0, x1, y1 = d.textbbox((0, 0), text, font=f)
        if (x1 - x0 <= max_w and y1 - y0 <= max_h) or size <= 8:
            return f, x1 - x0, y1 - y0, x0, y0
        size = int(size * 0.92)


def text_in(d, box, text, key, fill, align="c", top=None):
    """Draw text as large as it fits in box (x0, y0, x1, y1), centred vertically, aligned l/c/r."""
    x0, y0, x1, y1 = box
    f, w, h, ox, oy = fit(d, text, key, x1 - x0, y1 - y0, top)
    x = x0 if align == "l" else (x1 - w if align == "r" else (x0 + x1 - w) / 2)
    d.text((x - ox, (y0 + y1 - h) / 2 - oy), text, font=f, fill=fill)
    return w


def vtext(d, box, text, key, fill):
    """Japanese text top to bottom (tategaki) in box: one character a cell, the long vowel mark upright."""
    x0, y0, x1, y1 = box
    chars = ["丨" if c == "ー" else c for c in text if c != " "]
    step = (y1 - y0) / max(1, len(chars))
    size = min(x1 - x0, step) * 0.92
    f = font(key, size)
    for n, c in enumerate(chars):
        bx0, by0, bx1, by1 = d.textbbox((0, 0), c, font=f)
        cx, cy = (x0 + x1) / 2, y0 + (n + 0.5) * step
        d.text((cx - (bx0 + bx1) / 2, cy - (by0 + by1) / 2), c, font=f, fill=fill)


def gradient(img, a, b, vertical=True):
    from PIL import ImageDraw
    d = ImageDraw.Draw(img)
    W, H = img.size
    n = H if vertical else W
    for i in range(n):
        t = i / max(1, n - 1)
        c = tuple(int(a[k] * (1 - t) + b[k] * t) for k in range(3))
        d.line([(0, i), (W, i)] if vertical else [(i, 0), (i, H)], fill=c)


def _rng(*parts):
    import random
    return random.Random(zlib.crc32("|".join(map(str, parts)).encode()))


def shape(d, kind, cx, cy, s, col, accent):
    """A simple product picture: no logo, no real product."""
    if kind == "bottle":
        d.rounded_rectangle([cx - 0.18 * s, cy - 0.25 * s, cx + 0.18 * s, cy + 0.5 * s], int(0.08 * s), fill=col)
        d.rectangle([cx - 0.07 * s, cy - 0.5 * s, cx + 0.07 * s, cy - 0.2 * s], fill=col)
        d.rectangle([cx - 0.09 * s, cy - 0.56 * s, cx + 0.09 * s, cy - 0.48 * s], fill=accent)
        d.rectangle([cx - 0.18 * s, cy + 0.0 * s, cx + 0.18 * s, cy + 0.2 * s], fill=accent)
    elif kind == "phone":
        d.rounded_rectangle([cx - 0.24 * s, cy - 0.5 * s, cx + 0.24 * s, cy + 0.5 * s], int(0.07 * s), fill=(20, 20, 24))
        d.rounded_rectangle([cx - 0.21 * s, cy - 0.45 * s, cx + 0.21 * s, cy + 0.45 * s], int(0.05 * s), fill=col)
        d.ellipse([cx - 0.12 * s, cy - 0.2 * s, cx + 0.12 * s, cy + 0.04 * s], fill=accent)
    elif kind == "moon":
        d.ellipse([cx - 0.4 * s, cy - 0.4 * s, cx + 0.4 * s, cy + 0.4 * s], fill=col)
        d.ellipse([cx - 0.25 * s, cy - 0.5 * s, cx + 0.55 * s, cy + 0.3 * s], fill=accent)
    elif kind == "tag":
        d.polygon([(cx - 0.45 * s, cy - 0.25 * s), (cx + 0.25 * s, cy - 0.25 * s), (cx + 0.45 * s, cy),
                   (cx + 0.25 * s, cy + 0.25 * s), (cx - 0.45 * s, cy + 0.25 * s)], fill=col)
        d.ellipse([cx + 0.17 * s, cy - 0.05 * s, cx + 0.27 * s, cy + 0.05 * s], fill=accent)
    elif kind == "star":
        pts = []
        for k in range(10):
            r = 0.48 * s if k % 2 == 0 else 0.2 * s
            a = -math.pi / 2 + k * math.pi / 5
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        d.polygon(pts, fill=col)
    elif kind == "drop":
        d.ellipse([cx - 0.3 * s, cy - 0.1 * s, cx + 0.3 * s, cy + 0.5 * s], fill=col)
        d.polygon([(cx, cy - 0.5 * s), (cx - 0.27 * s, cy + 0.08 * s), (cx + 0.27 * s, cy + 0.08 * s)], fill=col)
    elif kind == "ring":
        d.ellipse([cx - 0.45 * s, cy - 0.45 * s, cx + 0.45 * s, cy + 0.45 * s], outline=col, width=int(0.1 * s))
        d.ellipse([cx - 0.2 * s, cy - 0.2 * s, cx + 0.2 * s, cy + 0.2 * s], fill=accent)
    elif kind == "plane":
        d.polygon([(cx - 0.5 * s, cy), (cx + 0.5 * s, cy - 0.08 * s), (cx + 0.5 * s, cy + 0.05 * s)], fill=col)
        d.polygon([(cx - 0.05 * s, cy - 0.02 * s), (cx + 0.15 * s, cy - 0.4 * s), (cx + 0.25 * s, cy - 0.04 * s)],
                  fill=col)
        d.polygon([(cx - 0.05 * s, cy + 0.02 * s), (cx + 0.1 * s, cy + 0.35 * s), (cx + 0.22 * s, cy + 0.03 * s)],
                  fill=col)
    elif kind == "shoe":
        d.polygon([(cx - 0.5 * s, cy + 0.2 * s), (cx - 0.45 * s, cy - 0.15 * s), (cx - 0.1 * s, cy - 0.12 * s),
                   (cx + 0.2 * s, cy + 0.05 * s), (cx + 0.5 * s, cy + 0.12 * s), (cx + 0.5 * s, cy + 0.25 * s)], fill=col)
        d.rectangle([cx - 0.5 * s, cy + 0.2 * s, cx + 0.5 * s, cy + 0.28 * s], fill=accent)
    elif kind == "cup":
        d.polygon([(cx - 0.32 * s, cy - 0.35 * s), (cx + 0.32 * s, cy - 0.35 * s), (cx + 0.24 * s, cy + 0.45 * s),
                   (cx - 0.24 * s, cy + 0.45 * s)], fill=col)
        d.rectangle([cx - 0.36 * s, cy - 0.45 * s, cx + 0.36 * s, cy - 0.33 * s], fill=accent)
        d.rectangle([cx - 0.3 * s, cy - 0.05 * s, cx + 0.3 * s, cy + 0.12 * s], fill=accent)
    else:
        d.ellipse([cx - 0.45 * s, cy - 0.45 * s, cx + 0.45 * s, cy + 0.45 * s], fill=col)
        d.ellipse([cx - 0.25 * s, cy - 0.25 * s, cx + 0.25 * s, cy + 0.25 * s], fill=accent)


def _mix(a, b, t):
    return tuple(int(a[k] * (1 - t) + b[k] * t) for k in range(3))


def draw_ad(loc, i, W, H):
    """One invented advertisement, laid out for its aspect: headline, line, wordmark and a product picture."""
    from PIL import Image, ImageDraw
    head, line, mark, kind, (a, b), fg = _ad(loc, i)
    img = Image.new("RGB", (W, H))
    gradient(img, a, b, vertical=(i % 2 == 0))
    d = ImageDraw.Draw(img)
    r = _rng(loc, i, W, H)
    jp = loc == "JP"
    hkey = "jp_sans" if jp else ("serif" if i % 3 == 1 else "sans")
    accent = _mix(fg, a, 0.35)
    col = _mix(b, (255, 255, 255), 0.55) if sum(b) < 300 else _mix(b, (0, 0, 0), 0.45)
    for _ in range(3):      # soft background discs
        rr = r.uniform(0.15, 0.45) * min(W, H)
        x, y = r.uniform(0, W), r.uniform(0, H)
        d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=_mix(a, b, r.uniform(0.2, 0.8)))
    if W >= 1.6 * H:          # wide: picture right, text left
        s = H * 0.8
        shape(d, kind, W * 0.78, H * 0.5, s, col, accent)
        vert = jp and W < 2.5 * H and i % 4 == 2
        if vert:                # a vertical Japanese headline at the left edge, the rest beside it
            vtext(d, (W * 0.04, H * 0.06, W * 0.04 + H * 0.16, H * 0.94), head, hkey, fg)
            text_in(d, (W * 0.2, H * 0.3, W * 0.6, H * 0.55), line, hkey, fg, "l")
            mbox = (W * 0.2, H * 0.72, W * 0.5, H * 0.86)
        else:
            text_in(d, (W * 0.05, H * 0.12, W * 0.6, H * 0.47), head, hkey, fg, "l")
            text_in(d, (W * 0.05, H * 0.52, W * 0.55, H * 0.68), line, hkey, fg, "l")
            mbox = (W * 0.05, H * 0.76, W * 0.45, H * 0.9)
        text_in(d, mbox, mark, "sans" if not _has_jp(mark) else "jp_sans", fg, "l")
    else:                       # tall or square: picture in the middle
        s = min(W, H) * 0.62
        shape(d, kind, W * 0.5, H * 0.52, s, col, accent)
        text_in(d, (W * 0.07, H * 0.05, W * 0.93, H * 0.22), head, hkey, fg)
        text_in(d, (W * 0.1, H * 0.78, W * 0.9, H * 0.86), line, hkey, fg)
        text_in(d, (W * 0.15, H * 0.88, W * 0.85, H * 0.96), mark, "sans" if not _has_jp(mark) else "jp_sans", fg)
    return img


KANBAN_STYLES = [((250, 250, 248), (20, 20, 22), (200, 20, 30)), ((255, 214, 0), (25, 20, 15), (25, 20, 15)),
                 ((190, 20, 30), (255, 255, 255), (255, 214, 0)), ((20, 60, 150), (255, 255, 255), (255, 255, 255)),
                 ((20, 20, 24), (255, 230, 150), (220, 30, 40)), ((0, 120, 80), (255, 255, 255), (255, 255, 255))]
FASCIA_STYLES = {
    "NL": [((20, 40, 70), (235, 225, 200), "serif"), ((120, 20, 30), (245, 240, 230), "times"),
           ((240, 238, 230), (30, 30, 35), "sans"), ((25, 70, 45), (235, 220, 160), "serif_it"),
           ((15, 15, 18), (225, 190, 90), "times"), ((200, 120, 40), (255, 255, 255), "sans")],
    "ES": [((120, 25, 20), (250, 235, 200), "serif"), ((240, 230, 205), (90, 30, 20), "times"),
           ((20, 70, 110), (255, 255, 255), "sans"), ((30, 30, 30), (230, 200, 120), "serif_it"),
           ((0, 110, 70), (255, 255, 255), "sans"), ((200, 160, 60), (40, 20, 10), "serif")],
    "DE": [((30, 30, 32), (230, 210, 150), "serif"), ((245, 242, 235), (20, 20, 20), "times"),
           ((150, 20, 25), (255, 255, 255), "sans"), ((40, 70, 50), (240, 230, 200), "serif_it"),
           ((20, 50, 90), (255, 255, 255), "sans"), ((210, 200, 180), (60, 40, 20), "serif")],
    "FI": [((20, 50, 90), (255, 255, 255), "sans"), ((245, 245, 245), (20, 30, 50), "sans"),
           ((60, 60, 65), (240, 220, 160), "serif"), ((0, 100, 80), (255, 255, 255), "sans"),
           ((140, 30, 40), (255, 240, 220), "times"), ((230, 220, 200), (40, 40, 40), "serif")],
    "EN": [((15, 15, 18), (255, 255, 255), "sans"), ((200, 20, 30), (255, 255, 255), "sans"),
           ((20, 60, 120), (255, 215, 120), "serif"), ((245, 245, 240), (20, 20, 20), "times"),
           ((0, 90, 60), (240, 220, 160), "serif_it"), ((250, 200, 0), (20, 20, 20), "sans")],
}


def draw_fascia(loc, kind, v, W=1024, H=None):
    from PIL import Image, ImageDraw
    H = H or (228 if loc == "EN" else 192)
    trade, name = FASCIA[loc][kind][v]
    styles = FASCIA_STYLES[loc]
    bg, fg, key = styles[zlib.crc32(f"{loc}{kind}{v}".encode()) % len(styles)]
    if kind == "pharmacy" and loc in ("ES", "DE", "FI", "NL"):
        bg, fg = (0, 120, 70), (255, 255, 255)
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)
    d.rectangle([5, 5, W - 6, H - 6], outline=fg, width=4)
    x0 = 24
    if kind == "pharmacy":      # the green cross of a European pharmacy (a public sign, no brand)
        c, s = (H * 0.55, H / 2), H * 0.34
        cross = (255, 255, 255) if bg[1] > bg[0] else (0, 150, 80)
        d.rectangle([c[0] - s, c[1] - s / 3, c[0] + s, c[1] + s / 3], fill=cross)
        d.rectangle([c[0] - s / 3, c[1] - s, c[0] + s / 3, c[1] + s], fill=cross)
        x0 = int(H * 1.05)
    if v == 0:      # trade big, name smaller under it
        text_in(d, (x0, H * 0.1, W - 24, H * 0.62), trade, key, fg)
        text_in(d, (x0 + 40, H * 0.66, W - 64, H * 0.9), name, "light" if key == "sans" else key, fg)
    else:           # name big, trade small on the left
        text_in(d, (x0, H * 0.18, W * 0.32, H * 0.82), trade, "light" if key == "sans" else key, fg)
        d.line([(W * 0.34, H * 0.2), (W * 0.34, H * 0.8)], fill=fg, width=3)
        text_in(d, (W * 0.37, H * 0.1, W - 24, H * 0.9), name, key, fg)
    return img


def draw_kanban(kind, v, W=1024, H=256):
    from PIL import Image, ImageDraw
    name, sub = KANBAN[kind][v]
    bg, fg, band = KANBAN_STYLES[zlib.crc32(f"jp{kind}{v}".encode()) % len(KANBAN_STYLES)]
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 14], fill=band)
    d.rectangle([0, H - 14, W, H], fill=band)
    key = "jp_serif" if kind in ("sushi", "izakaya", "books", "ramen") and v == 0 else "jp_sans"
    # a round crest with the name's first character (as many shops have; generic, not a logo of anyone)
    first = next(c for c in name if c != " ")
    d.ellipse([22, 34, 22 + H - 68, H - 34], fill=band)
    text_in(d, (40, 52, H - 64, H - 52), first, key, bg)
    text_in(d, (H - 20, 26, W - 30, H * 0.68), name, key, fg)
    text_in(d, (H + 20, H * 0.7, W - 70, H - 22), sub, "jp_sans", fg)
    return img


def draw_tenant(v, W=256, H=2560):
    from PIL import Image, ImageDraw
    floors = TENANTS[v]
    img = Image.new("RGB", (W, H), (240, 240, 236))
    d = ImageDraw.Draw(img)
    cols = [((250, 250, 248), (20, 20, 22)), ((200, 20, 30), (255, 255, 255)), ((255, 214, 0), (20, 20, 20)),
            ((20, 60, 150), (255, 255, 255)), ((20, 20, 24), (255, 230, 150)), ((0, 120, 80), (255, 255, 255))]
    ph = H / len(floors)
    r = _rng("tenant", v)
    order = list(range(len(cols)))
    r.shuffle(order)
    for n, (fl, word) in enumerate(floors):
        bg, fg = cols[order[n % len(cols)]]
        y0, y1 = n * ph + 6, (n + 1) * ph - 6
        d.rectangle([8, y0, W - 8, y1], fill=bg)
        text_in(d, (24, y0 + 10, W - 24, y0 + 90), fl, "sans", fg)
        vtext(d, (30, y0 + 110, W - 30, y1 - 20), word, "jp_sans", fg)
    return img


def draw_nobori(v, W=192, H=768):
    from PIL import Image, ImageDraw
    text, bg, fg = NOBORI[v]
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)
    band = fg if bg != (250, 245, 235) else (200, 30, 30)
    d.rectangle([0, 0, W, 40], fill=band)
    d.rectangle([0, H - 30, W, H], fill=band)
    vtext(d, (18, 64, W - 18, H - 50), text, "jp_sans", fg)
    return img


def draw_stand(v, W=256, H=640):
    from PIL import Image, ImageDraw
    text, bg, fg = STAND[v]
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)
    d.rectangle([6, 6, W - 7, H - 7], outline=fg, width=6)
    vtext(d, (30, 36, W - 30, H - 110), text, "jp_sans", fg)
    text_in(d, (60, H - 100, W - 60, H - 24), "↑" if v % 2 else "→", "jp_sans", fg)
    return img


def draw_aboard(loc, v, W=384, H=640):
    from PIL import Image, ImageDraw
    lines = ABOARD[loc][v]
    img = Image.new("RGB", (W, H), (110, 75, 45))
    d = ImageDraw.Draw(img)
    d.rectangle([22, 22, W - 22, H - 22], fill=(28, 36, 32))
    r = _rng("aboard", loc, v)
    for _ in range(140):    # chalk dust
        x, y = r.uniform(26, W - 26), r.uniform(26, H - 26)
        d.point((x, y), fill=(70, 78, 74))
    chalk = (238, 236, 225)
    text_in(d, (40, 50, W - 40, 170), lines[0], "serif_it", chalk)
    d.line([(60, 190), (W - 60, 190)], fill=chalk, width=3)
    text_in(d, (40, 230, W - 40, 360), lines[1], "serif_it", (250, 220, 150))
    text_in(d, (40, 400, W - 40, 540), lines[2], "serif_it", chalk)
    return img


def draw_screen(loc, k, W=768, H=432):
    """Four ad frames stacked (a vertical pan loops through them, as the generic screens)."""
    from PIL import Image
    atlas = Image.new("RGB", (W, 4 * H))
    for j in range(4):
        atlas.paste(draw_ad(loc, k * 3 + j, W, H), (0, j * H))
    return atlas


def draw_column(loc, v, W=1024, H=512):
    from PIL import Image, ImageDraw
    posters = COLUMN[loc][v]
    img = Image.new("RGB", (W, H), (228, 226, 218))
    d = ImageDraw.Draw(img)
    r = _rng("column", loc, v)
    for p, (big, small) in enumerate(posters):
        x0 = p * W // 3
        bg = (r.randint(30, 220), r.randint(30, 220), r.randint(30, 220))
        d.rectangle([x0 + 6, 14, x0 + W // 3 - 6, H - 14], fill=bg)
        fg = (255, 255, 255) if sum(bg) < 380 else (20, 20, 20)
        for _ in range(3):
            cx, cy, rr = x0 + r.randint(40, 300), r.randint(160, 360), r.randint(30, 100)
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=(r.randint(0, 255), r.randint(0, 255), r.randint(0, 255)))
        text_in(d, (x0 + 20, 30, x0 + W // 3 - 20, 110), big, "sans", fg)
        text_in(d, (x0 + 24, 430, x0 + W // 3 - 24, 480), small, "light", fg)
    return img


def draw(item):
    kind = item["draw"][0]
    a = item["draw"][1:]
    return {"fascia": draw_fascia, "kanban": draw_kanban, "tenant": draw_tenant, "nobori": draw_nobori,
            "stand": draw_stand, "aboard": draw_aboard, "ad": draw_ad, "screen": draw_screen,
            "column": draw_column}[kind](*a)


def pot(n, most=2048):
    """The power of two at or above n (at most `most`)."""
    return min(most, 1 << max(0, math.ceil(math.log2(max(1, n)))))


def textures(out, items=None):
    """Every catalogue item's face as T_CC_<tex>.png in out (RGB, opaque). Drawn at the face's own aspect, then
    stretched to power-of-two sides (the UVs span the whole texture, so it renders at its aspect again): Unreal makes
    mipmaps only for power-of-two textures, and without them a billboard or sign seen from afar shimmers. Returns
    the paths."""
    from PIL import Image
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    for it in items or catalog():
        p = out / f"T_CC_{it['tex']}.png"
        img = draw(it)
        size = (pot(img.width), pot(img.height))
        if size != img.size:
            img = img.resize(size, Image.LANCZOS)
        img.save(p, optimize=True)
        paths.append(p)
    return paths


def contact_sheet(paths, dest, cell=(320, 200), cols=8):
    """All faces small on one sheet, with their names (to look at them)."""
    from PIL import Image, ImageDraw
    rows = math.ceil(len(paths) / cols)
    sheet = Image.new("RGB", (cols * cell[0], rows * (cell[1] + 18)), (60, 60, 64))
    d = ImageDraw.Draw(sheet)
    for n, p in enumerate(paths):
        im = Image.open(p).convert("RGB")
        im.thumbnail(cell)
        x, y = (n % cols) * cell[0], (n // cols) * (cell[1] + 18)
        sheet.paste(im, (x + (cell[0] - im.width) // 2, y + (cell[1] - im.height) // 2))
        d.text((x + 4, y + cell[1] + 2), Path(p).stem[5:][:44], font=font("light", 12), fill=(230, 230, 230))
    sheet.save(dest)
    return dest


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0] if args else "build/signage")
    paths = textures(out)
    print(f"{len(paths)} faces in {out}")
    if "--sheet" in sys.argv:
        i = sys.argv.index("--sheet")
        dest = Path(sys.argv[i + 1]) if i + 1 < len(sys.argv) else out / "sheet.png"
        loc = [a for a in sys.argv if a.startswith("--loc=")]
        sel = [p for p in paths if not loc or f"sig_{loc[0][6:].lower()}_" in p.name]
        print(contact_sheet(sel, dest))
