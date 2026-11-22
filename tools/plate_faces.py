"""Country number plates for CARLA 1.0 vehicles: plate textures in each country's format, drawn from code.

CARLA's vehicles share one plate material (MI_LicensePlate: a 512 x 512 texture with the plate in its middle band,
x 10-502, y 148-385, a California "CARLA" plate). This draws, per country, NUM plates with random registrations in
that band, so the plate mesh keeps its UVs:

* NL: yellow, black characters, the EU band (12 stars, "NL"), the sidecodes of today's cars (XX-999-X, 9-XXX-99, ...);
* ES: white, EU band "E", 9999 XXX (consonants only); FI: white, black rim, EU band "FIN", XXX-999;
* DE: white, EU band "D", district code, two seals, letters and digits (M AB 1234); FR: white, blue bands left (EU,
  "F") and right (department number), AB-123-CD;
* CH: the long rear plate as photographed (Commons, St. Gallen 2007): Swiss arms, canton code, a dot at mid height,
  the number in groups of three, the canton's arms; white, black characters, no border, no EU band;
* JP: white, green characters and rim, two rows: the issuing office and class (品川 300), a kana and the number
  (あ 12-34); US: New York's current plate (white to gold, blue NEW YORK, ABC-1234, EXCELSIOR).

Each registration is drawn three times: T_PlateF / T_PlateR, the front and rear plate alone at their real
proportions (SIZES_CM: 520 x 110 mm in most of Europe, the Swiss front 300 x 80 mm without arms and rear 500 x
110 mm with them, Finland 442 x 118, Japan 330 x 165, New York 305 x 152), which the server mounts at the real size
in front of the vehicle's own plate (turned into a dark holder); and T_Plate, the plate in the band of CARLA's plate
texture, for vehicles whose mesh the server cannot read. Characters: Barlow Condensed
SemiBold (SIL Open Font License, --latin-font; DejaVu Sans Bold without it), fitted to each format's width;
Japanese: Noto Sans JP (SIL Open Font License, --jp-font). The server puts them on vehicles whose blueprint attribute license_plate names the country
(CARLA LicensePlates.cpp); tools/ue_plates.py imports them as /Game/CoupledCarla/Plates/<CC>/MI_Plate_<CC>_NN.

    python tools/plate_faces.py OUT_DIR [--num 16] [--jp-font NotoSansJP.ttf] [--countries NL ES ...]
"""
import argparse
import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SIZE = 512
BAND = (10, 148, 502, 385)          # CARLA's plate rectangle in the texture (x0, y0, x1, y1)
_SIDE = "band"                      # band (CARLA's plate texture) | front | rear (a plate of its real size)
_ASPECT = None                      # the plate's width / height for this side (None: 520 x 110 mm)
_RECT = None                        # the plate drawn last (x0, y0, x1, y1), cropped for front / rear
# real plate sizes (cm, width x height) per country and side: what the server mounts (CARLA LicensePlates.cpp)
SIZES_CM = {"NL": {"front": (52.0, 11.0), "rear": (52.0, 11.0)}, "DE": {"front": (52.0, 11.0), "rear": (52.0, 11.0)},
            "ES": {"front": (52.0, 11.0), "rear": (52.0, 11.0)}, "FR": {"front": (52.0, 11.0), "rear": (52.0, 11.0)},
            "FI": {"front": (44.2, 11.8), "rear": (44.2, 11.8)},
            "CH": {"front": (30.0, 8.0), "rear": (50.0, 11.0)},       # front: short, no arms; rear: long, arms
            "JP": {"front": (33.0, 16.5), "rear": (33.0, 16.5)}, "US": {"front": (30.5, 15.2), "rear": (30.5, 15.2)}}
HOLDER = (34, 35, 37)               # plate holder around a long plate
EU_BLUE = (0, 51, 153)
EU_YELLOW = (255, 204, 0)
NL_YELLOW = (247, 190, 0)
BLACK = (18, 18, 18)
WHITE = (246, 246, 242)
CONSONANTS_NL = "BDFGHJKLNPRSTVXZ"
CONSONANTS_ES = "BCDFGHJKLMNPRSTVWXYZ"
LETTERS = "ABCDEFGHJKLMNPRSTUVWXYZ"
JP_OFFICES = ["品川", "練馬", "足立", "横浜", "名古屋", "大阪", "京都", "神戸", "札幌", "福岡"]
JP_KANA = "さすせそたちつてとなにぬねのはひふほまみむめもやゆよらりるれろわ"
CH_CANTONS = ["ZH"] * 9 + ["LU", "ZG", "SZ", "SO", "FR", "TI", "VD"]   # cantons whose arms are drawn below
DE_DISTRICTS = ["M", "B", "HH", "K", "S", "F", "D", "DO", "AC", "KA"]


def font_path(latin=None):
    """The plate face: --latin-font (Barlow Condensed SemiBold, SIL OFL: a condensed DIN-like grotesque, as the
    European plate faces are), else DejaVu Sans Bold (shipped with matplotlib) condensed to width."""
    if latin:
        return latin
    import matplotlib
    return str(Path(matplotlib.get_data_path()) / "fonts" / "ttf" / "DejaVuSans-Bold.ttf")


def text_block(text, height, width, colour, path, spacing=0):
    """The text rendered `height` px tall (cap height) and squeezed or stretched to `width` px (RGBA)."""
    f = ImageFont.truetype(path, int(height * 1.38))
    l, t, r, b = f.getbbox(text)
    img = Image.new("RGBA", (r - l + 4, b - t + 4), (0, 0, 0, 0))
    ImageDraw.Draw(img).text((2 - l, 2 - t), text, font=f, fill=colour)
    return img.resize((max(1, int(width)), max(1, int(height))), Image.LANCZOS)


def paste_center(canvas, img, cx, cy):
    canvas.alpha_composite(img, (int(cx - img.width / 2), int(cy - img.height / 2)))


def eu_band(draw, canvas, x0, y0, w, h, code, fp):
    draw.rectangle([x0, y0, x0 + w, y0 + h], fill=EU_BLUE)
    cx, cy, r = x0 + w / 2, y0 + h * 0.33, w * 0.30
    for k in range(12):
        a = 2 * math.pi * k / 12
        sx, sy = cx + r * math.cos(a), cy + r * math.sin(a)
        s = max(1.2, w * 0.055)
        draw.polygon([(sx + s * math.cos(math.pi / 2 + i * 2 * math.pi / 5 * 2),
                       sy - s * math.sin(math.pi / 2 + i * 2 * math.pi / 5 * 2)) for i in range(5)], fill=EU_YELLOW)
    paste_center(canvas, text_block(code, h * 0.27, w * (0.42 * len(code) if len(code) < 3 else 0.86), WHITE, fp),
                 cx, y0 + h * 0.78)


def long_plate(bg, rim=None):
    """A canvas with the holder and an empty long plate: (canvas, draw, (x0, y0, x1, y1) of the plate)."""
    c = Image.new("RGBA", (SIZE, SIZE), HOLDER + (255,))
    d = ImageDraw.Draw(c)
    global _RECT
    x0, x1 = BAND[0], BAND[2]
    h = (x1 - x0) / (_ASPECT or 4.73)
    cy = (BAND[1] + BAND[3]) / 2
    y0, y1 = cy - h / 2, cy + h / 2
    k = SIZE / 512
    d.rounded_rectangle([x0, y0, x1, y1], radius=6 * k, fill=bg, outline=rim or bg, width=int(3 * k) if rim else 1)
    _RECT = (x0, y0, x1, y1)
    return c, d, (x0, y0, x1, y1)


def nl(rng, fp, jp=None):
    sc = rng.choice(["XX-999-X", "9-XXX-99", "99-XXX-9", "XXX-99-X", "X-999-XX", "XX-99-XX"])
    reg = "".join(rng.choice(CONSONANTS_NL) if ch == "X" else str(rng.randint(0, 9)) if ch == "9" else ch for ch in sc)
    c, d, (x0, y0, x1, y1) = long_plate(NL_YELLOW)
    h = y1 - y0
    bw = h * 0.42
    eu_band(d, c, x0 + 2, y0 + 2, bw, h - 4, "NL", fp)
    paste_center(c, text_block(reg, h * 0.66, (x1 - x0 - bw) * 0.86, BLACK, fp), x0 + bw + (x1 - x0 - bw) / 2, (y0 + y1) / 2)
    return c, reg


def es(rng, fp, jp=None):
    reg = "%04d %s" % (rng.randint(0, 9999), "".join(rng.choice(CONSONANTS_ES) for _ in range(3)))
    c, d, (x0, y0, x1, y1) = long_plate(WHITE, rim=BLACK)
    h = y1 - y0
    bw = h * 0.42
    eu_band(d, c, x0 + 3, y0 + 3, bw, h - 6, "E", fp)
    paste_center(c, text_block(reg, h * 0.62, (x1 - x0 - bw) * 0.84, BLACK, fp), x0 + bw + (x1 - x0 - bw) / 2, (y0 + y1) / 2)
    return c, reg


def fi(rng, fp, jp=None):
    reg = "%s-%d" % ("".join(rng.choice(LETTERS) for _ in range(3)), rng.randint(1, 999))
    c, d, (x0, y0, x1, y1) = long_plate(WHITE, rim=BLACK)
    h = y1 - y0
    bw = h * 0.42
    eu_band(d, c, x0 + 3, y0 + 3, bw, h - 6, "FIN", fp)
    paste_center(c, text_block(reg, h * 0.62, (x1 - x0 - bw) * 0.74, BLACK, fp), x0 + bw + (x1 - x0 - bw) / 2, (y0 + y1) / 2)
    return c, reg


def de(rng, fp, jp=None):
    dist = rng.choice(DE_DISTRICTS)
    letters = "".join(rng.choice(LETTERS) for _ in range(rng.randint(1, 2)))
    num = str(rng.randint(1, 9999))
    c, d, (x0, y0, x1, y1) = long_plate(WHITE, rim=BLACK)
    h = y1 - y0
    bw = h * 0.42
    eu_band(d, c, x0 + 3, y0 + 3, bw, h - 6, "D", fp)
    seal = h * 0.36
    avail = (x1 - x0 - bw) * 0.9
    unit = h * 0.5
    want = unit * (len(dist) + len(letters) + len(num) + 0.6) + seal + h * 0.3
    k = min(1.0, (avail - seal - h * 0.3) / (want - seal - h * 0.3))
    left = text_block(dist, h * 0.62, unit * k * len(dist), BLACK, fp)
    right = text_block(letters + " " + num, h * 0.62, unit * k * (len(letters) + len(num) + 0.6), BLACK, fp)
    total = left.width + seal + right.width + h * 0.3
    sx = x0 + bw + ((x1 - x0 - bw) - total) / 2
    paste_center(c, left, sx + left.width / 2, (y0 + y1) / 2)
    scx = sx + left.width + h * 0.15 + seal / 2
    d.ellipse([scx - seal / 2, y0 + h * 0.2, scx + seal / 2, y0 + h * 0.2 + seal], fill=(70, 120, 200), outline=BLACK)
    d.ellipse([scx - seal / 2, y0 + h * 0.6, scx + seal / 2, y0 + h * 0.6 + seal * 0.6], fill=(200, 40, 40))
    paste_center(c, right, sx + left.width + seal + h * 0.3 + right.width / 2, (y0 + y1) / 2)
    return c, f"{dist} {letters} {num}"


def fr(rng, fp, jp=None):
    reg = "%s-%03d-%s" % ("".join(rng.choice(LETTERS) for _ in range(2)), rng.randint(1, 999),
                          "".join(rng.choice(LETTERS) for _ in range(2)))
    c, d, (x0, y0, x1, y1) = long_plate(WHITE, rim=BLACK)
    h = y1 - y0
    bw = h * 0.42
    eu_band(d, c, x0 + 3, y0 + 3, bw, h - 6, "F", fp)
    d.rectangle([x1 - bw - 3, y0 + 3, x1 - 3, y1 - 3], fill=EU_BLUE)
    paste_center(c, text_block(str(rng.choice([69, 75, 13, 33, 59])), h * 0.3, bw * 0.7, WHITE, fp), x1 - bw / 2 - 3, y0 + h * 0.72)
    paste_center(c, text_block(reg, h * 0.6, (x1 - x0 - 2 * bw) * 0.86, BLACK, fp), (x0 + x1) / 2, (y0 + y1) / 2)
    return c, reg


CH_RED, CH_BLUE, CH_BLACK, CH_GREEN, CH_YELLOW = (213, 43, 30), (0, 94, 170), (20, 20, 20), (0, 130, 70), (250, 200, 0)


def shield_mask(w, h):
    """A heraldic shield (flat top, round point at the bottom) as an L mask w x h."""
    m = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(m)
    d.rectangle([0, 0, w - 1, int(h * 0.55)], fill=255)
    d.pieslice([0, int(h * 0.1) - int(h * 0.45), w - 1, h - 1], 0, 180, fill=255)
    return m


def canton_arms(code, w, h):
    """The arms of a canton (simplified blazon) on a w x h shield, RGBA."""
    img = Image.new("RGBA", (w, h), WHITE + (255,))
    d = ImageDraw.Draw(img)
    if code == "CH":                                     # gules, a cross couped argent
        d.rectangle([0, 0, w, h], fill=CH_RED)
        a, cx, cy = w * 0.12, w / 2, h * 0.43
        d.rectangle([cx - a, cy - 3.2 * a, cx + a, cy + 3.2 * a], fill=WHITE)
        d.rectangle([cx - 3.2 * a, cy - a, cx + 3.2 * a, cy + a], fill=WHITE)
    elif code == "ZH":                                   # per bend argent and azure
        d.polygon([(0, 0), (w, h), (0, h)], fill=CH_BLUE)
    elif code == "LU":                                   # per pale azure and argent
        d.rectangle([0, 0, w / 2, h], fill=CH_BLUE)
    elif code == "ZG":                                   # argent, a fess azure
        d.rectangle([0, h * 0.36, w, h * 0.64], fill=CH_BLUE)
    elif code == "SZ":                                   # gules, a couped cross argent in the canton
        d.rectangle([0, 0, w, h], fill=CH_RED)
        a, cx, cy = w * 0.05, w * 0.26, h * 0.2
        d.rectangle([cx - a, cy - 3 * a, cx + a, cy + 3 * a], fill=WHITE)
        d.rectangle([cx - 3 * a, cy - a, cx + 3 * a, cy + a], fill=WHITE)
    elif code == "SO":                                   # per fess gules and argent
        d.rectangle([0, 0, w, h / 2], fill=CH_RED)
    elif code == "FR":                                   # per fess sable and argent
        d.rectangle([0, 0, w, h / 2], fill=CH_BLACK)
    elif code == "TI":                                   # per pale gules and azure
        d.rectangle([0, 0, w / 2, h], fill=CH_RED)
        d.rectangle([w / 2, 0, w, h], fill=CH_BLUE)
    elif code == "VD":                                   # per fess argent and vert
        d.rectangle([0, h * 0.45, w, h], fill=CH_GREEN)
    img.putalpha(shield_mask(w, h))
    out = Image.new("RGBA", (w + 2, h + 2), (0, 0, 0, 0))
    edge = Image.new("RGBA", (w + 2, h + 2), (60, 60, 60, 255))
    edge.putalpha(shield_mask(w + 2, h + 2))
    out.alpha_composite(edge)
    out.alpha_composite(img, (1, 1))
    return out


def ch(rng, fp, jp=None):
    """The long rear plate (500 x 110 mm, as on the car: the front plate has no arms): the Swiss arms, the canton,
    a dot at mid height, the number in groups of three, the canton's arms; white, black, embossed rim, no border."""
    canton = rng.choice(CH_CANTONS)
    num = str(rng.randint(1, 999999) if rng.random() < 0.15 else rng.randint(10000, 999999))
    c, d, (x0, y0, x1, y1) = long_plate(WHITE, rim=(205, 205, 200))
    h, w = y1 - y0, x1 - x0
    cap = h * 0.70
    if _SIDE == "front":                                # the front plate: canton and number only
        tx0, tx1 = x0 + w * 0.05, x1 - w * 0.05
    else:
        sw, sh = int(h * 0.62), int(h * 0.76)
        sy = int(y0 + (h - sh) / 2)
        c.alpha_composite(canton_arms("CH", sw, sh), (int(x0 + w * 0.028), sy))
        c.alpha_composite(canton_arms(canton, sw, sh), (int(x1 - w * 0.028 - sw - 2), sy))
        tx0, tx1 = x0 + w * 0.028 + sw + w * 0.02, x1 - w * 0.028 - sw - w * 0.02
    groups = [num[max(0, k - 3):k] for k in range(len(num), 0, -3)][::-1]
    parts = [canton, "·"] + groups
    unit = cap * 0.47                                   # a character's width (the plate face is condensed)
    widths = [unit * len(canton), cap * 0.32] + [unit * len(g) for g in groups]
    gaps = [cap * 0.12, cap * 0.12] + [cap * 0.18] * (len(groups) - 1)
    total = sum(widths) + sum(gaps[:len(widths) - 1])
    k = min(1.0, (tx1 - tx0) / total)
    x = (tx0 + tx1) / 2 - total * k / 2
    cy = (y0 + y1) / 2
    for idx, (part, pw) in enumerate(zip(parts, widths)):
        pw *= k
        if part == "·":
            r = cap * 0.07
            d.ellipse([x + pw / 2 - r, cy - r, x + pw / 2 + r, cy + r], fill=BLACK)
        else:
            paste_center(c, text_block(part, cap, pw, BLACK, fp), x + pw / 2, cy)
        x += pw + (gaps[idx] * k if idx < len(gaps) else 0)
    return c, f"{canton}·{num}"


def jp(rng, fp, jpf):
    office, cls = rng.choice(JP_OFFICES), str(rng.randint(300, 599))
    kana, num = rng.choice(JP_KANA), rng.randint(1, 9999)
    numtxt = ("%d" % num) if num < 100 else "%2d-%02d" % (num // 100, num % 100)
    global _RECT
    green = (0, 102, 60)
    _RECT = BAND
    c = Image.new("RGBA", (SIZE, SIZE), HOLDER + (255,))
    d = ImageDraw.Draw(c)
    x0, y0, x1, y1 = BAND
    d.rounded_rectangle([x0, y0, x1, y1], radius=10, fill=WHITE, outline=green, width=6)
    h = y1 - y0
    paste_center(c, text_block(office + " " + cls, h * 0.26, (x1 - x0) * 0.52, green, jpf), (x0 + x1) / 2, y0 + h * 0.24)
    paste_center(c, text_block(kana, h * 0.24, h * 0.24, green, jpf), x0 + (x1 - x0) * 0.12, y0 + h * 0.66)
    paste_center(c, text_block(numtxt, h * 0.48, (x1 - x0) * 0.66, green, fp), x0 + (x1 - x0) * 0.58, y0 + h * 0.66)
    return c, f"{office} {cls} {kana} {numtxt}"


def us(rng, fp, jp=None):
    global _RECT
    reg = "%s-%04d" % ("".join(rng.choice(LETTERS) for _ in range(3)), rng.randint(0, 9999))
    _RECT = BAND
    c = Image.new("RGBA", (SIZE, SIZE), HOLDER + (255,))
    x0, y0, x1, y1 = BAND
    grad = Image.new("RGBA", (x1 - x0, y1 - y0))
    gd = ImageDraw.Draw(grad)
    for yy in range(y1 - y0):
        u = yy / (y1 - y0)
        col = tuple(int(a + (b - a) * max(0.0, (u - 0.45) / 0.55)) for a, b in zip(WHITE, (236, 168, 52)))
        gd.line([(0, yy), (x1 - x0, yy)], fill=col + (255,))
    mask = Image.new("L", grad.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, grad.width - 1, grad.height - 1], radius=12, fill=255)
    c.paste(grad, (x0, y0), mask)
    blue = (20, 52, 120)
    h = y1 - y0
    paste_center(c, text_block("NEW YORK", h * 0.14, (x1 - x0) * 0.42, blue, fp), (x0 + x1) / 2, y0 + h * 0.15)
    paste_center(c, text_block(reg, h * 0.42, (x1 - x0) * 0.8, blue, fp), (x0 + x1) / 2, y0 + h * 0.53)
    paste_center(c, text_block("EXCELSIOR", h * 0.09, (x1 - x0) * 0.3, blue, fp), (x0 + x1) / 2, y0 + h * 0.88)
    return c, reg


def render(cc, side, rng, fp, jpf):
    """One plate: side "band" = CARLA's 512 x 512 plate texture (the plate in its band, a holder around); "front" /
    "rear" = the plate alone at its real proportions (SIZES_CM), 1024 px wide, drawn on a 2x canvas."""
    global SIZE, BAND, _SIDE, _ASPECT
    saved = SIZE, BAND
    try:
        _SIDE = side
        if side == "band":
            _ASPECT = None
        else:
            SIZE, BAND = 1024, tuple(v * 2 for v in BAND)
            w, h = SIZES_CM[cc][side]
            _ASPECT = w / h
        img, reg = COUNTRIES[cc](rng, fp, jpf)
        if side != "band":
            x0, y0, x1, y1 = (int(round(v)) for v in _RECT)
            w, h = SIZES_CM[cc][side]
            img = img.crop((x0, y0, x1, y1)).resize((1024, int(round(1024 * h / w))), Image.LANCZOS)
        return img, reg
    finally:
        SIZE, BAND = saved
        _SIDE, _ASPECT = "band", None


COUNTRIES = {"NL": nl, "ES": es, "FI": fi, "DE": de, "FR": fr, "CH": ch, "JP": jp, "US": us}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("--num", type=int, default=16)
    ap.add_argument("--countries", nargs="*", default=list(COUNTRIES))
    ap.add_argument("--jp-font", default=None, help="a font with kanji and kana (Noto Sans JP, OFL)")
    ap.add_argument("--latin-font", default=None, help="the plate face (Barlow Condensed SemiBold, OFL)")
    ap.add_argument("--seed", type=int, default=2026)
    args = ap.parse_args(argv)
    fp = font_path(args.latin_font)
    index = {}
    for cc in args.countries:
        if cc == "JP" and not args.jp_font:
            print("JP skipped: no --jp-font")
            continue
        d = args.out / cc
        d.mkdir(parents=True, exist_ok=True)
        index[cc] = []
        for i in range(args.num):
            reg = None
            for side in ("band", "front", "rear"):
                img, reg_s = render(cc, side, random.Random(f"{args.seed}-{cc}-{i}"), fp, args.jp_font)
                reg = reg or reg_s
                name = {"band": "T_Plate", "front": "T_PlateF", "rear": "T_PlateR"}[side]
                img.convert("RGB").save(d / f"{name}_{cc}_{i:02d}.png")
            index[cc].append(reg)
        print(cc, index[cc][:4])
    Image.new("RGB", (8, 8), (28, 29, 31)).save(args.out / "T_PlateHolder.png")   # the holder (ue_plates.py)
    (args.out / "plates.json").write_text(__import__("json").dumps(
        {"registrations": index, "sizes_cm": {k: v for k, v in SIZES_CM.items() if k in index}}, ensure_ascii=False,
        indent=1))


if __name__ == "__main__":
    main()
