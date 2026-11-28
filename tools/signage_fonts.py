"""Subsets of Noto Sans CJK JP Bold and Noto Serif CJK JP Bold with only the characters the generated signage draws.

    python tools/signage_fonts.py /usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc \\
        /usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc

The full fonts are 20-27 MB collections; the subsets (unreal/StreetProps/fonts, about 100 KB each) let any machine,
Windows included, draw the Japanese signs (tools/signage_art.py) without a CJK font installed. Noto CJK is under the
SIL Open Font Licence 1.1 (unreal/StreetProps/fonts/OFL.txt), which allows subsetting and embedding; the subsets keep
the fonts' names with "signage" added (no reserved font name is claimed). Run it again after adding Japanese text to
signage_art (tests/test_signage.py fails on any character a subset lacks).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import signage_art  # noqa: E402

LATIN = "".join(chr(c) for c in range(0x20, 0x7f)) + "・〜、。『』%"


def jp_face(ttc):
    """The index of the Japanese face of a Noto CJK collection."""
    from fontTools.ttLib import TTCollection
    for i, f in enumerate(TTCollection(str(ttc)).fonts):
        names = " ".join(str(r) for r in f["name"].names if r.nameID in (1, 4))
        if " JP" in names:
            return i
    raise SystemExit(f"{ttc}: no JP face")


def subset(ttc, dest, text):
    from fontTools import subset as fs
    from fontTools.ttLib import TTFont
    font = TTFont(str(ttc), fontNumber=jp_face(ttc))
    opts = fs.Options()
    opts.layout_features = ["*"]
    opts.name_IDs = ["*"]
    opts.notdef_outline = True
    sub = fs.Subsetter(opts)
    sub.populate(text="".join(sorted(set(text))))
    sub.subset(font)
    for rec in font["name"].names:      # mark it as a derived subset
        if rec.nameID in (1, 4, 6):
            rec.string = str(rec) + (" signage" if rec.nameID != 6 else "-signage")
    dest.parent.mkdir(parents=True, exist_ok=True)
    font.save(str(dest))
    return dest


if __name__ == "__main__":
    sans, serif = sys.argv[1], sys.argv[2]
    text = "".join(signage_art.jp_texts()) + LATIN
    for src, key in ((sans, "jp_sans"), (serif, "jp_serif")):
        p = subset(src, signage_art.JP_FONTS[key], text)
        print(f"{p}: {p.stat().st_size / 1e3:.0f} kB, {len(set(text))} characters")
