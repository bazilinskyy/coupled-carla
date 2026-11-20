"""One report from a frame-budget GPU window: tools/frame_budget.py and tools/megalights_check.py output folders.

    python tools/frame_budget_report.py ~/cc58/ue5power/20261119-2353            # A_epic, B_megalights, C_lamps

Prints (and writes report.md in the folder): per level, case and camera spec the server fps, 1 % low and camera fps
against 60 fps; the cost of window geometry (TUeCampus -> TUeCampusWin, same case and camera); MegaLights on vs
off at night with lamps; the street-light census per radius; and the best camera setting per case with its
headroom. No CARLA needed.
"""
import json
import sys
from pathlib import Path

NEED = 60.0


def load(folder):
    rows = []
    for f in sorted(Path(folder).glob("*/frame_budget.json")):
        for r in json.loads(f.read_text()):
            r["_run"] = f.parent.name
            rows.append(r)
    lamps = []
    for f in sorted(Path(folder).glob("*/megalights.json")):
        lamps += json.loads(f.read_text())
    return rows, lamps


def cam_fps(r):
    c = r.get("camera_fps") or []
    return min(c) if c else None


def report(rows, lamps):
    ok = [r for r in rows if "error" not in r]
    out = ["## Frame budget (participant camera, asynchronous server)", "",
           "| run | variant | level | case | camera | server fps | 1 % low | camera fps | lamps lit | >= 60 |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for r in ok:
        out.append(f"| {r['_run']} | {r['variant']} | {r['level']} | {r['case']} | {r.get('cam', '')} | "
                   f"{r['server_fps']} | {r['low_1pct_fps']} | {cam_fps(r)} | {r.get('lamps_lit', '')} | "
                   f"{'yes' if (cam_fps(r) or 0) >= NEED and r['server_fps'] >= NEED else 'no'} |")
    errs = [r for r in rows if "error" in r]
    if errs:
        out += ["", "Errors: " + "; ".join(f"{r.get('variant')}/{r.get('level')}/{r.get('case')}/{r.get('cam')}: "
                                             f"{r['error']}" for r in errs)]
    # window geometry cost
    key = {}
    for r in ok:
        key[(r["_run"], r["variant"], r["case"], r.get("cam"), r["level"])] = r
    cost = []
    for (run, var, case, cam, lvl), r in key.items():
        if lvl != "TUeCampusWin":
            continue
        b = key.get((run, var, case, cam, "TUeCampus"))
        if b:
            cost.append((case, cam, b["server_fps"], r["server_fps"], cam_fps(b), cam_fps(r)))
    if cost:
        out += ["", "## Window geometry cost (TUeCampus -> TUeCampusWin)", "",
                "| case | camera | server fps | camera fps |", "|---|---|---|---|"]
        for case, cam, s0, s1, c0, c1 in cost:
            out.append(f"| {case} | {cam} | {s0} -> {s1} ({100 * (s1 - s0) / s0:+.0f} %) | {c0} -> {c1} |")
    # MegaLights
    ml = [r for r in ok if r["case"].startswith("night_r")]
    if ml:
        out += ["", "## MegaLights at night with lamps", "", "| variant | level | case | server fps | 1 % low | "
                "camera fps | lamps lit | street lights on |", "|---|---|---|---|---|---|---|---|"]
        for r in ml:
            out.append(f"| {r['variant']} | {r['level']} | {r['case']} | {r['server_fps']} | {r['low_1pct_fps']} | "
                       f"{cam_fps(r)} | {r.get('lamps_lit', '')} | {r.get('street_lights_on', '')}/"
                       f"{r.get('street_lights', '')} |")
    if lamps:
        out += ["", "## Street lights per radius", "", "| radius m | lamps placed | lit | street lights on | "
                "register flux set | road brightness off -> on | camera fps |", "|---|---|---|---|---|---|---|"]
        for r in lamps:
            f = r.get("fps") or {}
            out.append(f"| {int(r['radius_m'])} | {r['lamps_placed']} | {r['lamps_lit']} | {r['street_lights_on']}/"
                       f"{r['street_lights']} | {r.get('register_specs_applied')} | {r.get('brightness_off')} -> "
                       f"{r.get('brightness_on')} (x{r.get('lit_gain')}) | {cam_fps(f) if f else ''} |")
    # best camera per (level, case)
    best = {}
    for r in ok:
        k = (r["level"], r["case"])
        if cam_fps(r) is not None and (k not in best or cam_fps(r) > cam_fps(best[k])):
            best[k] = r
    if best:
        out += ["", "## Best camera setting per case", "", "| level | case | camera | camera fps | short of 60 by |",
                "|---|---|---|---|---|"]
        for (lvl, case), r in sorted(best.items()):
            out.append(f"| {lvl} | {case} | {r.get('cam')} | {cam_fps(r)} | {max(0.0, NEED - cam_fps(r)):.1f} |")
    return "\n".join(out) + "\n"


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        sys.exit(__doc__)
    folder = Path(argv[0]).expanduser()
    rows, lamps = load(folder)
    text = report(rows, lamps)
    (folder / "report.md").write_text(text)
    print(text)


if __name__ == "__main__":
    main()
