"""Which levels would show CARLA's stock (US) road signs in a country that is not the US: per level, the stock sign
actors a server spawns from the level's OpenDRIVE, the OpenDRIVE file it reads, and stock sign meshes baked into
the level, against the level's country (coupledcarla/localsigns.py has the designs).

    python tools/sign_check.py content <Content dir> [--levels A,B] [--towns] [--json out.json]
        offline, no server: a CARLA 1.0 tree's (or package source's) Content folder. For each level under
        Carla/Maps/Twins (and the towns with --towns): the .xodr files of its OpenDrive folder and the one an
        editor -game server reads (CARLA's UOpenDrive::LoadXODR takes the first *.xodr of that folder: a backup
        copy beside the level's own was read on 24-27 Nov 2026), the one a packaged server reads
        (Carla/Maps/OpenDrive/<N>.xodr: the level folders are not staged), the stop (206), give-way (205) and
        speed-limit (274) signals in each, and level files that reference stock sign meshes or blueprints.
    python tools/sign_check.py maps <maps dir> [--json out.json]
        generated maps (coupled-carla mapbuild: <map>/map.xodr and the country found for it).
    python tools/sign_check.py server [--host H] [--port P] [--levels A,B] [--replace] [--json out.json]
        a running server: loads each level, lists its stock sign actors (traffic.stop, traffic.yield,
        traffic.speed_limit.*; get_actors and get_static_actors), and with --replace runs localsigns.replace
        (what a coupled-carla trial does) and lists them again.

A level is a PROBLEM when a server would show a stock sign whose design is not its country's (speed and give-way
plates anywhere but the US family, the stop octagon in Japan) and coupled-carla would not replace it, or when its
OpenDrive folder holds more than one .xodr (the server may read the wrong one). Exit status 1 when any level is.
Pure Python 3.9+ for content/maps (it runs on win5090 and lin5080 as is); server mode needs the carla module.
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("localsigns", ROOT / "coupledcarla" / "localsigns.py")
ls = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ls)

# the twin levels' countries (ISO3; a level's place.json wins: carla-digitaltwins twinmodel writes it)
LEVEL_COUNTRY = {"TUeCampus": "NLD", "TUeCampusWin": "NLD", "EhvCentre": "NLD", "EhvCentreFacades": "NLD",
                 "EhvCity": "NLD", "AmsterdamDam": "NLD", "NycMidtown": "USA", "HelsinkiSenate": "FIN",
                 "SalamancaPlaza": "ESP", "ShibuyaCrossing": "JPN", "ZurichHB": "CHE", "Eixample": "ESP"}
ISO2_3 = {"NL": "NLD", "US": "USA", "FI": "FIN", "ES": "ESP", "JP": "JPN", "CH": "CHE", "DE": "DEU", "FR": "FRA",
          "GB": "GBR", "BE": "BEL", "AT": "AUT", "IT": "ITA", "UA": "UKR", "RW": "RWA", "CA": "CAN", "PT": "PRT",
          "PL": "POL", "CZ": "CZE", "SE": "SWE", "NO": "NOR", "DK": "DNK", "IE": "IRL", "LU": "LUX", "GR": "GRC",
          "AU": "AUS", "NZ": "NZL", "KR": "KOR", "CN": "CHN", "TW": "TWN", "IN": "IND", "BR": "BRA", "MX": "MEX",
          "ZA": "ZAF", "KE": "KEN", "PH": "PHL", "SG": "SGP", "PR": "PRI", "LR": "LBR"}
TOWNS = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD", "Town01_Opt", "Town02_Opt",
         "Town03_Opt", "Town04_Opt", "Town05_Opt", "Town06_Opt", "Town07_Opt", "Town10HD_Opt"]
# stock sign assets a level could have baked in (CARLA 0.9.16 and 1.0 names)
STOCK_ASSETS = [b"BP_SpeedLimit", b"BP_Stop01", b"BP_Yield01", b"SM_SpeedLimit", b"SM_SpeedSign", b"SM_Stop01",
                b"SM_stopSign", b"SM_Yield01", b"SM_yield_", b"SM_DoNoEnter"]
ASSET_KIND = {b"BP_SpeedLimit": "speed", b"SM_SpeedLimit": "speed", b"SM_SpeedSign": "speed", b"BP_Stop01": "stop",
              b"SM_Stop01": "stop", b"SM_stopSign": "stop", b"BP_Yield01": "give_way", b"SM_Yield01": "give_way",
              b"SM_yield_": "give_way", b"SM_DoNoEnter": "no_entry"}
KIND_OF_TYPE = {"274": "speed", "205": "give_way", "206": "stop"}


def iso3_of(code):
    c = (code or "").upper()
    return ISO2_3.get(c, c if len(c) == 3 else None) if c else None


def level_country(name, level_dir=None, signal_country=None):
    """(ISO3, where from) of a level: its place.json, the table, CARLA's towns (USA), the signals' country."""
    if level_dir is not None:
        p = Path(level_dir) / "place.json"
        try:
            c = iso3_of(json.loads(p.read_text()).get("country"))
            if c:
                return c, "place.json"
        except (OSError, ValueError, AttributeError):
            pass
    if name in LEVEL_COUNTRY:
        return LEVEL_COUNTRY[name], "table"
    if name.startswith("Town") or name == "Mine_01":
        return "USA", "CARLA town (North American)"
    c = iso3_of(signal_country) if signal_country and signal_country.upper() != "OPENDRIVE" else None
    return c, ("signals" if c else "unknown")


def editor_read(files, name):
    """(file read today, file read with the fix) among a level folder's .xodr files: CARLA's LoadXODR takes the
    first file listed (NTFS lists by name: SalamancaPlaza.prebake_signs.xodr before SalamancaPlaza.xodr; ext4 in
    hash order, by name here); with the own-xodr-first fix (unreal/patches/OpenDrive_OwnXodrFirst.patch) it takes
    <name>.xodr."""
    if not files:
        return None, None
    own = [f for f in files if f.name.lower() == f"{name}.xodr".lower()]
    first = sorted(files, key=lambda f: f.name.lower())[0]
    return first, (own[0] if own else first)


def wrong(counts, iso3):
    """{kind: n} of stock signs among counts {"274": n, ...} whose design is not country iso3's."""
    out = {}
    for t, kind in KIND_OF_TYPE.items():
        n = counts.get(t, 0)
        if n and not ls.stock_fits(kind, iso3):
            out[kind] = n
    return out


def scan_assets(paths):
    """{asset name: files} of stock sign assets referenced by the level files."""
    hits = {}
    for p in paths:
        try:
            data = p.read_bytes()
        except OSError:
            continue
        for a in STOCK_ASSETS:
            if a in data:
                hits[a.decode()] = hits.get(a.decode(), 0) + 1
    return hits


def level_files(content, rel, name):
    """The level's .umap and its World Partition actor files (Content/Carla/__ExternalActors__/Carla/Maps/...)."""
    d = content / rel
    out = [p for p in d.glob("*.umap") if p.stem == name or rel != "Carla/Maps"] if d.is_dir() else []
    sub = Path(rel) / name if rel == "Carla/Maps" else Path(rel)
    for top in ("__ExternalActors__", "__ExternalObjects__"):
        ext = content / "Carla" / top / sub
        if ext.is_dir():
            out += list(ext.rglob("*.uasset"))
    return out


def check_content(content, names=None, towns=False, scan=True):
    content = Path(content)
    maps = content / "Carla" / "Maps"
    rows = []
    levels = []
    tw = maps / "Twins"
    if tw.is_dir():
        levels += [("Carla/Maps/Twins/" + d.name, d.name) for d in sorted(tw.iterdir()) if d.is_dir()]
    if towns:     # CARLA 1.0 keeps the towns' .umap in Maps/ (0.9.16: Maps/<Town>/<Town>.umap)
        levels += [("Carla/Maps", t) for t in TOWNS if (maps / f"{t}.umap").exists()]
        levels += [("Carla/Maps/" + t, t) for t in TOWNS if (maps / t / f"{t}.umap").exists()]
    for rel, name in levels:
        if names and name not in names:
            continue
        ldir = content / rel
        if rel == "Carla/Maps":             # a town: Maps/<Town>.umap, its OpenDRIVE in Maps/OpenDrive
            files = [maps / "OpenDrive" / f"{name}.xodr"] if (maps / "OpenDrive" / f"{name}.xodr").exists() else []
            ldir = None
        else:
            files = sorted((ldir / "OpenDrive").glob("*.xodr")) if (ldir / "OpenDrive").is_dir() else []
        now, fixed = editor_read(files, name)
        pkg = maps / "OpenDrive" / f"{name}.xodr"
        per_file = {f.name: ls.xodr_signs(f) for f in files}
        e_counts = per_file.get(now.name, {}) if now else {}
        p_counts = ls.xodr_signs(pkg) if pkg.exists() else {}
        sc = e_counts.get("country") or p_counts.get("country")
        iso3, how = level_country(name, ldir, sc)
        assets = scan_assets(level_files(content, rel, name)) if scan else {}
        bad_assets = {a: n for a, n in assets.items()
                      if not ls.stock_fits(ASSET_KIND[next(k for k in ASSET_KIND if k.decode() == a)], iso3)}
        row = {"level": name, "country": iso3, "country_from": how,
               "xodr_files": {f: {k: v for k, v in c.items() if k != "country"} for f, c in per_file.items()},
               "editor_reads": now.name if now else None, "editor_reads_fixed": fixed.name if fixed else None,
               "editor_signs": {k: v for k, v in e_counts.items() if k != "country"},
               "package_reads": pkg.name if pkg.exists() else None,
               "package_signs": {k: v for k, v in p_counts.items() if k != "country"},
               "level_assets": assets}
        row["wrong_editor"] = wrong(e_counts, iso3)
        row["wrong_editor_fixed"] = wrong(per_file.get(fixed.name, {}), iso3) if fixed else {}
        row["wrong_package"] = wrong(p_counts, iso3)
        row["wrong_assets"] = bad_assets
        row["ambiguous_xodr"] = len(files) > 1
        row["problem"] = bool(row["wrong_editor"] or row["wrong_package"] or bad_assets or row["ambiguous_xodr"])
        rows.append(row)
    return rows


def check_maps(maps_dir):
    rows = []
    for d in sorted(Path(maps_dir).iterdir()):
        x = d / "map.xodr"
        if not x.exists():
            continue
        iso3 = None
        try:
            iso3 = json.loads((d / "real" / "crowdpeds_country.json").read_text()).get("iso3")
        except (OSError, ValueError, AttributeError):
            pass
        c = ls.xodr_signs(x)
        iso3 = iso3 or iso3_of(c.get("country") if (c.get("country") or "").upper() != "OPENDRIVE" else None)
        w = wrong(c, iso3)
        rows.append({"level": d.name, "country": iso3, "editor_reads": "map.xodr (sent by coupled-carla)",
                     "editor_signs": {k: v for k, v in c.items() if k != "country"}, "wrong_editor": w,
                     "replaced_at_runtime": bool(w),     # localsigns.replace in every trial
                     "problem": bool(w) and not iso3})
    return rows


def check_server(host, port, names, replace=False, timeout=600.0):
    import carla
    sys.path.insert(0, str(ROOT))
    from coupledcarla import localsigns as L     # replace() needs the package (catalogue, faces, lanes)
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    rows = []
    if not names:
        names = [client.get_world().get_map().name.rsplit("/", 1)[-1]]
    for name in names:
        world = client.get_world()
        if world.get_map().name.rsplit("/", 1)[-1] != name:
            world = client.load_world(name)
        world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
        iso3, how = level_country(name)
        before = [(a.id, t) for a, t in ls.stock_actors(world)]
        plan = ls.plan(before, iso3, {}, "auto")
        row = {"level": name, "country": iso3, "country_from": how, "stock_actors": _by_type(before),
               "wrong": {k: n for k, n in _by_type([(a, t) for a, t in before
                                                    if not ls.stock_fits(ls.kind_of(t)[0], iso3)]).items()}}
        if replace:
            row["replace"] = L.replace(client, world, iso3, log=print)
            world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
            after = [(a.id, t) for a, t in ls.stock_actors(world)]
            row["stock_actors_after"] = _by_type(after)
            row["wrong_after"] = _by_type([(a, t) for a, t in after if not ls.stock_fits(ls.kind_of(t)[0], iso3)])
            row["faces_after"] = sum(1 for a in world.get_actors().filter("static.prop.mesh")
                                     if "/CoupledCarla/Signs/" in a.attributes.get("mesh_path", ""))
        row["problem"] = bool(row.get("wrong_after", row["wrong"]))
        row["plan"] = sorted({(k, v, act) for _, k, v, act, _ in plan})
        rows.append(row)
    return rows


def _by_type(pairs):
    out = {}
    for _, t in pairs:
        out[t] = out.get(t, 0) + 1
    return dict(sorted(out.items()))


def table(rows):
    lines = []
    for r in rows:
        s = (f"{'PROBLEM' if r['problem'] else 'ok':8s} {r['level']:18s} {r.get('country') or '?':4s} "
             f"editor {r.get('editor_reads') or '-'} {r.get('editor_signs', r.get('stock_actors', {}))}")
        if "package_reads" in r:
            s += f" | package {r['package_reads'] or '-'} {r['package_signs']}"
        if r.get("level_assets"):
            s += f" | level meshes {r['level_assets']}"
        flags = []
        if r.get("ambiguous_xodr"):
            flags.append(f"{len(r['xodr_files'])} .xodr in the level's OpenDrive folder: {r['xodr_files']}; "
                         f"with the own-xodr-first fix: {r['editor_reads_fixed']}")
        for k in ("wrong_editor", "wrong_editor_fixed", "wrong_package", "wrong_assets", "wrong", "wrong_after"):
            if r.get(k):
                flags.append(f"{k}: {r[k]}")
        if r.get("replace"):
            flags.append(f"replaced {r['replace']['replaced']}, removed {r['replace']['removed']}, "
                         f"kept {r['replace']['kept']}")
        lines.append(s + ("\n         " + "; ".join(flags) if flags else ""))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("content")
    c.add_argument("content")
    c.add_argument("--levels", default="")
    c.add_argument("--towns", action="store_true")
    c.add_argument("--no-scan", action="store_true", help="skip the level files")
    m = sub.add_parser("maps")
    m.add_argument("maps")
    s = sub.add_parser("server")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=2000)
    s.add_argument("--levels", default="")
    s.add_argument("--replace", action="store_true")
    for p in (c, m, s):
        p.add_argument("--json")
    a = ap.parse_args(argv)
    names = [n for n in getattr(a, "levels", "").split(",") if n]
    if a.cmd == "content":
        rows = check_content(a.content, names, a.towns, not a.no_scan)
    elif a.cmd == "maps":
        rows = check_maps(a.maps)
    else:
        rows = check_server(a.host, a.port, names, a.replace)
    print(table(rows))
    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=1, default=str))
    return 1 if any(r["problem"] for r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
