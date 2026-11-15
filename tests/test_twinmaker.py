"""A CARLA 1.0 digital twin from the map maker (coupledcarla/twinmaker.py, mapmaker --twin, the page's checkbox):
the toolchain config, the twinmodel command, its progress lines and the experiment it writes. No network, no
twinmodel, no Unreal: the pipeline run is faked."""
import json
from pathlib import Path

import pytest
import yaml

from coupledcarla import mapmaker as mm
from coupledcarla import mapmaker_web as web
from coupledcarla import twinmaker as tm

REPO = Path(__file__).resolve().parent.parent


def place(**kw):
    p = {"name": "Plaza Mayor, Salamanca", "full": "Plaza Mayor, Salamanca, Castilla y León, Spain", "lat": 40.965,
         "lon": -5.664, "country": "Spain", "country_code": "ES", "settlement": "Salamanca", "kind": "road",
         "population": 144000, "source": "OpenStreetMap Nominatim"}
    p.update(kw)
    return p


@pytest.fixture
def proj(tmp_path, monkeypatch):
    root = tmp_path / "proj"
    (root / "experiments" / "cities").mkdir(parents=True)
    (root / "experiments" / "cities.yaml").write_text((REPO / "experiments" / "cities.yaml").read_text())
    monkeypatch.setattr(mm, "ROOT", root)
    monkeypatch.setattr(mm, "PLACES", root / "experiments" / "places")
    monkeypatch.setattr(mm, "CACHE", tmp_path / "cache" / "geocode.json")
    monkeypatch.setattr(tm, "PLACES", root / "experiments" / "places")
    monkeypatch.setattr(tm, "TWINS", root / "twins")

    def no_network(url, params, timeout=20):
        raise AssertionError(f"unexpected request to {url}")
    monkeypatch.setattr(mm, "_fetch_json", no_network)
    return root


def toolchain(tmp_path, bake=True):
    dt = tmp_path / "cdt"
    (dt / "TwinModel" / "twinmodel").mkdir(parents=True, exist_ok=True)
    (dt / "TwinModel" / "twinmodel" / "twin.py").write_text("")
    cfg = {"digitaltwins": str(dt), "python": "/usr/bin/env", "builds": str(tmp_path / "twins_build"),
           "max_rss_gb": 20, "no_nav_build": True}
    if bake:
        root = tmp_path / "carla"
        (root / "Unreal" / "CarlaUnreal").mkdir(parents=True, exist_ok=True)
        (root / "Unreal" / "CarlaUnreal" / "CarlaUnreal.uproject").write_text("{}")
        cfg.update(carla_root=str(root), engine=str(tmp_path / "ue"), carla_python="/usr/bin/env")
    return cfg


def write_config(tmp_path, monkeypatch, cfg):
    p = tmp_path / "twin.json"
    p.write_text(json.dumps(cfg))
    monkeypatch.setenv("COUPLED_CARLA_TWIN_CONFIG", str(p))
    return p


def test_status_without_and_with_a_toolchain(tmp_path, monkeypatch):
    monkeypatch.setenv("COUPLED_CARLA_TWIN_CONFIG", str(tmp_path / "missing.json"))
    st = tm.status()
    assert not st["configured"] and not st["ok"] and "docs/make_a_map.md" in st["how"]
    write_config(tmp_path, monkeypatch, {"digitaltwins": str(tmp_path / "nowhere")})
    st = tm.status()
    assert st["configured"] and not st["ok"] and "twin.py" in st["how"]
    write_config(tmp_path, monkeypatch, toolchain(tmp_path, bake=False))
    st = tm.status()
    assert st["ok"] and not st["bake"] and "without the bake" in st["why"]
    write_config(tmp_path, monkeypatch, toolchain(tmp_path))
    assert tm.status() == {"configured": True, "ok": True, "bake": True, "why": "ready", "how": ""}


def test_level_names():
    assert tm.level_name("plaza_mayor_salamanca") == "PlazaMayorSalamanca"
    assert tm.level_name("point_40_965n_5_664w") == "Point40965n5664w"
    assert tm.level_name("x" * 50) == "X" + "x" * 31
    assert tm.level_name("2nd_street") == "Twin2ndStreet"


def test_command(tmp_path, proj):
    p = mm.plan(size="small", found=place(), twin=True)
    cfg = toolchain(tmp_path)
    cmd = tm.command(cfg, p, "PlazaMayorSalamanca")
    assert cmd[:5] == ["/usr/bin/env", "-u", "-m", "twinmodel", "twin"]
    i = cmd.index("--bbox")
    assert [float(v) for v in cmd[i + 1:i + 5]] == p["bbox"]
    assert cmd[cmd.index("--name") + 1] == "PlazaMayorSalamanca"
    assert cmd[cmd.index("--out") + 1].endswith("twins_build/plaza_mayor_salamanca")
    assert "--bake" in cmd and cmd[cmd.index("--carla-root") + 1] == cfg["carla_root"]
    assert cmd[cmd.index("--max-rss-gb") + 1] == "20" and "--no-nav-build" in cmd
    nobake = tm.command(toolchain(tmp_path, bake=False), p, "X", force=True)
    assert "--bake" not in nobake and "--force" in nobake


def test_progress_lines():
    assert tm.parse_line("[twin] start build: Building roads") == ("start", "build", "Building roads")
    assert tm.parse_line("[twin] done buildings: 2,310 buildings from catastro") == \
        ("done", "buildings", "2,310 buildings from catastro")
    assert tm.parse_line("[twin] advice: Try again later.") == ("advice", None, "Try again later.")
    assert tm.parse_line("12:00:01 twinmodel.cli INFO profile: eu_dense") is None


def fake_runner(out_dir, rc=0, lines=None):
    def run(cmd, cfg, log_path, on_line):
        name = cmd[cmd.index("--name") + 1]
        default = ["[twin] done place: Plaza Mayor -> PlazaMayorSalamanca",
                   "[twin] start build: Building roads", "[twin] done build: terrain ign_wcs 790-812 m, photo ign_es",
                   "[twin] start export: x", "[twin] done export: 312 meshes",
                   "[twin] start buildings: x", "[twin] done buildings: 2,310 buildings from catastro",
                   "[twin] start ground: x", "[twin] done ground: 16 ground tiles",
                   "[twin] start credits: x", "[twin] done credits: 4 sources",
                   "[twin] start bake: x", "[twin] done bake: 401 actors"]
        for line in (lines if lines is not None else default):
            on_line(line + "\n")
        if rc == 0:
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / f"{name}.xodr").write_text("<OpenDRIVE/>")
            (out_dir / "ATTRIBUTION.md").write_text("# credits\n")
            (out_dir / "twin.json").write_text(json.dumps({
                "name": name, "level": f"/Game/Carla/Maps/Twins/{name}/{name}", "xodr": str(out_dir / f"{name}.xodr"),
                "buildings": 2310, "building_source": "catastro", "imagery": "ign_es", "elevation": "ign_wcs",
                "bake": {"level_saved": True}}))
        return rc
    return run


def test_make_twin_writes_the_experiment(tmp_path, proj, monkeypatch):
    p = mm.plan(size="small", found=place(), twin=True)
    assert p["experiment"] == "experiments/places/plaza_mayor_salamanca_twin.yaml"
    assert p["minutes"] >= mm.estimate_minutes(1.0) + tm.TWIN_MINUTES
    cfg = toolchain(tmp_path)
    out = tm.build_dir(cfg, "PlazaMayorSalamanca")
    events = []
    path = tm.make_twin(p, events.append, cfg=cfg, runner=fake_runner(out))
    assert path == proj / "experiments" / "places" / "plaza_mayor_salamanca_twin.yaml"
    assert (proj / "twins" / "PlazaMayorSalamanca.xodr").read_text() == "<OpenDRIVE/>"
    text = path.read_text(encoding="utf-8")
    assert "2310 real buildings from catastro" in text and "ATTRIBUTION.md" in text
    exp = yaml.safe_load(text)
    assert exp["map"]["opendrive"] == "twins/PlazaMayorSalamanca.xodr"
    assert exp["map"]["carla_level"] == "PlazaMayorSalamanca" and exp["map"]["bbox"] == p["bbox"]
    assert exp["name"] == "plaza_mayor_salamanca_twin" and exp["city"]["country_code"] == "ES"
    done = {e["step"]: e["text"] for e in events if e["state"] == "done"}
    assert done["twin_buildings"] == "2,310 buildings from catastro" and "twin_bake" in done
    from coupledcarla import config
    cfg_loaded = config.load(path)
    assert cfg_loaded["map"]["carla_level"] == "PlazaMayorSalamanca"
    write_config(tmp_path, monkeypatch, cfg)
    lines = tm.summary_lines(exp, None)
    assert lines[0].startswith("  Digital twin: CARLA 1.0 level PlazaMayorSalamanca")
    assert any("2,310 real buildings" in x for x in lines) and any("ATTRIBUTION.md" in x for x in lines)


def test_make_twin_failure_in_plain_words(tmp_path, proj):
    p = mm.plan(size="small", found=place(), twin=True)
    cfg = toolchain(tmp_path)
    events = []
    lines = ["[twin] start build: x", "[twin] failed build: exit code 2; the log is /x/twin.log",
             "[twin] failed twin: The road network could not be built.", "[twin] advice: See /x/twin.log."]
    with pytest.raises(tm.TwinError) as e:
        tm.make_twin(p, events.append, cfg=cfg, runner=fake_runner(tm.build_dir(cfg, "X"), rc=2, lines=lines))
    assert "road network" in str(e.value) and "See /x/twin.log" in e.value.advice
    assert any(ev["step"] == "twin_build" and ev["state"] == "failed" for ev in events)
    with pytest.raises(tm.TwinError):
        tm.make_twin(p, events.append, cfg={"digitaltwins": str(tmp_path / "none")})


def test_mapmaker_twin_cli_and_flow(tmp_path, proj, monkeypatch):
    write_config(tmp_path, monkeypatch, toolchain(tmp_path))
    built = []
    monkeypatch.setattr(mm, "search", lambda q, limit=1: [place()])
    monkeypatch.setattr(mm, "settlement_population", lambda pl: 144000)
    monkeypatch.setattr(tm, "_run", fake_runner(tm.build_dir(tm.load_config(), "PlazaMayorSalamanca")))
    monkeypatch.setattr(mm, "build", lambda exp, emit=None, force=False, check=True: built.append((exp, check)))
    monkeypatch.setattr(mm, "check_setup", lambda: [])
    rc = mm.main(["Plaza Mayor, Salamanca", "--twin", "--size", "small"])
    exp = proj / "experiments" / "places" / "plaza_mayor_salamanca_twin.yaml"
    assert rc == 0 and built == [(exp, False)] and exp.exists()
    assert not (proj / "experiments" / "places" / "plaza_mayor_salamanca.yaml").exists()   # no OSM map with --twin
    monkeypatch.setenv("COUPLED_CARLA_TWIN_CONFIG", str(tmp_path / "missing.json"))
    assert mm.main(["Plaza Mayor, Salamanca", "--twin"]) == 2                            # plain refusal


def test_web_twin_status_and_plan(tmp_path, proj, monkeypatch):
    import threading
    import urllib.request
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), BaseHTTPRequestHandler)
    port = httpd.server_address[1]
    httpd.RequestHandlerClass = web.make_handler(web.Jobs(), port)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        monkeypatch.setenv("COUPLED_CARLA_TWIN_CONFIG", str(tmp_path / "missing.json"))
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/twin", timeout=10) as r:
            assert json.loads(r.read())["configured"] is False
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=10) as r:
            page = r.read().decode()
        assert 'id="twinRow"' in page and "/api/twin" in page and "twin: $(\"twin\").checked" in page
        monkeypatch.setattr(mm, "settlement_population", lambda pl: 144000)
        p = web.make_plan({"found": place(), "size": "small", "twin": True})
        assert p["twin"] and p["experiment"].endswith("_twin.yaml")
        write_config(tmp_path, monkeypatch, toolchain(tmp_path))
        jobs = web.Jobs()
        monkeypatch.setattr(mm, "make_twin_map", lambda p, emit, force=False: (_ for _ in ()).throw(
            mm.MapMakerError("stop here", "")))
        job = jobs.start({"found": place(), "size": "small", "twin": True})
        assert [s["step"] for s in job["steps"]][:3] == ["place", "twin_build", "twin_export"]
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_twin_env_drops_sumo_proj(monkeypatch):
    monkeypatch.setenv("PROJ_LIB", "/sumo/data/proj")
    monkeypatch.setenv("GDAL_DATA", "/x")
    env = tm.twin_env()
    assert "PROJ_LIB" not in env and "GDAL_DATA" not in env and env["PYTHONUNBUFFERED"] == "1"
