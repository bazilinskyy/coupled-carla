"""tools/content_pack.py: the coupled-carla content packs for the official CARLA 1.0 package (no CARLA needed)."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools import content_pack as cp  # noqa: E402


def test_street_catalog_drops_code_plugin_classes():
    cat = cp.street_catalog()
    paths = [b["Path"] for b in cat["Blueprints"]]
    assert paths and not any(p.startswith("/Script/") for p in paths)
    names = {b["Name"] for b in cat["Blueprints"]}
    assert {"cc_nl_lamparm9", "cc_drone_consumer", "cc_streetlight06_a"} <= names


def test_build_steps_order_and_maps(tmp_path):
    carla = tmp_path / "carla"
    steps = cp.build_steps(carla, tmp_path / "meta.tar.gz", tmp_path / "ue", ["EhvCentre", "TUeCampus"],
                           ["street", "twins"], tmp_path / "work")
    whats = [w for w, _ in steps]
    # street: init, catalog, holder level, cook; then per twin its own pack: init, map, attribution, cook
    assert whats[:4] == ["init " + cp.STREET, "catalog", "holder level", "cook " + cp.STREET]
    assert whats[4:8] == ["init CoupledCarlaEhvCentre", "map EhvCentre", "attribution EhvCentre",
                          "cook CoupledCarlaEhvCentre"]
    assert whats[-1] == "cook CoupledCarlaTUeCampus"
    maps = [argv for w, argv in steps if w.startswith("map ")]
    assert [m[m.index("--map") + 1] for m in maps] == ["/Game/Carla/Maps/Twins/EhvCentre/EhvCentre",
                                                       "/Game/Carla/Maps/Twins/TUeCampus/TUeCampus"]
    assert [m[m.index("add") + 1] for m in maps] == ["CoupledCarlaEhvCentre", "CoupledCarlaTUeCampus"]
    holder = dict(steps)["holder level"]
    assert "-EnablePlugins=" + cp.STREET in holder and "-NullRHI" in holder
    script = [x for x in holder if x.startswith("-script=")][0]
    assert "--also " in script and script.endswith("also.json")
    assert all("--base" in argv for w, argv in steps if w.startswith("cook "))


def test_build_steps_skip_init_of_existing_pack(tmp_path):
    carla = tmp_path / "carla"
    d = cp.pack_dir(carla, cp.STREET)
    d.mkdir(parents=True)
    (d / "carla-pack.json").write_text("{}")
    whats = [w for w, _ in cp.build_steps(carla, "m", "ue", [], ["street"], tmp_path)]
    assert whats == ["catalog", "holder level", "cook " + cp.STREET]


def test_explicitly_loaded_is_restored(tmp_path):
    up = tmp_path / "P.uplugin"
    up.write_text(json.dumps({"ExplicitlyLoaded": True, "FileVersion": 3}))
    with cp.ExplicitlyLoadedOff(up):
        assert json.loads(up.read_text())["ExplicitlyLoaded"] is False
    assert json.loads(up.read_text())["ExplicitlyLoaded"] is True


def test_install_dry_run(tmp_path, capsys):
    cp.main(["install", "A.tar.gz", "B.tar.gz", "--server", str(tmp_path), "--dry-run"])
    out = capsys.readouterr().out
    assert out.count("carla_pack.py install") == 2


def test_carla_assets_cover_signals_trees_and_lamps():
    a = cp.carla_assets()
    assert "/Game/Carla/Static/StreetLight/BP_StreetLight06_E.BP_StreetLight06_E" in a["blueprints"]
    assert all(p.startswith("/Game/Carla/") for p in a["meshes"] + a["blueprints"])
    assert any("/Vegetation/" in p for p in a["meshes"])
    signals = (cp.ROOT / "coupledcarla" / "signals.py").read_text()
    for name in ("POLE_MESH", "HEAD_MESH"):
        path = signals.split(name + ' = "', 1)[1].split('"', 1)[0]
        assert path in a["meshes"]
