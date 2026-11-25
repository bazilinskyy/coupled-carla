"""Per-place camera profiles (look.place_profile / resolve_auto) and tools/install_profiles.py; no CARLA."""
import importlib.util
from pathlib import Path

from coupledcarla import look

TABLE = {"levels": {"TUeCampus": {"profile": "CoupledReal_TUeCampus", "status": "provisional"}},
         "countries": {"NL": {"profile": "CoupledReal", "status": "fitted"}}, "fallback": "Dashcam"}


def _tool():
    p = Path(__file__).resolve().parents[1] / "tools" / "install_profiles.py"
    spec = importlib.util.spec_from_file_location("install_profiles_tool", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_place_profile_by_level_then_country_then_fallback():
    assert look.place_profile("TUeCampus", "NL", TABLE) == ("CoupledReal_TUeCampus", "provisional")
    assert look.place_profile("EhvCentre", "nl", TABLE) == ("CoupledReal", "fitted")
    assert look.place_profile("ShibuyaCrossing", "JP", TABLE) == ("Dashcam", "fallback")
    assert look.place_profile(None, None, {}) == ("Dashcam", "fallback")


def test_auto_is_opt_in_and_reaches_every_camera():
    said = []
    tc = {"simulation": {"camera_profile": "Dashcam", "capture": {"profile": None}}, "map": {"carla_level": "TUeCampus"}}
    assert look.resolve_auto(tc, said.append, TABLE) is None and tc["simulation"]["camera_profile"] == "Dashcam"
    tc = {"simulation": {"camera_profile": "auto", "capture": {"profile": "auto"}}, "film": {"camera_profile": "auto"},
          "map": {"carla_level": "TUeCampus"}}
    assert look.resolve_auto(tc, said.append, TABLE) == "CoupledReal_TUeCampus"
    assert tc["simulation"]["camera_profile"] == tc["simulation"]["capture"]["profile"] == "CoupledReal_TUeCampus"
    assert tc["film"]["camera_profile"] == "CoupledReal_TUeCampus"
    assert "provisional" in said[-1]
    tc = {"simulation": {"camera_profile": "auto"}, "map": {}, "city": {"country_code": "nl"}}
    assert look.resolve_auto(tc, said.append, TABLE) == "CoupledReal"
    assert "install_profiles" in said[-1]                    # one of ours: say it must be on the server


def test_ours_and_the_shipped_table():
    assert look.ours("CoupledReal") and look.ours("CoupledClean")
    assert not look.ours("Dashcam") and not look.ours("NoSuchProfile") and not look.ours("")
    name, status = look.place_profile("NoSuchLevel", "XX")      # the shipped places.json falls back to Dashcam
    assert (name, status) == ("Dashcam", "fallback")
    for lvl in ("TUeCampus", "EhvCentre", "SalamancaPlaza", "ShibuyaCrossing"):
        name, status = look.place_profile(lvl)
        assert status in ("provisional", "fitted") and (look.ours(name) or name in look.STOCK), (lvl, name)


def test_install_copies_only_ours_and_never_touches_carlas(tmp_path):
    tool = _tool()
    content = tmp_path / "Content"
    pp = tool.target_dir(content=content)
    pp.mkdir(parents=True)
    (pp / "Dashcam.json").write_text("{}")                     # a stock profile stays as it is
    said = []
    first = tool.install(pp, check=True, log=said.append)
    assert first and set(first.values()) == {"new"} and not (pp / "CoupledReal.json").exists()
    assert tool.install(pp, log=said.append) == first
    assert (pp / "CoupledReal.json").read_bytes() == (look.PROFILES / "CoupledReal.json").read_bytes()
    assert (pp / "Dashcam.json").read_text() == "{}" and not (pp / "places.json").exists()
    assert set(tool.install(pp, log=said.append).values()) == {"same"}
    assert tool.target_dir(package=tmp_path / "Pkg") == tmp_path / "Pkg/CarlaUnreal/Content/Carla/Config/PostProcess"
    assert tool.main(["--content", str(content), "--check"]) == 0
