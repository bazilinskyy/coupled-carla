"""levelxodr.py and tools/level_xodr.py: a baked level's map dir must be built from the level's own OpenDRIVE."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from coupledcarla import levelxodr

ROOT = Path(__file__).resolve().parent.parent


def _xodr(roads=((1, 10.0, 0.0),), paving=None, geoid=False, signals=False, fmt="{:.6f}"):
    proj = "+proj=tmerc +lat_0=51.4 +lon_0=5.4" + (" +geoidgrids=nl_nlgeo2018.tif +vunits=m" if geoid else "")
    head = f'<userData code="twinPaving" value="{paving}"/>' if paving else ""
    out = [f'<?xml version="1.0"?>\n<OpenDRIVE><header revMajor="1" revMinor="4" date="x">'
           f"<geoReference><![CDATA[{proj}]]></geoReference>{head}</header>"]
    for rid, length, x in roads:
        sig = ('<signals><signal id="s%d" s="1.0" t="-2.0" type="206"/></signals>' % rid) if signals else ""
        out.append(f'<road id="{rid}" length="{fmt.format(length)}" junction="-1"><planView>'
                   f'<geometry s="0" x="{fmt.format(x)}" y="0" hdg="0" length="{fmt.format(length)}"><line/></geometry>'
                   f'</planView><lanes><laneSection s="0"><center><lane id="0" type="none"/></center>'
                   f'<right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>'
                   f'</right></laneSection></lanes>{sig}</road>')
    out.append('<junction id="100"/></OpenDRIVE>')
    return "\n".join(out)


def test_signature_ignores_format_geoid_and_signals_but_not_roads_or_paving():
    base = levelxodr.signature(_xodr(((1, 10.0, 0.0), (2, 5.0, 3.0)), paving="clinker,sett"))
    assert base["roads"] == 2 and base["junctions"] == 1 and base["paving"] == ["clinker", "sett"]
    same = levelxodr.signature(_xodr(((2, 5.0, 3.0), (1, 10.0, 0.0)), paving="sett,clinker", geoid=True,
                                     signals=True, fmt="{:.9f}"))
    assert levelxodr.differences(same, base) == []
    moved = levelxodr.signature(_xodr(((1, 10.0, 0.0), (2, 5.0, 3.2)), paving="clinker,sett"))
    d = levelxodr.differences(moved, base)
    assert len(d) == 1 and "1 roads with other geometry" in d[0] and "e.g. 2" in d[0]
    d = levelxodr.differences(levelxodr.signature(_xodr(((1, 10.0, 0.0),))), base)
    assert any("1 roads only in the level" in x for x in d) and any("paving mark none" in x for x in d)
    # sub-millimetre noise is not a difference
    assert levelxodr.differences(levelxodr.signature(_xodr(((1, 10.0002, 0.0), (2, 5.0, 3.0)),
                                                           paving="clinker,sett")), base) == []


class _World:
    def __init__(self, text):
        self.text = text

    def get_map(self):
        return SimpleNamespace(to_opendrive=lambda: self.text)


def _map(tmp_path, text):
    mdir = tmp_path / "maps" / "twin"
    mdir.mkdir(parents=True)
    (mdir / "map.xodr").write_text(text)
    return mdir


def test_check_stops_saves_the_level_copy_and_says_how_to_fix(tmp_path):
    level = _xodr(((1, 10.0, 0.0), (2, 5.0, 3.0)), paving="clinker,sett")
    cfg = {"map": {"carla_level": "Twin"}, "_path": "experiments/twin.yaml"}
    mdir = _map(tmp_path, _xodr(((1, 10.0, 0.0), (2, 5.0, 3.0)), paving="clinker,sett", signals=True))
    assert levelxodr.check(_World(level), mdir, cfg) is True
    assert not (mdir / "level.xodr").exists()
    stale = _map(tmp_path / "b", _xodr(((1, 10.0, 0.0), (277, 12.0, 0.0))))
    with pytest.raises(SystemExit) as e:
        levelxodr.check(_World(level), stale, cfg)
    msg = str(e.value)
    assert "not the OpenDRIVE of the level Twin" in msg and "277" in msg and "tools/level_xodr.py" in msg
    assert (stale / "level.xodr").read_text() == level
    logs = []
    assert levelxodr.check(_World(level), stale, {**cfg, "map": {**cfg["map"], "level_xodr": "warn"}},
                           logs.append) is False
    assert logs and logs[0].startswith("WARNING")
    assert levelxodr.check(_World(level), stale, {"map": {"carla_level": None}}) is None    # generated world


def test_stored_signature_is_used_only_for_the_same_file(tmp_path):
    mdir = _map(tmp_path, _xodr(((1, 10.0, 0.0),)))
    short = levelxodr.store_signature(mdir)
    assert set(short) == {"roads", "junctions", "paving", "hash"}
    cached = json.loads((mdir / "xodr_signature.json").read_text())
    assert levelxodr.map_signature(mdir) == cached
    (mdir / "map.xodr").write_text(_xodr(((1, 10.0, 0.0), (2, 4.0, 1.0))))     # replaced by hand: recomputed
    assert levelxodr.map_signature(mdir)["roads"] == 2


def _tool():
    spec = importlib.util.spec_from_file_location("cc_tool_level_xodr", ROOT / "tools" / "level_xodr.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_tool_refresh_keeps_the_old_source_and_rebuilds(tmp_path, monkeypatch):
    tool = _tool()
    level = _xodr(((1, 10.0, 0.0), (2, 5.0, 3.0)), paving="clinker,sett", signals=True)
    (tmp_path / "twins").mkdir()
    (tmp_path / "twins" / "Twin.xodr").write_text(_xodr(((1, 10.0, 0.0),)))
    cfg = {"_root": str(tmp_path), "_path": "x.yaml",
           "map": {"name": "twin", "opendrive": "twins/Twin.xodr", "carla_level": "Twin"}}
    monkeypatch.setattr(tool.config, "map_dir", lambda c: tmp_path / "maps" / "twin")
    before = tool.compare(cfg, level)
    assert before["source"] and before["map"] is None

    def build(c, force):
        mdir = tmp_path / "maps" / "twin"
        mdir.mkdir(parents=True, exist_ok=True)
        (mdir / "map.xodr").write_text(levelxodr.strip_geoid((tmp_path / "twins" / "Twin.xodr").read_text()))
        levelxodr.store_signature(mdir)
        assert force

    after = tool.refresh(cfg, level, build=build, log=lambda m: None)
    assert after["source"] == [] and after["map"] == []
    assert (tmp_path / "twins" / "Twin.xodr.before-xodr-refresh").read_text().count("<road ") == 1
