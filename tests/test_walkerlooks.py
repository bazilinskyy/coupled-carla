"""Walker appearance per place (coupledcarla/walkerlooks.py, data/walker_looks.csv, data/walker_demographics.yaml)."""
import random
from collections import Counter

import pytest

from coupledcarla import walkerlooks as wl


# --- a blueprint library without CARLA -------------------------------------------------------------------------

class _Attr:
    def __init__(self, v):
        self.v = v

    def as_str(self):
        return self.v


class _BP:
    def __init__(self, bid, age="adult", gender="female", wheel=False):
        self.id, self.attrs = bid, {"age": age, "gender": gender, "can_use_wheelchair": "true" if wheel else "false"}

    def has_attribute(self, k):
        return k in self.attrs

    def get_attribute(self, k):
        return _Attr(self.attrs[k])


class _Lib:
    def __init__(self, bps):
        self.bps = {b.id: b for b in bps}

    def filter(self, pattern):
        assert pattern == "walker.pedestrian.*"
        return list(self.bps.values())

    def find(self, bid):
        return self.bps[bid]


def _lib(version):
    rows = [r for r in wl.catalogue() if r["carla"] == version]
    return _Lib([_BP(r["blueprint"], r["carla_age"], r["carla_gender"] or r["presentation"],
                     r["wheelchair"] == "true") for r in rows])


@pytest.fixture(autouse=True)
def _reset():
    yield
    wl._street, wl._cfg = None, dict(wl.DEFAULTS)
    wl._bound.clear()


# --- the data ----------------------------------------------------------------------------------------------------

def test_catalogue_rows_are_valid():
    rows = wl.catalogue()
    by_ver = Counter(r["carla"] for r in rows)
    assert by_ver["1.0"] == 37 and by_ver["0.9.16"] == 49        # every stock walker of both versions
    for ver in by_ver:
        ids = [r["blueprint"] for r in rows if r["carla"] == ver]
        assert len(ids) == len(set(ids))
    for r in rows:
        assert r["group"] in wl.GROUPS, r
        assert all(g in wl.GROUPS and g != r["group"] for g in r["also_reads_as"]), r
        assert r["skin_tone"] in (1, 2, 3, 4, 5), r
        assert r["age_group"] in ("child", "adult", "older_adult"), r
        assert r["clothing"] in ("casual", "business", "workwear", "uniform", "tourist"), r
        assert r["presentation"] in ("female", "male"), r
        assert (r["age_group"] == "child") == (r["carla_age"] == "child"), r
        assert r["crowd_weight"] > 0


def test_every_place_resolves_and_cites_sources():
    data = wl.demographics()
    assert {"tokyo_shibuya", "eindhoven", "amsterdam", "salamanca", "zurich", "helsinki", "new_york_manhattan",
            "lyon", "kigali", "chernihiv"} <= set(data["places"])
    for key, p in data["places"].items():
        s = wl.street(key)
        assert abs(sum(s["mix"].values()) - 1.0) < 1e-9, key
        assert 0.0 <= s["visitor_share"] <= wl.MAX_VISITOR_SHARE, key
        assert p["residents"].get("sources"), key           # every place names where its residents come from
        for src in p["residents"]["sources"] + (p.get("visitors") or {}).get("sources", []):
            assert src["name"] and src["url"].startswith("http") and int(src["year"]) >= 2001, (key, src)
    for name in list(data["countries"]) + list(data["regions"]) + list(data["mixes"]):
        assert abs(sum(wl.resolve(name).values()) - 1.0) < 1e-9, name


def test_place_mixes_are_plausible():
    tokyo, ehv, nyc = (wl.street(k)["mix"] for k in ("tokyo_shibuya", "eindhoven", "new_york_manhattan"))
    assert tokyo["east_asian"] > 0.9                         # mostly East Asian, with a share of others
    assert 0.03 < 1 - tokyo["east_asian"] < 0.1
    assert 0.6 < ehv["white"] < 0.8 and ehv["mena"] > 0.05
    assert 0.15 < nyc["latin_american"] < 0.3 and 0.4 < nyc["white"] < 0.6
    assert wl.street("chernihiv")["mix"]["white"] > 0.95
    assert wl.street("kigali")["mix"]["black"] > 0.95


def test_visitor_share_formula():
    v = wl.visitor_share({"foreign_nights_per_year": 365 * 100, "population": 900, "hotspot_factor": 2})
    assert v == pytest.approx(100 / 1000 * 2)
    assert wl.visitor_share({"foreign_visitors_per_year": 365, "nights": 4, "population": 1}) == wl.MAX_VISITOR_SHARE
    assert wl.visitor_share({"visitor_share": 0.3}) == 0.3
    assert wl.visitor_share(None) == 0.0


def test_unknown_group_or_origin_fails_loudly():
    with pytest.raises(ValueError):
        wl.normalise({"martian": 1.0})
    with pytest.raises(KeyError):
        wl.resolve("Atlantis")


# --- where a map is ----------------------------------------------------------------------------------------------

def test_place_at():
    assert wl.place_at(35.6595, 139.7005, "JPN")[0] == "tokyo_shibuya"
    assert wl.place_at(51.4486, 5.4907, "NLD")[0] == "eindhoven"          # TU/e campus
    assert wl.place_at(52.37, 4.89, "NLD")[0] == "amsterdam"
    assert wl.place_at(48.14, 11.58, "DEU")[0] == "country:DEU"           # Munich: Germany's mix
    assert wl.place_at(-1.29, 36.82, "KEN")[0] == "region:sub_saharan_africa"
    assert wl.place_at(0.0, -160.0, None)[0] == "generic"


def test_street_for_config_and_towns():
    assert wl.street_for({"map": {"carla_town": "Town10HD_Opt"}})["key"] == "generic"
    assert wl.street_for({"traffic": {"walkers": {"looks": {"place": "lyon"}}}})["key"] == "lyon"
    assert wl.street_for({"traffic": {"walkers": {"looks": {"place": "fra"}}}})["key"] == "country:FRA"
    got = wl.street_for({"traffic": {"walkers": {"looks": {"mix": {"white": 1, "black": 1}}}}})
    assert got["mix"]["white"] == 0.5 and got["mix"]["east_asian"] == 0.0
    tc = {"map": {"bbox": [35.655, 139.695, 35.664, 139.706]}, "city": {"country": "Japan"}}
    assert wl.street_for(tc)["key"] == "tokyo_shibuya"
    assert wl.settings({"traffic": {"walkers": {"looks": False}}})["enabled"] is False


# --- the walkers a server offers ---------------------------------------------------------------------------------

def test_catalogue_version_from_the_library():
    for ver in ("1.0", "0.9.16"):
        entries, got, untagged = wl.entries_for(wl.library_walkers(_lib(ver)))
        assert got == ver and not untagged
    lib = _lib("1.0")
    lib.bps["walker.pedestrian.0090"] = _BP("walker.pedestrian.0090")      # an imported walker nobody tagged
    entries, got, untagged = wl.entries_for(wl.library_walkers(lib))
    assert got == "1.0" and untagged == ["walker.pedestrian.0090"]
    assert [e for e in entries if e["id"] == "walker.pedestrian.0090"][0]["group"] == "mixed_unclear"


def _synthetic():
    """Two adults (female, male), one child and one wheelchair walker per group except a child for east_asian."""
    out = []
    for g in wl.GROUPS:
        out.append({"id": f"{g}_f", "group": g, "also": (), "weight": 1.0, "child": False, "wheelchair": g == "black",
                    "presentation": "female", "clothing": "casual", "special": ""})
        out.append({"id": f"{g}_m", "group": g, "also": (), "weight": 1.0, "child": False, "wheelchair": False,
                    "presentation": "male", "clothing": "casual", "special": ""})
        if g != "east_asian":
            out.append({"id": f"{g}_c", "group": g, "also": (), "weight": 1.0, "child": True, "wheelchair": False,
                        "presentation": "female", "clothing": "casual", "special": ""})
    return out


def _plans(n_units, size=1, child=False):
    walkers = []
    for _ in range(n_units):
        lead = len(walkers)
        walkers.append({"legs": []})
        for j in range(size - 1):
            m = {"legs": [], "follow": (lead, 0.75, 0.0)}
            if child and j == 0:
                m["child"], m["bp"] = True, "carla_child"
            walkers.append(m)
    return walkers


@pytest.mark.parametrize("place", ["tokyo_shibuya", "eindhoven", "new_york_manhattan", "salamanca"])
def test_sampling_meets_the_target(place):
    st = wl.street(place)
    lk = wl.Looks(_synthetic(), st, seed=3)
    lk.dress(_plans(6000))
    real, n = lk.realised()
    assert n == 6000
    for g in wl.GROUPS:
        assert abs(real[g] - st["mix"][g]) < 0.02, (g, real[g], st["mix"][g])


def test_groups_share_a_plausible_mix():
    st = wl.street("new_york_manhattan")
    lk = wl.Looks(_synthetic(), st, {"family_same_group": 0.9}, seed=5)
    walkers = lk.dress(_plans(2000, size=3))
    same = [w["look"] == walkers[w["follow"][0]]["look"] for w in walkers if "follow" in w]
    assert 0.9 <= sum(same) / len(same) <= 0.96       # 90 % same group, plus re-draws that land on it anyway
    vis = [w["visitor"] == walkers[w["follow"][0]]["visitor"] for w in walkers if "follow" in w]
    assert all(vis)                                    # a tourist group is a tourist group
    assert 0.25 < sum(w["visitor"] for w in walkers) / len(walkers) < 0.35


def test_children_without_a_child_walker_of_their_group_walk_as_adults():
    lk = wl.Looks(_synthetic(), wl.street("country:CHN"), {"family_same_group": 1.0}, seed=1)
    walkers = lk.dress(_plans(300, size=2, child=True))
    kids = [w for w in walkers if "follow" in w]
    east = [w for w in kids if w["look"] == "east_asian"]
    assert east and all(not w["child"] and w["bp"].startswith("east_asian_") and not w["bp"].endswith("_c")
                        for w in east)
    assert lk.gaps["child"] == len(east)
    others = [w for w in kids if w["look"] != "east_asian"]
    assert all(w["child"] and w["bp"].endswith("_c") for w in others)


def test_wheelchair_users_need_a_wheelchair_walker_of_their_group():
    lk = wl.Looks(_synthetic(), wl.street("kigali"), seed=2)
    walkers = [{"legs": [], "wheelchair": True, "bp": "carla_wheel", "speed_factor": 0.8} for _ in range(200)]
    lk.dress(walkers)
    for w in walkers:
        if w["look"] == "black":
            assert w["wheelchair"] and w["bp"] == "black_f"
        else:
            assert not w["wheelchair"] and "speed_factor" not in w


def test_explicit_blueprints_are_kept_and_sampling_is_repeatable():
    st = wl.street("zurich")
    a = wl.Looks(_synthetic(), st, seed=9).dress(_plans(50) + [{"legs": [], "bp": "walker.pedestrian.0030"}])
    b = wl.Looks(_synthetic(), st, seed=9).dress(_plans(50) + [{"legs": [], "bp": "walker.pedestrian.0030"}])
    assert [w["bp"] for w in a] == [w["bp"] for w in b]
    assert a[-1]["bp"] == "walker.pedestrian.0030" and "look" not in a[-1]


def test_also_reads_as_fills_a_group_without_walkers():
    entries, _, _ = wl.entries_for(wl.library_walkers(_lib("1.0")))
    lk = wl.Looks(entries, wl.street("country:TUR"), seed=4)      # mena: no 1.0 walker is tagged mena first
    lk.dress(_plans(500))
    picked = Counter(lk.group_of(b) for b in lk.picked)
    assert not lk.gaps["group"]                                     # passers-for, not mixed_unclear stand-ins
    assert picked["south_asian"] + picked["latin_american"] + picked["white"] > 400


def test_stock_1_0_catalogue_tokyo_coverage():
    """CARLA 1.0's stock walkers: Tokyo's crowd is 2 East Asian faces in 5 outfits and no East Asian child (README:
    coverage gaps)."""
    entries, _, _ = wl.entries_for(wl.library_walkers(_lib("1.0")))
    east = {e["id"] for e in entries if e["group"] == "east_asian"}
    assert len(east) == 5 and not [e for e in entries if e["group"] == "east_asian" and e["child"]]
    lk = wl.Looks(entries, wl.street("tokyo_shibuya"), seed=1)
    lk.dress(_plans(1000))
    real, _ = lk.realised()
    assert real["east_asian"] > 0.9


# --- the trial's looks -------------------------------------------------------------------------------------------

def test_choose_blueprint_uniform_without_setup_and_local_with_it():
    lib = _lib("1.0")
    rng = random.Random(1)
    ids = Counter(wl.choose_blueprint(lib, rng).id for _ in range(400))
    assert len(ids) > 25                                             # uniform over the library, as before
    wl.setup({"map": {"bbox": [35.655, 139.695, 35.664, 139.706]}, "city": {"country": "Japan"}}, None,
             log=lambda m: None)
    lk = wl.for_library(lib)
    groups = Counter(lk.group_of(wl.choose_blueprint(lib, rng).id) for _ in range(400))
    assert groups["east_asian"] > 340
    k1, k2 = wl.choose_blueprint(lib, None, key="ped_1").id, wl.choose_blueprint(lib, None, key="ped_1").id
    assert k1 == k2


def test_setup_off_and_report():
    msgs = []
    assert wl.setup({"traffic": {"walkers": {"looks": False}}}, None, log=msgs.append) is None
    assert wl.for_library(_lib("1.0")) is None and "off" in msgs[-1]
    wl.setup({"map": {"carla_town": "Town10HD_Opt"}}, None, log=msgs.append)
    lib = _lib("1.0")
    lk = wl.for_library(lib)

    class A:
        def __init__(self, t):
            self.type_id = t
    lk.pick()
    real, n = wl.report([A("walker.pedestrian.0036"), A("walker.pedestrian.0039")], log=msgs.append)
    assert n == 2 and real["east_asian"] == 0.5 and "target" in msgs[-1]


def test_every_walker_spawner_goes_through_walkerlooks():
    """A uniform rng.choice over the walker blueprints anywhere but walkerlooks.py would bypass the place's look."""
    import re
    from pathlib import Path
    pkg = Path(wl.__file__).resolve().parent
    bad = []
    for f in sorted(pkg.rglob("*.py")):
        if f.name == "walkerlooks.py":
            continue
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            if re.search(r"\.choice\((self\.)?(bps|walker_bps|walkers_bps)\)", line):
                bad.append(f"{f.name}:{i}: {line.strip()}")
    assert not bad, bad


def test_check_tool_counts_a_trial(tmp_path, capsys):
    """tools/walker_looks_check.py on a trial folder: the right CARLA version's tags (0032 is an East Asian woman on
    1.0 but a police officer on 0.9.16)."""
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "walker_looks_check", Path(wl.__file__).resolve().parents[1] / "tools" / "walker_looks_check.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.version_of(["walker.pedestrian.0032", "walker.pedestrian.0051"]) == "1.0"
    assert tool.version_of(["walker.pedestrian.0032", "walker.pedestrian.0003"]) == "0.9.16"
    assert tool.version_of(["walker.pedestrian.0032"], default="1.0") == "1.0"
    (tmp_path / "config.yaml").write_text("map: {carla_town: Town10HD_Opt}\n")
    rows = ["timestamp;frame;actor_id;kind;role;type_id"]
    rows += [f"0.0;1;{i};walker;crowd;walker.pedestrian.{b}" for i, b in enumerate(["0032", "0033", "0036", "0039"])]
    rows += ["0.0;1;99;participant;driver;vehicle.lincoln.mkz_2020"]
    (tmp_path / "frames.csv").write_text("\n".join(rows) + "\n")
    st, real, n = tool.trial(tmp_path)
    assert n == 4 and real["east_asian"] == 0.75 and real["white"] == 0.25


def test_imported_walkers_join_the_stock_catalogue():
    """walker.pedestrian.0100-0111 (carla = cc rows) are used on a server that has them, besides 1.0's stock."""
    rows = [r for r in wl.catalogue() if r["carla"] == "cc"]
    assert len(rows) == 12 and {r["blueprint"][-4:] for r in rows} == {f"{i:04d}" for i in range(100, 112)}
    lib = _lib("1.0")
    for r in rows:
        lib.bps[r["blueprint"]] = _BP(r["blueprint"], r["carla_age"], r["carla_gender"])
    entries, version, untagged = wl.entries_for(wl.library_walkers(lib))
    assert version == "1.0" and not untagged
    groups = Counter(e["group"] for e in entries if not e["child"])
    assert groups["east_asian"] == 9 and groups["mena"] == 4 and groups["south_asian"] == 4
    assert [e for e in entries if e["group"] == "mena" and e["child"]]          # a girl for MENA families
    lk = wl.Looks(entries, wl.street("tokyo_shibuya"), seed=2)
    lk.dress(_plans(3000))
    used = Counter(lk.picked)
    assert max(used.values()) / 3000 < 0.2                                     # no walker in a fifth of the crowd
    on_stock_only, _, _ = wl.entries_for(wl.library_walkers(_lib("1.0")))
    assert not [e for e in on_stock_only if e["id"].endswith(("0100", "0105"))]  # absent where the server lacks them
