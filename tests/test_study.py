"""coupledcarla.study: balanced Latin squares, full counterbalancing, conditions and per-participant plans."""
import itertools
from collections import Counter

import yaml

from coupledcarla import config, study


def test_williams_rows_match_coupled_sim_and_balance():
    # coupled-sim's Counterbalance.WilliamsRow(4, r): offsets 0, 1, 3, 2 from the start r
    assert [study.williams_row(4, r) for r in range(4)] == [[0, 1, 3, 2], [1, 2, 0, 3], [2, 3, 1, 0], [3, 0, 2, 1]]
    for n in range(2, 8):
        rows = [study.williams_row(n, r) for r in range(study.square_rows(n, "latin"))]
        assert all(sorted(r) == list(range(n)) for r in rows)
        reps = len(rows) // n
        for pos in range(n):            # each condition in each position equally often
            assert Counter(r[pos] for r in rows) == {c: reps for c in range(n)}
        pairs = Counter((r[i], r[i + 1]) for r in rows for i in range(n - 1))
        assert len(pairs) == n * (n - 1) and len(set(pairs.values())) == 1    # carry-over balanced
    assert study.williams_row(3, 3) == study.williams_row(3, 0)[::-1]           # odd n: mirrored second half
    assert study.williams_row(4, 4) == study.williams_row(4, 0)                 # wraps


def test_full_counterbalancing_lists_every_order():
    perms = [tuple(study.nth_permutation(4, k)) for k in range(24)]
    assert perms == list(itertools.permutations(range(4)))
    assert study.nth_permutation(4, 24) == [0, 1, 2, 3]


def _cfg(tmp_path, doc):
    p = tmp_path / "exp.yaml"
    p.write_text(yaml.safe_dump({"name": "demo", "map": {"bbox": [51.0, 5.0, 51.1, 5.1]}, **doc}, sort_keys=False))
    return config.load(p)


def test_hand_written_blocks_with_practice_and_final(tmp_path):
    cfg = _cfg(tmp_path, {
        "conditions": {"rain": {"simulation": {"weather": "HardRainNoon"}}, "night": {"simulation": {"sun_altitude": -20}}},
        "trials": [{"duration_s": 20, "name": "practice"},
                   {"block": "A", "condition": "rain"}, {"block": "A", "duration_s": 30},
                   {"block": "B", "conditions": ["rain", "night"]}, {"block": "C"}, {"block": "D"},
                   {"name": "final"}]})
    p0 = study.plan(cfg, 0)
    assert [t["_index"] for t in p0] == list(range(7))                   # participant 0: the file's order
    p2 = study.plan(cfg, 2)                                              # Williams row 1 of 4 blocks: B C A D
    assert [t["_index"] for t in p2] == [0, 3, 4, 1, 2, 5, 6]
    assert [t["_position"] for t in p2] == list(range(7))
    assert p2[1]["_condition"] == "rain+night" and p2[1]["_block"] == "B"
    tc = config.trial_config(cfg, p2[1])
    assert tc["simulation"]["weather"] == "HardRainNoon" and tc["simulation"]["sun_altitude"] == -20
    assert study.plan(cfg, 2, "none") == [dict(t, _participant=2) for t in p0]
    r1, r2 = study.plan(cfg, 5, "random"), study.plan(cfg, 5, "random")
    assert [t["_index"] for t in r1] == [t["_index"] for t in r2]           # repeatable per participant
    assert r1[0]["_index"] == 0 and r1[-1]["_index"] == 6


def test_design_crosses_factors_and_patches_agents(tmp_path):
    cfg = _cfg(tmp_path, {
        "agents": [{"id": "av1", "route": [[51.01, 5.01], [51.02, 5.02]], "ehmi": "manual", "speed_kmh": 30},
                   {"id": "car2", "route": [[51.03, 5.03], [51.04, 5.04]]}],
        "conditions": {
            "ehmi_on": {"patch": {"agents": {"av1": {"ehmi": "auto"}}}},
            "ehmi_off": {"patch": {"agents": {"av1": {"ehmi": "off", "speed_kmh": 25}}}},
            "day": {"simulation": {"weather": "ClearNoon"}}, "dusk": {"simulation": {"weather": "ClearSunset"}}},
        "design": {"factors": {"ehmi": ["ehmi_on", "ehmi_off"], "light": ["day", "dusk"]}, "repeat": 2,
                   "order": "full", "trial": {"duration_s": 45},
                   "practice": [{"conditions": ["ehmi_off", "day"], "duration_s": 15}]}})
    trials = study.file_trials(cfg)
    assert len(trials) == 1 + 4 * 2
    assert trials[0]["_block"] is None and trials[0]["duration_s"] == 15
    assert [t["_condition"] for t in trials[1:3]] == ["ehmi_on+day"] * 2
    assert {t["duration_s"] for t in trials[1:]} == {45}
    plans = [study.plan(cfg, p) for p in range(1, 25)]
    orders = {tuple(t["_block"] for t in pl[1::2]) for pl in plans}
    assert len(orders) == 24                                              # every order of the 4 blocks once
    for pl in plans:                                                     # a block's repeats stay together
        assert all(pl[i]["_block"] == pl[i + 1]["_block"] for i in range(1, 9, 2))
    tc = config.trial_config(cfg, trials[5])                            # ehmi_off+day
    av1 = next(a for a in tc["agents"] if a["id"] == "av1")
    assert av1["ehmi"] == "off" and av1["speed_kmh"] == 25 and av1["route"] == [[51.01, 5.01], [51.02, 5.02]]
    assert next(a for a in tc["agents"] if a["id"] == "car2") == cfg["agents"][1]
    assert cfg["agents"][0]["ehmi"] == "manual"                          # the experiment itself is not changed


def test_block_by_keeps_one_factor_together(tmp_path):
    cfg = _cfg(tmp_path, {
        "conditions": {"a": {}, "b": {}, "x": {}, "y": {}, "z": {}},
        "design": {"factors": {"vehicle": ["a", "b"], "gap": ["x", "y", "z"]}, "block_by": "vehicle"}})
    pl = study.plan(cfg, 2)                     # 2 blocks, Williams row 1: b first
    assert [t["_condition"] for t in pl] == ["b+x", "b+y", "b+z", "a+x", "a+y", "a+z"]
    rows = study.session_rows(pl)
    assert rows[0] == {"participant": 2, "position": 0, "trial": 3, "block": "b", "condition": "b+x", "name": "",
                       "duration_s": 300}
    assert "block b" in study.label(pl[0])
