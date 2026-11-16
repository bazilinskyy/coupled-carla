"""The vendored netconvert_carla.py maps an OpenDRIVE road and lane to its SUMO edge when netconvert split the edge."""
import importlib.util
import os
from pathlib import Path

import pytest

pytest.importorskip("lxml")
pytest.importorskip("sumolib")
pytest.importorskip("carla")    # the module imports it at the top

SRC = Path(__file__).resolve().parent.parent / "coupledcarla" / "bridge" / "util" / "netconvert_carla.py"


@pytest.fixture(scope="module")
def nc():
    os.environ.setdefault("SUMO_HOME", "/nonexistent")   # the module exits without it; sumolib comes from pip here
    spec = importlib.util.spec_from_file_location("netconvert_carla_under_test", SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_sumo_id_of_split_edges(nc):
    t = nc.SumoTopology({}, {}, {
        ("28", -1): {("-28#0", 2), ("-28#1", 2)},                       # netconvert pieces (TU/e campus twin)
        ("5", -1): {("5.0.00", 0), ("5.40.00", 0)},                     # lane sections
        ("7", -1): {("7.0.00#0", 0), ("7.0.00#1", 0), ("7.50.00", 0)},  # both
        ("9", 1): {("-9.0.00", 1)},
    })
    assert t.get_sumo_id("28", -1, 0.8) == ("-28#1", 2)     # the road's end, where its successors begin
    assert t.get_sumo_id("5", -1, 10.0) == ("5.0.00", 0)
    assert t.get_sumo_id("5", -1, 45.0) == ("5.40.00", 0)
    assert t.get_sumo_id("7", -1, 3.0) == ("7.0.00#1", 0)
    assert t.get_sumo_id("7", -1, 60.0) == ("7.50.00", 0)
    assert t.get_sumo_id("9", 1) == ("-9.0.00", 1)
    assert t.get_sumo_id("1", -1) is None
