"""Provenance tags of placed scenery (coupledcarla/provenance.py) and the realism pilot's top-down metrics."""
import numpy as np

from coupledcarla import aerial, provenance as P


def test_building_tags_follow_the_source():
    assert P.building("3dbag") == ("measured", "3dbag")
    assert P.building("PLATEAU") == ("measured", "plateau")
    assert P.building("overture") == ("inferred", "overture")
    assert P.building(None) == ("inferred", "osm")


def test_element_tag_uses_the_register_source():
    assert P.element({"_src": "municipal", "tags": {}}) == ("register", "municipal")
    assert P.element({"tags": {"source": "bgt-bank"}}) == ("register", "bgt-bank")
    assert P.element({"tags": {}}) == ("register", "osm")


def test_encode_defaults_by_kind_and_round_trips():
    a = [0, 1.0, 2.0, 0.0, 0.0, 1.0, "tree"]
    b = [1, 5.0, 2.0, 0.0, 0.0, 1.0, "parked"]
    c = [2, 9.0, 9.0, 0.0, 0.0, 1.0, "building"]
    items = [a, b, c]
    enc = P.encode(items, {id(a): ("inferred", "canopy_map"), id(c): ("measured", "3dbag")})
    data = {"items": items, "prov": enc}
    assert P.of_items(data) == [("inferred", "canopy_map"), ("generated", "parking"), ("measured", "3dbag")]
    assert P.of_items({"items": items}) == [("register", "osm"), ("generated", "parking"), ("inferred", "osm")]


def test_summary_counts_and_near_share():
    items = [[0, 0.0, 0.0, 0, 0, 1, "tree"], [0, 10.0, 0.0, 0, 0, 1, "bush"], [0, 500.0, 0.0, 0, 0, 1, "bush"]]
    s = P.summary({"items": items}, near=[(0.0, 0.0)], radius=50.0)
    assert s["total"] == 3
    assert s["by_tag"] == {"register": 1, "generated": 2}
    assert s["near"]["by_tag"] == {"register": 1, "generated": 1}
    assert s["near"]["generated_share"] == 0.5


def test_twin_layers_and_country_lookup():
    assert P.country_at(51.44, 5.48) == "NL"
    assert P.country_at(40.96, -5.66) == "ES"
    assert P.country_at(35.66, 139.70) == "JP"
    assert P.country_at(-1.94, 30.06) is None
    assert P.twin_layers("NL")["buildings"][0] == "measured"
    assert P.twin_layers("XX") == {}


def test_vegetation_iou_of_identical_and_disjoint_images():
    img = np.zeros((100, 100, 3), np.uint8)
    img[:, :50] = (60, 140, 50)        # green left half
    img[:, 50:] = (128, 128, 128)
    other = img[:, ::-1].copy()          # green right half
    assert aerial.veg_iou(img, img) == 1.0
    assert aerial.veg_iou(img, other) == 0.0
    m = aerial.topdown_metrics(img, img)
    assert m["veg_iou"] == 1.0 and m["colour_hist_sim"] == 1.0


def test_aerial_services_listed():
    assert aerial.available("NL") and aerial.available("JP") and aerial.available("DE-NW")
    assert not aerial.available("RW")


def test_pilot_spread_keeps_far_apart_views():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("realism_score_mod", Path(__file__).resolve().parent.parent /
                                                  "tools" / "realism_score.py")
    rs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rs)
    views = [{"lat": 51.44 + i * 1e-5, "lon": 5.47} for i in range(50)] + [{"lat": 51.45, "lon": 5.48}]
    got = rs.spread(views, 3, seed=1)
    assert len(got) == 3
    assert {"lat": 51.45, "lon": 5.48} in got        # the far-away view is always kept
    assert rs.spread(views[:2], 5, seed=1) == views[:2]
