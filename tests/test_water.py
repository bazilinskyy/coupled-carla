"""Water on CARLA 1.0 (placement.py): the scenery's water bodies sent with the OpenDRIVE as <userData>, which
unreal/patches/OpenDriveGenerator_WaterTerrain.patch reads to lower CARLA's generated terrain under them."""
from coupledcarla import placement

SQUARE = [[-500.0, -500.0], [1500.0, -500.0], [1500.0, 1500.0], [-500.0, 1500.0], [-500.0, -500.0]]
ISLAND = [[100.0, -50.0], [300.0, -50.0], [300.0, 50.0], [100.0, 50.0], [100.0, -50.0]]
XODR = '<?xml version="1.0"?>\n<OpenDRIVE>\n<header/>\n</OpenDRIVE>\n'


def test_water_userdata_in_the_opendrive():
    water = [{"level": 1.1, "rings": [SQUARE, ISLAND]}, {"level": 4.6, "rings": [SQUARE[:4] + SQUARE[:1]]}]
    out, n = placement.carla_xodr(XODR, keep_lights=True, water=water)
    assert n == 0 and out.index('<userData code="coupledcarla.water">') < out.index("</OpenDRIVE>")
    lines = out.split('<userData code="coupledcarla.water">')[1].split("</userData>")[0].strip().splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("0 1.100 -500.0,-500.0 1500.0,-500.0") and lines[1].startswith("0 1.100 100.0,-50.0")
    assert lines[2].startswith("1 4.600 ")                         # the body's index, then its level


def test_no_water_leaves_the_opendrive_alone():
    assert placement.carla_xodr(XODR, keep_lights=True)[0] == XODR
    assert placement.with_water(XODR, [{"level": 1.0, "rings": [[[0, 0], [1, 1]]]}]) == XODR      # too few points
    assert placement.with_water("<not-opendrive/>", [{"level": 1.0, "rings": [SQUARE]}]) == "<not-opendrive/>"
