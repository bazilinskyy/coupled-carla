from shapely.geometry import LineString

from coupledcarla import road_surfaces as rs

XODR = """<?xml version="1.0"?>
<OpenDRIVE><header name="T"/>
<road id="1" length="10" junction="-1"><lanes><laneSection s="0"><center><lane id="0" type="none"/></center>
<right><lane id="-1" type="driving"><link/><width sOffset="0" a="3" b="0" c="0" d="0"/><roadMark sOffset="0"/>
<speed sOffset="0" max="8"/></lane><lane id="-2" type="sidewalk"><width sOffset="0" a="2" b="0" c="0" d="0"/></lane>
</right></laneSection></lanes></road>
<road id="2" length="10" junction="-1"><lanes><laneSection s="0"><right><lane id="-1" type="driving">
<material sOffset="0" surface="clinker"/></lane></right></laneSection></lanes></road>
</OpenDRIVE>"""


def test_style_of():
    assert rs.style_of({"surface": "paving_stones"}, "NL") == "clinker"
    assert rs.style_of({"surface": "paving_stones"}, "ES") == "pavers"
    assert rs.style_of({"surface": "cobblestone"}, "") == "sett"
    assert rs.style_of({"surface": "asphalt"}, "NL") is None


def test_write_materials_puts_one_material_on_car_lanes(tmp_path):
    x = tmp_path / "map.xodr"
    x.write_text(XODR)
    n = rs.write_materials(x, {1: "sett"})
    text = x.read_text()
    assert n == 1
    assert text.count("<material") == 1 and 'surface="sett"' in text       # road 2's stale material removed
    lane = text[text.index('<lane id="-1" type="driving">'):text.index("</lane>")]
    assert lane.index("<material") < lane.index("<speed")                    # OpenDRIVE order: before speed


def test_main_road_wins_inside_its_carriageway():
    roads = [{"line": LineString([(0, -50), (0, 50)]), "style": None, "rank": 7, "half": 5.0, "layer": 0},
             {"line": LineString([(0, 0), (60, 0)]), "style": "clinker", "rank": 4, "half": 3.2, "layer": 0}]
    c = rs.Classifier(roads)
    assert c(2.0, 1.0) is None          # in the junction, inside the primary's carriageway: asphalt
    assert c(20.0, 1.0) == "clinker"    # along the side street
    assert c(200.0, 0.0) is None        # no road near


class _World:
    def __init__(self, names):
        self.names = names

    def get_names_of_all_objects(self):
        return self.names


def test_server_paints_surfaces():
    assert rs.server_paints_surfaces(_World(["SM_X", "RoadSurface_clinker"]))
    assert not rs.server_paints_surfaces(_World(["SM_X"]))
