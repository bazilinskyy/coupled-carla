"""The quick stand-in world (map.standin): the OpenDRIVE cut to a disc around the start."""
import pytest

etree = pytest.importorskip("lxml.etree")     # standin.py reads the OpenDRIVE with lxml (CI installs neither SUMO nor lxml)

from coupledcarla.standin import crop_xodr  # noqa: E402

XODR = """<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4"/>
  <road id="1" junction="-1" length="100">
    <link><successor elementType="junction" elementId="10"/></link>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
    <signals><signal id="s1" s="90" t="-3"/></signals>
  </road>
  <road id="2" junction="-1" length="100">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView><geometry s="0" x="110" y="0" hdg="0" length="100"><line/></geometry></planView>
  </road>
  <road id="3" junction="10" length="10">
    <link><predecessor elementType="road" elementId="1" contactPoint="end"/>
          <successor elementType="road" elementId="2" contactPoint="start"/></link>
    <planView><geometry s="0" x="100" y="0" hdg="0" length="10"><line/></geometry></planView>
  </road>
  <road id="4" junction="-1" length="100">
    <link><predecessor elementType="road" elementId="2" contactPoint="end"/></link>
    <planView><geometry s="0" x="5000" y="0" hdg="0" length="100"><line/></geometry></planView>
    <signals><signal id="s4" s="10" t="-3"/></signals>
  </road>
  <controller id="c1"><control signalId="s1"/></controller>
  <controller id="c4"><control signalId="s4"/></controller>
  <junction id="10">
    <connection id="0" incomingRoad="1" connectingRoad="3" contactPoint="start"/>
    <controller id="c1"/>
  </junction>
</OpenDRIVE>
"""


def test_crop_keeps_the_roads_near_the_start_and_their_junction():
    text, st = crop_xodr(XODR, 20.0, 0.0, 150.0)
    root = etree.fromstring(text.encode())
    ids = sorted(r.get("id") for r in root.findall("road"))
    assert ids == ["1", "2", "3"] and st["roads"] == 3 and st["of"] == 4 and st["junctions"] == 1
    assert [c.get("id") for c in root.findall("controller")] == ["c1"]          # c4's signal went with road 4
    assert root.find("junction/controller").get("id") == "c1"


def test_crop_drops_links_to_roads_left_out():
    text, _ = crop_xodr(XODR, 5000.0, 0.0, 300.0)
    root = etree.fromstring(text.encode())
    assert [r.get("id") for r in root.findall("road")] == ["4"]
    assert root.find("road/link/predecessor") is None                          # road 2 is not there
    assert root.findall("junction") == [] and [c.get("id") for c in root.findall("controller")] == ["c4"]
