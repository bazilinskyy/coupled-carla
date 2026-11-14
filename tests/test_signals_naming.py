"""mapbuild.name_signals: every signal named, long ids shortened, a signal's own name kept."""
from coupledcarla import mapbuild


def test_name_signals(tmp_path):
    long1 = "joinedS_1428376399_268775144_42633748_cluster_25348050_42632719_#1more_18"
    f = tmp_path / "m.xodr"
    f.write_text("\n".join([f'<signal id="{long1}" name="{long1}" s="1"/>', f'<signal id="{long1[:-1]}9" s="2"/>',
                            '<signal id="s7" s="3" t="-4" name="Sign_Yield" dynamic="no">', '<signal id="abc" s="4"/>']))
    mapbuild.name_signals(f)
    lines = f.read_text().splitlines()
    assert lines[0] == '<signal id="sig0" name="sig0" s="1"/>'
    assert lines[1] == '<signal id="sig1" name="sig1" s="2"/>'
    assert lines[2] == '<signal id="s7" s="3" t="-4" name="Sign_Yield" dynamic="no">'
    assert lines[3] == '<signal id="abc" name="abc" s="4"/>'
