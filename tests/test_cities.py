

def test_flag():
    from coupledcarla.cities import ISO2, flag
    import yaml
    from pathlib import Path
    assert flag("Netherlands") == "\U0001F1F3\U0001F1F1" and flag(code="ua") == "\U0001F1FA\U0001F1E6"
    assert flag("Atlantis") == "" and flag(code="X1") == ""
    cat = yaml.safe_load((Path(__file__).resolve().parent.parent / "experiments" / "cities.yaml").read_text())
    assert all(c["country"] in ISO2 for c in cat["cities"])      # every city of the README table has its flag
