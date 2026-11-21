"""staticactors.py: the opt-in -carla-omit-static-actors server flag and the patched-client checks (never silent)."""
from types import SimpleNamespace

import pytest

from coupledcarla import config, staticactors


def _binary(tmp_path, name, patched):
    f = tmp_path / name
    f.write_bytes(b"\x7fELF" + b"\0" * (3 << 20) + (b"get_static_actors" if patched else b"get_actors_by_id")
                  + b"\0" * 100)
    return f


def test_client_supports_finds_the_rpc_name_across_blocks(tmp_path):
    assert staticactors.client_supports(_binary(tmp_path, "patched.so", True))
    assert not staticactors.client_supports(_binary(tmp_path, "stock.so", False))
    assert not staticactors.client_supports(tmp_path / "missing.so")
    # the name straddling a 1 MB read boundary is still found
    f = tmp_path / "edge.so"
    f.write_bytes(b"\0" * ((1 << 20) - 5) + b"get_static_actors" + b"\0" * 10)
    assert staticactors.client_supports(f)


def test_off_by_default_and_flag_only_when_on():
    sc = config.DEFAULTS["server"]
    assert sc["omit_static_actors"] is False
    assert staticactors.server_args(sc) == []
    assert staticactors.server_args({**sc, "omit_static_actors": True}) == ["-carla-omit-static-actors"]


def test_require_client_stops_with_a_clear_message(monkeypatch, tmp_path):
    stock = _binary(tmp_path, "carla.so", False)
    monkeypatch.setattr(staticactors, "client_module_path", lambda: stock)
    staticactors.require_client({"omit_static_actors": False})          # off: nothing to check
    with pytest.raises(SystemExit) as e:
        staticactors.require_client({"omit_static_actors": True})
    msg = str(e.value)
    assert "stock wheel" in msg and str(stock) in msg and "omit_static_actors: false" in msg
    patched = _binary(tmp_path, "carla2.so", True)
    monkeypatch.setattr(staticactors, "client_module_path", lambda: patched)
    staticactors.require_client({"omit_static_actors": True})            # patched: fine


class _World:
    def __init__(self, listed, sync=True):
        self.listed, self.ticks, self.sync = set(listed), 0, sync

    def get_settings(self):
        return SimpleNamespace(synchronous_mode=self.sync)

    def tick(self):
        self.ticks += 1

    def wait_for_tick(self, timeout):
        self.ticks += 1

    def get_snapshot(self):
        listed = self.listed
        return SimpleNamespace(find=lambda i: object() if i in listed else None)


def test_check_after_spawn_stock_server(monkeypatch):
    msgs = []
    w = _World(listed={1, 2, 3})
    assert staticactors.check_after_spawn(w, [1, 2, 3], {"omit_static_actors": False}, msgs.append) is False
    assert w.ticks == 1 and not msgs
    # the setting on, but the attached server sends them: said, not fatal
    assert staticactors.check_after_spawn(_World({1, 2}), [1, 2], {"omit_static_actors": True}, msgs.append) is False
    assert any("no saving" in m for m in msgs)


def test_check_after_spawn_omitting_server(monkeypatch, tmp_path):
    msgs = []
    monkeypatch.setattr(staticactors, "client_module_path", lambda: _binary(tmp_path, "stock.so", False))
    with pytest.raises(SystemExit) as e:          # a flagged server and a stock client: stop, never silent
        staticactors.check_after_spawn(_World(listed=set(), sync=False), [5, 6], {"omit_static_actors": False},
                                       msgs.append)
    assert "-carla-omit-static-actors" in str(e.value) and "stock wheel" in str(e.value)
    monkeypatch.setattr(staticactors, "client_module_path", lambda: _binary(tmp_path, "ok.so", True))
    assert staticactors.check_after_spawn(_World(listed={6}), [5, 6], {"omit_static_actors": True}, msgs.append)
    assert any("fetches them when they change" in m for m in msgs)
    assert staticactors.check_after_spawn(_World(set()), [], {}, msgs.append) is None
