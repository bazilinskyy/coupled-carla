"""staticactors.py: static actors out of the per-tick state (upstream always, our tree with the flag) and the
client checks (never silent)."""
from types import SimpleNamespace

import pytest

from coupledcarla import config, staticactors


@pytest.fixture(autouse=True)
def _no_upstream_call(monkeypatch):
    """Tests decide the client kind; the carla module of the test interpreter (if any) does not."""
    monkeypatch.setattr(staticactors, "_python_call", lambda: False)


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
    """listed: ids in the snapshot; known: ids world.get_actors() lists (default: the listed ones, i.e. a client that
    does not keep the static ones)."""

    def __init__(self, listed, sync=True, known=None):
        self.listed, self.ticks, self.sync = set(listed), 0, sync
        self.known = set(listed) if known is None else set(known)

    def get_actors(self):
        return [SimpleNamespace(id=i) for i in sorted(self.known)]

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
    with pytest.raises(SystemExit) as e:          # a server that omits them and a stock client: stop, never silent
        staticactors.check_after_spawn(_World(listed=set(), sync=False), [5, 6], {"omit_static_actors": False},
                                       msgs.append)
    assert "stock carla client" in str(e.value) and "same CARLA tree" in str(e.value)
    monkeypatch.setattr(staticactors, "client_module_path", lambda: _binary(tmp_path, "ok.so", True))
    assert staticactors.check_after_spawn(_World(listed={6}, known={5, 6}), [5, 6], {"omit_static_actors": True},
                                          msgs.append)
    assert any("fetches them when they change" in m for m in msgs)
    assert staticactors.check_after_spawn(_World(set()), [], {}, msgs.append) is None


def test_upstream_client_and_server(monkeypatch, tmp_path):
    """CARLA ue58-dev since f5fb36aed: no flag, the server always omits, the wheel has world.get_static_actors()."""
    monkeypatch.setattr(staticactors, "_python_call", lambda: True)
    monkeypatch.setattr(staticactors, "client_module_path", lambda: _binary(tmp_path, "up.so", True))
    assert staticactors.client_kind() == "upstream"
    staticactors.require_client({"omit_static_actors": True})
    msgs = []
    w = _World(listed=set(), known={5, 6})        # setting off: upstream omits anyway, and that is fine
    assert staticactors.check_after_spawn(w, [5, 6], {"omit_static_actors": False}, msgs.append) is True
    assert any("get_static_actors" in m and "f5fb36aed" in m for m in msgs)


def test_a_wheel_from_the_other_tree_stops_the_run(monkeypatch, tmp_path):
    """Our tree's wheel on an upstream server (or the reverse) has the RPC but lists no static actor: stop."""
    monkeypatch.setattr(staticactors, "client_module_path", lambda: _binary(tmp_path, "cc.so", True))
    assert staticactors.client_kind() == "cc"
    with pytest.raises(SystemExit) as e:
        staticactors.check_after_spawn(_World(listed=set(), known={5}), [5, 6], {}, print)
    assert "cc carla client" in str(e.value) and "f5fb36aed" in str(e.value) and "wheel_optc" in str(e.value)


def test_client_kind_from_the_binary(tmp_path):
    assert staticactors.client_kind(_binary(tmp_path, "a.so", True)) == "cc"
    assert staticactors.client_kind(_binary(tmp_path, "b.so", False)) == "stock"
