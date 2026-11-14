"""coupledcarla.responses + studytools: continuous responses from fake stations over the real control channel."""
import csv
import os
import types

import pytest

from coupledcarla import responses
from coupledcarla.coupling import ControlClient, ControlServer
from coupledcarla.studytools import HostStudy, StationStudy
from test_coupling import free_port, wait


def test_spec_forms():
    assert responses.spec(None) is None and responses.spec(False) is None
    both = responses.spec(True)
    assert both["button"] == responses.DEFAULT_TEXT["button"] and both["slider"] and not both["hidden"]
    s = responses.spec("slider,hidden")
    assert s["button"] is None and s["slider"] and s["hidden"]
    s = responses.spec({"button": "I would cross now", "slider_start": 20})
    assert s["button"] == "I would cross now" and s["slider"] is None and s["slider_start"] == 20
    assert responses.merge_cli(responses.spec("button"), "slider")["slider"]
    assert responses.merge_cli(None, "slider")["button"] is None and responses.merge_cli(None, None) is None


def test_button_presses_and_slider():
    r = responses.ResponseInput(responses.spec({"button": "b", "slider": "s", "slider_start": 95}))
    r.step(0.02, key=True)
    r.step(0.02, key=True)
    r.step(0.02, analog=0.3)                        # a trigger below the threshold: released, value kept
    assert (r.pressed, r.value, r.presses) == (False, 0.3, 1)
    r.step(0.02, analog=0.8)
    assert (r.pressed, r.presses) == (True, 2)
    for _ in range(50):                             # 1 s up at 40/s: clamped at 100
        r.step(0.02, slider_dir=1)
    assert r.slider == 100.0
    r.step(0.5, slider_dir=-1)
    assert r.slider == 80.0
    assert r.cells() == [0, "0.000", 2, "80.0"]          # the slider steps let go of the button
    only = responses.ResponseInput(responses.spec("slider"))
    only.step(0.02, key=True)
    assert only.cells() == ["", "", "", "50.0"] and only.message(3, 1.0)["b"] is None


def test_bot_schedule():
    r = responses.ResponseInput(responses.spec(True), bot=[[2.0, 1, None], [1.0, None, 70], [3.0, 0, 10]])
    r.bot(0.5)
    assert (r.pressed, r.slider) == (False, 50.0)
    r.bot(2.5)
    assert (r.pressed, r.presses, r.slider) == (True, 1, 70.0)
    r.bot(9)
    assert (r.pressed, r.slider) == (False, 10.0)


class FakeJoy:
    def __init__(self, name, buttons=(), axes=(), hats=()):
        self.name, self.buttons, self.axes, self.hats = name, dict(buttons), list(axes), list(hats)

    def get_name(self):
        return self.name

    def get_numbuttons(self):
        return 16

    def get_button(self, i):
        return self.buttons.get(i, 0)

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, i):
        return self.axes[i]

    def get_numhats(self):
        return len(self.hats)

    def get_hat(self, i):
        return self.hats[i]


def test_joysticks_by_kind():
    pad = FakeJoy("Xbox Wireless Controller", axes=[0, 0, 0, 0, 0, 0.0], hats=[(0, 1)])   # trigger half in, d-pad up
    assert responses.joystick_state(pad) == (False, 0.5, 1)
    pad.buttons[5] = 1
    assert responses.joystick_state(pad)[:2] == (True, 1.0)
    wheel = FakeJoy("Logitech G25 Racing Wheel", buttons={6: 1, 5: 1}, axes=[0.2, -1.0, -1.0, 0, 0])
    assert responses.joystick_state(wheel) == (True, 1.0, -1)          # pedal axes are not a trigger
    assert responses.joystick_state(wheel, {"buttons": [7]})[0] is False


class FakeLogger:
    def __init__(self):
        self.listeners, self.rows = [], []

    def event(self, ts, frame, name, actor_id="", detail=""):
        self.rows.append((ts, frame, name, actor_id, detail))
        for f in self.listeners:
            f(ts, frame, name, actor_id, detail)


def part(role, kind="pedestrian", station=True, **cfg):
    return types.SimpleNamespace(role=role, kind=kind, station=station, cfg={"role": role, **cfg}, actor=None,
                                 input=None, light=None, ctl=None)


def test_station_responses_reach_responses_csv(tmp_path):
    server = ControlServer(free_port(), log=lambda *a: None)
    parts = [part("pedestrian", response={"button": "I would cross now", "slider": "Safe?"}),
             part("driver", "driver", station=False, response="slider", bot={"response": [[0.04, None, 80]]}),
             part("av_passenger", "av_passenger")]
    tools = HostStudy({"name": "demo"}, {"duration_s": 10}, parts, tmp_path, log=lambda *a: None, control=server)
    by_role = {p.role: p for p in parts}
    server.on_hello = lambda role: tools.welcome(by_role[role])
    tools.bind(FakeLogger(), queue=lambda c: None)
    client = ControlClient("127.0.0.1", server.port, "pedestrian", retry_s=5)
    try:
        welcome = []
        assert wait(lambda: welcome.extend(m for m in client.poll() if m["type"] == "welcome") or welcome)
        args = types.SimpleNamespace(input="bot", response=None)
        welcome[0]["bot"] = {"response": [[0.02, 1, 30], [0.06, 0, None]]}
        st = StationStudy(welcome[0], args, client.send)
        assert st.resp.sp["button"] == "I would cross now" and st.tor is None
        cells = []
        for frame in range(1, 5):                      # the station's frames 1-4 at 50 Hz
            cells.append(st.sample(0.02 * frame, frame))
        assert cells[0] == [1, "1.000", 1, "30.0", ""] and cells[3][0] == 0
        got = []
        assert wait(lambda: got.extend(m for m in server.poll() if m.get("type") == "response") or len(got) >= 4)
        for frame, msg in enumerate(got[:2]):
            assert tools.on_message(msg, 0.02 * (frame + 1), 100 + frame)
            tools.log(0.02 * (frame + 1), 100 + frame)
        tools.log(0.06, 102)
        status = tools.status()
        ped = next(p for p in status["participants"] if p["role"] == "pedestrian")
        assert ped["response"] == {"button": 1, "value": 1.0, "presses": 1, "slider": 30.0}
        summary = {}
        tools.close(summary)
        assert summary["response_rows"] == 6
        rows = list(csv.DictReader(open(tmp_path / "responses.csv"), delimiter=";"))
        assert list(rows[0])[:7] == ["utc", "time", "frame", "button", "button_value", "presses", "slider"]
        ped_rows = [r for r in rows if r["role"] == "pedestrian"]
        drv_rows = [r for r in rows if r["role"] == "driver"]
        assert [r["frame"] for r in ped_rows] == ["100", "101", "102"]
        assert [r["station_frame"] for r in ped_rows] == ["1", "2", "2"]           # the latest sample per host frame
        assert ped_rows[0]["button"] == "1" and ped_rows[0]["slider"] == "30.0"
        assert [r["slider"] for r in drv_rows] == ["50.0", "80.0", "80.0"]          # host-local bot
        assert drv_rows[0]["button"] == "" and drv_rows[0]["age_ms"] == "0"
    finally:
        server.close()


def test_response_panel_draws_headless():
    pygame = pytest.importorskip("pygame")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    try:
        screen = pygame.Surface((640, 360))
        font = pygame.font.SysFont("sans", 18)
        r = responses.ResponseInput(responses.spec({"button": "Hold to cross", "slider": "Safe?", "slider_start": 50}))
        r.step(0.02, key=True)
        r.draw(screen, font)
        lamp = screen.get_at((640 // 2 - min(560, 600) // 2 + 22, 360 - (16 + 56 + 26) - 24 + 8 + 12))
        assert lamp.g > 180 and lamp.r < 100                     # green while held
        hidden = responses.ResponseInput(responses.spec("button,hidden"))
        blank = pygame.Surface((640, 360))
        hidden.draw(blank, font)
        assert blank.get_at((320, 300)) == pygame.Color(0, 0, 0)
    finally:
        pygame.quit()


def test_summaries_for_analysis_and_viewer(tmp_path):
    from coupledcarla import analysis, studytools, takeover
    trial = tmp_path / "20261115-101010_P03_trial2"
    trial.mkdir()
    log = responses.ResponseLog(trial / "responses.csv", ["pedestrian"])
    sched = {0.0: (0, 50.0), 1.0: (1, 60.0), 3.0: (0, 80.0)}
    state = (0, 50.0)
    for i in range(201):                                 # 4 s at 50 Hz: held from 1 s to 3 s
        t = i * 0.02
        state = sched.get(round(t, 2), state)
        log.update("pedestrian", {"b": state[0], "v": float(state[0]), "n": 1 if t >= 1.0 else 0, "s": state[1],
                                  "f": i, "recv": 0.0})
        log.log(t, 1000 + i, now=0.0)
    log.close()
    f, w = takeover.open_csv(trial / "takeover.csv")
    w.writerows([["1.000", "request", "at_s", "30.0", "0", "0", "driver"],
                 ["2.250", "takeover", "steer; reaction 1.150 s", "29.0", "0", "0", "driver"],
                 ["9.000", "request", "dashboard", "20.0", "0", "0", "driver"],
                 ["16.100", "no reaction", "BrakeToStop", "19.0", "0", "0", "driver"]])
    f.close()
    (trial / "config.yaml").write_text("name: demo\nstudy: {participant: 3, position: 1, index: 2, condition: a+b}\n")
    res, tor = studytools.analyse(trial, tmp_path)
    assert res == [{"role": "pedestrian", "rows": 201, "duration_s": 4.0, "button_held_s": 2.0, "button_share": 0.5,
                    "presses": 1, "first_press_s": 1.0, "slider_mean": 62.6, "slider_min": 50.0, "slider_max": 80.0,
                    "slider_end": 80.0}]
    assert tor == [{"role": "driver", "t_request": 1.0, "source": "at_s", "how": "steer", "reaction_s": 1.15,
                    "no_reaction": 0, "speed_kmh_at_request": 30.0},
                   {"role": "driver", "t_request": 9.0, "source": "dashboard", "how": "", "reaction_s": "",
                    "no_reaction": 1, "speed_kmh_at_request": 20.0}]
    s = responses.series(trial, t0=0.0)["pedestrian"]
    assert len(s) == 41 and s[0] == [0.0, 0, 50.0] and [1.0, 1, 60.0] in s
    res_all, tor_all = studytools.collect([trial], tmp_path)
    assert res_all[0]["participant"] == 3 and res_all[0]["condition"] == "a+b" and len(tor_all) == 2
    rows = list(csv.DictReader(open(tmp_path / "study_takeovers.csv"), delimiter=";"))
    assert rows[0]["trial_dir"] == trial.name and rows[0]["reaction_s"] == "1.15"
    assert analysis.trial_index(trial) == 2            # P03 in the folder name keeps the trial number readable
