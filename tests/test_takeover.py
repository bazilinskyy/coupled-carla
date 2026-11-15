"""coupledcarla.takeover + studytools: take-over requests between the host and a fake station over the real
control channel (no CARLA: a stand-in automated car and a stub carla module)."""
import csv
import sys
import types

import pytest

from coupledcarla import takeover
from coupledcarla.coupling import ControlClient, ControlServer
from coupledcarla.studytools import HostStudy, StationStudy
from coupledcarla.takeover import AUTOMATED, MANUAL, REQUESTED
from test_coupling import free_port, wait
from test_responses import FakeLogger, part

ROUTE = [[51.4431, 5.4913], [51.4428, 5.4890]]


def ctl(steer=0.0, throttle=0.0, brake=0.0):
    return types.SimpleNamespace(steer=steer, throttle=throttle, brake=brake)


def test_detect_and_brake():
    sp = takeover.spec({"route": ROUTE})
    assert takeover.detect(ctl(0.05, 0.05, 0.05), sp) is None
    assert takeover.detect(ctl(-0.2), sp) == "steer"
    assert takeover.detect(ctl(brake=0.3), sp) == "brake"
    assert takeover.detect(ctl(throttle=0.5), sp) == "throttle"
    assert takeover.detect(None, sp, button=True) == "button"
    assert takeover.brake_for(10.0, 3.0) == pytest.approx(0.1 + 3 / 9)
    assert takeover.brake_for(0.1, 3.0) == 1.0
    assert takeover.nearest_index([(0, 0), (10, 0), (20, 0)], 12, 3) == 1
    assert len(takeover.beeps(8000, -16, 2)) == 8000 * 2 * 2           # 1 s of 16-bit stereo


def test_state_machine_keep_driving(tmp_path):
    events = []
    f, w = takeover.open_csv(tmp_path / "takeover.csv")
    tk = takeover.TakeOver("driver", takeover.spec({"route": ROUTE, "takeover": {
        "at_s": [5, 30], "budget_s": 4, "no_reaction": "keep"}}), lambda t, n, d: events.append((t, n, d)), w,
        where=lambda: (36.0, 10.0, -5.0))
    tk.tick(4.9)
    assert tk.state == AUTOMATED
    tk.tick(5.0)
    assert tk.state == REQUESTED and tk.automated
    assert not tk.request(6.0, "dashboard")                 # one request at a time
    tk.tick(9.1)
    assert tk.state == AUTOMATED and tk.records[0]["no_reaction"]
    assert not tk.taken_over(9.2, "steer")                  # too late: the request ended
    tk.tick(30.0)
    assert tk.taken_over(31.25, "brake")                     # host-measured reaction when the station sent none
    assert tk.records[1]["reaction_s"] == 1.25 and not tk.automated
    assert tk.engage(40.0) and tk.state == AUTOMATED
    f.close()
    assert [e[1] for e in events] == ["takeover_request", "takeover_no_reaction", "takeover_request", "takeover",
                                      "automation_engaged"]
    rows = list(csv.DictReader(open(tmp_path / "takeover.csv")))
    assert [r["event"] for r in rows] == ["request", "no reaction", "request", "takeover", "engaged"]
    assert rows[3]["detail"] == "brake; reaction 1.250 s" and rows[3]["speed_kmh"] == "36.0" and rows[3]["y"] == "-5.00"


class Loc(types.SimpleNamespace):
    pass


class FakeCar:
    """Stands in for ai.ScriptedCar: drives itself, fires a takeover trigger after 1 s of driving."""

    def __init__(self):
        self.vehicle = types.SimpleNamespace(id=42, get_location=lambda: Loc(x=12.0, y=1.0),
                                             get_velocity=lambda: Loc(x=10.0, y=0.0))
        self.triggers = [[5, {"at_m": 10, "takeover": True}, False]]
        self.events, self.light, self.progress, self.target = [], None, 0, 0.0
        self.plan = [(types.SimpleNamespace(transform=types.SimpleNamespace(location=Loc(x=10.0 * i, y=0.0))), None)
                     for i in range(5)]
        self.agent = types.SimpleNamespace(plans=[])
        self.agent.set_global_plan = lambda plan, **kw: self.agent.plans.append(len(plan))
        self.steps = 0
        self.apply = None

    def step(self, t, dt, walkers=()):
        self.steps += 1
        if self.steps == 50:
            self.triggers[0][2] = True
            self.events.append(("trigger", "driver {'takeover': True}"))
        c = ctl(throttle=0.5)
        self.apply(c)
        return c


@pytest.fixture
def fake_carla(monkeypatch):
    stub = types.SimpleNamespace(command=types.SimpleNamespace(ApplyVehicleControl=lambda i, c: ("apply", i, c)),
                                 VehicleControl=lambda **kw: ctl(**kw))
    monkeypatch.setitem(sys.modules, "carla", stub)
    return stub


class FakeDashboard:
    def __init__(self):
        self.queue, self.status, self.url = [], None, "http://127.0.0.1:0/"

    def commands(self):
        out, self.queue = self.queue, []
        return out

    def set_status(self, s):
        self.status = s


def test_station_takes_over_and_hands_back(tmp_path, fake_carla):
    server = ControlServer(free_port(), log=lambda *a: None)
    automation = {"route": ROUTE, "speed_kmh": 30, "takeover": {"at_s": 1.0, "budget_s": 2.0, "beep": False}}
    drv = part("driver", "driver", automation=automation)
    dash = FakeDashboard()
    tools = HostStudy({"name": "demo"}, {"duration_s": 20}, [drv], tmp_path, log=lambda *a: None, control=server,
                      dashboard=dash)
    car = tools.cars["driver"] = FakeCar()
    drv.actor = car.vehicle
    server.on_hello = lambda role: tools.welcome(drv)
    sent, logger = [], FakeLogger()
    tools.bind(logger, queue=sent.append)
    client = ControlClient("127.0.0.1", server.port, "driver", retry_s=5)
    try:
        welcome = []
        assert wait(lambda: welcome.extend(m for m in client.poll() if m["type"] == "welcome") or welcome)
        assert welcome[0]["automation"]["takeover"]["budget_s"] == 2.0 and "response" not in welcome[0]
        st = StationStudy(welcome[0], types.SimpleNamespace(input="keyboard", response=None), client.send)

        def station_messages():
            for m in client.poll():
                st.on_message(m)

        tools.step(0.5, 0.02, frame=1)                          # automated: the car drives, the station does not
        assert sent[-1][:2] == ("apply", 42) and drv.ctl.throttle == 0.5
        assert wait(lambda: station_messages() or st.tor.state == AUTOMATED)
        assert st.drive(ctl(steer=0.5), [], now=100.0) is False      # steering before a request does nothing
        tools.step(1.0, 0.02, frame=2)                          # at_s: the request goes to the station
        assert wait(lambda: station_messages() or st.tor.state == REQUESTED)
        assert st.drive(ctl(steer=0.02), [], now=200.0) is False     # warning shown at 200.0
        assert st.drive(ctl(steer=0.4), [], now=200.8) is True       # took over by steering, 0.8 s later
        msgs = []
        assert wait(lambda: msgs.extend(server.poll()) or any(m.get("type") == "takeover" for m in msgs))
        for m in msgs:
            tools.on_message(m, 1.9, 3)
        assert tools.tor["driver"].state == MANUAL
        n = car.steps
        tools.step(2.0, 0.02, frame=4)
        assert car.steps == n and drv.ctl is None              # the station drives now
        assert tools.drives(drv) is False
        assert st.tor.may_drive(None, True, 201.0) is False     # the automation button: back to the automation
        msgs = []
        assert wait(lambda: msgs.extend(server.poll()) or any(m.get("type") == "engage" for m in msgs))
        for m in msgs:
            tools.on_message(m, 3.0, 5)
        assert tools.tor["driver"].state == AUTOMATED and car.agent.plans == [4]   # from route point 1 (x 10 m)
        for i in range(60):                                     # the trigger fires on the 50th step -> request
            tools.step(3.0 + 0.02 * i, 0.02, frame=10 + i)
        assert tools.tor["driver"].state == REQUESTED and tools.tor["driver"].source == "trigger"
        tools.step(6.0, 0.02, frame=200)                        # budget over: brake to a stop
        assert tools.tor["driver"].stopping and sent[-1][2].brake == pytest.approx(0.1 + 3 / 9)
        dash.queue.append({"cmd": "abort"})
        tools.step(6.02, 0.02, frame=201)
        assert tools.abort and not tools.stop
        summary = {}
        tools.close(summary)
        recs = summary["takeovers"]
        assert recs[0]["how"] == "steer" and recs[0]["reaction_s"] == 0.8 and recs[0]["host_reaction_s"] == 0.9
        assert recs[1]["source"] == "trigger" and recs[1]["no_reaction"] and summary["aborted"] == "trial ended early"
        names = [r[2] for r in logger.rows]
        assert names == ["takeover_request", "takeover", "automation_engaged", "trigger", "takeover_request",
                         "takeover_no_reaction", "trial_aborted"]
        rows = list(csv.DictReader(open(tmp_path / "takeover.csv")))
        assert [r["event"] for r in rows] == ["request", "takeover", "engaged", "request", "no reaction"]
        assert rows[0]["detail"] == "at_s" and rows[1]["detail"] == "steer; reaction 0.800 s"
        assert dash.status["automated"] == ["driver"]
    finally:
        server.close()


def test_host_local_bot_takes_over(tmp_path, fake_carla):
    class Autopilot:                        # the name is what matters (inputs.Autopilot)
        pass
    drv = part("driver", "driver", station=False, automation={"route": ROUTE, "takeover": {"bot_reaction_s": 1.0}})
    drv.input = Autopilot()
    tools = HostStudy({"name": "demo"}, {"duration_s": 20}, [drv], tmp_path, log=lambda *a: None)
    car = tools.cars["driver"] = FakeCar()
    drv.actor = car.vehicle
    tools.bind(FakeLogger(), queue=lambda c: None)
    assert tools.intercept(drv, ctl(steer=0.9), 0.5) is True       # automated: the bot's control is ignored
    tools.tor["driver"].request(1.0, "dashboard")
    assert tools.intercept(drv, ctl(), 1.5) is True
    assert tools.intercept(drv, ctl(), 2.0) is False               # 1 s after the request
    assert tools.tor["driver"].records[0]["how"] == "bot"
    assert tools.hud_lines(drv) == ["manual"]
    summary = {}
    tools.close(summary)
    assert summary["takeovers"][0]["reaction_s"] == 1.0


def test_station_bot_reacts_after_its_delay():
    sent = []
    st = StationStudy({"automation": takeover.spec({"route": ROUTE, "takeover": {"bot_reaction_s": 0.5}}),
                       "bot": {}}, types.SimpleNamespace(input="bot", response=None), sent.append)
    st.on_message({"type": "takeover", "state": "requested"})
    assert st.drive(ctl(steer=1.0), [], now=10.0) is False          # a bot ignores its own steering
    assert st.drive(ctl(), [], now=10.6) is True
    assert sent == [{"type": "takeover", "how": "bot", "reaction_s": 0.6}]
    assert st.sample(1.0, 7) == ["", "", "", "", MANUAL]


def test_warning_and_beep_draw_headless(monkeypatch):
    pygame = pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    pygame.init()
    try:
        st = takeover.StationTakeOver(takeover.spec({"route": ROUTE}), lambda m: None)
        screen = pygame.Surface((800, 450))
        font, big = pygame.font.SysFont("sans", 18), pygame.font.SysFont("sans", 40)
        st.draw(screen, big, font, 0.0)
        assert screen.get_at((400, 450 // 2 - 140)) == pygame.Color(0, 0, 0)       # automated: no warning
        st.on_message({"type": "takeover", "state": "requested"})
        st.draw(screen, big, font, 2.0)                                            # pulse at its brightest
        c = screen.get_at((400 - 200, 450 // 2 - 140))
        assert c.r > 150 and c.g < 80 and st.shown_at == 2.0                       # red box, time it first showed
        st.beep()                                                                  # dummy audio, or silently off
        assert st.sound_on or st.sp["takeover"]["beep"] is False
        st.on_message({"type": "takeover", "state": "manual"})
        st.beep()
        assert not st.sound_on
    finally:
        pygame.quit()


def test_reaction_counts_from_the_frame_that_showed_the_warning(monkeypatch):
    """A windowed station reads its input before it draws the frame: steering on the frame the request arrives
    on (before the participant could see the warning) is no take-over, and the reaction time starts when the
    warning is first drawn."""
    pygame = pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    pygame.init()
    try:
        sent = []
        st = takeover.StationTakeOver(takeover.spec({"route": ROUTE, "takeover": {"beep": False}}), sent.append)
        screen = pygame.Surface((800, 450))
        font, big = pygame.font.SysFont("sans", 18), pygame.font.SysFont("sans", 40)
        st.draw(screen, big, font, 99.0)                                  # a frame of automated driving
        st.on_message({"type": "takeover", "state": "requested"})
        assert st.may_drive(ctl(steer=0.5), False, 100.0) is False        # warning not on screen yet
        assert st.state == REQUESTED and not sent
        st.draw(screen, big, font, 100.02)                                # the frame that shows it
        assert st.may_drive(ctl(steer=0.5), False, 100.5) is True
        assert sent == [{"type": "takeover", "how": "steer", "reaction_s": 0.48}]
    finally:
        pygame.quit()
