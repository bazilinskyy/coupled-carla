"""coupledcarla.incar and coupledcarla.haptics: speed limits from the OpenDRIVE, timed messages, take-over requests
and responses, display values and conditions, drawing with pygame (skipped without it), haptic patterns, UDP cues
and the wheel's vibration. No CARLA."""
import json
import os
import socket
import types

import pytest

from coupledcarla import haptics, incar

XODR = """<?xml version="1.0"?>
<OpenDRIVE>
  <road id="7" junction="-1" length="200">
    <type s="0" type="town"><speed max="50" unit="km/h"/></type>
    <lanes>
      <laneSection s="0">
        <right><lane id="-1" type="driving"><speed sOffset="0" max="8.33"/></lane></right>
        <left><lane id="1" type="driving"><speed sOffset="0" max="13.89"/></lane></left>
      </laneSection>
      <laneSection s="120">
        <right><lane id="-1" type="driving"><speed sOffset="0" max="5.56"/></lane></right>
        <left><lane id="1" type="driving"/></left>
      </laneSection>
    </lanes>
  </road>
  <road id="8" junction="3" length="20">
    <lanes><laneSection s="0"><right><lane id="-1"><speed sOffset="0" max="3.65"/></lane></right></laneSection></lanes>
  </road>
</OpenDRIVE>"""


def test_speed_limits_from_opendrive():
    assert incar._kmh(8.33, None) == 30 and incar._kmh(13.89, "m/s") == 50 and incar._kmh(3.65, None) == 13
    assert incar._kmh(50, "km/h") == 50 and incar._kmh(30, "mph") == 48
    sl = incar.SpeedLimits(XODR)
    assert len(sl) == 2
    assert sl.lookup(7, 10.0, -1) == (30, False)
    assert sl.lookup(7, 10.0, 1) == (50, False)
    assert sl.lookup(7, 150.0, -1) == (20, False)
    assert sl.lookup(7, 150.0, 1) == (50, False)           # no lane speed: the road type's
    assert sl.lookup(8, 5.0, -1) == (13, True)              # a junction connector
    assert sl.lookup(99, 0, 1) == (None, False)


def test_limit_watcher_keeps_the_last_road_on_junctions():
    wps = {"road": types.SimpleNamespace(road_id=7, s=10.0, lane_id=-1, is_junction=False),
           "junction": types.SimpleNamespace(road_id=8, s=5.0, lane_id=-1, is_junction=True)}
    where = ["road"]
    cmap = types.SimpleNamespace(to_opendrive=lambda: XODR, get_waypoint=lambda loc: wps[where[0]])
    lw = incar.LimitWatcher(cmap, log=lambda s: None)
    for _ in range(100):
        if lw.limits is not None:
            break
        import time
        time.sleep(0.01)
    assert lw.update(0.0, None) == 30
    where[0] = "junction"
    assert lw.update(0.2, None) == 30                        # not looked up again within 0.5 s
    assert lw.update(1.0, None) == 30                        # a connector keeps the road's limit


class _Ctl:
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0, reverse=False):
        self.throttle, self.brake, self.steer, self.reverse = throttle, brake, steer, reverse


def test_messages_takeover_and_values():
    ev = []
    cfg = {"messages": [
        {"at": 1, "seconds": 2, "display": "hud", "text": "Follow the road"},
        {"at_m": 50, "seconds": 3, "display": "centre", "text": "Roadworks"},
        {"at": 5, "seconds": 4, "takeover": True, "text": "TAKE OVER"},
        {"on_state": "TAKEOVER", "seconds": 1, "display": "cluster", "text": "Hands on the wheel"}]}
    ic = incar.InCar(cfg, "driver", (1280, 720), log=lambda s: None, on_event=lambda *a: ev.append(a[:3]),
                     sim_time="2026-11-15 14:30")
    assert "centre" in ic.displays and ic.displays["centre"]["design"]["name"] == "message panel"
    t, kmh = 0.0, 36.0
    shown = {}
    while t < 12.0:
        ctl = _Ctl(throttle=0.3) if t < 6.5 else _Ctl(brake=0.6)
        ic.update(t, kmh, lights=incar.LEFT_BLINKER | incar.LOW_BEAM, control=ctl, limit=30, dt=0.05)
        for m, _u, target in ic.shown:
            shown.setdefault(m["text"], (round(t, 2), target))
        if t == 5.5:
            assert ic.state == "TAKEOVER" and ic.conds["takeover"] and ic.values["time_left"] == "3.5"
        t = round(t + 0.05, 2)
    assert shown["Follow the road"] == (1.0, "centre")          # no HUD configured: the centre screen
    assert shown["Roadworks"][0] == pytest.approx(5.0, abs=0.06)     # 50 m at 10 m/s
    assert shown["TAKE OVER"] == (5.0, "all")
    assert shown["Hands on the wheel"][0] == pytest.approx(5.05, abs=0.06)   # the state started
    kinds = [e[0] for e in ev]
    assert kinds.count("takeover_request") == 1 and kinds.count("message") == 4
    resp = [e for e in ev if e[0] == "takeover_response"]
    assert resp and resp[0][2] == "1.50 s"                    # braked at 6.5 s
    assert ic.state == "MANUAL" and ic.values["reaction"] == "1.50"
    assert ic.values["speed"] == 36 and ic.values["limit"] == 30 and ic.conds["over_limit"]
    assert ic.values["gear"] == "D2" and ic.conds["headlights"] and ic.conds["blinkers"]
    assert ic.values["clock"] == "14:30" and ic.values["distance"] == "0.1"
    passenger = incar.InCar({"centre": {"design": {"width": 0.2, "height": 0.1, "states": []}}}, "av_passenger",
                            (800, 600), log=lambda s: None)
    passenger.update(0.0, 20.0)
    assert passenger.state == "AUTOMATED" and passenger.values["mode"] == "Automated"
    assert passenger.displays["centre"]["rect"][2] == int(0.24 * 800)


def test_displays_draw_with_pygame():
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    pygame.init()
    cfg = {"cluster": True, "hud": "hud_speed", "centre": "centre_screen",
           "messages": [{"at": 0, "seconds": 5, "display": "cluster", "text": "Pedestrian ahead"}]}
    ic = incar.InCar(cfg, "driver", (1280, 720), log=lambda s: None)
    assert set(ic.displays) == {"cluster", "hud", "centre"}
    ic.update(0.0, 47.0, lights=incar.RIGHT_BLINKER, control=_Ctl(throttle=0.4), limit=50)
    screen = pygame.Surface((1280, 720))
    screen.fill((90, 120, 150))
    ic.draw(screen)
    for name, d in ic.displays.items():
        x, y, w, h = d["rect"]
        assert 0 <= x and x + w <= 1280 and 0 <= y and y + h <= 720
        px = pygame.surfarray.array3d(screen.subsurface((x, y, w, h)))
        assert (px != (90, 120, 150)).any(axis=2).mean() > 0.01, name     # something drawn there
    # every element type
    painter = incar.Painter(pygame)
    from coupledcarla import designs
    d = designs.load({"width": 1.0, "height": 0.5, "background": "#00000080", "states": {"ANY": [
        {"type": t, "x": 0.05 + 0.1 * i, "y": 0.1, "w": 0.09, "h": 0.8, "value": "{speed}", "text": "{speed}",
         "direction": "up", "animation": "pulse", "color": "#FF000080"}
        for i, t in enumerate(["rect", "lightbar", "circle", "arrow", "chevrons", "text", "gauge", "bar",
                               "speedsign"])]}})
    surf = painter.panel(d, (400, 200), "MANUAL", 0.3, {"speed": 40}, {}, limit=50)
    assert surf.get_size() == (400, 200)


def test_haptic_patterns_and_udp():
    assert haptics.strength([{"t": 0.0, "duration": 1.2, "pattern": "triple", "intensity": 0.8}], 0.1) == 0.8
    assert haptics.strength([{"t": 0.0, "duration": 1.2, "pattern": "triple", "intensity": 0.8}], 0.35) == 0.0
    assert haptics.strength([{"t": 0.0, "duration": 1.2, "pattern": "triple", "intensity": 0.8}], 1.3) == 0.0
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2.0)
    port = rx.getsockname()[1]
    wheel = types.SimpleNamespace(got=[])
    wheel.vibrate = lambda s: wheel.got.append(s)
    cues = []
    h = haptics.Haptics({"udp": [f"127.0.0.1:{port}"], "gamepad": False,
                         "cues": [{"on": "over_limit", "pattern": "double"}, {"at": 3, "name": "attention"},
                                  {"on": "ehmi:yielding", "intensity": 0.4}]},
                        wheel=wheel, role="driver", log=lambda s: None, on_cue=lambda *a: cues.append(a))
    h.cue("takeover", 1.0)
    msg = json.loads(rx.recv(1000))
    assert msg == {"cue": "takeover", "intensity": 1.0, "duration": 1.2, "pattern": "triple", "time": 1.0,
                   "role": "driver"}
    assert h.tick(1.1) == 1.0 and wheel.got[-1] == 1.0
    assert h.tick(1.35) == 0.0 and h.tick(3.0) == 0.0          # between pulses, then over
    over = [False, True, True, False, True]
    for i, o in enumerate(over):
        h.events(2.0 + i * 0.5, {"over_limit": o}, ehmi_near="yielding" if i == 2 else None)
    names = [c[1] for c in cues]
    assert names.count("over_limit") == 2                       # once per rising edge
    assert names.count("attention") == 1 and names.count("ehmi:yielding") == 1
    h.close()
    assert wheel.got[-1] == 0.0
    rx.close()


def test_wheel_vibrate_mixes_into_the_road_effect(monkeypatch):
    pytest.importorskip("pygame")
    from coupledcarla.inputs import steering_wheel as sw
    w = sw.SteeringWheel.__new__(sw.SteeringWheel)
    w._haptic, w._cue = None, 0.0
    w.vibrate(0.7)
    assert w._cue == 0.7
    w.vibrate(3.0)
    assert w._cue == 1.0
