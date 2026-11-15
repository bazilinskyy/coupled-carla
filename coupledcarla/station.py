"""A participant's station in a coupled experiment (coupled-sim's client).

    python -m coupledcarla.station --host <host machine> --role pedestrian
    python -m coupledcarla.station --host 131.155.x.y --role driver --input wheel
    python -m coupledcarla.station --host 127.0.0.1 --role av_passenger --input bot --headless --frames-every 2
    python -m coupledcarla.station --list-devices      # audio outputs, joysticks and screens of this machine

The station connects to the host's control channel (port 2100) to learn its role (driver, pedestrian,
av_passenger) and, at each trial start, which actor is theirs; and to the CARLA server (port 2000) to render
its own camera and apply its own controls directly, without a detour through the host. It stays connected
across trials and follows the host's trial sequence.

Controls
  driver        W/S or arrows, A/D steer, Q reverse, Space hand brake, Z/X blinkers, R back onto the lane
                (or --input wheel: Logitech G25 with force feedback)
  pedestrian    W/S forward/back, A/D sidestep, mouse or left/right arrows to look, Shift to run
  av_passenger  1-9 choose the AV's eHMI message (listed on screen), 0 switches it off
  drone         mode 2 sticks: W/S forward, A/D sideways, Up/Down climb, Left/Right turn, I/K gimbal, C view,
                M position/attitude mode, H return home (or --input gamepad / rc: an Xbox pad, an RC transmitter);
                the host flies the drone (drones.py) and moves this station's camera
  everyone      F visual sync (red bar on every station for 1 s), F3 sound calibration tone (with sound), Esc quit
  responses     B hold-to-respond button, N/M slider down/up (gamepad: right shoulder/trigger, d-pad; wheel: right
                wheel button, paddles), when the experiment gives the role a `response`
  automated     steer, brake or press the throttle to take over, Enter (G25 button 2) hands back to the automation;
                T requests a take-over (experimenter)
--input bot drives the role without a human (tests): the driver on CARLA's autopilot, the pedestrian along the
path in the experiment's `bot`, the passenger by the eHMI schedule in `bot`.

Lab options: --display N and --fullscreen put the window on screen N; --audio-device NAME plays the sound there
(two stations on one PC need two headsets); --camera-hz caps the camera rate (a remote station's images cross the
LAN raw: 1280x720 is 3.7 MB an image, so about 30 Hz fill gigabit Ethernet). Every 5 s the station reports to the
host (station_stats events): round trip, loop and camera frame rates, ticks and images received, tick-to-image
latency, the time to its first image, and which of sound, displays and haptics are on (labcheck.py, the dry run).

Each station writes logs/stations/<role>_<time>.csv: one row per rendered frame with wall-clock time, CARLA
frame and simulation time, round trip to the host, and the visual-sync flag, for aligning with eye trackers,
video or physiology recorded at that station.

With the experiment's `sound`, `displays`, `haptics` or a station-drawn designed eHMI, the station also plays the
soundscape, draws the in-car displays and messages and gives haptic cues (stationux.py), and logs each cue to
logs/stations/<role>_<time>_cues.csv.
"""
import argparse
import csv
import math
import socket
import threading
import time
from pathlib import Path

import carla
import numpy as np

from . import labcheck, multigpu
from .compat import listen, set_manual_exposure
from .coupling import ControlClient
from .studytools import CSV_COLUMNS, StationStudy

VIEWS = {
    "driver": carla.Transform(carla.Location(x=-0.05, y=-0.38, z=1.22)),
    "passenger": carla.Transform(carla.Location(x=-0.15, y=0.36, z=1.15)),   # front seat, right
    "chase": carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-10)),
    "hood": carla.Transform(carla.Location(x=1.6, z=1.6)),
}
EYE = 0.72        # metres above a walker's actor origin (its centre) to its eyes
EXIT_TRIAL_DONE, EXIT_HOST_CLOSED, EXIT_QUIT = 0, 3, 4


class Station:
    def __init__(self, args):
        self.args = args
        self.pending = []           # this tick's CARLA commands, sent in one batch (_flush)
        self.size = tuple(int(v) for v in args.size.split("x"))
        self.running = False
        self.control = ControlClient(args.control_host or args.host, args.control_port, args.role,
                                     on_message=self._on_message)
        self.welcome = None
        self.client = self.world = self.actor = self.camera = None
        self.image = None
        self.flash_until = 0.0
        self.heading = 0.0
        self.input = None
        self.ehmi_states = []
        self.ehmi = "off"
        self.bot = None
        self.drone_hud = []         # the drone pilot's HUD lines, from the host
        self.ux = None          # sound, in-car displays, haptics, station-drawn eHMIs (stationux.py), when asked for
        self.screen = None
        self.frames = 0
        self.fps_t0, self.fps_n, self.fps = time.time(), 0, 0.0
        self.count = labcheck.Counter()         # ticks, images, time to the first image (the stats)
        self.got_image = threading.Event()
        self.camera_go = threading.Event()      # multi-GPU camera turn granted by the host
        self.routing = None                     # multigpu.StationRouting, from the welcome
        self.last_saved = -1e9
        self.t0_sim = None
        self.trial = None
        self.tick_lag, self.image_lag, self.seen = [], [], {}      # ms after the host's tick: snapshot seen, camera image here
        self.study = None          # responses and take-over requests (studytools.StationStudy), from the welcome
        out = Path(args.out) / "stations"
        out.mkdir(parents=True, exist_ok=True)
        self.log_f = open(out / f"{args.role}_{time.strftime('%Y%m%d-%H%M%S')}.csv", "w", newline="", buffering=1)
        self.log_w = csv.writer(self.log_f, delimiter=";")
        self.log_w.writerow(["wall_time", "frame", "sim_time", "trial", "rtt_ms", "flash", "speed_kmh", "ehmi",
                             "clock_offset_ms", "tick_lag_ms", "image_lag_ms", *CSV_COLUMNS])
        if not args.headless:
            import pygame
            pygame.init()
            flags = pygame.HWSURFACE | pygame.DOUBLEBUF | (pygame.FULLSCREEN if args.fullscreen else 0)
            try:
                self.screen = pygame.display.set_mode(self.size, flags, display=args.display)
            except (TypeError, pygame.error):      # pygame 1.x, or no such screen
                self.screen = pygame.display.set_mode(self.size, flags)
            pygame.display.set_caption(f"coupled-carla: {args.role}")
            self.font = pygame.font.SysFont("monospace", 22, bold=True)
            self.big = pygame.font.SysFont("sans", 36, bold=True)

    # -- trial life cycle -------------------------------------------------------------------------------------
    def _on_message(self, msg):
        """Runs in the control reader thread. The end of a trial ends this (child) process at once: the host
        stops the CARLA server right after, and a process blocked in a CARLA call would hang or abort."""
        if msg.get("type") == "camera_go":
            self.camera_go.set()
        if msg.get("type") != "state":
            return
        if (msg.get("state") == "ended" and self.running) or msg.get("state") == "closed":
            try:
                self.log_f.flush()
                self.control.send({"type": "ended_ack"})
            except (OSError, ValueError):
                pass
            print(f"[station {self.args.role}] trial ended", flush=True)
            if self.ux is not None:
                self.ux.stop()      # closes the sound recording
            import os
            os._exit(EXIT_HOST_CLOSED if msg["state"] == "closed" else EXIT_TRIAL_DONE)

    def run(self):
        """One trial: wait for it, run it, return an exit code for the supervisor (see main)."""
        print(f"[station {self.args.role}] waiting for the host", flush=True)
        state = "loading"
        while True:
            for msg in self.control.poll():
                kind = msg.get("type")
                if kind == "welcome":
                    self.welcome = msg
                    self.ehmi_states = msg.get("ehmi_states", [])
                    self.study = StationStudy(msg, self.args, self.control.send)
                    print(f"[station {self.args.role}] {msg.get('kind')} in {msg.get('experiment')}", flush=True)
                elif kind == "state":
                    if msg["state"] == "running" and state != "running" and msg.get("actor_id"):
                        self.start(msg)
                        state = "running"
                        self.running = True
                    elif msg["state"] == "ended" and state == "running":
                        self.stop()
                        return EXIT_TRIAL_DONE
                    if msg["state"] == "closed":
                        if state == "running":
                            self.stop()
                        return EXIT_HOST_CLOSED
                elif kind == "flash":
                    self.flash_until = time.time() + 1.0
                elif kind in ("ehmi_state", "drones") and self.ux is not None:
                    self.ux.on_message(msg)
                elif kind == "drone_hud":
                    self.drone_hud = msg.get("lines") or []
                    if self.ux is not None:
                        self.ux.on_message(msg)
                elif self.study is not None:
                    self.study.on_message(msg)      # take-over state, pause
            if state == "running":
                if not self.tick():
                    self.stop()
                    return EXIT_QUIT
            else:
                self.idle_screen(state)
                time.sleep(0.05)

    def start(self, msg):
        args = self.args
        self.count.running()
        while True:   # the host restarts the CARLA server for every trial: wait for its port
            try:
                socket.create_connection((args.host, args.port), timeout=2).close()
                break
            except OSError:
                time.sleep(1)
        if self.routing is None:      # multi-GPU: a relay when secondaries are on other machines (multigpu.py)
            self.routing = multigpu.StationRouting(self.welcome.get("multigpu"), args.host, args.port,
                                                   log=lambda m: print(f"[station {args.role}] {m}", flush=True))
        self.client = carla.Client(*self.routing.address())
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        self.trial = msg.get("trial")       # before the camera exists: its callback names the frames folder by it
        for _ in range(100):
            self.actor = self.world.get_actor(msg["actor_id"])
            if self.actor:
                break
            self.world.wait_for_tick(5.0)
        kind = self.welcome.get("kind")
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.size[0]))
        bp.set_attribute("image_size_y", str(self.size[1]))
        bp.set_attribute("fov", str(args.fov))
        # CARLA 1.0: exposure from a post-process profile with auto exposure, which adapts only over consecutive
        # frames: a headless camera renders every tick (unless --camera-hz) and frames are kept every --frames-every
        # (compat.setup_camera). CARLA 0.9.x: a slow headless camera gets a fixed manual exposure instead.
        self.every_tick = bp.has_attribute("post_process_profile")
        if self.every_tick and self.welcome.get("camera_profile", "Dashcam"):
            bp.set_attribute("post_process_profile", str(self.welcome.get("camera_profile", "Dashcam")))
        hz = self.camera_hz()
        if hz > 0:
            bp.set_attribute("sensor_tick", str(1.0 / hz))
        for k, v in (self.welcome.get("camera_attributes") or {}).items():     # the experiment's film.attributes
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        if not self.every_tick and args.headless and 0 < hz <= 5:
            # few frames: auto exposure would never adapt
            set_manual_exposure(bp, {"pedestrian": 400, "av_passenger": 500}.get(self.welcome.get("kind"), 220))
        if kind == "pedestrian":
            self.heading = self.actor.get_transform().rotation.yaw

            def spawn():
                return self.world.spawn_actor(bp, self._eye_transform())       # moved every tick
        elif kind == "drone":
            def spawn():       # the host moves it with the drone every tick (gimbal, line of sight or chase)
                return self.world.spawn_actor(bp, self.actor.get_transform())
        else:
            view = self.welcome.get("view") or ("passenger" if kind == "av_passenger" else "driver")

            def spawn():
                return self.world.spawn_actor(bp, VIEWS.get(view, VIEWS["driver"]), attach_to=self.actor,
                                              attachment_type=carla.AttachmentType.Rigid)
        # with secondaries the camera should land on the role's own one (multigpu.py); otherwise spawn + listen
        # (a -nullrhi server without a rendering secondary streams nothing: the station goes on without images)
        say = lambda m: print(f"[station {args.role}] {m}", flush=True)  # noqa: E731

        def listen_cam(cam):
            self.no_render = not listen(cam, self._on_image, say, "camera")
        self.camera = self.routing.place_camera(spawn, listen_cam,
                                                self._destroy, self.got_image,
                                                turn=multigpu.CameraTurn(self.control.send, self.camera_go))
        bot = self.welcome.get("bot") or {}
        if kind == "driver":
            from . import inputs
            name = "autopilot" if args.input == "bot" else args.input
            self.input = inputs.make(name, self.actor)
            if hasattr(self.input, "send"):          # its light state goes with this tick's batch
                self.input.send = self.pending.append
            if hasattr(self.input, "restart"):
                self.input.restart(self.world.get_map().get_waypoint(self.actor.get_location()))
        elif kind == "pedestrian" and args.input == "bot":
            from .ai import PathWalker
            pts = [carla.Location(x=x, y=y) for x, y in bot.get("path", [])]
            self.bot = PathWalker(self.actor, pts, float(bot.get("speed", 1.4)), float(bot.get("start_s", 0)))
            self.bot.apply = lambda c, i=self.actor.id: self.pending.append(carla.command.ApplyWalkerControl(i, c))
            self.look_at = None
            if bot.get("look"):      # the bot keeps its eyes on this actor (role name), as a pedestrian would
                found = [a for a in self.world.get_actors() if a.attributes.get("role_name") == bot["look"]]
                self.look_at = found[0] if found else None
        elif kind == "av_passenger" and args.input == "bot":
            self.bot = sorted(bot.get("ehmi", []))
        elif kind == "drone":
            self.control.send({"type": "drone_camera", "camera_id": self.camera.id})
            if args.input == "bot":
                self.control.send({"type": "drone_input", "bot": True})
            else:
                from .droneinput import PygameInput
                self.input = PygameInput(args.input if args.input in ("gamepad", "rc") else "keyboard",
                                         self.welcome.get("drone_input") or {})
        if self.study is not None:
            self.study.start(self.actor, self.world, self.input, self.client)
        self.t0_sim = None
        self.trial = msg.get("trial")
        from .stationux import StationUX, wanted
        if wanted(self.welcome):
            self.ux = StationUX(self.welcome, args.role, Path(self.log_f.name).with_suffix(""), self.size,
                                args.headless, audio_device=args.audio_device)
            self.ux.start(self.world, self.actor, self.camera, self.trial, msg.get("agents"), self.input)
        print(f"[station {args.role}] trial {self.trial}: actor {self.actor.id} {self.actor.type_id}", flush=True)

    def camera_hz(self):
        """The camera's rate: --camera-hz, else every tick with a window and 1/--frames-every (1 Hz) headless."""
        a = self.args
        if a.camera_hz is not None:
            return max(0.0, float(a.camera_hz))
        if getattr(self, "every_tick", False):      # CARLA 1.0: every tick, so auto exposure adapts
            return 0.0
        return (1.0 / (a.frames_every or 1.0)) if a.headless else 0.0

    @staticmethod
    def _destroy(cam):
        try:
            cam.stop()
            cam.destroy()
        except RuntimeError:
            pass

    def stop(self):
        if self.ux is not None:
            self.ux.stop()
            self.ux = None
        if self.camera is not None:
            try:
                self.camera.stop()
                self.camera.destroy()
            except RuntimeError:
                pass
        self.camera = self.actor = self.input = self.bot = None
        self.image = None
        # Let go of the CARLA client before the host stops the server: a client whose server goes away
        # while it still holds streams can abort the whole process.
        self.world = self.client = None
        import gc
        gc.collect()
        try:
            self.control.send({"type": "ended_ack"})
        except OSError:
            pass
        print(f"[station {self.args.role}] trial ended", flush=True)

    # -- per tick ---------------------------------------------------------------------------------------------
    def tick(self):
        try:
            snap = self.world.wait_for_tick(10.0)
        except RuntimeError:
            return True      # host busy (or between ticks): keep listening to the control channel
        # the snapshot usually arrives before the host's tick message for it: keep the time, pair it up later
        self.seen[snap.frame] = time.time()
        self.count.frame(snap.frame)
        lag = None
        for f in [f for f in self.seen if f < snap.frame - 5]:
            lag_f = self.control.lag_ms(f, self.seen.pop(f))
            if lag_f is not None:
                self.tick_lag = (self.tick_lag + [lag_f])[-500:]
                lag = lag_f
        if self.actor is None:
            return True
        t_sim = snap.timestamp.elapsed_seconds
        self.t0_sim = t_sim if self.t0_sim is None else self.t0_sim
        t = t_sim - self.t0_sim
        events = []
        if self.screen is not None:
            import pygame
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                    self.control.send({"type": "bye"})
                    return False
                if e.type == pygame.KEYDOWN and e.key == pygame.K_f:
                    self.control.send({"type": "sync"})
        kind = self.welcome.get("kind")
        try:
            if kind == "driver":
                ctl = self.input.control(self.actor, events)
                drive = self.study.drive(ctl, events) if self.study is not None else True   # automation drives?
                if ctl is not None and drive:
                    self.pending.append(carla.command.ApplyVehicleControl(self.actor.id, ctl))
                if getattr(self.input, "reset", False):
                    self._reset_to_lane()
            elif kind == "pedestrian":
                if self.bot is not None:
                    self.bot.step(t)
                    if getattr(self, "look_at", None) is not None:
                        a, b = self.actor.get_location(), self.look_at.get_location()
                        want = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
                        self.heading += ((want - self.heading + 180) % 360 - 180) * 0.1
                else:
                    self._walk(events)
                self.pending.append(carla.command.ApplyTransform(self.camera.id, self._eye_transform()))
            elif kind == "av_passenger":
                self._ehmi_keys(events, t)
            elif kind == "drone" and self.input is not None:
                st = self.input.poll(events)
                self.control.send({"type": "drone_input", "s": st.as_list(), "g": round(st.gimbal, 3),
                                   "e": [e for e in st.events if e != "quit"]})
            self._flush()
        except RuntimeError:
            self.pending.clear()
            return True      # our actor went away (end of trial); the host will say so
        if self.ux is not None:
            self.ux.tick(snap, t, events, heading=self.heading if kind == "pedestrian" else None,
                         tor=self.study.tor if self.study is not None else None)
        cells = self.study.sample(t, snap.frame, self.screen) if self.study is not None else [""] * len(CSV_COLUMNS)
        self.render(t)
        v = self.actor.get_velocity()
        rtt = self.control.rtt()
        off = self.control.clock_offset()
        self.log_w.writerow([f"{time.time():.4f}", snap.frame, f"{t:.3f}", self.trial,
                             f"{rtt:.1f}" if rtt is not None else "", int(time.time() < self.flash_until),
                             f"{3.6 * math.hypot(v.x, v.y):.2f}", self.ehmi,
                             f"{1000 * off:.2f}" if off is not None else "",
                             f"{lag:.1f}" if lag is not None else "",
                             f"{self.image_lag[-1]:.1f}" if self.image_lag else "", *cells])
        self.frames += 1
        self.fps_n += 1
        if kind == "pedestrian" and self.frames % 10 == 0:      # where the pedestrian looks (eye contact)
            self.control.send({"type": "gaze", "yaw": round(self.heading, 1)})
        if time.time() - self.fps_t0 >= 5.0:
            self.fps = self.fps_n / (time.time() - self.fps_t0)
            self.fps_t0, self.fps_n = time.time(), 0
            self.count.window(0.0)
            self.control.send({"type": "stats", "rtt_ms": round(rtt, 2) if rtt is not None else None,
                               "fps": round(self.fps, 1), "frames": self.frames, "host": socket.gethostname(),
                               "latency": self.latency(), **{k: v for k, v in self.count.stats().items()
                                                             if k not in ("fps", "frames")},
                               "ticks_seen": self.count.frames, "camera_hz": self.camera_hz(),
                               "size": self.args.size, "ux": labcheck.ux_flags(self.welcome, self.ux),
                               "camera_on": self.routing.placed if self.routing is not None else None,
                               "no_render": bool(getattr(self, "no_render", False))})
        return True

    def _eye_transform(self):
        yaw = math.radians(self.heading)   # a little in front of the face, so the walker's own head is not in view
        loc = self.actor.get_location() + carla.Location(x=0.22 * math.cos(yaw), y=0.22 * math.sin(yaw), z=EYE)
        return carla.Transform(loc, carla.Rotation(yaw=self.heading, pitch=-3.0))

    def _walk(self, events):
        import pygame
        k = pygame.key.get_pressed()
        for e in events:
            if e.type == pygame.MOUSEMOTION and pygame.mouse.get_pressed()[0]:
                self.heading += e.rel[0] * 0.25
        self.heading += (-90 if k[pygame.K_LEFT] else 0) * 0.02 + (90 if k[pygame.K_RIGHT] else 0) * 0.02
        fwd = (1 if (k[pygame.K_w] or k[pygame.K_UP]) else 0) - (1 if (k[pygame.K_s] or k[pygame.K_DOWN]) else 0)
        side = (1 if k[pygame.K_d] else 0) - (1 if k[pygame.K_a] else 0)
        yaw = math.radians(self.heading)
        dx = fwd * math.cos(yaw) - side * math.sin(yaw)
        dy = fwd * math.sin(yaw) + side * math.cos(yaw)
        n = math.hypot(dx, dy)
        speed = (3.0 if (k[pygame.K_LSHIFT] or k[pygame.K_RSHIFT]) else 1.4) if n else 0.0
        self.pending.append(carla.command.ApplyWalkerControl(self.actor.id, carla.WalkerControl(
            direction=carla.Vector3D(dx / n if n else 0.0, dy / n if n else 0.0, 0.0), speed=speed)))

    def _flush(self):
        """This tick's controls, light state and camera move in one blocking batch: on CARLA 1.0 two or more
        fire-and-forget calls in a row hold up the next blocking call by ~40 ms (its rpclib lacks TCP_NODELAY,
        carla-simulator/carla#10305), as on the host (side.py)."""
        if self.pending:
            cmds = list(self.pending)
            self.pending.clear()      # in place: the input's send is this list's append
            self.client.apply_batch_sync(cmds, False)

    def _ehmi_keys(self, events, t):
        new = None
        if self.screen is not None:
            import pygame
            for e in events:
                if e.type == pygame.KEYDOWN and pygame.K_0 <= e.key <= pygame.K_9:
                    i = e.key - pygame.K_0
                    new = "off" if i == 0 else (self.ehmi_states[i - 1] if i - 1 < len(self.ehmi_states) else None)
        while isinstance(self.bot, list) and self.bot and self.bot[0][0] <= t:
            new = self.bot.pop(0)[1]
        if new and new != self.ehmi:
            self.ehmi = new
            self.control.send({"type": "ehmi", "state": new})

    def _reset_to_lane(self):
        wp = self.world.get_map().get_waypoint(self.actor.get_location(), project_to_road=True,
                                               lane_type=carla.LaneType.Driving)
        tf = wp.transform
        tf.location.z += 0.4
        self.actor.set_target_velocity(carla.Vector3D())
        self.actor.set_transform(tf)
        self.input.reset = False

    def latency(self):
        """Median and 95th percentile of the lags so far (ms), and the clock offset used."""
        def q(xs):
            xs = sorted(xs)
            return {"median": round(xs[len(xs) // 2], 1), "p95": round(xs[int(0.95 * (len(xs) - 1))], 1),
                    "n": len(xs)} if xs else None
        off = self.control.clock_offset()
        return {"clock_offset_ms": round(1000 * off, 2) if off is not None else None,
                "tick_to_station_ms": q(self.tick_lag), "tick_to_image_ms": q(self.image_lag)}

    # -- drawing ----------------------------------------------------------------------------------------------
    def _on_image(self, image):
        lag = self.control.lag_ms(image.frame, time.time())
        if lag is not None:
            self.image_lag = (self.image_lag + [lag])[-500:]
        self.count.image()
        self.got_image.set()
        if self.screen is None:
            # a camera that renders at least twice per kept frame keeps its frames on a grid of simulation time
            # (multiples of --frames-every), so several stations' frames (and the host's film cameras, film.py)
            # line up image for image (tools/make_videos.py); a slower camera keeps every image after the interval
            every = self.args.frames_every
            hz = self.camera_hz()
            if every and (hz == 0 or hz * every >= 2.0):
                k = round(image.timestamp / every)
                keep = k != self.last_saved and abs(image.timestamp - k * every) < max(0.011, 0.05 * every)
            else:
                k = image.timestamp
                keep = bool(every) and image.timestamp - self.last_saved >= 0.95 * every
            if keep:
                self.last_saved = k
                out = Path(self.args.out) / "stations" / f"{self.args.role}_trial{self.trial}_frames"
                out.mkdir(parents=True, exist_ok=True)
                if self.ux is not None and self.ux.wants_frames():     # with the in-car displays drawn over it
                    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
                    self.ux.save_frame(arr[:, :, 2::-1], out / f"{image.frame:08d}.jpg", self.args.fov)
                    return
                image.save_to_disk(str(out / f"{image.frame:08d}.jpg"))
            return
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        self.image = arr[:, :, 2::-1]

    def render(self, t):
        if self.screen is None or self.image is None:
            return
        import pygame
        self.screen.blit(pygame.surfarray.make_surface(self.image.swapaxes(0, 1)), (0, 0))
        v = self.actor.get_velocity()
        rtt = self.control.rtt()
        lines = [f"{self.args.role}  {3.6 * math.hypot(v.x, v.y):5.1f} km/h  t {t:6.1f} s",
                 f"host {rtt:.1f} ms  {self.fps:.0f} fps" if rtt is not None else ""]
        if self.welcome.get("kind") == "drone":
            lines = [lines[1]] + list(self.drone_hud)
        if self.welcome.get("kind") == "av_passenger":
            lines.append(f"eHMI: {self.ehmi}   " + "  ".join(f"{i + 1} {s}" for i, s in enumerate(self.ehmi_states))
                         + "  0 off")
        for i, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (255, 255, 255)), (16, 12 + 26 * i))
        if self.ux is not None:
            self.ux.draw(self.screen, self.args.fov)
        if self.study is not None:
            self.study.draw(self.screen, self.font, self.big)
        if time.time() < self.flash_until:
            pygame.draw.rect(self.screen, (255, 0, 0), (0, 0, self.size[0], 60))
        pygame.display.flip()

    def idle_screen(self, state):
        if self.screen is None:
            return
        import pygame
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                raise SystemExit(EXIT_QUIT)
        self.screen.fill((24, 28, 34))
        text = {"loading": "waiting for the experiment to start", "ended": "trial over - please wait"}.get(state, state)
        self.screen.blit(self.big.render(f"{self.args.role}: {text}", True, (230, 230, 230)), (40, self.size[1] // 2))
        pygame.display.flip()


def list_devices():
    """Prints this machine's audio outputs, joysticks (wheels, gamepads) and screens, for --audio-device and
    --display, and to check that the wheel and headphones are seen before a session."""
    import pygame
    pygame.init()
    try:
        from pygame._sdl2 import audio as sdl_audio
        outs = sdl_audio.get_audio_device_names(False)
    except Exception as e:  # noqa: BLE001 - pygame 1.x or no audio subsystem
        outs = [f"(cannot list: {e})"]
    print("audio outputs (--audio-device):")
    for name in outs:
        print(f"  {name}")
    pygame.joystick.init()
    print("joysticks:")
    for i in range(pygame.joystick.get_count()):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"  {i}: {j.get_name()}  axes {j.get_numaxes()}, buttons {j.get_numbuttons()}, hats {j.get_numhats()}")
    if not pygame.joystick.get_count():
        print("  (none)")
    try:
        sizes = pygame.display.get_desktop_sizes()
    except (AttributeError, pygame.error):
        sizes = []
    print("screens (--display):")
    for i, (w, h) in enumerate(sizes):
        print(f"  {i}: {w}x{h}")
    if not sizes:
        print("  (none found)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--list-devices", action="store_true", help="list audio outputs, joysticks and screens; exit")
    ap.add_argument("--host", default="127.0.0.1", help="the CARLA server (usually the host machine)")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--control-host", help="the host process, if not on the CARLA server's machine")
    ap.add_argument("--control-port", type=int, default=2100)
    ap.add_argument("--role")
    ap.add_argument("--input", default="keyboard", choices=["keyboard", "wheel", "bot", "gamepad", "rc"])
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fov", type=float, default=90)
    ap.add_argument("--headless", action="store_true", help="no window (with --input bot)")
    ap.add_argument("--frames-every", type=float, default=0.0, help="headless: save the view every N s")
    ap.add_argument("--out", default="logs")
    ap.add_argument("--response", help="button,slider[,hidden]: responses here even if the experiment has none")
    ap.add_argument("--camera-hz", type=float, help="camera images per second (0: every tick; default: every tick "
                                                    "with a window, 1/--frames-every headless)")
    ap.add_argument("--display", type=int, default=0, help="the screen for the window (0, 1, ...)")
    ap.add_argument("--fullscreen", action="store_true", help="full screen at --size on --display")
    ap.add_argument("--audio-device", help="play the sound on this output (names: --list-devices)")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.list_devices:
        list_devices()
        return
    if not args.role:
        ap.error("--role is required")
    if args.child:
        raise SystemExit(Station(args).run())
    # Supervisor: every trial runs in a fresh child process. CARLA's client library can abort a process whose
    # server goes away (the host restarts the server for every trial); a child that dies that way after its
    # trial costs nothing, and the next child joins the next trial.
    import subprocess
    import sys
    while True:
        code = subprocess.call([sys.executable, "-m", "coupledcarla.station", *sys.argv[1:], "--child"])
        if code in (EXIT_HOST_CLOSED, EXIT_QUIT):
            return
        if code != EXIT_TRIAL_DONE:
            print(f"[station {args.role}] trial process ended with code {code}; rejoining", flush=True)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
