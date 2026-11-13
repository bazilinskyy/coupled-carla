"""A participant's station in a coupled experiment (coupled-sim's client).

    python -m coupledcarla.station --host <host machine> --role pedestrian
    python -m coupledcarla.station --host 131.155.x.y --role driver --input wheel
    python -m coupledcarla.station --host 127.0.0.1 --role av_passenger --input bot --headless --frames-every 2

The station connects to the host's control channel (port 2100) to learn its role (driver, pedestrian,
av_passenger) and, at each trial start, which actor is theirs; and to the CARLA server (port 2000) to render
its own camera and apply its own controls directly, without a detour through the host. It stays connected
across trials and follows the host's trial sequence.

Controls
  driver        W/S or arrows, A/D steer, Q reverse, Space hand brake, Z/X blinkers, R back onto the lane
                (or --input wheel: Logitech G25 with force feedback)
  pedestrian    W/S forward/back, A/D sidestep, mouse or left/right arrows to look, Shift to run
  av_passenger  1-9 choose the AV's eHMI message (listed on screen), 0 switches it off
  everyone      F visual sync (red bar on every station for 1 s), Esc quit
--input bot drives the role without a human (tests): the driver on CARLA's autopilot, the pedestrian along the
path in the experiment's `bot`, the passenger by the eHMI schedule in `bot`.

Each station writes logs/stations/<role>_<time>.csv: one row per rendered frame with wall-clock time, CARLA
frame and simulation time, round trip to the host, and the visual-sync flag, for aligning with eye trackers,
video or physiology recorded at that station.
"""
import argparse
import csv
import math
import socket
import time
from pathlib import Path

import carla
import numpy as np

from .coupling import ControlClient

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
        self.screen = None
        self.frames = 0
        self.fps_t0, self.fps_n, self.fps = time.time(), 0, 0.0
        self.t0_sim = None
        out = Path(args.out) / "stations"
        out.mkdir(parents=True, exist_ok=True)
        self.log_f = open(out / f"{args.role}_{time.strftime('%Y%m%d-%H%M%S')}.csv", "w", newline="", buffering=1)
        self.log_w = csv.writer(self.log_f, delimiter=";")
        self.log_w.writerow(["wall_time", "frame", "sim_time", "trial", "rtt_ms", "flash", "speed_kmh", "ehmi"])
        if not args.headless:
            import pygame
            pygame.init()
            self.screen = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame.display.set_caption(f"coupled-carla: {args.role}")
            self.font = pygame.font.SysFont("monospace", 22, bold=True)
            self.big = pygame.font.SysFont("sans", 36, bold=True)

    # -- trial life cycle -------------------------------------------------------------------------------------
    def _on_message(self, msg):
        """Runs in the control reader thread. The end of a trial ends this (child) process at once: the host
        stops the CARLA server right after, and a process blocked in a CARLA call would hang or abort."""
        if msg.get("type") != "state":
            return
        if (msg.get("state") == "ended" and self.running) or msg.get("state") == "closed":
            try:
                self.log_f.flush()
                self.control.send({"type": "ended_ack"})
            except (OSError, ValueError):
                pass
            print(f"[station {self.args.role}] trial ended", flush=True)
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
            if state == "running":
                if not self.tick():
                    self.stop()
                    return EXIT_QUIT
            else:
                self.idle_screen(state)
                time.sleep(0.05)

    def start(self, msg):
        args = self.args
        while True:   # the host restarts the CARLA server for every trial: wait for its port
            try:
                socket.create_connection((args.host, args.port), timeout=2).close()
                break
            except OSError:
                time.sleep(1)
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
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
        if args.headless:
            bp.set_attribute("sensor_tick", str(args.frames_every or 1.0))
            bp.set_attribute("exposure_mode", "manual")   # few frames: auto exposure would never adapt
            bp.set_attribute("shutter_speed", {"pedestrian": "400", "av_passenger": "500"}.get(self.welcome.get("kind"), "220"))
        if kind == "pedestrian":
            self.heading = self.actor.get_transform().rotation.yaw
            self.camera = self.world.spawn_actor(bp, self._eye_transform())       # moved every tick
        else:
            view = self.welcome.get("view") or ("passenger" if kind == "av_passenger" else "driver")
            self.camera = self.world.spawn_actor(bp, VIEWS.get(view, VIEWS["driver"]), attach_to=self.actor,
                                                 attachment_type=carla.AttachmentType.Rigid)
        self.camera.listen(self._on_image)
        bot = self.welcome.get("bot") or {}
        if kind == "driver":
            from . import inputs
            name = "autopilot" if args.input == "bot" else args.input
            self.input = inputs.make(name, self.actor)
            if hasattr(self.input, "restart"):
                self.input.restart(self.world.get_map().get_waypoint(self.actor.get_location()))
        elif kind == "pedestrian" and args.input == "bot":
            from .ai import PathWalker
            pts = [carla.Location(x=x, y=y) for x, y in bot.get("path", [])]
            self.bot = PathWalker(self.actor, pts, float(bot.get("speed", 1.4)), float(bot.get("start_s", 0)))
            self.look_at = None
            if bot.get("look"):      # the bot keeps its eyes on this actor (role name), as a pedestrian would
                found = [a for a in self.world.get_actors() if a.attributes.get("role_name") == bot["look"]]
                self.look_at = found[0] if found else None
        elif kind == "av_passenger" and args.input == "bot":
            self.bot = sorted(bot.get("ehmi", []))
        self.t0_sim = None
        self.trial = msg.get("trial")
        print(f"[station {args.role}] trial {self.trial}: actor {self.actor.id} {self.actor.type_id}", flush=True)

    def stop(self):
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
                if ctl is not None:
                    self.actor.apply_control(ctl)
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
                self.camera.set_transform(self._eye_transform())
            elif kind == "av_passenger":
                self._ehmi_keys(events, t)
        except RuntimeError:
            return True      # our actor went away (end of trial); the host will say so
        self.render(t)
        v = self.actor.get_velocity()
        rtt = self.control.rtt()
        self.log_w.writerow([f"{time.time():.4f}", snap.frame, f"{t:.3f}", self.trial,
                             f"{rtt:.1f}" if rtt is not None else "", int(time.time() < self.flash_until),
                             f"{3.6 * math.hypot(v.x, v.y):.2f}", self.ehmi])
        self.frames += 1
        self.fps_n += 1
        if time.time() - self.fps_t0 >= 5.0:
            self.fps = self.fps_n / (time.time() - self.fps_t0)
            self.fps_t0, self.fps_n = time.time(), 0
            self.control.send({"type": "stats", "rtt_ms": round(rtt, 2) if rtt is not None else None,
                               "fps": round(self.fps, 1), "frames": self.frames, "host": socket.gethostname()})
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
        self.actor.apply_control(carla.WalkerControl(
            direction=carla.Vector3D(dx / n if n else 0.0, dy / n if n else 0.0, 0.0), speed=speed))

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

    # -- drawing ----------------------------------------------------------------------------------------------
    def _on_image(self, image):
        if self.screen is None:
            if self.args.frames_every:
                out = Path(self.args.out) / "stations" / f"{self.args.role}_trial{self.trial}_frames"
                out.mkdir(parents=True, exist_ok=True)
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
        if self.welcome.get("kind") == "av_passenger":
            lines.append(f"eHMI: {self.ehmi}   " + "  ".join(f"{i + 1} {s}" for i, s in enumerate(self.ehmi_states))
                         + "  0 off")
        for i, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (255, 255, 255)), (16, 12 + 26 * i))
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1", help="the CARLA server (usually the host machine)")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--control-host", help="the host process, if not on the CARLA server's machine")
    ap.add_argument("--control-port", type=int, default=2100)
    ap.add_argument("--role", required=True)
    ap.add_argument("--input", default="keyboard", choices=["keyboard", "wheel", "bot"])
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fov", type=float, default=90)
    ap.add_argument("--headless", action="store_true", help="no window (with --input bot)")
    ap.add_argument("--frames-every", type=float, default=0.0, help="headless: save the view every N s")
    ap.add_argument("--out", default="logs")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
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
