"""A station without --role (python -m coupledcarla.station --host <session machine>): it joins the session
dashboard's lobby (session.py), says which machine it is, which input devices and GPU it has, and waits until the
experimenter gives it a role; then it runs each trial in a fresh child process with that role, as a station started
with --role does. When the experimenter changes the role between trials, the next trial process has the new one.

The child processes are ordinary `station --role R` processes (plus --station-id, so the session knows which lobby
station each belongs to), which keeps plain hosts and --role stations working as before.
"""
import os
import platform
import socket
import subprocess
import sys
import time
import uuid

from .coupling import ControlClient

EXIT_TRIAL_DONE, EXIT_HOST_CLOSED, EXIT_QUIT = 0, 3, 4     # station.py's exit codes


def gpu_name():
    """The first NVIDIA GPU's name, else None (nvidia-smi missing: a Mac, an AMD or Intel GPU)."""
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True,
                             text=True, timeout=5)
        return out.stdout.strip().splitlines()[0] if out.returncode == 0 and out.stdout.strip() else None
    except (OSError, subprocess.SubprocessError, IndexError):
        return None


def input_devices(input_kind):
    """The input devices this station would use: its --input, plus the joysticks pygame sees (wheels, gamepads)."""
    found = ["bot (no human)"] if input_kind == "bot" else ["keyboard"]
    if input_kind == "bot":
        return found
    try:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        pygame.joystick.init()
        found += [pygame.joystick.Joystick(i).get_name() for i in range(pygame.joystick.get_count())]
        pygame.joystick.quit()
    except Exception:  # noqa: BLE001 - no pygame or no joystick subsystem: just the keyboard
        pass
    return found


def machine_info(args):
    return {"machine": args.name or socket.gethostname(), "inputs": input_devices(args.input), "gpu": gpu_name(),
            "input": args.input, "platform": platform.system(), "bot": args.input == "bot"}


class Lobby:
    """The station's lobby connection: hello without a role, then `lobby` / `assign` messages with its role."""

    def __init__(self, host, port, info, station_id, retry_s=300):
        self.station_id = station_id
        self.client = ControlClient(host, port, None, retry_s=retry_s,
                                    hello={"lobby": True, "station": station_id, "info": info})
        self.role, self.carla = None, {}
        self.closed = self.dropped = False
        self.refused = None

    def poll(self):
        """Reads what the session sent; True when the role changed."""
        changed = False
        for msg in self.client.poll():
            kind = msg.get("type")
            if kind in ("lobby", "assign"):
                role = msg.get("role") or None
                changed = changed or role != self.role
                self.role, self.carla = role, msg.get("carla") or {}
            elif kind == "lobby_refused":
                self.refused = msg.get("why") or "refused"
            elif kind == "state" and msg.get("state") == "closed":
                if msg.get("eof"):
                    self.dropped = True
                else:
                    self.closed = True
        return changed


def child_command(argv, role, station_id, carla, explicit_port, explicit_control_host, host):
    """The trial process's command line: this station's options plus the role (and the CARLA server the session
    uses, unless the station was told one)."""
    cmd = [sys.executable, "-m", "coupledcarla.station", *argv, "--role", role, "--station-id", station_id, "--child"]
    if not explicit_port and carla.get("port"):
        cmd += ["--port", str(int(carla["port"]))]
    if carla.get("host") and not explicit_control_host:     # CARLA on another machine than the session
        cmd += ["--control-host", host, "--host", str(carla["host"])]
    return cmd


def say(msg):
    print(f"[station] {msg}", flush=True)


def supervise(args, argv):
    """Runs the station in lobby mode until the session ends or the participant quits (Esc)."""
    sid = args.station_id or uuid.uuid4().hex[:8]
    info = machine_info(args)
    host = args.control_host or args.host
    child = child_role = None
    lobby = None
    try:
        while True:
            if lobby is None or lobby.dropped:
                if lobby is not None:
                    say("lost the session dashboard; reconnecting")
                say(f"joining the session at {host}:{args.control_port} as {info['machine']} "
                    f"({', '.join(info['inputs'])}{', ' + info['gpu'] if info['gpu'] else ''})")
                lobby = Lobby(host, args.control_port, info, sid)
            if lobby.poll():
                say(f"role: {lobby.role}" if lobby.role else "no role: waiting for one")
            if lobby.refused:
                say(lobby.refused)
                return 2
            if lobby.closed:
                say("the session is over")
                return 0
            if child is not None and child.poll() is not None:
                code, child = child.returncode, None
                if code == EXIT_QUIT:
                    return 0
                if code not in (EXIT_TRIAL_DONE, EXIT_HOST_CLOSED):
                    say(f"trial process ended with code {code}; rejoining")
                time.sleep(1.0)
            if child is not None and lobby.role != child_role:      # a new role between trials
                child.terminate()
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                child = None
            if child is None and lobby.role:
                child_role = lobby.role
                child = subprocess.Popen(child_command(argv, child_role, sid, lobby.carla,
                                                       args.port is not None, args.control_host is not None, host))
            time.sleep(0.2)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
