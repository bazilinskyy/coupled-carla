"""Start (or attach to) a CARLA server and connect to it."""
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import carla

WINDOWS = os.name == "nt"


def _port_open(host, port):
    try:
        socket.create_connection((host, port), timeout=2).close()
        return True
    except OSError:
        return False


class Server:
    def __init__(self, sc, log=print):
        self.sc, self.log, self.proc = sc, log, None

    def start(self):
        sc = self.sc
        if sc["launch"] and not _port_open(sc["host"], sc["port"]):
            root = Path(os.environ["CARLA_ROOT"])
            # CARLA 0.9.x ships CarlaUE4.sh, CARLA 1.0 (UE 5.8) CarlaUnreal.sh
            names = ["CarlaUE4.exe", "CarlaUnreal.exe"] if WINDOWS else ["CarlaUE4.sh", "CarlaUnreal.sh"]
            exe = next((root / n for n in names if (root / n).exists()), root / names[0])
            args = [str(exe), f"-carla-rpc-port={sc['port']}", f"-quality-level={sc['quality']}", "-nosound",
                    *(["-RenderOffScreen"] if sc["offscreen"] else []), *sc["extra_args"]]
            self.proc = subprocess.Popen(args, **({} if WINDOWS else {"start_new_session": True}))
            self.log(f"started {exe.name} (pid {self.proc.pid})")
        # A carla.Client made before the server listens never recovers, so wait for the port first.
        for _ in range(150):
            if _port_open(sc["host"], sc["port"]):
                break
            if self.proc and self.proc.poll() is not None:
                raise RuntimeError("CARLA server exited during start-up")
            time.sleep(2)
        else:
            raise RuntimeError(f"no CARLA server on {sc['host']}:{sc['port']}")
        time.sleep(3)
        client = carla.Client(sc["host"], sc["port"])
        client.set_timeout(60.0)
        self.log(f"connected to CARLA {client.get_server_version()} at {sc['host']}:{sc['port']}")
        return client

    def stop(self):
        if self.proc and self.proc.poll() is None:
            if WINDOWS:
                subprocess.call(["taskkill", "/T", "/F", "/PID", str(self.proc.pid)], stdout=subprocess.DEVNULL)
            else:
                os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait(timeout=30)
            for _ in range(30):   # the engine process can hold the port a little longer than the launcher
                if not _port_open(self.sc["host"], self.sc["port"]):
                    break
                time.sleep(1)
            self.log("CARLA server stopped")
