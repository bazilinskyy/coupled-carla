"""A second connection to the CARLA server for fire-and-forget calls: debug drawings (traffic lights, eHMIs),
traffic-light states and the spectator. The simulation loop never waits on it.

Why: on CARLA 1.0 several fire-and-forget calls in a row hold up the next blocking call (world.tick(),
actor.get_light_state(), ...) by about 40 ms. CARLA 1.0 builds rpclib from its branch `carla`, which lacks the
commit of CARLA 0.9.16's v2.2.1_c5 ("Disable the min size for a network packet to be sent") that set TCP_NODELAY
on both ends. The server answers every call, and with Nagle's algorithm its second answer waits for the client's
delayed ACK (40 ms on Linux), the blocking call's answer behind it. Measured on lin5080 against a CARLA 1.0
server: two debug draws, then any blocking call, 39-41 ms instead of 0.1 ms; the twin's traffic lights, drawn
every fifth tick, cost 8 ms per tick that way. On this connection nothing waits for the answers, so the stalls
do not matter; the runner sends everything else that does not answer in one batch per tick
(CarlaSimulation.queue), which never stalls. On CARLA 0.9.16 none of this changes anything.
"""
import os
import socket

import carla


def _rpc_sockets(port):
    """{fd: socket} of this process's TCP connections to `port` (POSIX; {} where fds cannot be listed)."""
    fd_dir = "/proc/self/fd" if os.path.isdir("/proc/self/fd") else "/dev/fd"
    found = {}
    try:
        names = os.listdir(fd_dir)
    except OSError:
        return found
    for name in names:
        try:
            s = socket.socket(fileno=int(name))     # not owned: detached below or kept until closed by us
        except (OSError, ValueError):
            continue
        try:
            if s.type == socket.SOCK_STREAM and s.family in (socket.AF_INET, socket.AF_INET6) and \
                    s.getpeername()[1] == port:
                found[s.fileno()] = s
                continue
        except OSError:
            pass
        s.detach()
    return found


def _detach_all(socks):
    for s in socks.values():
        s.detach()


class Side:
    """client, world, debug and spectator on their own connection. Make it after the world is loaded."""

    def __init__(self, host, port, timeout=60.0):
        before = _rpc_sockets(port)
        _detach_all(before)
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = self.client.get_world()
        self.debug = self.world.debug
        self.spectator = self.world.get_spectator()
        # Its own requests go out at once (nothing on it waits for the server's answers, so Nagle only delays them).
        new = _rpc_sockets(port)
        self.nodelay = 0
        for fd, s in new.items():
            if fd not in before:
                try:
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.nodelay += 1
                except OSError:
                    pass
        _detach_all(new)
