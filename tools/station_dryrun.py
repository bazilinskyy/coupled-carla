"""Dry run of a session's stations: a short trial with bot stations, and per station whether it is ready for people.

    python tools/station_dryrun.py experiments/eindhoven_crossing_study.yaml --duration 40
    python tools/station_dryrun.py exp.yaml --remote pedestrian=lin4080            # prints the command for lin4080
    python tools/station_dryrun.py exp.yaml --ssh pedestrian=lin4080:cc-wt-labprep --ssh-env env58.sh
    python tools/station_dryrun.py --report logs/<experiment>/<time>_trial0         # check a real session's trial

It starts the host (coupledcarla.run, headless, one trial), one bot station per station role on this machine
(--input bot --headless, the camera at the full rate unless --camera-hz), or prints the command for a role given
to --remote and waits for it, or starts it over ssh (--ssh). After the trial it reads the trial's events.csv and
summary.json (the stations' station_stats, labcheck.py) and prints per station: the time to join the host, the
time from the trial start to its first camera image, the round trip to the host, its loop and camera frame rates
against the host's tick rate, the tick-to-image latency (median and 95th percentile), world ticks it missed and
the share of camera images it got, and whether sound (with an audio device), in-car displays and station-drawn
eHMIs are on. The exit code is 1 when a threshold is missed (--max-rtt-ms etc., defaults in labcheck.THRESHOLDS),
2 when a station never joined or the host failed.
"""
import argparse
import os
import re
import shlex
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import labcheck  # noqa: E402

TRIAL_LINE = re.compile(r"trial \d+ -> (.+)$")
WELCOME_LINE = re.compile(r"\[station (\S+)\] \S+ in ")


def own_ip():
    """This machine's LAN address (the interface of the default route)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Proc:
    """A child process whose output goes to a log file, with the time each line arrived."""

    def __init__(self, name, argv, log_path, on_line=None, cwd=None, env=None):
        self.name, self.argv, self.on_line = name, argv, on_line
        self.t0 = time.time()
        self.log = open(log_path, "w", buffering=1)
        self.p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                  cwd=cwd, env=env)
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        for line in self.p.stdout:
            self.log.write(line)
            if self.on_line:
                self.on_line(self, line.rstrip("\n"), time.time())
        self.log.flush()

    def stop(self, wait=10.0):
        if self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.thread.join(timeout=2)
        self.log.close()


def station_roles(cfg):
    return [p["role"] for p in cfg.get("participants") or [] if p.get("station")]


def station_argv(python, role, server, control, out, hz, size, ports=(2000, 2100), extra=()):
    port, control_port = ports
    return [python, "-m", "coupledcarla.station", "--host", server, "--control-host", control, "--role", role,
            "--input", "bot", "--headless", "--camera-hz", f"{hz:g}", "--size", size, "--out", str(out),
            *(["--port", str(port)] if int(port) != 2000 else []),
            *(["--control-port", str(control_port)] if int(control_port) != 2100 else []), *extra]


def _pairs(items, what):
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"{what} takes ROLE=..., got {it!r}")
        k, v = it.split("=", 1)
        out[k] = v
    return out


def report(trial_dir, th, join_s=None, camera_hz=None, out=print):
    """Reads a trial folder, prints the table; returns the exit code (0 ok, 1 thresholds, 2 a station missing)."""
    trial = labcheck.read_trial(trial_dir)
    rows = labcheck.summarise(trial, join_s, camera_hz)
    checks = [labcheck.check(r, th) for r in rows]
    tchk = labcheck.check_trial(trial, th)
    summ = trial["trial"]
    out(f"trial {trial_dir}: {summ.get('sim_s', '-')} s simulated in {summ.get('wall_s', '-')} s, "
        f"RTF {summ.get('rtf', '-')} (min {tchk[0][2]}), {summ.get('ticks', '-')} ticks, "
        f"collisions {summ.get('collisions', '-')}")
    if not rows:
        out("no stations in this trial")
        return 2
    out(labcheck.table(rows, checks))
    for r, chk in zip(rows, checks):
        for name, value, limit, ok in chk:
            if not ok:
                out(f"  {r['role']}: {name} = {value} (limit {limit})")
        if r.get("camera_on"):
            out(f"  {r['role']}: camera rendered by secondary {r['camera_on']}")
    if trial["missing"]:
        out(f"stations missing at the start: {', '.join(trial['missing'])}")
    if not tchk[0][3]:
        out(f"  trial: RTF {tchk[0][1]} under {tchk[0][2]} (the host cannot keep real time)")
    if trial["missing"] or any(not r["joined"] or r["stats"] == 0 for r in rows):
        return 2
    return 0 if all(c[3] for chk in checks for c in chk) and tchk[0][3] else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("experiment", nargs="?")
    ap.add_argument("--report", metavar="TRIAL_DIR", help="only check an existing trial folder")
    ap.add_argument("--trial", type=int, default=0)
    ap.add_argument("--participant", type=int, default=0)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--roles", help="local bot stations, comma-separated (default: every station role not remote)")
    ap.add_argument("--remote", action="append", metavar="ROLE=MACHINE",
                    help="print the command for ROLE on MACHINE and wait for it")
    ap.add_argument("--ssh", action="append", metavar="ROLE=ALIAS:DIR",
                    help="start ROLE's bot station on ALIAS over ssh, in the coupled-carla folder DIR")
    ap.add_argument("--ssh-python", default="python", help="python on the ssh machines (after --ssh-env)")
    ap.add_argument("--ssh-env", help="file to source there first (venv with the carla wheel, CARLA_ROOT)")
    ap.add_argument("--server-host", help="the CARLA server for local stations (default: the experiment's)")
    ap.add_argument("--host-ip", help="this machine's LAN address, for remote stations (default: guessed)")
    ap.add_argument("--camera-hz", type=float, default=0.0, help="the bots' camera rate (0: every tick, as a human's)")
    ap.add_argument("--size", default="1280x720", help="the bots' camera size")
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--out", default=str(ROOT / "logs" / "dryrun"))
    ap.add_argument("--host-arg", action="append", default=[], help="extra argument for coupledcarla.run")
    for k, v in labcheck.THRESHOLDS.items():
        ap.add_argument("--" + k.replace("_", "-"), type=float, default=v, help=f"default {v}")
    ap.add_argument("--host-cmd", help=argparse.SUPPRESS)       # tests: a fake host
    ap.add_argument("--station-cmd", help=argparse.SUPPRESS)    # tests: fake stations
    a = ap.parse_args(argv)
    th = {k: getattr(a, k) for k in labcheck.THRESHOLDS}
    if a.report:
        return report(a.report, th)
    if not a.experiment:
        ap.error("an experiment, or --report TRIAL_DIR")

    from coupledcarla import config
    cfg = config.load(a.experiment)
    roles = station_roles(cfg)
    remote, ssh = _pairs(a.remote, "--remote"), _pairs(a.ssh, "--ssh")
    for r in list(remote) + list(ssh):
        if r not in roles:
            raise SystemExit(f"{r} is not a station role of {a.experiment} (station roles: {', '.join(roles)})")
    local = [r for r in (a.roles.split(",") if a.roles else roles) if r and r not in remote and r not in ssh]
    if not roles:
        raise SystemExit(f"{a.experiment} has no station: true participants")
    server = a.server_host or cfg["server"]["host"]
    host_ip = a.host_ip or own_ip()
    remote_server = host_ip if server in ("127.0.0.1", "localhost") else server
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(a.out) / stamp
    out.mkdir(parents=True, exist_ok=True)
    hz = a.camera_hz
    ports = (int(cfg["server"]["port"]), int(cfg["coupling"].get("port", 2100)))
    tick_hz = 1.0 / float(cfg["simulation"]["step"])
    per = labcheck.stream_mb_s(a.size, hz or tick_hz)
    n_remote = len(remote) + len(ssh)
    print(f"dry run of {a.experiment}, trial {a.trial}, {a.duration:.0f} s; logs in {out}")
    away = ", ".join(f"{r} on {m.split(':')[0]}" for r, m in {**remote, **ssh}.items())
    print(f"stations: {', '.join(local) or '-'} here; {away or 'none'} remote")
    print(f"camera {a.size} at {('%.0f Hz' % hz) if hz else 'every tick (%.0f Hz)' % tick_hz}: {per:.0f} MB/s per "
          f"station" + (f"; the {n_remote} remote ones need {per * n_remote:.0f} MB/s of the server's link "
                        f"(gigabit carries {labcheck.LINK_MB_S:.0f})" if n_remote else ""))
    if n_remote and per * n_remote > 0.9 * labcheck.LINK_MB_S:
        print(f"  warning: more than the link carries: expect dropped images; try --camera-hz "
              f"{int(0.85 * labcheck.LINK_MB_S / (labcheck.stream_mb_s(a.size, 1) * n_remote))} or a smaller --size")

    trial_dir = [None]
    joined = {}
    lock = threading.Lock()

    def on_host(proc, line, t):
        m = TRIAL_LINE.search(line)
        if m:
            trial_dir[0] = m.group(1).strip()

    def on_station(proc, line, t):
        m = WELCOME_LINE.search(line)
        if m:
            with lock:
                joined.setdefault(m.group(1), round(t - proc.t0, 2))

    env = {**os.environ, "PYTHONPATH": str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""),
           "PYTHONUNBUFFERED": "1"}
    if a.host_cmd:
        host_argv = shlex.split(a.host_cmd.format(experiment=a.experiment, trial=a.trial, duration=a.duration,
                                                  python=a.python, out=out))
    else:
        host_argv = [a.python, "-m", "coupledcarla.run", a.experiment, "--trial", str(a.trial), "--duration",
                     str(a.duration), "--headless", *(["--participant", str(a.participant)] if a.participant else []),
                     *a.host_arg]
    host = Proc("host", host_argv, out / "host.log", on_host, cwd=str(ROOT), env=env)
    procs = []
    time.sleep(1.0)
    for r in local:
        if a.station_cmd:
            argv_ = shlex.split(a.station_cmd.format(role=r, server=server, control="127.0.0.1", out=out, hz=hz,
                                                     size=a.size, python=a.python))
        else:
            argv_ = station_argv(a.python, r, server, "127.0.0.1", out, hz, a.size, ports)
        procs.append(Proc(r, argv_, out / f"station_{r}.log", on_station, cwd=str(ROOT), env=env))
    for r, spec in ssh.items():
        alias, _, folder = spec.partition(":")
        cmd = " ".join(shlex.quote(x) for x in station_argv(a.ssh_python, r, remote_server, host_ip,
                                                          f"logs/dryrun/{stamp}", hz, a.size, ports))
        pre = f"cd {shlex.quote(folder or '.')} && " + (f"source {shlex.quote(a.ssh_env)} && " if a.ssh_env else "")
        procs.append(Proc(r, ["ssh", alias, f"bash -lc {shlex.quote(pre + 'PYTHONPATH=. ' + cmd)}"],
                          out / f"station_{r}.log", on_station, env=env))
    for r, machine in remote.items():
        print(f"\nstart on {machine} (in its coupled-carla folder, with the carla 1.0 wheel's python):\n  " +
              " ".join(shlex.quote(x) for x in station_argv("python", r, remote_server, host_ip,
                                                          f"logs/dryrun/{stamp}", hz, a.size, ports)))
    if remote:
        print(f"\nthe host waits for them up to coupling.wait_for_stations_s "
              f"({cfg['coupling'].get('wait_for_stations_s', 600)} s)\n")

    try:
        while host.p.poll() is None:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("interrupted: stopping the host and the stations")
    finally:
        host.stop(wait=60)
        t_end = time.time() + 15
        for p in procs:
            while p.p.poll() is None and time.time() < t_end:
                time.sleep(0.2)
            p.stop()
    print(f"host exit code {host.p.returncode}")
    if not trial_dir[0] or not Path(trial_dir[0]).exists():
        print(f"no trial folder in the host's log ({out / 'host.log'})")
        return 2
    code = report(trial_dir[0], th, join_s=joined, camera_hz={r: hz for r in roles})
    if host.p.returncode not in (0, None) and code == 0:
        code = 2
    print(f"result: {'ok' if code == 0 else ('thresholds missed' if code == 1 else 'stations or host failed')}"
          f" (exit {code})")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
