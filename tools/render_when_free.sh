#!/bin/bash
# Render a demo clip on a shared GPU machine only while nothing else needs it, at the lowest priority (coupled-carla
# extras yield to everything else on the lab machines). Waits until no other UnrealEditor runs (one editor at a time
# per machine: bakes, packages, -game servers) and the GPU has >= FREE_MB free and <= BUSY_PCT busy, three minutes
# running; starts its own CARLA 1.0 -game server (run_server.sh, off-screen) on PORT; renders the clip with
# tools/make_videos.py against it; and stops that server. It stops only its own server's pid: run_server.sh's `stop`
# ends every UnrealEditor on the machine by name (a colleague's bake too). While rendering, a guard ends the server
# (and so the render) as soon as another UnrealEditor or GPU job starts or free memory falls under GUARD_MB. A server
# that does not answer within SERVER_WAIT_S (a first start compiles shaders) is ended too: 2026-11-27 on lin5080 the
# old version gave up after run_server.sh's 120 s and left the server running, which held up Pavlo's bakes for an hour.
# Gives up after MAX_WAIT_H hours.
#
#   CLIPS=experiments/videos/ships.yaml nohup setsid bash tools/render_when_free.sh hoek_van_holland_ships \
#       > render_when_free.log 2>&1 &
#
# Environment: CARLA_UE58_ROOT, CARLA_UNREAL_ENGINE_PATH (the CARLA 1.0 tree and engine), the repo's .venv.
set -u
CLIP=${1:?clip name}
PORT=${PORT:-2400}              # its own: 2000 is the usual server, 2300 Pavlo's render host on lin5080
FREE_MB=${FREE_MB:-10000}
BUSY_PCT=${BUSY_PCT:-10}
GUARD_MB=${GUARD_MB:-800}
SERVER_WAIT_S=${SERVER_WAIT_S:-900}
MAX_WAIT_H=${MAX_WAIT_H:-24}
REPO=$(cd "$(dirname "$0")/.." && pwd)
RUN=${CARLA_UE58_ROOT:?}/Skills/ue58/run-carla-ue58-server/scripts/run_server.sh
log() { echo "$(date '+%F %T') $*"; }
gpu() { nvidia-smi --query-gpu="$1" --format=csv,noheader,nounits | head -1; }
apps() { nvidia-smi --query-compute-apps=pid --format=csv,noheader | sort; }
editors() { pgrep -f 'UnrealEdito[r]' | sort; }                 # bakes, packages and servers alike
port_up() { (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; }
free_now() { [ -z "$(editors)" ] && [ "$(gpu memory.free)" -ge "$FREE_MB" ] && [ "$(gpu utilization.gpu)" -le "$BUSY_PCT" ]; }
SERVER=""
stop_ours() {           # only our own server, never run_server.sh's stop (it ends every UnrealEditor)
  [ -n "$SERVER" ] || return 0
  kill "$SERVER" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$SERVER" 2>/dev/null || break; sleep 1; done
  kill -9 "$SERVER" 2>/dev/null
  log "server $SERVER stopped"
  SERVER=""
}
trap 'stop_ours' EXIT

end=$(( $(date +%s) + MAX_WAIT_H * 3600 ))
while :; do
  ok=0
  for _ in 1 2 3; do free_now && ok=$((ok + 1)); sleep 60; done
  [ "$ok" -eq 3 ] && break
  [ "$(date +%s)" -ge "$end" ] && { log "gave up: the machine was not free for ${MAX_WAIT_H} h"; exit 1; }
  log "waiting: $(gpu memory.free) MB free, $(gpu utilization.gpu) % busy, editors: $(editors | paste -sd' ')"
  sleep 240
done
log "free ($(gpu memory.free) MB, no other editor): starting the server on port $PORT"
before=$(apps)
out=$(cd "$CARLA_UE58_ROOT" && PORT=$PORT DETACH=1 nice -n 19 bash "$RUN" game Town10HD_Opt 2>&1)
echo "$out"
SERVER=$(echo "$out" | sed -n 's/.*detached, pid \([0-9]*\).*/\1/p' | head -1)
[ -n "$SERVER" ] || SERVER=$(pgrep -f "UnrealEdito[r] .*-carla-rpc-port=$PORT" | head -1)
[ -n "$SERVER" ] || { log "no server process found"; exit 1; }
for _ in $(seq 1 "$SERVER_WAIT_S"); do port_up && break; kill -0 "$SERVER" 2>/dev/null || break; sleep 1; done
port_up || { log "server $SERVER did not answer on port $PORT within ${SERVER_WAIT_S} s"; exit 1; }
log "server $SERVER up"
(   # the guard: another editor or GPU job, or too little memory, ends our server
  while sleep 10; do
    kill -0 "$SERVER" 2>/dev/null || exit 0
    other_editors=$(editors | grep -vx "$SERVER" | paste -sd' ')
    other_apps=$(comm -23 <(apps) <(printf '%s\n' $before $(pgrep -P "$SERVER") "$SERVER" | sort -u) | paste -sd' ')
    if [ -n "$other_editors" ] || [ -n "$other_apps" ] || [ "$(gpu memory.free)" -lt "$GUARD_MB" ]; then
      log "another editor ($other_editors), GPU job ($other_apps) or low memory: stopping our server"
      kill "$SERVER" 2>/dev/null; sleep 30; kill -9 "$SERVER" 2>/dev/null; exit 0
    fi
  done
) &
guard=$!
cd "$REPO" && export SUMO_HOME=$(.venv/bin/python -c "import sumo;print(sumo.SUMO_HOME)") CARLA_ROOT=$CARLA_UE58_ROOT
export PATH=$SUMO_HOME/bin:$PATH PROJ_LIB=$SUMO_HOME/data/proj PYTHONUNBUFFERED=1
nice -n 19 .venv/bin/python tools/make_videos.py render "$CLIP" --file "${CLIPS:-experiments/videos/clips.yaml}" \
  --no-launch --port "$PORT" --python .venv/bin/python
status=$?
kill "$guard" 2>/dev/null
stop_ours
log "render of $CLIP ended with $status"
exit $status
