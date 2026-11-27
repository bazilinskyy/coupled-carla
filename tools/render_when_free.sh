#!/bin/bash
# Render a demo clip on a shared GPU machine only while its GPU is free, at the lowest priority (coupled-carla work
# yields to everything else on the lab machines). Waits until the GPU has >= FREE_MB free and <= BUSY_PCT busy for
# three minutes running, starts the CARLA 1.0 server with rendering (run_server.sh, off-screen), renders the clip with
# tools/make_videos.py against it, and stops the server. While rendering, a guard stops the server (and so the render)
# as soon as another GPU job starts or free memory falls under GUARD_MB. Gives up after MAX_WAIT_H hours.
#
#   CLIPS=experiments/videos/ships.yaml nohup setsid bash tools/render_when_free.sh hoek_van_holland_ships \
#       > render_when_free.log 2>&1 &
#
# Environment: CARLA_UE58_ROOT, CARLA_UNREAL_ENGINE_PATH (the CARLA 1.0 tree and engine), the repo's .venv.
set -u
CLIP=${1:?clip name}
FREE_MB=${FREE_MB:-10000}
BUSY_PCT=${BUSY_PCT:-10}
GUARD_MB=${GUARD_MB:-800}
MAX_WAIT_H=${MAX_WAIT_H:-24}
REPO=$(cd "$(dirname "$0")/.." && pwd)
RUN=${CARLA_UE58_ROOT:?}/Skills/ue58/run-carla-ue58-server/scripts/run_server.sh
log() { echo "$(date '+%F %T') $*"; }
gpu() { nvidia-smi --query-gpu="$1" --format=csv,noheader,nounits | head -1; }
apps() { nvidia-smi --query-compute-apps=pid --format=csv,noheader | sort; }
free_now() { [ "$(gpu memory.free)" -ge "$FREE_MB" ] && [ "$(gpu utilization.gpu)" -le "$BUSY_PCT" ]; }

end=$(( $(date +%s) + MAX_WAIT_H * 3600 ))
while :; do
  ok=0
  for _ in 1 2 3; do free_now && ok=$((ok + 1)); sleep 60; done
  [ "$ok" -eq 3 ] && break
  [ "$(date +%s)" -ge "$end" ] && { log "gave up: the GPU was not free for ${MAX_WAIT_H} h"; exit 1; }
  log "waiting: $(gpu memory.free) MB free, $(gpu utilization.gpu) % busy"
  sleep 240
done
log "GPU free ($(gpu memory.free) MB): starting the server"
before=$(apps)
cd "$CARLA_UE58_ROOT" && DETACH=1 nice -n 19 bash "$RUN" game Town10HD_Opt || { log "server did not start"; exit 1; }
ours=$(comm -13 <(echo "$before") <(apps) | paste -sd' ')
(   # the guard: anything else on the GPU, or too little memory, ends our server
  while sleep 10; do
    others=$(comm -23 <(apps) <(printf '%s\n' $ours $before | sort -u) | paste -sd' ')
    [ -z "$ours" ] && ours=$(comm -13 <(echo "$before") <(apps) | paste -sd' ')
    if [ -n "$others" ] || [ "$(gpu memory.free)" -lt "$GUARD_MB" ]; then
      log "another GPU job ($others) or low memory: stopping the server"; bash "$RUN" stop; exit 0
    fi
  done
) &
guard=$!
cd "$REPO" && export SUMO_HOME=$(.venv/bin/python -c "import sumo;print(sumo.SUMO_HOME)") CARLA_ROOT=$CARLA_UE58_ROOT
export PATH=$SUMO_HOME/bin:$PATH PROJ_LIB=$SUMO_HOME/data/proj PYTHONUNBUFFERED=1
nice -n 19 .venv/bin/python tools/make_videos.py render "$CLIP" --file "${CLIPS:-experiments/videos/clips.yaml}" --no-launch --port 2000 --python .venv/bin/python
status=$?
kill "$guard" 2>/dev/null
bash "$RUN" stop
log "render of $CLIP ended with $status"
exit $status
