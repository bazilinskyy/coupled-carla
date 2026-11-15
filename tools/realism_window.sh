#!/bin/bash
# The realism check of the Eindhoven twin before and after each change of the realism round (README "How close is the
# twin to the real street?"), for a GPU window on the CARLA 1.0 rendering server, then compared on a CPU machine.
#
# Run in the client environment beside a running server that has EhvCentre (win5090: WSL with ~/cc58/env58.sh, the
# packaged server of the task cc58-pkgserver, ~/cc58/gpu_window.sh holding the GPU):
#     bash tools/realism_window.sh              # ~35 min of GPU (5 x 201 views), then ~40 min of CPU on $CPU_HOST
#     ONLY="look overcast" bash tools/realism_window.sh      # some variants only (also: clean)
# CoupledReal is fitted with a model of Unreal's tone mapping (lookfit.py) that got the direction of each setting
# right on CARLA 0.9.16 but its size 30-50 % low, so its predicted gain must be measured here before it is adopted.
#
# Needs in this checkout:
#   maps/eindhoven_twin/                 the twin's map folder (geo.net.xml, map.xodr, real/ with the weather caches)
#   maps/eindhoven_twin/realism/scenery.before.json   the scenery of the first run (snapped, before this round)
#   maps/eindhoven_twin/realism/scenery.after.json    mapbuild --scenery of this round (not snapped yet)
#   maps/eindhoven_twin/realism/parked.after.json     its parked cars
#   $BASE/views.csv                       the views of the first run
# and on $CPU_HOST:$CPU_DIR the same branch with $BASE holding the first run's real images and labels.
set -u
cd "$(dirname "$0")/.."
PY=${PY:-python}
PORT=${PORT:-2000}
EXP=${EXP:-experiments/eindhoven_twin.yaml}
MAP=${MAP:-maps/eindhoven_twin}
BASE=${BASE:-logs/realism_base}
PKG=${PKG:-/mnt/c/cc58/carla/Build/Package/Carla-1.0-Win64-Shipping}
CPU_HOST=${CPU_HOST:-lin4080}
CPU_DIR=${CPU_DIR:-cc-wt-realism2}
CPU_PY=${CPU_PY:-'~/cc-wt-crowdcompare/.venv-seg/bin/python'}     # numpy, PIL, torch (CPU) and transformers
ONLY=${ONLY:-"before trees cars look overcast"}
BUDGET=${BUDGET:-12}       # minutes per variant (201 views take ~6)
log() { echo "$(date +%T) $*"; }

# variant -> scenery set and render options
declare -A SCN=([before]=before [trees]=after [cars]=after [clean]=after [look]=after [overcast]=after)
declare -A OPT=(
  [before]="--no-parked --moving-cars 0 --profile Dashcam"
  [trees]="--no-parked --moving-cars 0 --profile Dashcam"
  [cars]="--moving-cars 20 --profile Dashcam"
  [clean]="--moving-cars 20 --profile CoupledClean"      # optional (ONLY=...): the profile CoupledReal starts from
  [look]="--moving-cars 20 --profile CoupledReal"
  [overcast]="--moving-cars 20 --profile CoupledReal --cloud-scale 1.0"
)

use_scenery() {        # before | after: put that set in place
  cp "$MAP/realism/scenery.$1.json" "$MAP/scenery.json"
  if [ "$1" = after ]; then cp "$MAP/realism/parked.after.json" "$MAP/parked.json"; else rm -f "$MAP/parked.json"; fi
}

# 0. the profiles where the package reads them (loose files next to the cooked ones), and a check that it does
PP="$PKG/CarlaUnreal/Content/Carla/Config/PostProcess"
mkdir -p "$PP" && cp coupledcarla/data/postprocess/*.json "$PP/" && log "profiles copied to $PP"
mkdir -p logs/rw_check
head -2 "$BASE/views.csv" > logs/rw_check/views.csv
for prof in Dashcam CoupledReal; do
  rm -rf logs/rw_check/renders logs/rw_check/renders.csv
  $PY tools/realism_check.py render "$EXP" --out logs/rw_check --port $PORT --no-scenery --no-parked --moving-cars 0 \
      --profile $prof --budget-min 3 2>&1 | grep -v WARNING | tail -1
  f=$(ls logs/rw_check/renders/*.png | grep -v _sem | head -1)
  $PY -c "import numpy as n,PIL.Image as I,sys;print(round(float(n.asarray(I.open(sys.argv[1]).convert('L')).mean()),1))" "$f" \
      > logs/rw_check/$prof.mean
done
d=$($PY -c "print(abs(float(open('logs/rw_check/Dashcam.mean').read())-float(open('logs/rw_check/CoupledReal.mean').read())))")
log "mean brightness Dashcam $(cat logs/rw_check/Dashcam.mean), CoupledReal $(cat logs/rw_check/CoupledReal.mean)"
if $PY -c "import sys; sys.exit(0 if $d < 2 else 1)"; then
  log "the server did not take CoupledReal (no difference to Dashcam): it reads only cooked profiles. Copy"
  log "coupledcarla/data/postprocess/*.json to the source tree's Content/Carla/Config/PostProcess and package again."
  ONLY=$(echo $ONLY | sed 's/look//; s/overcast//')
fi

# 1. this round's scenery stood on the level's ground (rewrites scenery.json and parked.json; kept for the variants)
use_scenery after
log "snapping this round's scenery"
timeout 1200 $PY tools/snap_scenery.py "$EXP" --port $PORT 2>&1 | grep -v WARNING | tail -3
cp "$MAP/scenery.json" "$MAP/realism/scenery.after.json"; cp "$MAP/parked.json" "$MAP/realism/parked.after.json"

# 2. the renders
for v in $ONLY; do
  out=logs/rw_$v
  mkdir -p $out && cp "$BASE/views.csv" $out/
  use_scenery ${SCN[$v]}
  log "render $v: ${OPT[$v]}"
  $PY tools/realism_check.py render "$EXP" --out $out --port $PORT --budget-min $BUDGET ${OPT[$v]} 2>&1 \
      | grep -v WARNING | grep -E "rendered|scenery|parked|moving|budget|error|Error" | tail -6
done
use_scenery after

# 3. the comparisons on the CPU machine (the first run's real images and labels are there), two at a time
for v in $ONLY; do
  rsync -a logs/rw_$v "$CPU_HOST:$CPU_DIR/logs/" || { log "copy to $CPU_HOST failed"; exit 1; }
done
ssh "$CPU_HOST" "cd $CPU_DIR && for v in $ONLY; do d=logs/rw_\$v; mkdir -p \$d/seg; cp -n $BASE/ATTRIBUTION.csv \
  $BASE/pairs_summary.json \$d/ 2>/dev/null; [ -d \$d/real ] || cp -rl $BASE/real $BASE/panos \$d/; \
  cp -ln $BASE/seg/real_* $BASE/seg/pano_* \$d/seg/ 2>/dev/null; done; \
  printf '%s\n' $ONLY | OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 xargs -P 2 -I{} sh -c \
  'nice $CPU_PY tools/realism_check.py compare --out logs/rw_{} --device cpu > logs/rw_{}/compare.log 2>&1'; \
  nice $CPU_PY tools/realism_check.py table $(for v in $ONLY; do printf 'logs/rw_%s ' $v; done) \
  --json logs/rw_table.json"
log "done: the table above, and $CPU_HOST:$CPU_DIR/logs/rw_table.json"
