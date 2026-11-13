#!/bin/bash
# A coupled test on one machine: the host plus three bot stations (pedestrian, AV passenger, driver), headless,
# every trial of the experiment. Station views are saved every 0.5 s under logs/stations/.
#   bash tools/run_coupled_test.sh [experiment.yaml] [env.sh] [python]
set -e
cd "$(dirname "$0")/.."
EXP=${1:-experiments/eindhoven_crossing.yaml}; ENV=${2:-env.sh}; PY=${3:-.venv/bin/python}
source "$ENV"
export PYTHONPATH=$PWD
mkdir -p logs
$PY -m coupledcarla.run "$EXP" --headless > logs/test_host.log 2>&1 &
HOST=$!
sleep 5
for r in pedestrian av_passenger driver; do
  $PY -m coupledcarla.station --host 127.0.0.1 --role $r --input bot --headless --frames-every 0.5 \
      > logs/test_$r.log 2>&1 &
done
wait $HOST
sleep 8
pkill -f "[c]oupledcarla.station" || true
echo TESTDONE
