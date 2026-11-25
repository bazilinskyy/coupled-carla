#!/bin/bash
# Colour/exposure sweep of the per-place camera profiles (README "Camera look per place"): renders the frozen views
# of each twin with a candidate profile, scores them and compares with a baseline round on the same views
# (tools/realism_score.py compare). Run from a coupled-carla checkout with the realism pilot's frozen sets in
# logs/pilot/frozen, against a running CARLA 1.0 server with the twin levels on PORT.
#
#   CONTENT=/mnt/c/cc58/carla/Unreal/CarlaUnreal/Content BASE=r41b bash tools/colour_sweep.sh
#
# 1. installs coupled-carla's profiles on the server (tools/install_profiles.py; CARLA ignores a missing one
#    silently), 2. round ${PREFIX}fit: each place with its own profile from data/postprocess/places.json,
#    3. round ${PREFIX}cr: every place with CoupledReal (the Eindhoven fit), 4. scores both and compares with $BASE.
# GPU time: like a realism round per variant, about 4 min a place for 80 views at 1280 wide plus the top-down sets
# (TUeCampus 4, EhvCentre 6, Salamanca 2.5, Shibuya 3 min in round 41): ~16 min a variant, ~35 min for both with
# the server start. Scoring (segmentation + DINO) on the same GPU or another: ~5 min a place.
set -u
PORT=${PORT:-2000}; BASE=${BASE:-r41b}; PREFIX=${PREFIX:-rcol_}
PLACES=${PLACES:-"tue_campus ehv_centre salamanca shibuya"}
PY=${PY:-python}
declare -A LEVEL=([tue_campus]=TUeCampus [ehv_centre]=EhvCentre [salamanca]=SalamancaPlaza [shibuya]=ShibuyaCrossing)
[ -n "${CONTENT:-}" ] && $PY tools/install_profiles.py --content "$CONTENT"
profile_of(){ $PY -c "from coupledcarla import look; print(look.place_profile('$1')[0])"; }
for variant in fit cr; do
  R=${PREFIX}${variant}
  for p in $PLACES; do
    prof=$([ $variant = fit ] && profile_of "${LEVEL[$p]}" || echo CoupledReal)
    echo "$(date +%T) $R $p: profile $prof"
    timeout 1200 $PY tools/realism_score.py render $R --twins --places $p --port $PORT --top-sets top,top150 \
      --budget-min 10 --render-args "--profile $prof --season photo"
    $PY tools/realism_score.py score $R --places $p --align-from $BASE    # the baseline's alignment, view by view
  done
  $PY tools/realism_score.py compare $BASE $R --places $PLACES
done
