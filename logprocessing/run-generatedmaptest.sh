#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout_hard=30m
demo=GeneratedMapTest

# Prepare variables

case ${WORKER:-host} in
  host) indexes_maps=({1..10});;
  c1) indexes_maps=({1..2});;
  c2) indexes_maps=({3..4});;
  c3) indexes_maps=({5..6});;
  c4) indexes_maps=({7..8});;
  c5) indexes_maps=({9..10});;
esac

positions=({1..10})

#dirs_maps=(  # 4 robots
#  '2024-11-22_11:26:14_with_bridges'
#  '2024-11-22_11:27:17_without_bridges'
#)
dirs_maps=(  # 3 robots
  '2024-11-28_13:19:18_without_bridges'
  '2024-11-28_13:17:39_with_bridges'
)

passhums=(
  0
#  1
)

slownesses=(
  "no"
  "without rerouting"
  "with rerouting"
)

forcings=(
  "no"
  "change of priorities"
  "stops"
#  "ignoring human"
)

# Use variables
scenarios=()
for i_map in "${indexes_maps[@]}"; do
  for position in "${positions[@]}"; do
    for dir_maps in "${dirs_maps[@]}"; do
      filename_simple=map-generator/generated-maps/$dir_maps/scenario$i_map-$position.json
      filename=$(cd "$root"/..; realpath --canonicalize-existing --relative-to=. "$filename_simple")

      for slowness in "${slownesses[@]}"; do
        if [[ $dir_maps = *_without_bridges ]] && [ "$slowness" = 'with rerouting' ]; then
          continue
        fi

        for passhum in "${passhums[@]}"; do
          for forcing in "${forcings[@]}"; do
            scenario="$filename, passhum $passhum, slowness $slowness, forcing $forcing"
            scenarios+=("$scenario")
          done
        done
      done

      passhum=0

      forcing="ignoring human"
      slowness="no"
      scenario="$filename, passhum $passhum, slowness $slowness, forcing $forcing"
      scenarios+=("$scenario")

      forcing="ignoring human"
      slowness="without rerouting"
      scenario="$filename, passhum $passhum, slowness $slowness, forcing $forcing"
      scenarios+=("$scenario")
    done
  done
done

echo "Scenarios:"
printf -- "- %s\n" "${scenarios[@]}" | cat -n
#exit

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT TERM

set -x
exec "$root"/run-scenarios.sh "$timeout_hard" "$demo" "${scenarios[@]}"