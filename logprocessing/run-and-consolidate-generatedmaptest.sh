#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout_hard=20m
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
  '2024-11-28_13:17:39_with_bridges'
  '2024-11-28_13:19:18_without_bridges'
)

seeds=(1)

probabilities=(
  0
  1
)

# Use variables
scenarios=()
for i_map in "${indexes_maps[@]}"; do
  for position in "${positions[@]}"; do
    for dir_maps in "${dirs_maps[@]}"; do
      filename_simple=map-generator/generated-maps/$dir_maps/scenario$i_map-$position.json
      filename=$(cd "$root"/..; realpath --canonicalize-existing --relative-to=. "$filename_simple")
      for seed in "${seeds[@]}"; do
        for probabilityForcingForHuman in "${probabilities[@]}"; do
          case $probabilityForcingForHuman in
            0 | 0.0)
              variations=("baseline")
              ;;
            *)
              variations=("change of priorities" "stops")
              ;;
          esac

          for variation in "${variations[@]}"; do
            scenarios+=("$filename, $variation, seed $seed, probabilityForcingForHuman $probabilityForcingForHuman")
          done
        done
      done
    done
  done
done

echo "Scenarios:"
printf -- "- %s\n" "${scenarios[@]}"

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT TERM

set -x
reference=$(mktemp --tmpdir run-and-consolidate.XXXX)
#trap 'ls -l "$reference"; rm -f "$reference"' EXIT

"$root"/run-scenarios.sh "$timeout_hard" "$demo" "${scenarios[@]}" || echo "WARNING: exit $?"
#"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"  # will consolidate everything created after the reference file