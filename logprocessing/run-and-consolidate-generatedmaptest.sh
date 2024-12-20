#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout_hard=60m
demo=GeneratedMapTest

dirs_maps=(
  '2024-11-22_11:26:14_with_bridges'
  '2024-11-22_11:27:17_without_bridges'
)
indexes_maps=(
#  1
  3
)
scenarios=()
for i_map in "${indexes_maps[@]}"; do
  for i_locations in {1..10}; do
    for dir_maps in "${dirs_maps[@]}"; do
      filename_simple=map-generator/generated-maps/$dir_maps/scenario$i_map-$i_locations.json
      filename=$(cd "$root"/..; realpath --canonicalize-existing --relative-to=. "$filename_simple")
      for seed in {1..1}; do
        for probabilityForcingForHuman in 0 1; do
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

"$root"/run-scenarios.sh "$timeout_hard" "$demo" "${scenarios[@]}" || echo "[exit $?]"
"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"  # will consolidate everything created after the reference file