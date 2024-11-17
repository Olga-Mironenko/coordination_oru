#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout_hard=60m
demo=GeneratedMapTest

scenarios=()
for i_locations in {1..10}; do
  filename_simple=map-generator/generated-maps/current/scenario1-$i_locations.json
  filename=$(cd "$root"/..; realpath --canonicalize-existing --relative-to=. "$filename_simple")
  for seed in {1..1}; do
    for variation in "change of priorities" "stops"; do
      scenarios+=("$filename, $variation, seed $seed")
    done
  done
done

reference=$(mktemp --tmpdir run-and-consolidate.XXXX)
trap 'rm -f "$reference"' EXIT

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT

set -x
"$root"/run-scenarios.sh "$timeout_hard" "$demo" "${scenarios[@]}" || echo "[exit $?]"
"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"  # will consolidate everything created after the reference file