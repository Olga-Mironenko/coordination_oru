#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout=60s
demo=GeneratedMapTest

scenarios=()
for i in {1..5}; do
  filename_simple=map-generator/generated-maps/current/scenario$i.json
  filename=$(cd "$root"/..; realpath --canonicalize-existing --relative-to=. "$filename_simple")
  scenarios+=("$filename, with rerouting")
  scenarios+=("$filename, without rerouting")
done

reference=$(mktemp --tmpdir run-and-consolidate.XXXX)
trap 'rm -f "$reference"' EXIT

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT

set -x
"$root"/run-scenarios.sh "$timeout" "$demo" "${scenarios[@]}"
"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"  # will consolidate everything created after the reference file