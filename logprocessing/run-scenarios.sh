#!/bin/bash

# See `$repo/notes/run-demos.md`.

set -eu -o pipefail

[ $# -gt 2 ]
timeout=$1
demo=$2
shift 2
scenarios=("$@")

root=$(dirname "$0")
set -x
for scenario in "${scenarios[@]}"; do
  env SCENARIO="$scenario" "$root"/run-demo.sh "$demo" "$timeout" || echo "WARNING: exit $?"
done