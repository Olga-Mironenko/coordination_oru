#!/bin/bash

set -eux -o pipefail

[ $# -le 1 ]
reference=${1:-}

root=$(dirname "$0")

if [ "$reference" ]; then
  dir=$root/../logs/rundirs
  file=$dir/sorted-$(date +'%Y%m%d_%H%M%S').csv
  find "$dir" -mindepth 1 -maxdepth 1 -type d -newer "$reference" -print0 |
    xargs -0 --no-run-if-empty "$root"/consolidate-rundirs.sh >"$file"
else
  file=sorted.csv
  "$root"/consolidate-rundirs.sh */ >"$file"
fi