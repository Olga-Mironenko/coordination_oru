#!/bin/bash

set -eu -o pipefail

[ $# -ge 0 ]
if [ $# = 0 ]; then
  dirs=(2023*/)
else
  dirs=("$@")
fi

echo >&2 "Use consolidate.py instead!"
exit 1

files=()
for dir in "${dirs[@]}"; do
  files+=("$dir"/*.csv)
done

root=$(dirname "$0")
"$root"/multijoin.sh "${files[@]}" |  # cat >&2
  "$root"/csvtk -d$'\t' transpose |
  "$root"/csvtk -d, sort -k2,3