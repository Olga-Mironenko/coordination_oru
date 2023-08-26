#!/bin/bash

set -eu -o pipefail

[ $# -ge 0 ]
if [ $# = 0 ]; then
  dirs=(2023*/)
else
  dirs=("$@")
fi

files=()
for dir in "${dirs[@]}"; do
  files+=("$dir"/*.csv)
done

root=$(dirname "$0")
"$root"/multijoin.sh "${files[@]}" |
  "$root"/csvtk -d, transpose |
  "$root"/csvtk -d, sort -k3,1