#!/bin/bash

set -eu -o pipefail

[ $# = 1 ]
dir=$1
if [ -L "$dir" ]; then
  dir=$(readlink -f "$dir")
  base=$(basename "$dir")
  printf '*** %s ***\n\n' "$base"
fi

paste "$dir"/*.txt | pr -t -e38