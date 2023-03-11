#!/bin/bash

set -eu -o pipefail

inotifywait --monitor --event=open --format='%f' . |
  grep '[.]log$' --line-buffered |
  while read -r file; do
    if [ -e "$file" ]; then
      date=$(date +'%Y%m%d_%H%M%S')
      file_new=$file.$date
      mv "$file" "$file_new"
      echo >&2 "$file -> $file_new"
    fi
  done
