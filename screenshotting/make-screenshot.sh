#!/bin/bash

set -eux -o pipefail

[ $# = 1 ]
file_png=$1

set +e
for _ in {1..5}; do  # https://github.com/jordansissel/xdotool/issues/60
  id_window=$(xdotool search --onlyvisible --name 'localhost:8080 - Google Chrome' | tail -n1)
  if [ "$id_window" ]; then
    break
  fi
done
set -e

[ "$id_window" ]
import -window "$id_window" "$file_png"