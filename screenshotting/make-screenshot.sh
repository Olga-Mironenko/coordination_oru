#!/bin/bash

set -eux -o pipefail

[ $# = 1 ]
file_png=$1

case ${WORKER:-host} in
  host) browser="Google Chrome" ;;
  c*) browser="Chromium" ;;
  *) exit 1 ;;
esac

set +e
for _ in {1..5}; do  # https://github.com/jordansissel/xdotool/issues/60
  id_window=$(xdotool search --onlyvisible --name "localhost:8080 - $browser" | tail -n1)
  if [ "$id_window" ]; then
    break
  fi
done
set -e

[ "$id_window" ]
import -window "$id_window" "$file_png"