#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

date=$(date +'%Y%m%d_%H%M%S')

args=(
  --ansi=always  # colorize
  up
  --build
  --force-recreate
  --remove-orphans
)
(
  set -x
  export RUNDIRS=logs/rundirs/$date
  mkdir "$RUNDIRS"
  exec docker compose "${args[@]}"
) |&
  tee logs/compose-up_"$date".log