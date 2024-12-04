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

trap 'kill -s SIGINT $pid; docker compose down' SIGINT

set -x
export RUNDIRS=logs/rundirs/$date
mkdir "$RUNDIRS"
docker compose "${args[@]}" > >(tee /dev/stderr | ansi2txt >logs/compose-up_"$date".log) 2>&1 &
pid=$!

wait