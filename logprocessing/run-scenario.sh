#!/bin/bash

set -eu -o pipefail

[ $# = 1 ] || [ $# = 2 ]
demo=$1
timeout=${2:-infinity}

root=$(dirname "$0")
repo=$root/..
cd "$repo"

args_gradlew=(
   --no-daemon
   #--foreground
   --max-workers=1
   --no-parallel
   --no-scan
   #--debug
   run
   -Passert
   -Pdemo="$demo"
)

filename_log=$(mktemp "logs/tmp-$demo-XXXX.log")
trap 'rm "$filename_log"' 0

set -x +o pipefail
(
  code=0
  timeout --foreground --kill-after=10s "$timeout" \
    env FILENAME_LOG="$filename_log" \
    ./gradlew "${args_gradlew[@]}" ||
    code=$?
  echo "exit: $code"
  exit "$code"
) |& tee "$filename_log"
