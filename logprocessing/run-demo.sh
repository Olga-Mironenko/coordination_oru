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
set -x +o pipefail
timeout --foreground --kill-after=10s "$timeout" ./gradlew "${args_gradlew[@]}"  # |& tee ./logs/entire/"$demo".log