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
   --max-workers=1
   --no-parallel
   --no-scan
   run
   -Passert
   -Pdemo="$demo"
)
set -x +o pipefail
timeout --kill-after=10s "$timeout" ./gradlew "${args_gradlew[@]}" |&
  tee ./logs/entire/"$demo".log