#!/bin/bash

set -eu

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
set -x
timeout --kill-after=10s "$timeout" ./gradlew "${args_gradlew[@]}" |&
  tee ./logs/entire/"$demo".log