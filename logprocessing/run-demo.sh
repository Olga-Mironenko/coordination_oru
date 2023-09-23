#!/bin/bash

set -eu

[ $# = 1 ]
demo=$1

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
./gradlew "${args_gradlew[@]}" |& tee ./logs/entire/"$demo".log