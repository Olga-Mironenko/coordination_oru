#!/bin/bash

set -eu

[ $# = 1 ]
demo=$1

root=$(dirname "$0")
repo=$root/..
cd "$repo"

set -x
exec ./gradlew --no-daemon run -Pdemo="$demo" >./logs/entire/"$demo".log 2>&1