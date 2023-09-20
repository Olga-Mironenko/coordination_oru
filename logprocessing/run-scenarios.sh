#!/bin/bash

set -eu

[ $# -gt 2 ]
timeout=$1
demo=$2
shift 2
scenarios=("$@")

root=$(dirname "$0")
set -x
for scenario in "${scenarios[@]}"; do
  timeout --signal=INT "$timeout" env SCENARIO="$scenario" "$root"/run-demo.sh "$demo" || [ $? = 124 ]
done