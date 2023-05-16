#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

root=$(dirname "$0")
watch -n1 "$root"/show-statistics.sh "$root"/../logs/statistics/current