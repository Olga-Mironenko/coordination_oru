#!/bin/bash

set -eu -o pipefail

[ $# = 1 ]
dir=$1

root=$(dirname "$0")
"$root"/multijoin.sh "$dir"/*.csv | column -s, -t