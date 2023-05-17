#!/bin/bash

set -eu -o pipefail

[ $# -ge 0 ]
args_consolidate=("$@")

date=$(date +'%Y%m%d_%H%M%S')
csv=$date.csv

root=$(dirname "$0")
"$root"/consolidate-rundirs.sh "${args_consolidate[@]}" >"$csv"
libreoffice "$csv"