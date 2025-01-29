#!/bin/bash

set -eu -o pipefail

[ $# -ge 0 ]
csvs=("$@")

for csv in "${csvs[@]}"; do
  perl -pe 's/^(Linearization.*\t)\S.*$/$1-1/' -i "$csv"
done