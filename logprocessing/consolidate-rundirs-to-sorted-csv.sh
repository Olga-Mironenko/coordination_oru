#!/bin/bash

root=$(dirname "$0")

dir=$root/../logs/rundirs
file=$dir/sorted-$(date +'%Y%m%d_%H%M%S').csv

"$root"/consolidate-rundirs.sh "$dir"/202* >"$file"