#!/bin/bash

set -eu -o pipefail

[ $# = 2 ]
file_in=$1
file_out=$2

convert "$file_in" -crop 800x+0+500 -trim -bordercolor black -border 10 "$file_out"