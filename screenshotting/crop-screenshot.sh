#!/bin/bash

set -eu -o pipefail

[ $# = 2 ]
file_in=$1
file_out=$2

case ${WORKER:-host} in
  host) dh=500 ;;
  c*) dh=550 ;;
  *) exit 1 ;;
esac

convert "$file_in" -crop 800x+0+"$dh" -trim -bordercolor black -border 10 "$file_out"