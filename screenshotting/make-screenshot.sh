#!/bin/bash

set -eu -o pipefail

[ $# = 1 ]
file_png=$1

id_window=$(xdotool search --name "localhost:8080 - Google Chrome")
xwd -nobdrs -silent -id "$id_window" |
  convert xwd:- -strip "$file_png"